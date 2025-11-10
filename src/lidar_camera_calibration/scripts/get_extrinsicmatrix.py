#!/usr/bin/env python3

# === Lidar to Camera Extinsic matrix 구하기 ===
#!/usr/bin/env python3
# === LiDAR → Camera extrinsic (rect 이미지 대응) ===

import sys, cv2, yaml, numpy as np, argparse, os

def load_camera(yaml_path, use_rectified=False):
    with open(yaml_path, 'r') as f:
        cam = yaml.safe_load(f)

    # 기본(raw) K, D
    K = np.array(cam['camera_matrix']['data'], dtype=np.float64).reshape(3,3)
    D_raw = cam['distortion_coefficients']['data']
    if len(D_raw) < 5: D_raw += [0.0]*(5-len(D_raw))
    D = np.array(D_raw[:5], dtype=np.float64).reshape(1,5)

    # rect 이미지면 P에서 K_rect, D=0
    if use_rectified and 'projection_matrix' in cam:
        P = np.array(cam['projection_matrix']['data'], dtype=np.float64).reshape(3,4)
        K = P[:3,:3].copy()
        D = np.zeros((1,5), np.float64)
    return K, D

def load_pairs(path):
    obj, img = [], []
    with open(path, 'r') as f:
        for ln in f:
            s = ln.strip()
            if not s or s.startswith('#'): continue
            x, y, u, v = map(float, s.split()[:4])
            obj.append([x, y, 0.0])   # 2D LiDAR → z=0
            img.append([u, v])
    if len(obj) < 6:
        raise RuntimeError(f"Need >=6 pairs, got {len(obj)} from {path}")
    return np.array(obj, np.float32), np.array(img, np.float32)

def solve_extrinsic(obj, img, K, D, reproj_th_px=3.0, max_iter=5):
    """EPNP → LM refine, with simple inlier pruning by reprojection error."""
    idx = np.arange(len(obj))
    rvec = tvec = None

    for it in range(max_iter):
        ok, rvec, tvec = cv2.solvePnP(obj[idx], img[idx], K, D,
                                      flags=cv2.SOLVEPNP_EPNP)
        if not ok:
            raise RuntimeError("solvePnP failed (EPNP).")
        rvec, tvec = cv2.solvePnPRefineLM(obj[idx], img[idx], K, D, rvec, tvec)

        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
        proj = proj.reshape(-1, 2)
        err = np.linalg.norm(proj - img, axis=1)

        # 인라이어 선별
        inlier = err < reproj_th_px
        if inlier.sum() == len(idx):
            break
        # 항상 최소 개수(6) 유지
        new_idx = np.where(inlier)[0]
        if len(new_idx) < 6:
            # 너무 빡빡하면 문턱을 살짝 올려서 다시 시도
            reproj_th_px *= 1.5
            continue
        idx = new_idx

    # 최종 성능 통계
    proj, _ = cv2.projectPoints(obj[idx], rvec, tvec, K, D)
    proj = proj.reshape(-1, 2)
    err = np.linalg.norm(proj - img[idx], axis=1)
    rmse = float(np.sqrt(np.mean(err**2)))
    med = float(np.median(err))

    R, _ = cv2.Rodrigues(rvec)
    # R 정규화(수치오차 보정)
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    if np.linalg.det(R) < 0:  # 반사 방지
        R[:, -1] *= -1
        tvec[-1] *= -1

    return R, tvec.reshape(3,1), {
        'rmse_px': rmse,
        'median_px': med,
        'inliers': int(len(idx)),
        'total': int(len(obj))
    }

def save_yaml(T, K, out_path):
    out = {
        'image': {'rectified': True},
        'camera_intrinsics': {'K': K.flatten().tolist(), 'D':[0,0,0,0,0]},
        'extrinsics': {'T_lidar_camera': T.flatten().tolist()},
        'note': 'p_cam = T_lidar_camera * p_lidar_h (optical frame; homogeneous)'
    }
    with open(out_path, 'w') as f:
        yaml.dump(out, f)
    print(f"Saved: {out_path}")

def preview_overlay(obj, img, K, R, t, out_png):
    """재투영 미리보기(정합 품질 눈으로 확인)."""
    proj, _ = cv2.projectPoints(obj, cv2.Rodrigues(R)[0], t, K, np.zeros((1,5)))
    proj = proj.reshape(-1,2)
    # 빈 캔버스에 GT/Proj 점 찍기 (시각용)
    w = int(2*max(K[0,2], K[1,2], 640)); h = w*3//4
    canvas = np.zeros((h,w,3), np.uint8)
    for (u_gt,v_gt),(u_p,v_p) in zip(img, proj):
        cv2.circle(canvas, (int(round(u_gt)), int(round(v_gt))), 3, (0,255,0), -1) # GT green
        cv2.circle(canvas, (int(round(u_p)),  int(round(v_p))),  3, (0,0,255), -1) # Proj red
    cv2.imwrite(out_png, canvas)
    print(f"Preview saved: {out_png}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cam_yaml', help='camera_calibration.yaml')
    ap.add_argument('pairs_txt', help='data.txt with lines: x y u v')
    ap.add_argument('--rect', type=int, default=1, help='use rectified K (P[:3,:3]) and D=0 (default=1)')
    ap.add_argument('--th', type=float, default=3.0, help='inlier reprojection threshold px (default=3.0)')
    ap.add_argument('--out', default='lidar_to_camera_extrinsics.yaml')
    ap.add_argument('--preview', default='reproj_preview.png')
    ap.add_argument('--flipx', action='store_true', help='apply optical x-mirror (diag(-1,1,1)) if needed')
    args = ap.parse_args()

    K, D = load_camera(args.cam_yaml, use_rectified=bool(args.rect))
    obj, img = load_pairs(args.pairs_txt)

    R, t, stat = solve_extrinsic(obj, img, K, D, reproj_th_px=args.th)

    # 필요시 좌우 반전 보정(광학 x)
    if args.flipx:
        Sx = np.diag([-1.0, 1.0, 1.0])
        R = Sx @ R
        t = Sx @ t

    T = np.eye(4, dtype=np.float64)
    T[:3,:3] = R
    T[:3, 3] = t.flatten()

    proj, _ = cv2.projectPoints(obj, cv2.Rodrigues(R)[0], t, K, D)
    proj = proj.reshape(-1,2)
    rmse_all = float(np.sqrt(np.mean(np.sum((proj - img)**2, axis=1))))

    np.set_printoptions(suppress=True, precision=9)
    print("\n# LiDAR -> Camera extrinsic (optical frame)")
    print("R_CL =\n", R)
    print("t_CL =\n", t)
    print("T_CL (4x4) =\n", T)
    print(f"Stats: inliers={stat['inliers']}/{stat['total']}  rmse(px)={stat['rmse_px']:.3f}  median(px)={stat['median_px']:.3f}")
    print(f"RMSE(all pairs, px): {rmse_all:.3f}")

    save_yaml(T, K, args.out)
    # 간단 미리보기 이미지(선택)
    try:
        preview_overlay(obj, img, K, R, t, args.preview)
    except Exception as e:
        print("Preview skipped:", e)

if __name__ == "__main__":
    main()

"""
# solvePnP 스크립트
python3 solve_lidar2cam.py /home/you/calib/cam_calibration.yaml data.txt --rect 1

"""