#!/usr/bin/env python3

# Find Extinsic matrix Lidar to camera coordinate

import sys, cv2, yaml, numpy as np

def load_camera(yaml_path, use_rectified=False):
    with open(yaml_path, 'r') as f:
        cam = yaml.safe_load(f)
    K = np.array(cam['camera_matrix']['data'], dtype=np.float64).reshape(3,3)
    D_raw = cam['distortion_coefficients']['data']
    if len(D_raw) < 5: D_raw += [0.0]*(5-len(D_raw))
    D = np.array(D_raw[:5], dtype=np.float64).reshape(1,5)
    if use_rectified:
        P = np.array(cam['projection_matrix']['data'], dtype=np.float64).reshape(3,4)
        K = P[:3,:3].copy()
        D = np.zeros((1,5), np.float64)
    return K, D

def load_pairs(path):
    obj, img = [], []
    with open(path, 'r') as f:
        for line in f:
            if not line.strip() or line.strip().startswith('#'): continue
            x,y,u,v = map(float, line.split())
            obj.append([x, y, 0.0])   # 2D LiDAR → z=0
            img.append([u, v])
    if len(obj) < 6:
        raise RuntimeError(f"Need >=6 pairs, got {len(obj)}")
    return np.array(obj, np.float32), np.array(img, np.float32)

def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} camera_calibration.yaml data.txt [use_rectified(0/1)]")
        sys.exit(1)
    cam_yaml, data_txt = sys.argv[1], sys.argv[2]
    use_rect = bool(int(sys.argv[3])) if len(sys.argv) >= 4 else False

    K, D = load_camera(cam_yaml, use_rect)
    obj, img = load_pairs(data_txt)

    ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_EPNP)
    if not ok: raise RuntimeError("solvePnP failed")
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, D, rvec, tvec)

    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3,:3] = R
    T[:3, 3] = tvec.flatten()

    # reprojection RMSE
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    proj = proj.reshape(-1,2)
    rmse = np.sqrt(np.mean(np.sum((proj - img)**2, axis=1)))

    np.set_printoptions(suppress=True, precision=9)
    print("\n# LiDAR -> Camera extrinsic")
    print("R_CL =\n", R)
    print("t_CL =\n", tvec)
    print("T_CL (4x4) =\n", T)
    print(f"Reprojection RMSE (px): {rmse:.3f}")

    # Save YAML
    out = {
        'R_CL': {'rows':3,'cols':3,'data':R.flatten().tolist()},
        't_CL': {'rows':3,'cols':1,'data':tvec.flatten().tolist()},
        'T_CL_4x4': {'rows':4,'cols':4,'data':T.flatten().tolist()},
        'note': 'p_C = R_CL * p_L + t_CL  (LiDAR->Camera)'
    }
    with open('lidar_to_camera_extrinsics.yaml', 'w') as f:
        yaml.dump(out, f)
    print("Saved: lidar_to_camera_extrinsics.yaml")

if __name__ == "__main__":
    main()
