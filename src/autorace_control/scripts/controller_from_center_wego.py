#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys, time, math
from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import PointStamped

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

class LaneCenterController:
    def __init__(self):
        rospy.init_node("lane_center_controller")
        rospy.loginfo("lane_center_controller (center_point_px -> motor/servo Float64)")

        # --- Topics ---
        # 센터 포인트 토픽 (PointStamped)
        self.topic_center_point = rospy.get_param("~topic_center_point",
                                                  "/perception/center_point_px")
        # (참고용) 예전 dx 토픽 이름도 파라미터로 남겨둠 (실제 구독은 center_point 쪽으로 함)
        self.topic_dx_px = rospy.get_param("~topic_dx_px", "/perception/dx_px")

        # 모터/서보 토픽 (F1TENTH 스타일)
        self.motor_topic = rospy.get_param("~motor_topic", "/commands/motor/speed")
        self.servo_topic = rospy.get_param("~servo_topic", "/commands/servo/position")

        rospy.loginfo(f"[lane_ctrl] subscribe center='{rospy.resolve_name(self.topic_center_point)}'")
        rospy.loginfo(f"[lane_ctrl] publish motor='{rospy.resolve_name(self.motor_topic)}', "
                      f"servo='{rospy.resolve_name(self.servo_topic)}'")

        # --- BEV 중심 x (픽셀) : dx = cx - bev_center_x_px ---
        self.bev_center_x_px = float(rospy.get_param("~bev_center_x_px", 320.0))

        # --- 서보(조향) 스케일 ---
        # 0.0 = 최대 좌, 1.0 = 최대 우, 0.5 = 직진
        self.servo_center = float(rospy.get_param("~servo_center", 0.5))
        self.servo_min    = float(rospy.get_param("~servo_min",    0.0))
        self.servo_max    = float(rospy.get_param("~servo_max",    1.0))

        # steer_sign: dx 부호/조향 방향이 반대면 -1.0으로
        self.steer_sign   = float(rospy.get_param("~steer_sign", -1.0))

        # --- Control Params (기존 그대로) ---
        self.max_abs_dx_px = float(rospy.get_param("~max_abs_dx_px", 100.0))
        self.dx_tolerance  = float(rospy.get_param("~dx_tolerance", 3.0))
        self.steer_gain    = float(rospy.get_param("~steer_gain", 2.0)) # 1.0 -> 2.0 
        self.alpha_ema     = float(rospy.get_param("~steer_smoothing_alpha", 0.2))
        self.max_delta     = float(rospy.get_param("~max_steer_delta_per_cycle", 0.08))

        # 속도 계획 (단위: m/s 개념)
        self.base_speed_mps = float(rospy.get_param("~base_speed_mps", 7.0))
        self.min_speed_mps  = float(rospy.get_param("~min_speed_mps", 0.8))
        self.speed_drop_gain= float(rospy.get_param("~speed_drop_gain", 0.5))

        # 모터 스케일 (m/s -> PWM/속도 명령값)
        self.motor_min_cmd  = float(rospy.get_param("~motor_min_cmd", 0.0))
        self.motor_max_cmd  = float(rospy.get_param("~motor_max_cmd", 900.0))
        self.motor_gain     = float(rospy.get_param("~motor_gain",    300.0))
        # 예: speed_cmd 5 m/s → motor_cmd = 300*5 = 1500

        # 안전 타임아웃
        self.dx_timeout_sec = float(rospy.get_param("~dx_timeout_sec", 1.0))

        # --- Publisher/State ---
        self.motor_pub = rospy.Publisher(self.motor_topic, Float64, queue_size=10)
        self.servo_pub = rospy.Publisher(self.servo_topic, Float64, queue_size=10)

        self.prev_steer = 0.0      # 내부 steer_cmd (정규화 값, -1~+1)
        self._last_cb_t = None

        self._latest_steer_cmd = 0.0   # -1 ~ +1
        self._latest_speed_cmd = self.base_speed_mps

        # ── 여기서 center_point_px 를 구독 ──
        rospy.Subscriber(self.topic_center_point,
                         PointStamped,
                         self.CB_center,
                         queue_size=20)

    # ---------------- 콜백1: center_point_px → dx_px 계산 ----------------
    def CB_center(self, msg: PointStamped):
        """
        /perception/center_point_px (PointStamped: BEV 픽셀) 콜백.
        cx를 기준으로 dx_px = cx - bev_center_x_px 계산 후
        기존 CB_dx 로 넘겨서 동일 로직 사용.
        """
        self._last_cb_t = time.time()

        cx = float(msg.point.x)
        dx = cx - self.bev_center_x_px   # 오른쪽이 +, 왼쪽이 -

        # 기존 CB_dx 형태(Float32)를 그대로 쓰기 위해 가짜 메시지 생성
        dx_msg = Float32()
        dx_msg.data = dx
        self.CB_dx(dx_msg)

    # ---------------- 콜백2: dx_px -> 내부 steer_cmd, speed_cmd ----------------
    def CB_dx(self, msg: Float32):
        dx = float(msg.data)   # +: 차선 중심이 프레임 우측, -: 프레임 좌측

        # 작은 오차(dead zone)는 무시
        if abs(dx) <= self.dx_tolerance:
            err_norm = 0.0
        else:
            err_norm = clamp(dx / self.max_abs_dx_px, -1.0, 1.0)

        # -1 ~ +1 사이로 제한된 조향 명령 (논리적 값)
        steer_raw = math.tanh(self.steer_gain * err_norm)

        # EMA로 부드럽게
        steer_smooth = self.alpha_ema * steer_raw + (1.0 - self.alpha_ema) * self.prev_steer

        # 한 사이클당 변화량 제한
        delta = clamp(steer_smooth - self.prev_steer, -self.max_delta, self.max_delta)
        steer_cmd = self.prev_steer + delta
        self.prev_steer = steer_cmd

        # 속도 계획: 오차가 크면 속도 낮추기
        speed_cmd = clamp(self.base_speed_mps - self.speed_drop_gain * abs(err_norm),
                          self.min_speed_mps, self.base_speed_mps)

        self._latest_steer_cmd = steer_cmd   # -1 ~ +1
        self._latest_speed_cmd = speed_cmd   # m/s 개념

        print(f"[lane_ctrl][CB] dx={dx:.1f}px  errN={err_norm:.3f}  steer={steer_cmd:.3f}  v={speed_cmd:.2f}")
        sys.stdout.flush()
        rospy.loginfo_throttle(
            0.5,
            f"[lane_ctrl][CB] dx={dx:.1f}px errN={err_norm:.3f} steer={steer_cmd:.3f} v={speed_cmd:.2f}"
        )

    # ---------------- 메인 루프: 내부 명령 -> 실제 하드웨어 스케일 ----------------
    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            now = time.time()
            have_dx = (self._last_cb_t is not None) and ((now - self._last_cb_t) <= self.dx_timeout_sec)

            if have_dx:
                steer_cmd = self._latest_steer_cmd   # -1 ~ +1
                speed_cmd = self._latest_speed_cmd   # m/s
            else:
                steer_cmd = 0.0
                speed_cmd = 0.0
                rospy.loginfo_throttle(1.0, "[lane_ctrl] waiting /perception/center_point_px ... (timeout)")

            # ---- 1) 조향 변환: -1~+1 내부 → 0~1 (0=좌, 1=우) 하드웨어 스케일 ----
            steer_norm = clamp(steer_cmd, -1.0, 1.0)

            # 조향 방향 뒤집기 옵션
            steer_norm *= (-self.steer_sign)

            # 0.5 = 직진, ±0.5 범위 안에서 사용 → 0.0~1.0으로 매핑
            servo_range = min(self.servo_center - self.servo_min,
                              self.servo_max - self.servo_center)
            servo_hw = self.servo_center + steer_norm * servo_range
            servo_hw = clamp(servo_hw, self.servo_min, self.servo_max)

            # ---- 2) 속도 변환: m/s → 모터 명령 값 (예: PWM 0~2000) ----
            motor_cmd = self.motor_gain * speed_cmd
            motor_cmd = clamp(motor_cmd, self.motor_min_cmd, self.motor_max_cmd)

            # ---- 3) Publish ----
            self.motor_pub.publish(Float64(data=float(motor_cmd)))
            self.servo_pub.publish(Float64(data=float(servo_hw)))

            rospy.loginfo_throttle(
                0.5,
                f"[lane_ctrl][loop] have_dx={have_dx} "
                f"steer_cmd={steer_cmd:.3f} servo={servo_hw:.3f} motor={motor_cmd:.1f} v={speed_cmd:.2f}"
            )

            rate.sleep()

if __name__ == "__main__":
    try:
        node = LaneCenterController()
        node.run()
    except rospy.ROSInterruptException:
        pass
