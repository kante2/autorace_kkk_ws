#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion

def yaw_from_quat(q):
    # q: geometry_msgs/Quaternion
    quat_list = [q.x, q.y, q.z, q.w]
    (_, _, yaw) = euler_from_quaternion(quat_list)
    return yaw

class WheelOdomNode:
    def __init__(self):
        rospy.init_node('wheel_odom_node')
        rospy.loginfo('wheel_odom node started [ROS1] (motor/servo inputs)')

        # === Topics & frames ===
        self.imu_topic   = rospy.get_param('~imu_topic',  '/imu')
        self.odom_topic  = rospy.get_param('~odom_topic', '/odom')
        self.odom_frame  = rospy.get_param('~odom_frame', 'odom')
        self.base_frame  = rospy.get_param('~base_frame', 'base_link')

        # === Robot params ===
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.034)   # [m]
        self.wheel_base   = rospy.get_param('~wheel_base',   0.26)    # [m]

        # 속도 스케일: "명령 1 → RPM"을 기본 가정 (기존 혼동 교정)
        # 과거 파라미터 호환을 위해 ~rpm_scale도 받아들이되, 변수명은 의미가 명확한 rpm_per_cmd로 사용
        self.rpm_per_cmd = rospy.get_param('~rpm_per_cmd',
                             rospy.get_param('~rpm_scale', 0.073))  # [RPM per command]

        # 서보 중앙값/범위
        self.servo_center = rospy.get_param('~servo_center', 0.57165)
        self.servo_min    = rospy.get_param('~servo_min',    0.0)
        self.servo_max    = rospy.get_param('~servo_max',    1.0)

        # 비대칭 조향각 제한(도 단위 입력 → 라디안 변환)
        # 기본값을 질문에서 주신 조건으로 둠: left=25°, right=20°
        steer_left_deg  = rospy.get_param('~steer_left_limit_deg',  25.0)
        steer_right_deg = rospy.get_param('~steer_right_limit_deg', 20.0)
        self.steer_left_limit  = math.radians(steer_left_deg)   # 좌회전 최대 +25°
        self.steer_right_limit = math.radians(steer_right_deg)  # 우회전 최대  20°

        # 비대칭 맵 사용 여부: 중앙이 min/max 사이에 있어야 정상 동작
        self.use_asym_steer = True
        if not (self.servo_min < self.servo_center < self.servo_max):
            rospy.logwarn('servo_min < servo_center < servo_max 조건이 깨졌습니다. 값을 재확인하세요.')

        # 대칭 맵이 필요할 때만 사용(백업용 파라미터)
        self.steer_gain = rospy.get_param('~steer_gain', 1.2)  # 미사용 권장(비대칭 맵 사용중)

        # IMU 사용 여부
        self.use_imu_heading = rospy.get_param('~use_imu_heading', True)
        self.imu_drop_fallback_sec = rospy.get_param('~imu_drop_fallback_sec', 0.2)

        # === States ===
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.yaw_imu = None
        self.imu_prev_stamp = None   # 직전 IMU timestamp (yaw rate 계산용)
        self.imu_last_stamp = None   # 최신 IMU timestamp (staleness 체크용)
        self.wz_imu = 0.0
        self.wz_imu_valid = False

        self.motor_cmd = 0.0
        self.servo_pos = self.servo_center

        self.last_time = rospy.Time.now()

        # === Pub/Sub ===
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber('/commands/motor/speed',    Float64, self.cb_motor,  queue_size=10)
        rospy.Subscriber('/commands/servo/position', Float64, self.cb_servo,  queue_size=10)
        rospy.Subscriber(self.imu_topic,             Imu,     self.cb_imu,    queue_size=50)

    # --- Callbacks ---
    def cb_motor(self, msg: Float64):
        self.motor_cmd = float(msg.data)

    def cb_servo(self, msg: Float64):
        self.servo_pos = float(msg.data)

    def cb_imu(self, msg: Imu):
        yaw_now = yaw_from_quat(msg.orientation)

        # Razor IMU 9DoF는 header.stamp와 자이로 z를 제공하므로 적극 활용
        stamp = msg.header.stamp if msg.header.stamp and msg.header.stamp.to_sec() > 0 else rospy.Time.now()

        gyro_z = msg.angular_velocity.z
        if math.isfinite(gyro_z):
            self.wz_imu = gyro_z
            self.wz_imu_valid = True

        prev_stamp = self.imu_prev_stamp
        if (self.yaw_imu is not None) and (prev_stamp is not None):
            dyaw = math.atan2(math.sin(yaw_now - self.yaw_imu),
                              math.cos(yaw_now - self.yaw_imu))
            dt = (stamp - prev_stamp).to_sec()
            if 1e-4 < dt < 1.0:
                self.wz_imu = dyaw / dt
                self.wz_imu_valid = True

        self.yaw_imu = yaw_now
        self.imu_prev_stamp = stamp
        self.imu_last_stamp = stamp

    def servo_to_delta(self):
        """
        servo 명령 → 조향각(rad).
        주어진 실제 기구 조건:
          servo=0.0  → 좌최대 +25°
          servo=1.0  → 우최대 -20°   (부호 주의: 우회전은 음수로 처리)
          servo=0.57165 부근 → 0°
        """
        servo = min(max(self.servo_pos, self.servo_min), self.servo_max)

        if self.use_asym_steer:
            if servo <= self.servo_center:
                # 좌측 구간: center(0) → min(+25°)
                span = max(self.servo_center - self.servo_min, 1e-9)
                ratio = (self.servo_center - servo) / span  # 0~1
                delta = ratio * self.steer_left_limit       # +[0..left_limit]
            else:
                # 우측 구간: center(0) → max(-20°)
                span = max(self.servo_max - self.servo_center, 1e-9)
                ratio = (servo - self.servo_center) / span  # 0~1
                delta = - ratio * self.steer_right_limit    # -[0..right_limit]
        else:
            # 대칭 선형: 중앙 기준 좌(+), 우(-)로 가정
            delta = self.steer_gain * (servo - self.servo_center)

        # 안전 클램프(수치 오차 대비)
        delta = max(min(delta, self.steer_left_limit), -self.steer_right_limit)
        return delta

    def compute_v_body(self):
        """
        선속도 v [m/s] 계산.
        motor_cmd * (RPM/command) * (2πR) / 60
        """
        rpm = self.motor_cmd * self.rpm_per_cmd
        v = rpm * (2.0 * math.pi * self.wheel_radius) / 60.0
        return v

    # --- Main update ---
    def step(self):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        if dt <= 0.0 or dt > 1.0:
            self.last_time = now
            return

        # 1) 선속도 (몸체 좌표계 x축)
        v_body = self.compute_v_body()  # [m/s]

        # 2) 조향각(라디안)
        delta = self.servo_to_delta()

        # 3) 요 레이트(키네마틱)
        if abs(self.wheel_base) > 1e-6:
            omega_kin = v_body * math.tan(delta) / self.wheel_base
        else:
            omega_kin = 0.0

        # yaw 업데이트: IMU가 있으면 우선 적용, 없으면 적분
        imu_recent = False
        if self.imu_last_stamp is not None:
            imu_recent = 0.0 <= (now - self.imu_last_stamp).to_sec() < self.imu_drop_fallback_sec

        use_imu_heading_now = self.use_imu_heading and imu_recent and (self.yaw_imu is not None)
        wz_used = omega_kin

        if dt > 0.0:
            if use_imu_heading_now:
                self.yaw = self.yaw_imu
                if self.wz_imu_valid:
                    wz_used = self.wz_imu
            else:
                self.yaw += omega_kin * dt
                self.wz_imu_valid = False

            # yaw 정규화(±pi)
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

            # 바디 전진을 월드(odom)로 투영
            self.x += v_body * math.cos(self.yaw) * dt
            self.y += v_body * math.sin(self.yaw) * dt       

        # 6) Odometry 메시지
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # 평면 yaw만 반영한 쿼터니언
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(self.yaw * 0.5)

        odom.twist.twist.linear.x = v_body
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = wz_used

        # covariance: z/roll/pitch는 관측 안함 → 큰 값
        odom.pose.covariance = [
            0.05, 0,    0,    0,    0,    0,
            0,    0.05, 0,    0,    0,    0,
            0,    0,    1e6,  0,    0,    0,
            0,    0,    0,    1e6,  0,    0,
            0,    0,    0,    0,    1e6,  0,
            0,    0,    0,    0,    0,    0.1
        ]
        odom.twist.covariance = [
            0.05, 0,    0,    0,    0,    0,
            0,    0.05, 0,    0,    0,    0,
            0,    0,    1e6,  0,    0,    0,
            0,    0,    0,    1e6,  0,    0,
            0,    0,    0,    0,    1e6,  0,
            0,    0,    0,    0,    0,    0.1
        ]

        self.odom_pub.publish(odom)

        # 7) TF (odom -> base_link)
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            (0.0, 0.0, math.sin(self.yaw * 0.5), math.cos(self.yaw * 0.5)),
            now,
            self.base_frame,
            self.odom_frame
        )

        self.last_time = now
        print(f"yaw:{self.yaw:.3f} rad, wz:{wz_used:.3f} rad/s, omega_kin:{omega_kin:.3f} rad/s, v:{v_body:.3f} m/s, delta:{math.degrees(delta):.1f} deg")

    def run(self):
        rate_hz = rospy.get_param('~rate', 30.0)
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()

def main():
    node = WheelOdomNode()
    node.run()

if __name__ == '__main__':
    main()
