#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def yaw_from_quat(q):
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
        self.publish_tf  = rospy.get_param('~publish_tf', True)

        # === Robot params ===
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.034)   # m
        self.wheel_base   = rospy.get_param('~wheel_base',   0.26)    # m
        # 속도 스케일: "모터 명령 1 → 몇 m/s" 로 해석 (예: 0.025 m/s)
        self.rpm_scale    = rospy.get_param('~rpm_scale',    0.073)   # m/s per command // 1000명령에, 73RPM
        # 서보 중앙값(0~1), 서보→조향각 변환 gain [rad per 1.0 servo-unit]
        self.servo_center = rospy.get_param('~servo_center', 0.57165)
        self.steer_gain   = rospy.get_param('~steer_gain',   1.2)     # 튜닝 필요 (비대칭 맵 미사용 시 적용)
        self.servo_min    = rospy.get_param('~servo_min',    0.0)
        self.servo_max    = rospy.get_param('~servo_max',    1.0)

        steer_left_deg  = rospy.get_param('~steer_left_limit_deg',  None)
        steer_right_deg = rospy.get_param('~steer_right_limit_deg', None)
        self.use_asym_steer = (steer_left_deg is not None) and (steer_right_deg is not None)
        if self.use_asym_steer:
            self.steer_left_limit  = math.radians(steer_left_deg)
            self.steer_right_limit = math.radians(steer_right_deg)
            if not (self.servo_min < self.servo_center < self.servo_max):
                rospy.logwarn('servo_min < servo_center < servo_max 조건이 깨졌습니다. 비대칭 맵을 올바르게 사용하려면 값을 재확인하세요.')
        else:
            self.steer_left_limit = None
            self.steer_right_limit = None

        # IMU 사용 여부
        self.use_imu_heading = rospy.get_param('~use_imu_heading', True)

        # === States ===
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # odom 기준 yaw

        self.yaw_imu = None
        self.last_imu_stamp = None
        self.wz_imu = 0.0

        # imu-odom 정렬용 초기값
        self._aligned = False
        self._yaw_imu0 = 0.0
        self._yaw_odom0 = 0.0  # 보통 0

        self.motor_cmd = 0.0
        self.servo_pos = self.servo_center

        self.last_time = rospy.Time.now()

        # === Pub/Sub ===
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=20)
        self.odom_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber('/commands/motor/speed',    Float64, self.cb_motor,  queue_size=20)
        rospy.Subscriber('/commands/servo/position', Float64, self.cb_servo,  queue_size=20)
        rospy.Subscriber(self.imu_topic,             Imu,     self.cb_imu,    queue_size=50)

    # --- Callbacks ---
    def cb_motor(self, msg: Float64):
        self.motor_cmd = float(msg.data)

    def cb_servo(self, msg: Float64):
        self.servo_pos = float(msg.data)

    def cb_imu(self, msg: Imu):
        yaw_now = yaw_from_quat(msg.orientation)
        if self.yaw_imu is not None and self.last_imu_stamp is not None:
            dt = (msg.header.stamp - self.last_imu_stamp).to_sec()
            if 1e-6 < dt < 1.0:
                dyaw = math.atan2(math.sin(yaw_now - self.yaw_imu),
                                  math.cos(yaw_now - self.yaw_imu))
                self.wz_imu = dyaw / dt
        self.yaw_imu = yaw_now

    def servo_to_delta(self):
        """
        servo 명령 → 조향각(rad) 변환.
        비대칭 맵 파라미터가 설정되면 piecewise 선형 변환을 사용하고,
        아니면 기존 steer_gain 기반 선형 변환을 유지한다.
        """
        if not self.use_asym_steer:
            return self.steer_gain * (self.servo_pos - self.servo_center)

        servo_clamped = min(max(self.servo_pos, self.servo_min), self.servo_max)
        if servo_clamped <= self.servo_center:
            span = max(self.servo_center - self.servo_min, 1e-6)
            ratio = (self.servo_center - servo_clamped) / span
            return ratio * self.steer_left_limit

        span = max(self.servo_max - self.servo_center, 1e-6)
        ratio = (servo_clamped - self.servo_center) / span
        return ratio * self.steer_right_limit

    # --- Main update ---
    def step(self):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        if dt <= 0.0 or dt > 1.0:
            self.last_time = now
            return

        # 1) 선속도 v [m/s]: RPM = cmd * rpm_per_cmd ; v = RPM * (2π/60) * r
        rpm = self.motor_cmd * self.rpm_per_cmd
        v_body = (rpm * (2.0 * math.pi / 60.0)) * self.wheel_radius  # [m/s]
        # 1) 선속도 (몸체 좌표계 x축)
        v_body = self.motor_cmd * self.rpm_scale*self.wheel_radius*2*math.pi/60  # [m/s] 

        # 2) 조향각 delta [rad] (선형맵 + 포화)
        delta_servo = self.servo_pos - self.servo_center
        delta = self.steer_gain * delta_servo
        if delta > self.max_steer_rad:
            delta = self.max_steer_rad
        elif delta < -self.max_steer_rad:
            delta = -self.max_steer_rad

        # 3) 슬립각 β (front-steer only): beta = atan( lr/L * tan(delta) )
        beta = math.atan2(self.lr * math.tan(delta), self.L)

        # 4) 요율 (cosβ 감쇠 포함)
        omega_kin = 0.0
        if abs(self.L) > 1e-9:
            omega_kin = (v_body / self.L) * math.tan(delta) * math.cos(beta)
        # 2) 조향각(라디안)
        delta = self.servo_to_delta()
        
        # 3) 요 레이트(키네마틱)
        omega_kin = v_body * math.tan(delta) / self.wheel_base

        # 5) yaw 업데이트
        if self.use_imu_heading and (self.yaw_imu is not None):
            # 처음 한 번 IMU-odom 정렬 (odom yaw 기준을 imu 초기값에 맞춤)
            if not self._aligned:
                self._yaw_imu0 = self.yaw_imu
                self._yaw_odom0 = self.yaw     # 보통 0
                self._aligned = True
            # odom yaw = odom0 + (imu_yaw - imu0)
            yaw_est = self._yaw_odom0 + math.atan2(math.sin(self.yaw_imu - self._yaw_imu0),
                                                   math.cos(self.yaw_imu - self._yaw_imu0))
            self.yaw = math.atan2(math.sin(yaw_est), math.cos(yaw_est))
            wz_used = self.wz_imu
        else:
            self.yaw = math.atan2(math.sin(self.yaw + omega_kin * dt),
                                  math.cos(self.yaw + omega_kin * dt))
            wz_used = omega_kin

        # 6) 위치 적분 (움직임 방향은 ψ+β)
        psi_move = self.yaw + beta
        self.x += v_body * math.cos(psi_move) * dt
        self.y += v_body * math.sin(psi_move) * dt

        # 7) Odometry 메시지
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x  = v_body
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.angular.z = wz_used

        # 간단 covariance (z/roll/pitch는 크게)
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

        # 8) TF (odom -> base_link)
        if self.publish_tf:
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.0), q, now, self.base_frame, self.odom_frame
            )

        self.last_time = now
        print(f"yaw_rate:{self.yaw} omega_kin:{omega_kin}")


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
