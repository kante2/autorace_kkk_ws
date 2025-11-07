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
        self.imu_topic   = rospy.get_param('~imu_topic',  '/imu/data')
        self.odom_topic  = rospy.get_param('~odom_topic', '/wheel_odom')
        self.odom_frame  = rospy.get_param('~odom_frame', 'odom')
        self.base_frame  = rospy.get_param('~base_frame', 'base_link')
        self.publish_tf  = rospy.get_param('~publish_tf', True)

        # === Robot params ===
        # 총 휠베이스 L, 앞/뒤 분할 lf, lr (lf+lr != L이면 L을 lf+lr로 교정)
        L_param = rospy.get_param('~wheel_base', 0.26)  # [m]
        lf = rospy.get_param('~lf', None)
        lr = rospy.get_param('~lr', None)
        if lf is None or lr is None:
            # 제공 안되면 반반
            lf = L_param * 0.5
            lr = L_param * 0.5
        self.lf = float(lf)
        self.lr = float(lr)
        self.L  = self.lf + self.lr

        self.wheel_radius = rospy.get_param('~wheel_radius', 0.034)  # [m]

        # Motor cmd -> RPM 스케일 (예: 1000 명령 → 75 RPM 이면 0.075)
        self.rpm_per_cmd  = rospy.get_param('~rpm_per_cmd', 0.075)   # [RPM / command]

        # Servo → steering (rad) 변환
        self.servo_center   = rospy.get_param('~servo_center', 0.571)   # [servo unit]
        self.steer_gain     = rospy.get_param('~steer_gain',   1.2)     # [rad / servo_unit]
        self.max_steer_rad  = rospy.get_param('~max_steer_rad', 0.5)    # 포화

        # IMU 사용 및 정렬
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
        self.last_imu_stamp = msg.header.stamp

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
