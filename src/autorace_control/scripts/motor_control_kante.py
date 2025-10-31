#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64

class MotorServoCtrl(object):
    def __init__(self,latched=True):
        # latched=True: 마지막 메시지를 저장해 두었다가, 새 구독자가 붙으면 즉시 1번 보내줌
        self.motor_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1, latch=latched)
        self.servo_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1, latch=latched)

    def clamp_value(value, lower_bound, upper_bound):
        """value를 [lower_bound, upper_bound] 범위로 제한해서 반환"""
        if lower_bound > upper_bound:  # 실수 방지용 자동 교환
            lower_bound, upper_bound = upper_bound, lower_bound
        return max(lower_bound, min(value, upper_bound))

    def set_motor_speed(self, cmd, min_cmd=0.0, max_cmd=2000.0):
        """전진/후진 속도 명령 (장비 스케일에 맞게 클램프)"""
        msg = Float64()
        msg.data = self.clamp_value(cmd, min_cmd, max_cmd)
        self.motor_pub.publish(msg)

    def set_steering(self, pos, min_pos=0.0, max_pos=1.0):
        """
        조향 명령 (예: 0.0~1.0 스케일)
        0.0=좌, 1.0=우, 0.5=중립(직진) 가정
        """
        msg = Float64()
        msg.data = self.clamp_value(pos, min_pos, max_pos)
        self.servo_pub.publish(msg)

    def stop(self, motor_min=0.0, servo_center=0.5):
        """안전 정지(모터 0, 서보 중립)"""
        self.set_motor_speed(motor_min)
        self.set_steering(servo_center)


def main():
    rospy.init_node("motor_servo_ctrl_node")

    # 파라미터 (런치/파라서버에서 덮어쓰기 가능)
    motor_topic = rospy.get_param("~motor_topic", "/commands/motor/speed")
    servo_topic = rospy.get_param("~servo_topic", "/commands/servo/position")
    latched     = rospy.get_param("~latched", True)

    ctrl = MotorServoCtrl(motor_topic=motor_topic, servo_topic=servo_topic, latched=latched)

    # --- 데모 시퀀스: 1) 전진 2) 좌로 조향 3) 정지 ---
    rospy.loginfo("Go forward")
    ctrl.set_motor_speed(2000.0)  # 
    ctrl.set_steering(0.5)        # 중립(직진 가정)
    rospy.sleep(1.5)              # ros::Duration(1.5).sleep()와 동일(ROS 시간 기준)

    rospy.loginfo("Steer left")
    ctrl.set_steering(0.35)       # 장비에 맞게 조정
    rospy.sleep(1.0)

    rospy.loginfo("Stop")
    ctrl.stop()                   # motor=0.0, servo=0.5

    # 필요하면 루프 유지
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     # 주기 퍼블리시 / 상태 점검 등
    #     rate.sleep()

if __name__ == "__main__":
    main()
clamp_value