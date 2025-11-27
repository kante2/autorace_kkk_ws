#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main_hardcode_missions.py
  - 하드코딩된 직진 테스트(<미션1>)와 AB 패턴(<미션2>)을
    하나의 노드에서 순차적으로 실행하는 예시.
"""

import rospy
from std_msgs.msg import Float64


class HardcodeMissionRunner:
    def __init__(self):
        rospy.init_node("hardcode_mission_runner")

        # 토픽 이름 파라미터화 (필요하면 launch에서 바꾸기)
        servo_topic = rospy.get_param("~servo_topic",
                                      "/commands/servo/position")
        motor_topic = rospy.get_param("~motor_topic",
                                      "/commands/motor/speed")

        self.pub_servo = rospy.Publisher(servo_topic, Float64, queue_size=1)
        self.pub_motor = rospy.Publisher(motor_topic, Float64, queue_size=1)

        # publish 주기 (timeout -r 20 과 동일: 20Hz)
        self.rate = rospy.Rate(20.0)

        # ----- 미션 정의 -----
        self.mission1 = self.build_mission1()
        self.mission2 = self.build_mission2()

        # 실행 순서
        self.mission_sequence = [
            ("MISSION1_STRAIGHT_TEST", self.mission1),
            ("MISSION2_AB_RIGHT_MODE", self.mission2),
        ]

    # ---------------- 미션 1: 직진 + 색 구간 + 정지 + 직진 ----------------
    def build_mission1(self):
        """
        <미션1> 직진 하드코딩 결과를 기반으로 한 step 리스트.

        각 step: dict(name, servo, motor, duration)
        """
        servo_const = 0.567

        steps = [
            # 1. 직진 (5초, 속도 1000) -> 실제 이동거리 142cm
            dict(
                name="M1_STEP1_STRAIGHT_5S_V1000",
                servo=servo_const,
                motor=1000.0,
                duration=5.0,
            ),

            # 2. 빨간색 구간 - 느리게 (3초, 속도 900)
            dict(
                name="M1_STEP2_RED_SLOW_3S_V900",
                servo=servo_const,
                motor=900.0,
                duration=3.0,
            ),

            # 3. 파란색 구간 - 빠르게
            #   주석: 5초라고 적혀 있지만 실제 코드는 2초였음 (원문 유지)
            dict(
                name="M1_STEP3_BLUE_FAST_2S_V2000",
                servo=servo_const,
                motor=2000.0,
                duration=2.0,   # 필요하면 5.0 으로 수정해서 실험
            ),

            # 4. 직진 (기본 속도 1100, 5초)
            dict(
                name="M1_STEP4_STRAIGHT_5S_V1100",
                servo=servo_const,
                motor=1100.0,
                duration=5.0,
            ),

            # 5. 정지 (5초)
            dict(
                name="M1_STEP5_STOP_5S",
                servo=servo_const,
                motor=0.0,
                duration=5.0,
            ),

            # 6. 직진 (속도 1100, 6초)
            dict(
                name="M1_STEP6_STRAIGHT_6S_V1100",
                servo=servo_const,
                motor=1100.0,
                duration=6.0,
            ),
        ]
        return steps

    # ---------------- 미션 2: AB mode1 - right ----------------
    def build_mission2(self):
        """
        <미션2> AB (mode1-right) 하드코딩을 기반으로 한 step 리스트.
        1: 직진
        2: 오른쪽 조향 1초
        3: 왼쪽 조향 1초
        """
        steps = [
            # 1. 직진 3초
            dict(
                name="M2_STEP1_STRAIGHT_3S_V1000",
                servo=0.567,
                motor=1000.0,
                duration=3.0,
            ),

            # 2. 오른쪽으로 1초 스티어 (0.7) + 직진
            dict(
                name="M2_STEP2_RIGHT_1S_V1000",
                servo=0.7,
                motor=1000.0,
                duration=1.0,
            ),

            # 3. 왼쪽으로 1초 스티어 (0.3) + 직진
            dict(
                name="M2_STEP3_LEFT_1S_V1000",
                servo=0.3,
                motor=1000.0,
                duration=1.0,
            ),
        ]
        return steps

    # ---------------- 공통 실행 함수 ----------------
    def run_step(self, step):
        name = step["name"]
        servo = step["servo"]
        motor = step["motor"]
        duration = step["duration"]

        rospy.loginfo("  [STEP] %s  servo=%.3f, motor=%.1f, duration=%.2f",
                      name, servo, motor, duration)

        start = rospy.Time.now()
        msg_servo = Float64(servo)
        msg_motor = Float64(motor)

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed >= duration:
                break

            self.pub_servo.publish(msg_servo)
            self.pub_motor.publish(msg_motor)
            self.rate.sleep()

        rospy.loginfo("  [STEP DONE] %s", name)

    def stop_car(self, servo_center=0.567, duration=2.0):
        """
        안전하게 정지용.
        """
        rospy.loginfo("  [STOP] servo=%.3f, motor=0, duration=%.2fs",
                      servo_center, duration)
        start = rospy.Time.now()
        msg_servo = Float64(servo_center)
        msg_motor = Float64(0.0)

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed >= duration:
                break
            self.pub_servo.publish(msg_servo)
            self.pub_motor.publish(msg_motor)
            self.rate.sleep()

    def run(self):
        rospy.sleep(1.0)  # publisher 준비시간

        for mission_name, steps in self.mission_sequence:
            if rospy.is_shutdown():
                break

            rospy.loginfo("========== START %s ==========", mission_name)
            for step in steps:
                if rospy.is_shutdown():
                    break
                self.run_step(step)
            rospy.loginfo("========== END   %s ==========", mission_name)

            # 각 미션 사이에 잠깐 정지
            self.stop_car(servo_center=0.567, duration=2.0)

        # 전체 끝나면 완전 정지
        rospy.loginfo("[ALL MISSIONS DONE] final stop.")
        self.stop_car(servo_center=0.567, duration=3.0)


if __name__ == "__main__":
    try:
        runner = HardcodeMissionRunner()
        runner.run()
    except rospy.ROSInterruptException:
        pass
