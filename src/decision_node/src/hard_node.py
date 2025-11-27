#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main_hardcode_missions.py
  - 하드코딩된 직진 테스트(<미션1>),
    라바콘 솔로 CPP 실행(<MISSION2>),
    AB 패턴(<MISSION3>)
    을 하나의 노드에서 순차적으로 실행하는 예시.



실행방법
roscore &
rosrun decision_node hard_node.py


"""

import rospy
import subprocess
import signal

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
        self.mission1 = self.build_mission1()  # 직진/색/정지/직진
        self.mission2 = self.build_mission2()  # AB 패턴

        # 미션2(실행 순서상)는 라바콘 솔로 CPP 실행 시간 (초)
        self.labacorn_duration = rospy.get_param("~labacorn_duration", 10.0)

        # 라바콘 솔로 실행 커맨드 (기본: rosrun decision_node labacorn_solo)
        default_cmd = ["rosrun", "decision_node", "labacorn_solo"]
        self.labacorn_cmd = rospy.get_param("~labacorn_cmd", default_cmd)

    # ---------------- 미션 1: 직진 + 색 구간 + 정지 + 직진 ----------------
    def build_mission1(self):
        """
        <미션1> 직진 하드코딩 결과를 기반으로 한 step 리스트.

        각 step: dict(name, servo, motor, duration)
        """
        servo_const = 0.567 # -.567 - 0.570

        steps = [
            # 1. 직진 (145cm 가야 함) → 약 5.11초
            dict(
                name="M1_STEP1_STRAIGHT_5.11S_V1000",
                servo=servo_const,
                motor=1000.0,
                duration=5.11, # 5.11 -> 5.3
            ),

            # 2. 빨간색 구간 - 느리게 (200cm 가야 함) → 약 8.34초
            dict(
                name="M1_STEP2_RED_SLOW_8.34S_V900",
                servo=servo_const,
                motor=900.0,
                duration=8.34,
            ),

            # 3. 파란색 구간 - 빠르게 (100cm 가야 함) → 약 1.38초
            dict(
                name="M1_STEP3_BLUE_FAST_1.38S_V2000",
                servo=servo_const,
                motor=2000.0,
                duration=1.38,
            ),

            # 4. 직진 (150cm 가야 함) → 약 4.57초
            dict(
                name="M1_STEP4_STRAIGHT_4.57S_V1100",
                servo=servo_const-0.01,
                motor=1100.0,
                duration=4.57,
            ),

            # 5. 정지 (5초)
            dict(
                name="M1_STEP5_STOP_5S",
                servo=servo_const,
                motor=0.0,
                duration=5.0,
            ),

            # 6. 직진 (155cm 가야 함) → 약 4.72초
            dict(
                name="M1_STEP6_STRAIGHT_4.72S_V1100",
                servo=servo_const + 0.01,
                motor=1100.0,
                duration=5.0, # 5.0
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
            # 0. 왼쪽 회전 
            # dict(
            #     name="M2_STEP1_STRAIGHT_3S_V1000",
            #     servo=0.367,
            #     motor=1000.0,
            #     duration=1.0,
            # ),

            # dict(
            #     name="M2_STEP1_STRAIGHT_3S_V1000",
            #     servo=0.567,
            #     motor=1000.0,
            #     duration=0.5,
            # ),

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

    # ---------------- 공통 실행 함수 (STEP 기반) ----------------
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

    # ---------------- 라바콘 솔로 CPP 실행 (MISSION2) ----------------
    def run_labacorn_solo(self, duration=None):
        """
        <MISSION2>:
          - 외부 C++ 노드(labacorn_solo)를 rosrun으로 실행
          - duration 초 동안 기다렸다가 SIGINT(Ctrl+C)로 종료 시도
        """
        if duration is None:
            duration = self.labacorn_duration

        cmd = self.labacorn_cmd
        if isinstance(cmd, str):
            cmd = cmd.split()

        rospy.loginfo("[MISSION2_LABACORN_SOLO] start: %s (duration=%.2fs)",
                      " ".join(cmd), duration)

        try:
            proc = subprocess.Popen(cmd)
        except Exception as e:
            rospy.logerr("[MISSION2_LABACORN_SOLO] failed to start: %s", e)
            return

        # duration 동안 대기 (로봇은 labacorn_solo 노드가 제어)
        start = rospy.Time.now()
        wait_rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed >= duration:
                break
            wait_rate.sleep()

        # SIGINT 보내서 종료 시도
        rospy.loginfo("[MISSION2_LABACORN_SOLO] send SIGINT to external node")
        try:
            proc.send_signal(signal.SIGINT)
        except Exception as e:
            rospy.log.warn("[MISSION2_LABACORN_SOLO] failed to send SIGINT: %s", e)

        try:
            proc.wait(timeout=3.0)
        except Exception:
            rospy.logwarn("[MISSION2_LABACORN_SOLO] process did not exit in time")

        rospy.loginfo("[MISSION2_LABACORN_SOLO] done")

    # ---------------- 전체 실행 루프 ----------------
    def run(self):
        rospy.sleep(1.0)  # publisher 준비시간

        # ---------- MISSION1: 직진/색/정지/직진 ----------
        if rospy.is_shutdown():
            return

        rospy.loginfo("========== START MISSION1_STRAIGHT_TEST ==========")
        for step in self.mission1:
            if rospy.is_shutdown():
                break
            self.run_step(step)
        rospy.loginfo("========== END   MISSION1_STRAIGHT_TEST ==========")

        # MISSION1 끝난 뒤 잠깐 정지
        self.stop_car(servo_center=0.567, duration=2.0)

        # ---------- MISSION2: 라바콘 솔로 CPP ----------
        if rospy.is_shutdown():
            return

        rospy.loginfo("========== START MISSION2_LABACORN_SOLO ==========")
        # 실행 전 잠깐 정지
        self.stop_car(servo_center=0.567, duration=2.0)
        
        self.run_labacorn_solo(duration=self.labacorn_duration)

        # 실행 후 정지
        self.stop_car(servo_center=0.567, duration=3.0)
        rospy.loginfo("========== END   MISSION2_LABACORN_SOLO ==========")

        # ---------- MISSION3: AB 패턴 (mode1-right) ----------
        if rospy.is_shutdown():
            return

        rospy.loginfo("========== START MISSION3_AB_RIGHT_MODE ==========")
        for step in self.mission2:
            if rospy.is_shutdown():
                break
            self.run_step(step)
        rospy.loginfo("========== END   MISSION3_AB_RIGHT_MODE ==========")

        # 전체 끝나면 완전 정지
        rospy.loginfo("[ALL MISSIONS DONE] final stop.")
        self.stop_car(servo_center=0.567, duration=3.0)


if __name__ == "__main__":
    try:
        runner = HardcodeMissionRunner()
        runner.run()
    except rospy.ROSInterruptException:
        pass
