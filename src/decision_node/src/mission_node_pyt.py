#!/usr/bin/env python3
# mission_node_pyt.py
# 센서 enable + bag 트리거 + 하드코딩 직진을 단계별로 실행한다.

import rospy
from std_msgs.msg import Bool, Float64
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class Phase:
    name: str
    duration: float
    camera: bool = False     # 카메라 계열 enable
    lidar: bool = False      # 라이다 계열 enable
    bag_ab: bool = False     # AB bag enable
    bag_rotary: bool = False # Rotary bag enable
    bag_parking: bool = False# Parking bag enable
    hard: bool = False       # 하드코딩 주행 여부
    hard_motor: Optional[float] = None
    hard_servo: Optional[float] = None


class MissionNode:
    def __init__(self):
        rospy.init_node("mission_node_py")

        # ---- 토픽 파라미터 ----
        self.camera_enable_topic = rospy.get_param("~camera_enable_topic", "/mission/camera_enable")
        self.lidar_enable_topic = rospy.get_param("~lidar_enable_topic", "/mission/lidar_enable")

        self.crosswalk_enable_topic = rospy.get_param("~crosswalk_enable_topic", "/perception/crosswalk/enable")
        self.ab_enable_topic = rospy.get_param("~ab_enable_topic", "/perception/ab/enable")
        self.ab_sign_enable_topic = rospy.get_param("~ab_sign_enable_topic", "/ab_sign/enable")
        self.gate_enable_topic = rospy.get_param("~gate_enable_topic", "/perception/gate/enable")
        self.rotary_enable_topic = rospy.get_param("~rotary_enable_topic", "/perception/rotary/enable")
        self.parking_enable_topic = rospy.get_param("~parking_enable_topic", "/perception/parking/enable")
        # 라바콘은 라이다 측이라 lidar_on 시 함께 활성화된다고 가정

        self.servo_topic = rospy.get_param("~servo_topic", "/commands/servo/position")
        self.motor_topic = rospy.get_param("~motor_topic", "/commands/motor/speed")

        # bag enable 토픽 (bag_play 노드와 매칭)
        self.bag_ab_enable_topic = rospy.get_param("~bag_ab_enable_topic", "/mission/bag_enable/ab")
        self.bag_rotary_enable_topic = rospy.get_param("~bag_rotary_enable_topic", "/mission/bag_enable/rotary")
        self.bag_parking_enable_topic = rospy.get_param("~bag_parking_enable_topic", "/mission/bag_enable/parking")

        # ---- 기본 시간 파라미터 (초) ----
        cam1 = rospy.get_param("~cam1_sec", 20.0)      # 미션1: 차선
        lab1 = rospy.get_param("~lab1_sec", 8.0)       # 미션2: 라바콘
        bag_ab = rospy.get_param("~bag_ab_sec", 5.0)   # 미션3: AB bag
        rot1 = rospy.get_param("~rot1_sec", 8.0)       # 미션4: 로터리
        cam2 = rospy.get_param("~cam2_sec", 3.0)       # 미션5: 차선
        lab2 = rospy.get_param("~lab2_sec", 8.0)       # 미션6: 라바콘
        cam3 = rospy.get_param("~cam3_sec", 3.0)       # 미션7: 차선
        hard = rospy.get_param("~hard_sec", 5.0)       # 미션8: 직진 하드코딩
        gate = rospy.get_param("~gate_sec", 8.0)       # 미션9: 게이트
        cam4 = rospy.get_param("~cam4_sec", 4.0)       # 미션10: 차선
        park = rospy.get_param("~park_sec", 8.0)       # 미션11: 파킹

        # ---- 하드코딩 주행 기본값 ----
        self.hard_motor_default = rospy.get_param("~hard_motor_speed", 1000.0)
        self.hard_servo_default = rospy.get_param("~hard_servo_center", 0.574)
        timer_hz = rospy.get_param("~timer_hz", 20.0)

        # ---- 퍼블리셔 (latched for enables) ----
        self.pub_camera_enable = rospy.Publisher(self.camera_enable_topic, Bool, queue_size=1, latch=True)
        self.pub_lidar_enable = rospy.Publisher(self.lidar_enable_topic, Bool, queue_size=1, latch=True)

        self.pub_crosswalk_enable = rospy.Publisher(self.crosswalk_enable_topic, Bool, queue_size=1, latch=True)
        self.pub_ab_enable = rospy.Publisher(self.ab_enable_topic, Bool, queue_size=1, latch=True)
        self.pub_ab_sign_enable = rospy.Publisher(self.ab_sign_enable_topic, Bool, queue_size=1, latch=True)
        self.pub_gate_enable = rospy.Publisher(self.gate_enable_topic, Bool, queue_size=1, latch=True)
        self.pub_rotary_enable = rospy.Publisher(self.rotary_enable_topic, Bool, queue_size=1, latch=True)
        self.pub_parking_enable = rospy.Publisher(self.parking_enable_topic, Bool, queue_size=1, latch=True)

        self.pub_bag_ab = rospy.Publisher(self.bag_ab_enable_topic, Bool, queue_size=1, latch=True)
        self.pub_bag_rotary = rospy.Publisher(self.bag_rotary_enable_topic, Bool, queue_size=1, latch=True)
        self.pub_bag_parking = rospy.Publisher(self.bag_parking_enable_topic, Bool, queue_size=1, latch=True)

        self.pub_motor = rospy.Publisher(self.motor_topic, Float64, queue_size=1)
        self.pub_servo = rospy.Publisher(self.servo_topic, Float64, queue_size=1)

        # ---- 미션 시퀀스 ----
        # 1) 차선(카메라) -> 2) 라바콘(라이다) -> 3) AB bag -> 4) 로터리(라이다+bag)
        # 5) 차선(카메라) -> 6) 라바콘(라이다) -> 7) 차선(카메라)
        # 8) 직진 하드코딩 -> 9) 게이트(라이다) -> 10) 차선(카메라) -> 11) 파킹(라이다+bag)
        self.phases: List[Phase] = [
            Phase("M1_LANE_CAMERA", duration=cam1, camera=True),
            Phase("M2_LABACORN_LIDAR", duration=lab1, lidar=True),
            Phase("M3_AB_BAG", duration=bag_ab, bag_ab=True),
            Phase("M4_ROTARY_LIDAR_BAG", duration=rot1, lidar=True, bag_rotary=True),
            Phase("M5_LANE_CAMERA", duration=cam2, camera=True),
            Phase("M6_LABACORN_LIDAR", duration=lab2, lidar=True),
            Phase("M7_LANE_CAMERA", duration=cam3, camera=True),
            Phase("M8_HARD_DRIVE", duration=hard, hard=True),
            Phase("M9_GATE_LIDAR", duration=gate, lidar=True),
            Phase("M10_LANE_CAMERA", duration=cam4, camera=True),
            Phase("M11_PARKING_LIDAR_BAG", duration=park, lidar=True, bag_parking=True),
        ]

        self.current_idx = 0
        self.phase_start = rospy.Time.now()
        self.hard_active = False

        self.apply_phase(self.phases[self.current_idx])

        timer_period = 1.0 / max(1.0, timer_hz)
        self.timer = rospy.Timer(rospy.Duration.from_sec(timer_period), self.on_timer)
        rospy.loginfo("[mission_node_py] ready with %d phases, timer=%.2f Hz", len(self.phases), 1.0 / timer_period)

    # ---- helpers ----
    def publish_enable_set(self, camera_on: bool, lidar_on: bool):
        cam_msg = Bool(data=camera_on)
        lidar_msg = Bool(data=lidar_on)

        self.pub_camera_enable.publish(cam_msg)
        self.pub_crosswalk_enable.publish(cam_msg)
        self.pub_ab_enable.publish(cam_msg)
        self.pub_ab_sign_enable.publish(cam_msg)

        self.pub_lidar_enable.publish(lidar_msg)
        self.pub_gate_enable.publish(lidar_msg)
        self.pub_rotary_enable.publish(lidar_msg)
        self.pub_parking_enable.publish(lidar_msg)

    def publish_bag_enables(self, ab_on: bool, rotary_on: bool, parking_on: bool):
        self.pub_bag_ab.publish(Bool(data=ab_on))
        self.pub_bag_rotary.publish(Bool(data=rotary_on))
        self.pub_bag_parking.publish(Bool(data=parking_on))

    def publish_hard_drive(self, motor: float, servo: float):
        self.pub_motor.publish(Float64(motor))
        self.pub_servo.publish(Float64(servo))

    def stop_hard_drive(self):
        if not self.hard_active:
            return
        self.pub_motor.publish(Float64(0.0))
        self.hard_active = False

    def apply_phase(self, phase: Phase):
        # enable sets
        self.publish_enable_set(phase.camera, phase.lidar)
        self.publish_bag_enables(phase.bag_ab, phase.bag_rotary, phase.bag_parking)

        self.hard_active = phase.hard
        if phase.hard:
            motor = phase.hard_motor if phase.hard_motor is not None else self.hard_motor_default
            servo = phase.hard_servo if phase.hard_servo is not None else self.hard_servo_default
            self.publish_hard_drive(motor, servo)

        rospy.loginfo("[mission_node_py] Phase -> %s (dur=%.1fs cam=%d lidar=%d bag_ab=%d bag_rot=%d bag_park=%d hard=%d)",
                      phase.name, phase.duration, phase.camera, phase.lidar,
                      phase.bag_ab, phase.bag_rotary, phase.bag_parking, phase.hard)

    def next_phase(self):
        self.stop_hard_drive()
        self.current_idx += 1
        if self.current_idx >= len(self.phases):
            rospy.loginfo("[mission_node_py] all phases finished")
            self.publish_enable_set(False, False)
            self.publish_bag_enables(False, False, False)
            return
        self.phase_start = rospy.Time.now()
        self.apply_phase(self.phases[self.current_idx])

    # ---- timer ----
    def on_timer(self, _event):
        if self.current_idx >= len(self.phases):
            return

        phase = self.phases[self.current_idx]
        elapsed = (rospy.Time.now() - self.phase_start).to_sec()
        if elapsed >= phase.duration:
            self.next_phase()
            return

        if phase.hard:
            motor = phase.hard_motor if phase.hard_motor is not None else self.hard_motor_default
            servo = phase.hard_servo if phase.hard_servo is not None else self.hard_servo_default
            self.publish_hard_drive(motor, servo)  # keep commanding during hard step


if __name__ == "__main__":
    try:
        node = MissionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
