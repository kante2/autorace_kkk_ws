#!/usr/bin/env python3
# main_node_pyt.py
# 15초 lane_solo -> 8초 labacorn_solo 순차 실행

import rospy
import subprocess
import signal


class MissionRunner:
    def __init__(self):
        rospy.init_node("autorace_main_decision_py")

        # 실행할 미션 리스트: 순서, rosrun 커맨드, duration(초)
        self.missions = [
            # 3초간 차선 추종
            {
                "name": "LANE_SOLO_3S",
                "cmd": ["rosrun", "decision_node", "lane_solo"],
                "duration": 5.0,
            },

            # 빨간색: 2초간 색상코드 1을 퍼블리시 (mission_lane에서 속도=1000에 해당하도록)
            {
                "name": "COLOR_RED_2S",
                "cmd": [
                    "rostopic", "pub", "-r", "10",
                    "/perception/center_color_px",
                    "geometry_msgs/PointStamped",
                    "header: {stamp: now}",
                    "point: {x: 1, y: 0, z: 0}",
                ],
                "duration": 2.0,
            },

            # 파란색: 3초간 색상코드 2 퍼블리시 (mission_lane에서 속도=1500에 해당하도록)
            {
                "name": "COLOR_BLUE_3S",
                "cmd": [
                    "rostopic", "pub", "-r", "10",
                    "/perception/center_color_px",
                    "geometry_msgs/PointStamped",
                    "header: {stamp: now}",
                    "point: {x: 2, y: 0, z: 0}",
                ],
                "duration": 3.0,
            },

            # 5초간 차선 추종
            {
                "name": "LANE_SOLO_5S",
                "cmd": ["rosrun", "decision_node", "lane_solo"],
                "duration": 5.0,
            },

            # 7초간 정지 (모터 0을 7초 동안 계속 publish)
            {
                "name": "MOTOR_STOP_7S",
                "cmd": [
                    "rostopic", "pub", "-r", "10",
                    "/commands/motor/speed",
                    "std_msgs/Float64",
                    "data: 0.0",
                ],
                "duration": 7.0,
            },

            # 8초간 차선 추종
            {
                "name": "LANE_SOLO_8S",
                "cmd": ["rosrun", "decision_node", "lane_solo"],
                "duration": 8.0,
            },

            # 8초간 라바콘 미션
            {
                "name": "LABACORN_SOLO_8S",
                "cmd": ["rosrun", "decision_node", "labacorn_solo"],
                "duration": 8.0,
            },

            # 15초간 다시 차선 추종
            {
                "name": "LANE_SOLO_15S",
                "cmd": ["rosrun", "decision_node", "lane_solo"],
                "duration": 15.0,
            },
        ]

        self.current_idx = -1
        self.proc = None
        self.start_time = None

        # 10Hz 타이머로 상태 체크
        self.timer = rospy.Timer(rospy.Duration(0.1), self.on_timer)

        rospy.loginfo("[main_py] MissionRunner init done, start first mission")
        self.start_next_mission()

    def start_next_mission(self):
        # 이전 프로세스가 남아 있으면 정리
        self.stop_current_mission()

        self.current_idx += 1
        if self.current_idx >= len(self.missions):
            rospy.loginfo("[main_py] All missions finished, shutting down.")
            rospy.signal_shutdown("all missions done")
            return

        m = self.missions[self.current_idx]
        rospy.loginfo("[main_py] Starting mission %d: %s (duration=%.1fs)",
                      self.current_idx, m["name"], m["duration"])

        try:
            # rosrun decision_node XXX_solo 실행
            self.proc = subprocess.Popen(m["cmd"])
            self.start_time = rospy.Time.now()
        except Exception as e:
            rospy.logerr("[main_py] Failed to start mission %s: %s", m["name"], str(e))
            # 실패하면 바로 다음 미션으로 넘어감
            self.start_next_mission()

    def stop_current_mission(self):
        if self.proc is None:
            return

        if self.proc.poll() is None:
            rospy.loginfo("[main_py] Stopping current mission (pid=%d)", self.proc.pid)
            # SIGINT 먼저 보내서 정상 종료 유도 (rosnode처럼 ctrl+c)
            self.proc.send_signal(signal.SIGINT)

            try:
                self.proc.wait(timeout=3.0)
            except Exception:
                rospy.logwarn("[main_py] Mission did not exit, sending SIGTERM")
                self.proc.terminate()

        self.proc = None
        self.start_time = None

    def on_timer(self, event):
        # 현재 미션이 없으면 할 일 없음
        if self.proc is None or self.start_time is None:
            return

        # 프로세스가 이미 죽어버렸으면 바로 다음 미션으로
        if self.proc.poll() is not None:
            rospy.logwarn("[main_py] Mission process exited early, going to next.")
            self.start_next_mission()
            return

        # 경과 시간 체크
        now = rospy.Time.now()
        elapsed = (now - self.start_time).to_sec()

        m = self.missions[self.current_idx]
        if elapsed >= m["duration"]:
            rospy.loginfo("[main_py] Mission %s time over (%.2fs / %.2fs), switching to next.",
                          m["name"], elapsed, m["duration"])
            self.start_next_mission()


if __name__ == "__main__":
    try:
        runner = MissionRunner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
