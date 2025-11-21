import numpy as np
import math
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs

# lazy tf buffer (필요할 때만 생성해서 사용)
_tf_buffer = None
_tf_listener = None

def _get_tf_buffer():
    global _tf_buffer, _tf_listener
    if _tf_buffer is None:
        _tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        _tf_listener = tf2_ros.TransformListener(_tf_buffer)
    return _tf_buffer

def _lookup_base_heading(buf, target_frame, source_frame):
    """
    target_frame에서 본 source_frame(base_link) yaw를 조회.
    """
    try:
        tf = buf.lookup_transform(target_frame, source_frame,
                                  rospy.Time(0), timeout=rospy.Duration(0.2))
        q = tf.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return yaw
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("base_link heading 조회 실패(%s->%s): %s", target_frame, source_frame, str(e))
        return None

def compute_goal_from_lines(lines, min_width, min_depth, wall_offset):
    """
    parking_detect에서 받은 lines로부터
    1) 왼/오른쪽 벽, 뒷벽 라인 분류
    2) 폭/깊이 계산
    3) 뒷벽 중앙 + 법선 방향으로 goal pose 계산

    반환:
      success(bool), goal_x, goal_y, goal_yaw
    """

    if len(lines) < 3:
        print("line is under 3... ")
        return False, 0.0, 0.0, 0.0

    # inliers 많은 순으로 정렬하고 상위 3개 사용
    lines3 = sorted(lines, key=lambda x: -x["num_inliers"])[:3]

    # 각 라인의 법선 벡터 n = (a, b)
    normals = []
    for ln in lines3:
        a = ln["a"]
        b = ln["b"]
        n = np.array([a, b])  # 이미 정규화 되어 있음
        normals.append(n)

    # 3개 중 평행한 두 라인 찾기 (법선끼리의 내적 최대)
    best_pair = None
    best_dot = -1.0

    for i in range(3):
        for j in range(i + 1, 3):
            dot = abs(np.dot(normals[i], normals[j]))
            if dot > best_dot:
                best_dot = dot
                best_pair = (i, j)

    if best_pair is None:
        return False, 0.0, 0.0, 0.0

    i, j = best_pair
    rest = [0, 1, 2]
    rest.remove(i)
    rest.remove(j)
    k = rest[0]

    side1 = lines3[i]
    side2 = lines3[j]
    back  = lines3[k]

    n1 = normals[i]
    n2 = normals[j]
    n_back = normals[k]

    # 폭 계산: 평행 두 라인 사이 거리
    a1, b1, c1 = side1["a"], side1["b"], side1["c"]
    a2, b2, c2 = side2["a"], side2["b"], side2["c"]

    # 법선 방향 통일 (dot < 0 이면 뒤집기)
    if np.dot(np.array([a1, b1]), np.array([a2, b2])) < 0:
        a2, b2, c2 = -a2, -b2, -c2

    width = abs(c2 - c1)  # a^2 + b^2 = 1 가정하므로 이렇게 가능

    if width < min_width:
        rospy.loginfo("parking width %.2f < min_width %.2f", width, min_width)
        return False, 0.0, 0.0, 0.0

    # 깊이 계산: 로봇(원점)에서 뒷벽까지 거리
    a_b, b_b, c_b = back["a"], back["b"], back["c"]
    depth = abs(c_b)

    if depth < min_depth:
        rospy.loginfo("parking depth %.2f < min_depth %.2f", depth, min_depth)
        return False, 0.0, 0.0, 0.0

    # 뒷벽 중앙점 C (RANSAC inlier들의 centroid 사용)
    C = back["centroid"]  # shape (2,), [x, y]

    # 뒷벽 법선 방향을 슬롯 안쪽(로봇 쪽)으로 정렬
    # v = (로봇 - C) = -C
    v = -C
    if np.dot(n_back, v) < 0:
        n_back = -n_back

    # goal position: 뒷벽에서 wall_offset만큼 안쪽
    goal_pos = C + n_back * wall_offset
    goal_x = goal_pos[0]
    goal_y = goal_pos[1]

    # goal yaw: 뒷벽 법선 방향을 기준으로 반시계 90도 회전
    goal_yaw = math.atan2(n_back[1], n_back[0]) - math.pi / 2.0

    return True, goal_x, goal_y, goal_yaw

# _yaw_offset_cached = None

# def _get_goal_yaw_offset():
#     """
#     Lazily fetch yaw offset between lidar frame과 base_link.
#     기본값은 180deg (후방=0deg 스캔)이며, 필요시 ~goal_yaw_offset_deg로 덮어쓰기.
#     """
#     global _yaw_offset_cached
#     if _yaw_offset_cached is None:
#         yaw_offset_deg = rospy.get_param("~goal_yaw_offset_deg", 180.0)
#         _yaw_offset_cached = math.radians(yaw_offset_deg)
#     return _yaw_offset_cached

def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def publish_parking_goal(goal_pub,
                         goal_x,
                         goal_y,
                         goal_yaw,
                         frame_id="base_link",
                         target_frame="odom",
                         tf_buffer=None,
                         align_heading=True):
    """
    계산된 goal pose를 PoseStamped로 만들어 publish하는 함수
    goal_pub: geometry_msgs/PoseStamped publisher
    frame_id: goal_x, goal_y, goal_yaw가 표현된 프레임 (기본 base_link)
    target_frame: 퍼블리시할 목표 프레임(기본 odom). None이면 frame_id 그대로 퍼블리시
    tf_buffer: target_frame 변환에 사용할 tf2_ros.Buffer (없으면 내부에서 lazy 생성)
    """
    msg = PoseStamped()
    # transform 시점 문제를 피하기 위해 최신 TF로 변환할 수 있도록 0 사용
    # (tf2에서는 stamp=0이면 최신 available 변환을 사용)
    msg.header.stamp = rospy.Time(0)
    msg.header.frame_id = frame_id

    msg.pose.position.x = goal_x
    msg.pose.position.y = goal_y
    msg.pose.position.z = 0.0

    #yaw_offset = _get_goal_yaw_offset()
    #yaw_with_offset = _normalize_angle(goal_yaw + yaw_offset)
    q = quaternion_from_euler(0.0, 0.0, goal_yaw)
    msg.pose.orientation = Quaternion(*q)

    # 필요 시 tf로 target_frame으로 변환
    out_msg = msg
    if target_frame and target_frame != frame_id:
        buf = tf_buffer if tf_buffer is not None else _get_tf_buffer()
        try:
            out_msg = buf.transform(msg, target_frame, timeout=rospy.Duration(0.2))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("goal 변환 실패(%s->%s): %s", frame_id, target_frame, str(e))
            return None, None
        # goal yaw를 base_link 현재 헤딩과 정렬
        if align_heading:
            base_yaw = _lookup_base_heading(buf, target_frame, frame_id)
            if base_yaw is not None:
                q_heading = quaternion_from_euler(0.0, 0.0, base_yaw)
                out_msg.pose.orientation = Quaternion(*q_heading)
    goal_pub.publish(out_msg)
    return goal_yaw, out_msg  # yaw_with_offset
