import numpy as np
import math
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

def compute_goal_from_lines(lines,
                            min_width=1.8,
                            min_depth=3.5,
                            wall_offset=0.5):
    """
    parking_detect에서 받은 lines로부터
    1) 왼/오른쪽 벽, 뒷벽 라인 분류
    2) 폭/깊이 계산
    3) 뒷벽 중앙 + 법선 방향으로 goal pose 계산

    반환:
      success(bool), goal_x, goal_y, goal_yaw
    """

    if len(lines) < 3:
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

    # goal yaw: n_back 방향을 바라보도록
    goal_yaw = math.atan2(n_back[1], n_back[0])

    return True, goal_x, goal_y, goal_yaw

def publish_parking_goal(goal_pub,
                         goal_x,
                         goal_y,
                         goal_yaw,
                         frame_id="base_link"):
    """
    계산된 goal pose를 PoseStamped로 만들어 publish하는 함수
    goal_pub: geometry_msgs/PoseStamped publisher
    """
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id

    msg.pose.position.x = goal_x
    msg.pose.position.y = goal_y
    msg.pose.position.z = 0.0

    q = quaternion_from_euler(0.0, 0.0, goal_yaw)
    msg.pose.orientation = Quaternion(*q)

    goal_pub.publish(msg)
