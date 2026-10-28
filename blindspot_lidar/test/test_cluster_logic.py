import math

from blindspot_lidar import cluster_logic
import numpy as np
import pytest

# 실측 스캔과 같은 모양: 한 바퀴 360°, 약 0.5° 간격
NUM = 720
INC = 2 * math.pi / NUM
ANGLE_MIN = -math.pi
RANGE_MIN = 0.05
RANGE_MAX = 12.0

CLUSTER_DISTANCE = 0.10
MIN_POINTS = 3
X_MIN, X_MAX, Y_MAX = 0.0, 3.0, 1.5

IDENTITY = np.eye(3)


def empty_scan():
    return np.full(NUM, np.inf)


def add_object(ranges, distance, angle_deg, width=0.10):
    """폭 width [m]인 물체 앞면을 distance, angle_deg 방향에 찍는다."""
    half = math.atan2(width / 2, distance)
    center = math.radians(angle_deg)
    for i in range(NUM):
        angle = ANGLE_MIN + i * INC
        diff = math.atan2(math.sin(angle - center), math.cos(angle - center))
        if abs(diff) <= half:
            ranges[i] = distance
    return ranges


def run(ranges, translation=(0.0, 0.0, 0.15), rotation=IDENTITY, inc=INC):
    points, index = cluster_logic.scan_to_points(ranges, ANGLE_MIN, inc, RANGE_MIN, RANGE_MAX)
    floor = cluster_logic.to_floor(points, translation, rotation)
    full = cluster_logic.is_full_circle(len(ranges), inc)
    return cluster_logic.find_obstacles(
        floor, index, len(ranges), full, CLUSTER_DISTANCE, MIN_POINTS, X_MIN, X_MAX, Y_MAX)


def test_empty_scan_gives_no_obstacle():
    assert run(empty_scan()) == []


def test_invalid_ranges_are_dropped():
    ranges = empty_scan()
    ranges[:10] = np.nan
    ranges[10:20] = 0.0          # range_min 미만
    ranges[20:30] = 50.0         # range_max 초과
    points, index = cluster_logic.scan_to_points(ranges, ANGLE_MIN, INC, RANGE_MIN, RANGE_MAX)
    assert len(points) == 0 and len(index) == 0


@pytest.mark.parametrize(
    'distance, angle_deg, expected_x, expected_y',
    [
        (1.0, 0.0, 1.00, 0.00),
        (2.0, 0.0, 2.00, 0.00),
        (1.0, 20.0, 0.94, 0.34),
        (2.0, 20.0, 1.88, 0.68),
    ],
)
def test_evaluation_positions(distance, angle_deg, expected_x, expected_y):
    found = run(add_object(empty_scan(), distance, angle_deg))
    assert len(found) == 1
    x, y = found[0]
    assert x == pytest.approx(expected_x, abs=0.02)
    assert y == pytest.approx(expected_y, abs=0.02)


def test_two_objects_are_separate():
    ranges = add_object(empty_scan(), 1.0, 0.0)
    ranges = add_object(ranges, 2.0, 20.0)
    assert len(run(ranges)) == 2


def test_small_cluster_is_dropped():
    ranges = empty_scan()
    ranges[NUM // 2] = 1.0       # 정면 점 하나 (잡음)
    assert run(ranges) == []


def test_outside_area_is_not_published():
    ranges = add_object(empty_scan(), 1.0, 180.0)   # 지그 뒤 (x < 0)
    ranges = add_object(ranges, 1.0, 90.0)          # 왼쪽 옆 (|y| > 1.5)
    ranges = add_object(ranges, 2.0, 60.0)          # y = 1.73
    ranges = add_object(ranges, 3.5, 0.0)           # x > 3.0
    assert run(ranges) == []


def test_wrap_around_merges_first_and_last():
    # 스캔이 정면(0°)에서 시작하면 정면 물체가 배열의 처음과 끝으로 나뉜다
    ranges = np.full(NUM, np.inf)
    ranges[:5] = 1.0
    ranges[-5:] = 1.0
    points, index = cluster_logic.scan_to_points(ranges, 0.0, INC, RANGE_MIN, RANGE_MAX)
    floor = cluster_logic.to_floor(points, (0.0, 0.0, 0.0), IDENTITY)
    clusters = cluster_logic.cluster_points(
        floor, index, NUM, CLUSTER_DISTANCE, MIN_POINTS, True)
    assert len(clusters) == 1
    assert clusters[0].mean(axis=0) == pytest.approx([1.0, 0.0], abs=0.01)


def test_partial_scan_does_not_wrap():
    assert not cluster_logic.is_full_circle(360, INC)   # 180°만 덮는 스캔
    assert cluster_logic.is_full_circle(NUM, INC)


def test_uses_scan_angle_increment():
    # 사양(0.72°)과 다른 간격이어도 메시지의 angle_increment로 각도를 계산한다
    num = 500
    inc = 2 * math.pi / num
    ranges = np.full(num, np.inf)
    i = round((math.radians(20.0) - ANGLE_MIN) / inc)
    ranges[i - 2:i + 3] = 1.0
    found = run(ranges, inc=inc)
    assert len(found) == 1
    assert math.degrees(math.atan2(found[0][1], found[0][0])) == pytest.approx(20.0, abs=0.5)


def test_transform_offset_and_yaw():
    # laser가 base_jig보다 앞으로 0.1m, 왼쪽으로 90° 돌아 있으면
    # laser의 -90° 방향 1m 점은 base_jig 정면 1.1m가 된다
    yaw = math.pi / 2
    rotation = cluster_logic.quaternion_to_matrix(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    found = run(add_object(empty_scan(), 1.0, -90.0), translation=(0.1, 0.0, 0.15),
                rotation=rotation)
    assert len(found) == 1
    assert found[0] == pytest.approx((1.1, 0.0), abs=0.02)


def test_non_positive_ranges_are_dropped():
    # range_min이 0이어도 0 이하 거리는 점으로 쓰지 않는다
    points, _ = cluster_logic.scan_to_points([0.0, -1.0, 1.0], 0.0, INC, 0.0, RANGE_MAX)
    assert len(points) == 1


@pytest.mark.parametrize(
    'angle_min, angle_increment, range_min, range_max',
    [
        (ANGLE_MIN, 0.0, RANGE_MIN, RANGE_MAX),        # 각도 간격 0
        (ANGLE_MIN, math.nan, RANGE_MIN, RANGE_MAX),   # 각도 간격 nan
        (ANGLE_MIN, INC, -0.1, RANGE_MAX),             # 최소 거리 음수
        (ANGLE_MIN, INC, RANGE_MIN, math.nan),         # 최대 거리 nan
        (ANGLE_MIN, INC, 5.0, 1.0),                    # 최대 < 최소
    ],
)
def test_bad_scan_is_rejected(angle_min, angle_increment, range_min, range_max):
    with pytest.raises(ValueError):
        cluster_logic.scan_to_points([1.0], angle_min, angle_increment, range_min, range_max)


def test_infinite_range_max_is_allowed():
    points, _ = cluster_logic.scan_to_points([1.0], 0.0, INC, RANGE_MIN, math.inf)
    assert len(points) == 1


def test_quaternion_is_normalized_and_zero_is_rejected():
    assert cluster_logic.quaternion_to_matrix(0.0, 0.0, 0.0, 2.0) == pytest.approx(IDENTITY)
    with pytest.raises(ValueError):
        cluster_logic.quaternion_to_matrix(0.0, 0.0, 0.0, 0.0)


@pytest.mark.parametrize(
    'cluster_distance, min_points, x_min, x_max, y_max',
    [
        (0.0, 3, 0.0, 3.0, 1.5),        # 묶는 거리 0
        (0.1, 0, 0.0, 3.0, 1.5),        # 최소 점 수 0
        (0.1, 3, 3.0, 0.0, 1.5),        # x 범위 거꾸로
        (0.1, 3, 0.0, 3.0, 0.0),        # y 폭 0
        (math.nan, 3, 0.0, 3.0, 1.5),   # nan
    ],
)
def test_bad_params_are_rejected(cluster_distance, min_points, x_min, x_max, y_max):
    with pytest.raises(ValueError):
        cluster_logic.check_params(cluster_distance, min_points, x_min, x_max, y_max)


def test_default_params_are_accepted():
    cluster_logic.check_params(CLUSTER_DISTANCE, MIN_POINTS, X_MIN, X_MAX, Y_MAX)


def test_wrap_around_with_missing_first_beam():
    # 스캔이 정면에서 시작하고 첫 값이 비어 있어도 정면 물체가 둘로 나뉘거나 사라지지 않는다
    ranges = np.full(NUM, np.inf)
    ranges[[1, 2, NUM - 2, NUM - 1]] = 1.0
    ranges[NUM // 2] = 2.0                       # 뒤쪽 잡음 점 하나
    points, index = cluster_logic.scan_to_points(ranges, 0.0, INC, RANGE_MIN, RANGE_MAX)
    floor = cluster_logic.to_floor(points, (0.0, 0.0, 0.15), IDENTITY)
    found = cluster_logic.find_obstacles(
        floor, index, NUM, True, CLUSTER_DISTANCE, MIN_POINTS, X_MIN, X_MAX, Y_MAX)
    assert len(found) == 1
    assert found[0] == pytest.approx((1.0, 0.0), abs=0.01)
