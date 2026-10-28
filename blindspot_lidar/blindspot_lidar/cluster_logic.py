"""LaserScan 점을 바닥 좌표로 바꾸고 묶는 계산 (ROS에 의존하지 않음)."""

import math

import numpy as np


def check_params(cluster_distance, min_points, x_min, x_max, y_max):
    """설정 값이 말이 되는지 확인한다. 아니면 ValueError."""
    if not all(math.isfinite(v) for v in (cluster_distance, x_min, x_max, y_max)):
        raise ValueError('설정 값에 숫자가 아닌 값(nan, inf)이 있음')
    if cluster_distance <= 0:
        raise ValueError(f'cluster_distance는 0보다 커야 함: {cluster_distance}')
    if min_points < 1:
        raise ValueError(f'min_points는 1 이상이어야 함: {min_points}')
    if x_min >= x_max:
        raise ValueError(f'x_min({x_min})이 x_max({x_max})보다 작아야 함')
    if y_max <= 0:
        raise ValueError(f'y_max는 0보다 커야 함: {y_max}')


def check_scan(angle_min, angle_increment, range_min, range_max):
    """스캔 메시지의 각도 · 거리 설정이 쓸 수 있는 값인지 확인한다. 아니면 ValueError."""
    if not all(math.isfinite(v) for v in (angle_min, angle_increment, range_min)):
        raise ValueError('스캔의 angle_min, angle_increment, range_min에 nan 또는 inf가 있음')
    if angle_increment == 0:
        raise ValueError('스캔의 angle_increment가 0')
    if range_min < 0 or math.isnan(range_max) or range_max < range_min:
        raise ValueError(f'스캔의 거리 범위가 이상함: {range_min} ~ {range_max}')


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max):
    """
    LaserScan의 거리 값을 laser 프레임의 (x, y) 점으로 바꾼다.

    각도는 스캔 메시지의 angle_min, angle_increment로 계산한다(사양 값을 쓰지 않음).
    inf, nan, 0 이하, range_min 미만, range_max 초과인 값은 버린다.
    각도 · 거리 설정이 이상한 스캔이면 ValueError (check_scan).
    돌려주는 값: (N, 2) 점 배열과, 각 점이 원래 몇 번째 값이었는지 (N,) 배열.
    """
    check_scan(angle_min, angle_increment, range_min, range_max)
    ranges = np.asarray(ranges, dtype=float)
    index = np.arange(ranges.size)
    valid = (np.isfinite(ranges) & (ranges > 0)
             & (ranges >= range_min) & (ranges <= range_max))
    angles = angle_min + index[valid] * angle_increment
    r = ranges[valid]
    points = np.column_stack((r * np.cos(angles), r * np.sin(angles)))
    return points, index[valid]


def quaternion_to_matrix(x, y, z, w):
    """
    쿼터니언을 3x3 회전 행렬로 바꾼다.

    길이가 1이 아니면 1로 맞춘다. 길이가 0이거나 nan이면 ValueError.
    """
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('변환의 회전 값(쿼터니언)이 이상함')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def to_floor(points, translation, rotation):
    """
    점을 laser 프레임에서 base_jig 바닥 좌표 (x, y)로 바꾼다.

    translation: base_jig 기준 laser 위치 (x, y, z) [m].
    rotation: base_jig 기준 laser 방향 3x3 회전 행렬.
    스캔 평면의 점(z = 0)을 3차원으로 변환한 뒤 높이(z)를 버려 바닥에 내린다.
    """
    if len(points) == 0:
        return np.empty((0, 2))
    points3 = np.column_stack((points, np.zeros(len(points))))
    moved = points3 @ np.asarray(rotation).T + np.asarray(translation)
    return moved[:, :2]


def is_full_circle(num_ranges, angle_increment):
    """스캔 한 번이 한 바퀴(360°)를 덮으면 True. 이때 처음과 끝 값이 이웃한다."""
    return abs(num_ranges * abs(angle_increment) - 2 * np.pi) < 2 * abs(angle_increment)


def cluster_points(points, index, num_ranges, cluster_distance, min_points, full_circle):
    """
    스캔 순서로 이웃한 점끼리 거리가 cluster_distance 이하이면 같은 묶음으로 묶는다.

    index: 각 점의 원래 순번(scan_to_points가 돌려준 값). num_ranges: 전체 값 개수.
    full_circle이 True(360° 스캔)이면 처음과 끝이 이어지므로,
    마지막 점과 첫 점이 가까우면 한 묶음으로 합친다(그 사이 값이 비어 있어도 됨).
    점이 min_points개보다 적은 묶음은 버린다.
    돌려주는 값: 묶음마다 점 배열 (M, 2)의 리스트.
    """
    n = len(points)
    if n == 0:
        return []

    # 이웃 점 사이 거리가 멀면 그 자리에서 묶음을 끊는다
    gaps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cut = np.flatnonzero(gaps > cluster_distance) + 1
    groups = np.split(np.arange(n), cut)

    # 끝과 처음이 이어지는 경우(스캔 한 바퀴가 360°이고 두 점이 가까움) 합친다
    if (len(groups) > 1 and full_circle
            and np.linalg.norm(points[-1] - points[0]) <= cluster_distance):
        groups[0] = np.concatenate((groups.pop(), groups[0]))

    return [points[g] for g in groups if len(g) >= min_points]


def find_obstacles(points_floor, index, num_ranges, full_circle,
                   cluster_distance, min_points, x_min, x_max, y_max):
    """
    바닥 좌표의 점들을 묶어 장애물 위치 (x, y) 리스트를 돌려준다.

    위치는 묶음 점들의 중심이다. 중심이 x_min ≤ x ≤ x_max, |y| ≤ y_max 안인 것만 남긴다.
    """
    obstacles = []
    for cluster in cluster_points(points_floor, index, num_ranges,
                                  cluster_distance, min_points, full_circle):
        cx, cy = cluster.mean(axis=0)
        if x_min <= cx <= x_max and abs(cy) <= y_max:
            obstacles.append((float(cx), float(cy)))
    return obstacles
