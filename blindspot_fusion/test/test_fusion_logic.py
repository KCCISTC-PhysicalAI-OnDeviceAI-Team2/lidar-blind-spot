from collections import namedtuple

import pytest

from blindspot_fusion.fusion_logic import decide, select_sources

STALE_SEC = 0.5


@pytest.mark.parametrize(
    'lidar_time, camera_time, expected',
    [
        (10.0, 10.3, (True, True, [])),                  # 둘 다 최신
        (10.0, None, (True, False, ['camera none'])),    # 카메라 받은 적 없음
        (None, 10.0, (False, True, ['lidar none'])),     # LiDAR 받은 적 없음
        (10.0, 9.4, (True, False, ['camera stale'])),    # 카메라가 0.6초 오래됨
        (9.0, 10.0, (False, True, ['lidar stale'])),     # LiDAR가 1.0초 오래됨
        (10.0, 10.5, (False, True, ['lidar stale'])),    # 딱 0.5초 차이도 뺌
    ],
)
def test_select_sources(lidar_time, camera_time, expected):
    assert select_sources(lidar_time, camera_time, STALE_SEC) == expected


STOP_X = 1.5
SLOW_X = 2.5
HALF_WIDTH = 1.2
MIN_CONF = 0.5

# blindspot_msgs/Obstacle과 같은 모양의 가짜 장애물
Obs = namedtuple('Obs', ['label', 'x', 'y', 'confidence', 'source'])

LIDAR_STOP = Obs('unknown', 1.0, 0.0, 1.0, 'lidar')
LIDAR_SLOW = Obs('unknown', 2.0, 0.0, 1.0, 'lidar')
CAM_STOP = Obs('box', 1.0, 0.2, 0.9, 'camera')
CAM_SLOW = Obs('box', 2.0, 0.2, 0.9, 'camera')
CAM_NEAR = Obs('box', 0.8, 0.0, 0.9, 'camera')
CAM_LOW_CONF = Obs('box', 1.0, 0.0, 0.3, 'camera')
CAM_EDGE_CONF = Obs('box', 1.0, 0.0, 0.5, 'camera')
BEHIND = Obs('unknown', -0.5, 0.0, 1.0, 'lidar')


@pytest.mark.parametrize(
    'obstacles, expected',
    [
        ([], (0, False, False, None)),                           # 장애물 없음
        ([LIDAR_STOP], (2, True, False, LIDAR_STOP)),            # LiDAR만 정지 구역
        ([CAM_SLOW], (1, False, True, CAM_SLOW)),                # 카메라만 감속 구역
        ([CAM_LOW_CONF], (0, False, False, None)),               # 신뢰도 낮은 카메라는 무시
        ([CAM_EDGE_CONF], (2, False, True, CAM_EDGE_CONF)),      # 신뢰도 = 기준값은 사용
        ([LIDAR_SLOW, CAM_STOP], (2, True, True, CAM_STOP)),     # 더 위험한 판단(정지)을 고름
        ([CAM_STOP, CAM_NEAR], (2, False, True, CAM_NEAR)),      # 같은 판단이면 더 가까운 것
        ([BEHIND], (0, False, False, None)),                     # 지그 뒤는 무시
    ],
)
def test_decide(obstacles, expected):
    assert decide(obstacles, STOP_X, SLOW_X, HALF_WIDTH, MIN_CONF) == expected
