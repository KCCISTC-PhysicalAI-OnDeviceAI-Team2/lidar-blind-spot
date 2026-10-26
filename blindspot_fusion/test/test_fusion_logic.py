import pytest

from blindspot_fusion.fusion_logic import select_sources

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
