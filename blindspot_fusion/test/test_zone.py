import pytest

from blindspot_fusion.zone import classify_zone

STOP_X = 1.5
SLOW_X = 2.5
HALF_WIDTH = 0.7  # config/fusion_node.yaml의 half_width와 같은 값


@pytest.mark.parametrize(
    'x, y, expected',
    [
        # 평가 위치
        (1.00, 0.00, 2),   # 1.0m · 0° → 정지
        (2.00, 0.00, 1),   # 2.0m · 0° → 감속
        (0.94, 0.34, 2),   # 1.0m · 20° → 정지
        (1.88, 0.68, 1),   # 2.0m · 20° → 감속

        # 경계
        (0.00, 0.00, 2),   # 지그 바로 앞(x = 0) → 정지
        (1.50, 0.00, 1),   # 정지 끝 경계(x = 1.5) → 감속
        (2.50, 0.00, 0),   # 감속 끝 경계(x = 2.5) → 통과
        (1.00, 0.70, 2),   # 왼쪽 폭 경계(|y| = 0.7) → 구역 안
        (1.00, -0.70, 2),  # 오른쪽 폭 경계 → 구역 안

        # 구역 밖
        (1.00, 0.82, 0),   # 왼쪽 벽 위치(0.82m) → 통과
        (2.00, -0.82, 0),  # 오른쪽 벽 위치 → 통과
        (-0.50, 0.00, 0),  # 지그 뒤 → 통과
        (3.00, 0.00, 0),   # 감속 구역 너머 → 통과
    ],
)
def test_classify_zone(x, y, expected):
    assert classify_zone(x, y, STOP_X, SLOW_X, HALF_WIDTH) == expected
