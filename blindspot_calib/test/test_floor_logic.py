"""floor_logic(검출 상자 -> 바닥 좌표) 시험. ROS 없이 pytest로 돈다."""

from pathlib import Path
import re

from blindspot_calib import floor_logic
from blindspot_calib.homography import float_text, load_homography_yaml
import numpy as np
import pytest

CONFIG = Path(__file__).resolve().parents[1] / 'config'
RANGE = (0.0, 3.0, 1.5)


def sample_h():
    """config/homography.yaml의 행렬 (2026-10-28 줄자 장면 7점)."""
    return load_homography_yaml(CONFIG / 'homography.yaml')


def test_bottom_center_uses_lower_edge():
    # 영상 v는 아래로 커지므로 아래 가장자리는 center_y + size_y / 2
    assert floor_logic.bottom_center(320.0, 200.0, 60.0, 100.0) == (320.0, 250.0)


def test_box_on_sample_maps_near_one_meter():
    # sample 영상의 박스: 가로 302~362, 세로 110~228 -> 라이다로 앞면 0.98m
    found = floor_logic.box_to_floor(sample_h(), 332.0, 169.0, 60.0, 118.0, *RANGE)
    assert found is not None
    x, y = found
    assert abs(x - 1.0) < 0.05
    assert abs(y) < 0.05


def test_sample_dots_map_to_measured_points():
    # 2026-10-28 줄자 장면: 바닥 점(픽셀) -> 줄자로 잰 위치, 3cm 안
    dots = {(337.0, 226.0): (1.0, 0.0),
            (94.7, 143.9): (1.5, 0.5), (564.0, 141.0): (1.5, -0.5),
            (152.5, 96.0): (2.0, 0.5), (509.5, 95.0): (2.0, -0.5),
            (185.2, 68.4): (2.5, 0.5), (475.7, 65.8): (2.5, -0.5)}
    h = sample_h()
    for (u, v), (x, y) in dots.items():
        # 높이 0인 상자 = 그 점 자체
        found = floor_logic.box_to_floor(h, u, v, 0.0, 0.0, -10.0, 10.0, 10.0)
        assert np.hypot(found[0] - x, found[1] - y) < 0.03


def test_left_in_image_is_left_on_floor():
    h = sample_h()
    left = floor_logic.box_to_floor(h, 150.0, 200.0, 40.0, 40.0, *RANGE)
    right = floor_logic.box_to_floor(h, 490.0, 200.0, 40.0, 40.0, *RANGE)
    assert left[1] > 0 > right[1]


def test_above_horizon_is_dropped():
    # 수평선(약 v = -60) 위는 바닥이 아니므로 버린다
    assert floor_logic.box_to_floor(sample_h(), 320.0, -200.0, 40.0, 40.0, *RANGE) is None


def test_far_box_outside_range_is_dropped():
    # 수평선 바로 아래는 x가 아주 멀다 -> x_max(3m) 밖이라 버린다
    assert floor_logic.box_to_floor(sample_h(), 320.0, 0.0, 40.0, 20.0, *RANGE) is None


def test_side_range_limit():
    h = sample_h()
    u, v = 20.0, 228.0  # 1m 줄 왼쪽 끝, y 약 +0.5m
    assert floor_logic.box_to_floor(h, u, v, 0.0, 0.0, 0.0, 3.0, 1.5) is not None
    assert floor_logic.box_to_floor(h, u, v, 0.0, 0.0, 0.0, 3.0, 0.3) is None


def test_outside_image():
    assert not floor_logic.outside_image(640.0, 480.0, (640, 480))
    assert floor_logic.outside_image(700.0, 300.0, (640, 480))
    assert floor_logic.outside_image(300.0, 500.0, (640, 480))


@pytest.mark.parametrize('values', [
    [1.0] * 8,
    [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, float('nan')],
    [1.0, 2.0, 3.0, 2.0, 4.0, 6.0, 0.0, 0.0, 1.0],
])
def test_check_homography_rejects_bad_values(values):
    with pytest.raises(ValueError):
        floor_logic.check_homography(values)


def test_check_homography_accepts_sample():
    h = floor_logic.check_homography(sample_h().flatten().tolist())
    assert h.shape == (3, 3)


@pytest.mark.parametrize('x_min, x_max, y_max', [
    (3.0, 0.0, 1.5), (0.0, 3.0, 0.0), (0.0, float('inf'), 1.5),
])
def test_check_range_rejects_bad_values(x_min, x_max, y_max):
    with pytest.raises(ValueError):
        floor_logic.check_range(x_min, x_max, y_max)


def test_float_text_always_float():
    assert float_text(1) == '1.0'
    assert float_text(0.0) == '0.0'
    assert float_text(1e-20) == '1e-20'
    assert float(float_text(0.0001672451633)) == pytest.approx(0.0001672451633)


def test_homography_yaml_values_are_all_floats():
    # ROS 파라미터 배열은 정수와 실수가 섞이면 읽지 못한다
    text = (CONFIG / 'homography.yaml').read_text(encoding='utf-8')
    values = re.search(r'homography:\s*\[([^\]]*)\]', text).group(1).split(',')
    assert len(values) == 9
    for value in values:
        assert any(c in value for c in '.e'), value


@pytest.mark.parametrize('box', [
    (float('nan'), 100.0, 20.0, 40.0, 0.9),
    (100.0, 100.0, 0.0, 40.0, 0.9),
    (100.0, 100.0, 20.0, -1.0, 0.9),
    (100.0, 100.0, 20.0, 40.0, 1.1),
    (100.0, 100.0, 20.0, 40.0, -0.1),
])
def test_check_box_rejects_bad_values(box):
    with pytest.raises(ValueError):
        floor_logic.check_box(*box)


def test_check_box_accepts_normal_box():
    floor_logic.check_box(332.0, 169.0, 60.0, 118.0, 0.9)


def test_box_touching_image_bottom_is_kept():
    # 가까운 물체는 상자 아래가 화면 끝(v = 480)에 닿는다. STOP 구역이라 버리면 안 된다
    found = floor_logic.box_to_floor(sample_h(), 320.0, 380.0, 200.0, 200.0, *RANGE)
    assert found is not None
    assert 0.0 < found[0] < 1.0


def test_homography_yaml_is_marked_provisional():
    text = (CONFIG / 'homography.yaml').read_text(encoding='utf-8')
    assert 'provisional: true' in text


def config_range():
    """config/camera_to_floor_node.yaml의 발행 범위 (x_min, x_max, y_max)."""
    text = (CONFIG / 'camera_to_floor_node.yaml').read_text(encoding='utf-8')
    return tuple(float(re.search(rf'^\s*{name}:\s*([-0-9.]+)', text, re.M).group(1))
                 for name in ('x_min', 'x_max', 'y_max'))


def test_config_publishes_only_inside_zone_width():
    # interface.md 6장 판단 구역 |y| ≤ 0.7m 밖의 검출은 발행하지 않는다
    x_min, x_max, y_max = config_range()
    assert y_max == pytest.approx(0.7)
    h = sample_h()
    inv = np.linalg.inv(h)
    for x, y, inside in [(2.0, 0.65, True), (2.0, -0.65, True),
                         (2.0, 0.75, False), (2.0, -0.75, False),
                         (1.97, 0.35, True), (1.97, -0.35, True)]:
        p = inv @ np.array([x, y, 1.0])
        u, v = p[0] / p[2], p[1] / p[2]
        found = floor_logic.box_to_floor(
            h, u, v, 0.0, 0.0, x_min, x_max, y_max, image_size=[640, 480])
        assert (found is not None) is inside, (x, y)
