"""검출 상자를 바닥 좌표로 바꾸는 계산 (ROS에 의존하지 않음)."""

import math

from blindspot_calib.homography import pixel_to_floor, validate_homography


def check_homography(values):
    """
    파라미터로 받은 9개 값을 3x3 호모그래피 행렬로 바꾼다.

    개수가 9가 아니거나, 숫자가 아닌 값이 있거나, 역행렬이 없으면 ValueError.
    """
    return validate_homography(values)


def check_range(x_min, x_max, y_max):
    """발행 범위 설정이 말이 되는지 확인한다. 아니면 ValueError."""
    if not all(math.isfinite(v) for v in (x_min, x_max, y_max)):
        raise ValueError('발행 범위에 숫자가 아닌 값(nan, inf)이 있음')
    if x_min >= x_max:
        raise ValueError(f'x_min({x_min})이 x_max({x_max})보다 작아야 함')
    if y_max <= 0:
        raise ValueError(f'y_max는 0보다 커야 함: {y_max}')


def check_box(center_x, center_y, size_x, size_y, score):
    """검출 상자 값이 쓸 수 있는지 확인한다. 숫자가 아니거나 크기 · 신뢰도가 이상하면 ValueError."""
    if not all(math.isfinite(v) for v in (center_x, center_y, size_x, size_y, score)):
        raise ValueError('검출 상자에 숫자가 아닌 값(nan, inf)이 있음')
    if size_x <= 0 or size_y <= 0:
        raise ValueError(f'검출 상자 크기가 0 이하: {size_x} x {size_y}')
    if not 0.0 <= score <= 1.0:
        raise ValueError(f'신뢰도가 0~1 밖: {score}')


def bottom_center(center_x, center_y, size_x, size_y):
    """상자 아래 가운데 점(물체가 바닥에 닿는 점)의 픽셀 (u, v). 영상 v는 아래로 커진다."""
    return center_x, center_y + size_y / 2.0


def outside_image(u, v, image_size):
    """픽셀이 영상 밖이면 True. 영상 크기가 호모그래피를 구한 크기와 다른지 알아보는 데 쓴다."""
    width, height = image_size
    return not (0 <= u <= width and 0 <= v <= height)


def valid_detection(center_x, center_y, size_x, size_y, score):
    """ROS에서 받은 검출 수치의 유한성, 크기, 신뢰도 범위를 확인한다."""
    try:
        check_box(center_x, center_y, size_x, size_y, score)
    except ValueError:
        return False
    return True


def box_to_floor(h, center_x, center_y, size_x, size_y, x_min, x_max, y_max,
                 image_size=None):
    """
    검출 상자 하나를 바닥 좌표 (x, y) [m]로 바꾼다.

    상자 아래 가운데 점을 호모그래피로 바꾸고, x_min ≤ x ≤ x_max, |y| ≤ y_max 안이면
    (x, y)를, 밖이거나 바꿀 수 없으면(수평선 근처 · 위) None을 돌려준다.
    image_size가 있으면 영상 밖 접점을 제외한다. 경계 v=height는 유지한다.
    """
    # 크기 0인 점 입력은 오프라인 확인용이며 실제 검출 크기는 노드에서 검사한다.
    if not all(math.isfinite(value) for value in (center_x, center_y, size_x, size_y)):
        return None
    if size_x < 0 or size_y < 0:
        return None
    u, v = bottom_center(center_x, center_y, size_x, size_y)
    if image_size is not None and outside_image(u, v, image_size):
        return None
    try:
        x, y = pixel_to_floor(h, u, v)
    except ValueError:
        return None
    if not (x_min <= x <= x_max and abs(y) <= y_max):
        return None
    return float(x), float(y)
