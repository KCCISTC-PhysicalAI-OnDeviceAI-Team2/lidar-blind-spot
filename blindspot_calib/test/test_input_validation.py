"""검출 입력과 영상 경계를 계산 범위와 구분해 검증한다."""

from pathlib import Path

import numpy as np
import pytest

from blindspot_calib import floor_logic
from blindspot_calib.homography import load_homography_yaml


@pytest.mark.parametrize('values', [
    (320., 200., -1., 40., .9), (320., 200., 40., 0., .9),
    (float('nan'), 200., 40., 40., .9), (320., 200., 40., float('inf'), .9),
    (320., 200., 40., 40., float('nan')), (320., 200., 40., 40., 1.1),
    (320., 200., 40., 40., -.1),
])
def test_invalid_detection_is_rejected(values):
    assert not floor_logic.valid_detection(*values)


def test_valid_score_boundaries():
    assert floor_logic.valid_detection(320., 200., 40., 40., 0.)
    assert floor_logic.valid_detection(320., 200., 40., 40., 1.)


def test_bottom_border_retained_but_outside_rejected():
    h = load_homography_yaml(Path(__file__).parents[1] / 'config/homography.yaml')
    for bottom in (479., 480.):
        position = floor_logic.box_to_floor(
            h, 320., bottom-20., 40., 40., 0., 3., 1.5, image_size=[640, 480])
        assert position is not None
        assert 0 < position[0] < 1.5
    assert floor_logic.box_to_floor(
        h, 320., 461., 40., 40., 0., 3., 1.5, image_size=[640, 480]) is None
    assert floor_logic.box_to_floor(
        h, 700., 200., 40., 40., 0., 3., 1.5, image_size=[640, 480]) is None


def test_matrix_validity_is_scale_invariant():
    for scale in (1., 1e-20, 1e20):
        assert np.isfinite(floor_logic.check_homography(np.eye(3)*scale)).all()


def test_negative_box_cannot_be_projected():
    assert floor_logic.box_to_floor(np.eye(3), 1., 1., 1., -1., 0., 3., 1.5) is None
