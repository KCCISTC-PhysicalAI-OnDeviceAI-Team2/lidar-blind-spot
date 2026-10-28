"""확인점 개수만으로 독립 검증 상태를 표시하지 않는지 확인한다."""

import sys

import cv2
import numpy as np
import pytest

from blindspot_calib.compute_homography import main
from blindspot_calib.homography import load_floor_points


@pytest.mark.parametrize('independent,checks,expected', [
    (False, 3, False), (True, 2, False), (True, 3, True),
])
def test_validation_requires_explicit_independent_checks(
        tmp_path, monkeypatch, independent, checks, expected):
    floor = tmp_path/'floor.csv'
    pixels = tmp_path/'pixels.csv'
    image = tmp_path/'image.png'
    output = tmp_path/'homography.yaml'
    points = [(1., .5), (1., -.5), (2., .5), (2., -.5),
              (1.3, .2), (1.6, -.1), (1.8, .3)][:4+checks]
    floor.write_text('id,x_m,y_m,role\n' + ''.join(
        f'P{i},{x},{y},{"fit" if i < 4 else "check"}\n'
        for i, (x, y) in enumerate(points)), encoding='utf-8')
    pixels.write_text('id,u,v\n' + ''.join(
        f'P{i},{320-120*y/x},{80+120/x}\n' for i, (x, y) in enumerate(points)),
        encoding='utf-8')
    assert cv2.imwrite(str(image), np.zeros((480, 640, 3), dtype=np.uint8))
    args = ['compute_homography', '--floor', str(floor), '--pixels', str(pixels),
            '--image', str(image), '--out', str(output), '--runs-dir', str(tmp_path/'runs'),
            '--backup-dir', str(tmp_path/'backup')]
    if independent:
        args.append('--independent-checks')
    monkeypatch.setattr(sys, 'argv', args)
    main()
    text = output.read_text(encoding='utf-8')
    assert f'validation_performed: {str(expected).lower()}' in text
    assert 'provisional: true' in text


def test_nonfinite_reference_is_rejected(tmp_path):
    path = tmp_path/'floor.csv'
    path.write_text('id,x_m,y_m,role\nP,NaN,0,check\n', encoding='utf-8')
    with pytest.raises(ValueError):
        load_floor_points(path)
