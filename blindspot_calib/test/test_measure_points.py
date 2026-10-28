"""measure_points(정답 위치 오차 기록) 시험. ROS 없이 pytest로 돈다."""

import csv
from pathlib import Path
import sys

from blindspot_calib.homography import load_floor_points, load_homography_yaml
from blindspot_calib.measure_points import ERROR_HEADER, main, measure, record_checks
import cv2
import numpy as np
import pytest

PACKAGE = Path(__file__).resolve().parents[1]


def to_pixel(h, x, y):
    """바닥 (x, y)를 h의 역행렬로 픽셀로 되돌린다."""
    p = np.linalg.inv(h) @ np.array([x, y, 1.0])
    return p[0] / p[2], p[1] / p[2]


def test_interface_points_match_interface_doc():
    # interface.md 6장 평가 위치 4곳 + 오른쪽 대칭 2곳, 모두 check
    floor = load_floor_points(PACKAGE / 'calib/interface_points.csv')
    assert floor['E1'][:2] == (1.0, 0.0)
    assert floor['E4'][:2] == (1.97, 0.35)
    assert len(floor) >= 5
    assert all(role == 'check' for _, _, role in floor.values())


def test_measure_exact_and_offset_points():
    h = load_homography_yaml(PACKAGE / 'config/homography.yaml')
    floor = {'A': (1.0, 0.0, 'check'), 'B': (2.0, 0.5, 'check'), 'C': (1.5, 0.0, 'check')}
    pixels = {'A': to_pixel(h, 1.0, 0.0), 'B': to_pixel(h, 2.0, 0.53)}
    rows = measure(h, floor, pixels)
    assert [row[0] for row in rows] == ['A', 'B']  # 찍지 않은 C는 뺀다
    assert rows[0][-1] < 1e-9
    assert rows[1][-1] == pytest.approx(0.03, abs=1e-6)


def test_record_checks_changes_only_check_lines():
    text = (PACKAGE / 'config/homography.yaml').read_text(encoding='utf-8')
    out = record_checks(text, 6, 0.0123, 0.0456, True)
    assert 'validation_performed: true' in out
    assert 'check_points: 6' in out
    assert 'check_rms_m: 0.0123' in out and 'check_max_m: 0.0456' in out
    changed = [a for a, b in zip(text.splitlines(), out.splitlines()) if a != b]
    assert all(any(k in line for k in ('validation_performed', 'check_')) for line in changed)
    with pytest.raises(ValueError):
        record_checks('homography: [1.0]\n', 1, 0.0, 0.0, False)


def test_main_writes_interface_csv_and_records(tmp_path, monkeypatch):
    config = tmp_path / 'homography.yaml'
    config.write_text((PACKAGE / 'config/homography.yaml').read_text(encoding='utf-8'),
                      encoding='utf-8')
    h = load_homography_yaml(config)
    floor = load_floor_points(PACKAGE / 'calib/interface_points.csv')
    pixels = tmp_path / 'pixels.csv'
    pixels.write_text('id,u,v\n' + ''.join(
        f'{pid},{u:.3f},{v:.3f}\n' for pid, (x, y, _) in floor.items()
        for u, v in [to_pixel(h, x, y)]), encoding='utf-8')
    image = tmp_path / 'frame.png'
    assert cv2.imwrite(str(image), np.zeros((480, 640, 3), dtype=np.uint8))
    monkeypatch.setattr(sys, 'argv', [
        'measure_points', '--points', str(PACKAGE / 'calib/interface_points.csv'),
        '--pixels', str(pixels), '--config', str(config), '--image', str(image),
        '--runs-dir', str(tmp_path / 'runs'), '--backup-dir', str(tmp_path / 'backup'),
        '--record', '--independent-checks'])
    main()
    run = next((tmp_path / 'runs').iterdir())
    with open(run / 'position_error_camera_floor.csv', encoding='utf-8') as f:
        rows = list(csv.reader(f))
    assert rows[0] == ERROR_HEADER
    assert len(rows) == 1 + len(floor)
    assert all(float(row[-1]) < 0.002 for row in rows[1:])
    assert (run / 'measure_check.png').exists()
    text = config.read_text(encoding='utf-8')
    assert 'validation_performed: true' in text
    assert f'check_points: {len(floor)}' in text
    assert np.allclose(load_homography_yaml(config), h)
