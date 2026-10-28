import numpy as np
import pytest

from blindspot_calib.homography import backup_file, CHECK, FIT
from blindspot_calib.homography import compute_homography, floor_errors
from blindspot_calib.homography import homography_yaml, leave_one_out_errors
from blindspot_calib.homography import load_homography_yaml
from blindspot_calib.homography import load_floor_points, load_pixel_points
from blindspot_calib.homography import match_points, pixel_to_floor
from blindspot_calib.homography import save_errors_csv, save_pixel_points

# 바닥 (x, y, 1) -> 픽셀. 카메라 높이 0.44m, 아래 18도, 640x480에서 나오는 값
FLOOR_TO_PIXEL = np.array([
    [1468.13, -2204.60, 320.0],
    [419.84, 0.0, 1111.46],
    [4.5879, 0.0, 1.0],
])

GRID = [(x, y) for x in (1.0, 1.5, 2.0, 2.5) for y in (0.5, 0.0, -0.5)]


def to_pixel(x, y):
    p = FLOOR_TO_PIXEL @ np.array([x, y, 1.0])
    return p[0] / p[2], p[1] / p[2]


def test_recovers_floor_points():
    xy = np.array(GRID)
    uv = np.array([to_pixel(x, y) for x, y in GRID])
    h = compute_homography(uv, xy)
    assert floor_errors(h, uv, xy).max() < 1e-6
    # 격자 밖 점(1.2m, 0.3m)도 맞아야 한다
    x, y = pixel_to_floor(h, *to_pixel(1.2, 0.3))
    assert x == pytest.approx(1.2, abs=1e-6)
    assert y == pytest.approx(0.3, abs=1e-6)


def test_needs_four_points():
    uv = np.array([to_pixel(x, y) for x, y in GRID[:3]])
    with pytest.raises(ValueError):
        compute_homography(uv, np.array(GRID[:3]))


@pytest.mark.parametrize('pick', [
    [0, 0, 1, 2],     # 같은 점 두 번
    [1, 4, 7, 10],    # 모두 y=0 한 줄
    [0, 1, 2, 4],     # 3점이 x=1.0 한 줄 + 1점
])
def test_rejects_bad_layout(pick):
    xy = np.array([GRID[i] for i in pick])
    uv = np.array([to_pixel(x, y) for x, y in xy])
    with pytest.raises(ValueError):
        compute_homography(uv, xy)


def test_check_points_show_click_error():
    xy = np.array(GRID)
    uv = np.array([to_pixel(x, y) for x, y in GRID])
    uv_bad = uv.copy()
    uv_bad[4] += (0.0, 3.0)  # 1.5m 점을 3px 아래로 잘못 찍음
    h = compute_homography(np.delete(uv, 4, axis=0), np.delete(xy, 4, axis=0))
    err = floor_errors(h, uv_bad[4:5], xy[4:5])[0]
    assert 0.005 < err < 0.1


def test_csv_round_trip(tmp_path):
    floor_csv = tmp_path / 'floor.csv'
    floor_csv.write_text(
        '# comment\nid,x_m,y_m,role\nA,1.0,0.5,fit\nB,1.5,0.0,check\nC,2.0,0.0,fit\n')
    floor = load_floor_points(floor_csv)
    assert floor['B'] == (1.5, 0.0, CHECK)

    pixel_csv = tmp_path / 'pixels.csv'
    save_pixel_points(pixel_csv, {'A': (10.0, 20.0), 'B': (30.0, 40.0)})
    pixels = load_pixel_points(pixel_csv)
    matched = match_points(floor, pixels)
    assert matched[FIT][0] == ['A']   # C는 픽셀이 없어 빠진다
    assert matched[CHECK][0] == ['B']


def test_rejects_unknown_role(tmp_path):
    floor_csv = tmp_path / 'floor.csv'
    floor_csv.write_text('id,x_m,y_m,role\nA,1.0,0.5,test\n')
    with pytest.raises(ValueError):
        load_floor_points(floor_csv)


def test_yaml_round_trip(tmp_path):
    xy = np.array(GRID)
    uv = np.array([to_pixel(x, y) for x, y in GRID])
    h = compute_homography(uv, xy)
    path = tmp_path / 'homography.yaml'
    path.write_text(homography_yaml(h, (640, 480), 'test', 0.0, None, None, 12, 0, False))
    assert np.allclose(load_homography_yaml(path), h, rtol=1e-8)
    text = path.read_text()
    assert 'validation_performed: false' in text
    assert 'check_max_m: -1.0000' in text
    # 오차가 커도 합격처럼 보이는 값은 쓰지 않고, 오차 값을 그대로 남긴다
    text = homography_yaml(h, (640, 480), 'test', 0.0, 0.8, 1.0, 9, 3, True)
    assert 'check_max_m: 1.0000' in text
    assert 'loo_rms_m: -1.0000' in text
    text = homography_yaml(h, (640, 480), 'test', 0.0, None, None, 9, 0, False, 0.03, 0.045)
    assert 'loo_rms_m: 0.0300' in text and 'loo_max_m: 0.0450' in text


def test_leave_one_out_errors():
    xy = np.array(GRID)
    uv = np.array([to_pixel(x, y) for x, y in GRID])
    # 오차 없는 점이면 한 점을 빼도 그 점을 맞힌다
    assert np.nanmax(leave_one_out_errors(uv, xy)) < 1e-6
    # 한 점을 5픽셀 잘못 찍으면 그 점의 오차가 가장 크게 나온다
    bad = uv.copy()
    bad[5, 1] += 5.0
    errors = leave_one_out_errors(bad, xy)
    assert int(np.nanargmax(errors)) == 5
    assert len(leave_one_out_errors(uv[:4], xy[:4])) == 0


def test_errors_csv(tmp_path):
    xy = np.array(GRID)
    uv = np.array([to_pixel(x, y) for x, y in GRID])
    h = compute_homography(uv, xy)
    floor = {f'P{i}': (x, y, FIT) for i, (x, y) in enumerate(GRID)}
    pixels = {f'P{i}': tuple(p) for i, p in enumerate(uv) if i != 3}
    path = tmp_path / 'errors.csv'
    save_errors_csv(path, h, floor, pixels)
    lines = path.read_text().splitlines()
    assert lines[0].startswith('id,role,u,v')
    assert len(lines) == len(GRID)   # 헤더 + 픽셀 있는 11점
    assert all(float(line.split(',')[-1]) < 1e-4 for line in lines[1:])


def test_backup_keeps_old_file(tmp_path):
    target = tmp_path / 'homography.yaml'
    assert backup_file(target, tmp_path / 'backup') is None
    target.write_text('old')
    first = backup_file(target, tmp_path / 'backup')
    second = backup_file(target, tmp_path / 'backup')
    assert first != second
    assert open(first).read() == 'old' and open(second).read() == 'old'


def test_rejects_duplicate_ids(tmp_path):
    floor_csv = tmp_path / 'floor.csv'
    floor_csv.write_text('id,x_m,y_m,role\nA,1.0,0.5,fit\nA,1.5,0.0,fit\n')
    with pytest.raises(ValueError):
        load_floor_points(floor_csv)
    pixel_csv = tmp_path / 'pixels.csv'
    pixel_csv.write_text('id,u,v\nA,10,20\nA,30,40\n')
    with pytest.raises(ValueError):
        load_pixel_points(pixel_csv)


def test_rejects_horizon_pixel():
    xy = np.array(GRID)
    uv = np.array([to_pixel(x, y) for x, y in GRID])
    h = compute_homography(uv, xy)
    # 분모 h[2] . (u, v, 1) = 0 인 픽셀 = 바닥의 수평선
    u = 320.0
    v = -(h[2, 0] * u + h[2, 2]) / h[2, 1]
    with pytest.raises(ValueError):
        pixel_to_floor(h, u, v)
