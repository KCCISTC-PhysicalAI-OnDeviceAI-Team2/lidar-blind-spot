"""바닥 격자 점으로 영상 픽셀 -> 바닥 좌표(base_jig) 호모그래피를 구하고 저장한다."""

import csv
import os
import re
import shutil
import time

import cv2
import numpy as np

FIT = 'fit'
CHECK = 'check'


def load_floor_points(path):
    """
    바닥 점 CSV를 읽는다.

    열: id, x_m, y_m, role. role은 fit(계산에 씀) 또는 check(오차 확인용).
    '#'로 시작하는 줄은 건너뛴다. {id: (x, y, role)}를 돌려준다(파일 순서 유지).
    """
    points = {}
    with open(path, newline='', encoding='utf-8') as f:
        rows = (line for line in f if line.strip() and not line.startswith('#'))
        for row in csv.DictReader(rows):
            role = row['role'].strip().lower()
            pid = row['id'].strip()
            if role not in (FIT, CHECK):
                raise ValueError(f'id {pid}: role은 fit 또는 check ({role})')
            if not pid or pid in points:
                raise ValueError(f'바닥 점 id가 비었거나 두 번 나온다: "{pid}"')
            x, y = float(row['x_m']), float(row['y_m'])
            if not np.isfinite([x, y]).all():
                raise ValueError(f'id {pid}: 바닥 좌표는 유한한 값이어야 한다')
            points[pid] = (x, y, role)
    return points


def load_pixel_points(path):
    """클릭한 픽셀 CSV(id, u, v)를 읽어 {id: (u, v)}를 돌려준다."""
    points = {}
    with open(path, newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            pid = row['id'].strip()
            if not pid or pid in points:
                raise ValueError(f'픽셀 점 id가 비었거나 두 번 나온다: "{pid}"')
            u, v = float(row['u']), float(row['v'])
            if not np.isfinite([u, v]).all():
                raise ValueError(f'id {pid}: 픽셀 좌표는 유한한 값이어야 한다')
            points[pid] = (u, v)
    return points


def save_pixel_points(path, pixels):
    """{id: (u, v)}를 픽셀 CSV로 저장한다."""
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['id', 'u', 'v'])
        for pid, (u, v) in pixels.items():
            writer.writerow([pid, f'{u:.1f}', f'{v:.1f}'])


def match_points(floor, pixels):
    """
    두 파일에 모두 있는 점만 모아 role별로 나눈다.

    {role: (ids, uv 배열 Nx2, xy 배열 Nx2)}를 돌려준다.
    """
    out = {}
    for role in (FIT, CHECK):
        ids = [pid for pid, (_, _, r) in floor.items() if r == role and pid in pixels]
        uv = np.array([pixels[pid] for pid in ids], dtype=np.float64).reshape(-1, 2)
        xy = np.array([floor[pid][:2] for pid in ids], dtype=np.float64).reshape(-1, 2)
        out[role] = (ids, uv, xy)
    return out


def compute_homography(uv, xy):
    """
    픽셀 (u, v) -> 바닥 (x, y) 호모그래피 H(3x3)를 구한다.

    점이 4개 이상 필요하다. 모든 점을 쓰는 최소제곱(method=0)이라
    잘못 찍은 점이 있으면 오차 표에서 크게 보인다.
    """
    uv, xy = np.asarray(uv, dtype=np.float64), np.asarray(xy, dtype=np.float64)
    if uv.ndim != 2 or uv.shape[1:] != (2,) or uv.shape != xy.shape:
        raise ValueError('픽셀과 바닥 대응점은 같은 크기의 Nx2 배열이어야 한다')
    if len(uv) < 4:
        raise ValueError(f'fit 점이 4개 이상 필요하다 (지금 {len(uv)}개)')
    check_layout(np.asarray(uv, dtype=np.float64), np.asarray(xy, dtype=np.float64))
    h, _ = cv2.findHomography(uv, xy, 0)
    if h is None:
        raise ValueError('호모그래피를 구하지 못했다 (점이 한 줄에 몰려 있지 않은지 확인)')
    h = validate_homography(h)
    scale = h[2, 2] if abs(h[2, 2]) > 1e-12 else np.max(np.abs(h))
    return h / scale


def normalize(points):
    """중심을 0, 평균 크기를 1로 맞춘 점 배열."""
    centered = points - points.mean(axis=0)
    return centered / np.sqrt(np.mean(centered ** 2))


def check_layout(uv, xy):
    """
    호모그래피를 정할 수 없는 점 배치를 계산 전에 막는다.

    같은 점이 두 번 있거나, 모든 점이 한 직선 위에 있거나,
    4점 중 3점이 한 직선처럼 정보가 모자라는 배치면 ValueError.
    """
    for points, name in ((uv, '픽셀'), (xy, '바닥')):
        if not np.isfinite(points).all():
            raise ValueError(f'{name} 좌표에 숫자가 아닌 값이 있다')
        if len(np.unique(points, axis=0)) != len(points):
            raise ValueError(f'{name} 좌표에 같은 점이 두 번 있다 (클릭이나 CSV 확인)')
        if np.linalg.matrix_rank(points - points.mean(axis=0)) < 2:
            raise ValueError(f'{name} 점이 모두 한 직선 위에 있다 (좌우로도 퍼뜨린다)')
    # 정규화한 대응점으로 만든 DLT 식의 독립 식이 8개 미만이면 행렬이 하나로 정해지지 않는다
    rows = []
    for (u, v), (x, y) in zip(normalize(uv), normalize(xy)):
        rows.append([u, v, 1, 0, 0, 0, -x * u, -x * v, -x])
        rows.append([0, 0, 0, u, v, 1, -y * u, -y * v, -y])
    if np.linalg.matrix_rank(np.array(rows)) < 8:
        raise ValueError('점 배치가 부족하다 (3점 이상이 한 직선이면 다른 곳에 점을 더 둔다)')


def pixel_to_floor(h, u, v):
    """
    픽셀 (u, v) 한 점을 바닥 좌표 (x, y) [m]로 바꾼다.

    수평선 위(바닥이 아닌 곳)라 분모가 0에 가깝거나 결과가 무한대면 ValueError.
    """
    if not np.isfinite([u, v]).all():
        raise ValueError('픽셀 좌표는 유한한 값이어야 한다')
    p = h @ np.array([u, v, 1.0])
    if abs(p[2]) < 1e-12 * max(1.0, abs(p[0]), abs(p[1])):
        raise ValueError(f'픽셀 ({u:.1f}, {v:.1f})은 바닥으로 바꿀 수 없다 (수평선 근처)')
    x, y = p[0] / p[2], p[1] / p[2]
    if not (np.isfinite(x) and np.isfinite(y)):
        raise ValueError(f'픽셀 ({u:.1f}, {v:.1f}) 변환 결과가 숫자가 아니다')
    return x, y


def floor_errors(h, uv, xy):
    """각 점을 변환한 위치와 실제 위치의 거리 [m] 배열."""
    pred = np.array([pixel_to_floor(h, u, v) for u, v in uv]).reshape(-1, 2)
    return np.linalg.norm(pred - xy, axis=1)


def leave_one_out_errors(uv, xy):
    """
    계산에 쓴 점을 하나씩 빼고 나머지로 행렬을 구해, 뺀 점의 오차 [m]를 잰다.

    계산에 쓴 점 오차(fit)보다 실제 오차에 가까운 추정이다. 독립 check 점을 대신하지는 않는다.
    점이 5개 미만이면 빈 배열, 빼고 나면 배치가 모자라는 점은 nan으로 둔다.
    """
    uv, xy = np.asarray(uv, dtype=np.float64), np.asarray(xy, dtype=np.float64)
    if len(uv) < 5:
        return np.array([])
    errors = []
    for i in range(len(uv)):
        keep = np.arange(len(uv)) != i
        try:
            h = compute_homography(uv[keep], xy[keep])
            errors.append(floor_errors(h, uv[i:i + 1], xy[i:i + 1])[0])
        except ValueError:
            errors.append(np.nan)
    return np.array(errors)


def validate_homography(values):
    """행렬 크기, 유한성과 역변환 가능성을 확인한다."""
    h = np.asarray(values, dtype=np.float64)
    if h.size != 9:
        raise ValueError('homography는 행 우선 9개 값이어야 한다')
    h = h.reshape(3, 3)
    if not np.isfinite(h).all():
        raise ValueError('homography에 NaN 또는 무한대가 있다')
    scale = np.max(np.abs(h))
    if scale == 0 or np.linalg.matrix_rank(h / scale) != 3:
        raise ValueError('homography 행렬의 역행렬이 없다')
    return h


def float_text(value):
    """실수를 YAML에서 실수로 읽히는 글자로 바꾼다(1 -> 1.0, 1e-05는 그대로)."""
    return str(float(value))


def homography_yaml(h, image_size, source, fit_rms, check_rms, check_max, n_fit, n_check,
                    validation_performed, loo_rms=None, loo_max=None):
    """
    camera_to_floor_node가 파라미터 파일로 읽는 YAML 문자열을 만든다.

    validation_performed는 "독립 검증을 했다"는 뜻일 뿐 정확도 합격이 아니다.
    합격 여부는 check_rms_m, check_max_m을 팀이 정한 허용 오차와 비교해 사람이 판단한다.
    check 점이 없으면 오차 값은 -1.0으로 적는다.
    loo_rms, loo_max는 fit 점을 하나씩 빼고 잰 추정 오차다(없으면 -1.0).
    """
    # ROS 파라미터 배열은 한 가지 형식만 받으므로 1, 0도 1.0, 0.0처럼 실수로 쓴다
    values = ', '.join(float_text(v) for v in h.flatten())
    rms_m = check_rms if check_rms is not None else -1.0
    max_m = check_max if check_max is not None else -1.0
    loo_rms_m = loo_rms if loo_rms is not None else -1.0
    loo_max_m = loo_max if loo_max is not None else -1.0
    return (
        '# 영상 픽셀 (u, v) -> base_jig 바닥 (x, y) [m] 호모그래피. compute_homography가 만든다.\n'
        f'# 원본: {source}, fit 점 {n_fit}개 RMS {fit_rms:.4f} m\n'
        '# validation_performed는 검증을 했다는 뜻이고 합격 판정이 아니다. 오차 값을 보고 판단한다.\n'
        '/**:\n'
        '  ros__parameters:\n'
        f'    homography: [{values}]  # 3x3 행 우선\n'
        f'    image_size: [{image_size[0]}, {image_size[1]}]  # 계산에 쓴 영상 크기 [가로, 세로]\n'
        '    calibrated: true  # 격자로 계산함\n'
        '    provisional: true  # 현장 검증과 검토 전 임시 행렬\n'
        f'    validation_performed: {"true" if validation_performed else "false"}'
        '  # check 점 3개 이상 + 실제 사진으로 독립 오차를 쟀음\n'
        f'    check_points: {n_check}\n'
        f'    check_rms_m: {rms_m:.4f}  # check 점 오차 RMS [m], 없으면 -1\n'
        f'    check_max_m: {max_m:.4f}  # check 점 최대 오차 [m], 없으면 -1\n'
        f'    loo_rms_m: {loo_rms_m:.4f}  # fit 점 하나씩 빼고 잰 추정 오차 RMS [m], 없으면 -1\n'
        f'    loo_max_m: {loo_max_m:.4f}  # 같은 방법의 최대 오차 [m], 없으면 -1\n'
    )


def load_homography_yaml(path):
    """homography_yaml로 저장한 파일에서 3x3 행렬을 읽는다."""
    with open(path, encoding='utf-8') as f:
        m = re.search(r'homography:\s*\[([^\]]*)\]', f.read())
    if m is None:
        raise ValueError(f'{path}에 homography 값이 없다')
    values = [float(v) for v in m.group(1).split(',')]
    if len(values) != 9:
        raise ValueError(f'homography 값이 9개가 아니다 ({len(values)}개)')
    return validate_homography(values)


def save_errors_csv(path, h, floor, pixels):
    """점마다 실제 위치, 변환한 위치, 오차 [m]를 CSV로 저장한다."""
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['id', 'role', 'u', 'v', 'x_m', 'y_m', 'x_est_m', 'y_est_m', 'error_m'])
        for pid, (x, y, role) in floor.items():
            if pid not in pixels:
                continue
            u, v = pixels[pid]
            xe, ye = pixel_to_floor(h, u, v)
            err = float(np.hypot(xe - x, ye - y))
            writer.writerow([pid, role, f'{u:.1f}', f'{v:.1f}', x, y,
                             f'{xe:.4f}', f'{ye:.4f}', f'{err:.4f}'])


def backup_file(path, backup_dir):
    """path가 있으면 backup_dir에 시각을 붙여 복사하고 그 경로를 돌려준다. 없으면 None."""
    if not os.path.exists(path):
        return None
    os.makedirs(backup_dir, exist_ok=True)
    stem, ext = os.path.splitext(os.path.basename(path))
    dest = os.path.join(backup_dir, f'{stem}_{time.strftime("%Y%m%d_%H%M%S")}{ext}')
    n = 1
    while os.path.exists(dest):
        dest = os.path.join(backup_dir, f'{stem}_{time.strftime("%Y%m%d_%H%M%S")}_{n}{ext}')
        n += 1
    shutil.copy2(path, dest)
    return dest
