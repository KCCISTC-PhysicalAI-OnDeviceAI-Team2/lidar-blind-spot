"""
바닥에 표시한 점(정답 위치)을 저장된 호모그래피로 변환해 위치 오차를 cm로 기록한다.

C4 완료 기준 "격자 점 몇 개를 변환해 오차 기록"용이다. 행렬은 바꾸지 않는다.
점 목록 기본값은 interface.md 6장 평가 위치(calib/interface_points.csv)이고,
결과 CSV는 interface.md 8장 위치 오차 형식(센서 = camera_floor)이다.
"""

import argparse
import csv
import os
import re
import shutil

import cv2
import numpy as np

from blindspot_calib.compute_homography import make_run_dir, rms
from blindspot_calib.homography import backup_file, load_floor_points
from blindspot_calib.homography import load_homography_yaml, load_pixel_points
from blindspot_calib.homography import pixel_to_floor

SENSOR = 'camera_floor'
ERROR_HEADER = ['위치번호', '정답_x_m', '정답_y_m', '센서', '측정_x_m', '측정_y_m', '오차_m']


def measure(h, floor, pixels):
    """
    클릭한 점마다 (id, 정답 x, y, 측정 x, y, 오차 m)를 만든다.

    픽셀이 없는 점은 빼고, 바닥으로 바꿀 수 없는 점(수평선 위)은 ValueError.
    """
    rows = []
    for pid, (x, y, _) in floor.items():
        if pid not in pixels:
            continue
        xe, ye = pixel_to_floor(h, *pixels[pid])
        rows.append((pid, x, y, float(xe), float(ye), float(np.hypot(xe - x, ye - y))))
    return rows


def save_error_csv(path, rows):
    """interface.md 8장 위치 오차 CSV(UTF-8, 첫 줄 열 이름)로 저장한다."""
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(ERROR_HEADER)
        for pid, x, y, xe, ye, err in rows:
            writer.writerow([pid, f'{x:.2f}', f'{y:.2f}', SENSOR,
                             f'{xe:.4f}', f'{ye:.4f}', f'{err:.4f}'])


def record_checks(text, n_check, check_rms, check_max, performed):
    """호모그래피 YAML 글에서 check 결과 줄만 바꾼다. 행렬과 다른 줄은 그대로 둔다."""
    values = {
        'validation_performed': 'true' if performed else 'false',
        'check_points': str(n_check),
        'check_rms_m': f'{check_rms:.4f}',
        'check_max_m': f'{check_max:.4f}',
    }
    for key, value in values.items():
        text, count = re.subn(rf'^(\s*{key}:\s*)[^\s#]+', rf'\g<1>{value}', text,
                              flags=re.MULTILINE)
        if count != 1:
            raise ValueError(f'YAML에 {key} 줄이 한 개가 아니다 ({count}개)')
    return text


def draw(image, h, floor, pixels, rows, path):
    """정답 위치(초록 원)와 클릭(빨간 +)을 그리고 점마다 오차 cm를 적는다."""
    h_inv = np.linalg.inv(h)
    view = image.copy()
    errors = {pid: err for pid, *_, err in rows}
    for pid, (x, y, _) in floor.items():
        p = h_inv @ np.array([x, y, 1.0])
        if abs(p[2]) < 1e-12:
            continue
        est = (int(round(p[0] / p[2])), int(round(p[1] / p[2])))
        cv2.circle(view, est, 6, (0, 255, 0), 1)
        label = pid if pid not in errors else f'{pid} {errors[pid] * 100:.1f}cm'
        cv2.putText(view, label, (est[0] + 8, est[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        if pid in pixels:
            u, v = pixels[pid]
            cv2.drawMarker(view, (int(round(u)), int(round(v))), (0, 0, 255),
                           cv2.MARKER_CROSS, 10, 1)
    if not cv2.imwrite(path, view):
        raise SystemExit(f'확인 그림을 저장하지 못했다: {path}')


def main():
    """오차 표 출력, CSV · 그림 저장, (선택) YAML check 칸 기록."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--points', default='calib/interface_points.csv',
                        help='정답 위치 CSV (id,x_m,y_m,role)')
    parser.add_argument('--pixels', default='calib/interface_pixels.csv',
                        help='pick_points로 찍은 픽셀 CSV')
    parser.add_argument('--config', default='config/homography.yaml')
    parser.add_argument('--image', default='~/calib_work/grid_frame.png',
                        help='클릭에 쓴 사진(확인 그림용)')
    parser.add_argument('--runs-dir', default='~/calib_work/measure',
                        help='실행마다 <날짜_시각> 폴더에 결과를 남긴다')
    parser.add_argument('--record', action='store_true',
                        help='결과를 --config YAML의 check 칸에 기록한다(기존 파일은 백업)')
    parser.add_argument('--backup-dir', default='~/calib_work/backup')
    parser.add_argument('--independent-checks', action='store_true',
                        help='이 점들이 행렬 계산에 쓰지 않은 별도 실측점임을 밝힌다')
    args = parser.parse_args()

    h = load_homography_yaml(args.config)
    floor = load_floor_points(args.points)
    pixels = load_pixel_points(args.pixels)
    missing = [pid for pid in floor if pid not in pixels]
    rows = measure(h, floor, pixels)
    if not rows:
        raise SystemExit(f'{args.pixels}에 {args.points}의 점이 하나도 없다 (pick_points로 찍기)')

    print(f'{"위치":>4} {"정답 x":>7} {"정답 y":>7} {"측정 x":>7} {"측정 y":>7} {"오차 cm":>7}')
    for pid, x, y, xe, ye, err in rows:
        print(f'{pid:>4} {x:7.2f} {y:+7.2f} {xe:7.3f} {ye:+7.3f} {err * 100:7.1f}')
    errors = np.array([row[-1] for row in rows])
    print(f'\n{len(rows)}곳: RMS {rms(errors) * 100:.1f} cm, 최대 {errors.max() * 100:.1f} cm '
          f'({rows[int(errors.argmax())][0]})')
    if missing:
        print(f'찍지 않아 뺀 점: {", ".join(missing)}')

    run_dir = make_run_dir(os.path.expanduser(args.runs_dir))
    for src in (args.points, args.pixels, args.config):
        shutil.copy2(src, run_dir)
    csv_path = os.path.join(run_dir, 'position_error_camera_floor.csv')
    save_error_csv(csv_path, rows)
    print(f'위치 오차 CSV: {csv_path}')
    image = cv2.imread(os.path.expanduser(args.image))
    if image is not None:
        draw(image, h, floor, pixels, rows, os.path.join(run_dir, 'measure_check.png'))
        print(f'확인 그림: {os.path.join(run_dir, "measure_check.png")} '
              '(초록 원 = 정답 위치, 빨간 + = 클릭)')
    else:
        print(f'사진을 열지 못해 확인 그림은 건너뛴다: {args.image}')

    if args.record:
        performed = args.independent_checks and image is not None and len(rows) >= 3
        with open(args.config, encoding='utf-8') as f:
            text = record_checks(f.read(), len(rows), rms(errors), float(errors.max()),
                                 performed)
        backup = backup_file(args.config, os.path.expanduser(args.backup_dir))
        with open(args.config, 'w', encoding='utf-8') as f:
            f.write(text)
        print(f'기록: {args.config} check {len(rows)}개 '
              f'(validation_performed: {"true" if performed else "false"}), 백업 {backup}')
        if not performed:
            print('validation_performed를 true로 하려면 사진, 3곳 이상, --independent-checks가 필요하다')
    print('합격 여부는 자동으로 정하지 않는다. 팀 허용 오차와 비교해 판단한다')


if __name__ == '__main__':
    main()
