"""바닥 점 CSV와 픽셀 CSV로 호모그래피를 구해 YAML로 저장하고 점별 오차를 보여 준다."""

import argparse
import os
import shutil
import time

import cv2
import numpy as np

from blindspot_calib.homography import backup_file, CHECK, FIT
from blindspot_calib.homography import compute_homography, floor_errors
from blindspot_calib.homography import homography_yaml, leave_one_out_errors
from blindspot_calib.homography import load_floor_points
from blindspot_calib.homography import load_homography_yaml
from blindspot_calib.homography import load_pixel_points, match_points
from blindspot_calib.homography import pixel_to_floor, save_errors_csv


def rms(errors):
    """오차 배열의 RMS. 비어 있으면 None."""
    return float(np.sqrt(np.mean(errors ** 2))) if len(errors) else None


def print_table(h, groups):
    """
    점마다 실제 위치, 변환한 위치, 오차를 출력한다.

    groups: [(표시 이름, ids, uv, xy), ...]
    """
    print(f'{"id":>6} {"role":>6} {"x":>6} {"y":>6} {"x_est":>7} {"y_est":>7} {"err_cm":>7}')
    for label, ids, uv, xy in groups:
        for pid, (u, v), (x, y), err in zip(ids, uv, xy, floor_errors(h, uv, xy)):
            xe, ye = pixel_to_floor(h, u, v)
            print(f'{pid:>6} {label:>6} {x:6.2f} {y:+6.2f} {xe:7.3f} {ye:+7.3f} {err * 100:7.1f}')


def draw_check(image, h, floor, pixels, path):
    """바닥 점을 영상으로 되돌려 그린다. 클릭한 점(+)과 변환 결과(o)가 겹쳐야 한다."""
    h_inv = np.linalg.inv(h)
    view = image.copy()
    # 가운데 줄(y = 0)을 행렬로 되돌려 그린다. 확인용 표시일 뿐 측정점이 아니다
    centers = []
    for x in sorted({point[0] for point in floor.values()}):
        p = h_inv @ np.array([x, 0.0, 1.0])
        if abs(p[2]) < 1e-12:
            continue
        u, v = p[:2] / p[2]
        if np.isfinite([u, v]).all() and 0 <= u < view.shape[1] and 0 <= v < view.shape[0]:
            center = (int(round(u)), int(round(v)))
            centers.append(center)
            cv2.circle(view, center, 3, (255, 255, 0), 1)
            cv2.putText(view, f'{x:g}m predicted', (center[0] + 8, center[1] + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)
    if len(centers) >= 2:
        cv2.polylines(view, [np.array(centers)], False, (255, 255, 0), 1)
    for pid, (x, y, role) in floor.items():
        p = h_inv @ np.array([x, y, 1.0])
        est = (int(round(p[0] / p[2])), int(round(p[1] / p[2])))
        color = (0, 255, 0) if role == FIT else (0, 165, 255)
        cv2.circle(view, est, 5, color, 1)
        if pid in pixels:
            u, v = pixels[pid]
            cv2.drawMarker(view, (int(round(u)), int(round(v))), (0, 0, 255),
                           cv2.MARKER_CROSS, 10, 1)
        cv2.putText(view, pid, (est[0] + 6, est[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    if not cv2.imwrite(path, view):
        raise SystemExit(f'확인 그림을 저장하지 못했다: {path}')
    print(f'확인 그림: {path} (빨간 + = 클릭, 원 = 계산 결과, 초록 fit / 주황 check)')


def verify(path, floor, pixels):
    """저장된 행렬로 클릭한 모든 점의 오차를 잰다. 파일은 바꾸지 않는다."""
    h = load_homography_yaml(path)
    ids = [pid for pid in floor if pid in pixels]
    uv = np.array([pixels[pid] for pid in ids]).reshape(-1, 2)
    xy = np.array([floor[pid][:2] for pid in ids]).reshape(-1, 2)
    print_table(h, [('verify', ids, uv, xy)])
    err = floor_errors(h, uv, xy)
    if len(err):
        print(f'\n재확인 {len(err)}개: RMS {rms(err) * 100:.1f} cm, '
              f'최대 {err.max() * 100:.1f} cm')
        print('저장된 행렬로 다시 잰 값이다. 새로 찍은 점이면 독립 확인이지만, '
              '행렬 계산에 썼던 점을 그대로 다시 쓰면 독립 확인이 아니다')


def make_run_dir(root):
    """이번 실행 결과를 모을 폴더 root/<날짜_시각>을 만든다."""
    path = os.path.join(root, time.strftime('%Y%m%d_%H%M%S'))
    n = 1
    while os.path.exists(path):
        path = os.path.join(root, f'{time.strftime("%Y%m%d_%H%M%S")}_{n}')
        n += 1
    os.makedirs(path)
    return path


def main():
    """계산, 출력, 저장을 차례로 한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--floor', default='calib/floor_points.csv')
    parser.add_argument('--pixels', default='calib/pixel_points.csv')
    parser.add_argument('--out', default='config/homography.yaml')
    parser.add_argument('--image', default='~/calib_work/grid_frame.png',
                        help='클릭에 쓴 사진. 영상 크기와 확인 그림에 쓴다')
    parser.add_argument('--runs-dir', default='~/calib_work/runs',
                        help='실행마다 <날짜_시각> 폴더를 만들어 입력·결과를 모두 남긴다')
    parser.add_argument('--backup-dir', default='~/calib_work/backup',
                        help='덮어쓰기 전에 기존 YAML을 복사해 둘 폴더')
    parser.add_argument('--no-image', action='store_true',
                        help='사진 없이 640x480으로 가정하고 계산한다 (validation_performed: false로 저장)')
    parser.add_argument('--verify', action='store_true',
                        help='새로 계산하지 않고 --out의 행렬로 모든 점의 오차만 잰다 (지그를 옮긴 뒤 확인)')
    parser.add_argument('--independent-checks', action='store_true',
                        help='check 점이 추정점이 아닌 별도 실측점임을 명시한다 (합격 판정 아님)')
    args = parser.parse_args()

    floor = load_floor_points(args.floor)
    pixels = load_pixel_points(args.pixels)
    matched = match_points(floor, pixels)
    missing = [pid for pid in floor if pid not in pixels]
    if missing:
        print(f'픽셀이 없어 뺀 점: {", ".join(missing)}')
    unused = [pid for pid in pixels if pid not in floor]
    if unused:
        print(f'바닥 대응점 목록에서 제외한 픽셀: {", ".join(unused)}')

    if args.verify:
        verify(args.out, floor, pixels)
        return

    image_path = os.path.expanduser(args.image)
    image = cv2.imread(image_path)
    if image is None and not args.no_image:
        raise SystemExit(f'사진을 열지 못했다: {args.image}\n'
                         '사진 없이 계산하려면 --no-image (결과는 validation_performed: false)')

    ids_fit, uv_fit, xy_fit = matched[FIT]
    h = compute_homography(uv_fit, xy_fit)
    fit_err = floor_errors(h, uv_fit, xy_fit)
    loo_err = leave_one_out_errors(uv_fit, xy_fit)
    loo_ok = loo_err[np.isfinite(loo_err)] if len(loo_err) else loo_err
    ids_chk, uv_chk, xy_chk = matched[CHECK]
    chk_err = floor_errors(h, uv_chk, xy_chk)

    print_table(h, [(FIT, *matched[FIT]), (CHECK, *matched[CHECK])])
    print(f'\nfit   {len(fit_err)}개: RMS {rms(fit_err) * 100:.1f} cm, '
          f'최대 {fit_err.max() * 100:.1f} cm  (계산에 쓴 점이라 작게 나온다)')
    if len(loo_ok):
        worst = ids_fit[int(np.nanargmax(loo_err))]
        print(f'한 점 빼기 {len(loo_ok)}개: RMS {rms(loo_ok) * 100:.1f} cm, '
              f'최대 {loo_ok.max() * 100:.1f} cm ({worst})  <- check 점이 없을 때의 추정 오차')
    else:
        print('fit 점이 5개 미만이라 한 점 빼기 오차를 잴 수 없다')
    if len(chk_err):
        print(f'check {len(chk_err)}개: RMS {rms(chk_err) * 100:.1f} cm, '
              f'최대 {chk_err.max() * 100:.1f} cm  <- 기록할 값')
    else:
        print('check 점이 없다: 독립 오차를 모른다')

    performed = args.independent_checks and image is not None and len(chk_err) >= 3
    if not performed:
        why = []
        if image is None:
            why.append('사진 없이 계산')
        if len(chk_err) < 3:
            why.append(f'check 점 {len(chk_err)}개 (3개 이상 필요)')
        if not args.independent_checks:
            why.append('별도 실측 check 점임을 명시하지 않음')
        print(f'** 독립 검증 안 됨: {", ".join(why)}. validation_performed: false로 저장한다 **')

    run_dir = make_run_dir(os.path.expanduser(args.runs_dir))
    for src in (args.floor, args.pixels):
        shutil.copy2(src, run_dir)
    frame_info = os.path.splitext(image_path)[0] + '.json'
    if os.path.exists(frame_info):
        shutil.copy2(frame_info, run_dir)
    save_errors_csv(os.path.join(run_dir, 'homography_errors.csv'), h, floor, pixels)
    if image is not None:
        image_size = (image.shape[1], image.shape[0])
        shutil.copy2(image_path, run_dir)
        draw_check(image, h, floor, pixels, os.path.join(run_dir, 'homography_check.png'))
    else:
        image_size = (640, 480)

    text = homography_yaml(h, image_size, os.path.basename(run_dir), rms(fit_err),
                           rms(chk_err), float(chk_err.max()) if len(chk_err) else None,
                           len(fit_err), len(chk_err), performed,
                           rms(loo_ok), float(loo_ok.max()) if len(loo_ok) else None)
    with open(os.path.join(run_dir, 'homography.yaml'), 'w', encoding='utf-8') as f:
        f.write(text)
    print(f'이번 실행 기록: {run_dir}')

    backup = backup_file(args.out, os.path.expanduser(args.backup_dir))
    if backup:
        print(f'기존 {args.out} 백업: {backup}')
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    with open(args.out, 'w', encoding='utf-8') as f:
        f.write(text)
    print(f'저장: {args.out} (validation_performed: {"true" if performed else "false"})')
    print('합격 여부는 자동으로 정하지 않는다. check 오차를 팀 허용 오차와 비교해 판단한다')


if __name__ == '__main__':
    main()
