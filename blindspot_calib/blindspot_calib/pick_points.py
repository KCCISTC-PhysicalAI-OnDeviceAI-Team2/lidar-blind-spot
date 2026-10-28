"""
격자 사진에서 바닥 점을 차례로 클릭해 픽셀 좌표 CSV를 만든다.

조작: 왼쪽 클릭 = 지금 점 찍기, n = 화면에 없는 점 건너뛰기,
b = 이전 점으로 돌아가 다시 찍기, s = 저장하고 끝, q = 저장 없이 끝.
"""

import argparse
import os

import cv2

from blindspot_calib.homography import backup_file, load_floor_points, load_pixel_points
from blindspot_calib.homography import save_pixel_points

WINDOW = 'pick_points'
ZOOM_HALF = 20     # 확대창이 보여 줄 범위의 절반 [원본 px]
ZOOM_SCALE = 6     # 확대 배율


class Picker:
    """클릭 상태를 들고 화면을 그린다."""

    def __init__(self, image, floor, pixels, scale):
        self.image = image
        self.floor = floor
        self.ids = list(floor)
        self.pixels = pixels
        self.scale = scale
        self.index = next((i for i, p in enumerate(self.ids) if p not in pixels),
                          len(self.ids))
        self.mouse = None

    def current(self):
        """지금 찍을 점 id. 다 찍었으면 None."""
        return self.ids[self.index] if self.index < len(self.ids) else None

    def on_mouse(self, event, x, y, flags, param):
        """마우스 위치를 원본 픽셀로 바꿔 기록하고, 클릭이면 점을 찍는다."""
        self.mouse = (x / self.scale, y / self.scale)
        if event == cv2.EVENT_LBUTTONDOWN and self.current() is not None:
            self.pixels[self.current()] = self.mouse
            self.index += 1

    def skip(self):
        """지금 점을 찍지 않고 넘어간다."""
        if self.current() is not None:
            self.pixels.pop(self.current(), None)
            self.index += 1

    def back(self):
        """이전 점으로 돌아가 그 점을 지운다."""
        if self.index > 0:
            self.index -= 1
            self.pixels.pop(self.current(), None)

    def draw(self):
        """찍은 점, 안내 글, 확대창을 그린 화면을 돌려준다."""
        h, w = self.image.shape[:2]
        view = cv2.resize(self.image, (w * self.scale, h * self.scale),
                          interpolation=cv2.INTER_NEAREST)
        for pid, (u, v) in self.pixels.items():
            color = (0, 255, 0) if self.floor[pid][2] == 'fit' else (0, 165, 255)
            c = (int(u * self.scale), int(v * self.scale))
            cv2.drawMarker(view, c, color, cv2.MARKER_CROSS, 16, 1)
            cv2.putText(view, pid, (c[0] + 6, c[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        pid = self.current()
        if pid is None:
            text = 'done: s = save, b = back'
        else:
            x, y, role = self.floor[pid]
            text = f'click {pid} (x={x:.2f} y={y:+.2f} m, {role})  n=skip b=back s=save q=quit'
        cv2.rectangle(view, (0, 0), (view.shape[1], 26), (0, 0, 0), -1)
        cv2.putText(view, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1)

        if self.mouse is not None:
            self.draw_zoom(view)
        return view

    def draw_zoom(self, view):
        """마우스 주변을 크게 확대해 오른쪽 위에 붙인다."""
        h, w = self.image.shape[:2]
        mu, mv = int(round(self.mouse[0])), int(round(self.mouse[1]))
        size = 2 * ZOOM_HALF + 1
        patch = cv2.copyMakeBorder(self.image, ZOOM_HALF, ZOOM_HALF, ZOOM_HALF,
                                   ZOOM_HALF, cv2.BORDER_CONSTANT)
        patch = patch[mv:mv + size, mu:mu + size]
        if patch.shape[:2] != (size, size):
            return
        zoom = cv2.resize(patch, (size * ZOOM_SCALE, size * ZOOM_SCALE),
                          interpolation=cv2.INTER_NEAREST)
        mid = ZOOM_HALF * ZOOM_SCALE + ZOOM_SCALE // 2
        cv2.line(zoom, (mid, 0), (mid, zoom.shape[0]), (0, 0, 255), 1)
        cv2.line(zoom, (0, mid), (zoom.shape[1], mid), (0, 0, 255), 1)
        zh, zw = zoom.shape[:2]
        if zh + 30 <= view.shape[0] and zw <= view.shape[1]:
            view[30:30 + zh, view.shape[1] - zw:] = zoom


def main():
    """사진을 띄우고 클릭을 받아 CSV로 저장한다."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--image', default='~/calib_work/grid_frame.png')
    parser.add_argument('--floor', default='calib/floor_points.csv')
    parser.add_argument('--out', default='calib/pixel_points.csv')
    parser.add_argument('--scale', type=int, default=2, help='화면 확대 배율')
    args = parser.parse_args()

    image = cv2.imread(os.path.expanduser(args.image))
    if image is None:
        raise SystemExit(f'사진을 열지 못했다: {args.image}')
    floor = load_floor_points(args.floor)
    pixels = {}
    if os.path.exists(args.out):
        pixels = {k: v for k, v in load_pixel_points(args.out).items() if k in floor}
        print(f'이어서 찍기: {args.out}에 이미 {len(pixels)}개')

    picker = Picker(image, floor, pixels, args.scale)
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW, picker.on_mouse)
    while True:
        cv2.imshow(WINDOW, picker.draw())
        key = cv2.waitKey(30) & 0xFF
        if key == ord('n'):
            picker.skip()
        elif key == ord('b'):
            picker.back()
        elif key == ord('s'):
            ordered = {pid: pixels[pid] for pid in floor if pid in pixels}
            backup = backup_file(args.out, os.path.expanduser('~/calib_work/backup'))
            if backup:
                print(f'기존 {args.out} 백업: {backup}')
            save_pixel_points(args.out, ordered)
            print(f'저장: {args.out} ({len(ordered)}개)')
            break
        elif key == ord('q'):
            print('저장하지 않고 끝냄')
            break
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
