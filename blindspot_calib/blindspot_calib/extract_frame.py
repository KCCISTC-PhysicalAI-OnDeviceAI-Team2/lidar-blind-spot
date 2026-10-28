"""rosbag의 압축 영상에서 프레임 한 장을 PNG로 저장한다."""

import argparse
import json
import os

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import CompressedImage

from blindspot_calib.homography import backup_file


def read_frame(bag, topic, skip_sec, storage):
    """
    시작에서 skip_sec초 지난 뒤 rosbag의 첫 프레임을 돌려준다.

    (BGR 영상, 출처 정보 dict)를 돌려준다. 출처는 나중에 어느 프레임을 썼는지 찾는 데 쓴다.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id=storage),
        rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    first = None
    count = 0
    while reader.has_next():
        _, data, stamp = reader.read_next()
        count += 1
        first = stamp if first is None else first
        if (stamp - first) / 1e9 < skip_sec:
            continue
        msg = deserialize_message(data, CompressedImage)
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError('압축 영상을 풀지 못했다')
        info = {
            'bag': os.path.basename(os.path.normpath(bag)),
            'topic': topic,
            'frame_index': count - 1,
            'skip_sec': skip_sec,
            'bag_time_ns': stamp,
            'header_stamp': {'sec': msg.header.stamp.sec,
                             'nanosec': msg.header.stamp.nanosec},
            'width': img.shape[1],
            'height': img.shape[0],
        }
        return img, info
    raise RuntimeError(f'{topic} 프레임이 없다 (읽은 메시지 {count}개, skip {skip_sec}s)')


def main():
    """명령줄 인자를 받아 프레임을 저장한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='rosbag 폴더 (예: ~/bags/20261028_grid)')
    parser.add_argument('-o', '--out', default='~/calib_work/grid_frame.png')
    parser.add_argument('--topic', default='/camera/image_raw/compressed')
    parser.add_argument('--skip', type=float, default=2.0,
                        help='시작 후 건너뛸 초 (카메라 노출이 안정되도록)')
    parser.add_argument('--storage', default='sqlite3',
                        help='rosbag 저장 형식 (Humble 기본 sqlite3, .mcap이면 mcap)')
    args = parser.parse_args()

    img, info = read_frame(os.path.expanduser(args.bag), args.topic, args.skip,
                           args.storage)
    out = os.path.expanduser(args.out)
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    info_path = os.path.splitext(out)[0] + '.json'
    for old in (out, info_path):
        backup = backup_file(old, os.path.join(os.path.dirname(out) or '.', 'backup'))
        if backup:
            print(f'기존 {old} 백업: {backup}')
    if not cv2.imwrite(out, img):
        raise SystemExit(f'사진을 저장하지 못했다: {out} (폴더 권한, 확장자 .png 확인)')
    with open(info_path, 'w', encoding='utf-8') as f:
        json.dump(info, f, indent=2, ensure_ascii=False)
    print(f'저장: {out} ({img.shape[1]}x{img.shape[0]}), 출처: {info_path}')


if __name__ == '__main__':
    main()
