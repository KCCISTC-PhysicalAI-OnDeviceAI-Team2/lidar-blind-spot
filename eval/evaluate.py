"""평가 실행: _out rosbag과 조건표로 방식별 평가 결과 CSV를 만든다.

사용 예 (source 된 터미널, 저장소 폴더에서):
    python3 eval/evaluate.py --conditions ~/bags/fake_conditions.csv --bags ~/bags \
        --config blindspot_fusion/config/fusion_node.yaml --out ~/bags/fake_results.csv
"""

import argparse
import csv
from pathlib import Path

from blindspot_fusion.fusion_logic import decide, usable_obstacles
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import yaml

from eval_logic import is_detected, summarize

LEVELS = {'PASS': 0, 'SLOW': 1, 'STOP': 2}
TOPICS = {'lidar': '/lidar_obstacles', 'camera': '/camera_obstacles', 'fusion': '/safety_advice'}
COLUMNS = ['조건번호', '방식', '메시지_수', '검출_메시지_수', '검출률',
           '기대판단_일치_수', 'STOP_수', '평균_지연_ms', '최대_지연_ms']


def load_zone(config_path):
    """fusion_node.yaml에서 구역과 신뢰도 기준을 읽는다(노드와 같은 값)."""
    with open(config_path, encoding='utf-8') as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']
    return {k: params[k] for k in ('stop_x', 'slow_x', 'half_width', 'min_confidence')}


def read_bag(path):
    """rosbag의 메시지를 토픽별 목록으로 읽는다."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    msgs = {topic: [] for topic in types}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msgs[topic].append(deserialize_message(data, types[topic]))
    return msgs


def stamp_key(stamp):
    """메시지 시각(sec, nanosec)을 딕셔너리 키로 쓸 튜플로 바꾼다.

    융합 판단에 적힌 시각으로 그때 쓴 입력 메시지를 찾을 때 쓴다.
    """
    return (stamp.sec, stamp.nanosec)


def evaluate_condition(row, msgs, zone, radius):
    """조건 하나의 결과를 방식(lidar, camera, fusion)별로 센다.

    반환값: ([(방식, summarize 결과), ...], fusion에서 뺀 판단 수)
    """
    expected = LEVELS[row['기대_판단']]
    truth = None
    if row['정답_x_m'] != '':
        truth = (float(row['정답_x_m']), float(row['정답_y_m']))

    def detect(obstacles):
        """신뢰도로 거른 장애물 중 정답 위치 반경 안에 있는지 본다(빈 장면은 None)."""
        if truth is None:
            return None
        return is_detected(usable_obstacles(obstacles, zone['min_confidence']), *truth, radius)

    rows = []
    for method in ('lidar', 'camera'):
        results = []
        for m in msgs.get(TOPICS[method], []):
            level = decide(m.obstacles, zone['stop_x'], zone['slow_x'],
                           zone['half_width'], zone['min_confidence'])[0]
            results.append((detect(m.obstacles), level))
        rows.append((method, summarize(results, expected)))

    lidar_by = {stamp_key(m.header.stamp): m for m in msgs.get(TOPICS['lidar'], [])}
    camera_by = {stamp_key(m.header.stamp): m for m in msgs.get(TOPICS['camera'], [])}
    results, skipped = [], 0
    for adv in msgs.get(TOPICS['fusion'], []):
        used, missing = [], False
        for key, table in ((stamp_key(adv.lidar_stamp), lidar_by),
                           (stamp_key(adv.camera_stamp), camera_by)):
            if key == (0, 0):
                continue  # 이 판단에 쓰지 않은 소스
            if key not in table:
                missing = True  # rosbag에 없는 입력을 가리킴 → 빼고 센다
                break
            used.extend(table[key].obstacles)
        if missing:
            skipped += 1
            continue
        results.append((detect(used), adv.level))
    rows.append(('fusion', summarize(results, expected)))
    return rows, skipped


def main():
    parser = argparse.ArgumentParser(description='rosbag 평가 결과 CSV 만들기')
    parser.add_argument('--conditions', required=True, help='조건표 CSV')
    parser.add_argument('--bags', required=True, help='_out rosbag들이 있는 폴더')
    parser.add_argument('--config', required=True, help='fusion_node.yaml')
    parser.add_argument('--out', required=True, help='결과 CSV')
    parser.add_argument('--radius', type=float, default=0.2, help='검출 반경 [m]')
    args = parser.parse_args()

    zone = load_zone(Path(args.config).expanduser())
    with open(Path(args.conditions).expanduser(), encoding='utf-8-sig', newline='') as f:
        conditions = list(csv.DictReader(f))

    out_rows = []
    for row in conditions:
        bag = Path(args.bags).expanduser() / f"{row['rosbag']}_out"
        rows, skipped = evaluate_condition(row, read_bag(bag), zone, args.radius)
        for method, s in rows:
            rate = s['검출률']
            out_rows.append({'조건번호': row['조건번호'], '방식': method, **s,
                             '검출률': '' if rate == '' else f'{rate:.3f}',
                             '평균_지연_ms': '', '최대_지연_ms': ''})
        print(f"{row['조건번호']} fusion: rosbag에 없는 입력을 가리켜 뺀 판단 {skipped}개")

    out = Path(args.out).expanduser()
    with open(out, 'w', encoding='utf-8', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(out_rows)
    print(f'결과: {out}')


if __name__ == '__main__':
    main()
