"""평가 계산: rosbag 결과 메시지로 검출과 판단을 센다(ROS 없이 실행)."""

import math

from blindspot_fusion.zone import STOP


def is_detected(obstacles, truth_x, truth_y, radius):
    """정답 위치에서 반경 안에 장애물이 하나라도 있으면 True."""
    for o in obstacles:
        dist = math.hypot(o.x - truth_x, o.y - truth_y)
        if dist <= radius:
            return True
    return False


def summarize(results, expected_level):
    """메시지별 (검출 여부, 판단)을 세어 평가 결과 CSV 한 줄의 값을 만든다.

    results: [(detected, level), ...]. 빈 장면은 찾을 물체가 없어 detected가 None이고,
        이때 검출 메시지 수와 검출률은 빈칸('')으로 둔다.
    expected_level: 조건표의 기대 판단(PASS=0, SLOW=1, STOP=2).
    반환값: CSV 열 이름을 키로 하는 딕셔너리(조건번호 · 방식 · 지연 열은 제외).
    """
    n = len(results)
    no_target = all(d is None for d, _ in results)
    detected = sum(1 for d, _ in results if d)
    return {
        '메시지_수': n,
        '검출_메시지_수': '' if no_target else detected,
        '검출률': '' if no_target or n == 0 else detected / n,
        '기대판단_일치_수': sum(1 for _, lv in results if lv == expected_level),
        'STOP_수': sum(1 for _, lv in results if lv == STOP),
    }


def from_start(messages, start):
    """start(sec, nanosec) 이후에 찍힌 메시지만 남긴다.

    녹화 앞부분처럼 두 센서가 다 들어오기 전 메시지를 세지 않을 때 쓴다.
    """
    kept = []
    for m in messages:
        if (m.header.stamp.sec, m.header.stamp.nanosec) >= start:
            kept.append(m)
    return kept
