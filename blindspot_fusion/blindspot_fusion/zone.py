# Same values as blindspot_msgs/SafetyAdvice.level
PASS = 0
SLOW = 1
STOP = 2


def classify_zone(x, y, stop_x, slow_x, half_width):
    """
    장애물 하나의 위치로 구역을 판정해 SafetyAdvice의 level 값을 돌려준다.

    x, y: base_jig 기준 장애물 위치 [m] (x는 앞쪽, y는 왼쪽이 +).
    stop_x: 정지 구역이 끝나는 x [m].
    slow_x: 감속 구역이 끝나는 x [m].
    half_width: 구역 폭의 절반 [m].
    """
    if x < 0:
        return PASS
    elif x < stop_x and abs(y) <= half_width:
        return STOP
    elif x < slow_x and abs(y) <= half_width:
        return SLOW
    return PASS
