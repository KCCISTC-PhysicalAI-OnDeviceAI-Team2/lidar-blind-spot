"""eval_logic 테스트."""

from types import SimpleNamespace

import pytest

from eval.eval_logic import is_detected, summarize

TRUTH = (1.00, 0.00)
RADIUS = 0.2


def obs(x, y):
    """테스트용 장애물(.x, .y만 있으면 된다)."""
    return SimpleNamespace(x=x, y=y)


@pytest.mark.parametrize('obstacles, expected', [
    ([obs(1.12, 0.05)], True),                    # 반경 안
    ([obs(1.20, 0.00)], True),                    # 경계 포함(실제 거리 0.19999…)
    ([obs(1.21, 0.00)], False),                   # 경계 바로 밖
    ([obs(0.82, 0.85)], False),                   # 옆 벽만
    ([obs(0.82, 0.85), obs(1.05, 0.00)], True),   # 하나라도 반경 안
    ([], False),                                  # 장애물 없음
])
def test_is_detected(obstacles, expected):
    assert is_detected(obstacles, *TRUTH, RADIUS) == expected


def test_summarize_with_target():
    """찾을 물체가 있는 조건: 3개 중 2개 검출, 2개 STOP."""
    results = [(True, 2), (True, 2), (False, 0)]
    row = summarize(results, expected_level=2)
    assert row['메시지_수'] == 3
    assert row['검출_메시지_수'] == 2
    assert row['검출률'] == pytest.approx(2 / 3)
    assert row['기대판단_일치_수'] == 2
    assert row['STOP_수'] == 2


def test_summarize_empty_scene():
    """빈 장면: 검출 칸은 빈칸, STOP 1개는 오정지."""
    results = [(None, 0), (None, 2)]
    row = summarize(results, expected_level=0)
    assert row['메시지_수'] == 2
    assert row['검출_메시지_수'] == ''
    assert row['검출률'] == ''
    assert row['기대판단_일치_수'] == 1
    assert row['STOP_수'] == 1
