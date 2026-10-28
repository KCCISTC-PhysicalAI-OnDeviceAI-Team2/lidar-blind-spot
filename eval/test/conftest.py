"""eval 폴더의 모듈을 테스트에서 불러올 수 있게 경로를 더한다."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
