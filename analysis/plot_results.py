"""Plot LiDAR-only, camera-only and fusion results from the evaluation CSV.

Inputs (format defined in docs/interface.md, section 8):
  --eval        evaluation result CSV
                (조건번호, 방식, 메시지_수, 검출_메시지_수, 검출률,
                 기대판단_일치_수, STOP_수, 평균_지연_ms, 최대_지연_ms)
  --conditions  condition table CSV
                (조건번호, 물체, 클래스, 거리_m, 각도_deg, 정답_x_m, 정답_y_m,
                 조명, 기대_판단, rosbag, 메모)

Outputs (in --out):
  detection_rate.png     detection rate per condition, three methods
  decision_accuracy.png  share of messages that match the expected decision
  false_stop_rate.png    STOP share in empty scenes
  summary.csv            per-method averages used in the figures

Example:
  python3 plot_results.py --eval eval_results.csv --conditions conditions.csv --out figures
"""
import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
import pandas as pd  # noqa: E402

METHODS = ["lidar", "camera", "fusion"]
METHOD_LABELS = {"lidar": "LiDAR only", "camera": "Camera only", "fusion": "Fusion"}
METHOD_COLORS = {"lidar": "#8c8c8c", "camera": "#4c78a8", "fusion": "#e45756"}
EMPTY_CLASSES = {"", "none", "empty", "없음", "nan"}
KOREAN_FONTS = ["NanumGothic", "Noto Sans CJK KR", "Noto Sans KR", "Noto Sans CJK JP"]


def pick_korean_font():
    """Return an installed font that can draw Korean, or None."""
    installed = {f.name for f in font_manager.fontManager.ttflist}
    for name in KOREAN_FONTS:
        if name in installed:
            return name
    return None


def load(eval_path, cond_path):
    ev = pd.read_csv(eval_path, encoding="utf-8-sig")
    cond = pd.read_csv(cond_path, encoding="utf-8-sig")
    ev["방식"] = ev["방식"].str.strip().str.lower()

    unknown = set(ev["방식"]) - set(METHODS)
    if unknown:
        raise SystemExit(f"Unknown method values in eval CSV: {unknown}")

    missing = set(ev["조건번호"]) - set(cond["조건번호"])
    if missing:
        raise SystemExit(f"Condition IDs missing from condition table: {sorted(missing)}")

    df = ev.merge(cond, on="조건번호", how="left")
    df["is_empty"] = df["클래스"].fillna("").astype(str).str.strip().str.lower().isin(EMPTY_CLASSES)

    # Recompute ratios from counts so the figures do not depend on rounding in the CSV.
    df["detection_rate"] = df["검출_메시지_수"] / df["메시지_수"]
    df["decision_accuracy"] = df["기대판단_일치_수"] / df["메시지_수"]
    df["stop_rate"] = df["STOP_수"] / df["메시지_수"]
    return df


def condition_labels(rows, korean_ok):
    labels = []
    for _, r in rows.iterrows():
        if r["is_empty"]:
            labels.append("빈 장면" if korean_ok else f"{r['조건번호']} empty")
            continue
        name = r["물체"] if korean_ok else f"{r['조건번호']} {r['클래스']}"
        labels.append(f"{name}\n{r['거리_m']:.1f} m, {int(r['각도_deg'])}°")
    return labels


def grouped_bar(df, value_col, title, ylabel, out_path, korean_ok):
    cond_ids = list(dict.fromkeys(df.sort_values("조건번호")["조건번호"]))
    first_rows = df.drop_duplicates("조건번호").set_index("조건번호").loc[cond_ids].reset_index()
    labels = condition_labels(first_rows, korean_ok)

    width = 0.27
    fig, ax = plt.subplots(figsize=(max(8, len(cond_ids) * 1.1), 4.8))
    for i, m in enumerate(METHODS):
        sub = df[df["방식"] == m].set_index("조건번호")
        values = [sub[value_col].get(c, float("nan")) for c in cond_ids]
        counts = [sub["메시지_수"].get(c, None) for c in cond_ids]
        xs = [x + (i - 1) * width for x in range(len(cond_ids))]
        ax.bar(xs, values, width, label=METHOD_LABELS[m], color=METHOD_COLORS[m])
        for x, v, n in zip(xs, values, counts):
            if n is not None and v == v:  # v == v is False for NaN
                ax.text(x, v + 0.02, f"n={int(n)}", ha="center", fontsize=6, rotation=90)

    ax.set_xticks(range(len(cond_ids)))
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylim(0, 1.25)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def false_stop_bar(empty_df, out_path):
    sums = empty_df.groupby("방식")[["STOP_수", "메시지_수"]].sum()
    rates = sums["STOP_수"] / sums["메시지_수"]
    fig, ax = plt.subplots(figsize=(5, 4))
    values = [rates.get(m, float("nan")) for m in METHODS]
    bars = ax.bar([METHOD_LABELS[m] for m in METHODS], values,
                  color=[METHOD_COLORS[m] for m in METHODS])
    finite = [v for v in values if v == v]
    ymax = max(0.1, max(finite, default=0) * 1.3)
    for b, v in zip(bars, values):
        if v == v:
            ax.text(b.get_x() + b.get_width() / 2, v + ymax * 0.02, f"{v:.1%}", ha="center", fontsize=9)
    ax.set_ylim(0, ymax)
    ax.set_ylabel("STOP share in empty scenes")
    ax.set_title("False stop rate (lower is better)")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return rates


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--eval", required=True)
    ap.add_argument("--conditions", required=True)
    ap.add_argument("--out", default="figures")
    args = ap.parse_args()

    font = pick_korean_font()
    if font:
        plt.rcParams["font.family"] = font
    else:
        print("No Korean font found: using condition ID and class for labels.")
    plt.rcParams["axes.unicode_minus"] = False

    df = load(args.eval, args.conditions)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    objects = df[~df["is_empty"]]
    empty = df[df["is_empty"]]

    grouped_bar(objects, "detection_rate", "Detection rate by condition", "Detection rate",
                out / "detection_rate.png", font is not None)
    grouped_bar(df, "decision_accuracy", "Decision accuracy by condition", "Matches expected decision",
                out / "decision_accuracy.png", font is not None)

    if empty.empty:
        print("No empty-scene condition found: skipped false_stop_rate.png")
        fs = pd.Series(dtype=float)
    else:
        fs = false_stop_bar(empty, out / "false_stop_rate.png")

    summary = pd.DataFrame({
        "method": METHODS,
        "mean_detection_rate": [objects[objects["방식"] == m]["detection_rate"].mean() for m in METHODS],
        "mean_decision_accuracy": [df[df["방식"] == m]["decision_accuracy"].mean() for m in METHODS],
        "false_stop_rate": [fs.get(m, float("nan")) for m in METHODS],
    })
    summary.to_csv(out / "summary.csv", index=False, encoding="utf-8")

    print(summary.round(3).to_string(index=False))
    print(f"\nFigures and summary.csv saved to {out}/")


if __name__ == "__main__":
    main()
