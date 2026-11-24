"""Lab 3 (e) - scores lab3e_outer_loop square-wave runs against the final (e)
specifications (manual Lab 3 (e)) and checks them against the board's own verdict.

Per step (each reference edge; the start-up half after g is not a step):
  band       0.094-0.106 m at 0.10, 0.240-0.260 m at 0.25
  settle_s   time after the edge at which the ball entered the band for the LAST
             time and then stayed in it until the next edge (blank = ended outside)
  os_pct     largest excursion past the new reference / 0.15 m x 100
  pass       settle_s <= 8.5 s and os_pct < 45 % and the step is complete and
             has no stream gap (a gap means the control ISR paused)
Per run: the longest streak of consecutive passing steps; the demo needs 10
periods = 20 steps. The board prints the same verdict ('# e step ...'); any
step where the two disagree is listed.

Outputs, next to this script:
  lab3e_steps.csv   one row per step
  lab3e_plot.csv    file, run, t_s, r, y, band_lo, band_hi  (for the Excel plot with the bands)

Usage:
    py -3 analyze_lab3e.py                  # every "*.txt" next to this script
    py -3 analyze_lab3e.py e_run1.txt ...
"""
import csv
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "3d outer loop readings"))
import analyze_lab3d as d  # noqa: E402  (brings analyze_lab3b as d.b)

STEPS_CSV = HERE / "lab3e_steps.csv"
PLOT_CSV = HERE / "lab3e_plot.csv"

Y_LOW, Y_HIGH = 0.10, 0.25
STEP = Y_HIGH - Y_LOW
BANDS = {Y_LOW: (0.094, 0.106), Y_HIGH: (0.240, 0.260)}
SETTLE_MAX_S = 8.5
OS_MAX_PCT = 45.0
DEMO_PERIODS = 10
HALF_PERIOD_S = 10.0

BOARD_RE = re.compile(r"# e step (\d+) dir=([+-]) settle_s=(never|[-\d.]+) os_pct=([-\d.]+) (PASS|FAIL)")


def parse(lines):
    """Samples of the first square-wave run in the lines (analyze_lab3d tuples)."""
    for meta, run in d.parse_runs(lines):
        if meta["kind"] == "square":
            return run
    return []


def band(r):
    return BANDS[Y_HIGH] if abs(r - Y_HIGH) < 1e-6 else BANDS[Y_LOW]


def e_step(seg, t_edge, t_next, gaps):
    target = seg[0][1]
    up = target > (Y_LOW + Y_HIGH) / 2
    lo, hi = band(target)
    settle = None
    for k in range(len(seg) - 1, -1, -1):          # walk back over the final in-band stretch
        if not lo <= seg[k][6] <= hi:
            break
        settle = seg[k][0] - t_edge
    past = max((s[6] - target) if up else (target - s[6]) for s in seg)
    os_pct = max(0.0, past) / STEP * 100.0
    complete = seg[-1][0] - t_edge >= HALF_PERIOD_S - 0.05
    gap = any(t_edge <= g0 < t_next for g0, _ in gaps)
    ok = complete and not gap and settle is not None and settle <= SETTLE_MAX_S and os_pct < OS_MAX_PCT
    return {"t_edge_s": t_edge, "dir": "+" if up else "-", "to_m": target, "settle_s": settle,
            "os_pct": os_pct, "complete": complete, "gap": gap, "pass": ok}


def e_steps(run):
    if not run:
        return []
    edges = d.find_edges_safe(run)
    gaps = d.stream_gaps(run)
    out = []
    for i, i0 in enumerate(edges):
        i1 = edges[i + 1] if i + 1 < len(edges) else len(run)
        t_next = run[i1][0] if i1 < len(run) else float("inf")
        out.append(e_step(run[i0:i1], run[i0][0], t_next, gaps))
    return out


def best_streak(steps):
    """Longest run of consecutive passing steps. Incomplete steps (cut off by s)
    are ignored; a gap or a failure breaks the streak."""
    best = cur = 0
    for s in steps:
        if not s["complete"]:
            continue
        cur = cur + 1 if s["pass"] and not s["gap"] else 0
        best = max(best, cur)
    return best


def board_steps(lines):
    out = []
    for line in lines:
        m = BOARD_RE.search(line)
        if m:
            out.append({"n": int(m.group(1)), "dir": m.group(2),
                        "settle_s": None if m.group(3) == "never" else float(m.group(3)),
                        "os_pct": float(m.group(4)), "pass": m.group(5) == "PASS"})
    return out


def disagreements(ours, board):
    """Indices (into the complete steps) where our pass/fail or direction differs from the board."""
    full = [s for s in ours if s["complete"]]
    return [i for i, (a, b) in enumerate(zip(full, board)) if a["pass"] != b["pass"] or a["dir"] != b["dir"]]


def plot_rows(run):
    return [(s[0], s[1], s[6], *band(s[1])) for s in run]


def isr_gap_us(lines):
    for line in lines:
        m = re.search(r"isr_gap_max_us=(\d+)", line)
        if m:
            return int(m.group(1))
    return None


def report(name, n, meta, run, lines, step_rows, plot_csv_rows):
    steps = e_steps(run)
    full = [s for s in steps if s["complete"]]
    if not full:
        print(f"{name} run {n}: no complete steps")
        return
    streak = best_streak(steps)
    fails = [s for s in full if not s["pass"]]
    clean = [s for s in full if not s["gap"]]
    settles = [s["settle_s"] for s in clean if s["settle_s"] is not None]
    print(f"{name} run {n}: aw={meta['aw']}, {len(full)} complete steps, {len(full) - len(fails)} pass")
    print(f"    worst settle {max(settles) if settles else float('nan'):.2f} s (limit {SETTLE_MAX_S}), "
          f"worst OS {max(s['os_pct'] for s in clean) if clean else float('nan'):.1f} % (limit {OS_MAX_PCT:.0f})")
    print(f"    best streak {streak} steps = {streak / 2:.1f} periods -> "
          + ("DEMO-READY (>= 10 periods)" if streak >= 2 * DEMO_PERIODS else "short of 10 periods"))
    for s in fails:
        if s["gap"]:
            why = "stream gap (control ISR paused)"
        elif s["settle_s"] is None:
            why = "ended outside the band"
        elif s["settle_s"] > SETTLE_MAX_S:
            why = f"settled at {s['settle_s']:.2f} s"
        else:
            why = f"overshoot {s['os_pct']:.1f} %"
        print(f"    FAIL {s['dir']} at {s['t_edge_s']:.2f} s: {why}")
    gap = isr_gap_us(lines)
    if gap is not None:
        print(f"    isr_gap_max_us={gap}" + ("  WARNING: control ISR paused" if gap > 5000 else "  (clean)"))
    if not meta["stopped"]:
        print("    note: no '# stop' line - the capture ended before s was pressed")
    board = board_steps(lines)
    if board:
        diff = disagreements(steps, board)
        print(f"    board verdict: {len(board)} steps, "
              + ("agrees with this analysis" if not diff else f"DISAGREES at steps {[i + 1 for i in diff]}"))
    for s in steps:
        step_rows.append([name, n, meta["aw"], f"{s['t_edge_s']:.2f}", s["dir"], f"{s['to_m']:.3f}",
                          "" if s["settle_s"] is None else f"{s['settle_s']:.2f}", f"{s['os_pct']:.1f}",
                          int(s["complete"]), int(s["gap"]), int(s["pass"])])
    for row in plot_rows(run):
        plot_csv_rows.append([name, n] + [f"{v:.4f}" for v in row])


def main(argv):
    files = [Path(x) for x in argv] or sorted(HERE.glob("*.txt"))
    if not files:
        print("no captures found")
        return 1
    step_rows, plot_csv_rows = [], []
    for path in files:
        text = path.read_text(errors="replace").splitlines()
        starts = [i for i, line in enumerate(text) if line.strip().startswith("# start")]
        runs = d.parse_runs(text)          # one entry per '# start' that has samples
        for n, (meta, run) in enumerate(runs, 1):
            if meta["kind"] != "square":
                continue
            # board lines and the stop line belong to the run they follow
            i0 = next(i for i in starts if text[i].strip() == meta["start"])
            later = [i for i in starts if i > i0]
            report(path.name, n, meta, run, text[i0:later[0] if later else len(text)],
                   step_rows, plot_csv_rows)
    with open(STEPS_CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["file", "run", "aw", "t_edge_s", "dir", "to_m", "settle_s", "os_pct",
                    "complete", "gap", "pass"])
        w.writerows(step_rows)
    with open(PLOT_CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["file", "run", "t_s", "r", "y", "band_lo", "band_hi"])
        w.writerows(plot_csv_rows)
    print(f"wrote {STEPS_CSV.name}, {PLOT_CSV.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
