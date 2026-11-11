"""Lab 3 (b) - scores lab3_outer_loop square-wave runs and lines them up
with the continuous-time simulation.

Input: CoolTerm captures of the sketch's stream, lines
    t_ms,r,th_ref,th_sat,theta,V,y
('#' lines are messages and are skipped).

For every reference edge in a run this measures, as mte484lab3partbrun.m does:
  overshoot      peak beyond the new reference / 0.15 m step x 100
  settling time  last time y is outside +-2 % of the step (+-3 mm), from the edge
                 (blank if y never settles before the next edge)
  ss error       r - mean(y) over the last SS_WINDOW_S of the half period
  peak |V|, and the fraction of samples where the gear saturator clipped
and a symmetry summary (mean of the up steps vs mean of the down steps).

Part (b) has no pass/fail specs: the (c) specs (7 s, 45 %, zero error) are
printed only for reference.

Outputs, next to this script:
  lab3b_steps.csv     one row per step
  lab3b_overlay.csv   t_s, r, y_hw, y_sim_cont, y_sim_disc for each run, with
                      the simulation's steady period (20-40 s) phase-aligned to
                      the hardware's reference edges. Needs sim_lab3b.csv from
                      Matlab_file/Lab 3/mte484lab3partb_export.m.

Usage:
    py -3 analyze_lab3b.py                 # every "*.txt" next to this script
    py -3 analyze_lab3b.py run1.txt ...
"""
import csv
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SIM_CSV = HERE / "sim_lab3b.csv"
STEPS_CSV = HERE / "lab3b_steps.csv"
OVERLAY_CSV = HERE / "lab3b_overlay.csv"

Y_LOW, Y_HIGH = 0.10, 0.25
STEP = Y_HIGH - Y_LOW
SETTLE_BAND = 0.02 * STEP     # 2 % band [m]
SS_WINDOW_S = 1.0             # steady-state averaging window at the end of each step [s]
HALF_PERIOD_S = 10.0
SIM_PERIOD_START_S = 20.0     # first full period of the sim after its start-up transient
FIELDS = 7
REF_SAT = 0.7

# (c) specs, shown for reference only
TS_SPEC_C, OS_SPEC_C = 7.0, 45.0


def parse_stream(lines):
    """Splits a capture into runs (a new run at every '# start' line).
    Returns a list of runs, each a list of (t_s, r, th_ref, th_sat, theta, V, y)."""
    runs, current = [], None
    for raw in lines:
        line = raw.strip()
        if line.startswith("# start"):
            current = []
            runs.append(current)
            continue
        if line.startswith("# stop"):   # samples after a stop are the idle stream, not the run
            current = None
            continue
        if not line or line.startswith("#") or current is None:
            continue
        parts = line.split(",")
        if len(parts) != FIELDS:
            continue
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            continue
        vals[0] /= 1000.0
        current.append(tuple(vals))
    return [r for r in runs if r]


def find_edges(run):
    """Indices where the ball reference changes."""
    return [i for i in range(1, len(run)) if abs(run[i][1] - run[i - 1][1]) > STEP / 2]


def step_metrics(run, start, end):
    """Metrics for the samples run[start:end], which begin at a reference edge."""
    t0 = run[start][0]
    seg = run[start:end]
    target = seg[0][1]
    before = run[start - 1][1]
    up = target > before
    ys = [s[6] for s in seg]

    peak = max(ys) - target if up else target - min(ys)
    overshoot = max(0.0, peak) / STEP * 100.0

    outside = [k for k, y in enumerate(ys) if abs(y - target) > SETTLE_BAND]
    if not outside:
        settling = 0.0
    elif outside[-1] < len(ys) - 1:
        settling = seg[outside[-1] + 1][0] - t0
    else:
        settling = None

    t_end = seg[-1][0]
    tail = [s[6] for s in seg if s[0] >= t_end - SS_WINDOW_S]
    ss_error = target - sum(tail) / len(tail)

    return {
        "t_edge_s": t0,
        "dir": "+" if up else "-",
        "to_m": target,
        "settling_s": settling,
        "overshoot_pct": overshoot,
        "ss_error_mm": ss_error * 1000.0,
        "peak_abs_V": max(abs(s[5]) for s in seg),
        "sat_pct": 100.0 * sum(1 for s in seg if abs(s[2]) > REF_SAT) / len(seg),
        "complete": (t_end - t0) >= HALF_PERIOD_S - 0.05,
    }


def analyze_run(run):
    edges = find_edges(run)
    bounds = edges + [len(run)]
    return [step_metrics(run, bounds[k], bounds[k + 1]) for k in range(len(edges))]


HOLD_WINDOW_S = 2.0      # rest position = mean over the last 2 s before the stop
HOLD_STILL_MM = 1.0      # drift (mean of 2nd half - mean of 1st half of the window) below
                         # which the ball counts as stopped; a range test trips on ~1 mm sensor spikes


def hold_summary(run):
    """Where the ball came to rest in a fixed-reference (h) run: means over the
    last HOLD_WINDOW_S of the run, e = r - y (positive = ball short of r, on the
    side away from the motor)."""
    t_end = run[-1][0]
    tail = [s for s in run if s[0] >= t_end - HOLD_WINDOW_S]
    ys = [s[6] for s in tail]
    y = sum(ys) / len(ys)
    r = tail[-1][1]
    half = len(ys) // 2
    drift = sum(ys[half:]) / (len(ys) - half) - sum(ys[:half]) / max(half, 1)
    return {
        "duration_s": t_end - run[0][0],
        "r": r,
        "y": y,
        "e_mm": (r - y) * 1000.0,
        "theta": sum(s[4] for s in tail) / len(tail),
        "th_sat": sum(s[3] for s in tail) / len(tail),
        "still": abs(drift) * 1000.0 < HOLD_STILL_MM,
        "start_y": run[0][6],
    }


def load_sim(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return [(float(r["t_s"]), float(r["r"]), float(r["y_cont"]), float(r["y_disc"])) for r in rows]


def sim_period(sim):
    """The sim's first full steady period: starts at the up edge at SIM_PERIOD_START_S."""
    return [s for s in sim if SIM_PERIOD_START_S <= s[0] < SIM_PERIOD_START_S + 2 * HALF_PERIOD_S]


def sim_as_run(sim, column=2):
    """The sim in the stream's tuple layout, so analyze_run scores it the same
    way as the hardware (column 2 = y_cont, 3 = y_disc; V and theta not exported)."""
    return [(s[0], s[1], 0.0, 0.0, 0.0, 0.0, s[column]) for s in sim]


def overlay(run, period):
    """Hardware y next to the sim's steady period, aligned at each hardware up edge."""
    edges = [i for i in find_edges(run) if run[i][1] > run[i - 1][1]]
    if not edges or len(period) < 2:
        return []
    dt = period[1][0] - period[0][0]
    t_up = [run[i][0] for i in edges]
    rows = []
    for s in run:
        prior = [t for t in t_up if t <= s[0]]
        if not prior:
            continue
        k = int(round((s[0] - prior[-1]) / dt))
        if k >= len(period):
            continue
        _, _, yc, yd = period[k]
        rows.append((s[0], s[1], s[6], yc, yd))
    return rows


def fmt(v, nd):
    return "" if v is None else f"{v:.{nd}f}"


def summarize(steps):
    for d in "+-":
        sel = [s for s in steps if s["dir"] == d and s["complete"]]
        if not sel:
            continue
        settled = [s["settling_s"] for s in sel if s["settling_s"] is not None]
        print(f"  {'up  ' if d == '+' else 'down'} n={len(sel)}"
              f"  OS mean {sum(s['overshoot_pct'] for s in sel) / len(sel):5.1f} %"
              f"  ss_err mean {sum(s['ss_error_mm'] for s in sel) / len(sel):+6.1f} mm"
              f"  settled {len(settled)}/{len(sel)}"
              + (f", mean {sum(settled) / len(settled):.2f} s" if settled else ""))


def main(argv):
    files = [Path(a) for a in argv] or sorted(HERE.glob("*.txt"))
    if not files:
        print("no captures found")
        return 1
    sim = load_sim(SIM_CSV) if SIM_CSV.exists() else None
    period = sim_period(sim) if sim else []
    if sim:
        print("simulation (continuous C2), same metrics, the baseline the hardware is compared with:")
        summarize(analyze_run(sim_as_run(sim)))
    else:
        print(f"note: {SIM_CSV.name} not found, overlay skipped (run mte484lab3partb_export.m)")

    step_rows, overlay_rows = [], []
    for path in files:
        runs = parse_stream(path.read_text(errors="replace").splitlines())
        for n, run in enumerate(runs, 1):
            steps = analyze_run(run)
            if not steps:   # a fixed-reference (h) run
                h = hold_summary(run)
                print(f"{path.name} run {n}: HOLD r={h['r']:.3f}  released at y={h['start_y']:.3f}"
                      f"  rest y={h['y']:.4f}  e=r-y={h['e_mm']:+6.1f} mm"
                      f"  theta={h['theta']:+.3f}  th_sat={h['th_sat']:+.3f}"
                      f"  ({h['duration_s']:.0f} s, {'stopped' if h['still'] else 'STILL MOVING'})")
                continue
            print(f"{path.name} run {n}: {len(run)} samples, {len(steps)} steps")
            summarize(steps)
            for s in steps:
                step_rows.append([path.name, n, fmt(s["t_edge_s"], 2), s["dir"], fmt(s["to_m"], 3),
                                  fmt(s["settling_s"], 2), fmt(s["overshoot_pct"], 1),
                                  fmt(s["ss_error_mm"], 1), fmt(s["peak_abs_V"], 2),
                                  fmt(s["sat_pct"], 1), int(s["complete"])])
            for row in overlay(run, period):
                overlay_rows.append([path.name, n] + [f"{v:.4f}" for v in row])

    with open(STEPS_CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["file", "run", "t_edge_s", "dir", "to_m", "settling_s", "overshoot_pct",
                    "ss_error_mm", "peak_abs_V", "sat_pct", "complete"])
        w.writerows(step_rows)
    if overlay_rows:
        with open(OVERLAY_CSV, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["file", "run", "t_s", "r", "y_hw", "y_sim_cont", "y_sim_disc"])
            w.writerows(overlay_rows)
    print(f"wrote {STEPS_CSV.name}" + (f", {OVERLAY_CSV.name}" if overlay_rows else ""))
    print(f"(c) specs for reference only: settling <= {TS_SPEC_C} s, OS < {OS_SPEC_C} %, zero ss error")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
