"""Lab 2 (e) - ball position sensor scaling from the lab2_ball_sensor calibration.

Each '# point' line from the sketch is one ball position: the y typed in
(ball centre, metres from the left stopper's inner face) and the median of
500 A1 readings taken with the ball at rest against a non-conductive stop.

This fits   y [m] = BALL_M * reading [counts] + BALL_OFFSET   by least squares
through every point and reports:
  - BALL_M, BALL_OFFSET and R^2, plus the two lines to paste into the sketch
  - the residual at every point in mm (worst and RMS), flagging any over 3 mm
  - repeatability: the spread of readings at the same y across passes, in mm
  - sensor noise at rest: each point's standard deviation, in mm

The sensor reads where the ball touches the rods, directly under its centre,
so the end stoppers give y = R = 0.0127 m and y = L - R = 0.4043 m.

Corrections (RELABEL, EXCLUDE below) fix what the bench showed, without
editing the raw captures, and every one is printed when it is applied.

Usage (from any folder):
    py -3 analyze_ball_scaling.py                 # every ball_cal*.txt next to this script
    py -3 analyze_ball_scaling.py file.txt ...    # specific captures
"""
import math
import re
import statistics as st
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_GLOB = "ball_cal*.txt"

# ---------------------------------------------------------------- bench corrections
# The right stopper was typed as y = 0.4043 (L - R from the manual's 41.7 cm beam),
# but on this station its inner face is at 0.300 + 0.082 = 0.382 m (ruler 30 cm mark
# plus the measured 8.2 cm gap), so a ball resting on it has its centre at
# 0.382 - 0.0127 = 0.3693 m. The pass 1 line puts those readings at 0.369-0.371 m.
RELABEL = {0.4043: 0.3693}

# Pass 2's 30 cm (pt 10) and 20 cm (pt 12) points disagreed with pass 1 by 4.6 and
# 2.7 mm. A tie-break re-measurement (ball_cal3.txt) agreed with pass 1 to within
# 2 mm at the same steep tilt, so those two were pen placement errors.
EXCLUDE = {("ball_cal2.txt", 10), ("ball_cal2.txt", 12)}

FLAG_MM = 3.0                # a residual beyond this means misplacement or nonlinearity [mm]
ADC_FULL_SCALE = 16383       # 14-bit reading

POINT_RE = re.compile(r"^#\s*point\s+(\d+)\s+y=(-?\d+(?:\.\d+)?)\s+raw_median=(-?\d+(?:\.\d+)?)\s+"
                      r"raw_mean=(-?\d+(?:\.\d+)?)\s+raw_sd=(\d+(?:\.\d+)?)\s+n=(\d+)\s+"
                      r"theta=(-?\d+(?:\.\d+)?)")


# ---------------------------------------------------------------- loading
def parse_points(lines):
    """One dict per '# point' line, in capture order."""
    points = []
    for raw in lines:
        m = POINT_RE.match(raw.strip())
        if m:
            idx, y, med, mean, sd, n, theta = m.groups()
            points.append({"index": int(idx), "y": float(y), "raw": float(med), "mean": float(mean),
                           "sd": float(sd), "n": int(n), "theta": float(theta)})
    return points


def load_points(paths):
    """Points from every capture, each tagged with the file it came from."""
    points = []
    for path in paths:
        with open(path, encoding="utf-8", errors="replace") as f:
            points.extend({**p, "source": Path(path).name} for p in parse_points(f))
    return points


def apply_corrections(points, relabel, exclude):
    """New point list with bench labels corrected and excluded points dropped.
    Returns (kept points, human-readable notes on every change)."""
    kept, notes = [], []
    for p in points:
        if (p.get("source"), p["index"]) in exclude:
            notes.append(f"excluded {p.get('source')} pt {p['index']} (y = {p['y']:.4f} m)")
            continue
        new_y = relabel.get(round(p["y"], 4))
        if new_y is not None:
            notes.append(f"relabelled {p.get('source')} pt {p['index']}: y {p['y']:.4f} -> {new_y:.4f} m")
            p = {**p, "y": new_y}
        kept.append(p)
    return kept, notes


# ---------------------------------------------------------------- fit
def fit(points):
    """Least-squares y = m * raw + offset; returns (m, offset, r2)."""
    if len({p["y"] for p in points}) < 2:
        raise ValueError("need points at two or more different positions to fit a line")
    xs = [p["raw"] for p in points]
    ys = [p["y"] for p in points]
    x_bar, y_bar = st.fmean(xs), st.fmean(ys)
    sxx = sum((x - x_bar) ** 2 for x in xs)
    sxy = sum((x - x_bar) * (y - y_bar) for x, y in zip(xs, ys))
    if sxx == 0:
        raise ValueError("every reading is identical; check the ball sensor is connected (A1)")
    m = sxy / sxx
    offset = y_bar - m * x_bar
    ss_res = sum((y - (m * x + offset)) ** 2 for x, y in zip(xs, ys))
    ss_tot = sum((y - y_bar) ** 2 for y in ys)
    return m, offset, 1.0 - ss_res / ss_tot


def residuals_mm(points, m, offset):
    """Fitted minus actual y at each point [mm]."""
    return [((m * p["raw"] + offset) - p["y"]) * 1000.0 for p in points]


def flag_points(points, residuals):
    return [p for p, r in zip(points, residuals) if abs(r) > FLAG_MM]


def repeatability_mm(points, m):
    """{y: spread of the readings taken at that y, converted to mm}."""
    by_y = {}
    for p in points:
        by_y.setdefault(round(p["y"], 4), []).append(p["raw"])
    return {y: (max(raws) - min(raws)) * abs(m) * 1000.0 for y, raws in by_y.items()}


def noise_mm(point, m):
    """Standard deviation of one point's 500 readings, converted to mm."""
    return point["sd"] * abs(m) * 1000.0


# ---------------------------------------------------------------- output
def print_report(points, m, offset, r2):
    res = residuals_mm(points, m, offset)
    print(f"\n{'pt':>3} {'y [m]':>8} {'median':>9} {'fitted y':>9} {'resid mm':>9} {'noise mm':>9} {'theta':>8}")
    for p, r in zip(points, res):
        print(f"{p['index']:>3} {p['y']:>8.4f} {p['raw']:>9.1f} {m * p['raw'] + offset:>9.4f} "
              f"{r:>9.2f} {noise_mm(p, m):>9.3f} {p['theta']:>8.4f}")

    rms = math.sqrt(st.fmean(r * r for r in res))
    print(f"\nfit over {len(points)} points at {len({p['y'] for p in points})} positions:")
    print(f"  y [m] = {m:.6e} * reading + {offset:.6f}     R^2 = {r2:.6f}")
    print(f"  residual: worst {max(abs(r) for r in res):.2f} mm, RMS {rms:.2f} mm")
    print(f"  resolution: {abs(m) * 1000:.4f} mm per count, "
          f"{abs(m) * ADC_FULL_SCALE:.3f} m over the full 14-bit range")

    counts = {}
    for p in points:
        counts[round(p["y"], 4)] = counts.get(round(p["y"], 4), 0) + 1
    repeated = {y: s for y, s in repeatability_mm(points, m).items() if counts[y] > 1}
    if repeated:
        worst_y = max(repeated, key=repeated.get)
        print(f"  repeatability between passes: worst {repeated[worst_y]:.2f} mm (at y = {worst_y:.4f} m), "
              f"mean {st.fmean(repeated.values()):.2f} mm")
    noise = [noise_mm(p, m) for p in points]
    print(f"  noise at rest (sd of 500 readings): mean {st.fmean(noise):.3f} mm, worst {max(noise):.3f} mm")

    flagged = flag_points(points, res)
    if flagged:
        print(f"\nWARNING: {len(flagged)} point(s) more than {FLAG_MM:g} mm off the line: "
              + ", ".join(f"pt {p['index']} (y = {p['y']:.4f})" for p in flagged))
        print("  re-measure them (ball against the stop, nothing touching it) or check the ruler zero")

    print("\nFor the sketch (same form as THETA_M / THETA_OFFSET):")
    print(f"const float BALL_M      = {m:.6e}f;   // m per ADC count")
    print(f"const float BALL_OFFSET = {offset:.6f}f;      // m")


def main(argv):
    paths = [Path(a) for a in argv] or sorted(HERE.glob(DEFAULT_GLOB))
    missing = [p for p in paths if not p.exists()]
    if missing or not paths:
        print("missing capture: " + (", ".join(str(p) for p in missing) or DEFAULT_GLOB))
        return 1
    points, notes = apply_corrections(load_points(paths), RELABEL, EXCLUDE)
    print(f"captures: {', '.join(p.name for p in paths)}")
    for note in notes:
        print(f"# {note}")
    if not points:
        print("no '# point' lines found; measure with y<metres> then m in lab2_ball_sensor")
        return 1
    try:
        m, offset, r2 = fit(points)
    except ValueError as e:
        print(f"cannot fit: {e}")
        return 1
    print_report(points, m, offset, r2)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
