"""Lab 2 (f) - K3 from ball rolls with the beam held at fixed angles.

Model (manual eq. 4, 5):  phi = K2 * theta  with K2 = 0.0609,  and  y'' = K3 * phi.
With the beam held still, the ball's acceleration a is constant, so each roll
fits  y = y0 + v0*t + a*t^2/2,  and  a = K3 * K2 * theta.

For every hold run (lab2_inner_loop 'h' mode, '# start hold ref=...' line) this:
  1. finds each roll: a stretch of motion between two rests, at least 5 cm long
  2. keeps it only if it speeds up in its direction of travel, ends at a stopper
     and fits a parabola well (a pen pushing the ball back uphill fails these)
  3. fits the parabola over the middle of the roll, skipping 1 cm after release
     and 2 cm before the stopper, giving a and the mean gear angle theta
Then across all rolls it fits  a = s*theta + b:
  K3 = s / K2  (slope),  and  theta0 = -b / s  is the gear angle where the beam
  is actually level. Each roll's own K3 = a / (K2 * theta) is listed too.

The stream is t_ms,ref_raw,ref_sat,theta,V,y_m every 10 ms (y_m from Lab 2 (e)).

Usage (from any folder):
    py -3 analyze_k3.py                 # every k3_run*.txt next to this script
    py -3 analyze_k3.py file.txt ...    # specific captures
Output: k3_rolls.csv next to the first capture.
"""
import csv
import re
import statistics as st
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_GLOB = "k3_run*.txt"
ROLLS_CSV = "k3_rolls.csv"

K2 = 0.0609                  # beam angle per gear angle, given in the manual
G = 9.81                     # [m/s^2]
K3_SOLID_BALL = 5.0 / 7.0 * G   # a solid ball rolling on a flat surface, for comparison

Y_LEFT, Y_RIGHT = 0.0127, 0.3693   # ball centre at each stopper, Lab 2 (e) [m]
STOPPER_TOL = 0.015          # a roll must end this close to a stopper [m]
MEDIAN_WINDOW = 5            # samples, removes single-sample wiper spikes
REST_SAMPLES = 5             # 50 ms at the 10 ms stream
REST_M = 0.0005              # moved less than this in REST_SAMPLES = at rest [m]
MIN_ROLL_M = 0.05            # a roll covers at least this distance [m]
TRIM_START_M = 0.01          # skip this much after release [m]
TRIM_END_M = 0.02            # and this much before the stopper [m]
MIN_FIT_POINTS = 10
R2_MIN = 0.98

HOLD_RE = re.compile(r"^#\s*start hold ref=(-?\d+(?:\.\d+)?)")
START_RE = re.compile(r"^#\s*start\b")
STOP_RE = re.compile(r"^#\s*stop\b")
ROW_RE = re.compile(r"^(\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)$")

ROLL_FIELDS = ["source", "ref", "theta", "phi", "a", "k3", "v0", "r2", "n", "t_start", "y_from", "y_to"]


# ---------------------------------------------------------------- loading
def parse_holds(lines, source):
    """Every hold run in a capture: {source, ref, t[s], theta[], y[]}."""
    holds, cur = [], None
    for raw in lines:
        line = raw.strip()
        m = HOLD_RE.match(line)
        if m:
            cur = {"source": source, "ref": float(m.group(1)), "t": [], "theta": [], "y": []}
            holds.append(cur)
            continue
        if START_RE.match(line) or STOP_RE.match(line):
            cur = None
            continue
        m = ROW_RE.match(line)
        if m and cur is not None:
            t_ms, _, _, theta, _, y = m.groups()
            cur["t"].append(int(t_ms) / 1000.0)
            cur["theta"].append(float(theta))
            cur["y"].append(float(y))
    return [h for h in holds if h["y"]]


def load_holds(paths):
    holds = []
    for path in paths:
        with open(path, encoding="utf-8", errors="replace") as f:
            holds.extend(parse_holds(f, Path(path).name))
    return holds


# ---------------------------------------------------------------- fitting helpers
def median_filter(values, window):
    half = window // 2
    return [st.median(values[max(0, i - half):i + half + 1]) for i in range(len(values))]


def solve3(m, v):
    """Solve a 3x3 linear system by Gaussian elimination with partial pivoting."""
    a = [row[:] + [v[i]] for i, row in enumerate(m)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(a[r][col]))
        a[col], a[piv] = a[piv], a[col]
        if a[col][col] == 0:
            raise ValueError("singular system")
        for r in range(col + 1, 3):
            f = a[r][col] / a[col][col]
            for c in range(col, 4):
                a[r][c] -= f * a[col][c]
    x = [0.0, 0.0, 0.0]
    for r in (2, 1, 0):
        x[r] = (a[r][3] - sum(a[r][c] * x[c] for c in range(r + 1, 3))) / a[r][r]
    return x


def fit_parabola(ts, ys):
    """Least squares y = c0 + c1*tau + c2*tau^2 with tau from the first sample; returns (c0, c1, c2, r2)."""
    tau = [t - ts[0] for t in ts]
    s = [sum(x ** k for x in tau) for k in range(5)]
    sy = [sum(y * x ** k for x, y in zip(tau, ys)) for k in range(3)]
    c0, c1, c2 = solve3([[s[0], s[1], s[2]], [s[1], s[2], s[3]], [s[2], s[3], s[4]]], sy)
    y_bar = st.fmean(ys)
    ss_res = sum((y - (c0 + c1 * x + c2 * x * x)) ** 2 for x, y in zip(tau, ys))
    ss_tot = sum((y - y_bar) ** 2 for y in ys)
    return c0, c1, c2, (1.0 - ss_res / ss_tot) if ss_tot > 0 else 0.0


# ---------------------------------------------------------------- rolls
def motion_segments(yf):
    """(start, end) index pairs of stretches where the ball is moving."""
    n = len(yf)
    moving = [abs(yf[min(i + REST_SAMPLES, n - 1)] - yf[i]) >= REST_M for i in range(n)]
    segments, i = [], 0
    while i < n:
        if not moving[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and moving[j + 1]:
            j += 1
        segments.append((i, min(j + REST_SAMPLES, n - 1)))
        i = j + 1
    return segments


def find_rolls(hold):
    """Returns (rolls, rejected); each rejected entry says why."""
    t, y, theta = hold["t"], hold["y"], hold["theta"]
    yf = median_filter(y, MEDIAN_WINDOW)
    rolls, rejected = [], []
    for start, end in motion_segments(yf):
        dy = yf[end] - yf[start]
        if abs(dy) < MIN_ROLL_M:
            continue                                   # jitter or a nudge, not a roll
        d = 1 if dy > 0 else -1
        where = f"{hold['source']} ref={hold['ref']:+.3f} t={t[start]:.2f}s"
        if min(abs(yf[end] - Y_LEFT), abs(yf[end] - Y_RIGHT)) > STOPPER_TOL:
            rejected.append(f"{where}: ends at y={yf[end]:.3f} m, away from a stopper (pen push?)")
            continue
        # The window is contiguous: it ends the first time the ball gets within
        # TRIM_END_M of where it comes to rest, so a rebound off the stopper is never fitted
        idx = []
        for j in range(start, end + 1):
            if d * (yf[end] - yf[j]) < TRIM_END_M:
                break
            if d * (yf[j] - yf[start]) >= TRIM_START_M:
                idx.append(j)
        if len(idx) < MIN_FIT_POINTS:
            rejected.append(f"{where}: only {len(idx)} points inside the fit window")
            continue
        c0, c1, c2, r2 = fit_parabola([t[j] for j in idx], [y[j] for j in idx])
        a = 2.0 * c2
        if d * a <= 0:
            rejected.append(f"{where}: slows down along its path (a={a:+.4f}), not a free roll")
            continue
        if r2 < R2_MIN:
            rejected.append(f"{where}: poor parabola fit (R^2={r2:.4f})")
            continue
        th = st.fmean(theta[j] for j in idx)
        rolls.append({"source": hold["source"], "ref": hold["ref"], "theta": th, "phi": K2 * th,
                      "a": a, "v0": c1, "r2": r2, "n": len(idx), "t_start": t[idx[0]],
                      "y_from": yf[idx[0]], "y_to": yf[idx[-1]]})
    return rolls, rejected


MIN_THETA_FOR_K3 = 0.02      # below this, a / (K2*theta) is dominated by noise [rad]
MIN_SLOPE = 1e-6             # a vs theta slope below this means acceleration did not follow angle


def roll_k3(roll):
    """K3 from one roll; NaN when the beam is too close to level for the ratio to mean anything."""
    if abs(roll["theta"]) < MIN_THETA_FOR_K3:
        return float("nan")
    return roll["a"] / (K2 * roll["theta"])


def fit_k3(rolls):
    """a = s*theta + b across all rolls: K3 = s/K2, level offset theta0 = -b/s."""
    # Distinct hold angles, not measured theta: rolls at one hold differ slightly in theta
    if len({r.get("ref", round(r["theta"], 2)) for r in rolls}) < 2:
        raise ValueError("need rolls at two or more different hold angles")
    xs = [r["theta"] for r in rolls]
    ys = [r["a"] for r in rolls]
    x_bar, y_bar = st.fmean(xs), st.fmean(ys)
    sxx = sum((x - x_bar) ** 2 for x in xs)
    s = sum((x - x_bar) * (yv - y_bar) for x, yv in zip(xs, ys)) / sxx
    if abs(s) < MIN_SLOPE:
        raise ValueError("acceleration does not change with angle; check the rolls")
    b = y_bar - s * x_bar
    ss_res = sum((yv - (s * x + b)) ** 2 for x, yv in zip(xs, ys))
    ss_tot = sum((yv - y_bar) ** 2 for yv in ys)
    return {"slope": s, "intercept": b, "k3": s / K2, "theta0": -b / s,
            "r2": 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0, "n": len(rolls)}


def fit_k3_resistance(rolls):
    """a = s*theta + b_left*[rolling left] + b_right*[rolling right], one shared slope.

    Rolling resistance opposes motion, so it shifts the acceleration by a constant
    that depends on the direction of travel. Fitting one slope with a separate
    offset per direction keeps that friction out of K3:
      K3 = s / K2;  resistance = |b_left - b_right| / 2  [m/s^2];
      the ball cannot start rolling for theta between -b_left/s and -b_right/s;
      theta_level = the middle of that band, where the beam is actually level.
    """
    left = [r for r in rolls if r["y_to"] < r["y_from"]]
    right = [r for r in rolls if r["y_to"] > r["y_from"]]
    if not left or not right:
        raise ValueError("need rolls in both directions to separate rolling resistance")
    if len({r.get("ref", round(r["theta"], 2)) for r in rolls}) < 3:
        raise ValueError("need three or more hold angles for a slope plus two offsets")
    rows = [([r["theta"], 1.0, 0.0], r["a"]) for r in left] + [([r["theta"], 0.0, 1.0], r["a"]) for r in right]
    ata = [[sum(x[i] * x[j] for x, _ in rows) for j in range(3)] for i in range(3)]
    aty = [sum(x[i] * a for x, a in rows) for i in range(3)]
    s, b_left, b_right = solve3(ata, aty)
    if abs(s) < MIN_SLOPE:
        raise ValueError("acceleration does not change with angle; check the rolls")
    residuals = [a - (s * x[0] + b_left * x[1] + b_right * x[2]) for x, a in rows]
    band = sorted((-b_left / s, -b_right / s))
    return {"slope": s, "k3": s / K2, "b_left": b_left, "b_right": b_right,
            "resistance": abs(b_left - b_right) / 2, "band": band,
            "theta_level": (band[0] + band[1]) / 2,
            "rms": (sum(e * e for e in residuals) / len(residuals)) ** 0.5,
            "n_left": len(left), "n_right": len(right)}


# ---------------------------------------------------------------- output
def write_csv(path, rolls):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=ROLL_FIELDS)
        w.writeheader()
        for r in rolls:
            row = {**r, "k3": roll_k3(r)}
            w.writerow({k: (f"{row[k]:.6f}" if isinstance(row[k], float) else row[k]) for k in ROLL_FIELDS})


def print_rolls(rolls):
    print(f"\n{'source':<14}{'ref':>7}{'theta':>8}{'phi':>9}{'a m/s2':>10}{'K3':>8}{'v0':>8}{'R2':>8}{'y from->to':>16}")
    for r in rolls:
        print(f"{r['source']:<14}{r['ref']:>+7.2f}{r['theta']:>+8.3f}{r['phi']:>+9.4f}{r['a']:>+10.4f}"
              f"{roll_k3(r):>+8.2f}{r['v0']:>+8.3f}{r['r2']:>8.4f}   {r['y_from']:.3f}->{r['y_to']:.3f}")


def print_by_angle(rolls):
    print("\nper angle (mean of rolls):")
    for ref in sorted({r["ref"] for r in rolls}):
        group = [r for r in rolls if r["ref"] == ref]
        k3s = [k for k in (roll_k3(r) for r in group) if k == k]   # drop NaN (near level)
        a_mean = st.fmean(r["a"] for r in group)
        if not k3s:
            print(f"  ref {ref:+.2f} rad: {len(group)} roll(s), a = {a_mean:+.4f} m/s^2, K3 n/a (beam near level)")
            continue
        spread = f" +- {st.stdev(k3s):.2f}" if len(k3s) > 1 else ""
        print(f"  ref {ref:+.2f} rad: {len(group)} roll(s), a = {a_mean:+.4f} m/s^2, "
              f"K3 = {st.fmean(k3s):+.2f}{spread}")


def main(argv):
    paths = [Path(a) for a in argv] or sorted(HERE.glob(DEFAULT_GLOB))
    missing = [p for p in paths if not p.exists()]
    if missing or not paths:
        print("missing capture: " + (", ".join(str(p) for p in missing) or DEFAULT_GLOB))
        return 1

    holds = load_holds(paths)
    rolls, rejected = [], []
    for hold in holds:
        found, why = find_rolls(hold)
        rolls += found
        rejected += why
    print(f"captures: {', '.join(p.name for p in paths)}; {len(holds)} hold run(s), "
          f"{len(rolls)} roll(s) kept, {len(rejected)} movement(s) rejected")
    for reason in rejected:
        print(f"  # rejected {reason}")
    for ref in sorted({h["ref"] for h in holds} - {r["ref"] for r in rolls}):
        print(f"  # no roll at ref {ref:+.2f} rad (ball did not roll on its own?)")
    if not rolls:
        print("no rolls found; hold an angle with h<rad>, g, then release the ball near the high end")
        return 1

    print_rolls(rolls)
    print_by_angle(rolls)
    write_csv(paths[0].resolve().parent / ROLLS_CSV, rolls)

    try:
        fit = fit_k3(rolls)
    except ValueError as e:
        print(f"\ncannot fit a vs theta: {e}")
        return 1
    k3s = [k for k in (roll_k3(r) for r in rolls) if k == k] or [float("nan")]
    print(f"\na vs theta over {fit['n']} rolls:  a = {fit['slope']:+.4f} * theta {fit['intercept']:+.5f}   "
          f"R^2 = {fit['r2']:.4f}")
    print(f"  K3 = slope / K2 = {fit['k3']:+.3f} m/s^2 per rad of beam angle")
    print(f"  level offset: beam is level at theta = {fit['theta0']:+.4f} rad "
          f"(phi offset {K2 * fit['theta0']:+.5f} rad)")
    print(f"  per-roll K3: mean {st.fmean(k3s):+.3f}, sd {st.stdev(k3s) if len(k3s) > 1 else 0.0:.3f}")
    print(f"  compare: solid ball rolling on a flat surface (5/7)g = {K3_SOLID_BALL:.3f}; "
          f"|K3| / (5/7)g = {abs(fit['k3']) / K3_SOLID_BALL:.3f}")
    try:
        res = fit_k3_resistance(rolls)
    except ValueError as e:
        print(f"\nrolling-resistance model not fitted: {e}")
    else:
        print(f"\nwith rolling resistance (one slope, an offset per direction; "
              f"{res['n_left']} rolls left, {res['n_right']} right):")
        print(f"  K3 = {res['k3']:+.3f} m/s^2 per rad of beam angle   (RMS residual {res['rms']:.4f} m/s^2)")
        print(f"  rolling resistance = {res['resistance']:.4f} m/s^2 "
              f"(= a beam slope of {res['resistance'] / G * 1000:.2f} mrad)")
        print(f"  ball cannot start rolling for theta between {res['band'][0]:+.3f} and {res['band'][1]:+.3f} rad")
        print(f"  beam level at theta = {res['theta_level']:+.3f} rad "
              f"(phi offset {K2 * res['theta_level']:+.5f} rad)")
        print(f"  |K3| / (5/7)g = {abs(res['k3']) / K3_SOLID_BALL:.3f}")
    print(f"\nwrote {ROLLS_CSV} to {paths[0].resolve().parent}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
