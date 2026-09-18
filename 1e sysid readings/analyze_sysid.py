"""Lab 1 (e) - plant identification from the lab1_sysid step captures.

For every captured step, measures percent overshoot (OS) and time to first
peak (Tp), converts them to the closed-loop damping ratio and natural
frequency, then back to the plant parameters of  K1 / (s (tau s + 1)):

    zeta = -ln(OS/100) / sqrt(pi^2 + ln^2(OS/100))
    wn   = pi / (Tp sqrt(1 - zeta^2))
    tau  = 1 / (2 zeta wn)
    K1   = wn^2 tau / Kp          (Kp < 0, so K1 < 0)

Usage (from any folder):
    python analyze_sysid.py                  # every "kp*.txt" capture next to this script
    python analyze_sysid.py file1.txt ...    # specific captures

Outputs (written next to this script):
    sysid_steps.csv       one row per step, with flags
    sysid_summary.csv     mean and sd per gain, per gain and direction, and overall
    sysid_responses.csv   theta and V against time for every step, for report figures
"""
import csv
import math
import re
import statistics as st
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_PATTERN = "kp*.txt"

THETA0_SAMPLES = 3          # samples averaged for the starting angle (gear has not moved yet)
SS_FRACTION = 0.2           # last part of the window averaged for the steady-state angle
MEDIAN_WINDOW = 5           # centred median filter length, removes single-sample spikes
PEAK_FIT_HALF_MS = 8.0      # half-width of the parabola fit around the first peak [ms]
ONSET_FRAC = 0.05           # fraction of the step that counts as the gear starting to move
MIN_STEP = 0.1              # gear must move at least this far toward the new reference [rad]
OS_MIN_PCT = 3.0            # below this the overshoot is too small to trust
OS_MAX_PCT = 60.0           # above this the response is not a clean second-order step
FLAT_TOL_FRAC = 0.1         # "at the peak" = within this fraction of the overshoot
FLAT_RATIO_MAX = 1.6        # peak wider than this x an ideal second-order peak = flat
SS_ERR_WARN = 0.01          # steady-state error worth a warning [rad]

K1_RANGE = (1.50, 3.00)     # expected |K1| [rad/(V s)]
TAU_RANGE = (0.010, 0.035)  # expected tau [s]

HEADER_RE = re.compile(r"^#\s*capture\s+(\d+)\s+kp=(-?\d+\.?\d*)\s+dir=([+-])\s+"
                       r"dt_us=(\d+)\s+samples=(\d+)\s+sat=(\d+)")
END_RE = re.compile(r"^#\s*end capture\s+(\d+)")
ROW_RE = re.compile(r"^(\d+),(\d+),(-?\d+\.\d+),(-?\d+\.\d+),(\d+),(-?\d+\.\d+)$")

STEP_FIELDS = ["source", "id", "kp", "dir", "sat", "theta0", "ss", "ref", "ss_err", "peak",
               "os_pct", "tp_ms", "onset_ms", "v_max", "zeta", "wn", "k1", "tau",
               "peak_width_ms", "flat_ratio", "exclude", "warn"]
SUMMARY_FIELDS = ["group", "kp", "dir", "n", "os_mean", "tp_ms_mean", "zeta_mean", "wn_mean",
                  "k1_mean", "k1_sd", "tau_mean", "tau_sd"]


# ---------------------------------------------------------------- loading
def new_capture(match, source):
    cid, kp, sign, dt_us, samples, sat = match.groups()
    return {"source": source, "id": int(cid), "kp": float(kp), "dir": 1 if sign == "+" else -1,
            "dt_us": int(dt_us), "samples": int(samples), "sat": int(sat),
            "ref": None, "t": [], "theta": [], "v": []}


def parse_captures(lines, source):
    """Return the complete capture blocks in a CoolTerm log, one dict per step."""
    captures, cur = [], None
    for raw in lines:
        line = raw.strip()
        m = HEADER_RE.match(line)
        if m:
            if cur is not None:
                print(f"# skipped capture {cur['id']} in {source}: no end line before the next capture")
            cur = new_capture(m, source)
            continue
        if cur is None:
            continue
        m = ROW_RE.match(line)
        if m:
            _, t_us, ref, v, _, theta = m.groups()
            cur["ref"] = float(ref)
            cur["t"].append(int(t_us) / 1e6)
            cur["v"].append(float(v))
            cur["theta"].append(float(theta))
            continue
        m = END_RE.match(line)
        if m:
            if int(m.group(1)) == cur["id"] and len(cur["theta"]) == cur["samples"]:
                captures.append(cur)
            else:
                print(f"# skipped incomplete capture {cur['id']} in {source}")
            cur = None
    return captures


def load_file(path):
    with open(path, encoding="utf-8", errors="replace") as f:
        return parse_captures(f, Path(path).name)


# ---------------------------------------------------------------- signal helpers
def median_filter(values, window):
    half = window // 2
    return [st.median(values[max(0, i - half):i + half + 1]) for i in range(len(values))]


def solve3(a, b):
    """Solve a 3x3 linear system by Gaussian elimination with partial pivoting."""
    m = [row[:] + [rhs] for row, rhs in zip(a, b)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(m[r][col]))
        m[col], m[pivot] = m[pivot], m[col]
        for r in range(col + 1, 3):
            f = m[r][col] / m[col][col]
            m[r] = [x - f * y for x, y in zip(m[r], m[col])]
    x = [0.0, 0.0, 0.0]
    for r in (2, 1, 0):
        x[r] = (m[r][3] - sum(m[r][c] * x[c] for c in range(r + 1, 3))) / m[r][r]
    return x


def fit_vertex(ts, ys):
    """Least-squares parabola through (ts, ys); returns its vertex (t, y)."""
    t0 = st.fmean(ts)                       # centre the time axis for conditioning
    xs = [t - t0 for t in ts]
    s = [sum(x ** k for x in xs) for k in range(5)]
    r = [sum(y * x ** k for x, y in zip(xs, ys)) for k in range(3)]
    c, b, a = solve3([[s[0], s[1], s[2]], [s[1], s[2], s[3]], [s[2], s[3], s[4]]], r)
    if a == 0.0:
        raise ValueError("flat data, no vertex")
    xv = -b / (2.0 * a)
    return t0 + xv, a * xv * xv + b * xv + c


# ---------------------------------------------------------------- second-order formulas
def zeta_from_os(os_pct):
    ln = math.log(os_pct / 100.0)
    return -ln / math.sqrt(math.pi ** 2 + ln ** 2)


def wn_from_tp(tp, zeta):
    return math.pi / (tp * math.sqrt(1.0 - zeta ** 2))


def plant_from_closed_loop(zeta, wn, kp):
    """Invert  wn^2 = Kp K1 / tau  and  2 zeta wn = 1 / tau."""
    tau = 1.0 / (2.0 * zeta * wn)
    return wn ** 2 * tau / kp, tau


# ---------------------------------------------------------------- per-step analysis
def find_peak(cap, filtered):
    """Index, time and value of the first peak (parabola-refined), and whether it hit the window end."""
    sign = cap["dir"]
    i_pk = max(range(len(filtered)), key=lambda i: sign * filtered[i])
    half = max(2, round(PEAK_FIT_HALF_MS * 1000 / cap["dt_us"]))
    lo, hi = max(0, i_pk - half), min(len(filtered), i_pk + half + 1)
    at_end = hi == len(filtered)
    try:
        t_pk, pk = fit_vertex(cap["t"][lo:hi], filtered[lo:hi])
        if not cap["t"][lo] <= t_pk <= cap["t"][hi - 1]:
            raise ValueError("vertex outside the fit window")
    except (ValueError, ZeroDivisionError):
        t_pk, pk = cap["t"][i_pk], filtered[i_pk]
    return i_pk, t_pk, pk, at_end


def peak_width(filtered, i_pk, tol):
    """Number of consecutive samples around the peak that stay within tol of it."""
    level = filtered[i_pk]
    lo = hi = i_pk
    while lo > 0 and abs(filtered[lo - 1] - level) <= tol:
        lo -= 1
    while hi < len(filtered) - 1 and abs(filtered[hi + 1] - level) <= tol:
        hi += 1
    return hi - lo + 1


def flat_peak_ratio(cap, filtered, i_pk, overshoot, wd):
    """Measured peak width [ms] and its ratio to an ideal second-order peak with the same wd."""
    width_s = peak_width(filtered, i_pk, FLAT_TOL_FRAC * abs(overshoot)) * cap["dt_us"] / 1e6
    ideal_s = 2.0 * math.acos(1.0 - FLAT_TOL_FRAC) / wd
    return width_s * 1000.0, width_s / ideal_s


def onset_ms(cap, filtered, theta0, step):
    """Time for the gear to cover ONSET_FRAC of the step (shows backlash / stiction delay)."""
    for t, th in zip(cap["t"], filtered):
        if (th - theta0) / step > ONSET_FRAC:
            return t * 1000.0
    return math.nan


def identify(rec, cap, filtered, i_pk, t_pk):
    """Fill zeta, wn, K1, tau and the peak-shape check into rec."""
    rec["zeta"] = zeta_from_os(rec["os_pct"])
    rec["wn"] = wn_from_tp(t_pk, rec["zeta"])
    rec["k1"], rec["tau"] = plant_from_closed_loop(rec["zeta"], rec["wn"], cap["kp"])
    wd = rec["wn"] * math.sqrt(1.0 - rec["zeta"] ** 2)
    rec["peak_width_ms"], rec["flat_ratio"] = flat_peak_ratio(cap, filtered, i_pk, rec["peak"] - rec["ss"], wd)


def exclusions(cap, moved, os_pct, t_pk, at_end):
    """Reasons to leave a step out of the averages."""
    flags = ["saturated"] if cap["sat"] > 0 else []
    if not moved:
        return flags + ["no_step"]           # gear did not follow the reference
    if at_end or os_pct < OS_MIN_PCT:
        flags.append("low_os")
    if os_pct > OS_MAX_PCT:
        flags.append("high_os")
    if t_pk <= 0.0:
        flags.append("early_peak")
    return flags


def analyse_step(cap):
    theta = cap["theta"]
    filtered = median_filter(theta, MEDIAN_WINDOW)
    theta0 = st.fmean(theta[:THETA0_SAMPLES])
    ss = st.fmean(theta[-max(1, int(len(theta) * SS_FRACTION)):])
    step = ss - theta0
    moved = cap["dir"] * step >= MIN_STEP
    i_pk, t_pk, pk, at_end = find_peak(cap, filtered)

    rec = {"source": cap["source"], "id": cap["id"], "kp": cap["kp"], "dir": cap["dir"],
           "sat": cap["sat"], "theta0": theta0, "ss": ss, "ref": cap["ref"], "ss_err": cap["ref"] - ss,
           "peak": pk, "os_pct": (pk - ss) / step * 100.0 if moved else math.nan, "tp_ms": t_pk * 1000.0,
           "onset_ms": onset_ms(cap, filtered, theta0, step) if moved else math.nan,
           "v_max": max(abs(v) for v in cap["v"]),
           "zeta": math.nan, "wn": math.nan, "k1": math.nan, "tau": math.nan,
           "peak_width_ms": math.nan, "flat_ratio": math.nan}

    exclude = exclusions(cap, moved, rec["os_pct"], t_pk, at_end)
    # Saturated or high-OS steps still get values so they can be inspected
    if moved and not at_end and t_pk > 0.0 and 0.0 < rec["os_pct"] < 100.0:
        identify(rec, cap, filtered, i_pk, t_pk)

    warn = []
    if rec["flat_ratio"] > FLAT_RATIO_MAX:
        warn.append("flat_peak")
    if abs(rec["ss_err"]) > SS_ERR_WARN:
        warn.append("ss_error")
    rec["exclude"] = ";".join(exclude)
    rec["warn"] = ";".join(warn)
    return rec


# ---------------------------------------------------------------- summary
def group_stats(group, kp, direction, recs):
    def mean(key):
        return st.fmean(r[key] for r in recs)

    def sd(key):
        return st.stdev(r[key] for r in recs) if len(recs) > 1 else 0.0

    return {"group": group, "kp": kp, "dir": direction, "n": len(recs),
            "os_mean": mean("os_pct"), "tp_ms_mean": mean("tp_ms"), "zeta_mean": mean("zeta"),
            "wn_mean": mean("wn"), "k1_mean": mean("k1"), "k1_sd": sd("k1"),
            "tau_mean": mean("tau"), "tau_sd": sd("tau")}


def summarise(records):
    """Mean and sd per gain and direction, per gain, and over every usable step."""
    good = [r for r in records if not r["exclude"]]
    rows = []
    for kp in sorted({r["kp"] for r in good}, reverse=True):
        at_kp = [r for r in good if r["kp"] == kp]
        for direction in (1, -1):
            same_dir = [r for r in at_kp if r["dir"] == direction]
            if same_dir:
                rows.append(group_stats("kp_dir", kp, "+" if direction > 0 else "-", same_dir))
        rows.append(group_stats("kp", kp, "", at_kp))
    if good:
        rows.append(group_stats("all", "", "", good))
    return rows


# ---------------------------------------------------------------- output
def fmt(value, digits=4):
    if isinstance(value, float):
        return "" if math.isnan(value) else f"{value:.{digits}f}"
    return str(value)


def write_csv(path, fields, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(fields)
        for row in rows:
            w.writerow([fmt(row[k]) for k in fields])


def write_responses(path, captures):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["source", "id", "kp", "dir", "t_ms", "theta", "v"])
        for c in captures:
            for t, th, v in zip(c["t"], c["theta"], c["v"]):
                w.writerow([c["source"], c["id"], c["kp"], c["dir"], f"{t * 1000:.3f}", th, v])


def print_steps(records):
    print(f"{'file':<22}{'id':>4}{'Kp':>8}{'dir':>4}{'OS%':>7}{'Tp ms':>8}{'onset':>7}"
          f"{'zeta':>7}{'wn':>7}{'K1':>8}{'tau ms':>8}{'Vmax':>6}  flags")
    for r in records:
        flags = ";".join(x for x in (r["exclude"], r["warn"]) if x)
        print(f"{r['source'][:21]:<22}{r['id']:>4}{r['kp']:>8.2f}{'+' if r['dir'] > 0 else '-':>4}"
              f"{r['os_pct']:>7.1f}{r['tp_ms']:>8.1f}{fmt(r['onset_ms'], 0):>7}{fmt(r['zeta'], 3):>7}"
              f"{fmt(r['wn'], 1):>7}{fmt(r['k1'], 3):>8}{fmt(r['tau'] * 1000, 1):>8}{r['v_max']:>6.2f}  {flags}")


def print_summary(rows):
    print(f"\n{'group':<8}{'Kp':>8}{'dir':>4}{'n':>4}{'OS%':>7}{'Tp ms':>8}{'K1':>9}{'+-':>7}{'tau ms':>8}{'+-':>6}")
    for r in rows:
        kp = f"{r['kp']:.2f}" if r["kp"] != "" else ""
        print(f"{r['group']:<8}{kp:>8}{r['dir']:>4}{r['n']:>4}{r['os_mean']:>7.1f}{r['tp_ms_mean']:>8.1f}"
              f"{r['k1_mean']:>9.3f}{r['k1_sd']:>7.3f}{r['tau_mean'] * 1000:>8.2f}{r['tau_sd'] * 1000:>6.2f}")


def print_range_check(rows):
    overall = [r for r in rows if r["group"] == "all"]
    if not overall:
        print("\nno usable steps")
        return
    r = overall[0]
    k1_ok = K1_RANGE[0] <= abs(r["k1_mean"]) <= K1_RANGE[1]
    tau_ok = TAU_RANGE[0] <= r["tau_mean"] <= TAU_RANGE[1]
    print(f"\nK1  = {r['k1_mean']:.3f} rad/(V s)  |K1| expected {K1_RANGE[0]}-{K1_RANGE[1]}: {'OK' if k1_ok else 'OUT OF RANGE'}")
    print(f"tau = {r['tau_mean'] * 1000:.2f} ms  expected {TAU_RANGE[0] * 1000:.0f}-{TAU_RANGE[1] * 1000:.0f} ms: "
          f"{'OK' if tau_ok else 'OUT OF RANGE'}")


def main(argv):
    paths = [Path(a) for a in argv] or sorted(HERE.glob(DEFAULT_PATTERN))
    if not paths:
        print(f"no captures found; pass files or add {DEFAULT_PATTERN} next to this script")
        return 1
    captures = [c for p in paths for c in load_file(p)]
    if not captures:
        print("no complete capture blocks found")
        return 1

    records = [analyse_step(c) for c in captures]
    rows = summarise(records)
    print_steps(records)
    print_summary(rows)
    print_range_check(rows)

    write_csv(HERE / "sysid_steps.csv", STEP_FIELDS, records)
    write_csv(HERE / "sysid_summary.csv", SUMMARY_FIELDS, rows)
    write_responses(HERE / "sysid_responses.csv", captures)
    print(f"\nwrote sysid_steps.csv, sysid_summary.csv, sysid_responses.csv in {HERE}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
