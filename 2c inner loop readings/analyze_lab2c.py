"""Lab 2 (c) - checks D1[z] step captures from lab2_inner_loop against the
hardware specs and compares them with the part (b) simulation.

For every captured step (the sketch's 'c' / 'C' commands) this measures:
  overshoot      peak beyond the new reference / 1.4 rad step x 100
  settling time  last time theta is outside +-2 % of the step (+-0.028 rad)
                 around the reference, after a 5-point median filter that
                 removes single-sample sensor spikes
  ss error       mean of (ref - theta) over the last SS_WINDOW_MS of the step
  peak |V|       largest command sent to the motor, stiction offset included,
                 and its excess over 6 V

Spec verdict (manual p. 27, hardware): settling <= 0.40 s, overshoot <= 5 %,
zero steady-state error, |V| <= 6 V with an excess the size of the stiction
offsets allowed, theta inside +-pi/4, both step directions captured.

The comparison uses sim_lab2b_steps.csv (exported from MATLAB by
mte484lab2partb_export.m) when it sits next to this script; otherwise it
falls back to this script's own model of the 1 ms sampled-data loop.

Outputs, next to this script:
  lab2c_steps.csv     one row per step
  lab2c_overlay.csv   measured vs simulated theta and V, first step of each direction

Usage (from any folder):
    py -3 analyze_lab2c.py                 # every "*.txt" next to this script
    py -3 analyze_lab2c.py file1.txt ...   # specific captures
"""
import csv
import math
import re
import statistics as st
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SIM_CSV = HERE / "sim_lab2b_steps.csv"
STEPS_CSV = HERE / "lab2c_steps.csv"
OVERLAY_CSV = HERE / "lab2c_overlay.csv"

# ---------------------------------------------------------------- design (must match the sketch)
K1 = -1.883                  # plant gain, Lab 1 (e) [rad/(V s)]
TAU = 0.0216                 # plant time constant, Lab 1 (e) [s]
TS = 0.001                   # sampling period [s]
D1_NUM = (-2.454674639351807, 2.284293610869540)   # Tustin c2d of C1(s), Ts = 1 ms
D1_DEN = (1.0, -0.967822458684839)
STICTION_POS = 0.277         # added when u > 0 [V]
STICTION_NEG = 0.259         # subtracted when u < 0 [V]
REF_SAT = 0.7                # saturated reference level [rad]
STEP = 2 * REF_SAT           # step size, -0.7 -> +0.7 rad
CAPTURE_SAMPLES = 800        # samples per captured step, 1 ms each

# ---------------------------------------------------------------- specs and analysis settings
TS_MAX = 0.40                # 2 % settling time, hardware spec [s]
OS_MAX_PCT = 5.0             # overshoot spec [%]
V_MAX = 6.0                  # motor voltage spec [V]
V_ALLOW = 0.30               # excess allowed over V_MAX, ~ the largest stiction offset [V]
SS_TOL = 0.01                # |steady-state error| accepted as zero, noise is ~0.004 rad [rad]
THETA_MAX = math.pi / 4      # gear damage limit [rad]
SETTLE_FRAC = 0.02           # settling band as a fraction of the step
SS_WINDOW_MS = 100           # tail used for the steady-state error [ms]
MEDIAN_WINDOW = 5            # centred median filter length

HEADER_RE = re.compile(r"^#\s*capture\s+(\d+)\s+ctrl=(\S+)\s+dir=([+-])\s+dt_us=(\d+)\s+"
                       r"samples=(\d+)\s+over6=(\d+)")
END_RE = re.compile(r"^#\s*end capture\s+(\d+)")
ROW_RE = re.compile(r"^(\d+),(\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(\d+),(-?\d+\.\d+)$")

STEP_FIELDS = ["source", "id", "dir", "os_pct", "ts_s", "ss_err", "v_peak", "v_excess",
               "u_peak", "theta_max", "over6"]
OVERLAY_FIELDS = ["dir", "t_ms", "theta_meas", "theta_sim", "V_meas", "u_meas", "V_sim"]


# ---------------------------------------------------------------- model
def simulate_step(direction, n=CAPTURE_SAMPLES):
    """1 ms sampled-data loop: D1[z] -> ZOH -> K1 / (s (tau s + 1)), exact between samples.

    Starts settled on the old level (-direction * REF_SAT) and steps the
    reference at sample 0, the same alignment as a capture. Returns the
    linear controller output u, with no stiction (the model has none).
    """
    ref = direction * REF_SAT
    theta, omega = -ref, 0.0
    u1 = e1 = 0.0
    decay = math.exp(-TS / TAU)
    out = {"t": [], "theta": [], "u": []}
    for k in range(n):
        e = ref - theta
        u = (D1_NUM[0] * e + D1_NUM[1] * e1 - D1_DEN[1] * u1) / D1_DEN[0]
        u1, e1 = u, e
        out["t"].append(k * TS)
        out["theta"].append(theta)
        out["u"].append(u)
        w_ss = K1 * u
        theta += w_ss * TS + (omega - w_ss) * TAU * (1.0 - decay)
        omega = w_ss + (omega - w_ss) * decay
    return out


# ---------------------------------------------------------------- loading
def new_capture(match, source):
    cid, ctrl, sign, dt_us, samples, over6 = match.groups()
    return {"source": source, "id": int(cid), "ctrl": ctrl, "dir": 1 if sign == "+" else -1,
            "dt_us": int(dt_us), "samples": int(samples), "over6": int(over6),
            "ref": None, "t": [], "theta": [], "v": [], "u": []}


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
            _, t_us, ref, v, u, _, theta = m.groups()
            cur["ref"] = float(ref)
            cur["t"].append(int(t_us) / 1e6)
            cur["v"].append(float(v))
            cur["u"].append(float(u))
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


def load_sim_csv(path):
    """Read the MATLAB export: dir,t_ms,theta,V -> {+1/-1: {t, theta, u}}."""
    sim = {}
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            d = 1 if row["dir"].strip() == "+" else -1
            trace = sim.setdefault(d, {"t": [], "theta": [], "u": []})
            trace["t"].append(float(row["t_ms"]) / 1000.0)
            trace["theta"].append(float(row["theta"]))
            trace["u"].append(float(row["V"]))
    return sim


# ---------------------------------------------------------------- metrics
def median_filter(values, window):
    half = window // 2
    return [st.median(values[max(0, i - half):i + half + 1]) for i in range(len(values))]


def settling_time(t, filtered, ref):
    """Time after which theta stays inside the band; None if it never does."""
    band = SETTLE_FRAC * STEP
    outside = [i for i, x in enumerate(filtered) if abs(x - ref) > band]
    if not outside:
        return t[0]
    last = outside[-1]
    return None if last == len(t) - 1 else t[last + 1]


def metrics_from_arrays(direction, t, theta, v, u=None):
    ref = direction * REF_SAT
    filtered = median_filter(theta, MEDIAN_WINDOW)
    beyond = max(direction * (x - ref) for x in filtered)
    n_ss = max(1, int(SS_WINDOW_MS / 1000.0 / TS))
    v_peak = max(abs(x) for x in v)
    return {
        "dir": direction,
        "os_pct": max(0.0, beyond) / STEP * 100.0,
        "ts_s": settling_time(t, filtered, ref),
        "ss_err": st.fmean(ref - x for x in theta[-n_ss:]),
        "v_peak": v_peak,
        "v_excess": max(0.0, v_peak - V_MAX),
        "u_peak": max(abs(x) for x in (u if u is not None else v)),
        "theta_max": max(abs(x) for x in theta),
    }


def step_metrics(cap):
    rec = metrics_from_arrays(cap["dir"], cap["t"], cap["theta"], cap["v"], cap["u"])
    rec.update(source=cap["source"], id=cap["id"], over6=cap["over6"])
    return rec


def spec_checks(records):
    """[(name, ok, detail)] against the hardware specs, judged on the worst step."""
    dirs = {r["dir"] for r in records}
    unsettled = [r for r in records if r["ts_s"] is None]
    ts_worst = max((r["ts_s"] for r in records if r["ts_s"] is not None), default=math.nan)
    os_worst = max(r["os_pct"] for r in records)
    ss_worst = max(abs(r["ss_err"]) for r in records)
    v_worst = max(r["v_peak"] for r in records)
    th_worst = max(r["theta_max"] for r in records)
    ts_detail = (f"{len(unsettled)} step(s) never settled" if unsettled
                 else f"worst {ts_worst * 1000:.0f} ms, limit {TS_MAX * 1000:.0f} ms")
    return [
        ("both directions captured", dirs == {1, -1},
         f"{sum(r['dir'] > 0 for r in records)} up, {sum(r['dir'] < 0 for r in records)} down"),
        ("stable: every step returned to the reference", not unsettled,
         f"{len(records) - len(unsettled)} of {len(records)} steps"),
        ("gear stayed inside pi/4", th_worst <= THETA_MAX,
         f"max |theta| = {th_worst:.4f} rad, limit {THETA_MAX:.4f}"),
        ("steady-state error is zero", ss_worst <= SS_TOL,
         f"worst |ref - theta| = {ss_worst:.4f} rad, tolerance {SS_TOL}"),
        ("2 % settling time", not unsettled and ts_worst <= TS_MAX, ts_detail),
        ("overshoot", os_worst <= OS_MAX_PCT, f"worst {os_worst:.2f} %, limit {OS_MAX_PCT:.0f} %"),
        ("motor voltage", v_worst <= V_MAX + V_ALLOW,
         f"worst |V| = {v_worst:.3f} V, limit {V_MAX:.0f} V + {V_ALLOW:.2f} V stiction allowance "
         f"(offsets {STICTION_POS} / {STICTION_NEG} V)"),
    ]


# ---------------------------------------------------------------- comparison
def overlay_rows(captures, sim):
    """Measured vs simulated, sample by sample, first capture of each direction."""
    rows = []
    for d in (1, -1):
        cap = next((c for c in captures if c["dir"] == d), None)
        trace = sim.get(d)
        if cap is None or trace is None:
            continue
        n = min(len(cap["theta"]), len(trace["theta"]))
        for i in range(n):
            rows.append({"dir": "+" if d > 0 else "-", "t_ms": i,
                         "theta_meas": cap["theta"][i], "theta_sim": trace["theta"][i],
                         "V_meas": cap["v"][i], "u_meas": cap["u"][i], "V_sim": trace["u"][i]})
    return rows


def compare(captures, sim):
    """Mean measured metrics per direction vs the simulation's."""
    lines = []
    for d in (1, -1):
        recs = [step_metrics(c) for c in captures if c["dir"] == d]
        if not recs or d not in sim:
            continue
        s = metrics_from_arrays(d, sim[d]["t"], sim[d]["theta"], sim[d]["u"])
        settled = [r["ts_s"] for r in recs if r["ts_s"] is not None]
        ts_meas = st.fmean(settled) * 1000 if settled else math.nan
        ts_sim = s["ts_s"] * 1000 if s["ts_s"] is not None else math.nan
        lines.append(f"  dir {'+' if d > 0 else '-'}: OS {st.fmean(r['os_pct'] for r in recs):.2f} % "
                     f"vs {s['os_pct']:.2f} %, settling {ts_meas:.0f} ms vs {ts_sim:.0f} ms, "
                     f"peak |u| {st.fmean(r['u_peak'] for r in recs):.2f} V vs {s['u_peak']:.2f} V")
    return lines


# ---------------------------------------------------------------- output
def fmt(value, digits=4):
    if value is None:
        return ""
    return f"{value:.{digits}f}" if isinstance(value, float) else str(value)


def write_csv(path, fields, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for row in rows:
            w.writerow({k: fmt(row.get(k), 6) for k in fields})


def print_steps(records):
    print(f"\n{'source':<14}{'id':>4}{'dir':>4}{'OS %':>8}{'Ts ms':>8}{'ss err':>9}{'|V| max':>9}{'|u| max':>9}")
    for r in records:
        ts = "never" if r["ts_s"] is None else f"{r['ts_s'] * 1000:.0f}"
        print(f"{r['source']:<14}{r['id']:>4}{'+' if r['dir'] > 0 else '-':>4}{r['os_pct']:>8.2f}"
              f"{ts:>8}{r['ss_err']:>9.4f}{r['v_peak']:>9.3f}{r['u_peak']:>9.3f}")


def main(argv):
    paths = [Path(a) for a in argv] or sorted(HERE.glob("*.txt"))
    captures = [c for p in paths for c in load_file(p)]
    if not captures:
        print("no complete captures found; pass files or put them next to this script")
        return 1

    records = [step_metrics(c) for c in captures]
    print_steps(records)
    write_csv(STEPS_CSV, STEP_FIELDS, records)

    checks = spec_checks(records)
    print(f"\nspecs over {len(records)} steps (worst case):")
    for name, ok, detail in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    if SIM_CSV.exists():
        sim, label = load_sim_csv(SIM_CSV), SIM_CSV.name
    else:
        sim, label = {d: simulate_step(d) for d in (1, -1)}, "built-in model (no sim_lab2b_steps.csv)"
    print(f"\nmeasured vs simulation ({label}), mean per direction:")
    for line in compare(captures, sim):
        print(line)
    write_csv(OVERLAY_CSV, OVERLAY_FIELDS, overlay_rows(captures, sim))
    print(f"\nwrote {STEPS_CSV.name} and {OVERLAY_CSV.name}")

    ok = all(c[1] for c in checks)
    print("\nall specs met" if ok else "\nsome specs NOT met")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
