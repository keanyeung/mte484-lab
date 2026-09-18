"""Lab 1 (d) - stiction analysis for the lab1_stiction_test captures.

For every ramp run, finds the voltage at which the gear actually starts to
move (not the later on-board stop), then recommends the stiction offset for
each direction as  mean + 2 * standard deviation  of those breakaway voltages.

Usage (from any folder):
    python analyze_stiction.py

Outputs (written next to this script):
    stiction_runs.csv    one row per run
    example_run35.csv    theta vs V for one run, for the report figure
"""
import csv
import re
import statistics as st
from pathlib import Path

HERE = Path(__file__).resolve().parent

# (capture file, gear position in rad)
FILES = [
    ("run1 zero.txt", 0.0), ("run2 zero.txt", 0.0), ("run3 zero.txt", 0.0),
    ("run1 pos05.txt", 0.5), ("run2 pos05.txt", 0.5), ("run3 pos05.txt", 0.5),
    ("run1 posneg05.txt", -0.5), ("run2 posneg05.txt", -0.5), ("run3 posneg05.txt", -0.5),
]

IDLE, BASELINE, RAMP = 0, 1, 2
PRE_IDLE_SAMPLES = 25      # idle samples before a run used for the rest angle (0.5 s)
MEDIAN_WINDOW = 5          # centred median filter length, removes single-sample spikes
THRESHOLDS = (3.0, 5.0)    # onset thresholds as multiples of the rest noise sigma
PRIMARY_K = 3.0            # threshold used for the recommendation
SIGMA_FLOOR = 0.0007       # rad (~2 ADC counts), stops sigma collapsing to zero
EXPECTED_RUNS = range(1, 55)
EXAMPLE_RUN = 35

SAMPLE_RE = re.compile(r"^\s*(\d+),(\d+),(\d+),(-?\d+\.\d+),(\d+),(-?\d+\.\d+)\s*$")
SUMMARY_RE = re.compile(r"^#\s*run\s+(\d+)\s+stop\s+(\w+)\s+V=(-?\d+\.\d+)")


# ---------------------------------------------------------------- loading
def load_file(path):
    """Return (samples, summaries) from one CoolTerm capture."""
    samples, summaries = [], {}
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            m = SAMPLE_RE.match(line)
            if m:
                t, run, state, v, motor, theta = m.groups()
                samples.append({"t": int(t), "run": int(run), "state": int(state),
                                "v": float(v), "motor": int(motor), "theta": float(theta)})
                continue
            m = SUMMARY_RE.match(line.strip())
            if m:
                summaries[int(m.group(1))] = (m.group(2), float(m.group(3)))
    return samples, summaries


# ---------------------------------------------------------------- signal helpers
def rest_stats(thetas):
    """Rest angle (median) and robust noise sigma (1.4826 * MAD)."""
    theta0 = st.median(thetas)
    mad = st.median(abs(x - theta0) for x in thetas)
    return theta0, max(1.4826 * mad, SIGMA_FLOOR)


def median_filter(values, window):
    """Centred moving median; the window shrinks at the ends."""
    half = window // 2
    return [st.median(values[max(0, i - half): i + half + 1]) for i in range(len(values))]


def find_onset(ramp, filtered, theta0, sign, threshold):
    """Voltage in effect when theta leaves the rest band for good.

    sign = +1 for +V (theta decreases), -1 for -V (theta increases).
    Onset = first sample after which the filtered deviation, measured in the
    expected direction, stays above the threshold until the run stops.
    Each theta is read before that line's voltage is applied, so the voltage
    that caused the motion is the previous line's V.
    """
    last_below = -1
    for i, th in enumerate(filtered):
        if sign * (theta0 - th) <= threshold:
            last_below = i
    onset = last_below + 1
    if onset >= len(ramp):
        return None, None
    v_applied = ramp[onset - 1]["v"] if onset > 0 else 0.0
    return abs(v_applied), onset


# ---------------------------------------------------------------- per-run analysis
def analyse_run(run, ramp, rest, summary, position, fname):
    theta0, sigma = rest_stats([s["theta"] for s in rest])
    sign = 1 if max(ramp, key=lambda s: abs(s["v"]))["v"] > 0 else -1
    filtered = median_filter([s["theta"] for s in ramp], MEDIAN_WINDOW)
    reason, stop_v = summary if summary else ("MISSING", None)
    rec = {"run": run, "file": fname, "position_rad": position,
           "direction": "CW" if sign > 0 else "CCW", "command": "+" if sign > 0 else "-",
           "theta0_rad": theta0, "sigma_rad": sigma,
           "stop_V": abs(stop_v) if stop_v is not None else None, "stop_reason": reason,
           "_ramp": ramp, "_filtered": filtered}
    for k in THRESHOLDS:
        v, idx = find_onset(ramp, filtered, theta0, sign, k * sigma)
        rec[f"onset_V_k{k:g}"] = v
        if k == PRIMARY_K:
            rec["_onset_idx"] = idx
    primary = rec[f"onset_V_k{PRIMARY_K:g}"]
    rec["lag_V"] = (rec["stop_V"] - primary) if (primary is not None and rec["stop_V"] is not None) else None
    return rec


def extract_runs(samples, summaries, position, fname):
    """Split one capture into ramp runs (runs with no ramp, e.g. leftover idle, are skipped)."""
    records = []
    for run in sorted({s["run"] for s in samples}):
        idx = [i for i, s in enumerate(samples) if s["run"] == run]
        ramp = [samples[i] for i in idx if samples[i]["state"] == RAMP]
        if not ramp:
            continue
        first = idx[0]
        pre = [s for s in samples[max(0, first - PRE_IDLE_SAMPLES):first] if s["state"] == IDLE]
        base = [samples[i] for i in idx if samples[i]["state"] == BASELINE]
        records.append(analyse_run(run, ramp, pre + base, summaries.get(run), position, fname))
    return records


# ---------------------------------------------------------------- statistics
def describe(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return None
    sd = st.stdev(vals) if len(vals) > 1 else 0.0
    return {"n": len(vals), "mean": st.mean(vals), "sd": sd,
            "min": min(vals), "max": max(vals), "mean_2sd": st.mean(vals) + 2 * sd}


def print_table(records, key, title):
    print(f"\n{title}")
    print(f"{'direction':<9} {'position':>8} {'n':>3} {'mean':>7} {'sd':>7} {'min':>7} {'max':>7} {'mean+2sd':>9}")
    for direction in ("CW", "CCW"):
        for pos in (-0.5, 0.0, 0.5, None):
            subset = [r[key] for r in records if r["direction"] == direction
                      and (pos is None or r["position_rad"] == pos)]
            d = describe(subset)
            if d:
                label = "all" if pos is None else f"{pos:+.1f}"
                print(f"{direction:<9} {label:>8} {d['n']:>3} {d['mean']:>7.3f} {d['sd']:>7.3f} "
                      f"{d['min']:>7.3f} {d['max']:>7.3f} {d['mean_2sd']:>9.3f}")


# ---------------------------------------------------------------- checks
def run_checks(records, runs_per_file):
    print("Runs found per file:")
    for fname, runs in runs_per_file:
        print(f"  {fname:<20} runs {runs}")
    found = sorted(r["run"] for r in records)
    missing = [r for r in EXPECTED_RUNS if r not in found]
    dupes = sorted({r for r in found if found.count(r) > 1})
    n_cw = sum(r["direction"] == "CW" for r in records)
    not_motion = [r["run"] for r in records if r["stop_reason"] != "MOTION"]
    no_onset = [r["run"] for r in records if r[f"onset_V_k{PRIMARY_K:g}"] is None]
    bad_order = [r["run"] for r in records if r["lag_V"] is not None and r["lag_V"] < 0]
    print(f"\nChecks: {len(records)} runs ({n_cw} CW, {len(records) - n_cw} CCW)")
    print(f"  missing runs: {missing or 'none'}   duplicate runs: {dupes or 'none'}")
    print(f"  stop reason not MOTION: {not_motion or 'none'}")
    print(f"  no onset found: {no_onset or 'none'}   onset above stop V: {bad_order or 'none'}")


def print_sensitivity(records):
    k_lo, k_hi = (f"onset_V_k{k:g}" for k in THRESHOLDS)
    diffs = [r[k_hi] - r[k_lo] for r in records if r[k_lo] is not None and r[k_hi] is not None]
    big = [r["run"] for r in records if r[k_lo] is not None and r[k_hi] is not None
           and r[k_hi] - r[k_lo] > 0.010]
    print(f"\nThreshold sensitivity ({THRESHOLDS[1]:g} sigma minus {THRESHOLDS[0]:g} sigma): "
          f"mean {st.mean(diffs):.4f} V, max {max(diffs):.4f} V")
    print(f"  runs shifting more than 0.010 V: {big or 'none'}")


def print_recommendation(records):
    key = f"onset_V_k{PRIMARY_K:g}"
    print(f"\nRecommended offsets (mean + 2 sd of breakaway voltage, {PRIMARY_K:g} sigma onset):")
    for direction, const, sign in (("CW", "STICTION_POS", "+V"), ("CCW", "STICTION_NEG", "-V")):
        vals = [r[key] for r in records if r["direction"] == direction and r[key] is not None]
        d = describe(vals)
        covered = sum(v <= d["mean_2sd"] for v in vals)
        print(f"  {const} ({direction}, {sign}) = {d['mean_2sd']:.3f} V   "
              f"(mean {d['mean']:.3f}, sd {d['sd']:.3f}, covers {covered}/{len(vals)} runs)")


# ---------------------------------------------------------------- outputs
CSV_FIELDS = ["run", "file", "position_rad", "direction", "command", "theta0_rad", "sigma_rad",
              "onset_V_k3", "onset_V_k5", "stop_V", "stop_reason", "lag_V"]


def fmt(value):
    return f"{value:.4f}" if isinstance(value, float) else ("" if value is None else value)


def write_runs_csv(records, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_FIELDS)
        for r in sorted(records, key=lambda r: r["run"]):
            w.writerow([fmt(r[k]) for k in CSV_FIELDS])


def write_example_csv(rec, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "V", "theta_rad", "theta_filtered_rad", "onset_flag"])
        for i, (s, th_f) in enumerate(zip(rec["_ramp"], rec["_filtered"])):
            w.writerow([s["t"], f"{s['v']:.3f}", f"{s['theta']:.4f}", f"{th_f:.4f}",
                        1 if i == rec["_onset_idx"] else 0])


# ---------------------------------------------------------------- main
def main():
    records, runs_per_file = [], []
    for fname, position in FILES:
        path = HERE / fname
        if not path.exists():
            print(f"MISSING FILE: {path}")
            continue
        samples, summaries = load_file(path)
        recs = extract_runs(samples, summaries, position, fname)
        runs_per_file.append((fname, [r["run"] for r in recs]))
        records.extend(recs)

    run_checks(records, runs_per_file)
    print_table(records, "stop_V", "On-board stop voltage (|V|, from the # summary lines):")
    print_table(records, f"onset_V_k{PRIMARY_K:g}",
                f"Breakaway (onset) voltage, {PRIMARY_K:g} sigma threshold (|V|):")
    print_sensitivity(records)
    lags = [r["lag_V"] for r in records if r["lag_V"] is not None]
    print(f"\nDetection lag (stop V - onset V): mean {st.mean(lags):.3f} V, max {max(lags):.3f} V")
    print("\nSpot checks:")
    for run in (28, 35):
        r = next((x for x in records if x["run"] == run), None)
        if r:
            print(f"  run {run}: onset {r['onset_V_k3']:.3f} V, stop {r['stop_V']:.3f} V")
    print_recommendation(records)

    write_runs_csv(records, HERE / "stiction_runs.csv")
    example = next((r for r in records if r["run"] == EXAMPLE_RUN), None)
    if example:
        write_example_csv(example, HERE / f"example_run{EXAMPLE_RUN}.csv")
    print(f"\nWrote stiction_runs.csv and example_run{EXAMPLE_RUN}.csv to {HERE}")


if __name__ == "__main__":
    main()
