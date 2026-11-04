"""Lab 2 (d) - stiction offsets with the beam attached (ball off).

Runs the Lab 1 (d) analysis unchanged on the lab2_stiction_test captures: the
breakaway voltage of each ramp run is found by the same median-filtered,
3-sigma onset detection, and the offset for each direction is the same
mean + 2 * standard deviation rule. The functions are imported from
../1d stiction test readings/analyze_stiction.py rather than copied, so the
method cannot drift between the two labs.

It then compares against Lab 1 two ways: the offsets in use since Lab 1
(STICTION_POS = 0.277 V CW, STICTION_NEG = 0.259 V CCW), and the Lab 1
breakaway data itself (mean +- sd per direction), which answers the manual's
question of whether stiction changed in one direction or both.

Captures expected next to this script (3 positions, + and - runs in each):
    stiction_zero.txt   gear near 0 rad
    stiction_pos05.txt  gear near +0.5 rad
    stiction_neg05.txt  gear near -0.5 rad

Usage (from any folder):
    py -3 analyze_stiction_beam.py

Output, next to this script:
    stiction_beam_runs.csv   one row per run, same columns as Lab 1's stiction_runs.csv
"""
import importlib.util
import statistics as st
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
LAB1_DIR = HERE.parent / "1d stiction test readings"


def _load_lab1():
    spec = importlib.util.spec_from_file_location("analyze_stiction", LAB1_DIR / "analyze_stiction.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


LAB1 = _load_lab1()

FILES = [("stiction_zero.txt", 0.0), ("stiction_pos05.txt", 0.5), ("stiction_neg05.txt", -0.5)]
ONSET_KEY = f"onset_V_k{LAB1.PRIMARY_K:g}"
RUNS_CSV = "stiction_beam_runs.csv"

# Offsets in lab2_inner_loop until now, from Lab 1 (d)
LAB1_OFFSETS = {"CW": ("STICTION_POS", "+V", 0.277), "CCW": ("STICTION_NEG", "-V", 0.259)}


# ---------------------------------------------------------------- loading
RESET_MARK = "# geeWhiz Started"   # the sketch's banner: run numbers restart after it


def load_sessions(path):
    """Like Lab 1's load_file, but split at every board reset into (samples, summaries)
    per session, so run numbers that restart from 1 are never merged into one run."""
    sessions, samples, summaries = [], [], {}
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith(RESET_MARK) and (samples or summaries):
                sessions.append((samples, summaries))
                samples, summaries = [], {}
                continue
            m = LAB1.SAMPLE_RE.match(line)
            if m:
                t, run, state, v, motor, theta = m.groups()
                samples.append({"t": int(t), "run": int(run), "state": int(state),
                                "v": float(v), "motor": int(motor), "theta": float(theta)})
                continue
            m = LAB1.SUMMARY_RE.match(line.strip())
            if m:
                summaries[int(m.group(1))] = (m.group(2), float(m.group(3)))
    if samples or summaries:
        sessions.append((samples, summaries))
    return sessions


def load_records(folder=HERE, files=FILES):
    """Analyse every capture present; returns (records, names of missing files)."""
    records, missing = [], []
    for name, position in files:
        path = Path(folder) / name
        if not path.exists():
            missing.append(name)
            continue
        for samples, summaries in load_sessions(path):
            records.extend(LAB1.extract_runs(samples, summaries, position, name))
    return records, missing


def lab1_records():
    """The Lab 1 (d) runs, for the before/after comparison; [] if the folder is absent."""
    records, _ = load_records(LAB1_DIR, LAB1.FILES)
    return records


# ---------------------------------------------------------------- results
def recommend(records):
    """{direction: describe(onset voltages)} with the mean + 2 sd offset in 'mean_2sd'."""
    return {d: LAB1.describe([r[ONSET_KEY] for r in records if r["direction"] == d])
            for d in ("CW", "CCW")}


def compare_with_lab1(rec):
    rows = []
    for direction, (const, sign, old) in LAB1_OFFSETS.items():
        new = rec[direction]["mean_2sd"] if rec.get(direction) else None
        rows.append({"direction": direction, "constant": const, "sign": sign, "lab1_V": old,
                     "beam_V": new, "change_V": None if new is None else new - old})
    return rows


def print_checks(records, missing):
    print("Runs found per file:")
    for name, _ in FILES:
        runs = [r["run"] for r in records if r["file"] == name]
        n_cw = sum(r["direction"] == "CW" for r in records if r["file"] == name)
        print(f"  {name:<20} {len(runs):>2} runs ({n_cw} CW, {len(runs) - n_cw} CCW)")
    for name in missing:
        print(f"  MISSING FILE: {name}")
    not_motion = [(r["file"], r["run"], r["stop_reason"]) for r in records if r["stop_reason"] != "MOTION"]
    no_onset = [(r["file"], r["run"]) for r in records if r[ONSET_KEY] is None]
    print(f"  stop reason not MOTION: {not_motion or 'none'}")
    print(f"  no onset found: {no_onset or 'none'}")


def print_before_after(new_records, old_records):
    print("\nBreakaway voltage, Lab 1 (beam off) vs now (beam on, ball off), all positions:")
    print(f"{'direction':<9} {'Lab 1 mean +- sd':>18} {'n':>3}   {'beam mean +- sd':>17} {'n':>3}   {'change':>8}")
    for direction in ("CW", "CCW"):
        old = LAB1.describe([r[ONSET_KEY] for r in old_records if r["direction"] == direction])
        new = LAB1.describe([r[ONSET_KEY] for r in new_records if r["direction"] == direction])
        if not new:
            continue
        old_txt = f"{old['mean']:.3f} +- {old['sd']:.3f}" if old else "n/a"
        change = f"{new['mean'] - old['mean']:+.3f}" if old else "n/a"
        print(f"{direction:<9} {old_txt:>18} {old['n'] if old else 0:>3}   "
              f"{new['mean']:.3f} +- {new['sd']:.3f}  {new['n']:>3}   {change:>8} V")


def print_recommendation(rec):
    print(f"\nRecommended offsets (mean + 2 sd of breakaway voltage, {LAB1.PRIMARY_K:g} sigma onset):")
    for row in compare_with_lab1(rec):
        d = rec[row["direction"]]
        if d is None:
            print(f"  {row['constant']} ({row['direction']}): no runs")
            continue
        print(f"  {row['constant']} ({row['direction']}, {row['sign']}) = {row['beam_V']:.3f} V   "
              f"(Lab 1 {row['lab1_V']:.3f} V, change {row['change_V']:+.3f} V; "
              f"mean {d['mean']:.3f}, sd {d['sd']:.3f}, n {d['n']})")


# ---------------------------------------------------------------- main
def main(folder=HERE):
    folder = Path(folder)
    records, missing = load_records(folder)
    if not records:
        print(f"no ramp runs found in {folder}; expected {', '.join(n for n, _ in FILES)}")
        return 1

    print_checks(records, missing)
    LAB1.print_table(records, ONSET_KEY,
                     f"Breakaway (onset) voltage with the beam, {LAB1.PRIMARY_K:g} sigma threshold (|V|):")
    print_before_after(records, lab1_records())
    rec = recommend(records)
    print_recommendation(rec)

    LAB1.write_runs_csv(records, folder / RUNS_CSV)
    print(f"\nWrote {RUNS_CSV} to {folder}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
