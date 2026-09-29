"""Lab 1 (f) - checks a lab1_inner_loop capture proves the saturator works.

Verifies, from the logged stream, that:
  1. the raw reference really exceeded the saturator limit (otherwise nothing
     was demonstrated),
  2. the saturated reference never exceeded it, in either direction,
  3. clipping happened in BOTH directions, as the brief requires twice,
  4. the gear angle stayed inside the +-pi/4 rad damage limit,
  5. the loop tracked the clipped reference, not the raw one.

Usage (from any folder):
    python check_saturator.py                 # every "*.txt" next to this script
    python check_saturator.py file1.txt ...   # specific captures
"""
import math
import re
import statistics as st
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

REF_SAT = 0.7                 # saturator limit the sketch applies [rad]
THETA_MAX = math.pi / 4       # damage limit from the lab manual [rad]
SAT_TOL = 0.005               # allowance for rounding in the printed values [rad]
TRACK_TOL = 0.02              # allowed steady-state gap between theta and ref_sat [rad]
SETTLE_FRAC = 0.5             # ignore the first half of each level while it settles

ROW_RE = re.compile(r"^(\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)$")


def load(path):
    """Return the sample rows of one CoolTerm capture."""
    rows = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            m = ROW_RE.match(line.strip())
            if m:
                t, raw, sat, theta, v = m.groups()
                rows.append({"t": int(t) / 1000.0, "raw": float(raw), "sat": float(sat),
                             "theta": float(theta), "v": float(v)})
    return rows


def levels(rows):
    """Split the capture into runs of constant saturated reference."""
    out, start = [], 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or rows[i]["sat"] != rows[start]["sat"]:
            out.append(rows[start:i])
            start = i
    return out


def tracking_errors(rows):
    """Steady-state |theta - ref_sat| for every level held long enough to settle."""
    errors = []
    for level in levels(rows):
        if len(level) < 10:
            continue
        tail = level[int(len(level) * SETTLE_FRAC):]
        errors.append(abs(st.fmean(r["theta"] for r in tail) - tail[0]["sat"]))
    return errors


def check(path):
    rows = load(path)
    if not rows:
        print(f"{path.name}: no sample rows found")
        return False

    raw_hi, raw_lo = max(r["raw"] for r in rows), min(r["raw"] for r in rows)
    sat_hi, sat_lo = max(r["sat"] for r in rows), min(r["sat"] for r in rows)
    theta_max = max(abs(r["theta"]) for r in rows)
    v_max = max(abs(r["v"]) for r in rows)
    clipped_hi = any(r["raw"] > REF_SAT + SAT_TOL for r in rows)
    clipped_lo = any(r["raw"] < -REF_SAT - SAT_TOL for r in rows)
    errors = tracking_errors(rows)
    worst_error = max(errors) if errors else math.nan

    results = [
        ("raw reference exceeds the limit", clipped_hi or clipped_lo,
         f"raw spans {raw_lo:+.3f} to {raw_hi:+.3f} rad"),
        ("saturated reference stays inside the limit", max(abs(sat_hi), abs(sat_lo)) <= REF_SAT + SAT_TOL,
         f"sat spans {sat_lo:+.3f} to {sat_hi:+.3f} rad, limit +-{REF_SAT}"),
        ("clipping seen in both directions", clipped_hi and clipped_lo,
         f"positive: {clipped_hi}, negative: {clipped_lo}"),
        ("gear stayed inside pi/4", theta_max <= THETA_MAX,
         f"max |theta| = {theta_max:.4f} rad, limit {THETA_MAX:.4f}"),
        ("loop tracked the clipped reference", bool(errors) and worst_error <= TRACK_TOL,
         f"worst steady-state error = {worst_error:.4f} rad over {len(errors)} levels"),
    ]

    print(f"\n{path.name}: {len(rows)} samples, {rows[-1]['t'] - rows[0]['t']:.1f} s, max |V| = {v_max:.2f} V")
    for name, ok, detail in results:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    return all(ok for _, ok, _ in results)


def main(argv):
    paths = [Path(a) for a in argv] or sorted(HERE.glob("*.txt"))
    if not paths:
        print("no captures found; pass files or put them next to this script")
        return 1
    # Check every file before reporting; all() on a generator would stop at the first failure
    results = [check(p) for p in paths]
    ok = all(results)
    print(f"\n{sum(results)} of {len(results)} captures passed" if not ok else "\nall checks passed")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
