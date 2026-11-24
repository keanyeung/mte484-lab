"""Lab 3 (d) - scores lab3d_outer_loop runs (the part (c) controller on the
hardware) against the part (c) specs and lines them up with the simulation.

Reuses analyze_lab3b for parsing and per-step metrics, so (b) and (d) are
measured the same way. Adds:
  - the anti-windup mode of each run, read from its '# start ... aw=on|off' line
  - a spec verdict over the complete steps of each run (manual Lab 3 (c)):
      2 % settling <= 7 s, overshoot < 45 %, zero steady-state error
      (|error| over the last second <= SS_TOL_MM, the 2 % band)
  - the spread of the metrics across steps (consistency over several cycles)
  - an overlay against the simulation of the SAME anti-windup mode

Simulation sources, next to this script:
  sim_lab3d.csv   3c design review/sim.py export: t_s, r, y_aw_off, y_aw_on,
                  y_aw_off_stick, y_aw_on_stick (linear ball and stick-slip ball)
  lab3_partc2_timeseries.csv (optional, copied from Matlab_file/Lab 3/csv_exports):
                  the groupmate's Simulink (no anti-windup); used for aw=off when present

Outputs, next to this script:
  lab3d_steps.csv     one row per step
  lab3d_overlay.csv   file, run, aw, t_s, r, y_hw, y_sim, y_sim_stick

Usage:
    py -3 analyze_lab3d.py                 # every "*.txt" next to this script
    py -3 analyze_lab3d.py d_run1.txt ...
"""
import bisect
import csv
import re
import statistics as st
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "3b outer loop readings"))
import analyze_lab3b as b  # noqa: E402

SIM_CSV = HERE / "sim_lab3d.csv"
SIMULINK_CSV = HERE / "lab3_partc2_timeseries.csv"
STEPS_CSV = HERE / "lab3d_steps.csv"
OVERLAY_CSV = HERE / "lab3d_overlay.csv"

TS_SPEC_S = 7.0
OS_SPEC_PCT = 45.0
SS_TOL_MM = b.SETTLE_BAND * 1000.0     # 3 mm: "zero" error = inside the 2 % band


def parse_runs(lines):
    """[(meta, samples)] with meta = {"kind": "square"|"hold", "aw": "on"|"off"|"?",
    "start": the start line}. Samples are analyze_lab3b tuples."""
    chunks, current = [], None
    for raw in lines:
        if raw.strip().startswith("# start"):
            current = [raw]
            chunks.append(current)
        elif current is not None:
            current.append(raw)
    runs = []
    for chunk in chunks:
        parsed = b.parse_stream(chunk)
        if not parsed:
            continue
        start = chunk[0].strip()
        m = re.search(r"\baw=(on|off)\b", start)
        kind = "hold" if start.startswith("# start hold") else "square"
        samples = parsed[0]
        dropped = 0
        if kind == "square":   # a serial glitch (e.g. r = 0.0100) would fake two edges
            good = [s for s in samples if is_square_level(s[1])]
            dropped, samples = len(samples) - len(good), good
        meta = {"kind": kind, "aw": m.group(1) if m else "?", "start": start,
                "dropped": dropped, "stopped": any(x.strip().startswith("# stop") for x in chunk)}
        runs.append((meta, samples))
    return runs


def is_square_level(r):
    return abs(r - b.Y_LOW) < 1e-6 or abs(r - b.Y_HIGH) < 1e-6


GAP_S = 0.05     # stream lines are 10 ms apart; a longer gap means the ISR paused


def stream_gaps(run):
    """[(t_before, t_after)] wherever consecutive samples are more than GAP_S apart."""
    return [(p[0], q[0]) for p, q in zip(run, run[1:]) if q[0] - p[0] > GAP_S]


def steps_hit_by_gaps(steps, gaps, t_end):
    """Indices of steps whose half period [t_edge, next edge) contains a gap."""
    hit = set()
    for i, s in enumerate(steps):
        end = steps[i + 1]["t_edge_s"] if i + 1 < len(steps) else t_end
        if any(s["t_edge_s"] <= g0 < end for g0, _ in gaps):
            hit.add(i)
    return hit


def find_edges_safe(run):
    """Reference edges of a parsed (already filtered) square run."""
    return b.find_edges(run)


def verdict(steps):
    full = [s for s in steps if s["complete"]]
    if not full:
        return {"n": 0}
    ts = [s["settling_s"] for s in full]
    os_ = [s["overshoot_pct"] for s in full]
    ss = [s["ss_error_mm"] for s in full]
    worst_ts = None if any(x is None for x in ts) else max(ts)
    return {
        "n": len(full),
        "worst_ts": worst_ts,
        "worst_os": max(os_),
        "worst_ss": max(abs(x) for x in ss),
        "ts_ok": worst_ts is not None and worst_ts <= TS_SPEC_S,
        "os_ok": max(os_) < OS_SPEC_PCT,
        "ss_ok": max(abs(x) for x in ss) <= SS_TOL_MM,
        "os_sd": st.pstdev(os_),
        "ss_sd": st.pstdev(ss),
        "ts_sd": st.pstdev(ts) if worst_ts is not None else None,
    }


def load_sim_lines(lines):
    """Columns of either sim file as lists. The groupmate's Simulink export is
    mapped to t_s, r, y_simulink (its discrete-controller column)."""
    rows = list(csv.DictReader(lines))
    if not rows:
        return {}
    if "Time_s" in rows[0]:
        return {"t_s": [float(r["Time_s"]) for r in rows],
                "r": [float(r["Reference_m"]) for r in rows],
                "y_simulink": [float(r["Y_Discrete_m"]) for r in rows]}
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def resample(t_src, y_src, t_dst):
    """Holds the last source value at each destination time."""
    return [y_src[max(0, bisect.bisect_right(t_src, t + 1e-9) - 1)] for t in t_dst]


def load_sim():
    sim = load_sim_lines(SIM_CSV.read_text().splitlines()) if SIM_CSV.exists() else {}
    if SIMULINK_CSV.exists():
        sl = load_sim_lines(SIMULINK_CSV.read_text().splitlines())
        if sim:
            sim["y_simulink"] = resample(sl["t_s"], sl["y_simulink"], sim["t_s"])
        else:
            sim = sl
    return sim


def sim_columns(sim, aw):
    """(main, stick-slip) columns matching a run's anti-windup mode; None if absent."""
    if aw == "on":
        main, stick = "y_aw_on", "y_aw_on_stick"
    else:
        main = "y_simulink" if "y_simulink" in sim else "y_aw_off"
        stick = "y_aw_off_stick"
    return (main if main in sim else None), (stick if stick in sim else None)


def overlay(run, sim, aw):
    """(t, r, y_hw, y_sim, y_sim_stick) with the sim's steady period aligned to each
    hardware up edge (analyze_lab3b.overlay)."""
    main, stick = sim_columns(sim, aw)
    if main is None:
        return []
    ys = sim[stick] if stick else [float("nan")] * len(sim["t_s"])
    tuples = list(zip(sim["t_s"], sim["r"], sim[main], ys))
    return b.overlay(run, b.sim_period(tuples))


def mark(ok):
    return "PASS" if ok else "FAIL"


def fmt_ts(v):
    return "never settles" if v is None else f"{v:.2f} s"


def print_verdict(v, indent="    "):
    if v["n"] == 0:
        print(f"{indent}no complete steps")
        return
    print(f"{indent}(c) specs over {v['n']} complete steps:")
    print(f"{indent}  settling <= {TS_SPEC_S:.0f} s   : worst {fmt_ts(v['worst_ts']):<14} {mark(v['ts_ok'])}")
    print(f"{indent}  overshoot < {OS_SPEC_PCT:.0f} %   : worst {v['worst_os']:5.1f} %        {mark(v['os_ok'])}")
    print(f"{indent}  |ss error| <= {SS_TOL_MM:.0f} mm : worst {v['worst_ss']:5.1f} mm       {mark(v['ss_ok'])}")
    sd = f"OS sd {v['os_sd']:.1f} %, error sd {v['ss_sd']:.1f} mm"
    if v["ts_sd"] is not None:
        sd += f", settling sd {v['ts_sd']:.2f} s"
    print(f"{indent}  consistency: {sd}")


def print_sim_baseline(sim):
    print("simulation baseline (same metrics):")
    seen = set()
    for aw in ("off", "on"):
        for col in sim_columns(sim, aw):
            if not col or col in seen:
                continue
            seen.add(col)
            srun = [(t, r, 0.0, 0.0, 0.0, 0.0, y) for t, r, y in zip(sim["t_s"], sim["r"], sim[col])]
            v = verdict(b.analyze_run(srun))
            print(f"  {col:<15} aw={aw:<3}  worst settling {fmt_ts(v['worst_ts']):<14}"
                  f"  worst OS {v['worst_os']:5.1f} %  worst |error| {v['worst_ss']:4.1f} mm")


def report_run(name, n, meta, run, step_rows, overlay_rows, sim):
    steps = b.analyze_run(run)
    if meta["kind"] == "hold" or not steps:
        h = b.hold_summary(run)
        print(f"{name} run {n}: {meta['kind'].upper()} aw={meta['aw']} r={h['r']:.3f}"
              f"  released y={h['start_y']:.3f}  rest y={h['y']:.4f}  e={h['e_mm']:+6.1f} mm"
              f"  theta={h['theta']:+.3f}  ({h['duration_s']:.0f} s, "
              f"{'stopped' if h['still'] else 'STILL MOVING'})")
        return
    print(f"{name} run {n}: SQUARE aw={meta['aw']}, {len(run)} samples, {len(steps)} steps")
    if meta["dropped"]:
        print(f"    note: {meta['dropped']} corrupted line(s) dropped (reference not 0.10 / 0.25)")
    if not meta["stopped"]:
        print("    note: no '# stop' line - the capture ended before s was pressed")
    gaps = stream_gaps(run)
    hit = steps_hit_by_gaps(steps, gaps, run[-1][0])
    for g0, g1 in gaps:
        print(f"    WARNING: stream gap {g0:.3f} -> {g1:.3f} s ({(g1 - g0) * 1000:.0f} ms, control ISR paused)")
    if hit:
        print(f"    {len(hit)} step(s) containing a gap left out of the verdict: "
              + ", ".join(f"{steps[i]['dir']} at {steps[i]['t_edge_s']:.2f} s" for i in sorted(hit)))
    kept = [s for i, s in enumerate(steps) if i not in hit]
    b.summarize(kept)
    print_verdict(verdict(kept))
    for i, s in enumerate(steps):
        step_rows.append([name, n, meta["aw"], b.fmt(s["t_edge_s"], 2), s["dir"], b.fmt(s["to_m"], 3),
                          b.fmt(s["settling_s"], 2), b.fmt(s["overshoot_pct"], 1),
                          b.fmt(s["ss_error_mm"], 1), b.fmt(s["peak_abs_V"], 2),
                          b.fmt(s["sat_pct"], 1), int(s["complete"]), int(i in hit)])
    for row in (overlay(run, sim, meta["aw"]) if sim else []):
        overlay_rows.append([name, n, meta["aw"]] + [f"{v:.4f}" for v in row])


def main(argv):
    files = [Path(x) for x in argv] or sorted(HERE.glob("*.txt"))
    if not files:
        print("no captures found")
        return 1
    sim = load_sim()
    if sim:
        print_sim_baseline(sim)
    else:
        print(f"note: no {SIM_CSV.name}; run 'py -3 sim.py export' in 3c design review")

    step_rows, overlay_rows = [], []
    for path in files:
        for n, (meta, run) in enumerate(parse_runs(path.read_text(errors="replace").splitlines()), 1):
            report_run(path.name, n, meta, run, step_rows, overlay_rows, sim)

    with open(STEPS_CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["file", "run", "aw", "t_edge_s", "dir", "to_m", "settling_s", "overshoot_pct",
                    "ss_error_mm", "peak_abs_V", "sat_pct", "complete", "gap_excluded"])
        w.writerows(step_rows)
    if overlay_rows:
        with open(OVERLAY_CSV, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["file", "run", "aw", "t_s", "r", "y_hw", "y_sim", "y_sim_stick"])
            w.writerows(overlay_rows)
    print(f"wrote {STEPS_CSV.name}" + (f", {OVERLAY_CSV.name}" if overlay_rows else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
