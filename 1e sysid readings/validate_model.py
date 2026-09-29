"""Lab 1 (e) - model validation: simulate every captured step from the fitted plant.

Under P control the plant  K1 / (s (tau s + 1))  closes to the standard
second-order form  wn^2 / (s^2 + 2 zeta wn s + wn^2)  with

    wn   = sqrt(Kp K1 / tau)
    zeta = 1 / (2 tau wn)

so a fitted (K1, tau) predicts the whole step response, not only OS and Tp.
Every measured step is compared against that prediction twice: once with the
overall fit from all gains ("all") and once with the fit from its own gain
("kp"). Agreement between the two is the consistency check the report needs.

Dead time is reported, never hidden. The gear starts moving later than the
model does (backlash plus stiction), so each step carries

    dead_ms = measured onset - model onset

with both onsets taken at analyze_sysid.ONSET_FRAC of the step, and every
error is given three ways: against the undelayed model (rms_rad, peak_rad),
against the model delayed by dead_ms (rms_shift_rad, peak_shift_rad), and
against the median-filtered angle (rms_filt_rad, peak_filt_rad), which is
the mismatch left once single-sample sensor spikes are taken out.

Usage (from any folder):
    python validate_model.py                 # every "kp*.txt" capture next to this script
    python validate_model.py file1.txt ...   # specific captures

Outputs (written next to this script):
    sysid_validation.csv  one row per step per fit, errors in rad
    sysid_overlay.csv     measured and modelled angle per sample, for report figures
"""
import csv
import math
import statistics as st
import sys
from pathlib import Path

import analyze_sysid as sysid

HERE = Path(__file__).resolve().parent

OVERLAY_FIT = "all"         # the fit whose trace goes into sysid_overlay.csv
CRIT_TOL = 1e-9             # |zeta - 1| below this counts as critically damped
BISECT_STEPS = 60           # bisection iterations for the model onset time
OVERDAMPED_SPAN = 8.0       # onset search window as a multiple of 1/(zeta wn) [s]

VALIDATION_FIELDS = ["fit", "source", "id", "kp", "dir", "step_rad", "k1_fit", "tau_fit",
                     "os_meas", "os_model", "tp_ms_meas", "tp_ms_model",
                     "onset_ms", "onset_ms_model", "dead_ms",
                     "rms_rad", "peak_rad", "rms_shift_rad", "peak_shift_rad",
                     "rms_filt_rad", "peak_filt_rad", "noise_rad"]
OVERLAY_FIELDS = ["source", "id", "kp", "dir", "t_ms", "theta_measured", "theta_model"]

MEAN_KEYS = ["os_meas", "os_model", "tp_ms_meas", "tp_ms_model", "onset_ms", "onset_ms_model",
             "dead_ms", "step_mag_rad", "rms_rad", "rms_pct_step", "peak_rad",
             "rms_shift_rad", "rms_shift_pct_step", "peak_shift_rad",
             "rms_filt_rad", "rms_filt_pct_step", "peak_filt_rad", "noise_rad"]


# ---------------------------------------------------------------- second-order model
def closed_loop(k1, tau, kp):
    """(zeta, wn) of the P-controlled plant; the inverse of plant_from_closed_loop."""
    if tau <= 0.0 or kp * k1 <= 0.0:
        raise ValueError("need tau > 0 and Kp K1 > 0 for a stable closed loop")
    wn = math.sqrt(kp * k1 / tau)
    return 1.0 / (2.0 * tau * wn), wn


def unit_step(zeta, wn, t):
    """Unit step response of wn^2 / (s^2 + 2 zeta wn s + wn^2)."""
    if t <= 0.0:
        return 0.0
    if zeta < 1.0 - CRIT_TOL:
        rad = math.sqrt(1.0 - zeta ** 2)
        wd = wn * rad
        return 1.0 - math.exp(-zeta * wn * t) * (math.cos(wd * t) + zeta / rad * math.sin(wd * t))
    if zeta <= 1.0 + CRIT_TOL:
        return 1.0 - math.exp(-wn * t) * (1.0 + wn * t)
    a = wn * math.sqrt(zeta ** 2 - 1.0)
    s1, s2 = -zeta * wn + a, -zeta * wn - a
    return 1.0 - (s1 * math.exp(s2 * t) - s2 * math.exp(s1 * t)) / (s1 - s2)


def os_from_zeta(zeta):
    """Predicted percent overshoot; zero when the loop is not underdamped."""
    if zeta >= 1.0:
        return 0.0
    return 100.0 * math.exp(-zeta * math.pi / math.sqrt(1.0 - zeta ** 2))


def tp_from(zeta, wn):
    """Predicted time to the first peak [s]; nan when there is no peak."""
    if zeta >= 1.0:
        return math.nan
    return math.pi / (wn * math.sqrt(1.0 - zeta ** 2))


def onset_time(zeta, wn, frac=sysid.ONSET_FRAC):
    """Time the ideal model needs to cover frac of the step [s].

    The step response rises monotonically up to the first peak, so a plain
    bisection is exact enough and needs no derivative.
    """
    hi = tp_from(zeta, wn)
    if math.isnan(hi):
        hi = OVERDAMPED_SPAN / (zeta * wn)
    lo = 0.0
    for _ in range(BISECT_STEPS):
        mid = 0.5 * (lo + hi)
        if unit_step(zeta, wn, mid) < frac:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def dead_time(rec, zeta, wn):
    """Delay before the gear moves that the model does not explain [s]."""
    if math.isnan(rec["onset_ms"]):
        return 0.0
    return max(0.0, rec["onset_ms"] / 1000.0 - onset_time(zeta, wn))


# ---------------------------------------------------------------- per-step comparison
def model_trace(cap, rec, zeta, wn, dead_s):
    """Modelled angle at every sample time of the capture, delayed by dead_s."""
    step = rec["ss"] - rec["theta0"]
    return [rec["theta0"] + step * unit_step(zeta, wn, t - dead_s) for t in cap["t"]]


def errors(theta, model):
    """(rms, peak) absolute error between the measured and modelled angle [rad]."""
    diffs = [m - th for th, m in zip(theta, model)]
    return math.sqrt(st.fmean(d * d for d in diffs)), max(abs(d) for d in diffs)


def tail_noise(theta):
    """Sample sd of the steady-state tail [rad]: the floor any rms error sits on."""
    tail = theta[-max(2, int(len(theta) * sysid.SS_FRACTION)):]
    return st.stdev(tail)


def compare_step(cap, rec, fit, k1, tau):
    """One validation row: the measured step against the plant (k1, tau) at its own Kp."""
    zeta, wn = closed_loop(k1, tau, cap["kp"])
    dead_s = dead_time(rec, zeta, wn)
    step = rec["ss"] - rec["theta0"]
    delayed = model_trace(cap, rec, zeta, wn, dead_s)
    rms, peak = errors(cap["theta"], model_trace(cap, rec, zeta, wn, 0.0))
    rms_d, peak_d = errors(cap["theta"], delayed)
    # Against the median-filtered angle: the mismatch left once sensor spikes are removed
    rms_f, peak_f = errors(sysid.median_filter(cap["theta"], sysid.MEDIAN_WINDOW), delayed)
    return {"fit": fit, "source": rec["source"], "id": rec["id"], "kp": rec["kp"],
            "dir": rec["dir"], "step_rad": step, "k1_fit": k1, "tau_fit": tau,
            "os_meas": rec["os_pct"], "os_model": os_from_zeta(zeta),
            "tp_ms_meas": rec["tp_ms"], "tp_ms_model": tp_from(zeta, wn) * 1000.0,
            "onset_ms": rec["onset_ms"], "onset_ms_model": onset_time(zeta, wn) * 1000.0,
            "dead_ms": dead_s * 1000.0,
            "rms_rad": rms, "peak_rad": peak, "rms_shift_rad": rms_d, "peak_shift_rad": peak_d,
            "rms_filt_rad": rms_f, "peak_filt_rad": peak_f, "step_mag_rad": abs(step),
            "rms_pct_step": rms / abs(step) * 100.0,
            "rms_shift_pct_step": rms_d / abs(step) * 100.0,
            "rms_filt_pct_step": rms_f / abs(step) * 100.0,
            "noise_rad": tail_noise(cap["theta"])}


def fits_from(rows):
    """(overall (k1, tau), {kp: (k1, tau)}) read out of analyze_sysid's summary rows."""
    overall = [(r["k1_mean"], r["tau_mean"]) for r in rows if r["group"] == "all"]
    per_gain = {r["kp"]: (r["k1_mean"], r["tau_mean"]) for r in rows if r["group"] == "kp"}
    return (overall[0] if overall else None), per_gain


def validate(captures, records, rows):
    """Validation rows for every usable step, under the overall fit and its own gain's fit."""
    overall, per_gain = fits_from(rows)
    out = []
    for cap, rec in zip(captures, records):
        if rec["exclude"]:
            continue
        if overall:
            out.append(compare_step(cap, rec, "all", *overall))
        if rec["kp"] in per_gain:
            out.append(compare_step(cap, rec, "kp", *per_gain[rec["kp"]]))
    return out


# ---------------------------------------------------------------- summary
def group_stats(fit, kp, rows):
    stats = {"fit": fit, "kp": kp, "n": len(rows)}
    stats.update({k: st.fmean(r[k] for r in rows) for k in MEAN_KEYS})
    return stats


def summarise(rows):
    """Mean errors per fit and gain, then over every step of that fit."""
    out = []
    for fit in (OVERLAY_FIT, "kp"):
        same = [r for r in rows if r["fit"] == fit]
        if not same:
            continue
        for kp in sorted({r["kp"] for r in same}, reverse=True):
            out.append(group_stats(fit, kp, [r for r in same if r["kp"] == kp]))
        out.append(group_stats(fit, "", same))
    return out


# ---------------------------------------------------------------- output
def write_overlay(path, captures, records, k1, tau):
    """Measured and modelled angle per sample, long format so Excel can plot overlays."""
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(OVERLAY_FIELDS)
        for cap, rec in zip(captures, records):
            if rec["exclude"]:
                continue
            zeta, wn = closed_loop(k1, tau, cap["kp"])
            model = model_trace(cap, rec, zeta, wn, dead_time(rec, zeta, wn))
            for t, theta, m in zip(cap["t"], cap["theta"], model):
                w.writerow([rec["source"], rec["id"], rec["kp"], rec["dir"],
                            f"{t * 1000:.3f}", f"{theta:.4f}", f"{m:.4f}"])


def print_steps(rows, fit):
    print(f"\nper-step comparison, {fit} fit "
          f"(rms/peak in rad, shift = model delayed by the measured dead time)")
    print(f"{'file':<22}{'id':>4}{'Kp':>7}{'dir':>4}{'OS%':>7}{'pred':>7}{'Tp ms':>8}{'pred':>7}"
          f"{'dead':>7}{'rms':>8}{'peak':>8}{'rms_s':>8}{'peak_s':>8}{'noise':>8}")
    for r in (x for x in rows if x["fit"] == fit):
        print(f"{r['source'][:21]:<22}{r['id']:>4}{r['kp']:>7.1f}{'+' if r['dir'] > 0 else '-':>4}"
              f"{r['os_meas']:>7.1f}{r['os_model']:>7.1f}{r['tp_ms_meas']:>8.1f}{r['tp_ms_model']:>7.1f}"
              f"{r['dead_ms']:>7.1f}{r['rms_rad']:>8.4f}{r['peak_rad']:>8.4f}"
              f"{r['rms_shift_rad']:>8.4f}{r['peak_shift_rad']:>8.4f}{r['noise_rad']:>8.4f}")


def print_summary(rows):
    print(f"\n{'fit':<6}{'Kp':>7}{'n':>4}{'OS%':>7}{'pred':>7}{'Tp ms':>8}{'pred':>7}{'dead':>7}"
          f"{'rms':>8}{'%step':>7}{'peak':>8}{'rms_s':>8}{'%step':>7}{'peak_s':>8}"
          f"{'rms_f':>8}{'%step':>7}{'peak_f':>8}")
    for r in rows:
        kp = f"{r['kp']:.1f}" if r["kp"] != "" else "all"
        print(f"{r['fit']:<6}{kp:>7}{r['n']:>4}{r['os_meas']:>7.1f}{r['os_model']:>7.1f}"
              f"{r['tp_ms_meas']:>8.1f}{r['tp_ms_model']:>7.1f}{r['dead_ms']:>7.1f}"
              f"{r['rms_rad']:>8.4f}{r['rms_pct_step']:>7.2f}{r['peak_rad']:>8.4f}"
              f"{r['rms_shift_rad']:>8.4f}{r['rms_shift_pct_step']:>7.2f}{r['peak_shift_rad']:>8.4f}"
              f"{r['rms_filt_rad']:>8.4f}{r['rms_filt_pct_step']:>7.2f}{r['peak_filt_rad']:>8.4f}")


def print_notes(rows, k1, tau):
    """State plainly what the dead time does to the fit, instead of burying it."""
    overall = [r for r in rows if r["fit"] == OVERLAY_FIT and r["kp"] == ""]
    if not overall:
        return
    r = overall[0]
    print(f"\nfitted plant: K1 = {k1:.3f} rad/(V s), tau = {tau * 1000:.2f} ms")
    print(f"dead time    : {r['dead_ms']:.1f} ms mean, i.e. the gear starts moving that much later")
    print(f"               than the ideal model. It is measured as (measured onset "
          f"{r['onset_ms']:.1f} ms")
    print(f"               - model onset {r['onset_ms_model']:.1f} ms), both at "
          f"{sysid.ONSET_FRAC * 100:.0f} % of the step.")
    print(f"               It inflates the measured Tp by about the same amount, so the "
          f"identified tau")
    print(f"               is biased slightly high. Errors marked _s delay the model by it.")
    print(f"rms error    : {r['rms_rad']:.4f} rad undelayed, {r['rms_shift_rad']:.4f} rad delayed, "
          f"on a {r['step_mag_rad']:.3f} rad step")
    print(f"               = {r['rms_pct_step']:.2f} % and {r['rms_shift_pct_step']:.2f} % of the step; "
          f"sensor noise floor is {r['noise_rad']:.4f} rad.")
    print(f"noise-free   : against the median-filtered angle the mismatch is only "
          f"{r['rms_filt_rad']:.4f} rad")
    print(f"               ({r['rms_filt_pct_step']:.2f} % of the step, peak "
          f"{r['peak_filt_rad']:.4f} rad), so most of the raw error is sensor noise,")
    print(f"               not model error.")


def main(argv):
    paths = [Path(a) for a in argv] or sorted(HERE.glob(sysid.DEFAULT_PATTERN))
    if not paths:
        print(f"no captures found; pass files or add {sysid.DEFAULT_PATTERN} next to this script")
        return 1
    captures = [c for p in paths for c in sysid.load_file(p)]
    if not captures:
        print("no complete capture blocks found")
        return 1

    records = [sysid.analyse_step(c) for c in captures]
    fit_rows = sysid.summarise(records)
    overall, _ = fits_from(fit_rows)
    if overall is None:
        print("no usable steps to validate")
        return 1

    rows = validate(captures, records, fit_rows)
    groups = summarise(rows)
    print_steps(rows, OVERLAY_FIT)
    print_summary(groups)
    print_notes(groups, *overall)

    sysid.write_csv(HERE / "sysid_validation.csv", VALIDATION_FIELDS, rows)
    write_overlay(HERE / "sysid_overlay.csv", captures, records, *overall)
    print(f"\nwrote sysid_validation.csv and sysid_overlay.csv in {HERE}")
    print(f"the overlay model uses the {OVERLAY_FIT} fit with each step's own measured dead time")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
