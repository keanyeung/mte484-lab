"""Tests for analyze_stiction_beam.py using synthetic ramp captures.

Each synthetic run ramps the voltage exactly like lab2_stiction_test (20 ms
samples, +0.001 V per sample) and the gear starts to move once the applied
voltage passes a known breakaway value, so the analysis must recover it.

Usage (from this folder):
    py -3 -m unittest test_analyze_stiction_beam -v
"""
import contextlib
import io
import tempfile
import unittest
from pathlib import Path

import analyze_stiction_beam as beam

THETA_M = -3.645179e-4
THETA_OFFSET = 3.949147
RAMP_STEP = 0.001
MOVE_PER_SAMPLE = 0.004     # rad per sample once the gear breaks away
MOTION_STOP = 0.03          # the sketch stops a run after this much travel [rad]


# ---------------------------------------------------------------- synthetic data
def quantize(theta):
    counts = round((theta - THETA_OFFSET) / THETA_M)
    return counts, THETA_M * counts + THETA_OFFSET


def run_lines(run, prev_run, command, breakaway, theta0, t0):
    """Idle, baseline and ramp lines for one run, plus its '# run' summary.

    Each theta is read before that line's voltage is applied, so the gear
    moves on the sample after the applied voltage first exceeds breakaway.
    +V turns the gear CW (theta decreases), -V CCW (theta increases).
    """
    sign = 1 if command == "+" else -1
    lines, t = [], t0
    for _ in range(25):                              # idle, labelled with the previous run
        c, th = quantize(theta0)
        lines.append(f"{t},{prev_run},0,0.000,{c},{th:.4f}"); t += 20
    for _ in range(10):                              # baseline at 0 V
        c, th = quantize(theta0)
        lines.append(f"{t},{run},1,0.000,{c},{th:.4f}"); t += 20
    theta, v_prev, v = theta0, 0.0, 0.0
    while abs(theta - theta0) <= MOTION_STOP:
        if abs(v_prev) > breakaway:
            theta -= sign * MOVE_PER_SAMPLE
        v = sign * (abs(v) + RAMP_STEP)
        c, th = quantize(theta)
        lines.append(f"{t},{run},2,{v:.3f},{c},{th:.4f}"); t += 20
        v_prev = v
    lines.append(f"# run {run} stop MOTION V={v:.3f} theta_start={theta0:.4f} theta_end={theta:.4f}")
    return lines, t


def capture(theta0, runs):
    """runs: list of (command, breakaway). Returns the file text."""
    out, t, prev = ["# geeWhiz Started - stiction test (beam)"], 1000, 0
    for i, (cmd, b) in enumerate(runs, start=1):
        lines, t = run_lines(i, prev, cmd, b, theta0, t)
        out.extend(lines)
        prev = i
    return "\n".join(out) + "\n"


def write_set(folder, cw, ccw):
    """Write all three position files with the same breakaway lists."""
    for name, pos in beam.FILES:
        runs = [("+", b) for b in cw] + [("-", b) for b in ccw]
        (Path(folder) / name).write_text(capture(pos, runs))


# ---------------------------------------------------------------- tests
class LoadTest(unittest.TestCase):

    def test_recovers_breakaway_voltage_per_direction(self):
        with tempfile.TemporaryDirectory() as d:
            write_set(d, cw=[0.40, 0.42], ccw=[0.25, 0.27])
            records, missing = beam.load_records(Path(d))
        self.assertEqual(missing, [])
        self.assertEqual(len(records), 12)
        for r in records:
            expected = {"CW": (0.40, 0.42), "CCW": (0.25, 0.27)}[r["direction"]]
            onset = r[beam.ONSET_KEY]
            self.assertTrue(any(abs(onset - b) <= 0.0025 for b in expected),
                            f"{r['direction']} onset {onset} not near {expected}")

    def test_positions_come_from_the_file_names(self):
        with tempfile.TemporaryDirectory() as d:
            write_set(d, cw=[0.4], ccw=[0.3])
            records, _ = beam.load_records(Path(d))
        self.assertEqual(sorted({r["position_rad"] for r in records}), [-0.5, 0.0, 0.5])

    def test_reports_missing_files(self):
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / "stiction_zero.txt").write_text(capture(0.0, [("+", 0.4), ("-", 0.3)]))
            records, missing = beam.load_records(Path(d))
        self.assertEqual(len(records), 2)
        self.assertEqual(sorted(missing), ["stiction_neg05.txt", "stiction_pos05.txt"])


class BoardResetTest(unittest.TestCase):

    def test_run_numbers_restarting_after_a_reset_stay_separate(self):
        # Two sessions in one file, each numbering its runs from 1
        text = capture(0.0, [("+", 0.40), ("-", 0.25)]) + capture(0.0, [("+", 0.50), ("-", 0.30)])
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / "stiction_zero.txt").write_text(text)
            records, _ = beam.load_records(Path(d))
        self.assertEqual(len(records), 4)
        onsets = sorted(round(r[beam.ONSET_KEY], 2) for r in records)
        self.assertEqual(onsets, [0.25, 0.30, 0.40, 0.50])


class RecommendTest(unittest.TestCase):

    def test_offset_is_mean_plus_two_sd_per_direction(self):
        records = [{"direction": "CW", beam.ONSET_KEY: v} for v in (0.40, 0.44)] + \
                  [{"direction": "CCW", beam.ONSET_KEY: v} for v in (0.30, 0.30)]
        rec = beam.recommend(records)
        self.assertAlmostEqual(rec["CW"]["mean_2sd"], 0.42 + 2 * 0.028284271, places=6)
        self.assertAlmostEqual(rec["CCW"]["mean_2sd"], 0.30, places=9)

    def test_ignores_runs_with_no_onset(self):
        records = [{"direction": "CW", beam.ONSET_KEY: v} for v in (0.40, None, 0.40)] + \
                  [{"direction": "CCW", beam.ONSET_KEY: 0.3}]
        self.assertEqual(beam.recommend(records)["CW"]["n"], 2)


class CompareTest(unittest.TestCase):

    def test_change_against_lab1_offsets(self):
        rec = {"CW": {"mean_2sd": 0.477}, "CCW": {"mean_2sd": 0.259}}
        rows = {r["direction"]: r for r in beam.compare_with_lab1(rec)}
        self.assertAlmostEqual(rows["CW"]["change_V"], 0.200, places=9)
        self.assertAlmostEqual(rows["CCW"]["change_V"], 0.0, places=9)
        self.assertEqual(rows["CW"]["constant"], "STICTION_POS")
        self.assertEqual(rows["CCW"]["constant"], "STICTION_NEG")


class MainTest(unittest.TestCase):

    def test_main_writes_runs_csv_and_returns_zero(self):
        with tempfile.TemporaryDirectory() as d:
            write_set(d, cw=[0.40, 0.42], ccw=[0.25, 0.27])
            with contextlib.redirect_stdout(io.StringIO()) as out:
                rc = beam.main(Path(d))
            self.assertEqual(rc, 0)
            self.assertTrue((Path(d) / beam.RUNS_CSV).exists())
        self.assertIn("STICTION_POS", out.getvalue())

    def test_main_fails_without_any_capture(self):
        with tempfile.TemporaryDirectory() as d:
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(beam.main(Path(d)), 1)


if __name__ == "__main__":
    unittest.main()
