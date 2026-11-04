"""Tests for analyze_lab2c.py using synthetic D1[z] step captures.

Each synthetic capture comes from the 1 ms sampled-data loop (D1[z], ZOH,
plant K1 / (s (tau s + 1))), quantized through the Lab 1 (c) sensor scaling
and written in the lab2_inner_loop capture format, so the analysis must
recover the metrics the loop was built with.

Usage (from this folder):
    py -3 -m unittest test_analyze_lab2c -v
"""
import contextlib
import io
import math
import tempfile
import unittest
from pathlib import Path

import analyze_lab2c as lab

THETA_M = -3.645179e-4
THETA_OFFSET = 3.949147

# Independent check of the part (b) design (Euler at 10 us, made before this
# script existed): OS 1.26 %, 2 % settling 231 ms, peak |u| 5.687 V
SIM_OS_PCT = 1.26
SIM_TS_S = 0.231
SIM_U_PEAK = 5.687


# ---------------------------------------------------------------- synthetic data
def quantize(theta):
    """Round an angle to the nearest ADC count, as the sensor would."""
    counts = round((theta - THETA_OFFSET) / THETA_M)
    return counts, THETA_M * counts + THETA_OFFSET


def stiction(u):
    if u > 0:
        return u + lab.STICTION_POS
    if u < 0:
        return u - lab.STICTION_NEG
    return 0.0


def capture_text(cid, direction, theta, u, v=None, n=None, end=True):
    """One capture block in the sketch's format: header, rows, end line."""
    n = len(theta) if n is None else n
    v = [stiction(x) for x in u] if v is None else v
    over6 = sum(1 for x in v if abs(x) > lab.V_MAX)
    ref = direction * lab.REF_SAT
    lines = [f"# capture {cid} ctrl=D1 dir={'+' if direction > 0 else '-'} "
             f"dt_us=1000 samples={n} over6={over6}"]
    for i in range(len(theta)):
        counts, th = quantize(theta[i])
        lines.append(f"{i},{i * 1000},{ref:.3f},{v[i]:.3f},{u[i]:.3f},{counts},{th:.4f}")
    if end:
        lines.append(f"# end capture {cid}")
    return "\n".join(lines) + "\n"


def clean_step(direction):
    sim = lab.simulate_step(direction)
    return list(sim["theta"]), list(sim["u"])


def parse_one(text):
    caps = lab.parse_captures(io.StringIO(text), "synthetic")
    assert len(caps) == 1, f"expected one capture, got {len(caps)}"
    return caps[0]


def metrics_of(direction, theta, u, v=None):
    return lab.step_metrics(parse_one(capture_text(1, direction, theta, u, v)))


def check_by_name(checks, fragment):
    matches = [c for c in checks if fragment in c[0]]
    assert len(matches) == 1, f"no single check named like {fragment!r}"
    return matches[0]


# ---------------------------------------------------------------- model
class SimulateStepTest(unittest.TestCase):

    def test_reproduces_independent_part_b_numbers(self):
        sim = lab.simulate_step(+1)
        m = lab.metrics_from_arrays(+1, sim["t"], sim["theta"], sim["u"])
        self.assertAlmostEqual(m["os_pct"], SIM_OS_PCT, delta=0.1)
        self.assertAlmostEqual(m["ts_s"], SIM_TS_S, delta=0.003)
        self.assertAlmostEqual(m["v_peak"], SIM_U_PEAK, delta=0.01)

    def test_starts_on_the_old_level_and_ends_on_the_new(self):
        sim = lab.simulate_step(-1)
        self.assertAlmostEqual(sim["theta"][0], lab.REF_SAT, places=9)
        self.assertAlmostEqual(sim["theta"][-1], -lab.REF_SAT, delta=1e-4)
        self.assertEqual(len(sim["theta"]), lab.CAPTURE_SAMPLES)

    def test_directions_are_mirror_images(self):
        up, down = lab.simulate_step(+1), lab.simulate_step(-1)
        for a, b in zip(up["theta"], down["theta"]):
            self.assertAlmostEqual(a, -b, places=9)

    def test_up_step_needs_negative_voltage(self):
        # K1 < 0 at this station: raising theta takes V < 0
        self.assertLess(lab.simulate_step(+1)["u"][0], 0.0)


# ---------------------------------------------------------------- parsing
class ParseTest(unittest.TestCase):

    def test_round_trip(self):
        theta, u = clean_step(+1)
        cap = parse_one(capture_text(7, +1, theta, u))
        self.assertEqual(cap["id"], 7)
        self.assertEqual(cap["dir"], 1)
        self.assertEqual(len(cap["theta"]), len(theta))
        self.assertAlmostEqual(cap["ref"], lab.REF_SAT)
        self.assertAlmostEqual(cap["u"][0], u[0], places=3)
        self.assertAlmostEqual(cap["t"][1], 0.001)

    def test_skips_capture_without_end_line(self):
        theta, u = clean_step(+1)
        text = capture_text(1, +1, theta, u, end=False) + capture_text(2, -1, *clean_step(-1))
        caps = lab.parse_captures(io.StringIO(text), "synthetic")
        self.assertEqual([c["id"] for c in caps], [2])

    def test_skips_capture_with_missing_rows(self):
        theta, u = clean_step(+1)
        text = capture_text(1, +1, theta[:500], u[:500], n=len(theta))
        self.assertEqual(lab.parse_captures(io.StringIO(text), "synthetic"), [])

    def test_ignores_stream_and_message_lines(self):
        theta, u = clean_step(-1)
        text = "# start\n1234,-1.000,-0.700,-0.6990,0.000\n" + capture_text(3, -1, theta, u)
        self.assertEqual(len(lab.parse_captures(io.StringIO(text), "synthetic")), 1)


# ---------------------------------------------------------------- metrics
class MetricsTest(unittest.TestCase):

    def test_clean_step_recovers_design_metrics_both_directions(self):
        for direction in (+1, -1):
            with self.subTest(direction=direction):
                m = metrics_of(direction, *clean_step(direction))
                self.assertAlmostEqual(m["os_pct"], SIM_OS_PCT, delta=0.15)
                self.assertAlmostEqual(m["ts_s"], SIM_TS_S, delta=0.004)
                self.assertLess(abs(m["ss_err"]), 0.001)
                # Logged V includes the stiction offset on top of the linear u
                self.assertAlmostEqual(m["u_peak"], SIM_U_PEAK, delta=0.01)
                self.assertGreater(m["v_peak"], m["u_peak"])

    def test_single_sample_spike_does_not_move_settling(self):
        theta, u = clean_step(+1)
        clean = metrics_of(+1, theta, u)["ts_s"]
        theta[600] += 0.05
        self.assertAlmostEqual(metrics_of(+1, theta, u)["ts_s"], clean, delta=0.001)

    def test_overshoot_of_six_percent_is_measured(self):
        theta, u = clean_step(+1)
        for i in range(len(theta)):
            theta[i] += 0.084 * math.exp(-((i - 400) / 20.0) ** 2)   # bump at 400 ms, after the step has arrived
        self.assertGreater(metrics_of(+1, theta, u)["os_pct"], 5.0)

    def test_overshoot_measured_on_down_steps_too(self):
        theta, u = clean_step(-1)
        for i in range(len(theta)):
            theta[i] -= 0.084 * math.exp(-((i - 400) / 20.0) ** 2)
        self.assertGreater(metrics_of(-1, theta, u)["os_pct"], 5.0)

    def test_late_excursion_sets_settling_time(self):
        theta, u = clean_step(+1)
        for i in range(250, 450):
            theta[i] -= 0.05
        self.assertAlmostEqual(metrics_of(+1, theta, u)["ts_s"], 0.450, delta=0.003)

    def test_never_settling_reports_none(self):
        theta, u = clean_step(+1)
        theta = [x - 0.05 for x in theta]
        self.assertIsNone(metrics_of(+1, theta, u)["ts_s"])

    def test_steady_state_error_is_signed_ref_minus_theta(self):
        theta, u = clean_step(+1)
        theta = [x - 0.02 for x in theta]
        self.assertAlmostEqual(metrics_of(+1, theta, u)["ss_err"], 0.02, delta=0.001)

    def test_voltage_excess_over_six_volts(self):
        theta, u = clean_step(+1)
        u[5] = -6.25 + lab.STICTION_NEG       # a controller output that commands -6.25 V
        v = [stiction(x) for x in u]
        m = metrics_of(+1, theta, u, v)
        self.assertAlmostEqual(m["v_peak"], 6.25, places=3)
        self.assertAlmostEqual(m["v_excess"], 0.25, places=3)


# ---------------------------------------------------------------- spec verdict
class SpecChecksTest(unittest.TestCase):

    def records(self, edit=None):
        recs = []
        for cid, direction in enumerate((+1, -1, +1, -1)):
            theta, u = clean_step(direction)
            v = [stiction(x) for x in u]
            if edit is not None and cid == 0:
                theta, u, v = edit(theta, u, v)
            rec = lab.step_metrics(parse_one(capture_text(cid, direction, theta, u, v)))
            recs.append(rec)
        return recs

    def test_clean_set_passes_every_spec(self):
        checks = lab.spec_checks(self.records())
        failed = [c for c in checks if not c[1]]
        self.assertEqual(failed, [])

    def test_needs_both_directions(self):
        recs = [r for r in self.records() if r["dir"] > 0]
        ok = check_by_name(lab.spec_checks(recs), "both directions")[1]
        self.assertFalse(ok)

    def test_overshoot_fails(self):
        def bump(theta, u, v):
            return [x + 0.084 * math.exp(-((i - 400) / 20.0) ** 2)
                    for i, x in enumerate(theta)], u, v
        self.assertFalse(check_by_name(lab.spec_checks(self.records(bump)), "overshoot")[1])

    def test_slow_settling_fails(self):
        def late(theta, u, v):
            return [x - 0.05 if 250 <= i < 450 else x for i, x in enumerate(theta)], u, v
        self.assertFalse(check_by_name(lab.spec_checks(self.records(late)), "settling")[1])

    def test_voltage_excess_within_stiction_allowance_passes(self):
        def small(theta, u, v):
            u, v = list(u), list(v)
            v[5] = -(lab.V_MAX + 0.25)
            u[5] = v[5] + lab.STICTION_NEG      # keep the row consistent: V = u - offset
            return theta, u, v
        self.assertTrue(check_by_name(lab.spec_checks(self.records(small)), "voltage")[1])

    def test_voltage_excess_beyond_stiction_allowance_fails(self):
        def big(theta, u, v):
            u, v = list(u), list(v)
            v[5] = -(lab.V_MAX + 0.5)
            u[5] = v[5] + lab.STICTION_NEG
            return theta, u, v
        self.assertFalse(check_by_name(lab.spec_checks(self.records(big)), "voltage")[1])

    def test_steady_state_error_fails(self):
        def offset(theta, u, v):
            return [x - 0.02 for x in theta], u, v
        self.assertFalse(check_by_name(lab.spec_checks(self.records(offset)), "steady-state")[1])

    def test_gear_limit_fails(self):
        def runaway(theta, u, v):
            theta = list(theta)
            theta[100] = 0.82
            theta[101] = 0.82
            theta[102] = 0.82
            return theta, u, v
        self.assertFalse(check_by_name(lab.spec_checks(self.records(runaway)), "pi/4")[1])


# ---------------------------------------------------------------- simulation file and overlay
class SimFileTest(unittest.TestCase):

    def test_load_sim_csv(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "sim.csv"
            path.write_text("dir,t_ms,theta,V\n+,0,-0.7,-3.37\n+,1,-0.69,-3.5\n-,0,0.7,3.37\n")
            sim = lab.load_sim_csv(path)
        self.assertEqual(sim[1]["theta"], [-0.7, -0.69])
        self.assertEqual(sim[-1]["u"], [3.37])

    def test_overlay_uses_first_capture_of_each_direction(self):
        caps = [parse_one(capture_text(cid, d, *clean_step(d)))
                for cid, d in ((0, +1), (1, -1), (2, +1))]
        sim = {d: lab.simulate_step(d) for d in (+1, -1)}
        rows = lab.overlay_rows(caps, sim)
        self.assertEqual(len(rows), 2 * lab.CAPTURE_SAMPLES)
        self.assertEqual({r["dir"] for r in rows}, {"+", "-"})
        first = rows[0]
        self.assertAlmostEqual(first["theta_meas"], first["theta_sim"], delta=0.001)

    def test_overlay_truncates_to_shorter_trace(self):
        caps = [parse_one(capture_text(0, +1, *clean_step(+1)))]
        short = lab.simulate_step(+1)
        sim = {+1: {k: v[:300] for k, v in short.items()}}
        self.assertEqual(len(lab.overlay_rows(caps, sim)), 300)


# ---------------------------------------------------------------- beam-on runs (Lab 2 d)
START_BEAM = ("# start ctrl=D1 ref_amp=1.00 ref_sat=0.70 dt_ms=1 half_period_ms=1000 "
              "stiction_pos=0.450 stiction_neg=0.400\n")


class RunStictionTest(unittest.TestCase):

    def test_start_line_sets_the_capture_stiction(self):
        cap = parse_one(START_BEAM + capture_text(1, +1, *clean_step(+1)))
        self.assertEqual(cap["stiction"], (0.45, 0.40))

    def test_no_start_line_falls_back_to_the_part_c_offsets(self):
        cap = parse_one(capture_text(1, +1, *clean_step(+1)))
        self.assertEqual(cap["stiction"], (lab.STICTION_POS, lab.STICTION_NEG))

    def test_later_start_line_applies_to_later_captures_only(self):
        text = (capture_text(1, +1, *clean_step(+1)) + START_BEAM
                + capture_text(2, -1, *clean_step(-1)))
        caps = lab.parse_captures(io.StringIO(text), "synthetic")
        self.assertEqual([c["stiction"] for c in caps],
                         [(lab.STICTION_POS, lab.STICTION_NEG), (0.45, 0.40)])

    def test_voltage_allowance_follows_the_run_stiction(self):
        # -6.42 V: inside the 0.40 V beam offset + margin, beyond the part (c) one.
        # Each run gets a u consistent with its own offset, as the sketch would send.
        theta, u = clean_step(+1)
        v = [stiction(x) for x in u]
        v[5] = -(lab.V_MAX + 0.42)
        u_beam, u_plain = list(u), list(u)
        u_beam[5], u_plain[5] = v[5] + 0.40, v[5] + lab.STICTION_NEG
        beam = lab.step_metrics(parse_one(START_BEAM + capture_text(1, +1, theta, u_beam, v)))
        plain = lab.step_metrics(parse_one(capture_text(1, +1, theta, u_plain, v)))
        self.assertTrue(check_by_name(lab.spec_checks([beam]), "voltage")[1])
        self.assertFalse(check_by_name(lab.spec_checks([plain]), "voltage")[1])


class TransmissionGlitchTest(unittest.TestCase):
    """A serial glitch can corrupt one printed value (beam_run2: -5.668 arrived as
    -52.668). Each row is redundant: V must equal u plus the stiction offset, and
    theta must equal the scaled motor count, so a corrupted value is detectable."""

    def corrupt(self, text, sample, column, new_value):
        lines = text.splitlines()
        row = next(i for i, l in enumerate(lines) if l.startswith(f"{sample},{sample * 1000},"))
        fields = lines[row].split(",")
        fields[column] = new_value
        lines[row] = ",".join(fields)
        return "\n".join(lines) + "\n"

    def test_corrupted_voltage_is_repaired_from_u(self):
        theta, u = clean_step(+1)
        text = self.corrupt(capture_text(1, +1, theta, u), 82, 3, "-52.668")
        cap = parse_one(text)
        self.assertAlmostEqual(cap["v"][82], stiction(u[82]), places=2)
        self.assertEqual([r["i"] for r in cap["repaired"]], [82])
        self.assertLess(lab.step_metrics(cap)["v_peak"], 6.2)

    def test_corrupted_theta_is_repaired_from_the_motor_count(self):
        theta, u = clean_step(-1)
        good = parse_one(capture_text(1, -1, theta, u))["theta"][300]
        text = self.corrupt(capture_text(1, -1, theta, u), 300, 6, "-7.7000")
        cap = parse_one(text)
        self.assertAlmostEqual(cap["theta"][300], good, places=4)
        self.assertEqual([r["field"] for r in cap["repaired"]], ["theta"])

    def test_clean_capture_has_no_repairs(self):
        self.assertEqual(parse_one(capture_text(1, +1, *clean_step(+1)))["repaired"], [])

    def test_beam_offsets_are_used_for_the_voltage_check(self):
        theta, u = clean_step(+1)
        v = [x + 0.45 if x > 0 else (x - 0.40 if x < 0 else 0.0) for x in u]
        cap = parse_one(START_BEAM + capture_text(1, +1, theta, u, v))
        self.assertEqual(cap["repaired"], [])

    def test_cutoff_zero_volt_samples_are_not_repaired(self):
        # A single reading past 0.8 rad sends 0 V while u holds its last value
        theta, u = clean_step(+1)
        v = [stiction(x) for x in u]
        v[50] = 0.0
        self.assertEqual(parse_one(capture_text(1, +1, theta, u, v))["repaired"], [])


class OutputFolderTest(unittest.TestCase):

    def test_csvs_are_written_next_to_the_input_file(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "beam_run.txt"
            path.write_text(capture_text(0, +1, *clean_step(+1)) + capture_text(1, -1, *clean_step(-1)))
            with contextlib.redirect_stdout(io.StringIO()):
                rc = lab.main([str(path)])
            self.assertEqual(rc, 0)
            self.assertTrue((Path(d) / "lab2c_steps.csv").exists())
            self.assertTrue((Path(d) / "lab2c_overlay.csv").exists())


if __name__ == "__main__":
    unittest.main()
