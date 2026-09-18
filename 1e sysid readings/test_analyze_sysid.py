"""Tests for analyze_sysid.py using synthetic closed-loop step responses.

Each synthetic capture comes from a known plant (K1, tau) under P control,
quantized through the part (c) sensor scaling, so the analysis must recover
the plant parameters it was built from.

Usage (from this folder):
    python -m unittest test_analyze_sysid -v
"""
import math
import random
import unittest

import analyze_sysid as sysid

THETA_M = -3.645179e-4
THETA_OFFSET = 3.949147
K1_TRUE = -2.2
TAU_TRUE = 0.025


# ---------------------------------------------------------------- synthetic data
def closed_loop(k1, tau, kp):
    """Return (zeta, wn) of the P-controlled plant K1 / (s (tau s + 1))."""
    wn = math.sqrt(kp * k1 / tau)
    return 1.0 / (2.0 * tau * wn), wn


def step_thetas(k1, tau, kp, direction, n=600, dt=0.001, noise=0.0,
                spike_every=0, clamp_frac=None, seed=1):
    """Angles for one step from -dir*0.1 to +dir*0.1 rad, quantized to ADC counts."""
    rng = random.Random(seed)
    zeta, wn = closed_loop(k1, tau, kp)
    wd = wn * math.sqrt(1.0 - zeta * zeta)
    theta0, theta1 = -0.1 * direction, 0.1 * direction
    peak = theta1 + (theta1 - theta0) * math.exp(-zeta * math.pi / math.sqrt(1 - zeta * zeta))

    thetas = []
    for i in range(n):
        t = i * dt
        y = 1.0 - math.exp(-zeta * wn * t) * (math.cos(wd * t) + zeta / math.sqrt(1 - zeta * zeta) * math.sin(wd * t))
        theta = theta0 + (theta1 - theta0) * y
        if clamp_frac is not None:   # flatten the top of the first peak
            level = theta1 + clamp_frac * (peak - theta1)
            theta = min(theta, level) if direction > 0 else max(theta, level)
        theta += rng.gauss(0.0, noise) if noise else 0.0
        if spike_every and i % spike_every == spike_every // 2:
            theta += 0.015 * rng.choice((-1, 1))
        thetas.append(theta)
    return thetas


def capture_text(step_id, kp, direction, thetas, sat=0, dt_us=1000):
    """Format one step exactly as lab1_sysid.ino prints it."""
    lines = [f"# capture {step_id} kp={kp:.2f} dir={'+' if direction > 0 else '-'} "
             f"dt_us={dt_us} samples={len(thetas)} sat={sat}"]
    for i, theta in enumerate(thetas):
        motor = round((theta - THETA_OFFSET) / THETA_M)
        theta_q = THETA_M * motor + THETA_OFFSET
        lines.append(f"{i},{i * dt_us},{0.1 * direction:.3f},0.000,{motor},{theta_q:.4f}")
    lines.append(f"# end capture {step_id}")
    return lines


def analyse_synthetic(kp, direction, **kwargs):
    sat = kwargs.pop("sat", 0)
    lines = capture_text(1, kp, direction, step_thetas(K1_TRUE, TAU_TRUE, kp, direction, **kwargs), sat=sat)
    return sysid.analyse_step(sysid.parse_captures(lines, "synthetic")[0])


# ---------------------------------------------------------------- tests
class TestFormulas(unittest.TestCase):

    def test_zeta_from_overshoot_matches_textbook_value(self):
        # 16.3 % overshoot corresponds to zeta = 0.5
        self.assertAlmostEqual(sysid.zeta_from_os(16.303), 0.5, places=3)

    def test_plant_from_closed_loop_inverts_the_model(self):
        zeta, wn = closed_loop(K1_TRUE, TAU_TRUE, -20.0)
        k1, tau = sysid.plant_from_closed_loop(zeta, wn, -20.0)
        self.assertAlmostEqual(k1, K1_TRUE, places=6)
        self.assertAlmostEqual(tau, TAU_TRUE, places=6)

    def test_fit_vertex_finds_exact_parabola_peak(self):
        ts = [i * 0.001 for i in range(21)]
        ys = [0.3 - 50.0 * (t - 0.0123) ** 2 for t in ts]
        t_v, y_v = sysid.fit_vertex(ts, ys)
        self.assertAlmostEqual(t_v, 0.0123, places=6)
        self.assertAlmostEqual(y_v, 0.3, places=6)

    def test_median_filter_removes_single_sample_spike(self):
        values = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        self.assertEqual(sysid.median_filter(values, 5), [0.0] * 7)


class TestParsing(unittest.TestCase):

    def test_reads_header_fields_and_rows(self):
        lines = capture_text(7, -18.5, -1, step_thetas(K1_TRUE, TAU_TRUE, -18.5, -1), sat=3)
        cap = sysid.parse_captures(lines, "f.txt")[0]
        self.assertEqual((cap["id"], cap["kp"], cap["dir"], cap["sat"]), (7, -18.5, -1, 3))
        self.assertEqual(len(cap["theta"]), 600)
        self.assertAlmostEqual(cap["t"][1], 0.001)

    def test_ignores_stream_and_message_lines(self):
        lines = ["# start kp=-20.00 dt_ms=1", "-0.100,-0.0985,0.308", "# capture armed"]
        lines += capture_text(1, -20.0, 1, step_thetas(K1_TRUE, TAU_TRUE, -20.0, 1))
        lines += ["0.100,0.1009,0.296", "# stop"]
        caps = sysid.parse_captures(lines, "f.txt")
        self.assertEqual(len(caps), 1)
        self.assertEqual(len(caps[0]["theta"]), 600)

    def test_drops_truncated_capture(self):
        lines = capture_text(1, -20.0, 1, step_thetas(K1_TRUE, TAU_TRUE, -20.0, 1))[:300]
        self.assertEqual(sysid.parse_captures(lines, "f.txt"), [])


class TestRecovery(unittest.TestCase):

    def assert_recovers(self, result, tolerance):
        self.assertLess(abs(result["k1"] / K1_TRUE - 1.0), tolerance, result)
        self.assertLess(abs(result["tau"] / TAU_TRUE - 1.0), tolerance, result)

    def test_recovers_plant_for_each_gain_and_direction(self):
        for kp in (-15.0, -20.0, -25.0):
            for direction in (1, -1):
                with self.subTest(kp=kp, direction=direction):
                    result = analyse_synthetic(kp, direction)
                    self.assert_recovers(result, 0.03)
                    self.assertEqual(result["exclude"], "")

    def test_overshoot_is_positive_for_both_directions(self):
        up = analyse_synthetic(-20.0, 1)
        down = analyse_synthetic(-20.0, -1)
        self.assertGreater(up["os_pct"], 0.0)
        self.assertAlmostEqual(up["os_pct"], down["os_pct"], delta=0.5)

    def test_average_of_noisy_steps_recovers_plant(self):
        # One noisy step can be ~7 % off; the report averages many steps per gain
        results = [analyse_synthetic(-20.0, (1, -1)[s % 2], noise=0.0015, spike_every=40, seed=s)
                   for s in range(10)]
        overall = [r for r in sysid.summarise(results) if r["group"] == "all"][0]
        self.assertEqual(overall["n"], 10)
        self.assertLess(abs(overall["k1_mean"] / K1_TRUE - 1.0), 0.03)
        self.assertLess(abs(overall["tau_mean"] / TAU_TRUE - 1.0), 0.03)


class TestFlags(unittest.TestCase):

    def test_saturated_step_is_excluded(self):
        self.assertIn("saturated", analyse_synthetic(-20.0, 1, sat=5)["exclude"])

    def test_low_overshoot_step_is_excluded(self):
        self.assertIn("low_os", analyse_synthetic(-5.0, 1)["exclude"])

    def test_flat_topped_peak_is_warned(self):
        self.assertIn("flat_peak", analyse_synthetic(-20.0, 1, clamp_frac=0.5)["warn"])

    def test_clean_peak_is_not_warned(self):
        self.assertNotIn("flat_peak", analyse_synthetic(-20.0, 1)["warn"])

    def test_gear_that_never_moves_is_excluded_without_crashing(self):
        lines = capture_text(1, -20.0, 1, [-0.1] * 600)
        result = sysid.analyse_step(sysid.parse_captures(lines, "f.txt")[0])
        self.assertIn("no_step", result["exclude"])
        self.assertTrue(math.isnan(result["k1"]))

    def test_step_in_the_wrong_direction_is_excluded(self):
        down = step_thetas(K1_TRUE, TAU_TRUE, -20.0, -1)
        lines = capture_text(1, -20.0, 1, down)   # header says up, gear went down
        result = sysid.analyse_step(sysid.parse_captures(lines, "f.txt")[0])
        self.assertIn("no_step", result["exclude"])

    def test_implausibly_large_overshoot_is_excluded(self):
        # Kp = -400 gives zeta ~ 0.1, i.e. ~70 % overshoot
        self.assertIn("high_os", analyse_synthetic(-400.0, 1)["exclude"])

    def test_header_before_end_drops_unfinished_block(self):
        first = capture_text(1, -20.0, 1, step_thetas(K1_TRUE, TAU_TRUE, -20.0, 1))[:300]
        second = capture_text(2, -20.0, -1, step_thetas(K1_TRUE, TAU_TRUE, -20.0, -1))
        caps = sysid.parse_captures(first + second, "f.txt")
        self.assertEqual([c["id"] for c in caps], [2])


class TestSummary(unittest.TestCase):

    def test_summary_skips_excluded_steps(self):
        good = [analyse_synthetic(-20.0, d) for d in (1, -1)]
        bad = analyse_synthetic(-20.0, 1, sat=5)
        rows = sysid.summarise(good + [bad])
        overall = [r for r in rows if r["group"] == "all"][0]
        self.assertEqual(overall["n"], 2)
        self.assertAlmostEqual(overall["k1_mean"], K1_TRUE, delta=0.05)


if __name__ == "__main__":
    unittest.main()
