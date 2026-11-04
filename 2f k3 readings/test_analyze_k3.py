"""Tests for analyze_k3.py using synthetic hold-mode streams.

Each synthetic hold has the ball released from rest near the high end, rolling
with a known constant acceleration to the low stopper, then pushed back uphill
by the pen (which must not count as a roll), in the lab2_inner_loop stream
format: t_ms,ref_raw,ref_sat,theta,V,y_m every 10 ms.

Usage (from this folder):
    py -3 -m unittest test_analyze_k3 -v
"""
import contextlib
import io
import math
import random
import tempfile
import unittest
from pathlib import Path

import analyze_k3 as k3

K2 = 0.0609
DT = 0.010
Y_LEFT, Y_RIGHT = 0.0127, 0.3693
RELEASE_GAP = 0.035          # ball held this far from the high stopper before release


def rows_for(times, ys, theta, ref):
    return [f"{int(round(t * 1000))},{ref:.3f},{ref:.3f},{theta:.4f},0.000,{y:.4f}" for t, y in zip(times, ys)]


def hold_text(ref, accel, rolls=2, y_noise=0.0, theta=None, seed=1):
    """One hold: rest, then `rolls` x (release -> roll to stopper -> rest -> pen push back -> rest)."""
    rng = random.Random(seed)
    theta = ref if theta is None else theta
    downhill_left = accel < 0
    y_high = (Y_RIGHT - RELEASE_GAP) if downhill_left else (Y_LEFT + RELEASE_GAP)
    y_low = Y_LEFT if downhill_left else Y_RIGHT
    t, ys, times = 0.0, [], []

    def add(y):
        nonlocal t
        times.append(t)
        ys.append(y + rng.gauss(0.0, y_noise))
        t += DT

    for _ in range(100):                      # 1 s at rest, held at the high end
        add(y_high)
    for _ in range(rolls):
        tau = 0.0
        while True:                           # released from rest, constant acceleration
            y = y_high + 0.5 * accel * tau * tau
            if (downhill_left and y <= y_low) or (not downhill_left and y >= y_low):
                break
            add(y)
            tau += DT
        for _ in range(50):                   # resting against the low stopper
            add(y_low)
        steps = 80                            # pen pushes it back up at constant speed
        for i in range(1, steps + 1):
            add(y_low + (y_high - y_low) * i / steps)
        for _ in range(100):                  # held at the high end before the next release
            add(y_high)
    lines = [f"# start hold ref={ref:.3f} ctrl=D1 dt_ms=1 stiction_pos=0.134 stiction_neg=0.723"]
    lines += rows_for(times, ys, theta, ref)
    lines.append("# stop, samples beyond the D/A range: 0")
    return "\n".join(lines) + "\n"


def accel_for(theta, k3_true=-6.0, theta0=0.0):
    return k3_true * K2 * (theta - theta0)


class ParseTest(unittest.TestCase):

    def test_reads_hold_runs_and_ignores_others(self):
        text = ("# start ctrl=D1 ref_amp=1.00 ref_sat=0.70 dt_ms=1 half_period_ms=1000 "
                "stiction_pos=0.134 stiction_neg=0.723\n"
                "1000,-1.000,-0.700,-0.6990,0.000,0.1000\n# stop\n"
                + hold_text(0.2, accel_for(0.2)))
        holds = k3.parse_holds(io.StringIO(text), "synthetic")
        self.assertEqual(len(holds), 1)
        self.assertAlmostEqual(holds[0]["ref"], 0.2)
        self.assertGreater(len(holds[0]["y"]), 100)


class RollTest(unittest.TestCase):

    def holds(self, *texts):
        return k3.parse_holds(io.StringIO("".join(texts)), "synthetic")

    def test_finds_both_rolls_and_rejects_the_pen_pushes(self):
        hold = self.holds(hold_text(0.2, accel_for(0.2)))[0]
        rolls, rejected = k3.find_rolls(hold)
        self.assertEqual(len(rolls), 2)
        self.assertGreaterEqual(len(rejected), 2)            # the two pushes back uphill
        for r in rolls:
            self.assertAlmostEqual(r["a"], accel_for(0.2), delta=abs(accel_for(0.2)) * 0.02)
            self.assertGreater(r["r2"], 0.999)

    def test_recovers_acceleration_in_both_directions(self):
        for theta in (0.3, -0.3):
            with self.subTest(theta=theta):
                hold = self.holds(hold_text(theta, accel_for(theta)))[0]
                rolls, _ = k3.find_rolls(hold)
                self.assertEqual(len(rolls), 2)
                self.assertAlmostEqual(rolls[0]["a"], accel_for(theta), delta=abs(accel_for(theta)) * 0.02)

    def test_noisy_position_still_fits(self):
        hold = self.holds(hold_text(0.2, accel_for(0.2), y_noise=0.0003))[0]
        rolls, _ = k3.find_rolls(hold)
        self.assertEqual(len(rolls), 2)
        self.assertAlmostEqual(rolls[0]["a"], accel_for(0.2), delta=abs(accel_for(0.2)) * 0.05)

    def test_rebound_off_the_stopper_does_not_spoil_the_roll(self):
        # The ball bounces 3 cm back off the low stopper (0.2 s), then settles against it
        bounce = [Y_LEFT + 0.03 * (1 - abs(1 - i / 10)) for i in range(1, 21)]
        hold = self.holds(hold_text(0.3, accel_for(0.3), rolls=1))[0]
        arrive = next(i for i, y in enumerate(hold["y"]) if i > 100 and y <= Y_LEFT + 1e-9)
        hold["y"][arrive:arrive + len(bounce)] = bounce
        rolls, _ = k3.find_rolls(hold)
        self.assertEqual(len(rolls), 1)
        self.assertAlmostEqual(rolls[0]["a"], accel_for(0.3), delta=abs(accel_for(0.3)) * 0.03)

    def test_stalled_ball_gives_no_roll(self):
        lines = ["# start hold ref=0.100 ctrl=D1 dt_ms=1 stiction_pos=0.134 stiction_neg=0.723"]
        lines += rows_for([i * DT for i in range(500)], [0.30] * 500, 0.1, 0.1)
        hold = self.holds("\n".join(lines) + "\n")[0]
        rolls, _ = k3.find_rolls(hold)
        self.assertEqual(rolls, [])


class K3Test(unittest.TestCase):

    def all_rolls(self, k3_true, theta0):
        rolls = []
        for i, theta in enumerate((0.1, 0.2, 0.3, -0.1, -0.2, -0.3)):
            text = hold_text(theta, accel_for(theta, k3_true, theta0), seed=i)
            hold = k3.parse_holds(io.StringIO(text), "synthetic")[0]
            rolls += k3.find_rolls(hold)[0]
        return rolls

    def test_slope_gives_k3_and_intercept_gives_level_offset(self):
        rolls = self.all_rolls(-6.0, theta0=0.02)
        fit = k3.fit_k3(rolls)
        self.assertAlmostEqual(fit["k3"], -6.0, delta=0.12)
        self.assertAlmostEqual(fit["theta0"], 0.02, delta=0.003)
        self.assertGreater(fit["r2"], 0.999)

    def test_per_roll_k3(self):
        rolls = self.all_rolls(-6.0, theta0=0.0)
        for r in rolls:
            self.assertAlmostEqual(k3.roll_k3(r), -6.0, delta=0.15)

    def test_needs_two_angles(self):
        rolls = [{"ref": 0.2, "theta": 0.2, "a": -0.07}, {"ref": 0.2, "theta": 0.2, "a": -0.072}]
        with self.assertRaises(ValueError):
            k3.fit_k3(rolls)

    def test_one_angle_with_jittered_theta_is_still_one_angle(self):
        # Several rolls at one hold angle differ slightly in measured theta; that is not a slope
        rolls = [{"ref": 0.2, "theta": 0.2 + d, "a": -0.073} for d in (-0.002, 0.0, 0.003)]
        with self.assertRaises(ValueError):
            k3.fit_k3(rolls)

    def test_no_change_of_acceleration_with_angle_is_an_error_not_a_crash(self):
        rolls = [{"ref": th, "theta": th, "a": -0.07} for th in (0.1, 0.2, 0.3)]
        with self.assertRaises(ValueError):
            k3.fit_k3(rolls)

    def test_rolling_resistance_model_separates_slope_from_offsets(self):
        # a = s*theta + b_dir: the offset depends on which way the ball travels
        s_true, b_left, b_right = -6.0 * K2, -0.009, -0.029
        rolls = []
        for th in (0.1, 0.2, 0.3):          # positive theta: rolls toward y = 0 (left)
            rolls.append({"ref": th, "theta": th, "a": s_true * th + b_left, "y_from": 0.33, "y_to": 0.04})
        for th in (-0.2, -0.3, -0.4):       # negative theta: rolls toward the motor end (right)
            rolls.append({"ref": th, "theta": th, "a": s_true * th + b_right, "y_from": 0.05, "y_to": 0.35})
        fit = k3.fit_k3_resistance(rolls)
        self.assertAlmostEqual(fit["k3"], -6.0, places=6)
        self.assertAlmostEqual(fit["b_left"], b_left, places=9)
        self.assertAlmostEqual(fit["b_right"], b_right, places=9)
        self.assertAlmostEqual(fit["theta_level"], -(b_left + b_right) / 2 / s_true, places=6)
        self.assertAlmostEqual(fit["resistance"], abs(b_left - b_right) / 2, places=9)

    def test_rolling_resistance_model_needs_both_directions(self):
        rolls = [{"ref": th, "theta": th, "a": -0.3 * th, "y_from": 0.33, "y_to": 0.04} for th in (0.1, 0.2)]
        with self.assertRaises(ValueError):
            k3.fit_k3_resistance(rolls)

    def test_roll_k3_near_level_is_not_a_number(self):
        self.assertTrue(math.isnan(k3.roll_k3({"theta": 0.001, "a": -0.01})))


class MainTest(unittest.TestCase):

    def test_main_writes_csv_and_returns_zero(self):
        text = "".join(hold_text(th, accel_for(th), seed=i) for i, th in enumerate((0.2, -0.2, 0.3, -0.3)))
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "k3_run1.txt"
            path.write_text(text)
            with contextlib.redirect_stdout(io.StringIO()) as out:
                rc = k3.main([str(path)])
            self.assertEqual(rc, 0)
            self.assertTrue((Path(d) / k3.ROLLS_CSV).exists())
        self.assertIn("K3", out.getvalue())

    def test_main_fails_without_rolls(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "k3_run1.txt"
            path.write_text("# nothing\n")
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(k3.main([str(path)]), 1)


if __name__ == "__main__":
    unittest.main()
