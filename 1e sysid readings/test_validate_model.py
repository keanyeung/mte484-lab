"""Tests for validate_model.py using synthetic closed-loop step responses.

A capture built from a known (K1, tau) must be reproduced by the simulation
of that same plant to within the ADC quantization, and a deliberately wrong
K1 must show up as a clearly larger error. The synthetic helpers come from
test_analyze_sysid so both suites describe the same plant.

Usage (from this folder):
    python -m unittest test_validate_model -v
"""
import math
import unittest

import analyze_sysid as sysid
import validate_model as vm
from test_analyze_sysid import K1_TRUE, TAU_TRUE, capture_text, step_thetas

QUANT_RAD = 3.645179e-4     # one ADC count in rad, the floor on any error here
DELAY_SAMPLES = 20          # injected dead time for the delay test [samples at 1 ms]


# ---------------------------------------------------------------- helpers
def synthetic(kp, direction=1, thetas=None, **kwargs):
    """(capture, record) for one ideal step of the true plant."""
    if thetas is None:
        thetas = step_thetas(K1_TRUE, TAU_TRUE, kp, direction, **kwargs)
    cap = sysid.parse_captures(capture_text(1, kp, direction, thetas), "synthetic")[0]
    return cap, sysid.analyse_step(cap)


def compare(kp, k1=K1_TRUE, tau=TAU_TRUE, direction=1, thetas=None, **kwargs):
    cap, rec = synthetic(kp, direction, thetas, **kwargs)
    return vm.compare_step(cap, rec, "all", k1, tau)


def delayed_thetas(kp, direction=1, n_delay=DELAY_SAMPLES):
    """The true step, held at its starting angle for n_delay extra samples."""
    thetas = step_thetas(K1_TRUE, TAU_TRUE, kp, direction)
    return [thetas[0]] * n_delay + thetas[:-n_delay]


# ---------------------------------------------------------------- tests
class TestModel(unittest.TestCase):

    def test_closed_loop_inverts_plant_from_closed_loop(self):
        zeta, wn = vm.closed_loop(K1_TRUE, TAU_TRUE, -20.0)
        k1, tau = sysid.plant_from_closed_loop(zeta, wn, -20.0)
        self.assertAlmostEqual(k1, K1_TRUE, places=6)
        self.assertAlmostEqual(tau, TAU_TRUE, places=6)

    def test_unstable_sign_combination_is_rejected(self):
        with self.assertRaises(ValueError):
            vm.closed_loop(K1_TRUE, TAU_TRUE, +20.0)     # Kp and K1 must share a sign

    def test_unit_step_starts_at_zero_and_settles_at_one(self):
        zeta, wn = vm.closed_loop(K1_TRUE, TAU_TRUE, -20.0)
        self.assertEqual(vm.unit_step(zeta, wn, 0.0), 0.0)
        self.assertAlmostEqual(vm.unit_step(zeta, wn, 5.0), 1.0, places=6)

    def test_critically_and_over_damped_never_overshoot(self):
        for zeta in (1.0, 1.5, 4.0):
            with self.subTest(zeta=zeta):
                peak = max(vm.unit_step(zeta, 40.0, i * 0.001) for i in range(600))
                self.assertLessEqual(peak, 1.0 + 1e-9)
                self.assertEqual(vm.os_from_zeta(zeta), 0.0)
                self.assertTrue(math.isnan(vm.tp_from(zeta, 40.0)))

    def test_unit_step_peaks_at_the_predicted_overshoot_and_time(self):
        zeta, wn = vm.closed_loop(K1_TRUE, TAU_TRUE, -25.0)
        tp = vm.tp_from(zeta, wn)
        self.assertAlmostEqual(vm.unit_step(zeta, wn, tp), 1.0 + vm.os_from_zeta(zeta) / 100.0,
                               places=6)

    def test_onset_time_reaches_the_onset_fraction(self):
        zeta, wn = vm.closed_loop(K1_TRUE, TAU_TRUE, -20.0)
        t = vm.onset_time(zeta, wn)
        self.assertAlmostEqual(vm.unit_step(zeta, wn, t), sysid.ONSET_FRAC, places=6)


class TestReproduction(unittest.TestCase):

    def test_simulation_reproduces_its_own_plant(self):
        for kp in (-15.0, -20.0, -25.0):
            for direction in (1, -1):
                with self.subTest(kp=kp, direction=direction):
                    row = compare(kp, direction=direction)
                    self.assertLess(row["rms_rad"], 2.0 * QUANT_RAD, row)
                    self.assertLess(row["peak_rad"], 5.0 * QUANT_RAD, row)

    def test_predicted_overshoot_and_peak_time_match_the_measurement(self):
        row = compare(-20.0)
        self.assertAlmostEqual(row["os_model"], row["os_meas"], delta=0.3)
        self.assertAlmostEqual(row["tp_ms_model"], row["tp_ms_meas"], delta=1.0)

    def test_an_ideal_capture_shows_no_dead_time(self):
        row = compare(-20.0)
        self.assertLess(row["dead_ms"], 2.0, row)
        self.assertAlmostEqual(row["onset_ms"], row["onset_ms_model"], delta=2.0)

    def test_noise_is_reported_and_stays_near_the_quantization_floor(self):
        clean = compare(-20.0)
        noisy = compare(-20.0, noise=0.002)
        self.assertLess(clean["noise_rad"], 2.0 * QUANT_RAD)
        self.assertGreater(noisy["noise_rad"], 5.0 * QUANT_RAD)


class TestErrorGrowsWithABadFit(unittest.TestCase):

    def test_wrong_k1_produces_a_clearly_larger_error(self):
        good = compare(-20.0)
        bad = compare(-20.0, k1=1.5 * K1_TRUE)
        self.assertGreater(bad["rms_rad"], 10.0 * good["rms_rad"], (good, bad))
        self.assertGreater(bad["rms_pct_step"], 1.0)

    def test_wrong_tau_produces_a_clearly_larger_error(self):
        good = compare(-20.0)
        bad = compare(-20.0, tau=1.5 * TAU_TRUE)
        self.assertGreater(bad["rms_rad"], 10.0 * good["rms_rad"], (good, bad))

    def test_wrong_k1_also_mispredicts_overshoot_and_peak_time(self):
        bad = compare(-20.0, k1=1.5 * K1_TRUE)
        self.assertGreater(abs(bad["os_model"] - bad["os_meas"]), 2.0, bad)
        self.assertGreater(abs(bad["tp_ms_model"] - bad["tp_ms_meas"]), 5.0, bad)


class TestDeadTime(unittest.TestCase):

    def test_injected_delay_is_reported_as_dead_time(self):
        row = compare(-20.0, thetas=delayed_thetas(-20.0))
        self.assertAlmostEqual(row["dead_ms"], DELAY_SAMPLES, delta=3.0)

    def test_delaying_the_model_removes_most_of_that_error(self):
        row = compare(-20.0, thetas=delayed_thetas(-20.0))
        self.assertLess(row["rms_shift_rad"], 0.2 * row["rms_rad"], row)

    def test_dead_time_is_never_negative(self):
        # A capture that moves earlier than the model still reports zero, not a lead
        thetas = step_thetas(K1_TRUE, TAU_TRUE, -20.0, 1)[10:] + [0.1] * 10
        self.assertGreaterEqual(compare(-20.0, thetas=thetas)["dead_ms"], 0.0)


class TestRowsAndFits(unittest.TestCase):

    def build(self, kps=(-15.0, -20.0, -25.0)):
        caps, recs = [], []
        for i, kp in enumerate(kps):
            for direction in (1, -1):
                cap, rec = synthetic(kp, direction)
                cap["id"] = rec["id"] = 2 * i + (direction < 0)
                caps.append(cap)
                recs.append(rec)
        return caps, recs

    def test_every_step_is_validated_against_both_fits(self):
        caps, recs = self.build()
        rows = vm.validate(caps, recs, sysid.summarise(recs))
        self.assertEqual(len(rows), 2 * len(caps))
        self.assertEqual({r["fit"] for r in rows}, {"all", "kp"})

    def test_fits_from_reads_the_summary_rows(self):
        _, recs = self.build()
        overall, per_gain = vm.fits_from(sysid.summarise(recs))
        self.assertAlmostEqual(overall[0], K1_TRUE, delta=0.05)
        self.assertAlmostEqual(overall[1], TAU_TRUE, delta=0.001)
        self.assertEqual(sorted(per_gain), [-25.0, -20.0, -15.0])

    def test_excluded_steps_are_not_validated(self):
        caps, recs = self.build(kps=(-20.0,))
        recs[0]["exclude"] = "saturated"
        rows = vm.validate(caps, recs, sysid.summarise(recs))
        self.assertEqual({r["id"] for r in rows}, {recs[1]["id"]})

    def test_summary_groups_by_fit_and_gain(self):
        caps, recs = self.build()
        groups = vm.summarise(vm.validate(caps, recs, sysid.summarise(recs)))
        self.assertEqual(len(groups), 8)             # 3 gains + all, for each of 2 fits
        overall = [g for g in groups if g["fit"] == "all" and g["kp"] == ""][0]
        self.assertEqual(overall["n"], 6)
        self.assertLess(overall["rms_rad"], 2.0 * QUANT_RAD)

    def test_validation_rows_carry_every_declared_field(self):
        row = compare(-20.0)
        self.assertEqual(sorted(set(vm.VALIDATION_FIELDS) - set(row)), [])


if __name__ == "__main__":
    unittest.main()
