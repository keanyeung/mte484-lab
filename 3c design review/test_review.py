"""Checks for the Lab 3(c)/(d) controller: the split form the sketch runs is the
groupmate's D2[z], anti-windup only acts in saturation, and the constants typed
into lab3d_outer_loop.ino are the ones design.py computes."""
import re
import unittest
from pathlib import Path

import numpy as np

import design as d
import sim

ROOT = Path(__file__).resolve().parent.parent
SKETCH = ROOT / "lab3d_outer_loop" / "lab3d_outer_loop.ino"         # Lab 3 (d)
SKETCH_E = ROOT / "lab3e_outer_loop" / "lab3e_outer_loop.ino"       # Lab 3 (e), final/demo
SKETCHES = (SKETCH, SKETCH_E)
C2N, C2D = d.outer_design(d.POLES)
B, A = d.tustin(C2N, C2D)


def sketch_constants(path=SKETCH):
    text = path.read_text()
    return {m.group(1): float(m.group(2)) for m in re.finditer(
        r"constexpr\s+(?:double|float|uint8_t|int)\s+(\w+)\s*=\s*([-+0-9.eE]+)f?\s*;", text)}


class SplitFormTests(unittest.TestCase):
    def test_tustin_matches_groupmate_matlab(self):
        np.testing.assert_allclose(B, [-72.20855777230655, 144.3396717001068, -72.13114775402674],
                                   rtol=0, atol=1e-12)
        np.testing.assert_allclose(A, [1, -1.992230301822891, 0.992230301822891], rtol=0, atol=1e-14)

    def test_split_without_aw_equals_d2(self):
        rng = np.random.default_rng(1)
        plain, split = sim.LinearD2(B, A), sim.SplitD2(C2N, C2D, aw=False)
        for e in rng.normal(0, 0.05, 5000):
            self.assertAlmostEqual(plain.step(e), split.step(e), delta=1e-9)

    def test_aw_identical_while_unsaturated(self):
        on, off = sim.SplitD2(C2N, C2D, aw=True), sim.SplitD2(C2N, C2D, aw=False)
        for e in np.full(2000, 0.002):            # |u| stays far below 0.7 rad
            self.assertEqual(on.step(e), off.step(e))

    def test_aw_freezes_integrator_when_pushing_into_saturation(self):
        c = sim.SplitD2(C2N, C2D, aw=True)
        for _ in range(3000):
            u = c.step(0.15)                      # large positive error: u << -0.7 the whole time
        self.assertLess(u, -sim.SAT)
        self.assertEqual(c.i, 0.0)                # never allowed to integrate

    def test_aw_integrates_when_it_pulls_out_of_saturation(self):
        c = sim.SplitD2(C2N, C2D, aw=True)
        c.i = 5.0                                 # wound up: u = -(...) far below -0.7
        c.step(-0.01)                             # this error moves i down, toward unsaturating
        self.assertLess(c.i, 5.0)


class SketchConstantTests(unittest.TestCase):
    """Run on both the (d) sketch and the final (e) sketch."""

    def test_sketches_exist(self):
        for path in SKETCHES:
            self.assertTrue(path.exists(), f"missing {path}")

    def test_split_constants_match_design(self):
        want = d.split_coefficients(C2N, C2D)
        for path in SKETCHES:
            got = sketch_constants(path)
            for name in ("KP", "KI", "LB", "LA"):
                with self.subTest(sketch=path.name, name=name):
                    self.assertIn(f"D2_{name}", got)
                    self.assertAlmostEqual(got[f"D2_{name}"], want[name], delta=1e-12)

    def test_inner_and_saturator_unchanged(self):
        for path in SKETCHES:
            got = sketch_constants(path)
            with self.subTest(sketch=path.name):
                self.assertEqual(got["D1_A0"], -2.454674639351807)
                self.assertEqual(got["D1_A1"], 2.284293610869540)
                self.assertEqual(got["D1_B1"], -0.967822458684839)
                self.assertEqual(got["REF_SAT"], 0.7)

    def test_ball_offset_uses_beam_end_origin(self):
        # Lab 2 offset -0.004055 + (measured 0.030 - ball radius 0.0127) = 0.013245
        for path in SKETCHES:
            m = re.search(r"BALL_OFFSET\s*=\s*([-+0-9.eE]+)f", path.read_text())
            with self.subTest(sketch=path.name):
                self.assertAlmostEqual(float(m.group(1)), -0.004055 + (0.030 - 0.0127), places=9)
                self.assertEqual(sketch_constants(path)["Y_MIN_BALL"], 0.030)


class FinalSketchTests(unittest.TestCase):
    """Lab 3 (e): the on-board monitor uses the manual's assessment values."""

    def test_e_assessment_constants(self):
        got = sketch_constants(SKETCH_E)
        self.assertEqual((got["E_LOW_MIN"], got["E_LOW_MAX"]), (0.094, 0.106))
        self.assertEqual((got["E_HIGH_MIN"], got["E_HIGH_MAX"]), (0.240, 0.260))
        self.assertEqual(got["E_SETTLE_S"], 8.5)
        self.assertEqual(got["E_OS_PCT"], 45.0)
        self.assertEqual(got["E_PERIODS"], 10)

    def test_control_irq_raised_above_usb(self):
        got = sketch_constants(SKETCH_E)
        self.assertLess(got["CONTROL_IRQ_PRIORITY"], 12)      # USB serial is 12 on the R4 core
        self.assertIn("raiseControlIrqPriority();", SKETCH_E.read_text())

    def test_anti_windup_on_by_default(self):
        self.assertRegex(SKETCH_E.read_text(), r"bool\s+awOn\s*=\s*true;")


if __name__ == "__main__":
    unittest.main()
