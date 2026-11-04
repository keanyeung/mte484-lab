"""Tests for analyze_ball_scaling.py using synthetic '# point' lines.

Each synthetic point comes from a known linear sensor (counts = (y - offset) / m),
printed in the lab2_ball_sensor format, so the fit must recover m and offset.

Usage (from this folder):
    py -3 -m unittest test_analyze_ball_scaling -v
"""
import contextlib
import io
import tempfile
import unittest
from pathlib import Path

import analyze_ball_scaling as ball

M_TRUE = -2.6e-5          # m per count (raw falls as y grows, like the gear sensor)
OFFSET_TRUE = 0.43        # m
Y_POINTS = [0.0127, 0.0627, 0.1127, 0.1627, 0.2127, 0.2627, 0.3127, 0.3627, 0.4043]


def raw_for(y):
    return (y - OFFSET_TRUE) / M_TRUE


def point_line(n, y, raw, sd=2.0, theta=-0.2):
    return (f"# point {n} y={y:.4f} raw_median={raw:.1f} raw_mean={raw:.2f} "
            f"raw_sd={sd:.2f} n=500 theta={theta:.4f}")


def capture(points):
    """points: list of (y, raw). Two passes are just a longer list."""
    lines = ["# geeWhiz Started - ball sensor calibration", "1234,8000,-0.2000"]
    lines += [point_line(i, y, raw) for i, (y, raw) in enumerate(points, start=1)]
    return "\n".join(lines) + "\n"


def two_passes(shift=0.0):
    forward = [(y, raw_for(y)) for y in Y_POINTS]
    reverse = [(y, raw_for(y) + shift) for y in reversed(Y_POINTS)]
    return forward + reverse


class ParseTest(unittest.TestCase):

    def test_reads_every_point_and_ignores_other_lines(self):
        pts = ball.parse_points(io.StringIO(capture(two_passes())))
        self.assertEqual(len(pts), 18)
        self.assertAlmostEqual(pts[0]["y"], 0.0127)
        self.assertAlmostEqual(pts[0]["raw"], round(raw_for(0.0127), 1), places=6)
        self.assertEqual(pts[0]["n"], 500)

    def test_integer_median_is_accepted(self):
        text = "# point 1 y=0.1000 raw_median=8123 raw_mean=8123.40 raw_sd=1.20 n=500 theta=-0.1000\n"
        self.assertEqual(ball.parse_points(io.StringIO(text))[0]["raw"], 8123.0)


class FitTest(unittest.TestCase):

    def test_recovers_slope_and_offset(self):
        pts = ball.parse_points(io.StringIO(capture(two_passes())))
        m, offset, r2 = ball.fit(pts)
        self.assertAlmostEqual(m, M_TRUE, delta=1e-9)
        self.assertAlmostEqual(offset, OFFSET_TRUE, delta=1e-5)
        self.assertGreater(r2, 0.99999)

    def test_residuals_are_in_millimetres(self):
        pts = ball.parse_points(io.StringIO(capture(two_passes())))
        m, offset, _ = ball.fit(pts)
        res = ball.residuals_mm(pts, m, offset)
        self.assertEqual(len(res), 18)
        self.assertLess(max(abs(r) for r in res), 0.01)

    def test_a_misplaced_point_is_flagged(self):
        points = two_passes()
        points[4] = (points[4][0], raw_for(points[4][0] + 0.008))   # ball 8 mm off its mark
        pts = ball.parse_points(io.StringIO(capture(points)))
        m, offset, _ = ball.fit(pts)
        flagged = ball.flag_points(pts, ball.residuals_mm(pts, m, offset))
        self.assertIn(pts[4]["index"], [p["index"] for p in flagged])

    def test_needs_two_distinct_positions(self):
        pts = ball.parse_points(io.StringIO(capture([(0.1, 8000.0), (0.1, 8001.0)])))
        with self.assertRaises(ValueError):
            ball.fit(pts)


    def test_identical_readings_raise_a_clear_error(self):
        pts = ball.parse_points(io.StringIO(capture([(0.1, 8000.0), (0.3, 8000.0)])))
        with self.assertRaises(ValueError):
            ball.fit(pts)


class RepeatabilityTest(unittest.TestCase):

    def test_pass_difference_per_position_in_mm(self):
        # second pass reads 40 counts higher everywhere: 40 * 2.6e-5 m = 1.04 mm
        pts = ball.parse_points(io.StringIO(capture(two_passes(shift=40.0))))
        m, _, _ = ball.fit(pts)
        rep = ball.repeatability_mm(pts, m)
        self.assertEqual(len(rep), 9)
        for y, spread in rep.items():
            self.assertAlmostEqual(spread, 1.04, delta=0.01)

    def test_noise_converted_to_mm(self):
        pts = ball.parse_points(io.StringIO(capture(two_passes())))
        self.assertAlmostEqual(ball.noise_mm(pts[0], M_TRUE), 2.0 * 2.6e-5 * 1000, places=6)


class CorrectionsTest(unittest.TestCase):
    """Labels typed at the bench can be corrected, and points shown to be
    misplaced can be excluded, without editing the raw captures."""

    def points(self):
        return ball.parse_points(io.StringIO(capture(two_passes())))

    def test_relabel_changes_only_the_matching_y(self):
        kept, notes = ball.apply_corrections(self.points(), relabel={0.4043: 0.3693}, exclude=set())
        self.assertEqual(sum(p["y"] == 0.3693 for p in kept), 2)
        self.assertEqual(sum(p["y"] == 0.4043 for p in kept), 0)
        self.assertEqual(len(kept), 18)
        self.assertTrue(any("0.4043" in n for n in notes))

    def test_exclude_by_source_and_point_number(self):
        pts = self.points()
        for p in pts:
            p["source"] = "ball_cal2.txt"
        kept, notes = ball.apply_corrections(pts, relabel={}, exclude={("ball_cal2.txt", 3)})
        self.assertEqual(len(kept), 17)
        self.assertNotIn(3, [p["index"] for p in kept])
        self.assertTrue(any("ball_cal2.txt" in n and "3" in n for n in notes))

    def test_load_points_records_the_source_file(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "ball_cal9.txt"
            path.write_text(capture(two_passes()))
            pts = ball.load_points([path])
        self.assertEqual({p["source"] for p in pts}, {"ball_cal9.txt"})


class MainTest(unittest.TestCase):

    def test_prints_code_constants_and_returns_zero(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "ball_cal.txt"
            path.write_text(capture(two_passes()))
            with contextlib.redirect_stdout(io.StringIO()) as out:
                rc = ball.main([str(path)])
        self.assertEqual(rc, 0)
        self.assertIn("BALL_M", out.getvalue())
        self.assertIn("BALL_OFFSET", out.getvalue())

    def test_fails_without_points(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "empty.txt"
            path.write_text("# nothing here\n")
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(ball.main([str(path)]), 1)


if __name__ == "__main__":
    unittest.main()
