import math
import unittest

import analyze_lab3b as a


def synth_run(n_half=3, dt=0.01, tau=1.0, offset=0.0):
    """Square wave starting at Y_LOW; y follows each step as a first-order lag.
    offset shifts y away from the reference (a steady-state error)."""
    lines = ["# start square r=0.100<->0.250"]
    y = a.Y_LOW
    n = int(a.HALF_PERIOD_S / dt)
    for h in range(n_half):
        r = a.Y_LOW if h % 2 == 0 else a.Y_HIGH
        for k in range(n):
            t = (h * n + k) * dt
            y += (r + offset - y) * (dt / tau)
            lines.append(f"{int(round(t * 1000))},{r:.3f},0.1,0.1,0.1,0.5,{y:.5f}")
    return lines


class ParseTests(unittest.TestCase):
    def test_skips_messages_and_bad_lines(self):
        lines = ["# geeWhiz Started", "1,2,3", "# start square", "10,0.100,0,0,0,0,0.1",
                 "garbage", "# stop", "20,0.100,0,0,0,0,0.1"]
        runs = a.parse_stream(lines)
        self.assertEqual(len(runs), 1)
        self.assertEqual(len(runs[0]), 1)   # the line after "# stop" is idle stream
        self.assertAlmostEqual(runs[0][0][0], 0.010)

    def test_run_ends_at_stop_and_limit(self):
        lines = ["# start a", "0,0.1,0,0,0,0,0.1", "# stop LIMIT: |theta| above 0.80",
                 "10,0.1,0,0,0,0,0.1", "# start b", "0,0.1,0,0,0,0,0.1", "# stop, samples=1"]
        self.assertEqual([len(r) for r in a.parse_stream(lines)], [1, 1])

    def test_new_run_at_each_start(self):
        lines = ["# start a", "0,0.1,0,0,0,0,0.1", "# start b", "0,0.1,0,0,0,0,0.1"]
        self.assertEqual(len(a.parse_stream(lines)), 2)

    def test_data_before_first_start_ignored(self):
        self.assertEqual(a.parse_stream(["0,0.1,0,0,0,0,0.1"]), [])


class MetricTests(unittest.TestCase):
    def test_edges_found_at_reference_changes(self):
        run = a.parse_stream(synth_run(n_half=3))[0]
        self.assertEqual(len(a.find_edges(run)), 2)

    def test_first_order_settling_and_no_overshoot(self):
        run = a.parse_stream(synth_run(n_half=2, tau=1.0))[0]
        s = a.analyze_run(run)[0]
        self.assertEqual(s["dir"], "+")
        self.assertAlmostEqual(s["overshoot_pct"], 0.0, places=6)
        self.assertAlmostEqual(s["settling_s"], -math.log(0.02), delta=0.05)  # 3.91 s
        self.assertAlmostEqual(s["ss_error_mm"], 0.0, delta=0.1)
        self.assertTrue(s["complete"])

    def test_down_step_direction_and_error_sign(self):
        run = a.parse_stream(synth_run(n_half=3, offset=0.005))[0]
        down = a.analyze_run(run)[1]
        self.assertEqual(down["dir"], "-")
        self.assertAlmostEqual(down["ss_error_mm"], -5.0, delta=0.2)

    def test_never_settles_gives_none(self):
        run = a.parse_stream(synth_run(n_half=2, offset=0.01))[0]
        self.assertIsNone(a.analyze_run(run)[0]["settling_s"])

    def test_saturation_fraction(self):
        lines = ["# start", "0,0.1,0,0,0,0,0.1", "10,0.25,0.9,0.7,0,0,0.1",
                 "20,0.25,0.5,0.5,0,0,0.1"]
        s = a.analyze_run(a.parse_stream(lines)[0])[0]
        self.assertAlmostEqual(s["sat_pct"], 50.0)
        self.assertFalse(s["complete"])


class HoldTests(unittest.TestCase):
    def test_rest_position_and_error(self):
        lines = ["# start hold r=0.175"]
        for k in range(1000):   # 10 s, ball released at 0.30, resting at 0.120 after 5 s
            y = 0.30 if k == 0 else (0.20 if k < 500 else 0.120)
            lines.append(f"{k * 10},0.175,-0.05,-0.05,-0.06,0.2,{y}")
        run = a.parse_stream(lines)[0]
        self.assertEqual(a.analyze_run(run), [])
        h = a.hold_summary(run)
        self.assertAlmostEqual(h["e_mm"], 55.0, places=3)
        self.assertAlmostEqual(h["theta"], -0.06)
        self.assertAlmostEqual(h["start_y"], 0.30)
        self.assertTrue(h["still"])

    def test_moving_ball_flagged(self):
        lines = ["# start hold r=0.175"] + [f"{k * 10},0.175,0,0,0,0,{0.1 + k * 1e-4}"
                                            for k in range(500)]
        self.assertFalse(a.hold_summary(a.parse_stream(lines)[0])["still"])


class OverlayTests(unittest.TestCase):
    def test_sim_aligned_at_hardware_up_edge(self):
        run = a.parse_stream(synth_run(n_half=3))[0]
        period = [(20.0 + k * 0.01, 0, 1000.0 + k, 0.0) for k in range(2000)]
        rows = a.overlay(run, period)
        self.assertAlmostEqual(rows[0][0], 10.0, places=2)   # starts at the up edge
        self.assertAlmostEqual(rows[0][3], 1000.0)           # sim period sample 0
        self.assertAlmostEqual(rows[100][3], 1100.0)         # 1 s later

    def test_sim_scored_like_hardware(self):
        # sim starts HIGH like mte484lab3partb.slx: first edge at 10 s is a down step
        sim = [(k * 0.01, a.Y_HIGH if k < 1000 or k >= 2000 else a.Y_LOW, a.Y_HIGH, 0.0)
               for k in range(3000)]
        steps = a.analyze_run(a.sim_as_run(sim))
        self.assertEqual([s["dir"] for s in steps], ["-", "+"])
        self.assertAlmostEqual(steps[1]["ss_error_mm"], 0.0)

    def test_no_sim_no_overlay(self):
        run = a.parse_stream(synth_run(n_half=2))[0]
        self.assertEqual(a.overlay(run, []), [])


if __name__ == "__main__":
    unittest.main()
