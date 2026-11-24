import math
import unittest

import analyze_lab3d as a


def square_lines(aw="on", n_half=3, dt=0.01, tau=1.0, offset=0.0):
    """One square-wave run starting at Y_LOW; y follows each step as a first-order
    lag. offset shifts y away from the reference (a steady-state error)."""
    lines = [f"# start square r=0.100<->0.250 half_period_ms=10000 ctrl=D2c+D1 aw={aw} dt_ms=1"]
    y = 0.10
    n = int(10.0 / dt)
    for h in range(n_half):
        r = 0.10 if h % 2 == 0 else 0.25
        for k in range(n):
            t = (h * n + k) * dt
            y += (r + offset - y) * (dt / tau)
            lines.append(f"{int(round(t * 1000))},{r:.3f},0.1,0.1,0.1,0.5,{y:.5f}")
    lines.append("# stop, samples=1 saturated=0 aw_held=0 beyond_DA=0")
    return lines


class ParseTests(unittest.TestCase):
    def test_reads_aw_and_kind_from_start_lines(self):
        lines = square_lines("on") + ["# hold set"] + [
            "# start hold r=0.175 ctrl=D2c+D1 aw=off dt_ms=1", "0,0.175,0,0,0,0,0.17",
            "10,0.175,0,0,0,0,0.17", "# stop, samples=2"]
        runs = a.parse_runs(lines)
        self.assertEqual([(m["kind"], m["aw"]) for m, _ in runs], [("square", "on"), ("hold", "off")])
        self.assertEqual(len(runs[1][1]), 2)

    def test_corrupted_reference_line_dropped(self):
        lines = square_lines()
        lines[1500] = lines[1500].replace(",0.250,", ",0.0100,")   # serial glitch in a high half
        meta, run = a.parse_runs(lines)[0]
        self.assertEqual(meta["dropped"], 1)
        self.assertEqual(len(a.find_edges_safe(run)), 2)
        self.assertTrue(meta["stopped"])

    def test_stream_gap_found_and_step_excluded(self):
        lines = square_lines(n_half=3)
        del lines[1501:1651]                              # 1.5 s missing inside the up step (10-20 s)
        run = a.parse_runs(lines)[0][1]
        gaps = a.stream_gaps(run)
        self.assertEqual(len(gaps), 1)
        self.assertAlmostEqual(gaps[0][1] - gaps[0][0], 1.51, places=2)
        steps = a.b.analyze_run(run)
        self.assertEqual(a.steps_hit_by_gaps(steps, gaps, run[-1][0]), {0})

    def test_no_gap_in_clean_run(self):
        self.assertEqual(a.stream_gaps(a.parse_runs(square_lines())[0][1]), [])

    def test_missing_stop_line_flagged(self):
        meta, _ = a.parse_runs(square_lines()[:-1])[0]
        self.assertFalse(meta["stopped"])

    def test_old_sketch_runs_have_unknown_aw(self):
        runs = a.parse_runs(["# start square r=0.100<->0.250 ctrl=D2+D1", "0,0.1,0,0,0,0,0.1"])
        self.assertEqual(runs[0][0]["aw"], "?")


class VerdictTests(unittest.TestCase):
    def test_passing_run(self):
        run = a.parse_runs(square_lines(tau=1.0))[0][1]
        v = a.verdict(a.b.analyze_run(run))
        self.assertTrue(v["ts_ok"] and v["os_ok"] and v["ss_ok"])
        self.assertAlmostEqual(v["worst_ts"], -math.log(0.02), delta=0.05)

    def test_offset_fails_error_and_settling(self):
        run = a.parse_runs(square_lines(offset=0.01))[0][1]
        v = a.verdict(a.b.analyze_run(run))
        self.assertFalse(v["ss_ok"])
        self.assertFalse(v["ts_ok"])          # never enters the band -> worst settling = never
        self.assertIsNone(v["worst_ts"])

    def test_slow_run_fails_settling_only(self):
        run = a.parse_runs(square_lines(tau=2.0))[0][1]   # 2 % settling ~7.8 s
        v = a.verdict(a.b.analyze_run(run))
        self.assertFalse(v["ts_ok"])
        self.assertTrue(v["os_ok"])

    def test_incomplete_steps_ignored(self):
        run = a.parse_runs(square_lines()[:-500])[0][1]
        v = a.verdict(a.b.analyze_run(run))
        self.assertEqual(v["n"], 1)


class SimTests(unittest.TestCase):
    OURS = ["t_s,r,y_aw_off,y_aw_on,y_aw_off_stick,y_aw_on_stick",
            "0.00,0.25,0.10,0.11,0.12,0.13", "0.01,0.25,0.20,0.21,0.22,0.23"]
    THEIRS = ["Time_s,Reference_m,Y_Continuous_m,Y_Discrete_m,Theta_Continuous_rad",
              "0,0.25,0.1,0.1,0", "0.01,0.25,0.2,0.3,0"]

    def test_loads_our_export(self):
        sim = a.load_sim_lines(self.OURS)
        self.assertEqual(sim["y_aw_on"], [0.11, 0.21])

    def test_loads_groupmate_export_as_simulink(self):
        sim = a.load_sim_lines(self.THEIRS)
        self.assertEqual(sim["y_simulink"], [0.1, 0.3])     # the discrete column
        self.assertEqual(sim["r"], [0.25, 0.25])

    def test_column_choice_follows_aw(self):
        both = {**a.load_sim_lines(self.OURS), "y_simulink": [0, 0]}
        self.assertEqual(a.sim_columns(both, "on"), ("y_aw_on", "y_aw_on_stick"))
        self.assertEqual(a.sim_columns(both, "off"), ("y_simulink", "y_aw_off_stick"))
        self.assertEqual(a.sim_columns(a.load_sim_lines(self.OURS), "off"), ("y_aw_off", "y_aw_off_stick"))

    def test_overlay_uses_matching_column(self):
        run = a.parse_runs(square_lines(n_half=3))[0][1]
        sim = {"t_s": [20 + k * 0.01 for k in range(2000)], "r": [0.25] * 2000,
               "y_aw_on": [1.0] * 2000, "y_aw_on_stick": [2.0] * 2000,
               "y_aw_off": [3.0] * 2000, "y_aw_off_stick": [4.0] * 2000}
        rows = a.overlay(run, sim, "on")
        self.assertEqual((rows[0][3], rows[0][4]), (1.0, 2.0))
        self.assertAlmostEqual(rows[0][0], 10.0, places=2)


if __name__ == "__main__":
    unittest.main()
