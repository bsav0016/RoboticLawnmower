"""Tests for scoring a driven track and for the sweep tool, including closed-loop runs in the simulator."""
import contextlib
import io
import os
import sys
import tempfile
import unittest
from dataclasses import replace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import sweep_plan  # noqa: E402
from lawnmower import scoring, sim  # noqa: E402
from lawnmower.field import Field  # noqa: E402
from lawnmower.sim import RoverParams  # noqa: E402

SQUARE = [(0, 0), (10, 0), (10, 10), (0, 10)]
ZONE = [(4, 4), (6, 4), (6, 6), (4, 6)]


def lane(x, y0=0, y1=10, step=0.1):
    return [(x, y0 + i * step) for i in range(int((y1 - y0) / step) + 1)]


class ScoringTests(unittest.TestCase):
    def test_inside_mask(self):
        mask = scoring.inside_mask([5, 15, -1, 5], [5, 5, 5, 11], SQUARE)
        self.assertEqual(list(mask), [True, False, False, False])

    def test_mowable_mask_leaves_out_zones(self):
        mask = scoring.mowable_mask([5, 1], [5, 1], SQUARE, [ZONE])
        self.assertEqual(list(mask), [False, True])

    def test_one_lane_cuts_its_share_of_the_square(self):
        # A lane 2 m wide down the middle of a 10 m square is 20% of it.
        self.assertAlmostEqual(scoring.coverage_pct(lane(5), 2.0, SQUARE), 20.0, delta=1.0)

    def test_lanes_side_by_side_cover_everything(self):
        track = [p for x in (1, 3, 5, 7, 9) for p in lane(x)]
        self.assertAlmostEqual(scoring.coverage_pct(track, 2.0, SQUARE), 100.0, delta=0.5)

    def test_a_gap_between_lanes_shows_as_uncut_grass(self):
        track = [p for x in (1, 5, 9) for p in lane(x)]   # lanes 4 m apart, 2 m wide: half is missed
        self.assertAlmostEqual(scoring.coverage_pct(track, 2.0, SQUARE), 60.0, delta=1.5)

    def test_grass_outside_the_boundary_does_not_count(self):
        self.assertAlmostEqual(scoring.coverage_pct(lane(0), 2.0, SQUARE), 10.0, delta=1.0)   # half the strip is outside

    def test_no_go_zones_are_not_mowable(self):
        track = [p for x in (1, 3, 5, 7, 9) for p in lane(x)]
        # Driving straight through the zone cuts all of the mowable grass, and the zone is not counted as missed.
        self.assertAlmostEqual(scoring.coverage_pct(track, 2.0, SQUARE, [ZONE]), 100.0, delta=0.5)

    def test_no_track_cuts_nothing(self):
        self.assertEqual(scoring.coverage_pct([], 1.0, SQUARE), 0.0)

    def test_excursions_are_zero_for_a_legal_track(self):
        self.assertEqual(scoring.excursions(lane(1), SQUARE, [ZONE]), (0.0, 0.0))

    def test_excursion_outside_the_boundary(self):
        outside, into_zone = scoring.excursions([(5, 5), (12, 5), (5, 13)], SQUARE)
        self.assertAlmostEqual(outside, 3.0)   # the point 3 m above the top edge
        self.assertEqual(into_zone, 0.0)

    def test_excursion_into_a_zone(self):
        outside, into_zone = scoring.excursions([(4.5, 5), (5, 5)], SQUARE, [ZONE])
        self.assertEqual(outside, 0.0)
        self.assertAlmostEqual(into_zone, 1.0)   # the centre of a 2 m zone is 1 m from its edge


SMALL = Field(boundary=((37.0, -122.0), (37.0, -122.00004), (37.00004, -122.00004), (37.00004, -122.0)),
              pass_spacing_m=1.0)   # about 3.6 m by 4.4 m


class SweepTests(unittest.TestCase):
    def test_scenarios_leave_out_turn_depths_bigger_than_the_inset(self):
        found = sweep_plan.scenarios_from([0, 1], [0, 1], [0.5], [2.5])
        self.assertEqual([(s.inset_m, s.turn_depth_m) for s in found], [(0, 0), (1, 0), (1, 1)])

    def test_gps_error_scales_the_noise_to_the_requested_total(self):
        noise = sweep_plan.noise_for(5.0)
        self.assertAlmostEqual((noise.gps_std_m ** 2 + noise.gps_bias_std_m ** 2) ** 0.5, 5.0)

    def test_a_perfect_run_on_a_plain_rectangle_finishes_and_cuts_most_of_it(self):
        rover = RoverParams(motor_lag_s=0)
        scenario = sweep_plan.Scenario(0, 0, 0.5, 0.0, 0)   # pivot turns, no GPS error
        orig = sweep_plan.noise_for
        sweep_plan.noise_for = lambda _: sim.NO_NOISE
        try:
            score, waypoints = sweep_plan.run_once(SMALL, scenario, 0, rover)
        finally:
            sweep_plan.noise_for = orig
        self.assertTrue(score.finished)
        self.assertEqual(score.not_reached, 0)
        self.assertGreater(score.coverage_pct, 75.0)
        self.assertLess(score.max_outside_m, 1.0)

    def test_the_same_seed_gives_the_same_table(self):
        scenarios = sweep_plan.scenarios_from([0, 1], [0], [0.5], [2.5])
        a = sweep_plan.sweep(SMALL, scenarios, 2)
        b = sweep_plan.sweep(SMALL, scenarios, 2)
        self.assertEqual(sweep_plan.table_rows(a, 95, 1), sweep_plan.table_rows(b, 95, 1))

    def test_a_different_seed_set_changes_the_numbers(self):
        scenario = [sweep_plan.Scenario(0, 0, 0.5, 2.5)]
        one = sweep_plan.sweep(SMALL, scenario, 1)[0].runs[0]
        two = sweep_plan.run_once(SMALL, scenario[0], 1)[0]
        self.assertNotEqual(one.coverage_pct, two.coverage_pct)

    def test_a_field_that_cannot_be_planned_that_way_is_reported_not_raised(self):
        rows = sweep_plan.sweep(SMALL, [sweep_plan.Scenario(5, 0, 0.5, 2.5)], 1)   # inset swallows the whole field
        self.assertIsNotNone(rows[0].error)
        self.assertIn("cannot plan", sweep_plan.table_rows(rows, 95, 1)[0][-1])

    def test_verdict_names_what_failed(self):
        row = sweep_plan.Row(sweep_plan.Scenario(0, 0, 0.5, 2.5), runs=[
            sweep_plan.RunScore(False, 50.0, 3.0, 0.0, 100.0, 4)])
        self.assertEqual(row.problems(95, 1), ["unfinished", "coverage", "excursion"])
        good = sweep_plan.Row(sweep_plan.Scenario(0, 0, 0.5, 2.5), runs=[
            sweep_plan.RunScore(True, 97.0, 0.2, 0.0, 100.0, 0)])
        self.assertEqual(good.problems(95, 1), [])

    def test_best_row_prefers_one_that_meets_the_targets(self):
        scenario = sweep_plan.Scenario
        ok = sweep_plan.Row(scenario(1, 0, 0.5, 2.5), runs=[sweep_plan.RunScore(True, 96.0, 0.5, 0.0, 100.0, 0)])
        more = sweep_plan.Row(scenario(0, 0, 0.5, 2.5), runs=[sweep_plan.RunScore(True, 99.0, 4.0, 0.0, 90.0, 0)])
        self.assertIs(sweep_plan.best_row([more, ok], 95, 1), ok)
        self.assertIs(sweep_plan.best_row([ok, more], 99.5, 1), more)   # nothing meets 99.5: most coverage wins

    def test_track_svg_draws_the_route_over_the_plan(self):
        score, _ = sweep_plan.run_once(SMALL, sweep_plan.Scenario(0, 0, 0.5, 2.5), 0, keep_track=True)
        svg = sweep_plan.track_svg(SMALL, score.track)
        self.assertIn('id="track"', svg)
        self.assertIn('id="path"', svg)

    def test_command_line_prints_a_table_and_writes_files(self):
        with tempfile.TemporaryDirectory() as folder:
            field_file = os.path.join(folder, "field.json")
            with open(field_file, "w", encoding="utf-8") as f:
                f.write('{"boundary": [[37.0, -122.0], [37.0, -122.00004], [37.00004, -122.00004], '
                        '[37.00004, -122.0]], "pass_spacing_m": 1.0}')
            csv_file, svg_file = os.path.join(folder, "out.csv"), os.path.join(folder, "best.svg")
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                sweep_plan.main(["--field", field_file, "--inset", "0,1", "--seeds", "2", "--csv", csv_file,
                                 "--svg", svg_file])
            text = out.getvalue()
            self.assertIn("inset_m", text)
            self.assertIn("cut_%", text)
            self.assertTrue(os.path.getsize(csv_file) > 0)
            with open(svg_file, encoding="utf-8") as f:
                self.assertIn('id="track"', f.read())

    def test_bad_number_lists_exit_with_a_message(self):
        with self.assertRaises(SystemExit):
            sweep_plan.parse_numbers("0,one", "--inset")


class PreviewTrackTests(unittest.TestCase):
    def test_without_a_track_the_svg_is_unchanged(self):
        from create_waypoint_list import plan_field
        from lawnmower.preview import plan_svg
        waypoints = plan_field(SMALL)
        self.assertNotIn('id="track"', plan_svg(SMALL, waypoints))
        self.assertEqual(plan_svg(SMALL, waypoints), plan_svg(replace(SMALL), waypoints, track=None))


if __name__ == "__main__":
    unittest.main()
