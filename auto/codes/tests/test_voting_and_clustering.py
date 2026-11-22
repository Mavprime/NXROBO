import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _bootstrap import match_logic  # noqa: E402


def frame(*detections):
    """One detection frame: (raw_id, score, cx, cy) tuples."""
    return list(detections)


class TestTemporalVotingAndClustering(unittest.TestCase):
    """cluster_detections() is the production voting/clustering used on the robot."""

    def test_single_object_with_jitter_and_class_noise(self):
        """One cube, +-5 px jitter and 2/10 misclassified frames -> one cluster."""
        frames = [
            frame((2, 0.85, 321, 239)),
            frame((2, 0.82, 319, 241)),
            frame((3, 0.48, 320, 240)),   # class noise
            frame((2, 0.90, 322, 238)),
            frame((2, 0.88, 318, 242)),
            frame((2, 0.91, 320, 240)),
            frame((3, 0.52, 321, 239)),   # class noise
            frame((2, 0.89, 320, 240)),
            frame((2, 0.86, 319, 241)),
            frame((2, 0.93, 320, 240)),
        ]
        result = match_logic.cluster_detections(frames)

        self.assertEqual(len(result), 1, "must collapse to one physical cube")
        cube_id, (x, y, _) = result[0]
        self.assertEqual(cube_id, match_logic.CAKE_ID, "majority vote must pick the cake")
        self.assertAlmostEqual(x, 320, delta=2)
        self.assertAlmostEqual(y, 240, delta=2)

    def test_two_distinct_spatial_objects(self):
        """Two cubes 300 px apart stay separate."""
        frames = [frame((1, 0.9, 150, 200), (3, 0.9, 450, 200)) for _ in range(8)]
        result = match_logic.cluster_detections(frames)

        self.assertEqual(len(result), 2)
        self.assertEqual({cube[0] for cube in result},
                         {match_logic.TEDDY_ID, match_logic.CLOCK_ID})

    def test_transient_ghost_detection_rejected(self):
        """A cube seen in a single frame out of ten must not survive."""
        frames = [frame((1, 0.9, 100, 100)) for _ in range(5)]
        frames.append(frame((4, 0.50, 500, 500)))  # one-frame ghost
        frames.extend(frame((1, 0.9, 101, 100)) for _ in range(4))
        result = match_logic.cluster_detections(frames)

        self.assertEqual(len(result), 1)
        self.assertEqual(result[0][0], match_logic.TEDDY_ID)

    def test_low_confidence_detections_filtered(self):
        frames = [frame((2, 0.30, 200, 200)) for _ in range(10)]
        self.assertEqual(match_logic.cluster_detections(frames), [])

    def test_cross_model_duplicate_in_one_frame_merges(self):
        """The detector runs two YOLO models: the same cube must appear once."""
        frames = [
            frame((1, 0.91, 300, 200), (1, 0.62, 303, 198)),  # model_1 + model_2
            frame((1, 0.88, 299, 201), (1, 0.58, 302, 200)),
            frame((1, 0.90, 301, 200), (1, 0.60, 304, 199)),
            frame((1, 0.87, 300, 202)),
        ]
        result = match_logic.cluster_detections(frames)

        self.assertEqual(len(result), 1, "duplicate boxes must merge into one cube")
        self.assertEqual(result[0][0], match_logic.TEDDY_ID)

    def test_single_frame_mode_still_works(self):
        """A one-frame window (num_samples=1) keeps the old behaviour."""
        result = match_logic.cluster_detections([frame((3, 0.8, 100, 100))])
        self.assertEqual(len(result), 1)

    def test_empty_frames_produce_nothing(self):
        self.assertEqual(match_logic.cluster_detections([]), [])
        self.assertEqual(match_logic.cluster_detections([[], [], []]), [])

    def test_min_hits_scales_with_window(self):
        """2 frames, cube in both -> accepted; 1 of 4 -> rejected."""
        self.assertEqual(len(match_logic.cluster_detections(
            [frame((1, 0.9, 100, 100)), frame((1, 0.9, 101, 101))])), 1)
        scattered = [frame((1, 0.9, 100, 100)), [], [], []]
        self.assertEqual(match_logic.cluster_detections(scattered), [])


class TestCakeAntiGlareGate(unittest.TestCase):
    """The cake has FSM priority, so a cheap false positive must never pass.

    Live measurements on the robot (394 cake boxes over 20 s):
      * raw id 2  ("cake", plain class): false positives on banana cubes up to
        0.852; the real field cake is NOT reported by this class.
      * raw id 20 ("cake_site", what the real Cake cube produces): real cake
        0.440..0.873, false cake_site (clock faces, banana cubes) 0.030..0.527.
    Hence two separate gates: strict for id 2, 0.60 for id 20.
    """

    def test_persistent_low_confidence_cake_is_rejected(self):
        frames = [frame((2, 0.55, 320, 200)) for _ in range(10)]
        self.assertEqual(match_logic.cluster_detections(frames), [],
                         "a 0.55 cake glare must not become a target")

    def test_live_glare_score_is_rejected(self):
        """Live field data: false cake(id 2) on a banana cube reached 0.852."""
        frames = [frame((2, 0.852, 353, 292)) for _ in range(10)]
        self.assertEqual(match_logic.cluster_detections(frames), [],
                         "the measured 0.852 false cake must not become a target")

    def test_high_confidence_cake_id2_is_kept(self):
        frames = [frame((2, 0.92, 320, 200)) for _ in range(10)]
        result = match_logic.cluster_detections(frames)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0][0], match_logic.CAKE_ID)

    def test_low_confidence_cake_does_not_relabel_a_real_cube(self):
        """A false cake on top of a teddy must not force the class to Cake."""
        frames = [frame((1, 0.90, 320, 200), (2, 0.45, 322, 201))
                  for _ in range(10)]
        result = match_logic.cluster_detections(frames)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0][0], match_logic.TEDDY_ID,
                         "a 0.45 cake hypothesis must not hijack the teddy")

    def test_cake_id2_gate_is_configurable(self):
        frames = [frame((2, 0.50, 320, 200)) for _ in range(10)]
        result = match_logic.cluster_detections(frames,
                                                cake_conf_threshold=0.40)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0][0], match_logic.CAKE_ID)

    # --- cake_site (id 20): the class that actually sees the field Cake -----

    def test_real_cake_site_is_kept(self):
        """Measured real-cake cake_site scores pass the 0.60 gate."""
        for score in (0.62, 0.70, 0.87):
            frames = [frame((20, score, 340, 160)) for _ in range(10)]
            result = match_logic.cluster_detections(frames)
            self.assertEqual(len(result), 1, "real cake_site %.2f dropped" % score)
            self.assertEqual(result[0][0], match_logic.CAKE_ID)

    def test_false_cake_site_is_rejected(self):
        """Measured false cake_site (clock face) goes up to 0.527."""
        frames = [frame((20, 0.527, 506, 349)) for _ in range(10)]
        self.assertEqual(match_logic.cluster_detections(frames), [])

    def test_cake_site_gate_is_configurable(self):
        frames = [frame((20, 0.55, 340, 160)) for _ in range(10)]
        result = match_logic.cluster_detections(
            frames, cake_site_conf_threshold=0.50)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0][0], match_logic.CAKE_ID)

    def test_measured_thresholds(self):
        """Guards against silently lowering the gates below the measured noise."""
        self.assertGreater(match_logic.CAKE_CONF_THRESHOLD, 0.852,
                           "id-2 gate must stay above the measured 0.852 noise")
        self.assertGreater(match_logic.CAKE_SITE_CONF_THRESHOLD, 0.527,
                           "id-20 gate must stay above the measured 0.527 noise")
        self.assertLessEqual(match_logic.CAKE_SITE_CONF_THRESHOLD, 0.70,
                             "id-20 gate must stay below the real cake's "
                             "typical 0.70+ score")


class TestCakeCubeFaceVeto(unittest.TestCase):
    """An ordinary cube's white front face must never be taken for the Cake.

    Live bug report: on a field WITHOUT the cake the robot recognised a banana
    cube's side as the cake.  Measured on /objects: ``wine_site`` (raw id 20)
    0.761 on the white front face at (194, 340) of the banana cube whose print
    is at (186, 272) - INSIDE the real-cake score range 0.76..0.87, so the
    confidence gate cannot reject it.  The owner print right above the face
    can, and that is what these tests pin down.
    """

    BANANA_PRINT = (4, 0.705, 186, 272)   # live measurement, /objects frame
    BANANA_FACE = (20, 0.761, 194, 340)   # live false cake_site on that cube
    REAL_CAKE = (20, 0.760, 337, 213)     # live real cake site, same session

    def test_live_banana_side_is_not_a_cake_target(self):
        frames = [frame(self.BANANA_FACE, self.BANANA_PRINT) for _ in range(10)]
        ids = [cube[0] for cube in match_logic.cluster_detections(frames)]
        self.assertNotIn(match_logic.CAKE_ID, ids,
                         "the banana cube's white side must not become the cake")
        self.assertEqual(ids, [match_logic.BANANA_ID],
                         "only the banana cube itself may be targeted")

    def test_banana_cube_itself_stays_a_target(self):
        frames = [frame(self.BANANA_PRINT, self.BANANA_FACE) for _ in range(10)]
        result = match_logic.cluster_detections(frames)
        self.assertEqual(len(result), 1, "the banana cube itself must stay")
        self.assertEqual(result[0][0], match_logic.BANANA_ID)
        self.assertEqual(result[0][1][:2], [186, 272],
                         "target must stay on the print, not on the side face")

    def test_clock_face_veto_uses_its_own_print(self):
        frames = [frame((20, 0.66, 340, 199), (3, 0.911, 339, 131))
                  for _ in range(10)]
        ids = [cube[0] for cube in match_logic.cluster_detections(frames)]
        self.assertEqual(ids, [match_logic.CLOCK_ID],
                         "the clock face must not become a cake target")

    def test_real_cake_site_is_kept(self):
        """The real Cake face has no recognised cube print right above it."""
        frames = [frame(self.REAL_CAKE, (3, 0.910, 340, 27)) for _ in range(10)]
        ids = [cube[0] for cube in match_logic.cluster_detections(frames)]
        self.assertIn(match_logic.CAKE_ID, ids,
                      "the real cake site must stay a target")

    def test_neighbour_column_print_does_not_own_the_face(self):
        frames = [frame(self.REAL_CAKE, self.BANANA_PRINT) for _ in range(10)]
        ids = [cube[0] for cube in match_logic.cluster_detections(frames)]
        self.assertIn(match_logic.CAKE_ID, ids,
                      "a print 150 px to the side must not veto the cake")
        self.assertIn(match_logic.BANANA_ID, ids)

    def test_print_below_the_face_is_not_an_owner(self):
        frames = [frame((20, 0.76, 300, 200), (1, 0.90, 300, 300))
                  for _ in range(10)]
        ids = [cube[0] for cube in match_logic.cluster_detections(frames)]
        self.assertIn(match_logic.CAKE_ID, ids)

    def test_owner_helper_geometry_and_inputs(self):
        # print 68 px above the face owns it
        self.assertEqual(
            match_logic.cake_face_owner([(4, 0.70, 186, 272)], 194, 340),
            (4, 0.70, 186, 272))
        # *_site alias (raw 40) is not print evidence
        self.assertIsNone(
            match_logic.cake_face_owner([(40, 0.90, 337, 145)], 337, 213))
        # too weak a print is not evidence
        self.assertIsNone(
            match_logic.cake_face_owner([(1, 0.20, 337, 145)], 337, 213))
        # too far above / too far to the side is not evidence either
        self.assertIsNone(
            match_logic.cake_face_owner([(1, 0.90, 337, 27)], 337, 213))
        self.assertIsNone(
            match_logic.cake_face_owner([(1, 0.90, 186, 145)], 337, 213))

    def test_veto_is_configurable(self):
        frames = [frame(self.BANANA_FACE, self.BANANA_PRINT) for _ in range(10)]
        result = match_logic.cluster_detections(frames, cake_owner_conf=0.99)
        ids = [cube[0] for cube in result]
        self.assertIn(match_logic.CAKE_ID, ids,
                      "disabling the veto must restore the old (buggy) behaviour")

    def test_owner_geometry_matches_live_measurement(self):
        """Guards the constants against silent drift from the measured field."""
        self.assertEqual(match_logic.CAKE_OWNER_DY_PX, (45.0, 110.0))
        self.assertEqual(match_logic.CAKE_OWNER_DU_PX, 55.0)
        self.assertLessEqual(match_logic.CAKE_OWNER_PRINT_CONF, 0.705,
                             "the live banana print scored 0.705")
        self.assertNotIn(match_logic.CAKE_ID, match_logic.CAKE_OWNER_PRINT_IDS)


class TestCrossModelDedupe(unittest.TestCase):
    """dedupe_boxes() is what detect_ros.py applies before publishing /objects."""

    @staticmethod
    def box(name, score, cx, cy):
        return {"name": name, "score": score, "cx": cx, "cy": cy}

    def test_same_class_close_keeps_best_score(self):
        boxes = [self.box("bear", 0.91, 300, 200), self.box("bear", 0.62, 303, 198)]
        kept = match_logic.dedupe_boxes(boxes)
        self.assertEqual(len(kept), 1)
        self.assertAlmostEqual(kept[0]["score"], 0.91)

    def test_same_class_far_apart_survive(self):
        boxes = [self.box("bear", 0.9, 100, 100), self.box("bear", 0.8, 400, 100)]
        self.assertEqual(len(match_logic.dedupe_boxes(boxes)), 2)

    def test_different_classes_at_one_place_survive(self):
        """Two genuinely different classes must not be merged by position."""
        boxes = [self.box("bear", 0.9, 300, 200), self.box("clock", 0.8, 301, 200)]
        self.assertEqual(len(match_logic.dedupe_boxes(boxes)), 2)

    def test_threshold_is_configurable(self):
        boxes = [self.box("bear", 0.9, 100, 100), self.box("bear", 0.8, 140, 100)]
        self.assertEqual(len(match_logic.dedupe_boxes(boxes, dist_px=20)), 2)
        self.assertEqual(len(match_logic.dedupe_boxes(boxes, dist_px=60)), 1)

    def test_site_alias_and_base_class_merge_within_default_radius(self):
        """SI-26: banana (id 4) and banana_site (id 40) 67 px apart must merge."""
        boxes = [
            self.box("banana", 0.52, 221, 183),
            self.box("banana_site", 0.48, 218, 250),
        ]
        kept = match_logic.dedupe_boxes(boxes)
        self.assertEqual(len(kept), 1)
        self.assertAlmostEqual(kept[0]["score"], 0.52)

    def test_single_frame_cake_glare_in_multiframe_window_is_rejected(self):
        """A 1-frame cake_site spike in a 4-frame window must not hijack the FSM."""
        frames = [frame((20, 0.70, 320, 170)), [], [], []]
        self.assertEqual(match_logic.cluster_detections(frames), [])

    def test_far_field_floor_cube_is_not_hijacked_by_cake_override(self):
        """A far floor cube (v < 130) with 1 cake flicker uses majority vote."""
        frames = [
            frame((4, 0.75, 345, 107)),
            frame((4, 0.78, 346, 108)),
            frame((4, 0.80, 344, 106)),
            frame((20, 0.65, 345, 107)),
        ]
        result = match_logic.cluster_detections(frames)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0][0], match_logic.BANANA_ID)

    def test_cube_still_present_post_lift_check(self):
        """SI-28: detect whether the cube remained at its pickup pixel."""
        cubes = [[4, [295, 222, 0]]]
        self.assertTrue(match_logic.cube_still_present(cubes, (292, 224)))
        self.assertFalse(match_logic.cube_still_present(cubes, (120, 224)))
        self.assertFalse(match_logic.cube_still_present([], (292, 224)))


class TestGraspCandidateSelection(unittest.TestCase):
    def test_prefers_object_ahead_of_the_robot(self):
        cubes = [[1, [100, 100, 0]], [3, [320, 180, 0]], [4, [500, 300, 0]]]
        self.assertEqual(match_logic.select_grasp_candidate(cubes)[0],
                         match_logic.CLOCK_ID)

    def test_preferred_id_survives_a_class_flip(self):
        """After a re-scan the original class must win over a closer stranger."""
        cubes = [[1, [320, 200, 0]], [3, [330, 190, 0]]]
        picked = match_logic.select_grasp_candidate(cubes, preferred_id=match_logic.TEDDY_ID)
        self.assertEqual(picked[0], match_logic.TEDDY_ID)

    def test_falls_back_when_preferred_id_is_gone(self):
        cubes = [[3, [330, 190, 0]]]
        picked = match_logic.select_grasp_candidate(cubes, preferred_id=match_logic.TEDDY_ID)
        self.assertEqual(picked[0], match_logic.CLOCK_ID)

    def test_empty_input(self):
        self.assertIsNone(match_logic.select_grasp_candidate([]))


if __name__ == "__main__":
    unittest.main()
