import unittest
from collections import Counter


def process_detections_clustering(detections_stream, collected_frames, cluster_dist_thresh=45):
    """
    Pure algorithmic extraction of spatial clustering and temporal voting
    used in runtask.py CamAction.detector().
    """
    clusters = []
    for frame in detections_stream:
        for (canonical_id, cx, cy, score) in frame:
            if canonical_id not in [1, 2, 3, 4] or score < 0.45:
                continue

            matched_cluster = None
            for cl in clusters:
                cl_mean_x = sum(cl['xs']) / len(cl['xs'])
                cl_mean_y = sum(cl['ys']) / len(cl['ys'])
                if ((cx - cl_mean_x) ** 2 + (cy - cl_mean_y) ** 2) ** 0.5 < cluster_dist_thresh:
                    matched_cluster = cl
                    break

            if matched_cluster is not None:
                matched_cluster['ids'].append(canonical_id)
                matched_cluster['xs'].append(cx)
                matched_cluster['ys'].append(cy)
            else:
                clusters.append({
                    'ids': [canonical_id],
                    'xs': [cx],
                    'ys': [cy]
                })

    cube_list = []
    min_hits = max(2, int(collected_frames * 0.25)) if collected_frames >= 4 else 1
    for cl in clusters:
        if len(cl['ids']) >= min_hits:
            dominant_id = Counter(cl['ids']).most_common(1)[0][0]
            avg_x = int(round(sum(cl['xs']) / len(cl['xs'])))
            avg_y = int(round(sum(cl['ys']) / len(cl['ys'])))
            cube_list.append([dominant_id, [avg_x, avg_y, 0]])

    return cube_list


class TestTemporalVotingAndClustering(unittest.TestCase):
    """Test suite for spatial clustering and temporal voting on noisy vision detections."""

    def test_single_object_with_jitter_and_class_noise(self):
        """Single object with +-5px coordinate noise and minor ID misclassifications."""
        # 10 frames of Cake (canonical ID 2) at ~ (320, 240)
        # 8 frames detected as 2, 2 frames misclassified as 3 (Clock)
        stream = [
            [(2, 321, 239, 0.85)],
            [(2, 319, 241, 0.82)],
            [(3, 320, 240, 0.48)],  # Noise ID
            [(2, 322, 238, 0.90)],
            [(2, 318, 242, 0.88)],
            [(2, 320, 240, 0.91)],
            [(3, 321, 239, 0.52)],  # Noise ID
            [(2, 320, 240, 0.89)],
            [(2, 319, 241, 0.86)],
            [(2, 320, 240, 0.93)],
        ]
        result = process_detections_clustering(stream, collected_frames=10)

        self.assertEqual(len(result), 1, "Should identify exactly 1 object cluster")
        dominant_id, (avg_x, avg_y, _) = result[0]
        self.assertEqual(dominant_id, 2, "Temporal voting must resolve dominant class to Cake (2)")
        self.assertAlmostEqual(avg_x, 320, delta=2, msg="Averaged X must match true centroid")
        self.assertAlmostEqual(avg_y, 240, delta=2, msg="Averaged Y must match true centroid")

    def test_two_distinct_spatial_objects(self):
        """Two cubes spaced 150px apart must form distinct clusters without cross-talk."""
        # Cube 1: Bear (ID 1) around (150, 200)
        # Cube 2: Clock (ID 3) around (450, 200)
        stream = []
        for _ in range(8):
            stream.append([
                (1, 150, 200, 0.9),
                (3, 450, 200, 0.9)
            ])

        result = process_detections_clustering(stream, collected_frames=8)
        self.assertEqual(len(result), 2, "Must identify two distinct spatial clusters")

        ids_found = {item[0] for item in result}
        self.assertEqual(ids_found, {1, 3}, "Clusters must preserve respective classes")

    def test_transient_ghost_detection_rejected(self):
        """A transient detection appearing only once out of 10 frames must be rejected."""
        stream = [
            [(1, 100, 100, 0.9)],  # Valid Bear present across frames
            [(1, 102,  98, 0.9)],
            [(1,  99, 101, 0.9)],
            [(1, 101, 100, 0.9)],
            [(1, 100, 100, 0.9)],
            [(4, 500, 500, 0.50)],  # Single false positive (ghost) far away
            [(1, 100, 100, 0.9)],
            [(1, 101,  99, 0.9)],
            [(1, 100, 101, 0.9)],
            [(1, 100, 100, 0.9)],
        ]
        result = process_detections_clustering(stream, collected_frames=10)
        self.assertEqual(len(result), 1, "Ghost detection must be filtered out by min_hits threshold")
        self.assertEqual(result[0][0], 1)

    def test_low_confidence_detections_filtered_before_clustering(self):
        """Detections with score < 0.45 should be completely ignored."""
        stream = [
            [(2, 200, 200, 0.30)]  # Below 0.45 confidence
            for _ in range(10)
        ]
        result = process_detections_clustering(stream, collected_frames=10)
        self.assertEqual(len(result), 0, "Low confidence detections must produce 0 clusters")


if __name__ == "__main__":
    unittest.main()
