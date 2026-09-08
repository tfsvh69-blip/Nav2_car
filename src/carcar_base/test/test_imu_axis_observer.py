import unittest

from carcar_base.imu_axis_observer import summarize_samples


class ImuAxisObserverTest(unittest.TestCase):

    def test_summary_uses_all_six_axes(self) -> None:
        result = summarize_samples([
            (1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
            (3.0, 4.0, 5.0, 6.0, 7.0, 8.0),
        ])

        self.assertEqual(set(result), {'ax', 'ay', 'az', 'gx', 'gy', 'gz'})
        self.assertEqual(result['ax']['mean'], 2.0)
        self.assertEqual(result['gz']['mean'], 7.0)
        self.assertEqual(result['ax']['std'], 1.0)
        self.assertEqual(result['gz']['min'], 6.0)
        self.assertEqual(result['gz']['max'], 8.0)

    def test_empty_input_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            summarize_samples([])

    def test_invalid_samples_do_not_produce_calibration_statistics(self) -> None:
        for sample in ((0.0,) * 5, (0.0,) * 5 + (float('nan'),),
                       (float('inf'),) + (0.0,) * 5):
            with self.assertRaises(ValueError):
                summarize_samples([sample])
