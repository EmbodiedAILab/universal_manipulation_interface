import unittest
import time

from umi.common.status_interpolator import StatusInterpolator


class MyTestCase(unittest.TestCase):
    def test_status_interpolator(self):
        curr_t = time.monotonic()
        status_interpolator = StatusInterpolator(times=[curr_t],status=[0])
        t_target = curr_t+10
        status_interpolator = status_interpolator.schedule_waypoint(
            status=1,
            time=t_target,
            # max_pos_speed=self.max_speed,
            # max_rot_speed=self.max_speed,
            curr_time=curr_t,
            last_waypoint_time=curr_t-10,
        )

        target_status = status_interpolator(t_target+1)
        print(target_status)
        # self.assertEqual(True, False)  # add assertion here

    def test_status_interpolator2(self):
        curr_t = time.monotonic()
        status_interpolator = StatusInterpolator(times=[curr_t,curr_t+1,curr_t+2],status=[0,1,0])
        print(status_interpolator.status)

        target_status = status_interpolator(curr_t+2.1)
        print(target_status)


if __name__ == '__main__':
    unittest.main()
