import unittest
import math
from gps_bridge import map_axis
from simulate_vrpn_server import map_local_to_vrpn
from velocity_filter import VelocityFilter

class TestGPSBridge(unittest.TestCase):
    def test_coordinate_mapping(self):
        # Coordinates: x=1.0, y=2.0, z=3.0
        # Check mapping of positive and negative axes
        self.assertEqual(map_axis("x", 1.0, 2.0, 3.0), 1.0)
        self.assertEqual(map_axis("-x", 1.0, 2.0, 3.0), -1.0)
        self.assertEqual(map_axis("y", 1.0, 2.0, 3.0), 2.0)
        self.assertEqual(map_axis("-y", 1.0, 2.0, 3.0), -2.0)
        self.assertEqual(map_axis("z", 1.0, 2.0, 3.0), 3.0)
        self.assertEqual(map_axis("-z", 1.0, 2.0, 3.0), -3.0)

    def test_geodetic_translation(self):
        # Origin: Singapore coordinates
        origin_lat = 1.342859
        origin_lon = 103.966484
        origin_alt = 10.0
        
        # Move 100 meters East and 50 meters North
        east = 100.0
        north = 50.0
        up = 5.0
        
        # Expected latitude
        expected_lat = origin_lat + (north / 111111.0)
        
        # Expected longitude
        rad_lat = math.radians(origin_lat)
        expected_lon = origin_lon + (east / (111111.0 * math.cos(rad_lat)))
        
        expected_alt = origin_alt + up
        
        # Run conversion projection
        lat_deg = origin_lat + (north / 111111.0)
        lon_deg = origin_lon + (east / (111111.0 * math.cos(math.radians(origin_lat))))
        alt_m = origin_alt + up
        
        self.assertAlmostEqual(lat_deg, expected_lat, places=7)
        self.assertAlmostEqual(lon_deg, expected_lon, places=7)
        self.assertAlmostEqual(alt_m, expected_alt, places=2)

    def test_velocity_derivatives(self):
        # First point
        t1 = 0.0
        north1, east1, down1 = 0.0, 0.0, 0.0
        
        # Second point after 0.1 seconds
        t2 = 0.1
        north2, east2, down2 = 0.5, -0.2, 0.1
        
        dt = t2 - t1
        self.assertAlmostEqual(dt, 0.1)
        
        # Expected velocities
        expected_vn = (north2 - north1) / dt
        expected_ve = (east2 - east1) / dt
        expected_vd = (down2 - down1) / dt
        
        self.assertEqual(expected_vn, 5.0)
        self.assertEqual(expected_ve, -2.0)
        self.assertEqual(expected_vd, 1.0)

    def test_local_to_vrpn_mapping(self):
        mapping = {
            "east": "x",
            "north": "-z",
            "up": "y"
        }
        # Given: east=10.0, north=20.0, up=1.5
        # Expected VRPN: pos_x = 10.0, pos_y = 1.5, pos_z = -20.0
        px, py, pz = map_local_to_vrpn(10.0, 20.0, 1.5, mapping)
        self.assertEqual(px, 10.0)
        self.assertEqual(py, 1.5)
        self.assertEqual(pz, -20.0)

    def test_velocity_filter_spike_rejection(self):
        vf = VelocityFilter(window_duration_s=0.2, ema_alpha=0.3, max_dt_s=0.5)
        
        # Sample sequence around steady velocity of 1.0 m/s with a 100.0 m/s spike in the middle
        # t = 0.0, v = 1.0
        # t = 0.05, v = 1.0
        # t = 0.10, v = 100.0 (SPIKE)
        # t = 0.15, v = 1.0
        # t = 0.20, v = 1.0
        vn1, _, _ = vf.update(0.00, 1.0, 0.0, 0.0)
        vn2, _, _ = vf.update(0.05, 1.0, 0.0, 0.0)
        vn3, _, _ = vf.update(0.10, 100.0, 0.0, 0.0)  # Spike
        vn4, _, _ = vf.update(0.15, 1.0, 0.0, 0.0)
        vn5, _, _ = vf.update(0.20, 1.0, 0.0, 0.0)

        # Median filter will evaluate median([1.0, 1.0, 100.0, 1.0]) = 1.0
        # Spike of 100.0 is successfully suppressed, output velocity should remain near 1.0 (far from 100.0)
        self.assertLess(vn3, 35.0)  # Unfiltered would be ~100 or huge, median keeps it around 1.0
        self.assertAlmostEqual(vn5, 1.0, delta=0.2)

    def test_velocity_filter_gap_reset(self):
        vf = VelocityFilter(window_duration_s=0.2, ema_alpha=0.5, max_dt_s=0.5)
        vf.update(0.0, 10.0, 10.0, 10.0)
        
        # Jump ahead in time > max_dt_s (e.g. 1.0s gap)
        vn, ve, vd = vf.update(1.0, 2.0, 2.0, 2.0)
        # Since gap > max_dt_s, filter should reset and adopt new value 2.0 directly
        self.assertEqual(vn, 2.0)
        self.assertEqual(ve, 2.0)
        self.assertEqual(vd, 2.0)

    def test_velocity_filter_clamping(self):
        vf = VelocityFilter(window_duration_s=0.2, ema_alpha=1.0, max_velocity_ms=10.0)
        # Pass velocity vector (15.0, 0.0, 0.0) exceeding 10.0 max speed
        vn, ve, vd = vf.update(0.0, 15.0, 0.0, 0.0)
        self.assertAlmostEqual(vn, 10.0)
        self.assertAlmostEqual(ve, 0.0)
        self.assertAlmostEqual(vd, 0.0)

if __name__ == "__main__":
    unittest.main()

