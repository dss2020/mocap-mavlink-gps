import unittest
import math
from gps_bridge import map_axis

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

if __name__ == "__main__":
    unittest.main()
