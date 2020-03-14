import unittest
import ydlidar


class PyTestTestCase(unittest.TestCase):

  def testOSInitIsWrappedCorrectly(self):
    print("test os init.......")
    ydlidar.os_init();
    self.assertTrue(ydlidar.os_isOk());
    ydlidar.os_shutdown();
    self.assertFalse(ydlidar.os_isOk());

  def testParamtersIsWrappedCorrectly(self):
    print("test paramters.......")
    laser = ydlidar.CYdLidar();
    port = "/dev/ydlidar";
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
    ret, result = laser.getlidaropt_toString(ydlidar.LidarPropSerialPort);
    self.assertTrue(ret);
    self.assertEqual(port, result)
    baudrate = 230400;
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, baudrate);
    ret, baud = laser.getlidaropt_toInt(ydlidar.LidarPropSerialBaudrate);
    self.assertTrue(ret);
    self.assertEqual(baudrate, baud);
    maxRange = 16.0;
    laser.setlidaropt(ydlidar.LidarPropMaxRange, maxRange);
    ret, mRange = laser.getlidaropt_toFloat(ydlidar.LidarPropMaxRange);
    self.assertTrue(ret);
    self.assertEqual(maxRange, mRange);
    intensity = False;
    laser.setlidaropt(ydlidar.LidarPropIntenstiy, intensity);
    ret, quality = laser.getlidaropt_toBool(ydlidar.LidarPropIntenstiy);
    self.assertTrue(ret);
    self.assertEqual(intensity, quality);

if __name__ == "__main__":
  unittest.main()
