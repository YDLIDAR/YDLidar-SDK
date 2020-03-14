#include "lidar_test.h"

LidarTest::LidarTest() {
  std::string port = "/dev/ttyUSB0";
  m_lidar.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());

  int baudrate = 230400;
  m_lidar.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));

  bool singleChannel = false;
  m_lidar.setlidaropt(LidarPropSingleChannel, &singleChannel, sizeof(bool));

  float scanFrequency = 10;
  m_lidar.setlidaropt(LidarPropScanFrequency, &scanFrequency, sizeof(float));
}

LidarTest::~LidarTest() {}

void LidarTest::SetUp() {}

void LidarTest::TearDown() {}

TEST_F(LidarTest, SystemSignal) {
  ydlidar::os_init();
  EXPECT_EQ(ydlidar::os_isOk(), true);
  ydlidar::os_shutdown();
  EXPECT_EQ(ydlidar::os_isOk(), false);
}

TEST_F(LidarTest, SerialPort) {
  char port[100];
  m_lidar.getlidaropt(LidarPropSerialPort, port, 100);
  std::string lidar_port = port;
  EXPECT_EQ(lidar_port, "/dev/ttyUSB0");
}

TEST_F(LidarTest, SerialBaudrate) {
  int baudrate;
  m_lidar.getlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
  EXPECT_EQ(baudrate, 230400);
}

TEST_F(LidarTest, SingleChannel) {
  bool singleChannel;
  m_lidar.getlidaropt(LidarPropSingleChannel, &singleChannel, sizeof(bool));
  EXPECT_EQ(singleChannel, false);
}

TEST_F(LidarTest, ScanFequency) {
  float frequency;
  m_lidar.getlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));
  EXPECT_EQ(frequency, 10.f);
}


TEST_F(LidarTest, TurnOn) {
  bool ret = m_lidar.initialize();

  if (ret) {
    LidarVersion version;
    memset(&version, 0, sizeof(LidarVersion));
    m_lidar.GetLidarVersion(version);
    EXPECT_EQ(version.soft_major != 0, true);
    EXPECT_EQ(m_lidar.turnOn(), true);
    LaserScan scan;
    EXPECT_EQ(m_lidar.doProcessSimple(scan), true);

    EXPECT_EQ(m_lidar.turnOff(), true);
  }

  m_lidar.disconnecting();
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
