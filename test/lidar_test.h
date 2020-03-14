#ifndef LIDAR_GTEST_EXAMPLE_H_
#define LIDAR_GTEST_EXAMPLE_H_

#include "gtest/gtest.h"
#include "CYdLidar.h"

// The fixture for testing class LiDAR.
class LidarTest : public ::testing::Test {

 protected:

  // You can do set-up work for each test here.
  LidarTest();

  // You can do clean-up work that doesn't throw exceptions here.
  virtual ~LidarTest();

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  // Code here will be called immediately after the constructor (right
  // before each test).
  virtual void SetUp();

  // Code here will be called immediately after each test (right
  // before the destructor).
  virtual void TearDown();

  // The lidar library shaed by all tests
  CYdLidar m_lidar;
};

#endif // LIDAR_GTEST_EXAMPLE_H_
