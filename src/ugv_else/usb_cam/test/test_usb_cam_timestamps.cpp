// Deterministic regression for the upstream 0.8.1 microsecond conversion bug.
// Mock only clock reads inside utils.hpp; do not change any system clock.
#include <gtest/gtest.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>
#include <cmath>
#include <ctime>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <map>
#include "linux/videodev2.h"
#include "usb_cam/constants.hpp"

static timeval fixed_epoch = {1700000000, 0};
static timespec fixed_monotonic = {123, 123456000};
static int fixed_gettimeofday(timeval * value, void *)
{
  *value = fixed_epoch;
  return 0;
}
static int fixed_clock_gettime(clockid_t, timespec * value)
{
  *value = fixed_monotonic;
  return 0;
}

#define gettimeofday fixed_gettimeofday
#define clock_gettime fixed_clock_gettime
#include "usb_cam/utils.hpp"
#undef gettimeofday
#undef clock_gettime

TEST(test_usb_cam_timestamps, preserves_fractional_seconds)
{
  for (const auto microseconds : {0, 1, 123456, 500000, 999999}) {
    fixed_epoch.tv_usec = microseconds;
    const int64_t expected = 1700000000LL * 1000000 + microseconds -
      (123LL * 1000000 + 123456);
    EXPECT_EQ(usb_cam::utils::get_epoch_time_shift_us(), expected);
    const timeval captured = {fixed_monotonic.tv_sec, fixed_monotonic.tv_nsec / 1000};
    const auto stamp = usb_cam::utils::calc_img_timestamp(
      captured, usb_cam::utils::get_epoch_time_shift_us());
    EXPECT_EQ(stamp.tv_sec, fixed_epoch.tv_sec);
    EXPECT_EQ(stamp.tv_nsec, microseconds * 1000);
  }
}
