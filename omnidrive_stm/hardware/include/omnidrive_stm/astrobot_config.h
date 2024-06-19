#ifndef ASTROBOT_CONFIG_H
#define ASTROBOT_CONFIG_H

#include <string>


struct Config
{
  std::string wheel_0 = "roda1";
  std::string wheel_1 = "roda2";
  std::string wheel_2 = "roda3";
  std::string wheel_3 = "roda4";
  float loop_rate = 100;
  std::string device = "/dev/ttyUSB0";
  std::string baud_rate = "B115200";
//   std::string cs8 = "CS8";
//   std::string clocal = "CLOCAL";
//   std::string cread = "CREAD";
//   int timeout = 1000;
//   int enc_counts_per_rev = 1920;
};


#endif // ASTROBOT_CONFIG_H