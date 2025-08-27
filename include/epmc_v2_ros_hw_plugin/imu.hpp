#ifndef EPMC_V2_IMU_HPP
#define EPMC_V2_IMU_HPP

#include <string>
#include <cmath>


class IMU
{
    public:

    std::string name = "";
    double qw = 1.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;

    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
    
    double gx = 0.0;
    double gy = 0.0;
    double gz = 0.0;

    int use_imu = 0;

    IMU() = default;

    IMU(const std::string &imu_sensor_name)
    {
      setup(imu_sensor_name);
    }
  
    void setup(const std::string &imu_sensor_name)
    {
      name = imu_sensor_name;
    }
};


#endif // EPMC_V2_IMU_HPP
