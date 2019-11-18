#ifndef LIDAR_H
#define LIDAR_H

#include <vector>

//extern std::vector<double> lidar_data;


void simpleLidarCallback(ConstLaserScanStampedPtr &msg);

void lidarCallback(ConstLaserScanStampedPtr &msg);

#endif // LIDAR_H
