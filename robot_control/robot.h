#ifndef ROBOT_H
#define ROBOT_H

void statCallback(ConstWorldStatisticsPtr &_msg);

void poseCallback(ConstPosesStampedPtr &_msg);

#endif // ROBOT_H
