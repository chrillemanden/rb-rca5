#ifndef GAZEBO_H
#define GAZEBO_H


#include <gazebo/transport/transport.hh>

extern gazebo::transport::PublisherPtr movementPublisher;

void init_gazebo();

#endif // GAZEBO_H
