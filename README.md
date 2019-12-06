# rb-rca5
## 5th semester combined Artificial Intelligence, Computer Vision and Robotics project for the robotics education at the University of Southern Denmark

### Description
In this project a differential drive robot roams a gazebo-environment in which marbles can be picked up by driving into them. The marbles are distributed evenly accross the environment that is seperated into rooms by an occlusion map.
The original repository containing just the barebone environment can be found here: [jakobwilm/rb-rca5](https://github.com/jakobwilm/rb-rca5). The project description is provided in this repository: ``ProjectDescription``.

### Contents 
Below is a brief mention of the topics covered:

#### Artificial Intelligence
* Fuzzy Logic Control
* Q-Learning

#### Computer Vision
* Black and white corner detection
* Marble detection with histogram equalisation and houghCircles() and using contour detection

#### Robotics
* Brushfire algorithm
* Waypoint generation
* Particle filter for localisation
* Dijkstra's algorithm

The report ``AI_CV_ROB_Report.pdf`` describes these topics and their implementation in greater detail.


### Future Work
Below is a list of relevant topics to implement in the future:

#### Artificial Intelligence
* Deep Q-Networks for controlling the robot using the differential drive abstraction
* DDPG/PPO for controlling the robot directly

#### Computer Vision
* Detection of walls, corners and doorways in the live video feed

#### Robotics
* A-star algorithm
* Voronoi-diagrams on the occlusion map
* Marble, walls and corner detection using the onboard lidar
* Enhancement of the particle filter

#### Other
* Combine the different topics to solve the original project description 




