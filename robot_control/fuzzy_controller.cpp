#include "fl/Headers.h"
<<<<<<< HEAD

/* Driver includes */
#include "fuzzy_controller.h"

extern double global_minDist;
extern double global_angle;

// Declaring engines and input-outputvariables for the fuzzy controller
fl::Engine* engine; // = fl::FllImporter().fromFile("ObstacleAvoidance.fll");
std::string status;
fl::InputVariable* obstacle_distance; // = engine->getInputVariable("obstacle_distance");
fl::InputVariable* obstacle_angle; // = engine->getInputVariable("obstacle_angle");
fl::OutputVariable* steer; // = engine->getOutputVariable("mSteer");
fl::OutputVariable* output_speed; // = engine->getOutputVariable("robot_speed");

void init_fuzzy_controller()
{
    engine = fl::FllImporter().fromFile("ObstacleAvoidance.fll");

    engine->isReady(&status);

    if (not engine->isReady(&status))
    {
        std::cout << "not found" << std::endl;
        throw fl::Exception("[engine error] engine is not ready:n" + status, FL_AT);
    }

    obstacle_distance = engine->getInputVariable("obstacle_distance");
    obstacle_angle = engine->getInputVariable("obstacle_angle");
    steer = engine->getOutputVariable("mSteer");
    output_speed = engine->getOutputVariable("robot_speed");
}

void simple_fuzzy_avoidance(float arrSteer[])
=======
#include "fuzzy_controller.h"

/*
void simple_fuzzy_avoidance()
>>>>>>> refs/remotes/origin/master
{
    fl::scalar distance = global_minDist;
    fl::scalar angle = global_angle;
    obstacle_angle->setValue(angle);
    obstacle_distance->setValue(distance);
    engine->process();
<<<<<<< HEAD
    float dir = steer->getValue();
    float speed = output_speed->getValue();

    //std::cout << "global_minDist: " << distance << std::endl;
    //std::cout << "dir: " << dir << std::endl;

    arrSteer[0] = speed;
    arrSteer[1] = dir;

}
=======
    dir = steer->getValue();
    speed = output_speed->getValue();
    tick--;
    std::cout << "global_minDist: " << distance << std::endl;
    std::cout << "dir: " << dir << std::endl;
} */
>>>>>>> refs/remotes/origin/master
