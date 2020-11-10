#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
// All the webots classes are defined in the "webots" namespace
using namespace webots;
#define FRONT_WHEEL_IDX (0)
#define LEFT_WHEEL_IDX  (1)
#define RIGHT_WHEEL_IDX (2)
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
    // create the Robot instance.
    std::cout<<"load controller"<<std::endl;
    Robot *robot = new Robot();

    // get the time step of the current world.
    int timeStep = (int)robot->getBasicTimeStep();
    Motor *motor[3];
    motor[0] = robot->getMotor("base_wheel_0_joint");
    motor[1] = robot->getMotor("base_wheel_1_joint");
    motor[2] = robot->getMotor("base_wheel_2_joint");
    motor[0]->setPosition(INFINITY);
    motor[1]->setPosition(INFINITY);
    motor[2]->setPosition(INFINITY);
    // You should insert a getDevice-like function in order to get the
    // instance of a device of the robot. Something like:
    //  Motor *motor = robot->getMotor("motorname");
    //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
    //  ds->enable(timeStep);

    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot->step(timeStep) != -1) {
        // Read the sensors:
        // Enter here functions to read sensor data, like:
        //  double val = ds->getValue();
        motor[0]->setVelocity(0);
        motor[1]->setVelocity(4);
        motor[2]->setVelocity(4);
        // Process sensor data here.

        // Enter here functions to send actuator commands, like:
        //  motor->setPosition(10.0);
    };

    // Enter here exit cleanup code.

    delete robot;
    return 0;
}
