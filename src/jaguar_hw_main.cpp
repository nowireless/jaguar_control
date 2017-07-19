//
// Created by ryan on 7/16/17.
//

#include <ros/ros.h>
#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <jaguar_control/jaguar_hw_interface.h>

using namespace jaguar_control;

int main(int argc, char** argv) {
    ros::init(argc, argv, "jaguar_control");
    ros::NodeHandle nh;

    // NOTE: We run the ROS loop in a separate thread as external calls such
    // as service callbacks to load controllers can block the (main) control loop
    ros::AsyncSpinner spinner(2);
    spinner.start();


    // Create the hardware interface
    boost::shared_ptr<JaguarHWInterface> hw(new JaguarHWInterface(nh));

    hw->init();
    ros_control_boilerplate::GenericHWControlLoop controlLoop(nh, hw);

    ros::waitForShutdown();

    hw.reset();
    return 0;
}