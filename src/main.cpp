#include <thread>
#include <string>
#include <iostream>
#include <socketcan_interface/socketcan.h>
#include <odrive_can_ros/can_simple.hpp>

using namespace odrive_can_ros;
using namespace std::literals::chrono_literals;


int main(int argc, char* argv[])
{
    // Create CAN master node 
    CANSimple master;
    if ( !( master.add_axis(1, "odrive_axis_1") &&
            master.add_axis(2, "odrive_axis_2") ) )
    {
        fprintf(stderr, "Failed to create one or more axis. Aborting.\n");
        return -1;
    }

    // Create Interface to SocketCAN 
    const std::string can_device = "can0";
    can::ThreadedSocketCANInterfaceSharedPtr driver = 
        std::make_shared<can::ThreadedSocketCANInterface>();
    if (!driver->init(can_device, 0, can::NoSettings::create()))
    {
        fprintf(stderr, "Failed to initialize can_device at %s\n", can_device.c_str());
        return -1;
    }
    can::StateListenerConstSharedPtr state_listener = driver->createStateListener(
        [&driver](const can::State& s) {
            std::string err;
            driver->translateError(s.internal_error, err);
            fprintf(stderr, "CAN Device error: %s, asio: %s.\n", 
                err.c_str(), s.error_code.message().c_str());
        }
    );

    // Pass the SocketCAN handle to master
    master.init(driver);
    std::this_thread::sleep_for(500ms);
    
    // Background worker thread
    std::thread thr( [&master]() {
        while(master.is_ready()) {
            // master.get_encoder_estimates(master.axis("odrive_axis_1"));
            // master.get_encoder_estimates(master.axis("odrive_axis_2"));
            fprintf(stdout, 
                "Axis 1: [pos, vel] = [%4.3f, %4.3f]\n", 
                master.axis("odrive_axis_1").pos_enc_estimate,
                master.axis("odrive_axis_1").vel_enc_estimate);
            fprintf(stdout, 
                "Axis 2: [pos, vel] = [%4.3f, %4.3f]\n\n", 
                master.axis("odrive_axis_2").pos_enc_estimate,
                master.axis("odrive_axis_2").vel_enc_estimate);
            std::this_thread::sleep_for(500ms);
        } 
    } );

    master.clear_errors(master.axis("odrive_axis_1"));
    master.clear_errors(master.axis("odrive_axis_2"));
    master.set_axis_requested_state(master.axis("odrive_axis_1"), AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    master.set_axis_requested_state(master.axis("odrive_axis_2"), AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    std::this_thread::sleep_for(500ms);
    master.set_input_vel(master.axis("odrive_axis_1"), 2.0);
    master.set_input_vel(master.axis("odrive_axis_2"), 2.0);
    std::this_thread::sleep_for(2000ms);
    master.set_input_vel(master.axis("odrive_axis_1"), -2.0);
    master.set_input_vel(master.axis("odrive_axis_2"), -2.0);
    std::this_thread::sleep_for(2000ms);
    master.set_input_vel(master.axis("odrive_axis_1"), 0.0);
    master.set_input_vel(master.axis("odrive_axis_2"), 0.0);
    master.set_axis_requested_state(master.axis("odrive_axis_1"), AxisState::AXIS_STATE_IDLE);
    master.set_axis_requested_state(master.axis("odrive_axis_2"), AxisState::AXIS_STATE_IDLE);

    thr.join();
    driver.reset();

    return 0;
}
