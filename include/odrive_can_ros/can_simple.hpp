#ifndef __CAN_SIMPLE_HPP_
#define __CAN_SIMPLE_HPP_

#include <cstdint>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>

#include <odrive_can_ros/can_helpers.hpp>
#include <odrive_can_ros/ODriveEnums.h>

namespace odrive_can_ros
{

struct ODriveAxis
{
    uint32_t node_id;
    bool is_extended = false;
    MotorError motor_error = MOTOR_ERROR_NONE;
    EncoderError encoder_error = ENCODER_ERROR_NONE;
    SensorlessEstimatorError sensorless_estimator_error = SENSORLESS_ESTIMATOR_ERROR_NONE;
    AxisError axis_error = AXIS_ERROR_NONE;
    AxisState axis_state = AXIS_STATE_UNDEFINED;
    float pos_enc_estimate = 0.0;
    float vel_enc_estimate = 0.0;
    float pos_sensorless_estimate = 0.0;
    float vel_sensorless_estimate = 0.0;
    int32_t encoder_shadow_count = 0;
    int32_t encoder_count_cpr = 0;
    float vbus_voltage = 0.0;
    uint8_t controller_flags = 0;
    float idq_first=0.0, idq_second=0.0;
    ODriveAxis(unsigned int id) {node_id = id;} ;
};

class CANSimple 
{

typedef ODriveAxis Axis;
typedef can::DriverInterfaceSharedPtr CanBusDriverPtr;

public:
    enum CanSimpleMessage {
        MSG_CO_NMT_CTRL = 0x000,  // CANOpen NMT Message REC
        MSG_ODRIVE_HEARTBEAT,
        MSG_ODRIVE_ESTOP,
        MSG_GET_MOTOR_ERROR,  // Errors
        MSG_GET_ENCODER_ERROR,
        MSG_GET_SENSORLESS_ERROR,
        MSG_SET_AXIS_NODE_ID,
        MSG_SET_AXIS_REQUESTED_STATE,
        MSG_SET_AXIS_STARTUP_CONFIG,
        MSG_GET_ENCODER_ESTIMATES,
        MSG_GET_ENCODER_COUNT,
        MSG_SET_CONTROLLER_MODES,
        MSG_SET_INPUT_POS,
        MSG_SET_INPUT_VEL,
        MSG_SET_INPUT_TORQUE,
        MSG_SET_LIMITS,
        MSG_START_ANTICOGGING,
        MSG_SET_TRAJ_VEL_LIMIT,
        MSG_SET_TRAJ_ACCEL_LIMITS,
        MSG_SET_TRAJ_INERTIA,
        MSG_GET_IQ,
        MSG_GET_SENSORLESS_ESTIMATES,
        MSG_RESET_ODRIVE,
        MSG_GET_VBUS_VOLTAGE,
        MSG_CLEAR_ERRORS,
        MSG_SET_LINEAR_COUNT,
        MSG_SET_POS_GAIN,
        MSG_SET_VEL_GAINS,
        MSG_CO_HEARTBEAT_CMD = 0x700,  // CANOpen NMT Heartbeat  SEND
    };

    // CANSimple(CanBusDriverPtr driver);
    void init(CanBusDriverPtr driver);
    static void handle_can_message(const can_Message_t& msg, std::vector<Axis>& axes);

    // Get functions (msg.rtr bit must be set)
    bool get_motor_error(const Axis& axis);
    bool get_encoder_error(const Axis& axis);
    bool get_sensorless_error(const Axis& axis);
    bool get_encoder_estimates(const Axis& axis);
    bool get_encoder_count(const Axis& axis);
    bool get_iq(const Axis& axis);
    bool get_sensorless_estimates(const Axis& axis);
    bool get_vbus_voltage(const Axis& axis);

    // Set functions
    bool set_axis_nodeid(const Axis& axis, const uint32_t& node_id);
    bool set_axis_requested_state(const Axis& axis, const AxisState& state);
    bool set_axis_startup_config(const Axis& axis);
    bool set_input_pos(const Axis& axis, const float pos, const int16_t vel_ff=0, const int16_t torque_ff=0);
    bool set_input_vel(const Axis& axis, const float vel, const float torque_ff=0.0f);
    bool set_input_torque(const Axis& axis, const float torque);
    bool set_controller_modes(const Axis& axis, ControlMode ctrl_mode, InputMode input_mode);
    bool set_limits(const Axis& axis, const float vel_limit, const float torque_limit);
    bool set_traj_vel_limit(const Axis& axis, const float vel_limit);
    bool set_traj_accel_limits(const Axis& axis, const float accel, const float decel);
    bool set_traj_inertia(const Axis& axis, const float inertia);
    bool set_linear_count(const Axis& axis, const int32_t count);
    bool set_pos_gain(const Axis& axis, const float pgain);
    bool set_vel_gains(const Axis& axis, const float pgain, const float igain);

    // Other functions
    bool nmt(const Axis& axis);
    bool estop(const Axis& axis);
    bool clear_errors(const Axis& axis);
    bool start_anticogging(const Axis& axis);
    bool reset_odrive(const Axis& axis);

private:
    static void do_command(Axis& axis, const can_Message_t& cmd);

    bool send_message(const can_Message_t& msg);
    bool send_message(const Axis& axis, const CanSimpleMessage type, const bool rtr=false);
    
    // Get function callbacks
    static void hearbeat_callback(Axis& axis, const can_Message_t& msg);
    static void get_motor_error_callback(Axis& axis, const can_Message_t& msg);
    static void get_encoder_error_callback(Axis& axis, const can_Message_t& msg);
    static void get_sensorless_error_callback(Axis& axis, const can_Message_t& msg);
    static void get_encoder_estimates_callback(Axis& axis, const can_Message_t& msg);
    static void get_encoder_count_callback(Axis& axis, const can_Message_t& msg);
    static void get_iq_callback(Axis& axis, const can_Message_t& msg);
    static void get_sensorless_estimates_callback(Axis& axis, const can_Message_t& msg);
    static void get_vbus_voltage_callback(Axis& axis, const can_Message_t& msg);

    static constexpr uint8_t NUM_NODE_ID_BITS = 6;
    static constexpr uint8_t NUM_CMD_ID_BITS = 11 - NUM_NODE_ID_BITS;

    // Utility functions
    static constexpr uint32_t get_node_id(uint32_t msgID) {
        return (msgID >> NUM_CMD_ID_BITS);  // Upper 6 or more bits
    };

    static constexpr uint8_t get_cmd_id(uint32_t msgID) {
        return (msgID & 0x01F);  // Bottom 5 bits
    }

    CanBusDriverPtr canbus_;
    
};

} // namespace

#endif
