#include <odrive_can_ros/can_simple.hpp>

namespace odrive_can_ros
{

CANSimple::~CANSimple()
{
    if (ready_)
    {
        ready_ = false;
        canbus_->shutdown();
    }
    canbus_.reset();
}

bool CANSimple::add_axis(const int id, const std::string name)
{
    auto it = std::find_if(axes_.cbegin(), axes_.cend(),
        [&id](const auto &entry) { return entry.second.node_id == id; });
    if (it != axes_.cend()) {
        fprintf(stdout, "Cannot add axis %s with node ID %d.\n", name.c_str(), id);
        fprintf(stdout, "The node ID is already occupied by %s.\n", it->first.c_str());
        return false;
    }
    
    if (!name.empty())
    {
        axes_.insert( std::make_pair(name, Axis((unsigned int)id)) );
    } else 
    {
        axes_.insert( std::make_pair(std::to_string(id), Axis((unsigned int)id)) );
    }
    return true;
}

const CANSimple::Axis CANSimple::axis(const std::string name)
{
    return axes_.at(name);
}

void CANSimple::init(CanBusDriverPtr driver)
{
    canbus_ = std::move(driver);
    frame_listener_ = canbus_->createMsgListener( 
        [this](const can::Frame &f) {
            if (!f.isValid() || f.is_error)
                return;
            can_Message_t rxmsg;
            rxmsg.id = f.id;
            rxmsg.isExt = f.is_extended;
            rxmsg.len = f.dlc;
            rxmsg.rtr = f.is_rtr;
            std::copy(std::begin(f.data), std::end(f.data), std::begin(rxmsg.buf));
            handle_can_message(rxmsg);
        }
    );
    ready_ = true;
}

bool CANSimple::is_ready()
{
    return ready_;
}

void CANSimple::handle_can_message(const can_Message_t& msg) {
    //     Frame
    // nodeID | CMD
    // 6 bits | 5 bits
    if (!ready_) { return; }
    uint32_t nodeID = get_node_id(msg.id);
    AxisMap::iterator itr;
    for (itr = axes_.begin(); itr != axes_.end(); ++itr) 
    {
        if ((itr->second.node_id == nodeID) && (itr->second.is_extended == msg.isExt)) {
            do_command(itr->second, msg);
            return;
        }
    }
}

void CANSimple::do_command(Axis& axis, const can_Message_t& msg) {
    const uint32_t cmd = get_cmd_id(msg.id);
    switch (cmd) {
        case MSG_ODRIVE_HEARTBEAT:
            hearbeat_callback(axis, msg);
            break;
        case MSG_GET_MOTOR_ERROR:
            get_motor_error_callback(axis, msg);
            break;
        case MSG_GET_ENCODER_ERROR:
            get_encoder_error_callback(axis, msg);
            break;
        case MSG_GET_SENSORLESS_ERROR:
            get_sensorless_error_callback(axis, msg);
            break;
        case MSG_GET_ENCODER_ESTIMATES:
            get_encoder_estimates_callback(axis, msg);
            break;
        case MSG_GET_ENCODER_COUNT:
            get_encoder_count_callback(axis, msg);
            break;
        case MSG_GET_IQ:
            get_iq_callback(axis, msg);
            break;
        case MSG_GET_SENSORLESS_ESTIMATES:
            get_sensorless_estimates_callback(axis, msg);
            break;
        case MSG_GET_VBUS_VOLTAGE:
            get_vbus_voltage_callback(axis, msg);
            break;
        default:
            break;
    }
}

bool CANSimple::send_message(const can_Message_t& msg) {
    if (!ready_) { return false; }
    can::Frame f;
    f.id = msg.id;
    f.is_extended = msg.isExt;
    f.dlc = msg.len;
    f.is_rtr = msg.rtr;
    std::copy(std::begin(msg.buf), std::end(msg.buf), std::begin(f.data));
    return canbus_->send(f);
}

bool CANSimple::send_message(const Axis& axis, const CanSimpleMessage type, const bool rtr)
{
    if (!ready_) { return false; }
    can::Frame f;
    f.id = axis.node_id << NUM_CMD_ID_BITS;
    f.id += type; 
    f.is_extended = axis.is_extended;
    f.dlc = 1;
    f.is_rtr = rtr;
    return canbus_->send(f);
}

bool CANSimple::get_motor_error(const Axis& axis) {
    return send_message(axis, MSG_GET_MOTOR_ERROR, true);
}

bool CANSimple::get_encoder_error(const Axis& axis) {
    return send_message(axis, MSG_GET_ENCODER_ERROR, true);
}

bool CANSimple::get_sensorless_error(const Axis& axis) {
    return send_message(axis, MSG_GET_SENSORLESS_ERROR, true);
}

bool CANSimple::get_encoder_estimates(const Axis& axis) {
    return send_message(axis, MSG_GET_ENCODER_ESTIMATES, true);
}

bool CANSimple::get_encoder_count(const Axis& axis) {
    return send_message(axis, MSG_GET_ENCODER_COUNT, true);
}

bool CANSimple::get_iq(const Axis& axis) {
    return send_message(axis, MSG_GET_IQ, true);
}

bool CANSimple::get_sensorless_estimates(const Axis& axis) {
    return send_message(axis, MSG_GET_SENSORLESS_ESTIMATES, true);
}

bool CANSimple::get_vbus_voltage(const Axis& axis) {
    return send_message(axis, MSG_GET_VBUS_VOLTAGE, true);
}

void CANSimple::hearbeat_callback(Axis& axis, const can_Message_t& msg) {
    axis.axis_error = static_cast<AxisError>(can_getSignal<uint32_t>(msg, 0, 32, true));
    axis.axis_state = static_cast<AxisState>(can_getSignal<uint8_t>(msg, 32, 8, true));
    // uint8_t motor_flags = can_getSignal<uint8_t>(msg, 40, 8, true);     // reserved
    // uint8_t encoder_flags = can_getSignal<uint8_t>(msg, 48, 8, true);   // reserved
    axis.controller_flags = can_getSignal<uint8_t>(msg, 56, 8, true);
}

void CANSimple::get_motor_error_callback(Axis& axis, const can_Message_t& msg) {
    axis.motor_error = static_cast<MotorError>(can_getSignal<uint64_t>(msg, 0, 64, true));
}

void CANSimple::get_encoder_error_callback(Axis& axis, const can_Message_t& msg) {
    axis.encoder_error = static_cast<EncoderError>(can_getSignal<uint32_t>(msg, 0, 32, true));
}

void CANSimple::get_sensorless_error_callback(Axis& axis, const can_Message_t& msg) {
    axis.sensorless_estimator_error = static_cast<SensorlessEstimatorError>(
        can_getSignal<uint32_t>(msg, 0, 32, true));
}

void CANSimple::get_encoder_estimates_callback(Axis& axis, const can_Message_t& msg) {
    axis.pos_enc_estimate = can_getSignal<float>(msg, 0, 32, true);
    axis.vel_enc_estimate = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::get_encoder_count_callback(Axis& axis, const can_Message_t& msg) {
    axis.encoder_shadow_count = can_getSignal<int32_t>(msg, 0, 32, true);
    axis.encoder_count_cpr = can_getSignal<int32_t>(msg, 32, 32, true);
}

void CANSimple::get_iq_callback(Axis& axis, const can_Message_t& msg) {
    axis.idq_first = can_getSignal<float>(msg, 0, 32, true);
    axis.idq_second = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::get_sensorless_estimates_callback(Axis& axis, const can_Message_t& msg) {
    axis.pos_sensorless_estimate = can_getSignal<float>(msg, 0, 32, true);
    axis.vel_sensorless_estimate = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::get_vbus_voltage_callback(Axis& axis, const can_Message_t& msg) {
    axis.vbus_voltage = can_getSignal<float>(msg, 0, 32, true);
}

bool CANSimple::set_axis_nodeid(const Axis& axis, const uint32_t &node_id) {
    // This seems like it could cause trouble, so not implemented for now. 
    // can_Message_t txmsg;
    // txmsg.id = axis.node_id << NUM_CMD_ID_BITS;
    // txmsg.id += MSG_SET_AXIS_NODE_ID; 
    // txmsg.isExt = axis.is_extended;
    // txmsg.len = 8;
    // can_setSignal(txmsg, node_id, 0, 32, true);
    // return send_message(txmsg);
    return false;
}

bool CANSimple::set_axis_requested_state(const Axis& axis, const AxisState &state) {
    can_Message_t txmsg;
    txmsg.id = axis.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_SET_AXIS_REQUESTED_STATE;
    txmsg.isExt = axis.is_extended;
    txmsg.len = 8;
    can_setSignal(txmsg, state, 0, 32, true);
    return send_message(txmsg);
}

bool CANSimple::set_axis_startup_config(const Axis& axis) {
    // Not Implemented
    return false;
}

bool CANSimple::set_input_pos(const Axis& axis, const float pos, const int16_t vel_ff, const int16_t torque_ff) {
    can_Message_t txmsg;
    txmsg.id = axis.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_SET_INPUT_POS; 
    txmsg.isExt = axis.is_extended;
    txmsg.len = 8;
    can_setSignal<float>(txmsg, pos, 0, 32, true);
    can_setSignal<int16_t>(txmsg, vel_ff, 32, 16, true, 0.001f, 0);
    can_setSignal<int16_t>(txmsg, torque_ff, 48, 16, true, 0.001f, 0);
    return send_message(txmsg);
}

bool CANSimple::set_input_vel(const Axis& axis, const float vel, const float torque_ff) {
    can_Message_t txmsg;
    txmsg.id = axis.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_SET_INPUT_VEL; 
    txmsg.isExt = axis.is_extended;
    txmsg.len = 8;
    can_setSignal<float>(txmsg, vel, 0, 32, true);
    can_setSignal<float>(txmsg, torque_ff, 32, 32, true);
    return send_message(txmsg);
}

bool CANSimple::set_input_torque(const Axis& axis, const float torque) {
    can_Message_t txmsg;
    txmsg.id = axis.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_SET_INPUT_TORQUE; 
    txmsg.isExt = axis.is_extended;
    txmsg.len = 8;
    can_setSignal<float>(txmsg, torque, 0, 32, true);
    return send_message(txmsg);
}

bool CANSimple::set_controller_modes(const Axis& axis, ControlMode ctrl_mode, InputMode input_mode) {
    can_Message_t txmsg;
    txmsg.id = axis.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_SET_CONTROLLER_MODES; 
    txmsg.isExt = axis.is_extended;
    txmsg.len = 8;
    can_setSignal<int32_t>(txmsg, ctrl_mode, 0, 32, true);
    can_setSignal<int32_t>(txmsg, input_mode, 32, 32, true);
    return send_message(txmsg);
}

bool CANSimple::set_limits(const Axis& axis, const float vel_limit, const float torque_limit) {
    can_Message_t txmsg;
    txmsg.id = axis.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_SET_LIMITS; 
    txmsg.isExt = axis.is_extended;
    txmsg.len = 8;
    can_setSignal<float>(txmsg, vel_limit, 0, 32, true);
    can_setSignal<float>(txmsg, torque_limit, 32, 32, true);
    return send_message(txmsg);
}

bool CANSimple::set_traj_vel_limit(const Axis& axis, const float vel_limit) {
    can_Message_t txmsg;
    txmsg.id = axis.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_SET_TRAJ_VEL_LIMIT; 
    txmsg.isExt = axis.is_extended;
    txmsg.len = 8;
    can_setSignal<float>(txmsg, vel_limit, 0, 32, true);
    return send_message(txmsg);
}

bool CANSimple::set_traj_accel_limits(const Axis& axis, const float accel, const float decel) {
    can_Message_t txmsg;
    txmsg.id = axis.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_SET_TRAJ_ACCEL_LIMITS; 
    txmsg.isExt = axis.is_extended;
    txmsg.len = 8;
    can_setSignal<float>(txmsg, accel, 0, 32, true);
    can_setSignal<float>(txmsg, decel, 32, 32, true);
    return send_message(txmsg);
}

bool CANSimple::set_traj_inertia(const Axis& axis, const float inertia) {
    can_Message_t txmsg;
    txmsg.id = axis.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_SET_TRAJ_INERTIA; 
    txmsg.isExt = axis.is_extended;
    txmsg.len = 8;
    can_setSignal<float>(txmsg, inertia, 0, 32, true);
    return send_message(txmsg);
}

bool CANSimple::set_linear_count(const Axis& axis, const int32_t count) {
    can_Message_t txmsg;
    txmsg.id = axis.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_SET_LINEAR_COUNT; 
    txmsg.isExt = axis.is_extended;
    txmsg.len = 8;
    can_setSignal<int32_t>(txmsg, count, 0, 32, true);
    return send_message(txmsg);
}

bool CANSimple::set_pos_gain(const Axis& axis, const float pgain) {
    can_Message_t txmsg;
    txmsg.id = axis.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_SET_POS_GAIN; 
    txmsg.isExt = axis.is_extended;
    txmsg.len = 8;
    can_setSignal<float>(txmsg, pgain, 0, 32, true);
    return send_message(txmsg);
}

bool CANSimple::set_vel_gains(const Axis& axis, const float pgain, const float igain) {
    can_Message_t txmsg;
    txmsg.id = axis.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_SET_VEL_GAINS; 
    txmsg.isExt = axis.is_extended;
    txmsg.len = 8;
    can_setSignal<float>(txmsg, pgain, 0, 32, true);
    can_setSignal<float>(txmsg, igain, 0, 32, true);
    return send_message(txmsg);
}

bool CANSimple::nmt(const Axis& axis) {
    // Not implemented
    return false;
}

bool CANSimple::estop(const Axis& axis) {
    return send_message(axis, MSG_ODRIVE_ESTOP);
}

bool CANSimple::clear_errors(const Axis& axis) {
    return send_message(axis, MSG_CLEAR_ERRORS);
}

bool CANSimple::start_anticogging(const Axis& axis) {
    return send_message(axis, MSG_START_ANTICOGGING);
}

bool CANSimple::reset_odrive(const Axis& axis) {
    return send_message(axis, MSG_RESET_ODRIVE);
}


} // namespace
