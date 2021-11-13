odrv0.config.enable_brake_resistor = True
odrv0.config.dc_max_negative_current = -1.0 
odrv0.axis0.config.can.node_id = 2
odrv0.axis1.config.can.node_id = 3
odrv0.can.config.baud_rate = 250000



odrv0.axis0.motor.config.pole_pairs = 15
odrv0.axis0.motor.config.resistance_calib_max_voltage = 4
odrv0.axis0.motor.config.requested_current_range = 25 #Requires config save and reboot
odrv0.axis0.motor.config.current_control_bandwidth = 100
odrv0.axis0.motor.config.torque_constant = 8.27 / 16.0
odrv0.axis0.encoder.config.mode = ENCODER_MODE_HALL
odrv0.axis0.encoder.config.cpr = 90
odrv0.axis0.encoder.config.calib_scan_distance = 150
odrv0.config.gpio9_mode = GPIO_MODE_DIGITAL
odrv0.config.gpio10_mode = GPIO_MODE_DIGITAL
odrv0.config.gpio11_mode = GPIO_MODE_DIGITAL
odrv0.axis0.encoder.config.bandwidth = 100
odrv0.axis0.controller.config.pos_gain = 1
odrv0.axis0.controller.config.vel_gain = 0.02 * odrv0.axis0.motor.config.torque_constant * odrv0.axis0.encoder.config.cpr
odrv0.axis0.controller.config.vel_integrator_gain = 0.1 * odrv0.axis0.motor.config.torque_constant * odrv0.axis0.encoder.config.cpr
odrv0.axis0.controller.config.vel_limit = 10
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.save_configuration()

### WAIT FOR REBOOT

odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
odrv0.axis0.motor # Make sure there is no error here before proceeding
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.encoder.config.ignore_illegal_hall_state = True
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
odrv0.axis0.encoder.config.hall_polarity_calibrated = True
odrv0.save_configuration()

### WAIT FOR ENCODER CALIB

odrv0.axis0.encoder # Make sure there is no error here before proceeding
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.save_configuration()

###############################################################################

odrv0.axis1.motor.config.pole_pairs = 15
odrv0.axis1.motor.config.resistance_calib_max_voltage = 4
odrv0.axis1.motor.config.requested_current_range = 25 #Requires config save and reboot
odrv0.axis1.motor.config.current_control_bandwidth = 100
odrv0.axis1.motor.config.torque_constant = 8.27 / 16.0
odrv0.axis1.encoder.config.mode = ENCODER_MODE_HALL
odrv0.axis1.encoder.config.cpr = 90
odrv0.axis1.encoder.config.calib_scan_distance = 150
odrv0.config.gpio12_mode = GPIO_MODE_DIGITAL
odrv0.config.gpio13_mode = GPIO_MODE_DIGITAL
odrv0.config.gpio14_mode = GPIO_MODE_DIGITAL
odrv0.axis1.encoder.config.bandwidth = 100
odrv0.axis1.controller.config.pos_gain = 1
odrv0.axis1.controller.config.vel_gain = 0.02 * odrv0.axis0.motor.config.torque_constant * odrv0.axis0.encoder.config.cpr
odrv0.axis1.controller.config.vel_integrator_gain = 0.1 * odrv0.axis0.motor.config.torque_constant * odrv0.axis0.encoder.config.cpr
odrv0.axis1.controller.config.vel_limit = 10
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.save_configuration()

### WAIT FOR REBOOT

odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
odrv0.axis1.motor # Make sure there is no error here before proceeding
odrv0.axis1.motor.config.pre_calibrated = True
odrv0.axis1.encoder.config.ignore_illegal_hall_state = True
odrv0.axis1.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
odrv0.axis1.encoder.config.hall_polarity_calibrated = True
odrv0.save_configuration()

### WAIT FOR ENCODER CALIB

odrv0.axis1.encoder # Make sure there is no error here before proceeding
odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrv0.axis1.encoder.config.pre_calibrated = True
odrv0.save_configuration()

###############################################################################

odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_vel = 2# Your motor should spin here
odrv0.axis0.controller.input_vel = 0
odrv0.axis0.requested_state = AXIS_STATE_IDLE

odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.controller.input_vel = 2
odrv0.axis1.controller.input_vel = 0
odrv0.axis1.requested_state = AXIS_STATE_IDLE
