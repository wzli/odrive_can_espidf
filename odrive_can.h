#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// refer to https://docs.odriverobotics.com/can-protocol
#define ODRIVE_MSG_ID(AXIS_ID, CMD_ID) ((AXIS_ID << 5) | CMD_ID)
#define ODRIVE_AXIS_ID(MSG_ID) (MSG_ID >> 5)
#define ODRIVE_CMD_ID(MSG_ID) (MSG_ID & 0x1F)

// Command IDs
#define ODRIVE_CMD_CANOPEN_NMT 0x00
#define ODRIVE_CMD_HEARTBEAT 0x01
#define ODRIVE_CMD_ESTOP 0x02
#define ODRIVE_CMD_GET_MOTOR_ERROR 0x03
#define ODRIVE_CMD_GET_ENCODER_ERROR 0x04
#define ODRIVE_CMD_GET_SENSORLESS_ERROR 0x05
#define ODRIVE_CMD_SET_AXIS_NODE_ID 0x06
#define ODRIVE_CMD_SET_REQUESTED_STATE 0x07
#define ODRIVE_CMD_SET_STARTUP_CONFIG 0x08
#define ODRIVE_CMD_GET_ENCODER_ESTIMATES 0x09
#define ODRIVE_CMD_GET_ENCODER_COUNT 0x0A
#define ODRIVE_CMD_SET_CONTROLLER_MODES 0x0B
#define ODRIVE_CMD_SET_INPUT_POS 0x0C
#define ODRIVE_CMD_SET_INPUT_VEL 0x0D
#define ODRIVE_CMD_SET_INPUT_TORQUE 0x0E
#define ODRIVE_CMD_SET_LIMITS 0x0F
#define ODRIVE_CMD_START_ANTICOGGING 0x10
#define ODRIVE_CMD_SET_TRAJ_VEL_LIMIT 0x11
#define ODRIVE_CMD_SET_TRAJ_ACCEL_LIMITs 0x12
#define ODRIVE_CMD_SET_TRAJ_INERTIA 0x13
#define ODRIVE_CMD_GET_IQ 0x14
#define ODRIVE_CMD_GET_SENSORLESS_ESTIMATES 0x15
#define ODRIVE_CMD_REBOOT 0x16
#define ODRIVE_CMD_GET_VBUS_VOLTAGE 0x17
#define ODRIVE_CMD_CLEAR_ERRORS 0x18
#define ODRIVE_CMD_SET_LINEAR_COUNT 0x19
#define ODRIVE_CMD_CANOPEN_HEARTBEAT 0x700

// custom ODrive commands
//
// take single float input
#define ODRIVE_CMD_SET_POS_GAIN 0x1A
#define ODRIVE_CMD_SET_VEL_GAIN 0x1B
#define ODRIVE_CMD_SET_VEL_INTEGRATOR_GAIN 0x1C
#define ODRIVE_CMD_SET_CURRENT_CTRL_BW 0x1D
#define ODRIVE_CMD_SET_ENCODER_BW 0x1E

#define ODRIVE_CMD_GET_ODRIVE_ERROR 0x20
#define ODRIVE_CMD_SET_SAVE_CONFIGURATION 0x21

// Protocol Constants
typedef enum {
    ODRIVE_AXIS_STATE_UNDEFINED = 0,
    ODRIVE_AXIS_STATE_IDLE = 1,
    ODRIVE_AXIS_STATE_STARTUP_SEQUENCE = 2,
    ODRIVE_AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
    ODRIVE_AXIS_STATE_MOTOR_CALIBRATION = 4,
    // missing 5
    ODRIVE_AXIS_STATE_ENCODER_INDEX_SEARCH = 6,
    ODRIVE_AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,
    ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL = 8,
    ODRIVE_AXIS_STATE_LOCKIN_SPIN = 9,
    ODRIVE_AXIS_STATE_ENCODER_DIR_FIND = 10,
    ODRIVE_AXIS_STATE_HOMING = 11,
    ODRIVE_AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12,
    ODRIVE_AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 13,
} ODriveAxisState;

typedef enum {
    ODRIVE_AXIS_ERROR_INVALID_STATE = 0x1,
    ODRIVE_AXIS_ERROR_WATCHDOG_TIMER_EXPIRED = 0x800,
    ODRIVE_AXIS_ERROR_MIN_ENDSTOP_PRESSED = 0x1000,
    ODRIVE_AXIS_ERROR_MAX_ENDSTOP_PRESSED = 0x2000,
    ODRIVE_AXIS_ERROR_ESTOP_REQUESTED = 0x4000,
    ODRIVE_AXIS_ERROR_HOMING_WITHOUT_ENDSTOP = 0x20000,
    ODRIVE_AXIS_ERROR_OVER_TEMP = 0x40000,
    ODRIVE_AXIS_ERROR_UNKNOWN_POSITION = 0x80000,
} ODriveAxisError;

typedef enum {
    ODRIVE_CONTROL_MODE_VOLTAGE = 0,
    ODRIVE_CONTROL_MODE_TORQUE = 1,
    ODRIVE_CONTROL_MODE_VELOCITY = 2,
    ODRIVE_CONTROL_MODE_POSITION = 3,
} ODriveControlMode;

typedef enum {
    ODRIVE_INPUT_MODE_INACTIVE = 0,
    ODRIVE_INPUT_MODE_PASSTHROUGH = 1,
    ODRIVE_INPUT_MODE_VEL_RAMP = 2,
    ODRIVE_INPUT_MODE_POS_FILTER = 3,
    ODRIVE_INPUT_MODE_MIX_CHANNELS = 4,
    ODRIVE_INPUT_MODE_TRAP_TRAJ = 5,
    ODRIVE_INPUT_MODE_TORQUE_RAMP = 6,
    ODRIVE_INPUT_MODE_MIRROR = 7,
    ODRIVE_INPUT_MODE_TUNING = 8,
} ODriveInputMode;

// Protocol Messages

// position in units of turns
// velocity in units of turns per second
// torque in units of Nm
// refer to
// https://github.com/odriverobotics/ODrive/blob/master/Firmware/odrive-interface.yaml

typedef struct {
    uint32_t axis_error;
    uint32_t axis_current_state;
} ODriveHeartbeat;

typedef struct {
    float position;
    float velocity;
} ODriveEncoderEstimates;

typedef struct {
    int32_t shadow_count;
    int32_t count_cpr;
} ODriveEncoderCount;

typedef struct {
    int32_t control_mode;
    int32_t input_mode;
} ODriveControllerModes;

typedef struct {
    float position;
    int16_t velocity_x1000;
    int16_t torque_x1000;
} ODriveInputPosition;

typedef struct {
    float velocity;
    float torque;
} ODriveInputVelocity;

typedef struct {
    float velocity_limit;
    float current_limit;
} ODriveInputLimits;

typedef struct {
    float accel_limit;
    float decel_limit;
} ODriveTrajAccelLimit;

typedef struct {
    float setpoint;
    float measured;
} ODriveIq;

typedef ODriveEncoderEstimates ODriveSensorlessEstimates;

typedef struct {
    bool motor_error : 1;
    bool odrive_error : 1;
    bool encoder_error : 1;
    bool sensorless_error : 1;
    bool heartbeat : 1;
    bool encoder_count : 1;
    bool encoder_estimates : 1;
    bool sensorless_estimates : 1;
    bool iq : 1;
    bool vbus_voltage : 1;
} ODriveUpdates;

typedef struct {
    uint64_t motor_error;
    uint32_t odrive_error;
    uint32_t encoder_error;
    uint32_t sensorless_error;
    ODriveHeartbeat heartbeat;
    ODriveEncoderCount encoder_count;
    ODriveEncoderEstimates encoder_estimates;
    ODriveSensorlessEstimates sensorless_estimates;
    ODriveIq iq;
    float vbus_voltage;
    ODriveUpdates updates;

    void (*state_transition_callback)(
            uint8_t axis_id, ODriveAxisState new_state, ODriveAxisState old_state, void* context);
    void* state_transition_context;
} ODriveAxis;

// Public Functions
esp_err_t odrive_send_command(uint8_t axis_id, uint8_t cmd_id, const void* buf, uint8_t len);
esp_err_t odrive_send_get_command(uint8_t axis_id, uint8_t cmd_id);
esp_err_t odrive_receive_updates(ODriveAxis* axes, uint8_t len);
