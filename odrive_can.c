#include <string.h>
#include <esp_log.h>
#include "driver/twai.h"
#include "odrive_can.h"

static void log_msg(const twai_message_t* msg, const char* str) {
    ESP_LOGW("ODriveCan", "%s (id %x flags %x len %d)", str, msg->identifier, msg->flags,
            msg->data_length_code);
}

esp_err_t odrive_send_command(uint8_t axis_id, uint8_t cmd_id, const void* buf, int len) {
    twai_message_t msg = {};
    msg.identifier = ODRIVE_MSG_ID(axis_id, cmd_id);
    msg.data_length_code = len;
    memcpy(msg.data, buf, len);
    return twai_transmit(&msg, 0);
}

esp_err_t odrive_receive_updates(ODriveAxis* axes, uint8_t len) {
    assert(axes);
    assert(len < 32);
    esp_err_t error;
    twai_message_t msg;
    while ((error = twai_receive(&msg, 0)) == ESP_OK) {
        if (msg.flags) {
            log_msg(&msg, "received message with invalid flag");
            continue;
        }
        // check axis id
        uint8_t axis_id = ODRIVE_AXIS_ID(msg.identifier);
        if (axis_id >= len) {
            log_msg(&msg, "received message with invalid identifier");
            continue;
        }
        // check message length
        uint8_t cmd_id = ODRIVE_CMD_ID(msg.identifier);
        switch (cmd_id) {
            // fallthrough
            case ODRIVE_CMD_GET_ENCODER_ERROR:
            case ODRIVE_CMD_GET_SENSORLESS_ERROR:
            case ODRIVE_CMD_GET_VBUS_VOLTAGE:
                if (msg.data_length_code != 4) {
                    log_msg(&msg, "received message with invalid length");
                    continue;
                }
                break;
            default:
                if (msg.data_length_code != 8) {
                    log_msg(&msg, "received message with invalid length");
                    continue;
                }
        }
        // parse message
        switch (cmd_id) {
            case ODRIVE_CMD_HEARTBEAT: {
                ODriveAxisState old_state = axes[axis_id].heartbeat.axis_current_state;
                axes[axis_id].heartbeat = *(ODriveHeartbeat*) msg.data;
                if (old_state != axes[axis_id].heartbeat.axis_current_state &&
                        axes[axis_id].state_transition_callback) {
                    axes[axis_id].state_transition_callback(axis_id,
                            axes[axis_id].heartbeat.axis_current_state, old_state,
                            axes[axis_id].state_transition_context);
                }
                axes[axis_id].updates.heartbeat = true;
                break;
            }
            case ODRIVE_CMD_GET_ENCODER_COUNT:
                axes[axis_id].encoder_count = *(ODriveEncoderCount*) msg.data;
                axes[axis_id].updates.encoder_count = true;
                break;
            case ODRIVE_CMD_GET_ENCODER_ESTIMATES:
                axes[axis_id].encoder_estimates = *(ODriveEncoderEstimates*) msg.data;
                axes[axis_id].updates.encoder_estimates = true;
                break;
            case ODRIVE_CMD_GET_SENSORLESS_ESTIMATES:
                axes[axis_id].sensorless_estimates = *(ODriveSensorlessEstimates*) msg.data;
                axes[axis_id].updates.sensorless_estimates = true;
                break;
            case ODRIVE_CMD_GET_IQ:
                axes[axis_id].iq = *(ODriveIq*) msg.data;
                axes[axis_id].updates.iq = true;
                break;
            case ODRIVE_CMD_GET_VBUS_VOLTAGE:
                axes[axis_id].vbus_voltage = *(float*) msg.data;
                axes[axis_id].updates.vbus_voltage = true;
                break;
            case ODRIVE_CMD_GET_MOTOR_ERROR:
                axes[axis_id].motor_error = *(uint64_t*) msg.data;
                axes[axis_id].updates.motor_error = true;
                break;
            case ODRIVE_CMD_GET_ENCODER_ERROR:
                axes[axis_id].encoder_error = *(uint32_t*) msg.data;
                axes[axis_id].updates.encoder_error = true;
                break;
            case ODRIVE_CMD_GET_SENSORLESS_ERROR:
                axes[axis_id].sensorless_error = *(uint32_t*) msg.data;
                axes[axis_id].updates.sensorless_error = true;
                break;
            default:
                log_msg(&msg, "received message with invalid command");
                break;
        }
    }
    return error == ESP_ERR_TIMEOUT ? ESP_OK : error;
}
