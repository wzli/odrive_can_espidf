#include "odrive_can.h"
#include "driver/twai.h"
#include "freertos/task.h"

#define PIN_CAN_TX 12
#define PIN_CAN_RX 13
#define CAN_BAUD_RATE TWAI_TIMING_CONFIG_250KBITS
#define N_MOTORS 2

static void can_init() {
    twai_general_config_t g_config =
            TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
    g_config.intr_flags |= ESP_INTR_FLAG_IRAM;
    g_config.rx_queue_len = 32;
    g_config.tx_queue_len = 32;
    // make sure baudrate matches what is configured on the odrive
    twai_timing_config_t t_config = CAN_BAUD_RATE();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
}

static void odrive_state_transition_callback(
        uint8_t axis_id, ODriveAxisState new_state, ODriveAxisState old_state, void* context) {
    printf("axis_id %u, new_state %u, old_state %u\n", axis_id, new_state, old_state);
}

void app_main() {
    can_init();
    vTaskDelay(1);
    // objects to store received updates, callbacks are optional
    ODriveAxis axes[N_MOTORS] = {{.state_transition_callback = odrive_state_transition_callback,
                                         .state_transition_context = NULL},
            {.state_transition_callback = odrive_state_transition_callback,
                    .state_transition_context = NULL}};
    // initialize odrive axes
    for (int i = 0; i < N_MOTORS; ++i) {
        // send command for position control mode
        ODriveControllerModes mode = {
                ODRIVE_CONTROL_MODE_POSITION,
                ODRIVE_INPUT_MODE_PASSTHROUGH,
        };
        ESP_ERROR_CHECK(
                odrive_send_command(i, ODRIVE_CMD_SET_CONTROLLER_MODES, &mode, sizeof(mode)));
        vTaskDelay(1);
    }
    // main loop
    for (TickType_t tick = xTaskGetTickCount();; vTaskDelayUntil(&tick, 10)) {
        for (int i = 0; i < N_MOTORS; ++i) {
            ESP_ERROR_CHECK(odrive_send_command(i, ODRIVE_CMD_GET_MOTOR_ERROR, 0, 0));
            vTaskDelay(1);
            ESP_ERROR_CHECK(odrive_send_command(i, ODRIVE_CMD_GET_ENCODER_ERROR, 0, 0));
            vTaskDelay(1);
            uint32_t state = ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL;
            if (axes[i].heartbeat.axis_current_state != state) {
                // send command to request closed loop state
                ESP_ERROR_CHECK(odrive_send_command(
                        i, ODRIVE_CMD_SET_REQUESTED_STATE, &state, sizeof(state)));
            } else {
                // send command for position control
                ODriveInputPosition input_pos = {tick / 100, 0, 0};
                ESP_ERROR_CHECK(odrive_send_command(
                        i, ODRIVE_CMD_SET_INPUT_POS, &input_pos, sizeof(input_pos)));
            }
            vTaskDelay(1);
            // receive and print updates
            ESP_ERROR_CHECK(odrive_receive_updates(axes, N_MOTORS));
            printf("tick %u, axis_id %u, state %u, axis_err %u, motor_err %llu, enc_err %u, pos "
                   "%f, vel %f\n",
                    tick, i, axes[i].heartbeat.axis_current_state, axes[i].heartbeat.axis_error,
                    axes[i].motor_error, axes[i].encoder_error, axes[i].encoder_estimates.position,
                    axes[i].encoder_estimates.velocity);
        }
    }
}
