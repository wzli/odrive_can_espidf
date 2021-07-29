#include "odrive_can.h"
#include "driver/twai.h"
#include "esp_err.h"

#define PIN_CTX 12
#define PIN_CRX 13
#define N_MOTORS 2

static void can_init() {
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(PIN_CTX, PIN_CRX, TWAI_MODE_NORMAL);
    g_config.intr_flags |= ESP_INTR_FLAG_IRAM;
    g_config.rx_queue_len = 32;
    // make sure baudrate matches what is configured on the odrive
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
}

static void odrive_state_transition_callback(uint8_t axis_id,
        ODriveAxisState new_state, ODriveAxisState old_state, void* ctx) {
    printf("axis_id %u, new_state %u, old_state %u\n", axis_id,
            new_state, old_state);
}

void app_main() {
    can_init();

    // objects to store received updates, callbacks are optional
    ODriveAxis axes[N_MOTORS] = {
        {.state_transition_callback = odrive_state_transition_callback,
            .state_transition_context = NULL},
        {.state_transition_callback = odrive_state_transition_callback,
            .state_transition_context = NULL}
    };

    for (int i = 0; i < N_MOTORS; ++i) {
        // send command for position control mode
        ODriveControllerModes mode = {
            ODRIVE_CONTROL_MODE_POSITION,
            ODRIVE_INPUT_MODE_PASSTHROUGH,
        };
        ESP_ERROR_CHECK(odrive_send_command(i, ODRIVE_CMD_SET_CONTROLLER_MODES
                    &state, sizeof(state)));
        // send command to request closed loop state
        static const uint32_t state = ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL;
        ESP_ERROR_CHECK(odrive_send_command(i, ODRIVE_CMD_SET_REQUESTED_STATE,
                    &state, sizeof(state)));
        // send command for position control
        ODriveControlMode
    }

    ESP_ERROR_CHECK(odrive_receive_updates(odrives, N_MOTORS));
}
