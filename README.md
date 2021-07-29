# odrive_can_espidf
[ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html?highlight=component) component for [ODrive CAN interface](https://docs.odriverobotics.com/can-protocol).
Tested on **ESP-IDF v4.3** with **ODrive fw-v0.5.2**.

**For example usage, see -> [example.c](example.c)**

## ODrive Configuration
There is a built-in DIP switch that toggles the 120Ω CAN termination resistor (`CAN-R` is off `CAN+R` is on).

The following fields must be configured for each axis in [odrivetool](https://docs.odriverobotics.com/odrivetool.html):

```python
# currently the component only support consecutive IDs starting from 0
odrv0.axis0.config.can.node_id = 0 # 0 - 0x3F

# set can baudrate
odrv0.can.config.baud_rate = 250000 # max 1000000 (1Mbps)

# set encoder rate
odrv0.axis0.config.can.encoder_rate_ms = 10

# set heartbeat rate
odrv0.axis0.config.can.heartbeat_rate_ms = 100

# enable can output
odrv0.config.enable_can_a = True

# save and reboot
odrv0.save_configuration()
```