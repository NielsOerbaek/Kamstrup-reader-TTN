# ESP32-based reader for the Kamstrup Omnipower, sending over The Things Network

__NOTE: This will probably only work for Radius customers in the greater Copenhagen area. You need to write to [kundesupport@radiuselnet.dk](mailto:kundesupport@radiuselnet.dk) to get your decryption keys__

This sketch was made for a Heltec Lora 32 (V1), but it might work for other ESP32-based chips as well.

### Hookup guide:

1. Rename `secrets.h.TEMPLATE` to `secrets.h` and insert your Keys from Radius and your device/app info from The Things Network.
2. Upload sketch to board, see notes if this is trouble.
2. Connect pin: Heltec `G` pin <-> Kamstrup HAN slot top left pin
2. Connect pin: Heltec `23` pin <-> Kamstrup HAN slot top middle pin
3. Power the Heltec through USB charger


### History:
Adapted from [Claustn](https://github.com/Claustn/esp8266-kamstrup-mqtt) to use Lora,TTN instead of WiFi,MQTT.

Claustn adapted his code from [Asbjoern](https://github.com/Asbjoern/Kamstrup-Radius-Interface/) whose repo has many more features than this little thing.

---

## Notes:

- `esptool.py erase_flash` to reset the device, install via `pip install esptool`
- I am using the Heltec WiFi Lora 32, not V2.
- Mostly followed this [guide](https://nathanmcminn.com/2018/09/12/tutorial-heltec-esp32-board-the-things-network/)
    - Notably [this guide for installing esp32 support](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/debian_ubuntu.md)
    - Install [this LMIC library](https://github.com/matthijskooijman/arduino-lmic)