# ESP32-based reader for the Kamstrup Omnipower, Sending over The Things Network

### Notes:

    - `esptool.py erase_flash` to reset the device, install via `pip install esptool`
    - We are using the Heltec WiFi Lora 32, not V2.
    - Mostly followed this [guide](https://nathanmcminn.com/2018/09/12/tutorial-heltec-esp32-board-the-things-network/)
        - Notably [this guide for installing esp32 support](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/debian_ubuntu.md)
        - Install [this LMIC library](https://github.com/matthijskooijman/arduino-lmic)
    - But also installed this [library](https://github.com/HelTecAutomation/ESP32_LoRaWAN/archive/master.zip)
        - Not completely sure if it did anything...
