# Camera Mini


## Quick start
- Select ESP32 Dev Module in the Arduino board selection,and select Huge APP (3MB No OTA/1MB SPIFFS) in the Partition Scheme option,select Enable in PSRAM.
Replace ssid and password in the sketch with your hotspot and password Or leave the code unchanged use AP mode
- Upload a sketch to the board
- Connect to the hotspot turned on by the board, turn on the phone to scan wifi by default, you should be able to see the hotspots sent by the board, the default is `TTGO-CAMERA-xxxx`, the default is no password, open the browser and enter` 192.168.4.1` and you will see Web interface, slide to the bottom and click Start, you will see the real-time video stream
- There is a touch button under the camera, touch it, the camera screen will be reversed
- Before changing the code, you must know that this board uses integrated power management. Before coding, you must turn off the current limit of the AXP192, otherwise it may cause excessive current to cause protection.