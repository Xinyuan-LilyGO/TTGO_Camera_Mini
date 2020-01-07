# Camera Mini


## Quick start
- Select ESP32 Dev Module in the Arduino board selection,and select Huge APP (3MB No OTA/1MB SPIFFS) in the Partition Scheme option,select Enable in PSRAM.
Replace ssid and password in the sketch with your hotspot and password Or leave the code unchanged use AP mode
- Upload a sketch to the board
- Connect to the hotspot turned on by the board, turn on the phone to scan wifi by default, you should be able to see the hotspots sent by the board, the default is `TTGO-CAMERA-xxxx`, the default is no password, open the browser and enter` 192.168.4.1` and you will see Web interface, slide to the bottom and click Start, you will see the real-time video stream
- There is a touch button under the camera, touch it, the camera screen will be reversed
- Before changing the code, you must know that this board uses integrated power management. Before coding, you must turn off the current limit of the AXP192, otherwise it may cause excessive current to cause protection.
- There are two versions of Camera Min. The earlier version is green PCB and the new version is black PCB. If using older version please turn on **OLD_VERSION_CAMERA** macro,in sketch **16 line**


| Name  | OLD(Green PCB) | NEW(Black PCB) |
| ----- | -------------- | -------------- |
| PWDN  | N/A            | N/A            |
| RESET | N/A            | N/A            |
| XCLK  | 32             | 32             |
| SIOD  | 13             | 13             |
| SIOC  | 12             | 12             |
| Y9    | 39             | 39             |
| Y8    | 36             | 36             |
| Y7    | 23             | 38             |
| Y6    | 18             | 37             |
| Y5    | 15             | 15             |
| Y4    | 4              | 4              |
| Y3    | 14             | 14             |
| Y2    | 5              | 5              |
| VSYNC | 27             | 27             |
| HREF  | 25             | 25             |
| PCLK  | 19             | 19             |
| SDA   | 21             | 21             |
| SCL   | 21             | 22             |
| TOUCH | 33             | 33             |