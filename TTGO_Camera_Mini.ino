#include "esp_camera.h"
#include <WiFi.h>
#include "esp_wifi.h"
#include <Wire.h>

//! Set 1 to AP mode and 0 to station mode
#define SOFTAP_MODE 1


const uint8_t i2c_sda = 21;
const uint8_t i2c_scl = 22;
const uint8_t cap_touch = 33;

#if 0
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    32
#define SIOD_GPIO_NUM    13
#define SIOC_GPIO_NUM    12

#define Y9_GPIO_NUM      39
#define Y8_GPIO_NUM      36
#define Y7_GPIO_NUM      23
#define Y6_GPIO_NUM      18
#define Y5_GPIO_NUM      15
#define Y4_GPIO_NUM      4
#define Y3_GPIO_NUM      14
#define Y2_GPIO_NUM      5

#define VSYNC_GPIO_NUM   27
#define HREF_GPIO_NUM    25
#define PCLK_GPIO_NUM    19

#else

#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    32
#define SIOD_GPIO_NUM    13
#define SIOC_GPIO_NUM    12

#define Y9_GPIO_NUM      39
#define Y8_GPIO_NUM      36
#define Y7_GPIO_NUM      38
#define Y6_GPIO_NUM      37
#define Y5_GPIO_NUM      15
#define Y4_GPIO_NUM      4
#define Y3_GPIO_NUM      14
#define Y2_GPIO_NUM      5

#define VSYNC_GPIO_NUM   27
#define HREF_GPIO_NUM    25
#define PCLK_GPIO_NUM    19

#endif
/***************************************
 *  WiFi
 **************************************/
#define WIFI_SSID   "your wifi ssid"
#define WIFI_PASSWD "you wifi password"

void startCameraServer();


void i2cWrite(uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(0x34);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t i2cRead(uint8_t reg)
{
    Wire.beginTransmission(0x34);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(0x34, 1);
    return Wire.read();
}


void setup()
{
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.println();

    pinMode(cap_touch, INPUT);

    //! USB current limit must be disabled
    Wire.begin(i2c_sda, i2c_scl);

    uint8_t val = i2cRead(0x30);
    Serial.printf("RVAL:%u\n", val);
    i2cWrite(0x30, val & 0xFC);
    val = i2cRead(0x30);
    Serial.printf("WVAL:%u\n", val);

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    //init with high specs to pre-allocate larger buffers
    if (psramFound()) {
        config.frame_size = FRAMESIZE_UXGA;
        config.jpeg_quality = 10;
        config.fb_count = 2;
    } else {
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }

    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    //initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);//flip it back
        s->set_brightness(s, 1);//up the blightness just a bit
        s->set_saturation(s, -2);//lower the saturation
    }
    //drop down frame size for higher initial frame rate
    s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#endif


    String ip;
    char buff[128];
#ifdef SOFTAP_MODE
    Serial.println("Configuring access point...");
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    sprintf(buff, "TTGO-CAMERA-%02X:%02X", mac[4], mac[5]);
    WiFi.softAP(buff);
    ip = WiFi.softAPIP().toString();
#else
    WiFi.begin(WIFI_SSID, WIFI_PASSWD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    ip = WiFi.localIP().toString();
#endif

    startCameraServer();

    Serial.print("Camera Ready! Use 'http://");
    Serial.print(ip);
    Serial.println("' to connect");
}

bool en = true;
void loop()
{
    if (digitalRead(cap_touch)) {
        sensor_t *s = esp_camera_sensor_get();
        s->set_vflip(s, en = !en);
        delay(1000);
    }
}
