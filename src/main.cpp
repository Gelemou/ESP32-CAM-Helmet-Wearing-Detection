#include <Arduino.h>
#include "esp_camera.h"
#include "camera_pins.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <ThingsCloudWiFiManager.h>
#include <ThingsCloudMQTT.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h> // ADC采集
#include <Adafruit_GFX.h> // 显示屏
#include <Adafruit_SSD1306.h>

// Has PSRAM
#define CAMERA_MODEL_AI_THINKER 

// I2C引脚配置
#define I2C_SDA 15
#define I2C_SCL 14

// OLED显示屏参数
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRES 0x3c

// 创建实例连接
Adafruit_ADS1015 ads; // ADS1015
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// wifi账号密码
const char *ssid = "";
const char *password = "";

// MQTT连接
#define THINGSCLOUD_MQTT_HOST "bj-2-mqtt.iot-api.com"
#define THINGSCLOUD_DEVICE_ACCESS_TOKEN "4uf8na3exqbwduor"
#define THINGSCLOUD_PROJECT_KEY "QWn2NFlI9U"
ThingsCloudMQTT mqtt_client(
    THINGSCLOUD_MQTT_HOST,
    THINGSCLOUD_DEVICE_ACCESS_TOKEN,
    THINGSCLOUD_PROJECT_KEY);

// const char *mqtt_server = "bj-2-mqtt.iot-api.com";
// const int mqtt_port = 1883; // MQTT端口
// const char *mqtt_topic_sub = "esp32-cam/helmet/result"; // 订阅主题
// const char *access_token = "4uf8na3exqbwduor";

WiFiClient esp32Client;
// 必须实现这个回调函数，当 MQTT 连接成功后执行该函数。
void onMQTTConnect()
{
    Serial.println("MQTT 连接成功");
}
void pubSensors()
{
    // 这个示例模拟传感器数值，仅用于演示如何生成 JSON。实际项目中可读取传感器真实数据。
    DynamicJsonDocument obj(512);
    obj["temperature"] = 31.2;
    obj["humidity"] = 62.5;
    obj["co2"] = 2321;
    obj["light"] = 653;
    char attributes[512];
    serializeJson(obj, attributes);
    // 调用属性上报方法
    mqtt_client.reportAttributes(attributes);
}
//PubSubClient mqtt_client(esp32Client);
// mqtt回调函数
// void callback(char *topic, byte *payload, unsigned int length) {
//     Serial.print("收到主题: ");
//     Serial.println(topic);
//     Serial.print("内容: ");
//     for (int i = 0; i < length; i++) Serial.print((char)payload[i]);
//     Serial.println();
// }
// mqtt重连函数
// void reconnect() {
//     while (!mqtt_client.connect("esp32-cam")) {
//         if (mqtt_client.connect("esp32-cam")) {
//             Serial.println("已连接到 MQTT");
//            // mqtt_client.subscribe("v1/devices/me/telemetry");
//         } else {
//             Serial.println("连接到 MQTT 失败，重试...");
//             delay(2000);
//         }
//     }
// }
void startCameraServer();
void camera_init() {
    erial.setDebugOutput(true);
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

    if (psramFound()) {
        config.frame_size = FRAMESIZE_XGA;   // 1600x1200
        config.jpeg_quality = 10;             // 数值越小画质越好
        config.fb_count = 1;                  // 降低内存压力
    } else {
        Serial.println("PSRAM not found");

    }
    // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
    //                      for larger pre-allocated frame buffer.

    #if defined(CAMERA_MODEL_ESP_EYE)
        pinMode(13, INPUT_PULLUP);
        pinMode(14, INPUT_PULLUP);
    #endif

    // camera init
        esp_err_t err = esp_camera_init(&config);
        if (err != ESP_OK) {
            Serial.printf("Camera init failed with error 0x%x", err);
            return;
        }

        sensor_t *s = esp_camera_sensor_get();
        // initial sensors are flipped vertically and colors are a bit saturated
        if (s->id.PID == OV2640_PID) {
            s->set_vflip(s, 1); // flip it back
            s->set_brightness(s, 1); // up the brightness just a bit
            s->set_saturation(s, -2); // lower the saturation
        }
        // drop down frame size for higher initial frame rate
        s->set_framesize(s, FRAMESIZE_XGA);

    #if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
        s->set_vflip(s, 1);
        s->set_hmirror(s, 1);
    #endif
}
void setup() {
    Serial.begin(115200);
    // 初始化I2C总线
    Wire.begin(I2C_SDA, I2C_SCL);
    // 初始化OLED屏幕
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRES)) {
        Serial.println(F("SSD1306分配失败"))
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(F("OLED初始化成功"));
    display.display();
    delay(1000);
    //初始化ADS1015，显式传入地址0x48
    if (!ads.begin(0x48, &Wire)) {
        Serial.println("ADS1015初始化失败");
    } else {
        Serial.println("ADS1015初始化成功");
    }
    // 板载LED关闭
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);
    //  允许 SDK 的日志输出
    mqtt_client.enableDebuggingMessages();

    camera_init();

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");

    startCameraServer();

    Serial.print("Camera Ready! Use 'http://");
    Serial.print(WiFi.localIP());
    Serial.println("' to connect");

    // 设置MQTT服务器和回调函数
    // mqtt_client.setServer(mqtt_server, mqtt_port);
    // mqtt_client.setCallback(callback);

}
// 控制拍照频率
unsigned long lastTime = 0;
const int interval = 500; // 0.5s
// 控制mqtt发送频率
unsigned long lastMsgTime = 0;
const long mqttInterval = 2000;  // 2s
void loop() {
    int16_t adc_0 = ads.readADC_SingleEnded(0);
    //计算电压值(根据默认增益计算，每位3mV)
    float voltage = ads.computeVolts(adc0);
    // 串口打印
    Serial.print("光敏电阻电压:");
    Serial.print(voltage);
    //OLED屏幕更新显示
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("ESP32-CAM I2C Monitor");
    display.println("-----------------");
    display.setTextSize(2);
    display.print("ADC:");
    display.println(adc0);
    display.setTextSize(1);
    display.print("Volts:");
    display.print(voltage);
    display.println("V");
    display.display();

    // 连接MQTT服务器
    mqtt_client.loop();
    // if (!mqtt_client.connect("esp32-cam")) reconnect();
    unsigned long now = millis();
    now = millis();
    if (now - lastMsgTime >= mqttInterval) {
        lastMsgTime = now;
        // 发布MQTT消息
        pubSensors();
        //mqtt_client.publish("attributes", ("{\"temperature\": " + String(temperature) + ", \"humidity\": " + String(humidity) + "}").c_str());
        //mqtt_client.publish("attributes", ("{\"brightness\"}" ":" + String(brightness)).c_str());
    }
        // put your main code here, to run repeatedly:
        // 拍照
    if (millis() - lastTime > interval) {
        lastTime = millis();

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            return;
        }
        // 上传图片到Flask服务器
        WiFiClient client;
        HTTPClient http;
        http.begin(client, "http://192.168.1.3:8080/upload"); // Flask服务器地址
        http.addHeader("Content-Type", "image/jpeg");
        int httpResponseCode = http.POST(fb->buf, fb->len);
        if (httpResponseCode > 0) {
            Serial.printf("Image sent, response: %d\n", httpResponseCode);
        } else {
            Serial.printf("Failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
        }

        http.end();
        esp_camera_fb_return(fb);
    }
}
