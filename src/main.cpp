#include "esp_camera.h"
#include <Arduino.h>

#include "config.h"
#include "esp_http_server.h"
#include <Adafruit_ADS1X15.h> // ADC采集
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> // 显示屏
#include <HTTPClient.h>
#include <ThingsCloudMQTT.h>
#include <ThingsCloudWiFiManager.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
// 引脚定义必须在前 否则会初始化失败
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
// I2C引脚配置
#define I2C_SDA 15
#define I2C_SCL 14

// OLED显示屏参数
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRES 0x3c

// RCWL雷达代码
#define RCWL_PIN 13
bool RCWLState = LOW;
bool lastRCWLState = LOW;
void RCWLChangeState() { RCWLState = HIGH; }

// 板载LED引脚配置
#define LED_PIN 4
// LED滞回控制阈值 (单位: V)
#define LED_ON_THRESHOLD 1.8f  // 电压低于此值则点亮LED
#define LED_OFF_THRESHOLD 2.2f // 电压高于此值则关闭LED
// 创建实例连接
Adafruit_ADS1015 ads; // ADS1015
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

extern httpd_handle_t stream_httpd;
extern httpd_handle_t camera_httpd;
// 记录摄像头服务器当前运行状态
bool cameraIsRunning = false;
// 光敏电压
float voltage = 0;
// 光敏百分比
int lightPercent = 0;
// WiFi信号强度 dBm
long rssi = 0;
int16_t personCount = 0;
// 是否佩戴头盔
uint8_t isProtection = 0;
bool isProtectionChanged = false;     // 标记isProtection是否变化
unsigned long lastFullUploadTime = 0; // 上次完整上传时间
// LED状态跟踪 (滞回控制)
bool ledState = false; // false = 关闭, true = 点亮
// UDP配置
WiFiUDP udp;
#define localUdpPort 8266
char udpPacket[255];

// MQTT连接
ThingsCloudMQTT mqtt_client(THINGSCLOUD_MQTT_HOST,
                            THINGSCLOUD_DEVICE_ACCESS_TOKEN,
                            THINGSCLOUD_PROJECT_KEY);

WiFiClient esp32Client;
HTTPClient http;
// 必须实现这个回调函数，当 MQTT 连接成功后执行该函数。
void onMQTTConnect() { Serial.println("MQTT 连接成功"); }
void checkUdpUpdate() {
    // 持续监听UDP包
    int packetSize = udp.parsePacket();
    if (packetSize) {
        char packetBuffer[10];
        int len = udp.read(packetBuffer, 9);
        if (len > 0) {
            packetBuffer[len] = '\0';
            // 将接收到的 "1" 或 "0" 转为整数更新全局变量
            uint8_t oldIsProtection = isProtection; // 保存旧值
            isProtection = atoi(packetBuffer);
            if (oldIsProtection != isProtection) { // 如果isProtection发生变化
                isProtectionChanged = true;        // 标记为已变化
            }
            Serial.printf("收到 PC 指令，检测状态更新为: %d\n", isProtection);
        }
    }
}
void pubSensors() {

    DynamicJsonDocument obj(512);
    obj["RCWL"] = RCWLState;
    obj["WiFiState"] = rssi;
    obj["lightPercent"] = lightPercent;
    obj["personCount"] = personCount;
    obj["isProtection"] = isProtection == 1;
    char attributes[512];
    serializeJson(obj, attributes);
    // 调用属性上报方法
    mqtt_client.reportAttributes(attributes);
}
void startCameraServer();
void stopCameraServer() {
    // 仅停止HTTP服务器
    if (stream_httpd) {
        httpd_stop(stream_httpd);
        stream_httpd = NULL;
    }
    if (camera_httpd) {
        httpd_stop(camera_httpd);
        camera_httpd = NULL;
    }

    // 设置传感器进入低功耗模式
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_powerdown(s, 1); // 1=进入休眠模式
    }

    cameraIsRunning = false;
    Serial.println("HTTP Server Stopped, Camera in standby");
}
camera_config_t config;
void camera_init() {
    Serial.setDebugOutput(true);

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
        config.frame_size = FRAMESIZE_SVGA; // 640x480
        config.jpeg_quality = 6;            // 数值越小画质越好
        config.fb_count = 1;                // 降低内存压力
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
        s->set_vflip(s, 1);       // 垂直翻转
        s->set_brightness(s, 1);  // 微调亮度(-2~1) 光线充足则为0
        s->set_saturation(s, -2); // 饱和度
        s->set_whitebal(s, 1);    // 开启白平衡
        s->set_gain_ctrl(s, 1);   // 开启自动增益
        // s->set_agc_gain(s, 2);       // 手动调大增益（如果环境暗）
    }
    // drop down frame size for higher initial frame rate
    s->set_framesize(s, FRAMESIZE_SVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#endif
}
void sensorInit() {
    // 初始化RCWL引脚
    pinMode(RCWL_PIN, INPUT);
    Serial.println("RCWL-0515已启动");
    // 初始化I2C总线
    Wire.begin(I2C_SDA, I2C_SCL);
    // Wire.setClock(100000);
    // 初始化ADS1015，显式传入地址0x48
    if (!ads.begin(0x48, &Wire)) {
        Serial.println("ADS1015初始化失败");
        delay(100);
        ads.begin(0x48);
    } else {
        Serial.println("ADS1015初始化成功");
    }
    // 初始化OLED屏幕
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRES)) {
        Serial.println("SSD1306分配失败");
    } else {
        Serial.println("OLED初始化成功");
        display.clearDisplay();
    }
    // I2C自检
    Serial.println("Scanning I2C...");
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.print("Found device at 0x");
            Serial.println(address, HEX);
        }
    }
}
void OLED_Show() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    // WiFi连接状态
    display.print("WiFi:");
    display.println(WiFi.status() == WL_CONNECTED ? "Connected"
                                                  : "Disconnected");
    // RCWL雷达状态
    display.print("Motion:");
    display.println(RCWLState ? "Detected!" : "Searching");
    // 光敏电阻电压值
    display.print("voltage:");
    display.println(voltage);
    display.display();
}
void capture() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        return;
    }
    // 上传图片到Flask服务器
    http.begin(esp32Client,
               "http://192.168.1.6:8080/upload"); // Flask服务器地址

    http.addHeader("Content-Type", "image/jpeg");
    int httpResponseCode = http.POST(fb->buf, fb->len);
    if (httpResponseCode > 0) {
        Serial.printf("Image sent, response: %d\n", httpResponseCode);
    } else {
        Serial.printf("Failed, error: %s\n",
                      http.errorToString(httpResponseCode).c_str());
    }

    http.end();
    esp_camera_fb_return(fb);
}
void setup() {
    Serial.begin(115200);
    // 板载LED关闭
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    sensorInit();

    // 允许 SDK 的日志输出
    mqtt_client.enableDebuggingMessages();

    // 初始化摄像头(仅一次)
    camera_init();
    // 初始状态设为休眠
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_powerdown(s, 1);
    }
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    // 开启摄像头
    // startCameraServer();

    Serial.print("Camera Ready! Use 'http://");
    Serial.print(WiFi.localIP());
    Serial.println("' to connect");

    // 开启UDP监听
    udp.begin(localUdpPort);
    Serial.printf("正在监听UDP端口:%d\n", localUdpPort);
}
// 控制OLED频率 500ms
unsigned long lastDisplayTime = 0;
const int displayInterval = 500;
// 控制拍照频率 100ms
unsigned long lastTime = 0;
const int interval = 100;
// 控制mqtt发送频率 10s
// unsigned long lastMsgTime = 0; // 移除
const int mqttInterval = 10000; // 保持定义，用于时间间隔判断
// 控制检测频率 20ms
unsigned long lastCheck = 0;
const int checkInterval = 20;
unsigned long RCWLKeepTime = 0; // 记录RCWL状态保持时间
const int keepDuration = 10000; // 检测到人后状态位保持10秒
void loop() {
    // 获取WiFi强度
    rssi = WiFi.RSSI();
    unsigned long now = millis();

    // 连接MQTT服务器
    mqtt_client.loop();
    // 定时上报所有传感器数据
    if (now - lastFullUploadTime >= mqttInterval) {
        pubSensors();
        lastFullUploadTime = now;    // 更新上次完整上传时间
        isProtectionChanged = false; // 常规上传后，清除变化标记
    }
    // 如果isProtection有变化，且不是刚刚通过常规上传上报的，则立即上报
    else if (isProtectionChanged) {
        pubSensors();
        isProtectionChanged = false; // 清除变化标记
    }
    // RCWL检测代码
    if (now - lastCheck >= checkInterval) {
        bool currentRCWLState = digitalRead(RCWL_PIN);
        RCWLState = false;
        if (currentRCWLState == HIGH && lastRCWLState == LOW) {
            Serial.println("--检测到人员移动--");
            personCount++;
            RCWLKeepTime = now;
        }
        lastRCWLState = currentRCWLState;
        lastCheck = millis();
    }
    RCWLState = (now - RCWLKeepTime < keepDuration);

    // 有人状态
    if (RCWLState) {
        // 有人且关摄像头时开启
        if (!cameraIsRunning) {
            Serial.println("Action: Starting Camera Server...");
            // 唤醒传感器
            sensor_t *s = esp_camera_sensor_get();
            if (s) {
                s->set_powerdown(s, 0); // 0=退出休眠模式
                delay(100);             // 等待传感器稳定
            }
            startCameraServer();
            cameraIsRunning = true;
        }
        // 监听PC端发来的头盔识别结果
        checkUdpUpdate();
        // 读取ADS1015原始值和电压值
        ads.setGain(GAIN_TWOTHIRDS);
        int16_t adc_0 = ads.readADC_SingleEnded(0);
        voltage = ads.computeVolts(adc_0); // 计算电压值
        // 环境百分比
        lightPercent = (voltage / 3.3) * 100;
        if (lightPercent > 100) {
            lightPercent = 100;
        } else if (lightPercent < 0) {
            lightPercent = 0;
        }

        // 补光控制逻辑 (滞回控制)
        if (!ledState && voltage < LED_ON_THRESHOLD) {
            // LED当前关闭且电压低于开启阈值 -> 点亮LED
            digitalWrite(LED_PIN, HIGH);
            ledState = true;
        } else if (ledState && voltage > LED_OFF_THRESHOLD) {
            // LED当前点亮且电压高于关闭阈值 -> 关闭LED
            digitalWrite(LED_PIN, LOW);
            ledState = false;
        }
        // 若电压处于两个阈值之间，LED保持当前状态不变
    } else { // 无人模式
        // 无人且开摄像头时关闭
        if (cameraIsRunning) {
            Serial.println("Action: Stopping Camera Server...");
            stopCameraServer();
        }
        if (ledState) {
            digitalWrite(LED_PIN, LOW);
            ledState = false;
        }
        ads.setGain(GAIN_TWOTHIRDS);
        int16_t adc_0 = ads.readADC_SingleEnded(0);
        voltage = ads.computeVolts(adc_0);
        lightPercent = constrain((voltage / 3.3) * 100, 0, 100);
    }
    // OLED显示代码
    if (now - lastDisplayTime >= displayInterval) {
        OLED_Show();
        lastDisplayTime = millis();
    }

    // // 拍照
    // if (millis() - lastTime > interval) {
    //     // capture();
    //     RCWLState = true;
    //     lastTime = millis();
    // }
}
