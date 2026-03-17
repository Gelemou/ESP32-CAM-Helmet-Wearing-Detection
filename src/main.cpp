#include <Arduino.h>
#include "esp_camera.h"

#include "config.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ThingsCloudWiFiManager.h>
#include <ThingsCloudMQTT.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h> // ADC采集
#include <Adafruit_GFX.h>     
#include <Adafruit_SSD1306.h> // 显示屏

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
bool RCWLState = false;
void RCWLChangeState()
{
    RCWLState = true;
}

// 板载LED引脚配置
#define LED_PIN 4
// 创建实例连接
Adafruit_ADS1015 ads; // ADS1015
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 光敏电压
float voltage = 0;
// 光敏百分比
int lightPercent = 0;
// WiFi信号强度 dBm
long rssi = 0;
int16_t personCount = 0;

// MQTT连接
ThingsCloudMQTT mqtt_client(
    THINGSCLOUD_MQTT_HOST,
    THINGSCLOUD_DEVICE_ACCESS_TOKEN,
    THINGSCLOUD_PROJECT_KEY);

WiFiClient esp32Client;
// 必须实现这个回调函数，当 MQTT 连接成功后执行该函数。
void onMQTTConnect()
{
    Serial.println("MQTT 连接成功");
}
void pubSensors()
{
    DynamicJsonDocument obj(512);
    obj["RCWL"] = RCWLState;
    obj["WiFiState"] = rssi;
    obj["lightPercent"] = lightPercent;
    obj["personCount"] = personCount;
    char attributes[512];
    serializeJson(obj, attributes);
    // 调用属性上报方法
    mqtt_client.reportAttributes(attributes);
}
void startCameraServer();
camera_config_t config;
void camera_init()
{
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

    if (psramFound())
    {
        config.frame_size = FRAMESIZE_XGA; // 1600x1200
        config.jpeg_quality = 10;          // 数值越小画质越好
        config.fb_count = 1;               // 降低内存压力
    }
    else
    {
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
    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV2640_PID)
    {
        s->set_vflip(s, 1);       // flip it back
        s->set_brightness(s, 1);  // up the brightness just a bit
        s->set_saturation(s, -2); // lower the saturation
    }
    // drop down frame size for higher initial frame rate
    s->set_framesize(s, FRAMESIZE_XGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#endif
}
void sensorInit()
{
    // 初始化RCWL引脚
    pinMode(RCWL_PIN, INPUT);
    Serial.println("RCWL-0515已启动");
    // 初始化I2C总线
    Wire.begin(I2C_SDA, I2C_SCL);
    // Wire.setClock(100000);
    // 初始化ADS1015，显式传入地址0x48
    if (!ads.begin(0x48, &Wire))
    {
        Serial.println("ADS1015初始化失败");
        delay(100);
        ads.begin(0x48);
    }
    else
    {
        Serial.println("ADS1015初始化成功");
    }
    // 初始化OLED屏幕
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRES))
    {
        Serial.println("SSD1306分配失败");
    }
    else
    {
        Serial.println("OLED初始化成功");
        display.clearDisplay();
    }
    // I2C自检
    Serial.println("Scanning I2C...");
    for (byte address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0)
        {
            Serial.print("Found device at 0x");
            Serial.println(address, HEX);
        }
    }
}
void OLED_Show()
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    // WiFi连接状态
    display.print("WiFi:");
    display.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    // RCWL雷达状态
    display.print("Motion:");
    display.println(RCWLState ? "Detected!" : "Searching");
    // 光敏电阻电压值
    display.print("voltage:");
    display.println(voltage);
    display.display();
    delay(500);
}
void capture() {
    // 拍照
    if (millis() - lastTime > interval)
    {
        lastTime = millis();

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb)
        {
            Serial.println("Camera capture failed");
            return;
        }
        // 上传图片到Flask服务器
        HTTPClient http;
        http.begin(esp32Client, "http://192.168.1.3:8080/upload"); // Flask服务器地址
        http.addHeader("Content-Type", "image/jpeg");
        int httpResponseCode = http.POST(fb->buf, fb->len);
        if (httpResponseCode > 0)
        {
            Serial.printf("Image sent, response: %d\n", httpResponseCode);
        }
        else
        {
            Serial.printf("Failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
        }

        http.end();
        esp_camera_fb_return(fb);
    }
}
void setup()
{
    Serial.begin(115200);
    // 板载LED关闭
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    sensorInit();
    // 初始化LED PWM 5kHz 10Bit(0~1023)
    ledcSetup(0, 5000, 10);
    ledcAttachPin(LED_PIN, 0);

    //  允许 SDK 的日志输出
    mqtt_client.enableDebuggingMessages();
    camera_init();
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");

    // 开启摄像头
    startCameraServer();

    Serial.print("Camera Ready! Use 'http://");
    Serial.print(WiFi.localIP());
    Serial.println("' to connect");
}
// 控制OLED频率 500ms
unsigned long lastDisplayTime = 0;
const int displayInterval = 500;
// 控制拍照频率 500ms
unsigned long lastTime = 0;
const int interval = 500;
// 控制mqtt发送频率 10s
unsigned long lastMsgTime = 0;
const int mqttInterval = 10000;
// 控制RCWL检测频率 100ms
unsigned long lastCheck = 0;
const int checkInterval = 100;
void loop()
{
    // 获取WiFi强度
    rssi = WiFi.RSSI();
    unsigned long now = millis();
    // RCWL检测代码
    if (now - lastCheck >= checkInterval)
    {
        RCWLState = digitalRead(RCWL_PIN);
        if (RCWLState)
        {
            Serial.println("--检测到人员移动--");
            personCount++;
            RCWLState = false;
        }
        lastCheck = millis();
    }
    // OLED显示代码
    if (now - lastDisplayTime >= displayInterval)
    {
        OLED_Show();
        lastDisplayTime = millis();
    }

    // 读取ADS1015原始值和电压值
    int16_t adc_0 = ads.readADC_SingleEnded(0);
    voltage = ads.computeVolts(adc_0); // 计算电压值(根据默认增益计算，每位3mV)
    // 环境百分比
    lightPercent = (voltage / 3.3) * 100;
    if (lightPercent > 100) {
        lightPercent = 100;
    } else if (lightPercent < 0) {
        lightPercent = 0;
    }
    // 补光控制逻辑
    int16_t dutyCycle = 0;
    if (voltage < 1.0)
    {
        // 1V关闭 0.2V全亮
        dutyCycle = map(voltage * 100, 20, 100, 1023, 0);
        dutyCycle = constrain(dutyCycle, 0, 1023); // 限幅
    }
    else
    {
        dutyCycle = 0;
    }
    ledcWrite(0, dutyCycle);

    // 连接MQTT服务器
    mqtt_client.loop();
    if (now - lastMsgTime >= mqttInterval)
    {
        // 发布MQTT消息
        pubSensors();
        // mqtt_client.publish("attributes", ("{\"temperature\": " + String(temperature) + ", \"humidity\": " + String(humidity) + "}").c_str());
        // mqtt_client.publish("attributes", ("{\"brightness\"}" ":" + String(brightness)).c_str());
        lastMsgTime = millis();
    }
    
}
