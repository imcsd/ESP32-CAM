#include "esp_camera.h"
#include <WiFi.h>
#include <Preferences.h>

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD

#include "camera_pins.h"

void startCameraServer();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Check Preferences RESET
  checkPrefsReset();

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
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

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

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

  Serial.println("Init WiFi connection...");
  connectWiFi();

  startCameraServer();
}


// WiFi Connect Module Start
void connectWiFi() {
  // Read wifi setting from preferences.
  Preferences prefs;
  prefs.begin("wifi_config");
  String ssid = prefs.getString("ssid", "");
  String passwd = prefs.getString("passwd", "");
  prefs.end();

  // Check whether WiFi config exists
  if (ssid == "" || passwd == "") {
    SmartConfig();
  } else {
    connectWiFiByPasswd(ssid.c_str(), passwd.c_str());
  }

  Serial.println("\nWiFi Connected! \r\nLocal IP Address:");
  Serial.print(WiFi.localIP());
}

// Connect WiFi by passwd
void connectWiFiByPasswd(const char* ssid, const char* passwd) {
  Serial.println("Connecting WiFi...");
  Serial.printf("SSID: %s, Password: %s\n", ssid, passwd);
  WiFi.begin(ssid, passwd);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}

// Smart Config
void SmartConfig()
{
  WiFi.mode(WIFI_STA);
  Serial.println("\r\nWait for SmartConfig....");
  WiFi.beginSmartConfig();
  while (1)
  {
    Serial.print(".");
    delay(500);
    if ( WiFi.smartConfigDone())
    {
      Serial.println("\r\nSmartConfig Success");
      Serial.printf("SSID:%s\r\n", WiFi.SSID().c_str());
      Serial.printf("PSW:%s\r\n", WiFi.psk().c_str());

      // Save WiFi info to Preferences
      Preferences prefs;
      prefs.begin("wifi_config");
      prefs.putString("ssid", WiFi.SSID());
      prefs.putString("passwd", WiFi.psk());
      prefs.end();


      break;
    }
  }
}
// WiFi Connect Module END

// Preferences start
// Check Prefs Reset by jumper GPIO 12 to GND 3 sec. at startup.
void checkPrefsReset() {
  int RESET_PIN = 12;
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);

  // Check whether RESET_PIN is LOW
  if (digitalRead(RESET_PIN) == LOW) {
    unsigned long rst_millis = millis();
    while (digitalRead(RESET_PIN) == LOW) {}
    // Calculate RESET_PIN time
    if (millis() - rst_millis >= 3000) {
      Serial.println("\nRESET Preferences...");
      resetPrefs();
    }
  }
  digitalWrite(RESET_PIN, LOW);
}

// Wipe the Preferences
void resetPrefs() {
  // Clear Preferences
  Preferences prefs;
  prefs.begin("wifi_config");
  prefs.clear();
  prefs.end();

  // GPIO33 for LED1 in back of the board.
  const int LED_INDECATOR = 4;
  pinMode(LED_INDECATOR, OUTPUT);
  // Flash the LED n times each 500ms
  int n = 3;
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_INDECATOR, HIGH);
    delay(100);
    digitalWrite(LED_INDECATOR, LOW);
    delay(900);
  }
}
// Preferences end

void loop() {

}
