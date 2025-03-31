#include "esp_camera.h"

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
#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
#include "camera_pins.h"
#include <vector>
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"
#include "esp32-hal-ledc.h"
#include "sdkconfig.h"
#include "camera_index.h"
#include <ESP32Servo.h>
#include <optional>


// ===========================
// Enter your WiFi credentials
// ===========================

#pragma GCC diagnostic ignored "-Wformat"
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#include "face_recognition_tool.hpp"
#include "face_recognition_112_v1_s16.hpp"
#include "face_recognition_112_v1_s8.hpp"
#pragma GCC diagnostic error "-Wformat"
#pragma GCC diagnostic warning "-Wstrict-aliasing"
#define QUANT_TYPE 0
#define FACE_ID_SAVE_NUMBER 7

#define FACE_COLOR_WHITE  0x00FFFFFF
#define FACE_COLOR_BLACK  0x00000000
#define FACE_COLOR_RED    0x000000FF
#define FACE_COLOR_GREEN  0x0000FF00
#define FACE_COLOR_BLUE   0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN   (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)
#define LED_LEDC_GPIO            22  //configure LED pin
#define CONFIG_LED_MAX_INTENSITY 255

int led_duty = 0;
bool isStreaming = false;

Servo richardServoY;
Servo richardServoX;


void setupLedFlash(int pin) {
  ledcAttach(pin, 5000, 8);
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.print("starting.\n");
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  //config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
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

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
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

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

// Servo config
	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	richardServoX.setPeriodHertz(50);    // standard 50 hz servo
  richardServoY.setPeriodHertz(50);
	richardServoX.attach(5, 1000, 2000);
  richardServoY.attach(3, 1000, 2000);
	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep

}

typedef std::pair<int, int> Coords;

std::optional<Coords> getFaceCoordinates(camera_fb_t* fb) {
  // Calculate offset of center of face from center of camera.

    HumanFaceDetectMSR01 s1(0.05F, 0.65F, 10, 0.2F);
    std::list<dl::detect::result_t> &results = s1.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});

    if (results.empty()) { return std::nullopt; }

    Coords closestCoordinates;
    int bestDistance = 100000000;
    for (const auto& prediction : results) {
      // x: face x center, face y center
      int x = (int) ( prediction.box[0] + (prediction.box[2] - prediction.box[0]) / 2 );
      int y = (int) ( prediction.box[1] + (prediction.box[3] - prediction.box[1]) / 2);

      int thisDist = x*x + y*y;
      if (thisDist < bestDistance) {
        closestCoordinates.first = x;
        closestCoordinates.second = y;
        bestDistance = thisDist;
      }
    }
    return closestCoordinates;

}



Coords servoCoordinates = std::make_pair<int, int>(90, 90);

int last_logged = 0;

void loop() {

  camera_fb_t *fb = esp_camera_fb_get(); 
 
  if (fb != NULL && fb->format == PIXFORMAT_RGB565) {
    std::optional<Coords> faceCoords = getFaceCoordinates(fb);

    // if we lose the face, slowly move back to center.
    if (faceCoords) {
      int x = (faceCoords.value().first - fb->width / 2);
      int y = (faceCoords.value().second - fb->height / 2);
      if (last_logged % 10 == 0) {
        Serial.print("Found face : (");
        Serial.print(String(x));
        Serial.print(", ");
        Serial.print(String(y));
        Serial.print(")\n");
      }

      if (x > 0) {
        servoCoordinates.first = std::min(180, servoCoordinates.first + std::min(5, x));
      } else if (x < 0) {
        servoCoordinates.first = std::max(0, servoCoordinates.first + std::max(-5, x));
      }

      if (y > 0) {
        servoCoordinates.second = std::min(180, servoCoordinates.second + std::min(5, y));
      } else if (y < 0) {
        servoCoordinates.second = std::max(0, servoCoordinates.second + std::max(-5, y));
      }
      if (last_logged % 10 == 0) {
        Serial.print("Moving to : ("); Serial.print(String(servoCoordinates.first)); Serial.print(", ");
        Serial.print(String(servoCoordinates.second)); Serial.print(")\n");
      }
      last_logged++;
    }

  }
  esp_camera_fb_return(fb);
  richardServoX.write(servoCoordinates.first);
  richardServoY.write(servoCoordinates.second);
  delay(15);
}