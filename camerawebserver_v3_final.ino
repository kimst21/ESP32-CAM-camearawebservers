// ============================================================
// camerawebserver_v3.ino
// ESP32-CAM: UDP RAW 청크 재조립 후 Pico로 안정 전송 최종본
//
// PC -> ESP32-CAM:
//   S + size_lo + size_hi
//   D + index + raw payload
//   E
//
// ESP32-CAM -> Pico:
//   AA 55 + size_lo + size_hi + RAW 9216 bytes + 55 AA
// ============================================================

#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include <WiFi.h>
#include <WiFiUdp.h>

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

const char *ssid = "U+NetE51C";
const char *password = "DA9969P73#";

const int UDP_PORT = 4210;
const int STREAM_PORT = 81;

const uint32_t UART_BAUD = 115200;
const int UART_TX_PIN = 14;
const int UART_RX_PIN = 15;

#define RAW_W 96
#define RAW_H 96
#define RAW_SIZE 9216
#define RAW_MAX_SIZE 9216

#define PART_BOUNDARY "123456789000000000000987654321"

static const char* _STREAM_CONTENT_TYPE =
  "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY =
  "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART =
  "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

WiFiUDP udp;
HardwareSerial picoSerial(2);
httpd_handle_t stream_httpd = NULL;

uint8_t packetBuffer[1024];

uint8_t rawAssembly[RAW_MAX_SIZE];
size_t rawAssemblyPos = 0;
size_t rawExpectedSize = 0;
bool rawInProgress = false;
unsigned long rawStartedAt = 0;

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t* fb = NULL;
  esp_err_t res = ESP_OK;
  size_t jpg_len = 0;
  uint8_t* jpg_buf = NULL;
  char part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  while (true) {
    fb = esp_camera_fb_get();

    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
        esp_camera_fb_return(fb);
        fb = NULL;

        if (!jpeg_converted) {
          Serial.println("JPEG compression failed");
          res = ESP_FAIL;
        }
      } else {
        jpg_len = fb->len;
        jpg_buf = fb->buf;
      }
    }

    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }

    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, jpg_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }

    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char*)jpg_buf, jpg_len);
    }

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      jpg_buf = NULL;
    } else if (jpg_buf) {
      free(jpg_buf);
      jpg_buf = NULL;
    }

    if (res != ESP_OK) break;
  }

  return res;
}

void startStreamServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = STREAM_PORT;
  config.ctrl_port = STREAM_PORT + 1;

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.printf("Stream server started: %d\n", STREAM_PORT);
  } else {
    Serial.println("Stream server failed");
  }
}

void sendRawToPico(const uint8_t* raw_data, size_t raw_size) {
  if (raw_size != RAW_SIZE) {
    Serial.printf("[RAW] size error: %d\n", raw_size);
    return;
  }

  uint32_t checksum = 0;
  uint8_t minv = 255;
  uint8_t maxv = 0;

  for (size_t i = 0; i < raw_size; i++) {
    uint8_t v = raw_data[i];
    checksum += v;
    if (v < minv) minv = v;
    if (v > maxv) maxv = v;
  }

  Serial.println("[UART] RAW send start");
  Serial.printf("[UART] size=%d checksum=%lu min=%d max=%d avg=%lu\n",
                raw_size, checksum, minv, maxv, checksum / raw_size);

  const size_t CHUNK_SIZE = 64;
  const int CHUNK_DELAY_US = 2000;

  picoSerial.write(0xAA);
  picoSerial.write(0x55);

  picoSerial.write((uint8_t)(raw_size & 0xFF));
  picoSerial.write((uint8_t)((raw_size >> 8) & 0xFF));
  picoSerial.flush();
  delayMicroseconds(CHUNK_DELAY_US);

  size_t sent = 0;

  while (sent < raw_size) {
    size_t chunk = raw_size - sent;
    if (chunk > CHUNK_SIZE) chunk = CHUNK_SIZE;

    picoSerial.write(raw_data + sent, chunk);
    picoSerial.flush();

    sent += chunk;
    delayMicroseconds(CHUNK_DELAY_US);
  }

  picoSerial.write(0x55);
  picoSerial.write(0xAA);
  picoSerial.flush();

  Serial.printf("[UART] RAW send done: %d + header 6\n", raw_size);
}

bool setupCamera() {
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
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_DRAM;

  if (psramFound()) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.jpeg_quality = 10;
    Serial.println("PSRAM found");
  } else {
    Serial.println("PSRAM not found");
  }

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();

  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }

  s->set_framesize(s, FRAMESIZE_QVGA);

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== ESP32-CAM RAW Reassembly Firmware Final ===");

  if (!setupCamera()) {
    Serial.println("Camera setup failed");
    return;
  }

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("WiFi IP: ");
  Serial.println(WiFi.localIP());

  startStreamServer();

  udp.begin(UDP_PORT);
  Serial.printf("UDP listening: %d\n", UDP_PORT);

  picoSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.printf("UART to Pico: %d bps, TX=%d RX=%d\n",
                UART_BAUD, UART_TX_PIN, UART_RX_PIN);

  Serial.print("Stream URL: http://");
  Serial.print(WiFi.localIP());
  Serial.println(":81/stream");
}

void loop() {
  if (rawInProgress && millis() - rawStartedAt > 2000) {
    Serial.printf("[UDP] RAW timeout: pos=%d expected=%d\n",
                  rawAssemblyPos, rawExpectedSize);

    rawInProgress = false;
    rawAssemblyPos = 0;
    rawExpectedSize = 0;
  }

  int packetSize = udp.parsePacket();

  if (packetSize <= 0) {
    delay(1);
    return;
  }

  int len = udp.read(packetBuffer, sizeof(packetBuffer));
  if (len <= 0) return;

  char msgType = (char)packetBuffer[0];

  if (msgType == 'P' || msgType == 'N') {
    packetBuffer[len] = '\0';
    Serial.printf("[COORD] %s\n", packetBuffer);

    picoSerial.print((char*)packetBuffer);
    picoSerial.println();
    return;
  }

  if (msgType == 'S' && len >= 3) {
    rawExpectedSize =
      (uint8_t)packetBuffer[1] |
      ((uint8_t)packetBuffer[2] << 8);

    Serial.printf("[UDP] RAW Start: expected=%d\n", rawExpectedSize);

    if (rawExpectedSize != RAW_SIZE) {
      Serial.printf("[UDP] RAW size invalid: %d\n", rawExpectedSize);
      rawInProgress = false;
      rawAssemblyPos = 0;
      rawExpectedSize = 0;
      return;
    }

    rawAssemblyPos = 0;
    rawInProgress = true;
    rawStartedAt = millis();
    return;
  }

  if (msgType == 'D' && rawInProgress && len > 2) {
    size_t payload_len = len - 2;

    if (rawAssemblyPos + payload_len <= RAW_MAX_SIZE) {
      memcpy(rawAssembly + rawAssemblyPos, packetBuffer + 2, payload_len);
      rawAssemblyPos += payload_len;

      Serial.printf("[UDP] D chunk: +%d -> %d/%d\n",
                    payload_len, rawAssemblyPos, rawExpectedSize);
    } else {
      Serial.printf("[UDP] overflow: pos=%d + len=%d\n",
                    rawAssemblyPos, payload_len);

      rawInProgress = false;
      rawAssemblyPos = 0;
      rawExpectedSize = 0;
    }

    return;
  }

  if (msgType == 'E' && rawInProgress) {
    Serial.printf("[UDP] RAW End: %d/%d\n",
                  rawAssemblyPos, rawExpectedSize);

    if (rawAssemblyPos == RAW_SIZE && rawExpectedSize == RAW_SIZE) {
      sendRawToPico(rawAssembly, RAW_SIZE);
    } else {
      Serial.println("[UDP] RAW incomplete -> cancel UART send");
    }

    rawInProgress = false;
    rawAssemblyPos = 0;
    rawExpectedSize = 0;
    return;
  }

  Serial.printf("[UDP] unknown packet: 0x%02X len=%d\n", msgType, len);
}
