/**
 * WetterKompassDisplay
 *
 * Circular ESP32 weather dashboard for the Waveshare ESP32-S3-Touch-LCD-2.8C
 * using live data from an Ecowitt/GW3000 weather station.
 *
 * Target hardware: Waveshare ESP32-S3-Touch-LCD-2.8C, 480x480 RGB LCD
 * Author: Markus Woods
 * License: MIT for this sketch; ESP32_Display_Panel is Apache-2.0 licensed.
 *
 * This project uses ESP32_Display_Panel from Espressif / esp-arduino-libs for
 * board, LCD, touch and backlight initialization.
 */

  #include <Arduino.h>
  #include <WiFi.h>
  #include <HTTPClient.h>
  #include <ArduinoJson.h>
  #include <esp_display_panel.hpp>
  #include "esp_heap_caps.h"
  #include <math.h>
  #include <stdlib.h>
  #include <string.h>
  #include <time.h>
  #include "board_external_config.hpp"
  #include "secrets.h"

  using namespace esp_panel::board;
  using namespace esp_panel::drivers;

  // Configuration
  #define WEATHER_UPDATE_PERIOD_MS                (60000)
  #define BRIGHTNESS_UPDATE_PERIOD_MS             (60000)
  #define NTP_SYNC_TIMEOUT_MS                      (15000)

  #define COLOR_BLACK       0x0000
  #define COLOR_WHITE       0xFFFF
  #define COLOR_TEAL        0x0410
  #define COLOR_GRAY        0x8410
  #define COLOR_LIGHTBLUE   0x867F
  #define COLOR_TURQUOISE   0x0679
  #define COLOR_BLUE        0x001F
  #define COLOR_PURPLE      0x780F
  #define COLOR_GREEN       0x07E0
  #define COLOR_YELLOWGREEN 0xAFE5
  #define COLOR_YELLOW      0xFFE0
  #define COLOR_ORANGE      0xFD20
  #define COLOR_RED         0xF800

  #define TEST_WIND_DIRECTION_DEG 322
  #define TEST_WIND_SPEED_KMH 16.2f
  #define TEST_WBGT_C 21.1f
  #define TEST_TEMPERATURE_C 22.0f
  #define TEST_PRESSURE_HPA 1013.0f
  #define TEST_HUMIDITY_PERCENT 50.0f
  #define TEST_LIGHTNING_DISTANCE_KM 99.0f
  #define TEST_RAIN_RATE_MM_H 0.0f
  #define TEST_RAIN_DAY_MM 0.0f

  struct WeatherData {
      float wind_direction_deg;
      float wind_speed_kmh;
      float gust_speed_kmh;
      float wbgt_c;
      float temperature_c;
      float pressure_hpa;
      float humidity_percent;
      float lightning_distance_km;
      bool lightning_active;
      float rain_rate_mm_h;
      float rain_day_mm;
      bool rain_active;
  };

  // Global data structures
  Board *board = nullptr;
  uint16_t *lcd_draw_buffer = nullptr;
  size_t lcd_draw_buffer_pixels = 0;
  bool has_previous_weather_data = false;
  WeatherData previous_weather_data;

  float normalizeAngle(float angle);
  void drawCompassRing(LCD *lcd);
  bool weatherDataChanged(const WeatherData &current, const WeatherData &next);
  bool ensureDrawBuffer(int width, int height);
  bool syncTime(uint32_t timeout_ms = NTP_SYNC_TIMEOUT_MS);
  bool readFloatValue(JsonObjectConst item, float &value);
  bool readFloatKey(JsonObjectConst item, const char *key, float &value);
  void updateLightningDistanceToday(float distance_km);
  bool setDisplayBrightness(int percent);
  int getTargetBrightnessByTime();
  void updateDisplayBrightness();

  // Cached weather state
  WeatherData weather_data = {
      TEST_WIND_DIRECTION_DEG,
      TEST_WIND_SPEED_KMH,
      TEST_WIND_SPEED_KMH,
      TEST_WBGT_C,
      TEST_TEMPERATURE_C,
      TEST_PRESSURE_HPA,
      TEST_HUMIDITY_PERCENT,
      TEST_LIGHTNING_DISTANCE_KM,
      false,
      TEST_RAIN_RATE_MM_H,
      TEST_RAIN_DAY_MM,
      false
  };
  uint32_t last_weather_update_ms = 0;
  uint32_t last_brightness_update_ms = 0;
  int current_display_brightness_percent = -1;
  float lightning_distance_today_km = TEST_LIGHTNING_DISTANCE_KM;
  bool has_lightning_distance_today = false;
  int lightning_distance_today_day_key = -1;

  // WiFi / NTP
  bool connectWiFi(uint32_t timeout_ms = 15000)
  {
      Serial.println("Connecting WiFi");

      WiFi.mode(WIFI_STA);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

      uint32_t start_ms = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - start_ms < timeout_ms) {
          delay(250);
          Serial.print(".");
      }

      Serial.println();

      if (WiFi.status() == WL_CONNECTED) {
          Serial.printf("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
          return true;
      }

      Serial.println("WiFi connection failed");
      return false;
  }

  bool syncTime(uint32_t timeout_ms)
  {
      if (WiFi.status() != WL_CONNECTED) {
          return false;
      }

      configTzTime("CET-1CEST,M3.5.0/2,M10.5.0/3", "pool.ntp.org", "time.nist.gov");

      uint32_t start_ms = millis();
      time_t now = 0;
      while (millis() - start_ms < timeout_ms) {
          now = time(nullptr);
          if (now > 1600000000) {
              return true;
          }

          delay(250);
      }
      return false;
  }

  bool setDisplayBrightness(int percent)
  {
      int clamped_percent = max(0, min(100, percent));

      if (board == nullptr) {
          return false;
      }

      auto backlight = board->getBacklight();
      if (backlight == nullptr) {
          // The configured board does not expose a backlight driver.
          return false;
      }

      if (current_display_brightness_percent == clamped_percent) {
          return true;
      }

      if (!backlight->setBrightness(clamped_percent)) {
          return false;
      }

      current_display_brightness_percent = clamped_percent;
      return true;
  }

  int getTargetBrightnessByTime()
  {
      time_t now = time(nullptr);
      if (now <= 1600000000) {
          return 100;
      }

      struct tm timeinfo;
      localtime_r(&now, &timeinfo);
      int hour = timeinfo.tm_hour;

      if (hour >= 22 || hour < 6) {
          return 10;
      }

      if ((hour >= 19 && hour < 22) || (hour >= 6 && hour < 8)) {
          return 35;
      }

      return 100;
  }

  void updateDisplayBrightness()
  {
      setDisplayBrightness(getTargetBrightnessByTime());
  }

  void updateLightningDistanceToday(float distance_km)
  {
      time_t now = time(nullptr);
      if (now <= 1600000000) {
          return;
      }

      struct tm timeinfo;
      localtime_r(&now, &timeinfo);

      int day_key = timeinfo.tm_year * 366 + timeinfo.tm_yday;
      if (lightning_distance_today_day_key != day_key) {
          lightning_distance_today_day_key = day_key;
          has_lightning_distance_today = false;
      }

      lightning_distance_today_km = distance_km;
      has_lightning_distance_today = true;
  }

  bool ensureDrawBuffer(int width, int height)
  {
      size_t required_pixels = (size_t)width * (size_t)height;
      if (lcd_draw_buffer != nullptr && lcd_draw_buffer_pixels == required_pixels) {
          return true;
      }

      if (lcd_draw_buffer != nullptr) {
          heap_caps_free(lcd_draw_buffer);
          lcd_draw_buffer = nullptr;
          lcd_draw_buffer_pixels = 0;
      }

      lcd_draw_buffer = (uint16_t *)heap_caps_malloc(
          required_pixels * sizeof(uint16_t),
          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
      );

      if (lcd_draw_buffer == nullptr) {
          return false;
      }

      lcd_draw_buffer_pixels = required_pixels;
      return true;
  }

  // Ecowitt API / JSON parsing
  int parseLiveDataId(JsonVariantConst id)
  {
      if (id.is<int>()) {
          return id.as<int>();
      }

      const char *id_text = id.as<const char *>();
      if (id_text == nullptr) {
          return -1;
      }

      return (int)strtol(id_text, nullptr, 0);
  }

  bool parseFloatText(const char *text, float &value)
  {
      if (text == nullptr) {
          return false;
      }

      char *end_ptr = nullptr;
      float parsed_value = strtof(text, &end_ptr);

      if (end_ptr == text) {
          return false;
      }

      value = parsed_value;
      return true;
  }

  bool readJsonFloat(JsonVariantConst raw_value, float &value)
  {
      if (raw_value.isNull() || raw_value.is<JsonArrayConst>()) {
          return false;
      }

      if (raw_value.is<JsonObjectConst>()) {
          return readFloatValue(raw_value.as<JsonObjectConst>(), value);
      }

      const char *text = raw_value.as<const char *>();
      if (text != nullptr) {
          return parseFloatText(text, value);
      }

      value = raw_value.as<float>();
      return true;
  }

  bool readFloatValue(JsonObjectConst item, float &value)
  {
      const char *value_keys[] = {"val", "value", "data"};

      for (size_t i = 0; i < sizeof(value_keys) / sizeof(value_keys[0]); i++) {
          JsonVariantConst raw_value = item[value_keys[i]];
          if (!raw_value.isNull()) {
              return readJsonFloat(raw_value, value);
          }
      }

      return false;
  }

  bool readRainValue(JsonObjectConst item, float &value)
  {
      return readFloatKey(item, "val", value) ||
             readFloatKey(item, "value", value) ||
             readFloatKey(item, "data", value);
  }

  bool readFloatKey(JsonObjectConst item, const char *key, float &value)
  {
      JsonVariantConst raw_value = item[key];
      if (raw_value.isNull()) {
          return false;
      }

      return readJsonFloat(raw_value, value);
  }

  bool textEqualsIgnoreCase(const char *text, const char *expected)
  {
      if (text == nullptr) {
          return false;
      }

      while (*text != '\0' && *expected != '\0') {
          char a = *text;
          char b = *expected;

          if (a >= 'A' && a <= 'Z') {
              a += 'a' - 'A';
          }
          if (b >= 'A' && b <= 'Z') {
              b += 'a' - 'A';
          }
          if (a != b) {
              return false;
          }

          text++;
          expected++;
      }

      return *text == '\0' && *expected == '\0';
  }

  bool jsonStringEquals(JsonVariantConst value, const char *expected)
  {
      return textEqualsIgnoreCase(value.as<const char *>(), expected);
  }

  bool objectIsWH25(JsonObjectConst item)
  {
      return jsonStringEquals(item["unit"], "WH25") ||
             jsonStringEquals(item["name"], "WH25") ||
             jsonStringEquals(item["type"], "WH25") ||
             jsonStringEquals(item["sensor"], "WH25");
  }

  bool objectIsCommonList(JsonObjectConst item)
  {
      return jsonStringEquals(item["unit"], "common_list") ||
             jsonStringEquals(item["name"], "common_list") ||
             jsonStringEquals(item["type"], "common_list") ||
             jsonStringEquals(item["sensor"], "common_list");
  }

  bool objectIsLightning(JsonObjectConst item)
  {
      return jsonStringEquals(item["unit"], "lightning") ||
             jsonStringEquals(item["name"], "lightning") ||
             jsonStringEquals(item["type"], "lightning") ||
             jsonStringEquals(item["sensor"], "lightning") ||
             jsonStringEquals(item["unit"], "WH57") ||
             jsonStringEquals(item["name"], "WH57") ||
             jsonStringEquals(item["type"], "WH57") ||
             jsonStringEquals(item["sensor"], "WH57");
  }

  bool objectIsRainPiezo(JsonObjectConst item)
  {
      return jsonStringEquals(item["unit"], "srain_piezo") ||
             jsonStringEquals(item["name"], "srain_piezo") ||
             jsonStringEquals(item["type"], "srain_piezo") ||
             jsonStringEquals(item["sensor"], "srain_piezo");
  }

  bool objectIsPiezoRain(JsonObjectConst item)
  {
      return jsonStringEquals(item["unit"], "piezoRain") ||
             jsonStringEquals(item["name"], "piezoRain") ||
             jsonStringEquals(item["type"], "piezoRain") ||
             jsonStringEquals(item["sensor"], "piezoRain");
  }

  bool readLightningTimestampAgeSeconds(JsonObjectConst item, uint32_t &age_seconds)
  {
      float timestamp = 0.0f;
      const char *timestamp_keys[] = {"timestamp", "time", "last_time"};

      for (size_t i = 0; i < sizeof(timestamp_keys) / sizeof(timestamp_keys[0]); i++) {
          if (!readFloatKey(item, timestamp_keys[i], timestamp)) {
              continue;
          }

          if (timestamp < 0.0f) {
              return false;
          }

          time_t now = time(nullptr);

          if (timestamp > 1000000000000.0f) {
              timestamp /= 1000.0f;
          }

          if (timestamp > 1600000000.0f && now > 1600000000 && timestamp <= (float)now) {
              age_seconds = (uint32_t)((float)now - timestamp);
              return true;
          }

          if (timestamp <= 86400.0f && now > 1600000000) {
              struct tm timeinfo;
              localtime_r(&now, &timeinfo);
              uint32_t now_seconds_of_day = timeinfo.tm_hour * 3600UL + timeinfo.tm_min * 60UL + timeinfo.tm_sec;
              uint32_t lightning_seconds_of_day = (uint32_t)timestamp;

              if (now_seconds_of_day >= lightning_seconds_of_day) {
                  age_seconds = now_seconds_of_day - lightning_seconds_of_day;
              } else {
                  age_seconds = now_seconds_of_day + 86400UL - lightning_seconds_of_day;
              }

              return true;
          }

          return false;
      }

      return false;
  }

  bool weatherDataChanged(const WeatherData &current, const WeatherData &next)
  {
      const float value_epsilon = 0.05f;
      const float direction_epsilon = 0.5f;

      float direction_delta = fabs(normalizeAngle(current.wind_direction_deg - next.wind_direction_deg));
      if (direction_delta > 180.0f) {
          direction_delta = 360.0f - direction_delta;
      }

      return direction_delta > direction_epsilon ||
             fabs(current.wind_speed_kmh - next.wind_speed_kmh) > value_epsilon ||
             fabs(current.gust_speed_kmh - next.gust_speed_kmh) > value_epsilon ||
             fabs(current.wbgt_c - next.wbgt_c) > value_epsilon ||
             fabs(current.temperature_c - next.temperature_c) > value_epsilon ||
             fabs(current.pressure_hpa - next.pressure_hpa) > value_epsilon ||
             fabs(current.humidity_percent - next.humidity_percent) > value_epsilon ||
             fabs(current.lightning_distance_km - next.lightning_distance_km) > value_epsilon ||
             current.lightning_active != next.lightning_active ||
             fabs(current.rain_rate_mm_h - next.rain_rate_mm_h) > value_epsilon ||
             fabs(current.rain_day_mm - next.rain_day_mm) > value_epsilon ||
             current.rain_active != next.rain_active;
  }

  void parseLiveDataItems(
      JsonVariantConst node,
      WeatherData &data,
      bool &has_wind_dir,
      bool &has_wind,
      bool &has_gust,
      bool &has_wbgt,
      bool &has_temperature,
      bool &has_pressure,
      bool &has_humidity,
      bool &has_lightning,
      bool &has_rain,
      bool &has_rain_rate,
      bool &has_rain_day,
      bool inside_wh25 = false,
      bool inside_lightning = false,
      bool inside_rain = false,
      bool inside_common_list = false,
      bool inside_piezo_rain = false
  )
  {
      if (node.is<JsonObjectConst>()) {
          JsonObjectConst item = node.as<JsonObjectConst>();
          bool is_wh25 = inside_wh25 || objectIsWH25(item);
          bool is_lightning = inside_lightning || objectIsLightning(item);
          bool is_rain = inside_rain || objectIsRainPiezo(item);
          bool is_common_list = inside_common_list || objectIsCommonList(item);
          bool is_piezo_rain = inside_piezo_rain || objectIsPiezoRain(item);
          JsonVariantConst id = item["id"];
          if (id.isNull()) {
              id = item["unit_id"];
          }
          if (id.isNull()) {
              id = item["unitid"];
          }
          if (id.isNull()) {
              id = item["unit"];
          }

          if (is_wh25) {
              float pressure = 0.0f;
              if (readFloatKey(item, "rel", pressure)) {
                  data.pressure_hpa = pressure;
                  has_pressure = true;
              }
          }

          if (is_lightning) {
              float distance_km = 0.0f;
              uint32_t age_seconds = 0;
              bool has_distance = readFloatKey(item, "distance", distance_km) ||
                                  readFloatKey(item, "distance_km", distance_km);
              bool has_timestamp = readLightningTimestampAgeSeconds(item, age_seconds);

              if (has_distance || has_timestamp) {
                  if (has_distance) {
                      data.lightning_distance_km = distance_km;
                      updateLightningDistanceToday(distance_km);
                  }

                  if (has_timestamp) {
                      data.lightning_active = age_seconds <= 30UL * 60UL &&
                                              (!has_distance || distance_km < 20.0f);
                  } else {
                      data.lightning_active = has_distance && distance_km < 20.0f;
                  }
                  has_lightning = true;
              }
          }

          if (is_rain) {
              float rain_value = 0.0f;
              if (readRainValue(item, rain_value)) {
                  data.rain_rate_mm_h = rain_value;
                  data.rain_active = rain_value > 0.0f;
                  has_rain = true;
                  has_rain_rate = true;
              }
          }

          if (is_piezo_rain && !id.isNull()) {
              float rain_value = 0.0f;
              int parsed_id = parseLiveDataId(id);

              if (readRainValue(item, rain_value)) {
                  if (parsed_id == 0x0E) {
                      data.rain_rate_mm_h = rain_value;
                      data.rain_active = rain_value > 0.0f;
                      has_rain = true;
                      has_rain_rate = true;
                  } else if (parsed_id == 0x7C) {
                      data.rain_day_mm = rain_value;
                      has_rain = true;
                      has_rain_day = true;
                  }
              }
          }

          if (!id.isNull()) {
              float value = 0.0f;
              int parsed_id = parseLiveDataId(id);

              if (is_common_list && parsed_id == 0x07 && readFloatValue(item, value)) {
                  data.humidity_percent = value;
                  has_humidity = true;
              } else if (readFloatValue(item, value)) {
                  switch (parsed_id) {
                      case 0x02:
                          data.temperature_c = value;
                          has_temperature = true;
                          break;
                      case 0x07:
                          break;
                      case 0x0A:
                          data.wind_direction_deg = normalizeAngle(value);
                          has_wind_dir = true;
                          break;
                      case 0x0B:
                          data.wind_speed_kmh = value * 3.6f;
                          has_wind = true;
                          break;
                      case 0x0C:
                          data.gust_speed_kmh = value * 3.6f;
                          has_gust = true;
                          break;
                      case 0xA2:
                          data.wbgt_c = value;
                          has_wbgt = true;
                          break;
                      default:
                          break;
                  }
              }
          }

          for (JsonPairConst child : item) {
              bool child_inside_wh25 = is_wh25 || strcmp(child.key().c_str(), "WH25") == 0;
              bool child_inside_lightning = is_lightning ||
                                            textEqualsIgnoreCase(child.key().c_str(), "lightning") ||
                                            textEqualsIgnoreCase(child.key().c_str(), "WH57");
              bool child_inside_rain = is_rain || textEqualsIgnoreCase(child.key().c_str(), "srain_piezo");
              bool child_inside_common_list = is_common_list || textEqualsIgnoreCase(child.key().c_str(), "common_list");
              bool child_inside_piezo_rain = is_piezo_rain || textEqualsIgnoreCase(child.key().c_str(), "piezoRain");

              parseLiveDataItems(
                  child.value(),
                  data,
                  has_wind_dir,
                  has_wind,
                  has_gust,
                  has_wbgt,
                  has_temperature,
                  has_pressure,
                  has_humidity,
                  has_lightning,
                  has_rain,
                  has_rain_rate,
                  has_rain_day,
                  child_inside_wh25,
                  child_inside_lightning,
                  child_inside_rain,
                  child_inside_common_list,
                  child_inside_piezo_rain
              );
          }
      } else if (node.is<JsonArrayConst>()) {
          for (JsonVariantConst child : node.as<JsonArrayConst>()) {
              parseLiveDataItems(
                  child,
                  data,
                  has_wind_dir,
                  has_wind,
                  has_gust,
                  has_wbgt,
                  has_temperature,
                  has_pressure,
                  has_humidity,
                  has_lightning,
                  has_rain,
                  has_rain_rate,
                  has_rain_day,
                  inside_wh25,
                  inside_lightning,
                  inside_rain,
                  inside_common_list,
                  inside_piezo_rain
              );
          }
      }
  }

  bool updateWeatherData()
  {
      if (WiFi.status() != WL_CONNECTED) {
          return false;
      }

      char url[96];
      snprintf(url, sizeof(url), "http://%s/get_livedata_info", GW3000_IP);
      HTTPClient http;

      http.begin(url);
      int status_code = http.GET();

      if (status_code != HTTP_CODE_OK) {
          http.end();
          return false;
      }

      static DynamicJsonDocument doc(24576);
      doc.clear();
      DeserializationError error = deserializeJson(doc, http.getStream());
      http.end();
      if (error) {
          return false;
      }

      WeatherData parsed_data = weather_data;
      bool has_wind_dir = false;
      bool has_wind = false;
      bool has_gust = false;
      bool has_wbgt = false;
      bool has_temperature = false;
      bool has_pressure = false;
      bool has_humidity = false;
      bool has_lightning = false;
      bool has_rain = false;
      bool has_rain_rate = false;
      bool has_rain_day = false;

      parseLiveDataItems(
          doc.as<JsonVariantConst>(),
          parsed_data,
          has_wind_dir,
          has_wind,
          has_gust,
          has_wbgt,
          has_temperature,
          has_pressure,
          has_humidity,
          has_lightning,
          has_rain,
          has_rain_rate,
          has_rain_day
      );

      if (!has_wind_dir || !has_wind || !has_gust || !has_wbgt || !has_temperature || !has_humidity) {
          return false;
      }

      if (!has_lightning) {
          parsed_data.lightning_active = false;
      }

      if (!has_rain_rate) {
          parsed_data.rain_rate_mm_h = 0.0f;
      }

      if (!has_rain_day) {
          parsed_data.rain_day_mm = 0.0f;
      }

      parsed_data.rain_active = parsed_data.rain_rate_mm_h > 0.0f;

      if (!weatherDataChanged(weather_data, parsed_data)) {
          return false;
      }

      previous_weather_data = weather_data;
      has_previous_weather_data = true;
      weather_data = parsed_data;
      return true;
  }

  // Dashboard rendering
  uint16_t getWBGTColor(float wbgt_c)
  {
      if (wbgt_c < 20.0f) {
          return COLOR_GREEN;
      } else if (wbgt_c < 23.0f) {
          return COLOR_YELLOWGREEN;
      } else if (wbgt_c < 26.0f) {
          return COLOR_YELLOW;
      } else if (wbgt_c < 29.0f) {
          return COLOR_ORANGE;
      }

      return COLOR_RED;
  }

  uint16_t getWindSpeedColor(float wind_speed_kmh)
  {
      if (wind_speed_kmh < 5.0f) {
          return COLOR_LIGHTBLUE;
      } else if (wind_speed_kmh < 15.0f) {
          return COLOR_TURQUOISE;
      } else if (wind_speed_kmh < 30.0f) {
          return COLOR_GREEN;
      } else if (wind_speed_kmh < 50.0f) {
          return COLOR_YELLOW;
      } else if (wind_speed_kmh < 75.0f) {
          return COLOR_ORANGE;
      }

      return COLOR_RED;
  }

  void setPixel(uint16_t *buffer, int width, int height, int x, int y, uint16_t color)
  {
      if (x >= 0 && x < width && y >= 0 && y < height) {
          buffer[y * width + x] = color;
      }
  }

  uint16_t blendRGB565(uint16_t background, uint16_t foreground, uint8_t alpha)
  {
      uint8_t bg_r = ((background >> 11) & 0x1F) << 3;
      uint8_t bg_g = ((background >> 5) & 0x3F) << 2;
      uint8_t bg_b = (background & 0x1F) << 3;

      uint8_t fg_r = ((foreground >> 11) & 0x1F) << 3;
      uint8_t fg_g = ((foreground >> 5) & 0x3F) << 2;
      uint8_t fg_b = (foreground & 0x1F) << 3;

      uint8_t r = (bg_r * (255 - alpha) + fg_r * alpha) / 255;
      uint8_t g = (bg_g * (255 - alpha) + fg_g * alpha) / 255;
      uint8_t b = (bg_b * (255 - alpha) + fg_b * alpha) / 255;

      return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
  }

  void blendPixel(uint16_t *buffer, int width, int height, int x, int y, uint16_t color, uint8_t alpha)
  {
      if (x >= 0 && x < width && y >= 0 && y < height) {
          uint16_t *pixel = &buffer[y * width + x];
          *pixel = blendRGB565(*pixel, color, alpha);
      }
  }

  void drawFilledCircle(
      uint16_t *buffer,
      int width,
      int height,
      int cx,
      int cy,
      int radius,
      uint16_t color
  ) {
      int radius_sq = radius * radius;

      for (int y = -radius; y <= radius; y++) {
          for (int x = -radius; x <= radius; x++) {
              if (x * x + y * y <= radius_sq) {
                  setPixel(buffer, width, height, cx + x, cy + y, color);
              }
          }
      }
  }

  void drawFilledCircleAlpha(
      uint16_t *buffer,
      int width,
      int height,
      int cx,
      int cy,
      int radius,
      uint16_t color,
      uint8_t alpha
  ) {
      int radius_sq = radius * radius;

      for (int y = -radius; y <= radius; y++) {
          for (int x = -radius; x <= radius; x++) {
              if (x * x + y * y <= radius_sq) {
                  blendPixel(buffer, width, height, cx + x, cy + y, color, alpha);
              }
          }
      }
  }

  void drawStrokeLine(
      uint16_t *buffer,
      int width,
      int height,
      int x0,
      int y0,
      int x1,
      int y1,
      int thickness,
      uint16_t color
  ) {
      int dx = abs(x1 - x0);
      int sx = x0 < x1 ? 1 : -1;
      int dy = -abs(y1 - y0);
      int sy = y0 < y1 ? 1 : -1;
      int err = dx + dy;
      int radius = max(1, thickness / 2);

      while (true) {
          drawFilledCircle(buffer, width, height, x0, y0, radius, color);

          if (x0 == x1 && y0 == y1) {
              break;
          }

          int e2 = 2 * err;
          if (e2 >= dy) {
              err += dy;
              x0 += sx;
          }
          if (e2 <= dx) {
              err += dx;
              y0 += sy;
          }
      }
  }

  void drawGlyphSegment(
      uint16_t *buffer,
      int width,
      int height,
      int x,
      int y,
      int scale,
      int thickness,
      int x0,
      int y0,
      int x1,
      int y1,
      uint16_t color
  ) {
      drawStrokeLine(
          buffer,
          width,
          height,
          x + x0 * scale,
          y + y0 * scale,
          x + x1 * scale,
          y + y1 * scale,
          thickness,
          color
      );
  }

  void drawSmoothGlyph(
      uint16_t *buffer,
      int width,
      int height,
      char character,
      int x,
      int y,
      int scale,
      uint16_t color
  ) {
      int thickness = max(2, scale);
      bool segments[7] = {false, false, false, false, false, false, false};
      bool is_digit = true;

      switch (character) {
          case '0': segments[0] = segments[1] = segments[2] = segments[4] = segments[5] = segments[6] = true; break;
          case '1': segments[2] = segments[5] = true; break;
          case '2': segments[0] = segments[2] = segments[3] = segments[4] = segments[6] = true; break;
          case '3': segments[0] = segments[2] = segments[3] = segments[5] = segments[6] = true; break;
          case '4': segments[1] = segments[2] = segments[3] = segments[5] = true; break;
          case '5': segments[0] = segments[1] = segments[3] = segments[5] = segments[6] = true; break;
          case '6': segments[0] = segments[1] = segments[3] = segments[4] = segments[5] = segments[6] = true; break;
          case '7': segments[0] = segments[2] = segments[5] = true; break;
          case '8': segments[0] = segments[1] = segments[2] = segments[3] = segments[4] = segments[5] = segments[6] = true; break;
          case '9': segments[0] = segments[1] = segments[2] = segments[3] = segments[5] = segments[6] = true; break;
          default: is_digit = false; break;
      }

      if (is_digit) {
          if (segments[0]) drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 1, 0, 4, 0, color);
          if (segments[1]) drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 1, 0, 3, color);
          if (segments[2]) drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 5, 1, 5, 3, color);
          if (segments[3]) drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 1, 4, 4, 4, color);
          if (segments[4]) drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 5, 0, 7, color);
          if (segments[5]) drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 5, 5, 5, 7, color);
          if (segments[6]) drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 1, 8, 4, 8, color);
          return;
      }

      switch (character) {
          case 'C':
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 1, 0, 5, 0, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 1, 0, 7, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 1, 8, 5, 8, color);
              break;
          case 'N':
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 8, 0, 0, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 0, 5, 8, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 5, 8, 5, 0, color);
              break;
          case 'K':
          case 'k':
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 0, 0, 8, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 5, 0, 0, 4, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 4, 5, 8, color);
              break;
          case 'M':
          case 'm':
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 8, 0, 0, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 0, 2, 4, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 2, 4, 5, 0, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 5, 0, 5, 8, color);
              break;
          case 'H':
          case 'h':
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 0, 0, 8, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 4, 4, 4, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 5, 4, 5, 8, color);
              break;
          case 'E':
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 0, 0, 8, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 0, 5, 0, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 4, 4, 4, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 8, 5, 8, color);
              break;
          case 'P':
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 8, 0, 0, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 0, 4, 0, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 5, 1, 5, 3, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 4, 4, 4, color);
              break;
          case 'A':
          case 'a':
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 8, 0, 2, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 1, 0, 4, 0, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 5, 2, 5, 8, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 4, 5, 4, color);
              break;
          case 'S':
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 1, 0, 5, 0, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 1, 0, 3, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 1, 4, 4, 4, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 5, 5, 5, 7, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 8, 4, 8, color);
              break;
          case 'W':
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 0, 0, 8, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 0, 8, 2, 5, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 2, 5, 3, 8, color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 3, 8, 5, 0, color);
              break;
          case '-':
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 1, 4, 5, 4, color);
              break;
          case '/':
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 5, 0, 0, 8, color);
              break;
          case '%':
              drawFilledCircle(buffer, width, height, x + scale, y + scale, max(1, thickness / 2), color);
              drawFilledCircle(buffer, width, height, x + 4 * scale, y + 7 * scale, max(1, thickness / 2), color);
              drawGlyphSegment(buffer, width, height, x, y, scale, thickness, 5, 0, 0, 8, color);
              break;
          default:
              if ((uint8_t)character == 0xB0) {
                  int radius = max(2, scale);
                  drawFilledCircle(buffer, width, height, x + 2 * scale, y + scale, radius, color);
                  drawFilledCircle(buffer, width, height, x + 2 * scale, y + scale, max(1, radius - max(1, thickness / 2)), COLOR_BLACK);
              }
              break;
      }
  }

  int getTextCharWidth(char character, int scale)
  {
      if (character == ' ') {
          return 3 * scale;
      }

      if ((uint8_t)character == 0xB0) {
          return 4 * scale;
      }

      if (character == '/') {
          return 4 * scale;
      }

      if (character == '%') {
          return 6 * scale;
      }

      return 6 * scale;
  }

  bool isTextDigit(char character)
  {
      return character >= '0' && character <= '9';
  }

  int getTextCharSpacing(char current, char next, int scale)
  {
      if (next == '\0') {
          return 0;
      }

      if (isTextDigit(current) && isTextDigit(next)) {
          if (current == '1') {
              return scale * 2;
          }

          return scale;
      }

      return scale;
  }

  int getTextWidth(const char *text, int scale)
  {
      int text_width = 0;

      for (int i = 0; text[i] != '\0'; i++) {
          text_width += getTextCharWidth(text[i], scale);
          text_width += getTextCharSpacing(text[i], text[i + 1], scale);
      }

      return text_width;
  }

  void drawText(
      uint16_t *buffer,
      int width,
      int height,
      const char *text,
      int x,
      int y,
      int scale,
      uint16_t color
  ) {
      int cursor_x = x;

      for (int i = 0; text[i] != '\0'; i++) {
          int char_width = getTextCharWidth(text[i], scale);

          if (text[i] != ' ') {
              drawSmoothGlyph(buffer, width, height, text[i], cursor_x, y, scale, color);
          }

          cursor_x += char_width + getTextCharSpacing(text[i], text[i + 1], scale);
      }
  }

  void drawRainDecimalPoint(
      uint16_t *buffer,
      int width,
      int height,
      const char *text,
      int x,
      int y,
      int scale,
      uint16_t color
  ) {
      int cursor_x = x;

      for (int i = 0; text[i] != '\0'; i++) {
          int char_width = getTextCharWidth(text[i], scale);

          if (text[i] == '.') {
              drawFilledCircle(
                  buffer,
                  width,
                  height,
                  cursor_x + char_width / 2,
                  y + 8 * scale,
                  max(2, scale),
                  color
              );
              return;
          }

          cursor_x += char_width + getTextCharSpacing(text[i], text[i + 1], scale);
      }
  }

  void drawCompassLetter(
      uint16_t *buffer,
      int width,
      int height,
      char letter,
      int x,
      int y,
      int scale,
      uint16_t color
  ) {
      drawSmoothGlyph(buffer, width, height, letter, x, y, scale, color);
  }

  void transformWifiPoint(
      float local_x,
      float local_y,
      float radial_angle_rad,
      int origin_x,
      int origin_y,
      int &screen_x,
      int &screen_y
  ) {
      float tangent_x = cosf(radial_angle_rad);
      float tangent_y = sinf(radial_angle_rad);
      float inward_x = -sinf(radial_angle_rad);
      float inward_y = cosf(radial_angle_rad);

      screen_x = origin_x + (int)roundf(local_x * tangent_x + local_y * inward_x);
      screen_y = origin_y + (int)roundf(local_x * tangent_y + local_y * inward_y);
  }

  void drawRotatedArcStrokeAlpha(
      uint16_t *buffer,
      int width,
      int height,
      int origin_x,
      int origin_y,
      float radial_angle_rad,
      float local_cx,
      float local_cy,
      int radius,
      float start_deg,
      float end_deg,
      int thickness,
      uint16_t color,
      uint8_t alpha
  ) {
      for (float angle_deg = start_deg; angle_deg <= end_deg; angle_deg += 3.0f) {
          float angle_rad = angle_deg * PI / 180.0f;
          float local_x = local_cx + cosf(angle_rad) * radius;
          float local_y = local_cy + sinf(angle_rad) * radius;
          int x = 0;
          int y = 0;

          transformWifiPoint(local_x, local_y, radial_angle_rad, origin_x, origin_y, x, y);
          drawFilledCircleAlpha(buffer, width, height, x, y, max(1, thickness / 2), color, alpha);
      }
  }

  void drawWifiStatusIcon(
      uint16_t *buffer,
      int width,
      int height,
      int cx,
      int cy,
      int size,
      float radial_angle_deg,
      bool connected
  ) {
      uint16_t color = connected ? COLOR_GREEN : COLOR_RED;
      uint8_t alpha = connected ? 150 : 170;
      int thickness = max(2, size / 8);
      float radial_angle_rad = radial_angle_deg * PI / 180.0f;
      float dot_y = size / 3.0f;

      int dot_x = 0;
      int dot_screen_y = 0;
      transformWifiPoint(0.0f, dot_y, radial_angle_rad, cx, cy, dot_x, dot_screen_y);
      drawFilledCircleAlpha(buffer, width, height, dot_x, dot_screen_y, max(2, thickness), color, alpha);
      drawRotatedArcStrokeAlpha(buffer, width, height, cx, cy, radial_angle_rad, 0.0f, dot_y, size / 3, 215.0f, 325.0f, thickness, color, alpha);
      drawRotatedArcStrokeAlpha(buffer, width, height, cx, cy, radial_angle_rad, 0.0f, dot_y, size / 2, 210.0f, 330.0f, thickness, color, alpha);
      drawRotatedArcStrokeAlpha(buffer, width, height, cx, cy, radial_angle_rad, 0.0f, dot_y, (size * 2) / 3, 205.0f, 335.0f, thickness, color, alpha);

      if (!connected) {
          int x0 = 0;
          int y0 = 0;
          int x1 = 0;
          int y1 = 0;
          transformWifiPoint(-size / 2.0f, -size / 3.0f, radial_angle_rad, cx, cy, x0, y0);
          transformWifiPoint(size / 2.0f, size / 2.0f, radial_angle_rad, cx, cy, x1, y1);
          drawStrokeLine(
              buffer,
              width,
              height,
              x0,
              y0,
              x1,
              y1,
              thickness,
              COLOR_RED
          );
      }
  }

  bool pointInPolygon(int x, int y, const int *poly_x, const int *poly_y, int count)
  {
      bool inside = false;

      for (int i = 0, j = count - 1; i < count; j = i++) {
          bool intersects = ((poly_y[i] > y) != (poly_y[j] > y)) &&
                            (x < (poly_x[j] - poly_x[i]) * (y - poly_y[i]) / (poly_y[j] - poly_y[i] + 0.0001f) + poly_x[i]);
          if (intersects) {
              inside = !inside;
          }
      }

      return inside;
  }

  void drawLightningStatusIcon(
      uint16_t *buffer,
      int width,
      int height,
      int cx,
      int cy,
      int size,
      float radial_angle_deg,
      bool active
  ) {
      const int point_count = 6;
      float local_x[point_count] = {
          size * 0.12f,
          -size * 0.22f,
          size * 0.02f,
          -size * 0.18f,
          size * 0.28f,
          size * 0.06f
      };
      float local_y[point_count] = {
          -size * 0.48f,
          -size * 0.06f,
          -size * 0.06f,
          size * 0.48f,
          -size * 0.18f,
          -size * 0.18f
      };
      int poly_x[point_count];
      int poly_y[point_count];
      float radial_angle_rad = radial_angle_deg * PI / 180.0f;

      for (int i = 0; i < point_count; i++) {
          transformWifiPoint(local_x[i], local_y[i], radial_angle_rad, cx, cy, poly_x[i], poly_y[i]);
      }

      uint16_t outline_color = active ? COLOR_YELLOW : COLOR_GRAY;
      int thickness = max(2, size / 8);

      if (active) {
          int min_x = poly_x[0];
          int max_x = poly_x[0];
          int min_y = poly_y[0];
          int max_y = poly_y[0];

          for (int i = 1; i < point_count; i++) {
              min_x = min(min_x, poly_x[i]);
              max_x = max(max_x, poly_x[i]);
              min_y = min(min_y, poly_y[i]);
              max_y = max(max_y, poly_y[i]);
          }

          for (int y = min_y; y <= max_y; y++) {
              for (int x = min_x; x <= max_x; x++) {
                  if (pointInPolygon(x, y, poly_x, poly_y, point_count)) {
                      setPixel(buffer, width, height, x, y, COLOR_YELLOW);
                  }
              }
          }
      }

      for (int i = 0; i < point_count; i++) {
          int next = (i + 1) % point_count;
          drawStrokeLine(
              buffer,
              width,
              height,
              poly_x[i],
              poly_y[i],
              poly_x[next],
              poly_y[next],
              thickness,
              outline_color
          );
      }
  }

  void drawRainStatusIcon(
      uint16_t *buffer,
      int width,
      int height,
      int cx,
      int cy,
      int size,
      float radial_angle_deg,
      bool active,
      uint16_t active_color
  ) {
      const int point_count = 8;
      float local_x[point_count] = {
          0.0f,
          size * 0.26f,
          size * 0.36f,
          size * 0.28f,
          0.0f,
          -size * 0.28f,
          -size * 0.36f,
          -size * 0.26f
      };
      float local_y[point_count] = {
          -size * 0.48f,
          -size * 0.18f,
          size * 0.10f,
          size * 0.34f,
          size * 0.48f,
          size * 0.34f,
          size * 0.10f,
          -size * 0.18f
      };
      int poly_x[point_count];
      int poly_y[point_count];
      float radial_angle_rad = radial_angle_deg * PI / 180.0f;

      for (int i = 0; i < point_count; i++) {
          transformWifiPoint(local_x[i], local_y[i], radial_angle_rad, cx, cy, poly_x[i], poly_y[i]);
      }

      uint16_t outline_color = active ? active_color : COLOR_GRAY;
      int thickness = max(2, size / 8);

      if (active) {
          int min_x = poly_x[0];
          int max_x = poly_x[0];
          int min_y = poly_y[0];
          int max_y = poly_y[0];

          for (int i = 1; i < point_count; i++) {
              min_x = min(min_x, poly_x[i]);
              max_x = max(max_x, poly_x[i]);
              min_y = min(min_y, poly_y[i]);
              max_y = max(max_y, poly_y[i]);
          }

          for (int y = min_y; y <= max_y; y++) {
              for (int x = min_x; x <= max_x; x++) {
                  if (pointInPolygon(x, y, poly_x, poly_y, point_count)) {
                      setPixel(buffer, width, height, x, y, active_color);
                  }
              }
          }
      }

      for (int i = 0; i < point_count; i++) {
          int next = (i + 1) % point_count;
          drawStrokeLine(
              buffer,
              width,
              height,
              poly_x[i],
              poly_y[i],
              poly_x[next],
              poly_y[next],
              thickness,
              outline_color
          );
      }
  }

  void drawTrendArrow(
      uint16_t *buffer,
      int width,
      int height,
      int x,
      int y,
      int size,
      bool up,
      uint16_t color
  ) {
      int thickness = max(2, size / 5);
      int stem_top = up ? y + size / 4 : y;
      int stem_bottom = up ? y + size : y + (size * 3) / 4;
      int tip_y = up ? y : y + size;
      int wing_y = up ? y + size / 3 : y + (size * 2) / 3;

      drawStrokeLine(buffer, width, height, x, stem_top, x, stem_bottom, thickness, color);
      drawStrokeLine(buffer, width, height, x, tip_y, x - size / 3, wing_y, thickness, color);
      drawStrokeLine(buffer, width, height, x, tip_y, x + size / 3, wing_y, thickness, color);
  }

  void drawTrendArrowForValues(
      uint16_t *buffer,
      int width,
      int height,
      int x,
      int y,
      int size,
      float current,
      float previous,
      float epsilon
  ) {
      float diff = current - previous;
      if (fabs(diff) <= epsilon) {
          return;
      }

      drawTrendArrow(buffer, width, height, x, y, size, diff > 0.0f, COLOR_LIGHTBLUE);
  }

  void drawTrendArrowForValues(
      uint16_t *buffer,
      int width,
      int height,
      int x,
      int y,
      int size,
      float current,
      float previous,
      float epsilon,
      uint16_t up_color,
      uint16_t down_color
  ) {
      float diff = current - previous;
      if (fabs(diff) <= epsilon) {
          return;
      }

      drawTrendArrow(buffer, width, height, x, y, size, diff > 0.0f, diff > 0.0f ? up_color : down_color);
  }

  uint16_t getRainColor(float rain_rate_mm_h)
  {
      if (rain_rate_mm_h <= 0.0f) {
          return COLOR_GRAY;
      } else if (rain_rate_mm_h < 5.0f) {
          return COLOR_LIGHTBLUE;
      } else if (rain_rate_mm_h < 20.0f) {
          return COLOR_BLUE;
      } else if (rain_rate_mm_h < 50.0f) {
          return COLOR_PURPLE;
      }

      return COLOR_RED;
  }

  void formatRainValue(char *buffer, size_t buffer_size, float value, const char *unit)
  {
      if (value >= 100.0f) {
          snprintf(buffer, buffer_size, "%.0f %s", value, unit);
      } else {
          snprintf(buffer, buffer_size, "%.1f %s", value, unit);
      }
  }

  float normalizeAngle(float angle)
  {
      while (angle < 0) {
          angle += 360.0;
      }

      while (angle >= 360.0) {
          angle -= 360.0;
      }

      return angle;
  }

  float angleDifference(float a, float b)
  {
      float diff = normalizeAngle(a - b);

      if (diff > 180.0) {
          diff -= 360.0;
      }

      return fabs(diff);
  }

  void drawCompassRing(LCD *lcd)
  {
      const int width = lcd->getFrameWidth();
      const int height = lcd->getFrameHeight();

      if (!ensureDrawBuffer(width, height)) {
          return;
      }

      uint16_t *buffer = lcd_draw_buffer;

      for (int i = 0; i < width * height; i++) {
          buffer[i] = COLOR_BLACK;
      }

      const int cx = width / 2;
      const int cy = height / 2;

      const int edge_margin = 2;
      const int ring_thickness = 6;

      const int outer_radius = min(width, height) / 2 - edge_margin;
      const int inner_radius = outer_radius - ring_thickness;

      const int outer_sq = outer_radius * outer_radius;
      const int inner_sq = inner_radius * inner_radius;

      for (int y = 0; y < height; y++) {
          for (int x = 0; x < width; x++) {
              int dx = x - cx;
              int dy = y - cy;
              int dist_sq = dx * dx + dy * dy;

              if (dist_sq <= outer_sq && dist_sq >= inner_sq) {
                  buffer[y * width + x] = COLOR_TEAL;
              }
          }
      }

      int label_scale = max(1, min(width, height) / 150);
      int label_w = 6 * label_scale;
      int label_h = 9 * label_scale;

      int label_radius = outer_radius - ring_thickness - label_h - 4;

      const int wbgt_thickness = ring_thickness * 2;
      const int wbgt_outer_radius = label_radius - label_h / 2 - 4;
      const int wbgt_inner_radius = wbgt_outer_radius - wbgt_thickness;

      const int wbgt_outer_sq = wbgt_outer_radius * wbgt_outer_radius;
      const int wbgt_inner_sq = wbgt_inner_radius * wbgt_inner_radius;

      uint16_t wbgt_color = getWBGTColor(weather_data.wbgt_c);

      for (int y = 0; y < height; y++) {
          for (int x = 0; x < width; x++) {
              int dx = x - cx;
              int dy = y - cy;
              int dist_sq = dx * dx + dy * dy;

              if (dist_sq <= wbgt_outer_sq && dist_sq >= wbgt_inner_sq) {
                  buffer[y * width + x] = wbgt_color;
              }
          }
      }

      const int wind_outer_radius = outer_radius;
      const int wind_inner_radius = inner_radius - 2;

      const int wind_outer_sq = wind_outer_radius * wind_outer_radius;
      const int wind_inner_sq = wind_inner_radius * wind_inner_radius;

      float base_segment_width_deg = (float)label_w * 180.0 / (PI * outer_radius);
      float wind_half_width_deg = max(8.0f, base_segment_width_deg * 2.0f);
      float border_width_deg = 2.0f * 180.0 / (PI * outer_radius);

      uint16_t wind_color = getWindSpeedColor(weather_data.wind_speed_kmh);

      for (int y = 0; y < height; y++) {
          for (int x = 0; x < width; x++) {
              int dx = x - cx;
              int dy = y - cy;
              int dist_sq = dx * dx + dy * dy;

              if (dist_sq <= wind_outer_sq && dist_sq >= wind_inner_sq) {
                  float angle_deg = atan2((float)dy, (float)dx) * 180.0 / PI + 90.0;
                  angle_deg = normalizeAngle(angle_deg);

                  float diff = angleDifference(angle_deg, weather_data.wind_direction_deg);

                  if (diff <= wind_half_width_deg) {
                      float distance_to_segment_edge = wind_half_width_deg - diff;

                      if (distance_to_segment_edge <= border_width_deg) {
                          buffer[y * width + x] = COLOR_WHITE;
                      } else {
                          buffer[y * width + x] = wind_color;
                      }
                  }
              }
          }
      }

      const float wifi_angle_deg = 22.5f;
      const float wifi_angle_rad = wifi_angle_deg * PI / 180.0f;
      int wifi_radius = (inner_radius + wbgt_outer_radius) / 2;
      int wifi_size = max(14, min(width, height) / 18);
      int wifi_x = cx + (int)roundf(sinf(wifi_angle_rad) * wifi_radius);
      int wifi_y = cy - (int)roundf(cosf(wifi_angle_rad) * wifi_radius);

      drawWifiStatusIcon(
          buffer,
          width,
          height,
          wifi_x,
          wifi_y,
          wifi_size,
          wifi_angle_deg,
          WiFi.status() == WL_CONNECTED
      );

      const float lightning_angle_deg = 337.5f;
      const float lightning_angle_rad = lightning_angle_deg * PI / 180.0f;
      int lightning_radius = wifi_radius;
      int lightning_size = wifi_size;
      int lightning_x = cx + (int)roundf(sinf(lightning_angle_rad) * lightning_radius);
      int lightning_y = cy - (int)roundf(cosf(lightning_angle_rad) * lightning_radius);

      drawLightningStatusIcon(
          buffer,
          width,
          height,
          lightning_x,
          lightning_y,
          lightning_size,
          lightning_angle_deg,
          weather_data.lightning_active
      );

      const float rain_angle_deg = 292.5f;
      const float rain_angle_rad = rain_angle_deg * PI / 180.0f;
      int rain_radius = wifi_radius;
      int rain_size = wifi_size;
      int rain_x = cx + (int)roundf(sinf(rain_angle_rad) * rain_radius);
      int rain_y = cy - (int)roundf(cosf(rain_angle_rad) * rain_radius);
      const float rain_icon_angle_deg = 0.0f;

      drawRainStatusIcon(
          buffer,
          width,
          height,
          rain_x,
          rain_y,
          rain_size,
          rain_icon_angle_deg,
          weather_data.rain_active,
          getRainColor(weather_data.rain_rate_mm_h)
      );

      drawCompassLetter(
          buffer,
          width,
          height,
          'N',
          cx - label_w / 2,
          cy - label_radius - label_h / 2,
          label_scale,
          COLOR_WHITE
      );

      drawCompassLetter(
          buffer,
          width,
          height,
          'E',
          cx + label_radius - label_w / 2,
          cy - label_h / 2,
          label_scale,
          COLOR_WHITE
      );

      drawCompassLetter(
          buffer,
          width,
          height,
          'S',
          cx - label_w / 2,
          cy + label_radius - label_h / 2,
          label_scale,
          COLOR_WHITE
      );

      drawCompassLetter(
          buffer,
          width,
          height,
          'W',
          cx - label_radius - label_w / 2,
          cy - label_h / 2,
          label_scale,
          COLOR_WHITE
      );

      char temperature_text[8];
      snprintf(temperature_text, sizeof(temperature_text), "%.0f\xB0" "C", weather_data.temperature_c);

      int temperature_scale = max(3, min(width, height) / 68);
      int temperature_w = getTextWidth(temperature_text, temperature_scale);
      int temperature_h = 9 * temperature_scale;

      drawText(
          buffer,
          width,
          height,
          temperature_text,
          cx - temperature_w / 2,
          cy - temperature_h / 2,
          temperature_scale,
          COLOR_WHITE
      );

      int rain_scale = max(2, (temperature_scale * 2) / 5);
      char rain_rate_text[16];
      char rain_day_text[16];
      formatRainValue(rain_rate_text, sizeof(rain_rate_text), weather_data.rain_rate_mm_h, "mm/h");
      formatRainValue(rain_day_text, sizeof(rain_day_text), weather_data.rain_day_mm, "mm");

      int rain_rate_w = getTextWidth(rain_rate_text, rain_scale);
      int rain_day_w = getTextWidth(rain_day_text, rain_scale);
      int rain_text_h = 9 * rain_scale;
      int rain_day_y = cy - temperature_h / 2 - max(10, temperature_scale * 2) - rain_text_h;
      int rain_rate_y = rain_day_y - rain_text_h - max(2, rain_scale);
      uint16_t rain_text_color = blendRGB565(COLOR_BLACK, COLOR_WHITE, 170);

      drawText(
          buffer,
          width,
          height,
          rain_rate_text,
          cx - rain_rate_w / 2,
          rain_rate_y,
          rain_scale,
          rain_text_color
      );
      drawRainDecimalPoint(
          buffer,
          width,
          height,
          rain_rate_text,
          cx - rain_rate_w / 2,
          rain_rate_y,
          rain_scale,
          rain_text_color
      );

      drawText(
          buffer,
          width,
          height,
          rain_day_text,
          cx - rain_day_w / 2,
          rain_day_y,
          rain_scale,
          rain_text_color
      );
      drawRainDecimalPoint(
          buffer,
          width,
          height,
          rain_day_text,
          cx - rain_day_w / 2,
          rain_day_y,
          rain_scale,
          rain_text_color
      );

      if (has_previous_weather_data) {
          int temperature_arrow_size = max(8, temperature_h / 2);
          int temperature_arrow_x = cx + temperature_w / 2 + max(10, temperature_scale * 2);
          int temperature_arrow_y = cy - temperature_h / 2 - temperature_arrow_size / 4;
          uint16_t temperature_rise_color = blendRGB565(COLOR_BLACK, COLOR_RED, 150);

          drawTrendArrowForValues(
              buffer,
              width,
              height,
              temperature_arrow_x,
              temperature_arrow_y,
              temperature_arrow_size,
              weather_data.temperature_c,
              previous_weather_data.temperature_c,
              0.5f,
              temperature_rise_color,
              COLOR_LIGHTBLUE
          );
      }

      char pressure_text[16];
      snprintf(pressure_text, sizeof(pressure_text), "%.0f hPa", weather_data.pressure_hpa);

      int pressure_scale = max(2, (temperature_scale * 2) / 5);
      int pressure_w = getTextWidth(pressure_text, pressure_scale);
      int pressure_h = 9 * pressure_scale;
      int pressure_y = cy + temperature_h / 2 + max(12, temperature_scale * 2);
      uint16_t pressure_color = blendRGB565(COLOR_BLACK, COLOR_WHITE, 170);

      drawText(
          buffer,
          width,
          height,
          pressure_text,
          cx - pressure_w / 2,
          pressure_y,
          pressure_scale,
          pressure_color
      );

      if (has_previous_weather_data) {
          int pressure_arrow_size = max(6, pressure_h / 2);
          int pressure_arrow_x = cx + pressure_w / 2 + max(4, pressure_scale);
          int pressure_arrow_y = pressure_y - pressure_arrow_size / 4;

          drawTrendArrowForValues(
              buffer,
              width,
              height,
              pressure_arrow_x,
              pressure_arrow_y,
              pressure_arrow_size,
              weather_data.pressure_hpa,
              previous_weather_data.pressure_hpa,
              0.05f
          );
      }

      char humidity_text[12];
      snprintf(humidity_text, sizeof(humidity_text), "%.0f%%", weather_data.humidity_percent);

      int humidity_scale = pressure_scale;
      int humidity_w = getTextWidth(humidity_text, humidity_scale);
      int humidity_y = pressure_y + pressure_h + max(4, humidity_scale);
      uint16_t humidity_color = pressure_color;

      drawText(
          buffer,
          width,
          height,
          humidity_text,
          cx - humidity_w / 2,
          humidity_y,
          humidity_scale,
          humidity_color
      );

      int lightning_distance_value_scale = max(2, (pressure_scale * 3) / 4);
      int lightning_distance_unit_scale = max(1, lightning_distance_value_scale - 1);
      int lightning_distance_y = cy - wbgt_inner_radius + max(10, lightning_distance_value_scale * 3);
      int lightning_distance_day_offset = 9 * lightning_distance_value_scale + max(3, lightning_distance_unit_scale * 2);
      if (weather_data.lightning_active || has_lightning_distance_today) {
          char lightning_distance_value_text[8];
          char lightning_distance_unit_text[] = "km";
          snprintf(
              lightning_distance_value_text,
              sizeof(lightning_distance_value_text),
              "%.0f",
              weather_data.lightning_distance_km
          );

          int lightning_distance_value_w = getTextWidth(lightning_distance_value_text, lightning_distance_value_scale);
          int lightning_distance_unit_w = getTextWidth(lightning_distance_unit_text, lightning_distance_unit_scale);
          int lightning_distance_gap = max(2, lightning_distance_unit_scale);
          int lightning_distance_w = lightning_distance_value_w + lightning_distance_gap + lightning_distance_unit_w;
          int lightning_distance_x = cx - lightning_distance_w / 2;
          uint16_t lightning_distance_text_color = weather_data.lightning_active
              ? blendRGB565(COLOR_BLACK, COLOR_YELLOW, 190)
              : blendRGB565(COLOR_BLACK, COLOR_GRAY, 180);

          drawText(
              buffer,
              width,
              height,
              lightning_distance_value_text,
              lightning_distance_x,
              lightning_distance_y,
              lightning_distance_value_scale,
              lightning_distance_text_color
          );
          drawText(
              buffer,
              width,
              height,
              lightning_distance_unit_text,
              lightning_distance_x + lightning_distance_value_w + lightning_distance_gap,
              lightning_distance_y + max(1, lightning_distance_value_scale),
              lightning_distance_unit_scale,
              lightning_distance_text_color
          );

      }

      if (has_lightning_distance_today) {
          char lightning_distance_day_text[8];
          char lightning_distance_day_unit_text[] = "km";
          snprintf(
              lightning_distance_day_text,
              sizeof(lightning_distance_day_text),
              "%.0f",
              lightning_distance_today_km
          );

          int lightning_distance_day_value_scale = lightning_distance_value_scale;
          int lightning_distance_day_unit_scale = lightning_distance_unit_scale;
          int lightning_distance_day_value_w = getTextWidth(lightning_distance_day_text, lightning_distance_day_value_scale);
          int lightning_distance_day_unit_w = getTextWidth(lightning_distance_day_unit_text, lightning_distance_day_unit_scale);
          int lightning_distance_day_gap = max(2, lightning_distance_day_unit_scale);
          int lightning_distance_day_w = lightning_distance_day_value_w + lightning_distance_day_gap + lightning_distance_day_unit_w;
          int lightning_distance_day_y = lightning_distance_y + lightning_distance_day_offset;
          int lightning_distance_day_x = cx - lightning_distance_day_w / 2;
          uint16_t lightning_distance_day_color = blendRGB565(COLOR_BLACK, COLOR_WHITE, 170);

          drawText(
              buffer,
              width,
              height,
              lightning_distance_day_text,
              lightning_distance_day_x,
              lightning_distance_day_y,
              lightning_distance_day_value_scale,
              lightning_distance_day_color
          );
          drawText(
              buffer,
              width,
              height,
              lightning_distance_day_unit_text,
              lightning_distance_day_x + lightning_distance_day_value_w + lightning_distance_day_gap,
              lightning_distance_day_y + max(1, lightning_distance_day_value_scale),
              lightning_distance_day_unit_scale,
              lightning_distance_day_color
          );
      }

      char wind_speed_value_text[8];
      char wind_speed_unit_text[] = "kmh";
      snprintf(wind_speed_value_text, sizeof(wind_speed_value_text), "%.0f", weather_data.wind_speed_kmh);

      int wind_speed_value_scale = max(2, (pressure_scale * 3) / 4);
      int wind_speed_unit_scale = max(1, wind_speed_value_scale - 1);
      int wind_speed_value_w = getTextWidth(wind_speed_value_text, wind_speed_value_scale);
      int wind_speed_unit_w = getTextWidth(wind_speed_unit_text, wind_speed_unit_scale);
      int wind_speed_gap = max(2, wind_speed_unit_scale);
      int wind_speed_w = wind_speed_value_w + wind_speed_gap + wind_speed_unit_w;
      int wind_speed_h = max(9 * wind_speed_value_scale, 9 * wind_speed_unit_scale);
      int wind_speed_y = cy + wbgt_inner_radius - wind_speed_h - max(10, wind_speed_value_scale * 3);
      int wind_speed_x = cx - wind_speed_w / 2;
      uint16_t wind_speed_text_color = blendRGB565(COLOR_BLACK, COLOR_WHITE, 160);

      drawText(
          buffer,
          width,
          height,
          wind_speed_value_text,
          wind_speed_x,
          wind_speed_y,
          wind_speed_value_scale,
          wind_speed_text_color
      );
      drawText(
          buffer,
          width,
          height,
          wind_speed_unit_text,
          wind_speed_x + wind_speed_value_w + wind_speed_gap,
          wind_speed_y + max(1, wind_speed_value_scale),
          wind_speed_unit_scale,
          wind_speed_text_color
      );

      lcd->drawBitmap(0, 0, width, height, (const uint8_t *)buffer);
  }

  // Setup / Loop
  void setup()
  {
      Serial.begin(115200);

      if (connectWiFi()) {
          syncTime();
      }
      updateWeatherData();

      board = new Board(BOARD_EXTERNAL_CONFIG);
      assert(board->begin());

      auto backlight = board->getBacklight();
      if (backlight != nullptr) {
          backlight->off();
      }

      auto lcd = board->getLCD();
      if (lcd != nullptr) {
          drawCompassRing(lcd);
          last_weather_update_ms = millis();
      }

      updateDisplayBrightness();
      last_brightness_update_ms = millis();
  }

  void loop()
  {
      if (millis() - last_weather_update_ms >= WEATHER_UPDATE_PERIOD_MS) {
          if (updateWeatherData()) {
              auto lcd = board->getLCD();
              if (lcd != nullptr) {
                  drawCompassRing(lcd);
              }
          }

          last_weather_update_ms = millis();
      }

      if (millis() - last_brightness_update_ms >= BRIGHTNESS_UPDATE_PERIOD_MS) {
          updateDisplayBrightness();
          last_brightness_update_ms = millis();
      }

      delay(50);
  }
