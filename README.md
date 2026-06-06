# WetterKompassDisplay

WetterKompassDisplay is an Arduino/ESP32 sketch for a circular weather dashboard on a Waveshare ESP32-S3 touch display. It reads live data from an Ecowitt/GW3000 weather station and renders wind, temperature, pressure, humidity, rain, lightning distance, WBGT and status icons in a compact compass-style UI.

## Hardware

- Waveshare ESP32-S3-Touch-LCD-2.8C
- 480x480 RGB LCD
- Ecowitt/GW3000 weather station reachable in the local network
- WLAN access point for the ESP32

## Libraries

This project uses [`ESP32_Display_Panel`](https://github.com/esp-arduino-libs/ESP32_Display_Panel) by Espressif / `esp-arduino-libs` for board, LCD, touch and backlight setup.

Credits go to Espressif and the `esp-arduino-libs` maintainers for `ESP32_Display_Panel`. The library is licensed under Apache-2.0. Check the upstream repository and installed library package for the full license text and notices.

The included `board_external_config.*`, `esp_utils_conf.h` and `esp_panel_drivers_conf.h` files are based on or adapted from Espressif / `esp-arduino-libs` files and keep their original Apache-2.0 SPDX license headers.

No LVGL dependency is required for this sketch.

## Installation

1. Install Arduino IDE with ESP32 board support.
2. Install the `ESP32_Display_Panel` library from Espressif / `esp-arduino-libs`.
3. Open `WetterKompassDisplay.ino` in Arduino IDE.
4. Select the matching ESP32-S3 Waveshare board configuration and enable PSRAM.
5. Copy `secrets.example.h` to `secrets.h`.
6. Edit `secrets.h` with your WLAN credentials and GW3000 IP address or hostname.
7. Compile and upload the sketch.

`secrets.h` is intentionally listed in `.gitignore` and must not be committed.

## Configuration

The live data endpoint is built from `GW3000_IP`:

```text
http://<GW3000_IP>/get_livedata_info
```

The display updates weather data every 60 seconds:

```cpp
#define WEATHER_UPDATE_PERIOD_MS 60000
```

Backlight brightness is adjusted every 60 seconds based on local NTP time:

- `22:00-06:00`: 10 %
- `06:00-08:00` and `19:00-22:00`: 35 %
- `08:00-19:00`: 100 %

## Display Values

| Position | Value |
| --- | --- |
| Outer ring segment | Wind direction and wind speed color |
| Inner ring | WBGT color |
| Center | Temperature |
| Right of temperature | Temperature trend arrow |
| Above temperature | Rain rate and daily rain |
| Below temperature | Relative pressure |
| Right of pressure | Pressure trend arrow |
| Below pressure | Humidity |
| Upper right ring | WLAN status |
| Upper left ring | Lightning status |
| Left ring | Rain status |
| Upper inner area | Current or last lightning distance |
| Below lightning distance | Last lightning distance stored for the current day |

## Data Mapping

Values are parsed recursively from the GW3000 JSON response. Numeric values may be read from `val`, `value` or `data`. IDs may be provided as `id`, `unit_id`, `unitid` or `unit`.

| Display value | Sketch variable | JSON identifier | Parameter |
| --- | --- | --- | --- |
| Temperature | `temperature_c` | ID `0x02` | `val`, `value`, `data` |
| Wind direction | `wind_direction_deg` | ID `0x0A` | `val`, `value`, `data` |
| Wind speed | `wind_speed_kmh` | ID `0x0B` | `val`, `value`, `data`, converted from m/s to km/h |
| Wind gust | `gust_speed_kmh` | ID `0x0C` | `val`, `value`, `data`, converted from m/s to km/h |
| WBGT | `wbgt_c` | ID `0xA2` | `val`, `value`, `data` |
| Humidity | `humidity_percent` | `common_list` + ID `0x07` | `val`, `value`, `data` |
| Relative pressure | `pressure_hpa` | `WH25` | `rel` |
| Lightning distance | `lightning_distance_km` | `lightning` or `WH57` | `distance`, `distance_km` |
| Lightning active | `lightning_active` | `lightning` or `WH57` | `timestamp`, `time`, `last_time`, optional distance |
| Rain rate | `rain_rate_mm_h` | `srain_piezo` or `piezoRain` + ID `0x0E` | `val`, `value`, `data` |
| Daily rain | `rain_day_mm` | `piezoRain` + ID `0x7C` | `val`, `value`, `data` |

## License

The original WetterKompassDisplay sketch code is licensed under the MIT License by Markus Woods. See `LICENSE`.

Some configuration files in this repository are based on or adapted from Espressif / `esp-arduino-libs` files and keep their original SPDX headers:

- `board_external_config.cpp`
- `board_external_config.hpp`
- `esp_utils_conf.h`
- `esp_panel_drivers_conf.h`

Those files are licensed under Apache-2.0 as indicated in their file headers. A copy of the Apache-2.0 license text is included in `LICENSES/Apache-2.0.txt`.

`ESP32_Display_Panel` itself is maintained by Espressif / `esp-arduino-libs` and is licensed under Apache-2.0.
