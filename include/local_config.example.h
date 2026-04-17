#pragma once

#include <WiFi.h>

// Copy this file to `include/local_config.h` and fill in your local values.

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const IPAddress SERVER_IP(192, 168, 0, 10);
const uint16_t SERVER_PORT = 8001;
const uint16_t UDP_LOCAL_PORT = 4210;
