#include <WiFi.h>
#include <WiFiUdp.h>
#include <M5Unified.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include "local_config.h"

const unsigned long SAMPLE_INTERVAL_MS = 10;      // 100Hz
const unsigned long SEND_INTERVAL_MS = 100;       // send every 10 samples
const unsigned long DISPLAY_INTERVAL_MS = 1000;
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 8000;
const unsigned long BUTTON_DEBOUNCE_MS = 250;
const unsigned long RSSI_UPDATE_INTERVAL_MS = 1000;
const unsigned long UDP_SEND_GRACE_MS = 500;
const unsigned long UDP_RETRY_BACKOFF_MS = 1000;
const unsigned long SERVER_DISCOVERY_TIMEOUT_MS = 1200;
const size_t CALIBRATION_SAMPLES = 32;
const size_t MAX_BUFFERED_SAMPLES = 1000;
const size_t MAX_UDP_SAMPLES = 20;
const uint16_t UDP_PACKET_MAGIC = 0x5349;
const uint8_t UDP_PACKET_VERSION = 1;
const size_t UDP_HEADER_BYTES = 16;
const size_t UDP_SAMPLE_BYTES = 14;
const float ACCEL_SCALE = 1000.0f;
const float GYRO_SCALE = 100.0f;
const size_t LOG_LINES = 4;
const int BUTTON_PIN = 41;
const char SERVER_DISCOVERY_REQUEST[] = "SI_DISCOVER_V1";
const char SERVER_DISCOVERY_RESPONSE[] = "SI_SERVER_V1";

WiFiUDP udp;

struct ImuSample {
  uint32_t timestamp_ms;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
};

ImuSample sampleBuffer[MAX_BUFFERED_SAMPLES];
size_t bufferHead = 0;
size_t bufferedSamples = 0;
unsigned long droppedSamples = 0;

float lastAx = 0.0f;
float lastAy = 0.0f;
float lastAz = 0.0f;
float lastGx = 0.0f;
float lastGy = 0.0f;
float lastGz = 0.0f;
float rawAx = 0.0f;
float rawAy = 0.0f;
float rawAz = 0.0f;
float rawGx = 0.0f;
float rawGy = 0.0f;
float rawGz = 0.0f;

unsigned long lastSampleAt = 0;
unsigned long lastSendAt = 0;
unsigned long lastDisplayAt = 0;
unsigned long lastButtonAt = 0;
unsigned long lastRssiAt = 0;
unsigned long wifiReadyAt = 0;
unsigned long nextSendAttemptAt = 0;

bool imuReady = false;
bool lastWifiConnected = false;
bool sendPaused = false;
bool udpReady = false;
bool serverAutoDiscovered = false;
String lastSendStatus = "UDP: idle";
String logBuffer[LOG_LINES];
IPAddress activeServerIp = SERVER_IP;

float accelOffsetX = 0.0f;
float accelOffsetY = 0.0f;
float accelOffsetZ = 0.0f;
float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;
int lastRssiDbm = -127;
uint32_t packetSequence = 0;

bool sendBufferedSamples();
bool ensureUdpReady();
bool discoverServer();
size_t buildUdpPayload(size_t maxSamples, uint8_t* buffer, size_t capacity);
void enqueueSample(const ImuSample& sample);
ImuSample bufferedSampleAt(size_t index);
void discardBufferedSamples(size_t count);
int16_t quantizeScaled(float value, float scale);
void writeUint16LE(uint8_t* buffer, size_t offset, uint16_t value);
void writeInt16LE(uint8_t* buffer, size_t offset, int16_t value);
void writeUint32LE(uint8_t* buffer, size_t offset, uint32_t value);
IPAddress getBroadcastAddress();
bool isLowerIpAddress(const IPAddress& lhs, const IPAddress& rhs);
bool ipAddressIsSet(const IPAddress& ip);
void discardPendingUdpPackets();

void pushLog(const char* fmt, ...) {
  char message[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(message, sizeof(message), fmt, args);
  va_end(args);

  Serial.println(message);
  for (size_t i = 0; i + 1 < LOG_LINES; ++i) {
    logBuffer[i] = logBuffer[i + 1];
  }
  logBuffer[LOG_LINES - 1] = String(message);
}

const char* dominantAxis(float x, float y, float z) {
  const float absX = fabsf(x);
  const float absY = fabsf(y);
  const float absZ = fabsf(z);

  if (absX >= absY && absX >= absZ) {
    return x >= 0.0f ? "X+" : "X-";
  }
  if (absY >= absX && absY >= absZ) {
    return y >= 0.0f ? "Y+" : "Y-";
  }
  return z >= 0.0f ? "Z+" : "Z-";
}

void enqueueSample(const ImuSample& sample) {
  if (bufferedSamples < MAX_BUFFERED_SAMPLES) {
    const size_t tail = (bufferHead + bufferedSamples) % MAX_BUFFERED_SAMPLES;
    sampleBuffer[tail] = sample;
    ++bufferedSamples;
    return;
  }

  sampleBuffer[bufferHead] = sample;
  bufferHead = (bufferHead + 1) % MAX_BUFFERED_SAMPLES;
  ++droppedSamples;
}

ImuSample bufferedSampleAt(size_t index) {
  return sampleBuffer[(bufferHead + index) % MAX_BUFFERED_SAMPLES];
}

void discardBufferedSamples(size_t count) {
  if (count >= bufferedSamples) {
    bufferHead = 0;
    bufferedSamples = 0;
    return;
  }

  bufferHead = (bufferHead + count) % MAX_BUFFERED_SAMPLES;
  bufferedSamples -= count;
}

void drawDisplay() {
  M5.Display.setRotation(0);
  M5.Display.setTextSize(1.2);
  const bool wifiConnected = WiFi.status() == WL_CONNECTED;
  const bool serverReady = wifiConnected && ipAddressIsSet(activeServerIp);
  const uint16_t backgroundColor = !wifiConnected
      ? M5.Display.color565(96, 0, 0)
      : (serverReady ? BLACK : M5.Display.color565(72, 24, 0));
  M5.Display.setTextColor(WHITE, backgroundColor);
  M5.Display.fillScreen(backgroundColor);
  M5.Display.setCursor(0, 0);

  M5.Display.printf("WiFi:%s\n", wifiConnected ? "OK" : "NG");

  if (wifiConnected) {
    M5.Display.printf("IP:%s\n", WiFi.localIP().toString().c_str());
    M5.Display.printf("RSSI:%d dBm\n", lastRssiDbm);
  } else {
    M5.Display.println("IP:--");
    M5.Display.println("RSSI:--");
  }

  if (ipAddressIsSet(activeServerIp)) {
    M5.Display.printf("SRV:%s\n", activeServerIp.toString().c_str());
  } else {
    M5.Display.println("SRV:searching");
  }
  M5.Display.printf("MODE:%s\n", serverAutoDiscovered ? "AUTO" : "FIXED");
  M5.Display.printf("IMU:%s Q:%u\n", imuReady ? "OK" : "NG", static_cast<unsigned>(bufferedSamples));
  if (droppedSamples > 0) {
    M5.Display.printf("DROP:%lu\n", droppedSamples);
  }
  M5.Display.printf("AX:%s\n", dominantAxis(lastAx, lastAy, lastAz));
  M5.Display.printf("A %.2f %.2f %.2f\n", lastAx, lastAy, lastAz);
  M5.Display.printf("G %.1f %.1f %.1f\n", lastGx, lastGy, lastGz);
  M5.Display.println(lastSendStatus);

  for (size_t i = 0; i < LOG_LINES; ++i) {
    if (logBuffer[i].length() > 0) {
      M5.Display.println(logBuffer[i]);
    }
  }
}

bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  udp.stop();
  udpReady = false;
  activeServerIp = SERVER_IP;
  serverAutoDiscovered = false;

  pushLog("WiFi connecting...");

  unsigned long startedAt = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    if (millis() - lastDisplayAt >= DISPLAY_INTERVAL_MS) {
      lastDisplayAt = millis();
      drawDisplay();
    }
    if (millis() - startedAt > WIFI_CONNECT_TIMEOUT_MS) {
      pushLog("WiFi timeout");
      lastSendStatus = "BTN: reconnect";
      return false;
    }
  }

  pushLog("WiFi connected");
  pushLog("%s", WiFi.localIP().toString().c_str());
  lastRssiDbm = WiFi.RSSI();
  lastRssiAt = millis();
  wifiReadyAt = millis();
  lastSendAt = wifiReadyAt;
  nextSendAttemptAt = wifiReadyAt + UDP_SEND_GRACE_MS;
  lastSendStatus = sendPaused ? "UDP: paused" : "UDP: standby";
  discoverServer();
  return true;
}

bool ensureUdpReady() {
  if (udpReady) {
    return true;
  }

  if (udp.begin(UDP_LOCAL_PORT) == 1) {
    udpReady = true;
    pushLog("UDP ready %u", static_cast<unsigned>(UDP_LOCAL_PORT));
    return true;
  }

  pushLog("UDP begin failed");
  lastSendStatus = "UDP init NG";
  return false;
}

bool ipAddressIsSet(const IPAddress& ip) {
  return !(ip[0] == 0 && ip[1] == 0 && ip[2] == 0 && ip[3] == 0);
}

IPAddress getBroadcastAddress() {
  IPAddress broadcast;
  const IPAddress localIp = WiFi.localIP();
  const IPAddress subnetMask = WiFi.subnetMask();
  for (size_t i = 0; i < 4; ++i) {
    broadcast[i] = static_cast<uint8_t>((localIp[i] & subnetMask[i]) | (~subnetMask[i]));
  }
  return broadcast;
}

bool isLowerIpAddress(const IPAddress& lhs, const IPAddress& rhs) {
  for (size_t i = 0; i < 4; ++i) {
    if (lhs[i] < rhs[i]) {
      return true;
    }
    if (lhs[i] > rhs[i]) {
      return false;
    }
  }
  return false;
}

void discardPendingUdpPackets() {
  while (udp.parsePacket() > 0) {
    while (udp.available() > 0) {
      udp.read();
    }
  }
}

bool discoverServer() {
  if (WiFi.status() != WL_CONNECTED || !ensureUdpReady()) {
    return false;
  }

  discardPendingUdpPackets();

  const IPAddress broadcastIp = getBroadcastAddress();
  if (udp.beginPacket(broadcastIp, SERVER_PORT) != 1) {
    pushLog("SRV discovery send NG");
    return false;
  }

  udp.write(reinterpret_cast<const uint8_t*>(SERVER_DISCOVERY_REQUEST),
            strlen(SERVER_DISCOVERY_REQUEST));
  if (udp.endPacket() != 1) {
    pushLog("SRV discovery end NG");
    return false;
  }

  pushLog("SRV discovery -> %s", broadcastIp.toString().c_str());

  IPAddress bestServerIp;
  bool found = false;
  const unsigned long startedAt = millis();
  while (millis() - startedAt < SERVER_DISCOVERY_TIMEOUT_MS) {
    const int packetSize = udp.parsePacket();
    if (packetSize <= 0) {
      delay(20);
      continue;
    }

    char response[32];
    const int bytesToRead = packetSize < static_cast<int>(sizeof(response) - 1)
        ? packetSize
        : static_cast<int>(sizeof(response) - 1);
    const int readBytes = udp.read(reinterpret_cast<uint8_t*>(response), bytesToRead);
    response[readBytes > 0 ? readBytes : 0] = '\0';

    if (strcmp(response, SERVER_DISCOVERY_RESPONSE) != 0) {
      continue;
    }

    const IPAddress candidateIp = udp.remoteIP();
    if (!found || isLowerIpAddress(candidateIp, bestServerIp)) {
      bestServerIp = candidateIp;
      found = true;
    }
  }

  if (found) {
    activeServerIp = bestServerIp;
    serverAutoDiscovered = true;
    lastSendStatus = "SRV:auto " + activeServerIp.toString();
    pushLog("SRV auto %s", activeServerIp.toString().c_str());
    return true;
  }

  if (ipAddressIsSet(SERVER_IP)) {
    activeServerIp = SERVER_IP;
    serverAutoDiscovered = false;
    lastSendStatus = "SRV:fixed " + activeServerIp.toString();
    pushLog("SRV fallback %s", activeServerIp.toString().c_str());
  } else {
    activeServerIp = IPAddress(0, 0, 0, 0);
    serverAutoDiscovered = false;
    lastSendStatus = "SRV: not found";
    pushLog("SRV not found");
  }
  return false;
}

int16_t quantizeScaled(float value, float scale) {
  const float scaled = roundf(value * scale);
  if (scaled > 32767.0f) {
    return 32767;
  }
  if (scaled < -32768.0f) {
    return -32768;
  }
  return static_cast<int16_t>(scaled);
}

void writeUint16LE(uint8_t* buffer, size_t offset, uint16_t value) {
  buffer[offset] = static_cast<uint8_t>(value & 0xFF);
  buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void writeInt16LE(uint8_t* buffer, size_t offset, int16_t value) {
  writeUint16LE(buffer, offset, static_cast<uint16_t>(value));
}

void writeUint32LE(uint8_t* buffer, size_t offset, uint32_t value) {
  buffer[offset] = static_cast<uint8_t>(value & 0xFF);
  buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  buffer[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  buffer[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

size_t buildUdpPayload(size_t maxSamples, uint8_t* buffer, size_t capacity) {
  if (bufferedSamples == 0 || capacity < UDP_HEADER_BYTES + UDP_SAMPLE_BYTES) {
    return 0;
  }

  const size_t sendCount = bufferedSamples < maxSamples ? bufferedSamples : maxSamples;
  const ImuSample baseSample = bufferedSampleAt(0);
  const size_t payloadBytes = UDP_HEADER_BYTES + sendCount * UDP_SAMPLE_BYTES;
  if (payloadBytes > capacity) {
    return 0;
  }

  writeUint16LE(buffer, 0, UDP_PACKET_MAGIC);
  buffer[2] = UDP_PACKET_VERSION;
  buffer[3] = 0;
  writeUint32LE(buffer, 4, packetSequence);
  writeUint32LE(buffer, 8, baseSample.timestamp_ms);
  writeInt16LE(buffer, 12, static_cast<int16_t>(lastRssiDbm));
  buffer[14] = static_cast<uint8_t>(sendCount);
  buffer[15] = 0;

  for (size_t i = 0; i < sendCount; ++i) {
    const ImuSample sampleData = bufferedSampleAt(i);
    const size_t offset = UDP_HEADER_BYTES + i * UDP_SAMPLE_BYTES;
    uint32_t dtMs = sampleData.timestamp_ms - baseSample.timestamp_ms;
    if (dtMs > 65535UL) {
      dtMs = 65535UL;
    }

    writeUint16LE(buffer, offset, static_cast<uint16_t>(dtMs));
    writeInt16LE(buffer, offset + 2, quantizeScaled(sampleData.ax, ACCEL_SCALE));
    writeInt16LE(buffer, offset + 4, quantizeScaled(sampleData.ay, ACCEL_SCALE));
    writeInt16LE(buffer, offset + 6, quantizeScaled(sampleData.az, ACCEL_SCALE));
    writeInt16LE(buffer, offset + 8, quantizeScaled(sampleData.gx, GYRO_SCALE));
    writeInt16LE(buffer, offset + 10, quantizeScaled(sampleData.gy, GYRO_SCALE));
    writeInt16LE(buffer, offset + 12, quantizeScaled(sampleData.gz, GYRO_SCALE));
  }

  return payloadBytes;
}

void calibrateImu() {
  float sumAx = 0.0f;
  float sumAy = 0.0f;
  float sumAz = 0.0f;
  float sumGx = 0.0f;
  float sumGy = 0.0f;
  float sumGz = 0.0f;

  for (size_t i = 0; i < CALIBRATION_SAMPLES; ++i) {
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
    M5.Imu.getAccel(&ax, &ay, &az);
    M5.Imu.getGyro(&gx, &gy, &gz);
    sumAx += ax;
    sumAy += ay;
    sumAz += az;
    sumGx += gx;
    sumGy += gy;
    sumGz += gz;
    delay(4);
  }

  rawAx = sumAx / CALIBRATION_SAMPLES;
  rawAy = sumAy / CALIBRATION_SAMPLES;
  rawAz = sumAz / CALIBRATION_SAMPLES;
  rawGx = sumGx / CALIBRATION_SAMPLES;
  rawGy = sumGy / CALIBRATION_SAMPLES;
  rawGz = sumGz / CALIBRATION_SAMPLES;

  const float gravityReference = rawAz >= 0.0f ? 1.0f : -1.0f;
  accelOffsetX = rawAx;
  accelOffsetY = rawAy;
  accelOffsetZ = rawAz - gravityReference;
  gyroOffsetX = rawGx;
  gyroOffsetY = rawGy;
  gyroOffsetZ = rawGz;
  lastSendStatus = "CAL: reset";
  pushLog("Calibration reset");
}

void handleButtonPress() {
  const bool wifiConnected = WiFi.status() == WL_CONNECTED;
  if (wifiConnected && !sendPaused) {
    calibrateImu();
    return;
  }

  pushLog("Reconnect requested");
  WiFi.disconnect();
  sendPaused = false;

  if (connectWiFi()) {
    pushLog("Reconnect OK");
  } else {
    pushLog("Reconnect failed");
  }
}

void pollButton() {
  const unsigned long now = millis();
  if (now - lastButtonAt < BUTTON_DEBOUNCE_MS) {
    return;
  }

  if (digitalRead(BUTTON_PIN) == LOW) {
    lastButtonAt = now;
    handleButtonPress();
  }
}

bool sendBufferedSamples() {
  if (bufferedSamples == 0) {
    return true;
  }

  if (WiFi.status() != WL_CONNECTED || sendPaused || !ipAddressIsSet(activeServerIp) || !ensureUdpReady()) {
    return false;
  }

  uint8_t packet[UDP_HEADER_BYTES + MAX_UDP_SAMPLES * UDP_SAMPLE_BYTES];
  const size_t packetBytes = buildUdpPayload(MAX_UDP_SAMPLES, packet, sizeof(packet));
  if (packetBytes == 0) {
    sendPaused = true;
    lastSendStatus = "UDP pack NG";
    pushLog("UDP packet build failed");
    lastSendAt = millis();
    return false;
  }
  const size_t sendCount = (packetBytes - UDP_HEADER_BYTES) / UDP_SAMPLE_BYTES;

  if (udp.beginPacket(activeServerIp, SERVER_PORT) != 1) {
    lastSendStatus = "UDP retry";
    pushLog("UDP packet start failed");
    lastSendAt = millis();
    nextSendAttemptAt = lastSendAt + UDP_RETRY_BACKOFF_MS;
    return false;
  }

  const size_t written = udp.write(packet, packetBytes);
  const bool packetSent = written == packetBytes && udp.endPacket() == 1;
  if (packetSent) {
    lastSendStatus = "UDP:ok x" + String(sendCount) + " " + String(packetBytes) + "B";
    discardBufferedSamples(sendCount);
    ++packetSequence;
    lastSendAt = millis();
    return true;
  }

  lastSendStatus = "UDP retry";
  pushLog("UDP send failed");
  lastSendAt = millis();
  nextSendAttemptAt = lastSendAt + UDP_RETRY_BACKOFF_MS;
  return false;
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  Serial.begin(115200);
  delay(1000);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  M5.Display.setBrightness(40);
  drawDisplay();
  pushLog("ATOM-S3 IMU start");

  connectWiFi();
  lastWifiConnected = WiFi.status() == WL_CONNECTED;

  imuReady = M5.Imu.begin();
  if (!imuReady) {
    pushLog("IMU init failed");
    lastSendStatus = "IMU init NG";
    drawDisplay();
    while (true) {
      delay(1000);
      drawDisplay();
    }
  }

  pushLog("IMU init OK");
  if (!lastWifiConnected) {
    lastSendStatus = "BTN: reconnect";
  } else if (!ipAddressIsSet(activeServerIp)) {
    lastSendStatus = "SRV: search NG";
  } else if (sendPaused) {
    lastSendStatus = "UDP: paused";
  } else {
    lastSendStatus = "UDP: standby";
  }
  drawDisplay();
}

void loop() {
  M5.update();

  const bool wifiConnected = WiFi.status() == WL_CONNECTED;
  if (wifiConnected != lastWifiConnected) {
    pushLog(wifiConnected ? "WiFi restored" : "WiFi lost");
    if (!wifiConnected) {
      udp.stop();
      udpReady = false;
      wifiReadyAt = 0;
      nextSendAttemptAt = 0;
      lastSendStatus = "BTN: reconnect";
      pushLog("Press G41 button");
    }
    lastWifiConnected = wifiConnected;
  }

  pollButton();

  const unsigned long now = millis();
  if (wifiConnected && now - lastRssiAt >= RSSI_UPDATE_INTERVAL_MS) {
    lastRssiAt = now;
    lastRssiDbm = WiFi.RSSI();
  }

  if (imuReady && now - lastSampleAt >= SAMPLE_INTERVAL_MS) {
    lastSampleAt = now;

    M5.Imu.getAccel(&rawAx, &rawAy, &rawAz);
    M5.Imu.getGyro(&rawGx, &rawGy, &rawGz);

    const float adjustedAx = rawAx - accelOffsetX;
    const float adjustedAy = rawAy - accelOffsetY;
    const float adjustedAz = rawAz - accelOffsetZ;
    const float adjustedGx = rawGx - gyroOffsetX;
    const float adjustedGy = rawGy - gyroOffsetY;
    const float adjustedGz = rawGz - gyroOffsetZ;

    lastAx = adjustedAx;
    lastAy = adjustedAy;
    lastAz = adjustedAz;
    lastGx = adjustedGx;
    lastGy = adjustedGy;
    lastGz = adjustedGz;

    enqueueSample({
      now,
      adjustedAx,
      adjustedAy,
      adjustedAz,
      adjustedGx,
      adjustedGy,
      adjustedGz
    });
  }

  if (bufferedSamples > 0 &&
      now - wifiReadyAt >= UDP_SEND_GRACE_MS &&
      now >= nextSendAttemptAt &&
      (bufferedSamples >= MAX_UDP_SAMPLES || now - lastSendAt >= SEND_INTERVAL_MS)) {
    sendBufferedSamples();
  }

  if (now - lastDisplayAt >= DISPLAY_INTERVAL_MS) {
    lastDisplayAt = now;
    drawDisplay();
  }
}
