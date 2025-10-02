/*
* OTA upgrade example vya CAN bus usando MCP2515
* compilation date: 20/03/25
*/

#include "NeoPixelBus.h"
#include <Adafruit_MCP2515.h>
#include <Update.h>

#if defined(ARDUINO_ARCH_RP2040)
#include <hardware/watchdog.h>
#endif

 //define MCP2515 CAN 
#define CS_PIN    5
#define MISO_PIN  3
#define MOSI_PIN  4 
#define SCK_PIN   2
#define CAN_BAUDRATE (500000)

// CAN identifiers dedicated to OTA traffic
constexpr uint16_t OTA_COMMAND_CAN_ID = 0x381;   // Frames sent by host with OTA commands/data
constexpr uint16_t OTA_STATUS_CAN_ID  = 0x382;   // Status/acknowledgement frames sent back to host


#define colorSaturation 128

const uint16_t PixelCount = 1;
const uint8_t PixelPin = 16;

NeoPixelBus<NeoGrbFeature, NeoWs2812xMethod> strip(PixelCount, PixelPin);

RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor white(colorSaturation);
RgbColor black(0);
RgbColor yellow(colorSaturation,colorSaturation,0);


static bool ledOn = false;

//Instanciacion de MCP
Adafruit_MCP2515 mcp(CS_PIN,MISO_PIN,MOSI_PIN,SCK_PIN);

// OTA command opcodes
enum : uint8_t {
  OTA_CMD_BEGIN = 0x01,
  OTA_CMD_DATA  = 0x02,
  OTA_CMD_END   = 0x03,
  OTA_CMD_ABORT = 0x04
};

// OTA status codes
enum : uint8_t {
  OTA_STATUS_IDLE        = 0x00,
  OTA_STATUS_BEGIN_OK    = 0x10,
  OTA_STATUS_BEGIN_ERROR = 0x90,
  OTA_STATUS_DATA_OK     = 0x11,
  OTA_STATUS_DATA_ERROR  = 0x91,
  OTA_STATUS_COMPLETE    = 0x12,
  OTA_STATUS_VERIFY_FAIL = 0x92,
  OTA_STATUS_ABORTED     = 0x93
};

struct OtaSession {
  bool active = false;
  bool pendingReboot = false;
  uint32_t expectedSize = 0;
  uint32_t receivedSize = 0;
  uint16_t expectedCrc = 0;
  uint16_t runningCrc = 0xFFFF;
};

static OtaSession otaSession;

static uint16_t crc16_ccitt_update(uint16_t crc, uint8_t data) {
  crc ^= static_cast<uint16_t>(data) << 8;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x8000) {
      crc = (crc << 1) ^ 0x1021;
    } else {
      crc <<= 1;
    }
  }
  return crc;
}

static void resetCrc() {
  otaSession.runningCrc = 0xFFFF;
}

static void updateCrc(const uint8_t *data, size_t length) {
  for (size_t i = 0; i < length; i++) {
    otaSession.runningCrc = crc16_ccitt_update(otaSession.runningCrc, data[i]);
  }
}

static void sendOtaStatus(uint8_t status, uint8_t detail = 0x00) {
  mcp.beginPacket(OTA_STATUS_CAN_ID);
  mcp.write(status);
  mcp.write(detail);
  mcp.write(static_cast<uint8_t>((otaSession.receivedSize >> 0) & 0xFF));
  mcp.write(static_cast<uint8_t>((otaSession.receivedSize >> 8) & 0xFF));
  mcp.write(static_cast<uint8_t>((otaSession.receivedSize >> 16) & 0xFF));
  mcp.write(static_cast<uint8_t>((otaSession.receivedSize >> 24) & 0xFF));
  mcp.write(static_cast<uint8_t>((otaSession.expectedSize >> 0) & 0xFF));
  mcp.write(static_cast<uint8_t>((otaSession.expectedSize >> 8) & 0xFF));
  mcp.endPacket();
}

static void abortOta(uint8_t statusDetail) {
  if (otaSession.active) {
    Update.abort();
  }
  otaSession = OtaSession{};
  sendOtaStatus(OTA_STATUS_ABORTED, statusDetail);
}

static void requestReboot() {
  otaSession.pendingReboot = true;
}

static void handleOtaBegin(const uint8_t *payload, size_t length) {
  if (length < 7) {
    sendOtaStatus(OTA_STATUS_BEGIN_ERROR, 0x01);
    return;
  }

  if (otaSession.active) {
    Update.abort();
    otaSession = OtaSession{};
  }

  uint32_t imageSize = static_cast<uint32_t>(payload[0]) |
                       (static_cast<uint32_t>(payload[1]) << 8) |
                       (static_cast<uint32_t>(payload[2]) << 16) |
                       (static_cast<uint32_t>(payload[3]) << 24);
  uint16_t imageCrc = static_cast<uint16_t>(payload[4]) |
                      (static_cast<uint16_t>(payload[5]) << 8);

  if (imageSize == 0 || imageSize > 1024UL * 512UL) { // Basic sanity limit (512 KB)
    sendOtaStatus(OTA_STATUS_BEGIN_ERROR, 0x02);
    return;
  }

  if (!Update.begin(imageSize)) {
    sendOtaStatus(OTA_STATUS_BEGIN_ERROR, 0x03);
    return;
  }

  otaSession.active = true;
  otaSession.expectedSize = imageSize;
  otaSession.expectedCrc = imageCrc;
  otaSession.receivedSize = 0;
  resetCrc();
  sendOtaStatus(OTA_STATUS_BEGIN_OK);
}

static void handleOtaData(const uint8_t *payload, size_t length) {
  if (!otaSession.active) {
    sendOtaStatus(OTA_STATUS_DATA_ERROR, 0x10);
    return;
  }

  if (length < 1) {
    sendOtaStatus(OTA_STATUS_DATA_ERROR, 0x11);
    return;
  }

  uint8_t chunkLength = payload[0];
  if (chunkLength == 0 || chunkLength > 6 || length < (1 + chunkLength)) {
    sendOtaStatus(OTA_STATUS_DATA_ERROR, 0x12);
    return;
  }

  const uint8_t *chunk = payload + 1;

  if (otaSession.receivedSize + chunkLength > otaSession.expectedSize) {
    abortOta(0x13);
    return;
  }

  size_t written = Update.write(chunk, chunkLength);
  if (written != chunkLength) {
    abortOta(0x14);
    return;
  }

  otaSession.receivedSize += chunkLength;
  updateCrc(chunk, chunkLength);
  sendOtaStatus(OTA_STATUS_DATA_OK);
}

static void handleOtaEnd() {
  if (!otaSession.active) {
    sendOtaStatus(OTA_STATUS_BEGIN_ERROR, 0x20);
    return;
  }

  if (otaSession.receivedSize != otaSession.expectedSize) {
    abortOta(0x21);
    return;
  }

  if (otaSession.runningCrc != otaSession.expectedCrc) {
    Update.abort();
    sendOtaStatus(OTA_STATUS_VERIFY_FAIL, 0x22);
    otaSession = OtaSession{};
    return;
  }

  if (!Update.end(true)) {
    abortOta(0x23);
    return;
  }

  sendOtaStatus(OTA_STATUS_COMPLETE);
  otaSession.active = false;
  requestReboot();
}

/*
 * OTA command frame layout (payload after opcode):
 *  - 0x01 BEGIN: size[4] CRC16[2]
 *  - 0x02 DATA: chunkLength[1] chunkBytes[<=6]
 *  - 0x03 END : (no extra data)
 *  - 0x04 ABORT: (no extra data)
 */
static void processOtaFrame(uint16_t canId, const uint8_t *payload, size_t length) {
  if (canId != OTA_COMMAND_CAN_ID || length == 0) {
    return;
  }

  uint8_t opcode = payload[0];
  const uint8_t *data = payload + 1;
  size_t dataLength = (length > 0) ? length - 1 : 0;

  switch (opcode) {
    case OTA_CMD_BEGIN:
      handleOtaBegin(data, dataLength);
      break;
    case OTA_CMD_DATA:
      handleOtaData(data, dataLength);
      break;
    case OTA_CMD_END:
      handleOtaEnd();
      break;
    case OTA_CMD_ABORT:
      abortOta(0x30);
      break;
    default:
      sendOtaStatus(OTA_STATUS_BEGIN_ERROR, 0xFF);
      break;
  }
}

static void pollCanBus() {
  int packetSize = mcp.parsePacket();
  while (packetSize > 0) {
    uint16_t packetId = mcp.packetId();
    uint8_t buffer[8];
    size_t index = 0;

    while (mcp.available() && index < sizeof(buffer)) {
      buffer[index++] = mcp.read();
    }

    processOtaFrame(packetId, buffer, index);
    packetSize = mcp.parsePacket();
  }
}

void setup() {
  Serial.begin(115200);

  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error MCP2515 init.");
    while(1) delay(10);
  }

  Serial.println("MCP2515 started.");


  strip.Begin();
  strip.SetPixelColor(0, black);
  strip.Show();

  sendOtaStatus(OTA_STATUS_IDLE);
}

void loop() {
  pollCanBus();

  if (otaSession.pendingReboot) {
    Serial.println("Rebooting to apply OTA firmware...");
#if defined(ARDUINO_ARCH_RP2040)
    watchdog_reboot(0, 0, 0);
#else
    Serial.println("Manual restart required. Please disconnect and reconnect the power.");
    otaSession.pendingReboot = false;
#endif
  }

  static uint32_t lastBlink = 0;
  const uint32_t blinkIntervalMs = 500;
  uint32_t now = millis();
  if (now - lastBlink >= blinkIntervalMs) {
    lastBlink = now;
    ledOn = !ledOn;
    strip.SetPixelColor(0, ledOn ? white : black);
    strip.Show();
  }

}
