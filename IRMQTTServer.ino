
//#include <arduino_homekit_server.h>
#include "IRMQTTServer.h"
#include <Arduino.h>
#include <FS.h>
#include <ArduinoJson.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#endif  // ESP8266
#if defined(ESP32)
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <Update.h>
#endif  // ESP32
#include <WiFiClient.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRtext.h>
#include <IRtimer.h>
#include <IRutils.h>
#include <IRac.h>

#include <algorithm>  // NOLINT(build/include)
#include <memory>
#include <string>

using irutils::msToString;

#if REPORT_VCC
  ADC_MODE(ADC_VCC);
#endif  // REPORT_VCC

// Globals
#if defined(ESP8266)
ESP8266WebServer server(kHttpPort);
#endif  // ESP8266
#if defined(ESP32)
WebServer server(kHttpPort);
#endif  // ESP32
#if MDNS_ENABLE
MDNSResponder mdns;
#endif  // MDNS_ENABLE
WiFiClient espClient;
WiFiManager wifiManager;
bool flagSaveWifiConfig = false;
char HttpUsername[kUsernameLength + 1] = "admin";  // Default HTT username.
char HttpPassword[kPasswordLength + 1] = "";  // No HTTP password by default.
char Hostname[kHostnameLength + 1] = "ir_server";  // Default hostname.
uint16_t *codeArray;
uint32_t lastReconnectAttempt = 0;  // MQTT last attempt reconnection number
bool boot = true;
volatile bool lockIr = false;  // Primitive locking for gating the IR LED.
uint32_t sendReqCounter = 0;
bool lastSendSucceeded = false;  // Store the success status of the last send.
uint32_t lastSendTime = 0;
int8_t offset;  // The calculated period offset for this chip and library.
IRsend *IrSendTable[kNrOfIrTxGpios];
int8_t txGpioTable[kNrOfIrTxGpios] = {kDefaultIrLed};
String lastClimateSource;
#if IR_RX
IRrecv *irrecv = NULL;
decode_results capture;  // Somewhere to store inbound IR messages.
int8_t rx_gpio = kDefaultIrRx;
String lastIrReceived = "None";
uint32_t lastIrReceivedTime = 0;
uint32_t irRecvCounter = 0;
#endif  // IR_RX

// Climate stuff
IRac *climate[kNrOfIrTxGpios];
String channel_re = "(";  // Will be built later.
uint16_t chan = 0;  // The channel to use for the aircon HTML page.

TimerMs lastClimateIr = TimerMs();  // When we last sent the IR Climate mesg.
uint32_t irClimateCounter = 0;  // How many have we sent?
// Store the success status of the last climate send.
bool lastClimateSucceeded = false;
bool hasClimateBeenSent = false;  // Has the Climate ever been sent?



bool isSerialGpioUsedByIr(void) {
  const int8_t kSerialTxGpio = 1;  // The GPIO serial output is sent to.
                                   // Note: *DOES NOT* control Serial output.
#if defined(ESP32)
  const int8_t kSerialRxGpio = 3;  // The GPIO serial input is received on.
#endif  // ESP32
  // Ensure we are not trodding on anything IR related.
#if IR_RX
  switch (rx_gpio) {
#if defined(ESP32)
    case kSerialRxGpio:
#endif  // ESP32
    case kSerialTxGpio:
      return true;  // Serial port is in use by IR capture. Abort.
  }
#endif  // IR_RX
  for (uint16_t i = 0; i < kNrOfIrTxGpios; i++)
    switch (txGpioTable[i]) {
#if defined(ESP32)
      case kSerialRxGpio:
#endif  // ESP32
      case kSerialTxGpio:
        return true;  // Serial port is in use for IR sending. Abort.
    }
  return false;  // Not in use as far as we can tell.
}

// Debug messages get sent to the serial port.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void debug(const char *str) {
#if DEBUG
  if (isSerialGpioUsedByIr()) return;  // Abort.
  uint32_t now = millis();
  Serial.printf("%07u.%03u: %s\n", now / 1000, now % 1000, str);
#endif  // DEBUG
}
#pragma GCC diagnostic pop

// callback notifying us of the need to save the wifi config
void saveWifiConfigCallback(void) {
  debug("saveWifiConfigCallback called.");
  flagSaveWifiConfig = true;
}

// Forcibly mount the SPIFFS. Formatting the SPIFFS if needed.
//
// Returns:
//   A boolean indicating success or failure.
bool mountSpiffs(void) {
  debug("Mounting SPIFFS...");
  if (SPIFFS.begin()) return true;  // We mounted it okay.
  // We failed the first time.
  debug("Failed to mount SPIFFS!\nFormatting SPIFFS and trying again...");
  SPIFFS.format();
  if (!SPIFFS.begin()) {  // Did we fail?
    debug("DANGER: Failed to mount SPIFFS even after formatting!");
    delay(10000);  // Make sure the debug message doesn't just float by.
    return false;
  }
  return true;  // Success!
}

bool saveConfig(void) {
  debug("Saving the config.");
  bool success = false;
  DynamicJsonDocument json(kJsonConfigMaxSize);

  json[kHostnameKey] = Hostname;
  json[kHttpUserKey] = HttpUsername;
  json[kHttpPassKey] = HttpPassword;
#if IR_RX
  json[KEY_RX_GPIO] = static_cast<int>(rx_gpio);
#endif  // IR_RX
  for (uint16_t i = 0; i < kNrOfIrTxGpios; i++) {
    const String key = KEY_TX_GPIO + String(i);
    json[key] = static_cast<int>(txGpioTable[i]);
  }

  if (mountSpiffs()) {
    File configFile = SPIFFS.open(kConfigFile, "w");
    if (!configFile) {
      debug("Failed to open config file for writing.");
    } else {
      debug("Writing out the config file.");
      serializeJson(json, configFile);
      configFile.close();
      debug("Finished writing config file.");
      success = true;
    }
    SPIFFS.end();
  }
  return success;
}

bool loadConfigFile(void) {
  bool success = false;
  if (mountSpiffs()) {
    debug("mounted the file system");
    if (SPIFFS.exists(kConfigFile)) {
      debug("config file exists");

      File configFile = SPIFFS.open(kConfigFile, "r");
      if (configFile) {
        debug("Opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument json(kJsonConfigMaxSize);
        if (!deserializeJson(json, buf.get(), kJsonConfigMaxSize)) {
          debug("Json config file parsed ok.");

          strncpy(Hostname, json[kHostnameKey] | "", kHostnameLength);
          strncpy(HttpUsername, json[kHttpUserKey] | "", kUsernameLength);
          strncpy(HttpPassword, json[kHttpPassKey] | "", kPasswordLength);
          // Read in the GPIO settings.
#if IR_RX
          // Single RX gpio
          rx_gpio = json[KEY_RX_GPIO] | kDefaultIrRx;
#endif  // IR_RX
          // Potentially multiple TX gpios
          for (uint16_t i = 0; i < kNrOfIrTxGpios; i++)
            txGpioTable[i] = json[String(KEY_TX_GPIO + String(i)).c_str()] |
                           kDefaultIrLed;
          debug("Recovered Json fields.");
          success = true;
        } else {
          debug("Failed to load json config");
        }
        debug("Closing the config file.");
        configFile.close();
      }
    } else {
      debug("Config file doesn't exist!");
    }
    debug("Unmounting SPIFFS.");
    SPIFFS.end();
  }
  return success;
}

String timeElapsed(uint32_t const msec) {
  String result = msToString(msec);
  if (result.equalsIgnoreCase(D_STR_NOW))
    return result;
  else
    return result + F(" ago");
}

String timeSince(uint32_t const start) {
  if (start == 0)
    return F("Never");
  uint32_t diff = 0;
  uint32_t now = millis();
  if (start < now)
    diff = now - start;
  else
    diff = UINT32_MAX - start + now;
  return msToString(diff) + F(" ago");
}

String gpioToString(const int16_t gpio) {
  if (gpio == kGpioUnused)
    return F(D_STR_UNUSED);
  else
    return String(gpio);
}

int8_t getDefaultTxGpio(void) {
  for (int16_t i = 0; i < kNrOfIrTxGpios; i++)
    if (txGpioTable[i] != kGpioUnused) return txGpioTable[i];
  return kGpioUnused;
}

// Return a string containing the comma separated list of sending gpios.
String listOfTxGpios(void) {
  bool found = false;
  String result = "";
  for (uint16_t i = 0; i < kNrOfIrTxGpios; i++) {
    if (i) result += ", ";
    result += gpioToString(txGpioTable[i]);
    if (!found && txGpioTable[i] == getDefaultTxGpio()) {
      result += " (default)";
      found = true;
    }
  }
  return result;
}

String htmlMenu(void) {
  String html = F("<center>");
  html += htmlButton(kUrlRoot, F("主页"));
  html += htmlButton(kUrlAircon, F("空调控制"));
#if EXAMPLES_ENABLE
  html += htmlButton(kUrlExamples, F("示例"));
#endif  // EXAMPLES_ENABLE
  html += htmlButton(kUrlInfo, F("系统日志"));
  html += htmlButton(kUrlAdmin, F("管理员"));
  html += F("</center><hr>");
  return html;
}

String htmlSelectAcStateProtocol(const String name, const decode_type_t def,
                                 const bool simple) {
  String html = "<select name='" + name + "'>";
  for (uint8_t i = 1; i <= decode_type_t::kLastDecodeType; i++) {
    if (simple ^ hasACState((decode_type_t)i)) {
      switch (i) {
        case decode_type_t::RAW:
        case decode_type_t::PRONTO:
        case decode_type_t::GLOBALCACHE:
          break;
        default:
          html += htmlOptionItem(String(i), typeToString((decode_type_t)i),
                                i == def);
      }
    }
  }
  html += F("</select>");
  return html;
}

// Root web page with example usage etc.
void handleRoot(void) {
#if HTML_PASSWORD_ENABLE
  if (!server.authenticate(HttpUsername, HttpPassword)) {
    debug("Basic HTTP authentication failure for /.");
    return server.requestAuthentication();
  }
#endif
  String html = htmlHeader(F("ESP IR MQTT&&Homekit Server"));
  html += F("<center><small><i>" _MY_VERSION_ "</i></small></center>");
  html += htmlMenu();
  html += F(
    "<h3>发送一条简单的红外信息</h3><p>"
    "<form method='POST' action='/ir' enctype='multipart/form-data'>"
      D_STR_PROTOCOL ": ");
  html += htmlSelectAcStateProtocol(KEY_TYPE, decode_type_t::NEC, true);
  html += F(
      " " D_STR_CODE ": 0x<input type='text' name='" KEY_CODE "' min='0' "
        "value='0' size='16' maxlength='16'> "
      D_STR_BITS ": "
      "<select name='" KEY_BITS "'>"
        "<option selected='selected' value='0'>Default</option>");  // Default
  for (uint8_t i = 0; i < sizeof(kCommonBitSizes); i++) {
    String num = String(kCommonBitSizes[i]);
    html += F("<option value='");
    html += num;
    html += F("'>");
    html += num;
    html += F("</option>");
  }
  html += F(
      "</select>"
      " " D_STR_REPEAT ": <input type='number' name='" KEY_REPEAT "' min='0' "
        "max='99' value='0' size='2' maxlength='2'>"
      " <input type='submit' value='Send " D_STR_CODE "'>"
    "</form>"
    "<br><hr>"
    "<h3>Send a complex (Air Conditioner) IR message</h3><p>"
    "<form method='POST' action='/ir' enctype='multipart/form-data'>"
      D_STR_PROTOCOL ": ");
  html += htmlSelectAcStateProtocol(KEY_TYPE, decode_type_t::KELVINATOR, false);
  html += F(
      " State " D_STR_CODE ": 0x"
      "<input type='text' name='" KEY_CODE "' size='");
  html += String(kStateSizeMax * 2);
  html += F("' maxlength='");
  html += String(kStateSizeMax * 2);
  html += F("'"
          " value='"
#if EXAMPLES_ENABLE
                "190B8050000000E0190B8070000010F0"
#endif   // EXAMPLES_ENABLE
                "'>"
      " <input type='submit' value='Send A/C " D_STR_CODE "'>"
    "</form>"
    "<br><hr>"
    "<h3>Send an IRremote Raw IR message</h3><p>"
    "<form method='POST' action='/ir' enctype='multipart/form-data'>"
      "<input type='hidden' name='" KEY_TYPE "' value='30'>"
      "String: (freq,array data) <input type='text' name='" KEY_CODE "'"
      " size='132' value='"
#if EXAMPLES_ENABLE
          "38000,4420,4420,520,1638,520,1638,520,1638,520,520,520,520,520,"
          "520,520,520,520,520,520,1638,520,1638,520,1638,520,520,520,"
          "520,520,520,520,520,520,520,520,520,520,1638,520,520,520,520,520,"
          "520,520,520,520,520,520,520,520,1638,520,520,520,1638,520,1638,520,"
          "1638,520,1638,520,1638,520,1638,520"
#endif   // EXAMPLES_ENABLE
          "'>"
      " <input type='submit' value='Send Raw'>"
    "</form>"
    "<br><hr>"
    "<h3>Send a <a href='https://irdb.globalcache.com/'>GlobalCache</a>"
        " IR message</h3><p>"
    "<form method='POST' action='/ir' enctype='multipart/form-data'>"
      "<input type='hidden' name='" KEY_TYPE "' value='31'>"
      "String: 1:1,1,<input type='text' name='" KEY_CODE "' size='132'"
      " value='"
#if EXAMPLES_ENABLE
          "38000,1,1,170,170,20,63,20,63,20,63,20,20,20,20,20,20,20,20,20,"
          "20,20,63,20,63,20,63,20,20,20,20,20,20,20,20,20,20,20,20,20,63,20,"
          "20,20,20,20,20,20,20,20,20,20,20,20,63,20,20,20,63,20,63,20,63,20,"
          "63,20,63,20,63,20,1798"
#endif   // EXAMPLES_ENABLE
          "'>"
      " <input type='submit' value='Send GlobalCache'>"
    "</form>"
    "<br><hr>"
    "<h3>Send a <a href='http://www.remotecentral.com/cgi-bin/files/rcfiles.cgi"
      "?area=pronto&db=discrete'>Pronto code</a> IR message</h3><p>"
    "<form method='POST' action='/ir' enctype='multipart/form-data'>"
      "<input type='hidden' name='" KEY_TYPE "' value='25'>"
      "String (comma separated): <input type='text' name='" KEY_CODE "'"
      " size='132' value='"
#if EXAMPLES_ENABLE
          "0000,0067,0000,0015,0060,0018,0018,0018,0030,0018,0030,0018,"
          "0030,0018,0018,0018,0030,0018,0018,0018,0018,0018,0030,0018,0018,"
          "0018,0030,0018,0030,0018,0030,0018,0018,0018,0018,0018,0030,0018,"
          "0018,0018,0018,0018,0030,0018,0018,03f6"
#endif   // EXAMPLES_ENABLE
          "'>"
      " " D_STR_REPEAT ": <input type='number' name='" KEY_REPEAT "' min='0' "
          "max='99' value='0' size='2' maxlength='2'>"
      " <input type='submit' value='Send Pronto'>"
    "</form>"
    "<br>");
  html += htmlEnd();
  server.send(200, "text/html", html);
}

String addJsReloadUrl(const String url, const uint16_t timeout_s,
                      const bool notify) {
  String html = F(
      "<script type=\"text/javascript\">\n"
      "<!--\n"
      "  function Redirect() {\n"
      "    window.location=\"");
  html += url;
  html += F("\";\n"
      "  }\n"
      "\n");
  if (notify && timeout_s) {
    html += F("  document.write(\"You will be redirected to the main page in ");
    html += String(timeout_s);
    html += F(" " D_STR_SECONDS ".\");\n");
  }
  html += F("  setTimeout('Redirect()', ");
  html += String(timeout_s * 1000);  // Convert to mSecs
  html += F(");\n"
      "//-->\n"
      "</script>\n");
  return html;
}

#if EXAMPLES_ENABLE
// Web page with hardcoded example usage etc.
void handleExamples(void) {
#if HTML_PASSWORD_ENABLE
  if (!server.authenticate(HttpUsername, HttpPassword)) {
    debug("Basic HTTP authentication failure for /examples.");
    return server.requestAuthentication();
  }
#endif
  String html = htmlHeader(F("IR MQTT examples"));
  html += htmlMenu();
  html += F(
    "<h3>Hardcoded examples</h3>"
    "<p><a href=\"ir?" KEY_CODE "=38000,1,69,341,171,21,64,21,64,21,21,21,21,"
        "21,21,21,21,21,21,21,64,21,64,21,21,21,64,21,21,21,21,21,21,21,64,21,"
        "21,21,64,21,21,21,21,21,21,21,64,21,21,21,21,21,21,21,21,21,64,21,64,"
        "21,64,21,21,21,64,21,64,21,64,21,1600,341,85,21,3647"
        "&" KEY_TYPE "=31\">Sherwood Amp " D_STR_ON " (GlobalCache)</a></p>"
    "<p><a href=\"ir?" KEY_CODE "=38000,8840,4446,546,1664,546,1664,546,546,"
        "546,546,546,546,546,546,546,546,546,1664,546,1664,546,546,546,1664,"
        "546,546,546,546,546,546,546,1664,546,546,546,1664,546,546,546,1664,"
        "546,1664,546,1664,546,546,546,546,546,546,546,546,546,1664,546,546,"
        "546,546,546,546,546,1664,546,1664,546,1664,546,41600,8840,2210,546"
        "&" KEY_TYPE "=30\">Sherwood Amp " D_STR_OFF " (Raw)</a></p>"
    "<p><a href=\"ir?" KEY_CODE "=0000,006E,0022,0002,0155,00AA,0015,0040,0015,"
        "0040,0015,0015,0015,0015,0015,0015,0015,0015,0015,0015,0015,0040,0015,"
        "0040,0015,0015,0015,0040,0015,0015,0015,0015,0015,0015,0015,0040,0015,"
        "0015,0015,0015,0015,0040,0015,0040,0015,0015,0015,0015,0015,0015,0015,"
        "0015,0015,0015,0015,0040,0015,0015,0015,0015,0015,0040,0015,0040,0015,"
        "0040,0015,0040,0015,0040,0015,0640,0155,0055,0015,0E40"
        "&" KEY_TYPE "=25&" KEY_REPEAT "=1\">"
        "Sherwood Amp Input TAPE (Pronto)</a></p>"
    "<p><a href=\"ir?" KEY_TYPE "=7&" KEY_CODE "=E0E09966\">TV " D_STR_ON
        " (Samsung)</a></p>"
    "<p><a href=\"ir?" KEY_TYPE "=4&" KEY_CODE "=0xf50&bits=12\">" D_STR_POWER
        " " D_STR_OFF " (Sony 12 " D_STR_BITS ")</a></p>"
    "<p><a href=\"aircon/set?protocol=PANASONIC_AC&"
      KEY_MODEL "=LKE&"
      KEY_POWER "=on&"
      KEY_MODE "=auto&"
      KEY_FANSPEED "=min&"
      KEY_TEMP "=23\">"
      "Panasonic A/C " D_STR_MODEL " LKE, " D_STR_ON ", " D_STR_AUTO " "
      D_STR_MODE ", " D_STR_MIN " " D_STR_FAN ", 23C"
      " <i>(via HTTP aircon interface)</i></a></p>"
    "<p><a href=\"aircon/set?" KEY_TEMP "=27\">"
      "Change just the " D_STR_TEMP " to 27C <i>"
      "(via HTTP aircon interface)</i></a></p>"
    "<p><a href=\"aircon/set?" KEY_POWER "=off&" KEY_MODE "=auto\">"
      "Turn " D_STR_OFF " the current A/C <i>("
      "via HTTP aircon interface)</i></a></p>"
    "<br><hr>");
  html += htmlEnd();
  server.send(200, "text/html", html);
}
#endif  // EXAMPLES_ENABLE

String htmlOptionItem(const String value, const String text, bool selected) {
  String html = F("<option value='");
  html += value + '\'';
  if (selected) html += F(" selected='selected'");
  html += '>' + text + F("</option>");
  return html;
}

String htmlSelectBool(const String name, const bool def) {
  String html = "<select name='" + name + "'>";
  for (uint16_t i = 0; i < 2; i++)
    html += htmlOptionItem(IRac::boolToString(i), IRac::boolToString(i),
                           i == def);
  html += F("</select>");
  return html;
}

String htmlSelectClimateProtocol(const String name, const decode_type_t def) {
  String html = "<select name='" + name + "'>";
  for (uint8_t i = 1; i <= decode_type_t::kLastDecodeType; i++) {
    if (IRac::isProtocolSupported((decode_type_t)i)) {
      html += htmlOptionItem(String(i), typeToString((decode_type_t)i),
                             i == def);
    }
  }
  html += F("</select>");
  return html;
}

String htmlSelectModel(const String name, const int16_t def) {
  String html = "<select name='" + name + "'>";
  for (int16_t i = -1; i <= 6; i++) {
    String num = String(i);
    String text;
    if (i == -1)
      text = F("Default");
    else if (i == 0)
      text = F("Unknown");
    else
      text = num;
    html += htmlOptionItem(num, text, i == def);
  }
  html += F("</select>");
  return html;
}

String htmlSelectUint(const String name, const uint16_t max,
                      const uint16_t def) {
  String html = "<select name='" + name + "'>";
  for (uint16_t i = 0; i < max; i++) {
    String num = String(i);
    html += htmlOptionItem(num, num, i == def);
  }
  html += F("</select>");
  return html;
}

String htmlSelectGpio(const String name, const int16_t def,
                      const int8_t list[], const int16_t length) {
  String html = ": <select name='" + name + "'>";
  for (int16_t i = 0; i < length; i++) {
    String num = String(list[i]);
    html += htmlOptionItem(num, list[i] == kGpioUnused ? F("Unused") : num,
                           list[i] == def);
    html += F("</option>");
  }
  html += F("</select>");
  return html;
}

String htmlSelectMode(const String name, const stdAc::opmode_t def) {
  String html = "<select name='" + name + "'>";
  for (int8_t i = -1; i <= (int8_t)stdAc::opmode_t::kLastOpmodeEnum; i++) {
    String mode = IRac::opmodeToString((stdAc::opmode_t)i);
    html += htmlOptionItem(mode, mode, (stdAc::opmode_t)i == def);
  }
  html += F("</select>");
  return html;
}

String htmlSelectFanspeed(const String name, const stdAc::fanspeed_t def) {
  String html = "<select name='" + name + "'>";
  for (int8_t i = 0; i <= (int8_t)stdAc::fanspeed_t::kLastFanspeedEnum; i++) {
    String speed = IRac::fanspeedToString((stdAc::fanspeed_t)i);
    html += htmlOptionItem(speed, speed, (stdAc::fanspeed_t)i == def);
  }
  html += F("</select>");
  return html;
}

String htmlSelectSwingv(const String name, const stdAc::swingv_t def) {
  String html = "<select name='" + name + "'>";
  for (int8_t i = -1; i <= (int8_t)stdAc::swingv_t::kLastSwingvEnum; i++) {
    String swing = IRac::swingvToString((stdAc::swingv_t)i);
    html += htmlOptionItem(swing, swing, (stdAc::swingv_t)i == def);
  }
  html += F("</select>");
  return html;
}

String htmlSelectSwingh(const String name, const stdAc::swingh_t def) {
  String html = "<select name='" + name + "'>";
  for (int8_t i = -1; i <= (int8_t)stdAc::swingh_t::kLastSwinghEnum; i++) {
    String swing = IRac::swinghToString((stdAc::swingh_t)i);
    html += htmlOptionItem(swing, swing, (stdAc::swingh_t)i == def);
  }
  html += F("</select>");
  return html;
}

String htmlHeader(const String title, const String h1_text) {
  String html = F("<html><head><title>");
  html += title;
  html += F("</title><meta http-equiv=\"Content-Type\" "
            "content=\"text/html;charset=utf-8\">"
            "</head><body><center><h1>");
  if (h1_text.length())
    html += h1_text;
  else
    html += title;
  html += F("</h1></center>");
  return html;
}

String htmlEnd(void) {
  return F("</body></html>");
}

String htmlButton(const String url, const String button, const String text) {
  String html = F("<button type='button' onclick='window.location=\"");
  html += url;
  html += F("\"'>");
  html += button;
  html += F("</button> ");
  html += text;
  return html;
}

// Admin web page
void handleAirCon(void) {
  String html = htmlHeader(F("空调控制"));
  html += htmlMenu();
  if (kNrOfIrTxGpios > 1) {
    html += "<form method='POST' action='/aircon/set'"
        " enctype='multipart/form-data'>"
        "<table>"
        "<tr><td><b>Climate #</b></td><td>" +
        htmlSelectUint(KEY_CHANNEL, kNrOfIrTxGpios, chan) +
        "<input type='submit' value='Change'>"
        "</td></tr>"
        "</table>"
        "</form>"
        "<hr>";
  }
  if (climate[chan] != NULL) {
    html += "<h3>当前设置</h3>"
        "<form method='POST' action='/aircon/set'"
        " enctype='multipart/form-data'>"
        "<input type='hidden' name='" KEY_CHANNEL "' value='" + String(chan) +
            "'>" +
        "<table style='width:33%'>"
        "<tr><td>" D_STR_PROTOCOL "</td><td>" +
            htmlSelectClimateProtocol(KEY_PROTOCOL,
                                      climate[chan]->next.protocol) +
            "</td></tr>"
        "<tr><td>" D_STR_MODEL "</td><td>" +
            htmlSelectModel(KEY_MODEL, climate[chan]->next.model) +
            "</td></tr>"
        "<tr><td>" D_STR_POWER "</td><td>" +
            htmlSelectBool(KEY_POWER, climate[chan]->next.power) +
            "</td></tr>"
        "<tr><td>" D_STR_MODE "</td><td>" +
            htmlSelectMode(KEY_MODE, climate[chan]->next.mode) +
            "</td></tr>"
        "<tr><td>" D_STR_TEMP "</td><td>"
            "<input type='number' name='" KEY_TEMP "' min='16' max='90' "
            "step='0.5' value='" + String(climate[chan]->next.degrees, 1) + "'>"
            "<select name='" KEY_CELSIUS "'>"
                "<option value='on'" +
                (climate[chan]->next.celsius ? " selected='selected'" : "") +
                ">C</option>"
                "<option value='off'" +
                (!climate[chan]->next.celsius ? " selected='selected'" : "") +
                ">F</option>"
            "</select></td></tr>"
        "<tr><td>" D_STR_FAN "</td><td>" +
            htmlSelectFanspeed(KEY_FANSPEED, climate[chan]->next.fanspeed) +
            "</td></tr>"
        "<tr><td>" D_STR_SWINGV "</td><td>" +
            htmlSelectSwingv(KEY_SWINGV, climate[chan]->next.swingv) +
            "</td></tr>"
        "<tr><td>" D_STR_SWINGH "</td><td>" +
            htmlSelectSwingh(KEY_SWINGH, climate[chan]->next.swingh) +
            "</td></tr>"
        "<tr><td>" D_STR_QUIET "</td><td>" +
            htmlSelectBool(KEY_QUIET, climate[chan]->next.quiet) +
            "</td></tr>"
        "<tr><td>" D_STR_TURBO "</td><td>" +
            htmlSelectBool(KEY_TURBO, climate[chan]->next.turbo) +
            "</td></tr>"
        "<tr><td>" D_STR_ECONO "</td><td>" +
            htmlSelectBool(KEY_ECONO, climate[chan]->next.econo) +
            "</td></tr>"
        "<tr><td>" D_STR_LIGHT "</td><td>" +
            htmlSelectBool(KEY_LIGHT, climate[chan]->next.light) +
            "</td></tr>"
        "<tr><td>" D_STR_FILTER "</td><td>" +
            htmlSelectBool(KEY_FILTER, climate[chan]->next.filter) +
            "</td></tr>"
        "<tr><td>" D_STR_CLEAN "</td><td>" +
            htmlSelectBool(KEY_CLEAN, climate[chan]->next.clean) +
            "</td></tr>"
        "<tr><td>" D_STR_BEEP "</td><td>" +
            htmlSelectBool(KEY_BEEP, climate[chan]->next.beep) +
            "</td></tr>"
        "<tr><td>Force resend</td><td>" +
            htmlSelectBool(KEY_RESEND, false) +
            "</td></tr>"
        "</table>"
        "<input type='submit' value='Update & Send'>"
        "</form>";
  }
  html += htmlEnd();
  server.send(200, "text/html", html);
}

// Parse the URL args to find the Common A/C arguments.
void handleAirConSet(void) {
#if HTML_PASSWORD_ENABLE
  if (!server.authenticate(HttpUsername, HttpPassword)) {
    debug("Basic HTTP authentication failure for /aircon/set.");
    return server.requestAuthentication();
  }
#endif
  debug("New common a/c received via HTTP");
  uint16_t channel = chan;
  if (kNrOfIrTxGpios > 1) {
    // Scan for the channel number if needed.
    for (uint16_t i = 0; i < server.args(); i++) {
      if (server.argName(i).equals(KEY_CHANNEL)) {
        channel = server.arg(i).toInt();
      }
    }
  }
  // Change the HTML channel for the climate if it is within the correct range.
  if (channel < kNrOfIrTxGpios) chan = channel;

  IRac *ac_ptr = climate[chan];
  String html = htmlHeader(F("Aircon updated!"));
  if (ac_ptr != NULL) {
    bool force_resend = false;
    for (uint16_t i = 0; i < server.args(); i++) {
      if (server.argName(i).equals(KEY_RESEND))
        force_resend = IRac::strToBool(server.arg(i).c_str());
      else
        updateClimate(&(ac_ptr->next), server.argName(i), "", server.arg(i));
    }

    lastClimateSource = F("HTTP");
  } else {  // ac_ptr == NULL
    debug("No climate setup for the given channel. Aborting!");
    html = htmlHeader(F("Aircon update FAILED!"));
  }
  // Redirect back to the aircon page.
  html += addJsReloadUrl(kUrlAircon, kQuickDisplayTime, false);
  html += htmlEnd();
  server.send(200, "text/html", html);
}

String htmlDisabled(void) {
  String html = F(
      "<i>Updates disabled until you set a password. "
      "You will need to <a href='");
  html += kUrlWipe;
  html += F("'>wipe & reset</a> to set one.</i><br><br>");
  return html;
}

// Admin web page
void handleAdmin(void) {
  String html = htmlHeader(F("Administration"));
  html += htmlMenu();
  html += F("<h3>Special commands</h3>");

  html += htmlButton(
      kUrlReboot, F("Reboot"),
      F("A simple reboot of the ESP. <small>ie. No changes</small><br>"
        "<br>"));
  html += htmlButton(
      kUrlWipe, F("Wipe Settings"),
      F("<mark>Warning:</mark> Resets the device back to original settings. "
        "<small>ie. Goes back to AP/Setup mode.</small><br><br>"));
  html += htmlButton(kUrlGpio, F("GPIOs"), F("Change the IR GPIOs.<br>"));
#if FIRMWARE_OTA
  html += F("<hr><h3>Update firmware</h3><p>"
            "<b><mark>Warning:</mark></b><br> ");
  if (!strlen(HttpPassword))  // Deny if password not set
    html += htmlDisabled();
  else  // default password has been changed, so allow it.
    html += F(
        "<i>Updating your firmware may screw up your access to the device. "
        "If you are going to use this, know what you are doing first "
        "(and you probably do).</i><br>"
        "<form method='POST' action='/update' enctype='multipart/form-data'>"
          "Firmware to upload: <input type='file' name='update'>"
          "<input type='submit' value='Update'>"
        "</form>");
#endif  // FIRMWARE_OTA
  html += htmlEnd();
  server.send(200, "text/html", html);
}

uint32_t maxSketchSpace(void) {
#if defined(ESP8266)
  return (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
#else  // defined(ESP8266)
  return UPDATE_SIZE_UNKNOWN;
#endif  // defined(ESP8266)
}

#if REPORT_VCC
String vccToString(void) { return String(ESP.getVcc() / 1000.0); }
#endif  // REPORT_VCC

// Info web page
void handleInfo(void) {
  String html = htmlHeader(F("IR MQTT server info"));
  html += htmlMenu();
  html +=
    "<h3>General</h3>"
    "<p>Hostname: " + String(Hostname) + "<br>"
    "IP address: " + WiFi.localIP().toString() + "<br>"
    "MAC address: " + WiFi.macAddress() + "<br>"
    "Booted: " + timeSince(1) + "<br>" +
    "Version: " _MY_VERSION_ "<br>"
    "Built: " __DATE__
      " " __TIME__ "<br>"
    "Period Offset: " + String(offset) + "us<br>"
    "IR Lib Version: " _IRREMOTEESP8266_VERSION_ "<br>"
#if defined(ESP8266)
    "ESP8266 Core Version: " + ESP.getCoreVersion() + "<br>"
    "Free Sketch Space: " + String(maxSketchSpace() >> 10) + "k<br>"
#endif  // ESP8266
#if defined(ESP32)
    "ESP32 SDK Version: " + ESP.getSdkVersion() + "<br>"
#endif  // ESP32
    "Cpu Freq: " + String(ESP.getCpuFreqMHz()) + "MHz<br>"
    "IR Send GPIO(s): " + listOfTxGpios() + "<br>"
    + irutils::addBoolToString(kInvertTxOutput,
                               "Inverting GPIO output", false) + "<br>"
    "Total send requests: " + String(sendReqCounter) + "<br>"
    "Last message sent: " + String(lastSendSucceeded ? "Ok" : "FAILED") +
    " <i>(" + timeSince(lastSendTime) + ")</i><br>"
#if IR_RX
    "IR Recv GPIO: " + gpioToString(rx_gpio) +
#if IR_RX_PULLUP
    " (pullup)"
#endif  // IR_RX_PULLUP
    "<br>"
    "Total IR Received: " + String(irRecvCounter) + "<br>"
    "Last IR Received: " + lastIrReceived +
    " <i>(" + timeSince(lastIrReceivedTime) + ")</i><br>"
#endif  // IR_RX
    "Duplicate " D_STR_WIFI " networks: " +
        String(HIDE_DUPLICATE_NETWORKS ? "Hide" : "Show") + "<br>"
    "Min " D_STR_WIFI " signal required: "
#ifdef MIN_SIGNAL_STRENGTH
        + String(static_cast<int>(MIN_SIGNAL_STRENGTH)) +
#else  // MIN_SIGNAL_STRENGTH
        "8"
#endif  // MIN_SIGNAL_STRENGTH
        "%<br>"
    "Serial debugging: "
#if DEBUG
        + String(isSerialGpioUsedByIr() ? D_STR_OFF : D_STR_ON) +
#else  // DEBUG
        D_STR_OFF
#endif  // DEBUG
        "<br>"
#if REPORT_VCC
    "Vcc: ";
    html += vccToString();
    html += "V<br>"
#endif  // REPORT_VCC
    "</p>"

    "<h4>Climate Information</h4>"
    "<p>"
    "IR Send GPIO: " + String(txGpioTable[0]) + "<br>"
    "Last update source: " + lastClimateSource + "<br>"
    "Total sent: " + String(irClimateCounter) + "<br>"
    "Last send: " + String(hasClimateBeenSent ?
        (String(lastClimateSucceeded ? "Ok" : "FAILED") +
         " <i>(" + timeElapsed(lastClimateIr.elapsed()) + ")</i>") :
        "<i>Never</i>") + "<br>"

    "</p>"
    // Page footer
    "<hr><p><small><center>"
      "<i>(Note: Page will refresh every 60 " D_STR_SECONDS ".)</i>"
    "<centre></small></p>";
  html += addJsReloadUrl(kUrlInfo, 60, false);
  html += htmlEnd();
  server.send(200, "text/html", html);
}

void doRestart(const char* str, const bool serial_only) {

    debug(str);
  delay(2000);  // Enough time for messages to be sent.
  ESP.restart();
  delay(5000);  // Enough time to ensure we don't return.
}



// Reset web page
void handleReset(void) {
#if HTML_PASSWORD_ENABLE
  if (!server.authenticate(HttpUsername, HttpPassword)) {
    debug(("Basic HTTP authentication failure for " +
           String(kUrlWipe)).c_str());
    return server.requestAuthentication();
  }
#endif
  server.send(200, "text/html",
    htmlHeader(F("Reset WiFi Config"),
               F("Resetting the WiFiManager config back to defaults.")) +
    "<p>Device restarting. Try connecting in a few " D_STR_SECONDS ".</p>" +
    addJsReloadUrl(kUrlRoot, 10, true) +
    htmlEnd());
  // Do the reset.

  if (mountSpiffs()) {
    debug("Removing JSON config file");
    SPIFFS.remove(kConfigFile);
    SPIFFS.end();
  }
  delay(1000);
  debug("Reseting wifiManager's settings.");
  //homekit_storage_reset();
  wifiManager.resetSettings();
  doRestart("Rebooting...");
}

// Reboot web page
void handleReboot() {
#if HTML_PASSWORD_ENABLE
  if (!server.authenticate(HttpUsername, HttpPassword)) {
    debug(("Basic HTTP authentication failure for " +
           String(kUrlReboot)).c_str());
    return server.requestAuthentication();
  }
#endif
  server.send(200, "text/html",
    htmlHeader(F("Device restarting.")) +
    "<p>Try connecting in a few " D_STR_SECONDS ".</p>" +
    addJsReloadUrl(kUrlRoot, kRebootTime, true) +
    htmlEnd());
  doRestart("Reboot requested");
}

// Parse an Air Conditioner A/C Hex String/code and send it.
// Args:
//   irsend: A Ptr to the IRsend object to transmit via.
//   irType: Nr. of the protocol we need to send.
//   str: A hexadecimal string containing the state to be sent.
// Returns:
//   bool: Successfully sent or not.
bool parseStringAndSendAirCon(IRsend *irsend, const decode_type_t irType,
                              const String str) {
  uint8_t strOffset = 0;
  uint8_t state[kStateSizeMax] = {0};  // All array elements are set to 0.
  uint16_t stateSize = 0;

  if (str.startsWith("0x") || str.startsWith("0X"))
    strOffset = 2;
  // Calculate how many hexadecimal characters there are.
  uint16_t inputLength = str.length() - strOffset;
  if (inputLength == 0) {
    debug("Zero length AirCon code encountered. Ignored.");
    return false;  // No input. Abort.
  }

  switch (irType) {  // Get the correct state size for the protocol.
    case DAIKIN:
      // Daikin has 2 different possible size states.
      // (The correct size, and a legacy shorter size.)
      // Guess which one we are being presented with based on the number of
      // hexadecimal digits provided. i.e. Zero-pad if you need to to get
      // the correct length/byte size.
      // This should provide backward compatiblity with legacy messages.
      stateSize = inputLength / 2;  // Every two hex chars is a byte.
      // Use at least the minimum size.
      stateSize = std::max(stateSize, kDaikinStateLengthShort);
      // If we think it isn't a "short" message.
      if (stateSize > kDaikinStateLengthShort)
        // Then it has to be at least the version of the "normal" size.
        stateSize = std::max(stateSize, kDaikinStateLength);
      // Lastly, it should never exceed the "normal" size.
      stateSize = std::min(stateSize, kDaikinStateLength);
      break;
    case FUJITSU_AC:
      // Fujitsu has four distinct & different size states, so make a best guess
      // which one we are being presented with based on the number of
      // hexadecimal digits provided. i.e. Zero-pad if you need to to get
      // the correct length/byte size.
      stateSize = inputLength / 2;  // Every two hex chars is a byte.
      // Use at least the minimum size.
      stateSize = std::max(stateSize,
                           (uint16_t) (kFujitsuAcStateLengthShort - 1));
      // If we think it isn't a "short" message.
      if (stateSize > kFujitsuAcStateLengthShort)
        // Then it has to be at least the smaller version of the "normal" size.
        stateSize = std::max(stateSize, (uint16_t) (kFujitsuAcStateLength - 1));
      // Lastly, it should never exceed the maximum "normal" size.
      stateSize = std::min(stateSize, kFujitsuAcStateLength);
      break;
    case HITACHI_AC3:
      // HitachiAc3 has two distinct & different size states, so make a best
      // guess which one we are being presented with based on the number of
      // hexadecimal digits provided. i.e. Zero-pad if you need to to get
      // the correct length/byte size.
      stateSize = inputLength / 2;  // Every two hex chars is a byte.
      // Use at least the minimum size.
      stateSize = std::max(stateSize,
                           (uint16_t) (kHitachiAc3MinStateLength));
      // If we think it isn't a "short" message.
      if (stateSize > kHitachiAc3MinStateLength)
        // Then it probably the "normal" size.
        stateSize = std::max(stateSize,
                             (uint16_t) (kHitachiAc3StateLength));
      // Lastly, it should never exceed the maximum "normal" size.
      stateSize = std::min(stateSize, kHitachiAc3StateLength);
      break;
    case MWM:
      // MWM has variable size states, so make a best guess
      // which one we are being presented with based on the number of
      // hexadecimal digits provided. i.e. Zero-pad if you need to to get
      // the correct length/byte size.
      stateSize = inputLength / 2;  // Every two hex chars is a byte.
      // Use at least the minimum size.
      stateSize = std::max(stateSize, (uint16_t) 3);
      // Cap the maximum size.
      stateSize = std::min(stateSize, kStateSizeMax);
      break;
    case SAMSUNG_AC:
      // Samsung has two distinct & different size states, so make a best guess
      // which one we are being presented with based on the number of
      // hexadecimal digits provided. i.e. Zero-pad if you need to to get
      // the correct length/byte size.
      stateSize = inputLength / 2;  // Every two hex chars is a byte.
      // Use at least the minimum size.
      stateSize = std::max(stateSize, (uint16_t) (kSamsungAcStateLength));
      // If we think it isn't a "normal" message.
      if (stateSize > kSamsungAcStateLength)
        // Then it probably the extended size.
        stateSize = std::max(stateSize,
                             (uint16_t) (kSamsungAcExtendedStateLength));
      // Lastly, it should never exceed the maximum "extended" size.
      stateSize = std::min(stateSize, kSamsungAcExtendedStateLength);
      break;
    default:  // Everything else.
      stateSize = IRsend::defaultBits(irType) / 8;
      if (!stateSize || !hasACState(irType)) {
        // Not a protocol we expected. Abort.
        debug("Unexpected AirCon protocol detected. Ignoring.");
        return false;
      }
  }
  if (inputLength > stateSize * 2) {
    debug("AirCon code to large for the given protocol.");
    return false;
  }

  // Ptr to the least significant byte of the resulting state for this protocol.
  uint8_t *statePtr = &state[stateSize - 1];

  // Convert the string into a state array of the correct length.
  for (uint16_t i = 0; i < inputLength; i++) {
    // Grab the next least sigificant hexadecimal digit from the string.
    uint8_t c = tolower(str[inputLength + strOffset - i - 1]);
    if (isxdigit(c)) {
      if (isdigit(c))
        c -= '0';
      else
        c = c - 'a' + 10;
    } else {
      debug("Aborting! Non-hexadecimal char found in AirCon state:");
      debug(str.c_str());
      return false;
    }
    if (i % 2 == 1) {  // Odd: Upper half of the byte.
      *statePtr += (c << 4);
      statePtr--;  // Advance up to the next least significant byte of state.
    } else {  // Even: Lower half of the byte.
      *statePtr = c;
    }
  }
  if (!irsend->send(irType, state, stateSize)) {
    debug("Unexpected AirCon type in send request. Not sent.");
    return false;
  }
  return true;  // We were successful as far as we can tell.
}

// Count how many values are in the String.
// Args:
//   str:  String containing the values.
//   sep:  Character that separates the values.
// Returns:
//   The number of values found in the String.
uint16_t countValuesInStr(const String str, char sep) {
  int16_t index = -1;
  uint16_t count = 1;
  do {
    index = str.indexOf(sep, index + 1);
    count++;
  } while (index != -1);
  return count;
}

// Dynamically allocate an array of uint16_t's.
// Args:
//   size:  Nr. of uint16_t's need to be in the new array.
// Returns:
//   A Ptr to the new array. Restarts the ESP if it fails.
uint16_t * newCodeArray(const uint16_t size) {
  uint16_t *result;

  result = reinterpret_cast<uint16_t*>(malloc(size * sizeof(uint16_t)));
  // Check we malloc'ed successfully.
  if (result == NULL)  // malloc failed, so give up.
    doRestart(
        "FATAL: Can't allocate memory for an array for a new message! "
        "Forcing a reboot!", true);  // Send to serial only as we are in low mem
  return result;
}

#if SEND_GLOBALCACHE
// Parse a GlobalCache String/code and send it.
// Args:
//   irsend: A ptr to the IRsend object to transmit via.
//   str: A GlobalCache formatted String of comma separated numbers.
//        e.g. "38000,1,1,170,170,20,63,20,63,20,63,20,20,20,20,20,20,20,20,20,
//              20,20,63,20,63,20,63,20,20,20,20,20,20,20,20,20,20,20,20,20,63,
//              20,20,20,20,20,20,20,20,20,20,20,20,20,63,20,20,20,63,20,63,20,
//              63,20,63,20,63,20,63,20,1798"
//        Note: The leading "1:1,1," of normal GC codes should be removed.
// Returns:
//   bool: Successfully sent or not.
bool parseStringAndSendGC(IRsend *irsend, const String str) {
  uint16_t count;
  uint16_t *code_array;
  String tmp_str;

  // Remove the leading "1:1,1," if present.
  if (str.startsWith("1:1,1,"))
    tmp_str = str.substring(6);
  else
    tmp_str = str;

  // Find out how many items there are in the string.
  count = countValuesInStr(tmp_str, ',');

  // Now we know how many there are, allocate the memory to store them all.
  code_array = newCodeArray(count);

  // Now convert the strings to integers and place them in code_array.
  count = 0;
  uint16_t start_from = 0;
  int16_t index = -1;
  do {
    index = tmp_str.indexOf(',', start_from);
    code_array[count] = tmp_str.substring(start_from, index).toInt();
    start_from = index + 1;
    count++;
  } while (index != -1);
  irsend->sendGC(code_array, count);  // All done. Send it.
  free(code_array);  // Free up the memory allocated.
  if (count > 0)
    return true;  // We sent something.
  return false;  // We probably didn't.
}
#endif  // SEND_GLOBALCACHE

#if SEND_PRONTO
// Parse a Pronto Hex String/code and send it.
// Args:
//   irsend: A ptr to the IRsend object to transmit via.
//   str: A comma-separated String of nr. of repeats, then hexadecimal numbers.
//        e.g. "R1,0000,0067,0000,0015,0060,0018,0018,0018,0030,0018,0030,0018,
//              0030,0018,0018,0018,0030,0018,0018,0018,0018,0018,0030,0018,
//              0018,0018,0030,0018,0030,0018,0030,0018,0018,0018,0018,0018,
//              0030,0018,0018,0018,0018,0018,0030,0018,0018,03f6"
//              or
//              "0000,0067,0000,0015,0060,0018". i.e. without the Repeat value
//        Requires at least kProntoMinLength comma-separated values.
//        sendPronto() only supports raw pronto code types, thus so does this.
//   repeats:  Nr. of times the message is to be repeated.
//             This value is ignored if an embeddd repeat is found in str.
// Returns:
//   bool: Successfully sent or not.
bool parseStringAndSendPronto(IRsend *irsend, const String str,
                              uint16_t repeats) {
  uint16_t count;
  uint16_t *code_array;
  int16_t index = -1;
  uint16_t start_from = 0;

  // Find out how many items there are in the string.
  count = countValuesInStr(str, ',');

  // Check if we have the optional embedded repeats value in the code string.
  if (str.startsWith("R") || str.startsWith("r")) {
    // Grab the first value from the string, as it is the nr. of repeats.
    index = str.indexOf(',', start_from);
    repeats = str.substring(start_from + 1, index).toInt();  // Skip the 'R'.
    start_from = index + 1;
    count--;  // We don't count the repeats value as part of the code array.
  }

  // We need at least kProntoMinLength values for the code part.
  if (count < kProntoMinLength) return false;

  // Now we know how many there are, allocate the memory to store them all.
  code_array = newCodeArray(count);

  // Rest of the string are values for the code array.
  // Now convert the hex strings to integers and place them in code_array.
  count = 0;
  do {
    index = str.indexOf(',', start_from);
    // Convert the hexadecimal value string to an unsigned integer.
    code_array[count] = strtoul(str.substring(start_from, index).c_str(),
                                NULL, 16);
    start_from = index + 1;
    count++;
  } while (index != -1);

  irsend->sendPronto(code_array, count, repeats);  // All done. Send it.
  free(code_array);  // Free up the memory allocated.
  if (count > 0)
    return true;  // We sent something.
  return false;  // We probably didn't.
}
#endif  // SEND_PRONTO

#if SEND_RAW
// Parse an IRremote Raw Hex String/code and send it.
// Args:
//   irsend: A ptr to the IRsend object to transmit via.
//   str: A comma-separated String containing the freq and raw IR data.
//        e.g. "38000,9000,4500,600,1450,600,900,650,1500,..."
//        Requires at least two comma-separated values.
//        First value is the transmission frequency in Hz or kHz.
// Returns:
//   bool: Successfully sent or not.
bool parseStringAndSendRaw(IRsend *irsend, const String str) {
  uint16_t count;
  uint16_t freq = 38000;  // Default to 38kHz.
  uint16_t *raw_array;

  // Find out how many items there are in the string.
  count = countValuesInStr(str, ',');

  // We expect the frequency as the first comma separated value, so we need at
  // least two values. If not, bail out.
  if (count < 2)  return false;
  count--;  // We don't count the frequency value as part of the raw array.

  // Now we know how many there are, allocate the memory to store them all.
  raw_array = newCodeArray(count);

  // Grab the first value from the string, as it is the frequency.
  int16_t index = str.indexOf(',', 0);
  freq = str.substring(0, index).toInt();
  uint16_t start_from = index + 1;
  // Rest of the string are values for the raw array.
  // Now convert the strings to integers and place them in raw_array.
  count = 0;
  do {
    index = str.indexOf(',', start_from);
    raw_array[count] = str.substring(start_from, index).toInt();
    start_from = index + 1;
    count++;
  } while (index != -1);

  irsend->sendRaw(raw_array, count, freq);  // All done. Send it.
  free(raw_array);  // Free up the memory allocated.
  if (count > 0)
    return true;  // We sent something.
  return false;  // We probably didn't.
}
#endif  // SEND_RAW

uint8_t getDefaultIrSendIdx(void) {
  for (uint16_t i = 0; i < kNrOfIrTxGpios; i++)
    if (IrSendTable[i] != NULL) return i;
  return 0;
}

IRsend* getDefaultIrSendPtr(void) {
  return IrSendTable[getDefaultIrSendIdx()];
}

// Parse the URL args to find the IR code.
void handleIr(void) {
#if HTML_PASSWORD_ENABLE
  if (!server.authenticate(HttpUsername, HttpPassword)) {
    debug("Basic HTTP authentication failure for /ir.");
    return server.requestAuthentication();
  }
#endif
  uint64_t data = 0;
  String data_str = "";
  decode_type_t ir_type = decode_type_t::NEC;  // Default to NEC codes.
  uint16_t nbits = 0;
  uint16_t repeat = 0;
  int16_t channel = -1;

  for (uint16_t i = 0; i < server.args(); i++) {
    if (server.argName(i).equals(KEY_TYPE) ||
        server.argName(i).equals(KEY_PROTOCOL)) {
      ir_type = strToDecodeType(server.arg(i).c_str());
    } else if (server.argName(i).equals(KEY_CODE)) {
      data = getUInt64fromHex(server.arg(i).c_str());
      data_str = server.arg(i);
    } else if (server.argName(i).equals(KEY_BITS)) {
      nbits = server.arg(i).toInt();
    } else if (server.argName(i).equals(KEY_REPEAT)) {
      repeat = server.arg(i).toInt();
    } else if (server.argName(i).equals(KEY_CHANNEL)) {
      channel = server.arg(i).toInt();
    }
  }
  debug("New code received via HTTP");
  IRsend *tx_ptr = getDefaultIrSendPtr();
  if (channel >= 0 && channel < kNrOfIrTxGpios && IrSendTable[channel] != NULL)
    tx_ptr = IrSendTable[channel];
  lastSendSucceeded = sendIRCode(tx_ptr, ir_type, data, data_str.c_str(), nbits,
                                 repeat);
  String html = htmlHeader(F("IR command sent!"));
  html += addJsReloadUrl(kUrlRoot, kQuickDisplayTime, true);
  html += htmlEnd();
  server.send(200, "text/html", html);
}

// GPIO menu page
void handleGpio(void) {
#if HTML_PASSWORD_ENABLE
  if (!server.authenticate(HttpUsername, HttpPassword)) {
    debug("Basic HTTP authentication failure for /gpios.");
    return server.requestAuthentication();
  }
#endif
  String html = htmlHeader(F("GPIO config"));
  html += F(
      "<form method='POST' action='/gpio/set' enctype='multipart/form-data'>");
  html += htmlMenu();
  html += F("<h2><mark>WARNING: Choose carefully! You can cause damage to your "
            "hardware or make the device unresponsive.</mark></h2>");
  html += F("<h3>Send</h3>IR LED");
  for (uint16_t i = 0; i < kNrOfIrTxGpios; i++) {
    if (kNrOfIrTxGpios > 1) {
      html += F(" #");
      html += String(i);
    }
    html += htmlSelectGpio(KEY_TX_GPIO + String(i), txGpioTable[i], kTxGpios,
                           sizeof(kTxGpios));
  }
#if IR_RX
  html += F("<h3>Receive</h3>IR RX Module");
  html += htmlSelectGpio(KEY_RX_GPIO, rx_gpio, kRxGpios,
                         sizeof(kRxGpios));
#endif  // IR_RX
  html += F("<br><br><hr>");
  if (strlen(HttpPassword))  // Allow if password set
    html += F("<input type='submit' value='Save & Reboot'>");
  else
    html += htmlDisabled();
  html += F("</form>");
  html += htmlEnd();
  server.send(200, "text/html", html);
}

// GPIO setting page
void handleGpioSetting(void) {
  bool changed = false;
  if (!server.authenticate(HttpUsername, HttpPassword)) {
    debug("Basic HTTP authentication failure for /gpios.");
    return server.requestAuthentication();
  }
  String html = htmlHeader(F("Update GPIOs"));
  if (!strlen(HttpPassword)) {  // Don't allow if password not set
    html += htmlDisabled();
  } else {
    debug("Attempt to change GPIOs");
    for (uint16_t arg = 0; arg < server.args(); arg++) {
      int8_t num = std::max(static_cast<int8_t>(server.arg(arg).toInt()),
                            kGpioUnused);
#if IR_RX
      if (server.argName(arg).equals(KEY_RX_GPIO)) {
        if (rx_gpio != num) {
          rx_gpio = num;
          changed = true;
        }
      } else {
#endif  // IR_RX
        for (uint16_t i = 0; i < kNrOfIrTxGpios; i++) {
          if (server.argName(arg).equals(KEY_TX_GPIO + String(i))) {
            if (txGpioTable[i] != num) {
              txGpioTable[i] = num;
              changed = true;
            }
          }
        }
#if IR_RX
      }
#endif  // IR_RX
    }
    if (!changed) {
      html += F("<h2>No changes detected!</h2>");
    } else if (saveConfig()) {
      html += F("<h2>Saved changes & rebooting.</h2>");
    } else {
      html += F("<h2><mark>ERROR: Changes didn't save correctly! "
                "Rebooting.</h2>");
    }
  }
  html += addJsReloadUrl(changed ? kUrlRoot : kUrlGpio,
                         changed ? kRebootTime : kQuickDisplayTime,
                         true);
  html += htmlEnd();
  server.send(200, "text/html", html);
  if (changed) doRestart("GPIOs were changed. Rebooting!");
}

void handleNotFound(void) {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i < server.args(); i++)
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  server.send(404, "text/plain", message);
}

void setup_wifi(void) {
  delay(10);
  loadConfigFile();
  // We start by connecting to a WiFi network
  wifiManager.setTimeout(300);  // Time out after 5 mins.
  // Set up additional parameters for WiFiManager config menu page.
  wifiManager.setSaveConfigCallback(saveWifiConfigCallback);
  WiFiManagerParameter custom_hostname_text(
      "<br><center>Hostname</center>");
  wifiManager.addParameter(&custom_hostname_text);
  WiFiManagerParameter custom_hostname(
      kHostnameKey, kHostnameKey, Hostname, kHostnameLength);
  wifiManager.addParameter(&custom_hostname);
  WiFiManagerParameter custom_authentication_text(
      "<br><br><center>Web/OTA authentication</center>");
  wifiManager.addParameter(&custom_authentication_text);
  WiFiManagerParameter custom_http_username(
      kHttpUserKey, "username", HttpUsername, kUsernameLength);
  wifiManager.addParameter(&custom_http_username);
  WiFiManagerParameter custom_http_password(
      kHttpPassKey, "password (No OTA if blank)", HttpPassword, kPasswordLength,
      " type='password'");
  wifiManager.addParameter(&custom_http_password);

#if USE_STATIC_IP
  // Use a static IP config rather than the one supplied via DHCP.
  wifiManager.setSTAStaticIPConfig(kIPAddress, kGateway, kSubnetMask);
#endif  // USE_STATIC_IP
#if MIN_SIGNAL_STRENGTH
  wifiManager.setMinimumSignalQuality(MIN_SIGNAL_STRENGTH);
#endif  // MIN_SIGNAL_STRENGTH
  wifiManager.setRemoveDuplicateAPs(HIDE_DUPLICATE_NETWORKS);

  if (!wifiManager.autoConnect())
    // Reboot. A.k.a. "Have you tried turning it Off and On again?"
    doRestart(D_STR_WIFI " failed to connect and hit timeout. Rebooting...",
              true);


  strncpy(Hostname, custom_hostname.getValue(), kHostnameLength);
  strncpy(HttpUsername, custom_http_username.getValue(), kUsernameLength);
  strncpy(HttpPassword, custom_http_password.getValue(), kPasswordLength);
  if (flagSaveWifiConfig) {
    saveConfig();
  }
  debug("WiFi connected. IP address:");
  debug(WiFi.localIP().toString().c_str());
}

void init_vars(void) {

}

void setup(void) {
#if DEBUG
  if (!isSerialGpioUsedByIr()) {
#if defined(ESP8266)
    // Use SERIAL_TX_ONLY so that the RX pin can be freed up for GPIO/IR use.
    Serial.begin(BAUD_RATE, SERIAL_8N1, SERIAL_TX_ONLY);
#else  // ESP8266
    Serial.begin(BAUD_RATE, SERIAL_8N1);
#endif  // ESP8266
    while (!Serial)  // Wait for the serial connection to be establised.
      delay(50);
    Serial.println();
    debug("IRMQTTServer " _MY_VERSION_ " has booted.");
  }
#endif  // DEBUG

  setup_wifi();
  /*if (wifiManager.autoConnect()) {
    if (MAC_code1[0] == 'E') {
      A = 1996;
    } else if (MAC_code1[0] == 'A') {
      A = 1991;
    } else if (MAC_code1[0] == 'B') {
      A = 1992;
    } else if (MAC_code1[0] == 'C') {
      A = 1993;
    } else if (MAC_code1[0] == 'D') {
      A = 1994;
    } else if (MAC_code1[0] == 'F') {
      A = 1995;
    } else {
      A = 1997;
    }
    homekityes = A * 518;
    strcpy(s, String(temptopic).c_str());
    sscanf(s, "%d", &B);
    Serial.println(homekityes);
    Serial.println(B);
	
    my_homekit_setup();
  }*/

#if DEBUG
  // After the config has been loaded, check again if we are using a Serial GPIO
  if (isSerialGpioUsedByIr()) Serial.end();
#endif  // DEBUG

  channel_re.reserve(kNrOfIrTxGpios * 3);
  // Initialise all the IR transmitters.
  for (uint8_t i = 0; i < kNrOfIrTxGpios; i++) {
    if (txGpioTable[i] == kGpioUnused) {
      IrSendTable[i] = NULL;
      climate[i] = NULL;
    } else {
      IrSendTable[i] = new IRsend(txGpioTable[i], kInvertTxOutput);
      if (IrSendTable[i] != NULL) {
        IrSendTable[i]->begin();
        offset = IrSendTable[i]->calibrate();
      }
      climate[i] = new IRac(txGpioTable[i], kInvertTxOutput);
      if (climate[i] != NULL && i > 0) channel_re += '_' + String(i) + '|';
    }
  }
  lastClimateSource = F("None");
  if (channel_re.length() == 1) {
    channel_re = "";
  } else {
    channel_re.remove(channel_re.length() - 1, 1);  // Remove the last char.
    channel_re += F(")?");
  }
#if IR_RX
  if (rx_gpio != kGpioUnused)
    irrecv = new IRrecv(rx_gpio, kCaptureBufferSize, kCaptureTimeout, true);
  if (irrecv != NULL) {
#if DECODE_HASH
    // Ignore messages with less than minimum on or off pulses.
    irrecv->setUnknownThreshold(kMinUnknownSize);
#endif  // DECODE_HASH
    irrecv->enableIRIn(IR_RX_PULLUP);  // Start the receiver
  }
#endif  // IR_RX
  // Wait a bit for things to settle.
  delay(500);

  lastReconnectAttempt = 0;

#if MDNS_ENABLE
#if defined(ESP8266)
  if (mdns.begin(Hostname, WiFi.localIP())) {
#else  // ESP8266
  if (mdns.begin(Hostname)) {
#endif  // ESP8266
    debug("MDNS responder started");
  }
#endif  // MDNS_ENABLE

  // Setup the root web page.
  server.on(kUrlRoot, handleRoot);
#if EXAMPLES_ENABLE
  // Setup the examples web page.
  server.on(kUrlExamples, handleExamples);
#endif  // EXAMPLES_ENABLE
  // Setup the page to handle web-based IR codes.
  server.on("/ir", handleIr);
  // Setup the aircon page.
  server.on(kUrlAircon, handleAirCon);
  // Setup the aircon update page.
  server.on("/aircon/set", handleAirConSet);
  // Setup the info page.
  server.on(kUrlInfo, handleInfo);
  // Setup the admin page.
  server.on(kUrlAdmin, handleAdmin);
  // Setup a reset page to cause WiFiManager information to be reset.
  server.on(kUrlWipe, handleReset);
  // Reboot url
  server.on(kUrlReboot, handleReboot);
  // Show & pick which gpios are used for what etc.
  server.on(kUrlGpio, handleGpio);
  // Parse and update the new gpios.
  server.on(kUrlGpioSet, handleGpioSetting);


#if FIRMWARE_OTA
  // Setup the URL to allow Over-The-Air (OTA) firmware updates.
  if (strlen(HttpPassword)) {  // Allow if password is set.
    server.on("/update", HTTP_POST, [](){

        server.send(200, "text/html",
            htmlHeader(F("Updating firmware")) +
            "<hr>"
            "<h3>Warning! Don't " D_STR_POWER " " D_STR_OFF " the device for "
            "60 " D_STR_SECONDS "!</h3>"
            "<p>The firmware is uploading and will try to flash itself. "
            "It is important to not interrupt the process.</p>"
            "<p>The firmware upload seems to have " +
            String(Update.hasError() ? "FAILED!" : "SUCCEEDED!") +
            " Rebooting! </p>" +
            addJsReloadUrl(kUrlRoot, 20, true) +
            htmlEnd());
        doRestart("Post firmware reboot.");
      }, [](){
        if (!server.authenticate(HttpUsername, HttpPassword)) {
          debug("Basic HTTP authentication failure for /update.");
          return server.requestAuthentication();
        }
        HTTPUpload& upload = server.upload();
        if (upload.status == UPLOAD_FILE_START) {
          debug("Update:");
          debug(upload.filename.c_str());
#if defined(ESP8266)
          WiFiUDP::stopAll();
#endif  // defined(ESP8266)
          if (!Update.begin(maxSketchSpace())) {  // start with max available
#if DEBUG
            if (!isSerialGpioUsedByIr())
              Update.printError(Serial);
#endif  // DEBUG
          }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
          if (Update.write(upload.buf, upload.currentSize) !=
              upload.currentSize) {
#if DEBUG
            if (!isSerialGpioUsedByIr())
              Update.printError(Serial);
#endif  // DEBUG
          }
        } else if (upload.status == UPLOAD_FILE_END) {
          // true to set the size to the current progress
          if (Update.end(true)) {
            debug("Update Success:");
            debug(String(upload.totalSize).c_str());
            debug("Rebooting...");
          }
        }
        yield();
      });
    }
#endif  // FIRMWARE_OTA

  // Set up an error page.
  server.onNotFound(handleNotFound);

  server.begin();
  debug("HTTP server started");
}


void loop(void) {
  server.handleClient();  // Handle any web activity
  //my_homekit_loop();
#if IR_RX
  // Check if an IR code has been received via the IR RX module.
#if REPORT_UNKNOWNS
  if (irrecv != NULL && irrecv->decode(&capture)) {
#else  // REPORT_UNKNOWNS
  if (irrecv != NULL && irrecv->decode(&capture) &&
      capture.decode_type != UNKNOWN) {
#endif  // REPORT_UNKNOWNS
    lastIrReceivedTime = millis();
    lastIrReceived = String(capture.decode_type) + kCommandDelimiter[0] +
        resultToHexidecimal(&capture);
#if REPORT_RAW_UNKNOWNS
    if (capture.decode_type == UNKNOWN) {
      lastIrReceived += ";";
      for (uint16_t i = 1; i < capture.rawlen; i++) {
        uint32_t usecs;
        for (usecs = capture.rawbuf[i] * kRawTick; usecs > UINT16_MAX;
             usecs -= UINT16_MAX) {
          lastIrReceived += uint64ToString(UINT16_MAX);
          lastIrReceived += ",0,";
        }
        lastIrReceived += uint64ToString(usecs, 10);
        if (i < capture.rawlen - 1)
          lastIrReceived += ",";
      }
    }
#endif  // REPORT_RAW_UNKNOWNS
    // If it isn't an AC code, add the bits.
    if (!hasACState(capture.decode_type))
      lastIrReceived += kCommandDelimiter[0] + String(capture.bits);

    irRecvCounter++;
#if USE_DECODED_AC_SETTINGS
    if (decodeCommonAc(&capture)) lastClimateSource = F("IR");
#endif  // USE_DECODED_AC_SETTINGS
  }
#endif  // IR_RX
  delay(100);
}

// Arduino framework doesn't support strtoull(), so make our own one.
uint64_t getUInt64fromHex(char const *str) {
  uint64_t result = 0;
  uint16_t offset = 0;
  // Skip any leading '0x' or '0X' prefix.
  if (str[0] == '0' && (str[1] == 'x' || str[1] == 'X')) offset = 2;
  for (; isxdigit((unsigned char)str[offset]); offset++) {
    char c = str[offset];
    result *= 16;
    if (isdigit(c))
      result += c - '0';  // '0' .. '9'
    else if (isupper(c))
      result += c - 'A' + 10;  // 'A' .. 'F'
    else
      result += c - 'a' + 10;  // 'a' .. 'f'
  }
  return result;
}

// Transmit the given IR message.
//
// Args:
//   irsend:   A pointer to a IRsend object to transmit via.
//   ir_type:  enum of the protocol to be sent.
//   code:     Numeric payload of the IR message. Most protocols use this.
//   code_str: The unparsed code to be sent. Used by complex protocol encodings.
//   bits:     Nr. of bits in the protocol. 0 means use the protocol's default.
//   repeat:   Nr. of times the message is to be repeated. (Not all protocols.)
// Returns:
//   bool: Successfully sent or not.
bool sendIRCode(IRsend *irsend, decode_type_t const ir_type,
                uint64_t const code, char const * code_str, uint16_t bits,
                uint16_t repeat) {
  if (irsend == NULL) return false;
  bool success = true;  // Assume success.
  // Ensure we have enough repeats.
  repeat = std::max(IRsend::minRepeats(ir_type), repeat);
  if (bits == 0) bits = IRsend::defaultBits(ir_type);
  // Create a pseudo-lock so we don't try to send two codes at the same time.
  while (lockIr)
    delay(20);
  lockIr = true;


  // Turn off IR capture if we need to.
#if IR_RX && DISABLE_CAPTURE_WHILE_TRANSMITTING
  if (irrecv != NULL) irrecv->disableIRIn();  // Stop the IR receiver
#endif  // IR_RX && DISABLE_CAPTURE_WHILE_TRANSMITTING
  // send the IR message.
  switch (ir_type) {
#if SEND_PRONTO
    case decode_type_t::PRONTO:  // 25
      success = parseStringAndSendPronto(irsend, code_str, repeat);
      break;
#endif  // SEND_PRONTO
    case decode_type_t::RAW:  // 30
#if SEND_RAW
      success = parseStringAndSendRaw(irsend, code_str);
      break;
#endif
#if SEND_GLOBALCACHE
    case decode_type_t::GLOBALCACHE:  // 31
      success = parseStringAndSendGC(irsend, code_str);
      break;
#endif
    default:  // Everything else.
      if (hasACState(ir_type))  // protocols with > 64 bits
        success = parseStringAndSendAirCon(irsend, ir_type, code_str);
      else  // protocols with <= 64 bits
        success = irsend->send(ir_type, code, bits, repeat);
  }
#if IR_RX && DISABLE_CAPTURE_WHILE_TRANSMITTING
  // Turn IR capture back on if we need to.
  if (irrecv != NULL) irrecv->enableIRIn();  // Restart the receiver
#endif  // IR_RX && DISABLE_CAPTURE_WHILE_TRANSMITTING
  lastSendTime = millis();
  // Release the lock.
  lockIr = false;

  // Indicate that we sent the message or not.
  if (success) {
    sendReqCounter++;
    debug("Sent the IR message:");
  } else {
    debug("Failed to send IR Message:");
  }
  debug(D_STR_PROTOCOL ": ");
  debug(String(ir_type).c_str());
  // For "long" codes we basically repeat what we got.
  if (hasACState(ir_type) || ir_type == PRONTO || ir_type == RAW ||
      ir_type == GLOBALCACHE) {
    debug(D_STR_CODE ": ");
    debug(code_str);
    // Confirm what we were asked to send was sent.

  } else {  // For "short" codes, we break it down a bit more before we report.
    debug((D_STR_CODE ": 0x" + uint64ToString(code, 16)).c_str());
    debug((D_STR_BITS ": " + String(bits)).c_str());
    debug((D_STR_REPEAT ": " + String(repeat)).c_str());

  }
  return success;
}

bool sendInt(const String topic, const int32_t num, const bool retain) {

}

bool sendBool(const String topic, const bool on, const bool retain) {

}

bool sendString(const String topic, const String str, const bool retain) {

}

bool sendFloat(const String topic, const float_t temp, const bool retain) {

}


void updateClimate(stdAc::state_t *state, const String str,
                   const String prefix, const String payload) {

  if (str.equals(prefix + KEY_PROTOCOL)) {
    state->protocol = strToDecodeType(payload.c_str());
  } else if (str.equals(prefix + KEY_MODEL)) {
    state->model = IRac::strToModel(payload.c_str());
  } else if (str.equals(prefix + KEY_POWER)) {
    state->power = IRac::strToBool(payload.c_str());
	if (!state->power) {
		state->swingv = stdAc::swingv_t::kOff;
	}
  } else if (str.equals(prefix + KEY_MODE)) {
    state->mode = IRac::strToOpmode(payload.c_str());
  } else if (str.equals(prefix + KEY_TEMP)) {
    state->degrees = payload.toFloat();
  } else if (str.equals(prefix + KEY_FANSPEED)) {
    state->fanspeed = IRac::strToFanspeed(payload.c_str());
  } else if (str.equals(prefix + KEY_SWINGV)) {
    state->swingv = IRac::strToSwingV(payload.c_str());
  } else if (str.equals(prefix + KEY_SWINGH)) {
    state->swingh = IRac::strToSwingH(payload.c_str());
  } else if (str.equals(prefix + KEY_QUIET)) {
    state->quiet = IRac::strToBool(payload.c_str());
  } else if (str.equals(prefix + KEY_TURBO)) {
    state->turbo = IRac::strToBool(payload.c_str());
  } else if (str.equals(prefix + KEY_ECONO)) {
    state->econo = IRac::strToBool(payload.c_str());
  } else if (str.equals(prefix + KEY_LIGHT)) {
    state->light = IRac::strToBool(payload.c_str());
  } else if (str.equals(prefix + KEY_BEEP)) {
    state->beep = IRac::strToBool(payload.c_str());
  } else if (str.equals(prefix + KEY_FILTER)) {
    state->filter = IRac::strToBool(payload.c_str());
  } else if (str.equals(prefix + KEY_CLEAN)) {
    state->clean = IRac::strToBool(payload.c_str());
  } else if (str.equals(prefix + KEY_CELSIUS)) {
    state->celsius = IRac::strToBool(payload.c_str());
  } else if (str.equals(prefix + KEY_SLEEP)) {
    state->sleep = payload.toInt();
  }
}

bool sendClimate(const String topic_prefix, const bool retain,
                 const bool forceMQTT, const bool forceIR,
                 const bool enableIR, IRac *ac) {
  bool diff = false;
  bool success = true;
  const stdAc::state_t next = ac->getState();
  const stdAc::state_t prev = ac->getStatePrev();
  if (prev.protocol != next.protocol || forceMQTT) {
    diff = true;
    success &= sendString(topic_prefix + KEY_PROTOCOL,
                          typeToString(next.protocol), retain);
  }
  if (prev.model != next.model || forceMQTT) {
    diff = true;
    success &= sendInt(topic_prefix + KEY_MODEL, next.model, retain);
  }
  String mode_str = IRac::opmodeToString(next.mode);
  mode_str.toLowerCase();
  String swingv_str = IRac::swingvToString(next.swingv);
  swingv_str.toLowerCase();
  if (prev.power != next.power || forceMQTT) {
    diff = true;
    success &= sendBool(topic_prefix + KEY_POWER, next.power, retain);
  }
  if (prev.mode != next.mode || forceMQTT) {  
    success &= sendString(topic_prefix + KEY_MODE, mode_str, retain);
    diff = true;
  }
  if (prev.degrees != next.degrees || forceMQTT) {
    diff = true;
    success &= sendFloat(topic_prefix + KEY_TEMP, next.degrees, retain);
  }
  if (prev.celsius != next.celsius || forceMQTT) {
    diff = true;
    success &= sendBool(topic_prefix + KEY_CELSIUS, next.celsius, retain);
  }
  if (prev.fanspeed != next.fanspeed || forceMQTT) {
    diff = true;
    success &= sendString(topic_prefix + KEY_FANSPEED,
                          IRac::fanspeedToString(next.fanspeed), retain);
  }
  if (prev.swingv != next.swingv || forceMQTT) {
    diff = true;
    success &= sendString(topic_prefix + KEY_SWINGV,
                          swingv_str, retain);
  }  
  if (prev.swingh != next.swingh || forceMQTT) {
    diff = true;
    success &= sendString(topic_prefix + KEY_SWINGH,
                          IRac::swinghToString(next.swingh), retain);
  }
  if (prev.quiet != next.quiet || forceMQTT) {
    diff = true;
    success &= sendBool(topic_prefix + KEY_QUIET, next.quiet, retain);
  }
  if (prev.turbo != next.turbo || forceMQTT) {
    diff = true;
    success &= sendBool(topic_prefix + KEY_TURBO, next.turbo, retain);
  }
  if (prev.econo != next.econo || forceMQTT) {
    diff = true;
    success &= sendBool(topic_prefix + KEY_ECONO, next.econo, retain);
  }
  if (prev.light != next.light || forceMQTT) {
    diff = true;
    success &= sendBool(topic_prefix + KEY_LIGHT, next.light, retain);
  }
  if (prev.filter != next.filter || forceMQTT) {
    diff = true;
    success &= sendBool(topic_prefix + KEY_FILTER, next.filter, retain);
  }
  if (prev.clean != next.clean || forceMQTT) {
    diff = true;
    success &= sendBool(topic_prefix + KEY_CLEAN, next.clean, retain);
  }
  if (prev.beep != next.beep || forceMQTT) {
    diff = true;
    success &= sendBool(topic_prefix + KEY_BEEP, next.beep, retain);
  }
  if (prev.sleep != next.sleep || forceMQTT) {
    diff = true;
    success &= sendInt(topic_prefix + KEY_SLEEP, next.sleep, retain);
  }
  if (diff && !forceMQTT) {
    debug("Difference in common A/C state detected.");

  } else {
    debug("NO difference in common A/C state detected.");
  }
  // Only send an IR message if we need to.
  if (enableIR && ((diff && !forceMQTT) || forceIR)) {
    sendReqCounter++;
    if (ac == NULL) {  // No climate object is available.
      debug("Can't send climate state as common A/C object doesn't exist!");
      return false;
    }
    debug("Sending common A/C state via IR.");
#if IR_RX && DISABLE_CAPTURE_WHILE_TRANSMITTING
    // Turn IR capture off if we need to.
    if (irrecv != NULL) irrecv->disableIRIn();  // Stop the IR receiver
#endif  // IR_RX && DISABLE_CAPTURE_WHILE_TRANSMITTING
    lastClimateSucceeded = ac->sendAc();
#if IR_RX && DISABLE_CAPTURE_WHILE_TRANSMITTING
    // Turn IR capture back on if we need to.
    if (irrecv != NULL) irrecv->enableIRIn();  // Restart the receiver
#endif  // IR_RX && DISABLE_CAPTURE_WHILE_TRANSMITTING
    if (lastClimateSucceeded) hasClimateBeenSent = true;
    success &= lastClimateSucceeded;
    lastClimateIr.reset();
    irClimateCounter++;
  }
  // Mark the "next" value as old/previous.
  if (ac != NULL) {
    ac->markAsSent();
  }
  return success;
}

#if USE_DECODED_AC_SETTINGS && IR_RX
// Decode and use a valid IR A/C remote that we understand enough to convert
// to a Common A/C format.
// Args:
//   decode: A successful raw IR decode object.
// Returns:
//   A boolean indicating success or failure.
bool decodeCommonAc(const decode_results *decode) {
  if (!IRac::isProtocolSupported(decode->decode_type)) {
    debug("Inbound IR messages isn't a supported common A/C protocol");
    return false;
  }
  if (climate[0] == NULL) {
    debug("No common A/C object allocated for channel 0. Skipping.");
    return false;
  }
  stdAc::state_t state = climate[0]->next;
  debug("Converting inbound IR A/C message to common A/C");
  if (!IRAcUtils::decodeToState(decode, &state, &(climate[0]->next))) {
      debug("Failed to convert to common A/C.");  // This shouldn't happen!
      return false;
  }
#if IGNORE_DECODED_AC_PROTOCOL
  if (climate[0]->next.protocol != decode_type_t::UNKNOWN) {
    // Use the previous protocol/model if set.
    state.protocol = climate[0]->next.protocol;
    state.model = climate[0]->next.model;
  }
#endif  // IGNORE_DECODED_AC_PROTOCOL
  // Continue to use the previously prefered temperature units.
  // i.e. Keep using Celsius or Fahrenheit.
  if (climate[0]->next.celsius != state.celsius) {
    // We've got a mismatch, so we need to convert.
    state.degrees = climate[0]->next.celsius ?
        fahrenheitToCelsius(state.degrees) : celsiusToFahrenheit(state.degrees);
    state.celsius = climate[0]->next.celsius;
  }
  climate[0]->next = state;  // Copy over the new climate state.

  return true;
}
#endif  // USE_DECODED_AC_SETTINGS && IR_RX
