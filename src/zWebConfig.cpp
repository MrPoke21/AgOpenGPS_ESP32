#include "zWebConfig.h"
#include "zUDP.h"
#include "zDebugLog.h"
#include <Preferences.h>
#include <WebServer.h>

WiFiRuntimeConfig wifiRuntimeConfig;

static const char* NVS_NAMESPACE = "wificfg";
static Preferences prefs;
static WebServer configServer(WEB_SERVER_PORT);

static bool restartPending = false;
static uint32_t restartAtMs = 0;

// DEBUG ON toggle auto-disables if the browser stops sending heartbeats
// (page closed/navigated away/connection lost) - never persisted, always starts OFF.
static const uint32_t DEBUG_HEARTBEAT_TIMEOUT_MS = 4000;
static uint32_t debugHeartbeatAtMs = 0;

static void webServerTask(void* parameters);

static void copyBounded(char* dest, size_t destSize, const char* src) {
  strncpy(dest, src, destSize - 1);
  dest[destSize - 1] = '\0';
}

void loadWiFiRuntimeConfig() {
  // Defaults from Configuration.h - used to seed NVS on first boot
  wifiRuntimeConfig.mode = WIFI_MODE;
  copyBounded(wifiRuntimeConfig.apSsid, sizeof(wifiRuntimeConfig.apSsid), WIFI_SSID);
  copyBounded(wifiRuntimeConfig.apPass, sizeof(wifiRuntimeConfig.apPass), WIFI_PASS);
  copyBounded(wifiRuntimeConfig.staSsid, sizeof(wifiRuntimeConfig.staSsid), WIFI_SSID);
  copyBounded(wifiRuntimeConfig.staPass, sizeof(wifiRuntimeConfig.staPass), WIFI_PASS);
  wifiRuntimeConfig.udpPort = UDP_PORT;
  wifiRuntimeConfig.channel = WIFI_CHANNEL;

  prefs.begin(NVS_NAMESPACE, true);  // read-only
  bool configured = prefs.getBool("init", false);
  if (configured) {
    wifiRuntimeConfig.mode = prefs.getUChar("mode", wifiRuntimeConfig.mode);
    prefs.getString("apssid", wifiRuntimeConfig.apSsid, sizeof(wifiRuntimeConfig.apSsid));
    prefs.getString("appass", wifiRuntimeConfig.apPass, sizeof(wifiRuntimeConfig.apPass));
    prefs.getString("stassid", wifiRuntimeConfig.staSsid, sizeof(wifiRuntimeConfig.staSsid));
    prefs.getString("stapass", wifiRuntimeConfig.staPass, sizeof(wifiRuntimeConfig.staPass));
    wifiRuntimeConfig.udpPort = prefs.getUShort("udpport", wifiRuntimeConfig.udpPort);
    wifiRuntimeConfig.channel = prefs.getUChar("channel", wifiRuntimeConfig.channel);
  }
  prefs.end();

  if (!configured) {
    saveWiFiRuntimeConfig();  // seed NVS so next boot reads back the same defaults
  }
}

void saveWiFiRuntimeConfig() {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putBool("init", true);
  prefs.putUChar("mode", wifiRuntimeConfig.mode);
  prefs.putString("apssid", wifiRuntimeConfig.apSsid);
  prefs.putString("appass", wifiRuntimeConfig.apPass);
  prefs.putString("stassid", wifiRuntimeConfig.staSsid);
  prefs.putString("stapass", wifiRuntimeConfig.staPass);
  prefs.putUShort("udpport", wifiRuntimeConfig.udpPort);
  prefs.putUChar("channel", wifiRuntimeConfig.channel);
  prefs.end();
}

// Escape a value before embedding it in an HTML attribute/text node
static String htmlEscape(const char* s) {
  String out;
  for (const char* p = s; *p; ++p) {
    switch (*p) {
      case '&':  out += "&amp;";  break;
      case '<':  out += "&lt;";   break;
      case '>':  out += "&gt;";   break;
      case '"':  out += "&quot;"; break;
      default:   out += *p;
    }
  }
  return out;
}

static String channelOptions(uint8_t selected) {
  String out;
  for (uint8_t ch = 1; ch <= 13; ch++) {
    out += "<option value='" + String(ch) + "'" + (ch == selected ? " selected" : "") + ">" + String(ch) +
           ((ch == 1 || ch == 6 || ch == 11) ? " (ajanlott)" : "") + "</option>";
  }
  return out;
}

static void handleRoot() {
  String statusLine;
  if (wifiRuntimeConfig.mode == 1) {
    statusLine = "AP mod aktiv - IP: " + WiFi.softAPIP().toString() +
                 ", csatlakozott kliensek: " + String(getWiFiClientCount());
  } else if (getWiFiStatus() == WIFI_STA_CONNECTED) {
    statusLine = "Kliens mod - csatlakozva, IP: " + WiFi.localIP().toString();
  } else {
    statusLine = "Kliens mod - nincs kapcsolat a megadott halozathoz";
  }

  String page =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<title>AgOpenGPS ESP32 - WiFi beallitasok</title>"
    "<style>body{font-family:sans-serif;max-width:480px;margin:20px auto;padding:0 10px}"
    "fieldset{margin-bottom:16px}label{display:block;margin-top:8px}"
    "input[type=text],input[type=password],input[type=number],select{width:100%;box-sizing:border-box;padding:6px;margin-top:2px}"
    "button{margin-top:16px;padding:10px 20px;font-size:1em}</style></head><body>"
    "<h2>AgOpenGPS ESP32 - WiFi beallitasok</h2>"
    "<p><b>Allapot:</b> " + statusLine + "</p>"
    "<form method='POST' action='/save'>"
    "<fieldset><legend>Uzemmod</legend>"
    "<label><input type='radio' name='mode' value='1'" + String(wifiRuntimeConfig.mode == 1 ? " checked" : "") +
      "> Access Point (az eszkoz sajat halozatot hoz letre)</label>"
    "<label><input type='radio' name='mode' value='0'" + String(wifiRuntimeConfig.mode == 0 ? " checked" : "") +
      "> Kliens (csatlakozas meglevo halozathoz)</label>"
    "</fieldset>"
    "<fieldset><legend>Access Point beallitasok</legend>"
    "<label>SSID<input type='text' name='ap_ssid' maxlength='32' value='" + htmlEscape(wifiRuntimeConfig.apSsid) + "'></label>"
    "<label>Jelszo<input type='password' name='ap_pass' maxlength='63' value='" + htmlEscape(wifiRuntimeConfig.apPass) + "'></label>"
    "<label>WiFi csatorna (utkozes eseten probalj masikat)<select name='channel'>" + channelOptions(wifiRuntimeConfig.channel) + "</select></label>"
    "</fieldset>"
    "<fieldset><legend>Kliens (STA) beallitasok</legend>"
    "<label>SSID<input type='text' name='sta_ssid' maxlength='32' value='" + htmlEscape(wifiRuntimeConfig.staSsid) + "'></label>"
    "<label>Jelszo<input type='password' name='sta_pass' maxlength='63' value='" + htmlEscape(wifiRuntimeConfig.staPass) + "'></label>"
    "</fieldset>"
    "<fieldset><legend>Halozat</legend>"
    "<label>UDP port<input type='number' name='udp_port' min='1' max='65535' value='" + String(wifiRuntimeConfig.udpPort) + "'></label>"
    "</fieldset>"
    "<button type='submit'>Mentes es ujrainditas</button>"
    "</form>"
    "<h3>Debug log <label style='font-size:0.6em;font-weight:normal'>"
    "<input type='checkbox' id='dbgtoggle'> DEBUG ON</label></h3>"
    "<input type='text' id='logfilter' placeholder='Szuro (regex, ures = mind)'>"
    "<pre id='logbox' style='background:#111;color:#0f0;height:240px;overflow-y:auto;"
    "padding:8px;font-size:12px;white-space:pre-wrap;word-break:break-all;'></pre>"
    "<script>"
    "var logCursor=0;var logLines=[];var pending='';"
    "var logBox=document.getElementById('logbox');"
    "var logFilter=document.getElementById('logfilter');"
    "function renderLog(){"
      "var re=null;"
      "if(logFilter.value.length>0){"
        "try{re=new RegExp(logFilter.value,'i');logFilter.style.borderColor='';}"
        "catch(e){re=null;logFilter.style.borderColor='red';}"
      "}else{logFilter.style.borderColor='';}"
      "var out=[];"
      "for(var i=0;i<logLines.length;i++){if(!re||re.test(logLines[i]))out.push(logLines[i]);}"
      "logBox.textContent=out.join('\\n');"
      "logBox.scrollTop=logBox.scrollHeight;"
    "}"
    "function addChunk(t){"
      "pending+=t;"
      "var parts=pending.split('\\n');"
      "pending=parts.pop();"
      "for(var i=0;i<parts.length;i++)logLines.push(parts[i]);"
      "if(logLines.length>1000)logLines.splice(0,logLines.length-1000);"
      "renderLog();"
    "}"
    "function pollLog(){"
      "fetch('/log?after='+logCursor).then(function(r){"
        "var cur=r.headers.get('X-Log-Cursor');if(cur!==null)logCursor=parseInt(cur,10);"
        "return r.text();"
      "}).then(function(t){"
        "if(t.length>0)addChunk(t);"
      "}).catch(function(){});"
    "}"
    "logFilter.addEventListener('input',renderLog);"
    "setInterval(pollLog,500);pollLog();"
    "var dbgToggle=document.getElementById('dbgtoggle');var dbgHeartbeat=null;"
    "function dbgSet(on){"
      "fetch('/debug?on='+(on?1:0)).then(function(r){return r.text();}).then(function(t){dbgToggle.checked=(t==='1');});"
    "}"
    "function dbgPing(){dbgSet(true);}"
    "dbgToggle.addEventListener('change',function(){"
      "if(dbgToggle.checked){dbgPing();dbgHeartbeat=setInterval(dbgPing,1500);}"
      "else{dbgSet(false);if(dbgHeartbeat){clearInterval(dbgHeartbeat);dbgHeartbeat=null;}}"
    "});"
    "window.addEventListener('pagehide',function(){navigator.sendBeacon('/debug?on=0');});"
    "fetch('/debug').then(function(r){return r.text();}).then(function(t){"
      "dbgToggle.checked=(t==='1');"
      "if(dbgToggle.checked)dbgHeartbeat=setInterval(dbgPing,1500);"
    "});"
    "</script>"
    "</body></html>";

  configServer.send(200, "text/html; charset=utf-8", page);
}

static void handleLog() {
  uint32_t after = configServer.hasArg("after") ? (uint32_t)configServer.arg("after").toInt() : 0;

  static const size_t CHUNK = 2048;
  char buf[CHUNK + 1];
  size_t n = DebugLog.readNew(buf, CHUNK, after);
  buf[n] = '\0';

  configServer.sendHeader("X-Log-Cursor", String(after));
  configServer.send(200, "text/plain; charset=utf-8", buf);
}

static void handleDebugToggle() {
  if (configServer.hasArg("on")) {
    bool on = configServer.arg("on").toInt() != 0;
    setDebugEnabled(on);
    if (on) {
      debugHeartbeatAtMs = millis();
    }
  }
  configServer.send(200, "text/plain", isDebugEnabled() ? "1" : "0");
}

static void handleSave() {
  if (!configServer.hasArg("mode")) {
    configServer.send(400, "text/plain", "Missing mode");
    return;
  }

  wifiRuntimeConfig.mode = (configServer.arg("mode").toInt() == 1) ? 1 : 0;

  // Keep the previous SSID if the submitted one is blank (avoid locking the device out of WiFi)
  String apSsid = configServer.arg("ap_ssid");
  if (apSsid.length() > 0) {
    apSsid.toCharArray(wifiRuntimeConfig.apSsid, sizeof(wifiRuntimeConfig.apSsid));
  }
  configServer.arg("ap_pass").toCharArray(wifiRuntimeConfig.apPass, sizeof(wifiRuntimeConfig.apPass));

  String staSsid = configServer.arg("sta_ssid");
  if (staSsid.length() > 0) {
    staSsid.toCharArray(wifiRuntimeConfig.staSsid, sizeof(wifiRuntimeConfig.staSsid));
  }
  configServer.arg("sta_pass").toCharArray(wifiRuntimeConfig.staPass, sizeof(wifiRuntimeConfig.staPass));

  long port = configServer.arg("udp_port").toInt();
  wifiRuntimeConfig.udpPort = (port >= 1 && port <= 65535) ? (uint16_t)port : UDP_PORT;

  long channel = configServer.arg("channel").toInt();
  wifiRuntimeConfig.channel = (channel >= 1 && channel <= 13) ? (uint8_t)channel : WIFI_CHANNEL;

  saveWiFiRuntimeConfig();

  configServer.send(200, "text/html; charset=utf-8",
    "<!DOCTYPE html><html><head><meta charset='utf-8'></head><body>"
    "<p>Beallitasok mentve. Az eszkoz ujraindul...</p></body></html>");

  // Delay the restart so the HTTP response above has time to flush to the client
  restartPending = true;
  restartAtMs = millis() + 800;
}

bool initWiFiConfigPortal() {
  loadWiFiRuntimeConfig();

  bool ok = initWiFi();  // starts AP or STA per wifiRuntimeConfig.mode

  configServer.on("/", HTTP_GET, handleRoot);
  configServer.on("/save", HTTP_POST, handleSave);
  configServer.on("/log", HTTP_GET, handleLog);
  configServer.on("/debug", handleDebugToggle);
  configServer.onNotFound([]() {
    configServer.send(404, "text/plain", "Not found");
  });
  configServer.begin();

  // Serviced from its own Core 0 task (see webServerTask) - never called from
  // the shared loop() on Core 1, so a slow HTTP request can't stall
  // autosteerLoop()/gpsStream() timing.
  xTaskCreatePinnedToCore(
    webServerTask,
    "webConfig",
    8192,
    NULL,
    1,  // Low priority - well below udpSendTask(10)/autoSteerPacketPerser(25)
    NULL,
    0   // Core 0
  );

  return ok;
}

// Runs on its own Core 0 task (see initWiFiConfigPortal) - isolated from the
// Core 1 loop() so a slow HTTP request can never stall the autosteer/GPS timing.
static void webServerTask(void* parameters) {
  for (;;) {
    configServer.handleClient();

    // No heartbeat within the timeout window (page closed/navigated away/lost) - auto-disable
    if (isDebugEnabled() && (millis() - debugHeartbeatAtMs > DEBUG_HEARTBEAT_TIMEOUT_MS)) {
      setDebugEnabled(false);
    }

    if (restartPending && (int32_t)(millis() - restartAtMs) >= 0) {
      ESP.restart();
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
