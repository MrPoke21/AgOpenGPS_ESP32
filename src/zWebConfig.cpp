#include "zWebConfig.h"
#include "zUDP.h"
#include "zDebugLog.h"
#include "main.h"
#include "zInput.h"
#include "zHandlers.h"
#include "AutosteerPID.h"
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

// Copy an NMEA-derived char buffer keeping only display-safe characters.
// The values come from the GPS parser (digits/letters/dot/dash), this just
// guards the JSON string structure and limits the copy to the array size in
// case the buffer is not NUL-terminated.
static String jsonSafeText(const char* src, size_t maxLen) {
  String out;
  for (size_t i = 0; i < maxLen && src[i] != '\0'; i++) {
    char c = src[i];
    bool ok = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') ||
              (c >= 'a' && c <= 'z') || c == '.' || c == '-' || c == '+' || c == ' ';
    if (ok) out += c;
  }
  return out;
}

// Live telemetry snapshot for the web UI Telemetria tab (polled every 10 s by
// the browser, starts with the AUTOTUNE Kd value). Values are written by the
// Core 1 control loop and read here from the Core 0 web server task -
// individually aligned 32-bit loads are atomic on the ESP32, so at worst a
// single field can show a one-tick-stale value, which is fine for a monitor.
static void handleStatus() {
  String j;
  j.reserve(1400);

  // ── AUTOTUNE / PID (first - the tuning-relevant values) ─────────────────────
  j = "{\"kp\":" + String(steerSettings.Kp);
#ifdef USE_AUTOTUNE_PID
  j += ",\"kd\":" + String(getLearnedKd(), 1);
  j += ",\"ki\":" + String(KI_GAIN, 1);
  j += ",\"ival\":" + String(KI_GAIN * getIntegralError(), 1);
#else
  j += ",\"kd\":0,\"ki\":0,\"ival\":0";
#endif
  j += ",\"pval\":" + String(pValue, 1);
  j += ",\"err\":" + String(errorAbs, 2);
  j += ",\"pwm\":" + String(pwmDrive);

  // ── Autosteer state ─────────────────────────────────────────────────────────
  j += ",\"en\":" + String(steerEnable ? 1 : 0);
  j += ",\"angA\":" + String(steerAngleActual, 2);
  j += ",\"angS\":" + String(steerAngleSetPoint, 2);
  j += ",\"guid\":" + String(guidanceStatus);
  j += ",\"feAge\":" + String(millis() - lastFEPacketTime);

  // ── Sensors (WAS + current) ────────────────────────────────────────────────
  j += ",\"adc\":" + String(adcConnected ? 1 : 0);
  j += ",\"wasRaw\":" + String(filteredSteeringSensor);
  j += ",\"wasPos\":" + String(steeringPosition);
  j += ",\"hello\":" + String(helloSteerPosition);
  j += ",\"currRaw\":" + String(filteredCurrentSensor);
  j += ",\"czero\":" + String(current_zero);
  j += ",\"sens\":" + String(sensorReading, 1);

  // ── GPS / IMU ──────────────────────────────────────────────────────────────
  j += ",\"gga\":" + String(GGA_Available ? 1 : 0);
  j += ",\"fixt\":\"" + jsonSafeText(fixTime, sizeof(fixTime)) + "\"";
  j += ",\"sats\":\"" + jsonSafeText(numSats, sizeof(numSats)) + "\"";
  j += ",\"fixq\":\"" + jsonSafeText(fixQuality, sizeof(fixQuality)) + "\"";
  j += ",\"hdop\":\"" + jsonSafeText(HDOP, sizeof(HDOP)) + "\"";
  j += ",\"pos\":\"" + jsonSafeText(latitude, sizeof(latitude)) + jsonSafeText(latNS, sizeof(latNS)) +
       " " + jsonSafeText(longitude, sizeof(longitude)) + jsonSafeText(lonEW, sizeof(lonEW)) + "\"";
  j += ",\"speed\":" + String(gpsSpeed, 1);
  j += ",\"imu\":" + String(useBNO08x ? 1 : 0);
  j += ",\"yaw\":" + String(ypr.yaw, 1);
  j += ",\"roll\":" + String(ypr.roll, 1);
  j += ",\"pitch\":" + String(ypr.pitch, 1);

  // ── Switches ───────────────────────────────────────────────────────────────
  j += ",\"sw\":" + String(steerSwitch);
  j += ",\"work\":" + String(workSwitch);

  // ── AgOpenGPS-provided steering settings (context for the PID values) ──────
  j += ",\"lowpwm\":" + String(steerSettings.lowPWM);
  j += ",\"highpwm\":" + String(steerSettings.highPWM);
  j += ",\"minpwm\":" + String(steerSettings.minPWM);
  j += ",\"scount\":" + String(steerSettings.steerSensorCounts, 1);
  j += ",\"woff\":" + String(steerSettings.wasOffset);
  j += ",\"ack\":" + String(steerSettings.AckermanFix, 2);

  // ── System / WiFi health ───────────────────────────────────────────────────
  j += ",\"mode\":\"" + String(wifiRuntimeConfig.mode == 1 ? "AP" : "Kliens") + "\"";
  j += ",\"ip\":\"" + ((wifiRuntimeConfig.mode == 1) ? WiFi.softAPIP() : WiFi.localIP()).toString() + "\"";
  j += ",\"clients\":" + String(getWiFiClientCount());
  j += ",\"rssi\":" + String((wifiRuntimeConfig.mode == 0 && WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0);
  j += ",\"udp\":" + String(wifiRuntimeConfig.udpPort);
  j += ",\"heap\":" + String(ESP.getFreeHeap());
  j += ",\"minheap\":" + String(ESP.getMinFreeHeap());
  j += ",\"temp\":" + String(temperatureRead(), 1);
  j += ",\"up\":" + String(millis() / 1000UL);

  j += "}";

  configServer.send(200, "application/json; charset=utf-8", j);
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
    "<title>AgOpenGPS ESP32</title>"
    "<style>body{font-family:sans-serif;max-width:480px;margin:20px auto;padding:0 10px}"
    ".tabbar{display:flex;gap:4px}"
    ".tabbtn{flex:1;padding:10px 8px;font-size:1em;border:1px solid #ccc;border-bottom:none;"
    "background:#eee;border-radius:6px 6px 0 0;cursor:pointer;margin-top:0}"
    ".tabbtn.active{background:#fff;font-weight:bold}"
    ".tabpage{border:1px solid #ccc;padding:10px;border-radius:0 0 6px 6px;margin-bottom:16px}"
    "fieldset{margin-bottom:16px}label{display:block;margin-top:8px}"
    "input[type=text],input[type=password],input[type=number],select{width:100%;box-sizing:border-box;padding:6px;margin-top:2px}"
    "button{margin-top:16px;padding:10px 20px;font-size:1em}"
    "table{width:100%;border-collapse:collapse;font-size:0.95em}"
    "td{padding:4px 6px;border-bottom:1px solid #eee;vertical-align:top}"
    "td.v{text-align:right;font-family:monospace;white-space:nowrap}"
    "tr.hl td{background:#fff3cd;font-weight:bold}"
    ".on{color:#0a0;font-weight:bold}.off{color:#a00;font-weight:bold}"
    "#statMsg{color:#a00;font-weight:bold}"
    "h4{margin:12px 0 4px;color:#235}"
    "</style></head><body>"
    "<h2>AgOpenGPS ESP32</h2>"
    "<p><b>Allapot:</b> " + statusLine + "</p>"
    "<div class='tabbar'>"
    "<button type='button' class='tabbtn active' id='btn-page-cfg'>Beallitasok</button>"
    "<button type='button' class='tabbtn' id='btn-page-status'>Telemetria</button>"
    "</div>"
    "<div id='page-cfg' class='tabpage'>"
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
    "</div>"
    "<div id='page-status' class='tabpage' style='display:none'>"
    "<h3 style='margin-top:0'>Telemetria (10 masodpercenkenti frissites)</h3>"
    "<p>Utolso frissites: <span id='statUpdated'>-</span>"
    "<button type='button' id='statNow' style='margin:0 0 0 8px;padding:4px 10px;font-size:0.9em'>Frissites most</button></p>"
    "<p id='statMsg'></p>"
    "<div id='statusBox'></div>"
    "</div>"
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
    // ── Telemetria tab: 10 s-onkenti ertek-frissites /status JSON-bol ──────────
    "var F=["
    "{s:'AUTOTUNE / PID',hl:1,k:'kd',l:'AUTOTUNE Kd (ontanult D tag)',f:'n1'},"
    "{s:'AUTOTUNE / PID',k:'kp',l:'Kp (aranyos tag)',f:'n0'},"
    "{s:'AUTOTUNE / PID',k:'ki',l:'Ki erosites (fix)',f:'n1'},"
    "{s:'AUTOTUNE / PID',k:'ival',l:'I tag kimenet',f:'n1'},"
    "{s:'AUTOTUNE / PID',k:'pval',l:'P tag kimenet',f:'n1'},"
    "{s:'AUTOTUNE / PID',k:'err',l:'Szoghiba (abszolut)',f:'n2',u:'\\u00b0'},"
    "{s:'AUTOTUNE / PID',k:'pwm',l:'PWM kimenet (pwmDrive)',f:'n0'},"
    "{s:'Autokormany',k:'en',l:'Autokormany allapota',f:'b'},"
    "{s:'Autokormany',k:'angA',l:'WAS szog (tenyleges)',f:'n2',u:'\\u00b0'},"
    "{s:'Autokormany',k:'angS',l:'Celszog (AgOpenGPS)',f:'n2',u:'\\u00b0'},"
    "{s:'Autokormany',k:'guid',l:'Guidance statusz (bajt)',f:'n0'},"
    "{s:'Autokormany',k:'feAge',l:'Utso AgIO csomag ota',f:'ms'},"
    "{s:'Erzekelok (ADC)',k:'adc',l:'ADC (ADS1115) elerheto',f:'yn'},"
    "{s:'Erzekelok (ADC)',k:'wasRaw',l:'WAS szurt ADC ertek',f:'n0'},"
    "{s:'Erzekelok (ADC)',k:'wasPos',l:'WAS pozicio (ADC/2)',f:'n0'},"
    "{s:'Erzekelok (ADC)',k:'hello',l:'WAS hello pozicio',f:'n0'},"
    "{s:'Erzekelok (ADC)',k:'currRaw',l:'Aram szenzor ADC (szurt)',f:'n0'},"
    "{s:'Erzekelok (ADC)',k:'czero',l:'Aram nulla pont (auto-zero)',f:'n0'},"
    "{s:'Erzekelok (ADC)',k:'sens',l:'sensorReading (0-255)',f:'n1'},"
    "{s:'GPS / IMU',k:'gga',l:'GGA mondat erkezik',f:'yn'},"
    "{s:'GPS / IMU',k:'fixt',l:'GPS ido',f:'s'},"
    "{s:'GPS / IMU',k:'sats',l:'Muholdak szama',f:'s'},"
    "{s:'GPS / IMU',k:'fixq',l:'Fix minoseg',f:'s'},"
    "{s:'GPS / IMU',k:'hdop',l:'HDOP',f:'s'},"
    "{s:'GPS / IMU',k:'pos',l:'Pozicio',f:'s'},"
    "{s:'GPS / IMU',k:'speed',l:'Sebesseg',f:'n1',u:'km/h'},"
    "{s:'GPS / IMU',k:'imu',l:'BNO08x IMU elerheto',f:'yn'},"
    "{s:'GPS / IMU',k:'yaw',l:'IMU yaw (irany)',f:'n1',u:'\\u00b0'},"
    "{s:'GPS / IMU',k:'roll',l:'IMU roll',f:'n1',u:'\\u00b0'},"
    "{s:'GPS / IMU',k:'pitch',l:'IMU pitch',f:'n1',u:'\\u00b0'},"
    "{s:'Kapcsolok',k:'sw',l:'Kormany kapcsolo (1=engedelyezve)',f:'b'},"
    "{s:'Kapcsolok',k:'work',l:'Munka kapcsolo',f:'b'},"
    "{s:'Beallitasok (AgOpenGPS)',k:'lowpwm',l:'LowPWM (holtsav)',f:'n0'},"
    "{s:'Beallitasok (AgOpenGPS)',k:'highpwm',l:'HighPWM (maximum)',f:'n0'},"
    "{s:'Beallitasok (AgOpenGPS)',k:'minpwm',l:'MinPWM (indito PWM)',f:'n0'},"
    "{s:'Beallitasok (AgOpenGPS)',k:'scount',l:'SteerSensorCounts',f:'n1'},"
    "{s:'Beallitasok (AgOpenGPS)',k:'woff',l:'WAS offset',f:'n0'},"
    "{s:'Beallitasok (AgOpenGPS)',k:'ack',l:'Ackerman fix',f:'n2'},"
    "{s:'Rendszer',k:'mode',l:'WiFi uzemmod',f:'s'},"
    "{s:'Rendszer',k:'ip',l:'IP cim',f:'s'},"
    "{s:'Rendszer',k:'clients',l:'Csatlakozott WiFi kliensek',f:'n0'},"
    "{s:'Rendszer',k:'rssi',l:'WiFi jeloerosseg (RSSI)',f:'rssi'},"
    "{s:'Rendszer',k:'udp',l:'UDP port',f:'n0'},"
    "{s:'Rendszer',k:'heap',l:'Szabad RAM (heap)',f:'n0',u:'B'},"
    "{s:'Rendszer',k:'minheap',l:'Minimum szabad RAM',f:'n0',u:'B'},"
    "{s:'Rendszer',k:'temp',l:'CPU homerseklet',f:'n1',u:'\\u00b0C'},"
    "{s:'Rendszer',k:'up',l:'Futasi ido',f:'dur'}"
    "];"
    "var statBox=document.getElementById('statusBox');"
    "var statMsg=document.getElementById('statMsg');"
    "var statUpdated=document.getElementById('statUpdated');"
    "function fmtDur(sec){"
      "sec=Math.round(sec);var d=Math.floor(sec/86400);sec-=d*86400;"
      "var h=Math.floor(sec/3600);sec-=h*3600;var m=Math.floor(sec/60);sec-=m*60;"
      "var p=function(n){return(n<10?'0':'')+n;};"
      "return(d>0?d+' nap ':'')+p(h)+':'+p(m)+':'+p(sec);"
    "}"
    "function fmtVal(d,f){"
      "var v=d[f.k];"
      "if(v===undefined||v===null)return'-';"
      "if(f.f==='b')return v?'BE':'KI';"
      "if(f.f==='yn')return v?'Igen':'Nem';"
      "if(f.f==='s')return String(v);"
      "if(f.f==='ms')return Math.round(v)+' ms';"
      "if(f.f==='rssi')return v?Math.round(v)+' dBm':'-';"
      "if(f.f==='dur')return fmtDur(v);"
      "var n=Number(v);if(isNaN(n))return String(v);"
      "var dec=(f.f==='n2')?2:(f.f==='n1'?1:0);"
      "return n.toFixed(dec)+(f.u?' '+f.u:'');"
    "}"
    "function buildStatus(){"
      "var html='';var cur=null;var open=false;"
      "for(var i=0;i<F.length;i++){"
        "var f=F[i];"
        "if(f.s!==cur){"
          "if(open)html+='</table>';"
          "html+='<h4>'+f.s+'</h4><table>';"
          "cur=f.s;open=true;"
        "}"
        "html+='<tr'+(f.hl?' class=hl':'')+'><td>'+f.l+'</td><td class=v id=f_'+f.k+'>-</td></tr>';"
      "}"
      "if(open)html+='</table>';"
      "statBox.innerHTML=html;"
    "}"
    "function pollStatus(){"
      "fetch('/status').then(function(r){"
        "if(!r.ok)throw 0;"
        "return r.json();"
      "}).then(function(d){"
        "statMsg.textContent='';"
        "statUpdated.textContent=new Date().toLocaleTimeString();"
        "for(var i=0;i<F.length;i++){"
          "var f=F[i];var el=document.getElementById('f_'+f.k);"
          "if(!el)continue;"
          "var v=d[f.k];"
          "el.textContent=fmtVal(d,f);"
          "el.className='v';"
          "if(f.f==='b'||f.f==='yn'){el.className='v '+(v?'on':'off');}"
          "else if(f.k==='feAge'){el.className='v '+(v>1000?'off':'on');}"
        "}"
      "}).catch(function(){"
        "statMsg.textContent='Nincs kapcsolat - az ertekek nem frissulnek';"
      "});"
    "}"
    "function showTab(id){"
      "var pages=document.querySelectorAll('.tabpage');"
      "for(var i=0;i<pages.length;i++)pages[i].style.display='none';"
      "var btns=document.querySelectorAll('.tabbtn');"
      "for(var i=0;i<btns.length;i++)btns[i].classList.remove('active');"
      "document.getElementById(id).style.display='block';"
      "document.getElementById('btn-'+id).classList.add('active');"
      "if(id==='page-status')pollStatus();"
    "}"
    "document.getElementById('btn-page-cfg').addEventListener('click',function(){showTab('page-cfg');});"
    "document.getElementById('btn-page-status').addEventListener('click',function(){showTab('page-status');});"
    "document.getElementById('statNow').addEventListener('click',pollStatus);"
    "buildStatus();"
    "pollStatus();"
    "setInterval(pollStatus,10000);"
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
  configServer.on("/status", HTTP_GET, handleStatus);
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
