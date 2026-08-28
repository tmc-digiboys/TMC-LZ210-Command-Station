#include "webserver.h"
#include "xpressnet_handler.h"   // gAccessories, MAX_ACCESSORIES, gCentrale
#include "dcc_hal.h"              // gDccHal
#include "power_monitor.h"        // gPowerMonitor
#include "rs_bus_hal.h"           // gRsBusHal
#include "lenz_lan.h"             // gLenzLan
#include "z21_lan.h"              // gZ21Lan
#include "trace_log.h"             // traceLog()
#include "loco_repository.h"      // locoRepo()
#include "eeprom_store.h"         // eepromStore()
#include <stdio.h>
#include <string.h>

Webserver gWebserver;

// begin() — starts listening for incoming HTTP connections on the
// configured port (EEPROM key "web.port", web interface, default
// WEB_PORT) and resets the active-client counter to 0.
void Webserver::begin() {
    uint16_t port = eepromStore().getUint16("web.port", WEB_PORT);
    _server = new EthernetServer(port);
    _server->begin();
    _activeClients = 0;
}

// loop() — accepts at most one new client connection per call. If a
// client is waiting, increments the active-client counter, handles
// its entire request synchronously via _handleClient() (this server
// is fully blocking/single-request-at-a-time per call, relying on
// loop() being called frequently enough by the module registry), then
// closes the connection and decrements the counter again.
void Webserver::loop() {
    EthernetClient client = _server->accept();
    if (client) {
        _activeClients++;
        _handleClient(client);
        client.stop();
        _activeClients--;
    }
}

// ─────────────────────────────────────────────────────────────
//  Client handling
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
//  _handleClient() — parse one HTTP request and route it
//
//  Parses the request via _parseRequest(); if parsing failed or the
//  request was otherwise invalid, replies with a 404 and returns.
//
//  For GET requests: serves the stylesheet, the JSON status endpoint,
//  or the main page (extracting the `?p=` query parameter to select
//  which panel to render, defaulting to "internet" if none was
//  given). Any other GET path is redirected to "/".
//
//  For POST requests: dispatches to the matching action based on
//  path —
//    /config          → save submitted form values via _handleConfigPost()
//    /reset           → reset every parameter to its registered default
//                        and commit to flash, then redirect back to "/"
//    /fabriek         → factory reset: reads the raw request body itself
//                        (rather than going through the already-parsed
//                        HttpRequest) looking for the confirmation
//                        checkbox ("bevestig=1"); if present, resets all
//                        parameters, clears every turnout's known/state,
//                        clears the loco repository, and clears the
//                        RS-Bus cache — then redirects back to the
//                        factory-reset panel regardless of whether the
//                        checkbox was actually present, so the user sees
//                        the same panel either way
//    /clear/turnouts  → clears all turnout known/state flags and
//                        persists the (now-empty) accessory table
//    /clear/locos     → clears the loco repository
//    /clear/feedback  → clears the RS-Bus feedback cache
//  Any other POST path, or any method other than GET/POST, falls
//  through to a 404 response.
// ─────────────────────────────────────────────────────────────
void Webserver::_handleClient(EthernetClient& client) {
    HttpRequest req;
    if (!_parseRequest(client, req) || !req.valid) {
        _handle404(client);
        return;
    }

    if (strcmp(req.method, "GET") == 0) {
        if (_pathEquals(req.path, "/style.css")) { _handleCss(client); return; }
        if (_pathEquals(req.path, "/status"))    { _handleStatus(client); return; }
        if (_pathEquals(req.path, "/")) {
            // Read the ?p= parameter to select the panel
            char panel[32] = "internet";
            const char* pp = strstr(req.path, "p=");
            if (pp) {
                pp += 2;
                uint8_t pi = 0;
                while (*pp && *pp != '&' && *pp != ' ' && pi < 31)
                    panel[pi++] = *pp++;
                panel[pi] = 0;
            }
            _handleRoot(client, panel);
            return;
        }
        _sendRedirect(client, "/");
        return;
    }
    if (strcmp(req.method, "POST") == 0) {
        if (_pathEquals(req.path, "/config")) {
            _handleConfigPost(client, req); return;
        }
        if (_pathEquals(req.path, "/reset")) {
            eepromStore().resetToDefaults();
            eepromStore().commit();
            _sendRedirect(client, "/");
            return;
        }
        if (_pathEquals(req.path, "/fabriek")) {
            // Look for the confirmation checkbox in the body
            char body[64] = {};
            uint8_t bi = 0;
            while (client.available() && bi < sizeof(body)-1)
                body[bi++] = client.read();
            if (strstr(body, "bevestig=1")) {
                eepromStore().resetToDefaults();
                eepromStore().commit();
                for (uint16_t i = 0; i < MAX_ACCESSORIES; i++) {
                    gAccessories[i].known = false;
                    gAccessories[i].state = 0;
                }
                locoRepo().clearAll();
                gRsBusHal.clearAll();
            }
            _sendRedirect(client, "/?p=fabriek");
            return;
        }
        if (_pathEquals(req.path, "/clear/turnouts")) {
            for (uint16_t i = 0; i < MAX_ACCESSORIES; i++) {
                gAccessories[i].known = false;
                gAccessories[i].state = 0;
            }
            eepromStore().saveAccessories();
            _sendRedirect(client, "/?p=turnouts");
            return;
        }
        if (_pathEquals(req.path, "/clear/locos")) {
            locoRepo().clearAll();
            _sendRedirect(client, "/?p=locos");
            return;
        }
        if (_pathEquals(req.path, "/clear/feedback")) {
            gRsBusHal.clearAll();
            _sendRedirect(client, "/?p=feedback");
            return;
        }
    }
    _handle404(client);
}

// ─────────────────────────────────────────────────────────────
//  _parseRequest() — read and parse one HTTP request from the socket
//
//  Waits up to 1000ms for the first byte of data to arrive; returns
//  false (leaving req.valid false) if nothing arrives in that time.
//
//  Reads the request line (up to 127 characters, stripping the
//  trailing \r\n) and splits it on spaces into the method and path
//  (the HTTP version token after the second space is discarded).
//  Returns false if either space separator is missing (malformed
//  request line).
//
//  Reads and discards every subsequent header line until an empty
//  line is reached (marking the end of the headers), specifically
//  watching for a "Content-Length:" header to determine how large the
//  request body will be.
//
//  If a Content-Length was found and it fits within WEB_MAX_REQ_SIZE,
//  reads exactly that many bytes into req.body (waiting up to a
//  further 1000ms total for all of them to arrive), recording the
//  actual number of bytes read in req.bodyLen. A Content-Length of 0,
//  or one that would not fit in the fixed-size body buffer, simply
//  results in an empty body being used, without treating that as a
//  parse error.
//
//  Marks the request valid and returns true once the method and path
//  have been successfully parsed, regardless of whether a body was
//  present.
// ─────────────────────────────────────────────────────────────
bool Webserver::_parseRequest(EthernetClient& client, HttpRequest& req) {
    memset(&req, 0, sizeof(req));
    req.valid = false;

    // Wait up to 1000ms for data
    uint32_t t0 = millis();
    while (!client.available() && millis() - t0 < 1000) delay(1);
    if (!client.available()) return false;

    // Read the first line: METHOD PATH HTTP/1.x
    char line[128]; uint8_t li = 0;
    while (client.available() && li < 127) {
        char c = client.read();
        if (c == '\n') break;
        if (c != '\r') line[li++] = c;
    }
    line[li] = 0;

    // Parse method and path
    char* sp1 = strchr(line, ' ');
    if (!sp1) return false;
    *sp1 = 0;
    strncpy(req.method, line, 7);

    char* sp2 = strchr(sp1 + 1, ' ');
    if (!sp2) return false;
    *sp2 = 0;
    strncpy(req.path, sp1 + 1, 63);

    // Skip headers, looking for Content-Length
    uint16_t contentLength = 0;
    while (client.available()) {
        li = 0;
        while (client.available() && li < 127) {
            char c = client.read();
            if (c == '\n') break;
            if (c != '\r') line[li++] = c;
        }
        line[li] = 0;
        if (li == 0) break;  // empty line = end of headers
        if (strncmp(line, "Content-Length:", 15) == 0)
            contentLength = (uint16_t)atoi(line + 16);
    }

    // Read the body
    if (contentLength > 0 && contentLength < WEB_MAX_REQ_SIZE) {
        uint16_t read = 0;
        t0 = millis();
        while (read < contentLength && millis() - t0 < 1000) {
            if (client.available())
                req.body[read++] = client.read();
        }
        req.bodyLen = read;
    }

    req.valid = true;
    return true;
}

// ─────────────────────────────────────────────────────────────
//  Route handlers
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
//  _navItem() — sidebar navigation link helper
//
//  Renders one `<a>` link pointing at `/?p=<id>`, adding the "on"
//  CSS class if this item's id matches the currently active panel
//  (so the stylesheet can highlight it), with `label` as the link's
//  visible text.
// ─────────────────────────────────────────────────────────────
void Webserver::_navItem(EthernetClient& c, const char* id,
                          const char* label, const char* active) {
    c.print(F("<a class='s"));
    if (strcmp(id, active) == 0) c.print(F(" on"));
    c.print(F("' href='/?p="));
    c.print(id); c.print(F("'>"));
    c.print(label); c.print(F("</a>"));
}

// ─────────────────────────────────────────────────────────────
//  _printField() — print one configuration input field
//
//  Looks up the parameter definition for `key`; does nothing if it
//  is not registered. Prints the field's `<label>` (using the
//  caller-supplied, already human-readable text).
//
//  For ENUM parameters, renders a `<select>` populated from the
//  parameter's enumOpts array, marking whichever option matches the
//  parameter's current value as selected.
//
//  For BOOL parameters, renders a fixed two-option `<select>` (value
//  '1'/'0'), with the currently active option pre-selected.
//
//  For UINT8/UINT16/UINT32/STRING parameters, formats the current
//  value into a temporary buffer and renders it as a plain `<input
//  type='text'>`, with maxlength set to the parameter's registered
//  string size (for STRING) or a fixed 10 characters (for the numeric
//  types) to keep the field from accepting absurdly long input.
//  Unknown/unhandled parameter types are silently skipped (return
//  with nothing rendered beyond the label).
// ─────────────────────────────────────────────────────────────
void Webserver::_printField(EthernetClient& c, const char* key,
                             const char* label) {
    const ParamDef* p = eepromStore().findParam(key);
    if (!p) return;
    c.print(F("<label>")); c.print(label); c.print(F("</label>"));
    char val[64] = {};
    switch (p->type) {
        case ParamType::ENUM:
            c.print(F("<select name='")); c.print(key); c.print(F("'>"));
            for (uint8_t j = 0; p->enumOpts && p->enumOpts[j]; j++) {
                c.print(F("<option value='")); c.print(j); c.print(F("'"));
                if (p->value.u8 == j) c.print(F(" selected"));
                c.print(F(">")); c.print(p->enumOpts[j]);
                c.print(F("</option>"));
            }
            c.print(F("</select>")); return;
        case ParamType::BOOL:
            c.print(F("<select name='")); c.print(key); c.print(F("'>"));
            c.print(F("<option value='1'")); if (p->value.b) c.print(F(" selected"));
            c.print(F(">On</option><option value='0'"));
            if (!p->value.b) c.print(F(" selected"));
            c.print(F(">Off</option></select>")); return;
        case ParamType::UINT8:  snprintf(val,sizeof(val),"%u",p->value.u8);  break;
        case ParamType::UINT16: snprintf(val,sizeof(val),"%u",p->value.u16); break;
        case ParamType::UINT32: snprintf(val,sizeof(val),"%lu",p->value.u32);break;
        case ParamType::STRING: snprintf(val,sizeof(val),"%s",(const char*)p->value.bytes); break;
        default: return;
    }
    c.print(F("<input type='text' name='")); c.print(key);
    c.print(F("' value='")); c.print(val);
    c.print(F("' maxlength='"));
    c.print(p->type == ParamType::STRING ? p->size - 1 : 10);
    c.print(F("'>"));
}

// ─────────────────────────────────────────────────────────────
//  Root handler — one page with all panels and the sidebar
// ─────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────
//  CSS endpoint — /style.css
//
//  Serves the entire stylesheet as a series of print() calls (kept as
//  plain CSS text — class/id names are identifiers, not user-facing
//  text, so they are unchanged from the original).
// ─────────────────────────────────────────────────────────────
void Webserver::_handleCss(EthernetClient& c) {
    _sendHeader(c, 200, "text/css");
    c.print(F("body{margin:0;font-family:'Courier New',monospace;background:#0d0f12;color:#c8cdd5}"));
    c.print(F("*{box-sizing:border-box}"));
    c.print(F("#hdr{background:#111418;border-bottom:2px solid #f5820a;padding:0 20px;"));
    c.print(F("height:48px;display:flex;align-items:center;justify-content:space-between}"));
    c.print(F("#hdr h1{font-size:1rem;font-weight:700;letter-spacing:.12em;color:#f5820a;"));
    c.print(F("text-transform:uppercase;margin:0}#hdr .v{font-size:.6rem;color:#445}"));
    c.print(F("#wr{display:flex;height:calc(100vh - 48px)}"));
    c.print(F("#sb{width:190px;background:#111418;border-right:1px solid #1e2229;"));
    c.print(F("padding:8px 0;flex-shrink:0;overflow-y:auto}"));
    c.print(F("a.s{display:block;padding:6px 12px 6px 22px;font-size:.74rem;"));
    c.print(F("color:#8892a0;text-decoration:none;border-left:3px solid transparent}"));
    c.print(F(".ns{padding:7px 12px 1px;font-size:.58rem;color:#f5820a;"));
    c.print(F("letter-spacing:.12em;text-transform:uppercase;font-weight:700}"));
    c.print(F("a.s:hover{color:#c8cdd5;background:#1a1e25}"));
    c.print(F("a.s.on{color:#f5820a;background:#1a1e25;border-left-color:#f5820a}"));
    c.print(F("#ct{flex:1;padding:20px 24px;overflow-y:auto}"));
    c.print(F("h2{font-size:.85rem;color:#f5820a;letter-spacing:.08em;text-transform:uppercase;"));
    c.print(F("margin:0 0 16px;border-bottom:1px solid #1e2229;padding-bottom:7px}"));
    c.print(F("label{display:block;font-size:.68rem;color:#7a8490;margin-bottom:2px}"));
    c.print(F("input,select{width:100%;background:#0d0f12;border:1px solid #2a2f38;"));
    c.print(F("color:#c8cdd5;padding:6px 8px;font-family:'Courier New',monospace;"));
    c.print(F("font-size:.74rem;border-radius:2px;margin-bottom:9px}"));
    c.print(F("input:focus,select:focus{outline:none;border-color:#f5820a}"));
    c.print(F(".btn{background:#f5820a;color:#0d0f12;border:none;padding:6px 14px;"));
    c.print(F("font-family:'Courier New',monospace;font-size:.74rem;font-weight:700;"));
    c.print(F("cursor:pointer;border-radius:2px;margin-right:5px}.btn:hover{background:#ffa040}"));
    c.print(F(".bs{background:transparent;color:#7a8490;border:1px solid #2a2f38}"));
    c.print(F(".bs:hover{border-color:#f5820a;color:#f5820a}"));
    c.print(F("table{width:100%;border-collapse:collapse;font-size:.72rem}"));
    c.print(F("th{background:#1a1e25;color:#f5820a;padding:6px 9px;text-align:left;"));
    c.print(F("font-weight:700;border-bottom:2px solid #f5820a33}"));
    c.print(F("td{padding:5px 9px;border-bottom:1px solid #1a1e25}"));
    c.print(F("tr:hover td{background:#111418}.on{color:#2ecc71}.nb{color:#556}"));
    c.print(F(".err{color:#e74c3c;font-weight:700}"));
    c.print(F("fieldset{border:1px solid #1e2229;border-radius:2px;padding:11px 13px;margin-bottom:11px}"));
    c.print(F("legend{font-size:.64rem;color:#f5820a;letter-spacing:.1em;"));
    c.print(F("text-transform:uppercase;padding:0 5px}.row{display:flex;gap:12px}.col{flex:1}"));
}

// ─────────────────────────────────────────────────────────────
//  Sidebar helper — writes the full navigation bar
//
//  Renders three grouped sections (Systeem/System, Configuratie/
//  Configuration, Tabellen/Tables), each with a small uppercase
//  section header followed by its _navItem() links. Panel id strings
//  ("fabriek", "systeem", "lenzlan", ...) are internal routing
//  identifiers used elsewhere for the ?p= query parameter and the
//  strcmp() dispatch in _handleRoot() — left unchanged; only the
//  human-readable link labels are translated.
// ─────────────────────────────────────────────────────────────
void Webserver::_buildSidebar(EthernetClient& c, const char* active) {
    c.print(F("<div class='ns'>System</div>"));
    _navItem(c, "fabriek",  "Factory Reset",active);
    _navItem(c, "systeem",  "System",       active);
    _navItem(c, "internet", "Internet",     active);
    _navItem(c, "lenzlan",  "LenzLAN",      active);
    _navItem(c, "z21",      "Z21",          active);
    _navItem(c, "webif",    "Web Interface",active);
    _navItem(c, "tracelog", "Debug Log",    active);
    c.print(F("<div class='ns'>Configuration</div>"));
    _navItem(c, "dcchal",   "DCC HAL",      active);
    _navItem(c, "power",    "Power rails",  active);
    _navItem(c, "rs485",    "RS485",        active);
    _navItem(c, "rsbus",    "RS-Bus HAL",   active);
    _navItem(c, "xpressnet","XpressNet",    active);
    c.print(F("<div class='ns'>Tables</div>"));
    _navItem(c, "locos",    "Locomotives",  active);
    _navItem(c, "loconet",  "LocoNet Slots",active);
    _navItem(c, "turnouts", "Turnouts",     active);
    _navItem(c, "feedback", "Feedback",     active);
}

// ─────────────────────────────────────────────────────────────
//  Shell open — header + sidebar (shared by all panels)
// ─────────────────────────────────────────────────────────────
void Webserver::_shellOpen(EthernetClient& c, const char* active,
                            const char* title) {
    _sendHeader(c, 200, "text/html");
    c.print(F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
              "<title>TMC - "));
    c.print(title);
    c.print(F("</title>"
              "<link rel='stylesheet' href='/style.css'>"
              "</head><body>"
              "<div id='hdr'>"
              "<h1>&#9650; TMC Centrale</h1>"
              "<span class='v'>RP2350 DCC v2.0</span>"
              "</div>"
              "<div id='wr'>"
              "<nav id='sb'>"));
    _buildSidebar(c, active);
    c.print(F("</nav><div id='ct'>"));
}

// _shellClose() — closes the div/div/body/html tags opened by
// _shellOpen(), completing the page.
void Webserver::_shellClose(EthernetClient& c) {
    c.print(F("</div></div></body></html>"));
}

// ─────────────────────────────────────────────────────────────
//  Root — dispatches to the right panel based on ?p=
//
//  Compares `panel` against every known panel id and calls the
//  matching handler; falls back to _panelInternet() (the network
//  settings panel) if the id does not match anything recognised, or
//  if the query parameter was absent entirely.
// ─────────────────────────────────────────────────────────────
void Webserver::_handleRoot(EthernetClient& c, const char* panel) {
    if (strcmp(panel, "systeem")  == 0) { _panelSysteem(c);   return; }
    if (strcmp(panel, "lenzlan")  == 0) { _panelLenzLan(c);  return; }
    if (strcmp(panel, "z21")      == 0) { _panelZ21(c);      return; }
    if (strcmp(panel, "webif")    == 0) { _panelWebIf(c);    return; }
    if (strcmp(panel, "tracelog") == 0) { _panelTraceLog(c); return; }
    if (strcmp(panel, "dcchal")   == 0) { _panelDccHal(c);   return; }
    if (strcmp(panel, "power")    == 0) { _panelPower(c);    return; }
    if (strcmp(panel, "rs485")    == 0) { _panelRs485(c);    return; }
    if (strcmp(panel, "rsbus")    == 0) { _panelRsBus(c);    return; }
    if (strcmp(panel, "xpressnet") == 0) { _panelXpressNet(c); return; }
    if (strcmp(panel, "fabriek")    == 0) { _panelFabriek(c);   return; }
    if (strcmp(panel, "locos")    == 0) { _panelLocos(c);    return; }
    if (strcmp(panel, "loconet")  == 0) { _panelLoconet(c);  return; }
    if (strcmp(panel, "turnouts") == 0) { _panelTurnouts(c); return; }
    if (strcmp(panel, "feedback") == 0) { _panelFeedback(c); return; }
    _panelInternet(c);  // default
}

// ─────────────────────────────────────────────────────────────
//  Panel: System — general command-station settings
// ─────────────────────────────────────────────────────────────
// Short display string for a CmdType value, used in _panelSysteem()'s
// CommandBus drop log.
const __FlashStringHelper* Webserver::_cmdTypeName(CmdType t) {
    switch (t) {
        case CmdType::LOCO_SPEED:     return F("Loco speed");
        case CmdType::LOCO_FUNCTION:  return F("Loco function");
        case CmdType::LOCO_ESTOP:     return F("Loco e-stop");
        case CmdType::ACCESSORY_SET:  return F("Accessory");
        case CmdType::POWER_ON:       return F("Power on");
        case CmdType::POWER_OFF:      return F("Power off");
        case CmdType::EMERGENCY_STOP: return F("Emergency stop");
        case CmdType::CV_READ:        return F("CV read");
        case CmdType::CV_WRITE:       return F("CV write");
        case CmdType::POM_WRITE:      return F("POM write");
        case CmdType::POM_READ:       return F("POM read");
        case CmdType::LN_DISPATCH:    return F("LocoNet dispatch");
        default:                      return F("?");
    }
}

void Webserver::_panelSysteem(EthernetClient& c) {
    _shellOpen(c, "systeem", "System");
    c.print(F("<h2>System</h2>"
              "<form method='POST' action='/config'>"
              "<input type='hidden' name='_p' value='systeem'>"));
    c.print(F("<fieldset><legend>Turnout wiring</legend>"
              "<p class='hint'>Whether a straight/thrown command results in a "
              "physical turnout going straight or diverging is entirely "
              "wiring-dependent per RCN-213/the Z21 spec — neither protocol "
              "dictates it. Different installations (different decoders, "
              "different wiring) may need opposite settings here.</p>"));
    _printField(c, "sys.turnout_invert", "Invert turnout output bit");
    c.print(F("</fieldset>"));
    c.print(F("<fieldset><legend>CDE Booster short-circuit response</legend>"
              "<p class='hint'>Sets how the LZ210 responds when a CDE booster"
              " detects a short circuit (E-signal held continuously low longer"
              " than the threshold below).</p>"));
    _printField(c, "sys.cde_mode", "Response to short circuit");
    _printField(c, "sys.cde_short_mv", "Short-circuit voltage threshold (mV)");
    _printField(c, "sys.cde_short_us", "Short-circuit duration threshold (&micro;s)");
    _printField(c, "sys.cde_clear_us", "Recovery-confirm debounce (&micro;s)");
    c.print(F("</fieldset>"
              "<fieldset><legend>H-bridge nFAULT escalation (v2 hardware)</legend>"
              "<p class='hint'>An isolated nFAULT blip on any of the three "
              "H-bridges (DCC/SM/CDE) is expected and harmless — the "
              "DRV887x chips already recover from it automatically. Track "
              "power is only cut when EITHER faults recur repeatedly within "
              "the window below, OR nFAULT is observed continuously low for "
              "at least the duration below (needed since a genuinely "
              "persistent fault doesn't always trigger the first check under "
              "automatic-retry).</p>"));
    _printField(c, "sys.fault_window_ms",   "Window (ms)");
    _printField(c, "sys.fault_retry_count", "Faults within window to escalate");
    _printField(c, "sys.fault_duration_ms", "Continuous-low duration to escalate (ms)");
    c.print(F("</fieldset>"
              "<button class='btn'>Save</button></form>"));

    // CommandBus drop log — see command_bus.h's public accessors'
    // comment. A dropped command means the ring buffer (32 slots) was
    // still full of unprocessed commands when a protocol module tried
    // to dispatch a new one — only expected under an extreme burst or
    // if core0 stalls, but worth surfacing here since most dropped
    // commands are otherwise silently lost with no other indication.
    CommandBus& bus = CommandBus::instance();
    c.print(F("<h3>CommandBus drop log</h3><p>Totaal verloren commando's "
              "sinds opstarten: "));
    c.print(bus.dropTotal());
    c.print(F("</p>"));
    uint8_t n = bus.dropLogCount();
    if (n == 0) {
        c.print(F("<p class='hint'>Geen commando's verloren.</p>"));
    } else {
        c.print(F("<table><tr><th>Tijd (ms)</th><th>Bron</th><th>Commando</th></tr>"));
        for (uint8_t i = 0; i < n; i++) {
            const DispatchDropEntry& e = bus.dropLogEntry(i);
            c.print(F("<tr><td>")); c.print(e.ms);
            c.print(F("</td><td>")); c.print(e.sourceId);
            c.print(F("</td><td>")); c.print(_cmdTypeName(e.type));
            c.print(F("</td></tr>"));
        }
        c.print(F("</table>"));
    }
    _shellClose(c);
}

// ─────────────────────────────────────────────────────────────
//  Panel: Internet
// ─────────────────────────────────────────────────────────────
void Webserver::_panelInternet(EthernetClient& c) {
    _shellOpen(c, "internet", "Internet");
    c.print(F("<h2>Internet</h2>"
              "<form method='POST' action='/config'>"
              "<input type='hidden' name='_p' value='internet'>"));
    c.print(F("<fieldset><legend>Network</legend>"));
    _printField(c, "net.mac",      "MAC address");
    _printField(c, "net.ip",       "IP address");
    _printField(c, "net.gateway",  "Gateway");
    _printField(c, "net.dhcp",     "DHCP");
    c.print(F("</fieldset><fieldset><legend>mDNS</legend>"));
    _printField(c, "net.hostname", "Hostname (.local)");
    c.print(F("</fieldset><button class='btn'>Save</button></form>"));
    _shellClose(c);
}

// ─────────────────────────────────────────────────────────────
//  Panel: LenzLAN
// ─────────────────────────────────────────────────────────────
void Webserver::_panelLenzLan(EthernetClient& c) {
    _shellOpen(c, "lenzlan", "LenzLAN");
    c.print(F("<h2>LenzLAN</h2>"
              "<form method='POST' action='/config'>"
              "<input type='hidden' name='_p' value='lenzlan'>"));
    c.print(F("<fieldset><legend>TCP Server</legend>"));
    _printField(c, "lenz.port",    "TCP port");
    _printField(c, "lenz.maxconn", "Max connections");
    c.print(F("</fieldset><fieldset><legend>XpressNet Identification</legend>"));
    _printField(c, "xn.version", "Version number");
    _printField(c, "xn.kennung", "Kennung");
    _printField(c, "xn.li_address", "Interface device address (SV1, 1-31)");
    c.print(F("</fieldset><button class='btn'>Save</button></form>"));
    _shellClose(c);
}

// ─────────────────────────────────────────────────────────────
//  Panel: Z21
// ─────────────────────────────────────────────────────────────
void Webserver::_panelZ21(EthernetClient& c) {
    _shellOpen(c, "z21", "Z21");
    c.print(F("<h2>Z21</h2>"
              "<form method='POST' action='/config'>"
              "<input type='hidden' name='_p' value='z21'>"));
    c.print(F("<fieldset><legend>UDP Server</legend>"));
    _printField(c, "z21.port", "UDP port");
    c.print(F("</fieldset><fieldset><legend>Client Tracking</legend>"
              "<p class='hint'>Per the official Z21 LAN Protocol "
              "Specification, clients are expected to communicate at "
              "least once per minute or be dropped. Default here "
              "matches that (60s).</p>"));
    _printField(c, "z21.client_timeout_s", "Client inactivity timeout (seconds)");
    c.print(F("</fieldset><button class='btn'>Save</button></form>"));
    _shellClose(c);
}

// ─────────────────────────────────────────────────────────────
//  Panel: Web interface
// ─────────────────────────────────────────────────────────────
void Webserver::_panelWebIf(EthernetClient& c) {
    _shellOpen(c, "webif", "Web Interface");
    c.print(F("<h2>Web Interface</h2>"
              "<form method='POST' action='/config'>"
              "<input type='hidden' name='_p' value='webif'>"));
    c.print(F("<fieldset><legend>HTTP Server</legend>"));
    _printField(c, "web.enable", "Enabled");
    _printField(c, "web.port",   "HTTP port");
    c.print(F("</fieldset>"
              "<button class='btn'>Save</button>"
              "<button class='btn bs' type='button'"
              " onclick=\"if(confirm('Reset to defaults?'))"
              "{fetch('/reset',{method:'POST'}).then(()=>location.reload());}\">Reset</button>"
              "</form>"));
    _shellClose(c);
}

// ─────────────────────────────────────────────────────────────
//  Panel: Debug Log (TraceLog)
//
//  Configures and shows the live status of the TraceLog TCP debug
//  stream (see trace_log.h) — a generic facility any module can send
//  arbitrary trace lines to, viewable by connecting a plain TCP
//  client (e.g. netcat, PuTTY in raw mode) to the configured port
//  while enabled.
// ─────────────────────────────────────────────────────────────
void Webserver::_panelTraceLog(EthernetClient& c) {
    _shellOpen(c, "tracelog", "Debug Log");
    c.print(F("<h2>Debug Log</h2>"
              "<p class='hint'>Streams trace/debug lines from any "
              "module over a plain TCP connection — connect with e.g. "
              "netcat or PuTTY (raw mode) to the configured port while "
              "enabled. Off by default: no socket is opened at all "
              "unless enabled below. Only one viewer at a time; "
              "connecting again replaces the previous one.</p>"
              "<form method='POST' action='/config'>"
              "<input type='hidden' name='_p' value='tracelog'>"));
    c.print(F("<fieldset><legend>TCP Stream</legend>"));
    _printField(c, "debug.tcp_log_enable", "Enabled");
    _printField(c, "debug.tcp_log_port",   "TCP port");
    c.print(F("</fieldset><button class='btn'>Save</button></form>"));

    c.print(F("<fieldset><legend>Status</legend><table>"
              "<tr><td>Listening</td><td>"));
    c.print(eepromStore().getBool("debug.tcp_log_enable", false) ? F("Yes") : F("No"));
    c.print(F("</td></tr><tr><td>Client connected</td><td>"));
    c.print(traceLog().active() ? F("Yes") : F("No"));
    c.print(F("</td></tr></table></fieldset>"));
    _shellClose(c);
}

// ─────────────────────────────────────────────────────────────
//  Panel: DCC HAL
// ─────────────────────────────────────────────────────────────
// v2 hardware: short display string for an AckState value, used in
// _panelDccHal()'s H-bridge status table.
const __FlashStringHelper* Webserver::_ackStateName(AckState s) {
    switch (s) {
        case AckState::IDLE:      return F("idle");
        case AckState::SETTLING:  return F("settelen");
        case AckState::WAIT:      return F("wachten");
        case AckState::DETECTING: return F("detecteren");
        case AckState::SUCCESS:   return F("ACK ontvangen");
        case AckState::FAIL:      return F("geen/foutieve ACK");
        default:                  return F("?");
    }
}

void Webserver::_panelDccHal(EthernetClient& c) {
    _shellOpen(c, "dcchal", "DCC HAL");
    c.print(F("<h2>DCC HAL</h2>"
              "<form method='POST' action='/config'>"
              "<input type='hidden' name='_p' value='dcchal'>"));
    c.print(F("<fieldset><legend>DCC</legend>"));
    _printField(c, "dcc.steps",   "Speed steps");
    _printField(c, "dcc.railcom", "RailCom");
    _printField(c, "dcc.shortma", "Short-circuit threshold (mA)");
    c.print(F("</fieldset><fieldset><legend>Turnout addressing</legend>"));
    _printField(c, "dcc.swschema", "Schema");
    _printField(c, "dcc.swformat", "Format");
    c.print(F("</fieldset><fieldset><legend>Programming track</legend>"));
    _printField(c, "dcc.progreadmode", "CV readout mode");
    _printField(c, "dcc.progrepeat",   "Programming repeats (7-64)");
    _printField(c, "dcc.rstsrepeat",   "Reset start repeats (25-255)");
    _printField(c, "dcc.rstcrepeat",   "Reset continue repeats (6-64)");
    _printField(c, "dcc.svc_timeout_ms", "Service mode timeout (ms)");
    c.print(F("</fieldset>"
              "<fieldset><legend>H-bridges — ACK threshold (v2 hardware)</legend>"
              "<p class='hint'>Depends on which H-bridge chip variant is "
              "populated on this board (DRV8874 vs DRV8876) — adjust if "
              "ACK detection is unreliable.</p>"));
    _printField(c, "dcc.ack_mv", "DCC bridge (mV)");
    _printField(c, "sm.ack_mv",  "SM/programming track bridge (mV)");
    _printField(c, "cde.ack_mv", "CDE bridge (mV)");
    c.print(F("</fieldset><button class='btn'>Save</button></form>"));

    c.print(F("<h3>H-bridge status</h3>"
              "<table><tr><th>Bridge</th><th>nFAULT</th>"
              "<th>ACK state</th><th>SENSE reading</th></tr>"));
    struct BridgeRow { const HBridgeFault* fault; const DccCurrentMonitor* sense; };
    BridgeRow rows[] = {
        { &gDccHal.faultDcc(), &gDccHal.senseDcc() },
        { &gDccHal.faultSm(),  &gDccHal.senseSm()  },
        { &gDccHal.faultCde(), &gDccHal.senseCde() },
    };
    for (auto& row : rows) {
        c.print(F("<tr><td>")); c.print(row.fault->name());
        c.print(F("</td><td>"));
        c.print(row.fault->isFaulted() ? F("FAULT") : F("ok"));
        c.print(F("</td><td>")); c.print(_ackStateName(row.sense->ackState()));
        c.print(F("</td><td>")); c.print(row.sense->lastAdcValue());
        c.print(F("</td></tr>"));
    }
    c.print(F("</table>"));
    _shellClose(c);
}

// ─────────────────────────────────────────────────────────────
//  Panel: Power rails
//
//  Shows live readings from the three onboard power-rail monitors
//  (VIN, 12V, 5V — v2 hardware only, see power_monitor.h): both the
//  voltage actually measured across the divider's lower resistor,
//  and the reconstructed full rail voltage derived from the exact
//  resistor values.
// ─────────────────────────────────────────────────────────────
void Webserver::_panelPower(EthernetClient& c) {
    _shellOpen(c, "power", "Power rails");
    c.print(F("<h2>Power rails</h2>"
              "<table><tr><th>Rail</th><th>Gemeten (over weerstand)</th>"
              "<th>Gereconstrueerd (rail)</th><th>Nominaal</th></tr>"));

    const PowerRailMonitor* rails[] = {
        &gPowerMonitor.vin(), &gPowerMonitor.v12(), &gPowerMonitor.v5()
    };
    for (auto* r : rails) {
        c.print(F("<tr><td>")); c.print(r->name());
        c.print(F("</td><td>")); c.print(r->measuredMv() / 1000.0f, 3); c.print(F("V"));
        c.print(F("</td><td>")); c.print(r->railMv() / 1000.0f, 2); c.print(F("V"));
        c.print(F("</td><td>")); c.print(r->nominalMv() / 1000.0f, 2); c.print(F("V"));
        c.print(F("</td></tr>"));
    }
    c.print(F("</table>"));
    _shellClose(c);
}

// ─────────────────────────────────────────────────────────────
//  Panel: RS485 / RS-Bus
// ─────────────────────────────────────────────────────────────
void Webserver::_panelRs485(EthernetClient& c) {
    _shellOpen(c, "rs485", "RS485");
    c.print(F("<h2>XpressNet RS485</h2>"
              "<form method='POST' action='/config'>"
              "<input type='hidden' name='_p' value='rs485'>"));
    c.print(F("<fieldset><legend>RS485</legend>"));
    _printField(c, "rs485.enable", "Enabled");
    c.print(F("</fieldset><button class='btn'>Save</button></form>"));
    _shellClose(c);
}

void Webserver::_panelRsBus(EthernetClient& c) {
    _shellOpen(c, "rsbus", "RS-Bus HAL");
    c.print(F("<h2>RS-Bus HAL</h2>"
              "<form method='POST' action='/config'>"
              "<input type='hidden' name='_p' value='rsbus'>"));
    c.print(F("<fieldset><legend>RS-Bus</legend>"));
    _printField(c, "rsbus.enable", "Enabled");
    c.print(F("</fieldset>"
              "<fieldset><legend>Feedback filtering</legend>"
              "<p class='hint'>Newer feedback decoders only report when their "
              "state actually changes. Older decoders may report "
              "unconditionally on every poll cycle, which without filtering "
              "can flood connected clients with duplicate messages. Disable "
              "this only once all feedback decoders on the layout have been "
              "replaced with newer, self-filtering ones.</p>"));
    _printField(c, "rsbus.delta_filter", "Filter unchanged feedback");
    c.print(F("</fieldset><button class='btn'>Save</button></form>"));
    _shellClose(c);
}

// ─────────────────────────────────────────────────────────────
//  Panel: Locomotives
//
//  Renders a table of every active loco in the repository: address,
//  speed, direction arrow, whether F0 is on, a comma-separated list
//  of which functions are currently configured as momentary, and a
//  running/stopped status badge. Held under the repository read lock
//  for the whole table build.
// ─────────────────────────────────────────────────────────────
void Webserver::_panelLocos(EthernetClient& c) {
    _shellOpen(c, "locos", "Locomotives");
    c.print(F("<h2>Locomotives</h2>"
              "<table><tr>"
              "<th>Address</th><th>Speed</th><th>Direction</th>"
              "<th>F0</th><th>Momentary</th><th>Status</th></tr>"));
    if (locoRepo().lockRead(500)) {
        uint8_t count = locoRepo().count();
        for (uint8_t i = 0; i < count; i++) {
            const LocoBase* loco = locoRepo().at(i);
            if (!loco || !loco->active) continue;
            c.print(F("<tr><td>")); c.print(loco->address);
            c.print(F("</td><td>")); c.print(loco->speed);
            c.print(F("</td><td>"));
            c.print(loco->forward ? F("&#9654;") : F("&#9664;"));
            c.print(F("</td><td>"));
            c.print((loco->functions & 1) ? F("On") : F("-"));
            c.print(F("</td><td>"));
            bool anyMomentary = false;
            for (uint8_t fn = 0; fn <= 68; fn++) {
                bool mo;
                if (fn <= 31) {
                    mo = (loco->momentary >> fn) & 1;
                } else {
                    uint8_t idx = fn - 32;
                    mo = (loco->momentaryExt[idx / 8] >> (idx % 8)) & 1;
                }
                if (mo) {
                    if (anyMomentary) c.print(F(","));
                    c.print(F("F")); c.print(fn);
                    anyMomentary = true;
                }
            }
            if (!anyMomentary) c.print(F("-"));
            c.print(F("</td><td class='"));
            c.print(loco->speed > 0 ? F("on'>Running") : F("nb'>Stopped"));
            c.print(F("</td></tr>"));
        }
        locoRepo().unlock();
    }
    c.print(F("</table>"
              "<form method='POST' action='/clear/locos'>"
              "<button class='btn btn-danger' onclick='return confirm(&quot;Clear locomotive table?&quot;)'>"
              "Clear table</button></form>"));
    _shellClose(c);
}

// ─────────────────────────────────────────────────────────────
//  Panel: LocoNet Slots
//
//  Renders a table of only those locos that currently hold a real
//  LocoNet slot assignment (loco->ln.slot != LN_NO_SLOT) — i.e. locos
//  that have been touched by a LocoNet throttle such as a FREMO Fred,
//  as opposed to the "Locomotives" panel, which lists every active
//  loco in the repository regardless of which protocol last touched
//  it. Shows both the LocoNet-specific slot bookkeeping (slot number,
//  status, speed-step mode, throttle ID, time since last activity)
//  and the loco's general current state (address, speed, direction,
//  active functions) side by side, which is useful for diagnosing
//  LocoNet-specific issues (e.g. a throttle holding a stale slot
//  after a reflash — see project history) without needing a serial
//  log.
// ─────────────────────────────────────────────────────────────
void Webserver::_panelLoconet(EthernetClient& c) {
    _shellOpen(c, "loconet", "LocoNet Slots");
    c.print(F("<h2>LocoNet Slots</h2>"
              "<p class='hint'>Only locos currently holding a LocoNet slot "
              "are shown here (i.e. touched by a LocoNet throttle such as "
              "a Fred). See the Locomotives panel for every active loco "
              "regardless of protocol.</p>"
              "<table><tr>"
              "<th>Slot</th><th>Status</th><th>Steps</th><th>Throttle ID</th>"
              "<th>Last activity</th>"
              "<th>Address</th><th>Speed</th><th>Dir</th><th>Active F</th></tr>"));
    if (locoRepo().lockRead(500)) {
        uint32_t now = millis();
        uint8_t count = locoRepo().count();
        for (uint8_t i = 0; i < count; i++) {
            const LocoBase* loco = locoRepo().at(i);
            if (!loco || !loco->active || loco->ln.slot == LN_NO_SLOT) continue;

            static const char* const statLabel[] = { "FREE", "COMMON", "IDLE", "IN_USE" };
            static const char* const spdLabel[]  = { "14", "28T", "28", "128", "28A", "128A" };

            c.print(F("<tr><td>")); c.print(loco->ln.slot);
            c.print(F("</td><td"));
            if (loco->ln.stat == 3) c.print(F(" class='on'"));  // IN_USE highlighted
            c.print(F(">"));
            c.print(loco->ln.stat < 4 ? statLabel[loco->ln.stat] : "?");
            c.print(F("</td><td>"));
            c.print(loco->ln.spdMode < 6 ? spdLabel[loco->ln.spdMode] : "?");
            c.print(F("</td><td class='nb'>0x"));
            if (loco->ln.id1 < 0x10) c.print('0'); c.print(loco->ln.id1, HEX);
            if (loco->ln.id2 < 0x10) c.print('0'); c.print(loco->ln.id2, HEX);
            c.print(F("</td><td>"));
            c.print((now - loco->ln.lastActivityMs) / 1000);
            c.print(F("s ago</td><td>"));
            c.print(loco->address);
            c.print(F("</td><td>")); c.print(loco->speed);
            c.print(F("</td><td>"));
            c.print(loco->forward ? F("&#9654;") : F("&#9664;"));
            c.print(F("</td><td>"));
            bool anyOn = false;
            for (uint8_t fn = 0; fn <= 28; fn++) {
                if ((loco->functions >> fn) & 1) {
                    if (anyOn) c.print(F(","));
                    c.print(F("F")); c.print(fn);
                    anyOn = true;
                }
            }
            if (!anyOn) c.print(F("-"));
            c.print(F("</td></tr>"));
        }
        locoRepo().unlock();
    }
    c.print(F("</table>"));
    _shellClose(c);
}


// ─────────────────────────────────────────────────────────────
//  Panel: Turnouts
//
//  Renders a table of every turnout address with a known state
//  (gAccessories[i].known), showing its current position.
// ─────────────────────────────────────────────────────────────
void Webserver::_panelTurnouts(EthernetClient& c) {
    _shellOpen(c, "turnouts", "Turnouts");
    c.print(F("<h2>Turnouts</h2>"
              "<table><tr><th>Address</th><th>Position</th></tr>"));
    for (uint16_t i = 1; i < MAX_ACCESSORIES; i++) {
        if (!gAccessories[i].known) continue;
        c.print(F("<tr><td>")); c.print(i);
        c.print(F("</td><td class='"));
        c.print((gAccessories[i].state & 0x01)
                ? F("on'>&#9658; Thrown")
                : F("nb'>&#9654; Straight"));
        c.print(F("</td></tr>"));
    }
    c.print(F("</table>"
              "<form method='POST' action='/clear/turnouts'>"
              "<button class='btn btn-danger' onclick='return confirm(&quot;Clear turnout table?&quot;)'>"
              "Clear table</button></form>"));
    _shellClose(c);
}

// ─────────────────────────────────────────────────────────────
//  Panel: RS-Bus feedback
//
//  Renders a table of every RS-Bus module address that has ever been
//  seen, along with its raw type byte, both nibble values (as hex and
//  as individual on/off indicator squares), and diagnostic counters
//  (message count, parity errors, sampling errors — the latter two
//  highlighted with the 'err' CSS class whenever non-zero). See the
//  in-page hint text for the distinction between RS-Bus hardware
//  addressing (1-128) and the outward-facing protocol addressing
//  (0-127).
// ─────────────────────────────────────────────────────────────
void Webserver::_panelFeedback(EthernetClient& c) {
    _shellOpen(c, "feedback", "Feedback");
    c.print(F("<h2>RS-Bus Feedback</h2>"
              "<p class='hint'>Module = RS-Bus hardware address (1-128, as "
              "configured on the decoder). This is not the same as the "
              "protocol address (0-127) that XpressNet/LAN clients see — "
              "that is always 1 lower. Type = raw 2-bit decoder type from "
              "the last RS-Bus message (bits 6-5). Meaning not yet "
              "confirmed — kept for inspection during testing only. "
              "Messages/Parity/Sampling come directly from the "
              "RSbusMaster library and are useful for tracking down "
              "wiring/signal problems (many parity or sampling errors "
              "points to a fault on the RS-Bus, e.g. caused by "
              "interference).</p>"
              "<table><tr><th>Module</th><th>Type</th><th>N0</th><th>N1</th>"
              "<th>In 1-4</th><th>In 5-8</th>"
              "<th>Messages</th><th>Parity errors</th><th>Sampling errors</th></tr>"));
    for (uint16_t addr = 1; addr <= 128; addr++) {
        if (!gRsBusHal.isSeen(addr)) continue;  // module never seen — skip
        uint8_t  n0  = gRsBusHal.getNibble(addr, 0);
        uint8_t  n1  = gRsBusHal.getNibble(addr, 1);
        uint8_t  ty  = gRsBusHal.getType(addr);
        uint32_t cnt = gRsBusHal.getMessageCount(addr);
        uint16_t pErr = gRsBusHal.getParityErrors(addr);
        uint16_t sErr = gRsBusHal.getSamplingErrors(addr);
        c.print(F("<tr><td>")); c.print(addr);
        c.print(F("</td><td>")); c.print(ty);
        c.print(F("</td><td class='nb'>0x")); c.print(n0, HEX);
        c.print(F("</td><td class='nb'>0x")); c.print(n1, HEX);
        c.print(F("</td><td>"));
        for (int b=0;b<4;b++)
            c.print((n0>>b)&1 ? F("<span class='on'>&#9632;</span>")
                              : F("<span class='nb'>&#9633;</span>"));
        c.print(F("</td><td>"));
        for (int b=0;b<4;b++)
            c.print((n1>>b)&1 ? F("<span class='on'>&#9632;</span>")
                              : F("<span class='nb'>&#9633;</span>"));
        c.print(F("</td><td>")); c.print(cnt);
        c.print(F("</td><td"));
        if (pErr > 0) c.print(F(" class='err'"));
        c.print(F(">")); c.print(pErr);
        c.print(F("</td><td"));
        if (sErr > 0) c.print(F(" class='err'"));
        c.print(F(">")); c.print(sErr);
        c.print(F("</td></tr>"));
    }
    c.print(F("</table>"
              "<form method='POST' action='/clear/feedback'>"
              "<button class='btn btn-danger' onclick='return confirm(&quot;Clear feedback table?&quot;)'>"
              "Clear table</button></form>"));
    _shellClose(c);
}

// ─────────────────────────────────────────────────────────────
//  Panel: XpressNet configuration
// ─────────────────────────────────────────────────────────────
void Webserver::_panelXpressNet(EthernetClient& c) {
    _shellOpen(c, "xpressnet", "XpressNet");
    c.print(F("<form method='POST' action='/config'>"
              "<input type='hidden' name='_p' value='xpressnet'>"));
    c.print(F("<fieldset><legend>Turnout</legend>"
              "<p class='hint'>Applies only to addresses 1-512 (matching real "
              "Lenz LZ100 behaviour). Addresses above 512 never get a "
              "broadcast, regardless of this setting.</p>"));
    _printField(c, "xn.sw_broadcast", "Broadcast after switching command (0x42)");
    c.print(F("</fieldset><button class='btn'>Save</button></form>"));
    _shellClose(c);
}

// ─────────────────────────────────────────────────────────────
//  Panel: Factory reset
//
//  The underlying checkbox's `name='bevestig'` attribute and the
//  corresponding `bevestig=1` check in _handleClient() are left
//  unchanged (they are an internal form-field identifier, not
//  user-facing text) — only the visible label text is translated.
// ─────────────────────────────────────────────────────────────
void Webserver::_panelFabriek(EthernetClient& c) {
    _shellOpen(c, "fabriek", "Factory Reset");
    c.print(F("<form method='POST' action='/fabriek'>"));
    c.print(F("<fieldset><legend>Factory Reset</legend>"
              "<p>This clears all settings, locomotives, turnouts and "
              "feedback data and resets everything to factory defaults.</p>"
              "<label><input type='checkbox' name='bevestig' value='1'> "
              "I confirm that all settings will be erased</label>"
              "</fieldset>"
              "<button class='btn btn-danger'>Perform Factory Reset</button>"
              "</form>"));
    _shellClose(c);
}

// ═══════════════════════════════════════════════════════════════
//  Helper functions — HTTP and form processing
// ═══════════════════════════════════════════════════════════════

// _sendHeader() — writes a minimal HTTP response status line (200 OK
// or 404 Not Found, based on `code`) followed by a Content-Type
// header and "Connection: close" (this server does not support
// keep-alive), then the blank line separating headers from the body.
void Webserver::_sendHeader(EthernetClient& c, uint16_t code,
                             const char* contentType) {
    if (code == 200)
        c.print(F("HTTP/1.1 200 OK\r\n"));
    else
        c.print(F("HTTP/1.1 404 Not Found\r\n"));
    c.print(F("Content-Type: ")); c.print(contentType);
    c.print(F("\r\nConnection: close\r\n\r\n"));
}

// _sendRedirect() — writes a full HTTP 302 Found response pointing
// the client at `location`, used for the redirect-after-POST pattern
// this server relies on for form submissions.
void Webserver::_sendRedirect(EthernetClient& c, const char* location) {
    c.print(F("HTTP/1.1 302 Found\r\nLocation: "));
    c.print(location);
    c.print(F("\r\nConnection: close\r\n\r\n"));
}

// _handle404() — writes a minimal plain-text 404 response.
void Webserver::_handle404(EthernetClient& c) {
    _sendHeader(c, 404, "text/plain");
    c.print(F("404 Not Found"));
}

// _pathEquals() — compares `path` against `expected`, considering
// only the portion of `path` before any '?' query string (so
// "/?p=foo" matches expected="/"). Case-sensitive, exact-length match.
bool Webserver::_pathEquals(const char* path, const char* expected) {
    const char* q = strchr(path, '?');
    size_t len = q ? (size_t)(q - path) : strlen(path);
    return strncmp(path, expected, len) == 0 && strlen(expected) == len;
}

// _urlDecode() — decodes a URL-encoded string in place: '+' becomes a
// space, and "%xx" hex escapes become their corresponding byte value.
// Any other character is copied through unchanged. Always
// null-terminates the result.
void Webserver::_urlDecode(char* str) {
    char* out = str;
    while (*str) {
        if (*str == '+') { *out++ = ' '; str++; }
        else if (*str == '%' && str[1] && str[2]) {
            char hex[3] = { str[1], str[2], 0 };
            *out++ = (char)strtol(hex, nullptr, 16);
            str += 3;
        } else {
            *out++ = *str++;
        }
    }
    *out = 0;
}

// _findParam() — scans a URL-encoded form/query body for a "key="
// occurrence, and if found, copies its (not yet URL-decoded) value
// into the caller-supplied buffer (truncated to maxLen-1 characters,
// stopping at the next '&' or end of string), returning a pointer
// just past the copied value. Returns nullptr (with value[0] set to
// 0) if the key was not found anywhere in the body.
const char* Webserver::_findParam(const char* body, const char* key,
                                    char* value, uint8_t maxLen) {
    const char* p = body;
    size_t klen = strlen(key);
    while (*p) {
        if (strncmp(p, key, klen) == 0 && p[klen] == '=') {
            p += klen + 1;
            uint8_t vi = 0;
            while (*p && *p != '&' && vi < maxLen - 1)
                value[vi++] = *p++;
            value[vi] = 0;
            return p;
        }
        while (*p && *p != '&') p++;
        if (*p == '&') p++;
    }
    value[0] = 0;
    return nullptr;
}

// ─────────────────────────────────────────────────────────────
//  _parseFormBody() — parse a POSTed application/x-www-form-urlencoded
//  body and apply each key=value pair to the matching EepromStore
//  parameter
//
//  Walks the body byte-by-byte, splitting on '=' and '&' to extract
//  each key/value pair in turn (key capped at EEPROM_KEY_MAX_LEN-1,
//  value at 31 characters), URL-decoding the value in place.
//
//  Skips empty keys and the special "_p" field (used elsewhere to
//  remember which panel the form was submitted from, for the
//  redirect-after-POST — not an actual EepromStore parameter).
//
//  Looks up each remaining key via eepromStore().findParam(); if it
//  is not a registered parameter, the pair is silently ignored
//  (unknown/stale form fields do not cause an error). Otherwise
//  parses and applies the value using the setter matching the
//  parameter's registered type (UINT8/16/32 via atoi/atol, BOOL as
//  "value starts with '1'", STRING copied as-is, ENUM via atoi like
//  the numeric types).
// ─────────────────────────────────────────────────────────────
void Webserver::_parseFormBody(const char* body, uint16_t len) {
    char key[EEPROM_KEY_MAX_LEN];
    char val[32];
    const char* p = body;
    while (p < body + len && *p) {
        uint8_t ki = 0;
        while (*p && *p != '=' && *p != '&' && ki < EEPROM_KEY_MAX_LEN-1)
            key[ki++] = *p++;
        key[ki] = 0;
        if (*p == '=') p++;
        uint8_t vi = 0;
        while (*p && *p != '&' && vi < 31)
            val[vi++] = *p++;
        val[vi] = 0;
        if (*p == '&') p++;
        _urlDecode(val);
        if (ki == 0 || strcmp(key, "_p") == 0) continue;
        const ParamDef* pd = eepromStore().findParam(key);
        if (!pd) continue;
        switch (pd->type) {
            case ParamType::UINT8:  eepromStore().setUint8 (key,(uint8_t)atoi(val));  break;
            case ParamType::UINT16: eepromStore().setUint16(key,(uint16_t)atoi(val)); break;
            case ParamType::UINT32: eepromStore().setUint32(key,(uint32_t)atol(val)); break;
            case ParamType::BOOL:   eepromStore().setBool  (key,val[0]=='1');          break;
            case ParamType::STRING: eepromStore().setString(key,val);                  break;
            case ParamType::ENUM:   eepromStore().setEnum  (key,(uint8_t)atoi(val));   break;
            default: break;
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  _handleConfigPost() — POST /config handler
//
//  Applies every submitted field via _parseFormBody(), then commits
//  the resulting changes to flash immediately (rather than waiting
//  for EepromStore's own auto-commit delay), so settings changed via
//  the web interface are persisted right away.
//
//  Redirects back to the panel the form was submitted from: looks for
//  the hidden "_p" field in the raw request body (which every panel's
//  form includes precisely for this purpose) and builds a "/?p=<panel>"
//  redirect target from it; falls back to "/?p=internet" if that field
//  is missing or unreasonably long.
// ─────────────────────────────────────────────────────────────
void Webserver::_handleConfigPost(EthernetClient& c, const HttpRequest& req) {
    _parseFormBody(req.body, req.bodyLen);
    eepromStore().commit();
    // Redirect back to the panel via the _p hidden field
    char dest[48] = "/?p=internet";
    const char* pp = strstr(req.body, "_p=");
    if (pp) {
        pp += 3;
        size_t len = 0;
        while (pp[len] && pp[len] != '&') len++;
        if (len > 0 && len < 32) {
            strcpy(dest, "/?p=");
            strncat(dest, pp, len);
        }
    }
    _sendRedirect(c, dest);
}

// _handleStatus() — serves a small JSON object with the current track
// power state, emergency-stop state, and the connection counts of the
// LenzLAN and Z21 protocol modules, for use by external monitoring
// tools.
void Webserver::_handleStatus(EthernetClient& c) {
    _sendHeader(c, 200, "application/json");
    c.print(F("{\"power\":"));
    c.print(gDccHal.getPower() ? F("true") : F("false"));
    c.print(F(",\"emergency\":"));
    c.print(gCentrale.emergencyStop ? F("true") : F("false"));
    c.print(F(",\"lenz\":"));    c.print(gLenzLan.connectionCount());
    c.print(F(",\"z21\":"));     c.print(gZ21Lan.connectionCount());
    c.print(F("}"));
}

// _buildStatusJson() — thin alias for _handleStatus(), kept as a
// separate name for callers that conceptually want "build the status
// JSON" rather than "handle an HTTP request".
void Webserver::_buildStatusJson(EthernetClient& c) { _handleStatus(c); }
