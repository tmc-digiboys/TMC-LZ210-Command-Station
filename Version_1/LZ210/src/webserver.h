#pragma once
// ═══════════════════════════════════════════════════════════════
//  webserver.h  —  HTTP configuration interface (core1)
//
//  Provides a simple web interface for configuring and monitoring
//  the command station over HTTP on port 80.
//
//  Features:
//    - Network settings (IP, MAC, hostname, DHCP)
//    - Module-specific parameters (DCC, RS-Bus, XpressNet, etc.)
//    - System diagnostics (connections, loco table, turnout table)
//    - RS-Bus feedback table
//    - Factory reset with confirmation checkbox
//
//  Design:
//    Pure server-side HTML — no JavaScript required.
//    Single-page navigation via the ?p=<panel> query parameter.
//    Form submission via POST with redirect-after-post pattern.
//    Maximum WEB_MAX_CLIENTS simultaneous connections.
//
//  URL structure:
//    GET  /?p=<panel>  → configuration/status page
//    POST /config      → save EepromStore parameter values
//    POST /reset       → reset all parameters to defaults
//    POST /fabriek     → factory reset (requires bevestig=1 checkbox)
//    POST /clear/locos     → clear loco table
//    POST /clear/turnouts  → clear turnout table
//    POST /clear/feedback  → clear RS-Bus feedback cache
//    GET  /style.css       → stylesheet
//    GET  /status          → JSON status for monitoring
// ═══════════════════════════════════════════════════════════════
#include "module_arch.h"
#include "eeprom_store.h"
#include "module_registry.h"
#include "loco_repository.h"
#include <Ethernet.h>

#define WEB_PORT          80    // HTTP listen port
#define WEB_MAX_REQ_SIZE  512   // Maximum request body size (bytes)
#define WEB_MAX_CLIENTS   2     // Maximum simultaneous HTTP connections

class Webserver : public ProtocolModule {
public:
    Webserver() : ProtocolModule("Webserver", ModuleId::WEBSERVER,
                                  ModuleCore::CORE1) {}

    // Starts the HTTP server on WEB_PORT
    void begin()  override;

    // Accepts new connections and handles one pending request per call
    void loop()   override;

    // No event broadcasting — the web server is read-only for events
    void onEvent(const Event& ev) override {}

    uint8_t connectionCount() const override { return _activeClients; }
    uint8_t maxConnections()  const override { return WEB_MAX_CLIENTS; }

private:
    EthernetServer _server{WEB_PORT};
    uint8_t        _activeClients = 0;

    // Reads the HTTP request from the client and dispatches to handlers
    void _handleClient(EthernetClient& client);

    // ── HTTP request parsing ──────────────────────────────
    struct HttpRequest {
        char     method[8];
        char     path[64];
        char     body[WEB_MAX_REQ_SIZE];
        uint16_t bodyLen;
        bool     valid;
    };

    // Reads and parses one HTTP request. Returns false on timeout/error.
    bool _parseRequest(EthernetClient& client, HttpRequest& req);

    // Parses a URL-encoded form body and updates EepromStore parameters
    void _parseFormBody(const char* body, uint16_t len);

    // ── Route handlers ────────────────────────────────────

    // Serves the main page with sidebar navigation and the active panel
    void _handleRoot(EthernetClient& c, const char* panel);

    // Serves the CSS stylesheet
    void _handleCss(EthernetClient& c);

    // Handles POST /config — saves parameter values from form body
    void _handleConfigPost(EthernetClient& c, const HttpRequest& req);

    // Serves a JSON status object with module and connection counts
    void _handleStatus(EthernetClient& c);

    // Serves a 404 Not Found response
    void _handle404(EthernetClient& c);

    // ── HTTP response helpers ─────────────────────────────

    // Sends an HTTP response header with status code and content type
    void _sendHeader(EthernetClient& c, uint16_t code, const char* ct);

    // Sends an HTTP 302 redirect to the given location
    void _sendRedirect(EthernetClient& c, const char* location);

    // ── Page shell helpers ────────────────────────────────

    // Opens the HTML page shell (DOCTYPE, head, nav, main open tag)
    void _shellOpen(EthernetClient& c, const char* active, const char* title);

    // Closes the HTML page shell (main close, body, html)
    void _shellClose(EthernetClient& c);

    // Renders the sidebar navigation with all panel links
    void _buildSidebar(EthernetClient& c, const char* active);

    // Renders one navigation link item, highlighted if id==active
    void _navItem(EthernetClient& c, const char* id,
                  const char* label, const char* active);

    // Renders one form field for an EepromStore parameter key
    void _printField(EthernetClient& c, const char* key, const char* label);

    // ── Configuration panel handlers ──────────────────────
    // Each panel renders a section of the configuration page.

    void _panelSysteem  (EthernetClient& c);  // System: CDE short-circuit response etc.
    void _panelInternet (EthernetClient& c);  // Network: IP, MAC, hostname, DHCP
    void _panelLenzLan  (EthernetClient& c);  // LenzLAN: port, timeout
    void _panelZ21      (EthernetClient& c);  // Z21 LAN: settings
    void _panelWebIf    (EthernetClient& c);  // Web interface: port, auth
    void _panelDccHal   (EthernetClient& c);  // DCC: speed steps, RailCom, prog track
    void _panelRs485    (EthernetClient& c);  // XpressNet RS-485: pins, timing
    void _panelRsBus    (EthernetClient& c);  // RS-Bus: pins, poll interval
    void _panelXpressNet(EthernetClient& c);  // XpressNet: broadcast flags
    void _panelFabriek  (EthernetClient& c);  // Factory reset panel
    void _panelLocos    (EthernetClient& c);  // Loco table with clear button
    void _panelTurnouts (EthernetClient& c);  // Turnout table with clear button
    void _panelFeedback (EthernetClient& c);  // RS-Bus feedback table with clear button

    // Renders the JSON status object into the response
    void _buildStatusJson(EthernetClient& c);

    // ── URL and form utility functions ────────────────────

    // Returns true if path exactly matches expected (case-sensitive)
    static bool _pathEquals(const char* path, const char* expected);

    // Decodes a URL-encoded string in-place (%xx → char, + → space)
    static void _urlDecode(char* str);

    // Finds the value of a named parameter in a URL-encoded form body.
    // Writes up to maxLen bytes to value. Returns pointer to value or nullptr.
    static const char* _findParam(const char* body, const char* key,
                                   char* value, uint8_t maxLen);
};

extern Webserver gWebserver;
