// ═══════════════════════════════════════════════════════════════
//  led_status.cpp  —  LED status controller
//
//  Defines the global LedStatus instance used by all modules
//  to signal hardware events via the four status LEDs.
// ═══════════════════════════════════════════════════════════════
#include "led_status.h"

// Global LED controller — all modules use gLeds to signal events.
// All of LedStatus's and Led's actual logic is implemented inline in
// led_status.h; this file exists solely to provide the single,
// shared gLeds instance (constructed with its four Led members
// already wired to the correct GPIO pins via their default
// constructor arguments).
LedStatus gLeds;
