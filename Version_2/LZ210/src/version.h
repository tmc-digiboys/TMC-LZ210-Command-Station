// ═══════════════════════════════════════════════════════════════
//  version.h  —  central, single-source-of-truth version constants
//
//  Previously, the project's own firmware version existed only as a
//  literal string in LZ210.ino's boot log line ("LZ210 v1.0 boot")
//  and its header comment — not reachable from any other file (an
//  .ino isn't cleanly #include-able the way a genuine header is), and
//  nowhere for e.g. the web interface or OLED display to show it
//  without duplicating the literal string by hand and risking it
//  drifting out of sync. Likewise, "v2 hardware" appears throughout
//  the codebase (12+ places) purely as a COMMENT — there was no
//  actual, checkable constant distinguishing hardware revisions at
//  all.
//
//  This file is deliberately tiny and focused — nothing but version
//  identifiers — so it's trivial to #include from anywhere (LZ210.ino,
//  webserver.cpp for a "System Info" panel, oled_display.cpp for a
//  boot-screen version line, ...) without pulling in anything else.
//
//  Bump LZ210_FW_VERSION_* here when releasing a new firmware version;
//  nothing else needs to change to keep every display of the version
//  number (boot log, web interface, OLED, ...) consistent.
// ═══════════════════════════════════════════════════════════════
#pragma once

#define LZ210_FW_VERSION_MAJOR 1
#define LZ210_FW_VERSION_MINOR 0
// Human-readable combined string, e.g. for direct printing —
// kept in sync with MAJOR/MINOR above by hand (simple enough at only
// two components; revisit with a small sprintf-based helper if this
// ever grows a patch/build number too).
#define LZ210_FW_VERSION_STR "1.0"

// Hardware revision this build targets. Was previously only ever
// documented in scattered comments ("v2 hardware: ...") with no
// actual constant to check or display — this doesn't change any of
// that gating (each such comment still just describes what the
// surrounding code assumes), but gives a single, genuine value for
// anything that wants to REPORT which hardware revision a running
// build was compiled for (web interface, OLED, boot log).
#define LZ210_HW_REVISION 2
