// Part of readsb, a Mode-S/ADSB/TIS message decoder.
//
// interactive.c: aircraft tracking and interactive display
//
// Copyright (c) 2019 Michael Wolf <michael@mictronics.de>
//
// This code is based on a detached fork of dump1090-fa.
//
// Copyright (c) 2014,2015 Oliver Jowett <oliver@mutability.co.uk>
//
// This file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// This file incorporates work covered by the following copyright and
// license:
//
// Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//  *  Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//  *  Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "readsb.h"
#include <strings.h>

#ifndef DISABLE_INTERACTIVE
#include <curses.h>
#endif

#ifdef DISABLE_INTERACTIVE
void interactiveInit() {}
void interactiveCleanup(void) {}
void interactiveShowData(void) {}
#else

//
//========================= Interactive mode ===============================

static double convert_distance(int meter) {
    if (Modes.metric)
        return meter / 1000.0;
    else
        return meter / 1852.0;
}

static int convert_altitude(int ft) {
    if (Modes.metric)
        return (ft / 3.2828);
    else
        return ft;
}

static int convert_speed(int kts) {
    if (Modes.metric)
        return (kts * 1.852);
    else
        return kts;
}

// External declarations for distance and bearing calculations
extern double greatcircle(double lat0, double lon0, double lat1, double lon1, int approx);
extern double bearing(double lat0, double lon0, double lat1, double lon1);

//
//=========================================================================
//
// Show the currently captured interactive data on screen.
//

// Ownship input state for viewadsb
static char ownship_input[16];      // Buffer for ownship input
static int ownship_input_len = 0;   // Current input length
static int ownship_input_active = 0; // 1 if currently entering ownship
static int ownship_initialized = 0;  // 1 after first ownship setup from command line

// Local ownship tracking for viewadsb
static uint32_t viewadsb_ownship_hex = 0;
static char viewadsb_ownship_callsign[9] = "";

// NIC/NACp clamping input state for viewadsb
static int nicnacp_input_active = 0;
static char nicnacp_input[16];
static int nicnacp_input_len = 0;
static int viewadsb_nic_min = 0;
static int viewadsb_nacp_min = 0;

// Sort mode for viewadsb
typedef enum {
    SORT_DISTANCE = 0,
    SORT_ALTITUDE,
    SORT_CALLSIGN,
    SORT_CATEGORY,
    SORT_SPEED,
    SORT_RSSI,
    SORT_COUNT
} sort_mode_t;

static sort_mode_t current_sort_mode = SORT_DISTANCE;
static int sort_ascending = 1;  // 1 = ascending, 0 = descending
static int sort_input_active = 0;  // 1 if selecting sort mode

static const char *sort_mode_names[] = {
    "Distance",
    "Altitude",
    "Callsign",
    "Category",
    "Speed",
    "RSSI"
};

// Check if aircraft matches the local viewadsb ownship
static int isOwnship(struct aircraft *a) {
    // Check by hex ID (mask off non-ICAO address flag)
    if (viewadsb_ownship_hex != 0 && (a->addr & 0xFFFFFF) == viewadsb_ownship_hex) {
        return 1;
    }
    // Check by callsign (case-insensitive, handle trailing spaces)
    if (viewadsb_ownship_callsign[0] != '\0') {
        char callsign[9];
        strncpy(callsign, a->callsign, 8);
        callsign[8] = '\0';
        // Trim trailing spaces
        for (int i = 7; i >= 0 && callsign[i] == ' '; i--) {
            callsign[i] = '\0';
        }
        if (strcasecmp(callsign, viewadsb_ownship_callsign) == 0) {
            return 1;
        }
    }
    return 0;
}

// Get reference position for distance calculations
// Returns 1 if position is valid, 0 otherwise
// If ownship is set and has valid position, use ownship
// Otherwise use receiver position
static int getRefPosition(double *lat, double *lon, double *alt_m) {
    // In viewadsb mode, check if ownship is set and has valid position
    if (Modes.viewadsb && (viewadsb_ownship_hex != 0 || viewadsb_ownship_callsign[0] != '\0')) {
        struct craftArray *ca = &Modes.aircraftActive;
        for (int i = 0; i < ca->len; i++) {
            struct aircraft *a = ca->list[i];
            if (a && isOwnship(a)) {
                if (trackDataValid(&a->position_valid)) {
                    *lat = a->lat;
                    *lon = a->lon;
                    // Get altitude in meters - prefer barometric altitude
                    if (trackDataValid(&a->baro_alt_valid)) {
                        *alt_m = a->baro_alt * 0.3048;  // feet to meters
                    } else if (trackDataValid(&a->geom_alt_valid)) {
                        *alt_m = a->geom_alt * 0.3048;  // feet to meters
                    } else {
                        *alt_m = 0;
                    }
                    return 1;
                }
                break;
            }
        }
    }
    
    // Fall back to receiver position
    if (Modes.fUserLat != 0 || Modes.fUserLon != 0) {
        *lat = Modes.fUserLat;
        *lon = Modes.fUserLon;
        // fUserAlt == -2e6 means not set, treat as 0
        *alt_m = (Modes.fUserAlt > -1e6) ? Modes.fUserAlt : 0;
        return 1;
    }
    
    return 0;
}

// Calculate 3D distance including altitude difference
// Returns distance in meters
static double calculate3DDistance(double ref_lat, double ref_lon, double ref_alt_m,
                                   double tgt_lat, double tgt_lon, double tgt_alt_m) {
    // Get 2D great circle distance
    double horiz_dist = greatcircle(ref_lat, ref_lon, tgt_lat, tgt_lon, 0);
    
    // Calculate vertical distance
    double vert_dist = tgt_alt_m - ref_alt_m;
    
    // 3D distance using Pythagoras
    return sqrt(horiz_dist * horiz_dist + vert_dist * vert_dist);
}

// Set ownship from input string (hex or callsign)
static void setOwnshipFromInput(const char *input) {
    // Mark as initialized - user is now in control
    ownship_initialized = 1;
    
    if (!input || input[0] == '\0') {
        // Clear ownship - the periodic function will send the clear command
        viewadsb_ownship_hex = 0;
        viewadsb_ownship_callsign[0] = '\0';
        return;
    }

    // Check if input looks like hex (1-6 hex characters)
    int is_hex = 1;
    int len = strlen(input);
    if (len > 6) is_hex = 0;
    for (int i = 0; i < len && is_hex; i++) {
        char c = input[i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))) {
            is_hex = 0;
        }
    }

    if (is_hex && len <= 6) {
        // Treat as hex ID
        viewadsb_ownship_hex = (uint32_t)strtol(input, NULL, 16);
        viewadsb_ownship_callsign[0] = '\0';
    } else {
        // Treat as callsign
        viewadsb_ownship_hex = 0;
        strncpy(viewadsb_ownship_callsign, input, 8);
        viewadsb_ownship_callsign[8] = '\0';
        // Convert to uppercase
        for (int i = 0; viewadsb_ownship_callsign[i]; i++) {
            if (viewadsb_ownship_callsign[i] >= 'a' && viewadsb_ownship_callsign[i] <= 'z') {
                viewadsb_ownship_callsign[i] -= 32;
            }
        }
    }
    // The periodic interactiveSendOwnship() will detect the change and send to server
}

// Initialize ownship from command line (called after connection established)
// This is called periodically to handle reconnections
void interactiveSendOwnship(void) {
    if (!Modes.viewadsb) return;

    // Track what we last sent to avoid repeated sends
    static uint32_t last_sent_hex = 0;
    static char last_sent_callsign[9] = "";
    static int64_t last_connect_time = 0;

    // Check if we have an active connection
    if (Modes.net_connectors_count == 0) return;
    struct net_connector *con = &Modes.net_connectors[0];
    if (!con->connected || con->fd < 0) return;

    // Detect reconnection by checking if connection timestamp changed
    int reconnected = (con->lastConnect != last_connect_time);
    if (reconnected) {
        last_connect_time = con->lastConnect;
        // Reset last sent values to force resend after reconnection
        last_sent_hex = 0;
        last_sent_callsign[0] = '\0';
    }

    // Check if ownship was set from command line (first-time setup only)
    // Once initialized, user controls ownship via interactive input
    if (!ownship_initialized) {
        if (Modes.ownship_hex != 0) {
            viewadsb_ownship_hex = Modes.ownship_hex;
            ownship_initialized = 1;
        } else if (Modes.ownship_callsign[0] != '\0') {
            strncpy(viewadsb_ownship_callsign, Modes.ownship_callsign, 8);
            viewadsb_ownship_callsign[8] = '\0';
            ownship_initialized = 1;
        }
    }

    // Only send if changed from what we last sent
    if (viewadsb_ownship_hex != 0) {
        if (viewadsb_ownship_hex != last_sent_hex || last_sent_callsign[0] != '\0') {
            char hexstr[8];
            snprintf(hexstr, sizeof(hexstr), "%06X", viewadsb_ownship_hex);
            sendOwnshipCommand(con->fd, 'H', hexstr);
            last_sent_hex = viewadsb_ownship_hex;
            last_sent_callsign[0] = '\0';
        }
    } else if (viewadsb_ownship_callsign[0] != '\0') {
        if (strcmp(viewadsb_ownship_callsign, last_sent_callsign) != 0 || last_sent_hex != 0) {
            sendOwnshipCommand(con->fd, 'C', viewadsb_ownship_callsign);
            strncpy(last_sent_callsign, viewadsb_ownship_callsign, 8);
            last_sent_callsign[8] = '\0';
            last_sent_hex = 0;
        }
    } else {
        // No ownship set - send clear if we previously had one
        if (last_sent_hex != 0 || last_sent_callsign[0] != '\0') {
            sendOwnshipCommand(con->fd, 'X', NULL);
            last_sent_hex = 0;
            last_sent_callsign[0] = '\0';
        }
    }
}

// Parse NIC/NACp input: "8,9" -> nic=8 nacp=9, "8" -> nic=8 nacp=8
static void setNicNacpFromInput(const char *input) {
    if (!input || input[0] == '\0') {
        viewadsb_nic_min = 0;
        viewadsb_nacp_min = 0;
        return;
    }

    int nic = 0, nacp = 0;
    const char *comma = strchr(input, ',');
    if (comma) {
        nic = atoi(input);
        nacp = atoi(comma + 1);
    } else {
        nic = atoi(input);
        nacp = nic;
    }

    if (nic < 0) nic = 0;
    if (nic > 15) nic = 15;
    if (nacp < 0) nacp = 0;
    if (nacp > 15) nacp = 15;

    viewadsb_nic_min = nic;
    viewadsb_nacp_min = nacp;
}

// Periodically send NIC/NACp clamping to server (viewadsb only)
void interactiveSendNicNacp(void) {
    if (!Modes.viewadsb) return;

    static int last_sent_nic_min = -1;
    static int last_sent_nacp_min = -1;
    static int64_t last_connect_time = 0;

    if (Modes.net_connectors_count == 0) return;
    struct net_connector *con = &Modes.net_connectors[0];
    if (!con->connected || con->fd < 0) return;

    // Detect reconnection
    int reconnected = (con->lastConnect != last_connect_time);
    if (reconnected) {
        last_connect_time = con->lastConnect;
        last_sent_nic_min = -1;
        last_sent_nacp_min = -1;
    }

    // Only send if changed
    if (viewadsb_nic_min != last_sent_nic_min || viewadsb_nacp_min != last_sent_nacp_min) {
        sendNicNacpClampCommand(con->fd, viewadsb_nic_min, viewadsb_nacp_min);
        last_sent_nic_min = viewadsb_nic_min;
        last_sent_nacp_min = viewadsb_nacp_min;
    }
}

// While ncurses owns the screen, anything written to stderr (SDR messages, gain
// changes, warnings) would scribble over the display. Redirect stderr into a pipe
// and turn each line into an event on the feed status page instead.
static int stderrPipe = -1;   // read end
static int stderrSaved = -1;  // original stderr, restored on cleanup

static void interactiveCaptureStderr(void) {
    int fds[2];
    if (pipe(fds) != 0) {
        return;
    }
    fcntl(fds[0], F_SETFL, fcntl(fds[0], F_GETFL, 0) | O_NONBLOCK);
    fcntl(fds[1], F_SETFL, fcntl(fds[1], F_GETFL, 0) | O_NONBLOCK); // never block a writer, drop instead
    fflush(stderr);
    stderrSaved = dup(STDERR_FILENO);
    if (dup2(fds[1], STDERR_FILENO) < 0) {
        close(fds[0]);
        close(fds[1]);
        stderrSaved = -1;
        return;
    }
    close(fds[1]);
    stderrPipe = fds[0];
}

static void interactiveDrainStderr(void) {
    static char acc[FEED_EVENT_LEN];
    static int accLen;
    if (stderrPipe < 0) {
        return;
    }
    char buf[4096];
    ssize_t n;
    while ((n = read(stderrPipe, buf, sizeof(buf))) > 0) {
        for (ssize_t i = 0; i < n; i++) {
            char ch = buf[i];
            if (ch == '\n' || ch == '\r') {
                if (accLen > 0) {
                    acc[accLen] = '\0';
                    // strip the "[timestamp] " prefix of log_with_timestamp lines, events carry their own time
                    char *msg = acc;
                    if (msg[0] == '[') {
                        char *close = strstr(msg, "] ");
                        if (close) {
                            msg = close + 2;
                        }
                    }
                    feedEvent("%s", msg);
                }
                accLen = 0;
            } else if (accLen < (int) sizeof(acc) - 1) {
                acc[accLen++] = ch;
            }
        }
    }
}

static void interactiveRestoreStderr(void) {
    if (stderrSaved >= 0) {
        interactiveDrainStderr();
        fflush(stderr);
        dup2(stderrSaved, STDERR_FILENO);
        close(stderrSaved);
        stderrSaved = -1;
    }
    if (stderrPipe >= 0) {
        close(stderrPipe);
        stderrPipe = -1;
    }
}

void interactiveInit() {
    if (!Modes.interactive)
        return;

    interactiveCaptureStderr();

    initscr();
    // Initialize color support
    if (has_colors()) {
        start_color();
        use_default_colors();  // Allow -1 for default background
        // Define color pairs: pair number, foreground, background
        init_pair(1, COLOR_MAGENTA, -1);  // Magenta on default background
        init_pair(2, COLOR_GREEN, -1);    // Green on default background
        init_pair(3, COLOR_CYAN, -1);     // Cyan on default background (ownship)
        init_pair(4, COLOR_YELLOW, -1);   // Yellow: degraded link
        init_pair(5, COLOR_RED, -1);      // Red: link down / stale
    }

    // Enable non-blocking keyboard input
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    noecho();  // Don't echo typed characters

    clear();
    refresh();
}

// Print a single aircraft row with colors
// Returns 1 if printed, 0 if skipped
static int printAircraftRow(struct aircraft *a, int row, int64_t now, int is_ownship) {
    if (!a) return 0;
    if ((now - a->seen) >= Modes.interactive_display_ttl) return 0;

    int msgs = a->messages;
    if (msgs <= 1) return 0;

    char strSquawk[5] = " ";
    char strFl[7] = " ";
    char strGPSAlt[7] = " ";
    char strBaroRate[6] = " ";
    char strGeomRate[6] = " ";
    char strQNH[5] = " ";
    char strSelAlt[6] = " ";
    char strSelHdg[4] = " ";
    char strAPMode[13] = " ";
    char strCategory[3] = " ";
    char strTt[5] = " ";
    char strMagHdg[5] = " ";
    char strGs[5] = " ";
    char strIAS[5] = " ";
    char strTAS[5] = " ";
    char strMach[6] = " ";
    char strTrackRate[5] = " ";
    char strRoll[6] = " ";
    char strOAT[5] = " ";
    char strTAT[5] = " ";
    char strWS[5] = " ";
    char strWD[5] = " ";
    char strNIC[4] = " ";
    char strNACp[5] = " ";
    char strDist[7] = " ";
    char strBrg[4] = " ";

    if (trackDataValid(&a->squawk_valid)) {
        snprintf(strSquawk, 5, "%04x", a->squawk);
    }

    // ADS-B emitter category (A0-D7)
    if (a->category != 0) {
        snprintf(strCategory, 3, "%c%d",
                 'A' + ((a->category >> 4) - 0xA),
                 a->category & 0x0F);
    }

    if (trackDataValid(&a->gs_valid)) {
        snprintf(strGs, 5, "%3d", convert_speed(a->gs));
    }

    if (trackDataValid(&a->ias_valid)) {
        snprintf(strIAS, 5, "%3d", convert_speed(a->ias));
    }

    if (trackDataValid(&a->tas_valid)) {
        snprintf(strTAS, 5, "%3d", convert_speed(a->tas));
    }

    if (a->oat != 0.0) {
        snprintf(strOAT, 5, "%3.0f", a->oat);
    }

    if (a->tat != 0.0) {
        snprintf(strTAT, 5, "%3.0f", a->tat);
    }

    // NIC (Navigation Integrity Category) - from last computed position
    if (trackDataValid(&a->position_valid)) {
        snprintf(strNIC, 4, "%3u", a->pos_nic);
    }

    // NACp (Navigation Accuracy Category - Position)
    if (trackDataValid(&a->nac_p_valid)) {
        snprintf(strNACp, 5, "%4u", a->nac_p);
    }

    float windage_secs = (now - a->wind_updated)/1000;
    if (windage_secs < 10.0) {
        snprintf(strWS, 5, "%3.0f", a->wind_speed);
        snprintf(strWD, 5, "%3.0f", a->wind_direction);
    } else {
        strncpy(strWS, "  \0", 3);
        strncpy(strWD, "  \0", 3);
    }

    if (trackDataValid(&a->mach_valid)) {
        snprintf(strMach, 6, "%5.3f", a->mach);
    }

    if (trackDataValid(&a->track_valid)) {
        snprintf(strTt, 5, "%03.0f", a->track);
    }

    // Magnetic heading
    if (trackDataValid(&a->mag_heading_valid)) {
        snprintf(strMagHdg, 5, "%03.0f", a->mag_heading);
    }

    if (trackDataValid(&a->track_rate_valid)) {
        snprintf(strTrackRate, 5, "%4.1f", a->track_rate);
    }

    // Roll angle (positive = right wing down)
    if (trackDataValid(&a->roll_valid)) {
        snprintf(strRoll, 6, "%5.1f", a->roll);
    }

    if (msgs > 99999) {
        msgs = 0;
    }

    char strMode[5] = " ";
    char strLat[8] = " ";
    char strLon[9] = " ";
    double * pSig = a->signalLevel;
    double signalAverage = (pSig[0] + pSig[1] + pSig[2] + pSig[3] +
            pSig[4] + pSig[5] + pSig[6] + pSig[7]) / 8.0;

    strMode[0] = 'S';
    if (a->modeA_hit) {
        strMode[0] = 'a';
    }
    if (a->modeC_hit) {
        strMode[0] = 'c';
    }

    if (trackDataValid(&a->position_valid)) {
        snprintf(strLat, 8, "%7.03f", a->lat);
        snprintf(strLon, 9, "%8.03f", a->lon);
        
        // Calculate distance and bearing from reference position
        double ref_lat, ref_lon, ref_alt_m;
        if (getRefPosition(&ref_lat, &ref_lon, &ref_alt_m)) {
            // Get target altitude in meters - prefer barometric altitude
            double tgt_alt_m = 0;
            if (trackDataValid(&a->baro_alt_valid)) {
                tgt_alt_m = a->baro_alt * 0.3048;
            } else if (trackDataValid(&a->geom_alt_valid)) {
                tgt_alt_m = a->geom_alt * 0.3048;
            }
            
            // Calculate 3D distance
            double dist_m = calculate3DDistance(ref_lat, ref_lon, ref_alt_m,
                                                 a->lat, a->lon, tgt_alt_m);
            double dist_display = convert_distance((int)dist_m);
            
            if (dist_display < 100) {
                snprintf(strDist, 7, "%5.1f", dist_display);
            } else {
                snprintf(strDist, 7, "%5.0f", dist_display);
            }
            
            // Calculate bearing
            double brg = bearing(ref_lat, ref_lon, a->lat, a->lon);
            snprintf(strBrg, 4, "%03.0f", brg);
        }
    }

    if (trackDataValid(&a->airground_valid) && a->airground == AG_GROUND) {
        snprintf(strFl, 7, " grnd");
    } else if (Modes.use_gnss && trackDataValid(&a->geom_alt_valid)) {
        snprintf(strFl, 7, "%5dH", convert_altitude(a->geom_alt));
    } else if (trackDataValid(&a->baro_alt_valid)) {
        snprintf(strFl, 7, "%5d ", convert_altitude(a->baro_alt));
    }

    if (trackDataValid(&a->geom_alt_valid)) {
        snprintf(strGPSAlt, 7, "%5d", convert_altitude(a->geom_alt));
    }

    // Barometric vertical rate (fpm)
    if (trackDataValid(&a->baro_rate_valid)) {
        snprintf(strBaroRate, 6, "%5d", a->baro_rate);
    }

    // Geometric vertical rate (fpm)
    if (trackDataValid(&a->geom_rate_valid)) {
        snprintf(strGeomRate, 6, "%5d", a->geom_rate);
    }

    // QNH (altimeter setting in millibars)
    if (trackDataValid(&a->nav_qnh_valid)) {
        snprintf(strQNH, 5, "%4.0f", a->nav_qnh);
    }

    // Selected altitude (prefer MCP, fallback to FMS)
    if (trackDataValid(&a->nav_altitude_mcp_valid)) {
        snprintf(strSelAlt, 6, "%5d", a->nav_altitude_mcp);
    } else if (trackDataValid(&a->nav_altitude_fms_valid)) {
        snprintf(strSelAlt, 6, "%5d", a->nav_altitude_fms);
    }

    // Selected heading
    if (trackDataValid(&a->nav_heading_valid)) {
        snprintf(strSelHdg, 4, "%3.0f", a->nav_heading);
    }

    // Autopilot modes: AP=Autopilot, Vn=VNAV, Ah=AltHold, Ap=aPProach, Ln=LNAV, Tc=TCAS
    if (trackDataValid(&a->nav_modes_valid)) {
        int idx = 0;
        if (a->nav_modes & NAV_MODE_AUTOPILOT) { strAPMode[idx++] = 'A'; strAPMode[idx++] = 'P'; }
        if (a->nav_modes & NAV_MODE_VNAV)      { strAPMode[idx++] = 'V'; strAPMode[idx++] = 'n'; }
        if (a->nav_modes & NAV_MODE_ALT_HOLD)  { strAPMode[idx++] = 'A'; strAPMode[idx++] = 'h'; }
        if (a->nav_modes & NAV_MODE_APPROACH)  { strAPMode[idx++] = 'A'; strAPMode[idx++] = 'p'; }
        if (a->nav_modes & NAV_MODE_LNAV)      { strAPMode[idx++] = 'L'; strAPMode[idx++] = 'n'; }
        if (a->nav_modes & NAV_MODE_TCAS)      { strAPMode[idx++] = 'T'; strAPMode[idx++] = 'c'; }
        strAPMode[idx] = '\0';
    }

    // Print row with colors
    move(row, 0);

    // Ownship: Cyan for Hex and Flight ID
    if (is_ownship && has_colors()) {
        attron(COLOR_PAIR(3));
        printw("%s%06X", (a->addr & MODES_NON_ICAO_ADDRESS) ? "~" : " ", (a->addr & 0xffffff));
        attroff(COLOR_PAIR(3));
        printw(" %s  %-4s  ", strMode, strSquawk);
        attron(COLOR_PAIR(3));
        printw("%-8s", a->callsign);
        attroff(COLOR_PAIR(3));
        printw(" %2s ", strCategory);
    } else {
        // Normal: White for Hex, Mode, Sqwk, Flight, Category
        printw("%s%06X %s  %-4s  %-8s %2s ",
               (a->addr & MODES_NON_ICAO_ADDRESS) ? "~" : " ", (a->addr & 0xffffff),
               strMode, strSquawk, a->callsign, strCategory);
    }

    // Magenta: APMode, QNH, SelAlt, SelHdg
    if (has_colors()) attron(COLOR_PAIR(1));
    printw("%-12s %4s %5s   %3s", strAPMode, strQNH, strSelAlt, strSelHdg);
    if (has_colors()) attroff(COLOR_PAIR(1));

    // White: BaroAlt, BaRt, GPSAlt, GmRt, GSP, IAS, TAS, Mach
    printw(" %6s %5s %6s %5s  %3s  %3s  %3s %5s  ",
           strFl, strBaroRate, strGPSAlt, strGeomRate, strGs, strIAS, strTAS, strMach);

    // Green: OAT, TAT, WD, WS
    if (has_colors()) attron(COLOR_PAIR(2));
    printw("%3s  %3s  %3s  %3s", strOAT, strTAT, strWD, strWS);
    if (has_colors()) attroff(COLOR_PAIR(2));

    // White: Ttk, MHd, TkR, Roll, Lat, Long, Dist, Brg, NIC, NACp, RSSI, Msgs, Ti
    printw("  %3s  %3s  %4s %5s %7s %8s %5s %3s %3s %4s %5.1f %5d %2.0f",
           strTt, strMagHdg, strTrackRate, strRoll,
           strLat, strLon, strDist, strBrg, strNIC, strNACp, 10 * log10(signalAverage), msgs, (now - a->seen) / 1000.0);

    return 1;
}
void interactiveCleanup(void) {
    if (Modes.interactive) {
        endwin();
        interactiveRestoreStderr();
    }
}

static int compareDist(const void *p1, const void *p2) {
    struct aircraft *a1 = *(struct aircraft**) p1;
    struct aircraft *a2 = *(struct aircraft**) p2;
    if (a1 == NULL)
        return 1;
    if (a2 == NULL)
        return -1;

    int valid1 = trackDataValid(&a1->position_valid);
    int valid2 = trackDataValid(&a2->position_valid);
    if (!valid1 && !valid2) {
        return a1->addr - a2->addr;
    }
    if (valid1 != valid2) {
        return valid2 - valid1;
    }
    
    // Calculate 3D distance for comparison
    double ref_lat, ref_lon, ref_alt_m;
    if (!getRefPosition(&ref_lat, &ref_lon, &ref_alt_m)) {
        // No reference position, use 2D receiver_distance
        if (a1->receiver_distance == a2->receiver_distance)
            return a1->addr - a2->addr;
        return (a1->receiver_distance > a2->receiver_distance) ? 1 : -1;
    }
    
    // Get altitudes in meters - prefer barometric altitude
    double alt1_m = 0, alt2_m = 0;
    if (trackDataValid(&a1->baro_alt_valid)) {
        alt1_m = a1->baro_alt * 0.3048;
    } else if (trackDataValid(&a1->geom_alt_valid)) {
        alt1_m = a1->geom_alt * 0.3048;
    }
    if (trackDataValid(&a2->baro_alt_valid)) {
        alt2_m = a2->baro_alt * 0.3048;
    } else if (trackDataValid(&a2->geom_alt_valid)) {
        alt2_m = a2->geom_alt * 0.3048;
    }
    
    double dist1 = calculate3DDistance(ref_lat, ref_lon, ref_alt_m, a1->lat, a1->lon, alt1_m);
    double dist2 = calculate3DDistance(ref_lat, ref_lon, ref_alt_m, a2->lat, a2->lon, alt2_m);
    
    if (dist1 == dist2)
        return a1->addr - a2->addr;
    return (dist1 > dist2) ? 1 : -1;
}

// Compare by altitude (barometric preferred)
static int compareAlt(const void *p1, const void *p2) {
    struct aircraft *a1 = *(struct aircraft**) p1;
    struct aircraft *a2 = *(struct aircraft**) p2;
    if (a1 == NULL) return 1;
    if (a2 == NULL) return -1;

    int valid1 = trackDataValid(&a1->baro_alt_valid) || trackDataValid(&a1->geom_alt_valid);
    int valid2 = trackDataValid(&a2->baro_alt_valid) || trackDataValid(&a2->geom_alt_valid);
    
    // Aircraft on ground go to the end
    int g1 = trackDataValid(&a1->airground_valid) && a1->airground == AG_GROUND;
    int g2 = trackDataValid(&a2->airground_valid) && a2->airground == AG_GROUND;
    if (g1 && !g2) return sort_ascending ? 1 : -1;
    if (!g1 && g2) return sort_ascending ? -1 : 1;
    if (g1 && g2) return a1->addr - a2->addr;

    if (!valid1 && !valid2) return a1->addr - a2->addr;
    if (!valid1) return 1;
    if (!valid2) return -1;

    int alt1 = trackDataValid(&a1->baro_alt_valid) ? a1->baro_alt : a1->geom_alt;
    int alt2 = trackDataValid(&a2->baro_alt_valid) ? a2->baro_alt : a2->geom_alt;

    if (alt1 == alt2) return a1->addr - a2->addr;
    int result = (alt1 > alt2) ? 1 : -1;
    return sort_ascending ? result : -result;
}

// Compare by callsign
static int compareCallsign(const void *p1, const void *p2) {
    struct aircraft *a1 = *(struct aircraft**) p1;
    struct aircraft *a2 = *(struct aircraft**) p2;
    if (a1 == NULL) return 1;
    if (a2 == NULL) return -1;

    int has1 = a1->callsign[0] != '\0';
    int has2 = a2->callsign[0] != '\0';
    
    if (!has1 && !has2) return a1->addr - a2->addr;
    if (!has1) return 1;
    if (!has2) return -1;

    int result = strcmp(a1->callsign, a2->callsign);
    if (result == 0) return a1->addr - a2->addr;
    return sort_ascending ? result : -result;
}

// Compare by category
static int compareCategory(const void *p1, const void *p2) {
    struct aircraft *a1 = *(struct aircraft**) p1;
    struct aircraft *a2 = *(struct aircraft**) p2;
    if (a1 == NULL) return 1;
    if (a2 == NULL) return -1;

    if (a1->category == 0 && a2->category == 0) return a1->addr - a2->addr;
    if (a1->category == 0) return 1;
    if (a2->category == 0) return -1;

    if (a1->category == a2->category) return a1->addr - a2->addr;
    int result = (a1->category > a2->category) ? 1 : -1;
    return sort_ascending ? result : -result;
}

// Compare by ground speed
static int compareSpeed(const void *p1, const void *p2) {
    struct aircraft *a1 = *(struct aircraft**) p1;
    struct aircraft *a2 = *(struct aircraft**) p2;
    if (a1 == NULL) return 1;
    if (a2 == NULL) return -1;

    int valid1 = trackDataValid(&a1->gs_valid);
    int valid2 = trackDataValid(&a2->gs_valid);
    
    if (!valid1 && !valid2) return a1->addr - a2->addr;
    if (!valid1) return 1;
    if (!valid2) return -1;

    if (a1->gs == a2->gs) return a1->addr - a2->addr;
    int result = (a1->gs > a2->gs) ? 1 : -1;
    return sort_ascending ? result : -result;
}

// Compare by RSSI
static int compareRssi(const void *p1, const void *p2) {
    struct aircraft *a1 = *(struct aircraft**) p1;
    struct aircraft *a2 = *(struct aircraft**) p2;
    if (a1 == NULL) return 1;
    if (a2 == NULL) return -1;

    double rssi1 = (a1->signalLevel[0] + a1->signalLevel[1] + a1->signalLevel[2] + a1->signalLevel[3] +
                    a1->signalLevel[4] + a1->signalLevel[5] + a1->signalLevel[6] + a1->signalLevel[7]) / 8.0;
    double rssi2 = (a2->signalLevel[0] + a2->signalLevel[1] + a2->signalLevel[2] + a2->signalLevel[3] +
                    a2->signalLevel[4] + a2->signalLevel[5] + a2->signalLevel[6] + a2->signalLevel[7]) / 8.0;

    if (rssi1 == rssi2) return a1->addr - a2->addr;
    int result = (rssi1 > rssi2) ? 1 : -1;
    return sort_ascending ? result : -result;
}

// Wrapper for compareDist that respects sort direction
static int compareDistSorted(const void *p1, const void *p2) {
    int result = compareDist(p1, p2);
    return sort_ascending ? result : -result;
}

// Get the appropriate compare function for current sort mode
static int (*getCompareFunction(void))(const void *, const void *) {
    switch (current_sort_mode) {
        case SORT_ALTITUDE:  return compareAlt;
        case SORT_CALLSIGN:  return compareCallsign;
        case SORT_CATEGORY:  return compareCategory;
        case SORT_SPEED:     return compareSpeed;
        case SORT_RSSI:      return compareRssi;
        case SORT_DISTANCE:
        default:             return compareDistSorted;
    }
}

static int64_t next_clear;

// Keyboard handling for the interactive display
static void interactiveHandleKeys(void) {
    int ch;
    while ((ch = getch()) != ERR) {
        if (ch == 'f' || ch == 'F') {
            // toggle feed status page (readsb and viewadsb)
            Modes.feed_page = !Modes.feed_page;
            next_clear = 0;
            continue;
        }
        if (!Modes.viewadsb) {
            continue;
        }
        {
            if (sort_input_active) {
                // Sort selection mode
                if (ch == 27 || ch == 's' || ch == 'S') {  // ESC or 's' to exit sort mode
                    sort_input_active = 0;
                } else if (ch >= '1' && ch <= '6') {
                    sort_mode_t new_mode = (sort_mode_t)(ch - '1');
                    if (new_mode == current_sort_mode) {
                        // Same mode - toggle direction
                        sort_ascending = !sort_ascending;
                    } else {
                        // New mode - reset to ascending
                        current_sort_mode = new_mode;
                        sort_ascending = 1;
                    }
                    sort_input_active = 0;
                } else if (ch == 'd' || ch == 'D') {
                    current_sort_mode = SORT_DISTANCE;
                    sort_input_active = 0;
                } else if (ch == 'a' || ch == 'A') {
                    current_sort_mode = SORT_ALTITUDE;
                    sort_input_active = 0;
                } else if (ch == 'c' || ch == 'C') {
                    current_sort_mode = SORT_CALLSIGN;
                    sort_input_active = 0;
                } else if (ch == 't' || ch == 'T') {  // 't' for type/category
                    current_sort_mode = SORT_CATEGORY;
                    sort_input_active = 0;
                } else if (ch == 'g' || ch == 'G') {  // 'g' for ground speed
                    current_sort_mode = SORT_SPEED;
                    sort_input_active = 0;
                } else if (ch == 'r' || ch == 'R') {
                    current_sort_mode = SORT_RSSI;
                    sort_input_active = 0;
                }
            } else if (nicnacp_input_active) {
                // NIC/NACp input mode
                if (ch == 27) {  // ESC key - cancel input
                    nicnacp_input_len = 0;
                    nicnacp_input[0] = '\0';
                    nicnacp_input_active = 0;
                } else if (ch == '\n' || ch == '\r') {  // Enter - submit
                    if (nicnacp_input_len > 0) {
                        nicnacp_input[nicnacp_input_len] = '\0';
                        setNicNacpFromInput(nicnacp_input);
                    } else {
                        setNicNacpFromInput(NULL);
                    }
                    nicnacp_input_len = 0;
                    nicnacp_input[0] = '\0';
                    nicnacp_input_active = 0;
                } else if (ch == KEY_BACKSPACE || ch == 127 || ch == 8) {  // Backspace
                    if (nicnacp_input_len > 0) {
                        nicnacp_input_len--;
                        nicnacp_input[nicnacp_input_len] = '\0';
                    }
                    if (nicnacp_input_len == 0) {
                        nicnacp_input_active = 0;
                    }
                } else if (((ch >= '0' && ch <= '9') || ch == ',') && nicnacp_input_len < 15) {
                    nicnacp_input[nicnacp_input_len++] = (char)ch;
                    nicnacp_input[nicnacp_input_len] = '\0';
                }
            } else if (ownship_input_active) {
                // Ownship input mode
                if (ch == 27) {  // ESC key - cancel input
                    ownship_input_len = 0;
                    ownship_input[0] = '\0';
                    ownship_input_active = 0;
                } else if (ch == '\n' || ch == '\r') {  // Enter - submit
                    if (ownship_input_len > 0) {
                        ownship_input[ownship_input_len] = '\0';
                        setOwnshipFromInput(ownship_input);
                    } else {
                        // Empty input clears ownship
                        setOwnshipFromInput(NULL);
                    }
                    ownship_input_len = 0;
                    ownship_input[0] = '\0';
                    ownship_input_active = 0;
                } else if (ch == KEY_BACKSPACE || ch == 127 || ch == 8) {  // Backspace
                    if (ownship_input_len > 0) {
                        ownship_input_len--;
                        ownship_input[ownship_input_len] = '\0';
                    }
                    if (ownship_input_len == 0) {
                        ownship_input_active = 0;
                    }
                } else if (ch >= 32 && ch < 127 && ownship_input_len < 15) {  // Printable char
                    ownship_input[ownship_input_len++] = (char)ch;
                    ownship_input[ownship_input_len] = '\0';
                }
            } else {
                // Normal mode - check for command keys
                if (ch == 'o' || ch == 'O') {
                    ownship_input_active = 1;
                    ownship_input_len = 0;
                    ownship_input[0] = '\0';
                } else if (ch == 'n' || ch == 'N') {
                    nicnacp_input_active = 1;
                    nicnacp_input_len = 0;
                    nicnacp_input[0] = '\0';
                } else if (ch == 's' || ch == 'S') {
                    sort_input_active = 1;
                } else if (ch == 'q' || ch == 'Q') {
                    // Quick toggle sort direction
                    sort_ascending = !sort_ascending;
                }
            }
        }
    }

}


static void feedFmtDuration(char *buf, int len, int64_t sec) {
    if (sec < 0) {
        sec = 0;
    }
    if (sec >= 86400) {
        snprintf(buf, len, "%lldd%02lld:%02lld", (long long) (sec / 86400), (long long) ((sec / 3600) % 24), (long long) ((sec / 60) % 60));
    } else {
        snprintf(buf, len, "%02lld:%02lld:%02lld", (long long) (sec / 3600), (long long) ((sec / 60) % 60), (long long) (sec % 60));
    }
}

static void feedFmtCount(char *buf, int len, uint64_t n) {
    if (n >= 10000000ULL) {
        snprintf(buf, len, "%.1fM", n / 1e6);
    } else if (n >= 10000ULL) {
        snprintf(buf, len, "%.1fk", n / 1e3);
    } else {
        snprintf(buf, len, "%llu", (unsigned long long) n);
    }
}

static int feedEventCompare(const void *a, const void *b) {
    const struct feedEvent *ea = a;
    const struct feedEvent *eb = b;
    if (ea->time < eb->time) return -1;
    if (ea->time > eb->time) return 1;
    return 0;
}

// Feed status page: decoder summary and per-connector link health stay at the
// top, the event log scrolls underneath.
static void feedPageDraw(int64_t now) {
    struct feedStatus *fs = &Modes.feed;
    int rows = getmaxy(stdscr);
    int cols = getmaxx(stdscr);
    char buf[64];

    erase();

    // --- decoder line ---
    move(0, 0);
    if (has_colors()) attron(A_BOLD);
    printw("Decoder ");
    if (has_colors()) attroff(A_BOLD);
    int stale = fs->updated == 0 || now - fs->updated > 5 * SECONDS;
    if (stale) {
        if (has_colors()) attron(COLOR_PAIR(5) | A_BOLD);
        if (fs->updated == 0) {
            printw("no status received from readsb yet");
        } else {
            printw("STALE: no status from readsb for %lld s", (long long) ((now - fs->updated) / 1000));
        }
        if (has_colors()) attroff(COLOR_PAIR(5) | A_BOLD);
    } else {
        printw("%6.0f msg/s %5.0f pos/s %4d acft  ", fs->msgRate, fs->posRate, fs->aircraft);
        if (fs->gain == MODES_AUTO_GAIN || fs->gain == MODES_MAX_GAIN || fs->gain == 0) {
            printw("gain auto  ");
        } else {
            printw("gain %.1f dB  ", fs->gain / 10.0);
        }
        if (fs->samplesLostRate > 0) {
            if (has_colors()) attron(COLOR_PAIR(4));
            printw("lost samples %.0f/s  ", fs->samplesLostRate);
            if (has_colors()) attroff(COLOR_PAIR(4));
        }
        feedFmtDuration(buf, sizeof(buf), fs->uptimeSec);
        printw("up %s  ", buf);
        if (fs->gdl90Connected) {
            if (has_colors()) attron(COLOR_PAIR(2));
            printw("GDL90: EFB connected");
            if (has_colors()) attroff(COLOR_PAIR(2));
        } else {
            printw("GDL90: no EFB");
        }
        if (Modes.viewadsb && (Modes.received_efb_ownship_rate > 0 || Modes.received_efb_traffic_rate > 0)) {
            printw(" (own %.1fHz tfc %.1fHz)", Modes.received_efb_ownship_rate, Modes.received_efb_traffic_rate);
        }
    }

    // --- connector table ---
    move(1, 0);
    if (has_colors()) attron(A_BOLD);
    printw("%-30s %-18s %-5s %-9s %6s %6s %7s %6s %6s %7s %5s %7s %5s %4s %5s %3s %-9s %s",
            "Feed", "Protocol", "State", "Since", "msg/s", "kB/s", "Sent", "Loss1m", "LossT", "Ping", "PLoss", "TcpRTT", "Rtx", "KrnQ", "SendQ", "Rc", "Outage", "Last error");
    if (has_colors()) attroff(A_BOLD);
    mvhline(2, 0, ACS_HLINE, cols);

    int row = 3;
    if (fs->connCount == 0) {
        mvprintw(row++, 0, "(no net-connectors configured)");
    }
    for (int i = 0; i < fs->connCount && row < rows - 3; i++) {
        struct feedConnStatus *st = &fs->conn[i];
        char name[48];
        snprintf(name, sizeof(name), "%s:%s", st->name, st->port);
        char since[32], outage[32], sent[16], rtt[16], rtx[16], infl[16], l1[16], lt[16], ping[16], ploss[16];
        feedFmtDuration(since, sizeof(since), st->sinceSec);
        feedFmtDuration(outage, sizeof(outage), st->outageSec);
        feedFmtCount(sent, sizeof(sent), st->msgsSent);
        if (st->rtt >= 0) {
            snprintf(rtt, sizeof(rtt), "%dms", st->rtt);
        } else {
            snprintf(rtt, sizeof(rtt), "-");
        }
        if (st->ping >= 0) {
            snprintf(ping, sizeof(ping), "%dms", st->ping);
        } else {
            snprintf(ping, sizeof(ping), "-");
        }
        if (st->pingLoss >= 0) {
            snprintf(ploss, sizeof(ploss), "%d%%", st->pingLoss);
        } else {
            snprintf(ploss, sizeof(ploss), "-");
        }
        if (st->retrans >= 0) {
            feedFmtCount(rtx, sizeof(rtx), (uint64_t) st->retrans);
        } else {
            snprintf(rtx, sizeof(rtx), "-");
        }
        if (st->unacked >= 0) {
            snprintf(infl, sizeof(infl), "%dk", (st->unacked + 512) / 1024);
        } else {
            snprintf(infl, sizeof(infl), "-");
        }
        if (st->loss1m >= 0) {
            snprintf(l1, sizeof(l1), "%.1f%%", st->loss1m);
            snprintf(lt, sizeof(lt), "%.1f%%", st->lossTotal);
        } else {
            snprintf(l1, sizeof(l1), "-");
            snprintf(lt, sizeof(lt), "-");
        }

        const char *stateStr;
        int pair;
        if (stale) {
            stateStr = "?";
            pair = 5;
        } else if (st->state == 2 && st->dropActive) {
            stateStr = "DROP";
            pair = 4;
        } else if (st->state == 2) {
            stateStr = "UP";
            pair = 2;
        } else if (st->state == 1) {
            stateStr = "CONN";
            pair = 4;
        } else {
            stateStr = "DOWN";
            pair = 5;
        }

        mvprintw(row, 0, "%-30.30s %-18.18s ", name, st->protocol);
        if (has_colors()) attron(COLOR_PAIR(pair) | A_BOLD);
        printw("%-5s", stateStr);
        if (has_colors()) attroff(COLOR_PAIR(pair) | A_BOLD);
        printw(" %-9s %6.0f %6.1f %7s ", since, st->msgRate, st->byteRate / 1000.0, sent);

        int lossPair = (st->loss1m > 10) ? 5 : (st->loss1m > 0.5 ? 4 : 0);
        if (lossPair && has_colors()) attron(COLOR_PAIR(lossPair));
        printw("%6s %6s", l1, lt);
        if (lossPair && has_colors()) attroff(COLOR_PAIR(lossPair));

        int pingPair = (st->ping > 5000 || (st->ping < 0 && st->pingLoss > 0)) ? 5 : (st->ping > 2500 ? 4 : 0);
        if (pingPair && has_colors()) attron(COLOR_PAIR(pingPair));
        printw(" %7s", ping);
        if (pingPair && has_colors()) attroff(COLOR_PAIR(pingPair));

        int plossPair = (st->pingLoss > 25) ? 5 : (st->pingLoss > 5 ? 4 : 0);
        if (plossPair && has_colors()) attron(COLOR_PAIR(plossPair));
        printw(" %5s", ploss);
        if (plossPair && has_colors()) attroff(COLOR_PAIR(plossPair));

        int rttPair = (st->rtt > 2000) ? 5 : (st->rtt > 800 ? 4 : 0);
        if (rttPair && has_colors()) attron(COLOR_PAIR(rttPair));
        printw(" %7s", rtt);
        if (rttPair && has_colors()) attroff(COLOR_PAIR(rttPair));

        printw(" %5s %4s %4d%% %3d %-9s ", rtx, infl, st->sendqPct, st->reconnects, outage);
        if (st->state != 2 && st->lastErr[0]) {
            if (has_colors()) attron(COLOR_PAIR(5));
            printw("%.40s", st->lastErr);
            if (has_colors()) attroff(COLOR_PAIR(5));
        } else if (st->lastErr[0]) {
            printw("(last: %.32s)", st->lastErr);
        }
        row++;
    }

    // --- events ---
    mvhline(row, 0, ACS_HLINE, cols);
    mvprintw(row, 2, " Events ");
    row++;

    int avail = rows - 1 - row;
    if (avail < 0) {
        avail = 0;
    }
    // events arrive out of order in viewadsb (replayed history after local ones),
    // so show them sorted by time
    static struct feedEvent sorted[FEED_EVENTS_MAX];
    int first = (fs->eventHead - fs->eventCount + FEED_EVENTS_MAX) % FEED_EVENTS_MAX;
    for (int i = 0; i < fs->eventCount; i++) {
        sorted[i] = fs->events[(first + i) % FEED_EVENTS_MAX];
    }
    qsort(sorted, fs->eventCount, sizeof(struct feedEvent), feedEventCompare);
    int show = fs->eventCount < avail ? fs->eventCount : avail;
    int skip = fs->eventCount - show;
    for (int i = 0; i < show; i++) {
        struct feedEvent *e = &sorted[skip + i];
        time_t t = e->time / 1000;
        struct tm tm;
        localtime_r(&t, &tm);
        int pair = 0;
        if (strstr(e->msg, "established") || strstr(e->msg, "recovered") || strstr(e->msg, "Discovered")) {
            pair = 2;
        } else if (strstr(e->msg, "congested") || strstr(e->msg, "suppressing")) {
            pair = 4;
        } else if (strstr(e->msg, "timed out") || strstr(e->msg, "failed") || strstr(e->msg, "disconnected")
                || strstr(e->msg, "Error") || strstr(e->msg, "reconnecting") || strstr(e->msg, "Couldn't")) {
            pair = 5;
        }
        mvprintw(row + i, 0, "%02d:%02d:%02d ", tm.tm_hour, tm.tm_min, tm.tm_sec);
        if (pair && has_colors()) attron(COLOR_PAIR(pair));
        printw("%.*s", cols - 10, e->msg);
        if (pair && has_colors()) attroff(COLOR_PAIR(pair));
    }

    // --- bottom line ---
    move(rows - 1, 0);
    clrtoeol();
    printw("Feed status  [f]=aircraft list");
    if (Modes.viewadsb) {
        printw("  [o]=ownship [n]=NIC/NACp");
    }
    if (fs->eventCount) {
        printw("   %d events", fs->eventCount);
    }
}

void interactiveShowData(void) {
    static int64_t next_update;
    int64_t now = mstime();
    char progress;
    char spinner[4] = "|/-\\";

    // Refresh screen every (MODES_INTERACTIVE_REFRESH_TIME) miliseconde
    if (now < next_update)
        return;

    next_update = now + MODES_INTERACTIVE_REFRESH_TIME;

    interactiveDrainStderr();

    // Periodically send ownship and NIC/NACp clamping to server (viewadsb only)
    if (Modes.viewadsb) {
        interactiveSendOwnship();
        interactiveSendNicNacp();
    }

    interactiveHandleKeys();

    if (Modes.feed_page) {
        feedPageDraw(now);
        refresh();
        return;
    }

    // clear potential errors every 2 seconds
    if (now > next_clear) {
        next_clear = now + 2 * SECONDS;
        clear();
        
        // Print receiver info line (viewadsb only)
        if (Modes.viewadsb) {
            move(0, 0);
            
            // Count valid targets
            struct craftArray *ca_count = &Modes.aircraftActive;
            int target_count = 0;
            for (int i = 0; i < ca_count->len; i++) {
                struct aircraft *a = ca_count->list[i];
                if (a && a->messages > 1 && (now - a->seen) < Modes.interactive_display_ttl) {
                    target_count++;
                }
            }
            printw("Tgt:%d", target_count);
            
            // Show gain if received from readsb
            if (Modes.received_gain != 0 && Modes.received_gain != MODES_MAX_GAIN && Modes.received_gain != MODES_AUTO_GAIN) {
                printw("  Gain:%.1fdB", Modes.received_gain / 10.0);
            }
            
            printw("  ");
            
            double ref_lat, ref_lon, ref_alt_m;
            if (getRefPosition(&ref_lat, &ref_lon, &ref_alt_m)) {
                const char *pos_source = (viewadsb_ownship_hex != 0 || viewadsb_ownship_callsign[0] != '\0') ? "Ownship" : "Receiver";
                if (has_colors()) attron(COLOR_PAIR(3));
                printw("%s: %.4f, %.4f", pos_source, ref_lat, ref_lon);
                if (ref_alt_m != 0) {
                    if (Modes.metric) {
                        printw(", %.0fm", ref_alt_m);
                    } else {
                        printw(", %.0fft", ref_alt_m / 0.3048);
                    }
                }
                if (has_colors()) attroff(COLOR_PAIR(3));
            } else {
                printw("No receiver position set (use --lat/--lon or set ownship)");
            }
            
            // Show EFB update rates if connected
            if (Modes.received_efb_ownship_rate > 0 || Modes.received_efb_traffic_rate > 0) {
                printw("  ");
                if (has_colors()) attron(COLOR_PAIR(2));
                printw("EFB: Own:%.1fHz Tfc:%.1fHz", 
                       Modes.received_efb_ownship_rate, 
                       Modes.received_efb_traffic_rate);
                if (has_colors()) attroff(COLOR_PAIR(2));
            }
        }
        
        // print header with colors matching data columns
        int header_row = Modes.viewadsb ? 1 : 0;
        move(header_row, 0);
        printw(" Hex    M  Sqwk  Flight   Ct ");
        if (has_colors()) attron(COLOR_PAIR(1));
        printw("APMode        QNH SelAl SelHd");
        if (has_colors()) attroff(COLOR_PAIR(1));
        printw(" BaroAlt  BaRt GPSAlt  GmRt  GSP  IAS  TAS Mach   ");
        if (has_colors()) attron(COLOR_PAIR(2));
        printw("OAT  TAT  WD   WS");
        if (has_colors()) attroff(COLOR_PAIR(2));
        printw("  Ttk  MHd  TkR  Roll   Lat      Long   Dist Brg NIC NACp  RSSI  Msgs  Ti");
        mvhline(header_row + 1, 0, ACS_HLINE, 205);
    }


    progress = spinner[(now / 1000) % 4];
    int header_row = Modes.viewadsb ? 1 : 0;
    mvaddch(header_row, 205, progress);

    int rows = getmaxy(stdscr);
    int row = Modes.viewadsb ? 3 : 2;

    struct craftArray *ca = &Modes.aircraftActive;

    // sort active list (use read lock - display only operation)
    ca_lock_read(ca);
    qsort(ca->list, ca->len, sizeof(struct aircraft *), getCompareFunction());

    // Find and print ownship first (viewadsb mode only)
    struct aircraft *ownship_printed = NULL;
    if (Modes.viewadsb && (viewadsb_ownship_hex != 0 || viewadsb_ownship_callsign[0] != '\0')) {
        for (int i = 0; i < ca->len; i++) {
            struct aircraft *a = ca->list[i];
            if (a && isOwnship(a)) {
                if (printAircraftRow(a, row, now, 1)) {
                    row++;
                    ownship_printed = a;
                }
                break;
            }
        }
    }

    // Print all other aircraft
    for (int i = 0; i < ca->len; i++) {
        struct aircraft *a = ca->list[i];

        if (a && row < rows) {
            // Skip ownship if already printed first
            if (a == ownship_printed) continue;

            int is_own = Modes.viewadsb ? isOwnship(a) : 0;
            if (printAircraftRow(a, row, now, is_own)) {
                row++;
            }
        }
    }
    ca_unlock_read(ca);

    if (Modes.mode_ac) {
        for (unsigned i = 1; i < 4096 && row < rows; ++i) {
            if (modeAC_match[i] || modeAC_count[i] < 50 || modeAC_age[i] > 5)
                continue;

            char strMode[5] = "  A ";
            char strFl[7] = " ";
            unsigned modeA = indexToModeA(i);
            int modeC = modeAToModeC(modeA);
            if (modeC != INVALID_ALTITUDE) {
                strMode[3] = 'C';
                snprintf(strFl, 7, "%5d ", convert_altitude(modeC * 100));
            }

            mvprintw(row, 0,
                    "%7s %-4s  %04x  %-8s %6s %3s  %3s  %7s %8s %5s %5d %2d\n",
                    "", /* address */
                    strMode, /* mode */
                    modeA, /* squawk */
                    "", /* callsign */
                    strFl, /* altitude */
                    "", /* gs */
                    "", /* heading */
                    "", /* lat */
                    "", /* lon */
                    "", /* signal */
                    modeAC_count[i], /* messages */
                    modeAC_age[i]); /* age */
            ++row;
        }
    }


    move(row, 0);
    clrtobot();

    // Display status/input at bottom of screen (viewadsb only)
    if (Modes.viewadsb) {
        int bottom_row = getmaxy(stdscr) - 1;
        move(bottom_row, 0);
        clrtoeol();

        if (sort_input_active) {
            // Show sort selection menu
            printw("Sort: ");
            for (int i = 0; i < SORT_COUNT; i++) {
                if (i == (int)current_sort_mode) {
                    if (has_colors()) attron(COLOR_PAIR(3) | A_BOLD);
                    printw("[%d:%s%s] ", i + 1, sort_mode_names[i], sort_ascending ? "+" : "-");
                    if (has_colors()) attroff(COLOR_PAIR(3) | A_BOLD);
                } else {
                    printw("%d:%s ", i + 1, sort_mode_names[i]);
                }
            }
            printw(" (ESC=cancel)");
        } else if (nicnacp_input_active) {
            // Show NIC/NACp input prompt
            if (has_colors()) attron(COLOR_PAIR(3));
            printw("Min NIC,NACp (0-15): %s_", nicnacp_input);
            if (has_colors()) attroff(COLOR_PAIR(3));
            printw("  (Enter=set, ESC=cancel)");
        } else if (ownship_input_active) {
            // Show ownship input prompt
            if (has_colors()) attron(COLOR_PAIR(3));
            printw("Ownship: %s_", ownship_input);
            if (has_colors()) attroff(COLOR_PAIR(3));
            printw("  (Enter=set, ESC=cancel)");
        } else {
            // Show normal status
            printw("Sort:%s%s ", sort_mode_names[current_sort_mode], sort_ascending ? "+" : "-");
            if (viewadsb_ownship_hex != 0) {
                printw(" Ownship:%06X", viewadsb_ownship_hex);
            } else if (viewadsb_ownship_callsign[0] != '\0') {
                printw(" Ownship:%s", viewadsb_ownship_callsign);
            }
            if (viewadsb_nic_min > 0 || viewadsb_nacp_min > 0) {
                printw(" NIC/NACp min:%d/%d", viewadsb_nic_min, viewadsb_nacp_min);
            }
            printw("  [o]=ownship [n]=NIC/NACp [s]=sort [q]=reverse [f]=feed status");
        }
    }

    refresh();
}

#endif

//
//=========================================================================
//
