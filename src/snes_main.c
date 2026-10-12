/**
 * SNESRecomp - Native Windows / Cross-Platform Port Runner (snes_main.c)
 *
 * Kirby Super Star Native Recompilation Port
 * Features:
 * - 1080p Borderless Fullscreen with custom Kirby Super Star Bezel artwork
 * - F10 Bezel Toggle (Seamless switch between Bezel & Clean Fullscreen with black pillarboxes)
 * - F1 / F2 Quick Save State & Quick Load State with on-screen HUD toast feedback
 * - F9 Controller Preset Switcher (Xbox Standard, Switch, Trigger Action, Classic 1:1)
 * - F11 In-game Display / Settings Context Menu (replaces Escape)
 * - Esc Instant Clean Exit (safely flushes saves and closes)
 * - Isolated saves/ folder for battery saves (.srm) and save states (.state)
 * - Native 16:9 Widescreen PPU rendering (352x224) & 16:10 Widescreen (320x224)
 * - Runtime Aspect Ratio Switcher (16:9 Native, 16:10 Native, 4:3 CRT, 8:7 PAR, 16:9 Stretched)
 * - Upscaling Filters (Nearest Neighbor, Bilinear, Retro CRT Scanlines)
 * - 44.1 kHz 16-bit Stereo SPC700 / S-DSP Audio via Win32 waveOut
 * - XInput Gamepad (D-pad, Left Stick, Face buttons, Triggers LT/RT, Guide / Back+Start hotkey)
 */

#define _CRT_SECURE_NO_WARNINGS
#include "snes_runtime.h"
#include "game_recomp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mmsystem.h>
#endif

typedef enum {
    ASPECT_16_9_NATIVE = 0,  /* 352x224 True Native Widescreen (48px extra L/R) */
    ASPECT_16_10_NATIVE = 1, /* 320x224 True Native Widescreen (32px extra L/R) */
    ASPECT_4_3_CRT = 2,      /* 256x224 displayed at 4:3 CRT aspect ratio */
    ASPECT_8_7_PAR = 3,      /* 256x224 displayed at 8:7 pixel-perfect ratio */
    ASPECT_16_9_STRETCH = 4  /* 256x224 stretched to 16:9 */
} AspectMode;

typedef enum {
    FILTER_NEAREST = 0,
    FILTER_BILINEAR = 1,
    FILTER_SCANLINES = 2
} FilterMode;

typedef enum {
    PAD_PRESET_XBOX_STANDARD = 0,
    PAD_PRESET_NINTENDO_SWITCH = 1,
    PAD_PRESET_TRIGGER_ACTION = 2,
    PAD_PRESET_CLASSIC_1TO1 = 3
} PadPreset;

static snes_context_t* g_snes_ctx = NULL;
static AspectMode      g_aspect_mode = ASPECT_16_9_NATIVE;
static FilterMode      g_filter_mode = FILTER_NEAREST;
static int             g_window_scale = 3;
static bool            g_fullscreen = true;  /* Default to Fullscreen on initial launch */
static bool            g_bezel_enabled = true; /* Default to Custom Bezel active */
static bool            g_paused = false;
static bool            g_muted = false;
static bool            g_fast_forward = false;
static bool            g_gamepad_enabled = true;
static int             g_deadzone = 7849;
static int             g_current_preset = PAD_PRESET_XBOX_STANDARD;

static char            g_exe_dir[1024] = ".";
static char            g_sram_path[1152] = "saves\\kirby_super_star.srm";
static char            g_state_path[1152] = "saves\\state1.state";
static char            g_settings_path[1152] = "config.ini";
static char            g_settings_hyphen_path[1152] = "launch_settings.json";

/* On-screen Toast notification state */
static char            g_toast_msg[128] = "";
static DWORD           g_toast_expiry = 0;

static void show_toast(const char* msg) {
    if (!msg) return;
    strncpy(g_toast_msg, msg, sizeof(g_toast_msg) - 1);
    g_toast_msg[sizeof(g_toast_msg) - 1] = '\0';
    g_toast_expiry = GetTickCount() + 2500;
}

/* 8x8 bitmap font (ASCII 32 to 126) for on-screen HUD */
static const uint8_t g_font8x8[95][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* ' ' */
    {0x10, 0x10, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00}, /* '!' */
    {0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* '"' */
    {0x10, 0x78, 0x10, 0x78, 0x20, 0x00, 0x00, 0x00}, /* '#' */
    {0x38, 0x50, 0x30, 0x28, 0x78, 0x20, 0x00, 0x00}, /* '$' */
    {0x78, 0x10, 0x20, 0x3c, 0x5c, 0x00, 0x00, 0x00}, /* '%' */
    {0x50, 0x60, 0x68, 0x50, 0x78, 0x00, 0x00, 0x00}, /* '&' */
    {0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* ''' */
    {0x10, 0x20, 0x20, 0x20, 0x20, 0x10, 0x08, 0x00}, /* '(' */
    {0x10, 0x08, 0x08, 0x08, 0x08, 0x10, 0x20, 0x00}, /* ')' */
    {0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* '*' */
    {0x10, 0x10, 0x7c, 0x10, 0x10, 0x00, 0x00, 0x00}, /* '+' */
    {0x00, 0x00, 0x00, 0x00, 0x10, 0x30, 0x00, 0x00}, /* ',' */
    {0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00}, /* '-' */
    {0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00}, /* '.' */
    {0x10, 0x10, 0x20, 0x20, 0x40, 0x40, 0x00, 0x00}, /* '/' */
    {0x30, 0x58, 0x78, 0x68, 0x30, 0x00, 0x00, 0x00}, /* '0' */
    {0x30, 0x10, 0x10, 0x10, 0x38, 0x00, 0x00, 0x00}, /* '1' */
    {0x38, 0x28, 0x08, 0x10, 0x38, 0x00, 0x00, 0x00}, /* '2' */
    {0x38, 0x08, 0x18, 0x08, 0x38, 0x00, 0x00, 0x00}, /* '3' */
    {0x30, 0x30, 0x50, 0x78, 0x10, 0x00, 0x00, 0x00}, /* '4' */
    {0x38, 0x20, 0x38, 0x08, 0x30, 0x00, 0x00, 0x00}, /* '5' */
    {0x38, 0x40, 0x78, 0x48, 0x30, 0x00, 0x00, 0x00}, /* '6' */
    {0x70, 0x10, 0x20, 0x20, 0x40, 0x00, 0x00, 0x00}, /* '7' */
    {0x78, 0x48, 0x30, 0x48, 0x78, 0x00, 0x00, 0x00}, /* '8' */
    {0x30, 0x48, 0x78, 0x08, 0x70, 0x00, 0x00, 0x00}, /* '9' */
    {0x00, 0x10, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00}, /* ':' */
    {0x00, 0x10, 0x00, 0x00, 0x10, 0x30, 0x00, 0x00}, /* ';' */
    {0x00, 0x08, 0x10, 0x30, 0x08, 0x00, 0x00, 0x00}, /* '<' */
    {0x00, 0x38, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00}, /* '=' */
    {0x00, 0x20, 0x10, 0x18, 0x20, 0x00, 0x00, 0x00}, /* '>' */
    {0x08, 0x08, 0x38, 0x00, 0x20, 0x00, 0x00, 0x00}, /* '?' */
    {0x48, 0xb8, 0xb8, 0xb8, 0xa8, 0x80, 0x70, 0x00}, /* '@' */
    {0x10, 0x28, 0x28, 0x38, 0x48, 0x00, 0x00, 0x00}, /* 'A' */
    {0x3c, 0x24, 0x38, 0x24, 0x3c, 0x00, 0x00, 0x00}, /* 'B' */
    {0x30, 0x40, 0x40, 0x40, 0x30, 0x00, 0x00, 0x00}, /* 'C' */
    {0x70, 0x48, 0x48, 0x48, 0x70, 0x00, 0x00, 0x00}, /* 'D' */
    {0x78, 0x40, 0x70, 0x40, 0x78, 0x00, 0x00, 0x00}, /* 'E' */
    {0x78, 0x40, 0x70, 0x40, 0x40, 0x00, 0x00, 0x00}, /* 'F' */
    {0x38, 0x40, 0x40, 0x48, 0x38, 0x00, 0x00, 0x00}, /* 'G' */
    {0x48, 0x48, 0x78, 0x48, 0x48, 0x00, 0x00, 0x00}, /* 'H' */
    {0x70, 0x20, 0x20, 0x20, 0x70, 0x00, 0x00, 0x00}, /* 'I' */
    {0x38, 0x10, 0x10, 0x50, 0x20, 0x00, 0x00, 0x00}, /* 'J' */
    {0x48, 0x50, 0x60, 0x50, 0x48, 0x00, 0x00, 0x00}, /* 'K' */
    {0x40, 0x40, 0x40, 0x40, 0x78, 0x00, 0x00, 0x00}, /* 'L' */
    {0x48, 0x6c, 0x54, 0x48, 0x48, 0x00, 0x00, 0x00}, /* 'M' */
    {0x48, 0x68, 0x58, 0x48, 0x48, 0x00, 0x00, 0x00}, /* 'N' */
    {0x30, 0x48, 0x48, 0x48, 0x30, 0x00, 0x00, 0x00}, /* 'O' */
    {0x70, 0x48, 0x70, 0x40, 0x40, 0x00, 0x00, 0x00}, /* 'P' */
    {0x30, 0x48, 0x48, 0x58, 0x34, 0x00, 0x00, 0x00}, /* 'Q' */
    {0x70, 0x48, 0x70, 0x50, 0x48, 0x00, 0x00, 0x00}, /* 'R' */
    {0x38, 0x40, 0x30, 0x08, 0x70, 0x00, 0x00, 0x00}, /* 'S' */
    {0x70, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00}, /* 'T' */
    {0x48, 0x48, 0x48, 0x48, 0x30, 0x00, 0x00, 0x00}, /* 'U' */
    {0x48, 0x48, 0x48, 0x50, 0x20, 0x00, 0x00, 0x00}, /* 'V' */
    {0x48, 0x48, 0x54, 0x54, 0x28, 0x00, 0x00, 0x00}, /* 'W' */
    {0x48, 0x48, 0x30, 0x48, 0x48, 0x00, 0x00, 0x00}, /* 'X' */
    {0x88, 0x88, 0x50, 0x20, 0x20, 0x00, 0x00, 0x00}, /* 'Y' */
    {0x78, 0x08, 0x10, 0x20, 0x78, 0x00, 0x00, 0x00}, /* 'Z' */
    {0x30, 0x20, 0x20, 0x20, 0x30, 0x00, 0x00, 0x00}, /* '[' */
    {0x40, 0x40, 0x20, 0x20, 0x10, 0x10, 0x00, 0x00}, /* '\' */
    {0x30, 0x10, 0x10, 0x10, 0x30, 0x00, 0x00, 0x00}, /* ']' */
    {0x20, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* '^' */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00}, /* '_' */
    {0x20, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* '`' */
    {0x00, 0x38, 0x48, 0x48, 0x38, 0x00, 0x00, 0x00}, /* 'a' */
    {0x40, 0x58, 0x68, 0x68, 0x58, 0x00, 0x00, 0x00}, /* 'b' */
    {0x00, 0x38, 0x40, 0x40, 0x38, 0x00, 0x00, 0x00}, /* 'c' */
    {0x08, 0x38, 0x48, 0x48, 0x38, 0x00, 0x00, 0x00}, /* 'd' */
    {0x00, 0x30, 0x78, 0x40, 0x38, 0x00, 0x00, 0x00}, /* 'e' */
    {0x18, 0x20, 0x70, 0x20, 0x20, 0x00, 0x00, 0x00}, /* 'f' */
    {0x00, 0x38, 0x48, 0x38, 0x08, 0x30, 0x00, 0x00}, /* 'g' */
    {0x40, 0x70, 0x48, 0x48, 0x48, 0x00, 0x00, 0x00}, /* 'h' */
    {0x20, 0x00, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00}, /* 'i' */
    {0x10, 0x00, 0x10, 0x10, 0x10, 0x20, 0x00, 0x00}, /* 'j' */
    {0x40, 0x48, 0x70, 0x48, 0x48, 0x00, 0x00, 0x00}, /* 'k' */
    {0x20, 0x20, 0x20, 0x20, 0x10, 0x00, 0x00, 0x00}, /* 'l' */
    {0x00, 0x70, 0x54, 0x54, 0x54, 0x00, 0x00, 0x00}, /* 'm' */
    {0x00, 0x70, 0x48, 0x48, 0x48, 0x00, 0x00, 0x00}, /* 'n' */
    {0x00, 0x30, 0x48, 0x48, 0x30, 0x00, 0x00, 0x00}, /* 'o' */
    {0x00, 0x58, 0x68, 0x58, 0x40, 0x40, 0x00, 0x00}, /* 'p' */
    {0x00, 0x38, 0x48, 0x38, 0x08, 0x08, 0x00, 0x00}, /* 'q' */
    {0x00, 0x28, 0x30, 0x20, 0x20, 0x00, 0x00, 0x00}, /* 'r' */
    {0x00, 0x38, 0x40, 0x30, 0x48, 0x30, 0x00, 0x00}, /* 's' */
    {0x20, 0x70, 0x20, 0x20, 0x10, 0x00, 0x00, 0x00}, /* 't' */
    {0x00, 0x48, 0x48, 0x48, 0x30, 0x00, 0x00, 0x00}, /* 'u' */
    {0x00, 0x48, 0x48, 0x50, 0x20, 0x00, 0x00, 0x00}, /* 'v' */
    {0x00, 0x48, 0x54, 0x54, 0x28, 0x00, 0x00, 0x00}, /* 'w' */
    {0x00, 0x48, 0x30, 0x30, 0x48, 0x00, 0x00, 0x00}, /* 'x' */
    {0x00, 0x48, 0x48, 0x38, 0x08, 0x30, 0x00, 0x00}, /* 'y' */
    {0x00, 0x78, 0x10, 0x20, 0x78, 0x00, 0x00, 0x00}, /* 'z' */
    {0x10, 0x20, 0x40, 0x20, 0x10, 0x00, 0x00, 0x00}, /* '{' */
    {0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00}, /* '|' */
    {0x20, 0x10, 0x08, 0x10, 0x20, 0x00, 0x00, 0x00}, /* '}' */
    {0x20, 0x54, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00}  /* '~' */
};

static void apply_aspect_mode(AspectMode mode) {
    g_aspect_mode = mode;
    if (!g_snes_ctx) return;
    switch (mode) {
        case ASPECT_16_9_NATIVE:
            snes_runtime_set_extra_side_space(g_snes_ctx, SNES_WIDESCREEN_EXTRA_SIDE); /* 48 -> 352x224 */
            break;
        case ASPECT_16_10_NATIVE:
            snes_runtime_set_extra_side_space(g_snes_ctx, SNES_WIDESCREEN_16_10_SIDE); /* 32 -> 320x224 */
            break;
        case ASPECT_4_3_CRT:
        case ASPECT_8_7_PAR:
        case ASPECT_16_9_STRETCH:
        default:
            snes_runtime_set_extra_side_space(g_snes_ctx, 0); /* 0 -> 256x224 */
            break;
    }
}

static const char* aspect_mode_label(AspectMode mode) {
    switch (mode) {
        case ASPECT_16_9_NATIVE:  return "16:9 Widescreen (Native 352x224)";
        case ASPECT_16_10_NATIVE: return "16:10 Widescreen (Native 320x224)";
        case ASPECT_4_3_CRT:      return "4:3 Standard CRT TV";
        case ASPECT_8_7_PAR:      return "8:7 Pixel-Perfect";
        case ASPECT_16_9_STRETCH: return "16:9 Stretched";
        default:                  return "16:9 Widescreen";
    }
}

static void apply_pad_preset(int preset) {
    g_current_preset = (preset % 4 + 4) % 4;
    switch (g_current_preset) {
        case PAD_PRESET_XBOX_STANDARD:
            show_toast("CONTROLLER: Preset 1 (Xbox Standard: A=Jump, B=Attack, LT/RT=Guard)");
            break;
        case PAD_PRESET_NINTENDO_SWITCH:
            show_toast("CONTROLLER: Preset 2 (Nintendo Switch: A=Attack, B=Jump)");
            break;
        case PAD_PRESET_TRIGGER_ACTION:
            show_toast("CONTROLLER: Preset 3 (Trigger Action: RT=Attack, LT=Jump)");
            break;
        case PAD_PRESET_CLASSIC_1TO1:
            show_toast("CONTROLLER: Preset 4 (Classic 1:1 SNES Layout)");
            break;
    }
}

static void load_launch_settings_json(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return;
    char buf[4096];
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    buf[n] = '\0';

    /* Parse bezel */
    const char* bz = strstr(buf, "\"bezel\"");
    if (bz) {
        if (strstr(bz, "false")) g_bezel_enabled = false;
        else if (strstr(bz, "true")) g_bezel_enabled = true;
    }

    /* Parse preset */
    const char* ps = strstr(buf, "\"preset\"");
    if (ps) {
        const char* colon = strchr(ps, ':');
        if (colon) {
            int val = atoi(colon + 1);
            if (val >= 0 && val <= 3) g_current_preset = val;
        }
    }

    /* Parse native_widescreen boolean if present */
    if (strstr(buf, "\"native_widescreen\": false") || strstr(buf, "\"native_widescreen\":false")) {
        g_aspect_mode = ASPECT_4_3_CRT;
    } else if (strstr(buf, "\"native_widescreen\": true") || strstr(buf, "\"native_widescreen\":true")) {
        g_aspect_mode = ASPECT_16_9_NATIVE;
    }

    /* Parse aspect_ratio or aspect_mode */
    const char* ar = strstr(buf, "\"aspect_ratio\"");
    if (!ar) ar = strstr(buf, "\"aspect_mode\"");
    if (ar) {
        if (strstr(ar, "\"16:10\"")) {
            g_aspect_mode = ASPECT_16_10_NATIVE;
        } else if (strstr(ar, "\"16:9_stretch\"") || strstr(ar, "\"stretch\"")) {
            g_aspect_mode = ASPECT_16_9_STRETCH;
        } else if (strstr(ar, "\"16:9\"") || strstr(ar, "\"widescreen\"")) {
            g_aspect_mode = ASPECT_16_9_NATIVE;
        } else if (strstr(ar, "\"8:7\"") || strstr(ar, "\"pixel_perfect\"")) {
            g_aspect_mode = ASPECT_8_7_PAR;
        } else if (strstr(ar, "\"4:3\"")) {
            g_aspect_mode = ASPECT_4_3_CRT;
        }
    }

    /* Parse scale */
    const char* sc = strstr(buf, "\"scale\"");
    if (sc) {
        const char* colon = strchr(sc, ':');
        if (colon) {
            int val = atoi(colon + 1);
            if (val >= 1 && val <= 6) g_window_scale = val;
        }
    }

    /* Parse fullscreen */
    const char* fs = strstr(buf, "\"fullscreen\"");
    if (fs) {
        const char* colon = strchr(fs, ':');
        if (colon) {
            while (*colon == ':' || *colon == ' ' || *colon == '\t') colon++;
            if (strncmp(colon, "true", 4) == 0) g_fullscreen = true;
            else if (strncmp(colon, "false", 5) == 0) g_fullscreen = false;
        }
    }

    /* Parse upscaling_filter */
    const char* uf = strstr(buf, "\"upscaling_filter\"");
    if (uf) {
        if (strstr(uf, "\"bilinear\"")) g_filter_mode = FILTER_BILINEAR;
        else if (strstr(uf, "\"scanlines\"")) g_filter_mode = FILTER_SCANLINES;
        else g_filter_mode = FILTER_NEAREST;
    }

    /* Parse deadzone */
    const char* dz = strstr(buf, "\"deadzone\"");
    if (dz) {
        const char* colon = strchr(dz, ':');
        if (colon) {
            int val = atoi(colon + 1);
            if (val >= 0 && val <= 30000) g_deadzone = val;
        }
    }
}

static void write_single_settings_json(const char* path) {
    FILE* f = fopen(path, "wb");
    if (!f) return;
    const char* ar_str = "16:9";
    if (g_aspect_mode == ASPECT_16_10_NATIVE) ar_str = "16:10";
    else if (g_aspect_mode == ASPECT_4_3_CRT) ar_str = "4:3";
    else if (g_aspect_mode == ASPECT_8_7_PAR) ar_str = "8:7";
    else if (g_aspect_mode == ASPECT_16_9_STRETCH) ar_str = "16:9_stretch";

    const char* filter_str = "nearest";
    if (g_filter_mode == FILTER_BILINEAR) filter_str = "bilinear";
    else if (g_filter_mode == FILTER_SCANLINES) filter_str = "scanlines";

    bool is_ws = (g_aspect_mode == ASPECT_16_9_NATIVE || g_aspect_mode == ASPECT_16_10_NATIVE);

    fprintf(f,
        "{\n"
        "  \"fullscreen\": %s,\n"
        "  \"bezel\": %s,\n"
        "  \"preset\": %d,\n"
        "  \"native_widescreen\": %s,\n"
        "  \"aspect_ratio\": \"%s\",\n"
        "  \"aspect_mode\": \"%s\",\n"
        "  \"scale\": %d,\n"
        "  \"upscaling_filter\": \"%s\",\n"
        "  \"gamepad_enabled\": %s,\n"
        "  \"deadzone\": %d\n"
        "}\n",
        g_fullscreen ? "true" : "false",
        g_bezel_enabled ? "true" : "false",
        g_current_preset,
        is_ws ? "true" : "false",
        ar_str,
        ar_str,
        g_window_scale,
        filter_str,
        g_gamepad_enabled ? "true" : "false",
        g_deadzone
    );
    fclose(f);
}

static void save_launch_settings_json(const char* path) {
    write_single_settings_json(path);
    if (strcmp(path, g_settings_hyphen_path) != 0) {
        write_single_settings_json(g_settings_hyphen_path);
    }
}

/* Headless CLI runner for automated verification on both Windows and Linux */
static int run_headless_cli(int argc, char** argv) {
    int frames = 0;
    int press_start_at = -1;
    const char* dump_ppm = NULL;
    const char* rom_file = NULL;

    for (int i = 1; i < argc; i++) {
        if ((strcmp(argv[i], "--headless-frames") == 0 || strcmp(argv[i], "--frames") == 0) && i + 1 < argc) {
            frames = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--dump-ppm") == 0 && i + 1 < argc) {
            dump_ppm = argv[++i];
        } else if (strcmp(argv[i], "--press-start-at") == 0 && i + 1 < argc) {
            press_start_at = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--aspect") == 0 && i + 1 < argc) {
            const char* a = argv[++i];
            if (strcmp(a, "16:9") == 0) g_aspect_mode = ASPECT_16_9_NATIVE;
            else if (strcmp(a, "16:10") == 0) g_aspect_mode = ASPECT_16_10_NATIVE;
            else if (strcmp(a, "4:3") == 0) g_aspect_mode = ASPECT_4_3_CRT;
            else if (strcmp(a, "8:7") == 0) g_aspect_mode = ASPECT_8_7_PAR;
        } else if (strcmp(argv[i], "--rom") == 0 && i + 1 < argc) {
            rom_file = argv[++i];
        } else if (argv[i][0] != '-') {
            rom_file = argv[i];
        }
    }

    g_snes_ctx = snes_runtime_init();
    if (!g_snes_ctx) return 1;
    if (rom_file) {
        if (!snes_runtime_load_rom(g_snes_ctx, rom_file)) {
            fprintf(stderr, "Failed to load ROM: %s\n", rom_file);
            snes_runtime_free(g_snes_ctx);
            g_snes_ctx = NULL;
            return 2;
        }
    }
    apply_aspect_mode(g_aspect_mode);
    if (!snes_runtime_is_rom_loaded(g_snes_ctx)) {
        snes_recompiled_entry(g_snes_ctx);
    }

    for (int f = 0; f < frames; f++) {
        uint16_t btns = 0;
        if (press_start_at >= 0 && f >= press_start_at && (f % 60) < 6) {
            btns |= SNES_BTN_START;
        }
        g_snes_ctx->joy1_word = btns;
        snes_runtime_run_frame(g_snes_ctx);
    }

    int w = snes_runtime_get_frame_width(g_snes_ctx);
    int h = snes_runtime_get_frame_height(g_snes_ctx);
    printf("SNESRecomp Headless Complete: title='%s' frames=%u resolution=%dx%d widescreen=%d\n",
           snes_runtime_get_rom_title(g_snes_ctx),
           g_snes_ctx->frame_count,
           w, h,
           snes_runtime_get_widescreen(g_snes_ctx) ? 1 : 0);

    snes_runtime_free(g_snes_ctx);
    g_snes_ctx = NULL;
    return 0;
}

#ifdef _WIN32

#if defined(_MSC_VER)
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "winmm.lib")
#endif

/* Dynamic XInput definitions */
typedef struct {
    WORD  wButtons;
    BYTE  bLeftTrigger;
    BYTE  bRightTrigger;
    SHORT sThumbLX;
    SHORT sThumbLY;
    SHORT sThumbRX;
    SHORT sThumbRY;
} DYN_XINPUT_GAMEPAD;

typedef struct {
    DWORD              dwPacketNumber;
    DYN_XINPUT_GAMEPAD Gamepad;
} DYN_XINPUT_STATE;

typedef DWORD (WINAPI *PFN_XInputGetState)(DWORD dwUserIndex, DYN_XINPUT_STATE* pState);
static PFN_XInputGetState g_pXInputGetState = NULL;
static bool               g_xinput_searched = false;

#define XINPUT_BTN_DPAD_UP        0x0001
#define XINPUT_BTN_DPAD_DOWN      0x0002
#define XINPUT_BTN_DPAD_LEFT      0x0004
#define XINPUT_BTN_DPAD_RIGHT     0x0008
#define XINPUT_BTN_START          0x0010
#define XINPUT_BTN_BACK           0x0020
#define XINPUT_BTN_LEFT_THUMB     0x0040
#define XINPUT_BTN_RIGHT_THUMB    0x0080
#define XINPUT_BTN_LEFT_SHOULDER  0x0100
#define XINPUT_BTN_RIGHT_SHOULDER 0x0200
#define XINPUT_BTN_A              0x1000
#define XINPUT_BTN_B              0x2000
#define XINPUT_BTN_X              0x4000
#define XINPUT_BTN_Y              0x8000

static void init_dynamic_xinput(void) {
    if (g_xinput_searched) return;
    g_xinput_searched = true;
    const char* dlls[] = { "xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll" };
    for (int i = 0; i < 3; i++) {
        HMODULE mod = LoadLibraryA(dlls[i]);
        if (mod) {
            g_pXInputGetState = (PFN_XInputGetState)GetProcAddress(mod, "XInputGetState");
            if (g_pXInputGetState) break;
        }
    }
}

/* Menu Command IDs */
#define IDM_AR_16_9_NATIVE    2001
#define IDM_AR_16_10_NATIVE   2002
#define IDM_AR_4_3_CRT        2003
#define IDM_AR_8_7_PAR        2004
#define IDM_AR_16_9_STRETCH   2005

#define IDM_SCALE_1X          2011
#define IDM_SCALE_2X          2012
#define IDM_SCALE_3X          2013
#define IDM_SCALE_4X          2014
#define IDM_SCALE_5X          2015

#define IDM_FILTER_NEAREST    2021
#define IDM_FILTER_BILINEAR   2022
#define IDM_FILTER_SCANLINES  2023

#define IDM_TOGGLE_FULLSCREEN 2030
#define IDM_TOGGLE_BEZEL      2031
#define IDM_CYCLE_PRESET      2032
#define IDM_QUICK_SAVE        2040
#define IDM_QUICK_LOAD        2041
#define IDM_RESET_GAME        2042
#define IDM_PAUSE_GAME        2043
#define IDM_MUTE_AUDIO        2044
#define IDM_EXIT_GAME         2049

static HWND            g_hwnd = NULL;
static HMENU           g_menu_bar = NULL;
static WINDOWPLACEMENT g_wp_prev = { sizeof(WINDOWPLACEMENT) };

#define MASTER_FB_W 1920
#define MASTER_FB_H 1080
static uint32_t* g_master_fb = NULL;
static uint32_t* g_bezel_pixels = NULL;
static bool      g_bezel_loaded = false;

/* Win32 waveOut Streaming Audio */
#define AUDIO_BLOCK_COUNT    6
#define AUDIO_BLOCK_SAMPLES  (SNES_AUDIO_SAMPLES_PER_FRAME * 2)
static HWAVEOUT      g_wave_out = NULL;
static WAVEHDR       g_wave_hdrs[AUDIO_BLOCK_COUNT];
static int16_t       g_wave_bufs[AUDIO_BLOCK_COUNT][AUDIO_BLOCK_SAMPLES];
static int           g_wave_next = 0;

typedef MMRESULT (WINAPI *PFN_waveOutOpen)(LPHWAVEOUT, UINT, LPCWAVEFORMATEX, DWORD_PTR, DWORD_PTR, DWORD);
typedef MMRESULT (WINAPI *PFN_waveOutPrepareHeader)(HWAVEOUT, LPWAVEHDR, UINT);
typedef MMRESULT (WINAPI *PFN_waveOutWrite)(HWAVEOUT, LPWAVEHDR, UINT);
typedef MMRESULT (WINAPI *PFN_waveOutUnprepareHeader)(HWAVEOUT, LPWAVEHDR, UINT);
typedef MMRESULT (WINAPI *PFN_waveOutReset)(HWAVEOUT);
typedef MMRESULT (WINAPI *PFN_waveOutClose)(HWAVEOUT);
typedef MMRESULT (WINAPI *PFN_timeBeginPeriod)(UINT);
typedef MMRESULT (WINAPI *PFN_timeEndPeriod)(UINT);

static PFN_waveOutOpen            g_pfnWaveOutOpen = NULL;
static PFN_waveOutPrepareHeader   g_pfnWaveOutPrepareHeader = NULL;
static PFN_waveOutWrite           g_pfnWaveOutWrite = NULL;
static PFN_waveOutUnprepareHeader g_pfnWaveOutUnprepareHeader = NULL;
static PFN_waveOutReset           g_pfnWaveOutReset = NULL;
static PFN_waveOutClose           g_pfnWaveOutClose = NULL;
static PFN_timeBeginPeriod        g_pfnTimeBeginPeriod = NULL;
static PFN_timeEndPeriod          g_pfnTimeEndPeriod = NULL;
static bool                       g_winmm_loaded = false;

static void init_dynamic_winmm(void) {
    if (g_winmm_loaded) return;
    g_winmm_loaded = true;
    HMODULE h = LoadLibraryA("winmm.dll");
    if (!h) return;
    g_pfnWaveOutOpen            = (PFN_waveOutOpen)GetProcAddress(h, "waveOutOpen");
    g_pfnWaveOutPrepareHeader   = (PFN_waveOutPrepareHeader)GetProcAddress(h, "waveOutPrepareHeader");
    g_pfnWaveOutWrite           = (PFN_waveOutWrite)GetProcAddress(h, "waveOutWrite");
    g_pfnWaveOutUnprepareHeader = (PFN_waveOutUnprepareHeader)GetProcAddress(h, "waveOutUnprepareHeader");
    g_pfnWaveOutReset           = (PFN_waveOutReset)GetProcAddress(h, "waveOutReset");
    g_pfnWaveOutClose           = (PFN_waveOutClose)GetProcAddress(h, "waveOutClose");
    g_pfnTimeBeginPeriod        = (PFN_timeBeginPeriod)GetProcAddress(h, "timeBeginPeriod");
    g_pfnTimeEndPeriod          = (PFN_timeEndPeriod)GetProcAddress(h, "timeEndPeriod");
}

static void init_win32_audio(void) {
    init_dynamic_winmm();
    if (!g_pfnWaveOutOpen || !g_pfnWaveOutPrepareHeader || !g_pfnWaveOutWrite) return;

    WAVEFORMATEX wfx;
    memset(&wfx, 0, sizeof(wfx));
    wfx.wFormatTag      = WAVE_FORMAT_PCM;
    wfx.nChannels       = SNES_AUDIO_CHANNELS;
    wfx.nSamplesPerSec  = SNES_AUDIO_SAMPLE_RATE;
    wfx.wBitsPerSample  = 16;
    wfx.nBlockAlign     = (wfx.nChannels * wfx.wBitsPerSample) / 8;
    wfx.nAvgBytesPerSec = wfx.nSamplesPerSec * wfx.nBlockAlign;

    if (g_pfnWaveOutOpen(&g_wave_out, WAVE_MAPPER, &wfx, 0, 0, CALLBACK_NULL) != MMSYSERR_NOERROR) {
        g_wave_out = NULL;
        return;
    }

    for (int i = 0; i < AUDIO_BLOCK_COUNT; i++) {
        memset(&g_wave_hdrs[i], 0, sizeof(WAVEHDR));
        g_wave_hdrs[i].lpData         = (LPSTR)g_wave_bufs[i];
        g_wave_hdrs[i].dwBufferLength = AUDIO_BLOCK_SAMPLES * sizeof(int16_t);
        g_pfnWaveOutPrepareHeader(g_wave_out, &g_wave_hdrs[i], sizeof(WAVEHDR));
        g_wave_hdrs[i].dwFlags |= WHDR_DONE;
    }
}

static void pump_win32_audio(void) {
    if (!g_wave_out || !g_snes_ctx || !g_pfnWaveOutWrite) return;
    WAVEHDR* hdr = &g_wave_hdrs[g_wave_next];
    if (!(hdr->dwFlags & WHDR_DONE)) {
        int16_t discard[AUDIO_BLOCK_SAMPLES];
        snes_runtime_get_audio_samples(g_snes_ctx, discard, AUDIO_BLOCK_SAMPLES);
        return;
    }

    int16_t* dst = g_wave_bufs[g_wave_next];
    size_t got = snes_runtime_get_audio_samples(g_snes_ctx, dst, AUDIO_BLOCK_SAMPLES);
    if (got < AUDIO_BLOCK_SAMPLES) {
        int16_t fill = (got > 0) ? dst[got - 1] : 0;
        for (size_t i = got; i < AUDIO_BLOCK_SAMPLES; i++) dst[i] = fill;
    }
    if (g_muted || g_paused) {
        memset(dst, 0, AUDIO_BLOCK_SAMPLES * sizeof(int16_t));
    }

    hdr->dwBufferLength = AUDIO_BLOCK_SAMPLES * sizeof(int16_t);
    g_pfnWaveOutWrite(g_wave_out, hdr, sizeof(WAVEHDR));
    g_wave_next = (g_wave_next + 1) % AUDIO_BLOCK_COUNT;
}

static void close_win32_audio(void) {
    if (!g_wave_out) return;
    if (g_pfnWaveOutReset) g_pfnWaveOutReset(g_wave_out);
    for (int i = 0; i < AUDIO_BLOCK_COUNT; i++) {
        if (g_pfnWaveOutUnprepareHeader) {
            g_pfnWaveOutUnprepareHeader(g_wave_out, &g_wave_hdrs[i], sizeof(WAVEHDR));
        }
    }
    if (g_pfnWaveOutClose) g_pfnWaveOutClose(g_wave_out);
    g_wave_out = NULL;
}

/* Master Bezel Image Loader */
static void load_bezel_asset(const char* dir) {
    if (g_bezel_pixels) return;
    char raw_path[1152];
    snprintf(raw_path, sizeof(raw_path), "%s\\assets\\img\\bezel_master.raw", dir);

    FILE* f = fopen(raw_path, "rb");
    if (f) {
        size_t expected_size = MASTER_FB_W * MASTER_FB_H * sizeof(uint32_t);
        g_bezel_pixels = (uint32_t*)malloc(expected_size);
        if (g_bezel_pixels) {
            size_t read_bytes = fread(g_bezel_pixels, 1, expected_size, f);
            if (read_bytes == expected_size) {
                g_bezel_loaded = true;
            } else {
                free(g_bezel_pixels);
                g_bezel_pixels = NULL;
            }
        }
        fclose(f);
    }
}

static void compute_target_window_size(int* out_w, int* out_h) {
    int base_h = SNES_SCREEN_HEIGHT;
    int base_w = SNES_WIDESCREEN_WIDTH;
    switch (g_aspect_mode) {
        case ASPECT_16_9_NATIVE:
        case ASPECT_16_9_STRETCH:
            base_w = SNES_WIDESCREEN_WIDTH; /* 352 */
            break;
        case ASPECT_16_10_NATIVE:
            base_w = SNES_WIDESCREEN_16_10_WIDTH; /* 320 */
            break;
        case ASPECT_4_3_CRT:
            base_w = 299; /* 224 * 4 / 3 */
            break;
        case ASPECT_8_7_PAR:
        default:
            base_w = SNES_SCREEN_WIDTH; /* 256 */
            break;
    }
    *out_w = base_w * g_window_scale;
    *out_h = base_h * g_window_scale;
}

static void update_window_title(void) {
    if (!g_hwnd || !g_snes_ctx) return;
    char title[256];
    snprintf(
        title, sizeof(title),
        "%s | %s%s%s%s",
        snes_runtime_get_rom_title(g_snes_ctx),
        aspect_mode_label(g_aspect_mode),
        g_bezel_enabled ? " [Bezel]" : " [Clean Fullscreen]",
        g_paused ? " [PAUSED]" : "",
        g_muted ? " [MUTED]" : ""
    );
    SetWindowTextA(g_hwnd, title);
}

static void resize_client_area(void) {
    if (!g_hwnd || g_fullscreen) return;
    int target_w = 0, target_h = 0;
    compute_target_window_size(&target_w, &target_h);
    DWORD style = (DWORD)GetWindowLongPtrA(g_hwnd, GWL_STYLE);
    RECT rc = { 0, 0, target_w, target_h };
    AdjustWindowRect(&rc, style, g_menu_bar != NULL);
    SetWindowPos(
        g_hwnd, NULL, 0, 0,
        rc.right - rc.left, rc.bottom - rc.top,
        SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE
    );
}

static void set_fullscreen_mode(bool enable) {
    if (!g_hwnd) return;
    DWORD style = (DWORD)GetWindowLongPtrA(g_hwnd, GWL_STYLE);
    if (enable) {
        GetWindowPlacement(g_hwnd, &g_wp_prev);
        SetMenu(g_hwnd, NULL);
        SetWindowLongPtrA(g_hwnd, GWL_STYLE, style & ~WS_OVERLAPPEDWINDOW);
        MONITORINFO mi = { sizeof(mi) };
        if (GetMonitorInfoA(MonitorFromWindow(g_hwnd, MONITOR_DEFAULTTOPRIMARY), &mi)) {
            SetWindowPos(
                g_hwnd, HWND_TOP,
                mi.rcMonitor.left, mi.rcMonitor.top,
                mi.rcMonitor.right - mi.rcMonitor.left,
                mi.rcMonitor.bottom - mi.rcMonitor.top,
                SWP_NOOWNERZORDER | SWP_FRAMECHANGED
            );
        }
        g_fullscreen = true;
    } else {
        SetWindowLongPtrA(g_hwnd, GWL_STYLE, style | WS_OVERLAPPEDWINDOW);
        if (g_menu_bar) SetMenu(g_hwnd, g_menu_bar);
        SetWindowPlacement(g_hwnd, &g_wp_prev);
        SetWindowPos(g_hwnd, NULL, 0, 0, 0, 0,
                     SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOOWNERZORDER | SWP_FRAMECHANGED);
        g_fullscreen = false;
        resize_client_area();
    }
    save_launch_settings_json(g_settings_path);
}

static void populate_menu_items(HMENU root_menu) {
    HMENU view_menu = CreatePopupMenu();
    AppendMenuA(view_menu, MF_STRING | (g_aspect_mode == ASPECT_16_9_NATIVE ? MF_CHECKED : 0),
                IDM_AR_16_9_NATIVE, "16:9 Native Widescreen (352x224)\tW");
    AppendMenuA(view_menu, MF_STRING | (g_aspect_mode == ASPECT_16_10_NATIVE ? MF_CHECKED : 0),
                IDM_AR_16_10_NATIVE, "16:10 Native Widescreen (320x224)");
    AppendMenuA(view_menu, MF_STRING | (g_aspect_mode == ASPECT_4_3_CRT ? MF_CHECKED : 0),
                IDM_AR_4_3_CRT, "4:3 Standard CRT TV (256x224)");
    AppendMenuA(view_menu, MF_STRING | (g_aspect_mode == ASPECT_8_7_PAR ? MF_CHECKED : 0),
                IDM_AR_8_7_PAR, "8:7 Pixel-Perfect (256x224)");
    AppendMenuA(view_menu, MF_STRING | (g_aspect_mode == ASPECT_16_9_STRETCH ? MF_CHECKED : 0),
                IDM_AR_16_9_STRETCH, "16:9 Stretched Widescreen");
    AppendMenuA(view_menu, MF_SEPARATOR, 0, NULL);

    AppendMenuA(view_menu, MF_STRING | (g_bezel_enabled ? MF_CHECKED : 0),
                IDM_TOGGLE_BEZEL, "Toggle Bezel Artwork / Clean Fullscreen\tF10");
    AppendMenuA(view_menu, MF_STRING | (g_fullscreen ? MF_CHECKED : 0),
                IDM_TOGGLE_FULLSCREEN, "Fullscreen\tAlt+Enter");
    AppendMenuA(view_menu, MF_SEPARATOR, 0, NULL);

    AppendMenuA(view_menu, MF_STRING | (g_window_scale == 1 ? MF_CHECKED : 0), IDM_SCALE_1X, "Window Scale 1x\t1");
    AppendMenuA(view_menu, MF_STRING | (g_window_scale == 2 ? MF_CHECKED : 0), IDM_SCALE_2X, "Window Scale 2x\t2");
    AppendMenuA(view_menu, MF_STRING | (g_window_scale == 3 ? MF_CHECKED : 0), IDM_SCALE_3X, "Window Scale 3x\t3");
    AppendMenuA(view_menu, MF_STRING | (g_window_scale == 4 ? MF_CHECKED : 0), IDM_SCALE_4X, "Window Scale 4x\t4");
    AppendMenuA(view_menu, MF_STRING | (g_window_scale == 5 ? MF_CHECKED : 0), IDM_SCALE_5X, "Window Scale 5x\t5");
    AppendMenuA(view_menu, MF_SEPARATOR, 0, NULL);

    AppendMenuA(view_menu, MF_STRING | (g_filter_mode == FILTER_NEAREST ? MF_CHECKED : 0),
                IDM_FILTER_NEAREST, "Filter: Nearest Neighbor (Sharp Pixels)");
    AppendMenuA(view_menu, MF_STRING | (g_filter_mode == FILTER_BILINEAR ? MF_CHECKED : 0),
                IDM_FILTER_BILINEAR, "Filter: Bilinear (Smooth)");
    AppendMenuA(view_menu, MF_STRING | (g_filter_mode == FILTER_SCANLINES ? MF_CHECKED : 0),
                IDM_FILTER_SCANLINES, "Filter: CRT Scanlines");

    HMENU sys_menu = CreatePopupMenu();
    AppendMenuA(sys_menu, MF_STRING, IDM_QUICK_SAVE, "Quick Save State (Slot 1)\tF1");
    AppendMenuA(sys_menu, MF_STRING, IDM_QUICK_LOAD, "Quick Load State (Slot 1)\tF2");
    AppendMenuA(sys_menu, MF_STRING, IDM_CYCLE_PRESET, "Cycle Controller Preset\tF9");
    AppendMenuA(sys_menu, MF_SEPARATOR, 0, NULL);
    AppendMenuA(sys_menu, MF_STRING | (g_paused ? MF_CHECKED : 0), IDM_PAUSE_GAME, "Pause\tP");
    AppendMenuA(sys_menu, MF_STRING | (g_muted ? MF_CHECKED : 0), IDM_MUTE_AUDIO, "Mute Audio\tM");
    AppendMenuA(sys_menu, MF_STRING, IDM_RESET_GAME, "Reset Console\tCtrl+R");
    AppendMenuA(sys_menu, MF_SEPARATOR, 0, NULL);
    AppendMenuA(sys_menu, MF_STRING, IDM_EXIT_GAME, "Exit\tEsc");

    AppendMenuA(root_menu, MF_POPUP, (UINT_PTR)view_menu, "&Display && Widescreen");
    AppendMenuA(root_menu, MF_POPUP, (UINT_PTR)sys_menu, "&System");
}

static void rebuild_menu_bar(void) {
    if (!g_hwnd) return;
    if (g_menu_bar) DestroyMenu(g_menu_bar);
    g_menu_bar = CreateMenu();
    populate_menu_items(g_menu_bar);
    if (!g_fullscreen) {
        SetMenu(g_hwnd, g_menu_bar);
        DrawMenuBar(g_hwnd);
    }
}

static void show_context_menu(HWND hwnd) {
    HMENU popup = CreatePopupMenu();
    populate_menu_items(popup);
    POINT pt;
    GetCursorPos(&pt);
    SetForegroundWindow(hwnd);
    TrackPopupMenu(popup, TPM_RIGHTBUTTON, pt.x, pt.y, 0, hwnd, NULL);
    DestroyMenu(popup);
}

static void handle_menu_command(HWND hwnd, WORD cmd) {
    switch (cmd) {
        case IDM_AR_16_9_NATIVE:
            apply_aspect_mode(ASPECT_16_9_NATIVE);
            show_toast("ASPECT: 16:9 Widescreen (Native 352x224)");
            resize_client_area();
            break;
        case IDM_AR_16_10_NATIVE:
            apply_aspect_mode(ASPECT_16_10_NATIVE);
            show_toast("ASPECT: 16:10 Widescreen (Native 320x224)");
            resize_client_area();
            break;
        case IDM_AR_4_3_CRT:
            apply_aspect_mode(ASPECT_4_3_CRT);
            show_toast("ASPECT: 4:3 Standard CRT TV (256x224)");
            resize_client_area();
            break;
        case IDM_AR_8_7_PAR:
            apply_aspect_mode(ASPECT_8_7_PAR);
            show_toast("ASPECT: 8:7 Pixel-Perfect (256x224)");
            resize_client_area();
            break;
        case IDM_AR_16_9_STRETCH:
            apply_aspect_mode(ASPECT_16_9_STRETCH);
            show_toast("ASPECT: 16:9 Stretched Widescreen");
            resize_client_area();
            break;
        case IDM_TOGGLE_BEZEL:
            g_bezel_enabled = !g_bezel_enabled;
            show_toast(g_bezel_enabled ? "BEZEL: Custom Border Active (F10 to toggle)" : "BEZEL: Clean Fullscreen (No Borders)");
            break;
        case IDM_CYCLE_PRESET:
            apply_pad_preset(g_current_preset + 1);
            break;
        case IDM_SCALE_1X:
        case IDM_SCALE_2X:
        case IDM_SCALE_3X:
        case IDM_SCALE_4X:
        case IDM_SCALE_5X: {
            g_window_scale = (cmd - IDM_SCALE_1X) + 1;
            char scale_toast[64];
            snprintf(scale_toast, sizeof(scale_toast), "SCALE: Window Scale %dx", g_window_scale);
            show_toast(scale_toast);
            resize_client_area();
            break;
        }
        case IDM_FILTER_NEAREST:
            g_filter_mode = FILTER_NEAREST;
            show_toast("FILTER: Nearest Neighbor (Sharp Pixels)");
            break;
        case IDM_FILTER_BILINEAR:
            g_filter_mode = FILTER_BILINEAR;
            show_toast("FILTER: Bilinear Interpolation (Smooth)");
            break;
        case IDM_FILTER_SCANLINES:
            g_filter_mode = FILTER_SCANLINES;
            show_toast("FILTER: Retro CRT Scanlines");
            break;
        case IDM_TOGGLE_FULLSCREEN:
            set_fullscreen_mode(!g_fullscreen);
            break;
        case IDM_QUICK_SAVE:
            if (g_snes_ctx) {
                if (snes_runtime_save_state(g_snes_ctx, g_state_path)) {
                    show_toast("STATE: Slot 1 Saved (F1)");
                }
            }
            break;
        case IDM_QUICK_LOAD:
            if (g_snes_ctx) {
                if (snes_runtime_load_state(g_snes_ctx, g_state_path)) {
                    show_toast("STATE: Slot 1 Loaded (F2)");
                } else {
                    show_toast("STATE: Slot 1 Not Found (F1 to save)");
                }
            }
            break;
        case IDM_RESET_GAME:
            if (g_snes_ctx) {
                snes_runtime_reset(g_snes_ctx);
                show_toast("SYSTEM: Console Reset");
            }
            break;
        case IDM_PAUSE_GAME:
            g_paused = !g_paused;
            show_toast(g_paused ? "GAME: Paused (P to resume)" : "GAME: Resumed");
            break;
        case IDM_MUTE_AUDIO:
            g_muted = !g_muted;
            show_toast(g_muted ? "AUDIO: Muted (M to unmute)" : "AUDIO: Unmuted");
            break;
        case IDM_EXIT_GAME:
            PostMessageA(hwnd, WM_CLOSE, 0, 0);
            return;
        default:
            return;
    }
    rebuild_menu_bar();
    update_window_title();
    save_launch_settings_json(g_settings_path);
    InvalidateRect(hwnd, NULL, FALSE);
}

static void poll_win32_inputs(snes_context_t* ctx) {
    uint16_t joy1 = 0;

    /* Keyboard Polling */
    if (GetAsyncKeyState(VK_UP) & 0x8000)     joy1 |= SNES_BTN_UP;
    if (GetAsyncKeyState(VK_DOWN) & 0x8000)   joy1 |= SNES_BTN_DOWN;
    if (GetAsyncKeyState(VK_LEFT) & 0x8000)   joy1 |= SNES_BTN_LEFT;
    if (GetAsyncKeyState(VK_RIGHT) & 0x8000)  joy1 |= SNES_BTN_RIGHT;
    if (GetAsyncKeyState(VK_RETURN) & 0x8000) joy1 |= SNES_BTN_START;
    if ((GetAsyncKeyState(VK_RSHIFT) & 0x8000) ||
        (GetAsyncKeyState(VK_LSHIFT) & 0x8000) ||
        (GetAsyncKeyState(VK_BACK) & 0x8000)) joy1 |= SNES_BTN_SELECT;

    if (GetAsyncKeyState('X') & 0x8000) joy1 |= SNES_BTN_A;
    if (GetAsyncKeyState('Z') & 0x8000) joy1 |= SNES_BTN_B;
    if (GetAsyncKeyState('S') & 0x8000) joy1 |= SNES_BTN_X;
    if (GetAsyncKeyState('A') & 0x8000) joy1 |= SNES_BTN_Y;
    if ((GetAsyncKeyState('C') & 0x8000) || (GetAsyncKeyState('Q') & 0x8000)) joy1 |= SNES_BTN_L;
    if (GetAsyncKeyState('V') & 0x8000) joy1 |= SNES_BTN_R;

    g_fast_forward = (GetAsyncKeyState(VK_TAB) & 0x8000) != 0;

    /* XInput Controller Polling */
    if (g_gamepad_enabled) {
        init_dynamic_xinput();
        if (g_pXInputGetState) {
            DYN_XINPUT_STATE state;
            memset(&state, 0, sizeof(state));
            if (g_pXInputGetState(0, &state) == ERROR_SUCCESS) {
                WORD btns = state.Gamepad.wButtons;

                /* Directional Movement (D-pad & Left Analog Stick) */
                if (btns & XINPUT_BTN_DPAD_UP)        joy1 |= SNES_BTN_UP;
                if (btns & XINPUT_BTN_DPAD_DOWN)      joy1 |= SNES_BTN_DOWN;
                if (btns & XINPUT_BTN_DPAD_LEFT)      joy1 |= SNES_BTN_LEFT;
                if (btns & XINPUT_BTN_DPAD_RIGHT)     joy1 |= SNES_BTN_RIGHT;

                if (state.Gamepad.sThumbLX < -g_deadzone) joy1 |= SNES_BTN_LEFT;
                if (state.Gamepad.sThumbLX >  g_deadzone) joy1 |= SNES_BTN_RIGHT;
                if (state.Gamepad.sThumbLY >  g_deadzone) joy1 |= SNES_BTN_UP;
                if (state.Gamepad.sThumbLY < -g_deadzone) joy1 |= SNES_BTN_DOWN;

                /* Start & Select */
                if (btns & XINPUT_BTN_START) joy1 |= SNES_BTN_START;
                if (btns & XINPUT_BTN_BACK)  joy1 |= SNES_BTN_SELECT;

                /* Shoulder Buttons */
                if (btns & XINPUT_BTN_LEFT_SHOULDER)  joy1 |= SNES_BTN_L;
                if (btns & XINPUT_BTN_RIGHT_SHOULDER) joy1 |= SNES_BTN_R;

                /* Triggers: LT & RT */
                if (state.Gamepad.bLeftTrigger > 30)  joy1 |= SNES_BTN_L;
                if (state.Gamepad.bRightTrigger > 30) joy1 |= SNES_BTN_R;

                /* Face Button Presets */
                switch (g_current_preset) {
                    case PAD_PRESET_XBOX_STANDARD:
                        /* Xbox A = Jump (B), Xbox B = Attack/Inhale (A), Xbox X = Dash/Action (Y), Xbox Y = Ability Discard (X) */
                        if (btns & XINPUT_BTN_A) joy1 |= SNES_BTN_B;
                        if (btns & XINPUT_BTN_B) joy1 |= SNES_BTN_A;
                        if (btns & XINPUT_BTN_X) joy1 |= SNES_BTN_Y;
                        if (btns & XINPUT_BTN_Y) joy1 |= SNES_BTN_X;
                        break;
                    case PAD_PRESET_NINTENDO_SWITCH:
                        /* Nintendo Switch Layout: A=A, B=B, X=X, Y=Y */
                        if (btns & XINPUT_BTN_B) joy1 |= SNES_BTN_B;
                        if (btns & XINPUT_BTN_A) joy1 |= SNES_BTN_A;
                        if (btns & XINPUT_BTN_Y) joy1 |= SNES_BTN_X;
                        if (btns & XINPUT_BTN_X) joy1 |= SNES_BTN_Y;
                        break;
                    case PAD_PRESET_TRIGGER_ACTION:
                        /* Triggers handle primary actions */
                        if (state.Gamepad.bRightTrigger > 30) joy1 |= SNES_BTN_A;
                        if (state.Gamepad.bLeftTrigger > 30)  joy1 |= SNES_BTN_B;
                        if (btns & XINPUT_BTN_A) joy1 |= SNES_BTN_B;
                        if (btns & XINPUT_BTN_B) joy1 |= SNES_BTN_A;
                        if (btns & XINPUT_BTN_X) joy1 |= SNES_BTN_Y;
                        if (btns & XINPUT_BTN_Y) joy1 |= SNES_BTN_X;
                        break;
                    case PAD_PRESET_CLASSIC_1TO1:
                    default:
                        if (btns & XINPUT_BTN_B) joy1 |= SNES_BTN_A;
                        if (btns & XINPUT_BTN_A) joy1 |= SNES_BTN_B;
                        if (btns & XINPUT_BTN_Y) joy1 |= SNES_BTN_X;
                        if (btns & XINPUT_BTN_X) joy1 |= SNES_BTN_Y;
                        break;
                }

                /* Preset switch hotkey: Back + Start held together */
                static bool s_back_start_held = false;
                if ((btns & XINPUT_BTN_BACK) && (btns & XINPUT_BTN_START)) {
                    if (!s_back_start_held) {
                        s_back_start_held = true;
                        apply_pad_preset(g_current_preset + 1);
                    }
                } else {
                    s_back_start_held = false;
                }
            }
        }
    }

    ctx->joy1_word = joy1;
}

/* Render HUD Toast Badge Overlay directly onto 32-bit framebuffer */
static void render_toast_hud(uint32_t* fb, int fb_w, int fb_h) {
    if (GetTickCount() >= g_toast_expiry || !g_toast_msg[0]) return;

    size_t len = strlen(g_toast_msg);
    if (len == 0) return;

    int char_w = 16, char_h = 16;
    int pad_x = 24, pad_y = 12;
    int text_w = (int)len * char_w;
    int toast_w = text_w + pad_x * 2;
    if (toast_w > fb_w - 40) toast_w = fb_w - 40;
    int toast_h = char_h + pad_y * 2;

    int toast_x = (fb_w - toast_w) / 2;
    int toast_y = fb_h - toast_h - 32;

    /* Alpha blend dark badge background RGBA(16, 20, 32, 220) */
    uint32_t bg_color = 0x00101420;
    uint32_t border_color = 0x00F0C83C; /* Golden border */
    uint32_t text_color = 0x00FFFFFF;   /* Clean white */

    for (int y = toast_y; y < toast_y + toast_h; y++) {
        if (y < 0 || y >= fb_h) continue;
        uint32_t* row = &fb[y * fb_w];
        for (int x = toast_x; x < toast_x + toast_w; x++) {
            if (x < 0 || x >= fb_w) continue;

            /* Golden border outline (2px) */
            if (x < toast_x + 2 || x >= toast_x + toast_w - 2 ||
                y < toast_y + 2 || y >= toast_y + toast_h - 2) {
                row[x] = border_color;
            } else {
                /* Alpha blend (alpha ~ 86%) */
                uint32_t cur = row[x];
                uint32_t cr = (((cur >> 16) & 0xFF) * 36 + ((bg_color >> 16) & 0xFF) * 220) >> 8;
                uint32_t cg = (((cur >> 8) & 0xFF) * 36 + ((bg_color >> 8) & 0xFF) * 220) >> 8;
                uint32_t cb = ((cur & 0xFF) * 36 + (bg_color & 0xFF) * 220) >> 8;
                row[x] = (cr << 16) | (cg << 8) | cb;
            }
        }
    }

    /* Render text characters using 8x8 font scaled 2x */
    int text_x = toast_x + (toast_w - text_w) / 2;
    int text_y = toast_y + pad_y;

    for (size_t c = 0; c < len; c++) {
        char ch = g_toast_msg[c];
        if (ch < 32 || ch > 126) ch = ' ';
        int glyph_idx = ch - 32;

        int gx_pos = text_x + (int)c * char_w;
        for (int gy = 0; gy < 8; gy++) {
            uint8_t bits = g_font8x8[glyph_idx][gy];
            for (int gx = 0; gx < 8; gx++) {
                if (bits & (1 << (7 - gx))) {
                    for (int dy = 0; dy < 2; dy++) {
                        int py = text_y + gy * 2 + dy;
                        if (py < 0 || py >= fb_h) continue;
                        uint32_t* row = &fb[py * fb_w];
                        for (int dx = 0; dx < 2; dx++) {
                            int px = gx_pos + gx * 2 + dx;
                            if (px >= 0 && px < fb_w) {
                                row[px] = text_color;
                            }
                        }
                    }
                }
            }
        }
    }
}

/* Master Composite and Presentation Renderer */
static void render_frame_to_hdc(HDC hdc, int client_w, int client_h) {
    if (!g_snes_ctx || client_w <= 0 || client_h <= 0 || !g_master_fb) return;

    const uint32_t* src_fb = snes_runtime_get_framebuffer(g_snes_ctx);
    int fb_w = snes_runtime_get_frame_width(g_snes_ctx);
    int fb_h = snes_runtime_get_frame_height(g_snes_ctx);
    if (!src_fb || fb_w <= 0 || fb_h <= 0) return;

    if (g_fullscreen && g_bezel_enabled && g_bezel_loaded && g_bezel_pixels) {
        /* Composite Bezel + Game Frame inside 1920x1080 master buffer */
        memcpy(g_master_fb, g_bezel_pixels, MASTER_FB_W * MASTER_FB_H * sizeof(uint32_t));

        /* Inner Cutout in 1920x1080: [left=224, top=208, width=1472, height=806] */
        int cut_x = 224, cut_y = 208, cut_w = 1472, cut_h = 806;
        int target_w = cut_w;
        int target_h = cut_h;

        switch (g_aspect_mode) {
            case ASPECT_4_3_CRT:
                target_w = (cut_h * 4) / 3; /* 1074 */
                break;
            case ASPECT_8_7_PAR:
                target_w = (cut_h * 256) / 224; /* 921 */
                break;
            case ASPECT_16_10_NATIVE:
                target_w = (cut_h * 320) / 224; /* 1151 */
                break;
            case ASPECT_16_9_NATIVE:
                target_w = (cut_h * 352) / 224; /* 1266 */
                break;
            case ASPECT_16_9_STRETCH:
            default:
                target_w = cut_w; /* 1472 */
                break;
        }

        if (target_w > cut_w) target_w = cut_w;
        int dst_x = cut_x + (cut_w - target_w) / 2;
        int dst_y = cut_y;

        /* Scale src_fb directly into g_master_fb */
        for (int y = 0; y < target_h; y++) {
            int sy = (y * fb_h) / target_h;
            const uint32_t* srow = &src_fb[sy * fb_w];
            uint32_t* drow = &g_master_fb[(dst_y + y) * MASTER_FB_W + dst_x];

            if (g_filter_mode == FILTER_SCANLINES && (y & 1)) {
                for (int x = 0; x < target_w; x++) {
                    int sx = (x * fb_w) / target_w;
                    uint32_t c = srow[sx];
                    uint32_t r = (((c >> 16) & 0xFF) * 3) >> 2;
                    uint32_t g = (((c >> 8) & 0xFF) * 3) >> 2;
                    uint32_t b = ((c & 0xFF) * 3) >> 2;
                    drow[x] = (r << 16) | (g << 8) | b;
                }
            } else {
                for (int x = 0; x < target_w; x++) {
                    int sx = (x * fb_w) / target_w;
                    drow[x] = srow[sx];
                }
            }
        }
    } else if (g_fullscreen) {
        /* Clean Fullscreen mode: Pure black background with centered game */
        memset(g_master_fb, 0, MASTER_FB_W * MASTER_FB_H * sizeof(uint32_t));

        int target_h = MASTER_FB_H;
        int target_w = MASTER_FB_W;
        switch (g_aspect_mode) {
            case ASPECT_4_3_CRT:
                target_w = (MASTER_FB_H * 4) / 3; /* 1440 */
                break;
            case ASPECT_8_7_PAR:
                target_w = (MASTER_FB_H * 256) / 224; /* 1234 */
                break;
            case ASPECT_16_10_NATIVE:
                target_w = (MASTER_FB_H * 320) / 224; /* 1543 */
                break;
            case ASPECT_16_9_NATIVE:
                target_w = (MASTER_FB_H * 352) / 224; /* 1700 */
                break;
            case ASPECT_16_9_STRETCH:
            default:
                target_w = MASTER_FB_W; /* 1920 */
                break;
        }

        if (target_w > MASTER_FB_W) target_w = MASTER_FB_W;
        int dst_x = (MASTER_FB_W - target_w) / 2;
        int dst_y = 0;

        for (int y = 0; y < target_h; y++) {
            int sy = (y * fb_h) / target_h;
            const uint32_t* srow = &src_fb[sy * fb_w];
            uint32_t* drow = &g_master_fb[(dst_y + y) * MASTER_FB_W + dst_x];

            if (g_filter_mode == FILTER_SCANLINES && (y & 1)) {
                for (int x = 0; x < target_w; x++) {
                    int sx = (x * fb_w) / target_w;
                    uint32_t c = srow[sx];
                    uint32_t r = (((c >> 16) & 0xFF) * 3) >> 2;
                    uint32_t g = (((c >> 8) & 0xFF) * 3) >> 2;
                    uint32_t b = ((c & 0xFF) * 3) >> 2;
                    drow[x] = (r << 16) | (g << 8) | b;
                }
            } else {
                for (int x = 0; x < target_w; x++) {
                    int sx = (x * fb_w) / target_w;
                    drow[x] = srow[sx];
                }
            }
        }
    } else {
        /* Windowed mode: Render directly to client area preserving aspect ratio */
        memset(g_master_fb, 0, MASTER_FB_W * MASTER_FB_H * sizeof(uint32_t));
        int target_h = MASTER_FB_H;
        int target_w = MASTER_FB_W;
        switch (g_aspect_mode) {
            case ASPECT_4_3_CRT:      target_w = (MASTER_FB_H * 4) / 3; break;
            case ASPECT_8_7_PAR:      target_w = (MASTER_FB_H * 256) / 224; break;
            case ASPECT_16_10_NATIVE: target_w = (MASTER_FB_H * 320) / 224; break;
            case ASPECT_16_9_NATIVE:  target_w = (MASTER_FB_H * 352) / 224; break;
            default:                  target_w = MASTER_FB_W; break;
        }
        int dst_x = (MASTER_FB_W - target_w) / 2;
        int dst_y = 0;

        for (int y = 0; y < target_h; y++) {
            int sy = (y * fb_h) / target_h;
            const uint32_t* srow = &src_fb[sy * fb_w];
            uint32_t* drow = &g_master_fb[(dst_y + y) * MASTER_FB_W + dst_x];
            for (int x = 0; x < target_w; x++) {
                int sx = (x * fb_w) / target_w;
                drow[x] = srow[sx];
            }
        }
    }

    /* Render Toast HUD overlay onto master buffer */
    render_toast_hud(g_master_fb, MASTER_FB_W, MASTER_FB_H);

    /* Atomic Blit to Screen */
    BITMAPINFO bmi;
    memset(&bmi, 0, sizeof(bmi));
    bmi.bmiHeader.biSize        = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth       = MASTER_FB_W;
    bmi.bmiHeader.biHeight      = -MASTER_FB_H; /* Top-down DIB */
    bmi.bmiHeader.biPlanes      = 1;
    bmi.bmiHeader.biBitCount    = 32;
    bmi.bmiHeader.biCompression = BI_RGB;

    SetStretchBltMode(hdc, g_filter_mode == FILTER_BILINEAR ? HALFTONE : COLORONCOLOR);
    StretchDIBits(
        hdc,
        0, 0, client_w, client_h,
        0, 0, MASTER_FB_W, MASTER_FB_H,
        g_master_fb, &bmi, DIB_RGB_COLORS, SRCCOPY
    );
}

static LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    switch (uMsg) {
        case WM_COMMAND:
            handle_menu_command(hwnd, LOWORD(wParam));
            return 0;

        case WM_CONTEXTMENU:
            show_context_menu(hwnd);
            return 0;

        case WM_SYSKEYDOWN:
            if (wParam == VK_RETURN && (lParam & (1 << 29))) {
                set_fullscreen_mode(!g_fullscreen);
                return 0;
            }
            if (wParam == VK_F10) {
                handle_menu_command(hwnd, IDM_TOGGLE_BEZEL);
                return 0;
            }
            break;

        case WM_KEYDOWN:
            if (!(lParam & 0x40000000)) { /* Ignore key auto-repeat for toggles */
                switch (wParam) {
                    /* Hotkey F1: Quick Save State (Slot 1) */
                    case VK_F1:
                        handle_menu_command(hwnd, IDM_QUICK_SAVE);
                        return 0;

                    /* Hotkey F2: Quick Load State (Slot 1) */
                    case VK_F2:
                        handle_menu_command(hwnd, IDM_QUICK_LOAD);
                        return 0;

                    /* Hotkey F9: Cycle Controller Preset */
                    case VK_F9:
                        handle_menu_command(hwnd, IDM_CYCLE_PRESET);
                        return 0;

                    /* Hotkey F10: Toggle Bezel Artwork on / off */
                    case VK_F10:
                        handle_menu_command(hwnd, IDM_TOGGLE_BEZEL);
                        return 0;

                    /* Hotkey F11: In-game settings / overlay menu */
                    case VK_F11:
                        show_context_menu(hwnd);
                        return 0;

                    /* Hotkey Esc: Instant clean exit */
                    case VK_ESCAPE:
                        PostMessageA(hwnd, WM_CLOSE, 0, 0);
                        return 0;

                    /* Hotkey W: Cycle Aspect Ratio */
                    case 'W': {
                        AspectMode next = (AspectMode)((g_aspect_mode + 1) % 5);
                        handle_menu_command(hwnd, (WORD)(IDM_AR_16_9_NATIVE + next));
                        return 0;
                    }

                    /* Window scale hotkeys 1-5 */
                    case '1':
                    case '2':
                    case '3':
                    case '4':
                    case '5':
                        handle_menu_command(hwnd, (WORD)(IDM_SCALE_1X + (wParam - '1')));
                        return 0;

                    /* Hotkey P: Pause / Resume */
                    case 'P':
                    case VK_PAUSE:
                        handle_menu_command(hwnd, IDM_PAUSE_GAME);
                        return 0;

                    /* Hotkey M: Mute / Unmute Audio */
                    case 'M':
                        handle_menu_command(hwnd, IDM_MUTE_AUDIO);
                        return 0;
                }
            }
            break;

        case WM_PAINT: {
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hwnd, &ps);
            RECT rc;
            GetClientRect(hwnd, &rc);
            render_frame_to_hdc(hdc, rc.right - rc.left, rc.bottom - rc.top);
            EndPaint(hwnd, &ps);
            return 0;
        }

        case WM_CLOSE:
            if (g_snes_ctx) {
                snes_runtime_save_sram(g_snes_ctx, g_sram_path);
            }
            DestroyWindow(hwnd);
            return 0;

        case WM_DESTROY:
            if (g_snes_ctx) {
                g_snes_ctx->is_running = false;
            }
            PostQuitMessage(0);
            return 0;
    }
    return DefWindowProcA(hwnd, uMsg, wParam, lParam);
}

static void init_paths_from_module(void) {
    char mod_path[1024];
    DWORD len = GetModuleFileNameA(NULL, mod_path, sizeof(mod_path));
    if (len > 0 && len < sizeof(mod_path)) {
        char* last_sep = strrchr(mod_path, '\\');
        if (!last_sep) last_sep = strrchr(mod_path, '/');
        if (last_sep) {
            *last_sep = '\0';
            snprintf(g_exe_dir, sizeof(g_exe_dir), "%s", mod_path);

            /* Create isolated saves/ directory if it doesn't exist */
            char saves_dir[1024];
            snprintf(saves_dir, sizeof(saves_dir), "%s\\saves", g_exe_dir);
            CreateDirectoryA(saves_dir, NULL);

            snprintf(g_sram_path, sizeof(g_sram_path), "%s\\saves\\kirby_super_star.srm", g_exe_dir);
            snprintf(g_state_path, sizeof(g_state_path), "%s\\saves\\state1.state", g_exe_dir);
            snprintf(g_settings_path, sizeof(g_settings_path), "%s\\config.ini", g_exe_dir);
            snprintf(g_settings_hyphen_path, sizeof(g_settings_hyphen_path), "%s\\launch_settings.json", g_exe_dir);

            /* Migrate legacy save files from root if present */
            char old_sram[1024], old_state[1024];
            snprintf(old_sram, sizeof(old_sram), "%s\\game.srm", g_exe_dir);
            snprintf(old_state, sizeof(old_state), "%s\\quicksave.state", g_exe_dir);
            if (GetFileAttributesA(old_sram) != INVALID_FILE_ATTRIBUTES && GetFileAttributesA(g_sram_path) == INVALID_FILE_ATTRIBUTES) {
                MoveFileA(old_sram, g_sram_path);
            }
            if (GetFileAttributesA(old_state) != INVALID_FILE_ATTRIBUTES && GetFileAttributesA(g_state_path) == INVALID_FILE_ATTRIBUTES) {
                MoveFileA(old_state, g_state_path);
            }
        }
    }
}

static void try_load_fallback_rom_in_dir(snes_context_t* ctx, const char* dir) {
    if (!ctx || snes_runtime_is_rom_loaded(ctx)) return;
    const char* names[] = { "game.sfc", "game.smc", "Kirby Super Star (USA).sfc" };
    char candidate[1152];
    for (int i = 0; i < 3 && !snes_runtime_is_rom_loaded(ctx); i++) {
        snprintf(candidate, sizeof(candidate), "%s\\%s", dir, names[i]);
        snes_runtime_load_rom(ctx, candidate);
    }
    if (!snes_runtime_is_rom_loaded(ctx)) {
        const char* patterns[] = { "*.sfc", "*.smc" };
        for (int p = 0; p < 2 && !snes_runtime_is_rom_loaded(ctx); p++) {
            char search_pat[1152];
            snprintf(search_pat, sizeof(search_pat), "%s\\%s", dir, patterns[p]);
            WIN32_FIND_DATAA fd;
            HANDLE hFind = FindFirstFileA(search_pat, &fd);
            if (hFind != INVALID_HANDLE_VALUE) {
                snprintf(candidate, sizeof(candidate), "%s\\%s", dir, fd.cFileName);
                snes_runtime_load_rom(ctx, candidate);
                FindClose(hFind);
            }
        }
    }
}

static int run_win32_app(HINSTANCE hInstance, int nCmdShow) {
    init_paths_from_module();
    load_launch_settings_json(g_settings_hyphen_path);
    load_launch_settings_json(g_settings_path);

    /* Allocate master composite framebuffer */
    g_master_fb = (uint32_t*)calloc(MASTER_FB_W * MASTER_FB_H, sizeof(uint32_t));
    if (!g_master_fb) return 1;

    /* Load master bezel image */
    load_bezel_asset(g_exe_dir);

    g_snes_ctx = snes_runtime_init();
    if (!g_snes_ctx) return 1;

    /* Check command-line arguments */
    for (int i = 1; i < __argc; i++) {
        if (strcmp(__argv[i], "--windowed") == 0) {
            g_fullscreen = false;
        } else if (strcmp(__argv[i], "--bezel") == 0) {
            g_bezel_enabled = true;
        } else if (strcmp(__argv[i], "--no-bezel") == 0) {
            g_bezel_enabled = false;
        } else if (strcmp(__argv[i], "--rom") == 0 && i + 1 < __argc) {
            snes_runtime_load_rom(g_snes_ctx, __argv[++i]);
        } else if (__argv[i][0] != '-') {
            snes_runtime_load_rom(g_snes_ctx, __argv[i]);
        }
    }

    apply_aspect_mode(g_aspect_mode);
    if (!snes_runtime_is_rom_loaded(g_snes_ctx)) {
        snes_recompiled_entry(g_snes_ctx);
    }
    if (!snes_runtime_is_rom_loaded(g_snes_ctx)) {
        try_load_fallback_rom_in_dir(g_snes_ctx, g_exe_dir);
    }
    snes_runtime_load_sram(g_snes_ctx, g_sram_path);

    const char* CLASS_NAME = "SNESRecompWidescreenWindowClass";
    WNDCLASSEXA wc;
    memset(&wc, 0, sizeof(wc));
    wc.cbSize        = sizeof(WNDCLASSEXA);
    wc.style         = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
    wc.lpfnWndProc   = WindowProc;
    wc.hInstance     = hInstance;
    wc.hCursor       = LoadCursor(NULL, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
    wc.lpszClassName = CLASS_NAME;
    RegisterClassExA(&wc);

    g_menu_bar = CreateMenu();
    populate_menu_items(g_menu_bar);

    int init_w = 0, init_h = 0;
    compute_target_window_size(&init_w, &init_h);
    RECT rect = { 0, 0, init_w, init_h };
    AdjustWindowRect(&rect, WS_OVERLAPPEDWINDOW, TRUE);

    g_hwnd = CreateWindowExA(
        0, CLASS_NAME,
        "Kirby Super Star Recompiled",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT,
        rect.right - rect.left, rect.bottom - rect.top,
        NULL, g_menu_bar, hInstance, NULL
    );
    if (!g_hwnd) {
        snes_runtime_free(g_snes_ctx);
        return 1;
    }

    /* Set icon dynamically if available */
    char ico_path[1152];
    snprintf(ico_path, sizeof(ico_path), "%s\\Kirby Super Star.ico", g_exe_dir);
    HICON hIco = (HICON)LoadImageA(NULL, ico_path, IMAGE_ICON, 0, 0, LR_LOADFROMFILE | LR_DEFAULTSIZE);
    if (hIco) {
        SendMessageA(g_hwnd, WM_SETICON, ICON_BIG, (LPARAM)hIco);
        SendMessageA(g_hwnd, WM_SETICON, ICON_SMALL, (LPARAM)hIco);
    }

    update_window_title();
    ShowWindow(g_hwnd, nCmdShow);

    if (g_fullscreen) {
        g_fullscreen = false;
        set_fullscreen_mode(true);
    }

    init_win32_audio();
    if (g_pfnTimeBeginPeriod) g_pfnTimeBeginPeriod(1);

    LARGE_INTEGER freq, last_counter, now_counter;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&last_counter);
    const double target_frame_sec = 1.0 / 60.0988;

    MSG msg;
    memset(&msg, 0, sizeof(msg));
    while (g_snes_ctx && g_snes_ctx->is_running) {
        while (PeekMessageA(&msg, NULL, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                g_snes_ctx->is_running = false;
                break;
            }
            TranslateMessage(&msg);
            DispatchMessageA(&msg);
        }
        if (!g_snes_ctx->is_running) break;

        poll_win32_inputs(g_snes_ctx);

        if (!g_paused) {
            int steps = g_fast_forward ? 2 : 1;
            for (int s = 0; s < steps; s++) {
                snes_runtime_run_frame(g_snes_ctx);
                pump_win32_audio();
            }
            /* Periodic SRAM flush every ~5 seconds */
            if ((g_snes_ctx->frame_count % 300) == 0) {
                snes_runtime_save_sram(g_snes_ctx, g_sram_path);
            }
        }

        HDC hdc = GetDC(g_hwnd);
        if (hdc) {
            RECT rc;
            GetClientRect(g_hwnd, &rc);
            render_frame_to_hdc(hdc, rc.right - rc.left, rc.bottom - rc.top);
            ReleaseDC(g_hwnd, hdc);
        }

        /* 60 FPS Frame Pacing */
        if (!g_fast_forward) {
            QueryPerformanceCounter(&now_counter);
            double elapsed = (double)(now_counter.QuadPart - last_counter.QuadPart) / (double)freq.QuadPart;
            if (elapsed < target_frame_sec) {
                DWORD sleep_ms = (DWORD)((target_frame_sec - elapsed) * 1000.0);
                if (sleep_ms > 1) Sleep(sleep_ms - 1);
                do {
                    QueryPerformanceCounter(&now_counter);
                    elapsed = (double)(now_counter.QuadPart - last_counter.QuadPart) / (double)freq.QuadPart;
                } while (elapsed < target_frame_sec);
            }
            last_counter = now_counter;
        } else {
            QueryPerformanceCounter(&last_counter);
        }
    }

    if (g_pfnTimeEndPeriod) g_pfnTimeEndPeriod(1);
    close_win32_audio();
    snes_runtime_save_sram(g_snes_ctx, g_sram_path);
    snes_runtime_free(g_snes_ctx);
    g_snes_ctx = NULL;
    if (g_master_fb) free(g_master_fb);
    if (g_bezel_pixels) free(g_bezel_pixels);
    return 0;
}

static bool has_headless_flag(int argc, char** argv) {
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--headless-frames") == 0 ||
            strcmp(argv[i], "--dump-ppm") == 0 ||
            strcmp(argv[i], "--frames") == 0) {
            return true;
        }
    }
    return false;
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {
    (void)hPrevInstance;
    (void)lpCmdLine;
    if (has_headless_flag(__argc, __argv)) {
        return run_headless_cli(__argc, __argv);
    }
    return run_win32_app(hInstance, nCmdShow);
}

int main(int argc, char** argv) {
    if (has_headless_flag(argc, argv)) {
        return run_headless_cli(argc, argv);
    }
    return run_win32_app(GetModuleHandleA(NULL), SW_SHOWDEFAULT);
}

#else /* Non-Windows */

int main(int argc, char** argv) {
    if (argc > 1) {
        return run_headless_cli(argc, argv);
    }
    char* default_argv[] = { argv[0], "--headless-frames", "60", NULL };
    return run_headless_cli(3, default_argv);
}

#endif
