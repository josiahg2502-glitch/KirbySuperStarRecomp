# Kirby Super Star - Standalone PC Port (SNESRecomp)

A high-performance, standalone native Windows x64 recompilation port of **Kirby Super Star** (SNES / Super Famicom). Featuring custom 1080p Borderless Fullscreen Bezel artwork, seamless bezel toggle, authentic 4:3 and native 16:9 widescreen rendering, instant save states, isolated saves directory, multi-profile XInput controller support with trigger handling, and on-screen HUD notifications.

---

## Key Features

- **Custom 1080p Bezel Artwork**: High-definition Kirby Super Star custom bezel frame rendered in true 1080p borderless fullscreen (`assets/img/bezel_master.raw`).
- **Seamless Bezel Toggle (`F10`)**: Instantly toggle between the custom Bezel Artwork and Clean Fullscreen (authentic black pillarbox) on the fly.
- **On-Screen HUD Toasts**: Instant visual confirmation for save states, bezel toggles, controller presets, and settings with an authentic gold-bordered HUD toast pill.
- **Save State System (`F1` / `F2`)**:
  - `F1`: Quick save game state (saved to `saves/state1.state`).
  - `F2`: Quick load game state.
- **Isolated Saves Directory**:
  - All battery saves (`kirby_super_star.srm`) and save states (`state1.state`) are stored in `saves/`.
  - Automatically migrates existing root-level `.srm` and `.state` files.
- **Controller & Keyboard Support**:
  - Full XInput controller support with trigger handling (LT / RT).
  - Controller presets (`F9` or `Back + Start` to cycle):
    1. **Xbox Standard**: `A`=B, `X`=Y, `B`=A, `Y`=X, `LB`/`LT`=L, `RB`/`RT`=R
    2. **Nintendo Switch / SNES Authentic**: `B`=B, `Y`=Y, `A`=A, `X`=X, `LB`/`LT`=L, `RB`/`RT`=R
    3. **Trigger Action**: `LT`=B (Jump/Inhale), `RT`=Y (Attack)
    4. **Classic Arcade**: `X`=Y (Attack), `A`=B (Jump)
  - Interactive remapping via `Remap Controller.bat`.
- **Widescreen & Display Enhancements**:
  - Native 16:9 widescreen PPU rendering (`352x224`), 16:10 (`320x224`), 4:3 CRT, and 8:7 PAR.
  - Cycle aspect ratio anytime with `W`.
  - Window scaling from 1x to 5x with number keys `1` - `5`.
  - In-game settings context menu on `F11` or Right-Click.
- **Instant Clean Exit (`Esc`)**: Closes the game cleanly without hanging or leaving background processes.
- **Zero Runtime Dependencies**: Compiled to a standalone native 64-bit binary with embedded high-resolution application icon.

---

## Quick Start

1. Double-click **`Play Kirby Super Star.cmd`** (or `Kirby Super Star.exe`).
2. The game will launch in borderless fullscreen with the custom 1080p bezel.
3. Plug in any Xbox or XInput controller, or use keyboard controls.

---

## Default Controls

### Gamepad (Xbox / XInput)
| Button | Default (Xbox Standard) | Switch / SNES Layout | Trigger Action |
|---|---|---|---|
| **D-Pad / Left Stick** | D-Pad Movement | D-Pad Movement | D-Pad Movement |
| **A** | B (Jump / Inhale) | A | Jump / Inhale |
| **B** | A | B (Jump / Inhale) | A |
| **X** | Y (Attack / Action) | X | Attack / Action |
| **Y** | X | Y (Attack / Action) | X |
| **LB / LT** | L Shoulder (Guard) | L Shoulder | L Shoulder |
| **RB / RT** | R Shoulder (Guard) | R Shoulder | R Shoulder |
| **Start** | Start (Pause / Select Game) | Start | Start |
| **Back (View)** | Select | Select | Select |
| **Back + Start** | Cycle Controller Preset | Cycle Preset | Cycle Preset |

### Keyboard
| Key | Action |
|---|---|
| **Arrow Keys** | D-Pad (Up, Down, Left, Right) |
| **Z** | B Button (Jump / Inhale) |
| **X** | A Button |
| **A** | Y Button (Attack / Action) |
| **S** | X Button |
| **C** | L Shoulder (Guard) |
| **V** | R Shoulder (Guard) |
| **Enter** | Start |
| **Right Shift** | Select |

---

## Hotkeys

| Hotkey | Function |
|---|---|
| **`F1`** | **Quick Save State** (Slot 1 -> `saves/state1.state`) |
| **`F2`** | **Quick Load State** (Slot 1) |
| **`F9`** | **Cycle Controller Preset** (Standard / Switch / Trigger / Classic) |
| **`F10`** | **Toggle Bezel Artwork** (Custom Bezel ↔ Clean Fullscreen) |
| **`F11`** | **Display & Widescreen Context Menu** |
| **`Esc`** | **Instant Clean Exit** |
| **`W`** | Cycle Aspect Ratio (4:3 CRT ↔ 16:9 Widescreen ↔ 8:7 ↔ 16:9 Stretch) |
| **`1` – `5`** | Window Scaling (1x to 5x) |
| **`P`** | Pause / Unpause emulation |
| **`M`** | Mute / Unmute audio |
| **`Tab`** | Fast-Forward (while held) |

---

## Directory Structure

```
Kirby Super Star/
├── Kirby Super Star.exe      # Main standalone game executable
├── Play Kirby Super Star.cmd  # Friendly one-click launcher
├── Remap Controller.bat       # Controller remapping launcher
├── remap_controller.ps1       # Interactive controller config utility
├── START HERE.txt             # Quick start and hotkey guide
├── README.md                  # Detailed documentation
├── Kirby Super Star.ico       # High-resolution multi-size application icon
├── launch_settings.json       # Startup configuration
├── config.ini                 # Runtime settings
├── assets/
│   └── img/
│       ├── bezel_master.png   # 1920x1080 master bezel image
│       ├── bezel_master.raw   # Fast 32-bit pre-baked raw buffer
│       └── boxart_clean.png   # High-resolution box art
└── saves/
    ├── kirby_super_star.srm   # Battery save file
    └── state1.state           # Save state slot 1
```

---

## Building from Source

Requirements:
- Windows 10/11 x64
- MSVC (Visual Studio 2022+ / Build Tools) or Zig CC / GCC / Clang

To build:
```bat
build.bat
```
This compiles `src/game.rc` into a Windows resource file and compiles all C source files with optimizations, generating `Kirby Super Star.exe` and `game.exe`.
