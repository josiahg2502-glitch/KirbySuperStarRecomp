/**
 * SNESRecomp - Native C Hardware Runtime Engine Implementation
 *
 * Combines the recompiled 65C816 helper interface with a complete cycle-accurate
 * SNES hardware core, native 16:9 widescreen PPU renderer (352x224),
 * SPC700 + 8-Voice S-DSP stereo audio engine, and 8-channel DMA/HDMA controller.
 */

#include "snes_runtime.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#include <math.h>

#if defined(__has_include)
#  if __has_include("rom_embedded.h")
#    include "rom_embedded.h"
#    define SNES_HAS_EMBEDDED_ROM 1
#  endif
#endif

typedef uint8_t uint8;
typedef int8_t int8;
typedef uint16_t uint16;
typedef int16_t int16;
typedef uint32_t uint32;
typedef int32_t int32;
typedef uint64_t uint64;
typedef int64_t int64;
typedef unsigned int uint;
#ifndef countof
#define countof(x) (sizeof(x) / sizeof((x)[0]))
#endif
#ifndef arraysize
#define arraysize(x) (sizeof(x) / sizeof((x)[0]))
#endif
#ifndef FORCEINLINE
#define FORCEINLINE inline
#endif
#ifndef NOINLINE
#define NOINLINE
#endif
static inline uint32 UintMin(uint32 a, uint32 b) { return a < b ? a : b; }
static inline int IntMin(int a, int b) { return a < b ? a : b; }
static inline int IntMax(int a, int b) { return a > b ? a : b; }
static inline int IntClamp(int x, int a, int b) { return x < a ? a : (x > b ? b : x); }

typedef struct Snes Snes;
typedef struct Cpu Cpu;
typedef struct Apu Apu;
typedef struct Ppu Ppu;
typedef struct Dma Dma;
typedef struct Cart Cart;
typedef struct Input Input;
typedef struct Spc Spc;
typedef struct Dsp Dsp;

/* === saveload.h === */


typedef void SaveLoadFunc(void *ctx, void *data, size_t data_size);

#define SL(x) func(ctx, &x, sizeof(x))

/* === snes_regs.h === */


#define INIDISP 0x2100
#define OBSEL 0x2101
#define OAMADDL 0x2102
#define OAMADDH 0x2103
#define OAMDATA 0x2104
#define BGMODE 0x2105
#define MOSAIC 0x2106
#define BG1SC 0x2107
#define BG2SC 0x2108
#define BG3SC 0x2109
#define BG4SC 0x210a
#define BG12NBA 0x210b
#define BG34NBA 0x210c
#define BG1HOFS 0x210d
#define BG1VOFS 0x210e
#define BG2HOFS 0x210f
#define BG2VOFS 0x2110
#define BG3HOFS 0x2111
#define BG3VOFS 0x2112
#define BG4HOFS 0x2113
#define BG4VOFS 0x2114
#define VMAIN 0x2115
#define VMADDL 0x2116
#define VMADDH 0x2117
#define VMDATAL 0x2118
#define VMDATAH 0x2119
#define M7SEL 0x211a
#define M7A 0x211b
#define M7B 0x211c
#define M7C 0x211d
#define M7D 0x211e
#define M7X 0x211f
#define M7Y 0x2120
#define CGADD 0x2121
#define CGDATA 0x2122
#define W12SEL 0x2123
#define W34SEL 0x2124
#define WOBJSEL 0x2125
#define WH0 0x2126
#define WH1 0x2127
#define WH2 0x2128
#define WH3 0x2129
#define WBGLOG 0x212a
#define WOBJLOG 0x212b
#define TM 0x212c
#define TS 0x212d
#define TMW 0x212e
#define TSW 0x212f
#define CGWSEL 0x2130
#define CGADSUB 0x2131
#define COLDATA 0x2132
#define SETINI 0x2133
#define MPYL 0x2134
#define MPYM 0x2135
#define MPYH 0x2136
#define SLHV 0x2137
#define RDOAM 0x2138
#define RDVRAML 0x2139
#define RDVRAMH 0x213a
#define RDCGRAM 0x213b
#define OPHCT 0x213c
#define OPVCT 0x213d
#define STAT77 0x213e
#define STAT78 0x213f
#define APUI00 0x2140
#define APUI01 0x2141
#define APUI02 0x2142
#define APUI03 0x2143
#define WMDATA 0x2180
#define WMADDL 0x2181
#define WMADDM 0x2182
#define WMADDH 0x2183
#define JOYA 0x4016
#define JOYB 0x4017
#define NMITIMEN 0x4200
#define WRIO 0x4201
#define WRMPYA 0x4202
#define WRMPYB 0x4203
#define WRDIVL 0x4204
#define WRDIVH 0x4205
#define WRDIVB 0x4206
#define HTIMEL 0x4207
#define HTIMEH 0x4208
#define VTIMEL 0x4209
#define VTIMEH 0x420a
#define MDMAEN 0x420b
#define HDMAEN 0x420c
#define MEMSEL 0x420d
#define RDNMI 0x4210
#define TIMEUP 0x4211
#define HVBJOY 0x4212
#define RDIO 0x4213
#define RDDIVL 0x4214
#define RDDIVH 0x4215
#define RDMPYL 0x4216
#define RDMPYH 0x4217
#define JOY1L 0x4218
#define JOY1H 0x4219
#define JOY2L 0x421a
#define JOY2H 0x421b
#define JOY3L 0x421c
#define JOY3H 0x421d
#define JOY4L 0x421e
#define JOY4H 0x421f

#define DMAP0 0x4300
#define BBAD0 0x4301
#define A1T0L 0x4302
#define A1T0H 0x4303
#define A1B0 0x4304
#define DAS0L 0x4305
#define DAS0H 0x4306
#define DAS00 0x4307
#define A2A0L 0x4308
#define A2A0H 0x4309
#define NTRL0 0x430a
#define UNUSED0 0x430b
#define MIRR0 0x430f
#define DMAP1 0x4310
#define BBAD1 0x4311
#define A1T1L 0x4312
#define A1T1H 0x4313
#define A1B1 0x4314
#define DAS1L 0x4315
#define DAS1H 0x4316
#define DAS10 0x4317
#define A2A1L 0x4318
#define A2A1H 0x4319
#define NTRL1 0x431a
#define UNUSED1 0x431b
#define MIRR1 0x431f
#define DMAP2 0x4320
#define BBAD2 0x4321
#define A1T2L 0x4322
#define A1T2H 0x4323
#define A1B2 0x4324
#define DAS2L 0x4325
#define DAS2H 0x4326
#define DAS20 0x4327
#define A2A2L 0x4328
#define A2A2H 0x4329
#define NTRL2 0x432a
#define UNUSED2 0x432b
#define MIRR2 0x432f
#define DMAP3 0x4330
#define BBAD3 0x4331
#define A1T3L 0x4332
#define A1T3H 0x4333
#define A1B3 0x4334
#define DAS3L 0x4335
#define DAS3H 0x4336
#define DAS30 0x4337
#define A2A3L 0x4338
#define A2A3H 0x4339
#define NTRL3 0x433a
#define UNUSED3 0x433b
#define MIRR3 0x433f
#define DMAP4 0x4340
#define BBAD4 0x4341
#define A1T4L 0x4342
#define A1T4H 0x4343
#define A1B4 0x4344
#define DAS4L 0x4345
#define DAS4H 0x4346
#define DAS40 0x4347
#define A2A4L 0x4348
#define A2A4H 0x4349
#define NTRL4 0x434a
#define UNUSED4 0x434b
#define MIRR4 0x434f
#define DMAP5 0x4350
#define BBAD5 0x4351
#define A1T5L 0x4352
#define A1T5H 0x4353
#define A1B5 0x4354
#define DAS5L 0x4355
#define DAS5H 0x4356
#define DAS50 0x4357
#define A2A5L 0x4358
#define A2A5H 0x4359
#define NTRL5 0x435a
#define UNUSED5 0x435b
#define MIRR5 0x435f
#define DMAP6 0x4360
#define BBAD6 0x4361
#define A1T6L 0x4362
#define A1T6H 0x4363
#define A1B6 0x4364
#define DAS6L 0x4365
#define DAS6H 0x4366
#define DAS60 0x4367
#define A2A6L 0x4368
#define A2A6H 0x4369
#define NTRL6 0x436a
#define UNUSED6 0x436b
#define MIRR6 0x436f
#define DMAP7 0x4370
#define BBAD7 0x4371
#define A1T7L 0x4372
#define A1T7H 0x4373
#define A1B7 0x4374
#define DAS7L 0x4375
#define DAS7H 0x4376
#define DAS70 0x4377
#define A2A7L 0x4378
#define A2A7H 0x4379
#define NTRL7 0x437a
#define UNUSED7 0x437b
#define MIRR7 0x437f


/* === dsp_regs.h === */
#ifndef DSP_REGS_H
#define DSP_REGS_H

enum DspReg {
  V0VOLL = 0x00,
  V0VOLR = 0x01,
  V0PITCHL = 0x02,
  V0PITCHH = 0x03,
  V0SRCN = 0x04,
  V0ADSR1 = 0x05,
  V0ADSR2 = 0x06,
  V0GAIN = 0x07,
  V0ENVX = 0x08,
  V0OUTX = 0x09,
  MVOLL = 0x0C,
  EFB = 0x0D,
  FIR0 = 0x0F,
  V1VOLL = 0x10,
  V1VOLR = 0x11,
  V1PL = 0x12,
  V1PH = 0x13,
  V1SRCN = 0x14,
  V1ADSR1 = 0x15,
  V1ADSR2 = 0x16,
  V1GAIN = 0x17,
  V1ENVX = 0x18,
  V1OUTX = 0x19,
  MVOLR = 0x1C,
  FIR1 = 0x1F,
  V2VOLL = 0x20,
  V2VOLR = 0x21,
  V2PL = 0x22,
  V2PH = 0x23,
  V2SRCN = 0x24,
  V2ADSR1 = 0x25,
  V2ADSR2 = 0x26,
  V2GAIN = 0x27,
  V2ENVX = 0x28,
  V2OUTX = 0x29,
  EVOLL = 0x2C,
  PMON = 0x2D,
  FIR2 = 0x2F,
  V3VOLL = 0x30,
  V3VOLR = 0x31,
  V3PL = 0x32,
  V3PH = 0x33,
  V3SRCN = 0x34,
  V3ADSR1 = 0x35,
  V3ADSR2 = 0x36,
  V3GAIN = 0x37,
  V3ENVX = 0x38,
  V3OUTX = 0x39,
  EVOLR = 0x3C,
  NON = 0x3D,
  FIR3 = 0x3F,
  V4VOLL = 0x40,
  V4VOLR = 0x41,
  V4PL = 0x42,
  V4PH = 0x43,
  V4SRCN = 0x44,
  V4ADSR1 = 0x45,
  V4ADSR2 = 0x46,
  V4GAIN = 0x47,
  V4ENVX = 0x48,
  V4OUTX = 0x49,
  KON = 0x4C,
  EON = 0x4D,
  FIR4 = 0x4F,
  V5VOLL = 0x50,
  V5VOLR = 0x51,
  V5PL = 0x52,
  V5PH = 0x53,
  V5SRCN = 0x54,
  V5ADSR1 = 0x55,
  V5ADSR2 = 0x56,
  V5GAIN = 0x57,
  V5ENVX = 0x58,
  V5OUTX = 0x59,
  KOF = 0x5C,
  DIR = 0x5D,
  FIR5 = 0x5F,
  V6VOLL = 0x60,
  V6VOLR = 0x61,
  V6PL = 0x62,
  V6PH = 0x63,
  V6SRCN = 0x64,
  V6ADSR1 = 0x65,
  V6ADSR2 = 0x66,
  V6GAIN = 0x67,
  V6ENVX = 0x68,
  V6OUTX = 0x69,
  FLG = 0x6C,
  ESA = 0x6D,
  FIR6 = 0x6F,
  V7VOLL = 0x70,
  V7VOLR = 0x71,
  V7PL = 0x72,
  V7PH = 0x73,
  V7SRCN = 0x74,
  V7ADSR1 = 0x75,
  V7ADSR2 = 0x76,
  V7GAIN = 0x77,
  V7ENVX = 0x78,
  V7OUTX = 0x79,
  ENDX = 0x7C,
  EDL = 0x7D,
  FIR7 = 0x7F,
};
#endif  // DSP_REGS_H

/* === cpu.h === */

#ifndef CPU_H
#define CPU_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>



typedef struct Cpu Cpu;

struct Cpu {
  // reference to memory handler, for reading//writing
  void* mem;
  uint8_t memType; // used to define which type mem is
  // registers
  uint16_t a;
  uint16_t x;
  uint16_t y;
  uint16_t sp;
  uint16_t pc;
  uint16_t dp; // direct page (D)
  uint8_t k; // program bank (PB)
  uint8_t db; // data bank (B)
  // flags
  bool c;
  bool z;
  bool v;
  bool n;
  bool i;
  bool d;
  bool xf;
  bool mf;
  bool e;
  // interrupts
  bool irqWanted;
  bool nmiWanted;
  // power state (WAI/STP)
  bool waiting;
  bool stopped;
  // internal use
  uint8_t cyclesUsed; // indicates how many cycles an opcode used
  uint16_t spBreakpoint;
  bool in_emu;
};

Cpu* cpu_init(void* mem, int memType);
void cpu_free(Cpu* cpu);
void cpu_reset(Cpu* cpu);
int cpu_runOpcode(Cpu* cpu);
void cpu_saveload(Cpu *cpu, SaveLoadFunc *func, void *ctx);
uint8_t cpu_getFlags(Cpu *cpu);
void cpu_setFlags(Cpu *cpu, uint8_t val);

#endif


/* === dma.h === */

#ifndef DMA_H
#define DMA_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct Dma Dma;



typedef struct DmaChannel {
  uint8_t bAdr;
  uint8_t aBank;
  uint8_t indBank; // hdma
  uint8_t repCount; // hdma
  uint16_t aAdr;
  uint16_t size; // also indirect hdma adr
  uint16_t tableAdr; // hdma
  uint8_t unusedByte;
  bool dmaActive;
  bool hdmaActive;
  uint8_t mode;
  bool fixed;
  bool decrement;
  bool indirect; // hdma
  bool fromB;
  bool unusedBit;
  bool doTransfer; // hdma
  bool terminated; // hdma
  uint8_t offIndex;
} DmaChannel;

struct Dma {
  Snes* snes;
  DmaChannel channel[8];
  uint16_t hdmaTimer;
  uint32_t dmaTimer;
  bool dmaBusy;
};

Dma* dma_init(Snes* snes);
void dma_free(Dma* dma);
void dma_reset(Dma* dma);
uint8_t dma_read(Dma* dma, uint16_t adr); // 43x0-43xf
void dma_write(Dma* dma, uint16_t adr, uint8_t val); // 43x0-43xf
void dma_doDma(Dma* dma);
void dma_initHdma(Dma* dma);
void dma_doHdma(Dma* dma);
bool dma_cycle(Dma* dma);
void dma_startDma(Dma* dma, uint8_t val, bool hdma);
void dma_saveload(Dma *dma, SaveLoadFunc *func, void *ctx);

#endif


/* === ppu.h === */
#ifndef PPU_H
#define PPU_H



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>



typedef struct Ppu Ppu;

typedef struct BgLayer {
  uint16_t xhScroll;
  uint16_t xvScroll;
  bool xtilemapWider;
  bool xtilemapHigher;
  uint16_t xtilemapAdr;
  uint16_t xtileAdr;
  bool xxbigTiles;
  bool xxmosaicEnabled;
} BgLayer;

enum {
  kPpuExtraLeftRight = 64,
  kPpuXPixels = 256 + kPpuExtraLeftRight * 2,
};

typedef uint16_t PpuZbufType;

typedef struct PpuPixelPrioBufs {
  // This holds the prio in the upper 8 bits and the color in the lower 8 bits.
  PpuZbufType data[kPpuXPixels];
} PpuPixelPrioBufs;

enum {
  kPpuRenderFlags_NewRenderer = 1,
  // Render mode7 upsampled by 4x4
  kPpuRenderFlags_4x4Mode7 = 2,
  // Use 240 height instead of 224
  kPpuRenderFlags_Height240 = 4,
  // Disable sprite render limits
  kPpuRenderFlags_NoSpriteLimits = 8,
};

typedef struct Layer {
  bool xmainScreenEnabled;
  bool xsubScreenEnabled;
  bool xmainScreenWindowed;
  bool xsubScreenWindowed;
} Layer;

typedef struct WindowLayer {
  bool xwindow1enabled;
  bool xwindow2enabled;
  bool xwindow1inversed;
  bool xwindow2inversed;
  uint8_t xmaskLogic;
} WindowLayer;

#define PPU_SAVESTATE_REGS_SIZE 0x40
#define PPU_SAVESTATE_MEM_SIZE 0x10420

struct Ppu {
  // Snes registers. Saved to snapshot. Need to be stable
  // -- START OF SNAPSHOT, 0x40 bytes
  uint8 inidisp;
  uint8 obsel;
  uint8 oamaddl;
  uint8 oamaddh;
  uint8 bgmode;
  uint8 mosaic;
  uint8 bgXsc[4];
  uint16 bgTileAdr;
  uint8 m7sel;
  uint8 setini;
  uint16 hScroll[4];
  uint16 vScroll[4];
  int16_t m7matrix[8]; // a, b, c, d, x, y, h, v
  uint16 fixedColor;
  uint32 windowsel;
  uint8 window1left;
  uint8 window1right;
  uint8 window2left;
  uint8 window2right;
  uint16 wbgobjlog;
  uint8 screenEnabled[2];
  uint8 screenWindowed[2];
  uint8 cgadsub;
  uint8 cgwsel;
  // -- END OF SNAPSHOT

  // vram access
  uint16_t vramPointer;
  bool vramIncrementOnHigh;
  uint8_t vramRemapMode;
  uint8_t vramIncrement;
  uint16_t vramReadBuffer;
  // cgram access
  uint8_t cgramPointer;
  bool cgramSecondWrite;
  uint8_t cgramBuffer;
  // oam access
  uint8_t oamAdr;
  bool oamInHigh;
  bool oamSecondWrite;
  uint8_t oamBuffer;
  bool timeOver;
  bool rangeOver;
  uint8_t scrollPrev;
  uint8_t scrollPrev2;
  uint8_t mosaicStartLine;
  uint8_t m7prev;
  // mode 7 internal
  int32_t m7startX;
  int32_t m7startY;
  // settings
  bool evenFrame;
  bool frameOverscan; // if we are overscanning this frame (determined at 0,225)
  bool frameInterlace; // if we are interlacing this frame (determined at start vblank)
  // latching
  uint16_t hCount;
  uint16_t vCount;
  bool hCountSecond;
  bool vCountSecond;
  bool countersLatched;
  // pixel buffer (xbgr)
  // times 2 for even and odd frame

  uint8_t extraLeftCur, extraRightCur, extraLeftRight;
  uint8_t lastMosaicModulo;
  uint8_t lastBrightnessMult;
  bool lineHasSprites;
  PpuPixelPrioBufs bgBuffers[2];
  PpuPixelPrioBufs objBuffer;
  uint32_t renderPitch;
  uint8_t *renderBuffer;
  uint8_t brightnessMult[32 + 31];
  uint8_t brightnessMultHalf[32 * 2];
  uint8_t mosaicModulo[kPpuXPixels];

  void *pad2;

  // -- START OF SNAPSHOT, 0x10420 bytes
  uint16_t cgram[0x100];
  uint16_t oam[0x100];
  uint8_t highOam[0x20];
  uint16_t vram[0x8000];
  // -- END OF SNAPSHOT
  uint8_t vram_bgmode[0x8000];
  uint8_t vram_epoch;
  bool was_visible;
  uint16_t col_streamed_mask;
};

#define SPRITE_PRIO_TO_PRIO(prio, level6) (((prio) * 4 + 2) * 16 + 4 + (level6 ? 2 : 0))
#define SPRITE_PRIO_TO_PRIO_HI(prio) ((prio) * 4 + 2)

#define IS_SCREEN_ENABLED(ppu, sub, layer) (ppu->screenEnabled[sub] & (1 << layer))
#define IS_SCREEN_WINDOWED(ppu, sub, layer) (ppu->screenWindowed[sub] & (1 << layer))
#define GET_WINDOW_FLAGS(ppu, layer) (ppu->windowsel >> (layer * 4))

#define PPU_brightness(ppu) (ppu->inidisp & 0xf)
#define PPU_forcedBlank(ppu) (ppu->inidisp & 0x80)

#define PPU_objSize(ppu) (ppu->obsel >> 5)
#define PPU_objTileAdr1(ppu) ((ppu->obsel & 7) << 13)
#define PPU_objTileAdr2(ppu) (PPU_objTileAdr1(ppu) + (((ppu->obsel & 0x18) + 8) << 9))

#define PPU_objPriority(ppu) (ppu->oamaddh & 0x80)

#define PPU_mode(ppu) (ppu->bgmode & 7)
#define PPU_bg3priority(ppu) (ppu->bgmode & 0x8)
#define PPU_bigTiles(ppu, layer) (ppu->bgmode >> layer & 0x10)

#define PPU_mosaicEnabled(ppu, layer) (ppu->mosaic & (1 << layer))
#define PPU_mosaicSize(ppu) ((ppu->mosaic >> 4) + 1)

#define PPU_bgTilemapWider(ppu, layer) (ppu->bgXsc[layer] & 0x1)
#define PPU_bgTilemapHigher(ppu, layer) (ppu->bgXsc[layer] & 0x2)
#define PPU_bgTilemapAdr(ppu, layer) ((ppu->bgXsc[layer] & 0xfc) << 8)
#define PPU_bgTileAdr(ppu, layer) ((ppu->bgTileAdr >> (layer * 4) & 0xf) << 12)

#define PPU_m7xFlip(ppu) (ppu->m7sel & 0x1)
#define PPU_m7yFlip(ppu) (ppu->m7sel & 0x2)
#define PPU_m7charFill(ppu) (ppu->m7sel & 0x40)
#define PPU_m7largeField(ppu) (ppu->m7sel & 0x80)

#define PPU_directColor(ppu) ((ppu->cgwsel & 0x1) != 0)
#define PPU_addSubscreen(ppu) ((ppu->cgwsel & 0x2) != 0)
#define PPU_preventMathMode(ppu) (ppu->cgwsel >> 4 & 0x3)
#define PPU_clipMode(ppu) (ppu->cgwsel >> 6 & 0x3)

#define PPU_mathEnabled(ppu) (ppu->cgadsub & 0x3f)
#define PPU_halfColor(ppu) ((ppu->cgadsub & 0x40) != 0)
#define PPU_subtractColor(ppu) ((ppu->cgadsub & 0x80) != 0)

#define PPU_fixedColorR(ppu) (ppu->fixedColor & 0x1f)
#define PPU_fixedColorG(ppu) (ppu->fixedColor >> 5 & 0x1f)
#define PPU_fixedColorB(ppu) (ppu->fixedColor >> 10 & 0x1f)

#define PPU_interlace(ppu) ((ppu->setini & 0x1) != 0)
#define PPU_objInterlace(ppu) ((ppu->setini & 0x2) != 0)
#define PPU_overscan(ppu) ((ppu->setini & 0x4) != 0)
#define PPU_pseudoHires(ppu) ((ppu->setini & 0x8) != 0)
#define PPU_m7extBg(ppu) ((ppu->setini & 0x40) != 0)


enum {
  kWindow1Inversed = 1,
  kWindow1Enabled = 2,
  kWindow2Inversed = 4,
  kWindow2Enabled = 8,
};


Ppu* ppu_init(void);
void ppu_free(Ppu* ppu);
void ppu_copy(Ppu *ppu, Ppu *ppu_src);
void ppu_reset(Ppu* ppu);
bool ppu_checkOverscan(Ppu* ppu);
void ppu_handleVblank(Ppu* ppu);
void ppu_runLine(Ppu* ppu, int line);
uint8_t ppu_read(Ppu* ppu, uint8_t adr);
void ppu_write(Ppu* ppu, uint8_t adr, uint8_t val);
void ppu_saveload(Ppu *ppu, SaveLoadFunc *func, void *ctx);
void PpuSetExtraSideSpace(Ppu *ppu, int left, int right, int bottom);
void PpuBeginDrawing(Ppu *ppu, uint8_t *pixels, size_t pitch, uint32_t render_flags);

int PpuGetCurrentRenderScale(Ppu *ppu, uint32_t render_flags);

#endif


/* === cart.h === */

#ifndef CART_H
#define CART_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct Cart Cart;
typedef struct Sa1 Sa1;

struct Cart {
  Snes* snes;
  uint8_t type;

  uint8_t* rom;
  uint32_t romSize;
  uint8_t* ram;
  uint32_t ramSize;
  Sa1* sa1;
  uint64_t masterClock;
  uint32_t cpuBusAddress;
};

static uint8_t sa1_bus_read(void *opaque, uint32_t address);
static void sa1_bus_write(void *opaque, uint32_t address, uint8_t data);
void sa1_reset(Sa1 *sa1);
bool cart_cpu_irq_pending(const Cart* cart);
void cart_sync_coprocessors(Cart* cart);

// TODO: how to handle reset & load? (especially where to init ram)

Cart* cart_init(Snes* snes);
void cart_free(Cart* cart);
void cart_reset(Cart* cart); // will reset special chips etc, general reading is set up in load
void cart_load(Cart* cart, int type, uint8_t* rom, int romSize, int ramSize); // TODO: figure out how to handle (battery, cart-chips etc)
uint8_t cart_read(Cart* cart, uint8_t bank, uint16_t adr);
void cart_write(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val);
void cart_saveload(Cart *cart, SaveLoadFunc *func, void *ctx);

#endif


/* === input.h === */

#ifndef INPUT_H
#define INPUT_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct Input Input;



struct Input {
  Snes* snes;
  uint8_t type;
  // latchline
  bool latchLine;
  // for controller
  uint16_t currentState; // actual state
  uint16_t latchedState;
};

Input* input_init(Snes* snes);
void input_free(Input* input);
void input_reset(Input* input);
void input_cycle(Input* input);
uint8_t input_read(Input* input);

#endif


/* === dsp.h === */

#ifndef DSP_H
#define DSP_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>


typedef struct Dsp Dsp;



typedef struct DspChannel {
  // pitch
  uint16_t pitch;
  uint16_t pitchCounter;
  bool pitchModulation;
  // brr decoding
  int16_t decodeBuffer[19]; // 16 samples per brr-block, +3 for interpolation
  uint8_t srcn;
  uint16_t decodeOffset;
  uint8_t previousFlags; // from last sample
  int16_t old;
  int16_t older;
  bool useNoise;
  // adsr, envelope, gain
  uint16_t adsrRates[4]; // attack, decay, sustain, gain
  uint16_t rateCounter;
  uint8_t adsrState; // 0: attack, 1: decay, 2: sustain, 3: gain, 4: release
  uint16_t sustainLevel;
  bool useGain;
  uint8_t gainMode;
  bool directGain;
  uint16_t gainValue; // for direct gain
  uint16_t gain;
  // keyon/off
  bool keyOn;
  bool keyOff;
  // output
  int16_t sampleOut; // final sample, to be multiplied by channel volume
  int8_t volumeL;
  int8_t volumeR;
  bool echoEnable;
} DspChannel;

struct Dsp {
  uint8_t *apu_ram;
  // mirror ram
  uint8_t ram[0x80];
  // 8 channels
  DspChannel channel[8];
  // overarching
  uint16_t dirPage;
  bool evenCycle;
  bool mute;
  bool reset;
  int8_t masterVolumeL;
  int8_t masterVolumeR;
  // noise
  int16_t noiseSample;
  uint16_t noiseRate;
  uint16_t noiseCounter;
  // echo
  bool echoWrites;
  int8_t echoVolumeL;
  int8_t echoVolumeR;
  int8_t feedbackVolume;
  uint16_t echoBufferAdr;
  uint16_t echoDelay;
  uint16_t echoRemain;
  uint16_t echoBufferIndex;
  uint8_t firBufferIndex;
  int8_t firValues[8];
  int16_t firBufferL[8];
  int16_t firBufferR[8];
  // sample buffer (1 frame at 32040 Hz: 534 samples, *2 for stereo)
  int16_t sampleBuffer[534 * 2];
  uint16_t sampleOffset; // current offset in samplebuffer
};


typedef struct DspRegWriteHistory {
  uint32_t count;
  uint8_t addr[256];
  uint8_t val[256];
} DspRegWriteHistory;

Dsp* dsp_init(uint8_t *apu_ram);
void dsp_free(Dsp* dsp);
void dsp_reset(Dsp* dsp);
void dsp_cycle(Dsp* dsp);
uint8_t dsp_read(Dsp* dsp, uint8_t adr);
void dsp_write(Dsp* dsp, uint8_t adr, uint8_t val);
void dsp_getSamples(Dsp* dsp, int16_t* sampleData, int samplesPerFrame, int numChannels);
void dsp_saveload(Dsp *dsp, SaveLoadFunc *func, void *ctx);

#endif


/* === spc.h === */

#ifndef SPC_H
#define SPC_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct Spc Spc;




struct Spc {
  Apu* apu;
  // registers
  uint8_t a;
  uint8_t x;
  uint8_t y;
  uint8_t sp;
  uint16_t pc;
  // flags
  bool c;
  bool z;
  bool v;
  bool n;
  bool i;
  bool h;
  bool p;
  bool b;
  // stopping
  bool stopped;
  // internal use
  uint8_t cyclesUsed; // indicates how many cycles an opcode used
};

Spc* spc_init(Apu* apu);
void spc_free(Spc* spc);
void spc_reset(Spc* spc);
int spc_runOpcode(Spc* spc);
void spc_saveload(Spc *spc, SaveLoadFunc *func, void *ctx);

#endif


/* === apu.h === */

#ifndef APU_H
#define APU_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct Apu Apu;




typedef struct Timer {
  uint8_t cycles;
  uint8_t divider;
  uint8_t target;
  uint8_t counter;
  bool enabled;
} Timer;


struct Apu {
  Spc* spc;
  Dsp* dsp;
  uint8_t ram[0x10000];
  bool romReadable;
  uint8_t dspAdr;
  uint32_t cycles;
  uint8_t inPorts[6]; // includes 2 bytes of ram
  uint8_t outPorts[4];
  Timer timer[3];
  uint8_t cpuCyclesLeft;
  union {
    DspRegWriteHistory hist;
    void *padpad;
  };
};

typedef struct Apu2 {
  // Snes* snes;
  Spc* spc;
  Dsp* dsp;
  uint8_t ram[0x10000];
  bool romReadable;
  uint8_t dspAdr;
  uint32_t cycles;
  uint8_t inPorts[6]; // includes 2 bytes of ram
  uint8_t outPorts[4];
  Timer timer[3];
  uint8_t cpuCyclesLeft;
} Apu2;

Apu* apu_init();
void apu_free(Apu* apu);
void apu_reset(Apu* apu);
void apu_cycle(Apu* apu);
uint8_t apu_cpuRead(Apu* apu, uint16_t adr);
void apu_cpuWrite(Apu* apu, uint16_t adr, uint8_t val);
void apu_saveload(Apu *apu, SaveLoadFunc *func, void *ctx);

#endif


/* === snes.h === */

#ifndef SNES_H
#define SNES_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct Snes Snes;








struct Snes {
  Cpu* cpu;
  Apu* apu;
  Ppu* ppu;
  Dma* dma;
  Cart* cart;
  // input
  bool debug_cycles;
  bool disableHpos;
  Input* input1;
  Input* input2;
  

  // frame timing
  

  uint16_t hPos;
  uint16_t vPos;
  uint32_t frames;
  // cpu handling
  uint8_t cpuCyclesLeft;
  uint8_t cpuMemOps;
  double apuCatchupCycles;
  // nmi / irq
  bool hIrqEnabled;
  bool vIrqEnabled;
  bool nmiEnabled;
  uint16_t hTimer;
  uint16_t vTimer;
  bool inNmi;
  bool inIrq;
  bool inVblank;
  // joypad handling
  uint16_t portAutoRead[4]; // as read by auto-joypad read
  bool autoJoyRead;
  uint16_t autoJoyTimer; // times how long until reading is done
  bool ppuLatch;
  // multiplication/division
  uint8_t multiplyA;
  uint16_t multiplyResult;
  uint16_t divideA;
  uint16_t divideResult;
  // misc
  bool fastMem;
  uint8_t openBus;
  // ram
  uint8_t *ram;
  uint32_t ramAdr;
};

Snes* snes_init(uint8_t *ram);
void snes_free(Snes* snes);
void snes_reset(Snes* snes, bool hard);
// used by dma, cpu
uint8_t snes_readBBus(Snes* snes, uint8_t adr);
void snes_writeBBus(Snes* snes, uint8_t adr, uint8_t val);
uint8_t snes_read(Snes* snes, uint32_t adr);
void snes_write(Snes* snes, uint32_t adr, uint8_t val);
uint8_t snes_cpuRead(Snes* snes, uint32_t adr);
void snes_cpuWrite(Snes* snes, uint32_t adr, uint8_t val);
// debugging
void snes_printCpuLine(Snes *snes);
void snes_doAutoJoypad(Snes *snes);
// snes_other.c functions:

bool snes_loadRom(Snes* snes, uint8_t* data, int length);
void snes_saveload(Snes *snes, SaveLoadFunc *func, void *ctx);


#endif



/* === input.c === */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>




Input* input_init(Snes* snes) {
  Input* input = (Input * )malloc(sizeof(Input));
  input->snes = snes;
  // TODO: handle (where?)
  input->type = 1;
  input->currentState = 0;
  return input;
}

void input_free(Input* input) {
  free(input);
}

void input_reset(Input* input) {
  input->latchLine = false;
  input->latchedState = 0;
}

void input_cycle(Input* input) {
  if(input->latchLine) {
    input->latchedState = input->currentState;
  }
}

uint8_t input_read(Input* input) {
  if (input->latchLine) {
    input->latchedState = input->currentState;
  }
  uint8_t ret = (input->latchedState >> 15) & 1;
  input->latchedState = (input->latchedState << 1) | 1;
  return ret;
}


/* === cart.c === */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>



/*
 * Nintendo SA-1 coprocessor for SNESRecomp.
 *
 * The instruction engine is interp816 (LakeSnes, MIT). The surrounding SA-1
 * memory map and peripheral behavior are a C adaptation of ares' SA-1 core:
 *
 *   Copyright (c) 2004-2025 ares team, Near et al
 *   ISC license; see THIRD_PARTY_ATTRIBUTION.md.
 *
 * This file deliberately models the chip, not Super Mario RPG. No title
 * addresses or command shortcuts live here.
 */





#include <limits.h>
#include <stdlib.h>
#include <string.h>

enum {
  SA1_IRAM_SIZE = 2048,
  SA1_MASTER_CLOCKS_PER_CPU_CYCLE = 2,
};

typedef struct Sa1Io {
  /* $2200-$220f: CPU control and interrupt communication. */
  bool sa1_irq, sa1_ready, sa1_reset, sa1_nmi;
  uint8_t smeg;
  bool cpu_irq_enable, chdma_irq_enable;
  uint16_t reset_vector, nmi_vector, irq_vector;
  bool cpu_irq, cpu_irq_vector_switch, cpu_nmi_vector_switch;
  uint8_t cmeg;
  bool sa1_irq_enable, timer_irq_enable, dma_irq_enable, sa1_nmi_enable;
  uint16_t snes_nmi_vector, snes_irq_vector;

  /* Timer. */
  bool timer_linear, timer_v_enable, timer_h_enable;
  uint16_t h_count_target, v_count_target;

  /* Super MMC and write protection. */
  bool bank_mode[4];
  uint8_t bank_select[4];
  uint8_t snes_bwram_page;
  bool bwram_bitmap_window;
  uint8_t sa1_bwram_page;
  bool snes_bwram_write_enable, sa1_bwram_write_enable;
  uint8_t bwram_protect;
  uint8_t snes_iram_write_mask, sa1_iram_write_mask;

  /* DMA and character conversion. */
  bool dma_enable, dma_priority, char_conversion_enable;
  bool char_conversion_select, dma_dest_bwram;
  uint8_t dma_source;
  bool char_dma_end;
  uint8_t char_dma_size, char_dma_bpp;
  uint32_t dma_source_address, dma_dest_address;
  uint16_t dma_count;
  bool bitmap_2bpp;
  uint8_t bitmap_registers[16];

  /* Arithmetic and variable-bit reader. */
  bool arithmetic_accumulate, arithmetic_divide;
  uint16_t arithmetic_a, arithmetic_b;
  uint64_t arithmetic_result;
  bool arithmetic_overflow;
  bool variable_auto_increment;
  uint8_t variable_length, variable_bit;
  uint32_t variable_address;

  /* Flag registers. */
  bool cpu_irq_flag, chdma_irq_flag;
  bool sa1_irq_flag, timer_irq_flag, dma_irq_flag, sa1_nmi_flag;
  uint16_t latched_h_count, latched_v_count;
} Sa1Io;

struct Sa1 {
  uint8_t *rom;
  uint32_t rom_size;
  uint8_t *bwram;
  uint32_t bwram_size;
  uint8_t iram[SA1_IRAM_SIZE];
  Cpu *cpu;
  Sa1Io io;

  uint64_t master_clock;
  uint64_t instructions;
  uint32_t h_counter;
  uint16_t v_counter;
  uint8_t char_dma_line;
  bool bwram_char_dma;
  uint8_t open_bus;
  uint8_t interrupt_vector_kind; /* 1=NMI, 2=IRQ */
  uint32_t extra_cycles;
  uint32_t cpu_bus_address;
};

static uint8_t sa1_bus_read(void *opaque, uint32_t address);
static void sa1_bus_write(void *opaque, uint32_t address, uint8_t data);

static bool cpu_bus_owns_rom(const Sa1 *sa1) {
  uint32_t address = sa1->cpu_bus_address;
  return (address & 0x408000u) == 0x008000u ||
         (address & 0xc00000u) == 0xc00000u;
}

static uint32_t mirror_offset(uint32_t address, uint32_t size) {
  if (!size) return 0;
  if ((size & (size - 1)) == 0) return address & (size - 1);
  return address % size;
}

static uint8_t bwram_read_raw(Sa1 *sa1, uint32_t address) {
  if (!sa1->bwram || !sa1->bwram_size) return sa1->open_bus;
  return sa1->bwram[mirror_offset(address, sa1->bwram_size)];
}

static void bwram_write_raw(Sa1 *sa1, uint32_t address, uint8_t data) {
  if (!sa1->bwram || !sa1->bwram_size) return;
  sa1->bwram[mirror_offset(address, sa1->bwram_size)] = data;
}

static bool bwram_write_allowed(const Sa1 *sa1, uint32_t address) {
  if (sa1->io.snes_bwram_write_enable || sa1->io.sa1_bwram_write_enable)
    return true;
  uint32_t protected_size = 0x100u << sa1->io.bwram_protect;
  return (address & 0x3ffffu) >= protected_size;
}

static uint8_t bwram_read_bitmap(Sa1 *sa1, uint32_t address) {
  if (!sa1->io.bitmap_2bpp) {
    uint8_t value = bwram_read_raw(sa1, address >> 1);
    return (uint8_t)((value >> ((address & 1u) * 4u)) & 0x0f);
  }
  uint8_t value = bwram_read_raw(sa1, address >> 2);
  return (uint8_t)((value >> ((address & 3u) * 2u)) & 0x03);
}

static void bwram_write_bitmap(Sa1 *sa1, uint32_t address, uint8_t data) {
  uint32_t physical;
  unsigned shift;
  uint8_t mask;
  if (!sa1->io.bitmap_2bpp) {
    physical = address >> 1;
    shift = (address & 1u) * 4u;
    mask = 0x0f;
  } else {
    physical = address >> 2;
    shift = (address & 3u) * 2u;
    mask = 0x03;
  }
  if (!bwram_write_allowed(sa1, physical)) return;
  uint8_t value = bwram_read_raw(sa1, physical);
  value = (uint8_t)((value & ~(mask << shift)) | ((data & mask) << shift));
  bwram_write_raw(sa1, physical, value);
}

/* Convert a CPU-visible ROM address into the logical 0-4 MiB Super MMC
 * address before applying the four one-megabyte bank selectors. */
static bool rom_logical_offset(uint8_t bank, uint16_t address,
                               uint32_t *offset, bool *low_window) {
  uint8_t canonical = bank & 0x7f;
  if (address >= 0x8000 && canonical < 0x40) {
    uint32_t high_half = (bank & 0x80) ? 0x200000u : 0;
    *offset = high_half | ((uint32_t)canonical << 15)
            | (address & 0x7fffu);
    *low_window = true;
    return true;
  }
  if (bank >= 0xc0) {
    *offset = ((uint32_t)(bank - 0xc0) << 16) | address;
    *low_window = false;
    return true;
  }
  return false;
}

static uint32_t rom_mmc_offset(const Sa1 *sa1, uint32_t logical,
                               bool low_window) {
  unsigned segment = (logical >> 20) & 3u;
  if (low_window && !sa1->io.bank_mode[segment])
    return mirror_offset(logical, sa1->rom_size);
  uint32_t selected = (uint32_t)(sa1->io.bank_select[segment] & 7u) << 20;
  return mirror_offset(selected | (logical & 0x0fffffu), sa1->rom_size);
}

static bool cpu_vector_override(const Sa1 *sa1, uint8_t bank,
                                uint16_t address, uint8_t *value) {
  if ((bank & 0x7f) != 0 || address < 0xffe0 || address >= 0xfff0)
    return false;
  if (sa1->io.cpu_nmi_vector_switch &&
      (address == 0xffea || address == 0xffeb)) {
    *value = (uint8_t)(sa1->io.snes_nmi_vector >>
                       ((address & 1u) * 8u));
    return true;
  }
  if (sa1->io.cpu_irq_vector_switch &&
      (address == 0xffee || address == 0xffef)) {
    *value = (uint8_t)(sa1->io.snes_irq_vector >>
                       ((address & 1u) * 8u));
    return true;
  }
  return false;
}

static uint8_t rom_cpu_read(Sa1 *sa1, uint8_t bank, uint16_t address,
                            uint8_t open_bus) {
  uint8_t vector;
  if (cpu_vector_override(sa1, bank, address, &vector)) return vector;
  uint32_t logical;
  bool low_window;
  if (!sa1->rom || !sa1->rom_size ||
      !rom_logical_offset(bank, address, &logical, &low_window))
    return open_bus;
  return sa1->rom[rom_mmc_offset(sa1, logical, low_window)];
}

static uint8_t *rom_cpu_ptr(Sa1 *sa1, uint8_t bank, uint16_t address) {
  uint8_t ignored;
  if (cpu_vector_override(sa1, bank, address, &ignored)) return NULL;
  uint32_t logical;
  bool low_window;
  if (!sa1->rom || !sa1->rom_size ||
      !rom_logical_offset(bank, address, &logical, &low_window))
    return NULL;
  return &sa1->rom[rom_mmc_offset(sa1, logical, low_window)];
}

static uint8_t iram_read(Sa1 *sa1, uint32_t address) {
  return sa1->iram[address & (SA1_IRAM_SIZE - 1)];
}

static void iram_write(Sa1 *sa1, uint32_t address, uint8_t data,
                       bool from_sa1) {
  unsigned page = (address >> 8) & 7u;
  uint8_t mask = from_sa1 ? sa1->io.sa1_iram_write_mask
                          : sa1->io.snes_iram_write_mask;
  if (mask & (1u << page))
    sa1->iram[address & (SA1_IRAM_SIZE - 1)] = data;
}

static uint8_t variable_read(Sa1 *sa1, uint32_t address) {
  uint8_t bank = (uint8_t)(address >> 16);
  uint16_t lo = (uint16_t)address;
  uint32_t logical;
  bool low_window;
  if (rom_logical_offset(bank, lo, &logical, &low_window))
    return sa1->rom[rom_mmc_offset(sa1, logical, low_window)];
  if (((bank & 0x7f) < 0x40) && lo >= 0x6000 && lo < 0x8000) {
    uint32_t offset = (uint32_t)sa1->io.snes_bwram_page * 0x2000u
                    + (lo & 0x1fffu);
    return bwram_read_raw(sa1, offset);
  }
  if (bank >= 0x40 && bank < 0x50)
    return bwram_read_raw(sa1, ((uint32_t)(bank - 0x40) << 16) | lo);
  if (((bank & 0x7f) < 0x40) &&
      (lo < 0x0800 || (lo >= 0x3000 && lo < 0x3800)))
    return iram_read(sa1, lo);
  return 0xff;
}

static uint8_t variable_port(Sa1 *sa1, bool high) {
  uint32_t bits = (uint32_t)variable_read(sa1, sa1->io.variable_address)
                | ((uint32_t)variable_read(sa1,
                    (sa1->io.variable_address + 1) & 0xffffffu) << 8)
                | ((uint32_t)variable_read(sa1,
                    (sa1->io.variable_address + 2) & 0xffffffu) << 16);
  bits >>= sa1->io.variable_bit;
  uint8_t result = (uint8_t)(bits >> (high ? 8 : 0));
  if (high && sa1->io.variable_auto_increment) {
    unsigned advance = sa1->io.variable_bit + sa1->io.variable_length;
    sa1->io.variable_address =
        (sa1->io.variable_address + (advance >> 3)) & 0xffffffu;
    sa1->io.variable_bit = (uint8_t)(advance & 7u);
  }
  return result;
}

static void update_snes_irq(Sa1 *sa1) {
  (void)sa1;
  /* The line is queried through sa1_cpu_irq_pending(). Keeping it derived
   * avoids stealing the SNES timer IRQ latch from the base console model. */
}

static uint8_t io_read_cpu(Sa1 *sa1, uint16_t address, uint8_t open_bus) {
  if (address == 0x2300) {
    return (uint8_t)((sa1->io.cmeg & 0x0f)
        | (sa1->io.cpu_nmi_vector_switch ? 0x10 : 0)
        | (sa1->io.chdma_irq_flag ? 0x20 : 0)
        | (sa1->io.cpu_irq_vector_switch ? 0x40 : 0)
        | (sa1->io.cpu_irq_flag ? 0x80 : 0));
  }
  return open_bus;
}

static uint8_t io_read_sa1(Sa1 *sa1, uint16_t address) {
  switch (address) {
    case 0x2301:
      return (uint8_t)((sa1->io.smeg & 0x0f)
          | (sa1->io.sa1_nmi_flag ? 0x10 : 0)
          | (sa1->io.dma_irq_flag ? 0x20 : 0)
          | (sa1->io.timer_irq_flag ? 0x40 : 0)
          | (sa1->io.sa1_irq_flag ? 0x80 : 0));
    case 0x2302:
      sa1->io.latched_h_count = (uint16_t)(sa1->h_counter >> 2);
      sa1->io.latched_v_count = sa1->v_counter;
      return (uint8_t)sa1->io.latched_h_count;
    case 0x2303: return (uint8_t)(sa1->io.latched_h_count >> 8);
    case 0x2304: return (uint8_t)sa1->io.latched_v_count;
    case 0x2305: return (uint8_t)(sa1->io.latched_v_count >> 8);
    case 0x2306: return (uint8_t)(sa1->io.arithmetic_result >> 0);
    case 0x2307: return (uint8_t)(sa1->io.arithmetic_result >> 8);
    case 0x2308: return (uint8_t)(sa1->io.arithmetic_result >> 16);
    case 0x2309: return (uint8_t)(sa1->io.arithmetic_result >> 24);
    case 0x230a: return (uint8_t)(sa1->io.arithmetic_result >> 32);
    case 0x230b: return sa1->io.arithmetic_overflow ? 0x80 : 0;
    case 0x230c: return variable_port(sa1, false);
    case 0x230d: return variable_port(sa1, true);
    default: return 0xff;
  }
}

static void timer_advance(Sa1 *sa1, uint32_t clocks) {
  while (clocks--) {
    if (!sa1->io.timer_linear) {
      if (++sa1->h_counter >= 1364) {
        sa1->h_counter = 0;
        if (++sa1->v_counter >= 262) sa1->v_counter = 0;
      }
    } else {
      if (++sa1->h_counter >= 2048) {
        sa1->h_counter = 0;
        sa1->v_counter = (uint16_t)((sa1->v_counter + 1) & 0x01ff);
      }
    }
    bool match = false;
    if (sa1->io.timer_h_enable && sa1->io.timer_v_enable)
      match = sa1->h_counter == ((uint32_t)sa1->io.h_count_target << 2)
           && sa1->v_counter == sa1->io.v_count_target;
    else if (sa1->io.timer_h_enable)
      match = sa1->h_counter == ((uint32_t)sa1->io.h_count_target << 2);
    else if (sa1->io.timer_v_enable)
      match = sa1->h_counter == 0
           && sa1->v_counter == sa1->io.v_count_target;
    if (match) sa1->io.timer_irq_flag = true;
  }
}

static void dma_finish(Sa1 *sa1) {
  sa1->io.dma_irq_flag = true;
}

static uint8_t dma_source_read(Sa1 *sa1, uint32_t address) {
  if (sa1->io.dma_source == 0) {
    uint8_t bank = (uint8_t)(address >> 16);
    return rom_cpu_read(sa1, bank, (uint16_t)address, sa1->open_bus);
  }
  if (sa1->io.dma_source == 1) return bwram_read_raw(sa1, address);
  if (sa1->io.dma_source == 2) return iram_read(sa1, address);
  return sa1->open_bus;
}

static void dma_normal(Sa1 *sa1) {
  uint32_t count = sa1->io.dma_count ? sa1->io.dma_count : 0x10000u;
  /* ares' cycle-level core charges these clocks inside dmaNormal(). They are
   * in addition to the instruction that completed the DMA register write. */
  uint32_t cycles_per_byte =
      sa1->io.dma_source == 0 && !sa1->io.dma_dest_bwram ? 1u : 2u;
  while (count--) {
    uint8_t data = dma_source_read(sa1, sa1->io.dma_source_address++);
    if (sa1->io.dma_dest_bwram) {
      if (bwram_write_allowed(sa1, sa1->io.dma_dest_address))
        bwram_write_raw(sa1, sa1->io.dma_dest_address, data);
    } else {
      /* DMA is not subject to CIWP. */
      sa1->iram[sa1->io.dma_dest_address & 0x07ff] = data;
    }
    sa1->io.dma_source_address &= 0xffffffu;
    sa1->io.dma_dest_address =
        (sa1->io.dma_dest_address + 1u) & 0xffffffu;
    sa1->extra_cycles += cycles_per_byte;
  }
  sa1->io.dma_count = 0;
  dma_finish(sa1);
}

static void dma_cc1_begin(Sa1 *sa1) {
  sa1->bwram_char_dma = true;
  sa1->io.chdma_irq_flag = true;
  update_snes_irq(sa1);
}

static uint8_t dma_cc1_read(Sa1 *sa1, uint32_t address) {
  unsigned char_bytes = 1u << (6u - sa1->io.char_dma_bpp);
  unsigned char_mask = char_bytes - 1u;
  if ((address & char_mask) == 0) {
    unsigned bpp = 2u << (2u - sa1->io.char_dma_bpp);
    unsigned bytes_per_line =
        (8u << sa1->io.char_dma_size) >> sa1->io.char_dma_bpp;
    uint32_t tile = mirror_offset(address - sa1->io.dma_source_address,
                                  sa1->bwram_size) >> (6u - sa1->io.char_dma_bpp);
    uint32_t tile_y = tile >> sa1->io.char_dma_size;
    uint32_t tile_x = tile & ((1u << sa1->io.char_dma_size) - 1u);
    uint32_t source = sa1->io.dma_source_address
                    + tile_y * 8u * bytes_per_line + tile_x * bpp;
    for (unsigned y = 0; y < 8; y++) {
      uint64_t packed = 0;
      for (unsigned byte = 0; byte < bpp; byte++)
        packed |= (uint64_t)bwram_read_raw(sa1, source + byte) << (byte * 8u);
      source += bytes_per_line;
      uint8_t planar[8] = {0};
      for (unsigned x = 0; x < 8; x++) {
        for (unsigned plane = 0; plane < bpp; plane++) {
          planar[plane] |= (uint8_t)((packed & 1u) << (7u - x));
          packed >>= 1;
        }
      }
      for (unsigned byte = 0; byte < bpp; byte++) {
        uint32_t target = sa1->io.dma_dest_address + y * 2u
                        + ((byte & 6u) << 3) + (byte & 1u);
        sa1->iram[target & 0x07ff] = planar[byte];
      }
    }
  }
  return iram_read(sa1, sa1->io.dma_dest_address + (address & char_mask));
}

static void dma_cc2_line(Sa1 *sa1) {
  const uint8_t *source =
      &sa1->io.bitmap_registers[(sa1->char_dma_line & 1u) << 3];
  unsigned bpp = 2u << (2u - sa1->io.char_dma_bpp);
  uint32_t address = sa1->io.dma_dest_address & 0x07ffu;
  address &= ~((1u << (7u - sa1->io.char_dma_bpp)) - 1u);
  address += (sa1->char_dma_line & 8u) * bpp;
  address += (sa1->char_dma_line & 7u) * 2u;
  for (unsigned plane = 0; plane < bpp; plane++) {
    uint8_t output = 0;
    for (unsigned bit = 0; bit < 8; bit++)
      output |= (uint8_t)(((source[bit] >> plane) & 1u) << (7u - bit));
    sa1->iram[(address + ((plane & 6u) << 3) + (plane & 1u)) & 0x07ff] =
        output;
  }
  sa1->char_dma_line = (uint8_t)((sa1->char_dma_line + 1) & 15);
}

static void io_write_shared(Sa1 *sa1, uint16_t address, uint8_t data) {
  switch (address) {
    case 0x2231:
      sa1->io.char_dma_bpp = data & 3u;
      if (sa1->io.char_dma_bpp > 2) sa1->io.char_dma_bpp = 2;
      sa1->io.char_dma_size = (data >> 2) & 7u;
      if (sa1->io.char_dma_size > 5) sa1->io.char_dma_size = 5;
      sa1->io.char_dma_end = (data & 0x80) != 0;
      if (sa1->io.char_dma_end) sa1->bwram_char_dma = false;
      break;
    case 0x2232:
      sa1->io.dma_source_address =
          (sa1->io.dma_source_address & 0xffff00u) | data;
      break;
    case 0x2233:
      sa1->io.dma_source_address =
          (sa1->io.dma_source_address & 0xff00ffu) | ((uint32_t)data << 8);
      break;
    case 0x2234:
      sa1->io.dma_source_address =
          (sa1->io.dma_source_address & 0x00ffffu) | ((uint32_t)data << 16);
      break;
    case 0x2235:
      sa1->io.dma_dest_address =
          (sa1->io.dma_dest_address & 0xffff00u) | data;
      break;
    case 0x2236:
      sa1->io.dma_dest_address =
          (sa1->io.dma_dest_address & 0xff00ffu) | ((uint32_t)data << 8);
      if (sa1->io.dma_enable) {
        if (!sa1->io.char_conversion_enable &&
            !sa1->io.dma_dest_bwram) dma_normal(sa1);
        else if (sa1->io.char_conversion_enable &&
                 sa1->io.char_conversion_select) dma_cc1_begin(sa1);
      }
      break;
    case 0x2237:
      sa1->io.dma_dest_address =
          (sa1->io.dma_dest_address & 0x00ffffu) | ((uint32_t)data << 16);
      if (sa1->io.dma_enable && !sa1->io.char_conversion_enable &&
          sa1->io.dma_dest_bwram) dma_normal(sa1);
      break;
  }
}

static void io_write_cpu(Sa1 *sa1, uint16_t address, uint8_t data) {
  switch (address) {
    case 0x2200: {
      bool was_reset = sa1->io.sa1_reset;
      bool reset = (data & 0x20) != 0;
      if (was_reset && !reset) {
        cpu_reset(sa1->cpu);
        sa1->cpu->pc = sa1->io.reset_vector;
        sa1->cpu->k = 0;
        sa1->cpu->db = 0;
        sa1->cpu->stopped = false;
        sa1->cpu->waiting = false;
        sa1->io.sa1_iram_write_mask = 0;
      }
      sa1->io.smeg = data & 0x0f;
      sa1->io.sa1_nmi = (data & 0x10) != 0;
      sa1->io.sa1_reset = reset;
      sa1->io.sa1_ready = (data & 0x40) != 0;
      sa1->io.sa1_irq = (data & 0x80) != 0;
      if (sa1->io.sa1_irq) sa1->io.sa1_irq_flag = true;
      if (sa1->io.sa1_nmi) sa1->io.sa1_nmi_flag = true;
      break;
    }
    case 0x2201:
      sa1->io.chdma_irq_enable = (data & 0x20) != 0;
      sa1->io.cpu_irq_enable = (data & 0x80) != 0;
      update_snes_irq(sa1);
      break;
    case 0x2202:
      if (data & 0x20) sa1->io.chdma_irq_flag = false;
      if (data & 0x80) sa1->io.cpu_irq_flag = false;
      update_snes_irq(sa1);
      break;
    case 0x2203:
      sa1->io.reset_vector = (sa1->io.reset_vector & 0xff00) | data; break;
    case 0x2204:
      sa1->io.reset_vector = (sa1->io.reset_vector & 0x00ff)
                           | ((uint16_t)data << 8); break;
    case 0x2205:
      sa1->io.nmi_vector = (sa1->io.nmi_vector & 0xff00) | data; break;
    case 0x2206:
      sa1->io.nmi_vector = (sa1->io.nmi_vector & 0x00ff)
                         | ((uint16_t)data << 8); break;
    case 0x2207:
      sa1->io.irq_vector = (sa1->io.irq_vector & 0xff00) | data; break;
    case 0x2208:
      sa1->io.irq_vector = (sa1->io.irq_vector & 0x00ff)
                         | ((uint16_t)data << 8); break;
    case 0x2220: case 0x2221: case 0x2222: case 0x2223: {
      unsigned slot = address - 0x2220;
      sa1->io.bank_select[slot] = data & 7u;
      sa1->io.bank_mode[slot] = (data & 0x80) != 0;
      break;
    }
    case 0x2224: sa1->io.snes_bwram_page = data & 0x1f; break;
    case 0x2226: sa1->io.snes_bwram_write_enable = (data & 0x80) != 0; break;
    case 0x2228: sa1->io.bwram_protect = data & 0x0f; break;
    case 0x2229: sa1->io.snes_iram_write_mask = data; break;
    case 0x2231: case 0x2232: case 0x2233: case 0x2234:
    case 0x2235: case 0x2236: case 0x2237:
      io_write_shared(sa1, address, data); break;
  }
}

static int16_t signed_remainder(int16_t dividend, uint16_t divisor) {
  int32_t rem = dividend % (int32_t)divisor;
  if (rem < 0) rem += divisor;
  return (int16_t)rem;
}

static void arithmetic_execute(Sa1 *sa1) {
  if (!sa1->io.arithmetic_accumulate) {
    if (!sa1->io.arithmetic_divide) {
      int32_t product = (int16_t)sa1->io.arithmetic_a
                      * (int16_t)sa1->io.arithmetic_b;
      sa1->io.arithmetic_result = (uint32_t)product;
      sa1->io.arithmetic_b = 0;
    } else {
      uint16_t divisor = sa1->io.arithmetic_b;
      if (!divisor) {
        sa1->io.arithmetic_result = 0;
      } else {
        int16_t dividend = (int16_t)sa1->io.arithmetic_a;
        uint16_t remainder = (uint16_t)signed_remainder(dividend, divisor);
        int16_t quotient = (int16_t)((dividend - (int32_t)remainder) /
                                     (int32_t)divisor);
        sa1->io.arithmetic_result =
            ((uint32_t)remainder << 16) | (uint16_t)quotient;
      }
      sa1->io.arithmetic_a = 0;
      sa1->io.arithmetic_b = 0;
    }
  } else {
    int64_t add = (int16_t)sa1->io.arithmetic_a
                * (int16_t)sa1->io.arithmetic_b;
    uint64_t result = sa1->io.arithmetic_result + add;
    sa1->io.arithmetic_overflow = (result & ~UINT64_C(0xffffffffff)) != 0;
    sa1->io.arithmetic_result = result & UINT64_C(0xffffffffff);
    sa1->io.arithmetic_b = 0;
  }
}

static void io_write_sa1(Sa1 *sa1, uint16_t address, uint8_t data) {
  switch (address) {
    case 0x2209:
      sa1->io.cmeg = data & 0x0f;
      sa1->io.cpu_nmi_vector_switch = (data & 0x10) != 0;
      sa1->io.cpu_irq_vector_switch = (data & 0x40) != 0;
      sa1->io.cpu_irq = (data & 0x80) != 0;
      if (sa1->io.cpu_irq) sa1->io.cpu_irq_flag = true;
      update_snes_irq(sa1);
      break;
    case 0x220a:
      sa1->io.sa1_nmi_enable = (data & 0x10) != 0;
      sa1->io.dma_irq_enable = (data & 0x20) != 0;
      sa1->io.timer_irq_enable = (data & 0x40) != 0;
      sa1->io.sa1_irq_enable = (data & 0x80) != 0;
      break;
    case 0x220b:
      if (data & 0x10) sa1->io.sa1_nmi_flag = false;
      if (data & 0x20) sa1->io.dma_irq_flag = false;
      if (data & 0x40) sa1->io.timer_irq_flag = false;
      if (data & 0x80) sa1->io.sa1_irq_flag = false;
      break;
    case 0x220c:
      sa1->io.snes_nmi_vector = (sa1->io.snes_nmi_vector & 0xff00) | data; break;
    case 0x220d:
      sa1->io.snes_nmi_vector = (sa1->io.snes_nmi_vector & 0x00ff)
                              | ((uint16_t)data << 8); break;
    case 0x220e:
      sa1->io.snes_irq_vector = (sa1->io.snes_irq_vector & 0xff00) | data; break;
    case 0x220f:
      sa1->io.snes_irq_vector = (sa1->io.snes_irq_vector & 0x00ff)
                              | ((uint16_t)data << 8); break;
    case 0x2210:
      sa1->io.timer_h_enable = (data & 1) != 0;
      sa1->io.timer_v_enable = (data & 2) != 0;
      sa1->io.timer_linear = (data & 0x80) != 0;
      break;
    case 0x2211: sa1->h_counter = sa1->v_counter = 0; break;
    case 0x2212:
      sa1->io.h_count_target = (sa1->io.h_count_target & 0xff00) | data; break;
    case 0x2213:
      sa1->io.h_count_target = (sa1->io.h_count_target & 0x00ff)
                             | ((uint16_t)data << 8); break;
    case 0x2214:
      sa1->io.v_count_target = (sa1->io.v_count_target & 0xff00) | data; break;
    case 0x2215:
      sa1->io.v_count_target = (sa1->io.v_count_target & 0x00ff)
                             | ((uint16_t)data << 8); break;
    case 0x2225:
      sa1->io.sa1_bwram_page = data & 0x7f;
      sa1->io.bwram_bitmap_window = (data & 0x80) != 0;
      break;
    case 0x2227: sa1->io.sa1_bwram_write_enable = (data & 0x80) != 0; break;
    case 0x222a: sa1->io.sa1_iram_write_mask = data; break;
    case 0x2230:
      sa1->io.dma_source = data & 3u;
      sa1->io.dma_dest_bwram = (data & 4) != 0;
      sa1->io.char_conversion_select = (data & 0x10) != 0;
      sa1->io.char_conversion_enable = (data & 0x20) != 0;
      sa1->io.dma_priority = (data & 0x40) != 0;
      sa1->io.dma_enable = (data & 0x80) != 0;
      if (!sa1->io.dma_enable) sa1->char_dma_line = 0;
      break;
    case 0x2231: case 0x2232: case 0x2233: case 0x2234:
    case 0x2235: case 0x2236: case 0x2237:
      io_write_shared(sa1, address, data); break;
    case 0x2238:
      sa1->io.dma_count = (sa1->io.dma_count & 0xff00) | data; break;
    case 0x2239:
      sa1->io.dma_count = (sa1->io.dma_count & 0x00ff)
                        | ((uint16_t)data << 8); break;
    case 0x223f: sa1->io.bitmap_2bpp = (data & 0x80) != 0; break;
    case 0x2250:
      sa1->io.arithmetic_divide = (data & 1) != 0;
      sa1->io.arithmetic_accumulate = (data & 2) != 0;
      if (sa1->io.arithmetic_accumulate) sa1->io.arithmetic_result = 0;
      break;
    case 0x2251:
      sa1->io.arithmetic_a = (sa1->io.arithmetic_a & 0xff00) | data; break;
    case 0x2252:
      sa1->io.arithmetic_a = (sa1->io.arithmetic_a & 0x00ff)
                           | ((uint16_t)data << 8); break;
    case 0x2253:
      sa1->io.arithmetic_b = (sa1->io.arithmetic_b & 0xff00) | data; break;
    case 0x2254:
      sa1->io.arithmetic_b = (sa1->io.arithmetic_b & 0x00ff)
                           | ((uint16_t)data << 8);
      arithmetic_execute(sa1);
      break;
    case 0x2258: {
      sa1->io.variable_length = data & 0x0f;
      if (!sa1->io.variable_length) sa1->io.variable_length = 16;
      sa1->io.variable_auto_increment = (data & 0x80) != 0;
      if (!sa1->io.variable_auto_increment) {
        unsigned advance = sa1->io.variable_bit + sa1->io.variable_length;
        sa1->io.variable_address =
            (sa1->io.variable_address + (advance >> 3)) & 0xffffffu;
        sa1->io.variable_bit = (uint8_t)(advance & 7u);
      }
      break;
    }
    case 0x2259:
      sa1->io.variable_address =
          (sa1->io.variable_address & 0xffff00u) | data; break;
    case 0x225a:
      sa1->io.variable_address =
          (sa1->io.variable_address & 0xff00ffu) | ((uint32_t)data << 8); break;
    case 0x225b:
      sa1->io.variable_address =
          (sa1->io.variable_address & 0x00ffffu) | ((uint32_t)data << 16);
      sa1->io.variable_bit = 0;
      break;
    default:
      if (address >= 0x2240 && address <= 0x224f) {
        unsigned index = address - 0x2240;
        sa1->io.bitmap_registers[index] = data;
        if ((index == 7 || index == 15) && sa1->io.dma_enable &&
            sa1->io.char_conversion_enable &&
            !sa1->io.char_conversion_select) dma_cc2_line(sa1);
      }
      break;
  }
}

static uint8_t bwram_read_cpu(Sa1 *sa1, uint8_t bank, uint16_t address) {
  uint32_t offset;
  if (((bank & 0x7f) < 0x40) && address >= 0x6000 && address < 0x8000)
    offset = (uint32_t)sa1->io.snes_bwram_page * 0x2000u
           + (address & 0x1fffu);
  else
    offset = ((uint32_t)(bank & 0x0f) << 16) | address;
  return sa1->bwram_char_dma ? dma_cc1_read(sa1, offset)
                             : bwram_read_raw(sa1, offset);
}

static void bwram_write_cpu(Sa1 *sa1, uint8_t bank, uint16_t address,
                            uint8_t data) {
  uint32_t offset;
  if (((bank & 0x7f) < 0x40) && address >= 0x6000 && address < 0x8000)
    offset = (uint32_t)sa1->io.snes_bwram_page * 0x2000u
           + (address & 0x1fffu);
  else
    offset = ((uint32_t)(bank & 0x0f) << 16) | address;
  if (bwram_write_allowed(sa1, offset)) bwram_write_raw(sa1, offset, data);
}

static uint8_t bwram_read_sa1_window(Sa1 *sa1, uint16_t address) {
  uint32_t offset = (uint32_t)sa1->io.sa1_bwram_page * 0x2000u
                  + (address & 0x1fffu);
  return sa1->io.bwram_bitmap_window
      ? bwram_read_bitmap(sa1, offset) : bwram_read_raw(sa1, offset);
}

static void bwram_write_sa1_window(Sa1 *sa1, uint16_t address, uint8_t data) {
  uint32_t offset = (uint32_t)sa1->io.sa1_bwram_page * 0x2000u
                  + (address & 0x1fffu);
  if (sa1->io.bwram_bitmap_window) bwram_write_bitmap(sa1, offset, data);
  else if (bwram_write_allowed(sa1, offset)) bwram_write_raw(sa1, offset, data);
}

uint8_t sa1_cpu_read(Sa1 *sa1, uint8_t bank, uint16_t address,
                     uint8_t open_bus) {
  if (!sa1) return open_bus;
  sa1->open_bus = open_bus;
  uint8_t canonical = bank & 0x7f;
  uint8_t result = open_bus;
  if (canonical < 0x40 && address >= 0x2200 && address < 0x2400)
    result = io_read_cpu(sa1, address, open_bus);
  else if (canonical < 0x40 && address >= 0x3000 && address < 0x3800)
    result = iram_read(sa1, address);
  else if ((canonical < 0x40 && address >= 0x6000 && address < 0x8000) ||
           (bank >= 0x40 && bank < 0x50))
    result = bwram_read_cpu(sa1, bank, address);
  else
    result = rom_cpu_read(sa1, bank, address, open_bus);
  sa1->open_bus = result;
  return result;
}

void sa1_cpu_write(Sa1 *sa1, uint8_t bank, uint16_t address, uint8_t data) {
  if (!sa1) return;
  sa1->open_bus = data;
  uint8_t canonical = bank & 0x7f;
  if (canonical < 0x40 && address >= 0x2200 && address < 0x2400)
    io_write_cpu(sa1, address, data);
  else if (canonical < 0x40 && address >= 0x3000 && address < 0x3800)
    iram_write(sa1, address, data, false);
  else if ((canonical < 0x40 && address >= 0x6000 && address < 0x8000) ||
           (bank >= 0x40 && bank < 0x50))
    bwram_write_cpu(sa1, bank, address, data);
}

static uint8_t sa1_bus_read(void *opaque, uint32_t address) {
  Sa1 *sa1 = (Sa1 *)opaque;
  uint8_t bank = (uint8_t)(address >> 16);
  uint16_t lo = (uint16_t)address;
  uint8_t canonical = bank & 0x7f;
  uint8_t result = sa1->open_bus;

  if (bank == 0 && sa1->interrupt_vector_kind &&
      (lo == 0xffea || lo == 0xffeb || lo == 0xffee || lo == 0xffef ||
       lo == 0xfffa || lo == 0xfffb || lo == 0xfffe || lo == 0xffff)) {
    uint16_t vector = sa1->interrupt_vector_kind == 1
                    ? sa1->io.nmi_vector : sa1->io.irq_vector;
    result = (uint8_t)(vector >> ((lo & 1u) * 8u));
  } else if (canonical < 0x40 && lo >= 0x2200 && lo < 0x2400) {
    result = io_read_sa1(sa1, lo);
  } else if ((canonical < 0x40) &&
             (lo < 0x0800 || (lo >= 0x3000 && lo < 0x3800))) {
    result = iram_read(sa1, lo);
  } else if (canonical < 0x40 && lo >= 0x6000 && lo < 0x8000) {
    sa1->extra_cycles++;
    result = bwram_read_sa1_window(sa1, lo);
  } else if (bank >= 0x40 && bank < 0x60) {
    sa1->extra_cycles++;
    result = bwram_read_raw(sa1, ((uint32_t)(bank - 0x40) << 16) | lo);
  } else if (bank >= 0x60 && bank < 0x70) {
    sa1->extra_cycles++;
    result = bwram_read_bitmap(sa1, ((uint32_t)(bank - 0x60) << 16) | lo);
  } else {
    if (cpu_bus_owns_rom(sa1)) sa1->extra_cycles++;
    result = rom_cpu_read(sa1, bank, lo, sa1->open_bus);
  }
  sa1->open_bus = result;
  return result;
}

static void sa1_bus_write(void *opaque, uint32_t address, uint8_t data) {
  Sa1 *sa1 = (Sa1 *)opaque;
  uint8_t bank = (uint8_t)(address >> 16);
  uint16_t lo = (uint16_t)address;
  uint8_t canonical = bank & 0x7f;
  sa1->open_bus = data;
  if (canonical < 0x40 && lo >= 0x2200 && lo < 0x2400)
    io_write_sa1(sa1, lo, data);
  else if ((canonical < 0x40) &&
           (lo < 0x0800 || (lo >= 0x3000 && lo < 0x3800)))
    iram_write(sa1, lo, data, true);
  else if (canonical < 0x40 && lo >= 0x6000 && lo < 0x8000) {
    sa1->extra_cycles++;
    bwram_write_sa1_window(sa1, lo, data);
  } else if (bank >= 0x40 && bank < 0x60) {
    sa1->extra_cycles++;
    uint32_t offset = ((uint32_t)(bank - 0x40) << 16) | lo;
    if (bwram_write_allowed(sa1, offset)) bwram_write_raw(sa1, offset, data);
  } else if (bank >= 0x60 && bank < 0x70) {
    sa1->extra_cycles++;
    bwram_write_bitmap(sa1, ((uint32_t)(bank - 0x60) << 16) | lo, data);
  } else if ((address & 0x408000u) == 0x008000u ||
             (address & 0xc00000u) == 0xc00000u) {
    if (cpu_bus_owns_rom(sa1)) sa1->extra_cycles++;
  }
}

static void sa1_request_interrupt(Sa1 *sa1) {
  sa1->interrupt_vector_kind = 0;
  if (sa1->io.sa1_nmi_enable && sa1->io.sa1_nmi_flag) {
    sa1->interrupt_vector_kind = 1;
    sa1->cpu->nmiWanted = true;
    return;
  }
  bool irq = (sa1->io.timer_irq_enable && sa1->io.timer_irq_flag)
          || (sa1->io.dma_irq_enable && sa1->io.dma_irq_flag)
          || (sa1->io.sa1_irq_enable && sa1->io.sa1_irq_flag);
  if (irq) {
    sa1->interrupt_vector_kind = 2;
    sa1->cpu->irqWanted = true;
  }
}

Sa1 *sa1_create(uint8_t *rom, uint32_t rom_size,
                uint8_t *bwram, uint32_t bwram_size) {
  Sa1 *sa1 = (Sa1 *)calloc(1, sizeof(*sa1));
  if (!sa1) return NULL;
  sa1->rom = rom;
  sa1->rom_size = rom_size;
  sa1->bwram = bwram;
  sa1->bwram_size = bwram_size;
  sa1->cpu = cpu_init(sa1, 1);
  if (!sa1->cpu) {
    free(sa1);
    return NULL;
  }
  
  sa1_reset(sa1);
  return sa1;
}

void sa1_destroy(Sa1 *sa1) {
  if (!sa1) return;
  cpu_free(sa1->cpu);
  free(sa1);
}

void sa1_reset(Sa1 *sa1) {
  if (!sa1) return;
  memset(&sa1->io, 0, sizeof(sa1->io));
  memset(sa1->iram, 0, sizeof(sa1->iram));
  cpu_reset(sa1->cpu);
  
  sa1->io.sa1_reset = true;
  sa1->io.bank_select[0] = 0;
  sa1->io.bank_select[1] = 1;
  sa1->io.bank_select[2] = 2;
  sa1->io.bank_select[3] = 3;
  sa1->io.bwram_protect = 0x0f;
  sa1->io.variable_length = 16;
  sa1->master_clock = 0;
  sa1->instructions = 0;
  sa1->h_counter = 0;
  sa1->v_counter = 0;
  sa1->char_dma_line = 0;
  sa1->bwram_char_dma = false;
  sa1->open_bus = 0xff;
  sa1->interrupt_vector_kind = 0;
  sa1->extra_cycles = 0;
  sa1->cpu_bus_address = 0;
}

void sa1_set_cpu_bus_address(Sa1 *sa1, uint32_t address) {
  if (sa1) sa1->cpu_bus_address = address & 0xffffffu;
}

void sa1_sync(Sa1 *sa1, uint64_t master_clock) {
  if (!sa1 || master_clock <= sa1->master_clock) return;
  while (sa1->master_clock < master_clock) {
    if (sa1->io.sa1_reset || sa1->io.sa1_ready) {
      sa1->master_clock += SA1_MASTER_CLOCKS_PER_CPU_CYCLE;
      timer_advance(sa1, SA1_MASTER_CLOCKS_PER_CPU_CYCLE);
      continue;
    }
    sa1_request_interrupt(sa1);
    sa1->extra_cycles = 0;
    int cycles = cpu_runOpcode(sa1->cpu);
    if (cycles < 1) cycles = 1;
    uint32_t clocks =
        ((uint32_t)cycles + sa1->extra_cycles) *
        SA1_MASTER_CLOCKS_PER_CPU_CYCLE;
    sa1->master_clock += clocks;
    timer_advance(sa1, clocks);
    sa1->instructions++;
    sa1->interrupt_vector_kind = 0;
  }
}

uint8_t *sa1_cpu_memory_ptr(Sa1 *sa1, uint8_t bank, uint16_t address) {
  if (!sa1) return NULL;
  uint8_t canonical = bank & 0x7f;
  if (canonical < 0x40 && address >= 0x3000 && address < 0x3800)
    return &sa1->iram[address & 0x07ff];
  if (canonical < 0x40 && address >= 0x6000 && address < 0x8000) {
    uint32_t offset = (uint32_t)sa1->io.snes_bwram_page * 0x2000u
                    + (address & 0x1fffu);
    return sa1->bwram && sa1->bwram_size
        ? &sa1->bwram[mirror_offset(offset, sa1->bwram_size)] : NULL;
  }
  if (bank >= 0x40 && bank < 0x50) {
    uint32_t offset = ((uint32_t)(bank - 0x40) << 16) | address;
    return sa1->bwram && sa1->bwram_size
        ? &sa1->bwram[mirror_offset(offset, sa1->bwram_size)] : NULL;
  }
  return rom_cpu_ptr(sa1, bank, address);
}

bool sa1_cpu_irq_pending(const Sa1 *sa1) {
  return sa1 &&
      ((sa1->io.cpu_irq_enable && sa1->io.cpu_irq_flag) ||
       (sa1->io.chdma_irq_enable && sa1->io.chdma_irq_flag));
}

uint64_t sa1_instructions_executed(const Sa1 *sa1) {
  return sa1 ? sa1->instructions : 0;
}

uint64_t sa1_master_clock(const Sa1 *sa1) {
  return sa1 ? sa1->master_clock : 0;
}

void sa1_saveload(Sa1 *sa1, SaveLoadFunc *func, void *ctx) {
  if (!sa1 || !func) return;
  cpu_saveload(sa1->cpu, func, ctx);
  func(ctx, &sa1->io, sizeof(sa1->io));
  func(ctx, sa1->iram, sizeof(sa1->iram));
  func(ctx, &sa1->master_clock, sizeof(sa1->master_clock));
  func(ctx, &sa1->instructions, sizeof(sa1->instructions));
  func(ctx, &sa1->h_counter, sizeof(sa1->h_counter));
  func(ctx, &sa1->v_counter, sizeof(sa1->v_counter));
  func(ctx, &sa1->char_dma_line, sizeof(sa1->char_dma_line));
  func(ctx, &sa1->bwram_char_dma, sizeof(sa1->bwram_char_dma));
  func(ctx, &sa1->open_bus, sizeof(sa1->open_bus));
  func(ctx, &sa1->interrupt_vector_kind, sizeof(sa1->interrupt_vector_kind));
  func(ctx, &sa1->extra_cycles, sizeof(sa1->extra_cycles));
  func(ctx, &sa1->cpu_bus_address, sizeof(sa1->cpu_bus_address));
}

static uint8_t cart_readLorom(Cart* cart, uint8_t bank, uint16_t adr);
static void cart_writeLorom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val);
static uint8_t cart_readHirom(Cart* cart, uint8_t bank, uint16_t adr);
static void cart_writeHirom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val);

Cart* cart_init(Snes* snes) {
  Cart* cart = (Cart *)malloc(sizeof(Cart));
  cart->snes = snes;
  cart->type = 0;
  cart->rom = NULL;
  cart->romSize = 0;
  cart->ramSize = 0x40000;
  cart->ram = (uint8_t *)calloc(cart->ramSize, 1);
  cart->sa1 = NULL;
  cart->masterClock = 0;
  cart->cpuBusAddress = 0;
  return cart;
}

void cart_free(Cart* cart) {
  if (cart->sa1) sa1_destroy(cart->sa1);
  if (cart->rom) free(cart->rom);
  if (cart->ram) free(cart->ram);
  free(cart);
}

void cart_reset(Cart* cart) {
  /* Preserve battery-backed SRAM across soft resets */
  cart->masterClock = 0;
  cart->cpuBusAddress = 0;
  if (cart->sa1) sa1_reset(cart->sa1);
}

void cart_saveload(Cart *cart, SaveLoadFunc *func, void *ctx) {
  if (cart->ram && cart->ramSize > 0) {
    func(ctx, cart->ram, cart->ramSize);
  }
  if (cart->sa1) {
    func(ctx, &cart->masterClock, sizeof(cart->masterClock));
    func(ctx, &cart->cpuBusAddress, sizeof(cart->cpuBusAddress));
    sa1_saveload(cart->sa1, func, ctx);
  }
}

void cart_load(Cart* cart, int type, uint8_t* rom, int romSize, int ramSize) {
  cart->type = type;
  if (cart->sa1) {
    sa1_destroy(cart->sa1);
    cart->sa1 = NULL;
  }
  if(cart->rom != NULL) free(cart->rom);
  cart->rom = (uint8_t*)malloc(romSize);
  cart->romSize = romSize;
  if (ramSize < 0) ramSize = 0;
  if (type == 4 && ramSize < 0x10000) {
    ramSize = 0x10000; /* SA-1 boards have at least 64KB BW-RAM */
  }
  if (ramSize > 0x40000) ramSize = 0x40000;
  memset(cart->ram, 0, 0x40000);
  cart->ramSize = ramSize;
  memcpy(cart->rom, rom, romSize);
  cart->masterClock = 0;
  cart->cpuBusAddress = 0;
  if (type == 4) {
    cart->sa1 = sa1_create(cart->rom, cart->romSize, cart->ram, cart->ramSize);
  }
}

void cart_sync_coprocessors(Cart* cart) {
  if (!cart || !cart->sa1) return;
  sa1_set_cpu_bus_address(cart->sa1, cart->cpuBusAddress);
  sa1_sync(cart->sa1, cart->masterClock);
}

bool cart_cpu_irq_pending(const Cart* cart) {
  if (!cart) return false;
  if (cart->type == 4 && cart->sa1) return sa1_cpu_irq_pending(cart->sa1);
  return false;
}

static uint8_t cart_readExHirom(Cart* cart, uint8_t bank, uint16_t adr) {
  if ((bank & 0x7f) < 0x40 && adr >= 0x6000 && adr < 0x8000 && cart->ramSize > 0) {
    return cart->ram[(((bank & 0x3f) << 13) | (adr & 0x1fff)) & (cart->ramSize - 1)];
  }
  if (adr >= 0x8000 || (bank & 0x7f) >= 0x40) {
    uint32_t ex_offset = (bank < 0x80) ? 0x400000u : 0u;
    uint32_t addr = ex_offset | (((uint32_t)(bank & 0x3f) << 16) | adr);
    return cart->rom[addr & (cart->romSize - 1)];
  }
  return cart->snes->openBus;
}

uint8_t cart_read(Cart* cart, uint8_t bank, uint16_t adr) {
  if (cart->type == 1)
    return cart_readLorom(cart, bank, adr);

  switch(cart->type) {
    case 0: return cart->snes->openBus;
    case 1: return cart_readLorom(cart, bank, adr);
    case 2: return cart_readHirom(cart, bank, adr);
    case 3: return cart_readExHirom(cart, bank, adr);
    case 4:
      cart_sync_coprocessors(cart);
      return sa1_cpu_read(cart->sa1, bank, adr, cart->snes->openBus);
  }
  return cart->snes->openBus;
}

void cart_write(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  switch(cart->type) {
    case 0: break;
    case 1: cart_writeLorom(cart, bank, adr, val); break;
    case 2:
    case 3: cart_writeHirom(cart, bank, adr, val); break;
    case 4:
      cart_sync_coprocessors(cart);
      sa1_cpu_write(cart->sa1, bank, adr, val);
      if (sa1_cpu_irq_pending(cart->sa1)) {
        cart->snes->cpu->irqWanted = true;
      } else if (!cart->snes->inIrq) {
        cart->snes->cpu->irqWanted = false;
      }
      break;
  }
}

static uint8_t cart_readLorom(Cart* cart, uint8_t bank, uint16_t adr) {
  if(adr >= 0x8000) {
    // adr 8000-ffff in all banks or all addresses in banks 40-7f and c0-ff
    return cart->rom[((bank << 15) | (adr & 0x7fff)) & (cart->romSize - 1)];
  }
  if(((bank >= 0x70 && bank < 0x7e) || bank >= 0xf0) && adr < 0x8000 && cart->ramSize > 0) {
    // banks 70-7e and f0-ff, adr 0000-7fff
    return cart->ram[(((bank & 0xf) << 15) | adr) & (cart->ramSize - 1)];
  }
  if(bank & 0x40) {
    // adr 8000-ffff in all banks or all addresses in banks 40-7f and c0-ff
    return cart->rom[((bank << 15) | (adr & 0x7fff)) & (cart->romSize - 1)];
  }
  return cart->snes->openBus;
}

static void cart_writeLorom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  if(((bank >= 0x70 && bank < 0x7e) || bank >= 0xf0) && adr < 0x8000 && cart->ramSize > 0) {
    // banks 70-7e and f0-ff, adr 0000-7fff
    cart->ram[(((bank & 0xf) << 15) | adr) & (cart->ramSize - 1)] = val;
  }
}

static uint8_t cart_readHirom(Cart* cart, uint8_t bank, uint16_t adr) {
  bank &= 0x7f;
  if(bank < 0x40 && adr >= 0x6000 && adr < 0x8000 && cart->ramSize > 0) {
    // banks 00-3f and 80-bf, adr 6000-7fff
    return cart->ram[(((bank & 0x3f) << 13) | (adr & 0x1fff)) & (cart->ramSize - 1)];
  }
  if(adr >= 0x8000 || bank >= 0x40) {
    // adr 8000-ffff in all banks or all addresses in banks 40-7f and c0-ff
    return cart->rom[(((bank & 0x3f) << 16) | adr) & (cart->romSize - 1)];
  }
  return cart->snes->openBus;
}

static void cart_writeHirom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  bank &= 0x7f;
  if(bank < 0x40 && adr >= 0x6000 && adr < 0x8000 && cart->ramSize > 0) {
    // banks 00-3f and 80-bf, adr 6000-7fff
    cart->ram[(((bank & 0x3f) << 13) | (adr & 0x1fff)) & (cart->ramSize - 1)] = val;
  }
}


/* === snes_other.c === */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>






typedef struct CartHeader {
  // normal header
  uint8_t headerVersion; // 1, 2, 3
  char name[22]; // $ffc0-$ffd4 (max 21 bytes + \0), $ffd4=$00: header V2
  uint8_t speed; // $ffd5.7-4 (always 2 or 3)
  uint8_t type; // $ffd5.3-0
  uint8_t coprocessor; // $ffd6.7-4
  uint8_t chips; // $ffd6.3-0
  uint32_t romSize; // $ffd7 (0x400 << x)
  uint32_t ramSize; // $ffd8 (0x400 << x)
  uint8_t region; // $ffd9 (also NTSC/PAL)
  uint8_t maker; // $ffda ($33: header V3)
  uint8_t version; // $ffdb
  uint16_t checksumComplement; // $ffdc,$ffdd
  uint16_t checksum; // $ffde,$ffdf
  // v2/v3 (v2 only exCoprocessor)
  char makerCode[3]; // $ffb0,$ffb1: (2 chars + \0)
  char gameCode[5]; // $ffb2-$ffb5: (4 chars + \0)
  uint32_t flashSize; // $ffbc (0x400 << x)
  uint32_t exRamSize; // $ffbd (0x400 << x) (used for GSU?)
  uint8_t specialVersion; // $ffbe
  uint8_t exCoprocessor; // $ffbf (if coprocessor = $f)
  // calculated stuff
  int16_t score; // score for header, to see which mapping is most likely
  bool pal; // if this is a rom for PAL regions instead of NTSC
  uint8_t cartType; // calculated type
} CartHeader;

static void readHeader(uint8_t* data, int length, int location, CartHeader* header);

bool snes_loadRom(Snes* snes, uint8_t* data, int length) {
  // if smaller than smallest possible, don't load
  if(length < 0x8000) {
    printf("Failed to load rom: rom to small (%d bytes)\n", length);
    return false;
  }
  // check headers (LoROM, HiROM, ExHiROM, unheadered and SMC-headered)
  CartHeader headers[6];
  memset(headers, 0, sizeof(headers));
  for(int i = 0; i < 6; i++) {
    headers[i].score = -50;
  }
  if(length >= 0x8000) readHeader(data, length, 0x7fc0, &headers[0]);
  if(length >= 0x8200) readHeader(data, length, 0x81c0, &headers[1]);
  if(length >= 0x10000) readHeader(data, length, 0xffc0, &headers[2]);
  if(length >= 0x10200) readHeader(data, length, 0x101c0, &headers[3]);
  if(length >= 0x410000) readHeader(data, length, 0x40ffc0, &headers[4]);
  if(length >= 0x410200) readHeader(data, length, 0x4101c0, &headers[5]);
  // see which it is
  int max = -1000;
  int used = 0;
  for(int i = 0; i < 6; i++) {
    if(headers[i].score > max) {
      max = headers[i].score;
      used = i;
    }
  }
  if(used & 1) {
    // odd-numbered ones are for headered roms
    data += 0x200; // move pointer past header
    length -= 0x200; // and subtract from size
  }
  // check if we can load it
  if(headers[used].cartType > 4) {
    printf("Failed to load rom: unsupported type (%d)\n", headers[used].cartType);
    return false;
  }
  // expand to a power of 2
  int newLength = 0x8000;
  while(true) {
    if(length <= newLength) {
      break;
    }
    newLength *= 2;
  }
  uint8_t* newData = (uint8_t * )malloc(newLength);
  memcpy(newData, data, length);
  int test = 1;
  while(length != newLength) {
    if(length & test) {
      memcpy(newData + length, newData + length - test, test);
      length += test;
    }
    test *= 2;
  }
  cart_load(
    snes->cart, headers[used].cartType,
    newData, newLength, (int)headers[used].ramSize
  );
  snes_reset(snes, true); // reset after loading
  free(newData);
  return true;
}

static void readHeader(uint8_t* data, int length, int location, CartHeader* header) {
  // read name, TODO: non-ASCII names?
  for(int i = 0; i < 21; i++) {
    uint8_t ch = data[location + i];
    if(ch >= 0x20 && ch < 0x7f) {
      header->name[i] = ch;
    } else {
      header->name[i] = '.';
    }
  }
  header->name[21] = 0;
  // read rest
  header->speed = data[location + 0x15] >> 4;
  header->type = data[location + 0x15] & 0xf;
  header->coprocessor = data[location + 0x16] >> 4;
  header->chips = data[location + 0x16] & 0xf;
  uint8_t romShift = data[location + 0x17] > 15 ? 15 : data[location + 0x17];
  uint8_t ramShift = data[location + 0x18];
  header->romSize = 0x400u << romShift;
  header->ramSize = (ramShift > 0 && ramShift <= 8) ? (0x400u << ramShift) : 0u;
  header->region = data[location + 0x19];
  header->maker = data[location + 0x1a];
  header->version = data[location + 0x1b];
  header->checksumComplement = (data[location + 0x1d] << 8) + data[location + 0x1c];
  header->checksum = (data[location + 0x1f] << 8) + data[location + 0x1e];
  // read v3 and/or v2
  header->headerVersion = 1;
  if(header->maker == 0x33) {
    header->headerVersion = 3;
    // maker code
    for(int i = 0; i < 2; i++) {
      uint8_t ch = data[location - 0x10 + i];
      if(ch >= 0x20 && ch < 0x7f) {
        header->makerCode[i] = ch;
      } else {
        header->makerCode[i] = '.';
      }
    }
    header->makerCode[2] = 0;
    // game code
    for(int i = 0; i < 4; i++) {
      uint8_t ch = data[location - 0xe + i];
      if(ch >= 0x20 && ch < 0x7f) {
        header->gameCode[i] = ch;
      } else {
        header->gameCode[i] = '.';
      }
    }
    header->gameCode[4] = 0;
    header->flashSize = (data[location - 4] <= 12) ? (0x400u << data[location - 4]) : 0u;
    header->exRamSize = (data[location - 3] > 0 && data[location - 3] <= 8) ? (0x400u << data[location - 3]) : 0u;
    header->specialVersion = data[location - 2];
    header->exCoprocessor = data[location - 1];
  } else if(data[location + 0x14] == 0) {
    header->headerVersion = 2;
    header->exCoprocessor = data[location - 1];
  }
  // get region
  header->pal = (header->region >= 0x2 && header->region <= 0xc) || header->region == 0x11;
  header->cartType = (location >= 0x400000) ? 3 : (location < 0x9000 ? 1 : 2);
  if (header->cartType == 1 && header->coprocessor == 3 && (header->chips >= 3 && header->chips <= 6)) {
    header->cartType = 4; /* SA-1 */
  }
  // get score
  int score = 0;
  bool isHeaderedLoc = (location == 0x81c0 || location == 0x101c0 || location == 0x4101c0);
  bool hasSmcRemainder = ((length & 0x3ff) == 0x200);
  score += (isHeaderedLoc == hasSmcRemainder) ? 12 : -20;
  score += (header->speed == 2 || header->speed == 3) ? 5 : -4;
  score += (header->type <= 3 || header->type == 5) ? 5 : -2;
  if ((header->cartType == 1 && (header->type & 1) == 0) ||
      (header->cartType == 4 && header->type == 3) ||
      (header->cartType == 2 && (header->type & 1) == 1) ||
      (header->cartType == 3 && header->type == 5)) {
    score += 8;
  }
  score += (header->coprocessor <= 5 || header->coprocessor >= 0xe) ? 5 : -2;
  score += (header->chips <= 6 || header->chips == 9 || header->chips == 0xa) ? 5 : -2;
  score += (header->region <= 0x14) ? 5 : -2;
  score += (header->checksum + header->checksumComplement == 0xffff) ? 8 : -6;
  uint16_t resetVector = data[location + 0x3c] | (data[location + 0x3d] << 8);
  score += (resetVector >= 0x8000) ? 8 : -20;
  // check first opcode after reset
  int opcodeLoc = location + 0x40 - 0x8000 + (resetVector & 0x7fff);
  uint8_t opcode = 0xff;
  if (opcodeLoc >= 0 && opcodeLoc < length) {
    opcode = data[opcodeLoc];
  } else {
    score -= 14;
  }
  if(opcode == 0x78 || opcode == 0x18) {
    // sei, clc (for clc:xce)
    score += 6;
  }
  if(opcode == 0x4c || opcode == 0x5c || opcode == 0x9c) {
    // jmp abs, jml abl, stz abs
    score += 3;
  }
  if(opcode == 0x00 || opcode == 0xff || opcode == 0xdb) {
    // brk, sbc alx, stp
    score -= 6;
  }
  header->score = score;
}


/* === dma.c === */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>




static const int bAdrOffsets[8][4] = {
  {0, 0, 0, 0},
  {0, 1, 0, 1},
  {0, 0, 0, 0},
  {0, 0, 1, 1},
  {0, 1, 2, 3},
  {0, 1, 0, 1},
  {0, 0, 0, 0},
  {0, 0, 1, 1}
};

static const int transferLength[8] = {
  1, 2, 2, 4, 4, 4, 2, 4
};

static void dma_transferByte(Dma* dma, uint16_t aAdr, uint8_t aBank, uint8_t bAdr, bool fromB);

Dma* dma_init(Snes* snes) {
  Dma* dma = (Dma*)malloc(sizeof(Dma));
  dma->snes = snes;
  return dma;
}

void dma_free(Dma* dma) {
  free(dma);
}

void dma_reset(Dma* dma) {
  for(int i = 0; i < 8; i++) {
    dma->channel[i].bAdr = 0xff;
    dma->channel[i].aAdr = 0xffff;
    dma->channel[i].aBank = 0xff;
    dma->channel[i].size = 0xffff;
    dma->channel[i].indBank = 0xff;
    dma->channel[i].tableAdr = 0xffff;
    dma->channel[i].repCount = 0xff;
    dma->channel[i].unusedByte = 0xff;
    dma->channel[i].dmaActive = false;
    dma->channel[i].hdmaActive = false;
    dma->channel[i].mode = 7;
    dma->channel[i].fixed = true;
    dma->channel[i].decrement = true;
    dma->channel[i].indirect = true;
    dma->channel[i].fromB = true;
    dma->channel[i].unusedBit = true;
    dma->channel[i].doTransfer = false;
    dma->channel[i].terminated = false;
    dma->channel[i].offIndex = 0;
  }
  dma->hdmaTimer = 0;
  dma->dmaTimer = 0;
  dma->dmaBusy = false;
}

void dma_saveload(Dma *dma, SaveLoadFunc *func, void *ctx) {
  func(ctx, &dma->channel, sizeof(Dma) - offsetof(Dma, channel));
}

uint8_t dma_read(Dma* dma, uint16_t adr) {
  uint8_t c = (adr & 0x70) >> 4;
  switch(adr & 0xf) {
    case 0x0: {
      uint8_t val = dma->channel[c].mode;
      val |= dma->channel[c].fixed << 3;
      val |= dma->channel[c].decrement << 4;
      val |= dma->channel[c].unusedBit << 5;
      val |= dma->channel[c].indirect << 6;
      val |= dma->channel[c].fromB << 7;
      return val;
    }
    case 0x1: {
      return dma->channel[c].bAdr;
    }
    case 0x2: {
      return dma->channel[c].aAdr & 0xff;
    }
    case 0x3: {
      return dma->channel[c].aAdr >> 8;
    }
    case 0x4: {
      return dma->channel[c].aBank;
    }
    case 0x5: {
      return dma->channel[c].size & 0xff;
    }
    case 0x6: {
      return dma->channel[c].size >> 8;
    }
    case 0x7: {
      return dma->channel[c].indBank;
    }
    case 0x8: {
      return dma->channel[c].tableAdr & 0xff;
    }
    case 0x9: {
      return dma->channel[c].tableAdr >> 8;
    }
    case 0xa: {
      return dma->channel[c].repCount;
    }
    case 0xb:
    case 0xf: {
      return dma->channel[c].unusedByte;
    }
    default: {
      return dma->snes->openBus;
    }
  }
}

void dma_write(Dma* dma, uint16_t adr, uint8_t val) {
  uint8_t c = (adr & 0x70) >> 4;
  switch(adr & 0xf) {
    case 0x0: {
      
      dma->channel[c].mode = val & 0x7;
      dma->channel[c].fixed = val & 0x8;
      dma->channel[c].decrement = val & 0x10;
      dma->channel[c].unusedBit = val & 0x20;
      dma->channel[c].indirect = val & 0x40;
      dma->channel[c].fromB = val & 0x80;
      break;
    }
    case 0x1: {
      dma->channel[c].bAdr = val;
      break;
    }
    case 0x2: {
      dma->channel[c].aAdr = (dma->channel[c].aAdr & 0xff00) | val;
      break;
    }
    case 0x3: {
      dma->channel[c].aAdr = (dma->channel[c].aAdr & 0xff) | (val << 8);
      break;
    }
    case 0x4: {
      dma->channel[c].aBank = val;
      break;
    }
    case 0x5: {
      dma->channel[c].size = (dma->channel[c].size & 0xff00) | val;
      break;
    }
    case 0x6: {
      dma->channel[c].size = (dma->channel[c].size & 0xff) | (val << 8);
      break;
    }
    case 0x7: {
      dma->channel[c].indBank = val;
      break;
    }
    case 0x8: {
      dma->channel[c].tableAdr = (dma->channel[c].tableAdr & 0xff00) | val;
      break;
    }
    case 0x9: {
      dma->channel[c].tableAdr = (dma->channel[c].tableAdr & 0xff) | (val << 8);
      break;
    }
    case 0xa: {
      dma->channel[c].repCount = val;
      break;
    }
    case 0xb:
    case 0xf: {
      dma->channel[c].unusedByte = val;
      break;
    }
    default: {
      break;
    }
  }
}

void dma_doDma(Dma* dma) {
  /*if(dma->dmaTimer > 0) {
    dma->dmaTimer -= 2;
    return;
  }*/
  // figure out first channel that is active
  int i = 0;
  for(i = 0; i < 8; i++) {
    if(dma->channel[i].dmaActive) {
      break;
    }
  }
  if(i == 8) {
    // no active channels
    dma->dmaBusy = false;
    return;
  }
  // do channel i
  dma_transferByte(
    dma, dma->channel[i].aAdr, dma->channel[i].aBank,
    dma->channel[i].bAdr + bAdrOffsets[dma->channel[i].mode][dma->channel[i].offIndex++], dma->channel[i].fromB
  );
  dma->channel[i].offIndex &= 3;
  dma->dmaTimer += 6; // 8 cycles for each byte taken, -2 for this cycle
  if(!dma->channel[i].fixed) {
    dma->channel[i].aAdr += dma->channel[i].decrement ? -1 : 1;
  }
  dma->channel[i].size--;
  if(dma->channel[i].size == 0) {
    dma->channel[i].offIndex = 0; // reset offset index
    dma->channel[i].dmaActive = false;
    dma->dmaTimer += 8; // 8 cycle overhead per channel
  }
}

void dma_initHdma(Dma* dma) {
  dma->hdmaTimer = 0;
  bool hdmaHappened = false;
  for(int i = 0; i < 8; i++) {
    if(dma->channel[i].hdmaActive) {
      hdmaHappened = true;
      // terminate any dma
      dma->channel[i].dmaActive = false;
      dma->channel[i].offIndex = 0;
      // load address, repCount, and indirect address if needed
      dma->channel[i].tableAdr = dma->channel[i].aAdr;
      dma->channel[i].repCount = snes_read(dma->snes, (dma->channel[i].aBank << 16) | dma->channel[i].tableAdr++);
      dma->hdmaTimer += 8; // 8 cycle overhead for each active channel
      if(dma->channel[i].indirect) {
        dma->channel[i].size = snes_read(dma->snes, (dma->channel[i].aBank << 16) | dma->channel[i].tableAdr++);
        dma->channel[i].size |= snes_read(dma->snes, (dma->channel[i].aBank << 16) | dma->channel[i].tableAdr++) << 8;
        dma->hdmaTimer += 16; // another 16 cycles for indirect (total 24)
      }
      dma->channel[i].doTransfer = true;
    } else {
      dma->channel[i].doTransfer = false;
    }
    dma->channel[i].terminated = false;
  }
  if(hdmaHappened) dma->hdmaTimer += 16; // 18 cycles overhead, -2 for this cycle
}

void dma_doHdma(Dma* dma) {
  dma->hdmaTimer = 0;
  bool hdmaHappened = false;
  for(int i = 0; i < 8; i++) {
    if(dma->channel[i].hdmaActive && !dma->channel[i].terminated) {
      hdmaHappened = true;
      // terminate any dma
      dma->channel[i].dmaActive = false;
      dma->channel[i].offIndex = 0;
      // do the hdma
      dma->hdmaTimer += 8; // 8 cycles overhead for each active channel
      if(dma->channel[i].doTransfer) {
        for(int j = 0; j < transferLength[dma->channel[i].mode]; j++) {
          dma->hdmaTimer += 8; // 8 cycles for each byte transferred
          if(dma->channel[i].indirect) {
            dma_transferByte(
              dma, dma->channel[i].size++, dma->channel[i].indBank,
              dma->channel[i].bAdr + bAdrOffsets[dma->channel[i].mode][j], dma->channel[i].fromB
            );
          } else {
            dma_transferByte(
              dma, dma->channel[i].tableAdr++, dma->channel[i].aBank,
              dma->channel[i].bAdr + bAdrOffsets[dma->channel[i].mode][j], dma->channel[i].fromB
            );
          }
        }
      }
      dma->channel[i].repCount--;
      dma->channel[i].doTransfer = dma->channel[i].repCount & 0x80;
      if((dma->channel[i].repCount & 0x7f) == 0) {
        dma->channel[i].repCount = snes_read(dma->snes, (dma->channel[i].aBank << 16) | dma->channel[i].tableAdr++);
        if(dma->channel[i].indirect) {
          // TODO: oddness with not fetching high byte if last active channel and reCount is 0
          dma->channel[i].size = snes_read(dma->snes, (dma->channel[i].aBank << 16) | dma->channel[i].tableAdr++);
          dma->channel[i].size |= snes_read(dma->snes, (dma->channel[i].aBank << 16) | dma->channel[i].tableAdr++) << 8;
          dma->hdmaTimer += 16; // 16 cycles for new indirect address
        }
        if(dma->channel[i].repCount == 0) dma->channel[i].terminated = true;
        dma->channel[i].doTransfer = true;
      }
    }
  }
  if(hdmaHappened) dma->hdmaTimer += 16; // 18 cycles overhead, -2 for this cycle
}

static void dma_transferByte(Dma* dma, uint16_t aAdr, uint8_t aBank, uint8_t bAdr, bool fromB) {
  // TODO: invalid writes:
  //   accesing b-bus via a-bus gives open bus,
  //   $2180-$2183 while accessing ram via a-bus open busses $2180-$2183
  //   cannot access $4300-$437f (dma regs), or $420b / $420c
  if(fromB) {
    snes_write(dma->snes, (aBank << 16) | aAdr, snes_readBBus(dma->snes, bAdr));
  } else {
    uint8_t data = snes_read(dma->snes, (aBank << 16) | aAdr);
    snes_writeBBus(dma->snes, bAdr, data);
  }
}

bool dma_cycle(Dma* dma) {
  if(dma->hdmaTimer > 0) {
    dma->hdmaTimer -= 2;
    return true;
  } else if(dma->dmaBusy) {
    dma_doDma(dma);
    return true;
  }
  return false;
}

void dma_startDma(Dma* dma, uint8_t val, bool hdma) {
  for(int i = 0; i < 8; i++) {
    if(hdma) {
      dma->channel[i].hdmaActive = val & (1 << i);
    } else {
      dma->channel[i].dmaActive = val & (1 << i);
    }
  }
  if(!hdma) {
    dma->dmaBusy = val;
    dma->dmaTimer += dma->dmaBusy ? 16 : 0; // 12-24 cycle overhead for entire dma transfer
  }
}


/* === ppu.c === */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>





bool g_new_ppu = true;
void PpuSetExtraSideSpace(Ppu *ppu, int left, int right, int bottom) {
  ppu->extraLeftRight = kPpuExtraLeftRight;
  ppu->extraLeftCur = left > kPpuExtraLeftRight ? kPpuExtraLeftRight : left;
  ppu->extraRightCur = right > kPpuExtraLeftRight ? kPpuExtraLeftRight : right;
}
void PpuDrawWholeLineOldPpu(Ppu *ppu, int line);
static void PpuDrawWholeLine(Ppu *ppu, uint y);

static bool ppu_evaluateSprites(Ppu* ppu, int line);
static uint16_t ppu_getVramRemap(Ppu* ppu);


Ppu* ppu_init(void) {
  Ppu* ppu = malloc(sizeof(Ppu));
  return ppu;
}

void ppu_free(Ppu* ppu) {
  free(ppu);
}

void ppu_copy(Ppu *ppu, Ppu *ppu_src) {
  size_t pitch = ppu->renderPitch;
  uint8_t *renderBuffer = ppu->renderBuffer;
  memcpy(ppu, ppu_src, sizeof(*ppu));
  ppu->renderBuffer = renderBuffer;
  ppu->renderPitch = (uint32_t)pitch;
}

void ppu_reset(Ppu* ppu) {
  uint8_t el = ppu->extraLeftCur, er = ppu->extraRightCur, elr = ppu->extraLeftRight ? ppu->extraLeftRight : kPpuExtraLeftRight;
  {
    size_t pitch = ppu->renderPitch;
    uint8_t *renderBuffer = ppu->renderBuffer;
    memset(ppu, 0, sizeof(*ppu));
    ppu->renderBuffer = renderBuffer;
    ppu->renderPitch = (uint32_t)pitch;
  }
  ppu->vramIncrement = 1;
  ppu->extraLeftCur = el;
  ppu->extraRightCur = er;
  ppu->extraLeftRight = elr;
}

void ppu_saveload(Ppu *ppu, SaveLoadFunc *func, void *ctx) {
  func(ctx, &ppu->inidisp, PPU_SAVESTATE_REGS_SIZE);
  func(ctx, &ppu->cgram, PPU_SAVESTATE_MEM_SIZE);
}
void ppu_saveload_unused(Ppu *ppu, SaveLoadFunc *sli) {
  assert(offsetof(Ppu, cgwsel) + 1 - offsetof(Ppu, inidisp) == PPU_SAVESTATE_REGS_SIZE);
  assert(offsetof(Ppu, vram) + 0x10000 - offsetof(Ppu, cgram) == PPU_SAVESTATE_MEM_SIZE);
  uint32 version[2] = {'P' | 'P' << 8 | 'U' << 16 | '0' << 24, PPU_SAVESTATE_REGS_SIZE + PPU_SAVESTATE_MEM_SIZE};
  
  
  
}

void PpuBeginDrawing(Ppu *ppu, uint8_t *pixels, size_t pitch, uint32_t render_flags) {
  ppu->renderPitch = (uint)pitch;
  ppu->renderBuffer = pixels;
}

bool ppu_checkOverscan(Ppu* ppu) {
  // called at (0,225)
  ppu->frameOverscan = PPU_overscan(ppu); // set if we have a overscan-frame
  return ppu->frameOverscan;
}

void ppu_handleVblank(Ppu* ppu) {
  // called either right after ppu_checkOverscan at (0,225), or at (0,240)
  if(!PPU_forcedBlank(ppu)) {
    ppu->oamAdr = ppu->oamaddl;
    ppu->oamInHigh = ppu->oamaddh & 1;
    ppu->oamSecondWrite = false;
  }
  ppu->frameInterlace = PPU_interlace(ppu); // set if we have a interlaced frame
}

static inline void ClearBackdrop(PpuPixelPrioBufs *buf) {
  for (size_t i = 0; i != arraysize(buf->data); i += 4)
    *(uint64*)&buf->data[i] = 0x0500050005000500;
}

void ppu_runLine(Ppu* ppu, int line) {
  if(line == 0) {
    if (PPU_mosaicSize(ppu) != ppu->lastMosaicModulo) {
      int mod = PPU_mosaicSize(ppu);
      ppu->lastMosaicModulo = mod;
      for (int i = 0, j = 0; i < countof(ppu->mosaicModulo); i++) {
        ppu->mosaicModulo[i] = i - j;
        j = (j + 1 == mod ? 0 : j + 1);
      }
    }


    // pre-render line
    // TODO: this now happens halfway into the first line
    ppu->mosaicStartLine = 1;
    ppu->rangeOver = false;
    ppu->timeOver = false;
    ppu->evenFrame = !ppu->evenFrame;
  } else {  
    // Cache the brightness computation
    if (PPU_brightness(ppu) != ppu->lastBrightnessMult) {
      uint8_t ppu_brightness = PPU_brightness(ppu);
      ppu->lastBrightnessMult = ppu_brightness;
      for (int i = 0; i < 32; i++)
        ppu->brightnessMultHalf[i * 2] = ppu->brightnessMultHalf[i * 2 + 1] = ppu->brightnessMult[i] =
        ((i << 3) | (i >> 2)) * ppu_brightness / 15;
      // Store 31 extra entries to remove the need for clamping to 31.
      memset(&ppu->brightnessMult[32], ppu->brightnessMult[31], 31);
    }

    // evaluate sprites
    ClearBackdrop(&ppu->objBuffer);
    ppu->lineHasSprites = !PPU_forcedBlank(ppu) && ppu_evaluateSprites(ppu, line - 1);
    if (line == 120 && !PPU_forcedBlank(ppu)) {
      ppu->was_visible = true;
    }

    if (g_new_ppu && (PPU_mode(ppu) == 1 || PPU_mode(ppu) == 7) && (ppu->bgmode & 0xf0) == 0 && !PPU_directColor(ppu) && !PPU_pseudoHires(ppu)) {
      PpuDrawWholeLine(ppu, line);
    } else {
      PpuDrawWholeLineOldPpu(ppu, line);
    }
  }
}

typedef struct PpuWindows {
  int16 edges[6];
  uint8 nr;
  uint8 bits;
} PpuWindows;

static void PpuWindows_Clear(PpuWindows *win, Ppu *ppu, uint layer) {
  (void)layer;
  win->edges[0] = -ppu->extraLeftCur;
  win->edges[1] = 256 + ppu->extraRightCur;
  win->nr = 1;
  win->bits = 0;
}

static void PpuWindows_Calc(PpuWindows *win, Ppu *ppu, uint layer) {
  // Evaluate which spans to render based on the window settings.
  // There are at most 5 windows.
  // Algorithm from Snes9x
  uint32 winflags = GET_WINDOW_FLAGS(ppu, layer);
  uint nr = 1;
  int window_right = 256 + ppu->extraRightCur;
  win->edges[0] = -ppu->extraLeftCur;
  win->edges[1] = window_right;
  uint i, j;
  int t;
  int w1_left = (ppu->window1left <= 8 && ppu->window1right >= 247) ? win->edges[0] : ((ppu->window1left == 0) ? win->edges[0] : ppu->window1left);
  int w1_right = (ppu->window1left <= 8 && ppu->window1right >= 247) ? (window_right - 1) : ((ppu->window1right >= 254) ? (window_right - 1) : ppu->window1right);
  bool w1_ena = (winflags & kWindow1Enabled) && w1_left <= w1_right;
  if (w1_ena) {
    if (w1_left > win->edges[0]) {
      win->edges[nr] = w1_left;
      win->edges[++nr] = window_right;
    }
    if (w1_right + 1 < window_right) {
      win->edges[nr] = w1_right + 1;
      win->edges[++nr] = window_right;
    }
  }
  int w2_left = (ppu->window2left <= 8 && ppu->window2right >= 247) ? win->edges[0] : ((ppu->window2left == 0) ? win->edges[0] : ppu->window2left);
  int w2_right = (ppu->window2left <= 8 && ppu->window2right >= 247) ? (window_right - 1) : ((ppu->window2right >= 254) ? (window_right - 1) : ppu->window2right);
  bool w2_ena = (winflags & kWindow2Enabled) && w2_left <= w2_right;
  if (w2_ena) {
    for (i = 0; i <= nr && (t = w2_left) != win->edges[i]; i++) {
      if (t < win->edges[i]) {
        for (j = nr++; j >= i; j--)
          win->edges[j + 1] = win->edges[j];
        win->edges[i] = t;
        break;
      }
    }
    for (; i <= nr && (t = w2_right + 1) != win->edges[i]; i++) {
      if (t < win->edges[i]) {
        for (j = nr++; j >= i; j--)
          win->edges[j + 1] = win->edges[j];
        win->edges[i] = t;
        break;
      }
    }
  }
  win->nr = nr;
  // get a bitmap of how regions map to windows
  uint8 w1_bits = 0, w2_bits = 0;
  if (w1_ena) {
    for (i = 0; win->edges[i] != w1_left; i++);
    for (j = i; win->edges[j] != w1_right + 1; j++);
    w1_bits = ((1 << (j - i)) - 1) << i;
  }
  if ((winflags & (kWindow1Enabled | kWindow1Inversed)) == (kWindow1Enabled | kWindow1Inversed))
    w1_bits = ~w1_bits;
  if (w2_ena) {
    for (i = 0; win->edges[i] != w2_left; i++);
    for (j = i; win->edges[j] != w2_right + 1; j++);
    w2_bits = ((1 << (j - i)) - 1) << i;
  }
  if ((winflags & (kWindow2Enabled | kWindow2Inversed)) == (kWindow2Enabled | kWindow2Inversed))
    w2_bits = ~w2_bits;
  win->bits = w1_bits | w2_bits;
}

static inline uint32 ReadTile4bppWidescreen(const Ppu *ppu, uint layer, const uint16 * const tps[2], const uint16 *ptr) {
  uint8_t tag = ppu->vram_bgmode[ptr - ppu->vram];
  bool valid_mode = ((tag & 7) == ((PPU_mode(ppu) + 1) & 7));
  uint16 val = valid_mode ? *ptr : 0;
  if (tps[0] != tps[1]) {
    bool is_col_streamed = (ppu->col_streamed_mask & (1u << ((PPU_bgTilemapAdr(ppu, layer) >> 11) & 0xf))) != 0;
    if (is_col_streamed) {
      int global_col = (ptr >= tps[1] && ptr < tps[1] + 32) ? (32 + (int)(ptr - tps[1])) : (int)(ptr - tps[0]);
      int cam_col = ((int)ppu->hScroll[layer] >> 3) & 0x3f;
      int rel_col = (global_col - cam_col) & 0x3f;
      if (rel_col >= 48) rel_col -= 64; /* [-16 .. +47] relative to camera left */
      if ((rel_col < -2 || rel_col > 34) && !(tag & 0x80)) {
        int clamp_col = (rel_col < -2) ? ((cam_col - 2) & 0x3f) : ((cam_col + 34) & 0x3f);
        return tps[(clamp_col >> 5) & 1][clamp_col & 0x1f];
      }
      return val;
    }
  }
  if (tps[0] != tps[1] && ptr >= tps[1] && ptr < tps[1] + 32) {
    int col = (int)(ptr - tps[1]);
    bool unstreamed = !valid_mode ||
                      (val == 0 && ppu->hScroll[layer] < 128) ||
                      (val == tps[1][31] && val != tps[0][30] && val != tps[0][31]);
    if (unstreamed) {
      if (tps[0][30] == tps[0][28] && tps[0][31] == tps[0][29]) {
        return tps[0][30 + (col & 1)];
      }
      if (tps[0][28] == tps[0][24] && tps[0][29] == tps[0][25] &&
          tps[0][30] == tps[0][26] && tps[0][31] == tps[0][27]) {
        return tps[0][28 + (col & 3)];
      }
      if (tps[0][0] == tps[0][2] && tps[0][1] == tps[0][3]) {
        return tps[0][col & 1];
      }
      return 0;
    }
  }
  return val;
}

// Draw a whole line of a 4bpp background layer into bgBuffers
static void PpuDrawBackground_4bpp(Ppu *ppu, uint y, bool sub, uint layer, PpuZbufType zhi, PpuZbufType zlo) {
#define DO_PIXEL(i) do { \
  pixel = (bits >> i) & 1 | (bits >> (7 + i)) & 2 | (bits >> (14 + i)) & 4 | (bits >> (21 + i)) & 8; \
  if ((bits & (0x01010101 << i)) && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define DO_PIXEL_HFLIP(i) do { \
  pixel = (bits >> (7 - i)) & 1 | (bits >> (14 - i)) & 2 | (bits >> (21 - i)) & 4 | (bits >> (28 - i)) & 8; \
  if ((bits & (0x80808080 >> i)) && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define READ_TILE_4BPP(ptr) ReadTile4bppWidescreen(ppu, layer, tps, (ptr))
#define READ_BITS(ta, tile) (addr = &ppu->vram[((ta) + (tile) * 16) & 0x7fff], addr[0] | addr[8] << 16)
  enum { kPaletteShift = 6 };
  if (!IS_SCREEN_ENABLED(ppu, sub, layer))
    return;  // layer is completely hidden
  PpuWindows win;
  IS_SCREEN_WINDOWED(ppu, sub, layer) ? PpuWindows_Calc(&win, ppu, layer) : PpuWindows_Clear(&win, ppu, layer);
  y += ppu->vScroll[layer];
  int sc_offs = PPU_bgTilemapAdr(ppu, layer) + (((y >> 3) & 0x1f) << 5);
  if ((y & 0x100) && PPU_bgTilemapHigher(ppu, layer))
    sc_offs += PPU_bgTilemapWider(ppu, layer) ? 0x800 : 0x400;
  const uint16 *tps[2] = {
    &ppu->vram[sc_offs & 0x7fff],
    &ppu->vram[sc_offs + (PPU_bgTilemapWider(ppu, layer) ? 0x400 : 0) & 0x7fff]
  };
  int tileadr = PPU_bgTileAdr(ppu, layer), pixel;
  int tileadr1 = tileadr + 7 - (y & 0x7), tileadr0 = tileadr + (y & 0x7);
  bool clamp_4bpp_margins = (!PPU_bgTilemapWider(ppu, layer) && ppu->hScroll[layer] == 0 &&
                             !(tps[0][0] == tps[0][4] && tps[0][1] == tps[0][5] &&
                               tps[0][0] == tps[0][16] && tps[0][1] == tps[0][17] &&
                               tps[0][0] != 0));
  const uint16 *addr;
  for (size_t windex = 0; windex < win.nr; windex++) {
    if (win.bits & (1 << windex))
      continue;  // layer is disabled for this window part
    int sx_start = win.edges[windex];
    int sx_end = win.edges[windex + 1];
    if (clamp_4bpp_margins) {
      bool sym_edge = (tps[0][0] == tps[0][31]) || (tps[0][0] == tps[0][1] && tps[0][30] == tps[0][31]);
      if (sx_start < 0) {
        int left_end = IntMin(0, sx_end);
        uint32 ltile = sym_edge ? tps[0][0] : 0;
        int lta = (ltile & 0x8000) ? tileadr1 : tileadr0;
        PpuZbufType lz = (ltile & 0x2000) ? zhi : zlo;
        uint32 lbits = ltile ? READ_BITS(lta, ltile & 0x3ff) : 0;
        if (lbits) {
          lz += ((ltile & 0x1c00) >> kPaletteShift);
          PpuZbufType *ldst = ppu->bgBuffers[sub].data + sx_start + kPpuExtraLeftRight;
          for (int sx = sx_start; sx < left_end; sx++, ldst++) {
            int i = (ltile & 0x4000) ? (sx & 7) : (7 - (sx & 7));
            int lpix = ((lbits >> i) & 1) | ((lbits >> (7 + i)) & 2) | ((lbits >> (14 + i)) & 4) | ((lbits >> (21 + i)) & 8);
            if (lpix && lz > *ldst) *ldst = lz + lpix;
          }
        }
        sx_start = left_end;
      }
      if (sx_end > 256) {
        int right_start = IntMax(256, sx_start);
        uint32 rtile = sym_edge ? tps[0][31] : 0;
        int rta = (rtile & 0x8000) ? tileadr1 : tileadr0;
        PpuZbufType rz = (rtile & 0x2000) ? zhi : zlo;
        uint32 rbits = rtile ? READ_BITS(rta, rtile & 0x3ff) : 0;
        if (rbits) {
          rz += ((rtile & 0x1c00) >> kPaletteShift);
          PpuZbufType *rdst = ppu->bgBuffers[sub].data + right_start + kPpuExtraLeftRight;
          for (int sx = right_start; sx < sx_end; sx++, rdst++) {
            int i = (rtile & 0x4000) ? (sx & 7) : (7 - (sx & 7));
            int rpix = ((rbits >> i) & 1) | ((rbits >> (7 + i)) & 2) | ((rbits >> (14 + i)) & 4) | ((rbits >> (21 + i)) & 8);
            if (rpix && rz > *rdst) *rdst = rz + rpix;
          }
        }
        sx_end = right_start;
      }
      if (sx_start >= sx_end) continue;
    }
    bool left_streamed = (tps[0] != tps[1]) &&
                         ((ppu->col_streamed_mask & (1u << ((PPU_bgTilemapAdr(ppu, layer) >> 11) & 0xf))) != 0);
    if (!left_streamed && sx_start + (int)ppu->hScroll[layer] < 0 && ppu->hScroll[layer] < 128) {
      // Fill left pre-level margin by repeating the leftmost 16px or 32px of nametable 0
      bool rep2 = (tps[0][0] == tps[0][2] && tps[0][1] == tps[0][3]);
      bool rep4 = !rep2 && (tps[0][0] == tps[0][4] && tps[0][1] == tps[0][5] &&
                            tps[0][2] == tps[0][6] && tps[0][3] == tps[0][7]);
      while (sx_start < sx_end && sx_start + (int)ppu->hScroll[layer] < 0) {
        int step = IntMin(32, sx_end - sx_start);
        if (sx_start + step + (int)ppu->hScroll[layer] > 0)
          step = -(sx_start + (int)ppu->hScroll[layer]);
        uint mx = (uint)((sx_start + (int)ppu->hScroll[layer]) & 0x1f);
        PpuZbufType *mdst = ppu->bgBuffers[sub].data + sx_start + kPpuExtraLeftRight;
        for (int mi = 0; mi < step; mi++, mx = (mx + 1) & 0x1f, mdst++) {
          uint32 mtile = rep2 ? tps[0][(mx >> 3) & 1] : (rep4 ? tps[0][(mx >> 3) & 3] : tps[0][0]);
          int mta = (mtile & 0x8000) ? tileadr1 : tileadr0;
          PpuZbufType mz = (mtile & 0x2000) ? zhi : zlo;
          uint32 mbits = READ_BITS(mta, mtile & 0x3ff);
          if (mbits) {
            mz += ((mtile & 0x1c00) >> kPaletteShift);
            int msh = (mtile & 0x4000) ? (mx & 7) : (7 - (mx & 7));
            int mpix = ((mbits >> msh) & 1) | ((mbits >> (7 + msh)) & 2) | ((mbits >> (14 + msh)) & 4) | ((mbits >> (21 + msh)) & 8);
            if (mpix && mz > *mdst) *mdst = mz + mpix;
          }
        }
        sx_start += step;
      }
      if (sx_start >= sx_end) continue;
    }
    uint x = sx_start + ppu->hScroll[layer];
    uint w = sx_end - sx_start;
    PpuZbufType *dstz = ppu->bgBuffers[sub].data + sx_start + kPpuExtraLeftRight;
    const uint16 *tp = tps[x >> 8 & 1] + ((x >> 3) & 0x1f);
    const uint16 *tp_last = tps[x >> 8 & 1] + 31;
    const uint16 *tp_next = tps[(x >> 8 & 1) ^ 1];
#define NEXT_TP() if (tp != tp_last) tp += 1; else tp = tp_next, tp_next = tp_last - 31, tp_last = tp + 31;
    // Handle clipped pixels on left side
    if (x & 7) {
      int curw = IntMin(8 - (x & 7), w);
      w -= curw;
      uint32 tile = READ_TILE_4BPP(tp);
      NEXT_TP();
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      uint32 bits = READ_BITS(ta, tile & 0x3ff);
      if (bits) {
        z += ((tile & 0x1c00) >> kPaletteShift);
        if (tile & 0x4000) {
          bits >>= (x & 7), x += curw;
          do DO_PIXEL(0); while (bits >>= 1, dstz++, --curw);
        } else {
          bits <<= (x & 7), x += curw;
          do DO_PIXEL_HFLIP(0); while (bits <<= 1, dstz++, --curw);
        }
      } else {
        dstz += curw;
      }
    }
    // Handle full tiles in the middle
    while (w >= 8) {
      uint32 tile = READ_TILE_4BPP(tp);
      NEXT_TP();
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      uint32 bits = READ_BITS(ta, tile & 0x3ff);
      if (bits) {
        z += ((tile & 0x1c00) >> kPaletteShift);
        if (tile & 0x4000) {
          DO_PIXEL(0); DO_PIXEL(1); DO_PIXEL(2); DO_PIXEL(3);
          DO_PIXEL(4); DO_PIXEL(5); DO_PIXEL(6); DO_PIXEL(7);
        } else {
          DO_PIXEL_HFLIP(0); DO_PIXEL_HFLIP(1); DO_PIXEL_HFLIP(2); DO_PIXEL_HFLIP(3);
          DO_PIXEL_HFLIP(4); DO_PIXEL_HFLIP(5); DO_PIXEL_HFLIP(6); DO_PIXEL_HFLIP(7);
        }
      }
      dstz += 8, w -= 8;
    }
    // Handle remaining clipped part
    if (w) {
      uint32 tile = READ_TILE_4BPP(tp);
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      uint32 bits = READ_BITS(ta, tile & 0x3ff);
      if (bits) {
        z += ((tile & 0x1c00) >> kPaletteShift);
        if (tile & 0x4000) {
          do DO_PIXEL(0); while (bits >>= 1, dstz++, --w);
        } else {
          do DO_PIXEL_HFLIP(0); while (bits <<= 1, dstz++, --w);
        }
      }
    }
  }
#undef READ_BITS
#undef DO_PIXEL
#undef DO_PIXEL_HFLIP
}

// Draw a whole line of a 2bpp background layer into bgBuffers
static void PpuDrawBackground_2bpp(Ppu *ppu, uint y, bool sub, uint layer, PpuZbufType zhi, PpuZbufType zlo) {
#define DO_PIXEL(i) do { \
  pixel = (bits >> i) & 1 | (bits >> (7 + i)) & 2; \
  if (pixel && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define DO_PIXEL_HFLIP(i) do { \
  pixel = (bits >> (7 - i)) & 1 | (bits >> (14 - i)) & 2; \
  if (pixel && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define READ_BITS(ta, tile) (addr = &ppu->vram[(ta) + (tile) * 8 & 0x7fff], addr[0])
  enum { kPaletteShift = 8 };
  if (!IS_SCREEN_ENABLED(ppu, sub, layer))
    return;  // layer is completely hidden
  PpuWindows win;
  IS_SCREEN_WINDOWED(ppu, sub, layer) ? PpuWindows_Calc(&win, ppu, layer) : PpuWindows_Clear(&win, ppu, layer);
  y += ppu->vScroll[layer];
  int sc_offs = PPU_bgTilemapAdr(ppu, layer) + (((y >> 3) & 0x1f) << 5);
  if ((y & 0x100) && PPU_bgTilemapHigher(ppu, layer))
    sc_offs += PPU_bgTilemapWider(ppu, layer) ? 0x800 : 0x400;
  const uint16 *tps[2] = {
    &ppu->vram[sc_offs & 0x7fff],
    &ppu->vram[sc_offs + (PPU_bgTilemapWider(ppu, layer) ? 0x400 : 0) & 0x7fff]
  };
  int tileadr = PPU_bgTileAdr(ppu, layer), pixel;
  int tileadr1 = tileadr + 7 - (y & 0x7), tileadr0 = tileadr + (y & 0x7);
  bool clamp_hud_margins = (ppu->hScroll[layer] == 0 &&
                            !(tps[0][0] == tps[0][4] && tps[0][1] == tps[0][5] &&
                              tps[0][0] == tps[0][16] && tps[0][1] == tps[0][17] &&
                              tps[0][0] != 0));

  const uint16 *addr;
  for (size_t windex = 0; windex < win.nr; windex++) {
    if (win.bits & (1 << windex))
      continue;  // layer is disabled for this window part
    int sx_start = win.edges[windex];
    int sx_end = win.edges[windex + 1];
    if (clamp_hud_margins) {
      if (sx_start < 0) {
        int left_end = IntMin(0, sx_end);
        uint32 ltile = tps[0][0];
        int lta = (ltile & 0x8000) ? tileadr1 : tileadr0;
        PpuZbufType lz = (ltile & 0x2000) ? zhi : zlo;
        uint32 lbits = READ_BITS(lta, ltile & 0x3ff);
        if (lbits) {
          lz += ((ltile & 0x1c00) >> kPaletteShift);
          PpuZbufType *ldst = ppu->bgBuffers[sub].data + sx_start + kPpuExtraLeftRight;
          for (int sx = sx_start; sx < left_end; sx++, ldst++) {
            int i = (ltile & 0x4000) ? (sx & 7) : (7 - (sx & 7));
            int lpix = ((lbits >> i) & 1) | ((lbits >> (7 + i)) & 2);
            if (lpix && lz > *ldst) *ldst = lz + lpix;
          }
        }
        sx_start = left_end;
      }
      if (sx_end > 256) {
        int right_start = IntMax(256, sx_start);
        uint32 rtile = tps[0][31];
        int rta = (rtile & 0x8000) ? tileadr1 : tileadr0;
        PpuZbufType rz = (rtile & 0x2000) ? zhi : zlo;
        uint32 rbits = READ_BITS(rta, rtile & 0x3ff);
        if (rbits) {
          rz += ((rtile & 0x1c00) >> kPaletteShift);
          PpuZbufType *rdst = ppu->bgBuffers[sub].data + right_start + kPpuExtraLeftRight;
          for (int sx = right_start; sx < sx_end; sx++, rdst++) {
            int i = (rtile & 0x4000) ? (sx & 7) : (7 - (sx & 7));
            int rpix = ((rbits >> i) & 1) | ((rbits >> (7 + i)) & 2);
            if (rpix && rz > *rdst) *rdst = rz + rpix;
          }
        }
        sx_end = right_start;
      }
      if (sx_start >= sx_end) continue;
    }
    uint x = sx_start + ppu->hScroll[layer];
    uint w = sx_end - sx_start;
    PpuZbufType *dstz = ppu->bgBuffers[sub].data + sx_start + kPpuExtraLeftRight;
    const uint16 *tp = tps[x >> 8 & 1] + ((x >> 3) & 0x1f);
    const uint16 *tp_last = tps[x >> 8 & 1] + 31;
    const uint16 *tp_next = tps[(x >> 8 & 1) ^ 1];

#define NEXT_TP() if (tp != tp_last) tp += 1; else tp = tp_next, tp_next = tp_last - 31, tp_last = tp + 31;
    // Handle clipped pixels on left side
    if (x & 7) {
      int curw = IntMin(8 - (x & 7), w);
      w -= curw;
      uint32 tile = *tp;
      NEXT_TP();
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      uint32 bits = READ_BITS(ta, tile & 0x3ff);
      if (bits) {
        z += ((tile & 0x1c00) >> kPaletteShift);
        if (tile & 0x4000) {
          bits >>= (x & 7), x += curw;
          do DO_PIXEL(0); while (bits >>= 1, dstz++, --curw);
        } else {
          bits <<= (x & 7), x += curw;
          do DO_PIXEL_HFLIP(0); while (bits <<= 1, dstz++, --curw);
        }
      } else {
        dstz += curw;
      }
    }
    // Handle full tiles in the middle
    while (w >= 8) {
      uint32 tile = *tp;
      NEXT_TP();
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      uint32 bits = READ_BITS(ta, tile & 0x3ff);
      if (bits) {
        z += ((tile & 0x1c00) >> kPaletteShift);
        if (tile & 0x4000) {
          DO_PIXEL(0); DO_PIXEL(1); DO_PIXEL(2); DO_PIXEL(3);
          DO_PIXEL(4); DO_PIXEL(5); DO_PIXEL(6); DO_PIXEL(7);
        } else {
          DO_PIXEL_HFLIP(0); DO_PIXEL_HFLIP(1); DO_PIXEL_HFLIP(2); DO_PIXEL_HFLIP(3);
          DO_PIXEL_HFLIP(4); DO_PIXEL_HFLIP(5); DO_PIXEL_HFLIP(6); DO_PIXEL_HFLIP(7);
        }
      }
      dstz += 8, w -= 8;
    }
    // Handle remaining clipped part
    if (w) {
      uint32 tile = *tp;
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      uint32 bits = READ_BITS(ta, tile & 0x3ff);
      if (bits) {
        z += ((tile & 0x1c00) >> kPaletteShift);
        if (tile & 0x4000) {
          do DO_PIXEL(0); while (bits >>= 1, dstz++, --w);
        } else {
          do DO_PIXEL_HFLIP(0); while (bits <<= 1, dstz++, --w);
        }
      }
    }
  }
#undef NEXT_TP
#undef READ_BITS
#undef DO_PIXEL
#undef DO_PIXEL_HFLIP
}


// Draw a whole line of a 4bpp background layer into bgBuffers, with mosaic applied
static void PpuDrawBackground_4bpp_mosaic(Ppu *ppu, uint y, bool sub, uint layer, PpuZbufType zhi, PpuZbufType zlo) {
#define GET_PIXEL() pixel = (bits) & 1 | (bits >> 7) & 2 | (bits >> 14) & 4 | (bits >> 21) & 8
#define GET_PIXEL_HFLIP() pixel = (bits >> 7) & 1 | (bits >> 14) & 2 | (bits >> 21) & 4 | (bits >> 28) & 8
#define READ_BITS(ta, tile) (addr = &ppu->vram[((ta) + (tile) * 16) & 0x7fff], addr[0] | addr[8] << 16)
  enum { kPaletteShift = 6 };
  if (!IS_SCREEN_ENABLED(ppu, sub, layer))
    return;  // layer is completely hidden
  PpuWindows win;
  IS_SCREEN_WINDOWED(ppu, sub, layer) ? PpuWindows_Calc(&win, ppu, layer) : PpuWindows_Clear(&win, ppu, layer);
  y = ppu->mosaicModulo[y] + ppu->vScroll[layer];
  int sc_offs = PPU_bgTilemapAdr(ppu, layer) + (((y >> 3) & 0x1f) << 5);
  if ((y & 0x100) && PPU_bgTilemapHigher(ppu, layer))
    sc_offs += PPU_bgTilemapWider(ppu, layer) ? 0x800 : 0x400;
  const uint16 *tps[2] = {
    &ppu->vram[sc_offs & 0x7fff],
    &ppu->vram[sc_offs + (PPU_bgTilemapWider(ppu, layer) ? 0x400 : 0) & 0x7fff]
  };
  int tileadr = PPU_bgTileAdr(ppu, layer), pixel;
  int tileadr1 = tileadr + 7 - (y & 0x7), tileadr0 = tileadr + (y & 0x7);
  const uint16 *addr;
  for (size_t windex = 0; windex < win.nr; windex++) {
    if (win.bits & (1 << windex))
      continue;  // layer is disabled for this window part
    int sx = win.edges[windex];
    PpuZbufType *dstz = ppu->bgBuffers[sub].data + sx + kPpuExtraLeftRight;
    PpuZbufType *dstz_end = ppu->bgBuffers[sub].data + win.edges[windex + 1] + kPpuExtraLeftRight;
    uint x = sx + ppu->hScroll[layer];
    const uint16 *tp = tps[x >> 8 & 1] + ((x >> 3) & 0x1f);
    const uint16 *tp_last = tps[x >> 8 & 1] + 31, *tp_next = tps[(x >> 8 & 1) ^ 1];
    x &= 7;
    int mosaic_size = PPU_mosaicSize(ppu);
    int w = mosaic_size - (sx - ppu->mosaicModulo[sx]);
    do {
      w = IntMin(w, (int)(dstz_end - dstz));
      uint32 tile = *tp;
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      uint32 bits = READ_BITS(ta, tile & 0x3ff);
      if (tile & 0x4000) bits >>= x, GET_PIXEL(); else bits <<= x, GET_PIXEL_HFLIP();
      if (pixel) {
        pixel += (tile & 0x1c00) >> kPaletteShift;
        int i = 0;
        do {
          if (z > dstz[i])
            dstz[i] = pixel + z;
        } while (++i != w);
      }
      dstz += w, x += w;
      for (; x >= 8; x -= 8)
        tp = (tp != tp_last) ? tp + 1 : tp_next;
      w = mosaic_size;
    } while (dstz_end - dstz != 0);
  }
#undef READ_BITS
#undef GET_PIXEL
#undef GET_PIXEL_HFLIP
}

// Draw a whole line of a 2bpp background layer into bgBuffers, with mosaic applied
static void PpuDrawBackground_2bpp_mosaic(Ppu *ppu, int y, bool sub, uint layer, PpuZbufType zhi, PpuZbufType zlo) {
#define GET_PIXEL() pixel = (bits) & 1 | (bits >> 7) & 2
#define GET_PIXEL_HFLIP() pixel = (bits >> 7) & 1 | (bits >> 14) & 2
#define READ_BITS(ta, tile) (addr = &ppu->vram[((ta) + (tile) * 8) & 0x7fff], addr[0])
  enum { kPaletteShift = 8 };
  if (!IS_SCREEN_ENABLED(ppu, sub, layer))
    return;  // layer is completely hidden
  PpuWindows win;
  IS_SCREEN_WINDOWED(ppu, sub, layer) ? PpuWindows_Calc(&win, ppu, layer) : PpuWindows_Clear(&win, ppu, layer);
  y = ppu->mosaicModulo[y] + ppu->vScroll[layer];
  int sc_offs = PPU_bgTilemapAdr(ppu, layer) + (((y >> 3) & 0x1f) << 5);
  if ((y & 0x100) && PPU_bgTilemapHigher(ppu, layer))
    sc_offs += PPU_bgTilemapWider(ppu, layer) ? 0x800 : 0x400;
  const uint16 *tps[2] = {
    &ppu->vram[sc_offs & 0x7fff],
    &ppu->vram[sc_offs + (PPU_bgTilemapWider(ppu, layer) ? 0x400 : 0) & 0x7fff]
  };
  int tileadr = PPU_bgTileAdr(ppu, layer), pixel;
  int tileadr1 = tileadr + 7 - (y & 0x7), tileadr0 = tileadr + (y & 0x7);
  const uint16 *addr;
  for (size_t windex = 0; windex < win.nr; windex++) {
    if (win.bits & (1 << windex))
      continue;  // layer is disabled for this window part
    int sx = win.edges[windex];
    PpuZbufType *dstz = ppu->bgBuffers[sub].data + sx + kPpuExtraLeftRight;
    PpuZbufType *dstz_end = ppu->bgBuffers[sub].data + win.edges[windex + 1] + kPpuExtraLeftRight;
    uint x = sx + ppu->hScroll[layer];
    const uint16 *tp = tps[x >> 8 & 1] + ((x >> 3) & 0x1f);
    const uint16 *tp_last = tps[x >> 8 & 1] + 31, *tp_next = tps[(x >> 8 & 1) ^ 1];
    x &= 7;
    int mosaic_size = PPU_mosaicSize(ppu);
    int w = mosaic_size - (sx - ppu->mosaicModulo[sx]);
    do {
      w = IntMin(w, (int)(dstz_end - dstz));
      uint32 tile = *tp;
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      uint32 bits = READ_BITS(ta, tile & 0x3ff);
      if (tile & 0x4000) bits >>= x, GET_PIXEL(); else bits <<= x, GET_PIXEL_HFLIP();
      if (pixel) {
        pixel += (tile & 0x1c00) >> kPaletteShift;
        uint i = 0;
        do {
          if (z > dstz[i])
            dstz[i] = pixel + z;
        } while (++i != w);
      }
      dstz += w, x += w;
      for (; x >= 8; x -= 8)
        tp = (tp != tp_last) ? tp + 1 : tp_next;
      w = mosaic_size;
    } while (dstz_end - dstz != 0);
  }
#undef READ_BITS
#undef GET_PIXEL
#undef GET_PIXEL_HFLIP
}


// Assumes it's drawn on an empty backdrop
static void PpuDrawBackground_mode7(Ppu *ppu, uint y, bool sub, PpuZbufType z) {
  int layer = 0;
  if (!IS_SCREEN_ENABLED(ppu, sub, layer))
    return;  // layer is completely hidden
  PpuWindows win;
  IS_SCREEN_WINDOWED(ppu, sub, layer) ? PpuWindows_Calc(&win, ppu, layer) : PpuWindows_Clear(&win, ppu, layer);

  // expand 13-bit values to signed values
  int hScroll = ((int16_t)(ppu->m7matrix[6] << 3)) >> 3;
  int vScroll = ((int16_t)(ppu->m7matrix[7] << 3)) >> 3;
  int xCenter = ((int16_t)(ppu->m7matrix[4] << 3)) >> 3;
  int yCenter = ((int16_t)(ppu->m7matrix[5] << 3)) >> 3;
  int clippedH = hScroll - xCenter;
  int clippedV = vScroll - yCenter;
  clippedH = (clippedH & 0x2000) ? (clippedH | ~1023) : (clippedH & 1023);
  clippedV = (clippedV & 0x2000) ? (clippedV | ~1023) : (clippedV & 1023);
  uint8 mosaic_enabled = PPU_mosaicEnabled(ppu, 0);
  if (mosaic_enabled)
    y = ppu->mosaicModulo[y];
  uint32 ry = PPU_m7yFlip(ppu) ? 255 - y : y;
  uint32 m7startX = (ppu->m7matrix[0] * clippedH & ~63) + (ppu->m7matrix[1] * ry & ~63) +
    (ppu->m7matrix[1] * clippedV & ~63) + (xCenter << 8);
  uint32 m7startY = (ppu->m7matrix[2] * clippedH & ~63) + (ppu->m7matrix[3] * ry & ~63) +
    (ppu->m7matrix[3] * clippedV & ~63) + (yCenter << 8);
  for (size_t windex = 0; windex < win.nr; windex++) {
    if (win.bits & (1 << windex))
      continue;  // layer is disabled for this window part
    int x = win.edges[windex], x2 = win.edges[windex + 1], tile;
    PpuZbufType *dstz = ppu->bgBuffers[sub].data + x + kPpuExtraLeftRight;
    PpuZbufType *dstz_end = ppu->bgBuffers[sub].data + x2 + kPpuExtraLeftRight;
    uint32 rx = PPU_m7xFlip(ppu) ? 255 - x : x;
    uint32 xpos = m7startX + ppu->m7matrix[0] * rx;
    uint32 ypos = m7startY + ppu->m7matrix[2] * rx;
    uint32 dx = PPU_m7xFlip(ppu) ? -ppu->m7matrix[0] : ppu->m7matrix[0];
    uint32 dy = PPU_m7xFlip(ppu) ? -ppu->m7matrix[2] : ppu->m7matrix[2];
    uint32 outside_value = PPU_m7largeField(ppu) ? 0x3ffff : 0xffffffff;
    bool char_fill = PPU_m7charFill(ppu);
    if (mosaic_enabled) {
      int w = PPU_mosaicSize(ppu) - (x - ppu->mosaicModulo[x]);
      do {
        w = IntMin(w, (int)(dstz_end - dstz));
        if ((uint32)(xpos | ypos) > outside_value) {
          if (!char_fill)
            continue;
          tile = 0;
        } else {
          tile = ppu->vram[(ypos >> 11 & 0x7f) * 128 + (xpos >> 11 & 0x7f)] & 0xff;
        }
        uint8 pixel = ppu->vram[tile * 64 + (ypos >> 8 & 7) * 8 + (xpos >> 8 & 7)] >> 8;
        if (pixel) {
          int i = 0;
          do dstz[i] = pixel + z; while (++i != w);
        }
      } while (xpos += dx * w, ypos += dy * w, dstz += w, w = PPU_mosaicSize(ppu), dstz_end - dstz != 0);
    } else {
      do {
        if ((uint32)(xpos | ypos) > outside_value) {
          if (!char_fill)
            continue;
          tile = 0;
        } else {
          tile = ppu->vram[(ypos >> 11 & 0x7f) * 128 + (xpos >> 11 & 0x7f)] & 0xff;
        }
        uint8 pixel = ppu->vram[tile * 64 + (ypos >> 8 & 7) * 8 + (xpos >> 8 & 7)] >> 8;
        if (pixel)
          dstz[0] = pixel + z;
      } while (xpos += dx, ypos += dy, ++dstz != dstz_end);
    }
  }
}

static void PpuDrawSprites(Ppu *ppu, uint y, uint sub, bool clear_backdrop) {
  int layer = 4;
  if (!IS_SCREEN_ENABLED(ppu, sub, layer))
    return;  // layer is completely hidden
  PpuWindows win;
  IS_SCREEN_WINDOWED(ppu, sub, layer) ? PpuWindows_Calc(&win, ppu, layer) : PpuWindows_Clear(&win, ppu, layer);
  for (size_t windex = 0; windex < win.nr; windex++) {
    if (win.bits & (1 << windex))
      continue;  // layer is disabled for this window part
    int left = win.edges[windex];
    int width = win.edges[windex + 1] - left;
    PpuZbufType *src = ppu->objBuffer.data + left + kPpuExtraLeftRight;
    PpuZbufType *dst = ppu->bgBuffers[sub].data + left + kPpuExtraLeftRight;
    if (clear_backdrop) {
      memcpy(dst, src, width * sizeof(uint16));
    } else {
      do {
        if (src[0] > dst[0])
          dst[0] = src[0];
      } while (src++, dst++, --width);
    }
  }
}

static void PpuDrawBackgrounds(Ppu *ppu, int y, bool sub) {
  // Top 4 bits contain the prio level, and bottom 4 bits the layer type.
  // SPRITE_PRIO_TO_PRIO can be used to convert from obj prio to this prio.
  //  15: BG3 tiles with priority 1 if bit 3 of $2105 is set
  //  14: Sprites with priority 3 (4 * sprite_prio + 2)
  //  12: BG1 tiles with priority 1
  //  11: BG2 tiles with priority 1
  //  10: Sprites with priority 2 (4 * sprite_prio + 2)
  //  8: BG1 tiles with priority 0
  //  7: BG2 tiles with priority 0
  //  6: Sprites with priority 1 (4 * sprite_prio + 2)
  //  3: BG3 tiles with priority 1 if bit 3 of $2105 is clear
  //  2: Sprites with priority 0 (4 * sprite_prio + 2)
  //  1: BG3 tiles with priority 0
  //  0: backdrop

  if (PPU_mode(ppu) == 1) {
    if (ppu->lineHasSprites)
      PpuDrawSprites(ppu, y, sub, true);

    bool mosaic_size = PPU_mosaicSize(ppu) > 1;
    if (mosaic_size && PPU_mosaicEnabled(ppu, 0))
      PpuDrawBackground_4bpp_mosaic(ppu, y, sub, 0, 0xc000, 0x8000);
    else
      PpuDrawBackground_4bpp(ppu, y, sub, 0, 0xc000, 0x8000);

    if (mosaic_size && PPU_mosaicEnabled(ppu, 1))
      PpuDrawBackground_4bpp_mosaic(ppu, y, sub, 1, 0xb100, 0x7100);
    else
      PpuDrawBackground_4bpp(ppu, y, sub, 1, 0xb100, 0x7100);

    uint bg3prio = PPU_bg3priority(ppu) ? 0xf200 : 0x3200;
    if (mosaic_size && PPU_mosaicEnabled(ppu, 2))
      PpuDrawBackground_2bpp_mosaic(ppu, y, sub, 2, bg3prio, 0x1200);
    else
      PpuDrawBackground_2bpp(ppu, y, sub, 2, bg3prio, 0x1200);
  } else {
    // mode 7
    PpuDrawBackground_mode7(ppu, y, sub, 0x5000);
    if (ppu->lineHasSprites)
      PpuDrawSprites(ppu, y, sub, false);
  }
}

static NOINLINE void PpuDrawWholeLine(Ppu *ppu, uint y) {
  if (PPU_forcedBlank(ppu)) {
    uint8 *dst = &ppu->renderBuffer[(y - 1) * ppu->renderPitch];
    size_t n = sizeof(uint32) * (256 + ppu->extraLeftRight * 2);
    memset(dst, 0, n);
    return;
  }

  // Default background is backdrop
  ClearBackdrop(&ppu->bgBuffers[0]);

  // Render main screen
  PpuDrawBackgrounds(ppu, y, false);

  // Render also the subscreen?
  bool rendered_subscreen = false;
  if (PPU_preventMathMode(ppu) != 3 && PPU_addSubscreen(ppu) && PPU_mathEnabled(ppu)) {
    ClearBackdrop(&ppu->bgBuffers[1]);
    if (ppu->screenEnabled[1] != 0) {
      PpuDrawBackgrounds(ppu, y, true);
      rendered_subscreen = true;
    }
  }

  // Color window affects the drawing mode in each region
  PpuWindows cwin;
  PpuWindows_Calc(&cwin, ppu, 5);
  static const uint8 kCwBitsMod[8] = {
    0x00, 0xff, 0xff, 0x00,
    0xff, 0x00, 0xff, 0x00,
  };
  uint32 cw_clip_math = ((cwin.bits & kCwBitsMod[PPU_clipMode(ppu)]) ^ kCwBitsMod[PPU_clipMode(ppu) + 4]) |
    ((cwin.bits & kCwBitsMod[PPU_preventMathMode(ppu)]) ^ kCwBitsMod[PPU_preventMathMode(ppu) + 4]) << 8;

  uint32 *dst = (uint32*)&ppu->renderBuffer[(y - 1) * ppu->renderPitch], *dst_org = dst;

  dst += (ppu->extraLeftRight - ppu->extraLeftCur);

  uint32 windex = 0;
  do {
    uint32 left = cwin.edges[windex] + kPpuExtraLeftRight, right = cwin.edges[windex + 1] + kPpuExtraLeftRight;
    // If clip is set, then zero out the rgb values from the main screen.
    uint32 clip_color_mask = (cw_clip_math & 1) ? 0x1f : 0;
    uint32 math_enabled_cur = PPU_mathEnabled(ppu) & ((cw_clip_math & 0x100) ? -1 : 0);
    uint32 fixed_color = ppu->fixedColor;
    if (math_enabled_cur == 0 || fixed_color == 0 && !PPU_halfColor(ppu) && !rendered_subscreen) {
      // Math is disabled (or has no effect), so can avoid the per-pixel maths check
      uint32 i = left;
      do {
        uint32 color = ppu->cgram[ppu->bgBuffers[0].data[i] & 0xff];
        dst[0] = ppu->brightnessMult[color & clip_color_mask] << 16 |
          ppu->brightnessMult[(color >> 5) & clip_color_mask] << 8 |
          ppu->brightnessMult[(color >> 10) & clip_color_mask];
      } while (dst++, ++i < right);
    } else {
      uint8 *half_color_map = PPU_halfColor(ppu) ? ppu->brightnessMultHalf : ppu->brightnessMult;
      // Store this in locals
      math_enabled_cur |= PPU_addSubscreen(ppu) << 8 | PPU_subtractColor(ppu) << 9;
      // Need to check for each pixel whether to use math or not based on the main screen layer.
      uint32 i = left;
      do {
        uint32 color = ppu->cgram[ppu->bgBuffers[0].data[i] & 0xff], color2;
        uint8 main_layer = (ppu->bgBuffers[0].data[i] >> 8) & 0xf;
        uint32 r = color & clip_color_mask;
        uint32 g = (color >> 5) & clip_color_mask;
        uint32 b = (color >> 10) & clip_color_mask;
        uint8 *color_map = ppu->brightnessMult;
        if (math_enabled_cur & (1 << main_layer)) {
          if (math_enabled_cur & 0x100) {  // addSubscreen ?
            if ((ppu->bgBuffers[1].data[i] & 0xff) != 0)
              color2 = ppu->cgram[ppu->bgBuffers[1].data[i] & 0xff], color_map = half_color_map;
            else  // Don't halve if PPU_addSubscreen(ppu) && backdrop
              color2 = fixed_color;
          } else {
            color2 = fixed_color, color_map = half_color_map;
          }
          uint32 r2 = (color2 & 0x1f), g2 = ((color2 >> 5) & 0x1f), b2 = ((color2 >> 10) & 0x1f);
          if (math_enabled_cur & 0x200) {  // subtractColor?
            r = (r >= r2) ? r - r2 : 0;
            g = (g >= g2) ? g - g2 : 0;
            b = (b >= b2) ? b - b2 : 0;
          } else {
            r += r2;
            g += g2;
            b += b2;
          }
        }
        dst[0] = color_map[b] | color_map[g] << 8 | color_map[r] << 16;
      } while (dst++, ++i < right);
    }
  } while (cw_clip_math >>= 1, ++windex < cwin.nr);

}

static bool ppu_evaluateSprites(Ppu* ppu, int line) {
  static const uint8 spriteSizes[8][2] = {
    {8, 16}, {8, 32}, {8, 64}, {16, 32},
    {16, 64}, {32, 64}, {16, 32}, {16, 32}
  };

  // TODO: iterate over oam normally to determine in-range sprites,
  //   then iterate those in-range sprites in reverse for tile-fetching
  // TODO: rectangular sprites, wierdness with sprites at -256
  uint8_t index = PPU_objPriority(ppu) ? (ppu->oamaddl & 0xfe) : 0;
  int spritesFound = 0;
  int tilesFound = 0;
  for(int i = 0; i < 128; i++) {
    uint8_t y = ppu->oam[index] >> 8;
    // check if the sprite is on this line and get the sprite size
    uint8_t row = line - y;
    int spriteSize = spriteSizes[PPU_objSize(ppu)][(ppu->highOam[index >> 3] >> ((index & 7) + 1)) & 1];
    int spriteHeight = PPU_objInterlace(ppu) ? spriteSize / 2 : spriteSize;
    if(row < spriteHeight) {
      // in y-range, get the x location, using the high bit as well
      int x = ppu->oam[index] & 0xff;
      x |= ((ppu->highOam[index >> 3] >> (index & 7)) & 1) << 8;
      int extra = ppu->extraLeftCur;
      if(x > 255 + extra) x -= 512;
      if(x > -(spriteSize + extra)) {
        // break if we found 32 sprites already
        spritesFound++;
        if(spritesFound > 32) {
          ppu->rangeOver = true;
          break;
        }
        // update row according to obj-interlace
        if(PPU_objInterlace(ppu)) row = row * 2 + (ppu->evenFrame ? 0 : 1);
        // get some data for the sprite and y-flip row if needed
        int oam1 = ppu->oam[index + 1];
        int tile = oam1 & 0xff;
        int objAdr = (oam1 & 0x100) ? PPU_objTileAdr2(ppu) : PPU_objTileAdr1(ppu);
        int palette = (oam1 & 0xe00) >> 9;
        bool hFlipped = oam1 & 0x4000;
        if(oam1 & 0x8000) row = spriteSize - 1 - row;
        // fetch all tiles in x-range
        int paletteBase = 0x80 + 16 * ((oam1 & 0xe00) >> 9);
        int prio = SPRITE_PRIO_TO_PRIO((oam1 & 0x3000) >> 12, (oam1 & 0x800) == 0);
        PpuZbufType z = paletteBase + (prio << 8);

        for(int col = 0; col < spriteSize; col += 8) {
          if(col + x > -8 - extra && col + x < 256 + extra) {
            // break if we found 34 8*1 slivers already
            tilesFound++;
            if(tilesFound > 34) {
              ppu->timeOver = true;
              break;
            }
            // figure out which tile this uses, looping within 16x16 pages, and get it's data
            int usedCol = oam1 & 0x4000 ? spriteSize - 1 - col : col;
            int usedTile = ((((oam1 & 0xff) >> 4) + (row >> 3)) << 4) | (((oam1 & 0xf) + (usedCol >> 3)) & 0xf);
            uint16 *addr = &ppu->vram[(objAdr + usedTile * 16 + (row & 0x7)) & 0x7fff];
            uint32 plane = addr[0] | addr[8] << 16;
            // go over each pixel
            int px_left = IntMax(-(col + x + kPpuExtraLeftRight), 0);
            int px_right = IntMin(256 + kPpuExtraLeftRight - (col + x), 8);
            PpuZbufType *dst = ppu->objBuffer.data + col + x + px_left + kPpuExtraLeftRight;

            for (int px = px_left; px < px_right; px++, dst++) {
              int shift = oam1 & 0x4000 ? px : 7 - px;
              uint32 bits = plane >> shift;
              int pixel = (bits >> 0) & 1 | (bits >> 7) & 2 | (bits >> 14) & 4 | (bits >> 21) & 8;
              // draw it in the buffer if there is a pixel here, and the buffer there is still empty
              if (pixel != 0 && (dst[0] & 0xff) == 0)
                dst[0] = z + pixel;
            }

          }
        }
        if(tilesFound > 34) break; // break out of sprite-loop if max tiles found
      }
    }
    index += 2;
  }
  return tilesFound != 0;
}

static uint16_t ppu_getVramRemap(Ppu* ppu) {
  uint16_t adr = ppu->vramPointer;
  switch(ppu->vramRemapMode) {
    case 0: return adr;
    case 1: return (adr & 0xff00) | ((adr & 0xe0) >> 5) | ((adr & 0x1f) << 3);
    case 2: return (adr & 0xfe00) | ((adr & 0x1c0) >> 6) | ((adr & 0x3f) << 3);
    case 3: return (adr & 0xfc00) | ((adr & 0x380) >> 7) | ((adr & 0x7f) << 3);
  }
  return adr;
}

uint8_t ppu_read(Ppu* ppu, uint8_t adr) {
  switch(adr) {
  case 0x34:
  case 0x35:
  case 0x36: {
    int result = ppu->m7matrix[0] * (ppu->m7matrix[1] >> 8);
    return (result >> (8 * (adr - 0x34))) & 0xff;
  }
    case 0x37: {
      ///* ignore unmapped ppu read */
      return 0;
    }
    case 0x38: {
      uint8_t ret = 0;
      if(ppu->oamInHigh) {
        ret = ppu->highOam[((ppu->oamAdr & 0xf) << 1) | ppu->oamSecondWrite];
        if(ppu->oamSecondWrite) {
          ppu->oamAdr++;
          if(ppu->oamAdr == 0) ppu->oamInHigh = false;
        }
      } else {
        if(!ppu->oamSecondWrite) {
          ret = ppu->oam[ppu->oamAdr] & 0xff;
        } else {
          ret = ppu->oam[ppu->oamAdr++] >> 8;
          if(ppu->oamAdr == 0) ppu->oamInHigh = true;
        }
      }
      ppu->oamSecondWrite = !ppu->oamSecondWrite;
      return ret;
    }
    case 0x39: {
      uint16_t val = ppu->vramReadBuffer;
      if(!ppu->vramIncrementOnHigh) {
        ppu->vramReadBuffer = ppu->vram[ppu_getVramRemap(ppu) & 0x7fff];
        ppu->vramPointer += ppu->vramIncrement;
      }
      return val & 0xff;
    }
    case 0x3a: {
      uint16_t val = ppu->vramReadBuffer;
      if(ppu->vramIncrementOnHigh) {
        ppu->vramReadBuffer = ppu->vram[ppu_getVramRemap(ppu) & 0x7fff];
        ppu->vramPointer += ppu->vramIncrement;
      }
      return val >> 8;
    }
    case 0x3b: {
      uint8_t ret = 0;
      if(!ppu->cgramSecondWrite) {
        ret = ppu->cgram[ppu->cgramPointer] & 0xff;
      } else {
        ret = ((ppu->cgram[ppu->cgramPointer++] >> 8) & 0x7f);
      }
      ppu->cgramSecondWrite = !ppu->cgramSecondWrite;
      return ret;
    }
    case 0x3c: {
      uint8_t val = 0;
      if(ppu->hCountSecond) {
        val = ((ppu->hCount >> 8) & 1);
      } else {
        val = ppu->hCount & 0xff;
      }
      ppu->hCountSecond = !ppu->hCountSecond;
      return val;
    }
    case 0x3d: {
      uint8_t val = 0;
      if(ppu->vCountSecond) {
        val = ((ppu->vCount >> 8) & 1);
      } else {
        val = ppu->vCount & 0xff;
      }
      ppu->vCountSecond = !ppu->vCountSecond;
      return val;
    }
    case 0x3e: {
      uint8_t val = 0x1; // ppu1 version (4 bit)
      val |= ppu->rangeOver << 6;
      val |= ppu->timeOver << 7;
      return val;
    }
    case 0x3f: {
      uint8_t val = 0x3; // ppu2 version (4 bit), bit 4: ntsc/pal
      val |= ppu->countersLatched << 6;
      val |= ppu->evenFrame << 7;
      ppu->countersLatched = false; // TODO: only when ppulatch is set
      ppu->hCountSecond = false;
      ppu->vCountSecond = false;
      return val;
    }
    default: {
      /* ignore unmapped ppu read */
      return 0;
    }
  }
}

void ppu_write(Ppu* ppu, uint8_t adr, uint8_t val) {
//  if (adr != 24 && adr != 25)
//    printf("ppu_write(%d, %d)\n", adr, val);
  switch(adr) {
    case INIDISP & 0xff:
      if ((val & 0x80) && !(ppu->inidisp & 0x80) && ppu->was_visible) {
        ppu->vram_epoch = (ppu->vram_epoch + 1) & 0xf;
        ppu->was_visible = false;
      }
      ppu->inidisp = val;
      break;
    case OBSEL & 0xff:
      ppu->obsel = val;
      break;
    case OAMADDL & 0xff:
      ppu->oamaddl = val;
      ppu->oamAdr = val;
      ppu->oamInHigh = ppu->oamaddh & 1;
      ppu->oamSecondWrite = false;
      break;
    case OAMADDH & 0xff:
      ppu->oamaddh = val;
      ppu->oamInHigh = val & 1;
      ppu->oamAdr = ppu->oamaddl;
      ppu->oamSecondWrite = false;
      break;
    case 0x04: {
      if(ppu->oamInHigh) {
        ppu->highOam[((ppu->oamAdr & 0xf) << 1) | ppu->oamSecondWrite] = val;
        if(ppu->oamSecondWrite) {
          ppu->oamAdr++;
          if(ppu->oamAdr == 0) ppu->oamInHigh = false;
        }
      } else {
        if(!ppu->oamSecondWrite) {
          ppu->oamBuffer = val;
        } else {
          ppu->oam[ppu->oamAdr++] = (val << 8) | ppu->oamBuffer;
          if(ppu->oamAdr == 0) ppu->oamInHigh = true;
        }
      }
      ppu->oamSecondWrite = !ppu->oamSecondWrite;
      break;
    }
    case BGMODE & 0xff:
      /* bigTiles allowed */
      ppu->bgmode = val;
      break;
    case MOSAIC & 0xff:
      ppu->mosaic = val;
      ppu->mosaicStartLine = 0;// ppu->snes->vPos;
      break;
    case BG1SC & 0xff:
    case BG2SC & 0xff:
    case BG3SC & 0xff:
    case BG4SC & 0xff:
      ppu->bgXsc[adr - 7] = val;
      break;
    case BG12NBA & 0xff:
      ppu->bgTileAdr = ppu->bgTileAdr & 0xff00 | val;
      break;
    case BG34NBA & 0xff:
      ppu->bgTileAdr = ppu->bgTileAdr & 0xff | val << 8;
      break;
    case 0x0d: {
      ppu->m7matrix[6] = ((val << 8) | ppu->m7prev) & 0x1fff;
      ppu->m7prev = val;
      // fallthrough to normal layer BG-HOFS
    }
    case 0x0f:
    case 0x11:
    case 0x13: {
      ppu->hScroll[(adr - 0xd) / 2] = ((val << 8) | (ppu->scrollPrev & 0xf8) | (ppu->scrollPrev2 & 0x7)) & 0x3ff;
      ppu->scrollPrev = val;
      ppu->scrollPrev2 = val;
      break;
    }
    case 0x0e: {
      ppu->m7matrix[7] = ((val << 8) | ppu->m7prev) & 0x1fff;
      ppu->m7prev = val;
      // fallthrough to normal layer BG-VOFS
    }
    case 0x10:
    case 0x12:
    case 0x14: {
      ppu->vScroll[(adr - 0xe) / 2] = ((val << 8) | ppu->scrollPrev) & 0x3ff;
      ppu->scrollPrev = val;
      break;
    }
    case 0x15: {
      if((val & 3) == 0) {
        ppu->vramIncrement = 1;
      } else if((val & 3) == 1) {
        ppu->vramIncrement = 32;
      } else {
        ppu->vramIncrement = 128;
      }
      ppu->vramRemapMode = (val & 0xc) >> 2;
      ppu->vramIncrementOnHigh = val & 0x80;
      break;
    }
    case 0x16: {
      ppu->vramPointer = (ppu->vramPointer & 0xff00) | val;
      ppu->vramReadBuffer = ppu->vram[ppu_getVramRemap(ppu) & 0x7fff];
      break;
    }
    case 0x17: {
      ppu->vramPointer = (ppu->vramPointer & 0x00ff) | (val << 8);
      ppu->vramReadBuffer = ppu->vram[ppu_getVramRemap(ppu) & 0x7fff];
      break;
    }
    case 0x18: {
      // TODO: vram access during rendering (also cgram and oam)
      uint16_t vramAdr = ppu_getVramRemap(ppu);
      ppu->vram[vramAdr & 0x7fff] = (ppu->vram[vramAdr & 0x7fff] & 0xff00) | val;
      uint8_t colBit = (ppu->vramIncrement == 32) ? 0x80u : 0u;
      if (colBit) ppu->col_streamed_mask |= (uint16_t)(1u << (((vramAdr & 0x7fff) >> 11) & 0xf));
      else if ((vramAdr & 0x7ff) == 0 && ppu->vramIncrement == 1) ppu->col_streamed_mask &= ~(uint16_t)(1u << (((vramAdr & 0x7fff) >> 11) & 0xf));
      ppu->vram_bgmode[vramAdr & 0x7fff] = colBit | (uint8_t)((ppu->vram_epoch << 3) | ((PPU_mode(ppu) + 1) & 7));
      if(!ppu->vramIncrementOnHigh) ppu->vramPointer += ppu->vramIncrement;
      break;
    }
    case 0x19: {
      uint16_t vramAdr = ppu_getVramRemap(ppu);
      ppu->vram[vramAdr & 0x7fff] = (ppu->vram[vramAdr & 0x7fff] & 0x00ff) | (val << 8);
      uint8_t colBit = (ppu->vramIncrement == 32) ? 0x80u : 0u;
      if (colBit) ppu->col_streamed_mask |= (uint16_t)(1u << (((vramAdr & 0x7fff) >> 11) & 0xf));
      ppu->vram_bgmode[vramAdr & 0x7fff] = colBit | (uint8_t)((ppu->vram_epoch << 3) | ((PPU_mode(ppu) + 1) & 7));
      if(ppu->vramIncrementOnHigh) ppu->vramPointer += ppu->vramIncrement;
      break;
    }
    case M7SEL & 0xff:
      ppu->m7sel = val;
      break;
    case 0x1b:
    case 0x1c:
    case 0x1d:
    case 0x1e:
      ppu->m7matrix[adr - 0x1b] = (val << 8) | ppu->m7prev;
      ppu->m7prev = val;
      break;
    case 0x1f:
    case 0x20:
      ppu->m7matrix[adr - 0x1b] = ((val << 8) | ppu->m7prev) & 0x1fff;
      ppu->m7prev = val;
      break;
    case 0x21:
      ppu->cgramPointer = val;
      ppu->cgramSecondWrite = false;
      break;
    case 0x22:
      if(!ppu->cgramSecondWrite) {
        ppu->cgramBuffer = val;
      } else {
        ppu->cgram[ppu->cgramPointer++] = (val << 8) | ppu->cgramBuffer;
      }
      ppu->cgramSecondWrite = !ppu->cgramSecondWrite;
      break;
    case 0x23:
      ppu->windowsel = (ppu->windowsel & ~0xff) | val;
      break;
    case 0x24:
      ppu->windowsel = (ppu->windowsel & ~0xff00) | (val << 8);
      break;
    case 0x25:
      ppu->windowsel = (ppu->windowsel & ~0xff0000) | (val << 16);
      break;
    case 0x26:
      ppu->window1left = val;
      break;
    case 0x27:
      ppu->window1right = val;
      break;
    case 0x28:
      ppu->window2left = val;
      break;
    case 0x29:
      ppu->window2right = val;
      break;
    case WBGLOG & 0xff:
      ppu->wbgobjlog = ppu->wbgobjlog & 0xff00 | val;
      break;
    case WOBJLOG & 0xff:
      ppu->wbgobjlog = ppu->wbgobjlog & 0xff | val << 8;
      break;
    case TM & 0xff:
      ppu->screenEnabled[0] = val;
      break;
    case TS & 0xff:
      ppu->screenEnabled[1] = val;
      break;
    case TMW & 0xff:
      ppu->screenWindowed[0] = val;
      break;
    case TSW & 0xff:
      ppu->screenWindowed[1] = val;
      break;
    case CGWSEL & 0xff:
      ppu->cgwsel = val;
      break;
    case CGADSUB & 0xff:
      ppu->cgadsub = val;
      break;
    case COLDATA & 0xff:
      if (val & 0x80) ppu->fixedColor = (ppu->fixedColor & ~(0x1f << 10)) | (val & 0x1f) << 10; // blue
      if (val & 0x40) ppu->fixedColor = (ppu->fixedColor & ~(0x1f <<  5)) | (val & 0x1f) << 5;  // green
      if (val & 0x20) ppu->fixedColor = (ppu->fixedColor & ~(0x1f <<  0)) | (val & 0x1f) << 0;  // red
      break;
    case SETINI & 0xff:
      ppu->setini = val;
      break;
    default:
      break;
  }
}

int PpuGetCurrentRenderScale(Ppu *ppu, uint32_t render_flags) {
  return 1;
}


/* === ppu_old.c === */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>




static void ppu_handlePixel(Ppu *ppu, int x, int y);
static int ppu_getPixel(Ppu *ppu, int x, int y, bool sub, int *r, int *g, int *b);
static uint16_t ppu_getOffsetValue(Ppu *ppu, int col, int row);
static int ppu_getPixelForBgLayer(Ppu *ppu, int x, int y, int layer, bool priority);
static void ppu_handleOPT(Ppu *ppu, int layer, int *lx, int *ly);
static void ppu_calculateMode7Starts(Ppu *ppu, int y);
static int ppu_getPixelForMode7(Ppu *ppu, int x, int layer, bool priority);
static bool ppu_getWindowState(Ppu *ppu, int layer, int x);

// array for layer definitions per mode:
//   0-7: mode 0-7; 8: mode 1 + l3prio; 9: mode 7 + extbg

//   0-3; layers 1-4; 4: sprites; 5: nonexistent
static const int layersPerMode[10][12] = {
  {4, 0, 1, 4, 0, 1, 4, 2, 3, 4, 2, 3},
  {4, 0, 1, 4, 0, 1, 4, 2, 4, 2, 5, 5},
  {4, 0, 4, 1, 4, 0, 4, 1, 5, 5, 5, 5},
  {4, 0, 4, 1, 4, 0, 4, 1, 5, 5, 5, 5},
  {4, 0, 4, 1, 4, 0, 4, 1, 5, 5, 5, 5},
  {4, 0, 4, 1, 4, 0, 4, 1, 5, 5, 5, 5},
  {4, 0, 4, 4, 0, 4, 5, 5, 5, 5, 5, 5},
  {4, 4, 4, 0, 4, 5, 5, 5, 5, 5, 5, 5},
  {2, 4, 0, 1, 4, 0, 1, 4, 4, 2, 5, 5},
  {4, 4, 1, 4, 0, 4, 1, 5, 5, 5, 5, 5}
};

static const int prioritysPerMode[10][12] = {
  {3, 1, 1, 2, 0, 0, 1, 1, 1, 0, 0, 0},
  {3, 1, 1, 2, 0, 0, 1, 1, 0, 0, 5, 5},
  {3, 1, 2, 1, 1, 0, 0, 0, 5, 5, 5, 5},
  {3, 1, 2, 1, 1, 0, 0, 0, 5, 5, 5, 5},
  {3, 1, 2, 1, 1, 0, 0, 0, 5, 5, 5, 5},
  {3, 1, 2, 1, 1, 0, 0, 0, 5, 5, 5, 5},
  {3, 1, 2, 1, 0, 0, 5, 5, 5, 5, 5, 5},
  {3, 2, 1, 0, 0, 5, 5, 5, 5, 5, 5, 5},
  {1, 3, 1, 1, 2, 0, 0, 1, 0, 0, 5, 5},
  {3, 2, 1, 1, 0, 0, 0, 5, 5, 5, 5, 5}
};

static const int layerCountPerMode[10] = {
  12, 10, 8, 8, 8, 8, 6, 5, 10, 7
};

static const int bitDepthsPerMode[10][4] = {
  {2, 2, 2, 2},
  {4, 4, 2, 5},
  {4, 4, 5, 5},
  {8, 4, 5, 5},
  {8, 2, 5, 5},
  {4, 2, 5, 5},
  {4, 5, 5, 5},
  {8, 5, 5, 5},
  {4, 4, 2, 5},
  {8, 7, 5, 5}
};

void PpuDrawWholeLineOldPpu(Ppu *ppu, int line) {
  // actual line
  if (PPU_mode(ppu) == 7)
    ppu_calculateMode7Starts(ppu, line);
  for (int x = -ppu->extraLeftCur; x < 256 + ppu->extraRightCur; x++) {
    ppu_handlePixel(ppu, x, line);
  }
}

static void ppu_handlePixel(Ppu *ppu, int x, int y) {
  int r = 0, r2 = 0;
  int g = 0, g2 = 0;
  int b = 0, b2 = 0;
  if (!PPU_forcedBlank(ppu)) {
    int mainLayer = ppu_getPixel(ppu, x, y, false, &r, &g, &b);
    bool colorWindowState = ppu_getWindowState(ppu, 5, x);
    if (
      PPU_clipMode(ppu) == 3 ||
      (PPU_clipMode(ppu) == 2 && colorWindowState) ||
      (PPU_clipMode(ppu) == 1 && !colorWindowState)
      ) {
      r = 0;
      g = 0;
      b = 0;
    }
    int secondLayer = 5; // backdrop
    bool mathEnabled = mainLayer < 6 && (PPU_mathEnabled(ppu) >> mainLayer & 1) && !(
      PPU_preventMathMode(ppu) == 3 ||
      (PPU_preventMathMode(ppu) == 2 && colorWindowState) ||
      (PPU_preventMathMode(ppu) == 1 && !colorWindowState)
      );
    if ((mathEnabled && PPU_addSubscreen(ppu)) || PPU_pseudoHires(ppu) || PPU_mode(ppu) == 5 || PPU_mode(ppu) == 6) {
      secondLayer = ppu_getPixel(ppu, x, y, true, &r2, &g2, &b2);
    }
    // TODO: subscreen pixels can be clipped to black as well
    // TODO: math for subscreen pixels (add/sub sub to main)
    if (mathEnabled) {
      if (PPU_subtractColor(ppu)) {
        r -= (PPU_addSubscreen(ppu) && secondLayer != 5) ? r2 : PPU_fixedColorR(ppu);
        g -= (PPU_addSubscreen(ppu) && secondLayer != 5) ? g2 : PPU_fixedColorG(ppu);
        b -= (PPU_addSubscreen(ppu) && secondLayer != 5) ? b2 : PPU_fixedColorB(ppu);
      } else {
        r += (PPU_addSubscreen(ppu) && secondLayer != 5) ? r2 : PPU_fixedColorR(ppu);
        g += (PPU_addSubscreen(ppu) && secondLayer != 5) ? g2 : PPU_fixedColorG(ppu);
        b += (PPU_addSubscreen(ppu) && secondLayer != 5) ? b2 : PPU_fixedColorB(ppu);
      }
      if (PPU_halfColor(ppu) && (secondLayer != 5 || !PPU_addSubscreen(ppu))) {
        r >>= 1;
        g >>= 1;
        b >>= 1;
      }
      if (r > 31) r = 31;
      if (g > 31) g = 31;
      if (b > 31) b = 31;
      if (r < 0) r = 0;
      if (g < 0) g = 0;
      if (b < 0) b = 0;
    }
    if (!(PPU_pseudoHires(ppu) || PPU_mode(ppu) == 5 || PPU_mode(ppu) == 6)) {
      r2 = r; g2 = g; b2 = b;
    }
  }
  int row = y - 1;
  uint8 *pixelBuffer = (uint8 *)&ppu->renderBuffer[row * ppu->renderPitch + (x + ppu->extraLeftRight) * 4];
  pixelBuffer[0] = ((b << 3) | (b >> 2)) * PPU_brightness(ppu) / 15;
  pixelBuffer[1] = ((g << 3) | (g >> 2)) * PPU_brightness(ppu) / 15;
  pixelBuffer[2] = ((r << 3) | (r >> 2)) * PPU_brightness(ppu) / 15;
  pixelBuffer[3] = 0;
}

static int ppu_getPixel(Ppu *ppu, int x, int y, bool sub, int *r, int *g, int *b) {
  // figure out which color is on this location on main- or subscreen, sets it in r, g, b
  // returns which layer it is: 0-3 for bg layer, 4 or 6 for sprites (depending on palette), 5 for backdrop
  int actMode = PPU_mode(ppu) == 1 && PPU_bg3priority(ppu) ? 8 : PPU_mode(ppu);
  actMode = PPU_mode(ppu) == 7 && PPU_m7extBg(ppu) ? 9 : actMode;
  int layer = 5;
  int pixel = 0;
  for (int i = 0; i < layerCountPerMode[actMode]; i++) {
    int curLayer = layersPerMode[actMode][i];
    int curPriority = prioritysPerMode[actMode][i];
    bool layerActive = IS_SCREEN_ENABLED(ppu, sub, curLayer) && (
      !IS_SCREEN_WINDOWED(ppu, sub, curLayer) || !ppu_getWindowState(ppu, curLayer, x)
      );
    if (layerActive) {
      if (curLayer < 4) {
        if ((x < 0 || x >= 256) && !PPU_bgTilemapWider(ppu, curLayer) && ppu->hScroll[curLayer] == 0) {
          if (curLayer == 2 || (PPU_mode(ppu) == 2 && curLayer == 0)) continue;
        }
        // bg layer
        int lx = x;
        if ((x < 0 || x >= 256) && PPU_mode(ppu) == 3 && curLayer == 0 && !PPU_bgTilemapWider(ppu, 0) && ppu->hScroll[0] == 0) {
          lx = (y >= 178) ? (240 + (x & 15)) : 255;
        } else if ((x < 0 || x >= 256) && PPU_mode(ppu) == 2 && curLayer == 1 && !PPU_bgTilemapWider(ppu, 1) && ppu->hScroll[1] == 0 && ppu->vScroll[1] == 0) {
          lx = (x < 0) ? (x & 15) : (240 + (x & 15));
        }
        int ly = y;
        if (PPU_mosaicEnabled(ppu, curLayer) && PPU_mosaicSize(ppu) > 1) {
          lx -= lx % PPU_mosaicSize(ppu);
          ly -= (ly - ppu->mosaicStartLine) % PPU_mosaicSize(ppu);
        }
        if (PPU_mode(ppu) == 7) {
          pixel = ppu_getPixelForMode7(ppu, lx, curLayer, curPriority);
        } else {
          lx += ppu->hScroll[curLayer];
          if (PPU_mode(ppu) == 5 || PPU_mode(ppu) == 6) {
            lx *= 2;
            lx += (sub || PPU_mosaicEnabled(ppu, curLayer)) ? 0 : 1;
            if (PPU_interlace(ppu)) {
              ly *= 2;
              ly += (ppu->evenFrame || PPU_mosaicEnabled(ppu, curLayer)) ? 0 : 1;
            }
          }
          ly += ppu->vScroll[curLayer];
          if (PPU_mode(ppu) == 2 || PPU_mode(ppu) == 4 || PPU_mode(ppu) == 6) {
            ppu_handleOPT(ppu, curLayer, &lx, &ly);
          }
          pixel = ppu_getPixelForBgLayer(
            ppu, lx & 0x3ff, ly & 0x3ff,
            curLayer, curPriority
          );
        }
      } else {
        // get a pixel from the sprite buffer
        pixel = 0;
        if ((ppu->objBuffer.data[x + kPpuExtraLeftRight] >> 12) == SPRITE_PRIO_TO_PRIO_HI(curPriority))
          pixel = ppu->objBuffer.data[x + kPpuExtraLeftRight] & 0xff;
      }
    }
    if (pixel > 0) {
      layer = curLayer;
      break;
    }
  }
  if (PPU_directColor(ppu) && layer < 4 && bitDepthsPerMode[actMode][layer] == 8) {
    *r = ((pixel & 0x7) << 2) | ((pixel & 0x100) >> 7);
    *g = ((pixel & 0x38) >> 1) | ((pixel & 0x200) >> 8);
    *b = ((pixel & 0xc0) >> 3) | ((pixel & 0x400) >> 8);
  } else {
    uint16_t color = ppu->cgram[pixel & 0xff];
    *r = color & 0x1f;
    *g = (color >> 5) & 0x1f;
    *b = (color >> 10) & 0x1f;
  }
  if (layer == 4 && pixel < 0xc0) layer = 6; // sprites with palette color < 0xc0
  return layer;
}

static void ppu_handleOPT(Ppu *ppu, int layer, int *lx, int *ly) {
  int x = *lx;
  int y = *ly;
  int column = 0;
  if (PPU_mode(ppu) == 6) {
    column = ((x - (x & 0xf)) - ((ppu->hScroll[layer] * 2) & 0xfff0)) >> 4;
  } else {
    column = ((x - (x & 0x7)) - (ppu->hScroll[layer] & 0xfff8)) >> 3;
  }
  int opt_col = (column < 0 || column >= 32) ? (4 + (column & 3)) : column;
  if (opt_col > 0) {
    // fetch offset values from layer 3 tilemap
    int valid = layer == 0 ? 0x2000 : 0x4000;
    uint16_t hOffset = ppu_getOffsetValue(ppu, opt_col - 1, 0);
    uint16_t vOffset = 0;
    if (PPU_mode(ppu) == 4) {
      if (hOffset & 0x8000) {
        vOffset = hOffset;
        hOffset = 0;
      }
    } else {
      vOffset = ppu_getOffsetValue(ppu, opt_col - 1, 1);
    }
    if (PPU_mode(ppu) == 6) {
      // TODO: not sure if correct
      if (hOffset & valid) *lx = (((hOffset & 0x3f8) + (opt_col * 8)) * 2) | (x & 0xf);
    } else {
      if (hOffset & valid) *lx = ((hOffset & 0x3f8) + (opt_col * 8)) | (x & 0x7);
      else if (column < 0 || column >= 32) *lx = ((opt_col * 8) + (ppu->hScroll[layer] & 0xfff8)) | (x & 0x7);
    }
    // TODO: not sure if correct for interlace
    if (vOffset & valid) *ly = (vOffset & 0x3ff) + (y - ppu->vScroll[layer]);
  }
}

static uint16_t ppu_getOffsetValue(Ppu *ppu, int col, int row) {
  int x = col * 8 + ppu->hScroll[2];
  int y = row * 8 + ppu->vScroll[2];
  int tileBits = PPU_bigTiles(ppu, 2) ? 4 : 3;
  int tileHighBit = PPU_bigTiles(ppu, 2) ? 0x200 : 0x100;
  uint16_t tilemapAdr = PPU_bgTilemapAdr(ppu, 2) + (((y >> tileBits) & 0x1f) << 5 | ((x >> tileBits) & 0x1f));
  if ((x & tileHighBit) && PPU_bgTilemapWider(ppu, 2)) tilemapAdr += 0x400;
  if ((y & tileHighBit) && PPU_bgTilemapHigher(ppu, 2)) tilemapAdr += PPU_bgTilemapWider(ppu, 2) ? 0x800 : 0x400;
  return ppu->vram[tilemapAdr & 0x7fff];
}

static int ppu_getPixelForBgLayer(Ppu *ppu, int x, int y, int layer, bool priority) {
  // figure out address of tilemap word and read it
  bool wideTiles = PPU_bigTiles(ppu, layer) || PPU_mode(ppu) == 5 || PPU_mode(ppu) == 6;
  int tileBitsX = wideTiles ? 4 : 3;
  int tileHighBitX = wideTiles ? 0x200 : 0x100;
  int tileBitsY = PPU_bigTiles(ppu, layer) ? 4 : 3;
  int tileHighBitY = PPU_bigTiles(ppu, layer) ? 0x200 : 0x100;
  uint16_t tilemapAdr = PPU_bgTilemapAdr(ppu, layer) + (((y >> tileBitsY) & 0x1f) << 5 | ((x >> tileBitsX) & 0x1f));
  if ((x & tileHighBitX) && PPU_bgTilemapWider(ppu, layer)) tilemapAdr += 0x400;
  if ((y & tileHighBitY) && PPU_bgTilemapHigher(ppu, layer)) tilemapAdr += PPU_bgTilemapWider(ppu, layer) ? 0x800 : 0x400;
  uint16_t tile = ppu->vram[tilemapAdr & 0x7fff];
  // check priority, get palette
  if (((bool)(tile & 0x2000)) != priority) return 0; // wrong priority
  int paletteNum = (tile & 0x1c00) >> 10;
  // figure out position within tile
  int row = (tile & 0x8000) ? 7 - (y & 0x7) : (y & 0x7);
  int col = (tile & 0x4000) ? (x & 0x7) : 7 - (x & 0x7);
  int tileNum = tile & 0x3ff;
  if (wideTiles) {
    // if unflipped right half of tile, or flipped left half of tile
    if (((bool)(x & 8)) ^ ((bool)(tile & 0x4000))) tileNum += 1;
  }
  if (PPU_bigTiles(ppu, layer)) {
    // if unflipped bottom half of tile, or flipped upper half of tile
    if (((bool)(y & 8)) ^ ((bool)(tile & 0x8000))) tileNum += 0x10;
  }
  // read tiledata, ajust palette for mode 0
  int bitDepth = bitDepthsPerMode[PPU_mode(ppu)][layer];
  if (PPU_mode(ppu) == 0) paletteNum += 8 * layer;
  // plane 1 (always)
  int paletteSize = 4;
  uint16_t plane1 = ppu->vram[(PPU_bgTileAdr(ppu, layer) + ((tileNum & 0x3ff) * 4 * bitDepth) + row) & 0x7fff];
  int pixel = (plane1 >> col) & 1;
  pixel |= ((plane1 >> (8 + col)) & 1) << 1;
  // plane 2 (for 4bpp, 8bpp)
  if (bitDepth > 2) {
    paletteSize = 16;
    uint16_t plane2 = ppu->vram[(PPU_bgTileAdr(ppu, layer) + ((tileNum & 0x3ff) * 4 * bitDepth) + 8 + row) & 0x7fff];
    pixel |= ((plane2 >> col) & 1) << 2;
    pixel |= ((plane2 >> (8 + col)) & 1) << 3;
  }
  // plane 3 & 4 (for 8bpp)
  if (bitDepth > 4) {
    paletteSize = 256;
    uint16_t plane3 = ppu->vram[(PPU_bgTileAdr(ppu, layer) + ((tileNum & 0x3ff) * 4 * bitDepth) + 16 + row) & 0x7fff];
    pixel |= ((plane3 >> col) & 1) << 4;
    pixel |= ((plane3 >> (8 + col)) & 1) << 5;
    uint16_t plane4 = ppu->vram[(PPU_bgTileAdr(ppu, layer) + ((tileNum & 0x3ff) * 4 * bitDepth) + 24 + row) & 0x7fff];
    pixel |= ((plane4 >> col) & 1) << 6;
    pixel |= ((plane4 >> (8 + col)) & 1) << 7;
  }
  // return cgram index, or 0 if transparent, palette number in bits 10-8 for 8-color layers
  return pixel == 0 ? 0 : paletteSize * paletteNum + pixel;
}

static void ppu_calculateMode7Starts(Ppu *ppu, int y) {
  // expand 13-bit values to signed values
  int hScroll = ((int16_t)(ppu->m7matrix[6] << 3)) >> 3;
  int vScroll = ((int16_t)(ppu->m7matrix[7] << 3)) >> 3;
  int xCenter = ((int16_t)(ppu->m7matrix[4] << 3)) >> 3;
  int yCenter = ((int16_t)(ppu->m7matrix[5] << 3)) >> 3;
  // do calculation
  int clippedH = hScroll - xCenter;
  int clippedV = vScroll - yCenter;
  clippedH = (clippedH & 0x2000) ? (clippedH | ~1023) : (clippedH & 1023);
  clippedV = (clippedV & 0x2000) ? (clippedV | ~1023) : (clippedV & 1023);
  if (PPU_mosaicEnabled(ppu, 0) && PPU_mosaicSize(ppu) > 1) {
    y -= (y - ppu->mosaicStartLine) % PPU_mosaicSize(ppu);
  }
  uint8_t ry = PPU_m7yFlip(ppu) ? 255 - y : y;
  ppu->m7startX = (
    ((ppu->m7matrix[0] * clippedH) & ~63) +
    ((ppu->m7matrix[1] * ry) & ~63) +
    ((ppu->m7matrix[1] * clippedV) & ~63) +
    (xCenter << 8)
    );
  ppu->m7startY = (
    ((ppu->m7matrix[2] * clippedH) & ~63) +
    ((ppu->m7matrix[3] * ry) & ~63) +
    ((ppu->m7matrix[3] * clippedV) & ~63) +
    (yCenter << 8)
    );
}

static int ppu_getPixelForMode7(Ppu *ppu, int x, int layer, bool priority) {
  uint8_t rx = PPU_m7xFlip(ppu) ? 255 - x : x;
  int xPos = (ppu->m7startX + ppu->m7matrix[0] * rx) >> 8;
  int yPos = (ppu->m7startY + ppu->m7matrix[2] * rx) >> 8;
  bool outsideMap = xPos < 0 || xPos >= 1024 || yPos < 0 || yPos >= 1024;
  xPos &= 0x3ff;
  yPos &= 0x3ff;
  if (!PPU_m7largeField(ppu)) outsideMap = false;
  uint8_t tile = outsideMap ? 0 : ppu->vram[(yPos >> 3) * 128 + (xPos >> 3)] & 0xff;
  uint8_t pixel = outsideMap && !PPU_m7charFill(ppu) ? 0 : ppu->vram[tile * 64 + (yPos & 7) * 8 + (xPos & 7)] >> 8;
  if (layer == 1) {
    if (((bool)(pixel & 0x80)) != priority) return 0;
    return pixel & 0x7f;
  }
  return pixel;
}

static bool ppu_getWindowState(Ppu *ppu, int layer, int x) {
  uint32 winflags = GET_WINDOW_FLAGS(ppu, layer);

  if (!(winflags & kWindow1Enabled) && !(winflags & kWindow2Enabled)) {
    return false;
  }
  int min_x = -ppu->extraLeftCur;
  int max_x = 256 + ppu->extraRightCur - 1;
  int w1_left = (ppu->window1left <= 8 && ppu->window1right >= 247) ? min_x : ((ppu->window1left == 0) ? min_x : ppu->window1left);
  int w1_right = (ppu->window1left <= 8 && ppu->window1right >= 247) ? max_x : ((ppu->window1right >= 254) ? max_x : ppu->window1right);
  int w2_left = (ppu->window2left <= 8 && ppu->window2right >= 247) ? min_x : ((ppu->window2left == 0) ? min_x : ppu->window2left);
  int w2_right = (ppu->window2left <= 8 && ppu->window2right >= 247) ? max_x : ((ppu->window2right >= 254) ? max_x : ppu->window2right);
  if ((winflags & kWindow1Enabled) && !(winflags & kWindow2Enabled)) {
    bool test = x >= w1_left && x <= w1_right;
    return (winflags & kWindow1Inversed) ? !test : test;
  }
  if (!(winflags & kWindow1Enabled) && (winflags & kWindow2Enabled)) {
    bool test = x >= w2_left && x <= w2_right;
    return (winflags & kWindow2Inversed) ? !test : test;
  }
  bool test1 = x >= w1_left && x <= w1_right;
  bool test2 = x >= w2_left && x <= w2_right;
  if (winflags & kWindow1Inversed) test1 = !test1;
  if (winflags & kWindow2Inversed) test2 = !test2;
  switch (ppu->wbgobjlog >> (layer * 2) & 3) {
  case 0: return test1 || test2;
  case 1: return test1 && test2;
  case 2: return test1 != test2;
  case 3: return test1 == test2;
  }
  return false;
}


/* === dsp.c === */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <limits.h>



#define MY_CHANGES 1

static const int rateValues[32] = {
  0, 2048, 1536, 1280, 1024, 768, 640, 512,
  384, 320, 256, 192, 160, 128, 96, 80,
  64, 48, 40, 32, 24, 20, 16, 12,
  10, 8, 6, 5, 4, 3, 2, 1
};

static const int gaussValues[512] = {
  0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000,
  0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x002, 0x002, 0x002, 0x002, 0x002,
  0x002, 0x002, 0x003, 0x003, 0x003, 0x003, 0x003, 0x004, 0x004, 0x004, 0x004, 0x004, 0x005, 0x005, 0x005, 0x005,
  0x006, 0x006, 0x006, 0x006, 0x007, 0x007, 0x007, 0x008, 0x008, 0x008, 0x009, 0x009, 0x009, 0x00A, 0x00A, 0x00A,
  0x00B, 0x00B, 0x00B, 0x00C, 0x00C, 0x00D, 0x00D, 0x00E, 0x00E, 0x00F, 0x00F, 0x00F, 0x010, 0x010, 0x011, 0x011,
  0x012, 0x013, 0x013, 0x014, 0x014, 0x015, 0x015, 0x016, 0x017, 0x017, 0x018, 0x018, 0x019, 0x01A, 0x01B, 0x01B,
  0x01C, 0x01D, 0x01D, 0x01E, 0x01F, 0x020, 0x020, 0x021, 0x022, 0x023, 0x024, 0x024, 0x025, 0x026, 0x027, 0x028,
  0x029, 0x02A, 0x02B, 0x02C, 0x02D, 0x02E, 0x02F, 0x030, 0x031, 0x032, 0x033, 0x034, 0x035, 0x036, 0x037, 0x038,
  0x03A, 0x03B, 0x03C, 0x03D, 0x03E, 0x040, 0x041, 0x042, 0x043, 0x045, 0x046, 0x047, 0x049, 0x04A, 0x04C, 0x04D,
  0x04E, 0x050, 0x051, 0x053, 0x054, 0x056, 0x057, 0x059, 0x05A, 0x05C, 0x05E, 0x05F, 0x061, 0x063, 0x064, 0x066,
  0x068, 0x06A, 0x06B, 0x06D, 0x06F, 0x071, 0x073, 0x075, 0x076, 0x078, 0x07A, 0x07C, 0x07E, 0x080, 0x082, 0x084,
  0x086, 0x089, 0x08B, 0x08D, 0x08F, 0x091, 0x093, 0x096, 0x098, 0x09A, 0x09C, 0x09F, 0x0A1, 0x0A3, 0x0A6, 0x0A8,
  0x0AB, 0x0AD, 0x0AF, 0x0B2, 0x0B4, 0x0B7, 0x0BA, 0x0BC, 0x0BF, 0x0C1, 0x0C4, 0x0C7, 0x0C9, 0x0CC, 0x0CF, 0x0D2,
  0x0D4, 0x0D7, 0x0DA, 0x0DD, 0x0E0, 0x0E3, 0x0E6, 0x0E9, 0x0EC, 0x0EF, 0x0F2, 0x0F5, 0x0F8, 0x0FB, 0x0FE, 0x101,
  0x104, 0x107, 0x10B, 0x10E, 0x111, 0x114, 0x118, 0x11B, 0x11E, 0x122, 0x125, 0x129, 0x12C, 0x130, 0x133, 0x137,
  0x13A, 0x13E, 0x141, 0x145, 0x148, 0x14C, 0x150, 0x153, 0x157, 0x15B, 0x15F, 0x162, 0x166, 0x16A, 0x16E, 0x172,
  0x176, 0x17A, 0x17D, 0x181, 0x185, 0x189, 0x18D, 0x191, 0x195, 0x19A, 0x19E, 0x1A2, 0x1A6, 0x1AA, 0x1AE, 0x1B2,
  0x1B7, 0x1BB, 0x1BF, 0x1C3, 0x1C8, 0x1CC, 0x1D0, 0x1D5, 0x1D9, 0x1DD, 0x1E2, 0x1E6, 0x1EB, 0x1EF, 0x1F3, 0x1F8,
  0x1FC, 0x201, 0x205, 0x20A, 0x20F, 0x213, 0x218, 0x21C, 0x221, 0x226, 0x22A, 0x22F, 0x233, 0x238, 0x23D, 0x241,
  0x246, 0x24B, 0x250, 0x254, 0x259, 0x25E, 0x263, 0x267, 0x26C, 0x271, 0x276, 0x27B, 0x280, 0x284, 0x289, 0x28E,
  0x293, 0x298, 0x29D, 0x2A2, 0x2A6, 0x2AB, 0x2B0, 0x2B5, 0x2BA, 0x2BF, 0x2C4, 0x2C9, 0x2CE, 0x2D3, 0x2D8, 0x2DC,
  0x2E1, 0x2E6, 0x2EB, 0x2F0, 0x2F5, 0x2FA, 0x2FF, 0x304, 0x309, 0x30E, 0x313, 0x318, 0x31D, 0x322, 0x326, 0x32B,
  0x330, 0x335, 0x33A, 0x33F, 0x344, 0x349, 0x34E, 0x353, 0x357, 0x35C, 0x361, 0x366, 0x36B, 0x370, 0x374, 0x379,
  0x37E, 0x383, 0x388, 0x38C, 0x391, 0x396, 0x39B, 0x39F, 0x3A4, 0x3A9, 0x3AD, 0x3B2, 0x3B7, 0x3BB, 0x3C0, 0x3C5,
  0x3C9, 0x3CE, 0x3D2, 0x3D7, 0x3DC, 0x3E0, 0x3E5, 0x3E9, 0x3ED, 0x3F2, 0x3F6, 0x3FB, 0x3FF, 0x403, 0x408, 0x40C,
  0x410, 0x415, 0x419, 0x41D, 0x421, 0x425, 0x42A, 0x42E, 0x432, 0x436, 0x43A, 0x43E, 0x442, 0x446, 0x44A, 0x44E,
  0x452, 0x455, 0x459, 0x45D, 0x461, 0x465, 0x468, 0x46C, 0x470, 0x473, 0x477, 0x47A, 0x47E, 0x481, 0x485, 0x488,
  0x48C, 0x48F, 0x492, 0x496, 0x499, 0x49C, 0x49F, 0x4A2, 0x4A6, 0x4A9, 0x4AC, 0x4AF, 0x4B2, 0x4B5, 0x4B7, 0x4BA,
  0x4BD, 0x4C0, 0x4C3, 0x4C5, 0x4C8, 0x4CB, 0x4CD, 0x4D0, 0x4D2, 0x4D5, 0x4D7, 0x4D9, 0x4DC, 0x4DE, 0x4E0, 0x4E3,
  0x4E5, 0x4E7, 0x4E9, 0x4EB, 0x4ED, 0x4EF, 0x4F1, 0x4F3, 0x4F5, 0x4F6, 0x4F8, 0x4FA, 0x4FB, 0x4FD, 0x4FF, 0x500,
  0x502, 0x503, 0x504, 0x506, 0x507, 0x508, 0x50A, 0x50B, 0x50C, 0x50D, 0x50E, 0x50F, 0x510, 0x511, 0x511, 0x512,
  0x513, 0x514, 0x514, 0x515, 0x516, 0x516, 0x517, 0x517, 0x517, 0x518, 0x518, 0x518, 0x518, 0x518, 0x519, 0x519
};

static void dsp_cycleChannel(Dsp* dsp, int ch);
static void dsp_handleEcho(Dsp* dsp, int* outputL, int* outputR);
static void dsp_handleGain(Dsp* dsp, int ch);
static void dsp_decodeBrr(Dsp* dsp, int ch);
static int16_t dsp_getSample(Dsp* dsp, int ch, int sampleNum, int offset);
static void dsp_handleNoise(Dsp* dsp);

Dsp* dsp_init(uint8_t *apu_ram) {
  Dsp* dsp = (Dsp*)malloc(sizeof(Dsp));
  dsp->apu_ram = apu_ram;
  return dsp;
}

void dsp_free(Dsp* dsp) {
  free(dsp);
}

void dsp_reset(Dsp* dsp) {
  memset(dsp->ram, 0, sizeof(dsp->ram));
  dsp->ram[ENDX] = 0xff; // set ENDX bit for all channels
  for(int i = 0; i < 8; i++) {
    dsp->channel[i].pitch = 0;
    dsp->channel[i].pitchCounter = 0;
    dsp->channel[i].pitchModulation = false;
    memset(dsp->channel[i].decodeBuffer, 0, sizeof(dsp->channel[i].decodeBuffer));
    dsp->channel[i].srcn = 0;
    dsp->channel[i].decodeOffset = 0;
    dsp->channel[i].previousFlags = 0;
    dsp->channel[i].old = 0;
    dsp->channel[i].older = 0;
    dsp->channel[i].useNoise = false;
    memset(dsp->channel[i].adsrRates, 0, sizeof(dsp->channel[i].adsrRates));
    dsp->channel[i].rateCounter = 0;
    dsp->channel[i].adsrState = 0;
    dsp->channel[i].sustainLevel = 0;
    dsp->channel[i].useGain = false;
    dsp->channel[i].gainMode = 0;
    dsp->channel[i].directGain = false;
    dsp->channel[i].gainValue = 0;
    dsp->channel[i].gain = 0;
    dsp->channel[i].keyOn = false;
    dsp->channel[i].keyOff = false;
    dsp->channel[i].sampleOut = 0;
    dsp->channel[i].volumeL = 0;
    dsp->channel[i].volumeR = 0;
    dsp->channel[i].echoEnable = false;
  }
  dsp->dirPage = 0;
  dsp->evenCycle = false;
  dsp->mute = true;
  dsp->reset = true;
  dsp->masterVolumeL = 0;
  dsp->masterVolumeR = 0;
  dsp->noiseSample = -0x4000;
  dsp->noiseRate = 0;
  dsp->noiseCounter = 0;
  dsp->echoWrites = false;
  dsp->echoVolumeL = 0;
  dsp->echoVolumeR = 0;
  dsp->feedbackVolume = 0;
  dsp->echoBufferAdr = 0;
  dsp->echoDelay = 1;
  dsp->echoRemain = 1;
  dsp->echoBufferIndex = 0;
  dsp->firBufferIndex = 0;
  memset(dsp->firValues, 0, sizeof(dsp->firValues));
  memset(dsp->firBufferL, 0, sizeof(dsp->firBufferL));
  memset(dsp->firBufferR, 0, sizeof(dsp->firBufferR));
  memset(dsp->sampleBuffer, 0, sizeof(dsp->sampleBuffer));
  dsp->sampleOffset = 0;
}

void dsp_saveload(Dsp *dsp, SaveLoadFunc *func, void *ctx) {
  func(ctx, &dsp->ram, sizeof(Dsp) - offsetof(Dsp, ram));
}

void dsp_cycle(Dsp* dsp) {
  int totalL = 0;
  int totalR = 0;
  for(int i = 0; i < 8; i++) {
    dsp_cycleChannel(dsp, i);
    totalL += (dsp->channel[i].sampleOut * dsp->channel[i].volumeL) >> 6;
    totalR += (dsp->channel[i].sampleOut * dsp->channel[i].volumeR) >> 6;
    totalL = totalL < -0x8000 ? -0x8000 : (totalL > 0x7fff ? 0x7fff : totalL); // clamp 16-bit
    totalR = totalR < -0x8000 ? -0x8000 : (totalR > 0x7fff ? 0x7fff : totalR); // clamp 16-bit
  }
  totalL = (totalL * dsp->masterVolumeL) >> 7;
  totalR = (totalR * dsp->masterVolumeR) >> 7;
  totalL = totalL < -0x8000 ? -0x8000 : (totalL > 0x7fff ? 0x7fff : totalL); // clamp 16-bit
  totalR = totalR < -0x8000 ? -0x8000 : (totalR > 0x7fff ? 0x7fff : totalR); // clamp 16-bit
  dsp_handleEcho(dsp, &totalL, &totalR);
  if(dsp->mute) {
    totalL = 0;
    totalR = 0;
  }
  dsp_handleNoise(dsp);
  // put it in the samplebuffer, if space
  if (dsp->sampleOffset < 534) {
    dsp->sampleBuffer[dsp->sampleOffset * 2] = totalL;
    dsp->sampleBuffer[dsp->sampleOffset * 2 + 1] = totalR;
    dsp->sampleOffset++;
  }
  dsp->evenCycle = !dsp->evenCycle;
}

static void dsp_handleEcho(Dsp* dsp, int* outputL, int* outputR) {
  // get value out of ram
  uint16_t adr = dsp->echoBufferAdr + dsp->echoBufferIndex * 4;
  dsp->firBufferL[dsp->firBufferIndex] = (
    dsp->apu_ram[adr] + (dsp->apu_ram[(adr + 1) & 0xffff] << 8)
  );
  dsp->firBufferL[dsp->firBufferIndex] >>= 1;
  dsp->firBufferR[dsp->firBufferIndex] = (
    dsp->apu_ram[(adr + 2) & 0xffff] + (dsp->apu_ram[(adr + 3) & 0xffff] << 8)
  );
  dsp->firBufferR[dsp->firBufferIndex] >>= 1;
  // calculate FIR-sum
  int sumL = 0, sumR = 0;
  for(int i = 0; i < 8; i++) {
    sumL += (dsp->firBufferL[(dsp->firBufferIndex + i + 1) & 0x7] * dsp->firValues[i]) >> 6;
    sumR += (dsp->firBufferR[(dsp->firBufferIndex + i + 1) & 0x7] * dsp->firValues[i]) >> 6;
    if(i == 6) {
      // clip to 16-bit before last addition
      sumL = ((int16_t) (sumL & 0xffff)); // clip 16-bit
      sumR = ((int16_t) (sumR & 0xffff)); // clip 16-bit
    }
  }
  sumL = sumL < -0x8000 ? -0x8000 : (sumL > 0x7fff ? 0x7fff : sumL); // clamp 16-bit
  sumR = sumR < -0x8000 ? -0x8000 : (sumR > 0x7fff ? 0x7fff : sumR); // clamp 16-bit
  // modify output with sum
  int outL = *outputL + ((sumL * dsp->echoVolumeL) >> 7);
  int outR = *outputR + ((sumR * dsp->echoVolumeR) >> 7);
  *outputL = outL < -0x8000 ? -0x8000 : (outL > 0x7fff ? 0x7fff : outL); // clamp 16-bit
  *outputR = outR < -0x8000 ? -0x8000 : (outR > 0x7fff ? 0x7fff : outR); // clamp 16-bit
  // get echo input
  int inL = 0, inR = 0;
  for(int i = 0; i < 8; i++) {
    if(dsp->channel[i].echoEnable) {
      inL += (dsp->channel[i].sampleOut * dsp->channel[i].volumeL) >> 6;
      inR += (dsp->channel[i].sampleOut * dsp->channel[i].volumeR) >> 6;
      inL = inL < -0x8000 ? -0x8000 : (inL > 0x7fff ? 0x7fff : inL); // clamp 16-bit
      inR = inR < -0x8000 ? -0x8000 : (inR > 0x7fff ? 0x7fff : inR); // clamp 16-bit
    }
  }
  // write this to ram
  inL += (sumL * dsp->feedbackVolume) >> 7;
  inR += (sumR * dsp->feedbackVolume) >> 7;
  inL = inL < -0x8000 ? -0x8000 : (inL > 0x7fff ? 0x7fff : inL); // clamp 16-bit
  inR = inR < -0x8000 ? -0x8000 : (inR > 0x7fff ? 0x7fff : inR); // clamp 16-bit
  inL &= 0xfffe;
  inR &= 0xfffe;
  if(dsp->echoWrites) {
    dsp->apu_ram[adr] = inL & 0xff;
    dsp->apu_ram[(adr + 1) & 0xffff] = inL >> 8;
    dsp->apu_ram[(adr + 2) & 0xffff] = inR & 0xff;
    dsp->apu_ram[(adr + 3) & 0xffff] = inR >> 8;
  }
  // handle indexes
  dsp->firBufferIndex++;
  dsp->firBufferIndex &= 7;
  dsp->echoBufferIndex++;
  dsp->echoRemain--;
  if(dsp->echoRemain == 0) {
    dsp->echoRemain = dsp->echoDelay;
    dsp->echoBufferIndex = 0;
  }
}

static void dsp_cycleChannel(Dsp* dsp, int ch) {
  // handle pitch counter
  uint16_t pitch = dsp->channel[ch].pitch;
  if(ch > 0 && dsp->channel[ch].pitchModulation) {
    int factor = (dsp->channel[ch - 1].sampleOut >> 4) + 0x400;
    pitch = (pitch * factor) >> 10;
    if(pitch > 0x3fff) pitch = 0x3fff;
  }
  int newCounter = dsp->channel[ch].pitchCounter + pitch;
  if(newCounter > 0xffff) {
    // next sample
    dsp_decodeBrr(dsp, ch);
  }
  dsp->channel[ch].pitchCounter = newCounter;
  int16_t sample = 0;
  if(dsp->channel[ch].useNoise) {
    sample = dsp->noiseSample;
  } else {
    sample = dsp_getSample(dsp, ch, dsp->channel[ch].pitchCounter >> 12, (dsp->channel[ch].pitchCounter >> 4) & 0xff);
  }
#if !MY_CHANGES
  if(dsp->evenCycle) {
    // handle keyon/off (every other cycle)
    if(dsp->channel[ch].keyOff) {
      // go to release
      dsp->channel[ch].adsrState = 4;
    } else if(dsp->channel[ch].keyOn) {
      dsp->channel[ch].keyOn = false;
      // restart current sample
      dsp->channel[ch].previousFlags = 0;
      uint16_t samplePointer = dsp->dirPage + 4 * dsp->channel[ch].srcn;
      dsp->channel[ch].decodeOffset = dsp->apu_ram[samplePointer];
      dsp->channel[ch].decodeOffset |= dsp->apu_ram[(samplePointer + 1) & 0xffff] << 8;
      memset(dsp->channel[ch].decodeBuffer, 0, sizeof(dsp->channel[ch].decodeBuffer));
      dsp->channel[ch].gain = 0;
      dsp->channel[ch].adsrState = dsp->channel[ch].useGain ? 3 : 0;
    }
  }
#endif
  // handle reset
  if(dsp->reset) {
    dsp->channel[ch].adsrState = 4;
    dsp->channel[ch].gain = 0;
  }
  // handle envelope/adsr
  bool doingDirectGain = dsp->channel[ch].adsrState != 4 && dsp->channel[ch].useGain && dsp->channel[ch].directGain;
  uint16_t rate = dsp->channel[ch].adsrState == 4 ? 0 : dsp->channel[ch].adsrRates[dsp->channel[ch].adsrState];
  if(dsp->channel[ch].adsrState != 4 && !doingDirectGain && rate != 0) {
    dsp->channel[ch].rateCounter++;
  }
  if(dsp->channel[ch].adsrState == 4 || (!doingDirectGain && dsp->channel[ch].rateCounter >= rate && rate != 0)) {
    if(dsp->channel[ch].adsrState != 4) dsp->channel[ch].rateCounter = 0;
    dsp_handleGain(dsp, ch);
  }
  if(doingDirectGain) dsp->channel[ch].gain = dsp->channel[ch].gainValue;
  // set outputs
  dsp->ram[(ch << 4) | 8] = dsp->channel[ch].gain >> 4;
  sample = (sample * dsp->channel[ch].gain) >> 11;
  dsp->ram[(ch << 4) | 9] = sample >> 7;
  dsp->channel[ch].sampleOut = sample;
}

static void dsp_handleGain(Dsp* dsp, int ch) {
  switch(dsp->channel[ch].adsrState) {
    case 0: { // attack
      uint16_t rate = dsp->channel[ch].adsrRates[dsp->channel[ch].adsrState];
      dsp->channel[ch].gain += rate == 1 ? 1024 : 32;
      if(dsp->channel[ch].gain >= 0x7e0) dsp->channel[ch].adsrState = 1;
      if(dsp->channel[ch].gain > 0x7ff) dsp->channel[ch].gain = 0x7ff;
      break;
    }
    case 1: { // decay
      dsp->channel[ch].gain -= ((dsp->channel[ch].gain - 1) >> 8) + 1;
      if(dsp->channel[ch].gain < dsp->channel[ch].sustainLevel) dsp->channel[ch].adsrState = 2;
      break;
    }
    case 2: { // sustain
      dsp->channel[ch].gain -= ((dsp->channel[ch].gain - 1) >> 8) + 1;
      break;
    }
    case 3: { // gain
      switch(dsp->channel[ch].gainMode) {
        case 0: { // linear decrease
          dsp->channel[ch].gain -= 32;
          // decreasing below 0 will underflow to above 0x7ff
          if(dsp->channel[ch].gain > 0x7ff) dsp->channel[ch].gain = 0;
          break;
        }
        case 1: { // exponential decrease
          dsp->channel[ch].gain -= ((dsp->channel[ch].gain - 1) >> 8) + 1;
          break;
        }
        case 2: { // linear increase
          dsp->channel[ch].gain += 32;
          if(dsp->channel[ch].gain > 0x7ff) dsp->channel[ch].gain = 0;
          break;
        }
        case 3: { // bent increase
          dsp->channel[ch].gain += dsp->channel[ch].gain < 0x600 ? 32 : 8;
          if(dsp->channel[ch].gain > 0x7ff) dsp->channel[ch].gain = 0;
          break;
        }
      }
      break;
    }
    case 4: { // release
      dsp->channel[ch].gain -= 8;
      // decreasing below 0 will underflow to above 0x7ff
      if(dsp->channel[ch].gain > 0x7ff) dsp->channel[ch].gain = 0;
      break;
    }
  }
}

static int16_t dsp_getSample(Dsp* dsp, int ch, int sampleNum, int offset) {
  int16_t news = dsp->channel[ch].decodeBuffer[sampleNum + 3];
  int16_t olds = dsp->channel[ch].decodeBuffer[sampleNum + 2];
  int16_t olders = dsp->channel[ch].decodeBuffer[sampleNum + 1];
  int16_t oldests = dsp->channel[ch].decodeBuffer[sampleNum];
  int out = (gaussValues[0xff - offset] * oldests) >> 10;
  out += (gaussValues[0x1ff - offset] * olders) >> 10;
  out += (gaussValues[0x100 + offset] * olds) >> 10;
  out = ((int16_t) (out & 0xffff)); // clip 16-bit
  out += (gaussValues[offset] * news) >> 10;
  out = out < -0x8000 ? -0x8000 : (out > 0x7fff ? 0x7fff : out); // clamp 16-bit
  return out >> 1;
}

static void dsp_decodeBrr(Dsp* dsp, int ch) {
  // copy last 3 samples (16-18) to first 3 for interpolation
  dsp->channel[ch].decodeBuffer[0] = dsp->channel[ch].decodeBuffer[16];
  dsp->channel[ch].decodeBuffer[1] = dsp->channel[ch].decodeBuffer[17];
  dsp->channel[ch].decodeBuffer[2] = dsp->channel[ch].decodeBuffer[18];
  // handle flags from previous block
  if(dsp->channel[ch].previousFlags == 1 || dsp->channel[ch].previousFlags == 3) {
    // loop sample
    uint16_t samplePointer = dsp->dirPage + 4 * dsp->channel[ch].srcn;
    dsp->channel[ch].decodeOffset = dsp->apu_ram[(samplePointer + 2) & 0xffff];
    dsp->channel[ch].decodeOffset |= (dsp->apu_ram[(samplePointer + 3) & 0xffff]) << 8;
    if(dsp->channel[ch].previousFlags == 1) {
      // also release and clear gain
      dsp->channel[ch].adsrState = 4;
      dsp->channel[ch].gain = 0;
    }
    dsp->ram[ENDX] |= 1 << ch; // set ENDX bit for channel
  }
  uint8_t header = dsp->apu_ram[dsp->channel[ch].decodeOffset++];
  int shift = header >> 4;
  int filter = (header & 0xc) >> 2;
  dsp->channel[ch].previousFlags = header & 0x3;
  uint8_t curByte = 0;
  int old = dsp->channel[ch].old;
  int older = dsp->channel[ch].older;
  for(int i = 0; i < 16; i++) {
    int s = 0;
    if(i & 1) {
      s = curByte & 0xf;
    } else {
      curByte = dsp->apu_ram[dsp->channel[ch].decodeOffset++];
      s = curByte >> 4;
    }
    if(s > 7) s -= 16;
    if(shift <= 0xc) {
      s = (s << shift) >> 1;
    } else {
      s = (s >> 3) << 12;
    }
    switch(filter) {
      case 1: s += old + (-old >> 4); break;
      case 2: s += 2 * old + ((3 * -old) >> 5) - older + (older >> 4); break;
      case 3: s += 2 * old + ((13 * -old) >> 6) - older + ((3 * older) >> 4); break;
    }
    s = s < -0x8000 ? -0x8000 : (s > 0x7fff ? 0x7fff : s); // clamp 16-bit
    s = ((int16_t) ((s & 0x7fff) << 1)) >> 1; // clip 15-bit
    older = old;
    old = s;
    dsp->channel[ch].decodeBuffer[i + 3] = s;
  }
  dsp->channel[ch].older = older;
  dsp->channel[ch].old = old;
}

static void dsp_handleNoise(Dsp* dsp) {
  if(dsp->noiseRate != 0) {
    dsp->noiseCounter++;
  }
  if(dsp->noiseCounter >= dsp->noiseRate && dsp->noiseRate != 0) {
    int bit = (dsp->noiseSample & 1) ^ ((dsp->noiseSample >> 1) & 1);
    dsp->noiseSample = ((dsp->noiseSample >> 1) & 0x3fff) | (bit << 14);
    dsp->noiseSample = ((int16_t) ((dsp->noiseSample & 0x7fff) << 1)) >> 1;
    dsp->noiseCounter = 0;
  }
}

uint8_t dsp_read(Dsp* dsp, uint8_t adr) {
  return dsp->ram[adr];
}

void dsp_write(Dsp *dsp, uint8_t adr, uint8_t val) {
  int ch = adr >> 4;
  switch (adr) {
  case V0VOLL:
  case V1VOLL:
  case V2VOLL:
  case V3VOLL:
  case V4VOLL:
  case V5VOLL:
  case V6VOLL:
  case V7VOLL: {
    dsp->channel[ch].volumeL = val;
    break;
  }
  case V0VOLR:
  case V1VOLR:
  case V2VOLR:
  case V3VOLR:
  case V4VOLR:
  case V5VOLR:
  case V6VOLR:
  case V7VOLR: {
    dsp->channel[ch].volumeR = val;
    break;
  }
  case V0PITCHL:
  case V1PL:
  case V2PL:
  case V3PL:
  case V4PL:
  case V5PL:
  case V6PL:
  case V7PL: {
    dsp->channel[ch].pitch = (dsp->channel[ch].pitch & 0x3f00) | val;
    break;
  }
  case V0PITCHH:
  case V1PH:
  case V2PH:
  case V3PH:
  case V4PH:
  case V5PH:
  case V6PH:
  case V7PH: {
    dsp->channel[ch].pitch =
        ((dsp->channel[ch].pitch & 0x00ff) | (val << 8)) & 0x3fff;
    break;
  }
  case V0SRCN:
  case V1SRCN:
  case V2SRCN:
  case V3SRCN:
  case V4SRCN:
  case V5SRCN:
  case V6SRCN:
  case V7SRCN: {
    dsp->channel[ch].srcn = val;
    break;
  }
  case V0ADSR1:
  case V1ADSR1:
  case V2ADSR1:
  case V3ADSR1:
  case V4ADSR1:
  case V5ADSR1:
  case V6ADSR1:
  case V7ADSR1: {
    dsp->channel[ch].adsrRates[0] = rateValues[(val & 0xf) * 2 + 1];
    dsp->channel[ch].adsrRates[1] = rateValues[((val & 0x70) >> 4) * 2 + 16];
    dsp->channel[ch].useGain = (val & 0x80) == 0;
    break;
  }
  case V0ADSR2:
  case V1ADSR2:
  case V2ADSR2:
  case V3ADSR2:
  case V4ADSR2:
  case V5ADSR2:
  case V6ADSR2:
  case V7ADSR2: {
    dsp->channel[ch].adsrRates[2] = rateValues[val & 0x1f];
    dsp->channel[ch].sustainLevel = (((val & 0xe0) >> 5) + 1) * 0x100;
    break;
  }
  case V0GAIN:
  case V1GAIN:
  case V2GAIN:
  case V3GAIN:
  case V4GAIN:
  case V5GAIN:
  case V6GAIN:
  case V7GAIN: {
    dsp->channel[ch].directGain = (val & 0x80) == 0;
    if (val & 0x80) {
      dsp->channel[ch].gainMode = (val & 0x60) >> 5;
      dsp->channel[ch].adsrRates[3] = rateValues[val & 0x1f];
    } else {
      dsp->channel[ch].gainValue = (val & 0x7f) * 16;
    }
    break;
  }
  case MVOLL: {
    dsp->masterVolumeL = val;
    break;
  }
  case MVOLR: {
    dsp->masterVolumeR = val;
    break;
  }
  case EVOLL: {
    dsp->echoVolumeL = val;
    break;
  }
  case EVOLR: {
    dsp->echoVolumeR = val;
    break;
  }
  case KON: {
    for (int ch = 0; ch < 8; ch++) {
      dsp->channel[ch].keyOn = val & (1 << ch);
#if MY_CHANGES

      if (dsp->channel[ch].keyOn) {
        dsp->channel[ch].keyOn = false;
        // restart current sample
        dsp->channel[ch].previousFlags = 0;
        uint16_t samplePointer = dsp->dirPage + 4 * dsp->channel[ch].srcn;
        dsp->channel[ch].decodeOffset = dsp->apu_ram[samplePointer];
        dsp->channel[ch].decodeOffset |=
            dsp->apu_ram[(samplePointer + 1) & 0xffff] << 8;
        memset(dsp->channel[ch].decodeBuffer, 0,
               sizeof(dsp->channel[ch].decodeBuffer));
        dsp->channel[ch].gain = 0;
        dsp->channel[ch].adsrState = dsp->channel[ch].useGain ? 3 : 0;
      }
#endif
    }
    break;
  }
  case KOF: {
    for (int ch = 0; ch < 8; ch++) {
      dsp->channel[ch].keyOff = val & (1 << ch);
#if MY_CHANGES
      if (dsp->channel[ch].keyOff) {
        // go to release
        dsp->channel[ch].adsrState = 4;
      }
#endif
    }

    break;
  }
  case FLG: {
    dsp->reset = val & 0x80;
    dsp->mute = val & 0x40;
    dsp->echoWrites = (val & 0x20) == 0;
    dsp->noiseRate = rateValues[val & 0x1f];
    break;
  }
  case ENDX: {
    val = 0; // any write clears ENDx
    break;
  }
  case EFB: {
    dsp->feedbackVolume = val;
    break;
  }
  case PMON: {
    for (int i = 0; i < 8; i++) {
      dsp->channel[i].pitchModulation = val & (1 << i);
    }
    break;
  }
  case NON: {
    for (int i = 0; i < 8; i++) {
      dsp->channel[i].useNoise = val & (1 << i);
    }
    break;
  }
  case EON: {
    for (int i = 0; i < 8; i++) {
      dsp->channel[i].echoEnable = val & (1 << i);
    }
    break;
  }
  case DIR: {
    dsp->dirPage = val << 8;
    break;
  }
  case ESA: {
    dsp->echoBufferAdr = val << 8;
    break;
  }
  case EDL: {
    dsp->echoDelay =
        (val & 0xf) * 512; // 2048-byte steps, stereo sample is 4 bytes
    if (dsp->echoDelay == 0)
      dsp->echoDelay = 1;
    break;
  }
  case FIR0:
  case FIR1:
  case FIR2:
  case FIR3:
  case FIR4:
  case FIR5:
  case FIR6:
  case FIR7: {
    dsp->firValues[ch] = val;
    break;
  }
  }
  dsp->ram[adr] = val;
}

void dsp_getSamples(Dsp* dsp, int16_t* sampleData, int samplesPerFrame, int numChannels) {
  // resample from 534 samples per frame to wanted value
  float adder = 534.0f / samplesPerFrame;
  float location = 0.0f;

  if (numChannels == 1) {
    for (int i = 0; i < samplesPerFrame; i++) {
      int sampleL = dsp->sampleBuffer[((int)location) * 2];
      int sampleR = dsp->sampleBuffer[((int)location) * 2 + 1];
      sampleData[i] = (sampleL + sampleR) >> 1;
      location += adder;
    }
  } else {
    for (int i = 0; i < samplesPerFrame; i++) {
      int sampleL = dsp->sampleBuffer[((int)location) * 2];
      int sampleR = dsp->sampleBuffer[((int)location) * 2 + 1];
      sampleData[i * 2] = sampleL;
      sampleData[i * 2 + 1] = sampleR;
      location += adder;
    }
  }
  dsp->sampleOffset = 0;
}


/* === spc.c === */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>



static const int spcCyclesPerOpcode[256] = {
  2, 8, 4, 5, 3, 4, 3, 6, 2, 6, 5, 4, 5, 4, 6, 8,
  2, 8, 4, 5, 4, 5, 5, 6, 5, 5, 6, 5, 2, 2, 4, 6,
  2, 8, 4, 5, 3, 4, 3, 6, 2, 6, 5, 4, 5, 4, 5, 4,
  2, 8, 4, 5, 4, 5, 5, 6, 5, 5, 6, 5, 2, 2, 3, 8,
  2, 8, 4, 5, 3, 4, 3, 6, 2, 6, 4, 4, 5, 4, 6, 6,
  2, 8, 4, 5, 4, 5, 5, 6, 5, 5, 4, 5, 2, 2, 4, 3,
  2, 8, 4, 5, 3, 4, 3, 6, 2, 6, 4, 4, 5, 4, 5, 5,
  2, 8, 4, 5, 4, 5, 5, 6, 5, 5, 5, 5, 2, 2, 3, 6,
  2, 8, 4, 5, 3, 4, 3, 6, 2, 6, 5, 4, 5, 2, 4, 5,
  2, 8, 4, 5, 4, 5, 5, 6, 5, 5, 5, 5, 2, 2, 12,5,
  2, 8, 4, 5, 3, 4, 3, 6, 2, 6, 4, 4, 5, 2, 4, 4,
  2, 8, 4, 5, 4, 5, 5, 6, 5, 5, 5, 5, 2, 2, 3, 4,
  2, 8, 4, 5, 4, 5, 4, 7, 2, 5, 6, 4, 5, 2, 4, 9,
  2, 8, 4, 5, 5, 6, 6, 7, 4, 5, 5, 5, 2, 2, 6, 3,
  2, 8, 4, 5, 3, 4, 3, 6, 2, 4, 5, 3, 4, 3, 4, 3,
  2, 8, 4, 5, 4, 5, 5, 6, 3, 4, 5, 4, 2, 2, 4, 3
};

static uint8_t spc_read(Spc* spc, uint16_t adr);
static void spc_write(Spc* spc, uint16_t adr, uint8_t val);
static uint8_t spc_readOpcode(Spc* spc);
static uint16_t spc_readOpcodeWord(Spc* spc);
static uint8_t spc_getFlags(Spc* spc);
static void spc_setFlags(Spc* spc, uint8_t value);
static void spc_setZN(Spc* spc, uint8_t value);
static void spc_doBranch(Spc* spc, uint8_t value, bool check);
static uint8_t spc_pullByte(Spc* spc);
static void spc_pushByte(Spc* spc, uint8_t value);
static uint16_t spc_pullWord(Spc* spc);
static void spc_pushWord(Spc* spc, uint16_t value);
static uint16_t spc_readWord(Spc* spc, uint16_t adrl, uint16_t adrh);
static void spc_writeWord(Spc* spc, uint16_t adrl, uint16_t adrh, uint16_t value);
static void spc_doOpcode(Spc* spc, uint8_t opcode);

// addressing modes and opcode functions not declared, only used after defintions

static uint8_t spc_read(Spc* spc, uint16_t adr) {
  return apu_cpuRead(spc->apu, adr);
}

static void spc_write(Spc* spc, uint16_t adr, uint8_t val) {
  if (0 && adr == 0x5e)
    printf("writing to vol_dirty %d\n", val);
  apu_cpuWrite(spc->apu, adr, val);
}

Spc* spc_init(Apu* apu) {
  Spc* spc = (Spc * )malloc(sizeof(Spc));
  spc->apu = apu;
  return spc;
}

void spc_free(Spc* spc) {
  free(spc);
}

void spc_reset(Spc* spc) {
  spc->a = 0;
  spc->x = 0;
  spc->y = 0;
  spc->sp = 0;
  spc->pc = spc_read(spc, 0xfffe) | (spc_read(spc, 0xffff) << 8);
  spc->c = false;
  spc->z = false;
  spc->v = false;
  spc->n = false;
  spc->i = false;
  spc->h = false;
  spc->p = false;
  spc->b = false;
  spc->stopped = false;
  spc->cyclesUsed = 0;
}

void spc_saveload(Spc *spc, SaveLoadFunc *func, void *ctx) {
  func(ctx, &spc->a, offsetof(Spc, cyclesUsed) - offsetof(Spc, a));
}

int spc_runOpcode(Spc* spc) {
  spc->cyclesUsed = 0;
  if(spc->stopped) return 1;
  uint8_t opcode = spc_readOpcode(spc);
  spc->cyclesUsed = spcCyclesPerOpcode[opcode];
  spc_doOpcode(spc, opcode);
  return spc->cyclesUsed;
}

static uint8_t spc_readOpcode(Spc* spc) {
  return spc_read(spc, spc->pc++);
}

static uint16_t spc_readOpcodeWord(Spc* spc) {
  uint8_t low = spc_readOpcode(spc);
  return low | (spc_readOpcode(spc) << 8);
}

static uint8_t spc_getFlags(Spc* spc) {
  uint8_t val = spc->n << 7;
  val |= spc->v << 6;
  val |= spc->p << 5;
  val |= spc->b << 4;
  val |= spc->h << 3;
  val |= spc->i << 2;
  val |= spc->z << 1;
  val |= (uint8_t)spc->c;
  return val;
}

static void spc_setFlags(Spc* spc, uint8_t val) {
  spc->n = val & 0x80;
  spc->v = val & 0x40;
  spc->p = val & 0x20;
  spc->b = val & 0x10;
  spc->h = val & 8;
  spc->i = val & 4;
  spc->z = val & 2;
  spc->c = val & 1;
}

static void spc_setZN(Spc* spc, uint8_t value) {
  spc->z = value == 0;
  spc->n = value & 0x80;
}

static void spc_doBranch(Spc* spc, uint8_t value, bool check) {
  if(check) {
    spc->cyclesUsed += 2; // taken branch: 2 extra cycles
    spc->pc += (int8_t) value;
  }
}

static uint8_t spc_pullByte(Spc* spc) {
  spc->sp++;
  return spc_read(spc, 0x100 | spc->sp);
}

static void spc_pushByte(Spc* spc, uint8_t value) {
  spc_write(spc, 0x100 | spc->sp, value);
  spc->sp--;
}

static uint16_t spc_pullWord(Spc* spc) {
  uint8_t value = spc_pullByte(spc);
  return value | (spc_pullByte(spc) << 8);
}

static void spc_pushWord(Spc* spc, uint16_t value) {
  spc_pushByte(spc, value >> 8);
  spc_pushByte(spc, value & 0xff);
}

static uint16_t spc_readWord(Spc* spc, uint16_t adrl, uint16_t adrh) {
  uint8_t value = spc_read(spc, adrl);
  return value | (spc_read(spc, adrh) << 8);
}

static void spc_writeWord(Spc* spc, uint16_t adrl, uint16_t adrh, uint16_t value) {
  spc_write(spc, adrl, value & 0xff);
  spc_write(spc, adrh, value >> 8);
}

// adressing modes

static uint16_t spc_adrDp(Spc* spc) {
  return spc_readOpcode(spc) | (spc->p << 8);
}

static uint16_t spc_adrAbs(Spc* spc) {
  return spc_readOpcodeWord(spc);
}

static uint16_t spc_adrInd(Spc* spc) {
  return spc->x | (spc->p << 8);
}

static uint16_t spc_adrIdx(Spc* spc) {
  uint8_t pointer = spc_readOpcode(spc);
  return spc_readWord(spc, ((pointer + spc->x) & 0xff) | (spc->p << 8), ((pointer + spc->x + 1) & 0xff) | (spc->p << 8));
}

static uint16_t spc_adrImm(Spc* spc) {
  return spc->pc++;
}

static uint16_t spc_adrDpx(Spc* spc) {
  return ((spc_readOpcode(spc) + spc->x) & 0xff) | (spc->p << 8);
}

static uint16_t spc_adrDpy(Spc* spc) {
  return ((spc_readOpcode(spc) + spc->y) & 0xff) | (spc->p << 8);
}

static uint16_t spc_adrAbx(Spc* spc) {
  return (spc_readOpcodeWord(spc) + spc->x) & 0xffff;
}

static uint16_t spc_adrAby(Spc* spc) {
  return (spc_readOpcodeWord(spc) + spc->y) & 0xffff;
}

static uint16_t spc_adrIdy(Spc* spc) {
  uint8_t pointer = spc_readOpcode(spc);
  uint16_t adr = spc_readWord(spc, pointer | (spc->p << 8), ((pointer + 1) & 0xff) | (spc->p << 8));
  return (adr + spc->y) & 0xffff;
}

static uint16_t spc_adrDpDp(Spc* spc, uint16_t* src) {
  *src = spc_readOpcode(spc) | (spc->p << 8);
  return spc_readOpcode(spc) | (spc->p << 8);
}

static uint16_t spc_adrDpImm(Spc* spc, uint16_t* src) {
  *src = spc->pc++;
  return spc_readOpcode(spc) | (spc->p << 8);
}

static uint16_t spc_adrIndInd(Spc* spc, uint16_t* src) {
  *src = spc->y | (spc->p << 8);
  return spc->x | (spc->p << 8);
}

static uint8_t spc_adrAbsBit(Spc* spc, uint16_t* adr) {
  uint16_t adrBit = spc_readOpcodeWord(spc);
  *adr = adrBit & 0x1fff;
  return adrBit >> 13;
}

static uint16_t spc_adrDpWord(Spc* spc, uint16_t* low) {
  uint8_t adr = spc_readOpcode(spc);
  *low = adr | (spc->p << 8);
  return ((adr + 1) & 0xff) | (spc->p << 8);
}

static uint16_t spc_adrIndP(Spc* spc) {
  return spc->x++ | (spc->p << 8);
}

// opcode functions

static void spc_and(Spc* spc, uint16_t adr) {
  spc->a &= spc_read(spc, adr);
  spc_setZN(spc, spc->a);
}

static void spc_andm(Spc* spc, uint16_t dst, uint16_t src) {
  uint8_t value = spc_read(spc, src);
  uint8_t result = spc_read(spc, dst) & value;
  spc_write(spc, dst, result);
  spc_setZN(spc, result);
}

static void spc_or(Spc* spc, uint16_t adr) {
  spc->a |= spc_read(spc, adr);
  spc_setZN(spc, spc->a);
}

static void spc_orm(Spc* spc, uint16_t dst, uint16_t src) {
  uint8_t value = spc_read(spc, src);
  uint8_t result = spc_read(spc, dst) | value;
  spc_write(spc, dst, result);
  spc_setZN(spc, result);
}

static void spc_eor(Spc* spc, uint16_t adr) {
  spc->a ^= spc_read(spc, adr);
  spc_setZN(spc, spc->a);
}

static void spc_eorm(Spc* spc, uint16_t dst, uint16_t src) {
  uint8_t value = spc_read(spc, src);
  uint8_t result = spc_read(spc, dst) ^ value;
  spc_write(spc, dst, result);
  spc_setZN(spc, result);
}

static void spc_adc(Spc* spc, uint16_t adr) {
  uint8_t value = spc_read(spc, adr);
  int result = spc->a + value + spc->c;
  spc->v = (spc->a & 0x80) == (value & 0x80) && (value & 0x80) != (result & 0x80);
  spc->h = ((spc->a & 0xf) + (value & 0xf) + spc->c) > 0xf;
  spc->c = result > 0xff;
  spc->a = result;
  spc_setZN(spc, spc->a);
}

static void spc_adcm(Spc* spc, uint16_t dst, uint16_t src) {
  uint8_t value = spc_read(spc, src);
  uint8_t applyOn = spc_read(spc, dst);
  int result = applyOn + value + spc->c;
  spc->v = (applyOn & 0x80) == (value & 0x80) && (value & 0x80) != (result & 0x80);
  spc->h = ((applyOn & 0xf) + (value & 0xf) + spc->c) > 0xf;
  spc->c = result > 0xff;
  spc_write(spc, dst, result);
  spc_setZN(spc, result);
}

static void spc_sbc(Spc* spc, uint16_t adr) {
  uint8_t value = spc_read(spc, adr) ^ 0xff;
  int result = spc->a + value + spc->c;
  spc->v = (spc->a & 0x80) == (value & 0x80) && (value & 0x80) != (result & 0x80);
  spc->h = ((spc->a & 0xf) + (value & 0xf) + spc->c) > 0xf;
  spc->c = result > 0xff;
  spc->a = result;
  spc_setZN(spc, spc->a);
}

static void spc_sbcm(Spc* spc, uint16_t dst, uint16_t src) {
  uint8_t value = spc_read(spc, src) ^ 0xff;
  uint8_t applyOn = spc_read(spc, dst);
  int result = applyOn + value + spc->c;
  spc->v = (applyOn & 0x80) == (value & 0x80) && (value & 0x80) != (result & 0x80);
  spc->h = ((applyOn & 0xf) + (value & 0xf) + spc->c) > 0xf;
  spc->c = result > 0xff;
  spc_write(spc, dst, result);
  spc_setZN(spc, result);
}

static void spc_cmp(Spc* spc, uint16_t adr) {
  uint8_t value = spc_read(spc, adr) ^ 0xff;
  int result = spc->a + value + 1;
  spc->c = result > 0xff;
  spc_setZN(spc, result);
}

static void spc_cmpx(Spc* spc, uint16_t adr) {
  uint8_t value = spc_read(spc, adr) ^ 0xff;
  int result = spc->x + value + 1;
  spc->c = result > 0xff;
  spc_setZN(spc, result);
}

static void spc_cmpy(Spc* spc, uint16_t adr) {
  uint8_t value = spc_read(spc, adr) ^ 0xff;
  int result = spc->y + value + 1;
  spc->c = result > 0xff;
  spc_setZN(spc, result);
}

static void spc_cmpm(Spc* spc, uint16_t dst, uint16_t src) {
  uint8_t value = spc_read(spc, src) ^ 0xff;
  int result = spc_read(spc, dst) + value + 1;
  spc->c = result > 0xff;
  spc_setZN(spc, result);
}

static void spc_mov(Spc* spc, uint16_t adr) {
  spc->a = spc_read(spc, adr);
  spc_setZN(spc, spc->a);
}

static void spc_movx(Spc* spc, uint16_t adr) {
  spc->x = spc_read(spc, adr);
  spc_setZN(spc, spc->x);
}

static void spc_movy(Spc* spc, uint16_t adr) {
  spc->y = spc_read(spc, adr);
  spc_setZN(spc, spc->y);
}

static void spc_movs(Spc* spc, uint16_t adr) {
  spc_read(spc, adr);
  spc_write(spc, adr, spc->a);
}

static void spc_movsx(Spc* spc, uint16_t adr) {
  spc_read(spc, adr);
  spc_write(spc, adr, spc->x);
}

static void spc_movsy(Spc* spc, uint16_t adr) {
  spc_read(spc, adr);
  spc_write(spc, adr, spc->y);
}

static void spc_asl(Spc* spc, uint16_t adr) {
  uint8_t val = spc_read(spc, adr);
  spc->c = val & 0x80;
  val <<= 1;
  spc_write(spc, adr, val);
  spc_setZN(spc, val);
}

static void spc_lsr(Spc* spc, uint16_t adr) {
  uint8_t val = spc_read(spc, adr);
  spc->c = val & 1;
  val >>= 1;
  spc_write(spc, adr, val);
  spc_setZN(spc, val);
}

static void spc_rol(Spc* spc, uint16_t adr) {
  uint8_t val = spc_read(spc, adr);
  bool newC = val & 0x80;
  val = (val << 1) | (uint8_t)spc->c;
  spc->c = newC;
  spc_write(spc, adr, val);
  spc_setZN(spc, val);
}

static void spc_ror(Spc* spc, uint16_t adr) {
  uint8_t val = spc_read(spc, adr);
  bool newC = val & 1;
  val = (val >> 1) | (spc->c << 7);
  spc->c = newC;
  spc_write(spc, adr, val);
  spc_setZN(spc, val);
}

static void spc_inc(Spc* spc, uint16_t adr) {
  uint8_t val = spc_read(spc, adr) + 1;
  spc_write(spc, adr, val);
  spc_setZN(spc, val);
}

static void spc_dec(Spc* spc, uint16_t adr) {
  uint8_t val = spc_read(spc, adr) - 1;
  spc_write(spc, adr, val);
  spc_setZN(spc, val);
}

static void spc_doOpcode(Spc* spc, uint8_t opcode) {
  switch(opcode) {
    case 0x00: { // nop imp
      // no operation
      break;
    }
    case 0x01:
    case 0x11:
    case 0x21:
    case 0x31:
    case 0x41:
    case 0x51:
    case 0x61:
    case 0x71:
    case 0x81:
    case 0x91:
    case 0xa1:
    case 0xb1:
    case 0xc1:
    case 0xd1:
    case 0xe1:
    case 0xf1: { // tcall imp
      spc_pushWord(spc, spc->pc);
      uint16_t adr = 0xffde - (2 * (opcode >> 4));
      spc->pc = spc_readWord(spc, adr, adr + 1);
      break;
    }
    case 0x02:
    case 0x22:
    case 0x42:
    case 0x62:
    case 0x82:
    case 0xa2:
    case 0xc2:
    case 0xe2: { // set1 dp
      uint16_t adr = spc_adrDp(spc);
      spc_write(spc, adr, spc_read(spc, adr) | (1 << (opcode >> 5)));
      break;
    }
    case 0x12:
    case 0x32:
    case 0x52:
    case 0x72:
    case 0x92:
    case 0xb2:
    case 0xd2:
    case 0xf2: { // clr1 dp
      uint16_t adr = spc_adrDp(spc);
      spc_write(spc, adr, spc_read(spc, adr) & ~(1 << (opcode >> 5)));
      break;
    }
    case 0x03:
    case 0x23:
    case 0x43:
    case 0x63:
    case 0x83:
    case 0xa3:
    case 0xc3:
    case 0xe3: { // bbs dp, rel
      uint8_t val = spc_read(spc, spc_adrDp(spc));
      spc_doBranch(spc, spc_readOpcode(spc), val & (1 << (opcode >> 5)));
      break;
    }
    case 0x13:
    case 0x33:
    case 0x53:
    case 0x73:
    case 0x93:
    case 0xb3:
    case 0xd3:
    case 0xf3: { // bbc dp, rel
      uint8_t val = spc_read(spc, spc_adrDp(spc));
      spc_doBranch(spc, spc_readOpcode(spc), (val & (1 << (opcode >> 5))) == 0);
      break;
    }
    case 0x04: { // or  dp
      spc_or(spc, spc_adrDp(spc));
      break;
    }
    case 0x05: { // or  abs
      spc_or(spc, spc_adrAbs(spc));
      break;
    }
    case 0x06: { // or  ind
      spc_or(spc, spc_adrInd(spc));
      break;
    }
    case 0x07: { // or  idx
      spc_or(spc, spc_adrIdx(spc));
      break;
    }
    case 0x08: { // or  imm
      spc_or(spc, spc_adrImm(spc));
      break;
    }
    case 0x09: { // orm dp, dp
      uint16_t src = 0;
      uint16_t dst = spc_adrDpDp(spc, &src);
      spc_orm(spc, dst, src);
      break;
    }
    case 0x0a: { // or1 abs.bit
      uint16_t adr = 0;
      uint8_t bit = spc_adrAbsBit(spc, &adr);
      spc->c = spc->c | ((spc_read(spc, adr) >> bit) & 1);
      break;
    }
    case 0x0b: { // asl dp
      spc_asl(spc, spc_adrDp(spc));
      break;
    }
    case 0x0c: { // asl abs
      spc_asl(spc, spc_adrAbs(spc));
      break;
    }
    case 0x0d: { // pushp imp
      spc_pushByte(spc, spc_getFlags(spc));
      break;
    }
    case 0x0e: { // tset1 abs
      uint16_t adr = spc_adrAbs(spc);
      uint8_t val = spc_read(spc, adr);
      uint8_t result = spc->a + (val ^ 0xff) + 1;
      spc_setZN(spc, result);
      spc_write(spc, adr, val | spc->a);
      break;
    }
    case 0x0f: { // brk imp
      spc_pushWord(spc, spc->pc);
      spc_pushByte(spc, spc_getFlags(spc));
      spc->i = false;
      spc->b = true;
      spc->pc = spc_readWord(spc, 0xffde, 0xffdf);
      break;
    }
    case 0x10: { // bpl rel
      spc_doBranch(spc, spc_readOpcode(spc), !spc->n);
      break;
    }
    case 0x14: { // or  dpx
      spc_or(spc, spc_adrDpx(spc));
      break;
    }
    case 0x15: { // or  abx
      spc_or(spc, spc_adrAbx(spc));
      break;
    }
    case 0x16: { // or  aby
      spc_or(spc, spc_adrAby(spc));
      break;
    }
    case 0x17: { // or  idy
      spc_or(spc, spc_adrIdy(spc));
      break;
    }
    case 0x18: { // orm dp, imm
      uint16_t src = 0;
      uint16_t dst = spc_adrDpImm(spc, &src);
      spc_orm(spc, dst, src);
      break;
    }
    case 0x19: { // orm ind, ind
      uint16_t src = 0;
      uint16_t dst = spc_adrIndInd(spc, &src);
      spc_orm(spc, dst, src);
      break;
    }
    case 0x1a: { // decw dp
      uint16_t low = 0;
      uint16_t high = spc_adrDpWord(spc, &low);
      uint16_t value = spc_readWord(spc, low, high) - 1;
      spc->z = value == 0;
      spc->n = value & 0x8000;
      spc_writeWord(spc, low, high, value);
      break;
    }
    case 0x1b: { // asl dpx
      spc_asl(spc, spc_adrDpx(spc));
      break;
    }
    case 0x1c: { // asla imp
      spc->c = spc->a & 0x80;
      spc->a <<= 1;
      spc_setZN(spc, spc->a);
      break;
    }
    case 0x1d: { // decx imp
      spc->x--;
      spc_setZN(spc, spc->x);
      break;
    }
    case 0x1e: { // cmpx abs
      spc_cmpx(spc, spc_adrAbs(spc));
      break;
    }
    case 0x1f: { // jmp iax
      uint16_t pointer = spc_readOpcodeWord(spc);
      spc->pc = spc_readWord(spc, (pointer + spc->x) & 0xffff, (pointer + spc->x + 1) & 0xffff);
      break;
    }
    case 0x20: { // clrp imp
      spc->p = false;
      break;
    }
    case 0x24: { // and dp
      spc_and(spc, spc_adrDp(spc));
      break;
    }
    case 0x25: { // and abs
      spc_and(spc, spc_adrAbs(spc));
      break;
    }
    case 0x26: { // and ind
      spc_and(spc, spc_adrInd(spc));
      break;
    }
    case 0x27: { // and idx
      spc_and(spc, spc_adrIdx(spc));
      break;
    }
    case 0x28: { // and imm
      spc_and(spc, spc_adrImm(spc));
      break;
    }
    case 0x29: { // andm dp, dp
      uint16_t src = 0;
      uint16_t dst = spc_adrDpDp(spc, &src);
      spc_andm(spc, dst, src);
      break;
    }
    case 0x2a: { // or1n abs.bit
      uint16_t adr = 0;
      uint8_t bit = spc_adrAbsBit(spc, &adr);
      spc->c = spc->c | (~(spc_read(spc, adr) >> bit) & 1);
      break;
    }
    case 0x2b: { // rol dp
      spc_rol(spc, spc_adrDp(spc));
      break;
    }
    case 0x2c: { // rol abs
      spc_rol(spc, spc_adrAbs(spc));
      break;
    }
    case 0x2d: { // pusha imp
      spc_pushByte(spc, spc->a);
      break;
    }
    case 0x2e: { // cbne dp, rel
      uint8_t val = spc_read(spc, spc_adrDp(spc)) ^ 0xff;
      uint8_t result = spc->a + val + 1;
      spc_doBranch(spc, spc_readOpcode(spc), result != 0);
      break;
    }
    case 0x2f: { // bra rel
      spc->pc += (int8_t) spc_readOpcode(spc);
      break;
    }
    case 0x30: { // bmi rel
      spc_doBranch(spc, spc_readOpcode(spc), spc->n);
      break;
    }
    case 0x34: { // and dpx
      spc_and(spc, spc_adrDpx(spc));
      break;
    }
    case 0x35: { // and abx
      spc_and(spc, spc_adrAbx(spc));
      break;
    }
    case 0x36: { // and aby
      spc_and(spc, spc_adrAby(spc));
      break;
    }
    case 0x37: { // and idy
      spc_and(spc, spc_adrIdy(spc));
      break;
    }
    case 0x38: { // andm dp, imm
      uint16_t src = 0;
      uint16_t dst = spc_adrDpImm(spc, &src);
      spc_andm(spc, dst, src);
      break;
    }
    case 0x39: { // andm ind, ind
      uint16_t src = 0;
      uint16_t dst = spc_adrIndInd(spc, &src);
      spc_andm(spc, dst, src);
      break;
    }
    case 0x3a: { // incw dp
      uint16_t low = 0;
      uint16_t high = spc_adrDpWord(spc, &low);
      uint16_t value = spc_readWord(spc, low, high) + 1;
      spc->z = value == 0;
      spc->n = value & 0x8000;
      spc_writeWord(spc, low, high, value);
      break;
    }
    case 0x3b: { // rol dpx
      spc_rol(spc, spc_adrDpx(spc));
      break;
    }
    case 0x3c: { // rola imp
      bool newC = spc->a & 0x80;
      spc->a = (spc->a << 1) | (uint8_t)spc->c;
      spc->c = newC;
      spc_setZN(spc, spc->a);
      break;
    }
    case 0x3d: { // incx imp
      spc->x++;
      spc_setZN(spc, spc->x);
      break;
    }
    case 0x3e: { // cmpx dp
      spc_cmpx(spc, spc_adrDp(spc));
      break;
    }
    case 0x3f: { // call abs
      uint16_t dst = spc_readOpcodeWord(spc);
      spc_pushWord(spc, spc->pc);
      spc->pc = dst;
      break;
    }
    case 0x40: { // setp imp
      spc->p = true;
      break;
    }
    case 0x44: { // eor dp
      spc_eor(spc, spc_adrDp(spc));
      break;
    }
    case 0x45: { // eor abs
      spc_eor(spc, spc_adrAbs(spc));
      break;
    }
    case 0x46: { // eor ind
      spc_eor(spc, spc_adrInd(spc));
      break;
    }
    case 0x47: { // eor idx
      spc_eor(spc, spc_adrIdx(spc));
      break;
    }
    case 0x48: { // eor imm
      spc_eor(spc, spc_adrImm(spc));
      break;
    }
    case 0x49: { // eorm dp, dp
      uint16_t src = 0;
      uint16_t dst = spc_adrDpDp(spc, &src);
      spc_eorm(spc, dst, src);
      break;
    }
    case 0x4a: { // and1 abs.bit
      uint16_t adr = 0;
      uint8_t bit = spc_adrAbsBit(spc, &adr);
      spc->c = spc->c & ((spc_read(spc, adr) >> bit) & 1);
      break;
    }
    case 0x4b: { // lsr dp
      spc_lsr(spc, spc_adrDp(spc));
      break;
    }
    case 0x4c: { // lsr abs
      spc_lsr(spc, spc_adrAbs(spc));
      break;
    }
    case 0x4d: { // pushx imp
      spc_pushByte(spc, spc->x);
      break;
    }
    case 0x4e: { // tclr1 abs
      uint16_t adr = spc_adrAbs(spc);
      uint8_t val = spc_read(spc, adr);
      uint8_t result = spc->a + (val ^ 0xff) + 1;
      spc_setZN(spc, result);
      spc_write(spc, adr, val & ~spc->a);
      break;
    }
    case 0x4f: { // pcall dp
      uint8_t dst = spc_readOpcode(spc);
      spc_pushWord(spc, spc->pc);
      spc->pc = 0xff00 | dst;
      break;
    }
    case 0x50: { // bvc rel
      spc_doBranch(spc, spc_readOpcode(spc), !spc->v);
      break;
    }
    case 0x54: { // eor dpx
      spc_eor(spc, spc_adrDpx(spc));
      break;
    }
    case 0x55: { // eor abx
      spc_eor(spc, spc_adrAbx(spc));
      break;
    }
    case 0x56: { // eor aby
      spc_eor(spc, spc_adrAby(spc));
      break;
    }
    case 0x57: { // eor idy
      spc_eor(spc, spc_adrIdy(spc));
      break;
    }
    case 0x58: { // eorm dp, imm
      uint16_t src = 0;
      uint16_t dst = spc_adrDpImm(spc, &src);
      spc_eorm(spc, dst, src);
      break;
    }
    case 0x59: { // eorm ind, ind
      uint16_t src = 0;
      uint16_t dst = spc_adrIndInd(spc, &src);
      spc_eorm(spc, dst, src);
      break;
    }
    case 0x5a: { // cmpw dp
      uint16_t low = 0;
      uint16_t high = spc_adrDpWord(spc, &low);
      uint16_t value = spc_readWord(spc, low, high) ^ 0xffff;
      uint16_t ya = spc->a | (spc->y << 8);
      int result = ya + value + 1;
      spc->c = result > 0xffff;
      spc->z = (result & 0xffff) == 0;
      spc->n = result & 0x8000;
      break;
    }
    case 0x5b: { // lsr dpx
      spc_lsr(spc, spc_adrDpx(spc));
      break;
    }
    case 0x5c: { // lsra imp
      spc->c = spc->a & 1;
      spc->a >>= 1;
      spc_setZN(spc, spc->a);
      break;
    }
    case 0x5d: { // movxa imp
      spc->x = spc->a;
      spc_setZN(spc, spc->x);
      break;
    }
    case 0x5e: { // cmpy abs
      spc_cmpy(spc, spc_adrAbs(spc));
      break;
    }
    case 0x5f: { // jmp abs
      spc->pc = spc_readOpcodeWord(spc);
      break;
    }
    case 0x60: { // clrc imp
      spc->c = false;
      break;
    }
    case 0x64: { // cmp dp
      spc_cmp(spc, spc_adrDp(spc));
      break;
    }
    case 0x65: { // cmp abs
      spc_cmp(spc, spc_adrAbs(spc));
      break;
    }
    case 0x66: { // cmp ind
      spc_cmp(spc, spc_adrInd(spc));
      break;
    }
    case 0x67: { // cmp idx
      spc_cmp(spc, spc_adrIdx(spc));
      break;
    }
    case 0x68: { // cmp imm
      spc_cmp(spc, spc_adrImm(spc));
      break;
    }
    case 0x69: { // cmpm dp, dp
      uint16_t src = 0;
      uint16_t dst = spc_adrDpDp(spc, &src);
      spc_cmpm(spc, dst, src);
      break;
    }
    case 0x6a: { // and1n abs.bit
      uint16_t adr = 0;
      uint8_t bit = spc_adrAbsBit(spc, &adr);
      spc->c = spc->c & (~(spc_read(spc, adr) >> bit) & 1);
      break;
    }
    case 0x6b: { // ror dp
      spc_ror(spc, spc_adrDp(spc));
      break;
    }
    case 0x6c: { // ror abs
      spc_ror(spc, spc_adrAbs(spc));
      break;
    }
    case 0x6d: { // pushy imp
      spc_pushByte(spc, spc->y);
      break;
    }
    case 0x6e: { // dbnz dp, rel
      uint16_t adr = spc_adrDp(spc);
      uint8_t result = spc_read(spc, adr) - 1;
      spc_write(spc, adr, result);
      spc_doBranch(spc, spc_readOpcode(spc), result != 0);
      break;
    }
    case 0x6f: { // ret imp
      spc->pc = spc_pullWord(spc);
      break;
    }
    case 0x70: { // bvs rel
      spc_doBranch(spc, spc_readOpcode(spc), spc->v);
      break;
    }
    case 0x74: { // cmp dpx
      spc_cmp(spc, spc_adrDpx(spc));
      break;
    }
    case 0x75: { // cmp abx
      spc_cmp(spc, spc_adrAbx(spc));
      break;
    }
    case 0x76: { // cmp aby
      spc_cmp(spc, spc_adrAby(spc));
      break;
    }
    case 0x77: { // cmp idy
      spc_cmp(spc, spc_adrIdy(spc));
      break;
    }
    case 0x78: { // cmpm dp, imm
      uint16_t src = 0;
      uint16_t dst = spc_adrDpImm(spc, &src);
      spc_cmpm(spc, dst, src);
      break;
    }
    case 0x79: { // cmpm ind, ind
      uint16_t src = 0;
      uint16_t dst = spc_adrIndInd(spc, &src);
      spc_cmpm(spc, dst, src);
      break;
    }
    case 0x7a: { // addw dp
      uint16_t low = 0;
      uint16_t high = spc_adrDpWord(spc, &low);
      uint16_t value = spc_readWord(spc, low, high);
      uint16_t ya = spc->a | (spc->y << 8);
      int result = ya + value;
      spc->v = (ya & 0x8000) == (value & 0x8000) && (value & 0x8000) != (result & 0x8000);
      spc->h = ((ya & 0xfff) + (value & 0xfff) + 1) > 0xfff;
      spc->c = result > 0xffff;
      spc->z = (result & 0xffff) == 0;
      spc->n = result & 0x8000;
      spc->a = result & 0xff;
      spc->y = result >> 8;
      break;
    }
    case 0x7b: { // ror dpx
      spc_ror(spc, spc_adrDpx(spc));
      break;
    }
    case 0x7c: { // rora imp
      bool newC = spc->a & 1;
      spc->a = (spc->a >> 1) | (spc->c << 7);
      spc->c = newC;
      spc_setZN(spc, spc->a);
      break;
    }
    case 0x7d: { // movax imp
      spc->a = spc->x;
      spc_setZN(spc, spc->a);
      break;
    }
    case 0x7e: { // cmpy dp
      spc_cmpy(spc, spc_adrDp(spc));
      break;
    }
    case 0x7f: { // reti imp
      spc_setFlags(spc, spc_pullByte(spc));
      spc->pc = spc_pullWord(spc);
      break;
    }
    case 0x80: { // setc imp
      spc->c = true;
      break;
    }
    case 0x84: { // adc dp
      spc_adc(spc, spc_adrDp(spc));
      break;
    }
    case 0x85: { // adc abs
      spc_adc(spc, spc_adrAbs(spc));
      break;
    }
    case 0x86: { // adc ind
      spc_adc(spc, spc_adrInd(spc));
      break;
    }
    case 0x87: { // adc idx
      spc_adc(spc, spc_adrIdx(spc));
      break;
    }
    case 0x88: { // adc imm
      spc_adc(spc, spc_adrImm(spc));
      break;
    }
    case 0x89: { // adcm dp, dp
      uint16_t src = 0;
      uint16_t dst = spc_adrDpDp(spc, &src);
      spc_adcm(spc, dst, src);
      break;
    }
    case 0x8a: { // eor1 abs.bit
      uint16_t adr = 0;
      uint8_t bit = spc_adrAbsBit(spc, &adr);
      spc->c = spc->c ^ ((spc_read(spc, adr) >> bit) & 1);
      break;
    }
    case 0x8b: { // dec dp
      spc_dec(spc, spc_adrDp(spc));
      break;
    }
    case 0x8c: { // dec abs
      spc_dec(spc, spc_adrAbs(spc));
      break;
    }
    case 0x8d: { // movy imm
      spc_movy(spc, spc_adrImm(spc));
      break;
    }
    case 0x8e: { // popp imp
      spc_setFlags(spc, spc_pullByte(spc));
      break;
    }
    case 0x8f: { // movm dp, imm
      uint16_t src = 0;
      uint16_t dst = spc_adrDpImm(spc, &src);
      uint8_t val = spc_read(spc, src);
      spc_read(spc, dst);
      spc_write(spc, dst, val);
      break;
    }
    case 0x90: { // bcc rel
      spc_doBranch(spc, spc_readOpcode(spc), !spc->c);
      break;
    }
    case 0x94: { // adc dpx
      spc_adc(spc, spc_adrDpx(spc));
      break;
    }
    case 0x95: { // adc abx
      spc_adc(spc, spc_adrAbx(spc));
      break;
    }
    case 0x96: { // adc aby
      spc_adc(spc, spc_adrAby(spc));
      break;
    }
    case 0x97: { // adc idy
      spc_adc(spc, spc_adrIdy(spc));
      break;
    }
    case 0x98: { // adcm dp, imm
      uint16_t src = 0;
      uint16_t dst = spc_adrDpImm(spc, &src);
      spc_adcm(spc, dst, src);
      break;
    }
    case 0x99: { // adcm ind, ind
      uint16_t src = 0;
      uint16_t dst = spc_adrIndInd(spc, &src);
      spc_adcm(spc, dst, src);
      break;
    }
    case 0x9a: { // subw dp
      uint16_t low = 0;
      uint16_t high = spc_adrDpWord(spc, &low);
      uint16_t value = spc_readWord(spc, low, high) ^ 0xffff;
      uint16_t ya = spc->a | (spc->y << 8);
      int result = ya + value + 1;
      spc->v = (ya & 0x8000) == (value & 0x8000) && (value & 0x8000) != (result & 0x8000);
      spc->h = ((ya & 0xfff) + (value & 0xfff) + 1) > 0xfff;
      spc->c = result > 0xffff;
      spc->z = (result & 0xffff) == 0;
      spc->n = result & 0x8000;
      spc->a = result & 0xff;
      spc->y = result >> 8;
      break;
    }
    case 0x9b: { // dec dpx
      spc_dec(spc, spc_adrDpx(spc));
      break;
    }
    case 0x9c: { // deca imp
      spc->a--;
      spc_setZN(spc, spc->a);
      break;
    }
    case 0x9d: { // movxp imp
      spc->x = spc->sp;
      spc_setZN(spc, spc->x);
      break;
    }
    case 0x9e: { // div imp
      // TODO: proper division algorithm
      uint16_t value = spc->a | (spc->y << 8);
      int result = 0xffff;
      int mod = spc->a;
      if(spc->x != 0) {
        result = value / spc->x;
        mod = value % spc->x;
      }
      spc->v = result > 0xff;
      spc->h = (spc->x & 0xf) <= (spc->y & 0xf);
      spc->a = result;
      spc->y = mod;
      spc_setZN(spc, spc->a);
      break;
    }
    case 0x9f: { // xcn imp
      spc->a = (spc->a >> 4) | (spc->a << 4);
      spc_setZN(spc, spc->a);
      break;
    }
    case 0xa0: { // ei  imp
      spc->i = true;
      break;
    }
    case 0xa4: { // sbc dp
      spc_sbc(spc, spc_adrDp(spc));
      break;
    }
    case 0xa5: { // sbc abs
      spc_sbc(spc, spc_adrAbs(spc));
      break;
    }
    case 0xa6: { // sbc ind
      spc_sbc(spc, spc_adrInd(spc));
      break;
    }
    case 0xa7: { // sbc idx
      spc_sbc(spc, spc_adrIdx(spc));
      break;
    }
    case 0xa8: { // sbc imm
      spc_sbc(spc, spc_adrImm(spc));
      break;
    }
    case 0xa9: { // sbcm dp, dp
      uint16_t src = 0;
      uint16_t dst = spc_adrDpDp(spc, &src);
      spc_sbcm(spc, dst, src);
      break;
    }
    case 0xaa: { // mov1 abs.bit
      uint16_t adr = 0;
      uint8_t bit = spc_adrAbsBit(spc, &adr);
      spc->c = (spc_read(spc, adr) >> bit) & 1;
      break;
    }
    case 0xab: { // inc dp
      spc_inc(spc, spc_adrDp(spc));
      break;
    }
    case 0xac: { // inc abs
      spc_inc(spc, spc_adrAbs(spc));
      break;
    }
    case 0xad: { // cmpy imm
      spc_cmpy(spc, spc_adrImm(spc));
      break;
    }
    case 0xae: { // popa imp
      spc->a = spc_pullByte(spc);
      break;
    }
    case 0xaf: { // movs ind+
      uint16_t adr = spc_adrIndP(spc);
      spc_write(spc, adr, spc->a);
      break;
    }
    case 0xb0: { // bcs rel
      spc_doBranch(spc, spc_readOpcode(spc), spc->c);
      break;
    }
    case 0xb4: { // sbc dpx
      spc_sbc(spc, spc_adrDpx(spc));
      break;
    }
    case 0xb5: { // sbc abx
      spc_sbc(spc, spc_adrAbx(spc));
      break;
    }
    case 0xb6: { // sbc aby
      spc_sbc(spc, spc_adrAby(spc));
      break;
    }
    case 0xb7: { // sbc idy
      spc_sbc(spc, spc_adrIdy(spc));
      break;
    }
    case 0xb8: { // sbcm dp, imm
      uint16_t src = 0;
      uint16_t dst = spc_adrDpImm(spc, &src);
      spc_sbcm(spc, dst, src);
      break;
    }
    case 0xb9: { // sbcm ind, ind
      uint16_t src = 0;
      uint16_t dst = spc_adrIndInd(spc, &src);
      spc_sbcm(spc, dst, src);
      break;
    }
    case 0xba: { // movw dp
      uint16_t low = 0;
      uint16_t high = spc_adrDpWord(spc, &low);
      uint16_t val = spc_readWord(spc, low, high);
      spc->a = val & 0xff;
      spc->y = val >> 8;
      spc->z = val == 0;
      spc->n = val & 0x8000;
      break;
    }
    case 0xbb: { // inc dpx
      spc_inc(spc, spc_adrDpx(spc));
      break;
    }
    case 0xbc: { // inca imp
      spc->a++;
      spc_setZN(spc, spc->a);
      break;
    }
    case 0xbd: { // movpx imp
      spc->sp = spc->x;
      break;
    }
    case 0xbe: { // das imp
      if(spc->a > 0x99 || !spc->c) {
        spc->a -= 0x60;
        spc->c = false;
      }
      if((spc->a & 0xf) > 9 || !spc->h) {
        spc->a -= 6;
      }
      spc_setZN(spc, spc->a);
      break;
    }
    case 0xbf: { // mov ind+
      uint16_t adr = spc_adrIndP(spc);
      spc->a = spc_read(spc, adr);
      spc_setZN(spc, spc->a);
      break;
    }
    case 0xc0: { // di  imp
      spc->i = false;
      break;
    }
    case 0xc4: { // movs dp
      spc_movs(spc, spc_adrDp(spc));
      break;
    }
    case 0xc5: { // movs abs
      spc_movs(spc, spc_adrAbs(spc));
      break;
    }
    case 0xc6: { // movs ind
      spc_movs(spc, spc_adrInd(spc));
      break;
    }
    case 0xc7: { // movs idx
      spc_movs(spc, spc_adrIdx(spc));
      break;
    }
    case 0xc8: { // cmpx imm
      spc_cmpx(spc, spc_adrImm(spc));
      break;
    }
    case 0xc9: { // movsx abs
      spc_movsx(spc, spc_adrAbs(spc));
      break;
    }
    case 0xca: { // mov1s abs.bit
      uint16_t adr = 0;
      uint8_t bit = spc_adrAbsBit(spc, &adr);
      uint8_t result = (spc_read(spc, adr) & (~(1 << bit))) | (spc->c << bit);
      spc_write(spc, adr, result);
      break;
    }
    case 0xcb: { // movsy dp
      spc_movsy(spc, spc_adrDp(spc));
      break;
    }
    case 0xcc: { // movsy abs
      spc_movsy(spc, spc_adrAbs(spc));
      break;
    }
    case 0xcd: { // movx imm
      spc_movx(spc, spc_adrImm(spc));
      break;
    }
    case 0xce: { // popx imp
      spc->x = spc_pullByte(spc);
      break;
    }
    case 0xcf: { // mul imp
      uint16_t result = spc->a * spc->y;
      spc->a = result & 0xff;
      spc->y = result >> 8;
      spc_setZN(spc, spc->y);
      break;
    }
    case 0xd0: { // bne rel
      spc_doBranch(spc, spc_readOpcode(spc), !spc->z);
      break;
    }
    case 0xd4: { // movs dpx
      spc_movs(spc, spc_adrDpx(spc));
      break;
    }
    case 0xd5: { // movs abx
      spc_movs(spc, spc_adrAbx(spc));
      break;
    }
    case 0xd6: { // movs aby
      spc_movs(spc, spc_adrAby(spc));
      break;
    }
    case 0xd7: { // movs idy
      spc_movs(spc, spc_adrIdy(spc));
      break;
    }
    case 0xd8: { // movsx dp
      spc_movsx(spc, spc_adrDp(spc));
      break;
    }
    case 0xd9: { // movsx dpy
      spc_movsx(spc, spc_adrDpy(spc));
      break;
    }
    case 0xda: { // movws dp
      uint16_t low = 0;
      uint16_t high = spc_adrDpWord(spc, &low);
      spc_read(spc, low);
      spc_write(spc, low, spc->a);
      spc_write(spc, high, spc->y);
      break;
    }
    case 0xdb: { // movsy dpx
      spc_movsy(spc, spc_adrDpx(spc));
      break;
    }
    case 0xdc: { // decy imp
      spc->y--;
      spc_setZN(spc, spc->y);
      break;
    }
    case 0xdd: { // movay imp
      spc->a = spc->y;
      spc_setZN(spc, spc->a);
      break;
    }
    case 0xde: { // cbne dpx, rel
      uint8_t val = spc_read(spc, spc_adrDpx(spc)) ^ 0xff;
      uint8_t result = spc->a + val + 1;
      spc_doBranch(spc, spc_readOpcode(spc), result != 0);
      break;
    }
    case 0xdf: { // daa imp
      if(spc->a > 0x99 || spc->c) {
        spc->a += 0x60;
        spc->c = true;
      }
      if((spc->a & 0xf) > 9 || spc->h) {
        spc->a += 6;
      }
      spc_setZN(spc, spc->a);
      break;
    }
    case 0xe0: { // clrv imp
      spc->v = false;
      spc->h = false;
      break;
    }
    case 0xe4: { // mov dp
      spc_mov(spc, spc_adrDp(spc));
      break;
    }
    case 0xe5: { // mov abs
      spc_mov(spc, spc_adrAbs(spc));
      break;
    }
    case 0xe6: { // mov ind
      spc_mov(spc, spc_adrInd(spc));
      break;
    }
    case 0xe7: { // mov idx
      spc_mov(spc, spc_adrIdx(spc));
      break;
    }
    case 0xe8: { // mov imm
      spc_mov(spc, spc_adrImm(spc));
      break;
    }
    case 0xe9: { // movx abs
      spc_movx(spc, spc_adrAbs(spc));
      break;
    }
    case 0xea: { // not1 abs.bit
      uint16_t adr = 0;
      uint8_t bit = spc_adrAbsBit(spc, &adr);
      uint8_t result = spc_read(spc, adr) ^ (1 << bit);
      spc_write(spc, adr, result);
      break;
    }
    case 0xeb: { // movy dp
      spc_movy(spc, spc_adrDp(spc));
      break;
    }
    case 0xec: { // movy abs
      spc_movy(spc, spc_adrAbs(spc));
      break;
    }
    case 0xed: { // notc imp
      spc->c = !spc->c;
      break;
    }
    case 0xee: { // popy imp
      spc->y = spc_pullByte(spc);
      break;
    }
    case 0xef: { // sleep imp
      spc->stopped = true; // no interrupts, so sleeping stops as well
      break;
    }
    case 0xf0: { // beq rel
      spc_doBranch(spc, spc_readOpcode(spc), spc->z);
      break;
    }
    case 0xf4: { // mov dpx
      spc_mov(spc, spc_adrDpx(spc));
      break;
    }
    case 0xf5: { // mov abx
      spc_mov(spc, spc_adrAbx(spc));
      break;
    }
    case 0xf6: { // mov aby
      spc_mov(spc, spc_adrAby(spc));
      break;
    }
    case 0xf7: { // mov idy
      spc_mov(spc, spc_adrIdy(spc));
      break;
    }
    case 0xf8: { // movx dp
      spc_movx(spc, spc_adrDp(spc));
      break;
    }
    case 0xf9: { // movx dpy
      spc_movx(spc, spc_adrDpy(spc));
      break;
    }
    case 0xfa: { // movm dp, dp
      uint16_t src = 0;
      uint16_t dst = spc_adrDpDp(spc, &src);
      uint8_t val = spc_read(spc, src);
      spc_write(spc, dst, val);
      break;
    }
    case 0xfb: { // movy dpx
      spc_movy(spc, spc_adrDpx(spc));
      break;
    }
    case 0xfc: { // incy imp
      spc->y++;
      spc_setZN(spc, spc->y);
      break;
    }
    case 0xfd: { // movya imp
      spc->y = spc->a;
      spc_setZN(spc, spc->y);
      break;
    }
    case 0xfe: { // dbnzy rel
      spc->y--;
      spc_doBranch(spc, spc_readOpcode(spc), spc->y != 0);
      break;
    }
    case 0xff: { // stop imp
      spc->stopped = true;
      break;
    }
  }
}


/* === apu.c === */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>





static const uint8_t bootRom[0x40] = {
  0xcd, 0xef, 0xbd, 0xe8, 0x00, 0xc6, 0x1d, 0xd0, 0xfc, 0x8f, 0xaa, 0xf4, 0x8f, 0xbb, 0xf5, 0x78,
  0xcc, 0xf4, 0xd0, 0xfb, 0x2f, 0x19, 0xeb, 0xf4, 0xd0, 0xfc, 0x7e, 0xf4, 0xd0, 0x0b, 0xe4, 0xf5,
  0xcb, 0xf4, 0xd7, 0x00, 0xfc, 0xd0, 0xf3, 0xab, 0x01, 0x10, 0xef, 0x7e, 0xf4, 0x10, 0xeb, 0xba,
  0xf6, 0xda, 0x00, 0xba, 0xf4, 0xc4, 0xf4, 0xdd, 0x5d, 0xd0, 0xdb, 0x1f, 0x00, 0x00, 0xc0, 0xff
};

Apu* apu_init() {
  Apu* apu = (Apu * )malloc(sizeof(Apu));
  apu->spc = spc_init(apu);
  apu->dsp = dsp_init(apu->ram);
  return apu;
}

void apu_free(Apu* apu) {
  spc_free(apu->spc);
  dsp_free(apu->dsp);
  free(apu);
}

void apu_saveload(Apu *apu, SaveLoadFunc *func, void *ctx) {
  func(ctx, apu->ram, offsetof(Apu, hist) - offsetof(Apu, ram));
  dsp_saveload(apu->dsp, func, ctx);
  spc_saveload(apu->spc, func, ctx);
}

void apu_reset(Apu* apu) {
  apu->romReadable = true; // before resetting spc, because it reads reset vector from it
  spc_reset(apu->spc);
  dsp_reset(apu->dsp);
  memset(apu->ram, 0, sizeof(apu->ram));
  apu->dspAdr = 0;
  apu->cycles = 0;
  memset(apu->inPorts, 0, sizeof(apu->inPorts));
  memset(apu->outPorts, 0, sizeof(apu->outPorts));
  for(int i = 0; i < 3; i++) {
    apu->timer[i].cycles = 0;
    apu->timer[i].divider = 0;
    apu->timer[i].target = 0;
    apu->timer[i].counter = 0;
    apu->timer[i].enabled = false;
  }
  apu->cpuCyclesLeft = 7;
  apu->hist.count = 0;
}

void apu_cycle(Apu* apu) {
  if(apu->cpuCyclesLeft == 0) {
    apu->cpuCyclesLeft = spc_runOpcode(apu->spc);
  }
  apu->cpuCyclesLeft--;

  if((apu->cycles & 0x1f) == 0) {
    // every 32 cycles
    dsp_cycle(apu->dsp);
  }

  // handle timers
  for(int i = 0; i < 3; i++) {
    if(apu->timer[i].cycles == 0) {
      apu->timer[i].cycles = i == 2 ? 16 : 128;
      if(apu->timer[i].enabled) {
        apu->timer[i].divider++;
        if(apu->timer[i].divider == apu->timer[i].target) {
          apu->timer[i].divider = 0;
          apu->timer[i].counter++;
          apu->timer[i].counter &= 0xf;
        }
      }
    }
    apu->timer[i].cycles--;
  }

  apu->cycles++;
}

uint8_t apu_cpuRead(Apu* apu, uint16_t adr) {
  switch(adr) {
    case 0xf0:
    case 0xf1:
    case 0xfa:
    case 0xfb:
    case 0xfc: {
      return 0;
    }
    case 0xf2: {
      return apu->dspAdr;
    }
    case 0xf3: {
      return dsp_read(apu->dsp, apu->dspAdr & 0x7f);
    }
    case 0xf4:
    case 0xf5:
    case 0xf6:
    case 0xf7:
    case 0xf8:
    case 0xf9: {
      return apu->inPorts[adr - 0xf4];
    }
    case 0xfd:
    case 0xfe:
    case 0xff: {
      uint8_t ret = apu->timer[adr - 0xfd].counter;
      apu->timer[adr - 0xfd].counter = 0;
      return ret;
    }
  }
  if(apu->romReadable && adr >= 0xffc0) {
    return bootRom[adr - 0xffc0];
  }
  return apu->ram[adr];
}

void apu_cpuWrite(Apu* apu, uint16_t adr, uint8_t val) {
  switch(adr) {
    case 0xf0: {
      break; // test register
    }
    case 0xf1: {
      for(int i = 0; i < 3; i++) {
        if(!apu->timer[i].enabled && (val & (1 << i))) {
          apu->timer[i].divider = 0;
          apu->timer[i].counter = 0;
        }
        apu->timer[i].enabled = val & (1 << i);
      }
      if(val & 0x10) {
        apu->inPorts[0] = 0;
        apu->inPorts[1] = 0;
      }
      if(val & 0x20) {
        apu->inPorts[2] = 0;
        apu->inPorts[3] = 0;
      }
      apu->romReadable = val & 0x80;
      break;
    }
    case 0xf2: {
      apu->dspAdr = val;
      break;
    }
    case 0xf3: {
      int i = apu->hist.count;
      if (i != 256) {
        apu->hist.count = i + 1;
        apu->hist.addr[i] = apu->dspAdr;
        apu->hist.val[i] = val;
      }
      if(apu->dspAdr < 0x80) dsp_write(apu->dsp, apu->dspAdr, val);
      break;
    }
    case 0xf4:
    case 0xf5:
    case 0xf6:
    case 0xf7: {
      apu->outPorts[adr - 0xf4] = val;
      break;
    }
    case 0xf8:
    case 0xf9: {
      apu->inPorts[adr - 0xf4] = val;
      break;
    }
    case 0xfa:
    case 0xfb:
    case 0xfc: {
      apu->timer[adr - 0xfa].target = val;
      break;
    }
  }
  apu->ram[adr] = val;
}


/* === cpu.c === */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stddef.h>



static const int cyclesPerOpcode[256] = {
  7, 6, 7, 4, 5, 3, 5, 6, 3, 2, 2, 4, 6, 4, 6, 5,
  2, 5, 5, 7, 5, 4, 6, 6, 2, 4, 2, 2, 6, 4, 7, 5,
  6, 6, 8, 4, 3, 3, 5, 6, 4, 2, 2, 5, 4, 4, 6, 5,
  2, 5, 5, 7, 4, 4, 6, 6, 2, 4, 2, 2, 4, 4, 7, 5,
  6, 6, 2, 4, 7, 3, 5, 6, 3, 2, 2, 3, 3, 4, 6, 5,
  2, 5, 5, 7, 7, 4, 6, 6, 2, 4, 3, 2, 4, 4, 7, 5,
  6, 6, 6, 4, 3, 3, 5, 6, 4, 2, 2, 6, 5, 4, 6, 5,
  2, 5, 5, 7, 4, 4, 6, 6, 2, 4, 4, 2, 6, 4, 7, 5,
  3, 6, 4, 4, 3, 3, 3, 6, 2, 2, 2, 3, 4, 4, 4, 5,
  2, 6, 5, 7, 4, 4, 4, 6, 2, 5, 2, 2, 4, 5, 5, 5,
  2, 6, 2, 4, 3, 3, 3, 6, 2, 2, 2, 4, 4, 4, 4, 5,
  2, 5, 5, 7, 4, 4, 4, 6, 2, 4, 2, 2, 4, 4, 4, 5,
  2, 6, 3, 4, 3, 3, 5, 6, 2, 2, 2, 3, 4, 4, 6, 5,
  2, 5, 5, 7, 6, 4, 6, 6, 2, 4, 3, 3, 6, 4, 7, 5,
  2, 6, 3, 4, 3, 3, 5, 6, 2, 2, 2, 3, 4, 4, 6, 5,
  2, 5, 5, 7, 5, 4, 6, 6, 2, 4, 4, 2, 8, 4, 7, 5
};

static uint8_t cpu_read(Cpu* cpu, uint32_t adr);
static void cpu_write(Cpu* cpu, uint32_t adr, uint8_t val);
static uint8_t cpu_readOpcode(Cpu* cpu);
static uint16_t cpu_readOpcodeWord(Cpu* cpu);
void cpu_setFlags(Cpu* cpu, uint8_t value);
static void cpu_setZN(Cpu* cpu, uint16_t value, bool byte);
static void cpu_doBranch(Cpu* cpu, uint8_t value, bool check);
static uint8_t cpu_pullByte(Cpu* cpu);
static void cpu_pushByte(Cpu* cpu, uint8_t value);
static uint16_t cpu_pullWord(Cpu* cpu);
static void cpu_pushWord(Cpu* cpu, uint16_t value);
static uint16_t cpu_readWord(Cpu* cpu, uint32_t adrl, uint32_t adrh);
static void cpu_writeWord(Cpu* cpu, uint32_t adrl, uint32_t adrh, uint16_t value, bool reversed);
static void cpu_doInterrupt(Cpu* cpu, bool irq);
static void cpu_doOpcode(Cpu* cpu, uint8_t opcode);

// addressing modes and opcode functions not declared, only used after defintions

static uint8_t cpu_read(Cpu* cpu, uint32_t adr) {
  if (cpu->memType == 1) {
    return sa1_bus_read(cpu->mem, adr);
  }
  return snes_cpuRead((Snes*) cpu->mem, adr);
}

static void cpu_write(Cpu* cpu, uint32_t adr, uint8_t val) {
  if (cpu->memType == 1) {
    sa1_bus_write(cpu->mem, adr, val);
    return;
  }
  snes_cpuWrite((Snes*) cpu->mem, adr, val);
}

Cpu* cpu_init(void* mem, int memType) {
  Cpu* cpu = (Cpu * )malloc(sizeof(Cpu));
  cpu->mem = mem;
  cpu->memType = memType;
  return cpu;
}

void cpu_free(Cpu* cpu) {
  free(cpu);
}

void cpu_reset(Cpu* cpu) {
  cpu->a = 0;
  cpu->x = 0;
  cpu->y = 0;
  cpu->sp = 0x100;
  cpu->pc = cpu_read(cpu, 0xfffc) | (cpu_read(cpu, 0xfffd) << 8);
  cpu->dp = 0;
  cpu->k = 0;
  cpu->db = 0;
  cpu->c = false;
  cpu->z = false;
  cpu->v = false;
  cpu->n = false;
  cpu->i = true;
  cpu->d = false;
  cpu->xf = true;
  cpu->mf = true;
  cpu->e = true;
  cpu->irqWanted = false;
  cpu->nmiWanted = false;
  cpu->waiting = false;
  cpu->stopped = false;
  cpu->cyclesUsed = 0;
  cpu->spBreakpoint = 0x0;
  cpu->in_emu = 0;
}

void cpu_saveload(Cpu *cpu, SaveLoadFunc *func, void *ctx) {
  func(ctx, &cpu->a, offsetof(Cpu, cyclesUsed) - offsetof(Cpu, a));
  cpu->spBreakpoint = 0x0;
}

int cpu_runOpcode(Cpu* cpu) {
  cpu->cyclesUsed = 0;
  if(cpu->stopped) return 1;
  if(cpu->waiting) {
    if(cpu->irqWanted || cpu->nmiWanted) {
      cpu->waiting = false;
    }
    return 1;
  }
  // not stopped or waiting, execute a opcode or go to interrupt
  if((!cpu->i && cpu->irqWanted) || cpu->nmiWanted) {
    if (cpu->in_emu) {
      printf("nmi while in emu!\n");
    }

    cpu->cyclesUsed = 7; // interrupt: at least 7 cycles
    if(cpu->nmiWanted) {
      cpu->nmiWanted = false;
      cpu_doInterrupt(cpu, false);
    } else {
      // must be irq
      if (cpu->memType == 1) cpu->irqWanted = false;
      cpu_doInterrupt(cpu, true);
    }
  } else {
    uint8_t opcode = cpu_readOpcode(cpu);
    cpu->cyclesUsed = cyclesPerOpcode[opcode];
    cpu_doOpcode(cpu, opcode);
  }
  return cpu->cyclesUsed;
}

static uint8_t cpu_readOpcode(Cpu* cpu) {
  return cpu_read(cpu, (cpu->k << 16) | cpu->pc++);
}

static uint16_t cpu_readOpcodeWord(Cpu* cpu) {
  uint8_t low = cpu_readOpcode(cpu);
  return low | (cpu_readOpcode(cpu) << 8);
}

uint8_t cpu_getFlags(Cpu* cpu) {
  uint8_t val = cpu->n << 7;
  val |= cpu->v << 6;
  val |= cpu->mf << 5;
  val |= cpu->xf << 4;
  val |= cpu->d << 3;
  val |= cpu->i << 2;
  val |= cpu->z << 1;
  val |= (uint8_t)cpu->c;
  return val;
}

void cpu_setFlags(Cpu* cpu, uint8_t val) {
  cpu->n = val & 0x80;
  cpu->v = val & 0x40;
  cpu->mf = val & 0x20;
  cpu->xf = val & 0x10;
  cpu->d = val & 8;
  cpu->i = val & 4;
  cpu->z = val & 2;
  cpu->c = val & 1;
  if(cpu->e) {
    cpu->mf = true;
    cpu->xf = true;
    cpu->sp = (cpu->sp & 0xff) | 0x100;
  }
  if(cpu->xf) {
    cpu->x &= 0xff;
    cpu->y &= 0xff;
  }
}

static void cpu_setZN(Cpu* cpu, uint16_t value, bool byte) {
  if(byte) {
    cpu->z = (value & 0xff) == 0;
    cpu->n = value & 0x80;
  } else {
    cpu->z = value == 0;
    cpu->n = value & 0x8000;
  }
}

static void cpu_doBranch(Cpu* cpu, uint8_t value, bool check) {
  if(check) {
    cpu->cyclesUsed++; // taken branch: 1 extra cycle
    cpu->pc += (int8_t) value;
  }
}

static uint8_t cpu_pullByte(Cpu* cpu) {
  cpu->sp++;
  if(cpu->e) cpu->sp = (cpu->sp & 0xff) | 0x100;
  return cpu_read(cpu, cpu->sp);
}

static void cpu_pushByte(Cpu* cpu, uint8_t value) {
  cpu_write(cpu, cpu->sp, value);
  cpu->sp--;
  if(cpu->e) cpu->sp = (cpu->sp & 0xff) | 0x100;
}

static uint16_t cpu_pullWord(Cpu* cpu) {
  uint8_t value = cpu_pullByte(cpu);
  return value | (cpu_pullByte(cpu) << 8);
}

static void cpu_pushWord(Cpu* cpu, uint16_t value) {
  cpu_pushByte(cpu, value >> 8);
  cpu_pushByte(cpu, value & 0xff);
}

static uint16_t cpu_readWord(Cpu* cpu, uint32_t adrl, uint32_t adrh) {
  uint8_t value = cpu_read(cpu, adrl);
  return value | (cpu_read(cpu, adrh) << 8);
}

static void cpu_writeWord(Cpu* cpu, uint32_t adrl, uint32_t adrh, uint16_t value, bool reversed) {
  if(reversed) {
    cpu_write(cpu, adrh, value >> 8);
    cpu_write(cpu, adrl, value & 0xff);
  } else {
    cpu_write(cpu, adrl, value & 0xff);
    cpu_write(cpu, adrh, value >> 8);
  }
}

static void cpu_doInterrupt(Cpu* cpu, bool irq) {
  cpu_pushByte(cpu, cpu->k);
  cpu_pushWord(cpu, cpu->pc);
  cpu_pushByte(cpu, cpu_getFlags(cpu));
  cpu->cyclesUsed++; // native mode: 1 extra cycle
  cpu->i = true;
  cpu->d = false;
  cpu->k = 0;
  if(irq) {
    cpu->pc = cpu_readWord(cpu, 0xffee, 0xffef);
  } else {
    // nmi
    cpu->pc = cpu_readWord(cpu, 0xffea, 0xffeb);
  }
}

// addressing modes

static uint32_t cpu_adrImm(Cpu* cpu, uint32_t* low, bool xFlag) {
  if((xFlag && cpu->xf) || (!xFlag && cpu->mf)) {
    *low = (cpu->k << 16) | cpu->pc++;
    return 0;
  } else {
    *low = (cpu->k << 16) | cpu->pc++;
    return (cpu->k << 16) | cpu->pc++;
  }
}

static uint32_t cpu_adrDp(Cpu* cpu, uint32_t* low) {
  uint8_t adr = cpu_readOpcode(cpu);
  if(cpu->dp & 0xff) cpu->cyclesUsed++; // dpr not 0: 1 extra cycle
  *low = (cpu->dp + adr) & 0xffff;
  return (cpu->dp + adr + 1) & 0xffff;
}

static uint32_t cpu_adrDpx(Cpu* cpu, uint32_t* low) {
  uint8_t adr = cpu_readOpcode(cpu);
  if(cpu->dp & 0xff) cpu->cyclesUsed++; // dpr not 0: 1 extra cycle
  *low = (cpu->dp + adr + cpu->x) & 0xffff;
  return (cpu->dp + adr + cpu->x + 1) & 0xffff;
}

static uint32_t cpu_adrDpy(Cpu* cpu, uint32_t* low) {
  uint8_t adr = cpu_readOpcode(cpu);
  if(cpu->dp & 0xff) cpu->cyclesUsed++; // dpr not 0: 1 extra cycle
  *low = (cpu->dp + adr + cpu->y) & 0xffff;
  return (cpu->dp + adr + cpu->y + 1) & 0xffff;
}

static uint32_t cpu_adrIdp(Cpu* cpu, uint32_t* low) {
  uint8_t adr = cpu_readOpcode(cpu);
  if(cpu->dp & 0xff) cpu->cyclesUsed++; // dpr not 0: 1 extra cycle
  uint16_t pointer = cpu_readWord(cpu, (cpu->dp + adr) & 0xffff, (cpu->dp + adr + 1) & 0xffff);
  *low = (cpu->db << 16) + pointer;
  return ((cpu->db << 16) + pointer + 1) & 0xffffff;
}

static uint32_t cpu_adrIdx(Cpu* cpu, uint32_t* low) {
  uint8_t adr = cpu_readOpcode(cpu);
  if(cpu->dp & 0xff) cpu->cyclesUsed++; // dpr not 0: 1 extra cycle
  uint16_t pointer = cpu_readWord(cpu, (cpu->dp + adr + cpu->x) & 0xffff, (cpu->dp + adr + cpu->x + 1) & 0xffff);
  *low = (cpu->db << 16) + pointer;
  return ((cpu->db << 16) + pointer + 1) & 0xffffff;
}

static uint32_t cpu_adrIdy(Cpu* cpu, uint32_t* low, bool write) {
  uint8_t adr = cpu_readOpcode(cpu);
  if(cpu->dp & 0xff) cpu->cyclesUsed++; // dpr not 0: 1 extra cycle
  uint16_t pointer = cpu_readWord(cpu, (cpu->dp + adr) & 0xffff, (cpu->dp + adr + 1) & 0xffff);
  if(write && (!cpu->xf || ((pointer >> 8) != ((pointer + cpu->y) >> 8)))) cpu->cyclesUsed++;
  // x = 0 or page crossed, with writing opcode: 1 extra cycle
  *low = ((cpu->db << 16) + pointer + cpu->y) & 0xffffff;
  return ((cpu->db << 16) + pointer + cpu->y + 1) & 0xffffff;
}

static uint32_t cpu_adrIdl(Cpu* cpu, uint32_t* low) {
  uint8_t adr = cpu_readOpcode(cpu);
  if(cpu->dp & 0xff) cpu->cyclesUsed++; // dpr not 0: 1 extra cycle
  uint32_t pointer = cpu_readWord(cpu, (cpu->dp + adr) & 0xffff, (cpu->dp + adr + 1) & 0xffff);
  pointer |= cpu_read(cpu, (cpu->dp + adr + 2) & 0xffff) << 16;
  *low = pointer;
  return (pointer + 1) & 0xffffff;
}

static uint32_t cpu_adrIly(Cpu* cpu, uint32_t* low) {
  uint8_t adr = cpu_readOpcode(cpu);
  if(cpu->dp & 0xff) cpu->cyclesUsed++; // dpr not 0: 1 extra cycle
  uint32_t pointer = cpu_readWord(cpu, (cpu->dp + adr) & 0xffff, (cpu->dp + adr + 1) & 0xffff);
  pointer |= cpu_read(cpu, (cpu->dp + adr + 2) & 0xffff) << 16;
  *low = (pointer + cpu->y) & 0xffffff;
  return (pointer + cpu->y + 1) & 0xffffff;
}

static uint32_t cpu_adrSr(Cpu* cpu, uint32_t* low) {
  uint8_t adr = cpu_readOpcode(cpu);
  *low = (cpu->sp + adr) & 0xffff;
  return (cpu->sp + adr + 1) & 0xffff;
}

static uint32_t cpu_adrIsy(Cpu* cpu, uint32_t* low) {
  uint8_t adr = cpu_readOpcode(cpu);
  uint16_t pointer = cpu_readWord(cpu, (cpu->sp + adr) & 0xffff, (cpu->sp + adr + 1) & 0xffff);
  *low = ((cpu->db << 16) + pointer + cpu->y) & 0xffffff;
  return ((cpu->db << 16) + pointer + cpu->y + 1) & 0xffffff;
}

static uint32_t cpu_adrAbs(Cpu* cpu, uint32_t* low) {
  uint16_t adr = cpu_readOpcodeWord(cpu);
  *low = (cpu->db << 16) + adr;
  return ((cpu->db << 16) + adr + 1) & 0xffffff;
}

static uint32_t cpu_adrAbx(Cpu* cpu, uint32_t* low, bool write) {
  uint16_t adr = cpu_readOpcodeWord(cpu);
  if(write && (!cpu->xf || ((adr >> 8) != ((adr + cpu->x) >> 8)))) cpu->cyclesUsed++;
  // x = 0 or page crossed, with writing opcode: 1 extra cycle
  *low = ((cpu->db << 16) + adr + cpu->x) & 0xffffff;
  return ((cpu->db << 16) + adr + cpu->x + 1) & 0xffffff;
}

static uint32_t cpu_adrAby(Cpu* cpu, uint32_t* low, bool write) {
  uint16_t adr = cpu_readOpcodeWord(cpu);
  if(write && (!cpu->xf || ((adr >> 8) != ((adr + cpu->y) >> 8)))) cpu->cyclesUsed++;
  // x = 0 or page crossed, with writing opcode: 1 extra cycle
  *low = ((cpu->db << 16) + adr + cpu->y) & 0xffffff;
  return ((cpu->db << 16) + adr + cpu->y + 1) & 0xffffff;
}

static uint32_t cpu_adrAbl(Cpu* cpu, uint32_t* low) {
  uint32_t adr = cpu_readOpcodeWord(cpu);
  adr |= cpu_readOpcode(cpu) << 16;
  *low = adr;
  return (adr + 1) & 0xffffff;
}

static uint32_t cpu_adrAlx(Cpu* cpu, uint32_t* low) {
  uint32_t adr = cpu_readOpcodeWord(cpu);
  adr |= cpu_readOpcode(cpu) << 16;
  *low = (adr + cpu->x) & 0xffffff;
  return (adr + cpu->x + 1) & 0xffffff;
}

static uint16_t cpu_adrIax(Cpu* cpu) {
  uint16_t adr = cpu_readOpcodeWord(cpu);
  return cpu_readWord(cpu, (cpu->k << 16) | ((adr + cpu->x) & 0xffff), (cpu->k << 16) | ((adr + cpu->x + 1) & 0xffff));
}

// opcode functions

static void cpu_and(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low);
    cpu->a = (cpu->a & 0xff00) | ((cpu->a & value) & 0xff);
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    uint16_t value = cpu_readWord(cpu, low, high);
    cpu->a &= value;
  }
  cpu_setZN(cpu, cpu->a, cpu->mf);
}

static void cpu_ora(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low);
    cpu->a = (cpu->a & 0xff00) | ((cpu->a | value) & 0xff);
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    uint16_t value = cpu_readWord(cpu, low, high);
    cpu->a |= value;
  }
  cpu_setZN(cpu, cpu->a, cpu->mf);
}

static void cpu_eor(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low);
    cpu->a = (cpu->a & 0xff00) | ((cpu->a ^ value) & 0xff);
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    uint16_t value = cpu_readWord(cpu, low, high);
    cpu->a ^= value;
  }
  cpu_setZN(cpu, cpu->a, cpu->mf);
}

static void cpu_adc(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low);
    int result = 0;
    if(cpu->d) {
      result = (cpu->a & 0xf) + (value & 0xf) + cpu->c;
      if(result > 0x9) result = ((result + 0x6) & 0xf) + 0x10;
      result = (cpu->a & 0xf0) + (value & 0xf0) + result;
    } else {
      result = (cpu->a & 0xff) + value + cpu->c;
    }
    cpu->v = (cpu->a & 0x80) == (value & 0x80) && (value & 0x80) != (result & 0x80);
    if(cpu->d && result > 0x9f) result += 0x60;
    cpu->c = result > 0xff;
    cpu->a = (cpu->a & 0xff00) | (result & 0xff);
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    uint16_t value = cpu_readWord(cpu, low, high);
    int result = 0;
    if(cpu->d) {
      result = (cpu->a & 0xf) + (value & 0xf) + cpu->c;
      if(result > 0x9) result = ((result + 0x6) & 0xf) + 0x10;
      result = (cpu->a & 0xf0) + (value & 0xf0) + result;
      if(result > 0x9f) result = ((result + 0x60) & 0xff) + 0x100;
      result = (cpu->a & 0xf00) + (value & 0xf00) + result;
      if(result > 0x9ff) result = ((result + 0x600) & 0xfff) + 0x1000;
      result = (cpu->a & 0xf000) + (value & 0xf000) + result;
    } else {
      result = cpu->a + value + cpu->c;
    }
    cpu->v = (cpu->a & 0x8000) == (value & 0x8000) && (value & 0x8000) != (result & 0x8000);
    if(cpu->d && result > 0x9fff) result += 0x6000;
    cpu->c = result > 0xffff;
    cpu->a = result;
  }
  cpu_setZN(cpu, cpu->a, cpu->mf);
}

static void cpu_sbc(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low) ^ 0xff;
    int result = 0;
    if(cpu->d) {
      result = (cpu->a & 0xf) + (value & 0xf) + cpu->c;
      if(result < 0x10) result = (result - 0x6) & ((result - 0x6 < 0) ? 0xf : 0x1f);
      result = (cpu->a & 0xf0) + (value & 0xf0) + result;
    } else {
      result = (cpu->a & 0xff) + value + cpu->c;
    }
    cpu->v = (cpu->a & 0x80) == (value & 0x80) && (value & 0x80) != (result & 0x80);
    if(cpu->d && result < 0x100) result -= 0x60;
    cpu->c = result > 0xff;
    cpu->a = (cpu->a & 0xff00) | (result & 0xff);
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    uint16_t value = cpu_readWord(cpu, low, high) ^ 0xffff;
    int result = 0;
    if(cpu->d) {
      result = (cpu->a & 0xf) + (value & 0xf) + cpu->c;
      if(result < 0x10) result = (result - 0x6) & ((result - 0x6 < 0) ? 0xf : 0x1f);
      result = (cpu->a & 0xf0) + (value & 0xf0) + result;
      if(result < 0x100) result = (result - 0x60) & ((result - 0x60 < 0) ? 0xff : 0x1ff);
      result = (cpu->a & 0xf00) + (value & 0xf00) + result;
      if(result < 0x1000) result = (result - 0x600) & ((result - 0x600 < 0) ? 0xfff : 0x1fff);
      result = (cpu->a & 0xf000) + (value & 0xf000) + result;
    } else {
      result = cpu->a + value + cpu->c;
    }
    cpu->v = (cpu->a & 0x8000) == (value & 0x8000) && (value & 0x8000) != (result & 0x8000);
    if(cpu->d && result < 0x10000) result -= 0x6000;
    cpu->c = result > 0xffff;
    cpu->a = result;
  }
  cpu_setZN(cpu, cpu->a, cpu->mf);
}

static void cpu_cmp(Cpu* cpu, uint32_t low, uint32_t high) {
  int result = 0;
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low) ^ 0xff;
    result = (cpu->a & 0xff) + value + 1;
    cpu->c = result > 0xff;
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    uint16_t value = cpu_readWord(cpu, low, high) ^ 0xffff;
    result = cpu->a + value + 1;
    cpu->c = result > 0xffff;
  }
  cpu_setZN(cpu, result, cpu->mf);
}

static void cpu_cpx(Cpu* cpu, uint32_t low, uint32_t high) {
  int result = 0;
  if(cpu->xf) {
    uint8_t value = cpu_read(cpu, low) ^ 0xff;
    result = (cpu->x & 0xff) + value + 1;
    cpu->c = result > 0xff;
  } else {
    cpu->cyclesUsed++; // x = 0: 1 extra cycle
    uint16_t value = cpu_readWord(cpu, low, high) ^ 0xffff;
    result = cpu->x + value + 1;
    cpu->c = result > 0xffff;
  }
  cpu_setZN(cpu, result, cpu->xf);
}

static void cpu_cpy(Cpu* cpu, uint32_t low, uint32_t high) {
  int result = 0;
  if(cpu->xf) {
    uint8_t value = cpu_read(cpu, low) ^ 0xff;
    result = (cpu->y & 0xff) + value + 1;
    cpu->c = result > 0xff;
  } else {
    cpu->cyclesUsed++; // x = 0: 1 extra cycle
    uint16_t value = cpu_readWord(cpu, low, high) ^ 0xffff;
    result = cpu->y + value + 1;
    cpu->c = result > 0xffff;
  }
  cpu_setZN(cpu, result, cpu->xf);
}

static void cpu_bit(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low);
    uint8_t result = (cpu->a & 0xff) & value;
    cpu->z = result == 0;
    cpu->n = value & 0x80;
    cpu->v = value & 0x40;
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    uint16_t value = cpu_readWord(cpu, low, high);
    uint16_t result = cpu->a & value;
    cpu->z = result == 0;
    cpu->n = value & 0x8000;
    cpu->v = value & 0x4000;
  }
}

static void cpu_lda(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    cpu->a = (cpu->a & 0xff00) | cpu_read(cpu, low);
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    cpu->a = cpu_readWord(cpu, low, high);
  }
  cpu_setZN(cpu, cpu->a, cpu->mf);
}

static void cpu_ldx(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->xf) {
    cpu->x = cpu_read(cpu, low);
  } else {
    cpu->cyclesUsed++; // x = 0: 1 extra cycle
    cpu->x = cpu_readWord(cpu, low, high);
  }
  cpu_setZN(cpu, cpu->x, cpu->xf);
}

static void cpu_ldy(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->xf) {
    cpu->y = cpu_read(cpu, low);
  } else {
    cpu->cyclesUsed++; // x = 0: 1 extra cycle
    cpu->y = cpu_readWord(cpu, low, high);
  }
  cpu_setZN(cpu, cpu->y, cpu->xf);
}

static void cpu_sta(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    cpu_write(cpu, low, (uint8_t)cpu->a);
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    cpu_writeWord(cpu, low, high, cpu->a, false);
  }
}

static void cpu_stx(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->xf) {
    cpu_write(cpu, low, (uint8_t)cpu->x);
  } else {
    cpu->cyclesUsed++; // x = 0: 1 extra cycle
    cpu_writeWord(cpu, low, high, cpu->x, false);
  }
}

static void cpu_sty(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->xf) {
    cpu_write(cpu, low, (uint8_t)cpu->y);
  } else {
    cpu->cyclesUsed++; // x = 0: 1 extra cycle
    cpu_writeWord(cpu, low, high, cpu->y, false);
  }
}

static void cpu_stz(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    cpu_write(cpu, low, 0);
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    cpu_writeWord(cpu, low, high, 0, false);
  }
}

static void cpu_ror(Cpu* cpu, uint32_t low, uint32_t high) {
  bool carry = false;
  int result = 0;
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low);
    carry = value & 1;
    result = (value >> 1) | (cpu->c << 7);
    cpu_write(cpu, low, result);
  } else {
    cpu->cyclesUsed += 2; // m = 0: 2 extra cycles
    uint16_t value = cpu_readWord(cpu, low, high);
    carry = value & 1;
    result = (value >> 1) | (cpu->c << 15);
    cpu_writeWord(cpu, low, high, result, true);
  }
  cpu_setZN(cpu, result, cpu->mf);
  cpu->c = carry;
}

static void cpu_rol(Cpu* cpu, uint32_t low, uint32_t high) {
  int result = 0;
  if(cpu->mf) {
    result = (cpu_read(cpu, low) << 1) | (uint8_t)cpu->c;
    cpu->c = result & 0x100;
    cpu_write(cpu, low, result);
  } else {
    cpu->cyclesUsed += 2; // m = 0: 2 extra cycles
    result = (cpu_readWord(cpu, low, high) << 1) | (uint8_t)cpu->c;
    cpu->c = result & 0x10000;
    cpu_writeWord(cpu, low, high, result, true);
  }
  cpu_setZN(cpu, result, cpu->mf);
}

static void cpu_lsr(Cpu* cpu, uint32_t low, uint32_t high) {
  int result = 0;
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low);
    cpu->c = value & 1;
    result = value >> 1;
    cpu_write(cpu, low, result);
  } else {
    cpu->cyclesUsed += 2; // m = 0: 2 extra cycles
    uint16_t value = cpu_readWord(cpu, low, high);
    cpu->c = value & 1;
    result = value >> 1;
    cpu_writeWord(cpu, low, high, result, true);
  }
  cpu_setZN(cpu, result, cpu->mf);
}

static void cpu_asl(Cpu* cpu, uint32_t low, uint32_t high) {
  int result = 0;
  if(cpu->mf) {
    result = cpu_read(cpu, low) << 1;
    cpu->c = result & 0x100;
    cpu_write(cpu, low, result);
  } else {
    cpu->cyclesUsed += 2; // m = 0: 2 extra cycles
    result = cpu_readWord(cpu, low, high) << 1;
    cpu->c = result & 0x10000;
    cpu_writeWord(cpu, low, high, result, true);
  }
  cpu_setZN(cpu, result, cpu->mf);
}

static void cpu_inc(Cpu* cpu, uint32_t low, uint32_t high) {
  int result = 0;
  if(cpu->mf) {
    result = cpu_read(cpu, low) + 1;
    cpu_write(cpu, low, result);
  } else {
    cpu->cyclesUsed += 2; // m = 0: 2 extra cycles
    result = cpu_readWord(cpu, low, high) + 1;
    cpu_writeWord(cpu, low, high, result, true);
  }
  cpu_setZN(cpu, result, cpu->mf);
}

static void cpu_dec(Cpu* cpu, uint32_t low, uint32_t high) {
  int result = 0;
  if(cpu->mf) {
    result = cpu_read(cpu, low) - 1;
    cpu_write(cpu, low, result);
  } else {
    cpu->cyclesUsed += 2; // m = 0: 2 extra cycles
    result = cpu_readWord(cpu, low, high) - 1;
    cpu_writeWord(cpu, low, high, result, true);
  }
  cpu_setZN(cpu, result, cpu->mf);
}

static void cpu_tsb(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low);
    cpu->z = ((cpu->a & 0xff) & value) == 0;
    cpu_write(cpu, low, value | (cpu->a & 0xff));
  } else {
    cpu->cyclesUsed += 2; // m = 0: 2 extra cycles
    uint16_t value = cpu_readWord(cpu, low, high);
    cpu->z = (cpu->a & value) == 0;
    cpu_writeWord(cpu, low, high, value | cpu->a, true);
  }
}

static void cpu_trb(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low);
    cpu->z = ((cpu->a & 0xff) & value) == 0;
    cpu_write(cpu, low, value & ~(cpu->a & 0xff));
  } else {
    cpu->cyclesUsed += 2; // m = 0: 2 extra cycles
    uint16_t value = cpu_readWord(cpu, low, high);
    cpu->z = (cpu->a & value) == 0;
    cpu_writeWord(cpu, low, high, value & ~cpu->a, true);
  }
}

static inline void HookedFunctionRts(int is_long) { (void)is_long; }

static void cpu_doOpcode(Cpu* cpu, uint8_t opcode) {
  switch(opcode) {
    case 0x00: { // brk imp
      cpu_readOpcode(cpu);
      if (!cpu->e) cpu_pushByte(cpu, cpu->k);
      cpu_pushWord(cpu, cpu->pc);
      cpu_pushByte(cpu, cpu_getFlags(cpu));
      cpu->cyclesUsed++;
      cpu->i = true;
      cpu->d = false;
      cpu->k = 0;
      uint16_t vec = cpu->e ? 0xfffe : 0xffe6;
      cpu->pc = cpu_readWord(cpu, vec, vec + 1);
      break;
    }
    case 0x01: { // ora idx
      uint32_t low = 0;
      uint32_t high = cpu_adrIdx(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x02: { // cop imm(s)
      cpu_readOpcode(cpu);
      cpu_pushByte(cpu, cpu->k);
      cpu_pushWord(cpu, cpu->pc);
      cpu_pushByte(cpu, cpu_getFlags(cpu));
      cpu->cyclesUsed++; // native mode: 1 extra cycle
      cpu->i = true;
      cpu->d = false;
      cpu->k = 0;
      cpu->pc = cpu_readWord(cpu, 0xffe4, 0xffe5);
      break;
    }
    case 0x03: { // ora sr
      uint32_t low = 0;
      uint32_t high = cpu_adrSr(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x04: { // tsb dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_tsb(cpu, low, high);
      break;
    }
    case 0x05: { // ora dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x06: { // asl dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_asl(cpu, low, high);
      break;
    }
    case 0x07: { // ora idl
      uint32_t low = 0;
      uint32_t high = cpu_adrIdl(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x08: { // php imp
      cpu_pushByte(cpu, cpu_getFlags(cpu));
      break;
    }
    case 0x09: { // ora imm(m)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, false);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x0a: { // asla imp
      if(cpu->mf) {
        cpu->c = cpu->a & 0x80;
        cpu->a = (cpu->a & 0xff00) | ((cpu->a << 1) & 0xff);
      } else {
        cpu->c = cpu->a & 0x8000;
        cpu->a <<= 1;
      }
      cpu_setZN(cpu, cpu->a, cpu->mf);
      break;
    }
    case 0x0b: { // phd imp
      cpu_pushWord(cpu, cpu->dp);
      break;
    }
    case 0x0c: { // tsb abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_tsb(cpu, low, high);
      break;
    }
    case 0x0d: { // ora abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x0e: { // asl abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_asl(cpu, low, high);
      break;
    }
    case 0x0f: { // ora abl
      uint32_t low = 0;
      uint32_t high = cpu_adrAbl(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x10: { // bpl rel
      cpu_doBranch(cpu, cpu_readOpcode(cpu), !cpu->n);
      break;
    }
    case 0x11: { // ora idy(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrIdy(cpu, &low, false);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x12: { // ora idp
      uint32_t low = 0;
      uint32_t high = cpu_adrIdp(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x13: { // ora isy
      uint32_t low = 0;
      uint32_t high = cpu_adrIsy(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x14: { // trb dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_trb(cpu, low, high);
      break;
    }
    case 0x15: { // ora dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x16: { // asl dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_asl(cpu, low, high);
      break;
    }
    case 0x17: { // ora ily
      uint32_t low = 0;
      uint32_t high = cpu_adrIly(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x18: { // clc imp
      cpu->c = false;
      break;
    }
    case 0x19: { // ora aby(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAby(cpu, &low, false);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x1a: { // inca imp
      if(cpu->mf) {
        cpu->a = (cpu->a & 0xff00) | ((cpu->a + 1) & 0xff);
      } else {
        cpu->a++;
      }
      cpu_setZN(cpu, cpu->a, cpu->mf);
      break;
    }
    case 0x1b: { // tcs imp
      cpu->sp = cpu->a;
      break;
    }
    case 0x1c: { // trb abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_trb(cpu, low, high);
      break;
    }
    case 0x1d: { // ora abx(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, false);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x1e: { // asl abx
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, true);
      cpu_asl(cpu, low, high);
      break;
    }
    case 0x1f: { // ora alx
      uint32_t low = 0;
      uint32_t high = cpu_adrAlx(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x20: { // jsr abs
      uint16_t value = cpu_readOpcodeWord(cpu);
      cpu_pushWord(cpu, cpu->pc - 1);
      cpu->pc = value;
      break;
    }
    case 0x21: { // and idx
      uint32_t low = 0;
      uint32_t high = cpu_adrIdx(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x22: { // jsl abl
      uint16_t value = cpu_readOpcodeWord(cpu);
      uint8_t newK = cpu_readOpcode(cpu);
      cpu_pushByte(cpu, cpu->k);
      cpu_pushWord(cpu, cpu->pc - 1);
      cpu->pc = value;
      cpu->k = newK;
      break;
    }
    case 0x23: { // and sr
      uint32_t low = 0;
      uint32_t high = cpu_adrSr(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x24: { // bit dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_bit(cpu, low, high);
      break;
    }
    case 0x25: { // and dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x26: { // rol dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_rol(cpu, low, high);
      break;
    }
    case 0x27: { // and idl
      uint32_t low = 0;
      uint32_t high = cpu_adrIdl(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x28: { // plp imp
      cpu_setFlags(cpu, cpu_pullByte(cpu));
      break;
    }
    case 0x29: { // and imm(m)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, false);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x2a: { // rola imp
      int result = (cpu->a << 1) | (uint8_t)cpu->c;
      if(cpu->mf) {
        cpu->c = result & 0x100;
        cpu->a = (cpu->a & 0xff00) | (result & 0xff);
      } else {
        cpu->c = result & 0x10000;
        cpu->a = result;
      }
      cpu_setZN(cpu, cpu->a, cpu->mf);
      break;
    }
    case 0x2b: { // pld imp
      cpu->dp = cpu_pullWord(cpu);
      cpu_setZN(cpu, cpu->dp, false);
      break;
    }
    case 0x2c: { // bit abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_bit(cpu, low, high);
      break;
    }
    case 0x2d: { // and abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x2e: { // rol abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_rol(cpu, low, high);
      break;
    }
    case 0x2f: { // and abl
      uint32_t low = 0;
      uint32_t high = cpu_adrAbl(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x30: { // bmi rel
      cpu_doBranch(cpu, cpu_readOpcode(cpu), cpu->n);
      break;
    }
    case 0x31: { // and idy(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrIdy(cpu, &low, false);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x32: { // and idp
      uint32_t low = 0;
      uint32_t high = cpu_adrIdp(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x33: { // and isy
      uint32_t low = 0;
      uint32_t high = cpu_adrIsy(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x34: { // bit dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_bit(cpu, low, high);
      break;
    }
    case 0x35: { // and dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x36: { // rol dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_rol(cpu, low, high);
      break;
    }
    case 0x37: { // and ily
      uint32_t low = 0;
      uint32_t high = cpu_adrIly(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x38: { // sec imp
      cpu->c = true;
      break;
    }
    case 0x39: { // and aby(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAby(cpu, &low, false);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x3a: { // deca imp
      if(cpu->mf) {
        cpu->a = (cpu->a & 0xff00) | ((cpu->a - 1) & 0xff);
      } else {
        cpu->a--;
      }
      cpu_setZN(cpu, cpu->a, cpu->mf);
      break;
    }
    case 0x3b: { // tsc imp
      cpu->a = cpu->sp;
      cpu_setZN(cpu, cpu->a, false);
      break;
    }
    case 0x3c: { // bit abx(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, false);
      cpu_bit(cpu, low, high);
      break;
    }
    case 0x3d: { // and abx(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, false);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x3e: { // rol abx
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, true);
      cpu_rol(cpu, low, high);
      break;
    }
    case 0x3f: { // and alx
      uint32_t low = 0;
      uint32_t high = cpu_adrAlx(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x40: { // rti imp
      cpu_setFlags(cpu, cpu_pullByte(cpu));
      cpu->cyclesUsed++; // native mode: 1 extra cycle
      cpu->pc = cpu_pullWord(cpu);
      cpu->k = cpu_pullByte(cpu);
      break;
    }
    case 0x41: { // eor idx
      uint32_t low = 0;
      uint32_t high = cpu_adrIdx(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x42: { // wdm imm(s)
      cpu_readOpcode(cpu);
      break;
    }
    case 0x43: { // eor sr
      uint32_t low = 0;
      uint32_t high = cpu_adrSr(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x44: { // mvp bm
      uint8_t dest = cpu_readOpcode(cpu);
      uint8_t src = cpu_readOpcode(cpu);
      cpu->db = dest;
      cpu_write(cpu, (dest << 16) | cpu->y, cpu_read(cpu, (src << 16) | cpu->x));
      cpu->a--;
      cpu->x--;
      cpu->y--;
      if(cpu->a != 0xffff) {
        cpu->pc -= 3;
      }
      if(cpu->xf) {
        cpu->x &= 0xff;
        cpu->y &= 0xff;
      }
      break;
    }
    case 0x45: { // eor dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x46: { // lsr dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_lsr(cpu, low, high);
      break;
    }
    case 0x47: { // eor idl
      uint32_t low = 0;
      uint32_t high = cpu_adrIdl(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x48: { // pha imp
      if(cpu->mf) {
        cpu_pushByte(cpu, (uint8_t)cpu->a);
      } else {
        cpu->cyclesUsed++; // m = 0: 1 extra cycle
        cpu_pushWord(cpu, cpu->a);
      }
      break;
    }
    case 0x49: { // eor imm(m)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, false);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x4a: { // lsra imp
      cpu->c = cpu->a & 1;
      if(cpu->mf) {
        cpu->a = (cpu->a & 0xff00) | ((cpu->a >> 1) & 0x7f);
      } else {
        cpu->a >>= 1;
      }
      cpu_setZN(cpu, cpu->a, cpu->mf);
      break;
    }
    case 0x4b: { // phk imp
      cpu_pushByte(cpu, cpu->k);
      break;
    }
    case 0x4c: { // jmp abs
      cpu->pc = cpu_readOpcodeWord(cpu);
      break;
    }
    case 0x4d: { // eor abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x4e: { // lsr abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_lsr(cpu, low, high);
      break;
    }
    case 0x4f: { // eor abl
      uint32_t low = 0;
      uint32_t high = cpu_adrAbl(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x50: { // bvc rel
      cpu_doBranch(cpu, cpu_readOpcode(cpu), !cpu->v);
      break;
    }
    case 0x51: { // eor idy(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrIdy(cpu, &low, false);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x52: { // eor idp
      uint32_t low = 0;
      uint32_t high = cpu_adrIdp(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x53: { // eor isy
      uint32_t low = 0;
      uint32_t high = cpu_adrIsy(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x54: { // mvn bm
      uint8_t dest = cpu_readOpcode(cpu);
      uint8_t src = cpu_readOpcode(cpu);
      cpu->db = dest;
      cpu_write(cpu, (dest << 16) | cpu->y, cpu_read(cpu, (src << 16) | cpu->x));
      cpu->a--;
      cpu->x++;
      cpu->y++;
      if(cpu->a != 0xffff) {
        cpu->pc -= 3;
      }
      if(cpu->xf) {
        cpu->x &= 0xff;
        cpu->y &= 0xff;
      }
      break;
    }
    case 0x55: { // eor dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x56: { // lsr dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_lsr(cpu, low, high);
      break;
    }
    case 0x57: { // eor ily
      uint32_t low = 0;
      uint32_t high = cpu_adrIly(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x58: { // cli imp
      cpu->i = false;
      break;
    }
    case 0x59: { // eor aby(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAby(cpu, &low, false);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x5a: { // phy imp
      if(cpu->xf) {
        cpu_pushByte(cpu, (uint8_t)cpu->y);
      } else {
        cpu->cyclesUsed++; // m = 0: 1 extra cycle
        cpu_pushWord(cpu, cpu->y);
      }
      break;
    }
    case 0x5b: { // tcd imp
      cpu->dp = cpu->a;
      cpu_setZN(cpu, cpu->dp, false);
      break;
    }
    case 0x5c: { // jml abl
      uint16_t value = cpu_readOpcodeWord(cpu);
      cpu->k = cpu_readOpcode(cpu);
      cpu->pc = value;
      break;
    }
    case 0x5d: { // eor abx(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, false);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x5e: { // lsr abx
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, true);
      cpu_lsr(cpu, low, high);
      break;
    }
    case 0x5f: { // eor alx
      uint32_t low = 0;
      uint32_t high = cpu_adrAlx(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x60: { // rts imp
      //if (cpu->spBreakpoint)
      //  fprintf(stderr, "0x%x: rts 0x%x 0x%x\n", cpu->k<<16 | cpu->pc, cpu->spBreakpoint, cpu->sp);
      if (cpu->sp >= cpu->spBreakpoint && cpu->spBreakpoint) {
        assert(cpu->sp == cpu->spBreakpoint);
        cpu->spBreakpoint = 0;
        HookedFunctionRts(0);
      }
      cpu->pc = cpu_pullWord(cpu) + 1;
      break;
    }
    case 0x61: { // adc idx
      uint32_t low = 0;
      uint32_t high = cpu_adrIdx(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x62: { // per rll
      uint16_t value = cpu_readOpcodeWord(cpu);
      cpu_pushWord(cpu, cpu->pc + (int16_t) value);
      break;
    }
    case 0x63: { // adc sr
      uint32_t low = 0;
      uint32_t high = cpu_adrSr(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x64: { // stz dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_stz(cpu, low, high);
      break;
    }
    case 0x65: { // adc dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x66: { // ror dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_ror(cpu, low, high);
      break;
    }
    case 0x67: { // adc idl
      uint32_t low = 0;
      uint32_t high = cpu_adrIdl(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x68: { // pla imp
      if(cpu->mf) {
        cpu->a = (cpu->a & 0xff00) | cpu_pullByte(cpu);
      } else {
        cpu->cyclesUsed++; // 16-bit m: 1 extra cycle
        cpu->a = cpu_pullWord(cpu);
      }
      cpu_setZN(cpu, cpu->a, cpu->mf);
      break;
    }
    case 0x69: { // adc imm(m)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, false);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x6a: { // rora imp
      bool carry = cpu->a & 1;
      if(cpu->mf) {
        cpu->a = (cpu->a & 0xff00) | ((cpu->a >> 1) & 0x7f) | (cpu->c << 7);
      } else {
        cpu->a = (cpu->a >> 1) | (cpu->c << 15);
      }
      cpu->c = carry;
      cpu_setZN(cpu, cpu->a, cpu->mf);
      break;
    }
    case 0x6b: { // rtl imp
      //if (cpu->spBreakpoint)
      //  fprintf(stderr, "0x%x: rtl 0x%x 0x%x\n", cpu->k<<16 | cpu->pc, cpu->spBreakpoint, cpu->sp);

      if (cpu->sp >= cpu->spBreakpoint && cpu->spBreakpoint) {
        assert(cpu->sp == cpu->spBreakpoint);
        cpu->spBreakpoint = 0;
        HookedFunctionRts(1);
      }
      cpu->pc = cpu_pullWord(cpu) + 1;
      cpu->k = cpu_pullByte(cpu);
      break;
    }
    case 0x6c: { // jmp ind
      uint16_t adr = cpu_readOpcodeWord(cpu);
      cpu->pc = cpu_readWord(cpu, adr, (adr + 1) & 0xffff);
      break;
    }
    case 0x6d: { // adc abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x6e: { // ror abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_ror(cpu, low, high);
      break;
    }
    case 0x6f: { // adc abl
      uint32_t low = 0;
      uint32_t high = cpu_adrAbl(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x70: { // bvs rel
      cpu_doBranch(cpu, cpu_readOpcode(cpu), cpu->v);
      break;
    }
    case 0x71: { // adc idy(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrIdy(cpu, &low, false);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x72: { // adc idp
      uint32_t low = 0;
      uint32_t high = cpu_adrIdp(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x73: { // adc isy
      uint32_t low = 0;
      uint32_t high = cpu_adrIsy(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x74: { // stz dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_stz(cpu, low, high);
      break;
    }
    case 0x75: { // adc dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x76: { // ror dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_ror(cpu, low, high);
      break;
    }
    case 0x77: { // adc ily
      uint32_t low = 0;
      uint32_t high = cpu_adrIly(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x78: { // sei imp
      cpu->i = true;
      break;
    }
    case 0x79: { // adc aby(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAby(cpu, &low, false);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x7a: { // ply imp
      if(cpu->xf) {
        cpu->y = cpu_pullByte(cpu);
      } else {
        cpu->cyclesUsed++; // 16-bit x: 1 extra cycle
        cpu->y = cpu_pullWord(cpu);
      }
      cpu_setZN(cpu, cpu->y, cpu->xf);
      break;
    }
    case 0x7b: { // tdc imp
      cpu->a = cpu->dp;
      cpu_setZN(cpu, cpu->a, false);
      break;
    }
    case 0x7c: { // jmp iax
      cpu->pc = cpu_adrIax(cpu);
      break;
    }
    case 0x7d: { // adc abx(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, false);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x7e: { // ror abx
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, true);
      cpu_ror(cpu, low, high);
      break;
    }
    case 0x7f: { // adc alx
      uint32_t low = 0;
      uint32_t high = cpu_adrAlx(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x80: { // bra rel
      cpu->pc += (int8_t) cpu_readOpcode(cpu);
      break;
    }
    case 0x81: { // sta idx
      uint32_t low = 0;
      uint32_t high = cpu_adrIdx(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x82: { // brl rll
      cpu->pc += (int16_t) cpu_readOpcodeWord(cpu);
      break;
    }
    case 0x83: { // sta sr
      uint32_t low = 0;
      uint32_t high = cpu_adrSr(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x84: { // sty dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_sty(cpu, low, high);
      break;
    }
    case 0x85: { // sta dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x86: { // stx dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_stx(cpu, low, high);
      break;
    }
    case 0x87: { // sta idl
      uint32_t low = 0;
      uint32_t high = cpu_adrIdl(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x88: { // dey imp
      if(cpu->xf) {
        cpu->y = (cpu->y - 1) & 0xff;
      } else {
        cpu->y--;
      }
      cpu_setZN(cpu, cpu->y, cpu->xf);
      break;
    }
    case 0x89: { // biti imm(m)
      if(cpu->mf) {
        uint8_t result = (cpu->a & 0xff) & cpu_readOpcode(cpu);
        cpu->z = result == 0;
      } else {
        cpu->cyclesUsed++; // m = 0: 1 extra cycle
        uint16_t result = cpu->a & cpu_readOpcodeWord(cpu);
        cpu->z = result == 0;
      }
      break;
    }
    case 0x8a: { // txa imp
      if(cpu->mf) {
        cpu->a = (cpu->a & 0xff00) | (cpu->x & 0xff);
      } else {
        cpu->a = cpu->x;
      }
      cpu_setZN(cpu, cpu->a, cpu->mf);
      break;
    }
    case 0x8b: { // phb imp
      cpu_pushByte(cpu, cpu->db);
      break;
    }
    case 0x8c: { // sty abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_sty(cpu, low, high);
      break;
    }
    case 0x8d: { // sta abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x8e: { // stx abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_stx(cpu, low, high);
      break;
    }
    case 0x8f: { // sta abl
      uint32_t low = 0;
      uint32_t high = cpu_adrAbl(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x90: { // bcc rel
      cpu_doBranch(cpu, cpu_readOpcode(cpu), !cpu->c);
      break;
    }
    case 0x91: { // sta idy
      uint32_t low = 0;
      uint32_t high = cpu_adrIdy(cpu, &low, true);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x92: { // sta idp
      uint32_t low = 0;
      uint32_t high = cpu_adrIdp(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x93: { // sta isy
      uint32_t low = 0;
      uint32_t high = cpu_adrIsy(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x94: { // sty dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_sty(cpu, low, high);
      break;
    }
    case 0x95: { // sta dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x96: { // stx dpy
      uint32_t low = 0;
      uint32_t high = cpu_adrDpy(cpu, &low);
      cpu_stx(cpu, low, high);
      break;
    }
    case 0x97: { // sta ily
      uint32_t low = 0;
      uint32_t high = cpu_adrIly(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x98: { // tya imp
      if(cpu->mf) {
        cpu->a = (cpu->a & 0xff00) | (cpu->y & 0xff);
      } else {
        cpu->a = cpu->y;
      }
      cpu_setZN(cpu, cpu->a, cpu->mf);
      break;
    }
    case 0x99: { // sta aby
      uint32_t low = 0;
      uint32_t high = cpu_adrAby(cpu, &low, true);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x9a: { // txs imp
      cpu->sp = cpu->x;
      break;
    }
    case 0x9b: { // txy imp
      if(cpu->xf) {
        cpu->y = cpu->x & 0xff;
      } else {
        cpu->y = cpu->x;
      }
      cpu_setZN(cpu, cpu->y, cpu->xf);
      break;
    }
    case 0x9c: { // stz abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_stz(cpu, low, high);
      break;
    }
    case 0x9d: { // sta abx
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, true);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x9e: { // stz abx
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, true);
      cpu_stz(cpu, low, high);
      break;
    }
    case 0x9f: { // sta alx
      uint32_t low = 0;
      uint32_t high = cpu_adrAlx(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0xa0: { // ldy imm(x)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, true);
      cpu_ldy(cpu, low, high);
      break;
    }
    case 0xa1: { // lda idx
      uint32_t low = 0;
      uint32_t high = cpu_adrIdx(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xa2: { // ldx imm(x)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, true);
      cpu_ldx(cpu, low, high);
      break;
    }
    case 0xa3: { // lda sr
      uint32_t low = 0;
      uint32_t high = cpu_adrSr(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xa4: { // ldy dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_ldy(cpu, low, high);
      break;
    }
    case 0xa5: { // lda dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xa6: { // ldx dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_ldx(cpu, low, high);
      break;
    }
    case 0xa7: { // lda idl
      uint32_t low = 0;
      uint32_t high = cpu_adrIdl(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xa8: { // tay imp
      if(cpu->xf) {
        cpu->y = cpu->a & 0xff;
      } else {
        cpu->y = cpu->a;
      }
      cpu_setZN(cpu, cpu->y, cpu->xf);
      break;
    }
    case 0xa9: { // lda imm(m)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, false);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xaa: { // tax imp
      if(cpu->xf) {
        cpu->x = cpu->a & 0xff;
      } else {
        cpu->x = cpu->a;
      }
      cpu_setZN(cpu, cpu->x, cpu->xf);
      break;
    }
    case 0xab: { // plb imp
      cpu->db = cpu_pullByte(cpu);
      cpu_setZN(cpu, cpu->db, true);
      break;
    }
    case 0xac: { // ldy abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_ldy(cpu, low, high);
      break;
    }
    case 0xad: { // lda abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xae: { // ldx abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_ldx(cpu, low, high);
      break;
    }
    case 0xaf: { // lda abl
      uint32_t low = 0;
      uint32_t high = cpu_adrAbl(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xb0: { // bcs rel
      cpu_doBranch(cpu, cpu_readOpcode(cpu), cpu->c);
      break;
    }
    case 0xb1: { // lda idy(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrIdy(cpu, &low, false);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xb2: { // lda idp
      uint32_t low = 0;
      uint32_t high = cpu_adrIdp(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xb3: { // lda isy
      uint32_t low = 0;
      uint32_t high = cpu_adrIsy(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xb4: { // ldy dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_ldy(cpu, low, high);
      break;
    }
    case 0xb5: { // lda dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xb6: { // ldx dpy
      uint32_t low = 0;
      uint32_t high = cpu_adrDpy(cpu, &low);
      cpu_ldx(cpu, low, high);
      break;
    }
    case 0xb7: { // lda ily
      uint32_t low = 0;
      uint32_t high = cpu_adrIly(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xb8: { // clv imp
      cpu->v = false;
      break;
    }
    case 0xb9: { // lda aby(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAby(cpu, &low, false);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xba: { // tsx imp
      if(cpu->xf) {
        cpu->x = cpu->sp & 0xff;
      } else {
        cpu->x = cpu->sp;
      }
      cpu_setZN(cpu, cpu->x, cpu->xf);
      break;
    }
    case 0xbb: { // tyx imp
      if(cpu->xf) {
        cpu->x = cpu->y & 0xff;
      } else {
        cpu->x = cpu->y;
      }
      cpu_setZN(cpu, cpu->x, cpu->xf);
      break;
    }
    case 0xbc: { // ldy abx(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, false);
      cpu_ldy(cpu, low, high);
      break;
    }
    case 0xbd: { // lda abx(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, false);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xbe: { // ldx aby(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAby(cpu, &low, false);
      cpu_ldx(cpu, low, high);
      break;
    }
    case 0xbf: { // lda alx
      uint32_t low = 0;
      uint32_t high = cpu_adrAlx(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xc0: { // cpy imm(x)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, true);
      cpu_cpy(cpu, low, high);
      break;
    }
    case 0xc1: { // cmp idx
      uint32_t low = 0;
      uint32_t high = cpu_adrIdx(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xc2: { // rep imm(s)
      cpu_setFlags(cpu, cpu_getFlags(cpu) & ~cpu_readOpcode(cpu));
      break;
    }
    case 0xc3: { // cmp sr
      uint32_t low = 0;
      uint32_t high = cpu_adrSr(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xc4: { // cpy dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_cpy(cpu, low, high);
      break;
    }
    case 0xc5: { // cmp dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xc6: { // dec dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_dec(cpu, low, high);
      break;
    }
    case 0xc7: { // cmp idl
      uint32_t low = 0;
      uint32_t high = cpu_adrIdl(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xc8: { // iny imp
      if(cpu->xf) {
        cpu->y = (cpu->y + 1) & 0xff;
      } else {
        cpu->y++;
      }
      cpu_setZN(cpu, cpu->y, cpu->xf);
      break;
    }
    case 0xc9: { // cmp imm(m)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, false);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xca: { // dex imp
      if(cpu->xf) {
        cpu->x = (cpu->x - 1) & 0xff;
      } else {
        cpu->x--;
      }
      cpu_setZN(cpu, cpu->x, cpu->xf);
      break;
    }
    case 0xcb: { // wai imp
      cpu->waiting = true;
      break;
    }
    case 0xcc: { // cpy abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_cpy(cpu, low, high);
      break;
    }
    case 0xcd: { // cmp abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xce: { // dec abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_dec(cpu, low, high);
      break;
    }
    case 0xcf: { // cmp abl
      uint32_t low = 0;
      uint32_t high = cpu_adrAbl(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xd0: { // bne rel
      cpu_doBranch(cpu, cpu_readOpcode(cpu), !cpu->z);
      break;
    }
    case 0xd1: { // cmp idy(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrIdy(cpu, &low, false);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xd2: { // cmp idp
      uint32_t low = 0;
      uint32_t high = cpu_adrIdp(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xd3: { // cmp isy
      uint32_t low = 0;
      uint32_t high = cpu_adrIsy(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xd4: { // pei dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_pushWord(cpu, cpu_readWord(cpu, low, high));
      break;
    }
    case 0xd5: { // cmp dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xd6: { // dec dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_dec(cpu, low, high);
      break;
    }
    case 0xd7: { // cmp ily
      uint32_t low = 0;
      uint32_t high = cpu_adrIly(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xd8: { // cld imp
      cpu->d = false;
      break;
    }
    case 0xd9: { // cmp aby(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAby(cpu, &low, false);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xda: { // phx imp
      if(cpu->xf) {
        cpu_pushByte(cpu, (uint8_t)cpu->x);
      } else {
        cpu->cyclesUsed++; // m = 0: 1 extra cycle
        cpu_pushWord(cpu, cpu->x);
      }
      break;
    }
    case 0xdb: { // stp imp
      cpu->stopped = true;
      break;
    }
    case 0xdc: { // jml ial
      uint16_t adr = cpu_readOpcodeWord(cpu);
      cpu->pc = cpu_readWord(cpu, adr, (adr + 1) & 0xffff);
      cpu->k = cpu_read(cpu, (adr + 2) & 0xffff);
      break;
    }
    case 0xdd: { // cmp abx(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, false);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xde: { // dec abx
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, true);
      cpu_dec(cpu, low, high);
      break;
    }
    case 0xdf: { // cmp alx
      uint32_t low = 0;
      uint32_t high = cpu_adrAlx(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xe0: { // cpx imm(x)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, true);
      cpu_cpx(cpu, low, high);
      break;
    }
    case 0xe1: { // sbc idx
      uint32_t low = 0;
      uint32_t high = cpu_adrIdx(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xe2: { // sep imm(s)
      cpu_setFlags(cpu, cpu_getFlags(cpu) | cpu_readOpcode(cpu));
      break;
    }
    case 0xe3: { // sbc sr
      uint32_t low = 0;
      uint32_t high = cpu_adrSr(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xe4: { // cpx dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_cpx(cpu, low, high);
      break;
    }
    case 0xe5: { // sbc dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xe6: { // inc dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_inc(cpu, low, high);
      break;
    }
    case 0xe7: { // sbc idl
      uint32_t low = 0;
      uint32_t high = cpu_adrIdl(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xe8: { // inx imp
      if(cpu->xf) {
        cpu->x = (cpu->x + 1) & 0xff;
      } else {
        cpu->x++;
      }
      cpu_setZN(cpu, cpu->x, cpu->xf);
      break;
    }
    case 0xe9: { // sbc imm(m)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, false);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xea: { // nop imp
      // no operation
      break;
    }
    case 0xeb: { // xba imp
      uint8_t low = cpu->a & 0xff;
      uint8_t high = cpu->a >> 8;
      cpu->a = (low << 8) | high;
      cpu_setZN(cpu, high, true);
      break;
    }
    case 0xec: { // cpx abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_cpx(cpu, low, high);
      break;
    }
    case 0xed: { // sbc abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xee: { // inc abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_inc(cpu, low, high);
      break;
    }
    case 0xef: { // sbc abl
      uint32_t low = 0;
      uint32_t high = cpu_adrAbl(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xf0: { // beq rel
      cpu_doBranch(cpu, cpu_readOpcode(cpu), cpu->z);
      break;
    }
    case 0xf1: { // sbc idy(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrIdy(cpu, &low, false);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xf2: { // sbc idp
      uint32_t low = 0;
      uint32_t high = cpu_adrIdp(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xf3: { // sbc isy
      uint32_t low = 0;
      uint32_t high = cpu_adrIsy(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xf4: { // pea imm(l)
      cpu_pushWord(cpu, cpu_readOpcodeWord(cpu));
      break;
    }
    case 0xf5: { // sbc dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xf6: { // inc dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_inc(cpu, low, high);
      break;
    }
    case 0xf7: { // sbc ily
      uint32_t low = 0;
      uint32_t high = cpu_adrIly(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xf8: { // sed imp
      cpu->d = true;
      break;
    }
    case 0xf9: { // sbc aby(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAby(cpu, &low, false);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xfa: { // plx imp
      if(cpu->xf) {
        cpu->x = cpu_pullByte(cpu);
      } else {
        cpu->cyclesUsed++; // 16-bit x: 1 extra cycle
        cpu->x = cpu_pullWord(cpu);
      }
      cpu_setZN(cpu, cpu->x, cpu->xf);
      break;
    }
    case 0xfb: { // xce imp
      bool temp = cpu->c;
      cpu->c = cpu->e;
      cpu->e = temp;
      cpu_setFlags(cpu, cpu_getFlags(cpu)); // updates x and m flags, clears upper half of x and y if needed
      break;
    }
    case 0xfc: { // jsr iax
      uint16_t value = cpu_adrIax(cpu);
      cpu_pushWord(cpu, cpu->pc - 1);
      cpu->pc = value;
      break;
    }
    case 0xfd: { // sbc abx(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, false);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xfe: { // inc abx
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, true);
      cpu_inc(cpu, low, high);
      break;
    }
    case 0xff: { // sbc alx
      uint32_t low = 0;
      uint32_t high = cpu_adrAlx(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
  }
}


/* === snes.c === */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stddef.h>












static void snes_catchupApu(Snes* snes);
static uint8_t snes_readReg(Snes* snes, uint16_t adr);
static void snes_writeReg(Snes* snes, uint16_t adr, uint8_t val);
static uint8_t snes_rread(Snes* snes, uint32_t adr); // wrapped by read, to set open bus
static int snes_getAccessTime(Snes* snes, uint32_t adr);

Snes* snes_init(uint8_t *ram) {
  Snes* snes = (Snes * )malloc(sizeof(Snes));
  snes->ram = ram;
  snes->cpu = cpu_init(snes, 0);
  snes->apu = apu_init();
  snes->dma = dma_init(snes);
  snes->ppu = ppu_init();
  snes->cart = cart_init(snes);
  snes->input1 = input_init(snes);
  snes->input2 = input_init(snes);
  snes->debug_cycles = false;
  snes->disableHpos = false;
  return snes;
}

void snes_free(Snes* snes) {
  cpu_free(snes->cpu);
  apu_free(snes->apu);
  dma_free(snes->dma);
  ppu_free(snes->ppu);
  cart_free(snes->cart);
  input_free(snes->input1);
  input_free(snes->input2);
  free(snes);
}

void snes_saveload(Snes *snes, SaveLoadFunc *func, void *ctx) {
  cpu_saveload(snes->cpu, func, ctx);
  apu_saveload(snes->apu, func, ctx);
  dma_saveload(snes->dma, func, ctx);
  ppu_saveload(snes->ppu, func, ctx);
  cart_saveload(snes->cart, func, ctx);

  func(ctx, &snes->hPos, offsetof(Snes, openBus) + 1 - offsetof(Snes, hPos));
  func(ctx, snes->ram, 0x20000);
  func(ctx, &snes->ramAdr, 4);
    

  snes->disableHpos = false;
}

void snes_reset(Snes* snes, bool hard) {
  cart_reset(snes->cart); // reset cart first, because resetting cpu will read from it (reset vector)
  cpu_reset(snes->cpu);
  apu_reset(snes->apu);
  dma_reset(snes->dma);
  ppu_reset(snes->ppu);
  input_reset(snes->input1);
  input_reset(snes->input2);
  if (hard) memset(snes->ram, 0, 0x20000);
  snes->ramAdr = 0;
  snes->hPos = 0;
  snes->vPos = 0;
  snes->frames = 0;
  snes->cpuCyclesLeft = 52; // 5 reads (8) + 2 IntOp (6)
  snes->cpuMemOps = 0;
  snes->apuCatchupCycles = 0.0;
  snes->hIrqEnabled = false;
  snes->vIrqEnabled = false;
  snes->nmiEnabled = false;
  snes->hTimer = 0x1ff;
  snes->vTimer = 0x1ff;
  snes->inNmi = false;
  snes->inIrq = false;
  snes->inVblank = false;
  memset(snes->portAutoRead, 0, sizeof(snes->portAutoRead));
  snes->autoJoyRead = false;
  snes->autoJoyTimer = 0;
  snes->ppuLatch = true;
  snes->multiplyA = 0xff;
  snes->multiplyResult = 0xfe01;
  snes->divideA = 0xffff;
  snes->divideResult = 0x101;
  snes->fastMem = false;
  snes->openBus = 0;
}

void snes_printCpuLine(Snes *snes) {
  if (snes->debug_cycles) {
    static FILE *fout;

    if (!fout)
      fout = stdout;
    char line[80];
    line[0] = 0;
    fputs(line, fout);
    fprintf(fout, " 0x%x", snes->ppu->vram[0]);
    fputs("\n", fout);
    fflush(fout);
  }
}

static void snes_catchupApu(Snes* snes) {
  int catchupCycles = (int) snes->apuCatchupCycles;
  for(int i = 0; i < catchupCycles; i++) {
    apu_cycle(snes->apu);
  }
  snes->apuCatchupCycles -= (double) catchupCycles;
}

void snes_doAutoJoypad(Snes* snes) {
  // TODO: improve? (now calls input_cycle)
  memset(snes->portAutoRead, 0, sizeof(snes->portAutoRead));
  snes->input1->latchLine = true;
  snes->input2->latchLine = true;
  input_cycle(snes->input1); // latches the controllers
  input_cycle(snes->input2);
  snes->input1->latchLine = false;
  snes->input2->latchLine = false;
  for(int i = 0; i < 16; i++) {
    uint8_t val = input_read(snes->input1);
    snes->portAutoRead[0] |= ((val & 1) << (15 - i));
    snes->portAutoRead[2] |= (((val >> 1) & 1) << (15 - i));
    val = input_read(snes->input2);
    snes->portAutoRead[1] |= ((val & 1) << (15 - i));
    snes->portAutoRead[3] |= (((val >> 1) & 1) << (15 - i));
  }
}

uint8_t snes_readBBus(Snes* snes, uint8_t adr) {
  if(adr < 0x40) {
    return ppu_read(snes->ppu, adr);
  }
  if(adr < 0x80) {
    snes_catchupApu(snes);
    return snes->apu->outPorts[adr & 0x3];
  }
  if(adr == 0x80) {
    uint8_t ret = snes->ram[snes->ramAdr++];
    snes->ramAdr &= 0x1ffff;
    return ret;
  }
  return snes->openBus;
}

void snes_writeBBus(Snes* snes, uint8_t adr, uint8_t val) {
  if(adr < 0x40) {
    ppu_write(snes->ppu, adr, val);
    return;
  }
  if(adr < 0x80) {
    snes_catchupApu(snes); // catch up the apu before writing
    snes->apu->inPorts[adr & 0x3] = val;
    return;
  }
  switch(adr) {
    case 0x80: {
      snes->ram[snes->ramAdr++] = val;
      snes->ramAdr &= 0x1ffff;
      break;
    }
    case 0x81: {
      snes->ramAdr = (snes->ramAdr & 0x1ff00) | val;
      break;
    }
    case 0x82: {
      snes->ramAdr = (snes->ramAdr & 0x100ff) | (val << 8);
      break;
    }
    case 0x83: {
      snes->ramAdr = (snes->ramAdr & 0x0ffff) | ((val & 1) << 16);
      break;
    }
  }
}

static uint8_t snes_readReg(Snes* snes, uint16_t adr) {
  switch(adr) {
    case 0x4210: {
      uint8_t val = 0x2; // CPU version (4 bit)
      val |= snes->inNmi << 7;
      snes->inNmi = false;
      return val | (snes->openBus & 0x70);
    }
    case 0x4211: {
      uint8_t val = snes->inIrq << 7;
      snes->inIrq = false;
      snes->cpu->irqWanted = (snes->cart && cart_cpu_irq_pending(snes->cart));
      return val | (snes->openBus & 0x7f);
    }
    case 0x4212: {
      uint8_t val = (snes->autoJoyTimer > 0);
      val |= (snes->hPos >= 1024) << 6;
      val |= snes->inVblank << 7;
      return val | (snes->openBus & 0x3e);
    }
    case 0x4213: {
      return snes->ppuLatch << 7; // IO-port
    }
    case 0x4214: {
      return snes->divideResult & 0xff;
    }
    case 0x4215: {
      return snes->divideResult >> 8;
    }
    case 0x4216: {
      return snes->multiplyResult & 0xff;
    }
    case 0x4217: {
      return snes->multiplyResult >> 8;
    }
    case 0x4218:
    case 0x421a:
    case 0x421c:
    case 0x421e: {
      return snes->portAutoRead[(adr - 0x4218) / 2] & 0xff;
    }
    case 0x4219:
    case 0x421b:
    case 0x421d:
    case 0x421f: {
      return snes->portAutoRead[(adr - 0x4219) / 2] >> 8;
    }
    default: {
      return snes->openBus;
    }
  }
}

static void snes_writeReg(Snes* snes, uint16_t adr, uint8_t val) {
  switch(adr) {
    case 0x4200: {
      snes->autoJoyRead = val & 0x1;
      if(!snes->autoJoyRead) snes->autoJoyTimer = 0;
      snes->hIrqEnabled = val & 0x10;
      snes->vIrqEnabled = val & 0x20;
      if(!snes->nmiEnabled && (val & 0x80) && snes->inNmi) {
        snes->cpu->nmiWanted = true;
      }
      snes->nmiEnabled = val & 0x80;
      if(!snes->hIrqEnabled && !snes->vIrqEnabled) {
        snes->inIrq = false;
        snes->cpu->irqWanted = (snes->cart && cart_cpu_irq_pending(snes->cart));
      }
      break;
    }
    case 0x4201: {
      if(!(val & 0x80) && snes->ppuLatch) {
        // latch the ppu
        ppu_read(snes->ppu, 0x37);
      }
      snes->ppuLatch = val & 0x80;
      break;
    }
    case 0x4202: {
      snes->multiplyA = val;
      break;
    }
    case 0x4203: {
      snes->multiplyResult = snes->multiplyA * val;
      break;
    }
    case 0x4204: {
      snes->divideA = (snes->divideA & 0xff00) | val;
      break;
    }
    case 0x4205: {
      snes->divideA = (snes->divideA & 0x00ff) | (val << 8);
      break;
    }
    case 0x4206: {
      if(val == 0) {
        snes->divideResult = 0xffff;
        snes->multiplyResult = snes->divideA;
      } else {
        snes->divideResult = snes->divideA / val;
        snes->multiplyResult = snes->divideA % val;
      }
      break;
    }
    case 0x4207: {
      snes->hTimer = (snes->hTimer & 0x100) | val;
      break;
    }
    case 0x4208: {
      snes->hTimer = (snes->hTimer & 0x0ff) | ((val & 1) << 8);
      break;
    }
    case 0x4209: {
      snes->vTimer = (snes->vTimer & 0x100) | val;
      break;
    }
    case 0x420a: {
      snes->vTimer = (snes->vTimer & 0x0ff) | ((val & 1) << 8);
      break;
    }
    case 0x420b: {
      dma_startDma(snes->dma, val, false);
      break;
    }
    case 0x420c: {
      dma_startDma(snes->dma, val, true);
      break;
    }
    case 0x420d: {
      snes->fastMem = val & 0x1;
      break;
    }
    default: {
      break;
    }
  }
}

static uint8_t snes_rread(Snes* snes, uint32_t adr) {
  uint8_t bank = adr >> 16;
  adr &= 0xffff;
  if((bank & 0x7f) < 0x40 && adr < 0x4380) {
    if(adr < 0x2000) {
      return snes->ram[adr]; // ram mirror
    }
    if(adr >= 0x2100 && adr < 0x2200) {
      return snes_readBBus(snes, adr & 0xff); // B-bus
    }
    if(adr == 0x4016) {
      return input_read(snes->input1) | (snes->openBus & 0xfc);
    }
    if(adr == 0x4017) {
      return input_read(snes->input2) | (snes->openBus & 0xe0) | 0x1c;
    }
    if(adr >= 0x4200 && adr < 0x4220) {
      return snes_readReg(snes, adr); // internal registers
    }
    if(adr >= 0x4300 && adr < 0x4380) {
      return dma_read(snes->dma, adr); // dma registers
    }
  } else if ((bank & ~1) == 0x7e) {
    return snes->ram[((bank & 1) << 16) | adr]; // ram
  }

  // read from cart
  return cart_read(snes->cart, bank, adr);
}

int g_bp_addr = 0;


void snes_write(Snes* snes, uint32_t adr, uint8_t val) {
  snes->openBus = val;
  uint8_t bank = adr >> 16;
  adr &= 0xffff;
  if(bank == 0x7e || bank == 0x7f) {
    if ((adr & 0xffff) == g_bp_addr && g_bp_addr) {
      printf("@0x%x: Writing1 0x%X to 0x%x (frame %d) %.2x %.2x %.2x\n", snes->cpu->k * 65536 + snes->cpu->pc, val, adr & 0xffff, snes->ram[0x1a],
        snes->ram[snes->cpu->sp+1], snes->ram[snes->cpu->sp+2], snes->ram[snes->cpu->sp+3]);
    }

    snes->ram[((bank & 1) << 16) | adr] = val; // ram
  } else if(bank < 0x40 || (bank >= 0x80 && bank < 0xc0)) {
    if (adr < 0x2000) {
      if ((adr & 0xffff) == g_bp_addr && g_bp_addr) {
        printf("@0x%x: Writing2 0x%X to 0x%x (frame %d) %.2x %.2x %.2x\n", snes->cpu->k * 65536 + snes->cpu->pc, val, adr & 0xffff, snes->ram[0x1a],
          snes->ram[snes->cpu->sp+1], snes->ram[snes->cpu->sp+2], snes->ram[snes->cpu->sp+3]);
      }

      snes->ram[adr] = val; // ram mirror
    } else if(adr >= 0x2100 && adr < 0x2200) {
      snes_writeBBus(snes, adr & 0xff, val); // B-bus
    } else if(adr == 0x4016) {
      snes->input1->latchLine = val & 1;
      snes->input2->latchLine = val & 1;
      if (val & 1) {
        input_cycle(snes->input1);
        input_cycle(snes->input2);
      }
    } else if(adr >= 0x4200 && adr < 0x4220) {
      snes_writeReg(snes, adr, val); // internal registers
    } else if(adr >= 0x4300 && adr < 0x4380) {
      dma_write(snes->dma, adr, val); // dma registers
    }
  }
  cart_write(snes->cart, bank, adr, val);
  // write to cart
}

static int snes_getAccessTime(Snes* snes, uint32_t adr) {
  // optimization
  return 6;
}

uint8_t snes_read(Snes* snes, uint32_t adr) {
  uint8_t val = snes_rread(snes, adr);
  snes->openBus = val;
  return val;
}

uint8_t snes_cpuRead(Snes* snes, uint32_t adr) {
  snes->cpuMemOps++;
  snes->cpuCyclesLeft += snes_getAccessTime(snes, adr);
  if (snes->cart) snes->cart->cpuBusAddress = adr & 0xffffffu;
  return snes_read(snes, adr);
}

void snes_cpuWrite(Snes* snes, uint32_t adr, uint8_t val) {
  snes->cpuMemOps++;
  snes->cpuCyclesLeft += snes_getAccessTime(snes, adr);
  if (snes->cart) snes->cart->cpuBusAddress = adr & 0xffffffu;
  snes_write(snes, adr, val);
}

// debugging



/* =========================================================================
 * SNESRecomp Public Runtime API & Widescreen / Audio Bridge
 * ========================================================================= */
/* g_snes_rom_size and g_snes_rom_data are defined in game_recomp.c */

#define HW_PPU_SCRATCH_WIDTH (256 + kPpuExtraLeftRight * 2)
static uint32_t g_ppu_scratch_fb[HW_PPU_SCRATCH_WIDTH * 240];

#define AUDIO_RING_CAPACITY 16384
static int16_t  g_audio_ring[AUDIO_RING_CAPACITY];
static size_t   g_audio_ring_read = 0;
static size_t   g_audio_ring_count = 0;

static const double kApuCyclesPerMaster = (32040.0 * 32.0) / (1364.0 * 262.0 * 60.0);

static void hw_sync_to_ctx(snes_context_t* ctx) {
    if (!ctx || !ctx->hw_snes) return;
    Snes* snes = (Snes*)ctx->hw_snes;
    ctx->cpu.A      = snes->cpu->a;
    ctx->cpu.X      = snes->cpu->x;
    ctx->cpu.Y      = snes->cpu->y;
    ctx->cpu.SP     = snes->cpu->sp;
    ctx->cpu.DP     = snes->cpu->dp;
    ctx->cpu.DB     = snes->cpu->db;
    ctx->cpu.PB     = snes->cpu->k;
    ctx->cpu.PC     = snes->cpu->pc;
    ctx->cpu.flag_e = snes->cpu->e;
    ctx->cpu.P      = cpu_getFlags(snes->cpu);

    ctx->inidisp    = snes->ppu->inidisp;
    ctx->obsel      = snes->ppu->obsel;
    ctx->bg_mode    = snes->ppu->bgmode & 0x07;
    ctx->tm         = snes->ppu->screenEnabled[0];
    ctx->ts         = snes->ppu->screenEnabled[1];
    ctx->frame_count = snes->frames;

    if (snes->cart && snes->cart->ram && snes->cart->ramSize > 0) {
        size_t copy_sz = snes->cart->ramSize;
        if (copy_sz > sizeof(ctx->sram)) copy_sz = sizeof(ctx->sram);
        memcpy(ctx->sram, snes->cart->ram, copy_sz);
    }
}

static void hw_copy_framebuffers(snes_context_t* ctx) {
    int extra_lr = ctx->widescreen_enabled ? ctx->extra_left_right : 0;
    int out_w = 256 + extra_lr * 2;
    if (out_w > SNES_ULTRA_WIDESCREEN_WIDTH) out_w = SNES_ULTRA_WIDESCREEN_WIDTH;
    int start_col = kPpuExtraLeftRight - extra_lr;

    ctx->frame_width   = out_w;
    ctx->render_width  = out_w;
    ctx->frame_height  = SNES_SCREEN_HEIGHT;
    ctx->render_height = SNES_SCREEN_HEIGHT;

    for (int y = 0; y < SNES_SCREEN_HEIGHT; y++) {
        const uint32_t* src_row = &g_ppu_scratch_fb[y * HW_PPU_SCRATCH_WIDTH];
        const uint32_t* wide_row = src_row + start_col;
        uint32_t* dst_wide = &ctx->widescreen_framebuffer[y * out_w];
        uint32_t* dst_fb   = &ctx->framebuffer[y * out_w];
        for (int x = 0; x < out_w; x++) {
            uint32_t px = wide_row[x] | 0xFF000000u;
            dst_wide[x] = px;
            dst_fb[x]   = px;
        }
    }
}

static void hw_run_single_frame(snes_context_t* ctx, Snes* snes) {
    uint32_t start_frame = snes->frames;
    snes->apu->dsp->sampleOffset = 0;

    while (snes->frames == start_frame) {
        snes->apuCatchupCycles += kApuCyclesPerMaster * 2.0;
        if (snes->cart) {
            snes->cart->masterClock += 2;
        }

        if (!dma_cycle(snes->dma)) {
            if (snes->cart && snes->cart->sa1 && (snes->cpu->waiting || snes->cpuCyclesLeft <= 0)) {
                cart_sync_coprocessors(snes->cart);
                if (cart_cpu_irq_pending(snes->cart)) {
                    snes->cpu->irqWanted = true;
                } else if (!snes->inIrq) {
                    snes->cpu->irqWanted = false;
                }
            }
            if (snes->cpuCyclesLeft <= 0) {
                snes->cpuMemOps = 0;
                int cycles = cpu_runOpcode(snes->cpu);
                int extra = (cycles - snes->cpuMemOps) * 6;
                if (extra < 0) extra = 0;
                snes->cpuCyclesLeft += extra;
            }
            if (snes->cpuCyclesLeft >= 2) snes->cpuCyclesLeft -= 2;
            else snes->cpuCyclesLeft = 0;
        }

        if (snes->vIrqEnabled && snes->hIrqEnabled) {
            if (snes->vPos == snes->vTimer && snes->hPos == (4 * snes->hTimer)) {
                snes->inIrq = true;
                snes->cpu->irqWanted = true;
            }
        } else if (snes->vIrqEnabled && !snes->hIrqEnabled) {
            if (snes->vPos == snes->vTimer && snes->hPos == 0) {
                snes->inIrq = true;
                snes->cpu->irqWanted = true;
            }
        } else if (!snes->vIrqEnabled && snes->hIrqEnabled) {
            if (snes->hPos == (4 * snes->hTimer)) {
                snes->inIrq = true;
                snes->cpu->irqWanted = true;
            }
        }

        if (snes->hPos == 0) {
            bool startingVblank = false;
            if (snes->vPos == 0) {
                snes->inVblank = false;
                snes->inNmi = false;
                dma_initHdma(snes->dma);
            } else if (snes->vPos == 225) {
                startingVblank = !ppu_checkOverscan(snes->ppu);
            } else if (snes->vPos == 240) {
                if (!snes->inVblank) startingVblank = true;
            }
            if (startingVblank) {
                ppu_handleVblank(snes->ppu);
                snes->inVblank = true;
                snes->inNmi = true;
                if (snes->nmiEnabled) {
                    snes->cpu->nmiWanted = true;
                }
                if (snes->autoJoyRead) {
                    snes->autoJoyTimer = 4224;
                    snes_doAutoJoypad(snes);
                }
            }
        } else if (snes->hPos == 512) {
            if (!snes->inVblank) {
                ppu_runLine(snes->ppu, snes->vPos);
            }
        } else if (snes->hPos == 1024) {
            if (!snes->inVblank) {
                dma_doHdma(snes->dma);
            }
        }

        if (snes->autoJoyTimer > 0) {
            if (snes->autoJoyTimer >= 2) snes->autoJoyTimer -= 2;
            else snes->autoJoyTimer = 0;
        }

        snes->hPos += 2;
        if (snes->hPos >= 1364) {
            snes->hPos = 0;
            while (snes->apuCatchupCycles >= 1.0) {
                apu_cycle(snes->apu);
                snes->apuCatchupCycles -= 1.0;
            }
            snes->vPos++;
            if (snes->vPos >= 262) {
                snes->vPos = 0;
                snes->frames++;
            }
        }
    }

    /* Resample frame audio from S-DSP (534 stereo samples -> 735 stereo samples @ 44.1kHz) */
    int16_t frame_audio[SNES_AUDIO_SAMPLES_PER_FRAME * 2];
    dsp_getSamples(snes->apu->dsp, frame_audio, SNES_AUDIO_SAMPLES_PER_FRAME, 2);
    memcpy(ctx->audio_buffer, frame_audio, sizeof(frame_audio));
    ctx->audio_sample_count = SNES_AUDIO_SAMPLES_PER_FRAME * 2;

    for (int i = 0; i < SNES_AUDIO_SAMPLES_PER_FRAME * 2; i++) {
        if (g_audio_ring_count < AUDIO_RING_CAPACITY) {
            size_t write_idx = (g_audio_ring_read + g_audio_ring_count) % AUDIO_RING_CAPACITY;
            g_audio_ring[write_idx] = frame_audio[i];
            g_audio_ring_count++;
        } else {
            size_t write_idx = g_audio_ring_read;
            g_audio_ring[write_idx] = frame_audio[i];
            g_audio_ring_read = (g_audio_ring_read + 1) % AUDIO_RING_CAPACITY;
        }
    }
}

snes_context_t* snes_runtime_init(void) {
    snes_context_t* ctx = (snes_context_t*)calloc(1, sizeof(snes_context_t));
    if (!ctx) return NULL;

    ctx->cpu.context        = ctx;
    ctx->widescreen_enabled = true;
    ctx->extra_left_right   = SNES_WIDESCREEN_EXTRA_SIDE;
    ctx->frame_width        = SNES_WIDESCREEN_WIDTH;
    ctx->frame_height       = SNES_SCREEN_HEIGHT;
    ctx->render_width       = SNES_WIDESCREEN_WIDTH;
    ctx->render_height      = SNES_SCREEN_HEIGHT;
    ctx->active_framebuffer = ctx->widescreen_framebuffer;
    snprintf(ctx->rom_title, sizeof(ctx->rom_title), "SNES Port");

    Snes* hw = snes_init(ctx->wram);
    ctx->hw_snes = hw;
    ctx->hw      = hw;
    ctx->hw_rom_loaded = false;
    ctx->rom_loaded    = false;
    if (hw && hw->ppu) {
        PpuSetExtraSideSpace(hw->ppu, ctx->extra_left_right, ctx->extra_left_right, 0);
    }

    g_audio_ring_read = 0;
    g_audio_ring_count = 0;

    snes_runtime_reset(ctx);

#ifdef SNES_HAS_EMBEDDED_ROM
    if (!ctx->hw_rom_loaded && g_embedded_rom_size >= 0x8000) {
        snes_runtime_load_rom_memory(ctx, g_embedded_rom, (size_t)g_embedded_rom_size);
    }
#endif
    /* Automatically load embedded ROM if present in game_recomp.c */
    if (!ctx->hw_rom_loaded && g_snes_rom_size >= 0x8000) {
        snes_runtime_load_rom_memory(ctx, g_snes_rom_data, (size_t)g_snes_rom_size);
    }
    if (!ctx->hw_rom_loaded) {
        const char* fallbacks[] = { "game.sfc", "game.smc", "sm.smc", "smw.sfc", "zelda3.sfc" };
        for (int i = 0; i < 5 && !ctx->hw_rom_loaded; i++) {
            snes_runtime_load_rom(ctx, fallbacks[i]);
        }
    }
    snes_runtime_load_sram(ctx, "game.srm");

    return ctx;
}

void snes_runtime_free(snes_context_t* ctx) {
    if (!ctx) return;
    if (ctx->hw_snes) {
        snes_free((Snes*)ctx->hw_snes);
        ctx->hw_snes = NULL;
        ctx->hw = NULL;
    }
    if (ctx->owned_rom) {
        free(ctx->owned_rom);
        ctx->owned_rom = NULL;
    }
    free(ctx);
}

void snes_runtime_reset(snes_context_t* ctx) {
    if (!ctx) return;

    if (ctx->hw_rom_loaded && ctx->hw_snes) {
        Snes* hw = (Snes*)ctx->hw_snes;
        snes_reset(hw, false);
        int extra_lr = ctx->widescreen_enabled ? ctx->extra_left_right : 0;
        PpuSetExtraSideSpace(hw->ppu, extra_lr, extra_lr, 0);
        ctx->is_running = true;
        hw_sync_to_ctx(ctx);
        return;
    }

    memset(&ctx->cpu, 0, sizeof(snes_cpu_t));
    ctx->cpu.context = ctx;
    ctx->cpu.SP = 0x01FF;
    ctx->cpu.flag_e = true;
    ctx->cpu.P = FLAG_M | FLAG_X | FLAG_I;
    ctx->inidisp = 0x0F;
    ctx->tm = 0x1F;
    ctx->is_running = true;

    for (int i = 0; i < 256; i++) {
        uint8_t r = (uint8_t)((i * 37) & 0xFF);
        uint8_t g = (uint8_t)((i * 67) & 0xFF);
        uint8_t b = (uint8_t)((i * 97) & 0xFF);
        ctx->palette_cache[i] = (0xFFu << 24) | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    }
}

bool snes_runtime_load_rom_memory(snes_context_t* ctx, const uint8_t* data, size_t size) {
    if (!ctx || !data || size < 0x8000 || !ctx->hw_snes) return false;
    Snes* hw = (Snes*)ctx->hw_snes;
    if (!snes_loadRom(hw, (uint8_t*)data, (int)size)) {
        return false;
    }
    ctx->hw_rom_loaded = true;
    ctx->rom_loaded    = true;
    int extra_lr = ctx->widescreen_enabled ? ctx->extra_left_right : 0;
    PpuSetExtraSideSpace(hw->ppu, extra_lr, extra_lr, 0);

    /* Extract clean ASCII title from ROM header */
    const uint8_t* raw = data;
    size_t raw_sz = size;
    if (raw_sz >= 0x8200 && (raw_sz % 1024) == 512) {
        raw += 512;
        raw_sz -= 512;
    }
    size_t hdr_off = (hw->cart && hw->cart->type == 3) ? 0x40FFC0 :
                     (hw->cart && hw->cart->type == 2) ? 0xFFC0 : 0x7FC0;
    if (hdr_off + 21 <= raw_sz) {
        int last_non_space = -1;
        for (int i = 0; i < 21; i++) {
            uint8_t c = raw[hdr_off + i];
            ctx->rom_title[i] = (c >= 0x20 && c < 0x7F) ? (char)c : ' ';
            if (ctx->rom_title[i] != ' ') last_non_space = i;
        }
        ctx->rom_title[last_non_space + 1] = '\0';
    }
    hw_sync_to_ctx(ctx);
    return true;
}

bool snes_runtime_load_rom(snes_context_t* ctx, const char* rom_path) {
    if (!ctx || !rom_path || !rom_path[0]) return false;
    FILE* f = fopen(rom_path, "rb");
    if (!f) return false;
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return false;
    }
    long len = ftell(f);
    if (len < 0x8000) {
        fclose(f);
        return false;
    }
    rewind(f);
    uint8_t* buf = (uint8_t*)malloc((size_t)len);
    if (!buf) {
        fclose(f);
        return false;
    }
    size_t read_bytes = fread(buf, 1, (size_t)len, f);
    fclose(f);
    bool ok = snes_runtime_load_rom_memory(ctx, buf, read_bytes);
    free(buf);
    return ok;
}

bool snes_runtime_load_rom_file(snes_context_t* ctx, const char* rom_path) {
    return snes_runtime_load_rom(ctx, rom_path);
}

bool snes_runtime_is_rom_loaded(const snes_context_t* ctx) {
    return ctx && ctx->hw_rom_loaded;
}

const char* snes_runtime_get_rom_title(const snes_context_t* ctx) {
    if (!ctx || !ctx->rom_title[0]) return "SNES Port";
    return ctx->rom_title;
}

void snes_runtime_set_extra_side_space(snes_context_t* ctx, int extra_lr) {
    if (!ctx) return;
    if (extra_lr < 0) extra_lr = 0;
    if (extra_lr > kPpuExtraLeftRight) extra_lr = kPpuExtraLeftRight;
    ctx->extra_left_right   = extra_lr;
    ctx->widescreen_enabled = (extra_lr > 0);
    ctx->frame_width        = 256 + extra_lr * 2;
    ctx->render_width       = ctx->frame_width;
    ctx->active_framebuffer = ctx->widescreen_enabled ? ctx->widescreen_framebuffer : ctx->framebuffer;
    if (ctx->hw_snes) {
        Snes* hw = (Snes*)ctx->hw_snes;
        if (hw->ppu) {
            PpuSetExtraSideSpace(hw->ppu, extra_lr, extra_lr, 0);
        }
    }
}

int snes_runtime_get_extra_side_space(const snes_context_t* ctx) {
    return ctx ? ctx->extra_left_right : 0;
}

void snes_runtime_set_widescreen(snes_context_t* ctx, bool enabled) {
    if (!ctx) return;
    snes_runtime_set_extra_side_space(ctx, enabled ? SNES_WIDESCREEN_EXTRA_SIDE : 0);
}

bool snes_runtime_get_widescreen(const snes_context_t* ctx) {
    return ctx ? ctx->widescreen_enabled : false;
}

const uint32_t* snes_runtime_get_framebuffer(const snes_context_t* ctx) {
    if (!ctx) return NULL;
    return ctx->active_framebuffer ? ctx->active_framebuffer : ctx->framebuffer;
}

int snes_runtime_get_frame_width(const snes_context_t* ctx) {
    if (!ctx) return SNES_SCREEN_WIDTH;
    return ctx->frame_width > 0 ? ctx->frame_width : SNES_SCREEN_WIDTH;
}

int snes_runtime_get_frame_height(const snes_context_t* ctx) {
    if (!ctx) return SNES_SCREEN_HEIGHT;
    return ctx->frame_height > 0 ? ctx->frame_height : SNES_SCREEN_HEIGHT;
}

size_t snes_runtime_get_audio_samples(snes_context_t* ctx, int16_t* out_samples, size_t max_samples) {
    (void)ctx;
    if (!out_samples || max_samples == 0) return 0;
    size_t count = g_audio_ring_count < max_samples ? g_audio_ring_count : max_samples;
    for (size_t i = 0; i < count; i++) {
        out_samples[i] = g_audio_ring[g_audio_ring_read];
        g_audio_ring_read = (g_audio_ring_read + 1) % AUDIO_RING_CAPACITY;
    }
    g_audio_ring_count -= count;
    return count;
}

bool snes_runtime_save_sram(const snes_context_t* ctx, const char* path) {
    if (!ctx || !path || !path[0]) return false;
    FILE* f = fopen(path, "wb");
    if (!f) return false;
    const uint8_t* src = ctx->sram;
    size_t sz = 0x8000;
    if (ctx->hw_rom_loaded && ctx->hw_snes) {
        Snes* hw = (Snes*)ctx->hw_snes;
        if (hw->cart && hw->cart->ram && hw->cart->ramSize > 0) {
            src = hw->cart->ram;
            sz  = hw->cart->ramSize;
        }
    }
    size_t written = fwrite(src, 1, sz, f);
    fclose(f);
    return written == sz;
}

bool snes_runtime_load_sram(snes_context_t* ctx, const char* path) {
    if (!ctx || !path || !path[0]) return false;
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    size_t read_bytes = fread(ctx->sram, 1, sizeof(ctx->sram), f);
    fclose(f);
    if (read_bytes > 0 && ctx->hw_rom_loaded && ctx->hw_snes) {
        Snes* hw = (Snes*)ctx->hw_snes;
        if (hw->cart && hw->cart->ram && hw->cart->ramSize > 0) {
            size_t copy_sz = read_bytes < hw->cart->ramSize ? read_bytes : hw->cart->ramSize;
            memcpy(hw->cart->ram, ctx->sram, copy_sz);
        }
    }
    return read_bytes > 0;
}

typedef struct {
    FILE* f;
    bool  ok;
} FileSaveLoadCtx;

static void file_save_cb(void* user, void* data, size_t size) {
    FileSaveLoadCtx* c = (FileSaveLoadCtx*)user;
    if (!c->ok) return;
    if (fwrite(data, 1, size, c->f) != size) c->ok = false;
}

static void file_load_cb(void* user, void* data, size_t size) {
    FileSaveLoadCtx* c = (FileSaveLoadCtx*)user;
    if (!c->ok) return;
    if (fread(data, 1, size, c->f) != size) c->ok = false;
}

bool snes_runtime_save_state(const snes_context_t* ctx, const char* path) {
    if (!ctx || !path || !path[0]) return false;
    FILE* f = fopen(path, "wb");
    if (!f) return false;
    uint32_t magic = 0x53525332u; /* "SRS2" */
    bool ok = (fwrite(&magic, sizeof(magic), 1, f) == 1);
    if (ctx->hw_rom_loaded && ctx->hw_snes) {
        Snes* hw = (Snes*)ctx->hw_snes;
        FileSaveLoadCtx sctx = { f, ok };
        snes_saveload(hw, &file_save_cb, &sctx);
        file_save_cb(&sctx, hw->ppu->vram_bgmode, sizeof(hw->ppu->vram_bgmode));
        ok = sctx.ok;
    } else {
        ok = ok && (fwrite(&ctx->cpu, sizeof(ctx->cpu), 1, f) == 1);
        ok = ok && (fwrite(ctx->wram, sizeof(ctx->wram), 1, f) == 1);
    }
    fclose(f);
    return ok;
}

bool snes_runtime_load_state(snes_context_t* ctx, const char* path) {
    if (!ctx || !path || !path[0]) return false;
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    uint32_t magic = 0;
    bool ok = (fread(&magic, sizeof(magic), 1, f) == 1) && (magic == 0x53525332u);
    if (ok && ctx->hw_rom_loaded && ctx->hw_snes) {
        Snes* hw = (Snes*)ctx->hw_snes;
        FileSaveLoadCtx sctx = { f, true };
        snes_saveload(hw, &file_load_cb, &sctx);
        file_load_cb(&sctx, hw->ppu->vram_bgmode, sizeof(hw->ppu->vram_bgmode));
        int extra_lr = ctx->widescreen_enabled ? ctx->extra_left_right : 0;
        PpuSetExtraSideSpace(hw->ppu, extra_lr, extra_lr, 0);
        hw_sync_to_ctx(ctx);
        ok = sctx.ok;
    } else if (ok) {
        void* saved_ptr = ctx->cpu.context;
        ok = ok && (fread(&ctx->cpu, sizeof(ctx->cpu), 1, f) == 1);
        ctx->cpu.context = saved_ptr;
        ok = ok && (fread(ctx->wram, sizeof(ctx->wram), 1, f) == 1);
    }
    fclose(f);
    return ok;
}

uint8_t snes_read8(snes_cpu_t* cpu, uint32_t addr) {
    snes_context_t* ctx = (snes_context_t*)cpu->context;
    if (ctx && ctx->hw_rom_loaded && ctx->hw_snes) {
        return snes_read((Snes*)ctx->hw_snes, addr & 0xFFFFFFu);
    }
    uint8_t bank = (uint8_t)((addr >> 16) & 0xFF);
    uint16_t offset = (uint16_t)(addr & 0xFFFF);

    if (bank == 0x7E) return ctx->wram[offset];
    if (bank == 0x7F) return ctx->wram[0x10000 + offset];
    if (offset < 0x2000) return ctx->wram[offset];

    if (offset == 0x4210) {
        uint8_t val = ctx->nmi_flag | 0x02;
        ctx->nmi_flag = 0;
        return val;
    }
    if (offset == 0x4218) return (uint8_t)(ctx->joy1_word & 0xFF);
    if (offset == 0x4219) return (uint8_t)((ctx->joy1_word >> 8) & 0xFF);
    if (offset == 0x421A) return (uint8_t)(ctx->joy2_word & 0xFF);
    if (offset == 0x421B) return (uint8_t)((ctx->joy2_word >> 8) & 0xFF);

    if (offset >= 0x4300 && offset <= 0x437F) {
        return ctx->dma_regs[offset - 0x4300];
    }

    return 0;
}

void snes_write8(snes_cpu_t* cpu, uint32_t addr, uint8_t val) {
    snes_context_t* ctx = (snes_context_t*)cpu->context;
    if (ctx && ctx->hw_rom_loaded && ctx->hw_snes) {
        snes_write((Snes*)ctx->hw_snes, addr & 0xFFFFFFu, val);
        return;
    }
    uint8_t bank = (uint8_t)((addr >> 16) & 0xFF);
    uint16_t offset = (uint16_t)(addr & 0xFFFF);

    if (bank == 0x7E) { ctx->wram[offset] = val; return; }
    if (bank == 0x7F) { ctx->wram[0x10000 + offset] = val; return; }
    if (offset < 0x2000) { ctx->wram[offset] = val; return; }

    if (offset == 0x2100) { ctx->inidisp = val; return; }
    if (offset == 0x2101) { ctx->obsel = val; return; }
    if (offset == 0x2105) { ctx->bg_mode = val & 0x07; return; }
    if (offset >= 0x2107 && offset <= 0x210A) {
        ctx->bg_sc[offset - 0x2107] = (val & 0xFC) << 9;
        return;
    }
    if (offset == 0x210B) {
        ctx->bg_chr[0] = (val & 0x0F) << 13;
        ctx->bg_chr[1] = ((val >> 4) & 0x0F) << 13;
        return;
    }
    if (offset == 0x210C) {
        ctx->bg_chr[2] = (val & 0x0F) << 13;
        ctx->bg_chr[3] = ((val >> 4) & 0x0F) << 13;
        return;
    }
    if (offset == 0x2115) { ctx->vmain = val; return; }
    if (offset == 0x2116) { ctx->vmadd = (ctx->vmadd & 0xFF00) | val; return; }
    if (offset == 0x2117) { ctx->vmadd = ((uint16_t)val << 8) | (ctx->vmadd & 0x00FF); return; }
    if (offset == 0x2118) {
        uint16_t a = ((ctx->vmadd & 0x7FFF) << 1) % 0x10000;
        ctx->vram[a] = val;
        if (!(ctx->vmain & 0x80)) ctx->vmadd = (ctx->vmadd + 1) & 0x7FFF;
        return;
    }
    if (offset == 0x2119) {
        uint16_t a = (((ctx->vmadd & 0x7FFF) << 1) + 1) % 0x10000;
        ctx->vram[a] = val;
        if (ctx->vmain & 0x80) ctx->vmadd = (ctx->vmadd + 1) & 0x7FFF;
        return;
    }
    if (offset == 0x2121) { ctx->cgadd = (val & 0xFF) * 2; ctx->cg_second_write = false; return; }
    if (offset == 0x2122) {
        if (!ctx->cg_second_write) {
            ctx->cg_latch = val;
            ctx->cg_second_write = true;
        } else {
            uint16_t a = ctx->cgadd % 512;
            ctx->cgram[a] = ctx->cg_latch;
            ctx->cgram[a + 1] = val;
            uint16_t bgr = ctx->cg_latch | ((uint16_t)val << 8);
            uint8_t r = (uint8_t)((bgr & 0x1F) << 3);
            uint8_t g = (uint8_t)(((bgr >> 5) & 0x1F) << 3);
            uint8_t b = (uint8_t)(((bgr >> 10) & 0x1F) << 3);
            uint8_t idx = (uint8_t)((ctx->cgadd / 2) % 256);
            ctx->palette_cache[idx] = (0xFFu << 24) | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
            ctx->cgadd = (ctx->cgadd + 2) % 512;
            ctx->cg_second_write = false;
        }
        return;
    }
    if (offset == 0x212C) { ctx->tm = val; return; }

    if (offset == 0x4200) { ctx->nmitimen = val; return; }
    if (offset == 0x4202) { ctx->wrmpya = val; return; }
    if (offset == 0x4203) {
        ctx->wrmpyb = val;
        uint16_t res = (uint16_t)ctx->wrmpya * ctx->wrmpyb;
        ctx->rdmpyl = res;
        return;
    }

    if (offset >= 0x4300 && offset <= 0x437F) {
        ctx->dma_regs[offset - 0x4300] = val;
        return;
    }
}

uint16_t snes_read16(snes_cpu_t* cpu, uint32_t addr) {
    uint8_t lo = snes_read8(cpu, addr);
    uint8_t hi = snes_read8(cpu, addr + 1);
    return (uint16_t)(lo | (hi << 8));
}

void snes_write16(snes_cpu_t* cpu, uint32_t addr, uint16_t val) {
    snes_write8(cpu, addr, (uint8_t)(val & 0xFF));
    snes_write8(cpu, addr + 1, (uint8_t)((val >> 8) & 0xFF));
}

void snes_push8(snes_cpu_t* cpu, uint8_t val) {
    snes_write8(cpu, cpu->SP, val);
    cpu->SP = (cpu->SP - 1) & 0xFFFF;
}

uint8_t snes_pull8(snes_cpu_t* cpu) {
    cpu->SP = (cpu->SP + 1) & 0xFFFF;
    return snes_read8(cpu, cpu->SP);
}

void snes_push16(snes_cpu_t* cpu, uint16_t val) {
    snes_push8(cpu, (uint8_t)((val >> 8) & 0xFF));
    snes_push8(cpu, (uint8_t)(val & 0xFF));
}

uint16_t snes_pull16(snes_cpu_t* cpu) {
    uint8_t lo = snes_pull8(cpu);
    uint8_t hi = snes_pull8(cpu);
    return (uint16_t)(lo | (hi << 8));
}

void snes_set_nz8(snes_cpu_t* cpu, uint8_t val) {
    cpu->P &= ~(FLAG_N | FLAG_Z);
    if (val == 0) cpu->P |= FLAG_Z;
    if (val & 0x80) cpu->P |= FLAG_N;
}

void snes_set_nz16(snes_cpu_t* cpu, uint16_t val) {
    cpu->P &= ~(FLAG_N | FLAG_Z);
    if (val == 0) cpu->P |= FLAG_Z;
    if (val & 0x8000) cpu->P |= FLAG_N;
}

void snes_op_adc(snes_cpu_t* cpu, uint16_t val) {
    uint8_t c = (cpu->P & FLAG_C) ? 1 : 0;
    if (cpu->P & FLAG_M) {
        uint16_t sum = (cpu->A & 0xFF) + (val & 0xFF) + c;
        cpu->P &= ~(FLAG_C | FLAG_V);
        if (sum > 0xFF) cpu->P |= FLAG_C;
        if (!((cpu->A ^ val) & 0x80) && ((cpu->A ^ sum) & 0x80)) cpu->P |= FLAG_V;
        cpu->A = (cpu->A & 0xFF00) | (sum & 0xFF);
        snes_set_nz8(cpu, (uint8_t)(cpu->A & 0xFF));
    } else {
        uint32_t sum = (uint32_t)cpu->A + val + c;
        cpu->P &= ~(FLAG_C | FLAG_V);
        if (sum > 0xFFFF) cpu->P |= FLAG_C;
        if (!((cpu->A ^ val) & 0x8000) && ((cpu->A ^ sum) & 0x8000)) cpu->P |= FLAG_V;
        cpu->A = (uint16_t)(sum & 0xFFFF);
        snes_set_nz16(cpu, cpu->A);
    }
}

void snes_op_sbc(snes_cpu_t* cpu, uint16_t val) {
    snes_op_adc(cpu, ~val);
}

void snes_op_xce(snes_cpu_t* cpu) {
    bool old_c = (cpu->P & FLAG_C) != 0;
    if (cpu->flag_e) cpu->P |= FLAG_C;
    else cpu->P &= ~FLAG_C;
    cpu->flag_e = old_c;
    if (cpu->flag_e) cpu->P |= (FLAG_M | FLAG_X);
}

void snes_ppu_render_scanline(snes_context_t* ctx, int line) {
    if (!ctx || line < 0 || line >= SNES_SCREEN_HEIGHT) return;
    int line_offset = line * SNES_SCREEN_WIDTH;
    int wide_width = ctx->frame_width > 0 ? ctx->frame_width : SNES_WIDESCREEN_WIDTH;
    int wide_offset = line * wide_width;

    if (ctx->inidisp & 0x80) {
        for (int x = 0; x < SNES_SCREEN_WIDTH; x++) {
            ctx->framebuffer[line_offset + x] = 0xFF000000u;
        }
        for (int x = 0; x < wide_width; x++) {
            ctx->widescreen_framebuffer[wide_offset + x] = 0xFF000000u;
        }
        return;
    }

    float brightness = (float)(ctx->inidisp & 0x0F) / 15.0f;
    uint32_t bg_color = ctx->palette_cache[0];
    uint8_t b_r = (uint8_t)(((bg_color >> 16) & 0xFF) * brightness);
    uint8_t b_g = (uint8_t)(((bg_color >> 8) & 0xFF) * brightness);
    uint8_t b_b = (uint8_t)((bg_color & 0xFF) * brightness);
    uint32_t backdrop = (0xFFu << 24) | ((uint32_t)b_r << 16) | ((uint32_t)b_g << 8) | b_b;

    for (int x = 0; x < SNES_SCREEN_WIDTH; x++) {
        ctx->framebuffer[line_offset + x] = backdrop;
    }
    for (int x = 0; x < wide_width; x++) {
        ctx->widescreen_framebuffer[wide_offset + x] = backdrop;
    }
}

void snes_runtime_run_frame(snes_context_t* ctx) {
    if (!ctx || !ctx->is_running) return;

    if (ctx->hw_rom_loaded && ctx->hw_snes) {
        Snes* hw = (Snes*)ctx->hw_snes;
        hw->input1->currentState = ctx->joy1_word;
        hw->input2->currentState = ctx->joy2_word;
        int extra_lr = ctx->widescreen_enabled ? ctx->extra_left_right : 0;
        PpuSetExtraSideSpace(hw->ppu, extra_lr, extra_lr, 0);
        PpuBeginDrawing(hw->ppu, (uint8_t*)g_ppu_scratch_fb, HW_PPU_SCRATCH_WIDTH * 4, 0);
        hw_run_single_frame(ctx, hw);
        hw_copy_framebuffers(ctx);
        hw_sync_to_ctx(ctx);
        return;
    }

    for (int line = 0; line < 262; line++) {
        if (line < SNES_SCREEN_HEIGHT) {
            snes_ppu_render_scanline(ctx, line);
        } else if (line == 225) {
            ctx->nmi_flag = 0x80;
        }
    }
    ctx->frame_count++;
}
