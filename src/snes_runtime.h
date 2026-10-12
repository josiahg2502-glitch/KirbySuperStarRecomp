/**
 * SNESRecomp - Native C Hardware Runtime Engine Header
 *
 * Provides 65C816 CPU state, 24-bit memory mapping, PPU scanline rendering
 * with native 16:9 widescreen (352x224) support, SPC700/S-DSP stereo audio,
 * DMA/HDMA channels, and battery-backed SRAM / save states for statically
 * recompiled SNES C projects.
 */

#ifndef SNES_RUNTIME_H
#define SNES_RUNTIME_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Display & Native Widescreen Constants
 * ========================================================================= */
#define SNES_SCREEN_WIDTH             256
#define SNES_SCREEN_HEIGHT            224
#define SNES_SCREEN_HEIGHT_OVERSCAN   239

/* Native 16:9 Widescreen: 352x224 (48 extra pixels on left & right) */
#define SNES_WIDESCREEN_WIDTH         352
#define SNES_WIDESCREEN_HEIGHT        224
#define SNES_WIDESCREEN_EXTRA_SIDE    48

/* 16:10 Widescreen: 320x224 (32 extra pixels on left & right) */
#define SNES_WIDESCREEN_16_10_WIDTH   320
#define SNES_WIDESCREEN_16_10_SIDE    32

/* Maximum widescreen buffer stride (up to 64 extra pixels per side = 384+) */
#define SNES_ULTRA_WIDESCREEN_WIDTH   426

#define SNES_AUDIO_SAMPLE_RATE        44100
#define SNES_AUDIO_CHANNELS           2
#define SNES_AUDIO_SAMPLES_PER_FRAME  735  /* 44100 / 60 */

/* =========================================================================
 * 65C816 Status Register Flags (P)
 * ========================================================================= */
#define FLAG_C 0x01  /* Carry */
#define FLAG_Z 0x02  /* Zero */
#define FLAG_I 0x04  /* IRQ Disable */
#define FLAG_D 0x08  /* Decimal Mode */
#define FLAG_X 0x10  /* Index Register Select (1 = 8-bit, 0 = 16-bit) */
#define FLAG_M 0x20  /* Accumulator Select    (1 = 8-bit, 0 = 16-bit) */
#define FLAG_V 0x40  /* Overflow */
#define FLAG_N 0x80  /* Negative */

/* Aliases with SNES_ prefix */
#define SNES_FLAG_C FLAG_C
#define SNES_FLAG_Z FLAG_Z
#define SNES_FLAG_I FLAG_I
#define SNES_FLAG_D FLAG_D
#define SNES_FLAG_X FLAG_X
#define SNES_FLAG_M FLAG_M
#define SNES_FLAG_V FLAG_V
#define SNES_FLAG_N FLAG_N

/* =========================================================================
 * SNES Standard Controller Button Masks ($4218-$4219)
 * ========================================================================= */
#define SNES_BTN_B      0x8000
#define SNES_BTN_Y      0x4000
#define SNES_BTN_SELECT 0x2000
#define SNES_BTN_START  0x1000
#define SNES_BTN_UP     0x0800
#define SNES_BTN_DOWN   0x0400
#define SNES_BTN_LEFT   0x0200
#define SNES_BTN_RIGHT  0x0100
#define SNES_BTN_A      0x0080
#define SNES_BTN_X      0x0040
#define SNES_BTN_L      0x0020
#define SNES_BTN_R      0x0010

/* =========================================================================
 * 65C816 CPU State Structure
 * ========================================================================= */
typedef struct snes_cpu_t {
    uint16_t A;       /* 16-bit Accumulator */
    uint16_t X;       /* 16-bit Index X */
    uint16_t Y;       /* 16-bit Index Y */
    uint16_t SP;      /* 16-bit Stack Pointer */
    uint16_t DP;      /* 16-bit Direct Page */
    uint8_t  DB;      /* 8-bit Data Bank */
    uint8_t  PB;      /* 8-bit Program Bank */
    uint16_t PC;      /* 16-bit Program Counter */
    uint8_t  P;       /* 8-bit Processor Status */
    bool     flag_e;  /* Emulation mode flag */
    void*    context; /* Pointer to parent snes_context_t */
} snes_cpu_t;

/* =========================================================================
 * Full SNES Hardware Context Structure
 * ========================================================================= */
typedef struct snes_context_t {
    snes_cpu_t cpu;

    /* System Memory */
    uint8_t  wram[0x20000];  /* 128 KB Work RAM ($7E0000-$7FFFFF) */
    uint8_t  sram[0x20000];  /* Up to 128 KB Save RAM */
    uint8_t  vram[0x10000];  /* 64 KB Video RAM */
    uint8_t  cgram[0x200];   /* 512 Bytes Palette CGRAM */
    uint8_t  oam[0x220];     /* 544 Bytes Object Attribute Memory */
    uint8_t  aram[0x10000];  /* 64 KB SPC700 Audio RAM */

    /* PPU State */
    uint8_t  inidisp;        /* $2100 Screen Display Register */
    uint8_t  obsel;          /* $2101 Object Size & Base */
    uint8_t  bg_mode;        /* $2105 BG Mode */
    uint16_t bg_sc[4];       /* $2107-$210A BG Tilemap Base */
    uint16_t bg_chr[4];      /* $210B-$210C BG Character Base */
    uint16_t bg_hofs[4];     /* $210D-$2113 Horizontal Scroll */
    uint16_t bg_vofs[4];     /* $210E-$2114 Vertical Scroll */
    uint8_t  vmain;          /* $2115 VRAM Address Increment Mode */
    uint16_t vmadd;          /* $2116-$2117 VRAM Word Address */
    uint16_t cgadd;          /* $2121 CGRAM Address */
    uint8_t  cg_latch;
    bool     cg_second_write;
    uint8_t  tm;             /* $212C Main Screen Designation */
    uint8_t  ts;             /* $212D Sub Screen Designation */

    /* CPU & DMA I/O */
    uint8_t  nmitimen;       /* $4200 Interrupt & Joypad Enable */
    uint8_t  nmi_flag;       /* $4210 RDNMI */
    uint8_t  wrmpya;         /* $4202 Multiplicand A */
    uint8_t  wrmpyb;         /* $4203 Multiplier B */
    uint16_t rdmpyl;         /* $4216-$4217 Multiplication Result */
    uint8_t  dma_regs[0x80]; /* $4300-$437F DMA / HDMA Registers */

    /* Controller Input */
    uint16_t joy1_word;      /* Player 1 Joypad State */
    uint16_t joy2_word;      /* Player 2 Joypad State */

    /* Framebuffers (ARGB8888: 0xFFRRGGBB) */
    uint32_t framebuffer[SNES_ULTRA_WIDESCREEN_WIDTH * SNES_SCREEN_HEIGHT_OVERSCAN];
    uint32_t widescreen_framebuffer[SNES_ULTRA_WIDESCREEN_WIDTH * SNES_SCREEN_HEIGHT_OVERSCAN];
    uint32_t *active_framebuffer;
    int      frame_width;
    int      frame_height;
    int      render_width;
    int      render_height;
    int      extra_left_right;
    bool     widescreen_enabled;

    uint32_t palette_cache[256];
    uint32_t frame_count;
    bool     is_running;

    /* Stereo 16-bit PCM Audio Output Ring (44100 Hz) */
    int16_t  audio_buffer[4096];
    size_t   audio_sample_count;

    /* Internal Cycle-Accurate Hardware Core Handle */
    void    *hw_snes;
    void    *hw;
    bool     hw_rom_loaded;
    bool     rom_loaded;
    uint8_t *owned_rom;
    char     rom_title[32];
} snes_context_t;

/* Embedded ROM image generated in game_recomp.c */
extern const uint32_t g_snes_rom_size;
extern const uint8_t  g_snes_rom_data[];

/* =========================================================================
 * Runtime Initialization & Lifecycle
 * ========================================================================= */
snes_context_t* snes_runtime_init(void);
void            snes_runtime_free(snes_context_t* ctx);
void            snes_runtime_reset(snes_context_t* ctx);
void            snes_runtime_run_frame(snes_context_t* ctx);

/* =========================================================================
 * ROM, SRAM & Save State Management
 * ========================================================================= */
bool            snes_runtime_load_rom(snes_context_t* ctx, const char* rom_path);
bool            snes_runtime_load_rom_file(snes_context_t* ctx, const char* rom_path);
bool            snes_runtime_load_rom_memory(snes_context_t* ctx, const uint8_t* data, size_t size);
bool            snes_runtime_is_rom_loaded(const snes_context_t* ctx);
const char*     snes_runtime_get_rom_title(const snes_context_t* ctx);

bool            snes_runtime_save_sram(const snes_context_t* ctx, const char* path);
bool            snes_runtime_load_sram(snes_context_t* ctx, const char* path);
bool            snes_runtime_save_state(const snes_context_t* ctx, const char* path);
bool            snes_runtime_load_state(snes_context_t* ctx, const char* path);

/* =========================================================================
 * Widescreen, Video & Audio Output API
 * ========================================================================= */
void            snes_runtime_set_widescreen(snes_context_t* ctx, bool enabled);
bool            snes_runtime_get_widescreen(const snes_context_t* ctx);
void            snes_runtime_set_extra_side_space(snes_context_t* ctx, int extra_lr);
int             snes_runtime_get_extra_side_space(const snes_context_t* ctx);

const uint32_t* snes_runtime_get_framebuffer(const snes_context_t* ctx);
int             snes_runtime_get_frame_width(const snes_context_t* ctx);
int             snes_runtime_get_frame_height(const snes_context_t* ctx);

size_t          snes_runtime_get_audio_samples(snes_context_t* ctx, int16_t* out_samples, size_t max_samples);
void            snes_ppu_render_scanline(snes_context_t* ctx, int line);

/* =========================================================================
 * Memory Bus & ALU Helpers for Recompiled 65C816 Code
 * ========================================================================= */
uint8_t         snes_read8(snes_cpu_t* cpu, uint32_t addr);
uint16_t        snes_read16(snes_cpu_t* cpu, uint32_t addr);
void            snes_write8(snes_cpu_t* cpu, uint32_t addr, uint8_t val);
void            snes_write16(snes_cpu_t* cpu, uint32_t addr, uint16_t val);

void            snes_push8(snes_cpu_t* cpu, uint8_t val);
uint8_t         snes_pull8(snes_cpu_t* cpu);
void            snes_push16(snes_cpu_t* cpu, uint16_t val);
uint16_t        snes_pull16(snes_cpu_t* cpu);

void            snes_set_nz8(snes_cpu_t* cpu, uint8_t val);
void            snes_set_nz16(snes_cpu_t* cpu, uint16_t val);
void            snes_op_adc(snes_cpu_t* cpu, uint16_t val);
void            snes_op_sbc(snes_cpu_t* cpu, uint16_t val);
void            snes_op_xce(snes_cpu_t* cpu);

#ifdef __cplusplus
}
#endif

#endif /* SNES_RUNTIME_H */
