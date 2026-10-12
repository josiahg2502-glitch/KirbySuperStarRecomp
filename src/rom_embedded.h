/* Auto-generated ROM alias header */
#ifndef SNES_ROM_EMBEDDED_H
#define SNES_ROM_EMBEDDED_H
#include <stdint.h>
#include <stddef.h>
extern const uint32_t g_snes_rom_size;
extern const uint8_t g_snes_rom_data[];
#define g_embedded_rom_size ((size_t)g_snes_rom_size)
#define g_embedded_rom g_snes_rom_data
#endif
