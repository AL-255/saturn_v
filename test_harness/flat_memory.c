/* flat_memory.c — 1 MB flat nibble array replacing memory.c for test generation.
 *
 * The Saturn has a 20-bit nibble-addressed space (0x00000..0xFFFFF).
 * Each array cell holds one nibble (4 bits, stored in a byte).
 */
#include "harness.h"

#define MEM_SIZE (1 << 20)   /* 1 M nibbles */

static unsigned char mem[MEM_SIZE];

/* ── CRC helper (copied from memory.c) ── */
static inline Nibble calc_crc(Nibble nib) {
    saturn.crc = (saturn.crc >> 4) ^ (((saturn.crc ^ nib) & 0xf) * 0x1081);
    return nib;
}

/* ── read_nibbles: multi-nibble little-endian read (used by emulate.c) ── */
long read_nibbles(Address addr, int len) {
    long val = 0;
    for (int i = len - 1; i >= 0; i--) {
        val <<= 4;
        val |= mem[(addr + i) & 0xFFFFF] & 0xf;
    }
    return val;
}

/* ── bus function implementations ── */
static Nibble flat_fetch_nibble(Address addr) {
    return mem[addr & 0xFFFFF] & 0xf;
}

static void flat_write_nibble(Address addr, Nibble val) {
    mem[addr & 0xFFFFF] = val & 0xf;
}

static int flat_read_nibble_crc(Address addr) {
    return (int)calc_crc(mem[addr & 0xFFFFF] & 0xf);
}

/* ── function pointer definitions (normally in memory.c) ── */
void   (*bus_write_nibble)(Address addr, Nibble val) = NULL;
Nibble (*bus_fetch_nibble)(Address addr)             = NULL;
int    (*read_nibble_crc)(Address addr)              = NULL;

/* ── nibble_masks (extern declared in memory.h) ── */
long nibble_masks[16] = {
    0x0000000f, 0x000000f0, 0x00000f00, 0x0000f000,
    0x000f0000, 0x00f00000, 0x0f000000, 0xf0000000,
    0x0000000f, 0x000000f0, 0x00000f00, 0x0000f000,
    0x000f0000, 0x00f00000, 0x0f000000, 0xf0000000,
};

/* display_t is declared in memory.h and defined here (normally in memory.c) */
display_t display;

/* ── public API ── */

/* Write a sequence of nibbles starting at address addr */
void mem_write_nibbles(Address addr, const unsigned char *nibs, int count) {
    for (int i = 0; i < count; i++)
        mem[(addr + i) & 0xFFFFF] = nibs[i] & 0xf;
}

/* Write a hex string of nibbles (e.g. "A70F") to memory */
void mem_write_hex(Address addr, const char *hex) {
    for (int i = 0; hex[i]; i++) {
        char c = hex[i];
        unsigned char nib = (c >= '0' && c <= '9') ? c - '0'
                          : (c >= 'a' && c <= 'f') ? c - 'a' + 10
                          : (c >= 'A' && c <= 'F') ? c - 'A' + 10
                          : 0;
        mem[(addr + i) & 0xFFFFF] = nib;
    }
}

void mem_clear(void) {
    memset(mem, 0, sizeof(mem));
}

/* Read count nibbles starting at addr into buf. Used by test harness to
 * snapshot DAT data regions for the vector file. */
void mem_read_range(Address addr, unsigned char *buf, int count) {
    for (int i = 0; i < count; i++)
        buf[i] = mem[(addr + i) & 0xFFFFF] & 0xf;
}

/* Call once to install flat memory bus */
void flat_memory_init(void) {
    bus_fetch_nibble  = flat_fetch_nibble;
    bus_write_nibble  = flat_write_nibble;
    read_nibble_crc   = flat_read_nibble_crc;
}
