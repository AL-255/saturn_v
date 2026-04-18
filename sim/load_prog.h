/* load_prog.h — shared program/ROM loaders used by csim and vsim.
 * load_prog()    : whitespace-separated hex nibbles in a text file.
 * load_rom_bin() : packed HP48 ROM file. Each byte expands to two nibbles,
 *                  low-nibble first: mem[2i] = byte & 0xf, mem[2i+1] = byte >> 4.
 *                  This is the on-disk format x48ng's romio.c uses for 0x32/0x96-header ROMs.
 */
#ifndef LOAD_PROG_H
#define LOAD_PROG_H

#include <stdio.h>
#include <ctype.h>
#include <string.h>

/* Callback: store a nibble at index. Caller decides storage. */
typedef void (*store_nib_fn)(int idx, unsigned char nib);

static inline int load_prog(const char *path, store_nib_fn store) {
    FILE *fp = fopen(path, "r");
    if (!fp) { perror(path); return -1; }
    int idx = 0;
    int c;
    while ((c = fgetc(fp)) != EOF) {
        if (c == '#') { while (c != '\n' && c != EOF) c = fgetc(fp); continue; }
        if (isspace(c)) continue;
        unsigned char nib;
        if      (c >= '0' && c <= '9') nib = c - '0';
        else if (c >= 'a' && c <= 'f') nib = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') nib = c - 'A' + 10;
        else continue;
        store(idx++, nib);
    }
    fclose(fp);
    return idx;
}

static inline int load_rom_bin(const char *path, store_nib_fn store) {
    FILE *fp = fopen(path, "rb");
    if (!fp) { perror(path); return -1; }
    int idx = 0;
    int b;
    while ((b = fgetc(fp)) != EOF) {
        store(idx++, (unsigned char)(b & 0xf));
        store(idx++, (unsigned char)((b >> 4) & 0xf));
    }
    fclose(fp);
    return idx;
}

#endif
