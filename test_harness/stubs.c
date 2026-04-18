/* stubs.c — stub implementations for all non-CPU external symbols */
#include "harness.h"
#include <unistd.h>

/* ── debugger ── */
int  enter_debugger = 0;
bool in_debugger    = false;

/* ── options ── */
config_t x48ng_config = { .inhibit_shutdown = true };

/* ── timers ── */
t1_t2_ticks get_t1_t2(void)      { t1_t2_ticks t = {0,0}; return t; }
void reset_timer(int t)           { (void)t; }
void restart_timer(int t)         { (void)t; }
void start_timer(int t)           { (void)t; }
void stop_timer(int t)            { (void)t; }

/* ── serial / IO ── */
void serial_baud(int b)           { (void)b; }
void receive_char(void)           {}
void transmit_char(void)          {}

/* ── ui4x ── */
void ui4x_refresh_output(void)         {}
void ui4x_handle_pending_inputs(void)  {}

/* ── persistence port masks (used by memory dispatch) ── */
bool port1_is_ram = false;
long port1_mask   = 0;
bool port2_is_ram = false;
long port2_mask   = 0;

/* ── pause — used in SHUTDN; inhibit_shutdown=1 makes it unreachable ── */
/* provided by libc */
