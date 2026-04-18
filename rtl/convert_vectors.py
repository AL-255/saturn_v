#!/usr/bin/env python3
"""Convert ../test_harness/test_vectors.txt -> test_vectors.dat (parse-friendly).

For registers printed LSN-first in text, reverse to MSN-first so Verilog $sscanf
reads the correct 64-bit / 20-bit value directly.

Output format, one test = 4 lines:
    # <name>
    P <pc> <A> <B> <C> <D> <D0> <D1> <P> <ST> <PSTAT16> <CARRY> <HEX> <RSTKP> <R0>..<R7>
    I <n0> <n1> <n2> <n3> <n4> <n5> <n6> <n7>
    O <same fields as P>

All values are plain hex (no 0x). Counts are implicit by column position.
"""
import re, sys, pathlib

def rev(s):        return s[::-1]
def rev5(s):       return rev(s)      # for 5-nibble addresses
def rev16(s):      return rev(s)

def pstat16(s):
    v = 0
    for i, c in enumerate(s):
        if c == '1':
            v |= 1 << i
    return f'{v:04x}'

def st4(s):
    """s is 4 hex chars from generate_tests.c: char 0=XM, 1=SB, 2=SR, 3=MP.
    Produce single hex nibble ST where bit 0=XM, 1=SB, 2=SR, 3=MP."""
    xm = int(s[0], 16)
    sb = int(s[1], 16)
    sr = int(s[2], 16)
    mp = int(s[3], 16)
    v = (mp << 3) | (sr << 2) | (sb << 1) | xm
    return f'{v:x}'

def fmt_state(d):
    pc    = rev5(d['PC'])
    A     = rev16(d['A'])
    B     = rev16(d['B'])
    C     = rev16(d['C'])
    D     = rev16(d['D'])
    D0    = rev5(d['D0'])
    D1    = rev5(d['D1'])
    P     = d['P']
    ST    = st4(d['ST'])
    PSTAT = pstat16(d['PSTAT'])
    CARRY = d['CARRY']
    HEX   = d['HEXMODE']
    rp    = int(d['RSTK_PTR'])
    if rp < 0: rp = rp & 0xf
    RSTKP = f'{rp:x}'
    rstk  = [rev5(x) for x in d['RSTK'].split(',')]
    assert len(rstk) == 8
    # R0..R4 (16-char LSN-first); default all-zero if not present (old vectors)
    rn = [rev16(d.get(f'R{i}', '0'*16)) for i in range(5)]
    IN  = rev(d.get('IN', '0000'))    # 4 nibbles
    OUT = rev(d.get('OUT', '000'))    # 3 nibbles
    fields = [pc, A, B, C, D, D0, D1, P, ST, PSTAT, CARRY, HEX, RSTKP] + rstk + rn + [IN, OUT]
    return ' '.join(fields)

def parse_state_line(line):
    # "PC=X A=Y B=Z ..."
    d = {}
    for m in re.finditer(r'(\w+)=([^\s]+)', line):
        d[m.group(1)] = m.group(2)
    return d

def main():
    src = pathlib.Path('../test_harness/test_vectors.txt')
    dst = pathlib.Path('test_vectors.dat')
    tests, cur = [], None
    for line in src.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        if line.startswith('TEST '):
            cur = {'name': line[5:], 'mem': []}
        elif line.startswith('PRE '):
            cur['pre'] = parse_state_line(line[4:])
        elif line.startswith('POST '):
            cur['post'] = parse_state_line(line[5:])
        elif line.startswith('INST '):
            cur['inst'] = line[5:].split()
        elif line.startswith('MEM '):
            # "MEM <addr_msn_first_5> <data_lsn_first_16>"
            parts = line[4:].split()
            addr_hex = parts[0]          # already MSN-first in generator
            data_lsn = parts[1]          # LSN-first 16 chars
            data_msn = rev16(data_lsn)   # reverse for sscanf compat
            cur['mem'].append((addr_hex, data_msn))
        elif line.startswith('END'):
            tests.append(cur)
            cur = None
    with dst.open('w') as f:
        f.write(f'{len(tests)}\n')
        for t in tests:
            f.write(f'{t["name"]}\n')
            f.write(fmt_state(t['pre']) + '\n')
            # mem-preload count, then lines
            f.write(f'{len(t["mem"])}\n')
            for addr, data in t['mem']:
                f.write(f'{addr} {data}\n')
            f.write(' '.join(t['inst']) + '\n')
            f.write(fmt_state(t['post']) + '\n')
    print(f'wrote {dst} with {len(tests)} tests')

if __name__ == '__main__':
    main()
