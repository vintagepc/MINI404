"""
STM32HeaderParser: Parses STM32 HAL header files into structured register data.

Replaces the ad-hoc parsing logic embedded in stm32_reggen.py with a
structured two-phase class:

  Phase 1 – parse_registers()
    Reads typedef struct blocks (including array-register expansion),
    peripheral base addresses, and IRQ numbers.  Returns ParsedChipData.
    Call post_register_fixups() between phase 1 and phase 2.

  Phase 2 – parse_bitfields(periph_map, not_found)
    Reads the Pos/Msk #define section and populates register field
    definitions in the already-built periph_map.
    Call post_bitfield_fixups() after phase 2.

Array-register expansion
------------------------
HAL headers sometimes declare registers as C arrays rather than individual
named fields, e.g.:
    __IO uint32_t EXTICR[4];  /*!< ..., Address offset: 0x60 */
    __IO uint32_t IT_LINE_SR[32]; /*!< ..., Address offset: 0x80 */

The parser expands these to named Register objects (EXTICR1..EXTICR4,
IT_LINE_SR1..IT_LINE_SR32).  The index base (0-based or 1-based) is chosen
automatically by pre-scanning the bitfield section: if NAME1 appears as a
known register for the peripheral, 1-based is used; if NAME0 appears,
0-based; otherwise 1-based is the default.
"""

import re
import dataclasses
from dataclasses import dataclass
from typing import Optional

from stm32_regtypes import Register, RegisterBitField


# ---------------------------------------------------------------------------
# Result container
# ---------------------------------------------------------------------------

@dataclass
class ParsedChipData:
    periph_map:   dict  # {periph_name: {reg_name: Register}}
    periph_addrs: dict  # {addr_label: int}
    periph_irqs:  dict  # {irq_name: int}
    not_found:    dict  # {label: True} for unmatched bitfield defines (debug)


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------

class STM32HeaderParser:
    """Parse a single STM32 HAL header file into structured register data."""

    # ---- Typedef struct patterns ----
    # Single register:  __IO uint32_t NAME; /*!< desc  Address offset: 0xXX */
    _REG = re.compile(
        r'.+uint32_t\s+(\w+);\s*/\*!<\s*(.+?)\s+Address offset:\s+(0x[0-9A-Fa-f]+)\s*\*/')
    # Single register with continuation comment: Address offset: 0xXX + ...
    _REG2 = re.compile(
        r'.+uint32_t\s+(\w+);\s*/\*!<\s*(.+?)\s+Address offset:\s+(0x[0-9A-Fa-f]+)\s*\+')
    # Array register: __IO uint32_t NAME[N]; /*!< desc ... 0xBASE ... */
    # Capture: group 1=name, 2=count, 3=desc prefix, 4=first hex offset in comment
    _ARRAY = re.compile(
        r'.*?uint32_t\s+(\w+)\[(\d+)\];\s*/\*!<\s*(.*?)(0x[0-9A-Fa-f]+)')
    # Typedef struct closing line: } Name_TypeDef; or } NameTypeDef;
    _TYPEDEF_END = re.compile(r'}\s*([\w]+)TypeDef;')

    # ---- Address / IRQ patterns ----
    _ADDR = re.compile(r'#define\s+(\w+_BASE(?:_NS)?)\s+\(?([^/\)]+)\)?')
    _IRQ  = re.compile(r'\s+(\w+_IRQn)\s*[=\s]+(\d+)\s*,')

    # ---- Bitfield patterns ----
    _SHIFT = re.compile(r'\((\d+)U\)')
    _MSK   = re.compile(r'\((0x[0-9A-F]+)U?L?')
    _MSK2  = re.compile(r'\(([0-9]+)UL')
    _DESC  = re.compile(r'^.*?(/\*.*\*/)$')

    # Contiguous mask → bit-width lookup (all values 2^n - 1 for n in 1..32)
    _MASK2NBITS: dict = {(1 << n) - 1: n for n in range(1, 33)}

    # -----------------------------------------------------------------------
    # Construction / loading
    # -----------------------------------------------------------------------

    def __init__(self) -> None:
        self._lines: list[str] = []
        self._bf_start: int = -1          # index of first bitfield #define line
        self._known_regs: dict[str, set]  = {}  # PERIPH → {reg_names} from Pos/Msk scan

    def load(self, header_path: str) -> None:
        """
        Read the header into memory, locate the bitfield section boundary,
        and pre-scan register names so array expansion can pick the right
        index base.
        """
        with open(header_path, 'r') as fh:
            self._lines = fh.readlines()
        self._find_bitfield_start()
        self._pre_scan_bitfield_names()

    # -----------------------------------------------------------------------
    # Public two-phase API
    # -----------------------------------------------------------------------

    def parse_registers(self) -> ParsedChipData:
        """
        Phase 1: extract typedef structs (with array expansion), base
        addresses, and IRQ numbers.  Call before post_register_fixups().
        """
        periph_map:   dict = {}
        periph_addrs: dict = {}
        periph_irqs:  dict = {}

        end = self._bf_start if self._bf_start >= 0 else len(self._lines)
        i = 0
        while i < end:
            line = self._lines[i]

            irq_m = self._IRQ.match(line)
            if irq_m:
                periph_irqs[irq_m.group(1)] = int(irq_m.group(2))

            addr_m = self._ADDR.match(line)
            if addr_m:
                self._resolve_address(addr_m.group(1), addr_m.group(2), periph_addrs)
                i += 1
                continue

            if 'typedef struct' in line:
                block = [line]
                i += 1
                # Consume lines until the closing brace
                while i < end and '}' not in self._lines[i]:
                    block.append(self._lines[i])
                    i += 1
                if i < end:
                    block.append(self._lines[i])
                regs, name = self._parse_typedef(block)
                if name and regs:
                    periph_map[name] = regs

            i += 1

        return ParsedChipData(periph_map=periph_map, periph_addrs=periph_addrs,
                              periph_irqs=periph_irqs, not_found={})

    def parse_bitfields(self, periph_map: dict, not_found: dict) -> None:
        """
        Phase 2: walk the Pos/Msk #define section and populate register
        field definitions in periph_map (mutated in place).
        Call after post_register_fixups() and before post_bitfield_fixups().
        """
        if self._bf_start < 0:
            return
        for line in self._lines[self._bf_start:]:
            self._parse_bitfield_line(line, periph_map, not_found)

    # -----------------------------------------------------------------------
    # Internal: file-level helpers
    # -----------------------------------------------------------------------

    def _find_bitfield_start(self) -> None:
        """Locate the first line that looks like a Pos or Msk #define."""
        for i, line in enumerate(self._lines):
            if '#define' in line and ('Pos' in line or 'Msk' in line):
                self._bf_start = i
                return

    def _pre_scan_bitfield_names(self) -> None:
        """
        Pre-scan the bitfield section to collect the register names that the
        header's own Pos/Msk defines use.

        Example: '#define EXTI_EXTICR1_EXTI0_Pos' tells us peripheral EXTI
        has a register named EXTICR1.  This is used by _array_index_base()
        to decide whether NAME[4] should expand to NAME0..NAME3 or NAME1..NAME4.
        """
        if self._bf_start < 0:
            return
        for line in self._lines[self._bf_start:]:
            if '#define' not in line:
                continue
            tokens = line.split()
            if len(tokens) < 2:
                continue
            name = tokens[1]
            if not (name.endswith('Pos') or name.endswith('Msk')):
                continue
            parts = name.split('_')
            if len(parts) < 3:
                continue
            periph = parts[0]
            reg_candidate = parts[1]   # e.g. 'EXTICR1', 'ITLINE0', 'AFRL'
            self._known_regs.setdefault(periph, set()).add(reg_candidate)

    # -----------------------------------------------------------------------
    # Internal: typedef struct parsing
    # -----------------------------------------------------------------------

    def _parse_typedef(self, block: list[str]) -> tuple[dict, str]:
        """
        Parse a 'typedef struct { ... } Name_TypeDef;' block.
        Returns (register_dict, peripheral_name).
        """
        closing = block[-1]
        m = self._TYPEDEF_END.match(closing)
        if not m:
            return {}, ''
        # rstrip('_') handles trailing underscores from 'DMA__TypeDef' etc.
        # but preserves internal underscores like 'DMA_Channel'.
        periph_name = m.group(1).rstrip('_')

        regs: dict = {}
        for line in block[1:-1]:       # skip opening and closing lines
            if 'RESERVED' in line:
                continue
            arr = self._parse_array_reg(line, periph_name)
            if arr is not None:
                regs.update(arr)
                continue
            reg = self._parse_single_reg(line)
            if reg is not None:
                regs[reg.name] = reg
        return regs, periph_name

    def _parse_single_reg(self, line: str) -> Optional[Register]:
        """Parse a single 'uint32_t NAME; /*!< ... offset ... */' line."""
        m = self._REG.match(line) or self._REG2.match(line)
        if not m:
            return None
        name, desc, hex_addr = m.group(1), m.group(2).strip(), m.group(3)
        return Register(name=name, desc=desc, hex_addr=hex_addr,
                        int_addr=int(hex_addr, 16), fields={},
                        access=None, reset_value=0)

    def _parse_array_reg(self, line: str, periph_name: str) -> Optional[dict]:
        """
        Expand 'uint32_t NAME[N]; /*!< ... 0xBASE ... */' into N Registers.

        Naming uses _array_index_base() to select 0-based or 1-based indices,
        matching the convention used in the header's own bitfield defines.
        Registers are named NAME<n> at consecutive 4-byte offsets from base.
        """
        m = self._ARRAY.match(line)
        if not m:
            return None
        array_name = m.group(1)
        count      = int(m.group(2))
        desc       = m.group(3).strip().rstrip(',').rstrip()
        base_off   = int(m.group(4), 16)

        zero_based = self._array_index_base(periph_name, array_name, count)
        regs: dict = {}
        for idx in range(count):
            n        = idx if zero_based else idx + 1
            reg_name = f"{array_name}{n}"
            offset   = base_off + idx * 4
            hex_addr = f"0x{offset:02X}"
            regs[reg_name] = Register(
                name=reg_name,
                desc=f"{desc}[{idx}]",
                hex_addr=hex_addr,
                int_addr=offset,
                fields={},
                access=None,
                reset_value=0,
            )
        return regs

    def _array_index_base(self, periph: str, array_name: str, count: int) -> bool:
        """
        Return True  → use 0-based indices (NAME0, NAME1, …).
        Return False → use 1-based indices (NAME1, NAME2, …).

        Decision: look up what names the bitfield defines already use for this
        peripheral.  'EXTICR1' found → 1-based.  'ITLINE0' found → 0-based.
        If neither convention is detected, default to 1-based.
        """
        known = self._known_regs.get(periph.upper(), set())
        if f"{array_name}1" in known:
            return False   # 1-based
        if f"{array_name}0" in known:
            return True    # 0-based
        return False       # default: 1-based

    # -----------------------------------------------------------------------
    # Internal: address resolution
    # -----------------------------------------------------------------------

    def _resolve_address(self, name: str, expr: str, addrs: dict) -> None:
        """
        Evaluate a #define NAME_BASE expression, substituting previously
        resolved addresses and hex literals.  Uses eval() on controlled,
        pre-sanitised input (arithmetic of integer tokens only).
        """
        tokens = expr.split()
        for i, tok in enumerate(tokens):
            tok_clean = tok.rstrip('UL')
            if tok in addrs:
                tokens[i] = str(addrs[tok])
            elif tok_clean.startswith('0x') or tok_clean.startswith('0X'):
                try:
                    tokens[i] = str(int(tok_clean, 16))
                except ValueError:
                    pass
        try:
            addrs[name] = eval(''.join(tokens))  # noqa: S307
        except Exception:
            print(f"Address parse error: {name!r} = {expr!r}")

    # -----------------------------------------------------------------------
    # Internal: bitfield parsing
    # -----------------------------------------------------------------------

    def _parse_bitfield_line(self, line: str, periph_map: dict,
                              not_found: dict) -> None:
        """
        Process one line from the Pos/Msk section.

        Handles three cases:
          * NAME_Pos defines → record field shift
          * NAME_Msk defines → record field width (via _apply_mask)
          * Alias/description defines (value contains _Msk) → attach desc
        """
        tokens = line.split()
        if len(tokens) < 3:
            return

        def_name = tokens[1]
        ends_pos = def_name.endswith('Pos')
        ends_msk = def_name.endswith('Msk')

        if not ends_pos and not ends_msk:
            # May be a description/alias line
            if 'Msk' in tokens[2]:
                self._try_attach_desc(line, def_name, periph_map)
            return

        parts = def_name.split('_')
        if len(parts) < 3:
            return

        periph = parts[0]
        reg    = parts[1]
        field  = '_'.join(parts[2:-1])  # everything between reg and Pos/Msk suffix

        if periph not in periph_map:
            not_found[periph] = True
            return
        if reg not in periph_map[periph]:
            not_found[f'{periph}_{reg}'] = True
            return

        reg_obj = periph_map[periph][reg]

        if ends_pos:
            sm = self._SHIFT.match(tokens[2])
            if sm:
                reg_obj.fields[field] = RegisterBitField(
                    name=field, desc=None, width=None,
                    shift=int(sm.group(1)), permissions=None, unimplemented=False)
            else:
                print(f"Shift not found: {tokens}")

        else:  # ends_msk
            val = tokens[2]
            if val.startswith('(x'):      # malformed hex in some headers
                val = val.replace('x', '0x', 1)
            mm = self._MSK.match(val)
            mm2 = self._MSK2.match(val)
            if mm:
                self._apply_mask(int(mm.group(1), 16), field, reg_obj)
            elif mm2:
                self._apply_mask(int(mm2.group(1)), field, reg_obj)
            else:
                print(f"Mask not found: {line.rstrip()}")

    def _try_attach_desc(self, line: str, def_name: str,
                          periph_map: dict) -> None:
        """Attach a description comment to an already-parsed field."""
        parts = def_name.split('_')
        if len(parts) < 3:
            return
        periph = parts[0]
        reg    = parts[1]
        field  = '_'.join(parts[2:])
        dm = self._DESC.match(line)
        if dm and periph in periph_map and reg in periph_map.get(periph, {}):
            try:
                periph_map[periph][reg].fields[field].desc = dm.group(1)
            except KeyError:
                pass

    def _apply_mask(self, mask: int, field: str, reg: Register) -> None:
        """
        Set the bit-width of a field from its mask value.

        For a standard contiguous mask (2^n − 1 shifted left), a table
        lookup gives the width immediately.  For non-contiguous masks the
        algorithm scans LSB-to-MSB to find each contiguous run of 1-bits,
        records the first run's width on the original field, and creates
        numbered sub-fields (FIELD_1, FIELD_2, …) for subsequent runs.

        The 33-character bit string (one leading '0' prepended) guarantees
        any run that extends to bit 31 is always terminated by a '0'.
        """
        if mask in self._MASK2NBITS:
            if field in reg.fields:
                reg.fields[field].width = self._MASK2NBITS[mask]
            return

        # Non-contiguous mask path
        bits_str  = f"0{mask:032b}"   # 33 chars, LSB at right, leading '0' terminates
        cur_start = 0                 # tracks consumed-bit count for gap/block accounting
        block_idx = 0
        cur_run   = ""

        while bits_str:
            bit      = bits_str[-1]
            bits_str = bits_str[:-1]

            if bit == '1':
                cur_run += '1'
            elif cur_run:
                # End of a 1-run: record this sub-field.
                # cur_start + 1 gives the correct shift because the terminating
                # '0' is consumed without incrementing cur_start; each prior
                # terminator contributes exactly one missing increment, and +1
                # compensates for the most recent one.
                f_shift = cur_start + 1
                f_width = len(cur_run)
                if block_idx == 0:
                    if field in reg.fields:
                        reg.fields[field].width = f_width
                else:
                    sub = f"{field}_{block_idx}"
                    if field in reg.fields:
                        reg.fields[sub] = dataclasses.replace(reg.fields[field])
                        reg.fields[sub].name  = sub
                        reg.fields[sub].width = f_width
                        reg.fields[sub].shift += f_shift
                block_idx += 1
                cur_start += f_width
                cur_run    = ""
            else:
                cur_start += 1
