from dataclasses import dataclass, field
from stm32_regtypes import RegisterBitField


@dataclass
class RegisterFixup:
    reset_value: int = None
    unimplemented: bool = None
    remove: bool = False
    full_width: bool = False
    use_valid_mask: bool = False  # set reset_value = get_valid_mask() at apply time
    fields_unimplemented: list = field(default_factory=list)
    fields_unimplemented_except: list = field(default_factory=list)
    fields_remove: list = field(default_factory=list)


@dataclass
class PeripheralFixup:
    all_unimplemented: bool = False
    renames: dict = field(default_factory=dict)   # {old_name: new_name}
    registers: dict = field(default_factory=dict)

    def extend(self, renames: dict = None, **registers) -> 'PeripheralFixup':
        """Return a new PeripheralFixup inheriting this spec's settings plus additional fixups."""
        return PeripheralFixup(
            all_unimplemented=self.all_unimplemented,
            renames={**self.renames, **(renames or {})},
            registers={**self.registers, **registers},
        )


def apply_peripheral_fixup(periph_map: dict, periph_name: str, spec: PeripheralFixup):
    if periph_name not in periph_map:
        return
    periph = periph_map[periph_name]

    for old_name, new_name in spec.renames.items():
        if old_name in periph:
            reg = periph.pop(old_name)
            reg.name = new_name
            periph[new_name] = reg

    if spec.all_unimplemented:
        for reg in periph.values():
            reg.unimplemented = True

    for reg_name, reg_fixup in spec.registers.items():
        if reg_name not in periph:
            continue
        reg = periph[reg_name]

        if reg_fixup.remove:
            periph.pop(reg_name)
            continue

        if reg_fixup.use_valid_mask:
            reg.reset_value = reg.get_valid_mask()
        elif reg_fixup.reset_value is not None:
            reg.reset_value = reg_fixup.reset_value

        if reg_fixup.unimplemented is not None:
            reg.unimplemented = reg_fixup.unimplemented

        if reg_fixup.full_width:
            reg.fields[reg_name] = RegisterBitField(
                name=reg_name, desc=reg.desc, shift=0, width=32,
                permissions=None, unimplemented=False)

        if reg_fixup.fields_unimplemented_except:
            keep = set(reg_fixup.fields_unimplemented_except)
            for fname, f in reg.fields.items():
                if fname not in keep:
                    f.unimplemented = True

        for fname in reg_fixup.fields_unimplemented:
            if fname in reg.fields:
                reg.fields[fname].unimplemented = True

        for fname in reg_fixup.fields_remove:
            if fname in reg.fields:
                reg.fields.pop(fname)
