from stm32_regtypes import *

class stm32f427xx(STM32Fixups):
    @staticmethod
    def post_register_fixups(chip: STM32Chip):
        # For some reason this is a uint8_t in the header and it fails the regex match.
        chip.periph_map["CRC"]["IDR"] = Register(name="IDR", desc="CRC independent data register", hex_addr="0x04", int_addr=0x04, fields={}, access=None, reset_value=0)

    @staticmethod
    def supplemental_data(chip: STM32Chip):
        chip.periph_map["CRC"]["DR"].reset_value = 0xFFFFFFFF

        chip.periph_map["RNG"]["DR"].fields["DR"] = RegisterBitField(name="DR", desc="Data register", shift=0, width=32, permissions=None, unimplemented=False)
        for k,v in chip.periph_map["RNG"]["CR"].fields.items():
            if k != "RNGEN" and k != "IE":
                v.unimplemented = True
        chip.periph_map["RNG"]["SR"].fields["SECS"].unimplemented = True
        chip.periph_map["RNG"]["SR"].fields["SEIS"].unimplemented = True
