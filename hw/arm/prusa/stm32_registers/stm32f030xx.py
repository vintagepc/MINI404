from stm32_regtypes import *

class stm32f030xx(STM32Fixups):
    @staticmethod
    def post_register_fixups(chip: STM32Chip):
        # For some reason this is a uint8_t in the header and it fails the regex match.
        field = {}
        field["IDR"]=RegisterBitField(name="IDR", desc="Independent data register", shift=0, width=8, permissions=None, unimplemented=False)
        chip.periph_map["CRC"]["IDR"] = Register(name="IDR", desc="CRC independent data register", hex_addr="0x04", int_addr=0x04, fields=field, access=None, reset_value=0)
        # Insert the CCR register into the ADC peripheral. We'll remove it later, this is just so the bitfields get captured.
        chip.periph_map["ADC"]["CCR"] = Register(name="CCR", desc="ADC common control register", hex_addr="0x0", int_addr=0x0, fields={}, access=None, reset_value=0)
        #Rename ADC TR to TR1 - per header.
        chip.periph_map["ADC"]["TR1"] = chip.periph_map["ADC"]["TR"]
        chip.periph_map["ADC"].pop("TR")
        chip.periph_map["ADC"]["TR1"].name = "TR1"

    @staticmethod
    def post_bitfield_fixups(chip: STM32Chip):
        # Relocate the ADC CCR register to its own class for better overlap with how it's implemented in QEMU:
        chip.periph_map["ADCC"] = {}
        chip.periph_map["ADCC"]["CCR"] = chip.periph_map["ADC"]["CCR"]
        chip.periph_map["ADC"].pop("CCR")

    @staticmethod
    def supplemental_data(chip: STM32Chip):
        chip.periph_map["CRC"]["DR"].reset_value = 0xFFFFFFFF
        chip.periph_map["CRC"]["INIT"].reset_value = 0xFFFFFFFF
        for field in chip.periph_map["CRC"]["CR"].fields.values():
            if field.name not in ["RESET"]:
                field.unimplemented = True
        chip.periph_map["ADC"]["TR1"].reset_value = 0x0FFF0000
        chip.periph_map["ADC"]["TR1"].unimplemented = True
        chip.periph_map["ADC"]["CFGR2"].unimplemented = True
