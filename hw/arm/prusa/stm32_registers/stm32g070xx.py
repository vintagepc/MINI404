from stm32_regtypes import STM32Chip, STM32Fixups, Register

class stm32g070xx(STM32Fixups):
    @staticmethod
    def post_register_fixups(chip: STM32Chip):
        chip.periph_map["ADC"]["CCR"] = Register(name="CCR", desc="ADC common control register", hex_addr="0x0", int_addr=0x0, fields={}, access=None, reset_value=0)
        # For some reason this is a uint8_t in the header and it fails the regex match.
        #chip.periph_map["CRC"]["IDR"] = Register(name="IDR", desc="CRC independent data register", hex_addr="0x04", int_addr=0x04, fields={}, access=None, reset_value=0)

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
        chip.periph_map["CRC"]["POL"].reset_value = 0x04C11DB7
        for field in chip.periph_map["CRC"]["CR"].fields.values():
            if field.name not in ["RESET"]:
                field.unimplemented = True
