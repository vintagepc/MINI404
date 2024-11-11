from stm32_regtypes import STM32Chip, STM32Fixups, Register

class stm32f427xx(STM32Fixups):
    @staticmethod
    def post_register_fixups(chip: STM32Chip):
        # For some reason this is a uint8_t in the header and it fails the regex match.
        chip.periph_map["CRC"]["IDR"] = Register(name="IDR", desc="CRC independent data register", hex_addr="0x04", int_addr=0x04, fields={}, access=None, reset_value=0)

    @staticmethod
    def supplemental_data(chip: STM32Chip):
        chip.periph_map["CRC"]["DR"].reset_value = 0xFFFFFFFF

    @staticmethod
    def do_custom_gen(chip: STM32Chip):
        chip.generate_meta_only("CRC")
        chip.generate_meta_only("RNG")
