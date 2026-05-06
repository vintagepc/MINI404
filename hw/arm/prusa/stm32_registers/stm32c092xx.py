from stm32_regtypes import *

class stm32c092xx(STM32Fixups):
    @staticmethod
    def supplemental_data(chip: STM32Chip):
        pass

    @staticmethod
    def post_register_fixups(chip: STM32Chip):
        chip.periph_map["FDCAN"] = chip.periph_map.pop("FDCAN_Global")
        chip.periph_map["FDCAN"]["CKDIV"] = chip.periph_map["FDCAN_Config"]["CKDIV"]
        chip.periph_map.pop("FDCAN_Config")

    @staticmethod
    def post_bitfield_fixups(chip: STM32Chip):
        pass