from stm32_regtypes import STM32Chip, STM32Fixups, Register
from stm32_fixupspec import PeripheralFixup, RegisterFixup, apply_peripheral_fixup
from peripherals.adcc import inject_adc_ccr, split_adcc
from peripherals.crc import CRC_TYPE_A
from peripherals.exti import EXTI_TYPE_A

_ADC = PeripheralFixup(
    renames={"AWD1TR": "TR1", "AWD2TR": "TR2", "AWD3TR": "TR3", "TR3": "G070_TR3"},
    registers={
        "TR1":      RegisterFixup(reset_value=0x0FFF0000, unimplemented=True),
        "TR2":      RegisterFixup(reset_value=0x0FFF0000, unimplemented=True),
        "G070_TR3": RegisterFixup(reset_value=0x0FFF0000, unimplemented=True),
        "CFGR2":    RegisterFixup(fields_unimplemented_except=["OVSS", "OVSR", "OVSE"]),
    })

class stm32g070xx(STM32Fixups):
    @staticmethod
    def post_register_fixups(chip: STM32Chip):
        inject_adc_ccr(chip.periph_map)
        # Rename IT_LINE_SR{1..32} to datasheet names ITLINE{0..31} so that the
        # bitfield parser can capture SYSCFG_ITLINE{N}_SR_* defines in phase 2.
        syscfg = chip.periph_map["SYSCFG"]
        for i in range(32):
            old_name = f"IT_LINE_SR{i + 1}"
            if old_name in syscfg:
                reg = syscfg.pop(old_name)
                reg.name = f"ITLINE{i}"
                syscfg[f"ITLINE{i}"] = reg

    @staticmethod
    def post_bitfield_fixups(chip: STM32Chip):
        split_adcc(chip.periph_map)

    @staticmethod
    def supplemental_data(chip: STM32Chip):
        apply_peripheral_fixup(chip.periph_map, "CRC", CRC_TYPE_A)
        apply_peripheral_fixup(chip.periph_map, "ADC", _ADC)
        apply_peripheral_fixup(chip.periph_map, "EXTI", EXTI_TYPE_A)
