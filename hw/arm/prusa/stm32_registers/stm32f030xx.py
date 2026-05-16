from stm32_regtypes import *
from stm32_fixupspec import PeripheralFixup, RegisterFixup, apply_peripheral_fixup
from peripherals.adcc import inject_adc_ccr, split_adcc
from peripherals.crc import CRC_TYPE_A
from peripherals.usart import inject_usart_data_regs

_ADC_PRE = PeripheralFixup(renames={"TR": "TR1"})

_ADC = PeripheralFixup(registers={
    "TR1":   RegisterFixup(reset_value=0x0FFF0000, unimplemented=True),
    "CFGR2": RegisterFixup(unimplemented=True),
})

class stm32f030xx(STM32Fixups):
    common_periph = {"CRC": "CRC_TYPE_A", "I2C": "I2C_TYPE_A", "IWDG": "IWDG_TYPE_A", "ADCC": "ADCC_TYPE_A", "USART": "USART_TYPE_A", "SPI": "SPI_TYPE_A", "TIM": "TIM_TYPE_A"}

    @staticmethod
    def post_register_fixups(chip: STM32Chip):
        # IDR is uint8_t in the header; the regex match fails, so inject it manually.
        field = {}
        field["IDR"] = RegisterBitField(name="IDR", desc="Independent data register", shift=0, width=8, permissions=None, unimplemented=False)
        chip.periph_map["CRC"]["IDR"] = Register(name="IDR", desc="CRC independent data register", hex_addr="0x04", int_addr=0x04, fields=field, access=None, reset_value=0)
        inject_adc_ccr(chip.periph_map)
        inject_usart_data_regs(chip.periph_map)
        apply_peripheral_fixup(chip.periph_map, "ADC", _ADC_PRE)

    @staticmethod
    def post_bitfield_fixups(chip: STM32Chip):
        split_adcc(chip.periph_map)

    @staticmethod
    def supplemental_data(chip: STM32Chip):
        apply_peripheral_fixup(chip.periph_map, "CRC", CRC_TYPE_A)
        apply_peripheral_fixup(chip.periph_map, "ADC", _ADC)
