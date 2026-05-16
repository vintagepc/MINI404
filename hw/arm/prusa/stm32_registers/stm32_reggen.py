from stm32_regtypes import Register, RegisterAccess, RegisterBitField, RegisterPermission, STM32Chip
from stm32_headerparser import STM32HeaderParser
from stm32_fixupspec import apply_peripheral_fixup
from peripherals.adcc import ADCC_TYPE_A
from peripherals.iwdg import apply_iwdg_fixup
from peripherals.i2c import I2C_TYPE_A

from stm32f030xx import *
from stm32c092xx import *
from stm32f427xx import *
from stm32g070xx import *
from stm32h503xx import *


def global_supplemental_data(info: STM32Chip):
	apply_iwdg_fixup(info.periph_map)
	apply_peripheral_fixup(info.periph_map, "ADCC", ADCC_TYPE_A)
	apply_peripheral_fixup(info.periph_map, "I2C", I2C_TYPE_A)


def process_chip(info: STM32Chip):
	parser = STM32HeaderParser()
	parser.load(f"src/{info.header}")

	# Phase 1: extract typedef structs (with array expansion), addresses, IRQs
	parsed = parser.parse_registers()
	info.periph_map   = parsed.periph_map
	info.periph_addrs = parsed.periph_addrs
	info.periph_irqs  = parsed.periph_irqs
	info.not_found    = parsed.not_found

	# Post-register fixups run before bitfield parsing so injected registers
	# also get their bitfields captured in phase 2.
	info.fixups.post_register_fixups(info)

	# Phase 2: populate bitfield Pos/Msk definitions
	print("Started bitfields...")
	parser.parse_bitfields(info.periph_map, info.not_found)

	print("Not found: ", info.not_found)
	info.fixups.post_bitfield_fixups(info)
	info.fixups.supplemental_data(info)

	global_supplemental_data(info)
	info.generate_all()
	info.fixups.do_custom_gen(info)


chip_data = [
	STM32Chip(name="stm32f030", header="stm32f030xc.h", fixups=stm32f030xx, gen_list=[]),
	STM32Chip(name="stm32f427", header="stm32f427xx.h", fixups=stm32f427xx, gen_list=[]),
	STM32Chip(name="stm32g070", header="stm32g070xx.h", fixups=stm32g070xx, gen_list=[]),
	STM32Chip(name="stm32h503", header="stm32h503xx.h", fixups=stm32h503xx, gen_list=["RCC", "PWR", "ICACHE", "DMA", "FLASH"]),
	STM32Chip(name="stm32c092", header="stm32c092xx.h", fixups=stm32c092xx, gen_list=["RCC", "SYSCFG"]),
]


for chip in chip_data:
	print(f"Processing {chip.name}...")
	process_chip(chip)
