from stm32_regtypes import Register, RegisterAccess, RegisterBitField, RegisterPermission, STM32Chip
import re
import dataclasses

from stm32f030xx import *
from stm32f427xx import *
from stm32g070xx import *
from stm32h503xx import *

# Create a 2-level dictionary for the register map:
# top level is peripheral name, second level is register name.


reg_pattern = re.compile(r'.+uint32_t (\w+);.+/\*!<\s+(.+)\s+Address offset: (0x\w+)\s+\*/')
typedef_pattern = re.compile(r'}\s?(\w+)_TypeDef;')

def parse_register(line: str) -> Register:
	# Construct a regexp to find register name and address:
	match = reg_pattern.match(line)
	if "RESERVED" in line:
		return None
	elif match:
		#print(match.groups())
		return Register(name = match.group(1), desc=match.group(2).strip(), hex_addr = match.group(3), int_addr = int(match.group(3), 16), fields = {}, access = None, reset_value = 0)
	else:
		return None

mask2nbits = {
	1: 1,
	3: 2,
	7: 3,
	15: 4,
	31: 5,
	63: 6,
	127: 7,
	255: 8,
	511: 9,
	1023: 10,
	2047: 11,
	4095: 12,
	8191: 13,
	16383: 14,
	32767: 15,
	65535: 16,
	131071: 17,
	262143: 18,
	524287: 19,
	1048575: 20,
	2097151: 21,
	4194303: 22,
	8388607: 23,
	16777215: 24,
	33554431: 25,
	67108863: 26,
	134217727: 27,
	268435455: 28,
	536870911: 29,
	1073741823: 30,
	2147483647: 31,
	4294967295: 32
}

shift_pattern = re.compile(r'\((\d+)U\)')
msk_pattern = re.compile(r'\((0x[0-9A-F]+)UL')
msk2_pattern = re.compile(r'\(([0-9]+)UL')
desc_pattern = re.compile(r'^.*?(/\*.*\*/)$')

def parse_bitfield_mask(mask: int, field: str, reg: Register):
	if mask in mask2nbits:
		reg.fields[field].width = mask2nbits[mask]
	else:
		mskstr = f"0{mask:032b}"
		cur_start = 0
		n_blocks = 0
		cur_msk = ""
		while mskstr is not "":
			if mskstr[-1] == "1":
				cur_msk += "1"
			else:
				if cur_msk is not "":
					f_start = cur_start + 1
					f_width = len(cur_msk)
					if n_blocks == 0:
						reg.fields[field].width = f_width
					else:
						f_name = f"{field}_{n_blocks}"
						reg.fields[f_name] = dataclasses.replace(reg.fields[field])
						reg.fields[f_name].name = f_name
						reg.fields[f_name].width = f_width
						reg.fields[f_name].shift += f_start
						# print("Found field: ", f_name, f_width, f_start)
						# for f,v in reg.fields.items():
						# 	v.print()
					n_blocks += 1
					cur_msk = ""
					cur_start += f_width
				else:
					cur_start += 1
			mskstr = mskstr[:-1]

def parse_bitfield_info(line: str, info: STM32Chip):
	periph_map = info.periph_map
	not_found_regs = info.not_found
	tokens = line.split()
	if len(tokens) < 3:
		return
	def_parts = tokens[1].split("_")
	if len(def_parts) < 3:
		return

	periph = def_parts[0]
	reg = def_parts[1]
	field = def_parts[2]

	if tokens[1].endswith("Pos") or tokens[1].endswith("Msk"):
		field = "_".join(def_parts[2:-1])
		suffix = def_parts[-1]
	else:
		suffix = None


	# if len(def_parts) > 3:
	# 	suffix = def_parts[3]
	# elif field == "Msk" or field == "Pos": #Registers with no fields
	# 	suffix = field
	# 	field = reg
	# else:
	# 	suffix = None

	if periph not in periph_map:
		not_found_regs[f'{periph}'] = True
		return
	elif reg not in periph_map[periph]:
		not_found_regs[f'{periph}_{reg}'] = True
		return

	if suffix == "Pos":
		shift_match = shift_pattern.match(tokens[2])
		if not shift_match:
			print("Shift not found: ", tokens)
		else:
			shift = int(shift_match.group(1))
			periph_map[periph][reg].fields[field] = (RegisterBitField(name = field, desc = None, width = None, shift = shift, permissions = None, unimplemented = False))
	elif suffix == "Msk":
		# Bugfix, some includes have (x...UL) instead of (0x...UL)
		if tokens[2].startswith('(x'):
			tokens[2] = tokens[2].replace('x', '0x')
		msk_match = msk_pattern.match(tokens[2])
		msk2_match = msk2_pattern.match(tokens[2])
		if msk_match:
			intval = int(msk_match.group(1), 16)
			parse_bitfield_mask(intval, field, periph_map[periph][reg])
		elif msk2_match:
			intval = int(msk2_match.group(1))
			parse_bitfield_mask(intval, field, periph_map[periph][reg])
		else:
			print("Mask not found: ", line)
	elif suffix == None and "Msk" in tokens[2]:
		desc = desc_pattern.match(line)
		if desc:
			field = "_".join(def_parts[2:])
			try:
				periph_map[periph][reg].fields[field].desc = desc.group(1)
			except KeyError:
				print(f"Field {field} not found: ", line)

bitfield_pattern = re.compile(r'([A-Z0-9]+)_([A-Z0-9]+)_([A-Z0-9]+)')
pos_pattern = re.compile(r'\((\d+)U\)')
address_pattern = re.compile(r'#define (\w+_BASE(_NS)?)\s+\(?([^\/\)]+)\)?')
irq_pattern = re.compile(r'\s+(\w+_IRQn)[\s=]+(\d+),')

def global_supplemental_data(info: STM32Chip):
	if "IWDG" in info.periph_map:
		info.periph_map["IWDG"]["RLR"].reset_value = info.periph_map["IWDG"]["RLR"].get_valid_mask()
		if "WINR" in info.periph_map["IWDG"]:
			info.periph_map["IWDG"]["WINR"].reset_value = info.periph_map["IWDG"]["WINR"].get_valid_mask()
		if "EWCR" in info.periph_map["IWDG"]:
			info.periph_map["IWDG"]["EWCR"].unimplemented = True
		if "WINR" in info.periph_map["IWDG"]:
			info.periph_map["IWDG"]["WINR"].unimplemented = True
	if "ADCC" in info.periph_map:
		for field in info.periph_map["ADCC"]["CCR"].fields.values():
			if field.name not in ["PRESC", "VREFEN"]:
				field.unimplemented = True


def process_chip(info: STM32Chip):
	in_bitfield = False
	counter = 0
	with open(f"src/{info.header}", "r+") as f:
		for line in f:
			if "IRQn" in line:
				irq_match = irq_pattern.match(line)
				if irq_match:
					info.periph_irqs[irq_match.group(1)] = int(irq_match.group(2))
			addr_match = address_pattern.match(line)
			if addr_match:
				calc_bits = addr_match.group(3).split()
				for i in range(len(calc_bits)):
					if calc_bits[i] in info.periph_addrs:
						calc_bits[i] = f"{info.periph_addrs[calc_bits[i]]}"
					elif "0x" in calc_bits[i]:
						calc_bits[i] = f"{int(calc_bits[i].replace('UL',''), 16)}"
				try:
					info.periph_addrs[addr_match.group(1)] = eval("".join(calc_bits))
				except Exception as e:
					print("Address err:", addr_match.group(1), calc_bits)
					exit()
				
			if not in_bitfield and "typedef struct" in line:
				tmp_dict = {}
				while "}" not in line:
					line = f.readline()
					reg = parse_register(line)
					if reg is not None:
						tmp_dict[reg.name] = reg
				match = typedef_pattern.match(line)
				if match:
					info.periph_map[match.group(1)] = tmp_dict
					# print("Found Peripheral: ", match.group(1))
			if not in_bitfield and ("Pos" in line or "Msk" in line):
				print("Started bitfields...")
				in_bitfield = True
				info.fixups.post_register_fixups(info)
			if in_bitfield:
				parse_bitfield_info(line, info)
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
	STM32Chip(name="stm32h503", header="stm32h503xx.h", fixups=stm32h503xx, gen_list=["RCC", "PWR", "ICACHE"]),
	]


for chip in chip_data:
	process_chip(chip)
