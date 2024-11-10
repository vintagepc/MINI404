from enum import Enum
from dataclasses import dataclass
from abc import ABC, abstractmethod

class RegisterPermission(Enum):
    READ_WRITE = 0
    READ_ONLY = 1
    WRITE_ONLY = 2
    RESERVED = 3

class RegisterAccess(Enum):
    BYTE = 1
    HALF_WORD = 2
    WORD = 4

class STM32Fixups(ABC):
    @staticmethod
    @abstractmethod
    def post_register_fixups(chip):
        """Method to perform fixups on the register map after it has been loaded but before bitfields are processed"""
        pass
    
    @staticmethod
    @abstractmethod
    def post_bitfield_fixups(chip):
        """Method to perform fixups on the register map after bitfields have been processed"""
        pass

    @staticmethod
    @abstractmethod
    def supplemental_data(chip):
        """Method to add additional data to the register map that cannot be extracted from the header file alone"""
        pass

# A dataclass to store a chip name, and function pointers
# to either imported data "fixups", or functions that supply
# additional data that cannot be extracted from the header file alone.
# This allows for inputting additional data that persists even if the input header file is updated.
@dataclass
class STM32Chip:
    name: str
    header: str
    fixups: STM32Fixups
    periph_map: {}
    periph_addrs: {}
    periph_irqs: {}
    not_found: {}
    def __init__(self, name, header, fixups):
        self.name = name
        self.header = header
        self.fixups = fixups
        self.periph_map = {}
        self.periph_addrs = {}
        self.periph_irqs = {}
        self.not_found = {}

@dataclass
class RegisterBitField:
    name: str
    desc: str
    width: int
    shift: int
    permissions: RegisterPermission
    unimplemented: bool

    def print(self):
        print(f"\tName: {self.name}, Width: {self.width}, Shift: {self.shift}, Permissions: {self.permissions}, Unimplemented: {self.unimplemented}")

    def get_unimp(self):
        return 1 if self.unimplemented else 0



@dataclass 
class Register:
    name: str
    desc: str
    hex_addr: str
    int_addr: int
    fields: {}
    access: RegisterAccess
    reset_value: int
    unimplemented: bool = False

    def print(self):
        print(f"Name: {self.name}, Description: {self.desc}, Address: {self.hex_addr}, Access: {self.access}, Reset Value: {self.reset_value}")
        for field in self.fields:
            field.print()
    
    def get_valid_mask(self):
        mask = 0
        for c,field in self.fields.items():
            mask |= (2**field.width - 1) << field.shift
        return mask

    def get_unimp_mask(self):
        if (self.unimplemented):
            return 0xFFFFFFFF
        mask = 0
        for c,field in self.fields.items():
            if field.unimplemented:
                mask |= (2**field.width - 1) << field.shift
        return mask

    def print_fields_c(self) -> str:
        retval = ""
        
        fields_by_shift = [None] * 32
        for field in self.fields.values():
            fields_by_shift[field.shift] = field

        next = 0
        for field in fields_by_shift:
            if field is None:
                continue
            if field.shift > next:
                retval = retval + f"\t\tuint32_t _reserved{next:<5}:{field.shift - next:>2}; \n"
            retval = retval + f"\t\tuint32_t {field.name:<14}:{field.width:>2}; "
            if field.desc is not None:
                retval = retval + f"// {field.desc}"
            retval = retval + "\n"
            next = field.shift + field.width
        if next < 32:
            retval = retval + f"\t\tuint32_t _reserved{next:<5}:{32 - next:>2}; \n"
        return retval

    def print_c(self) -> str:
        valid_mask = self.get_valid_mask()
        retval = f"\t[RI_{self.name:<15}] = {{ .reset_val = 0x{self.reset_value:08X}, .not_reserved = true, .unimp_mask = 0x{self.get_unimp_mask():08X}, .mask = 0x{valid_mask:08X} }}, /* mask = 0b{valid_mask:032b} */\n"
        return retval
