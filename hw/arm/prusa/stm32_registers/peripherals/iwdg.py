from stm32_fixupspec import PeripheralFixup, RegisterFixup


def apply_iwdg_fixup(periph_map: dict):
    if "IWDG" not in periph_map:
        return
    iwdg = periph_map["IWDG"]
    iwdg["RLR"].reset_value = iwdg["RLR"].get_valid_mask()
    if "WINR" in iwdg:
        iwdg["WINR"].reset_value = iwdg["WINR"].get_valid_mask()
        iwdg["WINR"].unimplemented = True
    if "EWCR" in iwdg:
        iwdg["EWCR"].unimplemented = True
