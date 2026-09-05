def merge_fdcan(periph_map: dict):
    """Merge FDCAN_Global + FDCAN_Config into a single FDCAN peripheral."""
    if "FDCAN_Global" not in periph_map:
        return
    periph_map["FDCAN"] = periph_map.pop("FDCAN_Global")
    if "FDCAN_Config" in periph_map:
        periph_map["FDCAN"]["CKDIV"] = periph_map["FDCAN_Config"]["CKDIV"]
        periph_map.pop("FDCAN_Config")
