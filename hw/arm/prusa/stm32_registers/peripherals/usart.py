from stm32_regtypes import Register, RegisterBitField


def inject_usart_data_regs(periph_map: dict):
    """Inject RDR/TDR for chips that declare them as uint16_t (skipped by the uint32_t parser)."""
    if "USART" not in periph_map:
        return
    usart = periph_map["USART"]
    for name, hex_addr, int_addr, desc in [
        ("RDR", "0x24", 0x24, "USART Receive Data register"),
        ("TDR", "0x28", 0x28, "USART Transmit Data register"),
    ]:
        if name not in usart:
            usart[name] = Register(
                name=name, desc=desc, hex_addr=hex_addr, int_addr=int_addr,
                fields={}, access=None, reset_value=0)


def fix_usart(periph_map: dict):
    """Normalize USART register fields that chip headers define inconsistently.

    BRR: some chips define the field as LPUART (20-bit) for LPUART mode, others
    use DIV_MANTISSA/DIV_FRACTION, others have no Pos/Msk defines at all.
    Replace with a single canonical BRR:16 field across all USART_TYPE_A chips.

    RQR: most chips use old-style non-Pos/Msk defines so the parser captures
    nothing.  Inject the five standard request bits where missing.

    RDR/TDR: some headers have no Pos/Msk, leaving mask=0.  Inject the 9-bit
    data field where missing.
    """
    if "USART" not in periph_map:
        return
    usart = periph_map["USART"]

    if "BRR" in usart:
        usart["BRR"].fields = {
            "BRR": RegisterBitField(
                name="BRR", desc="Baud rate register [15:0]",
                shift=0, width=16, permissions=None, unimplemented=False)
        }

    if "RQR" in usart:
        rqr = usart["RQR"]
        for shift, (name, desc) in enumerate([
            ("ABRRQ", "Auto-Baud Rate Request"),
            ("SBKRQ", "Send Break Request"),
            ("MMRQ",  "Mute Mode Request"),
            ("RXFRQ", "Receive Data flush Request"),
            ("TXFRQ", "Transmit data flush Request"),
        ]):
            if name not in rqr.fields:
                rqr.fields[name] = RegisterBitField(
                    name=name, desc=desc, shift=shift, width=1,
                    permissions=None, unimplemented=False)

    for name, field_desc in [
        ("RDR", "RDR[8:0] bits (Receive Data value)"),
        ("TDR", "TDR[8:0] bits (Transmit Data value)"),
    ]:
        if name in usart and not usart[name].fields:
            usart[name].fields[name] = RegisterBitField(
                name=name, desc=field_desc,
                shift=0, width=9, permissions=None, unimplemented=False)
