from stm32_fixupspec import PeripheralFixup, RegisterFixup

# G070 and C092 share the same EXTI IMR1/EMR1 behaviour.
EXTI_TYPE_A = PeripheralFixup(registers={
    # IMR1/EMR1: all chip headers define aggregate convenience masks (IM, EM) in
    # addition to individual per-line bits. The non-contiguous mask parser also
    # generates numbered sub-fields (IM_1..IM_5) for each discontiguous run of
    # the aggregate mask. These artifacts overlap the real per-line fields and
    # cause the generated struct to exceed 32 bits — remove them explicitly.
    "IMR1": RegisterFixup(
        reset_value=0xFFF80000,
        unimplemented=True,
        fields_remove=["IM", "IM_1", "IM_2", "IM_3", "IM_4", "IM_5"],
    ),
    "EMR1": RegisterFixup(
        unimplemented=True,
        fields_remove=["EM"],
    ),
})
