#!/bin/sh
touch dummy.bin
build/qemu-system-buddy -machine prusa-mini -kernel dummy.bin -append scripthelpmd | sed -n '/# Scripting*/,/End Scripting options/p'  > ref/autogen/Scripting-Mini.md
build/qemu-system-buddy -machine prusa-mk4-027c -kernel dummy.bin -append scripthelpmd | sed -n '/# Scripting*/,/End Scripting options/p' > ref/autogen/Scripting-Mk4-027.md
build/qemu-system-buddy -machine prusa-mk3-35 -kernel dummy.bin -append scripthelpmd | sed -n '/# Scripting*/,/End Scripting options/p' > ref/autogen/Scripting-Mk3_5.md
build/qemu-system-buddy -machine prusa-mk3-39 -kernel dummy.bin -append scripthelpmd | sed -n '/# Scripting*/,/End Scripting options/p' > ref/autogen/Scripting-Mk3_9.md
#build/qemu-system-buddy -machine prusa-mk4-034 -kernel dummy.bin -append scripthelpmd | sed -n '/# Scripting*/,/End Scripting options/p' > ref/autogen/Scripting-Mk4-034.md
build/qemu-system-buddy -machine prusa-xl-050 -kernel dummy.bin -append scripthelpmd | sed -n '/# Scripting*/,/End Scripting options/p' > ref/autogen/Scripting-XL-050.md
build/qemu-system-buddy -machine prusa-xl-extruder-060-0 -kernel dummy.bin -append scripthelpmd,no-bridge | sed -n '/# Scripting*/,/End Scripting options/p' > ref/autogen/Scripting-XL-E-060.md
build/qemu-system-buddy -machine prusa-xl-bed-060 -kernel dummy.bin -append scripthelpmd,no-bridge | sed -n '/# Scripting*/,/End Scripting options/p' > ref/autogen/Scripting-XL-Bed-060.md

build/qemu-system-buddy -machine prusa-mini -kernel dummy.bin -append keyhelpmd | sed -n '/Available Key Controls:/,/End Key Controls/p' > ref/autogen/Keys-Mini.md
build/qemu-system-buddy -machine prusa-mk4-027c -kernel dummy.bin -append keyhelpmd | sed -n '/Available Key Controls:/,/End Key Controls/p' > ref/autogen/Keys-Mk4-027.md
build/qemu-system-buddy -machine prusa-mk3-35 -kernel dummy.bin -append keyhelpmd | sed -n '/Available Key Controls:/,/End Key Controls/p' > ref/autogen/Keys-Mk3_5.md
build/qemu-system-buddy -machine prusa-mk3-39 -kernel dummy.bin -append keyhelpmd | sed -n '/Available Key Controls:/,/End Key Controls/p' > ref/autogen/Keys-Mk3_9.md
build/qemu-system-buddy -machine prusa-xl-050 -kernel dummy.bin -append keyhelpmd | sed -n '/Available Key Controls:/,/End Key Controls/p' > ref/autogen/Keys-XL-050.md
build/qemu-system-buddy -machine prusa-xl-extruder-060-0 -kernel dummy.bin -append keyhelpmd,no-bridge | sed -n '/Available Key Controls:/,/End Key Controls/p' > ref/autogen/Keys-XL-E-060.md
build/qemu-system-buddy -machine prusa-xl-bed-060 -kernel dummy.bin -append keyhelpmd,no-bridge | sed -n '/Available Key Controls:/,/End Key Controls/p' > ref/autogen/Keys-XL-Bed-060.md
