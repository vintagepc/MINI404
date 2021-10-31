#!/bin/sh
touch dummy.bin
build/qemu-system-buddy -machine prusa-mini -kernel dummy.bin -append scripthelpmd | grep -A500 "# Scripting" > ref/autogen/Scripting-Mini.md
build/qemu-system-buddy -machine prusa-mini -kernel dummy.bin -append keyhelpmd | grep -A500 "Available Key Controls" > ref/autogen/Keys-Mini.md

