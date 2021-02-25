#!/bin/sh
touch dummy.bin
build/qemu-system-buddy -machine prusabuddy -kernel dummy.bin -append scripthelpmd | grep -A500 "# Scripting" > ref/autogen/Scripting-Mini.md

