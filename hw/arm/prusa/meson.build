subdir('stm32f407')

arm_ss.add(when: 'CONFIG_BUDDYBOARD', if_true: files(
        'buddyboard.c',
        'parts/st25dv64k.c',
        'parts/st7789v.c',
        'parts/tmc2209.c',
        'parts/buddy_input.c',
        'parts/thermistor.c',
        'parts/buddy_visuals.c',
        'parts/fan.c',
        'parts/heater.c'
    ))
# hw_arch += {'arm': buddy_ss}
