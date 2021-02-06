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
        'parts/heater.c',
        '3rdParty/shmemq404/shmemq.c'
    ))
# Required if using Message queue IPC
c = meson.get_compiler('c')
arm_ss.add(when: 'CONFIG_BUDDYBOARD', if_true: cc.find_library('rt'))

# hw_arch += {'arm': buddy_ss}
