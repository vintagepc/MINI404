subdir('stm32f407')

arm_ss.add(when: 'CONFIG_BUDDYBOARD', if_true: files(
        'prusa-mini.c',
        'parts/encoder_input.c',
        'parts/fan.c',
        'parts/heater.c',
        'parts/irsensor.c',
        'parts/mini_visuals.c',
        'parts/st7789v.c',
        'parts/thermistor.c',
        'parts/tmc2209.c',
        '3rdParty/shmemq404/shmemq.c',
        'utility/ArgHelper.cpp',
        'utility/IScriptable.cpp',
        'utility/ScriptHost.cpp',
        'utility/p404_script_console.c',
        'utility/p404scriptable.c',
    ))
# Required if using Message queue IPC
c = meson.get_compiler('c')
arm_ss.add(when: 'CONFIG_BUDDYBOARD', if_true: cc.find_library('rt'))

# hw_arch += {'arm': buddy_ss}
