subdir('3rdParty')
subdir('opengl')
subdir('stm32f407')
subdir('assets')

arm_ss.add(when: 'CONFIG_BUDDYBOARD', if_true: files(
        'prusa-mini.c',
        'parts/encoder_input.c',
        'parts/fan.c',
        'parts/gl_dashboard.c',
        'parts/heater.c',
        'parts/irsensor.c',
        'parts/st7789v.c',
        'parts/thermistor.c',
        'parts/tmc2209.c',
        'utility/ArgHelper.cpp',
        'utility/IScriptable.cpp',
        'utility/ScriptHost.cpp',
        'utility/p404_script_console.c',
        'utility/p404scriptable.c',
    ))

# Required if using Message queue IPC
c = meson.get_compiler('c')
arm_ss.add(when: 'CONFIG_BUDDYBOARD', if_true: cc.find_library('rt'))
arm_ss.add(when: ['CONFIG_BUDDYBOARD','CONFIG_OPENGL'], if_true: cc.find_library('glut'))
arm_ss.add(when: ['CONFIG_BUDDYBOARD','CONFIG_OPENGL'], if_true: cc.find_library('GLU'))
arm_ss.add(when: ['CONFIG_BUDDYBOARD','CONFIG_OPENGL'], if_true: cc.find_library('GL'))
arm_ss.add(when: ['CONFIG_BUDDYBOARD','CONFIG_OPENGL'], if_true: cc.find_library('GLEW'))

# hw_arch += {'arm': buddy_ss}
