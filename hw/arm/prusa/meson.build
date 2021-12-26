libglut = cc.find_library('glut', required: false)
if build_machine.system() == 'cygwin'
    message('Cross build, finding opengl32. libglew32 and libglu32')
    libgl = cc.find_library('libopengl32', required: false)
    libglu = cc.find_library('libglu32', required: false)
    libglew = cc.find_library('libglew32', required: false)
else
    message('Native build, looking for GLEW/GLU/GL')
    libgl = cc.find_library('GL', required: false)
    libglu = cc.find_library('GLU', required: false)
    libglew = cc.find_library('GLEW', required: false)
endif

if libglut.found() and libglu.found() and libglew.found() and libgl.found()
    message('OpenGL libraries found - enabling Mini404 GL extensions')
    buddy_gl = declare_dependency(compile_args: '-DBUDDY_HAS_GL')
    arm_ss.add(when: 'CONFIG_BUDDYBOARD', if_true: [libgl, libglu, libglut, libglew, buddy_gl])
    subdir('3rdParty')
    subdir('opengl')
    subdir('assets')
else
    message('OpenGL libraries not found. Install development packages for freeglut, glew, and mesa and re-run configure.')
endif

subdir('stm32f407')

arm_ss.add(when: 'CONFIG_BUDDYBOARD', if_true: files(
        'prusa-mini.c',
        'parts/encoder_input.c',
        'parts/fan.c',
        'parts/gl_dashboard.c',
        'parts/2d-dashboard.c',
        'parts/heater.c',
        'parts/irsensor.c',
        'parts/p404_key_input.c',
        'parts/pinda.c',
        'parts/st7789v.c',
        'parts/thermistor.c',
        'parts/tmc2209.c',
        'utility/ArgHelper.cpp',
        'utility/IKeyClient.cpp',
        'utility/IScriptable.cpp',
        'utility/KeyController.cpp',
        'utility/ScriptHost.cpp',
        'utility/Scriptable.cpp',
        'utility/p404_script_console.c',
        'utility/p404scriptable.c',
        'utility/p404_keyclient.c',
        'utility/p404_motor_if.c',
        'utility/text_helper.c',
    ))

# Use CONFIG_PRUSA_STM32_HACKS when building our target to enable our specific workarounds
prusa_hacks = declare_dependency(compile_args: '-DCONFIG_PRUSA_STM32_HACKS')
arm_ss.add(when: 'CONFIG_BUDDYBOARD', if_true: [prusa_hacks])
softmmu_ss.add(when: 'CONFIG_BUDDYBOARD', if_true: [prusa_hacks])
