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

libglut = cc.find_library('glut', required,false)
if meson.is_cross_build()
    message('Cross build, finding opengl32. libglew32 and libglu32')
    libgl = cc.find_library('libopengl32', required,false)
    libglu = cc.find_library('libglu32', required,false)
    libglew = cc.find_library('libglewl32', required,false)
#    arm_ss.add(when: ['CONFIG_BUDDYBOARD','CONFIG_OPENGL'], if_true: cc.find_library('libopengl32'))
#    arm_ss.add(when: ['CONFIG_BUDDYBOARD','CONFIG_OPENGL'], if_true: cc.find_library('libglu32'))
#    arm_ss.add(when: ['CONFIG_BUDDYBOARD','CONFIG_OPENGL'], if_true: cc.find_library('libglew32'))
else
    message('Native build, looking for GLEW/GLU/GL')
    libgl = cc.find_library('libopengl32', required,false)
    libglu = cc.find_library('libglu32', required,false)
    libglew = cc.find_library('libglewl32', required,false)
#    arm_ss.add(when: ['CONFIG_BUDDYBOARD','CONFIG_OPENGL'], if_true: cc.find_library('GLU'))
#    arm_ss.add(when: ['CONFIG_BUDDYBOARD','CONFIG_OPENGL'], if_true: cc.find_library('GL'))
#    arm_ss.add(when: ['CONFIG_BUDDYBOARD','CONFIG_OPENGL'], if_true: cc.find_library('GLEW'))
endif

if libglut.found() && libglu.found() && libglew.found() && libgl.found()
    message ("OpenGL libraries found - enabling Mini404 GL extensions");
    add_project_arguments('-DBUDDY_HAS_GL', language : 'c')
    add_project_arguments('-DBUDDY_HAS_GL', language : 'c++')
    arm_ss.add(when: true, if_true: libgl libglu libglut libglew)
else
    message ("OpenGL libraries not found. Install development packages for freeglut, glew, and mesa and re-run configure.");

endif
