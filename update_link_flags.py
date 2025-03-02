# Custom settings, as referred to as "extra_script" in platformio.ini
#
# See http://docs.platformio.org/en/latest/projectconf.html#extra-script

from SCons.Script import DefaultEnvironment


env = DefaultEnvironment()

env.Append(
    LINKFLAGS=[
        "-mthumb",
        "-mfloat-abi=hard",
        "-mfpu=fpv4-sp-d16",
        "-march=armv7e-m",
        "-u _printf_float",
       "-static",
       "-u _scanf_float",
       "-Wl,--start-group",
       "-lc",
       "-lm",
       "-Wl,--end-group",




    ]
)