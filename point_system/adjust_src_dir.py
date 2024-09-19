
Import("env")

# access to global build environment
print(env)

# Dump construction environments (for debug purpose)
# print(env.Dump())


# "env.GetProjectOption" shortcut for the active environment
VALUE_1 = env.GetProjectOption("custom_option1")

env['PROJECT_SRC_DIR'] = env['PROJECT_DIR'] + VALUE_1

# env.Append(CPPDEFINES=[('#undef', _DEBUG_)])

print("Setting the project directory to: {}".format(env['PROJECT_SRC_DIR']))
