set(PROJECT_NAME "modbus_master")

set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "master.c" "modbus.c")

set(COMPONENT_ADD_INCLUDEDIRS "")

idf_component_register(SRCS "master.c" "modbus.c"
                        INCLUDE_DIRS ".")