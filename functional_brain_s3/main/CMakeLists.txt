set(PROJECT_NAME "modbus_master")

set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "master.c" "modbus.c" "queue.c" "data_point.c" "sd_card_com.c")

set(COMPONENT_ADD_INCLUDEDIRS "")

idf_component_register(SRCS "master.c" "modbus.c" "queue.c" "data_point.c" "sd_card_com.c"
                        INCLUDE_DIRS ".")