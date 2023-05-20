cmake_minimum_required(VERSION 3.16)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

project(sdrx)
find_program(OBJCOPY arm-none-eabi-objcopy)
find_program(OBJDUMP arm-none-eabi-objdump)

set(COMP_FLAGS "-ggdb -Os -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS "-std=c17 ${COMP_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++17 -fno-exceptions ${COMP_FLAGS}")

set(CMAKE_LINKER arm-none-eabi-ld)
set(LINKER_FLAGS "-T ${CMAKE_SOURCE_DIR}/hw/linker/tm4c1249ncpdt.ld --entry ISR_Reset --gc-sections")
set(LINKER_FLAGS "${LINKER_FLAGS} -L/usr/lib/arm-none-eabi/lib/thumb/v7e-m+fp/hard")
set(CMAKE_C_LINK_EXECUTABLE "<CMAKE_LINKER> ${LINKER_FLAGS} <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")
set(CMAKE_CXX_LINK_EXECUTABLE "<CMAKE_LINKER> ${LINKER_FLAGS} <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")

# Include generated headers.
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(
        sdrx

        main.c
        snmp.c
        status.c
        hw/eeprom.c
        hw/emac.c
        hw/interrupts.c
        hw/sys.c
        lib/clk/clk.c
        lib/clk/comp.c
        lib/clk/mono.c
        lib/clk/tai.c
        lib/clk/util.c
        lib/delay.c
        lib/format.c
        lib/led.c
        lib/net.c
        lib/net/arp.c
        lib/net/dhcp.c
        lib/net/dns.c
        lib/net/icmp.c
        lib/net/ip.c
        lib/net/udp.c
        lib/net/util.c
        lib/ptp/common.c
        lib/ptp/pll.c
        lib/ptp/ptp.c
        lib/ptp/src.c
        lib/ptp/tcmp.c
        lib/rand.c
        lib/run.c
        lib/snmp/sensors.c
        lib/snmp/util.c
)

target_link_libraries(
        sdrx
        -static
        -lm
        -lc
)

# a custom target that is always built
add_custom_target(githeader ALL
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/githeader.h)

# creates githeader.h using cmake script
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/githeader.h
        COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/getgit.cmake)

# gitversion.h is a generated file
set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/gitversion.h
        PROPERTIES GENERATED TRUE
        HEADER_FILE_ONLY TRUE)

add_dependencies(sdrx githeader)

# split out debug symbols
add_custom_command(
        TARGET sdrx POST_BUILD
        COMMAND ${OBJCOPY} --only-keep-debug $<TARGET_FILE:sdrx> $<TARGET_FILE:sdrx>.debug
        COMMAND ${OBJCOPY} --strip-debug $<TARGET_FILE:sdrx>
        COMMAND ${OBJCOPY} --add-gnu-debuglink=$<TARGET_FILE_NAME:sdrx>.debug $<TARGET_FILE:sdrx>
)

# create file for lm4flash
add_custom_command(
        TARGET sdrx POST_BUILD
        COMMAND ${OBJCOPY} --output-target binary $<TARGET_FILE:sdrx> $<TARGET_FILE:sdrx>.bin
)
add_custom_command(
        TARGET sdrx POST_BUILD
        COMMAND ${OBJDUMP} -h -S $<TARGET_FILE:sdrx> > $<TARGET_FILE:sdrx>.lst
)
