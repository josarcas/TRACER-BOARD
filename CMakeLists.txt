cmake_minimum_required(VERSION 3.19)

include(pico_sdk_import.cmake)

project(Qtrazador)

pico_sdk_init()

add_executable(main
    main.c)


FILE(GLOB FreeRTOS_src FreeRTOS-Kernel/*.c)

add_library( FreeRTOS STATIC
    ${FreeRTOS_src}
    FreeRTOS-Kernel/portable/GCC/ARM_CM0/port.c
    FreeRTOS-Kernel/portable/MemMang/heap_4.c)

target_include_directories( FreeRTOS PUBLIC
    FreeRTOS-Kernel/include
    include/
    FreeRTOS-Kernel/portable/GCC/ARM_CM0)

target_link_libraries(main
	pico_stdlib
	hardware_i2c 
	hardware_uart 
	hardware_timer 
	hardware_dma 
	hardware_adc 
	hardware_spi 
	hardware_irq
	FreeRTOS)

pico_add_extra_outputs(main)