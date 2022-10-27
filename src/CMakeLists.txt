
add_executable(src
        main.c
        )

pico_enable_stdio_uart(src 1)

# pull in common dependencies
target_link_libraries(src
        pico_stdlib
        hardware_adc
        hardware_dma
        hardware_i2c
        hardware_spi
        hardware_watchdog
        hardware_uart
        hardware_irq
        hardware_sync
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(src)

include_directories(
	../inc
	)

## add url via pico_set_program_url
#//example_auto_set_url(adc)