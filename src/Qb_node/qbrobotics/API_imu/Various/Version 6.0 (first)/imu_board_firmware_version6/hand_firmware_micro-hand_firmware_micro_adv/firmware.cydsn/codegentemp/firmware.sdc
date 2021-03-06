# THIS FILE IS AUTOMATICALLY GENERATED
# Project: C:\Users\MattiaCP\Google Drive\QB\IMU\imu_board_firmware_version6\hand_firmware_micro-hand_firmware_micro_adv\firmware.cydsn\firmware.cyprj
# Date: Wed, 31 Aug 2016 13:58:31 GMT
#set_units -time ns
create_clock -name {isr_clock(routed)} -period 10000000 -waveform {0 5000000} [list [get_pins {ClockBlock/dclk_4}]]
create_clock -name {CyILO} -period 10000 -waveform {0 5000} [list [get_pins {ClockBlock/ilo}] [get_pins {ClockBlock/clk_100k}] [get_pins {ClockBlock/clk_1k}] [get_pins {ClockBlock/clk_32k}]]
create_clock -name {CyIMO} -period 41.666666666666664 -waveform {0 20.8333333333333} [list [get_pins {ClockBlock/imo}]]
create_clock -name {CyPLL_OUT} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/pllout}]]
create_clock -name {CyMASTER_CLK} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/clk_sync}]]
create_generated_clock -name {CyBUS_CLK} -source [get_pins {ClockBlock/clk_sync}] -edges {1 2 3} [list [get_pins {ClockBlock/clk_bus_glb}]]
create_clock -name {CyBUS_CLK(fixed-function)} -period 20.833333333333332 -waveform {0 10.4166666666667} [get_pins {ClockBlock/clk_bus_glb_ff}]
create_generated_clock -name {CLOCK_UART} -source [get_pins {ClockBlock/clk_sync}] -edges {1 3 7} [list [get_pins {ClockBlock/dclk_glb_0}]]
create_generated_clock -name {WATCHDOG_CLK} -source [get_pins {ClockBlock/clk_sync}] -edges {1 1966081 3932161} [list [get_pins {ClockBlock/dclk_glb_1}]]
create_clock -name {WATCHDOG_CLK(fixed-function)} -period 40960000 -waveform {0 20.8333333333333} [get_pins {ClockBlock/dclk_glb_ff_1}]
create_generated_clock -name {timer_clock} -source [get_pins {ClockBlock/clk_sync}] -edges {1 49 97} [list [get_pins {ClockBlock/dclk_glb_3}]]
create_generated_clock -name {isr_clock} -source [get_pins {ClockBlock/clk_sync}] -edges {1 480001 960001} [list [get_pins {ClockBlock/dclk_glb_4}]]
create_generated_clock -name {SPI_IMU_IntClock} -source [get_pins {ClockBlock/clk_sync}] -edges {1 25 49} [list [get_pins {ClockBlock/dclk_glb_2}]]

set_false_path -from [get_pins {__ONE__/q}]
set_false_path -from [get_pins {__ZERO__/q}]

# Component constraints for C:\Users\MattiaCP\Google Drive\QB\IMU\imu_board_firmware_version6\hand_firmware_micro-hand_firmware_micro_adv\firmware.cydsn\TopDesign\TopDesign.cysch
# Project: C:\Users\MattiaCP\Google Drive\QB\IMU\imu_board_firmware_version6\hand_firmware_micro-hand_firmware_micro_adv\firmware.cydsn\firmware.cyprj
# Date: Wed, 31 Aug 2016 13:58:23 GMT
