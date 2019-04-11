# THIS FILE IS AUTOMATICALLY GENERATED
# Project: C:\Users\MattiaCP\Desktop\FIRMWARES\Firmware IMU Mini\IMU_Glove_Mini\IMU_Glove_Mini.cydsn\IMU_Glove_Mini.cyprj
# Date: Sat, 30 Apr 2016 15:24:37 GMT
#set_units -time ns
create_clock -name {ADC_Ext_CP_Clk(routed)} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/dclk_1}]]
create_clock -name {isr_clock(routed)} -period 7000000 -waveform {0 3500000} [list [get_pins {ClockBlock/dclk_3}]]
create_clock -name {CyILO} -period 1000000 -waveform {0 500000} [list [get_pins {ClockBlock/ilo}] [get_pins {ClockBlock/clk_100k}] [get_pins {ClockBlock/clk_1k}] [get_pins {ClockBlock/clk_32k}]]
create_clock -name {CyIMO} -period 41.666666666666664 -waveform {0 20.8333333333333} [list [get_pins {ClockBlock/imo}]]
create_clock -name {CyPLL_OUT} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/pllout}]]
create_clock -name {CyMASTER_CLK} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/clk_sync}]]
create_generated_clock -name {CyBUS_CLK} -source [get_pins {ClockBlock/clk_sync}] -edges {1 2 3} [list [get_pins {ClockBlock/clk_bus_glb}]]
create_generated_clock -name {CLOCK_UART} -source [get_pins {ClockBlock/clk_sync}] -edges {1 13 27} -nominal_period 270.83333333333331 [list [get_pins {ClockBlock/dclk_glb_0}]]
create_clock -name {ADC_Ext_CP_Clk} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/dclk_glb_1}]]
create_generated_clock -name {SPIM_1_IntClock} -source [get_pins {ClockBlock/clk_sync}] -edges {1 25 49} [list [get_pins {ClockBlock/dclk_glb_2}]]
create_generated_clock -name {ADC_theACLK} -source [get_pins {ClockBlock/clk_sync}] -edges {1 151 301} [list [get_pins {ClockBlock/aclk_glb_0}]]
create_clock -name {ADC_theACLK(fixed-function)} -period 3125 -waveform {0 1562.5} [get_pins {ClockBlock/aclk_glb_ff_0}]
create_generated_clock -name {isr_clock} -source [get_pins {ClockBlock/clk_sync}] -edges {1 288001 672001} [list [get_pins {ClockBlock/dclk_glb_3}]]

set_false_path -from [get_pins {__ONE__/q}]

# Component constraints for C:\Users\MattiaCP\Desktop\FIRMWARES\Firmware IMU Mini\IMU_Glove_Mini\IMU_Glove_Mini.cydsn\TopDesign\TopDesign.cysch
# Project: C:\Users\MattiaCP\Desktop\FIRMWARES\Firmware IMU Mini\IMU_Glove_Mini\IMU_Glove_Mini.cydsn\IMU_Glove_Mini.cyprj
# Date: Sat, 30 Apr 2016 15:24:30 GMT