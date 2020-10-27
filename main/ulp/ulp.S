#include <soc/rtc_cntl_reg.h>
#include <soc/rtc_io_reg.h>
#include <soc/soc_ulp.h>

    .set wake_up_condition, 3
    .set samples_log, 4
    .set samples, 16 // max 16
    .set ldr_adc_channel, 6
    .set general_counter_1min, 0x258 //0x249F
    .set general_counter_max, 0xE0F // 0xE10

    .bss
    .global pir_sensor_readings
pir_sensor_readings:
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0

    .global pir_sensor_reading_number
pir_sensor_reading_number:
    .long 0

    .global ldr_sensor_readings
ldr_sensor_readings:
    .long 0
    .long 0

    .global ldr_sensor_reading_number
ldr_sensor_reading_number:
    .long 0

    .global pir_io_number
pir_io_number:
    .long 0

    .global general_counter
general_counter:
    .long 0

    .global general_counter_lo
general_counter_lo:
    .long 0

    .global finished_handling_data
finished_handling_data:
    .long 0

    .global main_cpu_awake
main_cpu_awake:
    .long 0

    .global data_loss
data_loss:
    .long 0

.global debug
debug:
    .long 0

    .text
    .global start
start:                                                   // if finished_handling_data:
    move r2, 0
    move r3, finished_handling_data                       //     general_counter = 0
    ld r0, r3, 0                                         // else:
    jumpr check_over_general_counter, 1, lt              //     jump to check_over_general_counter
    move r0, 0
    st r0, r3, 0
    move r3, general_counter
    move r0, 0
    st r0, r3, 0
    jump increment_general_counter


check_over_general_counter:
    move r3, general_counter
    ld r0, r3, 0
    jumpr increment_general_counter, general_counter_max, lt/*
    move r3, debug
    ld r2, r3, 0
    add r2, r2, 1
    st r2, r3, 0*/
    move r3, data_loss
    ld r0, r3, 0
    jumpr wake_up_main_cpu, 1, ge
    move r3, pir_sensor_reading_number
    ld r0, r3, 0
    add r0, r0, 1
    st r0, r3, 0
    move r3, ldr_sensor_reading_number
    ld r0, r3, 0
    add r0, r0, 1
    st r0, r3, 0
    move r3, data_loss
    move r0, 1
    st r0, r3, 0
    jump wake_up_main_cpu

increment_general_counter:
    //move r3, general_counter
    //ld r1, r3, 0
    //move r1, r0
    add r0, r0, 1
    st r0, r3, 0
    move r1, 0                                       // initialize a mintute counter on R1

get_minutes_check_1_minute:                          // if general_counter % general_counter_1min == 0:
    sub r0, r0, general_counter_1min                 //     jump to set_pir_sr_number
    jump read_pir_sensor, ov                         // else:
    jump check_if_should_send_data, eq                       //     R1 = general_counter % general_counter_1min
    add r1, r1, 1                                    //     jump to read_pir_sensor
    jump get_minutes_check_1_minute

check_if_should_send_data:
    add r1, r1, 1
    sub r0, r1, 3
    jump set_pir_sr_number, ov
    move r2, 1

set_pir_sr_number:                                   // ldr_sensor_reading_number = R0
    move r3, pir_sensor_reading_number
    st r1, r3, 0
    //add r1, r1, 1
    move r0, 0                                       // R0 = 0

check_3_minutes:                                     // R0 = R1 % 3
    sub r1, r1, 3
    jump set_ldr_sr_number, ov
    add r0, r0, 1
    jump check_3_minutes

set_ldr_sr_number:                                   // ldr_sensor_reading_number = R0
    move r3, ldr_sensor_reading_number
    st r0, r3, 0
/*
increment_pir_sr_number:                             // pir_sensor_reading_number += 1
    move r3, pir_sensor_reading_number
    ld r0, r3, 0
    add r0, r0, 1
    st r0, r3, 0*/

read_pir_sensor:
    READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)
    move r3, pir_io_number
    ld r3, r3, 0                                     // R0 = read pir sensor
    rsh r0, r0, r3                                   // if R0 != 0:
    and r0, r0, 1                                    //     call wake_up_main_cpu
    jump setup_ldr_sensor_reading, eq                //     jump to store_pir_sensor_reading
                    // else:
                               //     jump to setup_ldr_sensor_reading

store_pir_sensor_reading:                            // pir_sensor_readings[pir_sensor_reading_number] = R0
    move r3, pir_sensor_readings
    move r1, pir_sensor_reading_number
    ld r1, r1, 0
    add r3, r3, r1
    st r0, r3, 0
    move r2, 1

setup_ldr_sensor_reading:
    move r1, 0

read_ldr_sensor:                                     // while stage_reg < samples:
    move r0, 0                                       //     R0 = read ldr sensor
    adc r0, 0, ldr_adc_channel                       //     R1 += R0
    add r1, r1, r0                                   //     stage_reg+=1
    stage_inc 1                                      //     wait 2+5 cycles
    wait 5                                           // stage_reg = 0
    jumps read_ldr_sensor, samples, lt
    stage_rst

store_ldr_sensor_reading:                            // ldr_sensor_readings[ldr_sensor_reading_number] = ldr reading
    rsh r1, r1, samples_log
    move r3, ldr_sensor_readings
    move r0, ldr_sensor_reading_number
    ld r0, r0, 0
    add r3, r3, r0
    st r1, r3, 0

check_wake_up_condition:
    move r0, r2
    jumpr wake_up_main_cpu, 1, ge

end:
    halt

wake_up_main_cpu:
    move r0, main_cpu_awake
    ld r0, r0, 0
    jumpr wake_up_main_cpu_return, 1, ge
    READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
    and r0, r0, 1
    jump wake_up_main_cpu, eq
    wake

wake_up_main_cpu_return:/*
    move r0, r2
    jumpr set_pir_sr_number, 1, ge
    jump setup_ldr_sensor_reading*/
    jump end