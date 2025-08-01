/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * @brief HID ps4 Input Report for Boot Interfaces
     *
     * @see B.1, p.60 of Device Class Definition for Human Interface Devices (HID) Version 1.11
     */
    typedef struct
    {
        uint8_t header;

        uint8_t left_joy_x;
        uint8_t left_joy_y;
        uint8_t right_joy_x;
        uint8_t right_joy_y;

        union
        {
            struct
            {
                uint8_t dpad : 4;
                uint8_t square : 1;
                uint8_t cross : 1;
                uint8_t circle : 1;
                uint8_t triangle : 1;
            };
            uint8_t val;
        } buttons_1;

        union
        {
            struct
            {
                uint8_t L1 : 1;
                uint8_t R1 : 1;
                uint8_t L2 : 1;
                uint8_t R2 : 1;
                uint8_t share : 1;
                uint8_t option : 1;
                uint8_t L3 : 1;
                uint8_t R3 : 1;
            };
            uint8_t val;
        } buttons_2;

        union
        {
            struct
            {
                uint8_t PS_b : 1;
                uint8_t Tpad : 1;
                uint8_t counter : 6;
            };
            uint8_t val;
        } buttons_3;

        uint8_t left_trigger;
        uint8_t right_trigger;

        uint8_t extra[54];

    } __attribute__((packed)) hid_ps4_input_report_boot_t;
    typedef struct
    {
        uint8_t header;

        uint8_t left_joy_x;
        uint8_t left_joy_y;
        uint8_t right_joy_x;
        uint8_t right_joy_y;

        union
        {
            struct
            {
                uint8_t dpad : 4;
                uint8_t square : 1;
                uint8_t cross : 1;
                uint8_t circle : 1;
                uint8_t triangle : 1;
            };
            uint8_t val;
        } buttons_1;

        union
        {
            struct
            {
                uint8_t L1 : 1;
                uint8_t R1 : 1;
                uint8_t L2 : 1;
                uint8_t R2 : 1;
                uint8_t share : 1;
                uint8_t option : 1;
                uint8_t L3 : 1;
                uint8_t R3 : 1;
            };
            uint8_t val;
        } buttons_2;

        union
        {
            struct
            {
                uint8_t PS_b : 1;
                uint8_t Tpad : 1;
                uint8_t counter : 6;
            };
            uint8_t val;
        } buttons_3;

        uint8_t left_trigger;
        uint8_t right_trigger;

    } __attribute__((packed)) ps4_msg_t;

    enum DPADEnum
{
  DPAD_UP = 0x0,
  DPAD_UP_RIGHT = 0x1,
  DPAD_RIGHT = 0x2,
  DPAD_RIGHT_DOWN = 0x3,
  DPAD_DOWN = 0x4,
  DPAD_DOWN_LEFT = 0x5,
  DPAD_LEFT = 0x6,
  DPAD_LEFT_UP = 0x7,
  DPAD_OFF = 0x8,
};

enum ButtonEnum {
        /*@{/
        /** Directional Pad Buttons - available on most controllers */
         UP = 0,
        RIGHT = 1,
        DOWN = 2,
        LEFT = 3,
        /*@}/

        /*@{/
        /** Playstation buttons */
        TRIANGLE,
        CIRCLE,
        CROSS,
        SQUARE,

        SELECT,
        START,

        L3,
        R3,
    };


#ifdef __cplusplus
}
#endif //__cplusplus
