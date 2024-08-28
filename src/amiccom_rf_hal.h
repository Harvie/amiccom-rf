/** @file
 * @copyright
 * SPDX-FileCopyrightText:  2019-2020 Tomas Mudrunka <harvie@github>
 * SPDX-License-Identifier: GPL-2.0
 *
 * This file is where all platform dependent code belongs
 */

#pragma once

//Pin definition
#define PIN_RF_SDIO this->pin_rf_sdio
#define PIN_RF_SCK  this->pin_rf_sck
#define PIN_RF_SCS  this->pin_rf_scs
#define PIN_RF_GIO1 this->pin_rf_gio1 //== GIO1

#define DIR_IN	1
#define DIR_OUT 0

#define PULL_UP 1
#define PULL_NO 0

#define _nop_() asm("NOP")

#ifdef ESP_PLATFORM
#include <esp_attr.h>
#endif

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#ifdef MGOS_HAVE_MONGOOSE ////////////////////////////////////////////////////////////

//Mongoose GPIO API
#include <mgos_gpio.h>
#include <mgos_system.h>
#include <mgos_time.h>

#define delayms(ms) mgos_msleep((ms))
#define millis() ((unsigned long)(mgos_uptime_micros() / 1000))
//#define millis() (mgos_uptime() * 1000)

#define SET_GIO1(l)  mgos_gpio_write(PIN_RF_GIO1, (l))
#define SET_SDIO(l) mgos_gpio_write(PIN_RF_SDIO, (l))
#define SET_SCK(l)  mgos_gpio_write(PIN_RF_SCK, (l))
#define SET_SCS(l)  mgos_gpio_write(PIN_RF_SCS, (l))

#define GET_GIO1()  mgos_gpio_read(PIN_RF_GIO1)
#define GET_SDIO() mgos_gpio_read(PIN_RF_SDIO)
#define GET_SCK()  mgos_gpio_read(PIN_RF_SCK)
#define GET_SCS()  mgos_gpio_read(PIN_RF_SCS)

#define DIR_GIO1(d)  (d) ? mgos_gpio_set_mode(PIN_RF_GIO1, MGOS_GPIO_MODE_INPUT) : mgos_gpio_set_mode(PIN_RF_GIO1, MGOS_GPIO_MODE_OUTPUT)
#define DIR_SDIO(d) (d) ? mgos_gpio_set_mode(PIN_RF_SDIO, MGOS_GPIO_MODE_INPUT) : mgos_gpio_set_mode(PIN_RF_SDIO, MGOS_GPIO_MODE_OUTPUT)
#define DIR_SCK(d)  (d) ? mgos_gpio_set_mode(PIN_RF_SCK, MGOS_GPIO_MODE_INPUT) : mgos_gpio_set_mode(PIN_RF_SCK, MGOS_GPIO_MODE_OUTPUT)
#define DIR_SCS(d)  (d) ? mgos_gpio_set_mode(PIN_RF_SCS, MGOS_GPIO_MODE_INPUT) : mgos_gpio_set_mode(PIN_RF_SCS, MGOS_GPIO_MODE_OUTPUT)

#define PULL_GIO1(p)  (p) ? mgos_gpio_set_pull(PIN_RF_GIO1, MGOS_GPIO_PULL_UP) : mgos_gpio_set_pull(PIN_RF_GIO1, MGOS_GPIO_PULL_NONE)
#define PULL_SDIO(p) (p) ? mgos_gpio_set_pull(PIN_RF_SDIO, MGOS_GPIO_PULL_UP) : mgos_gpio_set_pull(PIN_RF_SDIO, MGOS_GPIO_PULL_NONE)
#define PULL_SCK(p)  (p) ? mgos_gpio_set_pull(PIN_RF_SCK, MGOS_GPIO_PULL_UP) : mgos_gpio_set_pull(PIN_RF_SCK, MGOS_GPIO_PULL_NONE)
#define PULL_SCS(p)  (p) ? mgos_gpio_set_pull(PIN_RF_SCS, MGOS_GPIO_PULL_UP) : mgos_gpio_set_pull(PIN_RF_SCS, MGOS_GPIO_PULL_NONE)


#elif AOS_KERNEL_H ////////////////////////////////////////////////////////////

//AliOS-Things
#include "aos/hal/gpio.h"

//TODO: add defines for AliOS-Things GPIO HAL

#elif ARDUINO ////////////////////////////////////////////////////////////

//Arduino
#include "Arduino.h"

#define delayms(ms) delay(ms)

#define SET_GIO1(l) digitalWrite(PIN_RF_GIO1, (l))
#define SET_SDIO(l) digitalWrite(PIN_RF_SDIO, (l))
#define SET_SCK(l)  digitalWrite(PIN_RF_SCK, (l))
#define SET_SCS(l)  digitalWrite(PIN_RF_SCS, (l))

#define GET_GIO1() digitalRead(PIN_RF_GIO1)
#define GET_SDIO() digitalRead(PIN_RF_SDIO)
#define GET_SCK()  digitalRead(PIN_RF_SCK)
#define GET_SCS()  digitalRead(PIN_RF_SCS)

#define DIR_GIO1(d) (d) ? pinMode((PIN_RF_GIO1), INPUT) : pinMode((PIN_RF_GIO1), OUTPUT)
#define DIR_SDIO(d) (d) ? pinMode(PIN_RF_SDIO, INPUT) : pinMode(PIN_RF_SDIO, OUTPUT)
#define DIR_SCK(d)  (d) ? pinMode(PIN_RF_SCK, INPUT) : pinMode(PIN_RF_SCK, OUTPUT)
#define DIR_SCS(d)  (d) ? pinMode(PIN_RF_SCS, INPUT) : pinMode(PIN_RF_SCS, OUTPUT)

#define PULL_GIO1(p) digitalWrite((PIN_RF_GIO1), (p))
#define PULL_SDIO(p) digitalWrite(PIN_RF_SDIO, (p))
#define PULL_SCK(p)  digitalWrite(PIN_RF_SCK, (p))
#define PULL_SCS(p)  digitalWrite(PIN_RF_SCS, (p))

#elif ESP_PLATFORM ////////////////////////////////////////////////////////////

//ESP32-IDF
#include "driver/gpio.h"

#define SET_GIO1(l) gpio_set_level(PIN_RF_GIO1, (l))
#define SET_SDIO(l) gpio_set_level(PIN_RF_SDIO, (l))
#define SET_SCK(l)  gpio_set_level(PIN_RF_SCK, (l))
#define SET_SCS(l)  gpio_set_level(PIN_RF_SCS, (l))

#define GET_GIO1() gpio_get_level(PIN_RF_GIO1)
#define GET_SDIO() gpio_get_level(PIN_RF_SDIO)
#define GET_SCK()  gpio_get_level(PIN_RF_SCK)
#define GET_SCS()  gpio_get_level(PIN_RF_SCS)

#define DIR_GIO1(d) gpio_set_direction(PIN_RF_GIO1, (d))
#define DIR_SDIO(d) gpio_set_direction(PIN_RF_SDIO, (d))
#define DIR_SCK(d)  gpio_set_direction(PIN_RF_SCK, (d))
#define DIR_SCS(d)  gpio_set_direction(PIN_RF_SCS, (d))

#define PULL_GIO1(p) gpio_set_pull_mode(PIN_RF_GIO1, (p))
#define PULL_SDIO(p) gpio_set_pull_mode(PIN_RF_SDIO, (p))
#define PULL_SCK(p)  gpio_set_pull_mode(PIN_RF_SCK, (p))
#define PULL_SCS(p)  gpio_set_pull_mode(PIN_RF_SCS, (p))

#else
	#error Trying to build Amiccom RF library on possibly unsupported platform

#endif
