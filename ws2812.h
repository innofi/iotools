// Copyright 2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once

#include "esp_err.h"

#define PULSE_PIN GPIO_NUM_18

void ws2812_init(int num_leds);
void ws2812_deinit( void );
esp_err_t ws2812_refresh( void );
bool ws2812_colortemp( int index, int temp, int bright, bool refresh );
void ws2812_color( int index, uint8_t red, uint8_t green, uint8_t blue, bool refresh );
void ws2812_bargraph( int length );