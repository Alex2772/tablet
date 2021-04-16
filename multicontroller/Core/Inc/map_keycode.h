//
// Created by alex2 on 11.02.2021.
//

#ifndef TABLET_KEYBOARD_CONVERT_H
#define TABLET_KEYBOARD_CONVERT_H

#include <stdint.h>

#define KEY_CUSTOM_FN 0xff
uint8_t map_scancode_to_keycode(uint8_t scancode);

#endif //TABLET_KEYBOARD_CONVERT_H
