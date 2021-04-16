//
// Created by alex2 on 11.02.2021.
//

#include <main.h>
#include "map_keycode.h"
#include "usb_hid_keys.h"

uint8_t map_scancode_to_keycode(uint8_t scancode) {
    switch (scancode) {
        // first row
        case 64: return KEY_ESC;
        case 54: return KEY_F1;
        case 89: return KEY_F2;
        case 79: return KEY_F3;
        case 81: return KEY_F4;
        case 34: return KEY_F5;
        case 151: return KEY_F6;
        case 123: return KEY_F7;
        case 218: return KEY_F8;
        case 197: return KEY_F9;
        case 196: return KEY_F10;
        case 144: return KEY_F11;
        case 198: return KEY_F12;
        case 24: return is_fn_down() ? KEY_INSERT : KEY_SYSRQ;
        case 204: return is_fn_down() ? KEY_PAUSE : KEY_DELETE;

        // second row
        case 72: return KEY_GRAVE;
        case 71: return KEY_1;
        case 53: return KEY_2;
        case 88: return KEY_3;
        case 158: return KEY_4;
        case 159: return KEY_5;
        case 170: return KEY_6;
        case 169: return KEY_7;
        case 214: return KEY_8;
        case 213: return KEY_9;
        case 179: return KEY_0;
        case 180: return KEY_MINUS;
        case 117: return KEY_BACKSPACE;

        // third row
        case 62: return KEY_TAB;
        case 61: return KEY_Q;
        case 43: return KEY_W;
        case 78: return KEY_E;
        case 98: return KEY_R;
        case 113: return KEY_T;
        case 114: return KEY_Y;
        case 99: return KEY_U;
        case 109: return KEY_I;
        case 108: return KEY_O;
        case 100: return KEY_P;
        case 219: return KEY_EQUAL;
        case 222:
        case 131: return KEY_BACKSLASH;

        // fourth row
        case 44: return KEY_CAPSLOCK;
        case 63: return KEY_A;
        case 45: return KEY_S;
        case 80: return KEY_D;
        case 127: return KEY_F;
        case 140: return KEY_G;
        case 141: return KEY_H;
        case 128: return KEY_J;
        case 138: return KEY_K;
        case 137: return KEY_L;
        case 129: return KEY_SEMICOLON;
        case 199: return KEY_ENTER;

        // fifth row
        case 118: return KEY_LEFTSHIFT;
        case 74: return KEY_Z;
        case 56: return KEY_X;
        case 91: return KEY_C;
        case 161: return KEY_V;
        case 160: return KEY_B;
        case 171: return KEY_N;
        case 172: return KEY_M;
        case 226: return KEY_COMMA;
        case 225: return KEY_DOT;
        case 181: return KEY_SLASH;
        case 145: return is_fn_down() ? KEY_PAGEUP : KEY_UP;
        case 132: return KEY_RIGHTSHIFT;

        // sixth row
        case 35: return KEY_LEFTCTRL;
        case 130: return KEY_CUSTOM_FN;
        case 150: return KEY_LEFTMETA;
        case 143: return KEY_LEFTALT;
        case 188: return KEY_RIGHTCTRL;
        case 25: return KEY_SPACE;
        case 190: return KEY_RIGHTALT;
        case 115: return KEY_LEFTBRACE;
        case 124: return KEY_RIGHTBRACE;
        case 142: return KEY_APOSTROPHE;
        case 17: return is_fn_down() ? KEY_HOME : KEY_LEFT;
        case 205: return is_fn_down() ? KEY_PAGEDOWN : KEY_DOWN;
        case 16: return is_fn_down() ? KEY_END : KEY_RIGHT;


        default: return KEY_NONE;
    }
}