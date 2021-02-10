/* Copyright 2015-2017 Jack Humbert
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H
#include "muse.h"

enum preonic_layers {
  _QWERTY,
  _GAMING,
  _LOWER,
  _RAISE,
  _ADJUST,
  _NUMPAD,
  _RIGHTWO
};



enum preonic_keycodes {
  QWERTY = SAFE_RANGE,
  GAMING,
  LOWER,
  RAISE,
  NUMPAD,
  BACKLIT,
  RHADLEY,
  RIGHTWO,
  KC_UPQM,
  KC_DNFS	
};


bool is_ctrl_shift_active = false;
uint16_t ctrl_shift_timer = 0;


void knob_code(uint16_t keycode) {
  uint16_t held_keycode_timer = timer_read();
  register_code(keycode);
  while (timer_elapsed(held_keycode_timer) < MEDIA_KEY_DELAY) {
      // no-op
  }
  unregister_code(keycode);
}

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
	
[_QWERTY] = LAYOUT_preonic_grid(
		KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_BSPC,
		KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    RSFT_T(KC_DEL),
		LCTL_T(KC_ESC),  KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,
		KC_LSFT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_UPQM, KC_ENT,
		C(KC_F), C(A(KC_DEL)), KC_LGUI, KC_LALT, LOWER,   KC_SPC,  RGB_TOG,  RAISE, MO(_RIGHTWO), KC_LEFT, KC_DNFS, KC_RGHT
	),

[_LOWER] = LAYOUT_preonic_grid(
		KC_TILD, KC_EXLM, KC_AT,   KC_HASH, KC_DLR,  KC_PERC, KC_CIRC, KC_AMPR, KC_ASTR, KC_LPRN, KC_RPRN, KC_BSPC,
		KC_TILD, KC_EXLM, KC_AT,   KC_HASH, KC_DLR,  KC_PERC, KC_CIRC, KC_AMPR, KC_ASTR, KC_LPRN, KC_RPRN, _______,
		KC_DEL, _______, _______,   C(KC_S),   C(KC_W),   _______,  _______,   KC_UNDS, KC_PLUS, KC_LCBR, KC_RCBR, KC_PIPE,
		_______, KC_WBAK, KC_WFWD, _______,   A(KC_F4),  _______,    TO(_NUMPAD), KC_MPLY, KC_MUTE, KC_HOME, KC_UP, KC_END,
		TO(_GAMING), _______, _______, _______, _______, _______,  _______, KC_MPRV, KC_MNXT, KC_LEFT, KC_DOWN, KC_RIGHT
	),

[_RAISE] = LAYOUT_preonic_grid(
		KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_BSPC,
		KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_DEL,
		RHADLEY, C(KC_X), C(KC_V), _______, C(KC_W),   _______,   _______,   KC_MINS, KC_EQL,  KC_LBRC, KC_RBRC, KC_BSLS,
		_______, KC_WBAK, KC_WFWD,  _______,   A(KC_F4),  _______,  _______, _______, _______, C(KC_PGUP), KC_PGUP, C(KC_PGDN),
		LSFT(KC_C), ______case KC_UPQM:
	  if (record ->event.pressed){
        if (get_mods() & MOD_BIT(KC_LSHIFT) || get_mods() & MOD_BIT(KC_RSHIFT)){
          register_code(KC_SLSH);
        } else {
          register_code(KC_UP);
        }
      } else {
        unregister_code(KC_SLSH);
        unregister_code(KC_UP);
      }
      return false;
      case KC_DNFS:
       if (record ->event.pressed){
              if (get_mods() & MOD_BIT(KC_LSHIFT) || get_mods() & MOD_BIT(KC_RSHIFT)){
          register_code(KC_PSLS);
        } else {
          register_code(KC_DOWN);
        }
      } else {
        unregister_code(KC_PSLS);
        unregister_code(KC_DOWN);
      }
      return false;_, _______, _______, _______, _______, _______, _______, _______, KC_WBAK, KC_PGDN, KC_WFWD
	),

[_RIGHTWO] = LAYOUT_preonic_grid(
        KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,
        KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,
        _______, C(KC_C), C(KC_V), C(S(KC_T)), C(KC_W), C(A(KC_T)), _______, _______, _______, _______, _______, RESET, 
		_______, LGUI(KC_UP), LGUI(KC_DOWN), LGUI(KC_H), A(KC_F4), _______, _______, KC_MFFD, KC_MRWD, C(KC_HOME), KC_UP, C(KC_END), 
		_______, RGB_HUI, RGB_SAI, RGB_VAI, _______, _______ ,_______ , KC_NO, _______, C(KC_LEFT), _______, C(KC_RIGHT)
	),
    
		[_GAMING] = LAYOUT_preonic_grid(
			KC_ESC, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, _______, 
			KC_NO, KC_NO ,KC_1, KC_2, KC_3, KC_4 , KC_5, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, 
			KC_NO, KC_TAB, KC_Q, KC_W, KC_NO , KC_E, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, 
			KC_NO, KC_I, KC_A, KC_S, KC_D, KC_F, KC_X, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, 
			TO(_QWERTY), KC_J, KC_NO, KC_NO,   KC_NO, KC_LALT, KC_SPC, KC_B, KC_C, KC_V, KC_NO, KC_NO 
		),
	
		[_NUMPAD] = LAYOUT_preonic_grid(
			KC_ESC, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_BSPC, 
			KC_TAB, KC_SLCK, KC_UP, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_P7, KC_P8, KC_P9, KC_PMNS, 
			KC_NLCK, KC_LEFT, KC_DOWN, KC_RIGHT, KC_NO, KC_NO, KC_NO, KC_NO, KC_P4, KC_P5, KC_P6, KC_PPLS, 
			KC_LSFT, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_P1, KC_P2, KC_P3, KC_SFTENT, 
			TO(_QWERTY), KC_NO, KC_NO, KC_NO, KC_NO, KC_SPC, KC_NO, KC_NO, KC_P0, KC_PDOT, KC_PDOT, KC_NO 
		),
    
		[_ADJUST] = LAYOUT_preonic_grid(
			KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,
			_______, RESET,   DEBUG,   _______, _______, _______, _______, TERM_ON, TERM_OFF,_______, _______, KC_DEL,
			_______, _______, MU_MOD,  AU_ON,   AU_OFF,  AG_NORM, AG_SWAP, QWERTY,  GAMING, _______,  _______, _______,
			_______, MUV_DE,  MUV_IN,  MU_ON,   MU_OFF,  MI_ON,   MI_OFF,  _______, _______, _______, RGB_MOD, _______,
			_______, _______, _______, _______, _______, _______, _______, _______, _______, RGB_HUI, RGB_SAI, RGB_VAI
		),

};

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
	  case KC_UPQM:
	  if (record ->event.pressed){
        if (get_mods() & MOD_BIT(KC_LSHIFT) || get_mods() & MOD_BIT(KC_RSHIFT)){
          register_code(KC_SLSH);
        } else {
          register_code(KC_UP);
        }
      } else {
        unregister_code(KC_SLSH);
        unregister_code(KC_UP);
      }
      return false;
      case KC_DNFS:
       if (record ->event.pressed){
              if (get_mods() & MOD_BIT(KC_LSHIFT) || get_mods() & MOD_BIT(KC_RSHIFT)){
          register_code(KC_PSLS);
        } else {
          register_code(KC_DOWN);
        }
      } else {
        unregister_code(KC_PSLS);
        unregister_code(KC_DOWN);
      }
      return false;
        case QWERTY:
          if (record->event.pressed) {
            set_single_persistent_default_layer(_QWERTY);
          }
          return false;
          break;
		case GAMING:
		  if (record->event.pressed) {
          layer_on(_GAMING);
          set_single_persistent_default_layer(_GAMING);
          }
		  return false;
		  break;
        case LOWER:
          if (record->event.pressed) {
            layer_on(_LOWER);
            update_tri_layer(_LOWER, _RAISE, _ADJUST);
          } else {
            layer_off(_LOWER);
            update_tri_layer(_LOWER, _RAISE, _ADJUST);
          }
          return false;
          break;
        case RAISE:
          if (record->event.pressed) {
            layer_on(_RAISE);
            update_tri_layer(_LOWER, _RAISE, _ADJUST);
          } else {
            layer_off(_RAISE);
            update_tri_layer(_LOWER, _RAISE, _ADJUST);
          }
          return false;
          break;
		case RIGHTWO:
          if (record->event.pressed) {
            layer_on(_RIGHTWO);
          } else {
            layer_off(_RIGHTWO);
          }
          return false;
          break;
        case BACKLIT:
          if (record->event.pressed) {
            register_code(KC_RSFT);
            #ifdef BACKLIGHT_ENABLE
              backlight_step();
            #endif
            #ifdef __AVR__
            writePinLow(E6);
            #endif
          } else {
            unregister_code(KC_RSFT);
            #ifdef __AVR__
            writePinHigh(E6);
            #endif
          }
          return false;
          break;
		case RHADLEY:
			if (record->event.pressed) {
			SEND_STRING("%>%");
			} else {
			}
			break;
      }
    return true;
};

bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

void encoder_update_user(uint8_t index, bool clockwise) {
  if (muse_mode) {
    if (IS_LAYER_ON(_RAISE)) {
      if (clockwise) {
        muse_offset++;
      } else {
        muse_offset--;
      }
    } else {
      if (clockwise) {
        muse_tempo+=1;
      } else {
        muse_tempo-=	1;		
      }
    }
  } else {
	  switch(biton32(layer_state)) {
		  case 6:
			if (clockwise) {
			   tap_code(KC_PGUP);  
			}  else {
			   tap_code(KC_PGDN);
			}
			case 2:
			if (clockwise) {
			   tap_code16(C(KC_Z));3 
			   break;
			}  else {
			   tap_code16(C(KC_Y));
			   break;
			}
			case 3:
			if (clockwise) {
			   tap_code16(S(KC_TAB));
			   break;
			 } else {
			   tap_code16(KC_TAB);
			   break;
			 }
			case 0:
            if (clockwise) {
               tap_code16(S(KC_F3));
            break;
            } else {
              tap_code16(KC_F3);
            break;
        }
	  }	
}
}

void dip_switch_update_user(uint8_t index, bool active) {
    switch (index) {
        case 0:
            if (active) {
                layer_on(_ADJUST);
            } else {
                layer_off(_ADJUST);
            }
            break;
        case 1:
            if (active) {
                muse_mode = true;
            } else {
                muse_mode = false;
            }
    }
}


void matrix_scan_user(void) {
#ifdef AUDIO_ENABLE
    if (muse_mode) {
        if (muse_counter == 0) {
            uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
            if (muse_note != last_muse_note) {
                stop_note(compute_freq_for_midi_note(last_muse_note));
                play_note(compute_freq_for_midi_note(muse_note), 0xF);
                last_muse_note = muse_note;
            }
        }
        muse_counter = (muse_counter + 1) % muse_tempo;
    } else {
        if (muse_counter) {
            stop_all_notes();
            muse_counter = 0;
        }
    }

#endif
}


bool music_mask_user(uint16_t keycode) {
  switch (keycode) {
    case RAISE:
    case LOWER:
      return false;
    default:
      return true;
  }
}
