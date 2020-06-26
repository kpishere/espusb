#include <esp82xxutil.h>
#include <gpio.h>
#include <common.h>
#include <string.h>

#include "keyboard_terminal_122.h"

typedef struct {
  unsigned char key;
  keyHandler handlerFunc;
} keyHandler_map_t;

typedef struct {
  ps2key key;
  usbKey usbKey;
} usbKey_map_t;

extern int keybt;
extern int keymod;
extern int keypress;

// Constants
const usbKey usbKeyUndefined = {0, Reserved0};

// Globals
int ps2kbState;
bool nextKeyIsOut = false;

void sendUSBModState(int mod) {
  sendUSBKey( ((usbKey){mod, Reserved0}) );
}

int usbModFromPS2State(int ps2State) {
  int mod = (int)none;
  // If LHS and RHS state and modifier conditions are blank, then pass-thru value
  if( ps2State & shiftLck ) mod |= leftShift;
  if( ps2State & shiftL ) mod |= leftShift;
  if( ps2State & resetQuit ) mod |= leftCtrl;
  if( ps2State & altL ) mod |= leftAlt;
  if( ps2State & shiftR ) mod |= rightShift;
  if( ps2State & ctrlEnter ) mod |= rightCtrl;
  if( ps2State & altR ) mod |= rightAlt;
  // unmapped: , leftGUI   = 0x08, rightGUI  = 0x80
  return mod;
}

// Apply state key event to bitfield status
void inKeyState(ps2key key) {
  switch( key.key ) {
    case term122_resetQuit:
      ps2kbState |= resetQuit;
      break;
    case term122_shiftL:
      ps2kbState |= shiftL;
      break;
    case term122_shiftLck:
      ps2kbState |= shiftLck;
      break;
    case term122_altL:
      ps2kbState |= altL;
      break;
    case term122_altR:
      ps2kbState |= altR;
      break;
    case term122_ctrlEnter:
      ps2kbState |= ctrlEnter;
      break;
    case term122_shiftR:
      ps2kbState |= shiftR;
      break;
  }
  sendUSBModState(usbModFromPS2State(ps2kbState));
}

usbKey usbKey_map_find(ps2key key) {
  static usbKey_map_t mappedKeyUsb112[] = { PS2Scancode3ToUSB112 };
  int i;
  usbKey val = usbKeyUndefined;
  for( i = 0; i < sizeof(mappedKeyUsb112) / sizeof(mappedKeyUsb112[0]); i++) {
    if ( mappedKeyUsb112[i].key.key == key.key && mappedKeyUsb112[i].key.states == key.states ) {
        val = mappedKeyUsb112[i].usbKey;
        break;
    }
  }
  return val;
};

void keyMap(ps2key key) {
  usbKey valUsb = {(int)none, ErrUndefined};
	usbKey valUsb_m = usbKeyUndefined;
  valUsb_m = usbKey_map_find(key);

  if( !compare_usbKey(valUsb_m,usbKeyUndefined) ) { // Explicit map only
    valUsb = valUsb_m;
  } else { // Pass-thru of state to modifier
    ps2key keyNoState = {key.key, ps2none};
		valUsb_m = usbKey_map_find(keyNoState);
    if( !compare_usbKey(valUsb_m,usbKeyUndefined) ) {
      valUsb = valUsb_m;
      if(valUsb.mod == (int)none) {
        valUsb.mod = usbModFromPS2State( key.states );
      }
    }
  }
  sendUSBKey(valUsb);
}

void outStateKeyCode(ps2key key) {
  nextKeyIsOut = true;
}
// Clear state of key up event
void outKeyState(unsigned char key) {
  nextKeyIsOut = false;
  switch( key ) {
    case (term122_resetQuit):
    case (term122_resetQuit | 0xF0): // 0xF1
      ps2kbState &= ~resetQuit;
      break;
    case (term122_shiftL):
    case (term122_shiftL | 0xF0): // 0xF2
      ps2kbState &= ~shiftL;
      break;
    case (term122_shiftLck):
    case (term122_shiftLck | 0xF0): // 0xF4
      ps2kbState &= ~shiftLck;
      break;
    case (term122_ctrlEnter):
    case (term122_ctrlEnter | 0xF0): // 0xF8
      ps2kbState &= ~ctrlEnter;
      break;
    case (term122_altL):
      ps2kbState &= ~(altL);
      break;
    case (term122_altR):
      ps2kbState &= ~(altR);
      break;
    case (term122_shiftR):
      ps2kbState &= ~(shiftR);
      break;
    case (term122_altL | 0xF0): // 0xF9 case (term122_altR | 0xF0): // 0xF9 case (term122_shiftR | 0xF0): // 0xF9
      ps2kbState &= ~(altL | altR | shiftR );
      break;
  }
  sendUSBModState(usbModFromPS2State(ps2kbState));
}


keyHandler keyHandler_map_find(unsigned char key  ) {
  static keyHandler_map_t mappedKeys[] = { PS2KeyEvent2Handler };
  int i;
  keyHandler val = NULL;
  for( i = 0; i < sizeof(mappedKeys) / sizeof(mappedKeys[0]); i++) {
    if ( mappedKeys[i].key == key ) {
        val = mappedKeys[i].handlerFunc;
        break;
    }
  }
  return val;
};
