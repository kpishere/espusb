#ifndef KeyboardTerminal122
#define KeyboardTerminal122

#include "usb_hid_112.h"

// State / mod key codes for Model M 122 key scancode 3 keyboard
#define term122_resetQuit 0x11
#define term122_shiftL 0x12
#define term122_shiftLck 0x14
#define term122_altL 0x19
#define term122_altR 0x39
#define term122_ctrlEnter 0x58
#define term122_shiftR 0x59

// States and mapping
typedef enum ps2kbStatesE {
  ps2none = 0x00, shiftLck = 0x01, shiftL = 0x02, resetQuit = 0x04, altL = 0x08, shiftR = 0x10, ctrlEnter = 0x20, altR = 0x40, RESERVED = 0x80
} ps2kbStates;

// Terminal 122 Key Codes + key-state (P/S2 Set 3) to USB + modifier
#define PS2Scancode3ToUSB112 \
  {{0x00,0}, {0,Reserved0}} /* Used for key up event */ \
, {{0x01,0}, {0,Application}} \
, {{0x01,0}, {0,Application}} /*Setup */ \
, {{0x03,0}, {0,PrintScreen}} /*Print*/ \
, {{0x04,0}, {0,Power}} /*Blank below SysReq, above Print*/ \
, {{0x05,0}, {0,SysReq_Attn}} \
, {{0x06,0}, {0,Escape}} /*Blank Clear - to right of SysReq*/\
, {{0x06, (int)altL}, {0,Clear}} /* Clear - to right of SysReq*/ \
, {{0x06, (int)altR}, {0,Clear}} /* Clear - to right of SysReq*/ \
, {{0x07,0}, {0,F1}} \
, {{0x08,0}, {0,F13}} \
, {{0x09,0}, {0,Mute}} /*Record Pause*/ \
, {{0x09, (int)altL}, {0,Pause}} /*Record Pause*/ \
, {{0x09, (int)altR}, {0,Pause}} /*Record Pause*/ \
, {{0x0A,0}, {0,Execute}} /*Play test*/ \
, {{0x0B,0}, {0,Help}} /* Help hex*/ \
, {{0x0B, (int)altL}, {0,KP_Hex}} /* Help hex*/ \
, {{0x0B, (int)altR}, {0,KP_Hex}} /* Help hex*/ \
, {{0x0C,0}, {0,Clear_Again}} /*ErrImp*/ \
, {{0x0D,0}, {0,Tab}} \
, {{0x0E,0}, {0,Acent_Tilde}} /* ' _ */ \
, {{0x0F,0}, {0,F2}} \
, {{0x10,0}, {0,F14}} \
, {{term122_resetQuit,0}, {0,ModLeftCTRL}} /*resetQuit*/ \
, {{term122_shiftL,0}, {0,ModLeftShift}} /*shiftL*/ \
, {{0x13,0}, {0,KP_LessThan}} /* < > */ \
, {{0x13, (int)shiftL}, {0,KP_GreaterThan}} /* < > */ \
, {{0x13, (int)shiftR}, {0,KP_GreaterThan}} /* < > */ \
, {{0x13, (int)shiftLck}, {0,KP_GreaterThan}} /* < > */ \
, {{term122_shiftLck,0}, {0,CapsLock}} /*shiftLck*/ \
, {{0x15,0}, {0,Q}} \
, {{0x16,0}, {0,One_Bang}} \
, {{0x17,0}, {0,F3}} \
, {{0x18,0}, {0,F15}} \
, {{term122_altL,0}, {0,ModLeftAlt}} /*altL*/ \
, {{0x1A,0}, {0,Z}} \
, {{0x1B,0}, {0,S}} \
, {{0x1C,0}, {0,A}} \
, {{0x1D,0}, {0,W}} \
, {{0x1E,0}, {0,Two_At}} \
, {{0x1F,0}, {0,F4}} \
, {{0x20,0}, {0,F16}} \
, {{0x21,0}, {0,C}} \
, {{0x22,0}, {0,X}} \
, {{0x23,0}, {0,D}} \
, {{0x24,0}, {0,E}} \
, {{0x25,0}, {0,Four_Dollar}} \
, {{0x26,0}, {0,Three_Hash}} \
, {{0x27,0}, {0,F5}} \
, {{0x28,0}, {0,F17}} \
, {{0x29,0}, {0,Spacebar}} \
, {{0x2A,0}, {0,V}} \
, {{0x2B,0}, {0,F}} \
, {{0x2C,0}, {0,T}} \
, {{0x2D,0}, {0,R}} \
, {{0x2E,0}, {0,Five_Perc}} \
, {{0x2F,0}, {0,F6}} \
, {{0x30,0}, {0,F18}} \
, {{0x31,0}, {0,N}} \
, {{0x32,0}, {0,B}} \
, {{0x33,0}, {0,H}} \
, {{0x34,0}, {0,G}} \
, {{0x35,0}, {0,Y}} \
, {{0x36,0}, {0,Six_Caret}} \
, {{0x37,0}, {0,F7}} \
, {{0x38,0}, {0,F19}} \
, {{term122_altR,0}, {0,ModRightAlt}} /* altR */ \
, {{0x3A,0}, {0,M}} \
, {{0x3B,0}, {0,J}} \
, {{0x3C,0}, {0,U}} \
, {{0x3D,0}, {0,Seven_And}} \
, {{0x3E,0}, {0,Eight_Asrisk}} \
, {{0x3F,0}, {0,F8}} \
, {{0x40,0}, {0,F20}} \
, {{0x41,0}, {0,KP_Comma}} \
, {{0x42,0}, {0,K}} \
, {{0x43,0}, {0,I}} \
, {{0x44,0}, {0,O}} \
, {{0x45,0}, {0,Zero_CloseParen}} \
, {{0x46,0}, {0,Nine_OpenParen}} \
, {{0x47,0}, {0,F9}} \
, {{0x48,0}, {0,F21}} \
, {{0x49,0}, {0,Period_Greater}} \
, {{0x49, (int)shiftL}, {0,Period_Greater}} /* < > */ \
, {{0x49, (int)shiftR}, {0,Period_Greater}} /* < > */ \
, {{0x49, (int)shiftLck}, {0,Period_Greater}} /* < > */ \
, {{0x4A,0}, {0,FwdSlash_Question}} \
, {{0x4B,0}, {0,L}} \
, {{0x4C,0}, {0,ColonSemi_Full}} \
, {{0x4D,0}, {0,P}} \
, {{0x4E,0}, {0,Minus_Underscore}} \
, {{0x4F,0}, {0,F10}} \
, {{0x50,0}, {0,F22}} \
, {{0x52,0}, {0,QuoteSingle_Double}} \
, {{0x53,0}, {leftShift,BraceOpenSq_Curl}} \
, {{0x53, (int)shiftL}, {leftShift, BraceCloseSq_Curl}} /*{ }*/ \
, {{0x53, (int)shiftR}, {leftShift, BraceCloseSq_Curl}} /*{ }*/ \
, {{0x53, (int)shiftLck}, {leftShift, BraceCloseSq_Curl}} /*{ }*/ \
, {{0x54,0}, {0,CurrencySubUnit}} /* ! c */ \
, {{0x54, (int)shiftL}, {0,KP_Bang}} /* ! c */ \
, {{0x54, (int)shiftR}, {0,KP_Bang}} /* ! c */ \
, {{0x54, (int)shiftLck}, {0,KP_Bang}} /* ! c */ \
, {{0x55,0}, {0,Equal_Plus}} \
, {{0x56,0}, {0,F11}} \
, {{0x57,0}, {0,F23}} \
, {{term122_ctrlEnter,0}, {0,ModRightCTRL}} /*ctrlEnter*/ \
, {{term122_shiftR,0}, {0,ModRightShift}} /* shiftR */ \
, {{0x5A,0}, {0,Return}} /* Field Exit */ \
, {{0x5B,0}, {0,BackSlash_Pipe}} \
, {{0x5E,0}, {0,F12}} \
, {{0x5F,0}, {0,F24}} \
, {{0x60,0}, {0,DownArrow}} \
, {{0x61,0}, {0,LeftArrow}} \
, {{0x62,0}, {0,Home}} \
, {{0x63,0}, {0,UpArrow}} \
, {{0x64,0}, {0,ReturnAlt}} \
, {{0x65,0}, {0,Paste}} /*Insert*/ \
, {{0x66,0}, {0,Delete_Backspace}} \
, {{0x67,0}, {leftShift, Tab }} \
, {{0x69,0}, {0,KP_1_End}} \
, {{0x6A,0}, {0,RightArrow}} \
, {{0x6B,0}, {0,KP_4_Left}} \
, {{0x6C,0}, {0,KP_7_Home}} \
, {{0x6D,0}, {0,DeleteFwd}} \
, {{0x6E,0}, {0,Copy}} \
, {{0x6F,0}, {0,Undo}} /*Blank Jump*/ \
, {{0x6F, (int)altR}, {0,Find}} \
, {{0x6F, (int)altL}, {0,Find}} \
, {{0x70,0}, {0,KP_0_Ins}} \
, {{0x71,0}, {0,KP_Dot_Del}} \
, {{0x72,0}, {0,KP_2_Down}} \
, {{0x73,0}, {0,KP_5}} \
, {{0x74,0}, {0,KP_6_Right}} \
, {{0x75,0}, {0,KP_8_Up}} \
, {{0x76,0}, {0,BraceOpenSq_Curl}} \
, {{0x77,0}, {0,BraceCloseSq_Curl}} \
, {{0x79,0}, {0,KP_Enter}} \
, {{0x7A,0}, {0,KP_3_PgDwn}} \
, {{0x7B,0}, {0,KP_Plus}} \
, {{0x7C,0}, {0,KP_Minus}} \
, {{0x7D,0}, {0,KP_9_PgUp}} \
, {{0x7E,0}, {0,KP_SlashFwd}} \
, {{0x83,0}, {0,Stop}} \
, {{0x84,0}, {0,KP_Mult}}

// With primary keycode press, call handler
#define PS2KeyEvent2Handler \
  {0x00, keyMap} \
, {0x01, keyMap} \
, {0x03, keyMap} \
, {0x04, keyMap} \
, {0x05, keyMap} \
, {0x06, keyMap} \
, {0x07, keyMap} \
, {0x08, keyMap} \
, {0x09, keyMap} \
, {0x0A, keyMap} \
, {0x0B, keyMap} \
, {0x0C, keyMap} \
, {0x0D, keyMap} \
, {0x0E, keyMap} \
, {0x0F, keyMap} \
, {0x10, keyMap} \
, {term122_shiftR, inKeyState} \
, {term122_shiftL, inKeyState} \
, {0x13, keyMap} \
, {term122_shiftLck, inKeyState} \
, {0x15, keyMap} \
, {0x16, keyMap} \
, {0x17, keyMap} \
, {0x18, keyMap} \
, {term122_altL, inKeyState} \
, {0x1A, keyMap} \
, {0x1B, keyMap} \
, {0x1C, keyMap} \
, {0x1D, keyMap} \
, {0x1E, keyMap} \
, {0x1F, keyMap} \
, {0x20, keyMap} \
, {0x21, keyMap} \
, {0x22, keyMap} \
, {0x23, keyMap} \
, {0x24, keyMap} \
, {0x25, keyMap} \
, {0x26, keyMap} \
, {0x27, keyMap} \
, {0x28, keyMap} \
, {0x29, keyMap} \
, {0x2A, keyMap} \
, {0x2B, keyMap} \
, {0x2C, keyMap} \
, {0x2D, keyMap} \
, {0x2E, keyMap} \
, {0x2F, keyMap} \
, {0x30, keyMap} \
, {0x31, keyMap} \
, {0x32, keyMap} \
, {0x33, keyMap} \
, {0x34, keyMap} \
, {0x35, keyMap} \
, {0x36, keyMap} \
, {0x37, keyMap} \
, {0x38, keyMap} \
, {term122_altR, inKeyState} \
, {0x3A, keyMap} \
, {0x3B, keyMap} \
, {0x3C, keyMap} \
, {0x3D, keyMap} \
, {0x3E, keyMap} \
, {0x3F, keyMap} \
, {0x40, keyMap} \
, {0x41, keyMap} \
, {0x42, keyMap} \
, {0x43, keyMap} \
, {0x44, keyMap} \
, {0x45, keyMap} \
, {0x46, keyMap} \
, {0x47, keyMap} \
, {0x48, keyMap} \
, {0x49, keyMap} \
, {0x4A, keyMap} \
, {0x4B, keyMap} \
, {0x4C, keyMap} \
, {0x4D, keyMap} \
, {0x4E, keyMap} \
, {0x4F, keyMap} \
, {0x50, keyMap} \
, {0x52, keyMap} \
, {0x53, keyMap} \
, {0x54, keyMap} \
, {0x55, keyMap} \
, {0x56, keyMap} \
, {0x57, keyMap} \
, {term122_ctrlEnter, inKeyState} \
, {term122_shiftR, inKeyState} \
, {0x5A, keyMap} \
, {0x5B, keyMap} \
, {0x5E, keyMap} \
, {0x5F, keyMap} \
, {0x60, keyMap} \
, {0x61, keyMap} \
, {0x62, keyMap} \
, {0x63, keyMap} \
, {0x64, keyMap} \
, {0x65, keyMap} \
, {0x66, keyMap} \
, {0x67, keyMap} \
, {0x69, keyMap} \
, {0x6A, keyMap} \
, {0x6B, keyMap} \
, {0x6C, keyMap} \
, {0x6D, keyMap} \
, {0x6E, keyMap} \
, {0x6F, keyMap} \
, {0x70, keyMap} \
, {0x71, keyMap} \
, {0x72, keyMap} \
, {0x73, keyMap} \
, {0x74, keyMap} \
, {0x75, keyMap} \
, {0x76, keyMap} \
, {0x77, keyMap} \
, {0x79, keyMap} \
, {0x7A, keyMap} \
, {0x7B, keyMap} \
, {0x7C, keyMap} \
, {0x7D, keyMap} \
, {0x7E, keyMap} \
, {0x83, keyMap} \
, {0x84, keyMap} \
, {0xF0, outStateKeyCode}

typedef struct {
  unsigned char key;
  int states;
} ps2key;
typedef void (*keyHandler)(ps2key val);

extern bool nextKeyIsOut;
extern int ps2kbState;

keyHandler keyHandler_map_find(unsigned char key  );

#endif
