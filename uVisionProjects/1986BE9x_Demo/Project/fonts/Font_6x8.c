/**
  ******************************************************************************
  * @file    Font_6x8.c
  * @author  Phyton Application Team
  * @version V2.0.0
  * @date    10.09.2010
  * @brief   Font 6 x 8 pixels (normal). Analog of Terminal Microsoft Windows.
  ******************************************************************************
  * <br><br>
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, PHYTON SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 Phyton</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "font_defs.h"

/** @addtogroup __1986BE9x_Eval_Demo 1986BE9x Demonstration Example
  * @{
  */

/** @addtogroup Fonts Fonts
  * @{
  */

/** @defgroup Font_6x8 Fonts 6x8
  * @{
  */

/* The symbol representation has the following format:                      */
/* Every byte describes all columns of the symbol 8 upper lines.            */
/* Columns are represented from left to right.                              */
/* Lowest bit of a byte describes upper line of column,                     */
/* Highest - lower line.                                                    */
/* Then it's all repeated for all columns of lower 8 symbol lines.          */

static ucint8_t Font_6x8_Data[] = {
  /* 0x00 - Space.*/
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x01 - smiling face.*/
  0x00, 0x3e, 0x45, 0x51, 0x45, 0x3e,

  /* 0x02 - painted smiling face.*/
  0x00, 0x3e, 0x6b, 0x6f, 0x6b, 0x3e,

  /* 0x03 - hearts.*/
  0x00, 0x1c, 0x3e, 0x7c, 0x3e, 0x1c,

  /* 0x04 - diamonds.*/
  0x00, 0x18, 0x3c, 0x7e, 0x3c, 0x18,

  /* 0x05 - clubs.*/
  0x00, 0x30, 0x36, 0x7f, 0x36, 0x30,

  /* 0x06 - spades.*/
  0x00, 0x18, 0x5c, 0x7e, 0x5c, 0x18,

  /* 0x07 - filled circle at center.*/
  0x00, 0x00, 0x18, 0x18, 0x00, 0x00,

  /* 0x08 - inverted filled circle at center.*/
  0xff, 0xff, 0xe7, 0xe7, 0xff, 0xff,

  /* 0x09 - unfilled circle at center.*/
  0x00, 0x3c, 0x24, 0x24, 0x3c, 0x00,

  /* 0x0a - inverted unfilled circle at center (ring).*/
  0xff, 0xc3, 0xdb, 0xdb, 0xc3, 0xff,

  /* 0x0b - male symbol (circle with pointer up).*/
  0x00, 0x30, 0x48, 0x4a, 0x36, 0x0e,

  /* 0x0c - female symbol (circle with cross down).*/
  0x00, 0x06, 0x29, 0x79, 0x29, 0x06,

  /* 0x0d - note I.*/
  0x00, 0x60, 0x70, 0x3f, 0x02, 0x04,

  /* 0x0e - note II.*/
  0x00, 0x60, 0x7e, 0x0a, 0x35, 0x3f,

  /* 0x0f - sun (circle with outgoing rays).*/
  0x00, 0x2a, 0x1c, 0x36, 0x1c, 0x2a,

  /* 0x10 - thick arrow right.*/
  0x00, 0x00, 0x7f, 0x3e, 0x1c, 0x08,

  /* 0x11 - thick arrow lefts.*/
  0x00, 0x08, 0x1c, 0x3e, 0x7f, 0x00,

  /* 0x12 - thin arrow up-down.*/
  0x00, 0x14, 0x36, 0x7f, 0x36, 0x14,

  /* 0x13 - two exclamations.*/
  0x00, 0x00, 0x5f, 0x00, 0x5f, 0x00,

  /* 0x14 - "PI" symbol.*/
  0x00, 0x06, 0x09, 0x7f, 0x01, 0x7f,

  /* 0x15 - paragraph symbol.*/
  0x00, 0x22, 0x4d, 0x55, 0x59, 0x22,

  /* 0x16 - thick underline.*/
  0x00, 0x60, 0x60, 0x60, 0x60, 0x00,

  /* 0x17 - underlined thin arrow up-down.*/
  0x00, 0x14, 0xb6, 0xff, 0xb6, 0x14,

  /* 0x18 - thin arrow up.*/
  0x00, 0x04, 0x06, 0x7f, 0x06, 0x04,

  /* 0x19 - thin arrow down.*/
  0x00, 0x10, 0x30, 0x7f, 0x30, 0x10,

  /* 0x1a - thin arrow right.*/
  0x00, 0x08, 0x08, 0x3e, 0x1c, 0x08,

  /* 0x1b - thin arrow left.*/
  0x00, 0x08, 0x1c, 0x3e, 0x08, 0x08,

  /* 0x1c - indentation symbol.*/
  0x00, 0x78, 0x40, 0x40, 0x40, 0x40,

  /* 0x1d - thin arrow left-right.*/
  0x00, 0x08, 0x3e, 0x08, 0x3e, 0x08,

  /* 0x1e - thick arrow up.*/
  0x00, 0x30, 0x3c, 0x3f, 0x3c, 0x30,

  /* 0x1f - thick arrow down.*/
  0x00, 0x03, 0x0f, 0x3f, 0x0f, 0x03,

  /* 0x20 - space (empty place).*/
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x21 - excalmation.*/
  0x00, 0x00, 0x06, 0x5f, 0x06, 0x00,

  /* 0x22 - double quote.*/
  0x00, 0x07, 0x03, 0x00, 0x07, 0x03,

  /* 0x23 - number sign.*/
  0x00, 0x24, 0x7e, 0x24, 0x7e, 0x24,

  /* 0x24 - dollar.*/
  0x00, 0x24, 0x2b, 0x6a, 0x12, 0x00,

  /* 0x25 - percent.*/
  0x00, 0x63, 0x13, 0x08, 0x64, 0x63,

  /* 0x26 - ampersand.*/
  0x00, 0x36, 0x49, 0x56, 0x20, 0x50,

  /* 0x27 - apostrophe.*/
  0x00, 0x00, 0x07, 0x03, 0x00, 0x00,

  /* 0x28 - open bracket.*/
  0x00, 0x00, 0x3e, 0x41, 0x00, 0x00,

  /* 0x29 - close bracket.*/
  0x00, 0x00, 0x41, 0x3e, 0x00, 0x00,

  /* 0x2a - asterisk (multiplication).*/
  0x00, 0x08, 0x3e, 0x1c, 0x3e, 0x08,

  /* 0x2b - plus.*/
  0x00, 0x08, 0x08, 0x3e, 0x08, 0x08,

  /* 0x2c - comma.*/
  0x00, 0x00, 0xe0, 0x60, 0x00, 0x00,

  /* 0x2d - dash.*/
  0x00, 0x08, 0x08, 0x08, 0x08, 0x08,

  /* 0x2e - dot.*/
  0x00, 0x00, 0x60, 0x60, 0x00, 0x00,

  /* 0x2f - left-right slash ('/').*/
  0x00, 0x20, 0x10, 0x08, 0x04, 0x02,

  /* 0x30 - '0'.*/
  0x00, 0x3e, 0x41, 0x41, 0x41, 0x3e,

  /* 0x31 - '1'.*/
  0x00, 0x00, 0x42, 0x7f, 0x40, 0x00,

  /* 0x32 - '2'.*/
  0x00, 0x62, 0x51, 0x49, 0x49, 0x46,

  /* 0x33 - '3'.*/
  0x00, 0x22, 0x49, 0x49, 0x49, 0x36,

  /* 0x34 - '4'.*/
  0x00, 0x18, 0x14, 0x12, 0x7f, 0x10,

  /* 0x35 - '5'.*/
  0x00, 0x2f, 0x49, 0x49, 0x49, 0x31,

  /* 0x36 - '6'.*/
  0x00, 0x3c, 0x4a, 0x49, 0x49, 0x30,

  /* 0x37 - '7'.*/
  0x00, 0x01, 0x71, 0x09, 0x05, 0x03,

  /* 0x38 - '8'.*/
  0x00, 0x36, 0x49, 0x49, 0x49, 0x36,

  /* 0x39 - '9'.*/
  0x00, 0x06, 0x49, 0x49, 0x29, 0x1e,

  /* 0x3a - colon.*/
  0x00, 0x00, 0x6c, 0x6c, 0x00, 0x00,

  /* 0x3b - semicolon.*/
  0x00, 0x00, 0xec, 0x6c, 0x00, 0x00,

  /* 0x3c - less.*/
  0x00, 0x08, 0x14, 0x22, 0x41, 0x00,

  /* 0x3d - equal.*/
  0x00, 0x24, 0x24, 0x24, 0x24, 0x24,

  /* 0x3e - greater.*/
  0x00, 0x00, 0x41, 0x22, 0x14, 0x08,

  /* 0x3f - question-mark.*/
  0x00, 0x02, 0x01, 0x59, 0x09, 0x06,

  /* 0x40 - "dog" ('@').*/
  0x00, 0x3e, 0x41, 0x5d, 0x55, 0x1e,

  /* 0x41 - 'A'.*/
  0x00, 0x7e, 0x11, 0x11, 0x11, 0x7e,

  /* 0x42 - 'B'.*/
  0x00, 0x7f, 0x49, 0x49, 0x49, 0x36,

  /* 0x43 - 'C'.*/
  0x00, 0x3e, 0x41, 0x41, 0x41, 0x22,

  /* 0x44 - 'D'.*/
  0x00, 0x7f, 0x41, 0x41, 0x41, 0x3e,

  /* 0x45 - 'E'.*/
  0x00, 0x7f, 0x49, 0x49, 0x49, 0x41,

  /* 0x46 - 'F'.*/
  0x00, 0x7f, 0x09, 0x09, 0x09, 0x01,

  /* 0x47 - 'G'.*/
  0x00, 0x3e, 0x41, 0x49, 0x49, 0x7a,

  /* 0x48 - 'H'.*/
  0x00, 0x7f, 0x08, 0x08, 0x08, 0x7f,

  /* 0x49 - 'I'.*/
  0x00, 0x00, 0x41, 0x7f, 0x41, 0x00,

  /* 0x4a - 'J'.*/
  0x00, 0x30, 0x40, 0x40, 0x40, 0x3f,

  /* 0x4b - 'K'.*/
  0x00, 0x7f, 0x08, 0x14, 0x22, 0x41,

  /* 0x4c - 'L'.*/
  0x00, 0x7f, 0x40, 0x40, 0x40, 0x40,

  /* 0x4d - 'M'.*/
  0x00, 0x7f, 0x02, 0x04, 0x02, 0x7f,

  /* 0x4e - 'N'.*/
  0x00, 0x7f, 0x02, 0x04, 0x08, 0x7f,

  /* 0x4f - 'O'.*/
  0x00, 0x3e, 0x41, 0x41, 0x41, 0x3e,

  /* 0x50 - 'P'.*/
  0x00, 0x7f, 0x09, 0x09, 0x09, 0x06,

  /* 0x51 - 'Q'.*/
  0x00, 0x3e, 0x41, 0x51, 0x21, 0x5e,

  /* 0x52 - 'R'.*/
  0x00, 0x7f, 0x09, 0x09, 0x19, 0x66,

  /* 0x53 - 'S'.*/
  0x00, 0x26, 0x49, 0x49, 0x49, 0x32,

  /* 0x54 - 'T'.*/
  0x00, 0x01, 0x01, 0x7f, 0x01, 0x01,

  /* 0x55 - 'U'.*/
  0x00, 0x3f, 0x40, 0x40, 0x40, 0x3f,

  /* 0x56 - 'V'.*/
  0x00, 0x1f, 0x20, 0x40, 0x20, 0x1f,

  /* 0x57 - 'W'.*/
  0x00, 0x3f, 0x40, 0x3c, 0x40, 0x3f,

  /* 0x58 - 'X'.*/
  0x00, 0x63, 0x14, 0x08, 0x14, 0x63,

  /* 0x59 - 'Y'.*/
  0x00, 0x07, 0x08, 0x70, 0x08, 0x07,

  /* 0x5a - 'Z'.*/
  0x00, 0x71, 0x49, 0x45, 0x43, 0x00,

  /* 0x5b - '['.*/
  0x00, 0x00, 0x7f, 0x41, 0x41, 0x00,

  /* 0x5c - '\'.*/
  0x00, 0x02, 0x04, 0x08, 0x10, 0x20,

  /* 0x5d - ']'.*/
  0x00, 0x00, 0x41, 0x41, 0x7f, 0x00,

  /* 0x5e - '^'.*/
  0x00, 0x04, 0x02, 0x01, 0x02, 0x04,

  /* 0x5f - '_'.*/
  0x80, 0x80, 0x80, 0x80, 0x80, 0x80,

  /* 0x60 - back quote.*/
  0x00, 0x00, 0x03, 0x07, 0x00, 0x00,

  /* 0x61 - 'a'.*/
  0x00, 0x20, 0x54, 0x54, 0x54, 0x78,

  /* 0x62 - 'b'.*/
  0x00, 0x7f, 0x44, 0x44, 0x44, 0x38,

  /* 0x63 - 'c'.*/
  0x00, 0x38, 0x44, 0x44, 0x44, 0x28,

  /* 0x64 - 'd'.*/
  0x00, 0x38, 0x44, 0x44, 0x44, 0x7f,

  /* 0x65 - 'e'.*/
  0x00, 0x38, 0x54, 0x54, 0x54, 0x08,

  /* 0x66 - 'f'.*/
  0x00, 0x08, 0x7e, 0x09, 0x09, 0x00,

  /* 0x67 - 'g'.*/
  0x00, 0x18, 0xa4, 0xa4, 0xa4, 0x7c,

  /* 0x68 - 'h'.*/
  0x00, 0x7f, 0x04, 0x04, 0x78, 0x00,

  /* 0x69 - 'i'.*/
  0x00, 0x00, 0x00, 0x7d, 0x40, 0x00,

  /* 0x6a - 'j'.*/
  0x00, 0x40, 0x80, 0x84, 0x7d, 0x00,

  /* 0x6b - 'k'.*/
  0x00, 0x7f, 0x10, 0x28, 0x44, 0x00,

  /* 0x6c - 'l'.*/
  0x00, 0x00, 0x00, 0x7f, 0x40, 0x00,

  /* 0x6d - 'm'.*/
  0x00, 0x7c, 0x04, 0x18, 0x04, 0x78,

  /* 0x6e - 'n'.*/
  0x00, 0x7c, 0x04, 0x04, 0x78, 0x00,

  /* 0x6f - 'o'.*/
  0x00, 0x38, 0x44, 0x44, 0x44, 0x38,

  /* 0x70 - 'p'.*/
  0x00, 0xfc, 0x44, 0x44, 0x44, 0x38,

  /* 0x71 - 'q'.*/
  0x00, 0x38, 0x44, 0x44, 0x44, 0xfc,

  /* 0x72 - 'r'.*/
  0x00, 0x44, 0x78, 0x44, 0x04, 0x08,

  /* 0x73 - 's'.*/
  0x00, 0x08, 0x54, 0x54, 0x54, 0x20,

  /* 0x74 - 't'.*/
  0x00, 0x04, 0x3e, 0x44, 0x24, 0x00,

  /* 0x75 - 'u'.*/
  0x00, 0x3c, 0x40, 0x20, 0x7c, 0x00,

  /* 0x76 - 'v'.*/
  0x00, 0x1c, 0x20, 0x40, 0x20, 0x1c,

  /* 0x77 - 'w'.*/
  0x00, 0x3c, 0x60, 0x30, 0x60, 0x3c,

  /* 0x78 - 'x'.*/
  0x00, 0x6c, 0x10, 0x10, 0x6c, 0x00,

  /* 0x79 - 'y'.*/
  0x00, 0x9c, 0xa0, 0x60, 0x3c, 0x00,

  /* 0x7a - 'z'.*/
  0x00, 0x64, 0x54, 0x54, 0x4c, 0x00,

  /* 0x7b - '{'.*/
  0x00, 0x08, 0x3e, 0x41, 0x41, 0x00,

  /* 0x7c - '|'.*/
  0x00, 0x00, 0x00, 0x77, 0x00, 0x00,

  /* 0x7d - '}'.*/
  0x00, 0x00, 0x41, 0x41, 0x3e, 0x08,

  /* 0x7e - '~'.*/
  0x00, 0x02, 0x01, 0x02, 0x01, 0x00,

  /* 0x7f - "house".*/
  0x00, 0x3c, 0x26, 0x23, 0x26, 0x3c,

  /* 0x80 - net of points dispersed.*/
  0x44, 0x11, 0x44, 0x11, 0x44, 0x11,

  /* 0x81 - net of points condensed.*/
  0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55,

  /* 0x82 - net of lines.*/
  0xbb, 0xee, 0xbb, 0xee, 0xbb, 0xee,

  /* 0x83 - pseudo graphics - vertical line.*/
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00,

  /* 0x84 - pseudo graphics - vertical line with branch left from center.*/
  0x08, 0x08, 0x08, 0xff, 0x00, 0x00,

  /* 0x85 - pseudo graphics - vertical line with double branch left from center.*/
  0x0a, 0x0a, 0x0a, 0xff, 0x00, 0x00,

  /* 0x86 - pseudo graphics - double vertical line with branch left from center.*/
  0x08, 0xff, 0x00, 0xff, 0x00, 0x00,

  /* 0x87 - pseudo graphics - upper right corner with double vertical line.*/
  0x08, 0xf8, 0x08, 0xf8, 0x00, 0x00,

  /* 0x88 - pseudo graphics - upper right corner with double horizontal line.*/
  0x0a, 0x0a, 0x0a, 0xfe, 0x00, 0x00,

  /* 0x89 - pseudo graphics - double vertical line with double branch left from center.*/
  0x0a, 0xfb, 0x00, 0xff, 0x00, 0x00,

  /* 0x8a - pseudo graphics - double vertical line.*/
  0x00, 0xff, 0x00, 0xff, 0x00, 0x00,

  /* 0x8b - pseudo graphics - double upper right corner.*/
  0x0a, 0xfa, 0x02, 0xfe, 0x00, 0x00,

  /* 0x8c - pseudo graphics - double lower right corner.*/
  0x0a, 0x0b, 0x08, 0x0f, 0x00, 0x00,

  /* 0x8d - pseudo graphics - lower right corner with double vertical line.*/
  0x08, 0x0f, 0x08, 0x0f, 0x00, 0x00,

  /* 0x8e - pseudo graphics - lower right corner with double horizontal line.*/
  0x0a, 0x0a, 0x0a, 0x0f, 0x00, 0x00,

  /* 0x8f - pseudo graphics - upper right corner.*/
  0x08, 0x08, 0x08, 0xf8, 0x00, 0x00,

  /* 0x90 - pseudo graphics - lower left corner.*/
  0x00, 0x00, 0x00, 0x0f, 0x08, 0x08,

  /* 0x91 - pseudo graphics - horizontal line with branch up from center.*/
  0x08, 0x08, 0x08, 0x0f, 0x08, 0x08,

  /* 0x92 - pseudo graphics - horizontal line with branch down from center.*/
  0x08, 0x08, 0x08, 0xf8, 0x08, 0x08,

  /* 0x93 - pseudo graphics - vertical line with branch right from center.*/
  0x00, 0x00, 0x00, 0xff, 0x08, 0x08,

  /* 0x94 - pseudo graphics - horizontal line at the center.*/
  0x08, 0x08, 0x08, 0x08, 0x08, 0x08,

  /* 0x95 - pseudo graphics - cross.*/
  0x08, 0x08, 0x08, 0xff, 0x08, 0x08,

  /* 0x96 - pseudo graphics - vertical line with double branch right from center.*/
  0x00, 0x00, 0x00, 0xff, 0x0a, 0x0a,

  /* 0x97 - pseudo graphics - double vertical line with branch right from center.*/
  0x00, 0xff, 0x00, 0xff, 0x08, 0x08,

  /* 0x98 - pseudo graphics - double left lower corner.*/
  0x00, 0x0f, 0x08, 0x0b, 0x0a, 0x0a,

  /* 0x99 - pseudo graphics - double left upper corner.*/
  0x00, 0xfe, 0x02, 0xfa, 0x0a, 0x0a,

  /* 0x9a - pseudo graphics - double horizontal line with double branch up from center.*/
  0x0a, 0x0b, 0x08, 0x0b, 0x0a, 0x0a,

  /* 0x9b - pseudo graphics - double horizontal line with double branch down from center.*/
  0x0a, 0xfa, 0x02, 0xfa, 0x0a, 0x0a,

  /* 0x9c - pseudo graphics - double vertical line with double branch right from center.*/
  0x00, 0xff, 0x00, 0xfb, 0x0a, 0x0a,

  /* 0x9d - pseudo graphics - double horizontal line at the center.*/
  0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a,

  /* 0x9e - pseudo graphics - double cross.*/
  0x0a, 0xfb, 0x00, 0xfb, 0x0a, 0x0a,

  /* 0x9f - pseudo graphics - double horizontal line with branch up from center.*/
  0x0a, 0x0a, 0x0a, 0x0b, 0x0a, 0x0a,

  /* 0xa0 - pseudo graphics - horizontal line with double branch up from center.*/
  0x08, 0x0f, 0x08, 0x0f, 0x08, 0x08,

  /* 0xa1 - pseudo graphics - double horizontal line with branch down from center.*/
  0x0a, 0x0a, 0x0a, 0xfa, 0x0a, 0x0a,

  /* 0xa2 - pseudo graphics - horizontal line with double branch down from center.*/
  0x08, 0xf8, 0x08, 0xf8, 0x08, 0x08,

  /* 0xa3 - pseudo graphics - lower left corner, double vertical line.*/
  0x00, 0x0f, 0x08, 0x0f, 0x08, 0x08,

  /* 0xa4 - pseudo graphics - lower left corner, double horizontal line.*/
  0x00, 0x00, 0x00, 0x0f, 0x0a, 0x0a,

  /* 0xa5 - pseudo graphics - upper left corner, double horizontal line.*/
  0x00, 0x00, 0x00, 0xfe, 0x0a, 0x0a,

  /* 0xa6 - pseudo graphics - upper left corner, double vertical line.*/
  0x00, 0xf8, 0x08, 0xf8, 0x08, 0x08,

  /* 0xa7 - pseudo graphics - double vertical line at the center with branches left and right.*/
  0x08, 0xff, 0x08, 0xff, 0x08, 0x08,

  /* 0xa8 - Russian capital 'YO'.*/
  0x00, 0x7e, 0x4b, 0x4a, 0x4b, 0x42,

  /* 0xa9 - pseudo graphics - double horizontal line at the center with branches up and down.*/
  0x0a, 0x0a, 0x0a, 0xff, 0x0a, 0x0a,

  /* 0xaa - pseudo graphics - lower right corner.*/
  0x08, 0x08, 0x08, 0x0f, 0x00, 0x00,

  /* 0xab - pseudo graphics - upper left corner.*/
  0x00, 0x00, 0x00, 0xf8, 0x08, 0x08,

  /* 0xac - pseudo graphics - filled place.*/
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff,

  /* 0xad - pseudo graphics - filled lower half.*/
  0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0,

  /* 0xae - pseudo graphics - filled left half.*/
  0xff, 0xff, 0xff, 0x00, 0x00, 0x00,

  /* 0xaf - pseudo graphics - filled right half.*/
  0x00, 0x00, 0x00, 0xff, 0xff, 0xff,

  /* 0xb0 - pseudo graphics - filled upper half.*/
  0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,

  /* 0xb1 - hearts (not filled).*/
  0x00, 0x3e, 0x49, 0x49, 0x41, 0x22,

  /* 0xb2 - EX symbol.*/
  0x00, 0x38, 0x54, 0x54, 0x44, 0x28,

  /* 0xb3 - unfilled EX symbol.*/
  0x00, 0x01, 0x40, 0x7e, 0x40, 0x01,

  /* 0xb4 - Special symbol: Russian "l".*/
  0x00, 0x00, 0x01, 0x7c, 0x41, 0x00,

  /* 0xb5 - Special symbol: Russian "l/ch".*/
  0x00, 0x27, 0x48, 0x4b, 0x48, 0x3f,

  /* 0xb6 - 'y' with upper tilde ('~').*/
  0x00, 0x9d, 0xa2, 0x62, 0x3d, 0x00,

  /* 0xb7 - small circle up.*/
  0x00, 0x06, 0x09, 0x09, 0x06, 0x00,

  /* 0xb8 - Russian low 'yo'.*/
  0x00, 0x38, 0x55, 0x54, 0x55, 0x08,

  /* 0xb9 - large filled circle at the center.*/
  0x00, 0x00, 0x18, 0x18, 0x00, 0x00,

  /* 0xba - small filled circle at the center.*/
  0x00, 0x00, 0x08, 0x00, 0x00, 0x00,

  /* 0xbb - square root symbol.*/
  0x00, 0x30, 0x40, 0x3e, 0x02, 0x02,

  /* 0xbc - number sign.*/
  0x7f, 0x06, 0x18, 0x7f, 0x13, 0x13,

  /* 0xbd - "sun".*/
  0x2a, 0x3e, 0x14, 0x14, 0x3e, 0x2a,

  /* 0xbe - filled square at the center.*/
  0x00, 0x3c, 0x3c, 0x3c, 0x3c, 0x00,

  /* 0xbf - empty place.*/
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xc0-0xdf - Russian capital letters.*/
  /* 0xc0 */
  0x00, 0x7e, 0x11, 0x11, 0x11, 0x7e,

  /* 0xc1 */
  0x00, 0x7f, 0x49, 0x49, 0x49, 0x31,

  /* 0xc2 */
  0x00, 0x7f, 0x49, 0x49, 0x49, 0x36,

  /* 0xc3 */
  0x00, 0x7f, 0x01, 0x01, 0x01, 0x01,

  /* 0xc4 */
  0xc0, 0x7e, 0x41, 0x41, 0x7f, 0xc0,

  /* 0xc5 */
  0x00, 0x7f, 0x49, 0x49, 0x49, 0x41,

  /* 0xc6 */
  0x00, 0x77, 0x08, 0x7f, 0x08, 0x77,

  /* 0xc7 */
  0x22, 0x49, 0x49, 0x49, 0x36, 0x00,

  /* 0xc8 */
  0x00, 0x7f, 0x20, 0x10, 0x08, 0x7f,

  /* 0xc9 */
  0x00, 0x7e, 0x21, 0x11, 0x09, 0x7e,

  /* 0xca */
  0x00, 0x7f, 0x08, 0x14, 0x22, 0x41,

  /* 0xcb */
  0x00, 0x40, 0x7e, 0x01, 0x01, 0x7f,

  /* 0xcc */
  0x00, 0x7f, 0x02, 0x04, 0x02, 0x7f,

  /* 0xcd */
  0x00, 0x7f, 0x08, 0x08, 0x08, 0x7f,

  /* 0xce */
  0x00, 0x3e, 0x41, 0x41, 0x41, 0x3e,

  /* 0xcf */
  0x00, 0x7f, 0x01, 0x01, 0x01, 0x7f,

  /* 0xd0 */
  0x00, 0x7f, 0x09, 0x09, 0x09, 0x06,

  /* 0xd1 */
  0x00, 0x3e, 0x41, 0x41, 0x41, 0x22,

  /* 0xd2 */
  0x00, 0x01, 0x01, 0x7f, 0x01, 0x01,

  /* 0xd3 */
  0x00, 0x27, 0x48, 0x48, 0x48, 0x3f,

  /* 0xd4 */
  0x00, 0x0e, 0x11, 0x7f, 0x11, 0x0e,

  /* 0xd5 */
  0x00, 0x63, 0x14, 0x08, 0x14, 0x63,

  /* 0xd6 */
  0x00, 0x7f, 0x40, 0x40, 0x7f, 0xc0,

  /* 0xd7 */
  0x00, 0x07, 0x08, 0x08, 0x08, 0x7f,

  /* 0xd8 */
  0x00, 0x7f, 0x40, 0x7f, 0x40, 0x7f,

  /* 0xd9 */
  0x00, 0x7f, 0x40, 0x7f, 0x40, 0xff,

  /* 0xda */
  0x03, 0x01, 0x7f, 0x48, 0x48, 0x30,

  /* 0xdb */
  0x00, 0x7f, 0x48, 0x48, 0x30, 0x7f,

  /* 0xdc */
  0x00, 0x7f, 0x48, 0x48, 0x48, 0x30,

  /* 0xdd */
  0x00, 0x22, 0x41, 0x49, 0x49, 0x3e,

  /* 0xde */
  0x00, 0x7f, 0x08, 0x3e, 0x41, 0x3e,

  /* 0xdf */
  0x00, 0x66, 0x19, 0x09, 0x09, 0x7f,

  /* 0xe0-0xff - Russian low letters.*/
  /* 0xe0 */
  0x00, 0x20, 0x54, 0x54, 0x54, 0x78,

  /* 0xe1 */
  0x00, 0x3c, 0x4a, 0x4a, 0x4a, 0x31,

  /* 0xe2 */
  0x00, 0x7c, 0x54, 0x54, 0x54, 0x28,

  /* 0xe3 */
  0x00, 0x7c, 0x04, 0x04, 0x0c, 0x00,

  /* 0xe4 */
  0xc0, 0x78, 0x44, 0x44, 0x7c, 0xc0,

  /* 0xe5 */
  0x00, 0x38, 0x54, 0x54, 0x54, 0x08,

  /* 0xe6 */
  0x00, 0x6c, 0x10, 0x7c, 0x10, 0x6c,

  /* 0xe7 */
  0x00, 0x28, 0x44, 0x54, 0x54, 0x28,

  /* 0xe8 */
  0x00, 0x7c, 0x20, 0x10, 0x08, 0x7c,

  /* 0xe9 */
  0x00, 0x7c, 0x20, 0x12, 0x0a, 0x7c,

  /* 0xea */
  0x00, 0x7c, 0x10, 0x28, 0x44, 0x00,

  /* 0xeb */
  0x40, 0x38, 0x04, 0x04, 0x7c, 0x00,

  /* 0xec */
  0x00, 0x7c, 0x08, 0x10, 0x08, 0x7c,

  /* 0xed */
  0x00, 0x7c, 0x10, 0x10, 0x10, 0x7c,

  /* 0xee */
  0x00, 0x38, 0x44, 0x44, 0x44, 0x38,

  /* 0xef */
  0x00, 0x7c, 0x04, 0x04, 0x04, 0x7c,

  /* 0xf0 */
  0x00, 0xfc, 0x44, 0x44, 0x44, 0x38,

  /* 0xf1 */
  0x00, 0x38, 0x44, 0x44, 0x44, 0x28,

  /* 0xf2 */
  0x00, 0x04, 0x04, 0x7c, 0x04, 0x04,

  /* 0xf3 */
  0x00, 0x9c, 0xa0, 0x60, 0x3c, 0x00,

  /* 0xf4 */
  0x00, 0x18, 0x24, 0x7c, 0x24, 0x18,

  /* 0xf5 */
  0x00, 0x6c, 0x10, 0x10, 0x6c, 0x00,

  /* 0xf6 */
  0x00, 0x7c, 0x40, 0x40, 0x7c, 0xc0,

  /* 0xf7 */
  0x00, 0x0c, 0x10, 0x10, 0x10, 0x7c,

  /* 0xf8 */
  0x00, 0x7c, 0x40, 0x7c, 0x40, 0x7c,

  /* 0xf9 */
  0x00, 0x7c, 0x40, 0x7c, 0x40, 0xfc,

  /* 0xfa */
  0x0c, 0x04, 0x7c, 0x50, 0x50, 0x20,

  /* 0xfb */
  0x00, 0x7c, 0x50, 0x50, 0x20, 0x7c,

  /* 0xfc */
  0x00, 0x7c, 0x50, 0x50, 0x50, 0x20,

  /* 0xfd */
  0x00, 0x28, 0x44, 0x54, 0x54, 0x38,

  /* 0xfe */
  0x00, 0x7c, 0x10, 0x38, 0x44, 0x38,

  /* 0xff */
  0x00, 0x48, 0x34, 0x14, 0x14, 0x7c
};

sFONT Font_6x8 = {
  8,                    /* Symbol height, in pixels.*/
  6,                    /* Symbol width, in pixels.*/
  255,                  /* Symbol number in the font.*/
  &Font_6x8_Data[0]     /* Font description table address.*/
};

/** @} */ /* End of group Font_6x8 */

/** @} */ /* End of group Fonts */

/** @} */ /* End of group __1986BE9x_Eval_Demo */

/******************* (C) COPYRIGHT 2010 Phyton *********************************
*
* END OF FILE Font_6x8.c */
