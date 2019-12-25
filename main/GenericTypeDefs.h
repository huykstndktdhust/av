/*********************************************************************
 *
 *                  Generic Type Definitions
 *
 *********************************************************************
 * FileName:        GenericTypeDefs.h
 * Dependencies:    None
 * Processor:       PIC18/PIC24F
 * Complier:        MCC18 v1.00.50 or higher, C30 3.10 or higher
 *                  HITECH PICC-18 V8.10PL1 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the �Company�) for its PICmicro� Microcontroller is intended and
 * supplied to you, the Company�s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 ********************************************************************/

/*
*   Final editors: SangTN@fsoft.com.vn
*   @date:         07-30-2019
*/

#ifndef __GENERIC_TYPE_DEFS_H
  #define __GENERIC_TYPE_DEFS_H
  #ifdef __cplusplus
    extern "C" {
  #endif
      #include "tracking_main.h"

      typedef  int8_t   vi8;
      typedef  uint8_t  vu8;
      typedef  int16_t  vi16;
      typedef  uint16_t vu16;
      typedef  int32_t  vi32;
      typedef  uint32_t vu32;
      typedef  int64_t  vi64;
      typedef  uint64_t vu64;
      typedef int8_t   i8;
      typedef uint8_t  u8;
      typedef int16_t  i16;
      typedef uint16_t u16;
      typedef int32_t  i32;
      typedef uint32_t u32;
      typedef int64_t  i64;
      typedef uint64_t u64;

      /*defines structures of BYTE, WORD, DWORD, Q_WORD*/

      //typedef enum _BOOL { FALSE = 0, TRUE } BOOL;
      typedef  int8_t  iByte;    // signed 8-bit
      typedef  uint8_t  Byte;    // 8-bit
      typedef  uint16_t Word;    // 16-bit
      typedef  int16_t iWord;    // signed 16-bit
      typedef  uint32_t D_Word;  // 32-bit
      typedef  int32_t iD_Word;  // signed 32-bit
      typedef  uint64_t Q_Word;  // 64bit
      typedef  uint64_t iQ_Word; // signed 64bit
      typedef union _BYTE_VAL
      {
        Byte B_Val;//byte value
        iByte iB_Val;//byte value
        struct
        {
          Byte b0:1;
          Byte b1:1;
          Byte b2:1;
          Byte b3:1;
          Byte b4:1;
          Byte b5:1;
          Byte b6:1;
          Byte b7:1;
        }bits;
        struct
        {
          Byte L_N:4;//low nibble
          Byte H_N:4;//high nibble
        }Nibbles;
      }BYTE_VAL;

      typedef union _WORD_VAL
      {
        Word W_Val;//word value
        iWord iW_Val;
        Byte By[2];//byte array
        struct
        {
          Byte L_B;//low byte
          Byte H_B;//high byte
        }byte;
        struct
        {
          Byte b0:1;
          Byte b1:1;
          Byte b2:1;
          Byte b3:1;
          Byte b4:1;
          Byte b5:1;
          Byte b6:1;
          Byte b7:1;
          Byte b8:1;
          Byte b9:1;
          Byte b10:1;
          Byte b11:1;
          Byte b12:1;
          Byte b13:1;
          Byte b14:1;
          Byte b15:1;
        }bits;
      }WORD_VAL;

      typedef union _D_WORD_VAL
      {
        D_Word D_Val;//double word value
        iD_Word iD_Val;
        Word W[2];//word array
        Byte By[4];//byte array
        struct
        {
          Word L_W;//low word
          Word H_W;//high word
        }word;
        struct
        {
          Byte b0:1;
          Byte b1:1;
          Byte b2:1;
          Byte b3:1;
          Byte b4:1;
          Byte b5:1;
          Byte b6:1;
          Byte b7:1;
          Byte b8:1;
          Byte b9:1;
          Byte b10:1;
          Byte b11:1;
          Byte b12:1;
          Byte b13:1;
          Byte b14:1;
          Byte b15:1;
          Byte b16:1;
          Byte b17:1;
          Byte b18:1;
          Byte b19:1;
          Byte b20:1;
          Byte b21:1;
          Byte b22:1;
          Byte b23:1;
          Byte b24:1;
          Byte b25:1;
          Byte b26:1;
          Byte b27:1;
          Byte b28:1;
          Byte b29:1;
          Byte b30:1;
          Byte b31:1;
        }bits;
      }D_WORD_VAL;

      typedef union _Q_WORD_VAL
      {
        Q_Word Q_Val;//quadruple word value
        iQ_Word iQ_Val;
        D_Word Dw[2];//double word array
        Word W[4];//word array
        Byte By[8];//byte array
        struct
        {
          Byte b0:1;
          Byte b1:1;
          Byte b2:1;
          Byte b3:1;
          Byte b4:1;
          Byte b5:1;
          Byte b6:1;
          Byte b7:1;
          Byte b8:1;
          Byte b9:1;
          Byte b10:1;
          Byte b11:1;
          Byte b12:1;
          Byte b13:1;
          Byte b14:1;
          Byte b15:1;
          Byte b16:1;
          Byte b17:1;
          Byte b18:1;
          Byte b19:1;
          Byte b20:1;
          Byte b21:1;
          Byte b22:1;
          Byte b23:1;
          Byte b24:1;
          Byte b25:1;
          Byte b26:1;
          Byte b27:1;
          Byte b28:1;
          Byte b29:1;
          Byte b30:1;
          Byte b31:1;
          Byte b32:1;
          Byte b33:1;
          Byte b34:1;
          Byte b35:1;
          Byte b36:1;
          Byte b37:1;
          Byte b38:1;
          Byte b39:1;
          Byte b40:1;
          Byte b41:1;
          Byte b42:1;
          Byte b43:1;
          Byte b44:1;
          Byte b45:1;
          Byte b46:1;
          Byte b47:1;
          Byte b48:1;
          Byte b49:1;
          Byte b50:1;
          Byte b51:1;
          Byte b52:1;
          Byte b53:1;
          Byte b54:1;
          Byte b55:1;
          Byte b56:1;
          Byte b57:1;
          Byte b58:1;
          Byte b59:1;
          Byte b60:1;
          Byte b61:1;
          Byte b62:1;
          Byte b63:1;
        }bits;
      }Q_WORD_VAL;
  #ifdef __cplusplus
    }
  #endif
#endif //__GENERIC_TYPE_DEFS_H_
