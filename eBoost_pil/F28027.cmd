/*
 Copyright (c) 2010-2016 by CodeSkin
 All rights reserved.

 NOTICE: This software embodies unpublished proprietary
 and trade secret information of CodeSkin.  Any copying,
 disclosure or transmission not expressly authorized by
 the owner is expressly prohibited. CodeSkin retains exclusive
 ownership of all intellectual property rights in this software,
 including all copyright, trade secret and patent rights.
 */

MEMORY
{
PAGE 0:
   FLASH       : origin = 0x3F0000, length = 0x007F80     /* on-chip FLASH */
   CSM_RSVD    : origin = 0x3F7F80, length = 0x000076     /* Part of FLASHA.  Program with all 0x0000 when CSM is in use. */
   BEGIN       : origin = 0x3F7FF6, length = 0x000002     /* Part of FLASHA.  Used for "boot to Flash" bootloader mode. */
   CSM_PWL_P0  : origin = 0x3F7FF8, length = 0x000008     /* Part of FLASHA.  CSM password locations in FLASHA */

   PRAML0      : origin = 0x008C00, length = 0x000400     /* on-chip RAM block L0 */

   RESET       : origin = 0x3FFFC0, length = 0x000002     /* part of boot ROM  */
   VECTORS     : origin = 0x3FFFC2, length = 0x00003E     /* part of boot ROM  */
   IQTABLES    : origin = 0x3FE000, length = 0x000B50     /* IQ Math Tables in Boot ROM */
   IQTABLES2   : origin = 0x3FEB50, length = 0x00008C     /* IQ Math Tables in Boot ROM */
   IQTABLES3   : origin = 0x3FEBDC, length = 0x0000AA      /* IQ Math Tables in Boot ROM */

   ROM         : origin = 0x3FF27C, length = 0x000D44     /* Boot ROM */

PAGE 1 :   /* Data Memory */
           /* Memory (RAM/FLASH/OTP) blocks can be moved to PAGE0 for program allocation */
           /* Registers remain on PAGE1                                                  */

   BOOT_RSVD   : origin = 0x000000, length = 0x000050     /* Part of M0, BOOT rom will use this for stack */
   RAMM0       : origin = 0x000050, length = 0x0003B0     /* on-chip RAM block M0 */
   RAMM1       : origin = 0x000400, length = 0x000400     /* on-chip RAM block M1 */

   DRAML0	   : origin = 0x008000, length = 0x000C00	  /* secure */
 }


SECTIONS
{
   /* Allocate program areas: */
   .cinit              : > FLASH,     PAGE = 0
   .pinit              : > FLASH,     PAGE = 0
   .text               : > FLASH,     PAGE = 0
   codestart           : > BEGIN,      PAGE = 0, { __APP_ENTRY = .;}

   ramfuncs            : LOAD = FLASH,
                         RUN = PRAML0,
                         LOAD_START(_RamfuncsLoadStart),
                         LOAD_END(_RamfuncsLoadEnd),
                         RUN_START(_RamfuncsRunStart),
						 LOAD_SIZE(_RamfuncsLoadSize),
                         PAGE = 0

   csmpasswds          : > CSM_PWL_P0, PAGE = 0
   csm_rsvd            : > CSM_RSVD,   PAGE = 0

   /* Allocate uninitalized data sections: */
   .stack              : > DRAML0,      PAGE = 1
   .ebss               : > DRAML0,      PAGE = 1
   .esysmem            : > DRAML0,      PAGE = 1

   /* Initalized sections to go in Flash */
   /* For SDFlash to program these, they must be allocated to page 0 */
   .econst             : > FLASH,     PAGE = 0
   .switch             : > FLASH,     PAGE = 0

   .reset              : > RESET,      PAGE = 0, TYPE = DSECT /* not used, */
}
