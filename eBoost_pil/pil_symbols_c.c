/*
   Copyright (c) 2017 by Plexim GmbH
   All rights reserved.

   A free license is granted to anyone to use this software for any legal
   non safety-critical purpose, including commercial applications, provided
   that:
   1) IT IS NOT USED TO DIRECTLY OR INDIRECTLY COMPETE WITH PLEXIM, and
   2) THIS COPYRIGHT NOTICE IS PRESERVED in its entirety.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
   OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
 */

#include "main.h"

// this file can be linked such that no Flash memory is consumed

#ifndef PIL_PREP_TOOL
#include "pil_symbols_c.inc" // will be automatically generated
#endif

#pragma RETAIN(PIL_D_Guid)
PIL_CONST_DEF(unsigned char, Guid[], CODE_GUID);

#pragma RETAIN(PIL_D_CompiledDate)
PIL_CONST_DEF(unsigned char, CompiledDate[], COMPILE_TIME_DATE_STR);

#pragma RETAIN(PIL_D_CompiledBy)
PIL_CONST_DEF(unsigned char, CompiledBy[], USER_NAME);

#pragma RETAIN(PIL_D_FrameworkVersion)
PIL_CONST_DEF(uint16_t, FrameworkVersion, PIL_FRAMEWORK_VERSION);

#pragma RETAIN(PIL_D_BaudRate)
PIL_CONST_DEF(uint32_t, BaudRate, BAUD_RATE);

#pragma RETAIN(PIL_D_StationAddress)
PIL_CONST_DEF(uint16_t, StationAddress, 0);

#pragma RETAIN(PIL_D_FirmwareDescription)
PIL_CONST_DEF(char, FirmwareDescription[], "eBoost project");
