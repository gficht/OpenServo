/*
    Copyright (c) 2006 Michael P. Thompson <mpthompson@gmail.com>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use, copy,
    modify, merge, publish, distribute, sublicense, and/or sell copies
    of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.

    $Id: openservo.h,v 1.7 2007/02/12 17:56:51 ginge Exp $
*/

#ifndef _OS_OPENSERVO_H_
#define _OS_OPENSERVO_H_ 1

// Define the device type and subtype.  These values are used so that
// external TWI clients can query the type of device the servo is.
#define OPENSERVO_DEVICE_TYPE           1
#define OPENSERVO_DEVICE_SUBTYPE        1

// Define the software major and minor version.  These values are used
// so that external TWI clients can query the version of the software
// running on the servo.
#define SOFTWARE_VERSION_MAJOR          0
#define SOFTWARE_VERSION_MINOR          2

// The default TWI address. Change this if you want to change the TWI address of the servo
#define REG_DEFAULT_TWI_ADDR		0x10

//
// Utility functions.
//
typedef int8_t bool;
#define FALSE   0
#define TRUE    -1

#ifndef NULL
    #define NULL 0
#endif
#define enterCritical() __asm__ __volatile__ ("in __tmp_reg__, __SREG__\n\t " \
                                              "push __tmp_reg__\n\t" \
                                              "cli" ::)

#define exitCritical()  __asm__ __volatile__ ("pop __tmp_reg__ \n\t" \
                                              "out __SREG__, __tmp_reg__" ::)

// Disable interrupts and returns SREG value used to restore interrupts.
inline static uint8_t disable_interrupts(void)
{
    uint8_t sreg;

    __asm__ volatile (
        "in %0,__SREG__\n\t"
        "cli\n\t"
        : "=r" ((uint8_t) sreg)
        :
    );

    return sreg;
}

// Restore interrupts Enables interrupts according to the SREG.
inline static void restore_interrupts(uint8_t sreg)
{
    __asm__ volatile (
        "out __SREG__,%0\n\t"
        :
        : "r" ((uint8_t) sreg)
    );
}

#endif // _OS_OPENSERVO_H_
