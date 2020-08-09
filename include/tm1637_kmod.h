/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (C) 2020 Alexander Mishin
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _TM1637_KMOD_H_
#define _TM1637_KMOD_H_

#define TM1637_MODE_PROPERTY	"mode"

#define TM1637_ADDRESS_AUTO	0x40
#define TM1637_ADDRESS_FIXED	0x44
#define TM1637_START_ADDRESS	0xc0

#define TM1637_BRIGHT_DARKEST	0
#define TM1637_BRIGHT_DARK	1
#define TM1637_BRIGHT_TYPICAL	2
#define TM1637_BRIGHTEST	7

#define	TM1637_COLON_POSITION	2 // 2-nd character
#define TM1637_MAX_COLOM	4
#define	TM1637_BUFFERSIZE	TM1637_MAX_COLOM + 2 // For ':' and '\n'

struct tm1637_clock_t {
    int tm_min;
    int tm_hour;
    bool tm_colon;
};

#define TM1637IOC_CLEAR			_IO('T', 1)
#define TM1637IOC_OFF			_IO('T', 2)
#define TM1637IOC_ON			_IO('T', 3)
#define TM1637IOC_SET_BRIGHTNESS	_IOW('T', 11, uint8_t)
#define TM1637IOC_SET_CLOCKPOINT	_IOW('T', 12, uint8_t)
#define TM1637IOC_SET_RAWMODE		_IOW('T', 13, uint8_t)
#define TM1637IOC_SET_CLOCK		_IOW('T', 14, struct tm1637_clock_t)
#define TM1637IOC_GET_RAWMODE		_IOR('T', 23, uint8_t)

struct s_message {
    char text[TM1637_BUFFERSIZE + 1]; // ??? +1
    int offset;
    int len;
};

MALLOC_DEFINE(M_TM1637BUF, "tm1637buffer", "Buffer for tm1637 module");

#endif /* _TM1637_KMOD_H_ */
