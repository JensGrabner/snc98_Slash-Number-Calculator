//=== itoa_ljust.cpp - Fast integer to ascii conversion           --*- C++ -*-//
//
// Substantially simplified (and slightly faster) version
// based on the following functions in Google's protocol buffers:
//
//    FastInt32ToBufferLeft()
//    FastUInt32ToBufferLeft()
//    FastInt64ToBufferLeft()
//    FastUInt64ToBufferLeft()
//
// Differences:
//    1) Greatly simplified
//    2) Avoids GOTO statements - uses "switch" instead and relies on
//       compiler constant folding and propagation for high performance
//    3) Avoids unary minus of signed types - undefined behavior if value
//       is INT_MIN in platforms using two's complement representation
//    4) Uses memcpy to store 2 digits at a time - lets the compiler
//       generate a 2-byte load/store in platforms that support
//       unaligned access, this is faster (and less code) than explicitly
//       loading and storing each byte
//
// Copyright (c) 2017 Jens Grabner
// jens@grabner-online.org
// https://github.com/JensGrabner/snc98_Slash-Number-Calculator/tree/master/Software/Arduino/libraries/itoa_ljust
//
// Copyright (c) 2016 Arturo Martin-de-Nicolas
// arturomdn@gmail.com
// https://github.com/amdn/itoa_ljust/
//
// Released under the BSD 3-Clause License, see Google's original copyright
// and license below.
//===----------------------------------------------------------------------===//

// Protocol Buffers - Google's data interchange format
// Copyright 2008 Google Inc.  All rights reserved.
// https://developers.google.com/protocol-buffers/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//===----------------------------------------------------------------------===//

#include "itoa_ljust.h"
#include <string.h>

    static const char lut[] =
        "0001020304050607080910111213141516171819"
        "2021222324252627282930313233343536373839"
        "4041424344454647484950515253545556575859"
        "6061626364656667686970717273747576777879"
        "8081828384858687888990919293949596979899";

    static inline uint16_t const& dd(uint8_t u) {
        return reinterpret_cast<uint16_t const*>(lut)[u];
    }

    template<typename T>
    static inline char* out(T const& obj, char* p) {
        memcpy(p, reinterpret_cast<const void*>(&obj), sizeof(T));
        return p + sizeof(T);
    }

    static inline uint8_t digits_8( uint8_t u, uint8_t k, uint8_t &d, char* &p, uint8_t n ) {
        uint8_t test = k;
        test *= 10;
        if (u < test) {
            d = u / k;
            p = out<char>('0'+d, p);
            --n;
        }
        if (n > 2) {
            d = u / test;
            p = out<char>('0'+d, p);
        }
        return n;
    }

    static inline uint8_t digits_16( uint16_t u, uint16_t k, uint16_t &d, char* &p, uint8_t n ) {
        uint16_t test = k;
        test *= 10;
        if (u < test) {
            d = u / k;
            p = out<char>('0'+d, p);
            --n;
        }
        if (n > 4) {
            d = u / test;
            p = out<char>('0'+d, p);
        }
        return n;
    }

    static inline uint8_t digits_32( uint32_t u, uint32_t k, uint32_t &d, char* &p, uint8_t n ) {
        uint32_t test = k;
        test *= 10;
        if (u < test) { 
            d = u / k;
            p = out<char>('0'+d, p);
            --n;
        }
        return n;
    }

    static inline char* itoa_8(uint8_t u, char* p, uint8_t d, uint8_t n) {
        switch(n) {
        case  3: u -= d * 100;
        case  2: p = out( dd(u), p );
        default: *p = '\0'; return p;
        }
    }

    static inline char* itoa_16(uint16_t u, char* p, uint16_t d, uint8_t n) {
        switch(n) {
        case  5: u -= d * 10000;
        case  4: d  = u /   100; p = out( dd(d), p );
        case  3: u -= d *   100;
        case  2: n = 2 ;
        default: return itoa_8( u, p, d, n );
        }
    }

    static inline char* itoa_32(uint32_t u, char* p, uint32_t d, uint8_t n) {
        switch(n) {
        case 10: d  = u / 100000000; p = out( dd(d), p );
        case  9: u -= d * 100000000;
        case  8: d  = u /   1000000; p = out( dd(d), p );
        case  7: u -= d *   1000000;
        case  6: d  = u /     10000; p = out( dd(d), p );
        case  5: u -= d *     10000;
        case  4: n = 4 ;
        case  3: ;
        case  2: ;
        default: return itoa_16( u, p, d, n );
        }
    }

    char* itoa_(uint8_t u, char* p) {
        uint8_t d = 0;
        uint8_t n;
             if (u >  99) n = digits_8(u,  10, d, p, 3);
        else              n = digits_8(u,   1, d, p, 2);
        return itoa_8( u, p, d, n );
    }

    char* itoa_(uint16_t u, char* p) {
        uint8_t lower = u;
        if (lower == u) return itoa_(lower, p);

        uint16_t d = 0;
        uint8_t n;
             if (u >  9999) n = digits_16(u,  1000, d, p, 5);
        else if (u <   100) n = digits_16(u,     1, d, p, 2);
        else                n = digits_16(u,   100, d, p, 4);
        return itoa_16( u, p, d, n );
    }

    char* itoa_(uint32_t u, char* p) {
        uint16_t lower = u;
        if (lower == u) return itoa_(lower, p);

        uint32_t d = 0;
        uint8_t n;
             if (u >  99999999) n = digits_32(u, 100000000, d, p, 10);
        else if (u <       100) n = digits_32(u,         1, d, p,  2);
        else if (u <     10000) n = digits_32(u,       100, d, p,  4);
        else if (u <   1000000) n = digits_32(u,     10000, d, p,  6);
        else                    n = digits_32(u,   1000000, d, p,  8);
        return itoa_32( u, p, d, n );
    }

    char* itoa_(uint64_t u, char* p) {
        uint32_t lower = u;
        if (lower == u) return itoa_(lower, p);

        uint64_t d = 0;
        uint64_t upper = 0;

        upper  = u / 1000000000;
        p  = itoa_(upper, p);
        u -= upper * 1000000000;
        d  =      u / 100000000;
        p  = out<char>('0'+d, p);
        return itoa_32( u, p, d, 9 );
    }

    char* itoa_(int8_t i, char* p) {
        uint8_t u;
        if (i < 0) {
            *p++ = '-';
            u = -i;
        }
        else {
            u = i;
        }
        return itoa_(u, p);
    }

    char* itoa_(int16_t i, char* p) {
        uint16_t u;
        if (i < 0) {
            *p++ = '-';
            u = -i;
        }
        else {
            u = i;
        }
        return itoa_(u, p);
    }

    char* itoa_(int32_t i, char* p) {
        uint32_t u;
        if (i < 0) {
            *p++ = '-';
            u = -i;
        }
        else {
            u = i;
        }
        return itoa_(u, p);
    }

    char* itoa_(int64_t i, char* p) {
        uint64_t u;
        if (i < 0) {
            *p++ = '-';
            u = -i;
        }
        else {
            u = i;
        }
        return itoa_(u, p);
    }


