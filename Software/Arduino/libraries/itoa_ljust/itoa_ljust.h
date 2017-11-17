#ifndef ITOA_LJUST_H
#define ITOA_LJUST_H

//=== itoa_ljust.h - Fast integer to ascii conversion             --*- C++ -*-//
//
// Fast and simple integer to ASCII conversion:
//
//   - 8, 16, 32 and 64-bit integers
//   - signed and unsigned
//   - user supplied buffer must be large enough for all decimal digits
//     in value plus minus sign if negative
//   - left-justified
//   - NUL terminated
//   - return value is pointer to NUL terminator
//
// Copyright (c) 2017 Jens Grabner
// jens@grabner-online.org
// https://github.com/JensGrabner/snc98_Slash-Number-Calculator/tree/master/Software/Arduino/libraries/itoa_ljust
//
// Copyright (c) 2016 Arturo Martin-de-Nicolas
// arturomdn@gmail.com
// https://github.com/amdn/itoa_ljust/
//===----------------------------------------------------------------------===//

#include <stdint.h>



    char* itoa_( uint8_t u, char* buffer);     // with prev..  '' <-->  '-'
    char* itoa_(  int8_t i, char* buffer);     // with prev..  '' <-->  '-'
    char* itoa_(uint16_t u, char* buffer);     // with prev..  '' <-->  '-'
    char* itoa_( int16_t i, char* buffer);     // with prev..  '' <-->  '-'
    char* itoa_(uint32_t u, char* buffer);     // with prev..  '' <-->  '-'
    char* itoa_( int32_t i, char* buffer);     // with prev..  '' <-->  '-'
    char* itoa_(uint64_t u, char* buffer);     // with prev..  '' <-->  '-'
    char* itoa_( int64_t i, char* buffer);     // with prev..  '' <-->  '-'

    char* itoa__( uint8_t u, char* buffer);    // with prev..  '  ' <-->  ' -'
    char* itoa__(  int8_t i, char* buffer);    // with prev..  '  ' <-->  ' -'
    char* itoa__(uint16_t u, char* buffer);    // with prev..  '  ' <-->  ' -'
    char* itoa__( int16_t i, char* buffer);    // with prev..  '  ' <-->  ' -'
    char* itoa__(uint32_t u, char* buffer);    // with prev..  '  ' <-->  ' -'
    char* itoa__( int32_t i, char* buffer);    // with prev..  '  ' <-->  ' -'
    char* itoa__(uint64_t u, char* buffer);    // with prev..  '  ' <-->  ' -'
    char* itoa__( int64_t i, char* buffer);    // with prev..  '  ' <-->  ' -'



#endif // ITOA_LJUST_H
