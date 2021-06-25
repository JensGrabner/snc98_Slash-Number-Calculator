#ifndef _Ratio32_h_
#define _Ratio32_h_

// Copyright (c) 2021 Jens Grabner
// Email: jens@grabner-online.org
// https://github.com/JensGrabner/snc98_Slash-Number-Calculator/tree/master/Software/Arduino/libraries/Ratio32

#include <stdint.h>
#include <stdlib.h> 
#include <Arduino.h>

////////// Defines ///////////////////////

#define int32_max     2147302920     // = int32_2_max * int32_2_max - 1
#define expo_test_0a  0x1993333333333333ULL   //  1842872967520006963 x 0,2

#define num_exp2_1_2     1073651460
#define denum_exp2_1_2   2147302920   // 1/2

#define expo_10_0            0x1UL   // 1
#define expo_10_1            0xAUL   // 10
#define expo_10_2           0x64UL   // 100
#define expo_10_3          0x3E8UL   // 1000
#define expo_10_4         0x2710UL   // 10000
#define expo_10_5        0x186A0UL   // 100000
#define expo_10_6        0xF4240UL   // 1000000
#define expo_10_7       0x989680UL   // 10000000
#define expo_10_8      0x5F5E100UL   // 100000000
#define expo_10_9     0x3B9ACA00UL   // 1000000000

#define expo_10_10           0x2540BE400ULL   // 10000000000
#define expo_10_11          0x174876E800ULL   // 100000000000
#define expo_10_12          0xE8D4A51000ULL   // 1000000000000
#define expo_10_13         0x9184E72A000ULL   // 10000000000000
#define expo_10_14        0x5AF3107A4000ULL   // 100000000000000
#define expo_10_15       0x38D7EA4C68000ULL   // 1000000000000000
#define expo_10_16      0x2386F26FC10000ULL   // 10000000000000000
#define expo_10_17     0x16345785D8A0000ULL   // 100000000000000000
#define expo_10_18     0xDE0B6B3A7640000ULL   // 1000000000000000000
#define expo_10_19    0x8AC7230489E80000ULL   // 10000000000000000000

/*
 * rational number "numerator / denominator"
 */
struct Ratio_32{   //     0.3 ... 3 x 10^expo
	 int8_t expo;        // <-- expo
	int32_t num;         // <-- numerator
	int32_t denom;       // <-- denominator
	uint8_t op;          // <-- operation
};

struct R_uint32{    //     0.3 ... 3
	uint32_t num;        // <-- numerator
	uint32_t denom;      // <-- denominator
};

struct R_uint64{    //     0.3 ... 3
	uint64_t num;        // <-- numerator
	uint64_t denom;      // <-- denominator
};

struct Ratio_64{  //     0.3 ... 3 x 10^expo
	int16_t expo;        // <-- expo
	int64_t num;         // <-- numerator
	int64_t denom;       // <-- denominator
};

struct Ratio_32_plus{  //     0.3 ... 3 x 10^expo
	int8_t expo;         // <-- expo
	int32_t num;         // <-- numerator
	int32_t denom;       // <-- denominator
	uint8_t op;          // <-- operation
	uint8_t op_priority; // <-- priority
	uint8_t bracket;     // <-- bracket
};
////////// Functions ///////////////////////

extern Ratio_32 min_x(Ratio_32);

extern Ratio_32 div_x(Ratio_32);

extern Ratio_32 abs_x(Ratio_32);

extern Ratio_32 clone(Ratio_32);

extern int8_t compare(Ratio_32, Ratio_32);

extern int8_t compare(R_uint32, R_uint32);

extern int8_t compare(R_uint64, R_uint64);

extern R_uint32 compare_extra(R_uint32, R_uint32, uint64_t, uint64_t);

extern Ratio_32 Reduce_Number(uint64_t, uint64_t, int8_t);

extern Ratio_32 to_Ratio_32(Ratio_64);

extern Ratio_32 to_Ratio_32(uint16_t);

extern Ratio_32 to_Ratio_32(uint16_t, int8_t);

extern Ratio_32 mul(Ratio_32, Ratio_32);

extern Ratio_32 add(Ratio_32, Ratio_32, int8_t);

#endif