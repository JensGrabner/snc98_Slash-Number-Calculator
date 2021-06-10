#ifndef Ratio32_h
#define Ratio32_h

// Copyright (c) 2021 Jens Grabner
// Email: jens@grabner-online.org
// https://github.com/JensGrabner/snc98_Slash-Number-Calculator/tree/master/Software/Arduino/libraries/Ratio32

#include <stdint.h>
#include <stdlib.h> 
#include <Arduino.h>

////////// Defines ///////////////////////

#define int32_max     2147302920     // = int32_2_max * int32_2_max - 1

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

extern Ratio_32 Reduce_Number(int8_t, uint64_t, uint64_t);

#endif