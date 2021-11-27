#ifndef _Ratio32_h_
#define _Ratio32_h_

// Copyright (c) 2021 Jens Grabner
// Email: jens@grabner-online.org
// https://github.com/JensGrabner/snc98_Slash-Number-Calculator/tree/master/Software/Arduino/libraries/Ratio32

#include <stdint.h>  // uintX_t types
#include <stdlib.h> 
#include <Arduino.h>

#include <int96.h>
// Original:  http://www.naughter.com/int96.html
// https://github.com/JensGrabner/snc98_Slash-Number-Calculator/tree/master/Software/Arduino/libraries/int96

////////// Defines ///////////////////////

#define expo_max_input  90
#define expo_min_input -90

#define int32_max     2147302920     // = int32_2_max * int32_2_max - 1
#define expo_test_0a  0x1993333333333333ULL   //  1842872967520006963 x 0,2

#define num_exp2_1_2     1073651460
#define denum_exp2_1_2   2147302920   // 1/2

// ---  log2()_Konstante  ---
#define num_lb_to_e       497083768
#define denum_lb_to_e     717140287   // Fehler .. -2,02e-19

// ---  log10()_Konstante  ---
#define num_lb_to_10      579001193
#define denum_lb_to_10   1923400330   // Fehler ..  2,09e-20

// ---  Null_no_Konstante  ---
#define Null_no_expo            0
#define Null_no_num             0
#define Null_no_denom  2147302920  //

// ---  ln_to_10_Konstante  --- 
#define num_ln_to_10     1784326399
#define denom_ln_to_10    774923109   // Fehler ..  2,96e-20

// ---  e_Konstante  ---
#define e_expo                  0
#define e_num          1696912706
#define e_denom         624259298     // Fehler .. -6.03e-19

/*
 * rational number "numerator / denominator"
 */
struct Ratio_32{   //     0.3 ... 3 x 10^expo
	 int8_t expo;        // <-- expo
	int32_t num;         // <-- numerator
	int32_t denom;       // <-- denominator
	uint8_t op;          // <-- operation
};

struct R_uint32{   //     0.3 ... 3
	uint32_t num;        // <-- numerator
	uint32_t denom;      // <-- denominator
};

struct R_uint64{   //     0.3 ... 3
	uint64_t num;        // <-- numerator
	uint64_t denom;      // <-- denominator
};

struct Ratio_64{   //     0.3 ... 3 x 10^expo
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

#define expo_10_0                  0x1UL    // 1
#define expo_10_1                  0xAUL    // 10
#define expo_10_2                 0x64UL    // 100
#define expo_10_3                0x3E8UL    // 1000
#define expo_10_4               0x2710UL    // 10000
#define expo_10_5              0x186A0UL    // 100000
#define expo_10_6              0xF4240UL    // 1000000
#define expo_10_7             0x989680UL    // 10000000
#define expo_10_8            0x5F5E100UL    // 100000000
#define expo_10_9           0x3B9ACA00UL    // 1000000000

#define expo_10_10         0x2540BE400ULL   // 10000000000
#define expo_10_11        0x174876E800ULL   // 100000000000
#define expo_10_12        0xE8D4A51000ULL   // 1000000000000
#define expo_10_13       0x9184E72A000ULL   // 10000000000000
#define expo_10_14      0x5AF3107A4000ULL   // 100000000000000
#define expo_10_15     0x38D7EA4C68000ULL   // 1000000000000000
#define expo_10_16    0x2386F26FC10000ULL   // 10000000000000000
#define expo_10_17   0x16345785D8A0000ULL   // 100000000000000000
#define expo_10_18   0xDE0B6B3A7640000ULL   // 1000000000000000000
#define expo_10_19  0x8AC7230489E80000ULL   // 10000000000000000000

// ---  cbrt(10)_Konstante  ---
#define cbrt_10_expo            0
#define cbrt_10_num     265396349
#define cbrt_10_denom   123186073  // Fehler ..  3,9e-18

#define cbrt_100_expo           1
#define cbrt_100_num    123186073
#define cbrt_100_denom  265396349  // Fehler .. -8,3e-18

// ---  sqrt(10)_Konstante  ---
#define sqrt_10_expo            0
#define sqrt_10_num    1499219281
#define sqrt_10_denom   474094764  // Fehler ..  7.03e-19

#define expo_71             2
#define num_71     1524585059
#define denum_71   2147302900   // 71/100


#define int32_max_16    1288381752     // = int32_max * 0.6

// http://www.luschny.de/math/factorial/approx/SimpleCases.html#AhighPrecisionApproximation
// http://www.luschny.de/math/factorial/approx/
// Approximations%20for%20the%20Factorial%20Function.pdf  page 8
// StieltjesLnFactorial(z)
// https://dlmf.nist.gov/5.10
// https://jamesmccaffrey.wordpress.com/2013/06/19/the-log-gamma-function-with-c/
// static double LogGammaContinued(double x)
// https://infogalactic.com/info/Factorial#Approximations_of_factorial
// ---  fa_0_Konstante  ---     1/12
#define fa_0_expo            -1 // 0,083333
#define fa_0_num     1731248500
#define fa_0_denom   2077498200 //

// ---  fa_1_Konstante  ---     1/30
#define fa_1_expo            -1 // 0,033333
#define fa_1_num      692499400
#define fa_1_denom   2077498200 //

// ---  fa_2_Konstante  ---     53/210
#define fa_2_expo            -1 // 0,252381
#define fa_2_num     2077498240
#define fa_2_denom    823159680 //

// ---  fa_3_Konstante  ---     195/371
#define fa_3_expo             0 // 0,525606
#define fa_3_num     1091946960
#define fa_3_denom   2077499088

// ---  fa_4_Konstante  ---     22999/22737
#define fa_4_expo             0 // 1,011523
#define fa_4_num     2077499670
#define fa_4_denom   2053833210

// ---  fa_5_Konstante  ---     29944523/19733142
#define fa_5_expo             0 // 1,517474
#define fa_5_num     2036227564
#define fa_5_denom   1341853656

// ---  fa_6_Konstante  ---     109535241009/48264275462
//                              19976135/8802041      -> Fehler 1,037e-18
#define fa_6_expo             0 // 2,269489
#define fa_6_num     2077518040
#define fa_6_denom    915412264

// ---  fa_7_Konstante  ---     29404527905795295658/
//                              9769214287853155785
//                              527522494/175261453  -> Fehler  1,531e-18
#define fa_7_expo             1 // 3,009917
#define fa_7_num      527522494
#define fa_7_denom   1752614530

#define fa_x_max              7

// ---  fa_ln_2pi_2_Konstante  ---
#define fa_ln_2pi_2_expo             0
#define fa_ln_2pi_2_num     1474345081
#define fa_ln_2pi_2_denom   1604400107 // Fehler .. +2,016e-19

#define num_exp2_0_1     2147302920
#define denum_exp2_0_1   2147302920   // 1/1

// ---  Pi_Konstante  ---
#define Pi_expo                 0
#define Pi_num         2137933792
#define Pi_denom        680525462  // Fehler ..  -9,77e-19

////////// Functions extern ////////////////

void animation(uint8_t, uint8_t);   // from  ..  snc98.ino
Ratio_32 sin_r(Ratio_32);           // from  ..  snc98.ino

////////// Functions ///////////////////////

Ratio_32 min_x(Ratio_32);

Ratio_32 div_x(Ratio_32);

Ratio_32 abs_x(Ratio_32);

Ratio_32 clone(Ratio_32);

int8_t compare(Ratio_32, Ratio_32);

int8_t compare(R_uint32, R_uint32);

int8_t compare(R_uint64, R_uint64);

R_uint32 compare_extra(R_uint32, R_uint32, uint64_t, uint64_t);

Ratio_32 Reduce_Number(uint64_t, uint64_t, int8_t);

Ratio_32 to_Ratio_32(Ratio_64);

Ratio_32 to_Ratio_32(int16_t);

Ratio_32 to_Ratio_32(uint16_t);

Ratio_32 to_Ratio_32(int16_t, int8_t);

Ratio_32 to_Ratio_32(uint16_t, int8_t);

Ratio_32 mul(Ratio_32, Ratio_32);

Ratio_32 add(Ratio_32, Ratio_32, int8_t);

Ratio_32 square(Ratio_32);

Ratio_32 cubic(Ratio_32);

Ratio_32 loge(Ratio_32);

Ratio_32 log10(Ratio_32);

Ratio_32 log2(Ratio_32);

Ratio_32 exp(Ratio_32);

Ratio_32 exp2(Ratio_32);

Ratio_32 exp10(Ratio_32);

Ratio_32 frac(Ratio_32);

Ratio_32 floor_(Ratio_32, int8_t);

Ratio_32 add_mul_spezial(Ratio_32, Ratio_32, Ratio_32, int8_t);

Ratio_32 sqrt(Ratio_32);

Ratio_32 cbrt(Ratio_32);

Ratio_32 factorial(Ratio_32);

#endif