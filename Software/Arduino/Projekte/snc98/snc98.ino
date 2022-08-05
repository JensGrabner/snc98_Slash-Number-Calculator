/**********************************************************************

	 Project: xx snc98 - Slash Number Calculator

	 MIT License

	 Copyright (c) 2020 Jens Grabner

	 Permission is hereby granted, free of charge, to any person obtaining
	 a copy of this software and associated documentation files
	 (the "Software"), to deal in the Software without restriction,
	 including without limitation the rights to use, copy, modify, merge,
	 publish, distribute, sublicense, and/or sell copies of the Software,
	 and to permit persons to whom the Software is furnished to do so,
	 subject to the following conditions:

	 The above copyright notice and this permission notice shall be
	 included in all copies or substantial portions of the Software.

	 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
	 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
	 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
	 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
	 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
	 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
	 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**********************************************************************/

/*                          HEADER

	*********************************************************************
	* Program for my "snc98 - Slash Number Calculator"                  *
	* If you do/have/found any updates, modifications, suggestions,     *
	* comments, ideas, bugs, flaws or anything else - contact me        *
	*                                                                   *
	* Program and schematics are supplied AS IS                         *
	* It's your job to check datasheets, schematics, source code...     *
	*                                                                   *
	* Make sure you understand it as I do not take ANY responsibility   *
	*   if something goes wrong or will not work                        *
	* All this is intended for inspiration only, if you create your     *
	*   own version, you CAN say, I did it (well, after hours of        *
	*   troubleshooting and screaming).                                 *
	*********************************************************************

*/
#include <Ratio32.h>

#include <string.h>
#include <stdint.h>
#include <stdlib.h>  // for itoa(); ltoa();

#include <Arduino.h>

// #include <OneWire.h>
// https://github.com/PaulStoffregen/OneWire

#include <avr/wdt.h>

#include <pins_arduino.h>  // ..\avr\variants\standard\pins_arduino.h
// https://github.com/MCUdude/MightyCore

#define MCP9808_I2CADDR_DEFAULT 0x18
#include <Wire.h>
#include <Adafruit_MCP9808.h>
// version=1.1.2
// https://github.com/adafruit/Adafruit_MCP9808_Library
// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

#include <BitBool.h>
// https://github.com/Chris--A/BitBool
// http://arduino.land/Code/BitBool/

#include <RingBufCPP.h>
// https://github.com/wizard97/Embedded_RingBuf_CPP

#include <int96.h>
// Original:  http://www.naughter.com/int96.html
// https://github.com/JensGrabner/snc98_Slash-Number-Calculator/tree/master/Software/Arduino/libraries/int96

// #include <r128.h>
// Original:  https://github.com/fahickman/r128


#include <itoa_ljust.h>
// https://github.com/JensGrabner/snc98_Slash-Number-Calculator/tree/master/Software/Arduino/libraries/itoa_ljust
char  display_string_itoa_[33];

#include <TimerOne.h>// version=1.0.0
// https://github.com/PaulStoffregen/TimerOne
// https://downloads.arduino.cc/libraries/github.com/khoih-prog/megaAVR_TimerInterrupt-1.0.0.zip
// folder: C:\Users\jens\Documents\Arduino\libraries\TimerOne
// TimerOne at version 1.1
// https://github.com/PaulStoffregen/TimerOne
// version=1.0.0
// #include <OneWireHub.h>
// https://github.com/orgua/OneWireHub

#define Debug_Level 18  //  0 - not Debug
											 //  1 - Test intern 1ms - Task by 100 ms
											 //  2 - Test intern 1ms - Task by 1000 ms
											 //  3 - Test Switch "=" up / down (analog)
											 //  4 - Test Switchnumber down (digital)
											 //  6 - Test Pendulum Time - Sinus 0.5 Hz
											 //  7 - get_Expo
											 //  8 - get mem_stack "=" Get_Mantisse();
											 //  9 - Display_Number();
											 // 11 - Error_Test;
											 // 12 - Memory_Plus Test;
											 // 13 - Protokoll Statepoint
											 // 16 - Expand Test
											 // 17 - Display_Status Test
											 // 18 - sqrt(x) : in 1.00 .. 100.00 _>_ out 1.00 .. 10.00
											 // 19 - 5 x sqrt(2)
											 // 20 - reduce test
											 // 21 - "=" - Output -- mem_stack_calc()
											 // 22 - reduce test spezial
											 // 23 - On - Off - Test
											 // 25 - Test Switchnumber down (digital) _ spezial
											 // 26 - Pint_pos Test
											 // 27 - cbrt(x) : in 1.0 .. 1000.0 _>_ out 1.00 .. 10.00
											 // 28 - Test cbrt steps
											 // 29 - log2(x) : in 1.9 .. 1024.0 _>_ out 1.00 .. 10.0
											 // 30 - log(x)  : in 0.3 .. 3.3 _>_ out -1.20 .. 1.20 Step 0.001
											 // 31 - Test Error
											 // 32 - 2^x Test  0,00000000100 .. 320
											 // 33 - log2 Test out -> 0,00000000100 .. 320
											 // 34 - factorial Test 0 .. 71.00 Step +0.01
											 // 35 - DS18B20 (0x28) Digital Thermometer, 12bit
											 // 36 - tanh Test 0.000010 .. 100.
											 // 37 - tanh Test -22.000 .. +22.001 Step 0.003.
											 // 38 - atanh Test 0.0 to 1.0    10 x  540       = 5400
											 // 39 - acosh Test 1.0 to 10.995  8 x 1800 + 200 = 14600
											 // 40 - Test Cordic
											 // 41 - sin Test  0 .. 90.000 Step +0.005  = 18000 - 100 ms
											 // 42 - tan Test  0.005 .. 89.995 Step +0.005  = 18000 - 100 ms
											 // 43 - log_ Test
											 // 44 - atan Test 0.0001 .. 1.0000 Step +0.0001
											 // 45 - atan Test 0.001  .. 10.000 Step +0.001
											 // 46 - atan Test 0.3332 .. 0.7500 Step +0.00004 -- cordic Test
											 // 51 - display_string
											 // 52 - test exp2()
											 // 53 - table log_tab[]
											 // 54 - Display_Memory_x( x )
											 // 55 - cordic shorter
											 // 56 - % Test
											 // 57 - new log2() Test
											 // 58 - Test exp()
											 // 59 - Test view 
											 // 60 - Test input( DISP_C ) or input( DISP_F )
											 // 61 - Test algorithm sqrt(e)
											 // 62 - Reduce_Number( ) - odd-odd continued fraction
											 // 63 - BitBool_test

#define sin_    3
#define cos_    2
#define tan_    1
#define atan_  -1
#define acos_  -2
#define asin_  -3

#define DISP_C -1
#define DISP_F  1

uint8_t mem_pointer        =  1;  //     mem_stack 0 .. 39
#define mem_stack_max_c      39   // 39  Variable in calculate
uint8_t mem_stack_max      =  mem_stack_max_c;   //  2 = Start in Standardmode 
uint8_t Std_mode           =  1;  //  ( xx ) implement  "Sci_mode"
 int8_t display_digit      =  5;  // 5 Digit nachfolgende 0
 int8_t display_digit_temp =  5;  // 5 Digit nachfolgende 0

#define operation_test_max    4  //  0 .. 3  Stacktiefe
#define Switch_down_max    1200  //                  140  %
#define Switch_down_start  1000  //  1020  ... 1020  100  %   ... 1000  <<--
#define Switch_up_b         800  //   750  ..   840   60  %   ...  800
#define Switch_down_b       600  //   570  ..   600   20  %   ...  600
#define Switch_up_start     400  //   360  ...  360    0  %   ...  500  <<--
#define Switch_down_extra   200  //                  -20  %   ...  400
																 //                  -40  %   ...  200
#define Average               9  //   4.5
#define Max_Buffer           24  //   Count of Switches

#define Beep_p A0      //  A0   Pin A7
#define Beep_m A4      //  A4   Pin A3
#define PWM    13      //     Sanguino  Pin 12 --> Pin 4 (PD4)
#define PWM_s  12      //
uint8_t PWM_Pin = PWM;
/*
#define SDA    17      //     Pin 17
#define SCL    16      //     Pin 16
*/
#define A_0    A7      //  A7   Pin A0
#define A_1    A6      //  A6   Pin A1
#define A_2    A5      //  A5   Pin A2

#define Out_A  27      //  A3   Pout A3
#define Out_B  26      //  A2   Pout A2
#define Out_C  25      //  A1   Pout A1

#define Rx_1          10
#define Tx_1          11
#define On_Off_PIN    18
bool    Power_on = true;
bool    switch_Pos_0 = false;

#define Digit_Count   15
constexpr uint8_t _i2caddr = 0x18;  // Adafruit_MCP9808 default adress
constexpr uint8_t index_display[Digit_Count] = {               //   Standard
//   0,  1,  2,  3, 14, 13,  4,  5,  6,  7, 23, 22, 21, 20, 19    //   old Project
//   0,  1,  3,  6,  7, 10, 11, 13, 14, 15, 19, 20, 21, 22, 23    //
//  14, 15, 19, 20, 21, 22, 23, 13, 11,  7, 10,  3,  6,  0,  1    // Main_CPU
		14, 15, 19, 20, 21, 22, 23, 13,  4,  7,  2,  3,  6,  0,  1    // new Main_CPU
};

#define Min_Out        0   // D2 and D4, D5 V-USB not used
#define Max_Out       23

#define Switch_Count  24

#define count_ascii  112
#define start_ascii   16
#define ascii_count   32

uint16_t taste[Switch_Count] = {
	Switch_down_start
};

uint32_t time_start;
uint32_t time_end;
uint32_t time_diff;

// ... up to 32 Switch possible - per bit a switch
uint32_t Switch_up         = 0;  // change Low --> High
uint32_t Switch_down_ptr   = 0;  // change High --> Low
uint32_t Switch_down       = 0;  // change High --> Low
uint32_t Switch_new        = 0;
uint32_t Switch_old        = 0;
uint32_t Switch_old_temp   = 0;
uint32_t Switch_delta      = 0;

uint16_t Switch_test   = Switch_down_start;
uint16_t Switch_read   = Switch_down_start;  // analog
uint8_t  Switch_number = 0;

uint8_t  Switch_up_down = 0;  // Bit 0 to 7 - Multiswitch

uint32_t time          = 0;
uint32_t time_down     = 0;
uint32_t time_old      = 0;
uint32_t time_is       = 0;
uint32_t time_test     = 0;

#define expo_max_input  90
#define expo_min_input -90

#define expo_max_in_3   33
#define expo_min_in_3  -33

#define expo_max       102
#define expo_min       -99

#define M_plus_0   134
#define M_minus_0  224
#define M_mul_0    233
#define M_div_0    242
#define Min_0      160
#define M_xch_0    178
#define MR_0        77
#define FIX_0       93
#define to_0       191
#define to_9       200
#define EE_0       201   // Beep
#define EE_9       210
#define p_p        211
#define input_0     48

#define int32_max_2  0x100000000ULL  // 4294967296 = 2^32
#define int32_max     2147302920     // = int32_2_max * int32_2_max - 1
#define int32_2_max        46339     // = sqrt(int32_max + 1)
#define int30_max     1073651460     // = int32_max / 2
#define int15_max          32767     // = 2^15 - 1
#define uint62_max 0x4000000000000000ULL  // 4611686018427387904 = 2^62

Ratio_32       calc_32       = {0, int32_max, int32_max, 0};
Ratio_32       calc_32_e     = {0, int32_max, int32_max, 0};
Ratio_32       temp_32_e     = {0, int32_max, int32_max, 0};
Ratio_32       temp_32_e_mul = {0, int32_max, int32_max, 0};
Ratio_32       test_32       = {0, int32_max, int32_max, 0};
Ratio_32       temp_32       = {0, int32_max, int32_max, 0};
Ratio_32       temp_32_exp   = {0, int32_max, int32_max, 0};
Ratio_32       temp_32_a     = {0, int32_max, int32_max, 0};
Ratio_32       temp_32_b     = {0, int32_max, int32_max, 0};
Ratio_32       temp_32_a1    = {0, int32_max, int32_max, 0};
Ratio_32       temp_32_b1    = {0, int32_max, int32_max, 0};
Ratio_32       temp_32_b2    = {0, int32_max, int32_max, 0};
Ratio_32       temp_32_cbrt  = {0, int32_max, int32_max, 0};
Ratio_32       temp_32_xxx   = {0, int32_max, int32_max, 0};
Ratio_32       temp_32_xxx_  = {0, int32_max, int32_max, 0};

Ratio_32       temp_32_log_a = {0, int32_max, int32_max, 0};
Ratio_32       temp_32_log_b = {0, int32_max, int32_max, 0};
Ratio_32       temp_32_log_1 = {0, int32_max, int32_max, 0};
Ratio_32       temp_32_log_x = {0, int32_max, int32_max, 0};
Ratio_32       b             = {0, int32_max, int32_max, 0};;

Ratio_32       temp_32_log   = {0, int32_max, int32_max, 0};
Ratio_32       temp_32_log2  = {0, int32_max, int32_max, 0};
Ratio_32       temp_32_ext   = {0, int32_max, int32_max, 0};
Ratio_32       temp_32_pow   = {0, int32_max, int32_max, 0};
Ratio_32 Tau_temp_32_corr_a  = {0, int32_max, int32_max, 0};
Ratio_64       temp_64       = {0, int32_max, int32_max};
Ratio_32_plus  temp_32_plus  = {0, int32_max, int32_max, ' ', 0, 0};

int8_t   cordic_test      = 0;

int8_t  temp_expo  = 0;          // <-- expo
int32_t temp_num   = int32_max;  // <-- numerator
int32_t temp_denom = int32_max;  // <-- denominator
uint8_t temp_op    = 0;          // <-- operation

int32_t temp_xxx   = 0;          // _=_

int96_a num_test    = 0;
int96_a denom_test  = 0;
int64_t num_test_64 = 0;

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

constexpr uint32_t expo_10[10] = {
	expo_10_0, expo_10_1, expo_10_2, expo_10_3, expo_10_4,
	expo_10_5, expo_10_6, expo_10_7, expo_10_8, expo_10_9 };

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

constexpr uint64_t expo_10_[10] = {
	expo_10_10, expo_10_11, expo_10_12, expo_10_13, expo_10_14,
	expo_10_15, expo_10_16, expo_10_17, expo_10_18, expo_10_19 };

constexpr uint8_t cordic_tab[] = {
	0x90, 0x3B, 0x58, 0xCE, 0x0A, 0xC3, 0x76, 0x9E, 0xCF,  //  0 - 4276394391812611791
	0x81, 0xAF, 0x0E, 0xF3, 0xCA, 0xC5, 0x8D, 0xFA,        //  1 -  121332155204079098
	0x80, 0x3D, 0x22, 0x50, 0xBF, 0xD7, 0x48, 0x32,        //  2 -   17207703790635058
	0x77, 0xE8, 0x47, 0x20, 0x33, 0x9F, 0x00,              //  3 -    2225717017550592
	0x70, 0xFF, 0x40, 0x8F, 0x8F, 0x07, 0x5D,              //  4 -     280652751505245
	0x70, 0x1F, 0xFA, 0x01, 0x1F, 0xC7, 0x61,              //  5 -      35158621144929
	0x63, 0xFF, 0xD0, 0x02, 0x3F, 0xE4,                    //  6 -       4397241352164
	0x60, 0x7F, 0xFE, 0x80, 0x04, 0x81,                    //  7 -        549730649217
	0x5F, 0xFF, 0xF4, 0x00, 0x09,                          //  8 -         68718690313
	0x51, 0xFF, 0xFF, 0xA0, 0x01,                          //  9 -          8589910017
	0x50, 0x3F, 0xFF, 0xFD, 0x00,                          // 10 -          1073741056
	0x47, 0xFF, 0xFF, 0xE8,                                // 11 -           134217704
	0x41, 0x00, 0x00, 0x00,                                // 12 -            16777216
	0x40, 0x20, 0x00, 0x00,                                // 13 -             2097152
	0x34, 0x00, 0x01,                                      // 14 -              262145
	0x30, 0x80, 0x00,                                      // 15 -               32768 
	0x30, 0x10, 0x01,                                      // 16 -                4097
	0x22, 0x00,                                            // 17 -                 512
	0x20, 0x41,                                            // 18 -                  65
	0x18,                                                  // 19 -                   8
	0x12,                                                  // 20 -                   2
	0x00};                                                 // 21 -                   0
	                      
																							//  9214364837600034815 = 2^63 - 2^53 - 1
#define expo_test_9          0x1129C0652ULL   //           4607182418 x 0,0000000005
#define expo_test_6        0x430B178B370ULL   //        4607182418800 x 0,0000005
#define expo_test_3a     0x68C154C985F06ULL   //     1842872967520006 x 0,0002
#define expo_test_1a   0x28EB851EB851EB8ULL   //   184287296752000696 x 0,02
#define expo_test_0b  0x265CCCCCCCCCCCCCULL   //  2764309451280010444 x 0,3
#define expo_test_000 0xFFBFFFFFFFFFFFFEULL   // 18428729675200069630 x 2

// ---  artand_1_3()_Konstante  ---
#define artand_1_3_expo            1
#define artand_1_3_num     883496917
#define artand_1_3_denom   479251082  // Fehler ..  5,52e-19
constexpr Ratio_32 artand_1_3 = { artand_1_3_expo, artand_1_3_num, artand_1_3_denom, 0};

// ---  artan_1_3()_Konstante  ---
#define artan_1_3_expo            0
#define artan_1_3_num     428408757
#define artan_1_3_denom  1331493454  // Fehler ..  2,20e-19
constexpr Ratio_32 artan_1_3 = { artan_1_3_expo, artan_1_3_num, artan_1_3_denom, 0};

// ---  sqrt(10)_Konstante  ---
#define sqrt_10_expo            0
#define sqrt_10_num    1499219281
#define sqrt_10_denom   474094764  // Fehler ..  7.03e-19

constexpr Ratio_32 sqrt_10_plus  = {0, sqrt_10_num, sqrt_10_denom, 0};
constexpr Ratio_32 sqrt_10_minus = {1, sqrt_10_denom, sqrt_10_num, 0};

// ---  exp2()_Konstante  ---
#define num_exp2_1_32      671032160
#define denum_exp2_1_32   2147302912  // 1/32
constexpr Ratio_32 exp2_1_32   = { -1, num_exp2_1_32, denum_exp2_1_32, 0};

#define num_exp2_0_1     2147302920
#define denum_exp2_0_1   2147302920   // 1/1
constexpr Ratio_32 exp2_0_1   = { 0, num_exp2_0_1, denum_exp2_0_1, 0};

#define num_exp2_1_4      536825730
#define denum_exp2_1_4   2147302920   // 1/4
constexpr Ratio_32 exp2_1_4   = { 0, num_exp2_1_4, denum_exp2_1_4, 0};

#define num_exp2__1_4     -536825730
#define denum_exp2__1_4   2147302920   // -1/4
constexpr Ratio_32 exp2__1_4   = { 0, num_exp2__1_4, denum_exp2__1_4, 0};

#define num_exp2_1_2     1073651460
#define denum_exp2_1_2   2147302920   // 1/2
constexpr Ratio_32 exp2_1_2   = { 0, num_exp2_1_2, denum_exp2_1_2, 0};
constexpr Ratio_32 exp2_1_2_div_x   = { 0, denum_exp2_1_2, num_exp2_1_2, 0};

#define num_exp2_1_3      715767640
#define denum_exp2_1_3   2147302920   // 1/3
constexpr Ratio_32 exp2_1_3   = { 0, num_exp2_1_3, denum_exp2_1_3, 0};

#define num_exp2__1_3     -715767640
#define denum_exp2__1_3   2147302920   // - 1/3
constexpr Ratio_32 exp2__1_3   = { 0, num_exp2__1_3, denum_exp2__1_3, 0};

#define num_exp2_1_6     2147302920
#define denum_exp2_1_6   1288381752   // 1/6
constexpr Ratio_32 exp2_1_6   = { -1, num_exp2_1_6, denum_exp2_1_6, 0};

// ---  log2()_Konstante  ---
#define num_lb_to_e       497083768
#define denum_lb_to_e     717140287   // Fehler .. -2,02-19
constexpr Ratio_32  lb_to_e   = { 0, num_lb_to_e, denum_lb_to_e, 0};

// ---  log10()_Konstante  ---
#define num_lb_to_10      579001193
#define denum_lb_to_10   1923400330   // Fehler ..  2,09e-20
constexpr Ratio_32 lb_to_10   = { 0, num_lb_to_10, denum_lb_to_10, 0};

// ---  ln_to_10_Konstante  --- 
#define num_ln_to_10     1784326399
#define denom_ln_to_10    774923109   // Fehler .. 2,956e-20
constexpr Ratio_32 ln_to_10   = {0, num_ln_to_10, denom_ln_to_10, 0};

// ---  log()_Konstante  ---
#define int32_max_125   1717842336     // = int32_max / 1.25
#define int32_max_25     858921168     // = int32_max / 2.5
#define int32_max_16    1288381752     // = int32_max * 0.6
#define num_log5e8      -441827468
#define denum_log5e8     220581553
#define num_log1e9      -265961484
#define denum_log1e9     128339561
constexpr Ratio_32 mul_6_0   = { 1, int32_max_16, int32_max, 0};
constexpr Ratio_32 __1e90    = { 90, int32_max, int32_max, 0};
constexpr Ratio_32 min__1e90 = { 90, -int32_max, int32_max, 0};
constexpr Ratio_32 log_1e0   = { 0, int32_max, int32_max, 0};
constexpr Ratio_32 log__1e0  = { 0, -int32_max, int32_max, 0};

Ratio_32  temp_32_mul       = {0, int32_max, int32_max, 0};
Ratio_32  temp_32_corr      = {0, int32_max, int32_max, 0};
Ratio_32  temp_32_corr_0_1  = {0, int32_max, int32_max, 0};
Ratio_32  temp_32_corr_a    = {0, int32_max, int32_max, 0};
Ratio_32  temp_32_corr_b    = {0, int32_max, int32_max, 0};
Ratio_32  temp_32_corr_c    = {0, int32_max, int32_max, 0};

#define expo_71             2
#define num_71     1524585059
#define denum_71   2147302900   // 71/100
constexpr Ratio_32 test_71   = { expo_71, num_71, denum_71, 0};

// --- sqrt(0.5)_Konstante  ---
#define sqrt_0_5_expo           0
#define sqrt_0_5_num   1311738121
#define sqrt_0_5_denom 1855077841  // Fehler ..  1,5e-19
constexpr Ratio_32 sqrt_0_5   = { 0, sqrt_0_5_num, sqrt_0_5_denom, 0};

// ---  cbrt(10)_Konstante  ---
#define cbrt_10_expo            0
#define cbrt_10_num     265396349
#define cbrt_10_denom   123186073  // Fehler ..  3,9e-18

#define cbrt_100_expo           1
#define cbrt_100_num    123186073
#define cbrt_100_denom  265396349  // Fehler .. -8,3e-18

constexpr Ratio_32 cbrt_10_plus   = { 0, cbrt_10_num, cbrt_10_denom, 0};
constexpr Ratio_32 cbrt_100_plus  = { 1, cbrt_100_num, cbrt_100_denom, 0};

// ---  circle_to cordic  ---  1 Grad  =  160978210179491618,6144888
#define circle_to_expo               20
#define circle_to_num         916970662  //
#define circle_to_denom      1582289134  //
constexpr Ratio_32  circle_to = {circle_to_expo, circle_to_num, circle_to_denom, 0};

// ---  circle_Konstante (circle) ---  360
#define circle_expo                3
#define circle_num         773029044  //
#define circle_denom      2147302900  //
constexpr Ratio_32  circle = {circle_expo, circle_num, circle_denom, 0};
constexpr Ratio_32  circle_div_x = {-circle_expo, circle_denom, circle_num, 0};

// ---  circle_Konstante (circle) ---  180
#define circle_2_expo                2
#define circle_2_num        2147302800  //
#define circle_2_denom      1192946000  //
constexpr Ratio_32  circle_2 = {circle_2_expo, circle_2_num, circle_2_denom, 0};

// ---  circle_Konstante (circle) ---  90
#define circle_4_expo                2
#define circle_4_num        1932572610  //
#define circle_4_denom      2147302900  //
constexpr Ratio_32  circle_4 = {circle_4_expo, circle_4_num, circle_4_denom, 0};

// ---  Tau_Konstante (2_Pi) ---
#define Tau_expo                1
#define Tau_num        1068966896  //
#define Tau_denom      1701313655  // Fehler .. -9,77e-19
constexpr Ratio_32   Tau = {Tau_expo, Tau_num, Tau_denom, 0};
constexpr Ratio_32   Tau_div_x = {-Tau_expo, Tau_denom, Tau_num, 0};

// ---  Pi_Konstante  ---
#define Pi_expo                 0
#define Pi_num         2137933792
#define Pi_denom        680525462  // Fehler ..  -9,77e-19
constexpr Ratio_32   Pi      = {Pi_expo, Pi_num, Pi_denom, 0};
constexpr Ratio_32   Pi_neg  = {Pi_expo, -Pi_num, Pi_denom, 0};

// ---  Pi_Konstante (Pi_2)  ---
#define Pi_2_expo               0
#define Pi_2_num       2137933792
#define Pi_2_denom     1361050924  // Fehler ..  -9,77e-19
constexpr Ratio_32  Pi_2 = {Pi_2_expo, Pi_2_num, Pi_2_denom, 0};

// ---  e_Konstante  ---
#define e_expo                  0
#define e_num          1696912706
#define e_denom         624259298  // Fehler .. -6.03e-19
constexpr Ratio_32  one_e = {e_expo, e_num, e_denom, 0};
constexpr Ratio_32  one_e_div_x = {e_expo, e_denom, e_num, 0};

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
constexpr Ratio_32  fa_0 = {fa_0_expo, fa_0_num, fa_0_denom, 0};

// ---  fa_1_Konstante  ---     1/30
#define fa_1_expo            -1 // 0,033333
#define fa_1_num      692499400
#define fa_1_denom   2077498200 //
constexpr Ratio_32  fa_1 = {fa_1_expo, fa_1_num, fa_1_denom, 0};

// ---  fa_2_Konstante  ---     53/210
#define fa_2_expo            -1 // 0,252381
#define fa_2_num     2077498240
#define fa_2_denom    823159680 //
constexpr Ratio_32  fa_2 = {fa_2_expo, fa_2_num, fa_2_denom, 0};

// ---  fa_3_Konstante  ---     195/371
#define fa_3_expo             0 // 0,525606
#define fa_3_num     1091946960
#define fa_3_denom   2077499088
constexpr Ratio_32  fa_3 = {fa_3_expo, fa_3_num, fa_3_denom, 0};

// ---  fa_4_Konstante  ---     22999/22737
#define fa_4_expo             0 // 1,011523
#define fa_4_num     2077499670
#define fa_4_denom   2053833210
constexpr Ratio_32  fa_4 = {fa_4_expo, fa_4_num, fa_4_denom, 0};

// ---  fa_5_Konstante  ---     29944523/19733142
#define fa_5_expo             0 // 1,517474
#define fa_5_num     2036227564
#define fa_5_denom   1341853656
constexpr Ratio_32  fa_5 = {fa_5_expo, fa_5_num, fa_5_denom, 0};

// ---  fa_6_Konstante  ---     109535241009/48264275462
//                              19976135/8802041      -> Fehler 1,037e-18
#define fa_6_expo             0 // 2,269489
#define fa_6_num     2077518040
#define fa_6_denom    915412264
constexpr Ratio_32  fa_6 = {fa_6_expo, fa_6_num, fa_6_denom, 0};

// ---  fa_7_Konstante  ---     29404527905795295658/
//                              9769214287853155785
//                              527522494/175261453  -> Fehler  1,531e-18
#define fa_7_expo             1 // 3,009917
#define fa_7_num      527522494
#define fa_7_denom   1752614530
constexpr Ratio_32  fa_7 = {fa_7_expo, fa_7_num, fa_7_denom, 0};

#define fa_x_max              7
constexpr Ratio_32 fa_x[] = { fa_7, fa_6, fa_5, fa_4, fa_3, fa_2, fa_1 };

// ---  fa_ln_2pi_2_Konstante  ---
#define fa_ln_2pi_2_expo             0
#define fa_ln_2pi_2_num     1474345081
#define fa_ln_2pi_2_denom   1604400107 // Fehler .. +2,016e-19
constexpr Ratio_32  fa_ln_2pi_2 = {fa_ln_2pi_2_expo, fa_ln_2pi_2_num, fa_ln_2pi_2_denom, 0};

// ---  Null_no_Konstante  ---
#define Null_no_expo            0
#define Null_no_num             0
#define Null_no_denom  2147302920  //
constexpr Ratio_32 Null_no = {Null_no_expo, Null_no_num, Null_no_denom, 0};

char    expo_temp_str[]    = "#00";
int8_t  expo_temp_8        =  1;   // atanh()

char    Expo_string_temp[] = "###" ;
 int16_t expo_temp_16           = 0;
 int16_t expo_temp_16_a         = 0;
 int16_t expo_temp_16_b         = 0;
 int16_t expo_temp_16_diff      = 0;
 int16_t expo_temp_16_diff_abs  = 0;

	uint32_t x0_a_32;
	uint32_t x0_b_32;

 int64_t calc_temp_64_b1    = 1;
 int64_t calc_temp_64_c1    = 1;

uint64_t calc_temp_u64_0 = 1;
uint64_t calc_temp_u64_1 = 1;
uint64_t calc_temp_u64_2 = 1;

uint32_t num_temp_u32    = 1;
uint32_t denom_temp_u32  = 1;
uint32_t num_temp_u32_   = 1;
uint32_t denom_temp_u32_ = 1;
uint32_t mul_temp_u32    = 1;
uint32_t calc_temp_u32   = 1;
 int32_t mul_temp_32     = 1;
 int32_t gcd_temp_32     = 1;
 int32_t calc_temp_32_0  = 1;
 int32_t calc_temp_32_1  = 1;
 int32_t calc_temp_32_2  = 1;

// int16_t calc_temp_16_0  = 1;
 int16_t calc_temp_16_1  = 1;
 int16_t calc_temp_16_2  = 1;
 int16_t calc_temp_16_3  = 1;

 constexpr int16_t denom_aaa = 1000;
 constexpr int16_t denom_a_4 = 4;

	int8_t  calc_temp_8_0  = 1;
	int8_t  calc_temp_8_1  = 1;
	int8_t  calc_temp_8_2  = 1;

	int8_t  test_signum_8  = 0;

volatile uint64_t num_temp_u64       = 1;
volatile uint64_t denom_temp_u64     = 1;
volatile uint8_t mem_stack_count    =  1;   // actual - mem_stack 1 .. 39
volatile uint8_t temp_operation     =  0;
volatile uint8_t temp_operation_a   =  0;
volatile uint8_t mem_stack_test     =  0;
volatile uint8_t count              =  0;   // Display wait counter
volatile uint8_t count_wait         =  0;   // Display wait counter

Ratio_32      number_stack[mem_stack_max_c + 1] = {     // shunting Stack
	{ 0, int32_max,  int32_max, ' ' }
};

Ratio_32      mem_stack_input[mem_stack_max_c + 1] = {  // before calc
	{ 0, int32_max,  int32_max, ' ' }
};

Ratio_32_plus mem_stack_calc[mem_stack_max_c + 1] = {   // after calc
	{ 0, int32_max,  int32_max, ' ', 0 ,0 }
};

// ---  circle_Tau  ---  360 / Tau  =
#define to_deg           12
#define to_rad           13

constexpr Ratio_32 to_xx[14] = {
	{ -1, 1250000000,  473176473, 0 },   // 0  7  ..  1 Liter =  (1 / 3,785411784) US-gal
	{  1,  473176473, 1250000000, 0 },   // 1  3  ..  1 gallon [US] = 3,785411784 Liter
	{  0,  565007021, 1245627260, 0 },   // 2  2  ..  1 lb = (1 / 2,2046226218488) kg
	{  0, 1245627260,  565007021, 0 },   // 3  8  ..  1 kg = 2,2046226218488 lbs
	{  0, 2147468400, 1334375000, 0 },   // 4  5  ..  1 Miles = 1,609344 km
	{ -2, 2145264442,  844592300, 0 },   // 5  4  ..  to mm
	{  2,  844592300, 2145264442, 0 },   // 6  6  ..  to mil
	{  0, 1334375000, 2147468400, 0 },   // 7  0  ..  to Miles
	{  0, 1191813600, 2145264480, 0 },   // 8  1  ..  to °C  5 / 9
	{  0, 2145264480, 1191813600, 0 },   // 9  9  ..  to °F  9 / 5
	{  1,-2145264480, 1206711270, 0 },   // 10  ..     -160 / 9
	{  1, 2145264480,  670395150, 0 },   // 11  ..        32
	{  2,  853380389, 1489429756, 0 },   // 12  ..     to deg    Fehler -1,003e-19
	{ -2, 1489429756,  853380389, 0 }    // 13  ..     to rad
};

constexpr char mem_str_1[]     = "##########111111111122222222223333333333";
constexpr char mem_str_0[]     = "#123456789012345678901234567890123456789";
																			 // 012345678
constexpr char Error_String_txt[]   = "0IuX[U^Vx";

uint8_t mem_extra_pointer  =   0;   // mem_extra  MR 0 .. MR 9
uint8_t extra_test_13      =   0;
uint8_t mem_extra_test     =   0;   // mem_extra  MR 0 .. MR 9
uint8_t mem_plus_test      =   0;   // m_plus 1 .. m_plus 9
uint8_t fix_extra_test     =   5;   // FIX_2  ..  FIX8
#define mem_extra_max         10    // mem_extra  MS 0 .. MS 9
#define mem_extra_max_4       13    // mem_extra  MS 0 .. MS 9
uint8_t to_extra_test      =   0;   // mem_extra  to_0 .. to_9
bool    to_temperature     = false;
bool    to_extra           = false;
bool    to_extra_          = false;
bool    expo_exchange      = false;
bool    mem_exchange       = false;
bool    mem_save           = false;
bool    Mr_0_test          = false;
bool    test_index         = true;
bool    Test_to_Result     = true;

Ratio_32 mem_extra_stack[ mem_extra_max_4 + 2 ] = {
	{ expo_min_input, int32_max, int32_max, 0 },    // MR  0
	{ 0, 2147395599, 2147395599, 0 },    // MR  1
	{ 0, 2147395598, 1073697799, 0 },    // MR  2
	{ 0, 2147395599,  715798533, 0 },    // MR  3
	{ 1,  858958238, 2147395595, 0 },    // MR  4
	{ 1, 1073697799, 2147395598, 0 },    // MR  5
	{ 1, 1288437357, 2147395595, 0 },    // MR  6
	{ 1, 1503176913, 2147395590, 0 },    // MR  7
	{ 1, 1717916476, 2147395595, 0 },    // MR  8
	{ 1, 1932656031, 2147395590, 0 },    // MR  9
	{ Pi_expo, Pi_num, Pi_denom, 0 },    // const Pi
	{ Tau_expo, Tau_num, Tau_denom, 0 }, // const Tau
	{ expo_min_input, int32_max, int32_max, 0 },    // MR 10 (12)
	{ expo_min_input, int32_max, int32_max, 0 },    // MR 11  M_Plus_temp
	{ 0, 0, int32_max, 0 }               // MR  0  Std_mode  (14)
};

#define led_bright_min    4
#define led_bright_max   10  //  0 -  16 mA
#define led_bright_start  6  //  2 -  47 mA
														 //  3 -  51 mA    +  4 mA
														 //  4 -  58 mA    +  7 mA   +  3 mA
														 //  5 -  68 mA    + 10 mA   +  3 mA
													// xx  6 -  83 mA    + 15 mA   +  5 mA
														 //  7 - 103 mA    + 20 mA   +  5 mA
														 //  8 - 134 mA    + 31 mA   + 11 mA
														 //  9 - 165 mA    + 31 mA      0 mA
														 // 10 - 196 mA    + 31 mA      0 mA

uint8_t led_bright_index = led_bright_start;
uint16_t test_pwm = 173;

constexpr uint16_t led_bright_plus[led_bright_max + 3] = {
	//                4     5    (6)    7     8     9     10
	0, 76,  86, 104, 132,  173,  232,  331,  444,  579,  742,  937,  1023 };
	//   10    18   28   41   59    99   113   135   163   195    86
	//      8    10    13   18    40   14    22    28    32  -109
	//         2     3    5   22   -26     8     6     4  -141

constexpr uint16_t led_bright[led_bright_max + 3] = {
	//               4     5    (6)    7     8     9     10
	0, 48,  61, 81, 108,  146,  199,  290,  397,  527,  685,  873,  1023 };
	//   13   20   27   38    53     91   107   130   158   188    150
	//      7    7   11    15    38    16    23    28    30     -38
	//        0    4     4    23    -22    7     5     2     68
/*
#define Beep_patt_p 0x993264C993264C99ULL   //  16.842 ms -- 1128.125 Hz -- 19x Peak
//      1001100100110010011001001100100110010011001001100100110010011001 -- binaer
#define Beep_patt_m 0x66CD9B366CD9B366ULL   //  16.842 ms -- 1128.125 Hz -- 19x Peak
//      0110011011001101100110110011011001101100110110011011001101100110 -- binaer
*/
#define Beep_patt_p 0x366CD9B366CD9B36ULL   //  16.579 ms -- 1085.714 Hz -- 18x Peak
//      0011011001101100110110011011001101100110110011011001101100110110 -- binaer
#define Beep_patt_m 0x6CD9B366CD9B366CULL   //  16.579 ms -- 1085.714 Hz -- 18x Peak
//      0110110011011001101100110110011011001101100110110011011001101100 -- binaer
#define max_Beep_count  105  //  65   Beep on
#define min_Beep_count -25   // -25   Beep off  = 90

uint8_t Countdown_view_end   = 0;
#define Countdown_view_end_0  190  // 2 x 95 Hz .. 2 Sec
bool    Countdown_view       = false;
bool    is_view              = false;
uint8_t Countdown_OFF = 0;
#define Countdown_Off_3 240   // 156   --  Start_Off
#define Countdown_Off_2 160   // 135   --  Start
#define Countdown_Off_1 120   // 114
#define Countdown_Off_0  20   //  15

uint8_t index_Switch  = 255;      // counter Switch-digit
uint8_t index_LED = 0;            // counter LED-digit
uint8_t index_Display = 0;        // counter Display-digit
// uint8_t index_Display_old = 0;    // counter Display-digit_old
volatile uint8_t index_a = 0;
volatile uint8_t index_b = 0;
volatile uint8_t index_c = 0;
volatile uint8_t index_i = 0;     // count _=_ output
volatile  int8_t index_j = 0;     // count _=_ output
uint8_t index_pendel_a = 0;       // 0 .. 189
uint8_t index_TIME = 255;         // counter Time
#define Time_LOW     263          // 263    t = (263 + 3/19) µs
#define Time_HIGH    264          // 264  1/t = 3800 Hz

bool    Display_rotate = false;
bool    Memory_xch = false;
volatile bool    Beep_on = false;
volatile bool    Beep_on_off = false;
volatile bool    Beep_on_off_temp = false;
volatile int8_t Beep_count = max_Beep_count;

bool    Constant_arithmetic = false;
bool    Found_constant = false;
bool    First_operation = false;
bool    Pendular_on = false;

// uint16_t display_bright = led_bright_max;

constexpr uint8_t led_font[count_ascii] = {
		0,  64,  68,  76,  92, 124, 125, 127, 111, 103,  99,  97,  96,  64,   0,   0,     //  ¦                ¦
		0, 107,  34,   0, 109,  18,  97,   2,  70, 112,  92,  70,  12,  64, 128,  82,     //  ¦ !"#$%&'()*+,-./¦
	 63,   6,  91,  79, 102, 109, 124,   7, 127, 103,   4,  20,  88,  72,  76,  83,     //  ¦0123456789:;<=>?¦
	123, 119, 127,  57,  15, 121, 113,  61, 118,  48,  30, 122,  56,  85,  55,  99,     //  ¦@ABCDEFGHIJKLMNO¦
	 83, 103,  49,  45,   7,  28,  42,  60,  73, 110,  27,  57, 100,  15,  35,   8,     //  ¦PQRSTUVWXYZ[\]^_¦
	 32,  95, 124,  88,  94, 123,  43, 111, 116,  16,  14, 120,  24,  21,  84,  92,     //  ¦`abcdefghijklmno¦
	 83,  53,  80, 108,  70,  29,  43, 106,   1, 102,   3,  24,  29,   5,   1,  54};    //  ¦pqrstuvwxyz{|}~ ¦


uint8_t count_led[8] = {      // 1 .. 7
	0
};

uint8_t display_a[Digit_Count + 1] = {
	0
};

uint8_t display_b[Digit_Count + 1] = {
	0
};
int8_t Digit = 0;
bool    Digit_Test = false;

bool    Rad_in_out = false;
																										// _123456789_123456789_123456789_1
static char display_string[ ascii_count ]           = " -1.2345678##- 12.# # =.       " ;
constexpr char string_end[ ascii_count ]         = " x-_Good  lUc k _ - x.         " ;
constexpr char string_start[ ascii_count ]       = "  _########### ## # # #        " ;
constexpr char display_string_888[ ascii_count ] = " 8.8.8.8.8.8.8.8.8.8.8.8.8.8.8." ;
#define Plus_Minus           1
#define Mantisse_0           2
#define Mantisse_1           3
#define Plus_Minus_Expo__   10
#define Plus_Minus_Expo_    11
#define Plus_Minus_Expo     13     // 12
#define MR_point            14     // 13
#define Expo_1              15     // 14
#define MS_point            12     // 15
#define Expo_0              16     // 16
#define Expo_point          17     // 17
#define Operation           18     // 18
#define Operation_point     19     // 19
#define Memory_1            20     // 20
#define Rad_point           21     // 21
#define Memory_0            22     // 22
#define Beep_point          23

char char_test;               // 0.. 127

uint8_t     index_mem = 255;  // 1..9  ... for MC (Memory's Clear)

bool        time_10ms = false;
uint8_t    index_10ms = 255;  // 0..13
uint32_t   count_10ms = 0;
bool       time_100ms = false;
uint8_t   index_100ms = 255;  // 0..7
uint32_t  count_100ms = 0;
bool      time_1000ms = false;
uint32_t count_1000ms = 0;
bool      time_7500ms = false;
uint8_t  index_7500ms = 255;  // 0..59

uint8_t    index_5min = 255;  // 0..39
#define    time_5min    39  // 39  5_min
#define    pendel_3min  23  // 23  3_min

bool    Init_expo = true;
bool    Display_new = true;
bool    Display_change = false;
bool    Display_mode = false;
#define Std_string_view  14  //
#define Std_string_count  9  //
static char Std_mode_string[ Std_string_view ] = " " ;
// Display_Status_old
volatile uint8_t Cursor_pos = 2 ;       //
volatile uint8_t Point_pos = 0;        //
uint8_t Repeat_pos = 0;       //
uint8_t Null_count = 0;       //
uint8_t Expo_count_temp = 0;
uint8_t Expo_count = 0;       //   Anzahl der Expo_Ziffern  --  maximal 2
uint8_t Number_count_temp = 0;
uint8_t Number_count = 0;     //   Anzahl der Ziffern  --  maximal 8
uint8_t Zero_count = 0;       //   Anzahl der Nullen   --  maximal 8
uint8_t Zero_after_Point = 0; //   Anzahl der Ziffern  --  maximal 7
uint8_t Zero_index = 0;
uint8_t Zero_index_a = 0;
char First_char;              //   0.. 127
char Temp_char[12]     = "           ";
char Temp_char_expo[5] = "    ";
char Char_temp = ' ';

uint8_t Switch_Code = 0;
uint8_t Switch_Code_old = 0;
uint8_t Switch_Test = 0;
uint8_t Start_mem = 0;
uint8_t Mem_Display_old = 0;
volatile uint8_t Display_Status_old = 0;

Display_Status_old[0] = false;   // ( "0" ) Create a bit reference to first bit.
Display_Status_old[1] = false;  // ( "." )
Display_Status_old[2] = false;  // ( "+/-" )
Display_Status_old[3] = false;  // ( "EE" )
Display_Status_old[4] = false;  // ( "FN" )
Display_Status_old[5] = false;  // ( "=" )
Display_Status_old[6] = false;  // ( "M+" )
Display_Status_old[7] = false;  // ( "+ - x /" )

volatile uint8_t Display_Status_new = 0;    // Switch up / down

Display_Status_new[0] = false;   // ( "0" ) Create a bit reference to first bit.
Display_Status_new[1] = false;  // ( "." )
Display_Status_new[2] = false;  // ( "+/-" )
Display_Status_new[3] = false;  // ( "EE" )
Display_Status_new[4] = false;  // ( "FN" )
Display_Status_new[5] = false;  // ( "=" )
Display_Status_new[6] = false;  // ( "M+" )
Display_Status_new[7] = false;  // ( "+ - x /" )

volatile uint8_t Display_Status = 0;
volatile uint8_t Display_Status_test = 0;
volatile uint8_t Display_Status_mem = 0;    // 1, 2, 3, 4

Display_Status_mem[0] = false;   // ( "0" ) Create a bit reference to first bit.
Display_Status_mem[1] = false;  // ( "." )
Display_Status_mem[2] = false;  // ( "+/-" )
Display_Status_mem[3] = false;  // ( "EE" )
Display_Status_mem[4] = false;  // ( "FN" )
Display_Status_mem[5] = false;  // ( "=" )
Display_Status_mem[6] = false;  // ( "M+" )
Display_Status_mem[7] = false;  // ( "+ - x /" )

// BitBool at version 1.2.0
// https://github.com/Chris--A
// https://github.com/Chris--A/BitBool#reference-a-si 


volatile BitBool< 8 > Display_Status_96{ 96 };
// volatile BitBool< 8 > Display_Status_new{ 0 };

Display_Status_new[0] = false;   // ( "0" ) Create a bit reference to first bit.
Display_Status_new[1] = false;  // ( "." )
Display_Status_new[2] = false;  // ( "+/-" )
Display_Status_new[3] = false;  // ( "EE" )
Display_Status_new[4] = false;  // ( "FN" )
Display_Status_new[5] = false;  // ( "=" )
Display_Status_new[6] = false;  // ( "M+" )
Display_Status_new[7] = false;  // ( "+ - x /" )

// uint8_t Display_Memory = 0;    // 0 .. 12
																	 //01234567890123456789012
static char    Display_Memory_1[] = "  mmE]{F mFm=OUX}_F_^Om";
static char    Display_Memory_0[] = "1 r__[zn=+Fc_V,_m_F-O^c";
static char    Memory_p_min_m_d[] = "++-*,r_";

bool    max_input = false;
bool    Mantisse_change = false;
bool    Expo_change = false;
bool    No_change = false;
bool    M_Plus_past = false;
bool    Error_first = false;
bool    Error_first_0 = false;

#define First_Display            0
#define Start_Mode               1
#define Input_Mantisse           2   //     Input Number
#define Input_dms                3   //     Input Number
#define Input_Fraction           4   //     Input Number
#define Input_Expo               5   //     Input Number
#define Input_Memory             6   //     Input Number
#define Input_Operation_0        7   //    Input Operation  no Input           // op_priority
#define Input_Operation_1        8   //    Input Operation  "+"  "-"           // op_priority
#define Input_Operation_2        9   //    Input Operation  "*"  "/"           // op_priority
#define Input_Operation_3       10   //    Input Operation  "//" "/p/" "mod"   // op_priority
#define Input_Operation_4       11   //    Input Operation  "sqrt(x,y)" "x^y" "AGM" "HM" "GM" "AM"
#define Input_Operation_5       12   //    Input Operation  "("  ")"           // op_priority
#define Display_Input_Error     13   //   Error Number
#define Display_Error           14   //   Error Number
#define Display_Result          15   //  Display Number
#define M_Plus_spezial          16   //  Display Number
#define Display_M_Plus          17   //  Display Number
#define Display_Fraction_a_b    18   //  Display Number
#define Display_Fraction_a_b_c  19   //  Display Number
#define Display_hms             20   //  Display Number
#define Off_Status              21   //  Off_Status

uint8_t Start_input = First_Display;
uint8_t Start_temp  = First_Display;
char Pointer_memory = '_';

int16_t display_expo = 0;
int64_t display_big = 1;
volatile uint32_t display_number = 1;
#define digit_count_max 8
int16_t display_expo_mod = 0;

uint8_t pgm_count_a = 0;
uint8_t pgm_content_a[3] = { 0 };

void reboot() {
	
	MCUSR = 0;
	wdt_disable();
	wdt_enable(WDTO_120MS);
	
	while (1) {
	}
}

void Print_Operation(uint8_t Switch_up) {
		switch (Switch_up) {

			case 30:                 //    _<--_
				Serial.print("(<<-)");
				break;

			case 32:                 //    FIX_hms
				Serial.print("(FIX_hms)");
				break;

			case 33:                 //    _1/x_
				Serial.print("(1/x)");
				break;

			case 34:                 //    _log2_
				Serial.print("(log2(x))");
				break;

			case 35:                 //    _+/-_
				Serial.print("(+/-)");
				break;

			case 36:                 //    PI()
				Serial.print("(PI())");
				break;

			case 37:                 //    _Deg
				Serial.print("(Deg)");
				break;

			case 38:                 //    _EE+3_
				Serial.print("(EE+3)");
				break;

			case 39:                 //    _EE-3_
				Serial.print("(EE-3)");
				break;

			case 40:                 //    _(_
				Serial.print("(_(_)");
				break;

			case 41:                 //    _)_
				Serial.print("(_)_)");
				break;

			case 42:                 //    _*_
				Serial.print("(_*_)");
				break;

			case 43:                 //    _+_
				Serial.print("(_+_)");
				break;

			case 44:                 //    e()
				Serial.print("(e())");
				break;

			case 45:                 //    _-_
				Serial.print("(_-_)");
				break;

			case 46:                 //    _._
				Serial.print("(_._)");
				break;

			case 47:                 //    _/_
				Serial.print("(_/_)");
				break;

			case 48:                 //    _0_
			case 49:                 //    _1_
			case 50:                 //    _2_
			case 51:                 //    _3_
			case 52:                 //    _4_
			case 53:                 //    _5_
			case 54:                 //    _6_
			case 55:                 //    _7_
			case 56:                 //    _8_
			case 57:                 //    _9_
				Serial.print("(");
				Serial.print(char(Switch_Code));
				Serial.print(")");
				break;

			case 60:                 //    _<--_
				Serial.print("(<--)");
				break;

			case 61:                 //    _=_
				Serial.print("(_=_)");
				break;

			case 64:                 //    _Rad
				Serial.print("(Rad)");
				break;

			case 65:                 //    sin()
				Serial.print("(sin())");
				break;

			case 66:                 //    cos()
				Serial.print("(cos())");
				break;

			case 67:                 //    tan()
				Serial.print("(tan())");
				break;

			case 68:                 //    asin()
				Serial.print("(asin())");
				break;

			case 69:                 //    acos()
				Serial.print("(acos())");
				break;

			case 70:                 //    atan()
				Serial.print("(atan())");
				break;

			case 71:                 //    sinh()
				Serial.print("(sinh())");
				break;

			case 72:                 //    cosh()
				Serial.print("(cosh())");
				break;

			case 73:                 //    tanh()
				Serial.print("(tanh())");
				break;

			case 74:                 //    asinh()
				Serial.print("(asinh())");
				break;

			case 75:                 //    acosh()
				Serial.print("(acosh())");
				break;

			case 76:                 //    atanh()
				Serial.print("(atanh())");
				break;

			case 77:                 //   _MR(0)
			case 78:                 //   _MR(1)
			case 79:                 //   _MR(2)
			case 80:                 //   _MR(3)
			case 81:                 //   _MR(4)
			case 82:                 //   _MR(5)
			case 83:                 //   _MR(6)
			case 84:                 //   _MR(7)
			case 85:                 //   _MR(8)
			case 86:                 //   _MR(9)
				extra_test_13 = Switch_Code - MR_0;
				Serial.print("(MR(");
				Serial.print(extra_test_13);
				Serial.print("))");
				break;

			case 87:                 //    FIX_a_b/c
				Serial.print("(FIX_a_b/c)");
				break;

			case 88:                 //    y_expo
				Serial.print("(y_expo)");
				break;

			case 89:                 //    y_root
				Serial.print("(y_root)");
				break;

			case 90:                 //    _CE_
				Serial.print("(_CE_)");
				break;

			case 91:                 //   "FN"- Light_up
				Serial.print("(Light_up)");
				break;

			case 92:                 //    _//_
				Serial.print("(_//_)");
				break;

			case 93:                 //   "FN"- Light_down
				Serial.print("(Light_down)");
				break;

			case 94:                 //    _/p/_  Phytagoras
				Serial.print("(_/p/_)");
				break;

			case 95:          // FIX_2        10 ..       99 -->  Display
			case 96:          // FIX_3       100 ..      999 -->  Display
			case 97:          // FIX_4      1000 ..     9999 -->  Display
			case 98:          // FIX_5     10000 ..    99999 -->  Display
			case 99:          // FIX_6    100000 ..   999999 -->  Display
			case 100:         // FIX_7   1000000 ..  9999999 -->  Display
			case 101:         // FIX_8  10000000 .. 99999999 -->  Display
				extra_test_13 = Switch_Code - FIX_0;
				Serial.print("(FIX_");
				Serial.print(extra_test_13);
				Serial.print(")");
				break;

			case 102:                //    FIX_E24
				Serial.print("(FIX_E24)");
				break;

			case 135:                //    M_plus(1)
			case 136:                //    M_plus(2)
			case 137:                //    M_plus(3)
			case 138:                //    M_plus(4)
			case 139:                //    M_plus(5)
			case 140:                //    M_plus(6)
			case 141:                //    M_plus(7)
			case 142:                //    M_plus(8)
			case 143:                //    M_plus(9)
				extra_test_13 = Switch_Code - M_plus_0;
				Serial.print("(M_plus(");
				Serial.print(extra_test_13);
				Serial.print("))");
				break;

			case 225:                //    M_minus(1)
			case 226:                //    M_minus(2)
			case 227:                //    M_minus(3)
			case 228:                //    M_minus(4)
			case 229:                //    M_minus(5)
			case 230:                //    M_minus(6)
			case 231:                //    M_minus(7)
			case 232:                //    M_minus(8)
			case 233:                //    M_minus(9)
				extra_test_13 = Switch_Code - M_minus_0;
				Serial.print("(M_minus(");
				Serial.print(extra_test_13);
				Serial.print("))");
				break;

			case 234:                //    M_mul(1)
			case 235:                //    M_mul(2)
			case 236:                //    M_mul(3)
			case 237:                //    M_mul(4)
			case 238:                //    M_mul(5)
			case 239:                //    M_mul(6)
			case 240:                //    M_mul(7)
			case 241:                //    M_mul(8)
			case 242:                //    M_mul(9)
				extra_test_13 = Switch_Code - M_mul_0;
				Serial.print("(M_mul(");
				Serial.print(extra_test_13);
				Serial.print("))");
				break;

			case 243:                //    M_div(1)
			case 244:                //    M_div(2)
			case 245:                //    M_div(3)
			case 246:                //    M_div(4)
			case 247:                //    M_div(5)
			case 248:                //    M_div(6)
			case 249:                //    M_div(7)
			case 250:                //    M_div(8)
			case 251:                //    M_div(9)
				extra_test_13 = Switch_Code - M_div_0;
				Serial.print("(M_div(");
				Serial.print(extra_test_13);
				Serial.print("))");
				break;

			case 112:                //    log(x)
				Serial.print("(log(x))");
				break;

			case 113:                //    e^x
				Serial.print("(e^x)");
				break;

			case 114:                //    2^x
				Serial.print("(2^x)");
				break;

			case 115:                //    log10(x)
				Serial.print("(log10(x))");
				break;

			case 116:                //    10^x
				Serial.print("(10^x)");
				break;

			case 117:                //    _x^2_
				Serial.print("(x^2)");
				break;

			case 118:                //    _sqrt()_
				Serial.print("(sqrt())");
				break;

			case 119:                //    _x!_
				Serial.print("(x!)");
				break;

			case 120:                //    _EE_
				Serial.print("(EE)");
				break;

			case 121:                //    clock
				Serial.print("(clock)");
				break;

			case 122:                //    Beep_On_Off
				Serial.print("(Beep_On_Off)");
				break;

			case 123:                //    Off
				Serial.print("(OFF)");
				break;

			case 124:                //    HM
				Serial.print("(HM)");
				break;

			case 125:                //    Off after  5min
				Serial.print("(OFF_5min)");
				break;

			case 127:                 //    -->
				Serial.print("(-->)");
				break;

			case 128:                 //   _AM_
				Serial.print("(AM)");
				break;

			case 129:                 //   _GM_
				Serial.print("(GM)");
				break;

			case 144:                 //    view_down
				Serial.print("view_down");
				break;

			case 145:                 //    view_up
				Serial.print("view_up");
				break;

			case 146:                 //    view_end
				Serial.print("view_end");
				break;

			case 147:                 //    fail_view
				Serial.print("fail_view");
				break;

			case 148:                 //    ->>
				Serial.print("(->>)");
				break;

			case 149:                 //    __/
				Serial.print("(__/)");
				break;

			case 150:                 //    Â° ' ''
				Serial.print("(o_-_=)");
				break;

			case 151:                 //    _AGM_
				Serial.print("AGM");
				break;

			case 152:                 //    _mod_
				Serial.print("mod");
				break;

			case 153:                 //    _Std_down_
				Serial.print("Std_on_off");
				break;

			case 154:                 //    _Std_up_
				Serial.print("Std_on_off_up");
				break;

			case 155:                 //    _DISP_Â°C_
				Serial.print("DISP_Â°C");
				break;

			case 156:                 //    _DISP_Â°F_
				Serial.print("DISP_Â°F");
				break;

			case 157:                 //    _rpt_pt_
				Serial.print("rpt_pt");
				break;

			case 160:                 //   SM(MCs)
				Serial.print("(MCs)");
				break;

			case 161:                 //   _MS(1)
			case 162:                 //   _MS(2)
			case 163:                 //   _MS(3)
			case 164:                 //   _MS(4)
			case 165:                 //   _MS(5)
			case 166:                 //   _MS(6)
			case 167:                 //   _MS(7)
			case 168:                 //   _MS(8)
			case 169:                 //   _MS(9)
				extra_test_13 = Switch_Code - Min_0;
				Serial.print("(MS(");
				Serial.print(extra_test_13);
				Serial.print("))");
				break;

			case 170:                //    Int
				Serial.print("(Int)");
				break;

			case 171:               //    Frac
				Serial.print("Frac");
				break;

			case 172:                //    x^3
				Serial.print("(x^3)");
				break;

			case 173:                //    cbrt()
				Serial.print("(cbrt())");
				break;

			case 174:                //    EE+1
				Serial.print("(EE+1)");
				break;

			case 175:                //    EE-1
				Serial.print("(EE-1)");
				break;

			case 176:                //    Dis_Cha_Dir_on
				Serial.print("(Dis_Cha_Dir_on)");
				break;

			case 177:                //    Dis_Cha_Dir_off
				Serial.print("(Dis_Cha_Dir_off)");
				break;

			case 179:                //   _M_xch(1)
			case 180:                //   _M_xch(2)
			case 181:                //   _M_xch(3)
			case 182:                //   _M_xch(4)
			case 183:                //   _M_xch(5)
			case 184:                //   _M_xch(6)
			case 185:                //   _M_xch(7)
			case 186:                //   _M_xch(8)
			case 187:                //   _M_xch(9)
				extra_test_13 = Switch_Code - M_xch_0;
				Serial.print("(M-xch(");
				Serial.print(extra_test_13);
				Serial.print("))");
				break;

			case 188:                //    Dis_Memory_X_off
				Serial.print("(Dis_Memory_X_off)");
				break;

			case 190:                //    rnd(x)
				Serial.print("(rnd(x))");
				break;

			case 191:                //   _to_xx(0)
			case 192:                //   _to_xx(1)
			case 193:                //   _to_xx(2)
			case 194:                //   _to_xx(3)
			case 195:                //   _to_xx(4)
			case 196:                //   _to_xx(5)
			case 197:                //   _to_xx(6)
			case 198:                //   _to_xx(7)
			case 199:                //   _to_xx(8)
			case 200:                //   _to_xx(9)
				extra_test_13 = Switch_Code - to_0;
				Serial.print("to_xx(");
				Serial.print(extra_test_13);
				Serial.print("))");
				break;

			case 201:                //    Beep
				Serial.print("(Beep)");
				break;

			case 202:                //    EE(1)
			case 203:                //    EE(2)
			case 204:                //    EE(3)
			case 205:                //    EE(4)
			case 206:                //    EE(5)
			case 207:                //    EE(6)
			case 208:                //    EE(7)
			case 209:                //    EE(8)
			case 210:                //    EE(9)
				extra_test_13 = Switch_Code - EE_0;
				Serial.print("EE(");
				Serial.print(extra_test_13);
				Serial.print(")");
				break;

			case 211:                 //    _/p/ /p/_
				Serial.print("(_/p/ /p/_)");
				break;

			case 212:                 //    _- -_
				Serial.print("(_- -_)");
				break;

			case 213:                 //    _* *_
				Serial.print("(_* *_)");
				break;

			case 214:                 //    _/ /_
				Serial.print("(_/ /_)");
				break;

			case 215:                 //    _// //_
				Serial.print("(_// //_)");
				break;

			case 216:                 //    _+ +_
				Serial.print("(_+ +_)");
				break;

			case 217:                 //    _y_expo y_expo_
				Serial.print("(_y_expo y_expo_)");
				break;

			case 218:                 //    _y_root y_root_
				Serial.print("(_y_root y_root_)");
				break;

			case 219:                 //    _HM HM_
				Serial.print("(_HM HM_)");
				break;

			case 220:                 //    _AM AM_
				Serial.print("(_AM AM_)");
				break;

			case 221:                 //    _GM GM_
				Serial.print("(_GM GM_)");
				break;

			case 222:                 //    _AGM AGM_
				Serial.print("(_AGM AGM_)");
				break;

			case 223:                 //    _mod_mod_
				Serial.print("(_mod mod_)");
				break;

			case 103:                 //    _M+()_
				Serial.print("(_M+()_)");
				break;

			case 104:                 //    _M-()_
				Serial.print("(_M-()_)");
				break;

			case 105:                 //    _Mx()_
				Serial.print("(_M*()_)");
				break;

			case 106:                 //    _M/()_
				Serial.print("(_M/()_)");
				break;

			case 107:                 //    _Mr()_
				Serial.print("(_Mr()_)");
				break;
		
			case 108:                 //    _M_()_
				Serial.print("(_M_()_)");
				break;
		
			case 252:                 //    _+(%)_
				Serial.print("(_+(%)_)");
				break;

			case 253:                 //    _-(%)_
				Serial.print("(_-(%)_)");
				break;

			case 254:                 //    _*(%)_
				Serial.print("(_*(%)_)");
				break;

			case 255:                 //    _/(%)_
				Serial.print("(_/(%)_)");
				break;

			default:
				Serial.print("__not define__");
				break;
		}
}

void Print_Statepoint() {         // Debug_Level == 13
	 /*
		time = millis();
		Serial.print("Time:");
		Serial.print("  ");
		Serial.print(time);
		Serial.print("  ");
	 */

	if ( Debug_Level == 13 ) {

		Print_Operation( Switch_Code );

		Serial.print("  ");

		switch (Start_input) {

			case First_Display:           //
				Serial.print("First_Display");
				break;

			case Start_Mode:              //
				Serial.print("Start_Mode");
				break;

			case Input_Mantisse:          //    Input Number
				Serial.print("Input_Mantisse");
				break;

			case Input_Expo:              //    Input Number
				Serial.print("Input_Expo");
				break;

			case Input_dms:               //    Input Number
				Serial.print("Input_dms");
				break;

			case Input_Fraction:          //    Input Number
				Serial.print("Input_Fraction");
				break;

			case Input_Memory:            //    Input_Memory
				Serial.print("Input_Memory");
				break;

			case Input_Operation_0:       //    Input Operation  no Input
				Serial.print("Input_Operation_0");
				break;

			case Input_Operation_1:       //    Input Operation  "+"  "-"
				Serial.print("Input_Operation_1");
				break;

			case Input_Operation_2:       //    Input Operation  "*"  "/"
				Serial.print("Input_Operation_2");
				break;

			case Input_Operation_3:       //    Input Operation  "//"  "/p/"  "mod"
				Serial.print("Input_Operation_3");
				break;

			case Input_Operation_4:       //    Input Operation  "sqrt(x,y)"  "x^y"  "HM"  "GM"  "AM"  "AGM"
				Serial.print("Input_Operation_4");
				break;

			case Input_Operation_5:       //    Input Operation  "("  ")"
				Serial.print("Input_Operation_5");
				break;

			case Display_Input_Error:     //    Display Number
				Serial.print("Display_Input_Error");
				break;

			case Display_Error:           //    Display Number
				Serial.print("Display_Error");
				break;

			case Display_Result:          //    Display Number
				Serial.print("Display_Result");
				break;

			case M_Plus_spezial:          //    Display Number
				Serial.print("M_Plus_spezial");
				break;

			case Display_M_Plus:          //    Display Number
				Serial.print("Display_M_Plus");
				break;

			case Display_Fraction_a_b:    //    Display Number
				Serial.print("Display_Fraction_a_b");
				break;

			case Display_Fraction_a_b_c:  //    Display Number
				Serial.print("Display_Fraction_a_b_c");
				break;

			case Display_hms:             //    Display Number
				Serial.print("Display_hms");
				break;

			case Off_Status:              //    Off Status
				Serial.print("Off_Status");
				break;

			default:
				Serial.println("__not define__");
				break;
		}
	}
}

void Print_Statepoint_after() {   // Debug_Level == 13
	if ( Debug_Level == 13 ) {

		Serial.print(" --> ");

		switch (Start_input) {

			case First_Display:           //
				Serial.println("First_Display");
				break;

			case Start_Mode:              //
				Serial.println("Start_Mode");
				break;

			case Input_Mantisse:          //    Input Number
				Serial.println("Input_Mantisse");
				break;

			case Input_Expo:              //    Input Number
				Serial.println("Input_Expo");
				break;

			case Input_dms:               //    Input Number
				Serial.println("Input_dms");
				break;

			case Input_Fraction:          //    Input Number
				Serial.println("Input_Fraction");
				break;

			case Input_Memory:            //    Input_Memory
				Serial.println("Input_Memory");
				break;

			case Input_Operation_0:       //   Input Operation  no Input
				Serial.println("Input_Operation_0");
				break;

			case Input_Operation_1:       //   Input Operation  "+"  "-"
				Serial.println("Input_Operation_1");
				break;

			case Input_Operation_2:       //   Input Operation  "*"  "/"
				Serial.println("Input_Operation_2");
				break;

			case Input_Operation_3:       //   Input Operation  "//"  "/p/"
				Serial.println("Input_Operation_3");
				break;

			case Input_Operation_4:       //   Input Operation  "sqrt(x,y)"  "x^y"  "HM"  "GM"  "AM"  "AGM"
				Serial.println("Input_Operation_4");
				break;

			case Input_Operation_5:       //   Input Operation  "("  ")"
				Serial.println("Input_Operation_5");
				break;

			case Display_Input_Error:     //  Display Number
				Serial.println("Display_Input_Error");
				break;

			case Display_Error:           //  Display Number
				Serial.println("Display_Error");
				break;

			case Display_Result:          //  Display Number
				Serial.println("Display_Result");
				break;

			case M_Plus_spezial:          //  Display Number
				Serial.println("M_Plus_spezial");
				break;

			case Display_M_Plus:          //  Display Number
				Serial.println("Display_M_Plus");
				break;

			case Display_Fraction_a_b:    //  Display Number
				Serial.println("Display_Fraction_a_b");
				break;

			case Display_Fraction_a_b_c:  //  Display Number
				Serial.println("Display_Fraction_a_b_c");
				break;

			case Display_hms:             //  Display Number
				Serial.println("Display_hms");
				break;

			case Off_Status:              //  Off Status
				Serial.println("Off_Status");
				break;

			default:
				Serial.println("__not define__");
				break;
		}
	}
}

void Test_mem_stack_input() {
	mem_stack_input[ mem_pointer ].op = 'x';
	if ( max_input == false ) {
		mem_stack_input[ mem_pointer ].op = '_';
	}
}
	
void Memory_to_Input_Operation() {
	mem_pointer = mem_stack_count;
	if ( Start_input == Display_Result ) {
		Result_to_Start_Mode();
	}
	if ( Start_input == Display_M_Plus ) {
		Result_to_Start_Mode();
	}
	if ( Start_input < Input_Operation_0 ) {      // Input Number
		if ( mem_extra_test == 0 ) {
			if ( Std_mode == 0 ) {
				left_right_mem_extra( 0, 14 );
			}
			else {
				left_right_mem_extra( 0, 12 );
			}
		}
		else {
			left_right_mem_extra( 0, mem_extra_test );
		}
		Start_input = Input_Memory;
	}
	if ( Start_input == Input_Memory ) {
		if ( mem_pointer > 0 ) {
			if ( mem_extra_test == 0 ) {
				if ( Std_mode == 0 ) {
					left_right_mem_extra( mem_pointer, 14 );
				}
				else {
					left_right_mem_extra( mem_pointer, 12 );
				}
			}
			else {
				left_right_mem_extra( mem_pointer, mem_extra_test );
			}
			mem_pointer = 0;
			Test_mem_stack_input();
			Display_Number(mem_stack_input[mem_pointer]);
			Mantisse_change = false;
			Expo_change = false;
			Init_expo = false;
		}
	}
	display_string[Memory_1] = Display_Memory_1[3];  // m
	display_string[Memory_0] = '0' + mem_extra_test;
	mem_save = true;

	Display_Memory_x( 23 );

	if ( (Start_input == Input_Operation_0) || (Start_input == Input_Memory) || (Start_input == M_Plus_spezial) ) {
		mem_save = false;
		mem_exchange = false;
		Start_input = Input_Operation_0;
		mem_pointer = 0;
	}
}

void Result_to_Start_Mode() {
	Beep__off();
	Constant_arithmetic = false;
	First_operation = false;
	mem_stack_count = 1;
	mem_pointer = mem_stack_count;
	Clear_String();
	Beep__on();
}

void Mux_change(uint8_t change) {
	switch (change) {

		case 1:
			digitalWrite(Out_A, HIGH);
			break;

		case 2:
			digitalWrite(Out_B, HIGH);
			break;

		case 3:
			digitalWrite(Out_A, LOW);
			break;

		case 4:
			digitalWrite(Out_C, HIGH);
			break;

		case 5:
			digitalWrite(Out_A, HIGH);
			break;

		case 6:
			digitalWrite(Out_B, LOW);
			break;

		case 7:
			digitalWrite(Out_A, LOW);
			break;

		case 0:
			digitalWrite(Out_C, LOW);
			time_10ms = true;
			break;
	}

	index_Display = change;
	Display_on();
}

void Display_on() {     // Segmente einschalten --> Segment "a - f - point"
	if ( Power_on == true ) {
		for ( Digit = Digit_Count - 1; Digit >= 0; Digit -= 1 ) {
			if ( display_a[Digit] > 0 ) {
				if ( (bitRead(display_a[Digit], index_Display)) == 1 ) {
					digitalWrite(index_display[Digit], LOW);
				}
			}
		}
	}
}

void Test_pwm() {
	if ( Number_count == 0 ) {
		test_pwm = led_bright[led_bright_index];    // duty cycle goes from 0 to 1023
	}
	else {
		test_pwm = led_bright_plus[led_bright_index];    // duty cycle goes from 0 to 1023
	}
}

void Error_String(uint8_t a) {
	Beep__off();
	Clear_String();
	Start_input = Display_Error;
	strcpy( display_string, string_start );
	display_string[1]  = ' ';
	display_string[2]  = a;
	display_string[3]  = '-';
	display_string[4]  = '#';
	display_string[5]  = 'E';
	display_string[6]  = 'r';
	display_string[7]  = 'r';
	display_string[8]  = 'o';
	display_string[9]  = 'r';
	display_string[10] = ' ';
	display_string[11] = '#';
	display_string[12] = '-';
	display_string[13] = a;
	if ( a == 'I' ) {
		display_string[2] = '1';
	}
	if ( a == '[' ) {
		display_string[2] = ']';
	}
	Beep__on();
}

void Clear_String() {   // String loeschen -- Eingabe Mantisse

	strcpy( display_string, string_start );          // normal

	Display_Memory_x( 25 );

	if ( Rad_in_out == true ) {
		display_string[Rad_point] = '.';
	}
	if ( Beep_on_off_temp == true ) {
		display_string[Beep_point] = '.';
	}
	if ( Std_mode == 0 ) {
		display_string[MR_point] = '.';
	}

//	Cursor_pos = 2;
//	Point_pos = 0;
	Repeat_pos = 0;
	Number_count = 0;
	Zero_count = 0;
	Zero_after_Point = 0;
	Zero_index = 0;
	Zero_index_a = 0;
	Init_expo = true;

	Mantisse_change = false;
	Expo_change = false;

	mem_stack_input[mem_pointer].num = int32_max;
	mem_stack_input[mem_pointer].denom = int32_max;
	mem_stack_input[mem_pointer].expo = expo_min_input;
	mem_stack_input[mem_pointer].op = ' ';  // temp_op

	Display_Memory_x( 23 );

	Start_input = Input_Mantisse;
	Display_new = true;
}

int16_t Get_Expo() {
	Expo_string_temp[0] = display_string[Plus_Minus_Expo];
	Expo_string_temp[1] = display_string[Expo_1];
	Expo_string_temp[2] = display_string[Expo_0];

	if ( Expo_string_temp[0] == '#' ) {
		Expo_string_temp[0] = '0';
	}
	if ( Expo_string_temp[1] == '#' ) {
		Expo_string_temp[1] = '0';
	}
	if ( Expo_string_temp[2] == '#' ) {
		Expo_string_temp[2] = '0';
	}

	if ( Debug_Level == 7 ) {
		Serial.println(Expo_string_temp);
	}
	return atoi(Expo_string_temp);
}

void Put_Expo() {
	if ( expo_temp_16 < 100 ) {
		if ( expo_temp_16 > -100 ) {
			itoa(expo_temp_16, Expo_string_temp, 10);

			if ( Debug_Level == 7 ) {
				Serial.println(Expo_string_temp);
			}

			if ( Expo_string_temp[1] < ' ' ) {
				display_string[Plus_Minus_Expo] = '#';
				display_string[Expo_1] = '0';
				display_string[Expo_0] = Expo_string_temp[0];
			}
			else {
				if ( Expo_string_temp[2] < ' ' ) {
					if ( Expo_string_temp[0] == '-' ) {
						display_string[Plus_Minus_Expo] = '-';
						display_string[Expo_1] = '0';
						display_string[Expo_0] = Expo_string_temp[1];
					}
					else {
						display_string[Plus_Minus_Expo] = '#';
						display_string[Expo_1] = Expo_string_temp[0];
						display_string[Expo_0] = Expo_string_temp[1];
					}
				}
				else {
					if ( Expo_string_temp[3] < ' ' ) {
						display_string[Plus_Minus_Expo] = '-';
						display_string[Expo_1] = Expo_string_temp[1];
						display_string[Expo_0] = Expo_string_temp[2];
					}
				}
			}
		}
		else {
			Error_String('U');  // expo < -99
		}
	}
	else {
		Error_String('^');    // expo > 99
	}
}

void Error_Test() {
	if ( Debug_Level == 11 ) {
		Serial.print("= ");
		Serial.print(mem_stack_input[mem_pointer].num);
		Serial.print(" / ");
		Serial.print(mem_stack_input[mem_pointer].denom);
		Serial.print(" x 10^ ");
		Serial.println(mem_stack_input[mem_pointer].expo);
	}
	if ( mem_stack_input[mem_pointer].expo > expo_max ) {
		Error_String('^');
		Error_first_0 = true;
	}
	if ( mem_stack_input[mem_pointer].expo == expo_max ) {
		num_temp_u32   = abs(mem_stack_input[mem_pointer].num) / 9;
		denom_temp_u32 = abs(mem_stack_input[mem_pointer].denom) / 10;
		if ( num_temp_u32 > denom_temp_u32 ) {
			Error_String('^');
			Error_first_0 = true;
		}
	}
	if ( mem_stack_input[mem_pointer].expo == expo_min ) {
		if ( abs(mem_stack_input[mem_pointer].num) < abs(mem_stack_input[mem_pointer].denom) ) {
			Error_String('U');  // expo < -99
			Error_first_0 = true;
		}
	}
	if ( mem_stack_input[mem_pointer].expo < expo_min ) {
		Error_String('U');  // expo < -99
		Error_first_0 = true;
	}
}

void Get_Number( uint8_t deep_test ) {
	if ( Number_count != Zero_count ) {
		change_number();
		Error_Test();
	}
	else {
		if ( Start_input != Input_Operation_0 ) {
			mem_stack_input[ mem_pointer ].num   = int32_max;
			mem_stack_input[ mem_pointer ].denom = int32_max;
			mem_stack_input[ mem_pointer ].expo  = expo_min_input;
			mem_stack_input[ mem_pointer ].op    = ' ';  // temp_op
		}
		if (display_string[Plus_Minus] == '-') {
			mem_stack_input[mem_pointer].num *= -1;
		}
	}
	if ( Start_input != Display_Error ) {
		copy_input_left_right( 0, mem_pointer );
		mem_extra_left_right( 0, 0 );
		Start_input  = Input_Operation_0;
		Start_input += deep_test;
	}
}

void Get_Expo_( uint8_t test ) {
	expo_temp_16 = Get_Expo();
	mem_stack_input[mem_pointer].expo = expo_temp_16;

	switch (Point_pos) {

		case 0:
			mem_stack_input[mem_pointer].expo = mem_stack_input[mem_pointer].expo + Number_count;
			break;

		case 1:
			break;

		case 2:
			mem_stack_input[mem_pointer].expo = mem_stack_input[mem_pointer].expo - Zero_after_Point;
			break;

		default:
			mem_stack_input[mem_pointer].expo = mem_stack_input[mem_pointer].expo + Point_pos - 2;
			break;
	}

	if ( test == 1 ) {
		if (abs(mem_stack_input[ mem_pointer ].num) >= mem_stack_input[ mem_pointer ].denom ) {
			--mem_stack_input[ mem_pointer ].expo;
		}
	}
}

void Get_Mantisse() {          // " -1.2345678#- 1 5# 1 9."
	uint8_t Number_pos    = 0;
	uint8_t Repeat_number = Number_count;

	Zero_after_Point = 0;
	Zero_index       = 2;
	
	First_char = display_string[Zero_index];
	if (Point_pos == 2) {
		Zero_index = 3;
		while (display_string[Zero_index] == '0') {
			++Zero_after_Point;
			++Zero_index;
		}
		First_char = display_string[Zero_index]; 
		if ( First_char == '.' ) {
			First_char = display_string[Zero_index + 1];
		}   
	}
	
	if ( Debug_Level == 8 ) {
		Serial.print("'");
		Serial.print(display_string);
		Serial.print("'  Number_count = ");
		Serial.print(Number_count);
		Serial.print("  Point_pos = ");
		Serial.print(Point_pos);
		Serial.print("  Repeat_pos = ");
		Serial.print(Repeat_pos);    
		Serial.print("  Zero_after_Point = ");
		Serial.println(Zero_after_Point);    
	}
	Zero_index_a = 2;
	Number_pos   = 0;

	Temp_char[1] = ' ';

	while ( Zero_index_a <= Number_count + 4 ) {
		 
		if ( display_string[Zero_index_a] == '.' ) {        
			Number_pos   += 1;
			Zero_index_a += 1;
			if ( Number_pos == 2 ) {
				Number_count -= 1;
			}
		}
		 
		Temp_char[Zero_index_a - Number_pos] = display_string[Zero_index_a];
		Zero_index_a += 1;
		Temp_char[Zero_index_a - Number_pos] = ' ';
	}

	Get_Expo_( 0 );

	if ( Debug_Level == 8 ) {
		Serial.print("'");
		Serial.print(Temp_char);
		Serial.println("'");
	}

	num_temp_u32   = atol(Temp_char);
	num_temp_u32  *= expo_10[Zero_after_Point];
	denom_temp_u32 = expo_10[Number_count];

	if ( First_char < '3' ) {
		denom_temp_u32 /= expo_10[1];
		--mem_stack_input[mem_pointer].expo;
	}

	Repeat_number += 3;
	Repeat_number -= Repeat_pos;
 
	if ( Repeat_pos > 0 ) {
		if ( (Repeat_pos - Point_pos) > 1 ) {
			denom_temp_u32 *= expo_10[1];
		}
	}
 
	if ( Zero_after_Point > 2 ) {
		if ( Repeat_pos == 0 ) {
			Zero_after_Point = 2;
		}
	}

	num_temp_u32   /= expo_10[Zero_after_Point];
	denom_temp_u32 /= expo_10[Zero_after_Point];
		
	if ( Debug_Level == 8 ) {
		Serial.print(" ");
		Serial.print(num_temp_u32);
		Serial.print(" / ");
		Serial.print(denom_temp_u32);
		Serial.print("  Repeat_number = ");
		Serial.print(Repeat_number);
		Serial.print("  Number_count = ");
		Serial.print(Number_count);
		Serial.print("  Point_pos = ");
		Serial.print(Point_pos);
		Serial.print("  First_char = ");
		Serial.println(First_char);
	}

	if ( Repeat_pos > 0 ) {
		while ( (num_temp_u32 * 10) < denom_temp_u32 ) {
			num_temp_u32 *= expo_10[1];
			--mem_stack_input[mem_pointer].expo;    
		}
		while ( num_temp_u32 < expo_10[Repeat_number] ) {
			num_temp_u32   *= expo_10[1];
			denom_temp_u32 *= expo_10[1];
			num_temp_u32   += num_temp_u32 / expo_10[Repeat_number];
		}
		num_temp_u32   -= num_temp_u32   / expo_10[Repeat_number];
		denom_temp_u32 -= denom_temp_u32 / expo_10[Repeat_number];
	}

	if ( Debug_Level == 8 ) {
		Serial.print(" ");
		Serial.print(num_temp_u32);
		Serial.print(" / ");
		Serial.print(denom_temp_u32);
		Serial.print("  Repeat_number = ");
		Serial.print(Repeat_number);
		Serial.print("  Number_count = ");
		Serial.print(Number_count);
		Serial.print("  Point_pos = ");
		Serial.print(Point_pos);
		Serial.print("  First_char = ");
		Serial.println(First_char);
	}

	if ( Repeat_number == 11 ) {
		if ( Point_pos > 0 ) {
			if ( Number_count < 8 ) {
				denom_temp_u32 *= expo_10[1];				
			}
		}
	}

	Expand_Number();

	mem_stack_input[mem_pointer].num = num_temp_u32;
	if (display_string[1] == '-') {
		mem_stack_input[mem_pointer].num *= -1;
	}
	mem_stack_input[mem_pointer].denom = denom_temp_u32;

	Error_Test();

	if ( Debug_Level == 8 ) {
		Serial.print("= ");
		Serial.print(mem_stack_input[mem_pointer].num);
		Serial.print(" / ");
		Serial.print(mem_stack_input[mem_pointer].denom);
		Serial.print(" x 10^ ");
		Serial.println(mem_stack_input[mem_pointer].expo);
	}

	Mantisse_change = false;
	Repeat_pos = 0;
}

Ratio_32 input( int8_t chk ) {
	tempsensor.wake();   // wake up, ready to read!
	uint8_t t_8 = tempsensor.getResolution();
  Ratio_32 temp = Null_no;
	uint16_t t = tempsensor.read16(MCP9808_REG_AMBIENT_TEMP);
	tempsensor.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere, stops temperature sampling

	if ( Debug_Level == 60 ) {
		Serial.print("uint16_t t_16: ");
		Serial.println(t);
	}

  if (t != 0xFFFF) {
    temp.num   = t & 0x0FFF;
    temp.denom = 16;
    if (t & 0x1000) {
      temp.num -= 4096;
    }
    if ( temp.num > 530 ) {
    	temp.denom *= 100;
    	temp.expo  += 2;
    } 
    else {
  	  if ( temp.num > 53 ) {
    		temp.denom *= 10;
    		temp.expo  += 1;
    	}
    }
    if ( temp.num < 6 ) {
    	temp.num   *= 10;
    	temp.expo  -= 1;
    }
  }

	gcd_temp_32 = int32_max;
	
	if ( temp.num != 0 ) {
		if (abs(temp.num) > temp.denom) {
			gcd_temp_32 /= abs(temp.num);
		}
		else {
			gcd_temp_32 /= temp.denom;
		}
		temp.num   *= gcd_temp_32;
		temp.denom *= gcd_temp_32;
	}
	else {
		temp.denom = int32_max;
	}

  if ( chk == DISP_F ) {
  	temp = add_mul_spezial( to_xx[ 11 ], to_xx[ 9 ], temp, 1 );
  }
  return temp;
}

void Expand_Number() {
	Ratio_32 temp_32 = Reduce_Number(num_temp_u32, denom_temp_u32, 0);

	num_temp_u32     = temp_32.num;
	denom_temp_u32   = temp_32.denom;

	if ( num_temp_u32 == 0 ) {
		denom_temp_u32 = int32_max;
	}
	if ( Debug_Level == 16 ) {
		Serial.print(gcd_temp_32);
		Serial.print("  __ = ");
		Serial.print(num_temp_u32);
		Serial.print(" / ");
		Serial.println(denom_temp_u32);
	}
}

void Get_Expo_change() {
	calc_temp_32_0 = mem_stack_input[ mem_pointer ].num;
	calc_temp_32_1 = mem_stack_input[ mem_pointer ].denom;
	mul_temp_u32 = abs(calc_temp_32_0 / calc_temp_32_1);
	mem_stack_input[ mem_pointer ].num = calc_temp_32_0;
	mem_stack_input[ mem_pointer ].denom = calc_temp_32_1;
	if ( mul_temp_u32 > 2 ) {
		--mem_stack_input[ mem_pointer ].expo;
	}
	Expo_change = false;
}

void Display_Number(Ratio_32 Display_Input) {
/*
 *   Round "half towards zero"
 *   https://en.wikipedia.org/wiki/Rounding#Round_half_towards_zero
 *   23.5     -->  23
 *  -23.5     --> -23
 *   23.50001 -->  24
 *  -23.50001 --> -24
 */

	 int8_t temp_denom         = -120;
	uint8_t display_digit_abs  = abs(display_digit);

	if ( Display_Input.denom < 9 ) {
		temp_denom = Display_Input.denom;
	}

	uint8_t temp_char = display_string[Operation];
	Point_pos = 0;
	Repeat_pos = 0;

	if ( Debug_Level == 9 ) {
		Serial.print("'");
		Serial.print(display_string);
		Serial.println("'");
	}

	if ( Display_Input.denom < 0 ) {
		Display_Input.num   *= -1;
		Display_Input.denom *= -1;
	}

	if ( to_temperature == true ) {
		if ( Display_Input.num >= 0 ) {
			Display_Input = add( Display_Input, exp2_0_1, 1 );
		}
		else {
			Display_Input = add( Display_Input, exp2_0_1, -1 );
		}
	}

	display_expo = Display_Input.expo;
	if ( abs(Display_Input.num) != 0 ) {
		display_big = abs(Display_Input.num);
	}
	else {
		display_big = abs(Display_Input.denom);
	}

	if ( display_big > abs(Display_Input.denom) ) {
		display_big *= expo_10[display_digit_abs -1];
		++display_expo;
	}
	else {
		display_big *= expo_10[display_digit_abs];
	}
		display_big += (abs(Display_Input.denom) / 2);
	--display_big;
	display_number = display_big / Display_Input.denom;

  if( display_number < 0 ){
  	display_number *= -1;
  }

	if (expo_10[display_digit_abs] == display_number) {
		display_number = expo_10[display_digit_abs - 1];
		++display_expo;
	}
	strcpy( display_string, string_start );

	if (Display_Input.num < 0) {
		display_number *= -1;;
	}
	ltoa(display_number, display_string, 10);

	strcpy(Temp_char, " ");
	strcat(Temp_char, display_string);
	strcpy(display_string, Temp_char);
	strcpy(Temp_char, " ");

	if (display_number > 0) {
		display_string[1] = ' ';
	//	strcat(Temp_char, display_string);
	//	strcpy(display_string, Temp_char);
	}
	else {
		display_string[1] = '-';
	}

	display_expo_mod = abs(display_expo % 3);
	if (display_expo_mod == 0) {
		display_expo = display_expo - 3;
	}
	else {
		if (display_expo < 0) {
			display_expo_mod = 3 - display_expo_mod;
		}
	}
	display_expo = display_expo - display_expo_mod;

	expo_temp_16 = display_expo;
	Put_Expo();
	if ( Start_input == Display_Error ) {
		Beep__on();
		return;
	}

	display_string[Plus_Minus_Expo_] = '#';
	if ( expo_temp_16 >= 0 ) {
		display_string[Plus_Minus_Expo] = '#';
		display_string[Plus_Minus_Expo__] = ' ';
	}
	display_string[display_digit_abs + 2] = '.';
	Point_pos = display_digit_abs;
	Point_pos += 2;

	if ( display_expo_mod <= 0 ) {
		display_expo_mod = display_expo_mod + 3;
	}

	if ( Debug_Level == 9 ) {
		Serial.print("'");
		Serial.print(display_string);
		Serial.println("'");
	}

	if ( display_expo_mod < display_digit_abs ) {
		for ( index_a = display_digit_abs + 1; index_a > (display_expo_mod + 2); index_a -= 1 ) {
			display_string[index_a + 1] = display_string[index_a];
		}
		display_string[index_a + 1] = display_string[index_a];
		display_string[index_a] = '.';
		Point_pos = index_a;
	}

	if ( (display_digit_abs == 2) && (display_expo_mod == 3) ) {
		display_string[4] = '0';
		display_string[5] = '.';
		Point_pos = 5;
	}

	Display_Memory_x( 25 );

	if ( Rad_in_out == true ) {
		display_string[Rad_point] = '.';
	}
	if ( Beep_on_off_temp == true ) {
		display_string[Beep_point] = '.';
	}
	if ( Std_mode == 0 ) {
		display_string[MR_point] = '.';
	}

	if ( display_string[Plus_Minus_Expo__] == ' ' ) {
		display_string[Plus_Minus_Expo__] = '#';
	}

	if ( Start_input >= Input_Memory ) {       // Input / Operation Display
		if ( Start_input < Input_Operation_5 ) {
			if ( Display_Status_new != 40 ) {
				display_string[Operation]  = Display_Input.op;
			}
			if ( Start_input > Input_Operation_0 ) {
				display_string[Operation_point] = '.';
			}
		}
	}

	if ( Start_input == Display_Result ) {
		display_string[Memory_0] = Display_Memory_0[8];  // =
	}

	Point_pos = display_expo_mod + 2;
	Number_count = display_digit_abs;
	Cursor_pos = display_digit_abs + 3;

	if ( display_string[Cursor_pos] == '.' ) {
		++Number_count;
		++Cursor_pos;
	}

	if ( abs(Display_Input.num) == 0 ) {
		display_string[Mantisse_0] = '.';
		display_string[Mantisse_1] = '0';
		Zero_count = display_digit_abs;
		--Point_pos;
	}

	Zero_count = 0;
	Zero_index = 0;
	while (display_string[Zero_index] != '#') {
		if ( display_string[Zero_index]  == '0' ) {
			++Zero_count;
		}
		++Zero_index;
	}

	if ( temp_denom >= 0 ) {
		Error_String(Error_String_txt[temp_denom]);
	}

	if ( Display_Status_new == 40 ) {
		display_string[Operation] = temp_char;
	}

	if ( Start_input == Input_Memory ) {
		display_string[Operation] = ' ';
	}

	if ( to_temperature == true ) {
		--display_string[2];
	}
	to_temperature   = false;

	if ( display_digit < 0 ) {
		display_digit_abs += 2;
		while ( display_string[display_digit_abs] == '0' ) {
			if ( display_digit_abs > 3 ) {
				if ( Point_pos < display_digit_abs ) {
					display_string[display_digit_abs] = '#';
					--Number_count;
					--Cursor_pos;
					--Zero_count;
				}
			}
			--display_digit_abs;
		}
	}

	if ( Debug_Level == 9 ) {
		Serial.print("'");
		Serial.print(display_string);
		Serial.println("'");
		Serial.println(display_digit);
		Serial.println(display_expo);
		Serial.println(display_expo_mod);
		Serial.println(Zero_count);
		Serial.println(display_digit_abs);
	}

	Display_Memory_x( 23 );
}

void Put_input_Point() {
	Point_pos = Cursor_pos;
	display_string[Cursor_pos] = '.';
	++Cursor_pos;
	display_string[Cursor_pos] = '_';
}

void Input_Repeat_Point() {
	Repeat_pos = Cursor_pos;
	if ( (Repeat_pos - Point_pos) > 1 ) {
		display_string[Cursor_pos] = '.';
		++Cursor_pos;
		display_string[Cursor_pos] = '_';
		if ( Repeat_pos < 7 ) {
			display_string[Cursor_pos + 1] = '~';
		}
	}
}

void Display_Off() {
	if ( Start_input == Display_Error ) {
		display_string[Cursor_pos]  = '.';
	}
	Repeat_pos = 0;
	Start_input = Off_Status;
	display_string[MR_point]        = ' ';
	display_string[Expo_1]          = ' ';
	display_string[MS_point]        = ' ';
	display_string[Expo_0]          = ' ';
	display_string[Expo_point]      = ' ';
	display_string[Operation]       = 'o';
	display_string[Operation_point] = ' ';
	display_string[Rad_point]       = ' ';
        Display_Memory_x( 18 );
	Display_new = true;
}

Ratio_32 div_u32(Ratio_32 a, uint32_t denom_x) {
	if ( a.num == 0 ) {
		return a;
	}

	if ( denom_x == 1 ) {
		return a;
	}

	if ( a.num > 0 ) {
		test_signum_8 = 1;
	}
	else {
		test_signum_8 = -1;
	}

	num_temp_u64    = abs(a.num);
	denom_temp_u64  = abs(a.denom);
	temp_32.expo    = a.expo;

	denom_temp_u64 *= denom_x;

	while ( num_temp_u64 < denom_temp_u64 ) {
		num_temp_u64   *= 5;
		denom_temp_u64 /= 2;
		--temp_32.expo;
	}

	// calc_temp_16_0 = num_temp_u64 / denom_temp_u64;
	if ( (num_temp_u64 / denom_temp_u64) > 2 ) {
		denom_temp_u64 *= 10;
		++temp_32.expo;
	}
										 // num_temp_u64 ,  denom_temp_u64
	temp_32      = Reduce_Number( num_temp_u64, denom_temp_u64, temp_32.expo );
	temp_32.num *= test_signum_8;

	return temp_32;
}

Ratio_32 mul_spezial(Ratio_32 a, Ratio_32 b, uint64_t corr) {
	if ( a.num == 0 ) {
		return a;
	}
	if ( b.num == 0 ) {
		return b;
	}
	temp_64.expo   = a.expo;
	temp_64.expo  += b.expo;

	temp_64.num    = a.num;
	temp_64.num   *= b.num;

	temp_64.denom  = a.denom;
	temp_64.denom *= b.denom;

	if ( temp_64.num > 0 ) {
		test_signum_8 = 1;
	}
	else {
		test_signum_8 = -1;
	}

	num_temp_u64   = abs(temp_64.num);
	denom_temp_u64 = abs(temp_64.denom);

	num_temp_u64  += num_temp_u64 / corr;

	return to_Ratio_32(temp_64);
}

Ratio_32 add_mul_spezial(Ratio_32 a, Ratio_32 b, Ratio_32 c, int8_t d) {
	return add( a, mul(b, c), d );
}

Ratio_32 floor_(Ratio_32 a, int8_t expo_test) {
// floor_(x) returns the largest integer n such that n <= x
	denom_temp_u64  = a.denom;
	temp_32.expo    = a.expo;

	if ( a.expo > expo_test ) {
		a.denom = 6;
		if ( a.num < 0 ) {
			a.denom = 5;
		}
		a.num   = 7;
		return a;
	}
	if ( a.expo == expo_test ) {
		if (abs(a.num) >= abs(a.denom)) {
			a.denom = 6;
			if ( a.num < 0 ) {
				a.denom = 5;
			}
			
			a.num   = 7;
			return a;
		}
	}

	if ( a.num > 0 ) {
		test_signum_8 = 1;
		num_temp_u64  = a.num;
	}
	else {
		test_signum_8 = -1;
		num_temp_u64  = -a.num;
	}

	if ( a.expo == 0 ) {
		if ( denom_temp_u64 > num_temp_u64) {
			temp_32.num   = int32_max;
			temp_32.num  *= test_signum_8;
			temp_32.denom = int32_max;
			temp_32.expo  = expo_min_input;
			return temp_32;
		}
	}

	if ( a.expo < 0 ) {
		temp_32.num   = int32_max;
		temp_32.num  *= test_signum_8;
		temp_32.denom = int32_max;
		temp_32.expo  = expo_min_input;
		return temp_32;
	}

	num_temp_u64 *= expo_10[temp_32.expo];
	temp_64.num   = num_temp_u64;
	temp_64.num  /= denom_temp_u64;

	temp_32.num   = temp_64.num;
	temp_32.denom = expo_10[temp_32.expo];

	num_temp_u32  = int32_max;
	if ( temp_32.num > temp_32.denom ) {
		num_temp_u32 /= temp_32.num;
	}
	else {
		num_temp_u32 /= temp_32.denom;
	}

	temp_32.num   *= num_temp_u32;     // Expand Number
	temp_32.num   *= test_signum_8;
	temp_32.denom *= num_temp_u32;     // Expand Number

	return temp_32;
}

Ratio_32 frac(Ratio_32 a) {
// returns the fractional part of x
	temp_32 = floor_(a, 8);
	if ( temp_32.denom < 9 ) {
		return temp_32;
	}
	return add( a, temp_32, -1 );
}

void calc_stack(uint8_t count_stack) {
	if ( count_stack > 1 ) {
		if ( (Found_constant == true) || count_stack > 2 ) {

			Beep__off();

			if ( count_stack > 2 ) {
				Found_constant = false;
			}

			number_stack[ count_stack ].num = mem_stack_calc[ count_stack ].num;
			number_stack[ count_stack ].denom = mem_stack_calc[ count_stack ].denom;
			number_stack[ count_stack ].expo = mem_stack_calc[ count_stack ].expo;

			while ( count_stack > 1 ) {

				--count_stack;

				number_stack[ count_stack ].num = mem_stack_calc[ count_stack ].num;
				number_stack[ count_stack ].denom = mem_stack_calc[ count_stack ].denom;
				number_stack[ count_stack ].expo = mem_stack_calc[ count_stack ].expo;

				if ( temp_operation < 252 ) {
					temp_operation = mem_stack_calc[ count_stack ].op;
				}
				if ( temp_operation > 251 ) {
					number_stack[ count_stack + 1 ].expo -= 2;
				}

				count = 0;
				count_wait = 0;
				switch (temp_operation) {

					case 254: //    _*%_
					case 213: //    _* *_
					case 42:  //    _*_
						number_stack[ count_stack ] = mul(number_stack[ count_stack ], number_stack[ count_stack + 1 ]);
						break;

					case 216: //    _+ +_
					case 43:  //    _+_
						number_stack[ count_stack ] = add( number_stack[ count_stack ], number_stack[ count_stack + 1 ], 1 );
						break;

					case 45:  //    _-_
						number_stack[ count_stack ] = add( number_stack[ count_stack ], number_stack[ count_stack + 1 ], -1 );
						break;

					case 255: //    _/%_
					case 47:  //    _/_
						number_stack[ count_stack ] = mul(number_stack[ count_stack ], div_x(number_stack[ count_stack + 1 ]));
						break;
				
					case 252: //    _+%_
						number_stack[ count_stack ] = add( number_stack[ count_stack ], mul( number_stack[ count_stack ], number_stack[ count_stack + 1 ]), 1);
						break;

					case 253: //    _-%_
						number_stack[ count_stack ] = add( number_stack[ count_stack ], mul( number_stack[ count_stack ], number_stack[ count_stack + 1 ]), -1);
						break;
				
					case 88:  //    _y_expo
						number_stack[ count_stack ] = powx_extra(number_stack[ count_stack ], number_stack[ count_stack + 1 ]);
						break;

					case 89:  //    _logx
						number_stack[ count_stack ] = logx(number_stack[ count_stack + 1 ], number_stack[ count_stack     ]);
						break;

					case 212: //    _- -_
						 number_stack[ count_stack ] = add( number_stack[ count_stack + 1 ], number_stack[ count_stack ], -1 );
						break;

					case 214: //    _/ /_
						number_stack[ count_stack ] = mul(number_stack[ count_stack + 1 ], div_x(number_stack[ count_stack ]));
						break;

					case 217: //    _y_expo y_expo_
						number_stack[ count_stack ] = powx_extra(number_stack[ count_stack + 1 ], number_stack[ count_stack ]);
						break;

					case 218: //    _y_root y_root_
						number_stack[ count_stack ] = logx(number_stack[ count_stack     ], number_stack[ count_stack + 1 ]);
						break;

					case 219: //    _HM HM_
					case 124: //    _HM_
						number_stack[ count_stack ] = div_x( add( div_x(number_stack[ count_stack ]), number_stack[ count_stack + 1 ], 6 ));
						break;

					case 220: //    _AM AM_
					case 128: //    _AM_
						number_stack[ count_stack ] = add( number_stack[ count_stack ], number_stack[ count_stack + 1 ], 2 );
						break;

					case 221: //    _GM GM_
					case 129: //    _GM_
						number_stack[ count_stack ] = sqrt(mul(number_stack[ count_stack ], number_stack[ count_stack + 1 ]));
						break;

					case 222: //    _AGM AGM_
					case 151: //    _AGM_
						number_stack[ count_stack ] = agm(number_stack[ count_stack ], number_stack[ count_stack + 1 ]);
						break;

					case 152: //    _mod_
						number_stack[ count_stack ] = mul(frac(mul(number_stack[ count_stack     ], div_x(number_stack[ count_stack + 1 ]))), number_stack[ count_stack + 1 ]);
						break;

					case 223: //    _mod_mod_
						number_stack[ count_stack ] = mul(frac(mul(number_stack[ count_stack + 1 ], div_x(number_stack[ count_stack     ]))), number_stack[ count_stack     ]);
						break;

					case 215: //    _// //_
					case 92:  //    _//_
						number_stack[ count_stack ] = div_x( add( div_x(number_stack[ count_stack ]), number_stack[ count_stack + 1 ], 3 ));
						break;

					case 94:  //    _/p/_
						number_stack[ count_stack ] = sqrt(add( square(number_stack[ count_stack ]), square(number_stack[ count_stack + 1 ]), 1 ));
						break;

					case 211: //   _/p/ /p/_
						number_stack[ count_stack ] = sqrt(add( square(number_stack[ count_stack ]), square(number_stack[ count_stack + 1 ]), -1 ));
						break;

					default:
						number_stack[ count_stack ] = number_stack[ count_stack + 1 ];
				}			

			}

			if ( temp_operation > 251 ) {
			  mem_stack_calc[ 1 ].op = temp_operation;
				temp_operation = 0;
			}

			mem_stack_calc[ 0 ].num    = number_stack[ 1 ].num;
			mem_stack_calc[ 0 ].denom  = number_stack[ 1 ].denom;
			mem_stack_calc[ 0 ].expo   = number_stack[ 1 ].expo;
			mem_stack_calc[ 0 ].op     = mem_stack_calc[ 1 ].op;

			mem_stack_input[ 0 ].num   = number_stack[ 1 ].num;
			mem_stack_input[ 0 ].denom = number_stack[ 1 ].denom;
			mem_stack_input[ 0 ].expo  = number_stack[ 1 ].expo;

			mem_extra_left_right( 0, 0 );
			
			Beep__on();
		}
	}
}

// integer square root of a positive number
// http://ww1.microchip.com/downloads/en/AppNotes/91040a.pdf
uint32_t sqrt(uint64_t num) {
  uint32_t res   = 0;    // result
  uint32_t add   = 0x1ULL << 31; // 'additional' bit is in position 31
  uint32_t temp;         // 'A'
  uint64_t quad;         // 'A^2'

  while ( add > 0 ) {    // 32x test and shift right
    temp  = res + add;
    quad  = temp;
    quad *= temp;
    if ( num >= quad ) { 
      res = temp;
    }
    add >>= 1;           // shift right the 'additional' bit 
  }
  return res;
}

Ratio_32 sqrt(Ratio_32 a) {
	if ( a.num == 0 ) {
		return a;
	}

	num_temp_u64    = abs(a.num);
	num_temp_u64   *= abs(a.denom);
	calc_temp_u32   = sqrt(num_temp_u64);

	if ( Debug_Level == 61 ) {
		Serial.print(a.num);
		Serial.print(" / ");
		Serial.println(a.denom);
		itoa_(num_temp_u64, display_string_itoa_);
		Serial.print(display_string_itoa_);
		Serial.print("  ");
		Serial.println(calc_temp_u32);
	}

	denom_temp_u64  = calc_temp_u32;
	num_temp_u64   += denom_temp_u64 * denom_temp_u64;
	denom_temp_u64 *= abs(a.denom);
	denom_temp_u64 *= 2;
	temp_32 = Reduce_Number( num_temp_u64, denom_temp_u64, a.expo );

	if ( a.expo < 4 ) {
		temp_32.expo += 110;
	}

	if ( (temp_32.expo % 2) == 1 ) {
		if ( abs(a.num) > abs(a.denom) ) {
			temp_32 = mul(temp_32, sqrt_10_minus);
		}
		else {
			temp_32 = mul(temp_32, sqrt_10_plus);
		}
	}

	temp_32.expo /= 2;
	if ( a.expo < 4 ) {
		temp_32.expo -= 55;
	}

	if ( a.num < 0 ) {
		 temp_32.num  *= -1;
	}

	return temp_32;
}
// -->  53 Zeilen
Ratio_32 cbrt(Ratio_32 a) {
	if ( a.num == 0 ) {
		return a;
	}

	temp_32_b2 = abs_x(a);

	int96_a a_num_calc   = temp_32_b2.num;

	a_num_calc   *= temp_32_b2.denom;
	a_num_calc   *= temp_32_b2.denom;

	a_num_calc.icbrt(a_num_calc);
	temp_32_b2.num = a_num_calc.lo;

	temp_32_b2.expo  = a.expo;
	--temp_32_b2.expo;

	if ( a.expo < 5 ) {
		temp_32_b2.expo += 111;  // 111 = 3 * 37
	}

	temp_32_b2.expo /= 3;
	if ( a.expo < 5 ) {
		temp_32_b2.expo -= 37;
	}

	switch (uint8_t(a.expo + 111) % 3) {

		case 0:
			++temp_32_b2.expo;
			break;

		case 1:
			temp_32_b2 = mul( temp_32_b2, cbrt_10_plus );  // 2,15^3 =  10.0
			break;

		case 2:
			temp_32_b2 = mul( temp_32_b2, cbrt_100_plus ); // 4,64^3 = 100.0
			break;
	}
 
	temp_32_b1 = square( temp_32_b2 );                      
	temp_32_b2 = add_mul_spezial( temp_32_b2, abs_x(a), div_x(temp_32_b1), 9 );  // = (2a + b ) / 3
 
	if (a.num < 0) {
		temp_32_b2.num *= -1;
	}

	return temp_32_b2;
}

Ratio_32 factorial(Ratio_32 a) {
/*
 * The following function, abgam() is based on a continued fraction numerical
 * method found in Abremowitz and Stegun, Handbook of Mathematical Functions
 *
 * http://www.realtimerendering.com/resources/GraphicsGems/gemsiii/sqfinal.c
 *
 * https://infogalactic.com/info/Factorial#Approximations_of_factorial
 *
 * info --> http://dx.doi.org/10.3247/sl2math08.005
 * 2008_MTH_Nemes_GammaApproximationUpdate.pdf .. Formula (4.1)
 * http://www.luschny.de/math/factorial/approx/SimpleCases.html#AhighPrecisionApproximation
 * http://www.luschny.de/math/factorial/approx/Approximations%20for%20the%20Factorial%20Function.pdf  page 8
 *
 * https://www2.eecs.berkeley.edu/Pubs/TechRpts/1979/ERL-m-79-71.pdf
 */
	uint8_t   fac_test    = 10;
	uint8_t   fac_count   = 10;
	 int8_t   test        = compare( a, test_71 );

  Ratio_32  temp_32_mul_a   = {0, int32_max, int32_max, 0};
  Ratio_32  temp_32_fac     = {1, int32_max_16, int32_max, 0}; // 6
  Ratio_32  temp_32_fac_sin = {1, int32_max_16, int32_max, 0}; // 6
	
	bool    input_near_null = false; //     -1.000 < input < 0.000
	
	if ( test > 0 ) { //  input > abs(71)
		a.denom = 4; // Error_String('[');
		Error_first = true;
		return a;
	}

	if ( a.expo == 0 ) { //  input = 1.000 or 0.000
		if ( (a.num == a.denom) || (a.num == 0) ) {
			return log_1e0;
		}
	}

	if ( a.num < 0 ) {
		a = add( a, log_1e0, 1 );
		if ( a.num > 0 ) {
			input_near_null = true;
			temp_32_corr_b  = div_x( a );
		}
	}

	if ( a.num > 0 ) {		

		temp_32_corr     = clone( log_1e0 );
		temp_32_corr_0_1 = clone( log_1e0 );
		if ( a.expo < 0 ) {
			a                = add( a, log_1e0, 1 );
			temp_32_corr_0_1 = div_x( a );
		}

		num_temp_u64    = 0;
		if ( a.expo >= 0 ) {
			num_temp_u64   = abs(a.num);
			num_temp_u64  *= expo_10[a.expo];
			num_temp_u64  /= abs(a.denom);
		}
		fac_test   = num_temp_u64;
		fac_count  = num_temp_u64;

		temp_32_fac      = add( a, log_1e0, 1 );
		if ( fac_test < 6 ) {
			while ( fac_count < 6 ) {
				++fac_count;
				temp_32_corr = mul( temp_32_fac, temp_32_corr );
				temp_32_fac  = add( temp_32_fac, log_1e0, 1 );
			}
			temp_32_corr = div_x( temp_32_corr );
			a = add( mul_6_0, frac(a), 1 );
		}

		while ( fac_test > 66 ) {
			temp_32_corr_0_1 = mul( a, temp_32_corr_0_1 );
			a                = add( a, log_1e0, -1 );
			--fac_test;
			if ( fac_test == 66 ) {
				temp_32_fac    = add( a, log_1e0, 1 );
			}
		}

		temp_32_mul = clone( temp_32_fac );

		for ( uint8_t index = 0; index < fa_x_max; index += 1 ) {
			temp_32_mul = add_mul_spezial(temp_32_fac, fa_x[index], div_x( temp_32_mul ), 1 );
		}

		temp_32_mul   = mul( fa_0, div_x( temp_32_mul ) );
		temp_32_mul_a = add( a, exp2_1_2, 1 );
		temp_32_mul   = add( temp_32_mul, add_mul_spezial( min_x(temp_32_fac), temp_32_mul_a, loge(temp_32_fac), 1 ), 1 );
		temp_32_mul   = exp( add( temp_32_mul, fa_ln_2pi_2, 1 ));

		temp_32_mul = mul( temp_32_mul, temp_32_corr );
		temp_32_mul = mul( temp_32_mul, temp_32_corr_0_1 );

		if ( input_near_null == true ) {
			return mul( temp_32_mul, temp_32_corr_b );
		}
		else {
			return temp_32_mul;
		}
	}
	else {
		temp_32_mul = frac( a );
    if ( (temp_32_mul.expo == -90) || (temp_32_mul.num == 0) ) {
			a.denom = 4; // Error_String('[');
			Error_first = true;
		  return a;
    }

		if ( Rad_in_out == true )  {
  	  temp_32_mul = mul( a, Pi );
	  }
	  else {
		  temp_32_mul = mul( a, circle_2 );
	  }
	  temp_32_fac_sin = sin( temp_32_mul );	  

	  temp_32_mul = factorial( min_x( add( a, exp2_0_1, 1 ) ) );

	  return mul( Pi_neg, div_x( mul( a, mul( temp_32_fac_sin , temp_32_mul ) ) ) );
	}
}

Ratio_32 sin_cos_tan(Ratio_32 a) {
	cordic_test = 0;

	if ( a.num == 0 ) {
		return a;
	}
	if ( Rad_in_out == true )  {
		temp_32_mul = mul( a, Tau_div_x );
	}
	else {
		temp_32_mul = mul( a, circle_div_x );
	}

	if ( temp_32_mul.expo > 5 ) { //  input > 300000
		if ( temp_32_mul.num > 0 ) {
			temp_32_mul.denom = 6;  // Error_String('^');  input >  300000
		}
		else {
			temp_32_mul.denom = 5;  // Error_String('U');  input < -300000
		}
		temp_32_mul.num = int32_max;
		Error_first = true;
		return temp_32_mul;
	}

	if ( temp_32_mul.expo > 4 ) { //  input > 30000
		if ( (abs(temp_32_mul.num) / 25) > (temp_32_mul.denom / 9) ) {
			if ( temp_32_mul.num > 0 ) {
				temp_32_mul.denom = 6;  // Error_String('^');  input >  277777,77
			}
			else {
				temp_32_mul.denom = 5;  // Error_String('U');  input < -277777,77
			}
			temp_32_mul.num = int32_max;
			Error_first = true;
			return temp_32_mul;
		}
	}

	temp_32_mul = frac(temp_32_mul);
	if ( temp_32_mul.expo == 0 ) {
		if ( abs( temp_32_mul.num ) > ( temp_32_mul.denom / 2 ) ) {
			if ( temp_32_mul.num > 0 ) {
				temp_32_mul = add( temp_32_mul, log_1e0, -1 );
			}
			else {
				temp_32_mul = add( temp_32_mul, log_1e0, 1 );
			}
		}
	}

	temp_64.num    = temp_32_mul.num;
	temp_64.denom  = temp_32_mul.denom;
	temp_64.num   *= 8;
	if ( temp_32_mul.expo >= -8 ) {
		temp_64.denom *= expo_10[ abs( temp_32_mul.expo ) ];
		cordic_test    = temp_64.num / temp_64.denom;
		if ( temp_32_mul.num > 0 ) {
			++cordic_test;
		}
		if ( temp_32_mul.num < 0 ) {
			--cordic_test;
		}
	}

	if ( temp_32_mul.expo < -78 ) {
		temp_32_mul.num   = 0;
		temp_32_mul.denom = int32_max;
		temp_32_mul.expo  = 0;
	}

	if ( Debug_Level == 40 ) {
		Serial.print("cordic_test = ");
		Serial.println(cordic_test);
	}

	switch (cordic_test) {

		case 5:                  //
		case 4:                  //
			return add( exp2_1_2, temp_32_mul, -1 );
			break;

		case 3:                  //
			return add( temp_32_mul, exp2_1_4, -1 );
			break;

		case 2:                  //
			return add( exp2_1_4, temp_32_mul, -1 );
			break;

		case -1:                 //
			return min_x( temp_32_mul );
			break;

		case -2:                 //
			return add( exp2_1_4, temp_32_mul, 1 );
			break;

		case -3:                 //
			return add( exp2__1_4, temp_32_mul, -1 );
			break;

		case -4:                 //
		case -5:                 //
			return add( exp2_1_2, temp_32_mul, 1 );
			break;

		case 1:                  //
		default:
			return temp_32_mul;
			break;
	}
}

Ratio_32 cordic(int8_t function) {
	uint8_t index_tab     = 0;
	int64_t test_cordic   = 0;
	int64_t cordic_add    = 0;
	uint8_t index_count   = 0;

	uint8_t cordic_tab_   = 0;

	int96_a temp_cordic   = 0;

	uint8_t k_count       = 1;

	int64_t x_cordic      = 1;
	int64_t y_cordic      = 0;
	int64_t z_cordic      = 0;

	int64_t tx_cordic     = 1;
	int64_t ty_cordic     = 0;
	int64_t tz_cordic     = 0;

	int64_t d_cordic      = -1;
	
	Tau_temp_32_corr_a    =  mul( Tau, temp_32_corr_a );
	
	if ( Error_first == true ) {
		return temp_32_corr_a;
	}

	if ( ( temp_32_corr_a.expo <  -8 ) || ( temp_32_corr_a.num == 0 ) ) {

		if ( function == sin_ ) {
			switch (cordic_test) {

				case -5:
				case -4:
					Tau_temp_32_corr_a = min_x( Tau_temp_32_corr_a ); [[fallthrough]];
				case  5:
				case  4:
				case  0:
					return Tau_temp_32_corr_a;
					break;

				case  3:
				case  2:
					return log_1e0;
					break;

				case -2:
				case -3:
					return log__1e0;
					break;

				default:
					break;
			}
		}
		if ( function == cos_ ) {
			switch (cordic_test) {

				case -5:
				case -4:
				case  5:
				case  4:
					return log__1e0;
					break;

				case  0:
					return log_1e0;
					break;

				case  3:
				case -3:
					Tau_temp_32_corr_a = min_x( Tau_temp_32_corr_a ); [[fallthrough]];
				case  2:
				case -2:
					return Tau_temp_32_corr_a;
					break;

				default:
					break;
			}
		}
		if ( function == tan_ ) {
			switch (cordic_test) {

				case  5:
				case  4:
					Tau_temp_32_corr_a = min_x( Tau_temp_32_corr_a ); [[fallthrough]];
				case -5:
				case -4:
				case  0:
					return Tau_temp_32_corr_a;
					break;

				case  3:
				case -2:
					Tau_temp_32_corr_a = min_x( Tau_temp_32_corr_a ); [[fallthrough]];
				case  2:
				case -3:
					if ( abs( Tau_temp_32_corr_a.num ) > 0 ) {
						return div_x( Tau_temp_32_corr_a );
					}
					else {
						temp_32_corr_a.num = int32_max;
						temp_32_corr_a.denom = 8;  // Error_String('x');
						Error_first = true;
					}
					return temp_32_corr_a;
					break;

				default:
					break;
			}
		}
	}

	if ( function > 0 ) {
		temp_32_corr_a = mul( circle_to, temp_32_corr_a );

		temp_cordic   = temp_32_corr_a.num;
		temp_cordic  *= expo_10_[temp_32_corr_a.expo - 10];
		temp_cordic  /= temp_32_corr_a.denom;

		tz_cordic     = int64_t(temp_cordic);
		tz_cordic    *= d_cordic;

		tx_cordic     = tx_cordic << 62;
		tx_cordic    += tx_cordic >> 1;
		tx_cordic    += tx_cordic >> 3;
		tx_cordic    += tx_cordic >> 6; // 7903817346035220480
	}

	if ( function == atan_ ) {
		tx_cordic  = temp_32_corr_a.num;
		tx_cordic  = tx_cordic << 31;
		ty_cordic  = temp_32_corr_a.denom;
		ty_cordic  = ty_cordic << 31;
		tz_cordic  = 0;

		while ( temp_32_corr_a.expo < 0 ) {
			tx_cordic += tx_cordic >> 1; // * 1.5
			tx_cordic  = tx_cordic >> 4; // / 16
			ty_cordic += ty_cordic >> 1; // * 1.5
			y_cordic   = ty_cordic;
			ty_cordic -= y_cordic >> 2;
			ty_cordic -= y_cordic >> 3; // * 0.625 = / 1.6
			++temp_32_corr_a.expo;
		}

		tx_cordic += tx_cordic >> 1; // * 1,5
		tx_cordic -= tx_cordic >> 3; // * 0,875 = 1,3125
		ty_cordic += ty_cordic >> 1; // * 1,5
		ty_cordic -= ty_cordic >> 3; // * 0,875 = 1,3125
	}

	x_cordic  = tx_cordic;
	y_cordic  = ty_cordic;
 /*
	if ( Debug_Level == 40 ) {
		itoa_(tz_cordic, display_string_itoa_);
		Serial.println(display_string_itoa_);
	}
 */
	for ( uint8_t index_a = 0; index_a < 32; index_a += 1 ) {  // 32
		index_count = cordic_tab[index_tab];
		index_count = index_count >> 4;
		cordic_add = 0;
		if ( index_count > 0 ) {
			for ( uint8_t index_b = 0; index_b < index_count; index_b += 1 ) {
				cordic_add  = cordic_add << 8;
				cordic_tab_ = cordic_tab[index_tab];
				if ( index_b == 0 ) {
					cordic_tab_ = cordic_tab_ << 4;
					cordic_tab_ = cordic_tab_ >> 4;
				}
				cordic_add += cordic_tab_;
				++index_tab;
			}
		}
		test_cordic  = test_cordic >> 1;
		test_cordic += cordic_add;

		if ( Debug_Level == 55 ) {
			Serial.print("index_a : ");
			Serial.print(index_a);
			Serial.print("  index_count : ");
			Serial.println(index_count);
		
			itoa_(test_cordic, display_string_itoa_);
			Serial.println(display_string_itoa_);
		
		}

		if ( function > 0 ) {
			if ( tz_cordic < 0 ) {
				d_cordic = 1;
			}
		}

		if ( function < 0 ) {
			if ( tx_cordic > 0 ) {
				d_cordic = 1;
			}
		}

		tx_cordic -= d_cordic * (y_cordic >> k_count);
		ty_cordic += d_cordic * (x_cordic >> k_count);
		tz_cordic += d_cordic * test_cordic;
  /*
		if ( Debug_Level == 40 ) {
			Serial.print(k_count);
			Serial.print(' ');

			itoa_(tz_cordic, display_string_itoa_);
			Serial.print(display_string_itoa_);

			Serial.print(' ');
			itoa_(tx_cordic, display_string_itoa_);
			Serial.print(display_string_itoa_);

			Serial.print(' ');
			itoa_(ty_cordic, display_string_itoa_);
			Serial.println(display_string_itoa_);
		}
  */
		x_cordic  = tx_cordic;
		y_cordic  = ty_cordic;
		++k_count;
		d_cordic  = -1;
	}

	if ( function > 0 ) {
		x_cordic   = x_cordic >> 31;   // 31
		y_cordic   = y_cordic >> 31;   // 31

		x_cordic  *= tz_cordic;
		y_cordic  *= tz_cordic;

		tx_cordic += y_cordic >> 32;   // 32
		ty_cordic -= x_cordic >> 32;   // 32
	
		if ( Debug_Level == 40 ) {
			Serial.print(" tz_cordic ");
			itoa_(tz_cordic, display_string_itoa_);
			Serial.print(display_string_itoa_);

			Serial.print(" tx_cordic ");
			itoa_(tx_cordic, display_string_itoa_);
			Serial.print(display_string_itoa_);

			Serial.print(" ty_cordic ");
			itoa_(ty_cordic, display_string_itoa_);
			Serial.println(display_string_itoa_);
		}
	
		temp_32_corr_a.expo  = -1;
		denom_temp_u64  = 920348428214616521;  // 32  --  920348428214616521
	}

	if ( function < 0 ) {
		z_cordic             = 1;
		z_cordic             = z_cordic << 63;
		temp_cordic          = z_cordic;
		temp_cordic         *= tx_cordic;
		temp_cordic         /= ty_cordic;
	 /*
		if ( Debug_Level == 40 ) {
			Serial.print("temp_cordic = ");
			itoa_(temp_cordic, display_string_itoa_);
			Serial.println(display_string_itoa_);
		}
	 */
		tz_cordic           -= int64_t(temp_cordic);
	 /*
		if ( Debug_Level == 40 ) {
			Serial.print("z_cordic = ");
			itoa_(z_cordic, display_string_itoa_);
			Serial.println(display_string_itoa_);
		}
	 */
		temp_32_corr_a.expo  = 0;
		num_temp_u64         = tz_cordic;
		denom_temp_u64       = 9223372036854775807;  // 32 - round -- 45Â° = pi() / 4;

		if ( Debug_Level == 46 ) {
			Serial.print("  ");
			Serial.print(micros() - time_test);
			Serial.print("  ");
			itoa_(num_temp_u64, display_string_itoa_);
			Serial.println(display_string_itoa_);
		}
	}

	if ( function == sin_ ) {
		num_temp_u64    = ty_cordic;
	}
	if ( function == cos_ ) {
		num_temp_u64    = tx_cordic;
	}
	if ( function == tan_ ) {
		num_temp_u64         = ty_cordic;
		denom_temp_u64       = tx_cordic;
		temp_32_corr_a.expo  = 0;
	}

	if ( (function == tan_) || (function == atan_) ) {
		while ( num_temp_u64 < expo_test_0a ) {
			num_temp_u64        *= 10;
			--temp_32_corr_a.expo;
		}
		if ( num_temp_u64 < expo_test_0b ) {
			num_temp_u64        *= 5;
			denom_temp_u64      /= 2;
			--temp_32_corr_a.expo;
		}
	}

	if ( (function == sin_) || (function == cos_) ){
		if ( abs(cordic_test) > 1 ) {
			if ( abs(cordic_test) < 4 ) {
				if ( function == sin_ ) {
					num_temp_u64    = tx_cordic;
				}
				if ( function == cos_ ) {
					num_temp_u64    = ty_cordic;
				}
			}
		}
		while ( num_temp_u64 < 276104528464384956 ) {
			num_temp_u64        *= 10;
			--temp_32_corr_a.expo;
		}
		if ( num_temp_u64 > 2761045284643849563 ) {
			denom_temp_u64      *= 10;
			++temp_32_corr_a.expo;
		}
	}

	temp_32_corr_a = Reduce_Number(num_temp_u64, denom_temp_u64,  temp_32_corr_a.expo );

	if ( function == sin_ ) {
		if ( cordic_test < 0 ) {
			return min_x( temp_32_corr_a );
		}
	}
	if ( function == cos_ ) {
		if ( abs(cordic_test) > 2 ) {
			return min_x( temp_32_corr_a );
		}
	}
	if ( function == tan_ ) {
		switch (cordic_test) {

			case  3:
			case -2:
				temp_32_corr_a = min_x( temp_32_corr_a ); [[fallthrough]];
			case  2:
			case -3:
				return div_x( temp_32_corr_a );
				break;

			case  5:
			case  4:
			case -1:
				return min_x( temp_32_corr_a );
				break;

			default:
				break;
		}
	}
	if ( function == atan_ ) {
		if ( abs( cordic_test ) >= 16 ) {
			temp_32_corr_a = add( temp_32_corr_a, artan_1_3 , 1 );
			cordic_test /= 16;
		}
		if ( abs( cordic_test ) >= 4 ) {
			temp_32_corr_a = add( temp_32_corr_a, artan_1_3, -1 );
			cordic_test /= 4;
		}
		if ( abs( cordic_test ) == 2 ) {
			temp_32_corr_a = add( Pi_2, temp_32_corr_a, -1 );
		}
		if ( cordic_test < 0 ) {
			temp_32_corr_a = min_x( temp_32_corr_a );
		}
		if ( Rad_in_out == false ) {
			return mul( temp_32_corr_a, to_xx[to_deg] );
		}
	}

	return temp_32_corr_a;
}

Ratio_32 sin(Ratio_32 a) {
	temp_32_corr_a = sin_cos_tan( a );
	return cordic( sin_ );
}

Ratio_32 asin(Ratio_32 a) {

	temp_32_corr_a = test_asin_acos( a );
	if ( Error_first == true ) {
		return temp_32_corr_a;
	}

	if ( Rad_in_out == true ) {
		return add( Pi_2, acos( a ), -1 );
	}
	return add( circle_4, acos( a ), -1 );
}

Ratio_32 sinh(Ratio_32 a) {
	if ( a.expo < -8 ) { //  input <= abs(3.000e-9)
		return a;
	}
	temp_32_exp = exp(a);
	return add( temp_32_exp, temp_32_exp , -6 );
}

Ratio_32 asinh(Ratio_32 a) {
	if ( a.expo < -6 ) { //  input <= abs(3.000e-7)
		return a;
	}
	if ( a.expo < -3 ) { //  input <= abs(3.000e-4)
		return add_mul_spezial( a, cubic( a ), exp2_1_6, -1 );
	}

  if ( a.expo > 13 ) {
  	temp_32_exp = loge( abs_x( add( a, a, 1 )));
  }
  else {
		temp_32_exp = loge( add( sqrt( add( square( a ), exp2_0_1, 1 )), abs_x( a ), 1 ));	
  }
	if ( a.num < 0 ) {
		return min_x( temp_32_exp );
	}
	return temp_32_exp;

}

Ratio_32 cos(Ratio_32 a) {
	temp_32_corr_a = sin_cos_tan( a );
	return cordic( cos_ );
}

Ratio_32 test_asin_acos(Ratio_32 a) {

	if ( a.expo == 0 ) { //  input  = 0.3000 .. 1.000
		if ( abs( a.num ) > a.denom ) {
			if ( a.num > 0 ) {
				a.denom = 6;  // Error_String('^');  input >  1
			}
			else {
				a.denom = 5;  // Error_String('U');  input < -1
			}
			Error_first = true;
			return a;
		}
	}
	if ( a.expo > 0 ) {  //  input >= 3.000
		if ( a.num > 0 ) {
			a.denom = 6;  // Error_String('^');  input >  3
		}
		else {
			a.denom = 5;  // Error_String('U');  input < -3
		}
		Error_first = true;
	}
	return a;
}

Ratio_32 acos(Ratio_32 a) {

	temp_32_corr_a = test_asin_acos( a );
	if ( Error_first == true ) {
		return temp_32_corr_a;
	}

	if ( a.num == 0 ) {
		if ( Rad_in_out == false ) {
			return mul( Pi_2, to_xx[to_deg] );
		}
		return Pi_2;
	}

	if ( a.expo == 0 ) { //  input  = 0.3000 .. 1.000
		if ( abs( a.num ) == a.denom ) {
			if ( a.num < 0 ) {
				if ( Rad_in_out == false ) {
					return mul( Pi , to_xx[to_deg] );
				}
				return Pi;
			}
		}
	}
	return mul( exp2_1_2_div_x, atan( sqrt( mul( add( exp2_0_1, a, -1), div_x( add( exp2_0_1, a, 1))))));
}

Ratio_32 cosh(Ratio_32 a) {
	if ( a.expo < -8 ) { //  input <= abs(3.000e-9)
		return log_1e0;
	}
	temp_32_exp = exp(a);
	return add( temp_32_exp, temp_32_exp, 6 );
}

Ratio_32 acosh(Ratio_32 a) {
	if ( a.num < 0 ) {
		a.denom = 2;  // Error_String('u');  input < 1
		Error_first = true;
		return a;
	}
	if ( a.expo == 0 ) {
		if ( a.num == a.denom ) { //  input = 1.000
			return Null_no;
		}
		if ( a.num < a.denom ) {  //  input < 1.000
			a.denom = 2;  // Error_String('u');  input < 1
			Error_first = true;
			return a;
		}
	}
	if ( a.expo < 0 ) { //  input < 0.300
		a.denom = 2;  // Error_String('u');  input < 1
		Error_first = true;
		return a;
	}

  if ( a.expo > 13 ) {
  	return asinh( a );
  }
	temp_32_exp = add( a, log_1e0, -1 );
	return loge( add( a, sqrt( mul( temp_32_exp, add( exp2_1_2_div_x, temp_32_exp, 1 ))), 1 ));
}

Ratio_32 tan(Ratio_32 a) {
	temp_32_corr_a = sin_cos_tan( a );
	return cordic( tan_ );
}

Ratio_32 atan(Ratio_32 a) {
cordic_test = 0;
bool    reverse = false;

	if ( a.num == 0 ) {
		return a;
	}

	if ( a.num > 0 ) {
		cordic_test +=  1;
	}
	if ( a.num < 0 ) {
		cordic_test -=  1;
		a.num       *= -1;
	}
	if ( a.expo == 0 ) {
		if ( (a.num / 4) > (a.denom / 3) ) { //  input >= abs(1.333)
			cordic_test *= 2;
			a = div_x( a );
		}
	}
	if ( a.expo > 0 ) { //  input >= abs(3.000)
		cordic_test *= 2;
		a = div_x( a );

		if ( a.expo < -3 ) { //  input <= abs(3.000e-4)
			reverse = true;
		}
	}

	if ( a.expo < -3 ) { //  input <= abs(3.000e-4)
		temp_32_corr_a = add_mul_spezial( a, cubic( a ), exp2_1_3, -1 );
		if ( reverse == true ) {
			temp_32_corr_a = add( Pi_2, temp_32_corr_a, -1 );
		}

		if ( cordic_test < 0 ) {
			temp_32_corr_a = min_x( temp_32_corr_a );
		}
		if ( Rad_in_out == false ) {
			return mul( temp_32_corr_a, to_xx[to_deg] );
		}
		return temp_32_corr_a;
	}

	temp_32_corr_a = clone( a );

	if ( a.expo < 0 ) { //  input <= abs(0.300)
		temp_32_corr_b = clone( exp2_1_3 );
		cordic_test *= 4;
	}

	if ( a.expo == 0 ) {
		if ( (a.num / 3) > (a.denom / 4) ) { //  input > abs(0.750)
			temp_32_corr_b = exp2__1_3;
			cordic_test *= 16;
		}
	}

	if ( abs( cordic_test ) >= 4 ) {
		temp_32_corr_a = mul( add( a, temp_32_corr_b, 1 ), div_x( add_mul_spezial( exp2_0_1, a, min_x( temp_32_corr_b ), 1 ) ) );
	}

	if ( Debug_Level == 46 ) {
		time_test = micros();
	}

	return cordic( atan_ );
}

Ratio_32 tanh(Ratio_32 a) {
	if ( a.expo < -8 ) { //  input <= abs(3.000e-9)
		return a;
	}
	if ( a.expo == 1 ) { //  input >= 12.000
		if ( ( abs( a.num ) / 6 ) >= ( a.denom / 5 ) ) {
			if ( a.num > 0 ) {
				return log_1e0;
			}
			else {
				return log__1e0;
			}
		}
	}
	if ( a.expo > 1 ) { //  input >= 30.000
		if ( a.num > 0 ) {
			return log_1e0;
		}
		else {
			return log__1e0;
		}
	}
	temp_32_exp = exp(a);
	return mul( add( temp_32_exp, div_x( temp_32_exp ), -1 ), div_x( add( temp_32_exp, temp_32_exp, 3 )) );
}

Ratio_32 atanh(Ratio_32 a) {
  uint64_t  calc_temp_64_a_abs = abs(a.num);

	if ( a.expo < -6 ) { //  input <= abs(3.000e-7)
		return a;
	}
	if ( a.expo < -3 ) { //  input <= abs(3.000e-4)
		return add_mul_spezial( a, cubic( a ), exp2_1_3, 1 );
	}
	if ( a.expo == 0 ) { //  input  = 1.000
		if ( abs( a.num ) == a.denom ) {
			if ( a.num > 0 ) {
				return __1e90;
			}
			else {
				return min__1e90;
			}
		}
		if ( abs( a.num ) > a.denom ) {
			if ( a.num > 0 ) {
				a.denom = 6;  // Error_String('^');  input >  1
			}
			else {
				a.denom = 5;  // Error_String('U');  input < -1
			}
			Error_first = true;
			return a;
		}
	}
	if ( a.expo > 0 ) {  //  input >= 3.000
		if ( a.num > 0 ) {
			a.denom = 6;  // Error_String('^');  input >  3
		}
		else {
			a.denom = 5;  // Error_String('U');  input < -3
		}
		Error_first = true;
		return a;
	}

	expo_temp_8        = 0;
	denom_temp_u64     = abs(a.denom);
	denom_temp_u64    *= expo_10[abs(a.expo)];
	num_temp_u64       = denom_temp_u64;
	num_temp_u64      += calc_temp_64_a_abs;
	denom_temp_u64    -= calc_temp_64_a_abs;
	denom_temp_u64    *= 3;
	while ( num_temp_u64 >= denom_temp_u64 ) {
		denom_temp_u64 *= expo_10[1];
		++expo_temp_8;
	}
	num_temp_u64 *= 3;

	temp_32 = Reduce_Number( num_temp_u64, denom_temp_u64, expo_temp_8 );

	temp_32_exp = mul( loge( temp_32 ), exp2_1_2 );
	if ( a.num < 0 ) {
		return min_x( temp_32_exp );
	}
	return temp_32_exp;
}

Ratio_32 exp(Ratio_32 a) {
	int8_t  test_signum_log     = 0;
	int8_t  count_x             = 0;  // 0 .. 8  - input > 1.00
	int8_t  count_y             = 0;  // 0 .. 33 - integer

  int96_a   temp_x_0;           // Summe  
            temp_x_0.hi   = 0;
	          temp_x_0.mid  = 0;
	          temp_x_0.lo   = 0;	

  int96_a   temp_x_1;           // x^1
            temp_x_1.hi   = 0;
	          temp_x_1.mid  = 0;
	          temp_x_1.lo   = 0;	

  int96_a   temp_denom;
            temp_denom.hi   = 0;
	          temp_denom.mid  = 0;
	          temp_denom.lo   = 0;	

	if ( a.num > 0 ) {
		test_signum_log =  1;
		if ( a.expo > 2 ) {   //  input >=  300
			a.expo =  115;
			return a;
		}
	}
	if ( a.num < 0 ) {
		test_signum_log = -1;
		if ( a.expo > 2 ) {   //  input =< -300
			a.expo = -115;
			return a;
		}
	}

	temp_32_log = abs_x(a);

	if ( test_signum_log == 0 ) { //  input =   0.000
		return log_1e0;
	}

	if ( a.expo < -11 ) { //  input =   0.000
		return log_1e0;
	}

	if ( a.expo == 0 ) { //  input = 1.000 or -1.000
		if ( a.num == a.denom ) {
			return one_e;
		}
		if ( -(a.num) == a.denom ) {
			return one_e_div_x;
		}
	}
	
  temp_x_1    = a.num;  
  temp_denom  = a.denom;

  while ( temp_32_log.expo > 0 ) {
  	temp_32_log.expo  -= 1;
  	temp_x_1          *= 10;
  }

	count_y   = temp_32_log.expo;
	count_y  *= 3;
	count_y  += 33;
	
	if ( temp_32_log.expo == 0 ) {
		while ( temp_x_1 > temp_denom ) {
			count_x    += 1;
			temp_denom *= 2;
		}
	}
	
	while ( temp_32_log.expo < 0 ) {
		temp_32_log.expo += 1;
		temp_denom *= 10;
	}
	
	temp_denom <<= count_y;
	
	temp_x_0     = temp_denom;
	temp_x_0    += temp_x_1;

	while ( temp_x_0.hi < 1073741824 ) {
		temp_x_0    *= 2;
		temp_x_1    *= 2;
		temp_denom  *= 2;
	}

	if ( Debug_Level == 58 ) {
		itoa_(temp_x_0, display_string_itoa_);
		Serial.println(display_string_itoa_);
		itoa_(temp_x_1, display_string_itoa_);
		Serial.println(display_string_itoa_);
		itoa_(temp_denom, display_string_itoa_);
		Serial.println(display_string_itoa_);
		Serial.println("------");
	}

	temp_x_0.mul_div95(temp_denom, temp_x_0);
	temp_x_1.mul_div95(temp_x_1, temp_x_1);
	temp_denom.mul_div95(temp_denom, temp_denom);

	temp_x_1  /= 2;
	temp_x_0  += temp_x_1;
	
	while ( count_y > 0 ) {
		count_y -= 1;
		
		if ( temp_x_0.hi < 1073741824 ) {
			temp_x_0    *= 2;
			temp_denom  *= 2;
		}
		
		temp_x_0.mul_div95(temp_x_0, temp_x_0);
		temp_denom.mul_div95(temp_denom, temp_denom);

		if ( (count_wait % 20) == 19 ) {
			display_b[12] = led_font[(count % 12) + 1];
			++count;
			Display_change = true;
		}
		++count_wait;
	}	

	while ( temp_x_0.hi > 0 ) {
		temp_x_0    /= 2;
		temp_denom  /= 2;
	}
	temp_x_0    /= 2;
	temp_denom  /= 2;

	if ( Debug_Level == 58 ) {
		itoa_(temp_x_0, display_string_itoa_);
		Serial.println(display_string_itoa_);
		itoa_(temp_denom, display_string_itoa_);
		Serial.println(display_string_itoa_);
		Serial.println("------");
	}

	num_temp_u64    = temp_x_0;
	denom_temp_u64  = temp_denom;
		
	temp_32_log = Reduce_Number( num_temp_u64, denom_temp_u64, 0 );
	
	while ( count_x > 0 ) {
		count_x -= 1;
		temp_32_log = square( temp_32_log );
	}

	if ( test_signum_log < 0 ) {
		return div_x( temp_32_log );
	}

	return temp_32_log;
}

Ratio_32 exp2(Ratio_32 a) {
	return exp( mul( lb_to_e, a ));
}

Ratio_32 exp10(Ratio_32 a) {
	return exp( mul( ln_to_10, a ));
}

Ratio_32 powx_extra(Ratio_32 base, Ratio_32 expo_) {
	temp_32 = frac( expo_ );
	if ( temp_32.denom < 9 ) {
		return temp_32;
	}
	if ( temp_32.num == 0 ) {
		return powx_(base, expo_);
	}
	else {
		if ( base.num >= 0 ) {
			return powx(base, expo_);
		}
		else {
			return min_x(powx(min_x(base), expo_));
		}
	}
}

Ratio_32 powx_(Ratio_32 base, Ratio_32 expo_) {
	Ratio_32 result = clone( log_1e0 );
	uint64_t      expo   = abs( expo_.num );
								expo  *= expo_10[ expo_.expo ];
								expo  /= expo_.denom;
	uint32_t   expo_32   = expo;
	uint8_t    wait_8    = 0;

	while (expo_32 > 0) {
		if ((expo_32 & 0x1UL) == 0x1UL) {
			result  = mul( result, base );
			++wait_8;
		}

		expo_32 = expo_32 >> 1;
		base = square( base );
		++wait_8;
	}

	if ( expo_.num < 0 ) {
		return div_x( result );
	}
	return result;
}

Ratio_32 powx(Ratio_32 a, Ratio_32 b) {
	if ( b.num == 0 ) {
		return clone( log_1e0 );
	}

	if ( a.num > 0 ) {
		if ( b.expo == 0 ) {
			if ( b.num == b.denom ) {
				return a;
			}
		}
		temp_32_pow = mul( log2(a), b );
		temp_32_ext = mul( log_1e0, temp_32_pow );

		temp_32_pow = abs_x(temp_32_pow);

		if ( temp_32_pow.expo == -9 ) { //  input < 1e-9
			if ( temp_32_pow.num < temp_32_pow.denom ) {
				temp_32_pow.denom = 3;   // Error_String('X');
				return temp_32_pow;
			}
		}

		if ( temp_32_pow.expo > 3 ) { //  input > 3000
			temp_32_pow.denom = 4;   // Error_String('[');
			return temp_32_pow;
		}

		if ( temp_32_pow.expo >=  0 ) {
			temp_32_pow = mul( exp2_1_32, temp_32_pow );
		}

		if ( temp_32_pow.expo ==  2 ) { //  input > 1066_2/3
			temp_32_pow.denom = 4;   // Error_String('[');
			return temp_32_pow;
		}

		if ( temp_32_pow.expo ==  1 ) { //  input > 320
			if ( temp_32_pow.num > temp_32_pow.denom ) {
				temp_32_pow.denom = 4;   // Error_String('[');
				return temp_32_pow;
			}
		}

		temp_32_pow = exp2(temp_32_ext);
		return temp_32_pow;
	}
	else {
		if ( b.expo == 0 ) {
			if ( b.num == b.denom ) {
				return a;
			}
		}
		a.denom = 2;  // 'u'
	}
	return a;
}

Ratio_32 logx(Ratio_32 a, Ratio_32 b) {
	if ( (a.num > 0) && (b.num > 0) ) {
		if ( b.expo == 0 ) {
			if ( b.num == b.denom ) {
				a.denom = 1;  // 'I';
				return a;
			}
		}
		temp_32_ext = div_x( log2( b ));
		return mul( temp_32_ext, log2(a) );
	}
	else {
		a.denom = 2;  // 'u'
	}
	return a;
}

Ratio_32 log2(Ratio_32 a) {
	
  int96_a   temp_num;
            temp_num.hi   = 2147483647;  // 2^95 - 1
	          temp_num.mid  = 4294967295; //  
	          temp_num.lo   = 4294967295;	

  int96_a   temp_denom;
            temp_denom.hi    = 0;
	          temp_denom.mid   = 0;
	          temp_denom.lo    = 0;	

  int96_a   log2_denom;
            log2_denom.hi    = 0;
	          log2_denom.mid   = 0; //  
	          log2_denom.lo    = 0;	

  int96_a   log2_sum;         // 
            log2_sum.hi      = 0;
	          log2_sum.mid     = 0; //  
	          log2_sum.lo      = 0;	

  int96_a   log2_sum_2;         // 
            log2_sum_2.hi    = 0;
	          log2_sum_2.mid   = 0; //  
	          log2_sum_2.lo    = 0;	

  int96_a   log2_add;          // 1.0, 0.5, 0.25, 0.125 ..
            log2_add.hi      = 1;
	          log2_add.mid     = 0; //  
	          log2_add.lo      = 0;	

	int8_t    test_signum_log  = 0;
	int8_t    test_temp_8      = 0;
	
  int96_a   log2_num;
            log2_num.hi      = 3; // log2(10) = 3,32192809488736234787032
	          log2_num.mid     = 1382670639; //   3,32192809488736234775413  -3,498e-20
	          log2_num.lo      = 879635447;
  
	int32_t   log2_denom_x     = 1;
	int8_t    log2_expo        = 0;

	if ( a.num > 0 ) {
		if ( a.expo > 0 ) {
			test_signum_log =  1;
		}
		if ( a.expo < 0 ) {
			test_signum_log = -1;
			a = div_x( a );
		}
		
		if ( a.expo == 0 ) {
			if ( a.num == a.denom ) { //  input = 1.000
				return Null_no;
			}
		}

		log2_num *= a.expo;
		
		if ( a.num > a.denom ) {
			test_temp_8 =  1;                        // 1.0 .. 2.0
			if ( (a.num - a.denom) >= a.denom ) {
				log2_num    += log2_add;
				a.denom     *= 2;
				if ( a.num == a.denom ) {
					test_temp_8  =  2;	                 // 2.0 .. 4.0			
				}
			}
		}
		if ( a.denom > a.num ) {
			test_temp_8   = -1;                      // 0.5 .. 1.0  
			if ( (a.denom - a.num) >= a.num ) {
				log2_num    -= log2_add;
				a.num       *= 2;
				if ( a.denom == a.num ) {
					test_temp_8  = -2;                   // 0,25 .. 0.5				
					if ( test_signum_log == 0 ) {
						test_signum_log = -1;
						log2_num        = log2_add;
					}
				} 
			}
		}

		if ( Debug_Level == 57 ) {
			Serial.print("a_expo   = ");
			Serial.println(a.expo);
			Serial.print("test_signum_log   = ");
			Serial.println(test_signum_log);
			Serial.print("test_temp_8   = ");
			Serial.println(test_temp_8);
		}

		if ( a.expo > 10 ) {
			log2_denom_x   = 100;
			log2_expo      = 2;
		}	
		else {
			if ( a.expo > 1 ) {
				log2_denom_x   = 10;
				log2_expo      = 1;
			}
		}
			
		if ( a.num > a.denom ) {   // 1.0 .. 2.0
			a = div_x( a );
		}
		
    if ( abs( test_temp_8 ) == 1 ) {
    	temp_num     /= a.denom;   // mul
    	temp_denom    = temp_num;  // mul
    	
    	temp_num     *= a.num;
    	temp_denom   *= a.denom;

			if ( Debug_Level == 57 ) {
				Serial.print("a_num   = ");
				Serial.println(a.num);
				Serial.print("a_denom = ");
				Serial.println(a.denom);
				Serial.print("temp_num_hi   = ");
				Serial.println(temp_num.hi);
				Serial.print("temp_denom_hi = ");
				Serial.println(temp_denom.hi);
			}

			for ( uint8_t index_b = 0; index_b < 64; index_b += 1 ) {
				
				temp_num.mul_div95(temp_num, temp_num);
				temp_denom.mul_div95(temp_denom, temp_denom);

				if ( (count_wait % 30) == 24 ) {
					display_b[12] = led_font[(count % 12) + 1];
					++count;
					Display_change = true;
				}
				++count_wait;

				log2_add /= 2;
				
				if ( (temp_denom.hi - temp_num.hi) > temp_num.hi ) {
					temp_num  *= 2;
					log2_sum  += log2_add;
					if ( Debug_Level == 57 ) {
						Serial.print("1");
					}
				}
				else {
					if ( Debug_Level == 57 ) {
						Serial.print("0");
					}
				}
			}		
   }
		
		log2_denom.hi   = log2_denom_x;
		log2_denom.mid  = 0; //  
		log2_denom.lo   = 0;	
		
		if ( test_temp_8 == 1 ) {
			log2_num += log2_sum;
		}

		if ( test_temp_8 == -1 ) {
			log2_num -= log2_sum;			
		}
		
		while ( log2_num.hi == 0 ) {
			log2_num    *= 10;
			log2_add    *= 10;
			log2_expo   -= 1;
			if ( log2_num.hi > 3 ) {  // 3.99 .. 0.4
				log2_denom  *= 10;
				log2_expo   += 1;				
			}
		}		

		while ( log2_add.lo > 1 ) {
			log2_add /= 2;

			if ( temp_denom.hi < 1073741824 ) {
				temp_num    *= 2;
				temp_denom  *= 2;
			}

			temp_num.mul_div95(temp_num, temp_num);
			temp_denom.mul_div95(temp_denom, temp_denom);

			if ( (count_wait % 30) == 24 ) {
				display_b[12] = led_font[(count % 12) + 1];
				++count;
				Display_change = true;
			}
			++count_wait;

			if ( (temp_denom.hi - temp_num.hi) > temp_num.hi ) {
				temp_num   *= 2;
				log2_sum_2 += log2_add;
				if ( Debug_Level == 57 ) {
					Serial.print("1");
				}
			}
			else {
				if ( Debug_Level == 57 ) {
					Serial.print("0");
				}
			}
		}		
		
		if ( Debug_Level == 57 ) {
			Serial.println("  ");
		}
		
		if ( test_temp_8 == 1 ) {
			log2_num += log2_sum_2;
		}

		if ( test_temp_8 == -1 ) {
			log2_num -= log2_sum_2;			
		}
		
		if ( Debug_Level == 57 ) {
			Serial.print("num_hi_u32    = ");
			Serial.println(log2_num.hi);
			Serial.print("num_mid_u32   = ");
			Serial.println(log2_num.mid);
			Serial.print("denom_hi_u32  = ");
			Serial.println(log2_denom.hi);
			Serial.print("denom_mid_u32 = ");
			Serial.println(log2_denom.mid);
		}

		while ( log2_num.hi > 0 ) {
			log2_num    /= 2;
			log2_denom  /= 2;
			if ( Debug_Level == 57 ) {
				Serial.println("----------------");
				Serial.print("num_hi_u32    = ");
				Serial.println(log2_num.hi);
				Serial.print("num_mid_u32   = ");
				Serial.println(log2_num.mid);
			}
		}
 		log2_num    /= 2;
		log2_denom  /= 2;
		
		num_temp_u64    = log2_num;
		denom_temp_u64  = log2_denom;
		
		if ( Debug_Level == 57 ) {
			Serial.println("----------------");
			Serial.print("num_hi_u32    = ");
			Serial.println(log2_num.hi);
			Serial.print("num_mid_u32   = ");
			Serial.println(log2_num.mid);
			Serial.println("----------------");
			Serial.print("denom_hi_u32  = ");
			Serial.println(log2_denom.hi);
			Serial.print("denom_mid_u32 = ");
			Serial.println(log2_denom.mid);
		}

		if ( num_temp_u64 == 0 ) {
			temp_32_log.num   = 0;
			temp_32_log.denom = int32_max;
			temp_32_log.expo  = 0;
		}
		else {
			temp_32_log = Reduce_Number( num_temp_u64, denom_temp_u64, log2_expo );
		}
	
		if ( test_signum_log < 0 ) {
			temp_32_log = min_x( temp_32_log );
		}

		return temp_32_log;
	}
	else {
		a.denom = 2;  // Error_String('u');  input <= 0
		Error_first = true;
		return a;
	}
}

Ratio_32 loge(Ratio_32 a) {
	if ( a.num > 0 ) {
		return mul( lb_to_e, log2(a) );
	}
	else {
		a.denom = 2; // Error_String('u');  input <= 0
		Error_first = true;
		return a;
	}
}

Ratio_32 log10(Ratio_32 a) {
	if ( a.num > 0 ) {
		return mul( lb_to_10, log2(a) );
	}
	else {
		a.denom = 2; // Error_String('u');  input <= 0
		Error_first = true;
		return a;
	}
}

Ratio_32 agm(Ratio_32 a, Ratio_32 b) {
	temp_32_a = add( abs_x(a), abs_x(b), 2 );
	temp_32_b = sqrt(mul(abs_x(a), abs_x(b)));
	for ( uint8_t index_z = 0; index_z < 9; index_z += 1 ) {
		if ((temp_32_a.num * temp_32_b.denom) == (temp_32_b.num * temp_32_a.denom)) {
			return temp_32_b;
		}
		if ((temp_32_a.num * temp_32_a1.denom) == (temp_32_a1.num * temp_32_a.denom)) {
			return temp_32_b;
		}
		temp_32_a1 = add( temp_32_a, temp_32_b, 2 );
		temp_32_b1 = sqrt(mul(temp_32_a, temp_32_b));
		if ((temp_32_a1.num * temp_32_b1.denom) == (temp_32_b1.num * temp_32_a1.denom)) {
			return temp_32_b;
		}
		if ((temp_32_a.num * temp_32_a1.denom) == (temp_32_a1.num * temp_32_a.denom)) {
			return temp_32_b;
		}
		temp_32_a  = add( temp_32_a1, temp_32_b1, 2 );
		temp_32_b  = sqrt(mul(temp_32_a1, temp_32_b1));
	}
	return temp_32_b;       //
}

Ratio_32 square(Ratio_32 a) {
	return mul(a, a);
}

Ratio_32 cubic(Ratio_32 a) {
	return mul(a, square(a));
}

void copy_input_left_right( uint8_t left, uint8_t right ) {
	mem_stack_input[ left ].num = mem_stack_input[ right ].num;
	mem_stack_input[ left ].denom = mem_stack_input[ right ].denom;
	mem_stack_input[ left ].expo = mem_stack_input[ right ].expo;
	mem_stack_input[ left ].op = mem_stack_input[ right ].op;
}

void copy_calc_left_right( uint8_t left, uint8_t right ) {
	mem_stack_calc[ left ].num = mem_stack_calc[ right ].num;
	mem_stack_calc[ left ].denom = mem_stack_calc[ right ].denom;
	mem_stack_calc[ left ].expo = mem_stack_calc[ right ].expo;
	mem_stack_calc[ left ].op = mem_stack_calc[ right ].op;
	mem_stack_calc[ left ].op_priority = mem_stack_calc[ right ].op_priority;
	mem_stack_calc[ left ].bracket = mem_stack_calc[ right ].bracket;
}

void mem_extra_left_right( uint8_t left, uint8_t right ) {
	mem_extra_stack[ left ].num   = mem_stack_input[ right ].num;
	mem_extra_stack[ left ].denom = mem_stack_input[ right ].denom;
	mem_extra_stack[ left ].expo  = mem_stack_input[ right ].expo;
	mem_extra_stack[ left ].op    = mem_stack_input[ right ].op; // temp_op_;
}

void left_right_mem_extra( uint8_t left, uint8_t right ) {
	mem_stack_input[ left ].num   = mem_extra_stack[ right ].num;
	mem_stack_input[ left ].denom = mem_extra_stack[ right ].denom;
	mem_stack_input[ left ].expo  = mem_extra_stack[ right ].expo;
	mem_stack_input[ left ].op    = mem_extra_stack[ right ].op;
}

void add_operation_to_mem( uint8_t deep_step, char op_add ) {
	if ( deep_step > 0 ) {
		if ( mem_stack_count < mem_stack_max ) {
			Constant_arithmetic = true;
			First_operation = true;
		}
	}

	if ( mem_stack_count < mem_stack_max) {
		temp_operation = Switch_Code;
	}

	if ( Start_input < Input_Memory ) {      // Input Number
		Get_Number( deep_step );
	}

	if ( Start_input == M_Plus_spezial ) {
		left_right_mem_extra( mem_stack_count, mem_extra_max_4 );
		mem_pointer  = mem_stack_count;
		Start_input  = Input_Operation_0;
		Start_input += deep_step;
		left_right_mem_extra( 0, 0 );
	}

	if ( Start_input == Display_Result ) {
		left_right_mem_extra( 0, 0 );
		mem_stack_count = 1;
		mem_pointer  = mem_stack_count;
		Start_input  = Input_Operation_0;
		Start_input += deep_step;
		copy_input_left_right( mem_pointer, 0 );
	}

	if ( Start_input > Input_Fraction ) {
		if ( Start_input < Input_Operation_5 ) {
			Start_input  = Input_Operation_0;
			if ( mem_stack_count < mem_stack_max) {
				mem_save = false;
				mem_exchange = false;
				Error_Test();
				if ( Start_input != Display_Error ) {
					if ( display_string[Operation] == op_add ) {
						if ( Std_mode == 0 ) {
							if ( display_string[Expo_point] == ' ' ) {
								display_string[Expo_point] = '.';
							}
							else {
								display_string[Expo_point] = ' ';
							}
						}
						if ( display_string[Memory_1] != 'O' ) {
							// Display_Memory_x( 25 );
						}
					}
					else {
						mem_stack_input[ mem_pointer ].op = op_add;
						if ( display_string[Memory_1] == 'O' ) {
							display_string[Operation] = op_add;
							display_string[Expo_point] = ' ';
						}
						else {
							Display_Number(mem_stack_input[mem_pointer]);
						}
						display_string[Operation_point] = '.';
					}
				}
				Start_input += deep_step;
			}
			else {
				if ( max_input == false ) {
					max_input = true;
					if ( mem_stack_max == 2 ) {
						pgm_count_a = 2;
						pgm_content_a[2] = 61;            // _=_
						pgm_content_a[1] = Switch_Code;   // + - * /
					}
					else {
						mem_stack_input[ mem_pointer ].op = 'x';
						Display_Number(mem_stack_input[mem_pointer]);
					}
				}
			}
		}
	}
	Display_Memory_x( 23 );
}

void Test_all_function() {
	if ( Debug_Level == 18 ) {
		time_start = millis();

		for ( uint16_t index = 100; index <= 10000; index += 1 ) {

			Serial.print(index);
			Serial.print("  ");

 	  	calc_32 = to_Ratio_32(index, -2);
		 /*		 
			Serial.print(calc_32.num);
			Serial.print("  ");
			Serial.print(calc_32.denom);
			Serial.print("  ");
			Serial.print(calc_32.expo);
			Serial.println("  ");
		 */ 
		 
			test_32 = sqrt(calc_32);
			Serial.print(test_32.num);
			Serial.print("  ");
			Serial.print(test_32.denom);
			Serial.print("  ");
			Serial.print(test_32.expo);
			Serial.println(" -> ");
     
		}
		test_index = false;
		time_end = millis();
		time_diff = time_end - time_start;
		Serial.print("Time: ");
		Serial.println(time_diff);
	}
	if ( Debug_Level == 19 ) {
		num_temp_u32   = 2;
		denom_temp_u32 = 1;

		Expand_Number();

		calc_32.expo = 0;
		calc_32.num = num_temp_u32;
		calc_32.denom = denom_temp_u32;

		for ( int32_t index = 1; index <= 5; index += 1 ) {
			calc_32 = sqrt(calc_32);

			Serial.print(calc_32.num);
			Serial.print(" / ");
			Serial.print(calc_32.denom);
			Serial.print("  ");
			Serial.print(calc_32.expo);
			Serial.println(" ");

		}
		Serial.println(" ");
		for ( int32_t index = 1; index <= 5; index += 1 ) {
			calc_32 = square(calc_32);

			Serial.print(calc_32.num);
			Serial.print(" / ");
			Serial.print(calc_32.denom);
			Serial.print("  ");
			Serial.print(calc_32.expo);
			Serial.println(" ");

		}
		test_index = false;
	}
	if ( Debug_Level == 20 ) {

		calc_32.expo  = 0;
		calc_32.num   = 241053109;
		calc_32.denom = 170450288;

		Serial.println(" ");

			Serial.print(calc_32.num);
			Serial.print(" / ");
			Serial.print(calc_32.denom);
			Serial.print("  ");
			Serial.print(calc_32.expo);
			Serial.println(" ");

			calc_32 = square(calc_32);

			Serial.print(calc_32.num);
			Serial.print(" / ");
			Serial.print(calc_32.denom);
			Serial.print("  ");
			Serial.print(calc_32.expo);
			Serial.println(" ");

		test_index = false;
	}
	if ( Debug_Level == 27 ) {
		time_start = millis();

		for ( uint16_t index = 10; index <= 10000; index += 1 ) {

			Serial.print(index);
			Serial.print("  ");

 	  	calc_32 = to_Ratio_32(index, -1);
		 /*
			Serial.print(calc_32.num);
			Serial.print("  ");
			Serial.print(calc_32.denom);
			Serial.print("  ");
			Serial.print(calc_32.expo);
			Serial.print("  ");
		 */
			test_32 = cbrt(calc_32);
			Serial.print(test_32.num);
			Serial.print("  ");
			Serial.print(test_32.denom);
			Serial.print("  ");
			Serial.print(test_32.expo);
			Serial.println(" -> ");

		}
		test_index = false;
		time_end = millis();
		time_diff = time_end - time_start;
		Serial.print("Time: ");
		Serial.println(time_diff);
	}
	if ( Debug_Level == 29 ) {
		time_start = millis();

		Serial.println(" ");
		for ( uint16_t index = 19; index <= 10240; index += 1 ) {

			Serial.print(index);
			Serial.print("  ");

 	  	calc_32 = to_Ratio_32(index, -1);
		 
			Serial.print(calc_32.num);
			Serial.print("  ");
			Serial.print(calc_32.denom);
			Serial.print("  ");
			Serial.print(calc_32.expo);
			Serial.println("  ");
		 /*
			test_32 = log2(calc_32);

			Serial.print(test_32.num);
			Serial.print("  ");
			Serial.print(test_32.denom);
			Serial.print("  ");
			Serial.print(test_32.expo);
			Serial.println(" -> ");
		 */
		}
		test_index = false;
		time_end = millis();
		time_diff = time_end - time_start;
		Serial.print("Time: ");
		Serial.println(time_diff);
	}
	if ( Debug_Level == 30 ) {
		time_start = millis();

		Serial.println(" ");                        // Step    1
		for ( uint16_t index = 300; index <= 3333; index += 1 ) {

 	  	calc_32 = to_Ratio_32(index, -3);
		 
			calc_32 = mul(log_1e0, calc_32);
		 
			Serial.print(calc_32.num);
			Serial.print("  ");
			Serial.print(calc_32.denom);
			Serial.print("  ");
			Serial.print(calc_32.expo);
			Serial.print("  ");
		 
			test_32 = loge(calc_32);

			Serial.print(test_32.num);
			Serial.print("  ");
			Serial.print(test_32.denom);
			Serial.print("  ");
			Serial.print(test_32.expo);
			Serial.println(" -> ");

		}
		test_index = false;
		time_end = millis();
		time_diff = time_end - time_start;
		Serial.print("Time: ");
		Serial.println(time_diff);
	}
	if ( Debug_Level == 32 ) {
		time_start = millis();

		calc_32.expo = -9;
		num_temp_u32_   = 100;
		denom_temp_u32_ = 100;
		Serial.println(" ");                        // Step    4
		for ( uint16_t index = 1; index <= 10121; index += 1 ) {
			Serial.print(index);
			Serial.print("  ");
			num_temp_u32   = num_temp_u32_;
			denom_temp_u32 = denom_temp_u32_;
			Expand_Number();

			calc_32.num   = num_temp_u32;
			calc_32.denom = denom_temp_u32;
		 /*
			Serial.print(calc_32.num);
			Serial.print("  ");
			Serial.print(calc_32.denom);
			Serial.print("  ");
			Serial.print(calc_32.expo);
			Serial.println("  ");
		 */
			test_32 = exp2(calc_32);
			Serial.print(test_32.num);
			Serial.print("  ");
			Serial.print(test_32.denom);
			Serial.print("  ");
			Serial.print(test_32.expo);
			Serial.println(" -> ");

			++num_temp_u32_;

			if ( num_temp_u32_ == 300 ) {
				++calc_32.expo;
				denom_temp_u32_ *= 10;
			}

			if ( num_temp_u32_ == denom_temp_u32_ ) {
				num_temp_u32_   = 100;
				denom_temp_u32_ = 100;
			}
		}
		test_index = false;
		time_end = millis();
		time_diff = time_end - time_start;
		Serial.print("Time: ");
		Serial.println(time_diff);
	}
	if ( Debug_Level == 33 ) {
    time_start = millis();

    temp_32_log2.expo = -9;
    num_temp_u32_   = 100;
    denom_temp_u32_ = 100;
    Serial.println(" ");            // 3375 Step    3
    for ( uint16_t index = 1; index <= 3375; index += 1 ) {
      Serial.print(index);
      Serial.print("  ");
      num_temp_u32   = num_temp_u32_;
      denom_temp_u32 = denom_temp_u32_;
      Expand_Number();

      temp_32_log2.num = num_temp_u32;
      temp_32_log2.denom = denom_temp_u32;
		/*	
      Serial.print(temp_32_log2.num);
      Serial.print("  ");
      Serial.print(temp_32_log2.denom);
      Serial.print("  ");
      Serial.print(temp_32_log2.expo);
      Serial.print("  ");
		*/	
      temp_32_cbrt = exp2(temp_32_log2);
		
      temp_32_cbrt.num  += index;
      temp_32_cbrt.num  /= index;
      temp_32_cbrt.num  *= index;
		
      Serial.print(temp_32_cbrt.num);
      Serial.print("  ");
      Serial.print(temp_32_cbrt.denom);
      Serial.print("  ");
      Serial.print(temp_32_cbrt.expo);
      Serial.print("  ");
			
      temp_32_xxx = log2(temp_32_cbrt);

      Serial.print(temp_32_xxx.num);
      Serial.print("  ");
      Serial.print(temp_32_xxx.denom);
      Serial.print("  ");
      Serial.print(temp_32_xxx.expo);
      
      Serial.println(" -> ");

      num_temp_u32_  += 3;

      if ( num_temp_u32_ == 301 ) {
        ++temp_32_log2.expo;
        denom_temp_u32_ *= 10;
      }

      if ( num_temp_u32_ == denom_temp_u32_ ) {
        num_temp_u32_   = 100;
        denom_temp_u32_ = 100;
      }

    }
    test_index = false;
    time_end = millis();
    time_diff = time_end - time_start;
    Serial.print("Time: ");
    Serial.println(time_diff);
  }
	if ( Debug_Level == 34 ) {
		time_start = millis();

		for ( uint16_t index = 1; index <=  7100; index += 1 ) {

	  	Serial.print(index);
	  	Serial.print("  ");

 	  	temp_32_cbrt = to_Ratio_32(index, -2);
	  /* 
			Serial.print(temp_32_cbrt.num);
			Serial.print("  ");
			Serial.print(temp_32_cbrt.denom);
			Serial.print("  ");
			Serial.println(temp_32_cbrt.expo);
		*/ 
		 
			if ( (index % 100) != 0 )  {
		    Serial.print("_"); 
		    Serial.println(index);
	           
		  	test_32 = factorial(temp_32_cbrt);
     
	  		Serial.print(test_32.num);
	  		Serial.print("  ");
		  	Serial.print(test_32.denom);
	  		Serial.print("  ");
	  		Serial.print(test_32.expo);
	  		Serial.println(" -> ");
	    
		  }
		}
		test_index = false;
		time_end = millis();
		time_diff = time_end - time_start;
		Serial.print("Time: ");
		Serial.println(time_diff);
	}
	if ( Debug_Level == 36 ) {
		time_start = millis();

		calc_32.expo    = -5;
		num_temp_u32_   = 10;
		denom_temp_u32_ = 10;
		Serial.println(" ");                        // Step    4
		for ( uint16_t index = 1; index <= 641; index += 1 ) {
			Serial.print(index);
			Serial.print("  ");
			num_temp_u32   = num_temp_u32_;
			denom_temp_u32 = denom_temp_u32_;
			Expand_Number();

			calc_32.num   = num_temp_u32;
			calc_32.denom = denom_temp_u32;
		 /*
			Serial.print(calc_32.num);
			Serial.print("  ");
			Serial.print(calc_32.denom);
			Serial.print("  ");
			Serial.print(calc_32.expo);
			Serial.println("  ");
		 */
			test_32 = tanh(calc_32);
			Serial.print(test_32.num);
			Serial.print("  ");
			Serial.print(test_32.denom);
			Serial.print("  ");
			Serial.print(test_32.expo);
			Serial.println(" -> ");

			++num_temp_u32_;

			if ( num_temp_u32_ == 30 ) {
				++calc_32.expo;
				denom_temp_u32_ *= 10;
			}

			if ( num_temp_u32_ == denom_temp_u32_ ) {
				if ( calc_32.expo < 1 ) {
					num_temp_u32_   = 10;
					denom_temp_u32_ = 10;
				}
			}
		}
		test_index = false;
		time_end = millis();
		time_diff = time_end - time_start;
		Serial.print("Time: ");
		Serial.println(time_diff);
	}
	if ( Debug_Level == 37 ) {
		time_start = millis();

		calc_32.expo    =     1;
		denom_temp_u32_ = 10000;
		Serial.println(" ");                        // Step    4
		for ( int16_t index = -22000; index <= 22001; index += 3 ) {
			Serial.print(index);
			Serial.print("  ");
			num_temp_u32   = abs(index);
			denom_temp_u32 = denom_temp_u32_;

			Expand_Number();

			calc_32.num   = num_temp_u32;
			calc_32.denom = denom_temp_u32;
			if ( index < 0 ) {
				calc_32.num  *= -1;
			}
		 /*
			Serial.print(calc_32.num);
			Serial.print("  ");
			Serial.print(calc_32.denom);
			Serial.print("  ");
			Serial.print(calc_32.expo);
			Serial.println("  ");
		 */
			test_32 = sinh(calc_32);
			Serial.print(test_32.num);
			Serial.print("  ");
			Serial.print(test_32.denom);
			Serial.print("  ");
			Serial.print(test_32.expo);
			Serial.println(" -> ");

			if ( index == -3001 ) {
				calc_32.expo    =    0;
				denom_temp_u32_ = 1000;
			}

			if ( index == -301 ) {
				calc_32.expo    =  -1;
				denom_temp_u32_ = 100;
			}

			if ( index == -31 ) {
				calc_32.expo    = -2;
				denom_temp_u32_ = 10;
			}

			if ( index == -4 ) {
				calc_32.expo    = -3;
				denom_temp_u32_ =  1;
			}

			if ( index == 2 ) {
				calc_32.expo    = -2;
				denom_temp_u32_ = 10;
			}

			if ( index == 29 ) {
				calc_32.expo    =  -1;
				denom_temp_u32_ = 100;
			}

			if ( index == 299 ) {
				calc_32.expo    =    0;
				denom_temp_u32_ = 1000;
			}

			if ( index == 2999 ) {
				calc_32.expo    =     1;
				denom_temp_u32_ = 10000;
			}
		}
		test_index = false;
		time_end = millis();
		time_diff = time_end - time_start;
		Serial.print("Time: ");
		Serial.println(time_diff);
	}
	if ( Debug_Level == 38 ) {
		time_start = millis();
		int8_t expo_test = -9;

		calc_32.expo    = expo_test;
		denom_temp_u32_ =   200;
		Serial.println(" ");

		while ( expo_test < 1 ) {
			for ( int16_t index = 60; index < 600; index += 1 ) {
				num_temp_u32   = index;
				denom_temp_u32 = denom_temp_u32_;

				Expand_Number();

				calc_32.num   = num_temp_u32;
				calc_32.denom = denom_temp_u32;
				calc_32.expo  = expo_test;

				if ( expo_test == 0 ) {
					if ( num_temp_u32 < denom_temp_u32 ) {

						Serial.print(calc_32.num);
						Serial.print("  ");
						Serial.print(calc_32.denom);
						Serial.print("  ");
						Serial.print(calc_32.expo);
						Serial.print("  ");

						test_32 = atanh(calc_32);
						Serial.print(test_32.num);
						Serial.print("  ");
						Serial.print(test_32.denom);
						Serial.print("  ");
						Serial.print(test_32.expo);
						Serial.println(" -> ");
					}
				}
				else {

					Serial.print(calc_32.num);
					Serial.print("  ");
					Serial.print(calc_32.denom);
					Serial.print("  ");
					Serial.print(calc_32.expo);
					Serial.print("  ");

					test_32 = atanh(calc_32);
					Serial.print(test_32.num);
					Serial.print("  ");
					Serial.print(test_32.denom);
					Serial.print("  ");
					Serial.print(test_32.expo);
					Serial.println(" -> ");
				}
			}
			++expo_test;
		}

		expo_test    = 0;

		denom_temp_u32_ = 20000;
		denom_temp_u32  = 20000;
			for ( uint32_t index = 19901; index < 19991; index += 1 ) {
				num_temp_u32   = index;

				Expand_Number();

				calc_32.num   = num_temp_u32;
				calc_32.denom = denom_temp_u32;

				Serial.print(calc_32.num);
				Serial.print("  ");
				Serial.print(calc_32.denom);
				Serial.print("  ");
				Serial.print(calc_32.expo);
				Serial.print("  ");

				test_32 = atanh(calc_32);
				Serial.print(test_32.num);
				Serial.print("  ");
				Serial.print(test_32.denom);
				Serial.print("  ");
				Serial.print(test_32.expo);
				Serial.println(" -> ");

				denom_temp_u32  = denom_temp_u32_;
			}

		denom_temp_u32_ = 200000;
		denom_temp_u32  = 200000;
			for ( uint32_t index = 199901; index < 199991; index += 1 ) {
				num_temp_u32   = index;

				Expand_Number();

				calc_32.num   = num_temp_u32;
				calc_32.denom = denom_temp_u32;

				Serial.print(calc_32.num);
				Serial.print("  ");
				Serial.print(calc_32.denom);
				Serial.print("  ");
				Serial.print(calc_32.expo);
				Serial.print("  ");

				test_32 = atanh(calc_32);
				Serial.print(test_32.num);
				Serial.print("  ");
				Serial.print(test_32.denom);
				Serial.print("  ");
				Serial.print(test_32.expo);
				Serial.println(" -> ");

				denom_temp_u32  = denom_temp_u32_;
			}

		denom_temp_u32_ = 2000000;
		denom_temp_u32  = 2000000;
			for ( uint32_t index = 1999901; index < 1999991; index += 1 ) {
				num_temp_u32   = index;

				Expand_Number();

				calc_32.num   = num_temp_u32;
				calc_32.denom = denom_temp_u32;

				Serial.print(calc_32.num);
				Serial.print("  ");
				Serial.print(calc_32.denom);
				Serial.print("  ");
				Serial.print(calc_32.expo);
				Serial.print("  ");

				test_32 = atanh(calc_32);
				Serial.print(test_32.num);
				Serial.print("  ");
				Serial.print(test_32.denom);
				Serial.print("  ");
				Serial.print(test_32.expo);
				Serial.println(" -> ");

				denom_temp_u32  = denom_temp_u32_;
			}

		denom_temp_u32_ = 20000000;
		denom_temp_u32  = 20000000;
			for ( uint32_t index = 19999901; index < 19999991; index += 1 ) {
				num_temp_u32   = index;

				Expand_Number();

				calc_32.num   = num_temp_u32;
				calc_32.denom = denom_temp_u32;

				Serial.print(calc_32.num);
				Serial.print("  ");
				Serial.print(calc_32.denom);
				Serial.print("  ");
				Serial.print(calc_32.expo);
				Serial.print("  ");

				test_32 = atanh(calc_32);
				Serial.print(test_32.num);
				Serial.print("  ");
				Serial.print(test_32.denom);
				Serial.print("  ");
				Serial.print(test_32.expo);
				Serial.println(" -> ");

				denom_temp_u32  = denom_temp_u32_;
			}

		denom_temp_u32_ = 200000000;
		denom_temp_u32  = 200000000;
			for ( uint32_t index = 199999901; index < 199999991; index += 1 ) {
				num_temp_u32   = index;

				Expand_Number();

				calc_32.num   = num_temp_u32;
				calc_32.denom = denom_temp_u32;

				Serial.print(calc_32.num);
				Serial.print("  ");
				Serial.print(calc_32.denom);
				Serial.print("  ");
				Serial.print(calc_32.expo);
				Serial.print("  ");

				test_32 = atanh(calc_32);
				Serial.print(test_32.num);
				Serial.print("  ");
				Serial.print(test_32.denom);
				Serial.print("  ");
				Serial.print(test_32.expo);
				Serial.println(" -> ");

				denom_temp_u32  = denom_temp_u32_;
			}


		denom_temp_u32_ = 2000000000;
		denom_temp_u32  = 2000000000;
		expo_test       = 0;
			for ( uint32_t index = 1999999901; index <= 1999999999; index += 1 ) {
				num_temp_u32  = index;
				calc_32.expo  = expo_test;

				Expand_Number();

				calc_32.num   = num_temp_u32;
				calc_32.denom = denom_temp_u32;

				Serial.print(calc_32.num);
				Serial.print("  ");
				Serial.print(calc_32.denom);
				Serial.print("  ");
				Serial.print(calc_32.expo);
				Serial.print("  ");

				test_32 = atanh(calc_32);
				Serial.print(test_32.num);
				Serial.print("  ");
				Serial.print(test_32.denom);
				Serial.print("  ");
				Serial.print(test_32.expo);
				Serial.println(" -> ");

				denom_temp_u32  = denom_temp_u32_;
			}

		test_index = false;
		time_end = millis();
		time_diff = time_end - time_start;
		Serial.print("Time: ");
		Serial.println(time_diff);
	}
	if ( Debug_Level == 39 ) {
		time_start = millis();

		uint64_t index_test      = 2000;
		uint64_t index_count     = 0;
		uint64_t index_count_add = 1;

		int8_t index_expo        = 0;

		uint64_t num_temp_u64_x   = 2000000000;
		uint64_t denom_temp_u64_x = 2000000000;

		for ( uint8_t index = 0; index < 8; index += 1 ) {
			while ( index_count < index_test ) {

				if ( num_temp_u64_x > 6000000000 ) {
					denom_temp_u64_x = 20000000000;
					index_expo = 1;
				}

				num_temp_u64 = num_temp_u64_x;
				denom_temp_u64 = denom_temp_u64_x;
				calc_32 = Reduce_Number( num_temp_u64, denom_temp_u64, index_expo );

				Serial.print(calc_32.num);
				Serial.print("  ");
				Serial.print(calc_32.denom);
				Serial.print("  ");
				Serial.print(calc_32.expo);
				Serial.print("  ");
				test_32 = acosh(calc_32);
				Serial.print(test_32.num);
				Serial.print("  ");
				Serial.print(test_32.denom);
				Serial.print("  ");
				Serial.print(test_32.expo);
				Serial.println("    ");

				num_temp_u64_x += index_count_add;
				index_count    += index_count_add;
			}
			index_count_add *= expo_10[1];
			index_test      *= expo_10[1];
		}

		test_index = false;
		time_end = millis();
		time_diff = time_end - time_start;
		Serial.print("Time: ");
		Serial.println(time_diff);
	}
	if ( Debug_Level == 41 ) {
		time_start = millis();

		num_temp_u32_   =   1;
		denom_temp_u32_ = 200;
		Serial.println(" ");            //
		for ( uint16_t index = 0; index <= 18000; index += 1 ) {
			temp_32_cbrt.expo = 0;
			Serial.print(index);
			Serial.print("  ");
			num_temp_u32   = num_temp_u32_;
			num_temp_u32  *= index;
			denom_temp_u32 = denom_temp_u32_;
			if ( num_temp_u32 < 600 ) {
				num_temp_u32 *= 10;
				--temp_32_cbrt.expo;
			}
			if ( num_temp_u32 < 61 ) {
				num_temp_u32 *= 10;
				--temp_32_cbrt.expo;
			}
			if ( num_temp_u32 > 600 ) {
				denom_temp_u32 *= 10;
				++temp_32_cbrt.expo;
			}
			if ( num_temp_u32 > 6000 ) {
				denom_temp_u32 *= 10;
				++temp_32_cbrt.expo;
			}
			Expand_Number();

			temp_32_cbrt.num   = num_temp_u32;
			temp_32_cbrt.denom = denom_temp_u32;

			if ( index == 0 ) {
				temp_32_cbrt.num  = temp_32_cbrt.denom;
				temp_32_cbrt.expo = -20;
			}

			temp_32_xxx = sin(temp_32_cbrt);
	 /*
			Serial.print(temp_32_cbrt.num);
			Serial.print("  ");
			Serial.print(temp_32_cbrt.denom);
			Serial.print("  ");
			Serial.print(temp_32_cbrt.expo);
			Serial.println("  ");
	 */
			Serial.print(temp_32_xxx.num);
			Serial.print("  ");
			Serial.print(temp_32_xxx.denom);
			Serial.print("  ");
			Serial.print(temp_32_xxx.expo);
			Serial.println(" -> ");

		}
		test_index = false;
		time_end = millis();
		time_diff = time_end - time_start;
		Serial.print("Time: ");
		Serial.println(time_diff);
	}
	if ( Debug_Level == 42 ) {
		time_start = millis();

		num_temp_u32_   =   1;
		denom_temp_u32_ = 200;
		Serial.println(" ");            //
		for ( uint16_t index = 1; index < 18000; index += 1 ) {
			temp_32_cbrt.expo = 0;
			Serial.print(index);
			Serial.print("  ");
			num_temp_u32   = num_temp_u32_;
			num_temp_u32  *= index;
			denom_temp_u32 = denom_temp_u32_;
			if ( num_temp_u32 < 600 ) {
				num_temp_u32 *= 10;
				--temp_32_cbrt.expo;
			}
			if ( num_temp_u32 < 61 ) {
				num_temp_u32 *= 10;
				--temp_32_cbrt.expo;
			}
			if ( num_temp_u32 > 600 ) {
				denom_temp_u32 *= 10;
				++temp_32_cbrt.expo;
			}
			if ( num_temp_u32 > 6000 ) {
				denom_temp_u32 *= 10;
				++temp_32_cbrt.expo;
			}
			Expand_Number();

			temp_32_cbrt.num   = num_temp_u32;
			temp_32_cbrt.denom = denom_temp_u32;

			temp_32_xxx = tan(temp_32_cbrt);
	 /*
			Serial.print(temp_32_cbrt.num);
			Serial.print("  ");
			Serial.print(temp_32_cbrt.denom);
			Serial.print("  ");
			Serial.print(temp_32_cbrt.expo);
			Serial.println("  ");
	 */
			Serial.print(temp_32_xxx.num);
			Serial.print("  ");
			Serial.print(temp_32_xxx.denom);
			Serial.print("  ");
			Serial.print(temp_32_xxx.expo);
			Serial.println(" -> ");

		}
		test_index = false;
		time_end = millis();
		time_diff = time_end - time_start;
		Serial.print("Time: ");
		Serial.println(time_diff);
	}
	if ( Debug_Level == 44 ) {
		time_start = millis();

		Serial.println(" ");            //
		for ( uint16_t index = 1; index < 10001; index += 1 ) {
			num_temp_u32   = index;
			denom_temp_u32 = 10000;
			temp_32_cbrt.expo = 0;
			Serial.print(index);
			Serial.print("  ");
			if ( num_temp_u32 < 4 ) {
				num_temp_u32 *= 10;
				--temp_32_cbrt.expo;
			}
			if ( num_temp_u32 < 31 ) {
				num_temp_u32 *= 10;
				--temp_32_cbrt.expo;
			}
			if ( num_temp_u32 < 301 ) {
				num_temp_u32 *= 10;
				--temp_32_cbrt.expo;
			}
			if ( num_temp_u32 < 3001 ) {
				num_temp_u32 *= 10;
				--temp_32_cbrt.expo;
			}
			Expand_Number();

			temp_32_cbrt.num   = num_temp_u32;
			temp_32_cbrt.denom = denom_temp_u32;

			temp_32_xxx = atan( temp_32_cbrt );
	 /*
			Serial.print(temp_32_cbrt.num);
			Serial.print("  ");
			Serial.print(temp_32_cbrt.denom);
			Serial.print("  ");
			Serial.print(temp_32_cbrt.expo);
			Serial.println("  ");
	 */
			Serial.print(temp_32_xxx.num);
			Serial.print("  ");
			Serial.print(temp_32_xxx.denom);
			Serial.print("  ");
			Serial.print(temp_32_xxx.expo);
			Serial.println(" -> ");

		}
		test_index = false;
		time_end = millis();
		time_diff = time_end - time_start;
		Serial.print("Time: ");
		Serial.println(time_diff);
	}
	if ( Debug_Level == 45 ) {
		time_start = millis();

		Serial.println(" ");            //
		for ( uint16_t index = 1; index < 10001; index += 1 ) {
			num_temp_u32   = index;
			denom_temp_u32 = 10000;
			temp_32_cbrt.expo = 1;
			Serial.print(index);
			Serial.print("  ");
			if ( num_temp_u32 < 4 ) {
				num_temp_u32 *= 10;
				--temp_32_cbrt.expo;
			}
			if ( num_temp_u32 < 31 ) {
				num_temp_u32 *= 10;
				--temp_32_cbrt.expo;
			}
			if ( num_temp_u32 < 301 ) {
				num_temp_u32 *= 10;
				--temp_32_cbrt.expo;
			}
			if ( num_temp_u32 < 3001 ) {
				num_temp_u32 *= 10;
				--temp_32_cbrt.expo;
			}
			Expand_Number();

			temp_32_cbrt.num   = num_temp_u32;
			temp_32_cbrt.denom = denom_temp_u32;

			temp_32_xxx = atan( temp_32_cbrt );
	 /*
			Serial.print(temp_32_cbrt.num);
			Serial.print("  ");
			Serial.print(temp_32_cbrt.denom);
			Serial.print("  ");
			Serial.print(temp_32_cbrt.expo);
			Serial.println("  ");
	 */
			Serial.print(temp_32_xxx.num);
			Serial.print("  ");
			Serial.print(temp_32_xxx.denom);
			Serial.print("  ");
			Serial.print(temp_32_xxx.expo);
			Serial.println(" -> ");

		}
		test_index = false;
		time_end = millis();
		time_diff = time_end - time_start;
		Serial.print("Time: ");
		Serial.println(time_diff);
	}
	if ( Debug_Level == 46 ) {
		time_start = millis();

		Serial.println(" ");            //
		for ( uint16_t index = 8330; index < 18751; index += 1 ) {  // 8330
			num_temp_u32   = index;
			denom_temp_u32 = 25000;
			temp_32_cbrt.expo = 0;
			Serial.print(index);
			Serial.print("  ");
			Expand_Number();

			temp_32_cbrt.num   = num_temp_u32;
			temp_32_cbrt.denom = denom_temp_u32;
	 /*
			Serial.print(temp_32_cbrt.num);
			Serial.print("  ");
			Serial.print(temp_32_cbrt.denom);
			Serial.print("  ");
			Serial.print(temp_32_cbrt.expo);
			Serial.println("  ");
	 */
			temp_32_xxx = atan( temp_32_cbrt );
		}
		test_index = false;
		time_end = millis();
		time_diff = time_end - time_start;
		Serial.print("Time: ");
		Serial.println(time_diff);
	}
}

void Test_Switch_up_down() {
	if ( Debug_Level == 63 ) {
		Serial.print("BitBool_test_1  ");
		Display_Status = 0;
		
		Serial.println(Display_Status);
	}			

	if ( Switch_down < Switch_old ) {
			Switch_delta  = Switch_old;
			Switch_delta -= Switch_down;

			switch (Switch_delta) {   // change  -->   Display_Status_new

				case 1:
					Display_Status_new[3] = 0; //       "EE"       Write to the bit.
					if ( Display_Status_new[5] == 1 ) {  //     "=" 
						Display_Status_new[6] = 1;        //     "M+"
						Display_Status_mem[2] = 1;           //     "+/-"
					}
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_2  ");
						Serial.println(Display_Status);
					}			
					break;

				case 2:
					Display_Status_new[4] = 0;           //     "FN"       Write to the bit.
					if ( Display_Status_new[5] == 1 ) {
						Display_Status_new[6] = 1;         //     "M+"
						Display_Status_mem[2] = 1;           //     "+/-"
					}
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_3  ");
						Serial.println(Display_Status);
					}			
					break;

				case 4:
					Display_Status_new[5] = 0;           //     "="        Write to the bit.
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_4  ");
						Serial.println(Display_Status);
					}			
					break;

				case 3:
				case 5:
				case 6:
				case 7:
					Display_Status_new[3] = 0;           //     "EE"       Write to the bit.
					Display_Status_new[4] = 0;           //     "FN"       Write to the bit.
					Display_Status_new[5] = 0;           //     "="        Write to the bit.
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_5  ");
						Serial.println(Display_Status);
					}			
					break;

				case 2048:       //     _)_
				case 16:         //     _+_
				case 32:         //     _-_
				case 128:        //     _x_				
				case 256:        //     _/_
				case 1024:       //     _(_
					if ( Display_Status_new[5] == 1 ) {
						Display_Status_new[4] = 0;
						Display_Status_new[6] = 1;
					}
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_6  ");
						Serial.println(Display_Status);
					}			
					break;

				case 4096:             //  1
					Display_Status_new[0] = 0;           //     "0"        Write to the bit.
					if ( Display_Status_old == 25 ) {
						Mr_0_test = true;
						mem_extra_test = 0;
					}
					if ( Display_Status_new[5] == 1 ) {
						Display_Status_new[6] = 1;         //     "M+"
					}
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_7  ");
						Serial.println(Display_Status);
					}			
					break;

				case 16384:            //  4
					Display_Status_new[2] = 0;           //     "+/-"      Write to the bit.
					if ( Display_Status_new[5] == 1 ) {
						Display_Status_new[6] = 1;         //     "M+"
					}
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_8  ");
						Serial.println(Display_Status);
					}			
					break;

				case 12288:            //  3
				case 20480:            //  5
				case 24576:            //  6
				case 28672:            //  7
					Display_Status_new[0] = 0;           //     "0"        Write to the bit.
					Display_Status_new[1] = 0;           //     "."        Write to the bit.
					Display_Status_new[2] = 0;           //     "+/-"      Write to the bit.
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_9  ");
						Serial.println(Display_Status);
					}			
					break;

				case 8:                //   "1/x"
					if ( Display_mode == true ) {   //
						Switch_Code = 154;   //              Std_on_off_up
					}
					break;

				case 8192:             //  2
					Display_Status_new[1] = 0;           //     "."        Write to the bit.
					if ( Display_Status_old == 10 ) {
						Display_Status_old = Display_Status_new;
					}
					if ( Display_Status_new[1] == 1 ) {
						Display_Status_new[6] = 1;         //     "M+"
					}
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_10  ");
						Serial.println(Display_Status);
					}			
					break;

				default:
					Display_Status_old = 255;
					switch (Display_Status_new) {

						case 0:          // __
						case 4:          // +/-
						case 8:          // inv
						case 16:         // FN
						case 24:         // MR
						case 32:         // =
						case 40:         // Display
						case 48:         // MS
						case 64:         // M_plus
							Display_Status_old = Display_Status_new;
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_11  ");
						Serial.println(Display_Status);
					}			
							break;
					}
					break;
			}

		if ( is_view == true ) {
			Switch_Code = 145;
		}
	}

	if ( Switch_down > Switch_old ) {
			Switch_delta  = Switch_down;
			Switch_delta -= Switch_old;

			switch (Switch_down) {    // --->   Switch delta  -->  Zahlen

				case 1:          //    _EE_
					Switch_Code = 120;
					break;

				case 4:          //    _=_
					Display_Status_new[5] = 1;           //     "="        Write to the bit.
					if ( Display_Status_new[6] == 0 ) {  //     "M+"
						Switch_Code = 61;
					}
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_12  ");
						Serial.println(Display_Status);
					}			
					break;

				case 7:          //   "EE" + "FN"  + "="  three Switch pressed
					Switch_Code = 59;   //                  EE_FN_=
					break;

				case 517:      //    "EE" +  "=" + "CE"   three Switch pressed
				case 515:      //    "FN" + "EE" + "CE"   three Switch pressed
					Switch_Code = 91;   //          new   Light up
					break;

				case 69:       //    "EE" +  "=" + "<--"  three Switch pressed
				case 67:       //    "FN" + "EE" + "<--"  three Switch pressed
					Switch_Code = 93;   //          new   Light down
					break;

				case 13:       //    "EE" +  "=" + "1/x"  three Switch pressed       
				case 11:       //    "FN" + "EE" + "1/x"  three Switch pressed
					Switch_Code = 153;  //           new   Std_on_off
					break;

				case 1540:     //     "=" + "CE" +  "("   three Switch pressed
				case 1542:     //     "=" + "CE" +  "("   three Switch pressed
					Switch_Code = 148;  //            new   ->>
					break;

				case 4097:     //    "EE" +  "0"          two Switch pressed
					if ( Display_Status_old == 8 ) {
						Switch_Code = 190;  //                RND
					}
					break;

				case 4098:     //    "FN" +  "0"          two Switch pressed
					if ( Display_Status_old == 16 ) {
						Switch_Code = 119;  //                x!
					}
					break;

				case 8193:     //    "EE" + "."           two Switch pressed
					if ( Display_Status_old == 8 ) {
						Switch_Code = 155; //            new  DISP_Â°C
					}
					break;

				case 8194:     //    "FN" + "."           two Switch pressed
					if ( Display_Status_old == 16 ) {
						Switch_Code = 122; //            new  Beep
					}
					break;

				case 12288:     //    "0" + "."           two Switch pressed
					Switch_Code = 150; //              new  "Â° ' ''" hh:mm:ss  Input
					break;

				case 16385:     //    "EE" + "+/-"        two Switch pressed
					if ( Display_Status_old == 8 ) {
						Switch_Code = 156; //            new  DISP_Â°F
					}
					break;

				case 16386:    //    "FN"- "+/-"          two Switch pressed
					if ( Display_Status_old == 16 ) {
						Switch_Code = 121; //            new  clock
					}
					break;

				case 24576:     //    "." + "+/-"         two Switch pressed
					Switch_Code = 149; //              new  Fraction a_b/c     Input
					break;

				case 4099:     //     "EE" + "FN" + "0"   three Switch pressed
					if ( Display_Status_new == 24 ) {
						Switch_Code = 77;  //            new  MR(0)
					}
					break;

				case 4101:     //     "EE" +  "=" + "0"   three Switch pressed
					Switch_Code = 32;  //              new  FIX_dms
					break;

				case 20480:    //     "+/-" + to_xx(0)    two Switch pressed
					Switch_Code = 191;  //             new
					break;

				case 16389:    //     "EE" + "=" + "+/-"  three Switch pressed
					Switch_Code = 160;  //                  MCs
					break;

				default:
					break;
			}

			switch (Switch_delta) {   // change  -->   Display_Status_new

				case 1:
					Display_Status_new[3] = 1;           //     "EE"       Write to the bit.
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_13  ");
						Serial.println(Display_Status);
					}			
					break;

				case 2:
					Display_Status_new[4] = 1;           //     "FN"       Write to the bit.
					break;

				case 4:
					if ( Display_Status_new[3] == 1 ) {  //      "EE"
						Display_Status_new[0] = 1;           //     "="        Write to the bit.
					}
					if ( Display_Status_new[4] == 1 ) {   //     "FN"
						Display_Status_new[5] = 1;          //     "="        Write to the bit.
					}
					if ( Start_input == Display_Result ) {
						Display_Status_new[5] = 1;           //     "="        Write to the bit.
					}
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_14  ");
						Serial.println(Display_Status);
					}			
					break;

				case 3:
					Display_Status_new[3] = 1;           //     "EE"       Write to the bit.
					Display_Status_new[4] = 1;           //     "FN"       Write to the bit.
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_15  ");
						Serial.println(Display_Status);
					}			
					break;

				case 6:
					Display_Status_new[4] = 1;           //     "FN"       Write to the bit.
					Display_Status_new[5] = 1;           //     "="        Write to the bit.
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_16  ");
						Serial.println(Display_Status);
					}			
					break;

				case 5:
					Display_Status_new[3] = 1;           //     "EE"       Write to the bit.
					Display_Status_new[5] = 1;           //      "="        Write to the bit.
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_17  ");
						Serial.println(Display_Status);
					}			
					break;

				case 7:
					Display_Status_new[3] = 1;           //     "EE"       Write to the bit.
					Display_Status_new[4] = 1;           //     "FN"       Write to the bit.
					Display_Status_new[5] = 1;           //     "="        Write to the bit.
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_18  ");
						Serial.println(Display_Status);
					}			
					break;

				case 4096:             //  1
					if ( Display_Status_new == 0 ) {
						Switch_Code = 48; //     _0_
					}
					Display_Status_new[0] = 1;           //     "0"         Write to the bit.
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_19  ");
						Serial.println(Display_Status);
					}			
					break;

				case 8192:             //  2
					if ( Display_Status_new == 0 ) {
						Switch_Code = 46; //     _._
					}
					Display_Status_new[1] = 1;           //     "."         Write to the bit.
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_20  ");
						Serial.println(Display_Status);
					}			
					break;

				case 16384:            //  4
					if ( Display_Status_new == 0 ) {
						Switch_Code = 35; //     _+/-_
					}
					Display_Status_new[2] = 1;           //     "+/-"       Write to the bit.
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_21  ");
						Serial.println(Display_Status);
					}			
					break;

				case 12288:            //  3
					Display_Status_new[0] = 1;           //     "0"         Write to the bit.
					Display_Status_new[1] = 1;           //     "."         Write to the bit.
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_22  ");
						Serial.println(Display_Status);
					}			
					break;

				case 24576:            //  6
					Display_Status_new[1] = 1;           //     "."         Write to the bit.
					Display_Status_new[2] = 1;           //     "+/-"       Write to the bit.
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_23  ");
						Serial.println(Display_Status);
					}			
					break;

				case 20480:            //  5
					Display_Status_new[0] = 1;           //     "0"         Write to the bit.
					Display_Status_new[2] = 1;           //     "+/-"       Write to the bit.
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_24  ");
						Serial.println(Display_Status);
					}			
					break;

				case 28672:            //  7
					Display_Status_new[0] = 1;           //     "0"         Write to the bit.
					Display_Status_new[1] = 1;           //     "."         Write to the bit.
					Display_Status_new[2] = 1;           //     "+/-"       Write to the bit.
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_25  ");
						Serial.println(Display_Status_new);
					}			
					break;

				case 32768:       //   1
				case 98304:
					switch (Display_Status_new) {

						case 0:
							Switch_Code = 49;
							break;

						case 1:
						case 3:
							Switch_Code = 202;  //           new  EE(1)
							break;

						case 2:
							Switch_Code = 179;  //           new  M_xch(1)
							break;

						case 4:
							Switch_Code = 192;  //           new  to_xx(1)
							break;

						case 8:
							Switch_Code = 89;   //                _y_root_
							break;

						case 16:
							Switch_Code = 88;   //                _y_expo_
							break;

						case 24:
							Switch_Code = 78;  //            new  MR(1)
							break;

						case 96:
							Switch_Code = 135;  //           new  M_plus(1)
							if ( Display_Status_mem == 5 ) {
								Switch_Code -= 57; //          Mr(1)
								break;
							}
							if ( Display_Status_mem == 6 ) {
								Switch_Code += 26; //          Min(1)
								break;
							}
							if ( Display_Status_mem > 1 ) {
								Switch_Code += ( 9 * Display_Status_mem );
								Switch_Code += 72;
							}
							break;

						case 40:
							Switch_Code = 87;  //            new  FIX_a_b/c
							break;

						case 48:
						case 112:
							Switch_Code = 161;  //           new  Min(1)
							break;
					}
					break;

				case 65536:      //    2
					switch (Display_Status_new) {

						case 0:
							Switch_Code = 50;
							break;

						case 1:
						case 3:
							Switch_Code = 203;  //           new  EE(2)
							break;

						case 2:
							Switch_Code = 180;  //           new  M_xch(2)
							break;

						case 4:
							Switch_Code = 193;  //           new  to_xx(2)
							break;

						case 8:
							Switch_Code = 118;  //                sqrt()
							break;

						case 16:
							Switch_Code = 117;  //                x^2
							break;

						case 24:
							Switch_Code = 79;  //            new  MR(2)
							break;

						case 96:
							Switch_Code = 136;  //           new  M_plus(2)
							if ( Display_Status_mem == 5 ) {
								Switch_Code -= 57; //          Mr(1)
								break;
							}
							if ( Display_Status_mem == 6 ) {
								Switch_Code += 26; //          Min(1)
								break;
							}
							if ( Display_Status_mem > 1 ) {
								Switch_Code += ( 9 * Display_Status_mem );
								Switch_Code += 72;
							}
							break;

						case 40:
							Switch_Code = 95;  //            new  FIX_2
							break;

						case 48:
						case 112:
							Switch_Code = 162;  //           new  Min(2)
							break;
					}
					break;

				case 131072:     //    3
				case 196608:
					switch (Display_Status_new) {

						case 0:
							Switch_Code = 51;
							break;

						case 1:
						case 3:
							Switch_Code = 204;  //           new  EE(3)
							break;

						case 2:
							Switch_Code = 181;  //           new  M_xch(3)
							break;

						case 4:
							Switch_Code = 194;  //           new  to_xx(3)
							break;

						case 8:
							Switch_Code = 173;  //                cbrt()
							break;

						case 16:
							Switch_Code = 172;  //                x^3
							break;

						case 24:
							Switch_Code = 80;  //            new  MR(3)
							break;

						case 96:
							Switch_Code = 137;  //           new  M_plus(3)
							if ( Display_Status_mem == 5 ) {
								Switch_Code -= 57; //          Mr(1)
								break;
							}
							if ( Display_Status_mem == 6 ) {
								Switch_Code += 26; //          Min(1)
								break;
							}
							if ( Display_Status_mem > 1 ) {
								Switch_Code += ( 9 * Display_Status_mem );
								Switch_Code += 72;
							}
							break;

						case 40:
							Switch_Code = 96;  //            new  FIX_3
							break;

						case 48:
						case 112:
							Switch_Code = 163;  //           new  Min(3)
							break;
					}
					break;

				case 262144:     //    4
				case 786432:
					switch (Display_Status_new) {

						case 0:
							Switch_Code = 52;
							break;

						case 1:
						case 3:
							Switch_Code = 205;  //           new  EE(4)
							break;

						case 2:
							Switch_Code = 182;  //           new  M_xch(4)
							break;

						case 4:
							Switch_Code = 195;  //           new  to_xx(4)
							break;

						case 8:
							Switch_Code = 74;  //            new  asinh(x)
							break;

						case 16:
							Switch_Code = 71;  //                 sinh(x)
							break;

						case 24:
							Switch_Code = 81;  //            new  MR(4)
							break;

						case 96:
							Switch_Code = 138;  //           new  M_plus(4)
							if ( Display_Status_mem == 5 ) {
								Switch_Code -= 57; //          Mr(1)
								break;
							}
							if ( Display_Status_mem == 6 ) {
								Switch_Code += 26; //          Min(1)
								break;
							}
							if ( Display_Status_mem > 1 ) {
								Switch_Code += ( 9 * Display_Status_mem );
								Switch_Code += 72;
							}
							break;

						case 40:
							Switch_Code = 97;  //            new  FIX_4
							break;

						case 48:
						case 112:
							Switch_Code = 164;  //           new  Min(4)
							break;
					}
					break;

				case 524288:     //    5
					switch (Display_Status_new) {

						case 0:
							Switch_Code = 53;
							break;

						case 1:
						case 3:
							Switch_Code = 206;  //           new  EE(5)
							break;

						case 2:
							Switch_Code = 183;  //           new  M_xch(5)
							break;

						case 4:
							Switch_Code = 196;  //           new  to_xx(5)
							break;

						case 8:
							Switch_Code = 75;  //            new  acosh(x)
							break;

						case 16:
							Switch_Code = 72;  //                 cosh(x)
							break;

						case 24:
							Switch_Code = 82;  //            new  MR(5)
							break;

						case 96:
							Switch_Code = 139;  //           new  M_plus(5)
							if ( Display_Status_mem == 5 ) {
								Switch_Code -= 57; //          Mr(1)
								break;
							}
							if ( Display_Status_mem == 6 ) {
								Switch_Code += 26; //          Min(1)
								break;
							}
							if ( Display_Status_mem > 1 ) {
								Switch_Code += ( 9 * Display_Status_mem );
								Switch_Code += 72;
							}
							break;

						case 40:
							Switch_Code = 98;  //            new  FIX_5
							break;

						case 48:
						case 112:
							Switch_Code = 165;  //           new  Min(5)
							break;
					}
					break;

				case 1048576:    //    6
				case 1572864:
					switch (Display_Status_new) {

						case 0:
							Switch_Code = 54;
							break;

						case 1:
						case 3:
							Switch_Code = 207;  //           new  EE(6)
							break;

						case 2:
							Switch_Code = 184;  //           new  M_xch(6)
							break;

						case 4:
							Switch_Code = 197;  //           new  to_xx(6)
							break;

						case 8:
							Switch_Code = 76;  //            new  atanh(x)
							break;

						case 16:
							Switch_Code = 73;  //                 tanh(x)
							break;

						case 24:
							Switch_Code = 83;  //            new  MR(6)
							break;

						case 96:
							Switch_Code = 140;  //           new  M_plus(6)
							if ( Display_Status_mem == 5 ) {
								Switch_Code -= 57; //          Mr(1)
								break;
							}
							if ( Display_Status_mem == 6 ) {
								Switch_Code += 26; //          Min(1)
								break;
							}
							if ( Display_Status_mem > 1 ) {
								Switch_Code += ( 9 * Display_Status_mem );
								Switch_Code += 72;
							}
							break;

						case 40:
							Switch_Code = 99;  //            new  FIX_6
							break;

						case 48:
						case 112:
							Switch_Code = 166;  //           new  Min(6)
							break;
					}
					break;

				case 2097152:    //    7
				case 6291456:
					switch (Display_Status_new) {

						case 0:
							Switch_Code = 55;
							break;

						case 1:
						case 3:
							Switch_Code = 208;  //           new  EE(7)
							break;

						case 2:
							Switch_Code = 185;  //           new  M_xch(7)
							break;

						case 4:
							Switch_Code = 198;  //           new  to_xx(7)
							break;

						case 8:
							Switch_Code = 68;  //            new  asin(x)
							break;

						case 16:
							Switch_Code = 65;  //                 sin(x)
							break;

						case 24:
							Switch_Code = 84;  //            new  MR(7)
							break;

						case 96:
							Switch_Code = 141;  //           new  M_plus(7)
							if ( Display_Status_mem == 5 ) {
								Switch_Code -= 57; //          Mr(1)
								break;
							}
							if ( Display_Status_mem == 6 ) {
								Switch_Code += 26; //          Min(1)
								break;
							}
							if ( Display_Status_mem > 1 ) {
								Switch_Code += ( 9 * Display_Status_mem );
								Switch_Code += 72;
							}
							break;

						case 40:
							Switch_Code = 100;  //           new  FIX_7
							break;

						case 48:
						case 112:
							Switch_Code = 167;  //           new  Min(7)
							break;
					}
					break;

				case 4194304:    //    8
					switch (Display_Status_new) {

						case 0:
							Switch_Code = 56;
							break;

						case 1:
						case 3:
							Switch_Code = 209;  //           new  EE(8)
							break;

						case 2:
							Switch_Code = 186;  //           new  M_xch(8)
							break;

						case 4:
							Switch_Code = 199;  //           new  to_xx(8)
							break;

						case 8:
							Switch_Code = 69;  //            new  acos(x)
							break;

						case 16:
							Switch_Code = 66;  //                 cos(x)
							break;

						case 24:
							Switch_Code = 85;  //            new  MR(8)
							break;

						case 96:
							Switch_Code = 142;  //           new  M_plus(8)
							if ( Display_Status_mem == 5 ) {
								Switch_Code -= 57; //          Mr(1)
								break;
							}
							if ( Display_Status_mem == 6 ) {
								Switch_Code += 26; //          Min(1)
								break;
							}
							if ( Display_Status_mem > 1 ) {
								Switch_Code += ( 9 * Display_Status_mem );
								Switch_Code += 72;
							}
							break;

						case 40:
							Switch_Code = 101;  //           new  FIX_8
							break;

						case 48:
						case 112:
							Switch_Code = 168;  //           new  Min(8)
							break;
					}
					break;

				case 8388608:    //    9
				case 12582912:
					switch (Display_Status_new) {

						case 0:
							Switch_Code = 57;
							break;

						case 1:
						case 3:
							Switch_Code = 210;  //           new  EE(9)
							break;

						case 2:
							Switch_Code = 187;  //           new  M_xch(9)
							break;

						case 4:
							Switch_Code = 200;  //           new  to_xx(9)
							break;

						case 8:
							Switch_Code = 70;  //            new  atan(x)
							break;

						case 16:
							Switch_Code = 67;  //                 tan(x)
							break;

						case 24:
							Switch_Code = 86;  //            new  MR(9)
							break;

						case 96:
							Switch_Code = 143;  //           new  M_plus(9)
							if ( Display_Status_mem == 5 ) {
								Switch_Code -= 57; //          Mr(1)
								break;
							}
							if ( Display_Status_mem == 6 ) {
								Switch_Code += 26; //          Min(1)
								break;
							}
							if ( Display_Status_mem > 1 ) {
								Switch_Code += ( 9 * Display_Status_mem );
								Switch_Code += 72;
							}
							break;

						case 40:
							Switch_Code = 102;  //           new  FIX_E24
							break;

						case 48:
						case 112:
							Switch_Code = 169;  //           new  Min(9)
							break;
					}
					break;

				case 2048:       //    )
				case 3072:
					switch (Display_Status_new) {

						case 0:
							Switch_Code = 41;
							break;

						case 8:
							Switch_Code = 116;  //                10^x
							break;

						case 16:
							Switch_Code = 115;  //                log10(x)
							break;

						case 24:
						case 40:
							Switch_Code = 123;  //           new  Off
							break;

						case 32:
						case 96:
							Switch_Code = 108;  //           new   _M_()_
							Display_Status_new[4] = 1;
							Display_Status_new[6] = 0;         //     "M+"
							if ( Debug_Level == 63 ) {
								Serial.print("BitBool_test_26  ");
								Serial.println(Display_Status);
							}			
							break;
					}
					break;

				case 1024:       //    (
					switch (Display_Status_new) {

						case 0:
							Switch_Code = 40;
							break;

						case 8:
							Switch_Code = 113;  //                e^x
							break;

						case 16:
							Switch_Code = 112;  //                ln(x)
							break;

						case 32:
						case 48:
						case 96:
							Switch_Code = 175;  //                _EE-1_ ">"
							break;

						case 40:
							Switch_Code = 201;  //          new   Beep
							break;
					}
					break;

				case 512:        //   _CE_
				case 1536:
					switch (Display_Status_new) {

						case 0:
							Switch_Code = 90;
							break;

						case 8:
							Switch_Code = 114;  //                 _2^x_
							break;

						case 16:
							Switch_Code = 34;   //                 lb
							break;

						case 24:
						case 40:
							if ( Switch_old < 6 ) {
								Switch_Code = 91;   //           new   Light up
							}
							break;

						case 32:
						case 48:
						case 96:
							Switch_Code = 174;  //                _EE+1_ "<"
							break;
					}
					break;

				case 256:         //    _/_
				case 384:
					switch (Display_Status_new) {

						case 0:
							Switch_Code = 47;
							break;

						case 1:  // EE to ..
							if ( Start_input == Input_Expo ) {
								Switch_Code = 38; //           new   _EE+3_
							}
							break;

						case 8:
							Switch_Code = 128;   //                _AM_
							break;

						case 16:
							Switch_Code = 129;   //                _GM_
							break;

						case 24:
							Switch_Code = 38;   //           new   _EE+3_
							break;

						case 32:
						case 96:
							Switch_Code = 106;   //           new   M/()
							Display_Status_new[4] = 1;
							Display_Status_new[6] = 0;
							if ( Debug_Level == 63 ) {
								Serial.print("BitBool_test_27  ");
								Serial.println(Display_Status);
							}			
							break;
					}
					break;

				case 128:        //    _*_
					switch (Display_Status_new) {

						case 0:
							Switch_Code = 42;
							break;

						case 8:
							Switch_Code = 151;   //                _AGM_
							break;

						case 16:
							Switch_Code = 124;   //                _HM_
							break;

						case 32:
						case 96:
							Switch_Code = 105;   //           new   Mx()
							Display_Status_new[4] = 1;
							Display_Status_new[6] = 0;
							if ( Debug_Level == 63 ) {
								Serial.print("BitBool_test_28  ");
								Serial.println(Display_Status);
					}			
							break;

						case 24:
						case 40://     EE + =  _*_      three Switch pressed
						case 48:
							Display_rotate = true;
							Switch_Code = 176;   //          new   _Cha_Dis_Dir_on_
							break;
					}
					break;

				case 64:          //    <--
				case 192:
					switch (Display_Status_new) {

						case 0:
							Switch_Code = 60;
							break;

						case 8:
							Switch_Code = 170;  //                 Int
							break;

						case 16:
							Switch_Code = 171;  //                 Frac
							break;

						case 24:
						case 40:
							if ( Switch_old < 6 ) {
								Switch_Code = 93;   //           new   Light down
							}
							break;
					}
					break;

				case 32:          //    _-_
				case 48:
					switch (Display_Status_new) {

						case 0:
							Switch_Code = 45;
							break;

						case 1:  // EE to ..
							if ( Start_input == Input_Expo ) {
								Switch_Code = 39; //           new   _EE-3_
							}
							break;

						case 8:
							Switch_Code = 152;  //           new   _mod_
							break;

						case 16:
							Switch_Code = 94;   //           new   _/p/_  Phytagoras
							break;

						case 24:
							Switch_Code = 39;   //           new   _EE-3_
							break;

						case 32:
						case 96:
							Switch_Code = 104;  //           new   M-()
							Display_Status_new[4] = 1;
							Display_Status_new[6] = 0;
							if ( Debug_Level == 63 ) {
								Serial.print("BitBool_test_29  ");
								Serial.println(Display_Status);
							}			
							break;
					}
					break;

				case 16:         //     _+_
					switch (Display_Status_new) {

						case 0:
							Switch_Code = 43;
							break;

						case 8:
							if ( Rad_in_out == true ) {
								Switch_Code = 37;   //         new   _Deg_
							}
							else {
								Switch_Code = 64; //           new   _Rad_
							}
							break;

						case 16:
							Switch_Code = 92;   //           new   //
							break;

						case 32:
						case 96:
							Switch_Code = 103;  //           new   M+()
							Display_Status_new[4] = 1;
							Display_Status_new[6] = 0;
							if ( Debug_Level == 63 ) {
								Serial.print("BitBool_test_30  ");
								Serial.println(Display_Status);
							}			
							break;
					}
					break;

				case 8:           //    _1/x_
				case 24:
					switch (Display_Status_new) {

						case 0:
							Switch_Code = 33;
							break;

						case 8:
							Switch_Code = 44; //             new  _e_
							break;

						case 16:
							Switch_Code = 36;   //           new   Pi()
							break;

						case 24:
						case 40:
							if ( Switch_old < 6 ) {
								Switch_Code = 153;  //           new   Std_on_off
							}
							break;
					}
					break;
			}

			if ( Start_input > Input_Operation_0 ) {
				if ( Start_input < Input_Operation_5 ) {
					Display_Status_new[5] = 0;
					if ( Debug_Level == 31 ) {
						Serial.print("BitBool_test_1  ");
						Serial.println(Display_Status);
					}			
				}
			}			

		if ( is_view == true ) {
			Switch_Code = 147;          //    fail_view
			if ( Switch_down == 2052 ) {
				Switch_Code = 145;        //    view_up
			}
			if ( ( Switch_delta == 512 ) || ( Switch_delta == 1024 ) ) {
				Countdown_view_end = 1;   //    view_end
			}
		}
	}

	Switch_old = Switch_down;


	if ( Debug_Level == 54 ) {
		 Serial.print("Display_Status_new = ");
		 Serial.println(Display_Status_new);
	}
}

void Mantisse_add_strg_eins() {
	Mantisse_change = true;
	display_string[Cursor_pos] = '1';
	++Cursor_pos;
	++Number_count;
	if ( Point_pos == 0 ) {
		Put_input_Point();
	}
	if ( Number_count == 8 ) {
		display_string[Cursor_pos] = '.';
	}
	else {
		display_string[Cursor_pos] = '_';
	}
}

void Beep__on() {
	if ( Display_Status_new == Display_Status_96 ) {
		if ( Beep_on == false ) {
			Beep_on = true;
			Beep_count = max_Beep_count;
		}
		else {
			if ( Beep_count > 63 ) {
				Beep_count = max_Beep_count;
			}
		}
	}
	else {
		Display_new = true;
	}
	if ( Debug_Level == 17 ) {
		Serial.println("Beep__on");
		Serial.print("Display_Status_old =  ");
		Serial.println(Display_Status_old);
		Serial.print("Display_Status_new =  ");
		Serial.println(Display_Status_new);
	}
}

void Beep__off() {
	Display_new = true;
	Beep_on = false;
	Beep_count = max_Beep_count;
}

void copy_stack_input_2_calc( uint8_t left, uint8_t right ) {
	M_Plus_past = false;
	First_operation = false;
	if ( Start_input < Input_Operation_0 ) {
		First_operation = false;
	}
	if ( Start_input < Display_Result ) {
		mem_stack_calc[ left ].num = mem_stack_input[ right ].num;
		mem_stack_calc[ left ].denom = mem_stack_input[ right ].denom;
		mem_stack_calc[ left ].expo = mem_stack_input[ right ].expo;
	}
	mem_stack_calc[ left ].op          = temp_operation;
	mem_stack_calc[ left ].op_priority = Start_input;
	++mem_stack_count;
	mem_pointer = mem_stack_count;
}

void Input_Operation_2_function() {
	if ( Start_input > Input_Operation_0 ) {
		if ( Start_input < Input_Operation_5 ) {
			if ( M_Plus_past == true ) {
				copy_stack_input_2_calc( mem_stack_count, mem_stack_count  );
				copy_input_left_right( mem_stack_count, mem_stack_count - 1 );
				copy_input_left_right( 0, mem_stack_count - 1 );
			}
			else {
				copy_stack_input_2_calc( mem_stack_count, 0 );
				copy_input_left_right( mem_stack_count, 0 );
			}
			Start_input = Input_Operation_0;
		}
	}
}

void Function_2_display() {
	Test_mem_stack_input();
	if ( First_operation == true ) {
		First_operation = false;
		mem_pointer = mem_stack_count;
		copy_input_left_right( mem_stack_count, 0 );
	}
	Display_Number(mem_stack_input[mem_pointer]);
}

void Function_1_number() {
	Beep__off();
	if ( Start_input < Input_Memory ) {      // Input Number
		Get_Number( 0 );
	}
	if ( Start_input == Display_Result ) {
		left_right_mem_extra( 0, 0 );
		mem_stack_count = 1;
		mem_pointer = mem_stack_count;
		copy_input_left_right( 0, 0 );
		copy_input_left_right( mem_pointer, 0 );
		Start_input = Input_Operation_0;
	}
	Input_Operation_2_function();
	if ( (Start_input == Input_Operation_0) || (Start_input == Input_Memory) || (Start_input == M_Plus_spezial) ) {
		mem_save = false;
		mem_exchange = false;
		Start_input = Input_Operation_0;
		mem_pointer = 0;

		count = 0;
		count_wait = 0;
		switch (Switch_Code) {

			case 33:                 //    _1/x_
				mem_stack_input[ mem_pointer ] = div_x(mem_stack_input[ mem_pointer ]);
				break;

			case 34:                 //    _log2_
				mem_stack_input[ mem_pointer ] = log2(mem_stack_input[ mem_pointer ]);
				break;

			case 112:                //    _loge_
				mem_stack_input[ mem_pointer ] = loge(mem_stack_input[ mem_pointer ]);
				break;

			case 65:                 //    _sin_
				mem_stack_input[ mem_pointer ] = sin(mem_stack_input[ mem_pointer ]);
				break;

			case 66:                 //    _cos_
				mem_stack_input[ mem_pointer ] = cos(mem_stack_input[ mem_pointer ]);
				break;

			case 67:                 //    _tan_
				mem_stack_input[ mem_pointer ] = tan(mem_stack_input[ mem_pointer ]);
				break;

			case 68:                 //    _asin_
				mem_stack_input[ mem_pointer ] = asin(mem_stack_input[ mem_pointer ]);
				break;

			case 69:                 //    _acos_
				mem_stack_input[ mem_pointer ] = acos(mem_stack_input[ mem_pointer ]);
				break;

			case 70:                 //    _atan_
				mem_stack_input[ mem_pointer ] = atan(mem_stack_input[ mem_pointer ]);
				break;

			case 71:                 //    _sinh_
				mem_stack_input[ mem_pointer ] = sinh(mem_stack_input[ mem_pointer ]);
				break;

			case 72:                 //    _cosh_
				mem_stack_input[ mem_pointer ] = cosh(mem_stack_input[ mem_pointer ]);
				break;

			case 73:                 //    _tanh_
				mem_stack_input[ mem_pointer ] = tanh(mem_stack_input[ mem_pointer ]);
				break;

			case 74:                 //    _asinh_
				mem_stack_input[ mem_pointer ] = asinh(mem_stack_input[ mem_pointer ]);
				break;

			case 75:                 //    _acosh_
				mem_stack_input[ mem_pointer ] = acosh(mem_stack_input[ mem_pointer ]);
				break;

			case 76:                 //    _atanh_
				mem_stack_input[ mem_pointer ] = atanh(mem_stack_input[ mem_pointer ]);
				break;

			case 113:                //    e^x
				mem_stack_input[ mem_pointer ] = exp(mem_stack_input[ mem_pointer ]);
				break;

			case 114:                //    2^x
				mem_stack_input[ mem_pointer ] = exp2(mem_stack_input[ mem_pointer ]);
				break;

			case 115:                //    _log10_
				mem_stack_input[ mem_pointer ] = log10(mem_stack_input[ mem_pointer ]);
				break;

			case 116:                //    10^x
				mem_stack_input[ mem_pointer ] = exp10(mem_stack_input[ mem_pointer ]);
				break;

			case 117:                //    _x^2_
				mem_stack_input[ mem_pointer ] = square(mem_stack_input[ mem_pointer ]);
				break;

			case 118:                //    _sqrt()_
				mem_stack_input[ mem_pointer ] = sqrt(mem_stack_input[ mem_pointer ]);
				break;

			case 119:                //    x!
				mem_stack_input[ mem_pointer ] = factorial(mem_stack_input[ mem_pointer ]);
				break;

			case 155:                //    DISP_Â°C
				mem_stack_input[ mem_pointer ] = input( DISP_C );
				break;

			case 156:                //    DISP_Â°F
				mem_stack_input[ mem_pointer ] = input( DISP_F );
				break;

			case 170:                //    _Int_
				mem_stack_input[ mem_pointer ] = floor_(mem_stack_input[ mem_pointer ], 8);
				break;

			case 171:                //    _Frac_
				mem_stack_input[ mem_pointer ] = frac(mem_stack_input[ mem_pointer ]);
				break;

			case 172:                //    _x^3_
				mem_stack_input[ mem_pointer ] = cubic(mem_stack_input[ mem_pointer ]);
				break;

			case 173:                //    _cbrt()_
				mem_stack_input[ mem_pointer ] = cbrt(mem_stack_input[ mem_pointer ]);
				break;
		}

		Error_first_0 = false;
		Error_Test();
		if ( Error_first_0 == true ) {
			Error_first = true;
		}
		if ( Start_input != Display_Error ) {
			Function_2_display();
			if ( Switch_Code == 190 ) {
				Get_Mantisse();
				if ( display_string[Plus_Minus_Expo] == '-' ) {
					display_string[Plus_Minus_Expo] = '=';
				}
				if ( display_string[Plus_Minus_Expo] == '#' ) {
					display_string[Plus_Minus_Expo] = '_';
				}
			}
			if ( (Switch_Code == 155) || (Switch_Code == 156) ) {
				display_string[Memory_1] = 'O';
				if ( Switch_Code == 155 ) {
					display_string[Memory_0] = 'C';
				}
				if ( Switch_Code == 156 ) {
					display_string[Memory_0] = 'F';
				}
			}
		}
		else {
			mem_pointer = mem_stack_count;
		}
		Beep__on();
	}
	Display_Memory_x( 23 );
}

void Mem_plus_minus_mul_div() {
	uint8_t operation  = Switch_Code;
	operation         /= 9;
	mem_extra_test     = Switch_Code % 9;
	mem_extra_test    += 1;
	mem_plus_test   = mem_extra_test;
	if ( (Start_input == Input_Mantisse) || (Start_input == Input_Expo) || (Start_input == Input_Memory) || (Start_input == Input_Operation_0) ) {
		if ( Start_input == Input_Operation_0 ) {
			mem_save = false;
			mem_exchange = false;
		}
		M_Plus_past = true;
		Function_1_number();
		mem_extra_left_right( mem_extra_max_4, 0 );
		left_right_mem_extra( mem_pointer, 0 );
		Start_input = M_Plus_spezial;
	}
	if ( Start_input == M_Plus_spezial ) {
		copy_input_left_right( mem_pointer, 0 );
		switch (operation) {

			case 15: 
				mem_stack_input[ 0 ] = add( mem_extra_stack[ mem_extra_test ], mem_extra_stack[ mem_extra_max_4 ], 1 );
				break;

			case 25: 
				mem_stack_input[ 0 ] = add( mem_extra_stack[ mem_extra_test ], mem_extra_stack[ mem_extra_max_4 ], -1 );
				break;

			case 26: 
				mem_stack_input[ 0 ] = mul( mem_extra_stack[ mem_extra_test ], mem_extra_stack[ mem_extra_max_4 ] );
				break;

			case 27: 
				mem_stack_input[ 0 ] = mul( mem_extra_stack[ mem_extra_test ], div_x( mem_extra_stack[ mem_extra_max_4 ] ) );
				break;
		}				
	}
	if ( Start_input == Display_Result ) {
		mem_stack_count = 1;
		copy_input_left_right( 1, 0 );
		mem_pointer = 0;
		switch (operation) {

			case 15: 
				mem_stack_input[ 0 ] = add( mem_extra_stack[ mem_extra_test ], mem_extra_stack[ 0 ], 1 );
				break;

			case 25: 
				mem_stack_input[ 0 ] = add( mem_extra_stack[ mem_extra_test ], mem_extra_stack[ 0 ], -1 );
				break;

			case 26: 
				mem_stack_input[ 0 ] = mul( mem_extra_stack[ mem_extra_test ], mem_extra_stack[ 0 ] );
				break;

			case 27: 
				mem_stack_input[ 0 ] = mul( mem_extra_stack[ mem_extra_test ], div_x( mem_extra_stack[ 0 ] ) );
				break;
		}				
		if ( Debug_Level == 12 ) {
			Serial.print("= ___M+ 64bit___ ");
			Serial.print(mem_extra_stack[ mem_extra_test ].num);
			Serial.print(" / ");
			Serial.print(mem_extra_stack[ mem_extra_test ].denom);
			Serial.print(" x 10^ ");
			Serial.println(mem_extra_stack[ mem_extra_test ].expo);
		}
	}
	if ( (Start_input == M_Plus_spezial) || (Start_input == Display_Result) ) {
		Error_Test();
		if ( Start_input != Display_Error ) {
			mem_extra_left_right( mem_extra_test, 0 );
			Display_Number(mem_stack_input[mem_pointer]);
			display_string[Operation] = ' ';
			display_string[Memory_1] = Display_Memory_1[9];  // m;
			display_string[Memory_0] = '0' + mem_extra_test;
	
			// Display_Memory_x( 23 );
		}
		else {
			mem_pointer     = 1;
			mem_stack_count = 1;
		}
	}
	if ( Start_input == Display_M_Plus ) {
		left_right_mem_extra( 0, 0 );
		Start_input = Display_Result;
	}	
}

void change_number() {
	if ( Expo_change == true ) {
		Get_Expo_change();
	}
	if ( Mantisse_change == true ) {
		Get_Mantisse();
	}
}

void Display_Memory_x( uint8_t Mem_Display ) {

	if ( Debug_Level == 54 ) {
		 Serial.print("Mem_Display = ");
		 Serial.println(Mem_Display);
	}

	switch (Mem_Display) {
		case 23:
			Display_Memory_1[ 19 ] = display_string[Memory_1];
			Display_Memory_0[ 19 ] = display_string[Memory_0];
			break;

		case 24:
			Display_Memory_1[ 19 ] = ' ';
			Display_Memory_0[ 19 ] = ' ';
			break;

		case 25:
			display_string[Memory_1] = mem_str_1[mem_pointer];
			display_string[Memory_0] = mem_str_0[mem_pointer];
			break;

		case 17:
			if ( Start_input <= Input_Operation_0 ) {
				return;
			}
			if ( Start_input > Input_Operation_5 ) {
				return;
			}
			break;

		case 9:
			display_string[Memory_1] = Display_Memory_1[Mem_Display];
			display_string[Memory_0] = Memory_p_min_m_d[Display_Status_mem];
			break;

		default:
			display_string[Memory_1] = Display_Memory_1[Mem_Display];
			display_string[Memory_0] = Display_Memory_0[Mem_Display];
			break;
	}
	
	if ( (Mem_Display_old == 3) || (Mem_Display_old == 12) ) {
		display_string[Memory_1] = Display_Memory_1[Mem_Display_old];
		display_string[Memory_0] = Display_Memory_0[Mem_Display_old];
	}

	Mem_Display_old = Mem_Display;
}

bool    Test_buffer = false;
uint8_t Number_of_buffer = 0;
// Create a RingBufCPP object designed to hold a Max_Buffer of Switch_down
// RingBufCPP at version 1.1
RingBufCPP < uint32_t, Max_Buffer > q;

// Define various ADC prescaler
static const unsigned char PS_16  = (1 << ADPS2);
static const unsigned char PS_32  = (1 << ADPS2) |                (1 << ADPS0);
static const unsigned char PS_64  = (1 << ADPS2) | (1 << ADPS1);
static const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

// the setup routine runs once when you press reset:
void setup() {

	// pinMode(On_Off_PIN, INPUT_PULLUP);
/*
	pinMode(On_Off_PIN, OUTPUT);
	digitalWrite(On_Off_PIN, LOW);
*/
	// Initialize the digital pin as an output.
	// Pin 13 has an LED connected on most Arduino boards

	// http://www.microsmart.co.za/technical/2014/03/01/advanced-arduino-adc/
	// set up the ADC
	ADCSRA &= ~PS_128;  // remove bits set by Arduino library

	// you can choose a prescaler from above.
	// PS_16, PS_32, PS_64 or PS_128
	// set a2d prescale factor to 32
	// 12 MHz / 64 = 187.5 KHz,  inside the desired 50-200 KHz range.
	// 12 MHz / 32 = 375.0 KHz, outside the desired 50-200 KHz range.
	// 12 MHz / 16 = 750.0 KHz, outside the desired 50-200 KHz range.
	ADCSRA |= PS_32;    // set our own prescaler to 32

	if ( Debug_Level == 23 ) {
		strcpy( display_string, display_string_888 );    // On_Off_Test
	}

	uint8_t pin;
	for ( pin = Min_Out; pin <= Max_Out; pin += 1 ) {
		pinMode(pin, INPUT_PULLUP);
		pinMode(pin, OUTPUT);
		if ( pin == On_Off_PIN) {
			digitalWrite(pin, LOW);
		}
		else {
			digitalWrite(pin, HIGH);
		}
	}

	pinMode(SDA, INPUT);           // Pin 17
	pinMode(SCL, OUTPUT);          // Pin 16
	pinMode(SDA, INPUT_PULLUP);    // Pin 17

	pinMode(A_0, INPUT);           // set pin to input
	pinMode(A_1, INPUT);           // set pin to input
	pinMode(A_2, INPUT);           // set pin to input

	pinMode(Rx_1, INPUT_PULLUP);   // Pin D10
	digitalWrite(Tx_1, HIGH);      // Pin D11

	// OneWire ds(Rx_1);              // on pin D10 (a 4.7K resistor is necessary)

	pinMode(Beep_m, INPUT_PULLUP); // Pin A3
	pinMode(Beep_p, INPUT_PULLUP); // Pin A7

	pinMode(Out_A, OUTPUT);        // Pin A3
	digitalWrite(Out_A, LOW);
	pinMode(Out_B, OUTPUT);        // Pin A2
	digitalWrite(Out_B, LOW);
	pinMode (Out_C, OUTPUT);       // Pin A1
	digitalWrite(Out_C, LOW);

	analogReference(EXTERNAL);

	Timer1.initialize(Time_LOW);  // sets timer1 to a period of 263 microseconds
	TCCR1A |= (1 << COM1B1) | (1 << COM1B0);  // inverting mode for Pin OC1B --> D4
	Timer1.attachInterrupt(timer_Isr);  // attach the service routine here
	Timer1.pwm(PWM_Pin, led_bright[led_bright_index]);  // duty cycle goes from 0 to 1023
	interrupts();                      // Ermöglichen von Interrupts

	// start serial port at 115200 bps and wait for port to open:
	if ( Debug_Level > 0 ) {
		Serial.begin(115200);
		while (!Serial) ;     // wait for Arduino Serial Monitor
	}

	q.add(Switch_down);
	q.pull(&Switch_down);

	digitalWrite(PWM_s, LOW);

  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x19) for example, also can be left in blank for default address use
  // Also there is a table with all addres possible for this sensor, you can connect multiple sensors
  // to the same i2c bus, just configure each sensor with a different address and define multiple objects for that
  //  A2 A1 A0 address
  //  0  0  0   0x18  this is the default address
  //  0  0  1   0x19
  //  0  1  0   0x1A
  //  0  1  1   0x1B
  //  1  0  0   0x1C
  //  1  0  1   0x1D
  //  1  1  0   0x1E
  //  1  1  1   0x1F
	if (!tempsensor.begin(0x18)) {
    while (1);
  }

  tempsensor.setResolution(3); // sets the resolution mode of reading, the modes are defined in the table bellow:
  // Mode Resolution SampleTime
  //  0    0.5°C       30 ms
  //  1    0.25°C      65 ms
  //  2    0.125°C     130 ms
  //  3    0.0625°C    250 ms
}

// the loop routine runs over and over again forever:
void loop() {
  // tempsensor.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere, stops temperature sampling

	if ( Countdown_OFF < Countdown_Off_0 ) {
		if ( Countdown_OFF > 0 ) {
			digitalWrite(On_Off_PIN, HIGH);
		}
	}

	if ( Switch_Code > 0 ) {     // Main Responce about Switch

		Print_Statepoint();
		Beep__on();
		switch (Switch_Code) {

			case 90:                 //    _CE_
				if ( Error_first == true ) {
					Start_input = Input_Operation_0;
				}
				else {
					if ( (Pendular_on == false) && (Start_input == Display_Error) ) {
						mem_pointer = mem_stack_count;
						Start_input = Start_Mode;
						Switch_Code =   0;
						index_5min  = 255;
					}
					if ( Pendular_on == true) {
						Pendular_on = false;
						Beep_on_off = Beep_on_off_temp;
						mem_pointer = mem_stack_count;
						Start_input = Start_Mode;
						Switch_Code =   0;
						index_5min  = 255;
					}
				}
				if ( (Pendular_on == false) && (Start_input == Display_Error) ) {
					mem_pointer = mem_stack_count;
					Start_input = Start_Mode;
					Switch_Code =   0;
					index_5min  = 255;
				}
				if ( Pendular_on == true) {
					Pendular_on = false;
					Beep_on_off = Beep_on_off_temp;
					mem_pointer = mem_stack_count;
					Start_input = Start_Mode;
					Switch_Code =   0;
					index_5min  = 255;
				}
				break;

			case 121:                //    clock
				if (Pendular_on == false) {
					Pendular_on = true;
					Start_mem = Start_input;
				}
				else {
					Pendular_on = false;
					Start_input = Start_mem;
				}
				break;

			case 122:                //    Beep_On_Off
				if ( Beep_on_off == true ) {
					Display_Memory_x( 20 );     //  "no"
					display_string[Beep_point] = ' ';
					Beep_on_off = false;
				}
				else {
					Display_Memory_x( 21 );     //  "on"
					display_string[Beep_point] = '.';
					Beep_on_off = true;
				}
				Beep_on_off_temp = Beep_on_off; // make synchron
				break;

			case 123:                //    Off
				Display_Status_new = 152;
				Display_Off();
				Countdown_OFF = Countdown_Off_2;      // Switch Off  2x Beep
				break;

			case 125:                //    Off after  5min
				Display_Status_new = 152;
				Display_Off();
				Countdown_OFF = Countdown_Off_3;  // Switch Off  3x Beep
				break;

			default:
				Beep__off();
				break;
		}
		Print_Statepoint_after();

		if ( (Pendular_on == false) && (Start_input != Display_Error) ) {

			Print_Statepoint();
			Beep__on();
			switch (Switch_Code) {

				case 33:                 //    _1/x_
				case 34:                 //    _log2_
				case 65:                 //    sin(x)
				case 66:                 //    cos(x)
				case 67:                 //    tan(x)
				case 68:                 //    asin(x)
				case 69:                 //    acos(x)
				case 70:                 //    atan(x)
				case 71:                 //    sinh(x)
				case 72:                 //    cosh(x)
				case 73:                 //    tanh(x)
				case 74:                 //    asinh(x)
				case 75:                 //    acosh(x)
				case 76:                 //    atanh(x)
				case 112:                //    log(x)
				case 113:                //    e^x
				case 114:                //    2^x
				case 115:                //    log10(x)
				case 116:                //    10^x
				case 117:                //    _x^2_
				case 118:                //    _sqrt()_
				case 119:                //    x!
				case 155:                //    DISP_Â°C
				case 156:                //    DISP_Â°F
				case 170:                //    _Int_
				case 171:                //    _Frac_
				case 172:                //    _x^3_
				case 173:                //    _cbrt()_
				case 190:                //    _rnd(x)_
					Function_1_number();
					break;

				case 35:                 //    _+/-_
					Input_Operation_2_function();
					if ( (Start_input == Input_Operation_0) || (Start_input == Input_Memory) ) {
						mem_save = false;
						mem_exchange = false;
						Start_input = Input_Operation_0;
						mem_pointer = 0;
						mem_stack_input[ mem_pointer ].num *= -1;
						if ( display_string[Plus_Minus] == '-' ) {
							display_string[Plus_Minus] = ' ';
						}
						else {
							display_string[Plus_Minus] = '-';
						}
						Function_2_display();
					}
					if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) || (Start_input == M_Plus_spezial) ) {
						mem_stack_count = 1;
						mem_pointer = mem_stack_count;
						Start_input = Input_Mantisse;
						mem_stack_count = 1;
						left_right_mem_extra( mem_pointer, 0 );

						display_digit_temp = display_digit;
						display_digit = 8;
						Display_Number(mem_stack_input[mem_pointer]);
						display_string[Cursor_pos] = '.';
						display_digit = display_digit_temp;
					}
					if ( Start_input == Input_Mantisse ) {
						mem_stack_input[ mem_pointer ].num *= -1;
						if ( display_string[Plus_Minus] == '-' ) {
							display_string[Plus_Minus] = ' ';
						}
						else {
							display_string[Plus_Minus] = '-';
						}
					}
					if ( Start_input == Input_Expo ) {
						if ( display_string[Plus_Minus_Expo] == '-' ) {
							display_string[Plus_Minus_Expo] = '#';
						}
						else {
							display_string[Plus_Minus_Expo] = '-';
						}
						Get_Expo_( 1 );
					}
					break;

				case 36:                 //    PI()
					if ( (Start_input == Input_Operation_0) || (Start_input == M_Plus_spezial) ) {
						Start_input = Input_Memory;
					}
					if ( Start_input > Input_Operation_0 ) {
						if ( Start_input < Input_Operation_5 ) {
							copy_stack_input_2_calc( mem_stack_count, mem_pointer );
							Clear_String();
							Start_input = Input_Memory;
						}
					}
					if ( (Start_input < Input_Operation_0) || (Display_Error < Start_input) ) {
						mem_extra_test = 10;
						Memory_to_Input_Operation();
					}
					else {
						Display_Memory_x( 17 );
					}
					break;

				case 37:                 //    _Deg_
					if ( Rad_in_out == true ) {
						if ( Start_input < Input_Memory ) {      // Input Number
							Get_Number( 0 );
						}
						if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) || (Start_input == M_Plus_spezial) ) {
							mem_stack_count = 1;
							mem_pointer = mem_stack_count;
							copy_input_left_right( mem_pointer, 0 );
							Start_input = Input_Operation_0;
						}
						if ( (Start_input == Input_Operation_0) || (Start_input == Input_Memory) || (Start_input == M_Plus_spezial) ) {
							Error_Test();
							mem_pointer = mem_stack_count;
							if ( Start_input != Display_Error ) {
								mem_pointer = 0;
								Rad_in_out = false;
								mem_stack_input[ mem_pointer ] = mul(mem_stack_input[ mem_pointer ], to_xx[ to_deg ]);
								if ( max_input == false ) {
									mem_stack_input[ mem_pointer ].op = '_';
								}
								else {
									mem_stack_input[ mem_pointer ].op = 'x';
								}
								Error_Test();
								mem_pointer = mem_stack_count;
								if ( Start_input != Display_Error ) {
									mem_pointer = 0;
									Display_Number(mem_stack_input[mem_pointer]);
								}
							}
						}
					}
					Display_Memory_x( 17 );
					break;

				case 38:                 //    _EE+3_
					if ( Start_input < Input_Memory ) {      // Input Number
						if ( Number_count == Zero_count ) {
							Mantisse_add_strg_eins();
						}
						Init_expo = false;
						expo_temp_16 = Get_Expo() + 102;
						expo_temp_16 = expo_temp_16 / 3;
						if ( expo_temp_16 < (expo_max_in_3 + 34) ) {
							Expo_change = true;
							++expo_temp_16;
							expo_temp_16 = (expo_temp_16 * 3) - 102;
							Put_Expo();
						}
					}
					break;

				case 39:                 //    _EE-3_
					if ( Start_input < Input_Memory ) {      // Input Number
						if ( Number_count == Zero_count ) {
							Mantisse_add_strg_eins();
						}
						Init_expo = false;
						expo_temp_16 = Get_Expo() + 102;
						if ( (expo_temp_16 % 3) != 0 ) {
							expo_temp_16 = expo_temp_16 + 3;
						}
						expo_temp_16 = expo_temp_16 / 3;
						if ( expo_temp_16 > (expo_min_in_3 + 34) ) {
							Expo_change = true;
							--expo_temp_16;
							expo_temp_16 *= 3;
							expo_temp_16 -= 102;
							Put_Expo();
						}
					}
					break;

				case 41:                 //    _)_
					if ( Debug_Level == 56 ) {
						Serial.print("Start_input: ");
						Serial.println(Start_input);
					}
					switch ( Start_input ) {
						case Input_Operation_0:
						case Input_Memory:
						case M_Plus_spezial:
						case Input_Mantisse:
						case Input_Expo:
							if ( ( Std_mode == 0 ) && ( mem_stack_count == 2 ) ) {
								if ( Debug_Level == 56 ) {
									Serial.print("Operation_input(0): ");
									Serial.println(mem_stack_input[ 0 ].op);
									Serial.print("Operation_input(1): ");
									Serial.println(mem_stack_input[ 1 ].op);
									Serial.print("Operation_calc(0): ");
									Serial.println(mem_stack_calc[ 0 ].op);
									Serial.print("Operation_calc(1): ");
									Serial.println(mem_stack_calc[ 1 ].op);
								}
								mem_stack_input[ 1 ].op = mem_stack_calc[ 1 ].op;
								switch ( mem_stack_input[ 1 ].op ) {
									case 43:  //    _+_
										Char_temp      = '+';
										temp_operation = 252;
										break;
	
									case 45:  //    _-_
										Char_temp      = '-';
										temp_operation = 253;
										break;										
										
									case 42:  //    _*_
										Char_temp      = '*';
										temp_operation = 254;
										break;
										
									case 47:  //    _/_
										Char_temp      = ',';
										temp_operation = 255;
										break;										
								}
							}
							else {
								First_operation = false;
								max_input = false;
								mem_pointer = mem_stack_count;
								Clear_String();
								Start_input = Start_Mode;
							}
							break;
					}

					if ( Start_input > Input_Operation_0 ) {
						if ( Start_input < Input_Operation_5 ) {
							First_operation = false;
							add_operation_to_mem( 0, '_' );
							temp_operation = 0;
							display_string[Operation_point] = ' ';
						}
					}
					break;

				case 43:                 //    _+_
					add_operation_to_mem( 1, '+' );
					if ( display_string[Expo_point] == '.' ) {
						Found_constant = false;
						temp_operation = 216;
					}
					break;

				case 45:                 //    _-_
					add_operation_to_mem( 1, '-' );
					if ( display_string[Expo_point] == '.' ) {
						Found_constant = false;
						temp_operation = 212;
					}
					break;

				case 42:                 //    _*_
					add_operation_to_mem( 2, '*' );
					if ( display_string[Expo_point] == '.' ) {
						Found_constant = false;
						temp_operation = 213;
					}
					break;

				case 47:                 //    _/_
					add_operation_to_mem( 2, ',' );
					if ( display_string[Expo_point] == '.' ) {
						Found_constant = false;
						temp_operation = 214;
					}
					break;

				case 44:                 //    Tau()
					if ( (Start_input == Input_Operation_0) || (Start_input == M_Plus_spezial)) {
						Start_input = Input_Memory;
					}
					if ( Start_input > Input_Operation_0 ) {
						if ( Start_input < Input_Operation_5 ) {
							copy_stack_input_2_calc( mem_stack_count, mem_pointer );
							Clear_String();
							Start_input = Input_Memory;
						}
					}
					if ( (Start_input < Input_Operation_0) || (Display_Error < Start_input) ) {
						mem_extra_test = 11;
						Memory_to_Input_Operation();
					}
					else {
						Display_Memory_x( 17 );
					}
					break;

				case 46:                 //    _._
					if ( Start_input == Display_Result ) {
						Start_input = Display_M_Plus;
					}
					if ( Start_input > Input_Operation_0 ) {
						if ( Start_input < Input_Operation_5 ) {
							copy_stack_input_2_calc( mem_stack_count, mem_pointer );
							Clear_String();
						}
					}
					if ( Start_input == Input_Mantisse ) {
						if ( Point_pos == 0 ) {
							if ( Number_count < 8 ) {
								Put_input_Point();
							}
						}
					}
					break;

				case 48:                 //    _0_
					if ( display_string[Memory_1] == Display_Memory_1[4] ) {   // E
						Start_input = Input_Memory;
					}
					if ( Start_input > Input_Operation_0 ) {
						if ( Start_input < Input_Operation_5 ) {
							copy_stack_input_2_calc( mem_stack_count, mem_pointer );
							Clear_String();
						}
					}
					if ( Start_input == Input_Mantisse ) {
						if ( Number_count < 8 ) {
							if ( Zero_count < 7 ) {
								if (( Number_count > 0 ) or ( Point_pos > 0 )) {
									Mantisse_change = true;
									display_string[Cursor_pos] = Switch_Code;
									++Cursor_pos;
									++Number_count;
									++Zero_count;
									if ( Number_count == 8 ) {
										display_string[Cursor_pos] = '.';
									}
									else {
										display_string[Cursor_pos] = '_';
									}
								}
								if ( Number_count == 0 ) {
									Put_input_Point();
								}
							}
						}
					}
					if ( Start_input == Input_Expo ) {
						display_string[Expo_1] = display_string[Expo_0];
						display_string[Expo_0] = Switch_Code;
						Get_Expo_( 1 );
					}
					if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) || (Start_input == M_Plus_spezial) || (Start_input == Input_Operation_0) ) {
						Result_to_Start_Mode();
						Start_input = Input_Memory;
					}
					if ( Start_input == Input_Memory ) {
						First_operation = false;
						max_input = false;
						mem_pointer = mem_stack_count;
						Clear_String();
						Put_input_Point();
						mem_stack_input[ mem_pointer ].num = int32_max;
						mem_stack_input[ mem_pointer ].denom = int32_max;
						mem_stack_input[ mem_pointer ].expo = expo_min_input;
						mem_stack_input[ mem_pointer ].op = temp_op;
					}
					break;

				case 49:                 //    _1_
				case 50:                 //    _2_
				case 51:                 //    _3_
				case 52:                 //    _4_
				case 53:                 //    _5_
				case 54:                 //    _6_
				case 55:                 //    _7_
				case 56:                 //    _8_
				case 57:                 //    _9_
					if ( Start_input == Input_Expo ) {
						display_string[Expo_1] = display_string[Expo_0];
						display_string[Expo_0] = Switch_Code;
						Display_Memory_x( 25 );
						Get_Expo_( 1 );
					}
					if ( display_string[Memory_1] == Display_Memory_1[4] ) {   // E
						Start_input = Input_Memory;
					}
					if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) || (Start_input == M_Plus_spezial) ) {
						Result_to_Start_Mode();
					}
					if ( Start_input == Input_Memory ) {
						First_operation = false;
						max_input = false;
						mem_pointer = mem_stack_count;
						Clear_String();
					}
					if ( Start_input > Input_Operation_0 ) {
						if ( Start_input < Input_Operation_5 ) {
							copy_stack_input_2_calc( mem_stack_count, mem_pointer );
							Clear_String();
						}
					}
					if ( Start_input < Input_Expo ) {
						if ( Number_count < 8 ) {
							Mantisse_change = true;
							display_string[Cursor_pos] = Switch_Code;
							++Cursor_pos;
							++Number_count;
							if ( Number_count == 8 ) {
								display_string[Cursor_pos] = '.';
							}
							else {
								display_string[Cursor_pos] = '_';
							}
						}
					}
					break;

				case 60:                 //    _<--_
					if ( display_string[Memory_0] == 'o' ) {
						temp_operation = mem_stack_input[ 1 ].op;
					}
					Mantisse_change = false;
					Expo_change = false;
					if ( Start_input == Input_Mantisse ) {
						if (Cursor_pos > 2) {
							Mantisse_change = true;
							display_string[Cursor_pos] = '#';
							--Cursor_pos;
							--Number_count;
							char_test = display_string[Cursor_pos];
							if ( char_test == '0' ) {
								--Zero_count;
							}
							if ( char_test == '.' ) {
								char_test = '_';
								if ( Repeat_pos > 0 ) {
									if ( (Repeat_pos - Point_pos) == 1 ) {
										++Cursor_pos;
									}
									Repeat_pos = 0;
								}
								else {
									display_string[Cursor_pos] = '#';
									Point_pos = 0;
								}
								++Number_count;
							}
							else {
								char_test = '_';
							}
							if ( Number_count == 0 ) {
								Start_input = Start_Mode;
								break;
							}
							display_string[Cursor_pos] = char_test;
							display_string[Cursor_pos + 1] = '#';
							if ( Repeat_pos > 0 ) {
								if ( Number_count < 7 ) {
									display_string[Cursor_pos + 2] = '#';
								}
							}
						}
					}
					if ( Start_input == Input_Expo ) {
						if ( ( display_string[Expo_1] == '0' ) && ( display_string[Expo_0] == '0' ) ) {
							display_string[Cursor_pos] = Pointer_memory;
							display_string[Expo_point] = ' ';
							display_string[Expo_1] = '#';
							display_string[Expo_0] = '#';
							display_string[Plus_Minus_Expo] = '#';
							Start_input = Input_Mantisse;
							Init_expo = true;
						}
						else {
							display_string[Expo_0] = display_string[Expo_1];
							display_string[Expo_1] = '0';
						}
					}
					if ( Start_input > Input_Memory ) {   // no Number Input

						Init_expo = false;
						mem_pointer = 0;

						if ( Start_input > Input_Operation_0 ) {
							First_operation = false;
							Input_Operation_2_function();
							mem_pointer = mem_stack_count;
							First_operation = false;
						}

						if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) || (Start_input == M_Plus_spezial) ) {
							mem_stack_count = 1;
							mem_pointer = mem_stack_count;
							copy_input_left_right( mem_pointer, 0 );
						}
						Start_input = Input_Mantisse;
						Init_expo = false;
						display_digit_temp = display_digit;
						display_digit = 8;
						Display_Number(mem_stack_input[mem_pointer]);
						display_string[Cursor_pos] = '.';
						display_digit = display_digit_temp;
						max_input = false;
					}
					No_change = true;
					break;

				case 61:                 //    _=_
					if ( Debug_Level == 21 ) {
						Serial.print("mem_stack_count =  ");
						Serial.print(mem_stack_count);
						Serial.print(" --> ");
						Serial.println(Start_input);
					}
					if ( Start_input == M_Plus_spezial ) {
						left_right_mem_extra( mem_pointer, mem_extra_max_4 );
						mem_extra_left_right( 0, mem_pointer );
						left_right_mem_extra( mem_pointer, 0 );
						Test_to_Result = false;
						Start_input = Input_Operation_0;
						copy_input_left_right( 0, mem_pointer );
						mem_extra_left_right( 0, 0 );
						mem_pointer = 0;
					}
					mem_plus_test = 0;
					if ( Found_constant == false ) {
						if ( mem_stack_count == 1 ) {
							if ( Start_input > Input_Operation_0 ) {
								if ( Start_input < Input_Operation_5 ) {
									Found_constant = true;
									if ( temp_operation > p_p ) {
										mem_stack_calc[ 1 ].num = mem_stack_input[ 1 ].num;
										mem_stack_calc[ 1 ].denom = mem_stack_input[ 1 ].denom;
										mem_stack_calc[ 1 ].expo = mem_stack_input[ 1 ].expo;
										mem_stack_calc[ 1 ].op_priority = Start_input;
										mem_stack_calc[ 1 ].op = temp_operation;
									}
								}
								if ( Start_input == Display_Result ) {
									Get_Number( 0 );
								}
							}
						}
						if ( mem_stack_count == 2 ) {
							Found_constant = true;
						}
					}
					if ( Start_input > Start_Mode ) {
						if ( Start_input < Input_Operation_0 ) {      // Input Number
							if ( Test_to_Result == true ) {
								Test_to_Result = false;
								Start_input = Input_Operation_0;
								if ( No_change == true ) {
									No_change = false;
								}
								else {
									mem_stack_input[ mem_pointer ].expo = expo_min_input;
									if (display_string[Plus_Minus] == '-') {
										mem_stack_input[mem_pointer].num *= -1;
									}
								}
								Get_Number( 0 );
							}
						}
					}
					if ( Start_input != Display_Error ) {
						if ( Constant_arithmetic == false ) {
							Start_input = Input_Operation_0;
						}
						else {
							mem_pointer = 0;
						}
						if ( Start_input == Input_Operation_0 ) {
							mem_extra_left_right( 0, mem_pointer );
							Error_Test();
						}
						if ( mem_stack_count > 1 ) {
							copy_stack_input_2_calc( mem_stack_count, mem_pointer );
						}
						else {
							if ( temp_operation > p_p ) {
								copy_stack_input_2_calc( 2, mem_pointer );
							}
							else {
								copy_stack_input_2_calc( 1, mem_pointer );
							}
						}
						if ( mem_stack_count > 2 ) {
							--mem_stack_count;
							if ( Std_mode == 1 ) {
								for ( index_i = 1; index_i < mem_stack_count; index_i += 1 ) {
									if ( mem_stack_calc[index_i].op == 45 ) {     // _-_
										mem_stack_calc[index_i].op = 43;            // _+_

										mem_stack_calc[index_i + 1].num *= -1;
									}
									if ( mem_stack_calc[index_i].op == 47 ) {     // _/_
										mem_stack_calc[index_i].op = 42;            // _*_

										temp_xxx = mem_stack_calc[index_i + 1].num;
										mem_stack_calc[index_i + 1].expo *= -1;

										mem_stack_calc[index_i + 1].num   = mem_stack_calc[index_i + 1].denom;
										mem_stack_calc[index_i + 1].denom = temp_xxx;										
										if ( temp_xxx < 0 ) {
											mem_stack_calc[index_i + 1].num   *= -1;
											mem_stack_calc[index_i + 1].denom *= -1;										
										}
									}
								}
							}
						}
						if ( Constant_arithmetic == true ) {
							mem_stack_test = mem_stack_count;
						}
						if ( Start_input > Input_Operation_0 ) {
							Input_Operation_2_function();
						}
						mem_stack_calc[mem_stack_count].op = 61;
						if ( Start_input == Display_Result ) {
							if ( Debug_Level == 21 ) {
								Serial.println("--xxxx--");
							}
							mem_pointer = mem_stack_count;
							--mem_stack_count;
							if ( temp_operation > p_p ) {          //
								copy_calc_left_right( 2, 0 );
							}
							else {
								copy_calc_left_right( 1, 0 );
							}
							mem_pointer = 0;
							Start_input = Input_Operation_0;
						}
						if ( Constant_arithmetic == false ) {
							mem_stack_test = mem_stack_count;
						}
						if ( Debug_Level == 21 ) {
							for ( index_i = 1; index_i <= mem_stack_test; index_i += 1 ) {
								if ( index_i < 10 ) {
									Serial.print(" ");
								}
								Serial.print(index_i);
								Serial.print(". -> ( ");
								if ( mem_stack_calc[index_i].num >= 0 ) {
									Serial.print(" ");
								}
								Serial.print(mem_stack_calc[index_i].num);
								if ( mem_stack_calc[index_i].expo > 0 ) {
									for ( index_j = 1; index_j <= mem_stack_calc[index_i].expo; index_j += 1 ) {
										Serial.print("0");
									}
								}
								Serial.print(" / ");
								Serial.print(mem_stack_calc[index_i].denom);
								if ( mem_stack_calc[index_i].expo < 0 ) {
									for ( index_j = -1; index_j >= mem_stack_calc[index_i].expo; index_j -= 1 ) {
										Serial.print("0");
									}
								}
								Serial.print(" ) ");
								Print_Operation( mem_stack_calc[index_i].op );
								Serial.println(" ");
							}
							Serial.print(mem_stack_calc[1].op);
							Serial.println(" --------");
						}
						if ( Start_input != Display_Result ) {
							Start_input = Display_Result;
							M_Plus_past = false;
							mem_save = true;
							mem_pointer = 0;
							mem_stack_count = 1;
							calc_stack(mem_stack_test);
							Display_Number(mem_stack_input[mem_pointer]);
							mem_extra_test = 0;
							max_input = false;
							Beep__on();
						}
					}
					mem_extra_stack[ 12 ].num = mem_extra_stack[ 0 ].num;
					mem_extra_stack[ 12 ].denom = mem_extra_stack[ 0 ].denom;
					mem_extra_stack[ 12 ].expo = mem_extra_stack[ 0 ].expo;
					mem_extra_stack[ 12 ].op = mem_extra_stack[ 0 ].op;
					if ( Std_mode == 0 ) {
						if ( Display_Status_new == 32 ) {
							mem_extra_stack[ 14 ].num = mem_extra_stack[ 0 ].num;
							mem_extra_stack[ 14 ].denom = mem_extra_stack[ 0 ].denom;
							mem_extra_stack[ 14 ].expo = mem_extra_stack[ 0 ].expo;
							mem_extra_stack[ 14 ].op = mem_extra_stack[ 0 ].op;
						}
					}

					if ( Debug_Level == 21 ) {
						Serial.print("mem_stack_count =  ");
						Serial.print(mem_stack_count);
						Serial.print(" --> ");
						Serial.println(Start_input);
					}
					break;

				case 64:                 //    _Rad_
					if ( Rad_in_out == false ) {
						if ( Start_input < Input_Memory ) {      // Input Number
							Get_Number( 0 );
						}
						if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) || (Start_input == M_Plus_spezial) ) {
							mem_stack_count = 1;
							mem_pointer = mem_stack_count;
							copy_input_left_right( mem_pointer, 0 );
							Start_input = Input_Operation_0;
						}
						if ( (Start_input == Input_Operation_0) || (Start_input == Input_Memory) || (Start_input == M_Plus_spezial) ) {
							Error_Test();
							mem_pointer = mem_stack_count;
							if ( Start_input != Display_Error ) {
								mem_pointer = 0;
								Rad_in_out = true;
								mem_stack_input[ mem_pointer ] = mul(mem_stack_input[ mem_pointer ], to_xx[ to_rad ]);
								if ( max_input == false ) {
									mem_stack_input[ mem_pointer ].op = '_';
								}
								else {
									mem_stack_input[ mem_pointer ].op = 'x';
								}
								Error_Test();
								mem_pointer = mem_stack_count;
								if ( Start_input != Display_Error ) {
									mem_pointer = 0;
									Display_Number(mem_stack_input[mem_pointer]);
								}
							}
						}
					}
					Display_Memory_x( 17 );
					break;

				case 77:                 //   _MR(0)
				case 78:                 //   _MR(1)
				case 79:                 //   _MR(2)
				case 80:                 //   _MR(3)
				case 81:                 //   _MR(4)
				case 82:                 //   _MR(5)
				case 83:                 //   _MR(6)
				case 84:                 //   _MR(7)
				case 85:                 //   _MR(8)
				case 86:                 //   _MR(9)
					mem_extra_test = Switch_Code - MR_0;
					if ( (Start_input == Input_Operation_0) || (Start_input == M_Plus_spezial) ) {
						Start_input = Input_Memory;
					}
					if ( Start_input > Input_Operation_0 ) {
						if ( Start_input < Input_Operation_5 ) {
							copy_stack_input_2_calc( mem_stack_count, mem_pointer );
							Clear_String();
							Start_input = Input_Memory;
						}
					}
					if ( (Start_input < Input_Operation_0) || (Display_Error < Start_input) ) {
						Memory_to_Input_Operation();
					}
					break;

				case 87:                 //    FIX_a_b/c
					break;

				case 88:                 //    y_expo
					add_operation_to_mem( 4, '%' );
					if ( display_string[Expo_point] == '.' ) {
						Found_constant = false;
						temp_operation = 217;
					}
					break;

				case 89:                 //    y_root
					add_operation_to_mem( 4, 'z' );
					if ( display_string[Expo_point] == '.' ) {
						Found_constant = false;
						temp_operation = 218;
					}
					break;

				case 90:                 //    _CE_
					if ( display_string[Memory_0] == 'o' ) {
						temp_operation = mem_stack_input[ 1 ].op;
					}
					if ( Start_input == Off_Status ) {
						Pendular_on = false;
						Beep_on_off = Beep_on_off_temp;
						mem_pointer = mem_stack_count;
						Start_input = Start_Mode;
						digitalWrite(On_Off_PIN, LOW);
						Switch_Code =   0;
						index_5min  = 255;
					}
					if ( Start_input == Input_Memory ) {
						Start_input = Input_Operation_0;
					}
					if ( Start_input < Input_Memory ) {    // Input Number
						if ( (Number_count > 0) || (Point_pos > 0) || display_string[Plus_Minus] == '-') {
							if ( mem_pointer == 0 ) {
								Start_input = Input_Operation_0;
							}
							else {
								Start_input = Start_Mode;
							}
						}
						else {
							if ( mem_stack_count > 1 ) {
								--mem_stack_count;
								max_input = false;
								Start_input = Input_Operation_0;
							}
						}
					}
					if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) || (Start_input == M_Plus_spezial) ) {
						Result_to_Start_Mode();
						Found_constant = false;
					}
					if ( Start_input > Input_Operation_0 ) {
						if ( Start_input < Input_Operation_5 ) { // Input_Operation_1 .. Input_Operation_4
							copy_stack_input_2_calc( mem_stack_count, mem_pointer );
							Clear_String();
							Start_input = Input_Operation_0;
							if ( M_Plus_past == true ) {
								M_Plus_past = false;
								copy_input_left_right( mem_stack_count, 0 );
							}
							else {
								copy_input_left_right( mem_stack_count, mem_stack_count - 1 );
							}
						}
					}
					Mantisse_change = false;
					Expo_change = false;
					No_change = true;
					Error_first = false;
					if ( Start_input == Input_Operation_0 ) {
						mem_pointer = mem_stack_count;
						Start_input = Input_Mantisse;
						Init_expo = false;
						display_digit_temp = display_digit;
						display_digit = 8;
						mem_stack_input[ mem_pointer ].op = temp_op;
						max_input = false;
						Display_Number(mem_stack_input[mem_pointer]);
						display_string[Cursor_pos] = '.';
						while ( display_string[Cursor_pos - 1] == '0' ) {
							display_string[Cursor_pos] = '#';
							--Cursor_pos;
							--Number_count;
							--Zero_count;
							display_string[Cursor_pos] = '_';
						}
						display_digit = display_digit_temp;
					}
					if ( display_string[Memory_0] == 'o' ) {
						display_string[Operation] = '_';
						display_string[Memory_1] = '#';
						display_string[Memory_0] = '#';						
					}
					break;

				case 91:                 //   "FN"- Light_up
					led_bright_index += 1;;
					if ( led_bright_index > led_bright_max ) {
						led_bright_index = led_bright_max;
						Beep__off();
					}
					break;

				case 92:                 //    _//_
					add_operation_to_mem( 3, '|' );
					if ( display_string[Expo_point] == '.' ) {
						Found_constant = false;
						temp_operation = 215;
					}
					break;

				case 93:                 //   "FN"- Light_down
					led_bright_index -= 1;
					if ( led_bright_index < led_bright_min ) {
						led_bright_index = led_bright_min;
						Beep__off();
					}
					break;

				case 94:                 //    _/p/_  Phytagoras
					add_operation_to_mem( 3, 'P' );
					if ( display_string[Expo_point] == '.' ) {
						Found_constant = false;
						temp_operation = 211;
					}
					break;

				case 95:          // FIX_2        10 ..       99 -->  Display
				case 96:          // FIX_3       100 ..      999 -->  Display
				case 97:          // FIX_4      1000 ..     9999 -->  Display
				case 98:          // FIX_5     10000 ..    99999 -->  Display
				case 99:          // FIX_6    100000 ..   999999 -->  Display
				case 100:         // FIX_7   1000000 ..  9999999 -->  Display
				case 101:         // FIX_8  10000000 .. 99999999 -->  Display
					fix_extra_test = Switch_Code - FIX_0;
					if ( Start_input < Input_Memory ) {  // Input Number
						Get_Number( 0 );
					}
					mem_pointer = 0;                           // Display Number
					if ( abs(display_digit) == fix_extra_test ) {
						display_digit *= -1;
					}
					else {
						display_digit  = fix_extra_test;
					}

					Display_Number(mem_stack_input[mem_pointer]);
					Display_Memory_x( 5 );
					break;

				case 135:                //   _M_plus(1)
				case 136:                //   _M_plus(2)
				case 137:                //   _M_plus(3)
				case 138:                //   _M_plus(4)
				case 139:                //   _M_plus(5)
				case 140:                //   _M_plus(6)
				case 141:                //   _M_plus(7)
				case 142:                //   _M_plus(8)
				case 143:                //   _M_plus(9)

				case 225:                //   _M_minus(1)
				case 226:                //   _M_minus(2)
				case 227:                //   _M_minus(3)
				case 228:                //   _M_minus(4)
				case 229:                //   _M_minus(5)
				case 230:                //   _M_minus(6)
				case 231:                //   _M_minus(7)
				case 232:                //   _M_minus(8)
				case 233:                //   _M_minus(9)

				case 234:                //   _M_mul(1)
				case 235:                //   _M_mul(2)
				case 236:                //   _M_mul(3)
				case 237:                //   _M_mul(4)
				case 238:                //   _M_mul(5)
				case 239:                //   _M_mul(6)
				case 240:                //   _M_mul(7)
				case 241:                //   _M_mul(8)
				case 242:                //   _M_mul(9)

				case 243:                //   _M_div(1)
				case 244:                //   _M_div(2)
				case 245:                //   _M_div(3)
				case 246:                //   _M_div(4)
				case 247:                //   _M_div(5)
				case 248:                //   _M_div(6)
				case 249:                //   _M_div(7)
				case 250:                //   _M_div(8)
				case 251:                //   _M_div(9)
					Mem_plus_minus_mul_div();
					break;

				case 120:                //    _EE_
					if ( Number_count == Zero_count ) {
						if ( Start_input == Input_Mantisse ) {
							Mantisse_add_strg_eins();
							Pointer_memory = display_string[Cursor_pos];
							display_string[Cursor_pos] = '#';
							display_string[Expo_point] = '.';
							Start_input = Input_Expo;
							if (Init_expo == true) {
								Init_expo = false;
								display_string[Expo_1] = '0';
								display_string[Expo_0] = '0';
							}
							break;
						}
					}
					else {
						if ( Start_input == Input_Mantisse ) {
							Pointer_memory = display_string[Cursor_pos];
							if ( Point_pos == 0 ) {
								if ( Number_count < 8 ) {
									Put_input_Point();
								}
							}
							if ( Repeat_pos == 0 ) {
								display_string[Cursor_pos] = '#';
							}
							else {
								display_string[Cursor_pos] = '~';
							}
							display_string[Expo_point] = '.';
							Start_input = Input_Expo;
							if (Init_expo == true) {
								Init_expo = false;
								display_string[Expo_1] = '0';
								display_string[Expo_0] = '0';
							}
							break;
						}
						if ( Start_input == Input_Expo ) {
							display_string[Cursor_pos] = Pointer_memory;
							display_string[Expo_point] = ' ';
							Start_input = Input_Mantisse;
							break;
						}
					}
					break;

				case 124:                //    _HM_
					add_operation_to_mem( 4, 'h' );
					if ( display_string[Expo_point] == '.' ) {
						Found_constant = false;
						temp_operation = 219;
					}
					break;

				case 128:                //    _AM_
					add_operation_to_mem( 4, 'f' );
					if ( display_string[Expo_point] == '.' ) {
						Found_constant = false;
						temp_operation = 220;
					}
					break;

				case 129:                //    _GM_
					add_operation_to_mem( 4, 'q' );
					if ( display_string[Expo_point] == '.' ) {
						Found_constant = false;
						temp_operation = 221;
					}
					break;

				case 144:                //    view_down
					if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) || (Start_input == M_Plus_spezial) || (Start_input == Input_Operation_0) ) {
						is_view    = true;
						num_test   = abs( mem_stack_input[ mem_pointer ].num );
						denom_test = mem_stack_input[ mem_pointer ].denom;
						
						if ( num_test.lo < denom_test.lo ) {
							num_test   *= expo_10_[4];     // 1e14
						}
						else {
							num_test   *= expo_10_[3];     // 1e13
						}
						num_test     += mem_stack_input[ mem_pointer ].denom / 2;
						num_test     /= denom_test;
						num_test_64   = num_test;
						itoa_(num_test_64, display_string_itoa_);
						if ( Debug_Level == 59 ) {
							Serial.println(display_string_itoa_);
						}
						for ( index_mem = 0; index_mem < Std_string_view; index_mem += 1 ) {
							Std_mode_string[index_mem]    = display_string[index_mem + 2];
							display_string[index_mem + 2] = display_string_itoa_[index_mem];
						}
						Beep__on();
					}
					break;

				case 145:                //    view_up
					Countdown_view_end =  Countdown_view_end_0; 
					Countdown_view     = true;
					break;

				case 146:                //    view_end
					if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) || (Start_input == M_Plus_spezial) || (Start_input == Input_Operation_0) ) {
						for ( index_mem = 0; index_mem < Std_string_view; index_mem += 1 ) {
							display_string[index_mem + 2] = Std_mode_string[index_mem];
						}
						Beep__on();
						is_view = false;
					}
					break;

				case 149:                 //    __/
					Print_Statepoint();
					break;

				case 151:                 //    _AGM_
					add_operation_to_mem( 4, 'A' );
					if ( display_string[Expo_point] == '.' ) {
						Found_constant = false;
						temp_operation = 222;
					}
					break;

				case 152:                 //    _mod_
					add_operation_to_mem( 3, 'S' );
					if ( display_string[Expo_point] == '.' ) {
						Found_constant = false;
						temp_operation = 223;
					}
					break;

				case 153:                 //    _Std_down_
					Display_mode = true;
					for ( index_mem = 0; index_mem < Std_string_count; index_mem += 1 ) {
						Std_mode_string[index_mem] = display_string[index_mem + 1];
					}
					if ( Std_mode == 1 ) {
						Std_mode = 0;
						if ( display_digit > 0 ) {
							display_digit *= -1;
						}
						mem_stack_max = 2;
						display_string[3] = 'k';
						display_string[4] = 'd';
						display_string[MR_point] = '.';
					}
					else {
						Std_mode = 1;
						if ( display_digit < 0 ) {
							display_digit *= -1;
						}
						mem_stack_max = mem_stack_max_c;
						display_string[3] = 'c';
						display_string[4] = 'i';
						display_string[MR_point] = ' ';
					}
					display_digit_temp = display_digit;
					display_string[1] = '_';
					display_string[2] = '5';
					display_string[5] = '.';
					display_string[6] = '_';
					display_string[7] = '#';
					display_string[8] = '#';
					display_string[9] = '#';
					if ( Point_pos > 9 ) {
						display_string[5] = '#';
					}
					if ( Point_pos == 0 ) {
						display_string[5] = '_';
						display_string[6] = '#';
					}
					if ( Repeat_pos > 0 ) {
						if ((Repeat_pos - Point_pos) > 1 ) {
							display_string[7] = '.';
							if ( Repeat_pos == 10 ) {  
								if ( display_string[Expo_point] != '.' ) {
									display_string[5] = '_';
									display_string[6] = '#';
								}
								display_string[7] = '#';
							}
						}
					}
					if ( Debug_Level == 26 ) {
						Serial.print("Point_pos = ");
						Serial.print(Point_pos);
						Serial.print(" -> Repeat_pos = ");
						Serial.println(Repeat_pos);
					}
					Beep__off();
					break;

				case 154:                 //    _Std_up_
					Display_mode = false;
					for ( index_mem = 0; index_mem < Std_string_count; index_mem += 1 ) {
						display_string[index_mem + 1] = Std_mode_string[index_mem];
					}
					Beep__off();
					break;

				case 157:                 //    _rpt_pt_
					if ( Repeat_pos == 0 ) {
						if ( Number_count < 8 ) {
							Input_Repeat_Point();
						}
					}
					if ( Debug_Level == 51 ) {
						Serial.print(display_string);
						Serial.print(" rpt_pos > ");
						Serial.print(Repeat_pos);
						Serial.print(" pt_pos > ");
						Serial.println(Point_pos);
					}
					break;

				case 160:                 //   SM(MCs)
					for ( index_mem = 1; index_mem < 10; index_mem += 1 ) {
						mem_extra_stack[ index_mem ].num = int32_max;
						mem_extra_stack[ index_mem ].denom = int32_max;
						mem_extra_stack[ index_mem ].expo = expo_min_input;
						mem_extra_stack[ index_mem ].op = temp_op;
					}
					break;

				case 161:                 //   _MS(1)
				case 162:                 //   _MS(2)
				case 163:                 //   _MS(3)
				case 164:                 //   _MS(4)
				case 165:                 //   _MS(5)
				case 166:                 //   _MS(6)
				case 167:                 //   _MS(7)
				case 168:                 //   _MS(8)
				case 169:                 //   _MS(9)
					mem_extra_test = Switch_Code - Min_0;
					if ( Start_input < Input_Memory ) {      // Input Number
						if ( Number_count != Zero_count ) {
							change_number();
							if ( Start_input != Display_Error ) {
								copy_input_left_right( 0, mem_pointer );
								Start_input = Input_Memory;
							}
						}
					}
					if ( Start_input == Display_Result ) {
						mem_stack_count = 1;
						mem_pointer = mem_stack_count;
						Start_input = Input_Memory;
						left_right_mem_extra( mem_pointer, 0 );
						mem_pointer = 0;
					}
					if ( (Start_input == Input_Operation_0) || (Start_input == Input_Memory) || (Start_input == M_Plus_spezial) ) {
						mem_extra_left_right( mem_extra_test, mem_pointer );
						Test_mem_stack_input();
						Start_input = Input_Memory;
						Display_Number(mem_stack_input[mem_pointer]);
						mem_save = true;
						display_string[Memory_1] = Display_Memory_1[3];  // m
						display_string[Memory_0] = '0' + mem_extra_test;
					}
					break;

				case 174:                //    _EE+1_
					if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) || (Start_input == M_Plus_spezial) || (Start_input == Input_Operation_0) ) {
						if ( display_string[Point_pos - 1] > '/' ) {
							expo_temp_16 = Get_Expo();
							if ( expo_temp_16 < 99 ) {
								Init_expo = false;
								++expo_temp_16;
								Put_Expo();
								display_string[Point_pos] = display_string[Point_pos - 1];
								--Point_pos;
								display_string[Point_pos] = '.';
							}
						}
					}
					break;

				case 175:                //    EE-1
					if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) || (Start_input == M_Plus_spezial) || (Start_input == Input_Operation_0) ) {
						if ( display_string[Point_pos + 1] > '/' ) {
							expo_temp_16 = Get_Expo();
							if ( expo_temp_16 > -99 ) {
								Init_expo = false;
								--expo_temp_16;
								Put_Expo();
								display_string[Point_pos] = display_string[Point_pos + 1];
								++Point_pos;
								display_string[Point_pos] = '.';
							}
						}
					}
					break;

				case 176:                //    Dis_Cha_Dir_on
					Print_Statepoint_after();
					break;

				case 179:                //   _M_xch(1)   // MEx
				case 180:                //   _M_xch(2)
				case 181:                //   _M_xch(3)
				case 182:                //   _M_xch(4)
				case 183:                //   _M_xch(5)
				case 184:                //   _M_xch(6)
				case 185:                //   _M_xch(7)
				case 186:                //   _M_xch(8)
				case 187:                //   _M_xch(9)
					if ( (Number_count != Zero_count) || (mem_save == true) || (mem_exchange == true) || (Start_input == Input_Operation_0) ) {
						mem_extra_test = Switch_Code - M_xch_0;
						temp_num = mem_extra_stack[ mem_extra_test ].num;
						temp_denom = mem_extra_stack[ mem_extra_test ].denom;
						temp_expo = mem_extra_stack[ mem_extra_test ].expo;
						if ( Start_input < Input_Memory ) {      // Input Number
							change_number();
							if ( Start_input != Display_Error ) {
								copy_input_left_right( 0, mem_pointer );
								Start_input = Input_Memory;
							}
						}
						if ( Start_input == Input_Operation_0 ) {
							Start_input = Input_Memory;
						}
						if ( Start_input == Input_Memory ) {
							mem_pointer = 0;
							mem_extra_left_right( mem_extra_test, mem_pointer );
							mem_stack_input[ mem_pointer ].num = temp_num;
							mem_stack_input[ mem_pointer ].denom = temp_denom;
							mem_stack_input[ mem_pointer ].expo = temp_expo;
							Test_mem_stack_input();
							Display_Number(mem_stack_input[mem_pointer]);
							mem_exchange = true;
							mem_save = false;
							display_string[Memory_1] = Display_Memory_1[4];  // E
							display_string[Memory_0] = '0' + mem_extra_test;
						}
					}
					break;

				case 191:                //   _to_xx(0)
				case 192:                //   _to_xx(1)
				case 193:                //   _to_xx(2)
				case 194:                //   _to_xx(3)
				case 195:                //   _to_xx(4)
				case 196:                //   _to_xx(5)
				case 197:                //   _to_xx(6)
				case 198:                //   _to_xx(7)
				case 199:                //   _to_xx(8)
				case 200:                //   _to_xx(9)
					to_extra_test = Switch_Code - to_0;
					if ( Start_input < Input_Memory ) {      // Input Number
						Get_Number( 0 );
					}
					if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) || (Start_input == M_Plus_spezial) ) {
						mem_stack_count = 1;
						mem_pointer = mem_stack_count;
						copy_input_left_right( mem_pointer, 0 );
						Start_input = Input_Operation_0;
					}
					if ( (Start_input == Input_Operation_0) || (Start_input == Input_Memory)  ) {
						Start_input = Input_Operation_0;
						Error_Test();
						mem_pointer = mem_stack_count;
						if ( Start_input != Display_Error ) {
							mem_pointer = 0;
							to_temperature = false;
							if ( to_extra_test < 8 ) {
								mem_stack_input[ mem_pointer ] = mul( mem_stack_input[ mem_pointer ], to_xx[ to_extra_test ] );
							} 
							else {
								mem_stack_input[ mem_pointer ] = add_mul_spezial( to_xx[ to_extra_test + 2 ], to_xx[ to_extra_test ], mem_stack_input[ mem_pointer ], 1 );
								if ( mem_stack_input[ mem_pointer ].expo == 0 ) {
									if ( abs(mem_stack_input[ mem_pointer ].num) < abs(mem_stack_input[ mem_pointer ].denom) ) {
										to_temperature = true;
									}
								}
								if ( mem_stack_input[ mem_pointer ].expo < 0 ) {
									to_temperature = true;
								}
							}

							Error_Test();
							mem_pointer = mem_stack_count;
							if ( Start_input != Display_Error ) {
								if ( to_extra_ == true ) {
									mem_pointer = 0;
									Test_mem_stack_input();
									Display_Number(mem_stack_input[mem_pointer]);
									if ( to_extra_test < 8 ) {
										display_string[Memory_1] = Display_Memory_1[12];  // =
									}
									else {
										display_string[Operation] = '_';
										display_string[Memory_1] = 'O';
									}
									display_string[Memory_0]  = '0' + to_extra_test;
									if ( to_extra_test == 8 ) {
										display_string[Memory_0] = 'C';
									}
									if ( to_extra_test == 9 ) {
										display_string[Memory_0] = 'F';
									}
									to_extra = true;
								}
							}
						}
					}
					Display_Memory_x( 23 );
					break;

				case 202:                //    EE(1)
				case 203:                //    EE(2)
				case 204:                //    EE(3)
				case 205:                //    EE(4)
				case 206:                //    EE(5)
				case 207:                //    EE(6)
				case 208:                //    EE(7)
				case 209:                //    EE(8)
				case 210:                //    EE(9)
					to_extra_test = Switch_Code - EE_0;
					if ( Start_input == Input_Expo ) {
						display_string[Plus_Minus_Expo] = '#';
						display_string[Expo_1] = '0';
						switch (to_extra_test) {

							case 1:
								display_string[Plus_Minus_Expo] = '-'; [[fallthrough]];
							case 9:
								display_string[Expo_1] = '1';
								display_string[Expo_0] = '2';
								break;

							case 2:
								display_string[Plus_Minus_Expo] = '-'; [[fallthrough]];
							case 8:
								display_string[Expo_0] = '9';
								break;

							case 3:
								display_string[Plus_Minus_Expo] = '-'; [[fallthrough]];
							case 7:
								display_string[Expo_0] = '6';
								break;

							case 4:
								display_string[Plus_Minus_Expo] = '-'; [[fallthrough]];
							case 6:
								display_string[Expo_0] = '3';
								break;

							case 5:
								display_string[Expo_0] = '0';
								break;

						}
						display_string[Memory_1] = Display_Memory_1[15];  // X
						display_string[Memory_0] = '0' + to_extra_test;

						expo_exchange = true;
					}
					break;
			
				case 252:                 //    _+(%)_
				case 253:                 //    _-(%)_
				case 254:                 //    _*(%)_
				case 255:                 //    _/(%)_
				case 122:                 //    Beep_On_Off
				case 147:                 //    fail_view
				case 211:                 //    _/p/ /p/_
				case 212:                 //    _- -_
				case 213:                 //    _* *_
				case 214:                 //    _/ /_
				case 215:                 //    _// //_
				case 216:                 //    _+ +_
				case 217:                 //    _y_expo y_expo_
				case 218:                 //    _y_root y_root_
				case 219:                 //    _HM HM_
				case 220:                 //    _AM AM_
				case 221:                 //    _GM GM_
				case 222:                 //    _AGM AGM_
				case 223:                 //    _mod_mod_
				case 201:                 //    Beep
					Switch_Code = 0;
					break;

				case 103:                //   _M+()_
				case 104:                //   _M-()_
				case 105:                //   _Mx()_
				case 106:                //   _M/()_
				case 107:                //   _Mr()_
				case 108:                //   _M_()_
					display_string[Memory_1] = Display_Memory_1[2];
					Display_Status_mem       = Switch_Code;
					Display_Status_mem      -= 102;
					display_string[Memory_0] = Memory_p_min_m_d[Display_Status_mem];					
					break;
			/*
				case 188:                //    Dis_Memory_X_off
				case 30:                 //    <--
				case 32:                 //    FIX_hms
				case 102:                //    FIX_E24
				case 177:                //    Dis_Cha_Dir_off
				case 40:                 //    _(_
				case 148:                //    _->>_
				case 150:                //    Â° ' ''
				case 127:                //    _-->_
			*/
				default:
					Beep__off();
					break;
			}
			Print_Statepoint_after();
		}

		if ( Switch_Code > EE_9 ) {         //    EE(9)
			Switch_Code = 0;
		}
		else {
			if (  Switch_Code > 0 ) {
				if ( temp_operation > EE_9 ) {  //    EE(9)
					Switch_Code = temp_operation;
					if ( temp_operation > 251 ) {  //    (%)
						Function_1_number();
						display_string[Operation] = Char_temp;
						display_string[Memory_1] = 'O';
						display_string[Memory_0] = 'o';
					}
				}
				else {
					if ( Switch_Code == 108 ) {
						Switch_Code = 144;
					}
					else {
						Switch_Code = 0;
					}
				}
			}
		}

		if ( Test_to_Result == false ) {
			Test_to_Result = true;
			if ( mem_stack_test == 1 ) {
				mem_stack_test = 0;
				Switch_Code = 61;               //    _=_
			}
		}

		if ( pgm_count_a > 0 ) {
			Switch_Code  = pgm_content_a[pgm_count_a];            //    _=_
			--pgm_count_a;
		}

		index_5min  = 255;
	}

	if ( time_7500ms == true ) { // here al that will make in 7.5 sec
		time_7500ms = false;
		++index_5min;              // Switch Off in 5min
		if ( index_5min == pendel_3min ) {  // 23 - after 3min
			if (Pendular_on == false) {
				Pendular_on = true;
				Start_mem = Start_input;
				Beep_on_off = false;
			}
		}
		if ( index_5min == time_5min ) {  // 39 - after 5min
			Beep_on_off = true;
			Switch_Code = 125;       // Off 5min
		}
	}

	if ( time_1000ms == true ) { // here al that will make in 1 Hz
		time_1000ms = false;
		++count_1000ms;            // 1000 ms

		if ( Debug_Level == 2 ) {  // 996,09375 ms  <>  1000 ms
			time = millis();
			Serial.print(count_1000ms);
			Serial.print("  ");
			Serial.println(time);
		}
	}

	if ( time_100ms == true ) {  // here al that will make in 10 Hz
		time_100ms = false;
		++count_100ms;             // 100 ms
		++index_100ms;
		++index_7500ms;            // 7500 ms

		if ( index_100ms == 9 ) {
			index_100ms = 255;
			time_1000ms = true;
		}

		if ( index_7500ms == 74 ) {
			index_7500ms = 255;
			time_7500ms = true;
		}

		if ( Debug_Level == 1 ) {  // 99,609375 ms  <>  100 ms
			time = millis();
			Serial.print(count_100ms);
			Serial.print("  ");
			Serial.println(time);
		}
		if ( count_100ms == 8 ) {  // after 3 Second --> Input_Cursor
			Print_Statepoint();
			Start_input = Start_Mode;
			Switch_Code =   0;

			Display_Status_new = 0;    // Switch up / down
			Display_Status_old = 0;
			Display_Status_mem = 0;

			Print_Statepoint_after();
			if ( Debug_Level == 23 ) {  // On - Off - Test
				Display_Status_new = 152;
				Display_Off();
				Countdown_OFF = Countdown_Off_2;      // Switch Off  2x Beep
			}
		}
	}

	if ( Display_new == true ) { // Display refresh
		if ( display_string[5] == 'E' ) {
			if ( Switch_Code_old > EE_9 ) {         //
				Beep__on();
			}
			if ( Debug_Level == 31 ) {
				Serial.print("Switch_Code =  ");
				Serial.println(Switch_Code);
			}
		}
		Switch_Code_old = Switch_Code;
		index_LED = 0;

					if ( Debug_Level == 51 ) {
						Serial.print(display_string);
						Serial.print(" _rpt_pos > ");
						Serial.print(Repeat_pos);
						Serial.print(" _pt_pos > ");
						Serial.println(Point_pos);
					}

		for ( index_a = 0; index_a <= Digit_Count; index_a += 1 ) {
			if ( display_string[index_LED] == '_' ) {
				if ( Repeat_pos > 0 ) {
					if ( Number_count < 8 ) {
						if ( display_string[index_LED + 1] != '.' ) {
							display_string[index_LED + 1] = '~';
						}
					}
				}
			}
			if ( display_string[index_LED] == '~' ) {
				if ( display_string[index_LED + 1] == '~' ) {
					if ( Number_count < 8 ) {
						display_string[index_LED + 1] = '#';
					}
				}
				if ( Repeat_pos == 0 ) {
					display_string[index_LED] = '#';
				}
			}
			
			if ( index_LED == MS_point ) {
				if ( (Repeat_pos - Point_pos) == 1 ) {
					if ( Number_count == 8 ) {
						display_string[index_LED] = '~';
					}
				}
			}

			if ( display_string[index_LED] == '.' ) {
				--index_a;
				display_b[index_a] = display_b[index_a] + led_font[( display_string[index_LED] - start_ascii )] ;
			}
			else {
				display_b[index_a] = led_font[( display_string[index_LED] - start_ascii )] ;
			}

			if ( display_string[index_LED - 1] == '~' ) {
				display_b[index_a - 1] = display_b[index_a] + led_font[( display_string[index_LED - 1] - start_ascii )];
				if ( Number_count == 8 ) {
					--index_a;
				}
				if ( Number_count == 7 ) {
					if ( (Repeat_pos - Point_pos) != 1 ) {
						if ( display_string[MS_point] != '#' ) {
							--index_a;
						}
					}
				}
			}
		 
			if ( index_LED == MS_point ) {
				if ( display_string[index_LED] == '#' ) {
					--index_a;
					if ( (Repeat_pos - Point_pos) == 1 ) {
						--index_a;
					}
				}
			}
			if ( index_LED == Plus_Minus_Expo__ ) {
				if ( display_string[index_LED] == '#' ) {
					if ( Point_pos == 0 ) {
						--index_a;
					}
				}
			}
			if ( index_LED == Plus_Minus_Expo_ ) {
				if ( display_string[index_LED] == '#' ) {
					if ( Repeat_pos == 0 ) {
						--index_a;
					}
				}
			}
		 
			if ( display_string[index_LED] == ' ' ) {
				++index_LED;
				if ( display_string[index_LED] == ' ' ) {
					++index_LED;
				}
				else {
					display_b[index_a] = led_font[( display_string[index_LED] - start_ascii )] ;
					++index_LED;
				}
			}
			else {
				++index_LED;
			}
		}

		if ( Number_count == 7 ) {
			if ( (Repeat_pos - Point_pos) == 1 ) {  
				 if ( display_string[Plus_Minus_Expo_] == '~' ) {
					display_b[9] += led_font[ '~' - start_ascii  ];
				}
			}
		}
		if ( Number_count == 8 ) {
			if ( Repeat_pos > 0 ) {
				if ( (Repeat_pos - Point_pos) > 1 ) {
					if ( display_string[MS_point] != '~' ) {
						display_b[9] += led_font[ '~' - start_ascii  ];
					}
				}
			}
		}

					if ( Debug_Level == 51 ) {
						Serial.print(display_string);
						Serial.print(" __rpt_pos > ");
						Serial.print(Repeat_pos);
						Serial.print(" __pt_pos > ");
						Serial.println(Point_pos);
					}

		for ( index_b = 0; index_b < 8; index_b += 1 ) {
			count_led[index_b] = 0;
			for ( index_c = 0; index_c < Digit_Count; index_c += 1 ) {
				if ( (bitRead(display_b[index_c], index_b)) == 1 ) {
					switch (index_c) {

						case 14:
						case 13:
						case 12:
							if ( index_b > 0 ) {
								count_led[index_b - 1] += 5;    // 5
							}
							else {
								count_led[7] += 5;              // 5
							}
							break;

						case 11:
						case 10:
						case 9:
						case 8:
						case 7:
							if ( index_b > 0 ) {
								count_led[index_b - 1] += 6;    // 6
							}
							else {
								count_led[7] += 6;              // 6
							}
							break;

						case 6:
						case 5:
						case 4:
						case 3:
							if ( index_b > 0 ) {
								count_led[index_b - 1] += 7;    // 7
							}
							else {
								count_led[7] += 7;              // 7
							}
							break;

						case 2:
						case 1:
						case 0:
							if ( index_b > 0 ) {
								count_led[index_b - 1] += 9;    // 9
							}
							else {
								count_led[7] += 9;              // 9
							}
					}
				}
			}
		}
		Display_new = false;
		Display_change = true;
	}

	if ( switch_Pos_0 == true ) {
		for ( index_a = 0; index_a < Switch_Count; index_a += 1 ) {
			if (taste[index_a] > Switch_up_b) {
				taste[index_a] = Switch_down_max;
			}
		}
		switch_Pos_0 = false;
	}

	if ( time_10ms == true ) {   // here al that will make in 95 Hz
		time_10ms = false;
		++count_10ms;              // (10 + 30/57) ms
		++index_10ms;

		if ( Countdown_view == true ) {
			--Countdown_view_end;
			if ( Countdown_view_end == 0 ) {
				Countdown_view = false;
				Switch_Code = 146;
			}
		}

		if ( Countdown_OFF > 0 ) {
			--Countdown_OFF;


			switch (Countdown_OFF) { // Off

				case Countdown_Off_0 - 1:
					reboot();
					break;

				case Countdown_Off_0 + 1:
					Power_on = false;
					break;

				case 25:
					strcpy( display_string, string_start );
					display_string[2]  = '#';
					Display_new = true;
					break;

				case 35:
					Beep__on();
					break;

				case Countdown_Off_1 - 10:
					Pendular_on = false;
					strcpy( display_string, string_end );
					Display_new = true; [[fallthrough]];
				case Countdown_Off_2 - 10:
				case Countdown_Off_3 -  5:
					Beep__on();
					break;
			}
		}

		if ( Start_input == Start_Mode ) {
			Clear_String();
			Beep__on();
		}

		switch (index_10ms) {

			case 0:
			case 9:
				time_100ms = true;
				break;

			case 18:
				index_10ms = 255;
				break;
		}

		if ( Pendular_on == true ) {
			switch (index_pendel_a) {

				case 0:
				case 181:
					display_b[0] = 32;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  -15.0");
					}
					break;

				case 8:
				case 175:
					display_b[0] = 34;
					display_b[1] = 0;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  -14.0");
					}
					break;

				case 14:
				case 171:
					display_b[0] = 2;
					display_b[1] = 32;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  -13.0");
					}
					break;

				case 18:
				case 168:
					display_b[0] = 0;
					display_b[1] = 34;
					display_b[2] = 0;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  -12.0");
					}
					break;

				case 21:
				case 165:
					display_b[1] = 2;
					display_b[2] = 48;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  -11.0");
					}
					break;

				case 24:
				case 162:
					display_b[1] = 0;
					display_b[2] = 54;
					display_b[3] = 0;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  -10.0");
					}
					break;

				case 27:
				case 159:
					display_b[2] = 6;
					display_b[3] = 48;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  -9.0");
					}
					break;

				case 30:
					Beep__on();
					Display_new = false; [[fallthrough]];
				case 157:
					display_b[2] = 0;
					display_b[3] = 54;
					display_b[4] = 0;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  -8.0");
					}
					break;

				case 32:
				case 155:
					display_b[3] = 6;
					display_b[4] = 16;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  -7.0");
					}
					break;

				case 34:
				case 153:
					display_b[3] = 0;
					display_b[4] = 20;
					display_b[5] = 0;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  -6.0");
					}
					break;

				case 36:
				case 151:
					display_b[4] = 4;
					display_b[5] = 16;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  -5.0");
					}
					break;

				case 38:
				case 149:
					display_b[4] = 0;
					display_b[5] = 20;
					display_b[6] = 0;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  -4.0");
					}
					break;

				case 40:
				case 147:
					display_b[5] = 4;
					display_b[6] = 16;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  -3.0");
					}
					break;

				case 42:
				case 145:
					display_b[5] = 0;
					display_b[6] = 20;
					display_b[7] = 0;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  -2.0");
					}
					break;

				case 44:
				case 143:
					display_b[6] = 4;
					display_b[7] = 16;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  -1.0");
					}
					break;

				case 46:
				case 141:
					display_b[6] = 0;
					display_b[7] = 20;
					display_b[8] = 0;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  0.0");
					}
					break;

				case 48:
				case 139:
					display_b[7] = 4;
					display_b[8] = 16;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  1.0");
					}
					break;

				case 50:
				case 137:
					display_b[7] = 0;
					display_b[8] = 20;
					display_b[9] = 0;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  2.0");
					}
					break;

				case 52:
				case 135:
					display_b[8] = 4;
					display_b[9] = 16;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  3.0");
					}
					break;

				case 54:
				case 133:
					display_b[8] = 0;
					display_b[9] = 20;
					display_b[10] = 0;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  4.0");
					}
					break;

				case 56:
				case 131:
					display_b[9] = 4;
					display_b[10] = 16;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  5.0");
					}
					break;

				case 58:
				case 129:
					display_b[9] = 0;
					display_b[10] = 20;
					display_b[11] = 0;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  6.0");
					}
					break;

				case 60:
				case 127:
					display_b[10] = 4;
					display_b[11] = 48;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  7.0");
					}
					break;

				case 125:
					Beep__on();
					Display_new = false; [[fallthrough]];
				case 62:
					display_b[10] = 0;
					display_b[11] = 54;
					display_b[12] = 0;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  8.0");
					}
					break;

				case 64:
				case 122:
					display_b[11] = 6;
					display_b[12] = 48;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  9.0");
					}
					break;

				case 67:
				case 119:
					display_b[11] = 0;
					display_b[12] = 54;
					display_b[13] = 0;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  10.0");
					}
					break;

				case 70:
				case 116:
					display_b[12] = 6;
					display_b[13] = 32;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  11.0");
					}
					break;

				case 73:
				case 113:
					display_b[12] = 0;
					display_b[13] = 34;
					display_b[14] = 0;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  12.0");
					}
					break;

				case 76:
				case 109:
					display_b[13] = 2;
					display_b[14] = 32;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  13.0");
					}
					break;

				case 80:
				case 103:
					display_b[13] = 0;
					display_b[14] = 34;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  14.0");
					}
					break;

				case 86:
				case 94:
					display_b[14] = 2;

					if ( Debug_Level == 6 ) {
						time = millis();
						Serial.print(time);
						Serial.println("  15.0");
					}
					break;

			}
			Display_change = true;
		}

		if ( index_pendel_a == 189 ) {
			index_pendel_a = 255;
		}
		++index_pendel_a;

		if ( Beep_on == false ) {
			Test_buffer = q.pull(&Switch_down);
			if ( Test_buffer == true ) {
				if ( Debug_Level == 4 ) {
					Serial.print(time);
					Serial.print("  Switch_down  ");
					Serial.print(Switch_down);
					Serial.print("  Switch_old  ");
					Serial.println(Switch_old);
				}
				Test_Switch_up_down();				
			}
		}

		if ( Debug_Level == 3 ) {
			time_down = millis();
			time = time_down - time_old;
			Serial.print(time);
			Serial.print("  ");
			Serial.println(taste[2]);
		}

		if ( Display_Status_new != Display_Status_old ) {
			if ( Switch_down == 0 ) {
				Display_Status_new = 0;
				Mr_0_test = false;
				Display_Status_mem = 0;
			}

			Display_Memory_x( 19 );

			if ( expo_exchange == true ) {
				display_string[Memory_1] = Display_Memory_1[15];  // X;
				display_string[Memory_0] = '0' + to_extra_test;
			}

			if ( to_extra == true ) {
				display_string[Memory_1] = Display_Memory_1[12];  // =
				display_string[Memory_0] = '0' + to_extra_test;
			}

			if ( Display_Status_new > Display_Status_old ) {
				expo_exchange = false;
				mem_exchange = false;
				mem_save = false;
			}

			Beep__on();
			switch (Display_Status_new) {   // Display --> Display_Status_new

				case 24:       // MR
					Display_Memory_x( 2 );      // MR
					if ( Mr_0_test == true ) {
						display_string[Memory_0] = '0' + mem_extra_test;
						Mr_0_test = false;
					}
					break;

				case 44:       // MCs
					Display_Memory_x( 22 );      // Mc
					break;

				case 48:       // MS
				case 112:      // MS
					if ( (Number_count != Zero_count) || (mem_save == true) || (Start_input == Display_Result) || (Start_input == Input_Operation_0) || (Start_input == Input_Memory) || (Start_input == M_Plus_spezial) ) {
						Display_Memory_x( 3 );      // MS
						mem_extra_test = 0;
					}
					else {
						Display_Memory_x( 0 );
					}

					if ( Start_input == Display_M_Plus ) {
						Display_Memory_x( 3 );      // MS
						mem_extra_test = 0;
						Start_input = Display_Result;
						mem_extra_left_right( 0, 0 );
					}
					break;

				case 40:      // Display
					Display_Memory_x( 5 );
					break;

				case 1:       // EE to ..
					if ( Start_input == Input_Expo ) {
						Display_Memory_x( 15 );
					}
					else {
						if ( Display_Status_old == 3 ) {
							Display_Memory_x( 16 );
							Switch_Code = 157;  //  rpt_pt
						}
					}
					if ( expo_exchange == true ) {
						display_string[Memory_0] = '0' + to_extra_test;
					}
					break;

				case 2:       // MEx
					if ( (Start_input == Input_Memory) || (Start_input == Display_Result) || (Start_input == Display_M_Plus) || (Start_input == M_Plus_spezial) ) {
						Start_input = Input_Operation_0;
					}
					if ( (Number_count != Zero_count) || (mem_save == true) || (mem_exchange == true) || (Start_input == Input_Operation_0) ) {
						Display_Memory_x( 4 );
					}
					else {
						Display_Memory_x( 17 );
					}
					if ( Start_input == Display_M_Plus ) {
						Start_input = Display_Result;
					}
					if ( Start_input == Display_Result ) {
						mem_save = true;
					}
					if ( mem_exchange == true ) {
						display_string[Memory_0] = '0' + mem_extra_test;
						Display_Memory_x( 23 );
					}
					break;

				case 4:       // -->]  to ..
					if ( Display_Status_old == 6 ) {
						Display_Memory_x( 12 );
						to_extra_ = true;
					}
					if ( to_extra == true ) {
						display_string[Memory_1] = Display_Memory_1[12];  // =
						display_string[Memory_0] = '0' + to_extra_test;
					}
					break;

				case 8:       // Invers
					Display_Memory_x( 6 );
					if ( Switch_down == 3) {
						Switch_Code = 120;
					}
					if ( (Start_input < Input_Operation_0) || (Display_Error < Start_input) ) {
						if ( mem_save == true ) {
							display_string[Memory_1] = Display_Memory_1[2];    // MR
							display_string[Memory_0] = '0' + mem_extra_test;
						}
					}
					break;

				case 16:      // Fn
					Display_Memory_x( 7 );
					if ( (Start_input < Input_Operation_0) || (Display_Error < Start_input) ) {
						if ( mem_save == true ) {
							display_string[Memory_1] = Display_Memory_1[2];    // MR
							display_string[Memory_0] = '0' + mem_extra_test;
						}
					}
					break;

				case 32:      // =
				case 96:      // =
					Display_Memory_x( 8 );
					if ( Start_input < Display_Result ) {
						Display_Memory_x( 25 );  
					}
					if ( (Number_count != Zero_count) || (mem_save == true) || (mem_exchange == true) || (Start_input == Input_Memory) || (Start_input == Input_Operation_0) ) {
						Display_Memory_x( 9 );  //  m_plus
						if ( mem_extra_test > 0 ) {
							display_string[Memory_0] = '0' + mem_extra_test;
							mem_extra_test = 0;
						}
					}
					break;

				case 152:     // Off
					Display_Memory_x( 10 );
					break;

				case 3:       // Â°"
					if ( Start_input == Input_Expo ) {  // EE to ..
						Display_Memory_x( 15 );
					}
					else {
						Display_Memory_x( 13 );
					}
					if ( expo_exchange == true ) {
						display_string[Memory_0] = '0' + to_extra_test;
					}
					break;

				case 6:       // _,_/
					Display_Memory_x( 14 );
					break;

				default:
					Beep__off();
					break;
			}

			if ( Start_input == Off_Status ) {
				Display_Memory_x( 10 );     // Off
			}
			if ( Display_Status_old == 255 ) {
				Beep__off();
			}
			if ( Display_Status_old > Display_Status_new ) {
				Beep__off();

				switch (Display_Status_old) {

					case 12:
					case  9:
					case  5:
					case  1:   // EE to ..
					case  4:   // -->]  to ..
						Display_Memory_x( 19 );
						if ( Display_Status_new == 0 ) {
							if ( expo_exchange == true ) {
								Display_Memory_x( 24 );
								expo_exchange = false;
							}
							to_extra  = false;
							to_extra_ = false;
						}
						break;

					case  2:   //  MEx
						Display_Memory_x( 19 );
						if ( Start_input == Input_Memory ) {
							Start_input = Input_Operation_0;
						}
						break;

					case 0:         // __
					case 16:        // FN
					case 24:        // MR
					case 32:        // =
					case 40:        // Display
					case 48:        // MS
						if ( mem_save == false ) {
							Display_Status_new = 0;
						}
						Display_Memory_x( 19 );
						break;
				}
			
				if ( Start_input == Display_Result ) {
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_39  ");
						Serial.println(Display_Status);
					}			
					if ( Display_Status_new[6] == 0 ) {
						Display_Memory_x( 8 );
						if ( mem_plus_test > 0 ) {
							display_string[Memory_1] = Display_Memory_1[3];      // MS
							display_string[Memory_0] = '0' + mem_plus_test;
						}
						if ( mem_extra_test > 0 ) {
							display_string[Memory_1] = Display_Memory_1[3];      // MS
							display_string[Memory_0] = '0' + mem_extra_test;
							mem_extra_test = 0;
						}
					}
				}

				if ( Start_input == Input_Mantisse ) {
					if ( Debug_Level == 63 ) {
						Serial.print("BitBool_test_40  ");
						Serial.println(Display_Status_new);
					}			
					if ( Display_Status_new[6] == 0 ) { //      "M+"
						Display_Memory_x( 19 );
					}
				}
			}

			if ( Debug_Level == 17 ) {
				Serial.print("Display_Status_old =  ");
				Serial.println(Display_Status_old);
			}

			Display_Status_old = Display_Status_new;
			Display_new = true;

			if ( Debug_Level == 17 ) {
				Serial.print("Display_Status_new =  ");
				Serial.println(Display_Status_new);
			}
		}

		if ( Debug_Level == 3 ) {
			time_old = time_down;
		}
	}

	if ( test_index == true ) {
		Test_all_function();
	}
}

volatile uint16_t temp_pwm;

/// --------------------------
/// Timer ISR Timer Routine
/// --------------------------
void timer_Isr() {
        temp_pwm = test_pwm;

	++index_Switch;
	++index_TIME;

	switch (index_TIME) {

		case  3:       // +33
		case 35:       // +32
		case 65:       // +30
			Timer1.setPeriod(Time_HIGH);  // sets timer1 to a period of 264 microseconds
			break;

		case  8:       //  5x
		case 40:       //  5x
		case 70:       //  5x
			Timer1.setPeriod(Time_LOW);   // sets timer1 to a period of 263 microseconds
			break;

		case 94:
			index_TIME = 255;
			break;
	}

	if ( index_Switch % 5 == 0 ) {   // Segmente ausschalten --> Segment "a - f - point"
		for ( Digit = Digit_Count - 1; Digit >= 0; Digit -= 1) {
			if ( display_a[Digit] > 0 ) {
				if ( (bitRead(display_a[Digit], index_Display)) == 1 ) {
					digitalWrite(index_display[Digit], HIGH);
				}
			}
			if ( Display_change == true ) {
				display_a[Digit] = display_b[Digit];
			}
		}
		Display_change = false;
		temp_pwm += count_led[index_Display];
		if ( index_Display == 6 ) {
			temp_pwm = led_bright[led_bright_index + 2];    // duty cycle goes from 0 to 1023
		}
		if ( Power_on == true ) {
			Timer1.pwm(PWM_Pin, temp_pwm);    // duty cycle goes from 0 to 1023
		}
		else {
			Timer1.pwm(PWM_Pin, 0);    // duty cycle goes from 0 to 1023
		}
	}

	switch (index_Switch) {          //  =  Tastenabfrage - Analog

		case 85:            //  1
			Mux_change(1); [[fallthrough]];
		case 1:
		case 4:
		case 43:
		case 82:
			Switch_number = 0;
			Switch_read = analogRead(A_0);
			break;

		case 5:             //  1
			Mux_change(1); [[fallthrough]];
		case 2:
		case 41:
		case 44:
		case 83:
			Switch_number = 1;
			Switch_read = analogRead(A_1);
			break;

		case 45:            //  1
			Mux_change(1); [[fallthrough]];
		case 3:
		case 42:
		case 81:
		case 84:
			Switch_number = 2;
			Switch_read = analogRead(A_2);
			break;

		case 10:            //  2
			Mux_change(2); [[fallthrough]];
		case 7:
		case 46:
		case 49:
		case 88:
			Switch_number = 3;
			Switch_read = analogRead(A_0);
			break;

		case 50:            //  2
			Mux_change(2); [[fallthrough]];
		case 8:
		case 47:
		case 86:
		case 89:
			Switch_number = 4;
			Switch_read = analogRead(A_1);
			break;

		case 90:            //  2
			Mux_change(2); [[fallthrough]];
		case 6:
		case 9:
		case 48:
		case 87:
			Switch_number = 5;
			Switch_read = analogRead(A_2);
			break;

		case 55:            //  3
			Mux_change(3); [[fallthrough]];
		case 13:
		case 52:
		case 91:
		case 94:
			Switch_number = 6;
			Switch_read = analogRead(A_0);
			break;

		case 95:            //  3
			Mux_change(3); [[fallthrough]];
		case 11:
		case 14:
		case 53:
		case 92:
			Switch_number = 7;
			Switch_read = analogRead(A_1);
			break;

		case 15:            //  3
			Mux_change(3); [[fallthrough]];
		case 12:
		case 51:
		case 54:
		case 93:
			Switch_number = 8;
			Switch_read = analogRead(A_2);
			break;

		case 100:           //  4
			Mux_change(4); [[fallthrough]];
		case 16:
		case 19:
		case 58:
		case 97:
			Switch_number = 9;
			Switch_read = analogRead(A_0);
			break;

		case 20:            //  4
			Mux_change(4); [[fallthrough]];
		case 17:
		case 56:
		case 59:
		case 98:
			Switch_number = 10;
			Switch_read = analogRead(A_1);
			break;

		case 60:            //  4
			Mux_change(4); [[fallthrough]];
		case 18:
		case 57:
		case 96:
		case 99:
			Switch_number = 11;
			Switch_read = analogRead(A_2);
			break;

		case 25:            //  5
			Mux_change(5); [[fallthrough]];
		case 22:
		case 61:
		case 64:
		case 103:
			Switch_number = 12;
			Switch_read = analogRead(A_0);
			break;

		case 65:            //  5
			Mux_change(5); [[fallthrough]];
		case 23:
		case 62:
		case 101:
		case 104:
			Switch_number = 13;
			Switch_read = analogRead(A_1);
			break;

		case 105:           //  5
			Mux_change(5); [[fallthrough]];
		case 21:
		case 24:
		case 63:
		case 102:
			Switch_number = 14;
			Switch_read = analogRead(A_2);
			break;

		case 70:            //  6
			Mux_change(6); [[fallthrough]];
		case 28:
		case 67:
		case 106:
		case 109:
			Switch_number = 15;
			Switch_read = analogRead(A_0);
			break;

		case 110:           //  6
			Mux_change(6); [[fallthrough]];
		case 26:
		case 29:
		case 68:
		case 107:
			Switch_number = 16;
			Switch_read = analogRead(A_1);
			break;

		case 30:            //  6
			Mux_change(6); [[fallthrough]];
		case 27:
		case 66:
		case 69:
		case 108:
			Switch_number = 17;
			Switch_read = analogRead(A_2);
			break;

		case 115:           //  7
			Mux_change(7); [[fallthrough]];
		case 31:
		case 34:
		case 73:
		case 112:
			Switch_number = 18;
			Switch_read = analogRead(A_0);
			break;

		case 35:            //  7
			Mux_change(7); [[fallthrough]];
		case 32:
		case 71:
		case 74:
		case 113:
			Switch_number = 19;
			Switch_read = analogRead(A_1);
			break;

		case 75:            //  7
			Mux_change(7); [[fallthrough]];
		case 33:
		case 72:
		case 111:
		case 114:
			Switch_number = 20;
			Switch_read = analogRead(A_2);
			break;

		case 40:            //  0
			Mux_change(0); [[fallthrough]];
		case 37:
		case 76:
		case 118:
			Switch_number = 21;
			Switch_read = analogRead(A_0);
			break;

		case 79:
			Test_pwm();
			Switch_number = 21;
			Switch_read = analogRead(A_0);
			break;

		case 80:            //  0
			Mux_change(0); [[fallthrough]];
		case 38:
		case 77:
		case 116:
			Switch_number = 22;
			Switch_read = analogRead(A_1);
			break;

		case 119:
			index_Switch = 255;
			Test_pwm();
			Switch_number = 22;
			Switch_read = analogRead(A_1);
			break;

		case 0:             //  0
			Mux_change(0); [[fallthrough]];
		case 36:
		case 78:
		case 117:
			Switch_number = 23;
			Switch_read = analogRead(A_2);
			break;

		case 39:
			Test_pwm();
			Switch_number = 23;
			Switch_read = analogRead(A_2);
			break;
	}
	
	taste[Switch_number] -= (taste[Switch_number] << 1) / Average;
	taste[Switch_number] += (Switch_read << 1) / Average;

	if ( bitRead(Switch_new, Switch_number) == 0 ) {
		if ( bitRead(Switch_up, Switch_number) == 0 ) {  //  Pos. 0
			if (taste[Switch_number] < Switch_up_b) {
				if ( switch_Pos_0 == false ) {
					bitWrite(Switch_up, Switch_number, 1);
					switch_Pos_0 = true;
				}
			}
		}
		else {                                           //  Pos. 1
			if (taste[Switch_number] < Switch_down_b) {
				bitWrite(Switch_new, Switch_number, 1);
				taste[Switch_number] = Switch_up_start;
			}
		}
	}
	else {
		if ( bitRead(Switch_up, Switch_number) == 1 ) {  //  Pos. 2
			if (taste[Switch_number] > Switch_down_b) {
				bitWrite(Switch_up, Switch_number, 0);
			}
		}
		else {                                           //  Pos. 3
			if (taste[Switch_number] > Switch_up_b) {
				bitWrite(Switch_new, Switch_number, 0);
				taste[Switch_number] = Switch_down_start;
			}
		}
	}

	if ( Start_input > First_Display ) {
		if ( Switch_new != Switch_old_temp ) {
			Test_buffer = q.add(Switch_new);
			Switch_old_temp = Switch_new;
			if ( Test_buffer == false ) {    // Failure - Pendel go on
				if (Pendular_on == false) {
					Pendular_on = true;
					Start_mem = Start_input;
					Beep_on_off = false;
				}
			}
		}
	}

	if ( Beep_on == true ) {
		if ( Beep_count > min_Beep_count ) {
			--Beep_count;
		}
		if (Beep_count < 0) {
			digitalWrite(Beep_m, HIGH);
			digitalWrite(Beep_p, HIGH);
			if ( Beep_count <= min_Beep_count ) {
				pinMode(Beep_m, INPUT_PULLUP); // Pin A3
				pinMode(Beep_p, INPUT_PULLUP); // Pin A7
				Beep_count = max_Beep_count;
				Beep_on = false;
			}
		}
		else {       //  0 .. 63
			if ( Beep_on_off == true ) {
				if ( Beep_count > 63 ) {
					pinMode(Beep_m, OUTPUT);     // Pin A3
					digitalWrite(Beep_m, HIGH);
					pinMode(Beep_p, OUTPUT);     // Pin A7
					digitalWrite(Beep_p, HIGH);
				}
				else {   //  0 .. 63
					digitalWrite( Beep_m, bitRead(Beep_patt_m, Beep_count) );    // Toggle Beep
					digitalWrite( Beep_p, bitRead(Beep_patt_p, Beep_count) );    // Toggle Beep
				}
			}
		}
	}
}
