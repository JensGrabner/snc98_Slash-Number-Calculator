/**********************************************************************

   Project:   snc98 - Slash Number Calculator

   Developer: Jens Grabner
   Email:     jens@grabner-online.org
   Date:      Dec 2017

   Copyright CERN 2013.
   This documentation describes Open Hardware and is licensed
   under the CERN OHL v. 1.2 or later.

   You may redistribute and modify this documentation under
   the terms of the CERN Open Hardware Licence v.1.2.
   (http://ohwr.org/cernohl).

   This documentation is distributed
   WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING OF
   MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A
   PARTICULAR PURPOSE.

   Please see the CERN OHL v.1.2 for applicable conditions.

**********************************************************************/

/*                          HEADER

  *********************************************************************
  * Program for my "Retro Calculator with 15 Digit"  v 0.2            *
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

  Changes:
  2015_02_20  -  V 0.1  - Initial Release
                          Display and Switches will work - (Debug_Level 5)
  2015_03_13  -  V 0.1a - Pendulum will work - (Debug_Level 6)
  2015_04_08  -  V 0.1b - Change to Arduino 1.6.2
                          Software Switch off - Hardware Rev. 0.4
  2015_07_08  -  V 0.1e - get_Expo - (Debug_Level 7) EE+3 and EE-3
  2015_08_06  -  V 0.1f - Display_Number - (Debug_Level 9)
  2015_10_22  -  V 0.1g - Hardware Rev. 0.6
  2015_12_18  -  V 0.2a - x^2 Test - (Debug_Level 10)
  2016_01_25  -  V 0.2b - Memory_Plus Test - (Debug_Level 12)

*/

#include <avr/wdt.h>

// https://github.com/MCUdude/MightyCore
#include <pins_arduino.h>  // ..\avr\variants\standard\pins_arduino.h
// https://github.com/Chris--A/BitBool
#include <BitBool.h>
// https://github.com/wizard97/Embedded_RingBuf_CPP
#include <RingBufCPP.h>

#include <int96.h>
// Original:  http://www.naughter.com/int96.html
// https://github.com/JensGrabner/snc98_Slash-Number-Calculator/tree/master/Software/Arduino/libraries/int96

#include <stdlib.h>         // for itoa(); ltoa();
#include <string.h>
#include <stdint.h>
#include <math.h>           // for sqrtf();
#include <inttypes.h>


// https://github.com/PaulStoffregen/TimerOne
#include <TimerOne.h>

#define Debug_Level  0 //  0 - not Debug
                       //  1 - Test intern 1ms - Task by 100 ms
                       //  2 - Test intern 1ms - Task by 1000 ms
                       //  3 - Test Switch "+" up / down (analog)
                       //  4 - Test Switchnumber down (digital)
                       //  5 - Monitor Switch_Code (Text) 128 - Functions
                       //  6 - Test Pendulum Time - Sinus 0.5 Hz
                       //  7 - get_Expo
                       //  8 - get mem_stack "=" Get_Mantisse();
                       //  9 - Display_Number();
                       // 11 - Error_Test;
                       // 12 - Memory_Plus Test;
                       // 13 - Protokoll Statepoint
                       // 15 - M+ Test
                       // 16 - Expand Test
                       // 17 - Display_Status Test
                       // 18 - out 1.00 .. 100.00 sqr-Test: out 1.00 .. 10.00
                       // 19 - 5 x sqrt(2)
                       // 20 - reduce test 
                       // 21 - "=" - Output -- mem_stack_calc()
                       // 22 - reduce test spezial

uint8_t mem_pointer        =  1;   //     mem_stack 0 .. 19
#define mem_stack_max         2    // 19  Variable in calculate

#define operation_test_max    4  //  0 .. 3  Stacktiefe
                                 //                     74HC4053  (100 Ohm)
#define Switch_down_start  1020  //  1020  ... 1020  100  %   ... 1020  <<--
#define Switch_up_b         624  //   750  ..   840   40  %   ...  624
#define Switch_down_b       492  //   570  ..   600   20  %   ...  492
#define Switch_up_start     294  //   360  ...  360    0  %   ...  360  <<--
                                 //                  -10  %   ...  294
#define Average               6  //     6
#define Max_Buffer           24  // Count of Switches

#define Beep_p A0      //  A0   Pin A7
#define Beep_m A4      //  A4   Pin A3
#define PWM    13      //     Sanguino  Pin 12 --> Pin 4 (PD4)
#define PWM_s  12      //
uint8_t PWM_Pin = PWM;

#define SDA    17      //     Pin 17
#define SCL    16      //     Pin 16

#define A_0    A7      //  A7   Pin A0
#define A_1    A6      //  A6   Pin A1
#define A_2    A5      //  A5   Pin A2

#define Out_A  A3      //  A3   Pin A4
#define Out_B  A2      //  A2   Pin A5
#define Out_C  A1      //  A1   Pin A6

#define On_Off_PIN    18
boolean Power_on = true;

#define Digit_Count   15

static const uint8_t index_display[Digit_Count] = {               //   Standard
//  0,  1,  2,  3, 14, 13,  4,  5,  6,  7, 23, 22, 21, 20, 19     //   old Project
//  0,  1,  3,  6,  7, 10, 11, 13, 14, 15, 19, 20, 21, 22, 23     //
   14, 15, 19, 20, 21, 22, 23, 13, 11,  7, 10,  3,  6,  0,  1     // Main_CPU
// 14, 15, 19, 20, 21, 22, 23, 13,  4,  7,  2,  3,  6,  0,  1     // new Main_CPU
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

#define expo_max_input  90
#define expo_min_input -90

#define expo_max_in_3   33
#define expo_min_in_3  -33

#define expo_max       102
#define expo_min       -99

#define _PI_        36
#define _e_         44

#define Min_0      160
#define M_xch_0    178
#define MR_0        77
#define FIX_0       93
#define M_plus_0   102
#define to_0       191
#define to_9       200
#define EE_0       201   // Beep
#define EE_9       210
#define p_p        211
#define input_0     48

#define int32_max_2  0x100000000ULL  // 4294967296 = 2^32
#define int32_max     2147302920     // = int32_2_max * int32_2_max - 1
#define int32_2_max        46339     // = sqrt(int32_max + 1)
#define tuning_fraction     1024     // 1024
#define int30_max     1018629247     // = int32_max / 2
#define int15_max          32767     // = 2^15 - 1

/*
 * rational number "numerator / denominator"
 */
struct AVRational_32{  //     0.3 ... 3 x 10^expo
   int8_t expo;        // <-- expo
  int32_t num;         // <-- numerator
  int32_t denom;       // <-- denominator
  uint8_t op;          // <-- operation
};

struct AVRational_64{  //     0.3 ... 3 x 10^expo
  int16_t expo;        // <-- expo
  int64_t num;         // <-- numerator
  int64_t denom;       // <-- denominator
};

struct AVRational_32_plus{  //     0.3 ... 3 x 10^expo
   int8_t expo;        // <-- expo
  int32_t num;         // <-- numerator
  int32_t denom;       // <-- denominator
  uint8_t op;          // <-- operation
  uint8_t op_priority; // <-- priority
};

struct Rational_32{    //     0.3 ... 3
  uint32_t num;        // <-- numerator
  uint32_t denom;      // <-- denominator
};

struct Rational_64{    //     0.3 ... 3
  uint64_t num;        // <-- numerator
  uint64_t denom;      // <-- denominator
};

AVRational_32       calc_32  = {0, int32_max, int32_max, 0};
AVRational_32       test_32      = {0, int32_max, int32_max, 0};
AVRational_32       temp_32      = {0, int32_max, int32_max, 0};
AVRational_32       temp_32_a    = {0, int32_max, int32_max, 0};
AVRational_32       temp_32_b    = {0, int32_max, int32_max, 0};
AVRational_32       temp_32_a1   = {0, int32_max, int32_max, 0};
AVRational_32       temp_32_b1   = {0, int32_max, int32_max, 0};
AVRational_64       temp_64      = {0, int32_max, int32_max};
AVRational_32_plus  temp_32_plus = {0, int32_max, int32_max, ' ', 0};

int8_t  temp_expo  = 0;          // <-- expo
int32_t temp_num   = int32_max;  // <-- numerator
int32_t temp_denom = int32_max;  // <-- denominator
uint8_t temp_op    = 0;          // <-- operation
uint8_t temp_op_   = 0;          // <-- operation

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

static const uint32_t expo_10[10] = {
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

static const uint64_t expo_10_[9] = {
  expo_10_10, expo_10_11, expo_10_12, expo_10_13, expo_10_14,
  expo_10_15, expo_10_16, expo_10_17, expo_10_18 };
                                              //  9214364837600034815 = 2^63 - 2^53 - 1
#define expo_test_10aaa       0x3097AE14ULL   //            815246868 = expo_test_0a / 2^31
#define expo_test_9a          0x685A0267ULL   //           1750729319 x 0,00000000019
#define expo_test_9b          0x9F4603ABULL   //           2672165803 x 0,00000000029
#define expo_test_9          0x10D1E0632ULL   //           4515038770 x 0,00000000049
#define expo_test_6a       0x1979F9962E8ULL   //        1750729319144 x 0,00000019
#define expo_test_6b       0x26E297E5398ULL   //        2672165802904 x 0,00000029
#define expo_test_6        0x41B3D4834F8ULL   //        4515038770424 x 0,00000049
#define expo_test_3a     0x638476F2A5A47ULL   //     1750729319144007 x 0,00019
#define expo_test_3b     0x97E52157689CAULL   //     2672165802904010 x 0,00029
#define expo_test_3     0x100A67620EE8D1ULL   //     4515038770424017 x 0,00049
#define expo_test_2a    0x3E32CA57A786C2ULL   //    17507293191440066 x 0,0019
#define expo_test_2b    0x5EEF34D6A161E5ULL   //    26721658029040101 x 0,0029
#define expo_test_2     0xA06809D495182AULL   //    45150387704240170 x 0,0049
#define expo_test_1a   0x26DFBE76C8B4395ULL   //   175072931914400661 x 0,019
#define expo_test_1b   0x3B55810624DD2F2ULL   //   267216580290401010 x 0,029
#define expo_test_1    0x64410624DD2F1AAULL   //   451503877042401706 x 0,049                                                  913113831648622805
#define expo_test_0a  0x184BD70A3D70A3D6ULL   //  1750729319144006614 x 0,19
#define expo_test_00  0x7FDFFFFFFFFFFFFFULL   //  9214364837600034815 x 1
#define expo_test_000 0xFFBFFFFFFFFFFFFEULL   // 18428729675200069630 x 2

// ---  sqrt(10)_Konstante  ---
#define sqrt_10_expo            0
#define sqrt_10_num    1499219281
#define sqrt_10_denom   474094764  // Fehler ..  7.03e-19
static const AVRational_32 sqrt_10_plus  = {0, sqrt_10_num, sqrt_10_denom, 0};
static const AVRational_32 sqrt_10_minus = {1, sqrt_10_denom, sqrt_10_num, 0};

// ---  Tau_Konstante (2_Pi) ---
#define Tau_expo                1
#define Tau_num        1135249722
#define Tau_denom      1806806049  // Fehler ..  1,34e-17
static const AVRational_32   Tau = {Tau_expo, Tau_num, Tau_denom, 0};

// ---  Pi_Konstante  ---
#define Pi_expo                 0
#define Pi_num         1892082870
#define Pi_denom        602268683  // Fehler ..  6,69e-18
static const AVRational_32   Pi  = {Pi_expo, Pi_num, Pi_denom, 0};

// ---  e_Konstante  ---
#define e_expo                  0
#define e_num           848456353
#define e_denom         312129649  // Fehler .. -6.03e-19

char    expo_temp_str[]    = "#00";
int8_t  expo_temp_8        =  1;

char    Expo_string_temp[] = "###" ;
 int16_t expo_temp_16           = 0;
 int16_t expo_temp_16_a         = 0;
 int16_t expo_temp_16_b         = 0;
 int16_t expo_temp_16_diff      = 0;
 int16_t expo_temp_16_diff_abs  = 0;
uint64_t num_temp_u64      = 1;
uint64_t num_temp_u64_a    = 1;
uint64_t num_temp_u64_b    = 1;
uint64_t denom_test_u64    = 1;
uint64_t denom_temp_u64    = 1;
uint64_t denom_temp_u64_a  = 1;
uint64_t denom_temp_u64_b  = 1;

uint16_t count_reduce = 0;

  uint64_t b0_num =       0;
  uint64_t b1_num =       0;
  uint64_t b2_num =       1;
  uint64_t b3_num =       1;

  uint64_t b0_denum =     1;
  uint64_t b1_denum =     1;
  uint64_t b2_denum =     0;
  uint64_t b3_denum =     0;

  uint64_t b_num_avg =    1;
  uint64_t b_denum_avg =  0;

  uint64_t a0_num =       0;
  uint64_t a1_num =       0;
  uint64_t a2_num =       1;
  uint64_t a3_num =       1;

  uint64_t a0_denum =     1;
  uint64_t a1_denum =     1;
  uint64_t a2_denum =     0;
  uint64_t a3_denum =     0;

  uint64_t a_num_avg =    1;
  uint64_t a_denum_avg =  0;

uint64_t x0_a_64 =      1;
uint64_t x0_a_64_test = 1;
uint64_t x0_a_64_corr = 1;
uint64_t x0_a_64_max  = 1;
uint32_t x0_a_32 =      1;

uint64_t x0_b_64 =      1;
uint64_t x0_b_64_test = 1;
uint64_t x0_b_64_corr = 1;
uint64_t x0_b_64_max  = 1;
uint32_t x0_b_32 =      1;

uint64_t x0_a_64_mul  = 1;
uint64_t x0_a_64_div  = 1;

boolean first_value = false;
boolean exact_value = false;
boolean NoOverflowSoFar = true;

  int8_t  k_  = 0;
uint64_t  x1_ = 0;
uint64_t  x0_ = 1;
uint64_t  p1_ = 0;
uint64_t  q1_ = 1;
uint64_t  p0_ = 1;
uint64_t  q0_ = 0;

 int64_t calc_temp_64_a     = 1;
uint64_t calc_temp_64_a_abs = 1;
 int64_t calc_temp_64_b     = 1;
uint64_t calc_temp_64_b_abs = 1;
 int64_t calc_temp_64_b1    = 1;
 int64_t calc_temp_64_c     = 1;
uint64_t calc_temp_64_c_abs = 1;
 int64_t calc_temp_64_c1    = 1;
 int64_t calc_temp_64_d     = 1;
uint64_t calc_temp_64_d_abs = 1;

uint64_t old_num_u64_a0  = 1;
uint64_t old_num_u64_b0  = 1;
uint64_t old_denum_u64_0 = 1;

uint64_t calc_temp_u64_0 = 1;
uint64_t calc_temp_u64_1 = 1;
uint64_t calc_temp_u64_2 = 1;
 int64_t gcd_temp_64     = 1;

uint32_t num_temp_u32    = 1;
uint32_t denom_temp_u32  = 1;
uint32_t mul_temp_u32    = 1;
uint32_t mul_temp_u32_a  = 1;
uint32_t mul_temp_u32_b  = 1;
uint32_t calc_temp_u32   = 1;
 int32_t mul_temp_32     = 1;
 int32_t gcd_temp_32     = 1;
 int32_t calc_temp_32_0  = 1;
 int32_t calc_temp_32_1  = 1;
 int32_t calc_temp_32_2  = 1;

 int16_t calc_temp_16_0  = 1;
 int16_t calc_temp_16_1  = 1;
 int16_t calc_temp_16_2  = 1;

  int8_t  calc_temp_8_0  = 1;
  int8_t  calc_temp_8_1  = 1;
  int8_t  calc_temp_8_2  = 1;

  int8_t  test_signum_8  = 0;

uint8_t display_digit      =  5;
uint8_t display_digit_temp =  5;

volatile uint8_t mem_stack_count    =  1;   // actual - mem_stack 1 .. 19
volatile uint8_t temp_operation     =  0;
volatile uint8_t mem_stack_test     =  0;
volatile uint8_t count              =  0;   // Display wait counter

AVRational_32 number_stack[mem_stack_max + 1] = {          // shunting Stack
  { 0, int32_max, int32_max, ' ' }
};

AVRational_32 mem_stack_input[mem_stack_max + 1] = {       // before calc
  { 0, int32_max, int32_max, ' ' }
};

AVRational_32_plus mem_stack_calc[mem_stack_max + 1] = {   //  after calc
  { 0, int32_max, int32_max, 0, 0 }
};

static const AVRational_32 to_xx[14] = {
  { -1, 1250000000,  473176473, 0 },   // 0  7  ..  1 Liter = (1 / 3,785411784) US-gal
  {  1,  473176473, 1250000000, 0 },   // 1  3  ..  1 gallon [US] = 3,785411784 Liter
  {  0,  565007021, 1245627260, 0 },   // 2  2  ..  1 lb = (1 / 2,2046226218488) kg
  {  0, 1245627260,  565007021, 0 },   // 3  8  ..  1 kg = 2,2046226218488 lbs
  {  0, 2147468400, 1334375000, 0 },   // 4  5  ..  1 Miles = 1,609344 km
  { -2, 2145264442,  844592300, 0 },   // 5  4  ..  to mm
  {  2,  844592300, 2145264442, 0 },   // 6  6  ..  to mil
  {  0, 1334375000, 2147468400, 0 },   // 7  0  ..  to Miles
  {  0, 1191813600, 2145264480, 0 },   // 8  1  ..  to °C  5 / 9
  {  0, 2145264480, 1191813600, 0 },   // 9  9  ..  to °F  9 / 5
  {  1,-2145264480,  670395150, 0 },   // 10  ..       -32
  {  1, 2145264480, 1206711270, 0 },   // 11  ..     +160 / 9
  {  2,  853380389, 1489429756, 0 },   // 12  ..     to deg
  { -2, 1489429756,  853380389, 0 }    // 13  ..     to rad
};

static const char mem_str_1[]     = "##########1111111111";
static const char mem_str_0[]     = "#1234567890123456789";

uint8_t mem_extra_pointer  =  0;   // mem_extra  MR 0 .. MR 9
uint8_t extra_test_13      =  0;
uint8_t mem_extra_test     =  0;   // mem_extra  MR 0 .. MR 9
uint8_t fix_extra_test     =  5;   // FIX_2  ..  FIX8
#define mem_extra_max        10    // mem_extra  MS 0 .. MS 9
uint8_t to_extra_test      =  0;   // mem_extra  to_0 .. to_9
boolean to_extra           = false;
boolean expo_exchange      = false;
boolean mem_exchange       = false;
boolean mem_save           = false;
boolean Mr_0_test          = false;
boolean test_index         = true;
boolean Test_to_Result     = true;

AVRational_32 mem_extra_stack[mem_extra_max + 2] = {
  {-90, int32_max, int32_max, 0 },     // MR 0
  {  0, 2147395599, 2147395599, 0 },   // MR 1
  {  0, 2147395598, 1073697799, 0 },   // MR 2
  {  0, 2147395599,  715798533, 0 },   // MR 3
  {  1,  858958238, 2147395595, 0 },   // MR 4
  {  1, 1073697799, 2147395598, 0 },   // MR 5
  {  1, 1288437357, 2147395595, 0 },   // MR 6
  {  1, 1503176913, 2147395590, 0 },   // MR 7
  {  1, 1717916476, 2147395595, 0 },   // MR 8
  {  1, 1932656031, 2147395590, 0 },   // MR 9
  {  Pi_expo, Pi_num, Pi_denom, 0 },   // const Pi
  {  Tau_expo, Tau_num, Tau_denom, 0 } // const Tau
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

static const uint16_t led_bright_plus[led_bright_max + 3] = {
  //                4     5    (6)    7     8     9     10
  0, 76,  86, 104, 132,  173,  232,  331,  444,  579,  742,  937,  1023 };
  //   10    18   28   41   59    99   113   135   163   195    86
  //      8    10    13   18    40   14    22    28    32  -109
  //         2     3    5   22   -26     8     6     4  -141

static const uint16_t led_bright[led_bright_max + 3] = {
  //               4     5    (6)    7     8     9     10
  0, 48,  61, 81, 108,  146,  199,  290,  397,  527,  685,  873,  1023 };
  //   13   20   27   38    53     91   107   130   158   188    150
  //      7    7   11    15    38    16    23    28    30     -38
  //        0    4     4    23    -22    7     5     2     68

#define Beep_patt_p 0x993264C993264C99ULL   // 16.8421 ms -- 1128.125 Hz -- 19x Peak
//      1001100100110010011001001100100110010011001001100100110010011001 -- binaer
#define Beep_patt_m 0x66CD9B366CD9B366ULL   // 16.8421 ms -- 1128.125 Hz -- 19x Peak
//      0110011011001101100110110011011001101100110110011011001101100110 -- binaer
#define max_Beep_count  65   //  65   Beep on
#define min_Beep_count -25   // -25   Beep off  = 90

uint8_t Countdown_OFF = 0;
#define Countdown_Off_3 220   // 156   --  Start_Off
#define Countdown_Off_2 160   // 135   --  Start
#define Countdown_Off_1 120   // 114
#define Countdown_Off_0  20   //  15

uint8_t index_Switch  = 255;      // counter Switch-digit
uint8_t index_LED = 0;            // counter LED-digit
uint8_t index_Display = 0;        // counter Display-digit
uint8_t index_Display_old = 0;    // counter Display-digit_old
volatile uint8_t index_a = 0;
volatile uint8_t index_b = 0;
volatile uint8_t index_c = 0;
volatile uint8_t index_i = 0;     // count _=_ output
volatile  int8_t index_j = 0;     // count _=_ output
uint8_t index_pendel_a = 0;       // 0 .. 189
uint8_t index_TIME = 255;         // counter Time
#define Time_LOW     263          // 263    t = (263 + 3/19) µs
#define Time_HIGH    264          // 264  1/t = 3800 Hz

boolean Display_rotate = false;
boolean Memory_xch = false;
boolean Beep_on = false;
boolean Beep_on_off = true;
boolean Beep_on_off_temp = true;
int8_t Beep_count = max_Beep_count;

boolean Constant_arithmetic = false;
boolean Found_constant = false;
boolean First_operation = false;
boolean Pendular_on = false;

uint16_t display_bright = led_bright_max;

static const uint8_t led_font[count_ascii] = {
    0,  64,  68,  76,  92, 124, 125, 127, 111, 103,  99,  97,  96,  64,   0,   0,     //  ¦                ¦
    0, 107,  34,   0, 109,  18, 125,   2,  57,  15,  92,  70,  12,  64, 128,  82,     //  ¦ !"#$%&'()*+,-./¦
   63,   6,  91,  79, 102, 109, 124,   7, 127, 103,   4,  20,  88,  72,  76,  83,     //  ¦0123456789:;<=>?¦
  123, 119, 127,  57,  15, 121, 113,  61, 118,  48,  30, 122,  56,  85,  55,  99,     //  ¦@ABCDEFGHIJKLMNO¦
  115, 103,  49,  45,   7,  28,  34,  60,  73, 110,  27,  57, 100,  15,  35,   8,     //  ¦PQRSTUVWXYZ[\]^_¦
   32,  95, 124,  88,  94, 123,  43, 111, 116,   4,  14, 120,  24,  21,  84,  92,     //  ¦`abcdefghijklmno¦
   83,  53,  80, 108,  70,  20,  42, 106,  65, 102,  82,  24,  20,   3,   1,  54};    //  ¦pqrstuvwxyz{|}~ ¦

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
boolean Digit_Test = false;

boolean Rad_in_out = false;

char display_string[ ascii_count ]              = " -1.2345678#- 1 2 # # =.       " ;
static const char string_end[ ascii_count ]     = " ~-_Good  lUc k _ - ~.         " ;
static const char string_start[ ascii_count ]   = "  _########## # # # # #        " ;
static const char display_string_888[ ascii_count ] = " 8.8.8.8.8.8.8.8.8.8.8.8.8.8.8." ;
#define Plus_Minus           1
#define Mantisse_0           2
#define Mantisse_1           3
#define Plus_Minus_Expo__   10
#define Plus_Minus_Expo_    11
#define Plus_Minus_Expo     12
#define MR_point            13
#define Expo_1              14
#define MS_point            15
#define Expo_0              16     // 16
#define Expo_point          17     // 17
#define Operation           18     // 18
#define Operation_point     19     // 19
#define Memory_1            20     // 20
#define Rad_point           21     // 21
#define Memory_0            22     // 22

char char_test;               // 0.. 127

uint8_t     index_mem = 255;  // 1..9  ... for MC (Memory's Clear)

boolean     time_10ms = false;
uint8_t    index_10ms = 255;  // 0..13
uint32_t   count_10ms = 0;
boolean    time_100ms = false;
uint8_t   index_100ms = 255;  // 0..7
uint32_t  count_100ms = 0;
boolean   time_1000ms = false;
uint32_t count_1000ms = 0;
boolean   time_7500ms = false;
uint8_t  index_7500ms = 255;  // 0..59

uint8_t    index_5min = 255;  // 0..39
#define    time_5min    39  // 39  5_min
#define    pendel_3min  23  // 23  3_min

boolean Init_expo = true;
boolean Display_new = true;
boolean Display_change = false;

uint8_t Cursor_pos = 2;       //
uint8_t Point_pos = 0;        //
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

uint8_t Switch_Code = 0;
uint8_t Switch_Test = 0;
uint8_t Start_mem = 0;
uint8_t Display_Status_new = 0;    // Switch up / down
uint8_t Display_Status_old = 0;
// https://github.com/Chris--A
// https://github.com/Chris--A/BitBool#reference-a-single-bit-of-another-object-or-bitbool---
auto bit_0 = toBitRef( Display_Status_new, 0 );  // ( "0" ) Create a bit reference to first bit.
auto bit_1 = toBitRef( Display_Status_new, 1 );  // ( "." )
auto bit_2 = toBitRef( Display_Status_new, 2 );  // ( "+/-" )
auto bit_3 = toBitRef( Display_Status_new, 3 );  // ( "EE" )
auto bit_4 = toBitRef( Display_Status_new, 4 );  // ( "FN" )
auto bit_5 = toBitRef( Display_Status_new, 5 );  // ( "=" )
uint8_t Display_Memory = 0;    // 0 .. 12
static const char    Display_Memory_1[] = "  mmE){F mFm=O_X";
static const char    Display_Memory_0[] = "1 r__(}n=+Fc_V,_";

boolean max_input = false;
boolean Mantisse_change = false;
boolean Expo_change = false;

#define First_Display            0
#define Start_Mode               1
#define Input_Mantisse           2   //     Input Number
#define Input_Expo               3   //     Input Number
#define Input_dms                4   //     Input Number
#define Input_Fraction           5   //     Input Number
#define Input_Memory             6   //     Input Number
#define Input_Operation_0        7   //    Input Operation  no Input     // op_priority
#define Input_Operation_1        8   //    Input Operation  "+"  "-"     // op_priority
#define Input_Operation_2        9   //    Input Operation  "*"  "/"     // op_priority
#define Input_Operation_3       10   //    Input Operation  "//"  "/p/"  // op_priority
#define Input_Operation_4       11   //    Input Operation  "sqrt(x,y)" "x^y" "HM" "GM""AM"//
#define Input_Operation_5       12   //    Input Operation  "("  ")"     // op_priority
#define Display_Input_Error     13   //   Error Number
#define Display_Error           14   //   Error Number
#define Display_Result          15   //  Display Number
#define Display_M_Plus          16   //  Display Number
#define Display_Fraction_a_b    17   //  Display Number
#define Display_Fraction_a_b_c  18   //  Display Number
#define Display_hms             19   //  Display Number
#define Off_Status              20   //  Off_Status

uint8_t Start_input = First_Display;
char Pointer_memory = '_';

int16_t display_expo = 0;
int64_t display_big = 1;
int32_t display_number = 1;
#define digit_count_max 8
int16_t display_expo_mod = 0;

void reboot() {
	MCUSR = 0;
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}

void Print_Operation( uint8_t Switch_up) {
    switch (Switch_up) {

      case 30:                 //    <--
        Serial.print("(<<-)");
        break;

      case 32:                 //    FIX_hms
        Serial.print("(FIX_hms)");
        break;

      case 33:                 //    _1/x_
        Serial.print("(1/x)");
        break;

      case 34:                 //    lb
        Serial.print("(lb(x))");
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

      case 103:                //    M_plus(1)
      case 104:                //    M_plus(2)
      case 105:                //    M_plus(3)
      case 106:                //    M_plus(4)
      case 107:                //    M_plus(5)
      case 108:                //    M_plus(6)
      case 109:                //    M_plus(7)
      case 110:                //    M_plus(8)
      case 111:                //    M_plus(9)
        extra_test_13 = Switch_Code - M_plus_0;
        Serial.print("(M_plus(");
        Serial.print(extra_test_13);
        Serial.print("))");
        break;

      case 112:                //    ln(x)
        Serial.print("(ln(x))");
        break;

      case 113:                //    e^x
        Serial.print("(e^x)");
        break;

      case 114:                //    2^x
        Serial.print("(2^x)");
        break;

      case 115:                //    log(x)
        Serial.print("(log(x))");
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

      case 119:                //    x!
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
        Serial.print("(->>)");
        break;

      case 128:                 //    AM
        Serial.print("(AM)");
        break;

      case 129:                 //    GM
        Serial.print("(GM)");
        break;

      case 148:                 //    <<-->>
        Serial.print("(<<-->>)");
        break;

      case 149:                 //    __/
        Serial.print("(__/)");
        break;

      case 150:                 //    ° ' ''
        Serial.print("(o_-_=)");
        break;

      case 151:                 //    _AGM_
        Serial.print("AGM");
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

      case 171:                //    Frac
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

      case Input_Operation_3:       //    Input Operation  "//"  "/p/"
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

void Memory_to_Input_Operation() {
  mem_pointer = mem_stack_count;
  if ( Start_input == Display_Result ) {
    Result_to_Start_Mode();
  }
  if ( Start_input == Display_M_Plus ) {
    Result_to_Start_Mode();
  }
  if ( Start_input < Input_Operation_0 ) {      // Input Number
    left_right_mem_extra( 0, mem_extra_test );
    Start_input = Input_Memory;
  }
  if ( Start_input == Input_Memory ) {
    if ( mem_pointer > 0 ) {
      left_right_mem_extra( mem_pointer, mem_extra_test );
      mem_pointer = 0;
      mem_stack_input[ mem_pointer ].op = temp_op_;
      if ( max_input == false ) {
        mem_stack_input[ mem_pointer ].op += '_';
      }
      else {
        mem_stack_input[ mem_pointer ].op += 'x';
      }
      Display_Number();
      Mantisse_change = false;
      Expo_change = false;
      Init_expo = false;
    }
  }
  display_string[Memory_1] = Display_Memory_1[3];  // m
  display_string[Memory_0] = '0' + mem_extra_test;
  mem_save = true;
}

void Result_to_Start_Mode() {
  Constant_arithmetic = false;
  First_operation = false;
  mem_stack_count = 1;
  mem_pointer = mem_stack_count;
  Clear_String();
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

int32_t gcd_iter_32(int32_t u_0, int32_t v_0) {
  int32_t t_0;
  if ( u_0 < 0 ) {
    u_0 *= -1;
  }
  // return abs(expo_10[0]);  // no GCD Test

  while (v_0 != 0) {
    t_0 = u_0;
    u_0 = v_0;
    v_0 = t_0 % v_0;
  }
  return abs(u_0);      // return u < 0 ? -u : u; /* abs(u) */
}

void Error_String(uint8_t a) {
  Clear_String();
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
  Display_new = true;
  Start_input = Display_Error;
}

void Clear_String() {   // String loeschen -- Eingabe Mantisse
  strcpy( display_string, string_start );      // normal
  // strcpy( display_string, display_string_888 );    // On_Off_Test
  display_string[Memory_1] = mem_str_1[mem_pointer];
  display_string[Memory_0] = mem_str_0[mem_pointer];

  if ( Rad_in_out == true ) {
    display_string[Rad_point] = '.';
  }

  Cursor_pos = 2;
  Point_pos = 0;
  Number_count = 0;
  Zero_count = 0;
  Zero_after_Point = 0;
  Zero_index = 0;
  Zero_index_a = 0;
  Init_expo = true;

  Mantisse_change = false;
  Expo_change = false;
  Start_input = Input_Mantisse;

  mem_stack_input[mem_pointer].num = int32_max;
  mem_stack_input[mem_pointer].denom = int32_max;
  mem_stack_input[mem_pointer].expo = 0;
  mem_stack_input[mem_pointer].op = ' ';  // temp_op_
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
  }
  if ( mem_stack_input[mem_pointer].expo == expo_max ) {
    num_temp_u32   = abs(mem_stack_input[mem_pointer].num) / 9;
    denom_temp_u32 = abs(mem_stack_input[mem_pointer].denom) / 10;
    if ( num_temp_u32 > denom_temp_u32 ) {
      Error_String('^');
    }
  }
  if ( mem_stack_input[mem_pointer].expo == expo_min ) {
    if ( abs(mem_stack_input[mem_pointer].num) < abs(mem_stack_input[mem_pointer].denom) ) {
      Error_String('U');
    }
  }
  if ( mem_stack_input[mem_pointer].expo < expo_min ) {
    Error_String('U');
  }
}

void Get_Number( uint8_t deep_test ) {
  if ( Number_count != Zero_count ) {
    if ( Mantisse_change == true ) {
      Get_Mantisse();
    }
    else {
      if ( Expo_change == true ) {
        Get_Expo_change();
      }
    }
  }
  else {
    mem_stack_input[ mem_pointer ].num = int32_max;
    mem_stack_input[ mem_pointer ].denom = int32_max;
    mem_stack_input[ mem_pointer ].expo = expo_min_input;
    mem_stack_input[ mem_pointer ].op = temp_op;
    if (display_string[1] == '-') {
      mem_stack_input[mem_pointer].num *= -1;
    }
  }
  if ( Start_input != Display_Error ) {
    copy_input_left_right( 0, mem_pointer );
    mem_extra_left_right( 0, 0 );
    Start_input = Input_Operation_0;
    Start_input += deep_test;
  }
}

void Get_Mantisse() {          // " -1.2345678#- 1 5# 1 9."
  if ( Debug_Level == 8 ) {
    Serial.print("'");
    Serial.print(display_string);
    Serial.println("'");
  }
  Zero_after_Point = 0;
  if (Point_pos == 2) {
    Zero_index = 3;
    while (display_string[Zero_index] == '0') {
      ++Zero_after_Point;
      ++Zero_index;
    }
    First_char = display_string[Zero_index];
    Zero_index_a = 0;
    char_test = display_string[Zero_index];
    while ( char_test != ' ' ) {
      Temp_char[Zero_index_a] =  display_string[Zero_index];
      ++Zero_index;
      char_test = display_string[Zero_index];
      if ( char_test == '_' ) {
        char_test = ' ';
      }
      if ( char_test == '-' ) {
        char_test = ' ';
      }
      ++Zero_index_a;
      Temp_char[Zero_index_a] =  ' ';
    }
  }
  else {
    First_char = display_string[2];
    Zero_index_a = 2;
    while ( Zero_index_a <= Number_count + 1 ) {
      if ( Point_pos == 0 ) {
        Temp_char[Zero_index_a - 2] = display_string[Zero_index_a];
      }
      else {
        if ( Zero_index_a >= Point_pos ) {
          Temp_char[Zero_index_a - 2] = display_string[Zero_index_a + 1];
        }
        else {
          Temp_char[Zero_index_a - 2] = display_string[Zero_index_a];
        }
      }
      Temp_char[Zero_index_a - 1] = ' ';
      ++Zero_index_a;
    }
  }
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

  num_temp_u32 = atol(Temp_char);
  num_temp_u32 *= expo_10[Zero_after_Point];
  denom_temp_u32 = expo_10[Number_count];

  if ( First_char < '3' ) {
    num_temp_u32 *= expo_10[1];
    --mem_stack_input[mem_pointer].expo;
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
}

void Reduce_Number() {	
	
	Rational_32 a;
	Rational_32 b;
	Rational_64 a_64;
	Rational_64 b_64;
	int8_t   test_cmp_64 = 0;
	int8_t   test_cmp_32 = 0;
  int8_t   test_temp_8 = 0;

  if ( num_temp_u64 > denom_temp_u64 ) {
    test_temp_8     =  1;
  }
  else {
    test_temp_8     = -1;          // num_temp_u64 <--> denom_temp_u64
    calc_temp_u64_0 = denom_temp_u64;
    denom_temp_u64  = num_temp_u64;
    num_temp_u64    = calc_temp_u64_0;
  }

  count_reduce = 0;
  exact_value  = false;
  first_value  = false;

 /**
  *  https://hg.python.org/cpython/file/3.5/Lib/fractions.py#l252
  *  https://ffmpeg.org/doxygen/2.8/rational_8c_source.html#l00035
  *  http://link.springer.com/article/10.1007%2Fs00607-008-0013-8
  *  https://math.boku.ac.at/udt/vol05/no2/3zhabitsk10-2.pdf  Page_5
  */

 /**
  *  Thill M. - A more precise rounding algorithm for natural numbers.
  *  .. Computing Jul 2008, Volume 82, Issue 2, pp 189–198
  *  http://link.springer.com/article/10.1007/s00607-008-0006-7
  *  =====
  *  Thill M. - A more precise rounding algorithm for natural numbers.
  *  .. Computing Sep 2008, Volume 82, Issue 4, pp 261–262
  *  http://link.springer.com/article/10.1007/s00607-008-0013-8
  *  http://link.springer.com/content/pdf/10.1007/s00607-008-0013-8.pdf
  */

  num_temp_u64_a   = num_temp_u64;
  denom_temp_u64_a = denom_temp_u64;
  num_temp_u64_b   = num_temp_u64;
  denom_temp_u64_b = denom_temp_u64;

  a1_num   = 1;                     // p1
  a2_num   = num_temp_u64_a;
  if ( denom_temp_u64_a > 0 ) {
    a2_num  /= denom_temp_u64_a;    // p2
  }
  a1_denum = 0;                     // q1
  a2_denum = 1;                     // q2

  b1_num   = 1;                     // p1
  b2_num   = num_temp_u64_b;
  --b2_num;
  if ( denom_temp_u64_b > 0 ) {
    b2_num  /= denom_temp_u64_b;    // p2
  }
  ++b2_num;
  b1_denum = 0;                     // q1
  b2_denum = 1;                     // q2

  old_num_u64_a0    = num_temp_u64_a;
  num_temp_u64_a    = denom_temp_u64_a;
  denom_temp_u64_a  = old_num_u64_a0;
  denom_temp_u64_a -= a2_num * num_temp_u64_a;

  x0_a_64 = num_temp_u64_a;
  if ( denom_temp_u64_a > 0 ) {
    x0_a_64 /= denom_temp_u64_a;
  }
  else {
    x0_a_64 = 1;
    first_value = true;
    exact_value = true;    //   denom_temp_u64_a == 0
    old_num_u64_a0 = 0;    // break out
  }

  old_num_u64_b0    = num_temp_u64_b;
  num_temp_u64_b    = denom_temp_u64_b;
  denom_temp_u64_b  = b2_num * num_temp_u64_b;
  denom_temp_u64_b -= old_num_u64_b0;
    
  x0_b_64 = num_temp_u64_b;
  --x0_b_64;
  if ( denom_temp_u64_b > 0 ) {
    x0_b_64 /= denom_temp_u64_b;    // p2
  }
  else {
    x0_b_64 = 0;
  }
  ++x0_b_64;

  a3_num    = a1_num;
  a3_num   += a2_num * x0_a_64;
  a3_denum  = a1_denum;
  a3_denum += a2_denum * x0_a_64;

  b3_num    = b2_num * x0_b_64;
  b3_num   -= b1_num;
  b3_denum  = b2_denum * x0_b_64;
  b3_denum -= b1_denum;

  count_reduce += 1;

  if ( Debug_Level == 20 ) {
    x0_a_32 = x0_a_64;
    Serial.print("__x0_a_32_");
    Serial.print(count_reduce);
    Serial.print(" = ");
    Serial.println(x0_a_32);
    x0_a_32 = a3_num;
    Serial.print(x0_a_32);
    Serial.print(" / ");
    x0_a_32 = a3_denum;
    Serial.println(x0_a_32);
  }

  if ( a3_num > int32_max ) {
    x0_a_64_mul  = expo_test_000;
    x0_a_64_mul /= a3_num;
    a3_num      *= x0_a_64_mul;
    a3_denum    *= x0_a_64_mul;
    x0_a_64_div  = a3_num;
    x0_a_64_div /= int32_max;
    a3_num      += x0_a_64_div / 2;
    a3_num      /= x0_a_64_div;
    a3_denum    += x0_a_64_div / 2;
    a3_denum    /= x0_a_64_div;
    --a3_num;
    --a3_denum;

    a.num   = a3_num;
    a.denom = a3_denum;
    b.num   = a2_num;
    b.denom = a2_denum;

    if ( Debug_Level == 15 ) {
      x0_a_32 = a3_num;
      Serial.print(x0_a_32);
      Serial.print(" / ");
      x0_a_32 = a3_denum;
      Serial.println(x0_a_32);
    }
    old_num_u64_a0 = 0;    // break out
  }

  if ( Debug_Level == 20 ) {
    x0_b_32 = x0_b_64;
    Serial.print("__x0_b_32_");
    Serial.print(count_reduce);
    Serial.print(" = ");
    Serial.println(x0_b_32);
    x0_a_32 = b3_num;
    Serial.print(x0_a_32);
    Serial.print(" / ");
    x0_a_32 = b3_denum;
    Serial.println(x0_a_32);
  }

  if ( denom_temp_u64 == 0 ) {
    a.num   = 0;
    a.denom = int32_max;
  }
  else {
    while ( old_num_u64_a0 > 0 ) {
    	    	
      ++count_reduce;

    	if ( Debug_Level == 22 ) {
        Serial.print("count_reduce 0 = ");
        Serial.println(count_reduce);
    	}

      a0_num = a1_num;
      a1_num = a2_num;
      a2_num = a3_num;
      a0_denum = a1_denum;
      a1_denum = a2_denum;
      a2_denum = a3_denum;

      b0_num = b1_num;
      b1_num = b2_num;
      b2_num = b3_num;
      b0_denum = b1_denum;
      b1_denum = b2_denum;
      b2_denum = b3_denum;

      old_num_u64_a0    = denom_temp_u64_a;
      denom_temp_u64_a  = num_temp_u64_a;
      denom_temp_u64_a -= x0_a_64 * old_num_u64_a0;
      num_temp_u64_a    = old_num_u64_a0;

      old_num_u64_b0    = denom_temp_u64_b;
      denom_temp_u64_b  = x0_b_64 * old_num_u64_b0;
      denom_temp_u64_b -= num_temp_u64_b;
      num_temp_u64_b    = old_num_u64_b0;

      x0_a_64_test = x0_a_64;
      x0_a_64 = num_temp_u64_a;
      if ( denom_temp_u64_a > 0 ) {
        x0_a_64 /= denom_temp_u64_a;
      }
      else {
        x0_a_64 = 1;
        exact_value = true;    //   denom_temp_u64_a == 0
        old_num_u64_a0 = 0;    // break out
      }

      x0_b_64_test = x0_b_64;
      x0_b_64 = num_temp_u64_b;
      --x0_b_64;
      if ( denom_temp_u64_b > 0 ) {
        x0_b_64 /= denom_temp_u64_b;    // p2
      }
      else {
        x0_b_64 = 0;
        old_num_u64_a0 = 0;    // break out
      }
      ++x0_b_64;

      a3_num    = a1_num;
      a3_num   += a2_num * x0_a_64;
      a3_denum  = a1_denum;
      a3_denum += a2_denum * x0_a_64;

      b3_num    = b2_num * x0_b_64;
      b3_num   -= b1_num;
      b3_denum  = b2_denum * x0_b_64;
      b3_denum -= b1_denum;

      if ( Debug_Level == 20 ) {
        x0_a_32 = x0_a_64;
        Serial.print("__x0_a_32_");
        Serial.print(count_reduce);
        Serial.print(" = ");
        Serial.println(x0_a_32);
        x0_a_32 = a2_num;
        Serial.print(x0_a_32);
        Serial.print(" / ");
        x0_a_32 = a2_denum;
        Serial.println(x0_a_32);
      }
      if ( Debug_Level == 20 ) {
        x0_b_32 = x0_b_64;
        Serial.print("__x0_b_32_");
        Serial.print(count_reduce);
        Serial.print(" = ");
        Serial.println(x0_b_32);
        x0_b_32 = b2_num;
        Serial.print(x0_b_32);
        Serial.print(" / ");
        x0_b_32 = b2_denum;
        Serial.println(x0_b_32);
      }

      if ( denom_temp_u64_a == 0 ) {
        a.num   = a2_num;
        a.denom = a2_denum;
        b.num   = a1_num;
        b.denom = a1_denum;
      }

      if ( denom_temp_u64_b == 0 ) {
        a.num   = b2_num;
        a.denom = b2_denum;
        b.num   = b1_num;
        b.denom = b1_denum;
      }

      if ( b3_num >= int32_max ) {

        x0_b_64 = int32_max;
        x0_b_64 -= b1_num;
        if ( b2_num > 0 ) {
          x0_b_64 /= b2_num;
          b3_num   = b2_num;
          b3_denum = b2_denum;
        }
        else {
          if ( Debug_Level == 20 ) {
            Serial.println("tuning_fraction");
          }
          x0_b_64  = tuning_fraction;
          b2_num   = 0;
          b2_denum = 0;
        }

        b3_num    = b2_num * x0_b_64;
        b3_num   -= b1_num;
        b3_denum  = b2_denum * x0_b_64;
        b3_denum -= b1_denum;

        if ( denom_temp_u64_b == 0 ) {
          exact_value = true;    //   denom_temp_u64_b == 0
        }

        if ( x0_b_64 == 0 ) {
          if ( Debug_Level == 20 ) {
            Serial.print("-> x0_b_64 == 0 <- b2_num = ");
            x0_b_32 = b2_num;
            Serial.println(x0_b_32);
          }
          x0_b_64_corr  = int32_max;
          x0_b_64_corr *= x0_b_64_test;
          x0_b_64_corr /= b2_num;
          if ( Debug_Level == 20 ) {
            Serial.print("x0_b_64_corr = ");
            x0_b_32 = x0_b_64_corr;
            Serial.println(x0_b_32);
            Serial.print("b1_num = ");
            x0_b_32 = b1_num;
            Serial.println(x0_b_32);
            Serial.print("count_reduce = ");
            Serial.println(count_reduce);
          }

          if ( x0_b_64_corr > 8 ) {
            if  ( count_reduce > 2 ) {
              b3_num    = b1_num * x0_b_64_corr;
              b3_num   -= b0_num;
              b3_denum  = b1_denum * x0_b_64_corr;
              b3_denum -= b0_denum;
            }
            else {
              b3_num   = b1_num;
              b3_denum = b1_denum;
            }
          }
          else {
            b3_num   = b2_num;
            b3_denum = b2_denum;
          }

          a.num   = b3_num;
          a.denom = b3_denum;
          b.num   = b2_num;
          b.denom = b2_denum;
        }
        else {
          b_num_avg    = b1_num * b2_denum;
          b_num_avg   += b1_denum * b2_num;
          b_denum_avg  = b1_denum * b2_denum;
          b_denum_avg *= 2;
          b_num_avg   *= b3_denum;
          b_denum_avg *= b3_num;

          if ( b_num_avg >= b_num_avg ) {
            if ( Debug_Level == 20 ) {
              Serial.println("-> x0_b_64_max > int32_max <-");
              Serial.print("x0_b_64 = ");
              x0_b_32 = x0_b_64;
              Serial.println(x0_b_32);
            }

            a.num = b1_num;
            a.denom = b1_denum;
            b.num = b0_num;
            b.denom = b0_denum;

            if ( b2_num < int32_max ) {
              if ( b2_num > b1_num ) {
                a.num   = b2_num;
                a.denom = b2_denum;
                b.num   = b1_num;
                b.denom = b1_denum;
              }
            }
          }
          else {
            if ( Debug_Level == 20 ) {
              Serial.println("-> x0_b_64_max <= int32_max <-");
              Serial.print("x0_b_64 = ");
              x0_b_32 = x0_b_64;
              Serial.println(x0_b_32);
            }
            a.num     = b2_num;
            a.denom   = b2_denum;
            b.num   = b1_num;
            b.denom = b1_denum;
          }
        }

        if ( Debug_Level == 20 ) {
          // x0_b_32 = x0_b_64;
          // Serial.print("__--x0_b_32 = ");
          // Serial.println(x0_b_32);
          x0_b_32 = b1_num;
          Serial.print(x0_b_32);
          Serial.print(" b1 / b1 ");
          x0_b_32 = b1_denum;
          Serial.println(x0_b_32);
          x0_b_32 = b2_num;
          Serial.print(x0_b_32);
          Serial.print(" b2 / b2 ");
          x0_b_32 = b2_denum;
          Serial.println(x0_b_32);
        }

        old_num_u64_a0 = 0;
      }

      if ( a3_num >= int32_max ) {

        x0_a_64 = int32_max;
        x0_a_64 -= a1_num;
        if ( a2_num > 0 ) {
          x0_a_64 /= a2_num;
        }
        else {
          if ( Debug_Level == 20 ) {
            Serial.println("tuning_fraction");
          }
          x0_a_64  = tuning_fraction;
          a2_num   = 0;
          a2_denum = 0;
        }

        a3_num    = a1_num;
        a3_num   += a2_num * x0_a_64;
        a3_denum  = a1_denum;
        a3_denum += a2_denum * x0_a_64;

        if ( denom_temp_u64_a == 0 ) {
          exact_value = true;    //   denom_temp_u64_a == 0
        }

        if ( x0_a_64 == 0 ) {
          if ( Debug_Level == 20 ) {
            Serial.print("-> x0_a_64 == 0 <- a2_num = ");
            x0_a_32 = a2_num;
            Serial.println(x0_a_32);
          }
          x0_a_64_corr  = int32_max;
          x0_a_64_corr *= x0_a_64_test;
          x0_a_64_corr /= a2_num;
          --x0_a_64_corr;
          if ( Debug_Level == 20 ) {
            Serial.print("x0_a_64_corr = ");
            x0_a_32 = x0_a_64_corr;
            Serial.println(x0_a_32);
            Serial.print("a1_num = ");
            x0_a_32 = a1_num;
            Serial.println(x0_a_32);
            Serial.print("count_reduce = ");
            Serial.println(count_reduce);
          }

          if ( x0_a_64_corr > 8 ) {
            if ( count_reduce > 2 ) {
              a3_num    = a1_num * x0_a_64_corr;
              a3_num   += a0_num;
              a3_denum  = a1_denum * x0_a_64_corr;
              a3_denum += a0_denum;
            }
            else {
              a3_num   = a1_num;
              a3_denum = a1_denum;
            }
          }
          else {
            a3_num = a2_num;
            a3_denum = a2_denum;
          }

          a.num   = a3_num;
          a.denom = a3_denum;
          b.num   = a2_num;
          b.denom = a2_denum;
        }
        else {
          a_num_avg    = a1_num * a2_denum;
          a_num_avg   += a1_denum * a2_num;
          a_denum_avg  = a1_denum * a2_denum;
          a_denum_avg *= 2;
          a_num_avg   *= a3_denum;
          a_denum_avg *= a3_num;

          if ( a_num_avg >= a_denum_avg ) {
            if ( Debug_Level == 20 ) {
              Serial.println("-> x0_a_64_max > int32_max <-");
              Serial.print(" x0_a_64 = ");
              x0_a_32 =  x0_a_64;
              Serial.println(x0_a_32);
            }

            a.num     = a1_num;
            a.denom   = a1_denum;
            b.num   = a0_num;
            b.denom = a0_denum;

            if ( a2_num < int32_max ) {
              if ( a2_num > a1_num ) {
                a.num   = a2_num;
                a.denom = a2_denum;
                b.num   = a1_num;
                b.denom = a1_denum;
              }
            }
          }
          else {
            if ( Debug_Level == 20 ) {
              Serial.println("-> x0_a_64_max <= int32_max <-");
              Serial.print("x0_a_64_max = ");
              x0_a_32 = x0_a_64_max;
              Serial.println(x0_a_32);
            }
            if ( old_num_u64_a0 == 0 ) {
              b.num   = a.num;
              b.denom = a.denom;
            }
            else {
              b.num   = a1_num;
              b.denom = a1_denum;
            }
            a.num   = a2_num;
            a.denom = a2_denum;            
          }
        }

        if ( Debug_Level == 20 ) {
          // x0_b_32 = x0_b_64;
          // Serial.print("__--x0_b_32 = ");
          // Serial.println(x0_b_32);
          x0_b_32 = a1_num;
          Serial.print(x0_b_32);
          Serial.print(" a1 / a1 ");
          x0_b_32 = a1_denum;
          Serial.println(x0_b_32);
          x0_b_32 = a2_num;
          Serial.print(x0_b_32);
          Serial.print(" a2 / a2 ");
          x0_b_32 = a2_denum;
          Serial.println(x0_b_32);
        }

        old_num_u64_a0 = 0;
      }
    }
  }

  if ( first_value == true ) {
  	first_value = false;
    a.num   = b2_num;
    a.denom = b2_denum;
    b.num   = b1_num;
    b.denom = b1_denum;
  }

  if ( Debug_Level == 22 ) {
    a_64.num    = expo_10_4;
    a_64.num   *= a.num;
    a_64.denom  = expo_10_4;
    a_64.denom *= a.denom;
    b_64.num    = expo_10_4;
    b_64.num   *= b.num;
    b_64.denom  = expo_10_4;
    b_64.denom *= b.denom;
    test_cmp_64 = compare(b_64, a_64);
  }
    
  test_cmp_32 = compare(b, a);
  
  if ( Debug_Level == 22 ) {
    Serial.print(b.num);
    Serial.print("  _  /  _  ");
    Serial.print(b.denom);
    if ( test_cmp_32 == 0 ) {
    	Serial.print("   =");
    }
    if ( test_cmp_32 < 0 ) {
    	Serial.print("   <");
    }
    if ( test_cmp_32 > 0 ) {
    	Serial.print("   >");
    }
    if ( test_cmp_64 == 0 ) {
    	Serial.println("   =");
    }
    if ( test_cmp_64 < 0 ) {
    	Serial.println("   <");
    }
    if ( test_cmp_64 > 0 ) {
    	Serial.println("   >");
    }
    Serial.print(a.num);
    Serial.print(" a_b / a_b ");
    Serial.println(a.denom);
  }

  if ( test_cmp_32 > 0 ) {
    a = compare_extra(a, b);
  }

  if ( test_cmp_32 < 0 ) {
    a = compare_extra(b, a);
  }

  num_temp_u32   = a.num;
  denom_temp_u32 = a.denom;

  if ( test_temp_8 < 0 ) {
  	num_temp_u32   = a.denom;  // Tausch
    denom_temp_u32 = a.num;    // Tausch
  }

  Expand_Number();
}

void Expand_Number() {
  gcd_temp_32 = gcd_iter_32(num_temp_u32, denom_temp_u32);
  num_temp_u32 /= gcd_temp_32;
  denom_temp_u32 /= gcd_temp_32;
  if ( num_temp_u32 > 0 ) {
    mul_temp_u32  = int32_max;
    if ( num_temp_u32 > denom_temp_u32 ) {
      mul_temp_u32 /= num_temp_u32;
    }
    else {
      mul_temp_u32 /= denom_temp_u32;
    }
    num_temp_u32 *= mul_temp_u32;
    denom_temp_u32 *= mul_temp_u32;
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
  Get_Mantisse();
  mul_temp_u32 = abs(calc_temp_32_0 / calc_temp_32_1);
  mem_stack_input[ mem_pointer ].num = calc_temp_32_0;
  mem_stack_input[ mem_pointer ].denom = calc_temp_32_1;
  if ( mul_temp_u32 > 2 ) {
    --mem_stack_input[ mem_pointer ].expo;
  }
}

void Display_Number() {
/*
 *   Round "half towards zero"
 *   https://en.wikipedia.org/wiki/Rounding#Round_half_towards_zero
 *   23.5     -->  23
 *  -23.5     --> -23
 *   23.50001 -->  24
 *  -23.50001 --> -24
 */

  uint8_t temp_char = display_string[Operation];

  if ( Debug_Level == 9 ) {
    Serial.print("'");
    Serial.print(display_string);
    Serial.println("'");
  }

  if ( mem_stack_input[mem_pointer].denom < 0 ) {
    mem_stack_input[mem_pointer].num *= -1;
    mem_stack_input[mem_pointer].denom *= -1;
  }
  display_expo = mem_stack_input[mem_pointer].expo;
  if ( abs(mem_stack_input[mem_pointer].num) != 0 ) {
    display_big = abs(mem_stack_input[mem_pointer].num);
  }
  else {
    display_big = abs(mem_stack_input[mem_pointer].denom);
  }

  if ( display_big > abs(mem_stack_input[mem_pointer].denom) ) {
    display_big = expo_10[display_digit -1] * display_big;
    ++display_expo;
  }
  else {
    display_big = expo_10[display_digit] * display_big;
  }
  display_big = display_big + (abs(mem_stack_input[mem_pointer].denom) / 2) - 1;
  display_number = display_big / mem_stack_input[mem_pointer].denom;

  if (abs(display_number) == expo_10[display_digit]) {
    display_number = expo_10[display_digit - 1];
    ++display_expo;
  }
  strcpy( display_string, string_start );

  if (mem_stack_input[mem_pointer].num < 0) {
    display_number *= -1;;
  }
  ltoa(display_number, display_string, 10);

  strcpy(Temp_char, " ");
  strcat(Temp_char, display_string);
  strcpy(display_string, Temp_char);
  strcpy(Temp_char, " ");

  if (display_number >= 0) {
    strcat(Temp_char, display_string);
    strcpy(display_string, Temp_char);
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
    return;
  }

  display_string[Plus_Minus_Expo_] = '#';
  if ( expo_temp_16 >= 0 ) {
    display_string[Plus_Minus_Expo] = '#';
    display_string[Plus_Minus_Expo__] = ' ';
  }
  display_string[display_digit + 2] = '.';

  if ( display_expo_mod <= 0 ) {
    display_expo_mod = display_expo_mod + 3;
  }

  if ( Debug_Level == 9 ) {
    Serial.print("'");
    Serial.print(display_string);
    Serial.println("'");
  }

  if ( display_expo_mod < display_digit ) {
    for ( index_a = display_digit + 1; index_a > (display_expo_mod + 2); index_a -= 1 ) {
      display_string[index_a + 1] = display_string[index_a];
    }
    display_string[index_a + 1] = display_string[index_a];
    display_string[index_a] = '.';
  }

  if ( (display_digit == 2) && (display_expo_mod == 3) ) {
    display_string[4] = '0';
    display_string[5] = '.';
  }

  display_string[Memory_1] = mem_str_1[mem_pointer];
  display_string[Memory_0] = mem_str_0[mem_pointer];

  if ( Rad_in_out == true ) {
    display_string[Rad_point] = '.';
  }

  if ( display_string[Plus_Minus_Expo__] == ' ' ) {
    display_string[Plus_Minus_Expo__] = '#';
  }

  if ( Start_input >= Input_Memory ) {       // Input / Operation Display
    if ( Start_input < Display_Error ) {
      if ( Display_Status_new != 40 ) {
        display_string[Operation] = mem_stack_input[mem_pointer].op;
        display_string[Operation] -= temp_op_;
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
  Number_count = display_digit;
  Cursor_pos = display_digit + 3;

  if ( display_string[Cursor_pos] == '.' ) {
    ++Number_count;
    ++Cursor_pos;
  }

  if ( abs(mem_stack_input[mem_pointer].num) == 0 ) {
    display_string[Mantisse_0] = '.';
    display_string[Mantisse_1] = '0';
    Zero_count = display_digit;
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

  if ( abs(mem_stack_input[mem_pointer].denom) == 0 ) {
    Error_String('0');
  }

  if ( Display_Status_new == 40 ) {
    display_string[Operation] = temp_char;
  }

  if ( Debug_Level == 9 ) {
    Serial.print("'");
    Serial.print(display_string);
    Serial.println("'");
    Serial.println(display_digit);
    Serial.println(display_expo);
    Serial.println(display_expo_mod);
    Serial.println(Zero_count);
  }
}

void Display_Memory_Plus() {
  Start_input = Display_M_Plus;
  display_string[Operation] = ' ';
  display_string[Memory_1] = Display_Memory_1[3];  // m;
  display_string[Memory_0] = '0' + mem_extra_test;
}

void Put_input_Point() {
  Beep__on();
  Point_pos = Cursor_pos;
  display_string[Cursor_pos] = '.';
  ++Cursor_pos;
  display_string[Cursor_pos] = '_';
}

void Display_Off() {
  if ( Start_input == Display_Error ) {
    display_string[Cursor_pos]  = '.';
  }
  Start_input = Off_Status;
  display_string[MR_point]        = ' ';
  display_string[Expo_1]          = ' ';
  display_string[MS_point]        = ' ';
  display_string[Expo_0]          = ' ';
  display_string[Expo_point]      = ' ';
  display_string[Operation]       = 'o';
  display_string[Operation_point] = ' ';
  display_string[Memory_1]        = 'F';
  display_string[Rad_point]       = ' ';
  display_string[Memory_0]        = 'F';
  Display_new = true;
}

void Display_wait() {
  count += 1;	
  display_b[12] = led_font[count];
  Display_change = true;
  if ( count == 12 ) {
    count = 0;
  }
}

void Expand_Reduce_add() {
  calc_temp_64_d = calc_temp_64_a;
  calc_temp_64_d += calc_temp_64_b;
  if ( calc_temp_64_d > 0 ) {
    test_signum_8 =  1;
  }
  else {
    test_signum_8 = -1;
  }

  num_temp_u64    = abs(calc_temp_64_d);  // max:  9223372036854775807
  denom_temp_u64  = abs(calc_temp_64_c);
  denom_test_u64  = denom_temp_u64;
  denom_test_u64 /= 5;

      if ( Debug_Level == 15 ) {

        Serial.print("_00_ = ");
        calc_temp_u64_0 = num_temp_u64;
        calc_temp_u64_0 /= int32_max_2;
        x0_a_32 = calc_temp_u64_0;
        Serial.print(x0_a_32);
        Serial.print(" / ");
        calc_temp_u64_0 = denom_temp_u64;
        calc_temp_u64_0 /= int32_max_2;
        x0_a_32 = calc_temp_u64_0;
        Serial.println(x0_a_32);
      }

  if ( num_temp_u64 > 0 ) {
    while ( num_temp_u64 < denom_test_u64 ) {
      num_temp_u64 *= 10;
      --temp_64.expo;
    }
  }

  mul_temp_u32 = 1;
  if ( num_temp_u64 > denom_temp_u64 ) {
    calc_temp_16_0 = num_temp_u64 / denom_temp_u64;
    if ( calc_temp_16_0 > 2 ) {
      if ( denom_temp_u64 < expo_test_0a ) {
         denom_temp_u64 *= 10;
      }
      else {
        num_temp_u64   /= 2;
        denom_temp_u64 *= 5;
      }
      ++temp_64.expo;
      if ( denom_temp_u64 > 0 ) {
        mul_temp_u32 = int32_max / denom_temp_u64;
      }
    }
    else {
      if ( num_temp_u64 > 0 ) {
        mul_temp_u32 = int32_max / num_temp_u64;
      }
    }
  }
  else {
    calc_temp_16_0 = denom_temp_u64 / num_temp_u64;
    if ( calc_temp_16_0 > 2 ) {
      if ( num_temp_u64 < expo_test_0a ) {
        num_temp_u64   *= 10;
      }
      else {
        num_temp_u64   *= 5;
        denom_temp_u64 /= 2;
      }
      --temp_64.expo;
      if ( num_temp_u64 > 0 ) {
        mul_temp_u32 = int32_max / num_temp_u64;
      }
    }
    else {
      if ( denom_temp_u64 > 0 ) {
        mul_temp_u32 = int32_max / denom_temp_u64;
      }
    }
  }

  temp_32.expo = temp_64.expo;
  if ( temp_64.expo >  115 ) {
    temp_32.expo =  115;
  }
  if ( temp_64.expo < -115 ) {
    temp_32.expo = -115;
  }

  if ( mul_temp_u32 > 0 ) {
    temp_32.num = num_temp_u64;
    temp_32.num *= mul_temp_u32;    // expand
    temp_32.num *= test_signum_8;
    temp_32.denom = denom_temp_u64;
    temp_32.denom *= mul_temp_u32;  // expand
    if ( temp_32.num == 0 ) {
      temp_32.denom = int32_max;
      temp_32.expo  = 0;
    }
  }
  else {   // num_temp_u64 ,  denom_temp_u64
    if ( num_temp_u64 > 0 ) {
      if ( Debug_Level == 15 ) {

        Serial.print("_11_ = ");
        calc_temp_u64_0 = num_temp_u64;
        calc_temp_u64_0 /= int32_max_2;
        x0_a_32 = calc_temp_u64_0;
        Serial.print(x0_a_32);
        Serial.print(" / ");
        calc_temp_u64_0 = denom_temp_u64;
        calc_temp_u64_0 /= int32_max_2;
        x0_a_32 = calc_temp_u64_0;
        Serial.println(x0_a_32);
      }

      Reduce_Number();              // reduce
      temp_32.num = num_temp_u32;
      temp_32.num *= test_signum_8;
      temp_32.denom = denom_temp_u32;
    }
    else {
      temp_32.num   = 0;
      temp_32.denom = int32_max;
      temp_32.expo  = 0;
    }
  }
}

AVRational_32 mul(AVRational_32 a, AVRational_32 b) {
  temp_64.expo   = a.expo;
  temp_64.expo  += b.expo;

  gcd_temp_32 = gcd_iter_32(a.num, a.denom);
  a.num /= gcd_temp_32;
  a.denom /= gcd_temp_32;

  gcd_temp_32 = gcd_iter_32(b.num, b.denom);
  b.num /= gcd_temp_32;
  b.denom /= gcd_temp_32;

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

  mul_temp_u32   = 1;
  mul_temp_u32_a = 1;
  mul_temp_u32_b = 1;

  if ( num_temp_u64 > denom_temp_u64 ) {
    calc_temp_16_0 = num_temp_u64 / denom_temp_u64;
    if ( calc_temp_16_0 > 2 ) {
      if ( denom_temp_u64 < expo_test_0a ) {
        denom_temp_u64   *= 10;
      }
      else {
        num_temp_u64     /= 2;
        denom_temp_u64   *= 5;
      }
      ++temp_64.expo;
    }
  }
  else {
    calc_temp_16_0 = denom_temp_u64 / num_temp_u64;
    if ( calc_temp_16_0 > 2 ) {
      if ( num_temp_u64 < expo_test_0a ) {
        num_temp_u64   *= 10;
      }
      else {
        num_temp_u64   *= 5;
        denom_temp_u64 /= 2;
      }
      --temp_64.expo;
    }
  }

  if ( denom_temp_u64 > 0 ) {
    mul_temp_u32_a = int32_max / denom_temp_u64;
  }

  if ( num_temp_u64 > 0 ) {
    mul_temp_u32_b = int32_max / num_temp_u64;
  }

  if ( mul_temp_u32_a < mul_temp_u32_b ) {
    mul_temp_u32 = mul_temp_u32_a;
  }
  else {
    mul_temp_u32 = mul_temp_u32_b;
  }

  temp_32.expo = temp_64.expo;
  if ( temp_64.expo >  115 ) {
    temp_32.expo =  115;
  }
  if ( temp_64.expo < -115 ) {
    temp_32.expo = -115;
  }

  if ( mul_temp_u32 > 0 ) {
    temp_32.num    = num_temp_u64;
    temp_32.num   *= mul_temp_u32;    // expand
    temp_32.denom  = denom_temp_u64;
    temp_32.denom *= mul_temp_u32;  // expand
  }
  else {   // num_temp_u64 ,  denom_temp_u64
    Reduce_Number();                // reduce
    temp_32.num = num_temp_u32;
    temp_32.denom = denom_temp_u32;
  }

  if ( temp_32.num == 0 ) {
    temp_32.expo = 0;
  }

  temp_32.num *= test_signum_8;
  return temp_32;
}

AVRational_32 add(AVRational_32 a, AVRational_32 b, uint8_t c) {

  expo_temp_16    = a.expo;
  expo_temp_16   -= b.expo;

  if ( expo_temp_16 >  18 ) {
    temp_32.num   = a.num;
    temp_32.denom = a.denom;
    temp_32.expo  = a.expo;
    return temp_32;
  }
  if ( expo_temp_16 < -18 ) {
    temp_32.num   = b.num;
    temp_32.denom = b.denom;
    temp_32.expo  = b.expo;
    return temp_32;
  }

  gcd_temp_32 = gcd_iter_32(a.num, a.denom);
  a.num /= gcd_temp_32;
  a.denom /= gcd_temp_32;

  gcd_temp_32 = gcd_iter_32(b.num, b.denom);
  b.num /= gcd_temp_32;
  b.denom /= gcd_temp_32;

  if ( expo_temp_16 >= 0 ) {
    calc_temp_64_a   = a.num;
    calc_temp_64_a  *= b.denom;
    calc_temp_64_b   = b.num;
    calc_temp_64_b  *= a.denom;
    temp_64.expo     = a.expo;
  }
  else {
    calc_temp_64_b   = a.num;
    calc_temp_64_b  *= b.denom;
    calc_temp_64_a   = b.num;
    calc_temp_64_a  *= a.denom;
    temp_64.expo     = b.expo;
    expo_temp_16    *= -1;
  }

  calc_temp_64_c   = a.denom;
  calc_temp_64_c  *= b.denom;
  calc_temp_64_c  *= c;
  calc_temp_64_c_abs = abs(calc_temp_64_c);

  if ( expo_temp_16 >=  9 ) {
    if ( calc_temp_64_c_abs < expo_test_9b ) {
      calc_temp_64_a *= expo_10[9];
      calc_temp_64_c *= expo_10[9];
      expo_temp_16   -= 9;
    }
  }

  if ( expo_temp_16 >=  6 ) {
    if ( calc_temp_64_c_abs < expo_test_6b ) {
      calc_temp_64_a *= expo_10[6];
      calc_temp_64_c *= expo_10[6];
      expo_temp_16   -= 6;
    }
  }

  if ( expo_temp_16 >=  3 ) {
    if ( calc_temp_64_c_abs < expo_test_3b ) {
      calc_temp_64_a *= expo_10[3];
      calc_temp_64_c *= expo_10[3];
      expo_temp_16   -= 3;
    }
  }

  if ( expo_temp_16 >=  2 ) {
    if ( calc_temp_64_c_abs < expo_test_2b ) {
      calc_temp_64_a *= expo_10[2];
      calc_temp_64_c *= expo_10[2];
      expo_temp_16   -= 2;
    }
  }

  if ( expo_temp_16 >=  1 ) {
    if ( calc_temp_64_c_abs < expo_test_1b ) {
      calc_temp_64_a *= expo_10[1];
      calc_temp_64_c *= expo_10[1];
      expo_temp_16   -= 1;
    }
  }

  if ( expo_temp_16 ==  0 ) {
    Expand_Reduce_add();
    return temp_32;
  }
  else {
    calc_temp_64_b_abs = abs(calc_temp_64_b);

    if ( expo_temp_16 <  10 ) {
      calc_temp_64_b_abs += (expo_10[expo_temp_16] / 2);
      calc_temp_64_b_abs /= expo_10[expo_temp_16];
    }
    else {
      calc_temp_64_b_abs += (expo_10_[expo_temp_16 - 10] / 2);
      calc_temp_64_b_abs /= expo_10_[expo_temp_16 - 10];
    }

    if ( calc_temp_64_b > 0 ) {
      calc_temp_64_b  = calc_temp_64_b_abs;
    }
    else {
      calc_temp_64_b  = calc_temp_64_b_abs;
      calc_temp_64_b *= -1;
    }

    Expand_Reduce_add();
    return temp_32;
  }
}

AVRational_32 div_x(AVRational_32 a) {
  temp_32.expo = -a.expo;
  if ( a.num > 0 ) {
    temp_32.num   = a.denom;
    temp_32.denom = a.num;
  }
  else {
    temp_32.num   = -a.denom;
    temp_32.denom = -a.num;
  }
  return temp_32;
}

AVRational_32 min_x(AVRational_32 a) {
  temp_32.expo = a.expo;
  if ( a.denom < 0 ) {
    temp_32.num   = a.num;
    temp_32.denom = -a.denom;
  }
  else {
    temp_32.num   = -a.num;
    temp_32.denom = a.denom;
  }
  return temp_32;
}

AVRational_32 abs_x(AVRational_32 a) {
  temp_32.expo  = a.expo;
  temp_32.num   = abs(a.num);
  temp_32.denom = abs(a.denom);
  return temp_32;
}

AVRational_32 floor_(AVRational_32 a, int8_t expo_test) {
// floor_(x) returns the largest integer n such that n <= x
  denom_temp_u64  = a.denom;
  temp_32.expo    = a.expo;

  if ( a.expo > expo_test ) {
    if ( a.num < 0 ) {
      Error_String('U');
    }
    else {
      Error_String('^');
    }
    return a;
  }
  if ( a.expo == expo_test ) {
  	if (abs(a.num) >= abs(a.denom)) {
      if ( a.num < 0 ) {
        Error_String('U');
      }
      else {
        Error_String('^');
      }
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

AVRational_32 frac(AVRational_32 a) {
// returns the fractional part of x
  temp_32 = add(a, min_x(floor_(a, 8)), 1);
  return temp_32;
}

int8_t compare(Rational_32 a, Rational_32 b) {
  uint64_t test_a = a.num;
  uint64_t test_b = b.num;
  int8_t   comp = 0;
	
  test_a *= b.denom;
  test_b *= a.denom;
	
  if ( test_a > test_b ) {
    comp = 1;
    return comp;
  }

  if ( test_a < test_b ) {
    comp = -1;
    return comp;
  }
	
  return comp;
}

int8_t compare(Rational_64 a, Rational_64 b) {
	  int8_t comp    = 0;
  uint64_t test_a  = 0;  // Factor
  uint64_t test_b  = 0;	 // Factor
  uint32_t test_print = 0;  	
  
  while ( comp == 0 ) {
    test_a  = a.num;  // Factor
    test_a /= a.denom;
    test_b  = b.num;	 // Factor
    test_b /= b.denom;

    if ( test_a > test_b ) {
      comp = 1;
      return comp;
    }

    if ( test_a < test_b ) {
      comp = -1;
      return comp;
    }
  
    test_a *= a.denom;
    a.num  -= test_a;
  
    if ( a.num == 0 ) {
      if ( Debug_Level == 22 ) {
      	test_print = a.denom;
     	  Serial.print("  --> a_denom ");
    	  Serial.print(test_print);
      	test_print = b.denom;
     	  Serial.print("  --> b_denom ");
    	  Serial.println(test_print);
 	    }

      if ( a.denom > b.denom ) {
        comp = 1;
        return comp;
      }

      if ( a.denom < b.denom ) {
        comp = -1;
        return comp;
      }
  
      return comp;
    }
  
    test_b *= b.denom;
    b.num  -= test_b;

    test_a  = a.denom;  // Factor
    test_a /= a.num;
    test_b  = b.denom;	 // Factor
    test_b /= b.num;

    if ( test_a > test_b ) {
      comp = -1;
      return comp;
    }

    if ( test_a < test_b ) {
      comp = 1;
      return comp;
    }

    test_a  *= a.num;
    a.denom -= test_a;
  
    if ( a.denom == 0 ) {
      if ( Debug_Level == 22 ) {
      	test_print = a.num;
     	  Serial.print("  --> a_num ");
    	  Serial.print(test_print);
      	test_print = b.num;
     	  Serial.print("  --> b_num ");
    	  Serial.println(test_print);
 	    }

      if ( a.num > b.num ) {
        comp = -1;
        return comp;
      }

      if ( a.num < b.num ) {
        comp = 1;
        return comp;
      }

      return comp;
    }
  
    test_b  *= b.num;
    b.denom -= test_b;
  }
  return comp;
}

Rational_32 compare_extra(Rational_32 a, Rational_32 b) {
	Rational_64 calc;
	Rational_64 input;
	int8_t comp = 0;
  uint64_t test = 0; 
	  	
  input.num   = num_temp_u64;
  input.denom = denom_temp_u64;
	
  calc.num  = a.num;
  calc.num *= b.denom;
  test      = b.num;
  test     *= a.denom;
  calc.num += test;
	
  calc.denom  = a.denom;
  calc.denom *= b.denom;
  calc.denom *= 2;
	
  comp = compare(calc, input);
	
  if ( comp > 0 ) {
    return a;	
  }
	
  if ( comp < 0 ) {
    return b;	
  }

  if ( a.num > b.num ) {
    return a;
  }
  else {
    return b;
  }
}

void calc_stack(uint8_t count_stack) {
  if ( count_stack == 2 ) {
    if ( Found_constant == true ) {
      number_stack[ 1 ].num = mem_stack_calc[ 1 ].num;
      number_stack[ 1 ].denom = mem_stack_calc[ 1 ].denom;
      number_stack[ 1 ].expo = mem_stack_calc[ 1 ].expo;

      number_stack[ 2 ].num = mem_stack_calc[ 2 ].num;
      number_stack[ 2 ].denom = mem_stack_calc[ 2 ].denom;
      number_stack[ 2 ].expo = mem_stack_calc[ 2 ].expo;
      
      switch (temp_operation) {
      	
        case 42:  //  _*_
        case 213: //  _* *_
        	temp_32 = mul(number_stack[1], number_stack[2]);
          break;

        case 43:  //   _+_ 
        case 216: //   _+ +_
        	temp_32 = add(number_stack[1], number_stack[2], 1);
          break;

        case 45:  //   _-_
        	temp_32 = add(number_stack[1], min_x(number_stack[2]), 1);
          break;
                    
        case 47:  //   _/_
        	temp_32 = mul(number_stack[1], div_x(number_stack[2]));
          break;

        case 212:  //   _- -_
         	temp_32 = add(number_stack[2], min_x(number_stack[1]), 1);
          break;
                    
        case 214: //   _/ /_
        	temp_32 = mul(number_stack[2], div_x(number_stack[1]));
          break;

        case 124: //   _HM_ 
        case 219: //   _HM HM_
        	temp_32 = div_x(add(div_x(number_stack[1]), div_x(number_stack[2]), 2));
          break;

        case 128: //   _AM_ 
        case 220: //   _AM AM_
        	temp_32 = add(number_stack[1], number_stack[2], 2);
          break;

        case 129: //   _GM_ 
        case 221: //   _GM GM_
        	temp_32 = sqrt(mul(number_stack[1], number_stack[2]));
          break;

        case 151: //   _AGM_ 
        case 222: //   _AGM AGM_
          count = 0;
        	temp_32 = agm(number_stack[1], number_stack[2]);
          break;

        case 92:  //   _//_ 
        case 215: //   _// //_
        	temp_32 = div_x(add(div_x(number_stack[1]), div_x(number_stack[2]), 1));
          break;

        case 94:  //   _/p/_ 
        	temp_32 = sqrt(add(square(number_stack[1]), square(number_stack[2]), 1));
          break;

        case 211: //   _/p/ /p/_ 
        	temp_32 = sqrt(add(square(number_stack[1]), min_x(square(number_stack[2])), 1));
          break;

        default:
        	temp_32 = number_stack[ 2 ];
      }
      
      mem_stack_calc[ 0 ].num = temp_32.num;
      mem_stack_calc[ 0 ].denom = temp_32.denom;
      mem_stack_calc[ 0 ].expo = temp_32.expo;
      mem_stack_calc[ 0 ].op = mem_stack_calc[1].op;

      mem_stack_input[ 0 ].num = temp_32.num;
      mem_stack_input[ 0 ].denom = temp_32.denom;
      mem_stack_input[ 0 ].expo = temp_32.expo;
      
      mem_extra_left_right( 0, 0 );
  	}
  }
}

uint32_t int_sqrt_64( uint64_t x ) { // (730 / 4096) ms - 12Mhz
                                     // 180 µs
  if ( x == 0 ) {
    return 0;
  }

  uint64_t a     = 0;
  uint64_t input = x;

  a = x = sqrtf(input);    // +/- 127 Fehler
                           //  70 µs
  x     = input / x;       //    (450 / 4096) ms - 12Mhz
  a = x = (x + a) >> 1;    // 110 µs
                           // 0/-   1 Fehler
  return x;
}

AVRational_32 sqrt(AVRational_32 a) {

  num_temp_u64    = abs(a.num);
  num_temp_u64   *= abs(a.denom);
  calc_temp_u32   = int_sqrt_64(num_temp_u64);
  denom_temp_u64  = calc_temp_u32;
  num_temp_u64   += denom_temp_u64 * denom_temp_u64;
  denom_temp_u64 *= abs(a.denom);
  denom_temp_u64 *= 2;
  Reduce_Number();
  temp_32.num     = num_temp_u32;
  temp_32.denom   = denom_temp_u32;

  temp_32.expo  = a.expo;
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

AVRational_32 agm(AVRational_32 a, AVRational_32 b) {
  temp_32_a = add(abs_x(a), abs_x(b), 2);
  temp_32_b = sqrt(mul(abs_x(a), abs_x(b)));
  for ( uint8_t index_z = 0; index_z < 7; index_z += 1 ) {
    if ((temp_32_a.num * temp_32_b.denom) == (temp_32_b.num * temp_32_a.denom)) {
      return temp_32_b;
    } 
    if ((temp_32_a.num * temp_32_a1.denom) == (temp_32_a1.num * temp_32_a.denom)) {
      return temp_32_b;
    }
    temp_32_a1 = add(temp_32_a, temp_32_b, 2);
    Display_wait();
    temp_32_b1 = sqrt(mul(temp_32_a, temp_32_b));
    if ((temp_32_a1.num * temp_32_b1.denom) == (temp_32_b1.num * temp_32_a1.denom)) {
      return temp_32_b;
    } 
    if ((temp_32_a.num * temp_32_a1.denom) == (temp_32_a1.num * temp_32_a.denom)) {
      return temp_32_b;
    }  
    temp_32_a  = add(temp_32_a1, temp_32_b1, 2);
    Display_wait();
    temp_32_b  = sqrt(mul(temp_32_a1, temp_32_b1));
  }
  return temp_32_b;       // 
}

AVRational_32 square(AVRational_32 a) {
  temp_32 = mul(a, a);          // 1,83664477 44079e+93  calc
  return temp_32;               // 1,83664477 30683e+93  exact
}

AVRational_32 cubic(AVRational_32 a) {
  temp_32 = mul(a, square(a));  // 299.482631 91683e+48  calc
  return temp_32;               // 2.99482631 72691e+50  exact
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
}

void mem_extra_left_right( uint8_t left, uint8_t right ) {
  mem_extra_stack[ left ].num = mem_stack_input[ right ].num;
  mem_extra_stack[ left ].denom = mem_stack_input[ right ].denom;
  mem_extra_stack[ left ].expo = mem_stack_input[ right ].expo;
  mem_extra_stack[ left ].op = mem_stack_input[ right ].op; // temp_op_;
}

void left_right_mem_extra( uint8_t left, uint8_t right ) {
  mem_stack_input[ left ].num = mem_extra_stack[ right ].num;
  mem_stack_input[ left ].denom = mem_extra_stack[ right ].denom;
  mem_stack_input[ left ].expo = mem_extra_stack[ right ].expo;
  mem_stack_input[ left ].op = mem_extra_stack[ right ].op;
}

void add_operation_to_mem( uint8_t deep_step, char op_add ) {
  if ( deep_step > 0 ) {
    if ( mem_stack_count < mem_stack_max ) {
    	Constant_arithmetic = true;
      First_operation = true;
    }
  }
  temp_operation = Switch_Code;
  if ( Start_input < Input_Memory ) {      // Input Number
    Get_Number( deep_step );
  }
  if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) ) {
    mem_stack_count = 1;
    mem_pointer = mem_stack_count;
    Start_input  = Input_Operation_0;
    Start_input += deep_step;
    copy_input_left_right( mem_pointer, 0 );
  }
  if ( Start_input > Input_Fraction ) {
    if ( Start_input < Display_Input_Error ) {
      Start_input  = Input_Operation_0;
      if ( mem_stack_count < mem_stack_max) {
        mem_save = false;
        mem_exchange = false;
        Error_Test();
        if ( Start_input != Display_Error ) {
          if ( display_string[Operation] == op_add ) {
            if ( display_string[Expo_point] == ' ' ) {
              display_string[Expo_point] = '.';
            }
            else {
              display_string[Expo_point] = ' ';
            }
            display_string[Memory_1] = mem_str_1[mem_pointer];
            display_string[Memory_0] = mem_str_0[mem_pointer];
          }
          else {
            mem_stack_input[ mem_pointer ].op = op_add;
            Display_Number();
            display_string[Operation_point] = '.';
          }
        }
        Start_input += deep_step;
      }
      else {
        if ( max_input == false ) {
          max_input = true;
          mem_stack_input[ mem_pointer ].op  = temp_op_;
          mem_stack_input[ mem_pointer ].op += 'x';
          Start_input  = Input_Operation_0;
          temp_operation = mem_stack_calc[ mem_stack_count-1 ].op;
          Display_Number();
          Beep__on();
        }
      }
      if ( max_input == false ) {
        Beep__on();
      }
    }
  }
}

void Test_all_function() {
  if ( Debug_Level == 18 ) {
    time_start = millis();

    Serial.println(" ");
    for ( int32_t index = 100; index <= 10000; index += 1 ) {
      Serial.print(index);
      Serial.print("  ");
      calc_32.expo = 0;
      num_temp_u32 = index;
      denom_temp_u32 = 100;
      if ( num_temp_u32 > denom_temp_u32 ) {
        gcd_temp_32 = num_temp_u32 / denom_temp_u32;
        while ( gcd_temp_32 > 2 ) {
          ++calc_32.expo;
          gcd_temp_32    /= 10;
          denom_temp_u32 *= 10;
        }
      }
      else {
        gcd_temp_32 = denom_temp_u32 / num_temp_u32;
        while ( gcd_temp_32 > 2 ) {
          --calc_32.expo;
          gcd_temp_32    /= 10;
          num_temp_u32   *= 10;
        }
      }
      Expand_Number();

      calc_32.num = num_temp_u32;
      calc_32.denom = denom_temp_u32;
    /*
      Serial.print(calc_32.num);
      Serial.print("  ");
      Serial.print(calc_32.denom);
      Serial.print("  ");
      Serial.print(calc_32.expo);
      Serial.print("  ");
 
      test_32 = sqrt(calc_32);
      Serial.print(test_32.num);
      Serial.print("  ");
      Serial.print(test_32.denom);
      Serial.print("  ");
      Serial.print(test_32.expo);
      Serial.println(" -> ");
    */
      test_32 = square(test_32);

      Serial.print(test_32.num);
      Serial.print("  ");
      Serial.print(test_32.denom);
      Serial.print("  ");
      Serial.print(test_32.expo);
      Serial.println("  "); 

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
}

void Test_Switch_up_down() {
  if ( Switch_old > Switch_down ) {
      Switch_delta  = Switch_old;
      Switch_delta -= Switch_down;

      switch (Switch_delta) {   // change  -->   Display_Status_new

        case 1:
          bit_3 = 0;           //     "EE"       Write to the bit.
          break;

        case 2:
          bit_4 = 0;           //     "FN"       Write to the bit.
          break;

        case 4:
          bit_5 = 0;           //     "="        Write to the bit.
          break;

        case 3:
        case 5:
        case 6:
        case 7:
          bit_3 = 0;           //     "EE"       Write to the bit.
          bit_4 = 0;           //     "FN"       Write to the bit.
          bit_5 = 0;           //     "="        Write to the bit.
          break;

        case 128:              //   "x"
          if ( Display_rotate == true ) {
            Display_rotate = false;
            Switch_Code = 177;   //             Dis_Cha_Dir_off
          }
          break;

        case 4096:             //  1
          bit_0 = 0;           //     "0"         Write to the bit.
          if ( Display_Status_old == 25 ) {
            Mr_0_test = true;
          }
          break;

        case 8192:             //  2
          bit_1 = 0;           //     "."         Write to the bit.
          break;

        case 16384:            //  4
          bit_2 = 0;           //     "+/-"       Write to the bit.
          break;

        case 12288:            //  3
        case 20480:            //  5
        case 24576:            //  6
        case 28672:            //  7
          bit_0 = 0;           //     "0"         Write to the bit.
          bit_1 = 0;           //     "."         Write to the bit.
          bit_2 = 0;           //     "+/-"       Write to the bit.
          break;

        case 32768:      //    1
        case 98304:

        case 65536:      //    2

        case 131072:     //    3
        case 196608:

        case 262144:     //    4
        case 786432:

        case 524288:     //    5

        case 1048576:    //    6
        case 1572864:

        case 2097152:    //    7
        case 6291456:

        case 4194304:    //    8

        case 8388608:    //    9
        case 12582912:

        case 2048:       //    )
        case 3072:

        case 1024:       //    (

        case 512:        //    _CE_
        case 1536:

        case 256:        //    _/_
        case 384:

        case 64:         //    <--
        case 192:

        case 32:         //    _-_
        case 48:

        case 16:         //     _+_

        case 8:          //    _1/x_
        case 24:

          Display_Status_old = 255;
          switch (Display_Status_new) {

            case 0:         // __
            case 4:         // +/-
            case 8:         // inv
            case 16:        // FN
            case 24:        // MR
            case 40:        // Display
            case 48:        // MS
            case 128:       // "+" "-" "x" "/"
              Display_Status_old = Display_Status_new;
              break;
          }
          break;
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
          bit_5 = 1;           //     "="        Write to the bit.
          Switch_Code = 61;
          break;

        case 7:          //   "EE" + "FN"  + "="  three Switch pressed
          Switch_Code = 59;   //                  EE_FN_=
          break;

        case 1028:     //     "="  +  "("   new   two Switch pressed
          Switch_Code = 175;  //                  _EE-1_ ">"
          break;

        case 516:      //     "="  +  "CE"  new   two Switch pressed
          Switch_Code = 174;  //                  _EE+1_ "<"
          break;

        case 1030:     //    "FN" + "=" +  "("    three Switch pressed
          Switch_Code = 127;  //            new   -->
          break;

        case 518:      //    "FN" + "=" +  "CE"   three Switch pressed
          Switch_Code = 30;  //             new   <--
          break;

        case 1540:     //    "=" + "CE"  +  "("   three Switch pressed
        case 1542:     //    "=" + "CE"  +  "("   three Switch pressed
          Switch_Code = 148;  //            new   <<-->>
          break;

        case 4098:     //    "FN" +  "0"          two Switch pressed
          if ( Display_Status_new == 16 ) {
            Switch_Code = 119;  //                x!
          }
          break;

        case 4097:     //    "EE" +  "0"          two Switch pressed
          if ( Display_Status_new == 8 ) {
            Switch_Code = 190;  //                RND
          }
          break;

        case 8194:     //    "FN"- "."            two Switch pressed
          if ( Display_Status_new == 16 ) {
            Switch_Code = 122; //            new  Beep
          }
          break;

        case 12288:     //    "0" + "."           two Switch pressed
          Switch_Code = 150; //              new  "° ' ''" hh:mm:ss  Input
          break;

        case 16386:    //    "FN"- "+/-"          two Switch pressed
          if ( Display_Status_new == 16 ) {
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

        case 11:       //    "pi()"               defect
        case 19:       //    "pi()"               defect
        case 27:       //    "EE" + "FN" + "1/x"  defect
        case 67:       //    "<"                  defect
        case 72:       //    "<--"                defect
        case 131:      //    "<"                  defect
        case 194:      //    "<"                  defect
        case 515:      //    "EE" + "FN" +  "<"   defect
        case 585:      //    "e()"                defect
        case 576:      //    "CE"                 defect
        case 1027:     //    "("                  defect
        case 1537:     //    "("                  defect
        case 1538:     //    "("                  defect
        case 2054:     //    "FN" + "=" +  "("    defect
        case 196614:   //    "2" + "3" + "MS_on"  defect
        case 1572870:  //    "5" + "6" + "MS_on"  defect
        case 12582918: //    "8" + "9" + "MS_on"  defect
          break;

        default:
          switch (Switch_delta) {

            case 4096:       //    0
            case 12288:
              Switch_Code = 48;
              break;

            case 8192:       //    _._
              Switch_Code = 46;
              break;

            case 16384:      //    +/-
            case 24576:
              Switch_Code = 35;
              break;
          }
      }

      switch (Switch_delta) {   // change  -->   Display_Status_new

        case 1:
          bit_3 = 1;           //     "EE"       Write to the bit.
          break;

        case 2:
          bit_4 = 1;           //     "FN"       Write to the bit.
          break;

        case 4:
          if ( bit_3 == 1 ) {
            bit_5 = 1;           //     "="        Write to the bit.
          }
          if ( bit_4 == 1 ) {
            bit_5 = 1;           //     "="        Write to the bit.
          }
          if ( Start_input == Display_Result ) {
            bit_5 = 1;           //     "="        Write to the bit.
          }
          break;

        case 3:
          bit_3 = 1;           //     "EE"       Write to the bit.
          bit_4 = 1;           //     "FN"       Write to the bit.
          break;

        case 6:
          bit_4 = 1;           //     "FN"       Write to the bit.
          bit_5 = 1;           //     "="        Write to the bit.
          break;

        case 5:
          bit_3 = 1;           //     "EE"       Write to the bit.
          bit_5 = 1;           //     "="        Write to the bit.
          break;

        case 7:
          bit_3 = 1;           //     "EE"       Write to the bit.
          bit_4 = 1;           //     "FN"       Write to the bit.
          bit_5 = 1;           //     "="        Write to the bit.
          break;


        case 4096:             //  1
          bit_0 = 1;           //     "0"         Write to the bit.
          break;

        case 8192:             //  2
          bit_1 = 1;           //     "."         Write to the bit.
          break;

        case 16384:            //  4
          bit_2 = 1;           //     "+/-"       Write to the bit.
          break;

        case 12288:            //  3
          bit_0 = 1;           //     "0"         Write to the bit.
          bit_1 = 1;           //     "."         Write to the bit.
          break;

        case 24576:            //  6
          bit_1 = 1;           //     "."         Write to the bit.
          bit_2 = 1;           //     "+/-"       Write to the bit.
          break;

        case 20480:            //  5
          bit_0 = 1;           //     "0"         Write to the bit.
          bit_2 = 1;           //     "+/-"       Write to the bit.
          break;

        case 28672:            //  7
          bit_0 = 1;           //     "0"         Write to the bit.
          bit_1 = 1;           //     "."         Write to the bit.
          bit_2 = 1;           //     "+/-"       Write to the bit.
          break;

        case 32768:      //    1
        case 98304:
          switch (Display_Status_new) {

            case 0:
              Switch_Code = 49;
              break;

            case 1:
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

            case 32:
              Switch_Code = 103;  //           new  M_plus(1)
              break;

            case 40:
              Switch_Code = 87;  //            new  FIX_a_b/c
              break;

            case 48:
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

            case 32:
              Switch_Code = 104;  //           new  M_plus(2)
              break;

            case 40:
              Switch_Code = 95;  //            new  FIX_2
              break;

            case 48:
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

            case 32:
              Switch_Code = 105;  //           new  M_plus(3)
              break;

            case 40:
              Switch_Code = 96;  //            new  FIX_3
              break;

            case 48:
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

            case 32:
              Switch_Code = 106;  //           new  M_plus(4)
              break;

            case 40:
              Switch_Code = 97;  //            new  FIX_4
              break;

            case 48:
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

            case 32:
              Switch_Code = 107;  //           new  M_plus(5)
              break;

            case 40:
              Switch_Code = 98;  //            new  FIX_5
              break;

            case 48:
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

            case 32:
              Switch_Code = 108;  //           new  M_plus(6)
              break;

            case 40:
              Switch_Code = 99;  //            new  FIX_6
              break;

            case 48:
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

            case 32:
              Switch_Code = 109;  //           new  M_plus(7)
              break;

            case 40:
              Switch_Code = 100;  //           new  FIX_7
              break;

            case 48:
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

            case 32:
              Switch_Code = 110;  //           new  M_plus(8)
              break;

            case 40:
              Switch_Code = 101;  //           new  FIX_8
              break;

            case 48:
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

            case 32:
              Switch_Code = 111;  //           new  M_plus(9)
              break;

            case 40:
              Switch_Code = 102;  //           new  FIX_E24
              break;

            case 48:
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
              Switch_Code = 115;  //                log(x)
              break;

            case 24:
              Switch_Code = 123;  //           new  Off
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

            case 40:
              Switch_Code = 201;  //          new   Beep
              break;
          }
          break;

        case 512:        //    _CE_
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
              Switch_Code = 91;   //           new   Light up
              break;

          }
          break;

        case 256:        //    _/_
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

            case 24:
            case 32:
            case 40:
            case 48:      //     EE +  =  + _*_      three Switch pressed
              Display_rotate = true;
              Switch_Code = 176;   //          new   _Cha_Dis_Dir_on_
              break;
          }
          break;

        case 64:         //    <--
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
              Switch_Code = 93;   //           new   Light down
              break;
          }
          break;

        case 32:         //    _-_
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
              Switch_Code = 37;   //           new   grd
              break;

            case 16:
              Switch_Code = 94;   //           new   _/p/_  Phytagoras
              break;

            case 24:
              Switch_Code = 39;   //           new   _EE-3_
              break;
          }
          break;

        case 16:         //     _+_
          switch (Display_Status_new) {

            case 0:
              Switch_Code = 43;
              break;

            case 8:
              Switch_Code = 64; //             new  rad
              break;

            case 16:
              Switch_Code = 92;   //           new   //
              break;
          }
          break;

        case 8:          //    _1/x_
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
          }
          break;
      }
  }
  Switch_old = Switch_down;
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
	Display_new = true;
  if ( Beep_on == false ) {
    Beep_on = true;
  }
}

void copy_stack_input_2_calc( uint8_t left, uint8_t right ) {
  if ( Start_input < Input_Operation_0 ) {
    First_operation = false;
  }
  if ( Start_input < Display_Result ) {
    mem_stack_calc[ left ].num = mem_stack_input[ right ].num;
    mem_stack_calc[ left ].denom = mem_stack_input[ right ].denom;
    mem_stack_calc[ left ].expo = mem_stack_input[ right ].expo; 		
  }
  mem_stack_calc[ left ].op = temp_operation;
  mem_stack_calc[ left ].op_priority = Start_input;
  mem_stack_count += 1;
}

void Input_Operation_2_function() {
  if ( Start_input > Input_Operation_0 ) {
    if ( Start_input < Display_Input_Error ) {
      copy_stack_input_2_calc( mem_stack_count, mem_pointer );
      copy_input_left_right( mem_stack_count, mem_pointer );
      Start_input = Input_Operation_0;
    }
  }
}

void Function_2_display() {
  mem_stack_input[ mem_pointer ].op = temp_op_;
  if ( max_input == false ) {
    mem_stack_input[ mem_pointer ].op += '_';
  }
  else {
    mem_stack_input[ mem_pointer ].op += 'x';
  }
  if ( First_operation == true ) {
    First_operation = false;
    mem_pointer = mem_stack_count;
    copy_input_left_right( mem_stack_count, 0 );
  }
  Display_Number();
}

void Function_1_number() {
  if ( Start_input < Input_Memory ) {      // Input Number
    Get_Number( 0 );
  }
  if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) ) {
    mem_stack_count = 1;
    mem_pointer = mem_stack_count;
    copy_input_left_right( mem_pointer, 0 );
    Start_input = Input_Operation_0;
  }
  Input_Operation_2_function();
  if ( (Start_input == Input_Operation_0) || (Start_input == Input_Memory) ) {
    mem_save = false;
    mem_exchange = false;
    Start_input = Input_Operation_0;
    mem_pointer = 0;
    
    switch (Switch_Code) {
    	
      case 33:                 //    _1/x_
      	mem_stack_input[ mem_pointer ] = div_x(mem_stack_input[ mem_pointer ]);
      	break;

    	case 117:                //    _x^2_
    	  mem_stack_input[ mem_pointer ] = square(mem_stack_input[ mem_pointer ]);
    	  break;
    	  
    	case 118:                //    _sqrt()_
    	  mem_stack_input[ mem_pointer ] = sqrt(mem_stack_input[ mem_pointer ]);
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
    }
    
    Error_Test();
    if ( Start_input != Display_Error ) {
      Function_2_display();
    }
    else {
      mem_pointer = mem_stack_count;
    }
  }
  Beep__on();
}
	
boolean Test_buffer = false;
uint8_t Number_of_buffer = 0;
// Create a RingBufCPP object designed to hold a Max_Buffer of Switch_down
RingBufCPP<uint32_t, Max_Buffer > q;

// Define various ADC prescaler
static const unsigned char PS_16  = (1 << ADPS2);
static const unsigned char PS_32  = (1 << ADPS2) |                (1 << ADPS0);
static const unsigned char PS_64  = (1 << ADPS2) | (1 << ADPS1);
static const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

// the setup routine runs once when you press reset:
void setup() {
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

  uint8_t pin;
  for ( pin = Min_Out; pin <= Max_Out; pin += 1 ) {
    pinMode(pin, INPUT_PULLUP);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }

  digitalWrite(On_Off_PIN, LOW);

  pinMode(SDA, INPUT);           // Pin 17
  pinMode(SCL, OUTPUT);          // Pin 16
  pinMode(SDA, INPUT_PULLUP);    // Pin 17

  pinMode(A_0, INPUT);           // set pin to input
  pinMode(A_1, INPUT);           // set pin to input
  pinMode(A_2, INPUT);           // set pin to input

  pinMode(Beep_m, INPUT_PULLUP); // Pin A3
  pinMode(Beep_p, INPUT_PULLUP); // Pin A7

  pinMode(Out_A, OUTPUT);        // Pin A4
  digitalWrite(Out_A, LOW);
  pinMode(Out_B, OUTPUT);        // Pin A5
  digitalWrite(Out_B, LOW);
  pinMode (Out_C, OUTPUT);       // Pin A6
  digitalWrite(Out_C, LOW);

  analogReference(EXTERNAL);

  Timer1.initialize(Time_LOW);  // sets timer1 to a period of 263 microseconds
  TCCR1A |= (1 << COM1B1) | (1 << COM1B0);  // inverting mode for Pin OC1B --> D4
  Timer1.attachInterrupt( timerIsr );  // attach the service routine here
  Timer1.pwm(PWM_Pin, led_bright[led_bright_index]);  // duty cycle goes from 0 to 1023

  // start serial port at 115200 bps and wait for port to open:
  if ( Debug_Level > 0 ) {
    Serial.begin(115200);
    delay(1);
  }

  q.add(Switch_down);
  q.pull(&Switch_down);

  digitalWrite(PWM_s, LOW);
}

// the loop routine runs over and over again forever:
void loop() {
	
	if ( Countdown_OFF < Countdown_Off_0 ) {
		if ( Countdown_OFF > 0 ) {
      digitalWrite(On_Off_PIN, HIGH);
	  }
	}

  if ( Switch_Code > 0 ) {     // Main Responce about Switch

    Print_Statepoint();
    switch (Switch_Code) {

      case 91:                 //   "FN"- Light_up
        if ( Debug_Level == 5 ) {
          Serial.println("Light_up");
        }
        ++led_bright_index;
        if ( led_bright_index > led_bright_max ) {
          led_bright_index = led_bright_max;
        }
        else {
          Beep__on();
        }
        break;

      case 93:                 //   "FN"- Light_down
        if ( Debug_Level == 5 ) {
          Serial.println("Light_down");
        }
        --led_bright_index;
        if ( led_bright_index < led_bright_min ) {
          led_bright_index = led_bright_min;
        }
        else {
          Beep__on();
        }
        break;

      case 121:                //    clock
        if ( Debug_Level == 5 ) {
          Serial.println("clock");
        }

        if (Pendular_on == false) {
          Pendular_on = true;
          Start_mem = Start_input;
        }
        else {
          Pendular_on = false;
          Start_input = Start_mem;
          Beep__on();
        }
        break;

      case 122:                //    Beep_On_Off
        if ( Debug_Level == 5 ) {
          Serial.println("Beep_On_Off");
        }
        Beep__on();
        Beep_on_off = !Beep_on_off;     // (toggle)
        Beep_on_off_temp = Beep_on_off; // make synchron
        break;

      case 123:                //    Off
        if ( Debug_Level == 5 ) {
          Serial.println("OFF");
        }
        Display_Status_new = 152;
        Display_Off();
        Countdown_OFF = Countdown_Off_2;      // Switch Off  2x Beep
        break;

      case 125:                //    Off after  5min
        if ( Debug_Level == 5 ) {
          Serial.println("OFF_5min");
        }
        Display_Status_new = 152;
        Display_Off();
        Countdown_OFF = Countdown_Off_3;  // Switch Off  3x Beep
        break;

      case 90:                 //    _CE_
        if ( Debug_Level == 5 ) {
          Serial.println("_CE_");
        }
        if ( (Pendular_on == false) && (Start_input == Display_Error) ) {
          Beep__on();
          mem_pointer = mem_stack_count;
          Start_input = Start_Mode;
          Switch_Code =   0;
          index_5min  = 255;
        }
        if ( Pendular_on == true) {
          Pendular_on = false;
          Beep__on();
          Beep_on_off = Beep_on_off_temp;
          mem_pointer = mem_stack_count;
          Start_input = Start_Mode;
          Switch_Code =   0;
          index_5min  = 255;
        }
        break;

    }
    Print_Statepoint_after();

    if ( (Pendular_on == false) && (Start_input != Display_Error) ) {

      Print_Statepoint();
      switch (Switch_Code) {

        case 30:                 //    <--
          if ( Debug_Level == 5 ) {
            Serial.println("<<-");
          }
          Beep__on();
          break;

        case 32:                 //    FIX_hms
          if ( Debug_Level == 5 ) {
            Serial.println("FIX_hms");
          }
          Beep__on();
          break;

        case 33:                 //    _1/x_
          if ( Debug_Level == 5 ) {
            Serial.println("1/x");
          }
          Function_1_number();
          break;

        case 34:                 //    lb
          if ( Debug_Level == 5 ) {
            Serial.println("lb(x)");
          }
          Beep__on();
          break;

        case 35:                 //    _+/-_
          if ( Debug_Level == 5 ) {
            Serial.println("+/-");
          }
          Display_Status_old = 0;
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
          if ( ( Start_input == Display_Result ) || ( Start_input == Display_M_Plus ) ) {
            mem_stack_count = 1;
            mem_pointer = mem_stack_count;
            Start_input = Input_Mantisse;
            mem_stack_count = 1;
            left_right_mem_extra( mem_pointer, 0 );

            display_digit_temp = display_digit;
            display_digit = 8;
            Display_Number();
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
            Expo_change = true;
            if ( display_string[Plus_Minus_Expo] == '-' ) {
              display_string[Plus_Minus_Expo] = '#';
            }
            else {
              display_string[Plus_Minus_Expo] = '-';
            }
          }
          Beep__on();
          break;

        case 36:                 //    PI()
          if ( Debug_Level == 5 ) {
            Serial.println("PI()");
          }
          if ( Start_input > Input_Operation_0 ) {
            if ( Start_input < Display_Input_Error ) {
              First_operation = false;
              copy_stack_input_2_calc( mem_stack_count, mem_pointer );
              mem_pointer = mem_stack_count;
              Clear_String();
              Start_input = Input_Memory;
            }
          }
          if ( (Start_input < Input_Operation_0) || (Display_Error < Start_input) ) {
            mem_extra_test = 10;
            Memory_to_Input_Operation();
          }
          else {
            display_string[Memory_1] = '_';
            display_string[Memory_0] = '_';
          }
          Beep__on();
          break;

        case 37:                 //    _Deg_
          if ( Debug_Level == 5 ) {
            Serial.println("Deg");
          }
          if ( Rad_in_out == true ) {
            if ( Start_input < Input_Memory ) {      // Input Number
              Get_Number( 0 );
            }
            if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) ) {
              mem_stack_count = 1;
              mem_pointer = mem_stack_count;
              copy_input_left_right( mem_pointer, 0 );
              Start_input = Input_Operation_0;
            }
            if ( (Start_input == Input_Operation_0) || (Start_input == Input_Memory) ) {
              Error_Test();
              mem_pointer = mem_stack_count;
              if ( Start_input != Display_Error ) {
                mem_pointer = 0;
                Rad_in_out = false;
                mem_stack_input[ mem_pointer ] = mul(mem_stack_input[ mem_pointer ], to_xx[ 12 ]);
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
                  Display_Number();
                }
              }
            }
            if ( Start_input > Input_Operation_0 ) {
              if ( Start_input < Display_Input_Error ) {
                display_string[Memory_1] = '_';
                display_string[Memory_0] = '_';
              }
            }
          }
          else {
            display_string[Memory_1] = '_';
            display_string[Memory_0] = '_';
          }
          Beep__on();
          break;

        case 38:                 //    _EE+3_
          if ( Debug_Level == 5 ) {
            Serial.println("EE+3");
          }
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
              Beep__on();
            }
          }
          break;

        case 39:                 //    _EE-3_
          if ( Debug_Level == 5 ) {
            Serial.println("EE-3");
          }
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
              expo_temp_16 = (expo_temp_16 * 3) - 102;
              Put_Expo();
              Beep__on();
            }
          }
          break;

        case 40:                 //    _(_
          if ( Debug_Level == 5 ) {
            Serial.println("_(_");
          }
          Beep__on();
          break;

        case 41:                 //    _)_
          if ( Debug_Level == 5 ) {
            Serial.println("_)_");
          }
          if ( (Start_input == Input_Operation_0) || (Start_input == Input_Memory) ) {
            First_operation = false;
            max_input = false;
            Clear_String();
            Start_input = Start_Mode;          
          }
          if ( Start_input > Input_Operation_0 ) {
            if ( Start_input < Display_Input_Error ) {
              First_operation = false;
              add_operation_to_mem( 0, '_' );
              temp_operation = 0;
              display_string[Operation_point] = ' ';
            }
          }
          if ( Start_input == Input_Mantisse ) {
            if ( display_string[ 2 ]  == '_' ) {
              if ( mem_stack_count > 1 ) {
                mem_stack_count -= 1;
                mem_pointer = mem_stack_count;
                max_input = false;
                Start_input = Start_Mode;
              }
            }
          }
          Beep__on();
          break;

        case 42:                 //    _*_
          if ( Debug_Level == 5 ) {
            Serial.println("_*_");
          }
          add_operation_to_mem( 2, '*' );
          if ( display_string[Expo_point] == '.' ) {
            temp_operation = 213;
          }
          break;

        case 43:                 //    _+_
          if ( Debug_Level == 5 ) {
            Serial.println("_+_");
          }
          add_operation_to_mem( 1, '+' );
          if ( display_string[Expo_point] == '.' ) {
            temp_operation = 216;
          }
          break;

        case 44:                 //    e()
          if ( Debug_Level == 5 ) {
            Serial.println("Tau()");
          }
          if ( Start_input > Input_Operation_0 ) {
            if ( Start_input < Display_Input_Error ) {
              First_operation = false;
              copy_stack_input_2_calc( mem_stack_count, mem_pointer );
              mem_pointer = mem_stack_count;
              Clear_String();
              Start_input = Input_Memory;
            }
          }
          if ( (Start_input < Input_Operation_0) || (Display_Error < Start_input) ) {
            mem_extra_test = 11;
            Memory_to_Input_Operation();
          }
          else {
            display_string[Memory_1] = '_';
            display_string[Memory_0] = '_';
          }
          Beep__on();
          break;

        case 45:                 //    _-_
          if ( Debug_Level == 5 ) {
            Serial.println("_-_");
          }
          add_operation_to_mem( 1, '-' );
          if ( display_string[Expo_point] == '.' ) {
            temp_operation = 212;
          }
          break;

        case 46:                 //    _._
          if ( Debug_Level == 5 ) {
            Serial.println("_._");
          }
          if ( Start_input == Display_M_Plus ) {
            Beep__on();
          }
          if ( Start_input == Display_Result ) {
            Start_input = Display_M_Plus;
            Beep__on();
          }
          if ( Start_input == Input_Operation_0 ) {
            if ( mem_pointer > 0 ) {
              Clear_String();
            }
          }
          if ( Start_input == Input_Memory ) {
            First_operation = false;
            max_input = false;
            mem_pointer = mem_stack_count;
            Clear_String();
            Start_input = Input_Mantisse;
          }
          if ( Start_input > Input_Operation_0 ) {
            if ( Start_input < Display_Input_Error ) {
              First_operation = false;
              copy_stack_input_2_calc( mem_stack_count, mem_pointer );
              mem_pointer = mem_stack_count;
              Clear_String();
              Start_input = Input_Mantisse;
            }
          }
          if ( Start_input == Input_Mantisse ) {
            if ( Point_pos == 0 ) {
              if ( Number_count < 8 ) {
                Put_input_Point();
                Beep__on();
              }
            }
            else {
              Beep__on();
            }
          }
          if ( Start_input == Input_Expo ) {
            Beep__on();
          }
          break;

        case 47:                 //    _/_
          if ( Debug_Level == 5 ) {
            Serial.println("_/_");
          }
          add_operation_to_mem( 2, ',' );
          if ( display_string[Expo_point] == '.' ) {
            temp_operation = 214;
          }
          break;

        case 48:                 //    _0_
          if ( Debug_Level == 5 ) {
            Serial.println(char(Switch_Code));
          }
          if ( Start_input > Input_Operation_0 ) {
            if ( Start_input < Display_Input_Error ) {
              First_operation = false;
              copy_stack_input_2_calc( mem_stack_count, mem_pointer );
              mem_pointer = mem_stack_count;
              Clear_String();
              Start_input = Input_Mantisse;
            }
          }
          if ( Start_input == Input_Mantisse ) {
            if ( Number_count < 8 ) {
              if ( Zero_count < 7 ) {
                if (( Number_count > 0 ) or ( Point_pos > 0 )) {
                  Beep__on();
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
            Expo_change = true;
            display_string[Expo_1] = display_string[Expo_0];
            display_string[Expo_0] = Switch_Code;
          }
          if ( Start_input == Display_M_Plus ) {
            Result_to_Start_Mode();
            Put_input_Point();
          }
          if ( Start_input == Display_Result ) {
            Result_to_Start_Mode();
            Put_input_Point();
          }
          if ( Start_input == Input_Memory ) {
            First_operation = false;
            max_input = false;
            mem_pointer = mem_stack_count;
            Clear_String();
            Start_input = Input_Mantisse;
            Put_input_Point();
          }
          Beep__on();
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
          if ( Debug_Level == 5 ) {
            Serial.println(char(Switch_Code));
          }
          if ( Start_input == Display_M_Plus ) {
            Result_to_Start_Mode();
          }
          if ( Start_input == Display_Result ) {
            Result_to_Start_Mode();
          }
          if ( Start_input == Input_Memory ) {
            First_operation = false;
            max_input = false;
            mem_pointer = mem_stack_count;
            Clear_String();
            Start_input = Input_Mantisse;
          }
          if ( Start_input > Input_Operation_0 ) {
            if ( Start_input < Display_Input_Error ) {
              First_operation = false;
              copy_stack_input_2_calc( mem_stack_count, mem_pointer );
              mem_pointer = mem_stack_count;
              Clear_String();
              Start_input = Input_Mantisse;
            }
          }
          if ( Start_input == Input_Mantisse ) {
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
              Beep__on();
            }
          }
          if ( Start_input == Input_Expo ) {
            Expo_change = true;
            display_string[Expo_1] = display_string[Expo_0];
            display_string[Expo_0] = Switch_Code;
            display_string[Memory_1] = mem_str_1[mem_pointer];
            display_string[Memory_0] = mem_str_0[mem_pointer];
            Beep__on();
          }
          break;

        case 60:                 //    _<--_
          if ( Debug_Level == 5 ) {
            Serial.println("<--");
          }
          if ( Start_input == Input_Mantisse ) {
            if (Cursor_pos > 2) {
              Beep__on();
              Mantisse_change = true;
              display_string[Cursor_pos] = '#';
              --Cursor_pos;
              --Number_count;
              char_test = display_string[Cursor_pos];
              if ( char_test == '0' ) {
                --Zero_count;
              }
              if ( char_test == '.' ) {
                Point_pos = 0;
                ++Number_count;
                display_string[Cursor_pos] = '#';
              }
              if ( Number_count == 0 ) {
                Start_input = Start_Mode;
                break;
              }
              display_string[Cursor_pos] = '_';
              Beep__on();
            }
          }
          if ( Start_input == Input_Expo ) {
            display_string[Cursor_pos] = Pointer_memory;
            display_string[Expo_point] = ' ';
            Start_input = Input_Mantisse;
            Beep__on();
          }
          if ( Start_input > Input_Fraction ) {   // no Number Input
            Mantisse_change = false;
            Init_expo = false;
            mem_pointer = 0;

            if ( Start_input > Input_Operation_0 ) {
              First_operation = false;
              copy_stack_input_2_calc( mem_stack_count, mem_pointer );
              copy_input_left_right( mem_stack_count, mem_pointer );
              mem_pointer = mem_stack_count;
              First_operation = false;
            }

            if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) ) {
              mem_stack_count = 1;
              mem_pointer = mem_stack_count;
              copy_input_left_right( mem_pointer, 0 );
            }
            Start_input = Input_Mantisse;
            display_digit_temp = display_digit;
            display_digit = 8;
            Display_Number();
            display_string[Cursor_pos] = '.';
            display_digit = display_digit_temp;
            max_input = false;
            Beep__on();
          }
          break;

        case 61:                 //    _=_
          if ( Debug_Level == 5 ) {
            Serial.println("_=_");
          }
          if ( Debug_Level == 21 ) {
            Serial.print("mem_stack_count =  ");
            Serial.print(mem_stack_count);
            Serial.print(" --> ");
            Serial.println(Start_input);
          }
          if ( mem_stack_count == 1 ) {
            if ( Start_input > Input_Operation_0 ) {
              if ( Start_input < Display_Input_Error ) {
                Found_constant = true;
              }
            }
          }
          if ( mem_stack_count == 2 ) {
            Found_constant = true;
          }
          if ( Start_input == Input_Operation_0 ) {
            mem_extra_left_right( 0, mem_pointer );
          }
          if ( Start_input > Start_Mode ) {
            if ( Start_input < Input_Operation_0 ) {      // Input Number
              if ( Test_to_Result == true ) {
                Test_to_Result = false;
                Get_Number( 0 );
                mem_pointer = 0;
                Start_input = Input_Operation_0;
              }
            }
          }
          if ( Constant_arithmetic == false ) {
            Start_input = Input_Operation_0;
          }
          else {
            mem_pointer = 0;
          }
          if ( Debug_Level == 21 ) {
            Serial.print("mem_stack_count =  ");
            Serial.print(mem_stack_count);
            Serial.print(" --> ");
            Serial.println(Start_input);
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
            mem_stack_count -= 1;
          }
          if ( Constant_arithmetic == true ) {
            mem_stack_test = mem_stack_count;
          }          
          if ( Start_input > Input_Operation_0 ) {
            Input_Operation_2_function();
          }
          mem_stack_calc[mem_stack_count].op = 61;
          if ( (Start_input == Display_Result) ) {
            if ( Debug_Level == 21 ) {
              Serial.println("--xxxx--");
            }          	
            mem_pointer = mem_stack_count;
            mem_stack_count -= 1;
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
            if ( Start_input != Display_Error ) {
              Start_input = Display_Result;
              mem_save = true;
              mem_pointer = 0;
              mem_stack_count = 1;
              calc_stack(mem_stack_test);
              Display_Number();
              mem_extra_test = 0;
              max_input = false;
            }
          }
          break;

        case 64:                 //    _Rad_
          if ( Debug_Level == 5 ) {
            Serial.println("Rad");
          }
          if ( Rad_in_out == false ) {
            if ( Start_input < Input_Memory ) {      // Input Number
              Get_Number( 0 );
            }
            if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) ) {
              mem_stack_count = 1;
              mem_pointer = mem_stack_count;
              copy_input_left_right( mem_pointer, 0 );
              Start_input = Input_Operation_0;
            }
            if ( (Start_input == Input_Operation_0) || (Start_input == Input_Memory) ) {
              Error_Test();
              mem_pointer = mem_stack_count;
              if ( Start_input != Display_Error ) {
                mem_pointer = 0;
                Rad_in_out = true;
                mem_stack_input[ mem_pointer ] = mul(mem_stack_input[ mem_pointer ], to_xx[ 13 ]);
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
                  Display_Number();
                }
              }
            }
            if ( Start_input > Input_Operation_0 ) {
              if ( Start_input < Display_Input_Error ) {
                display_string[Memory_1] = '_';
                display_string[Memory_0] = '_';
              }
            }
          }
          else {
            display_string[Memory_1] = '_';
            display_string[Memory_0] = '_';
          }
          Beep__on();
          break;

        case 65:                 //    sin()
          if ( Debug_Level == 5 ) {
            Serial.println("sin()");
          }
          Beep__on();
          break;

        case 66:                 //    cos()
          if ( Debug_Level == 5 ) {
            Serial.println("cos()");
          }
          Beep__on();
          break;

        case 67:                 //    tan()
         if ( Debug_Level == 5 ) {
            Serial.println("tan()");
          }
          Beep__on();
          break;

        case 68:                 //    asin()
          if ( Debug_Level == 5 ) {
            Serial.println("asin()");
          }
          Beep__on();
          break;

        case 69:                 //    acos()
          if ( Debug_Level == 5 ) {
            Serial.println("acos()");
          }
          Beep__on();
          break;

        case 70:                 //    atan()
          if ( Debug_Level == 5 ) {
            Serial.println("atan()");
          }
          Beep__on();
          break;

        case 71:                 //    sinh()
          if ( Debug_Level == 5 ) {
            Serial.println("sinh()");
          }
          Beep__on();
          break;

        case 72:                 //    cosh()
          if ( Debug_Level == 5 ) {
            Serial.println("cosh()");
          }
          Beep__on();
          break;

        case 73:                 //    tanh()
          if ( Debug_Level == 5 ) {
            Serial.println("tanh()");
          }
          Beep__on();
          break;

        case 74:                 //    asinh()
          if ( Debug_Level == 5 ) {
            Serial.println("asinh()");
          }
          Beep__on();
          break;

        case 75:                 //    acosh()
          if ( Debug_Level == 5 ) {
            Serial.println("acosh()");
          }
          Beep__on();
          break;

        case 76:                 //    atanh()
          if ( Debug_Level == 5 ) {
            Serial.println("atanh()");
          }
          Beep__on();
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
          if ( Debug_Level == 5 ) {
            Serial.print("MR(");
            Serial.print(mem_extra_test);
            Serial.println(")");
          }
          if ( Start_input > Input_Operation_0 ) {
            if ( Start_input < Display_Input_Error ) {
              First_operation = false;
              copy_stack_input_2_calc( mem_stack_count, mem_pointer );
              mem_pointer = mem_stack_count;
              Clear_String();
              Start_input = Input_Memory;
            }
          }
          if ( (Start_input < Input_Operation_0) || (Display_Error < Start_input) ) {
            Memory_to_Input_Operation();
            Beep__on();
          }
          break;

        case 87:                 //    FIX_a_b/c
          if ( Debug_Level == 5 ) {
            Serial.println("FIX_a_b/c");
          }
          Beep__on();
          break;

        case 88:                 //    y_expo
          if ( Debug_Level == 5 ) {
            Serial.println("y_expo");
          }
          add_operation_to_mem( 4, '%' );
          if ( display_string[Expo_point] == '.' ) {
            temp_operation = 217;
          }
          break;

        case 89:                 //    y_root
          if ( Debug_Level == 5 ) {
            Serial.println("y_root");
          }
          add_operation_to_mem( 4, 'R' );
          if ( display_string[Expo_point] == '.' ) {
            temp_operation = 218;
          }
          break;

        case 90:                 //    _CE_
          if ( Debug_Level == 5 ) {
            Serial.println("_CE_");
          }
          if ( Start_input == Off_Status ) {
            Pendular_on = false;
            Beep__on();
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
              Beep__on();
              if ( mem_pointer == 0 ) {
                Start_input = Input_Operation_0;
              }
              else {
                Start_input = Start_Mode;
              }
            }
            else {
              if ( mem_stack_count > 1 ) {
                mem_stack_count -= 1;
                max_input = false;
                Start_input = Input_Operation_0;
              }
            }
          }
          if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) ) {
          	Constant_arithmetic = false;
          	Found_constant = false;
            First_operation = false;
            mem_stack_count = 1;
            mem_pointer = mem_stack_count;
            Start_input = Start_Mode;
            Beep__on();
          }
          if ( Start_input > Input_Operation_0 ) {
            if ( Start_input < Display_Input_Error ) {
              First_operation = false;
              copy_stack_input_2_calc( mem_stack_count, mem_pointer );
              mem_pointer = mem_stack_count;
              Clear_String();
              copy_input_left_right( mem_stack_count, mem_stack_count - 1 );
              Start_input = Input_Operation_0;
            }
          }
          if ( Start_input > Input_Memory ) {
            if ( Start_input <= Input_Operation_0 ) {
              Mantisse_change = false;
              mem_pointer = mem_stack_count;

              if ( Start_input > Input_Operation_0 ) {
                copy_input_left_right( mem_stack_count, mem_pointer );
                copy_stack_input_2_calc( mem_stack_count, mem_pointer );
              }

              Start_input = Input_Mantisse;
              Init_expo = false;
              display_digit_temp = display_digit;
              display_digit = 8;
              mem_stack_input[ mem_pointer ].op = temp_op;
              max_input = false;
              Display_Number();
              display_string[Cursor_pos] = '.';
              while ( display_string[Cursor_pos - 1] == '0' ) {
                display_string[Cursor_pos] = '#';
                --Cursor_pos;
                --Number_count;
                --Zero_count;
                display_string[Cursor_pos] = '_';
              }
              Beep__on();
              display_digit = display_digit_temp;
            }
          }
          break;

        case 92:                 //    _//_
          if ( Debug_Level == 5 ) {
            Serial.println("_//_");
          }
          add_operation_to_mem( 3, '|' );
          if ( display_string[Expo_point] == '.' ) {
            temp_operation = 215;
          }
          break;

        case 94:                 //    _/p/_  Phytagoras
          if ( Debug_Level == 5 ) {
            Serial.println("_/p/_");
          }
          add_operation_to_mem( 3, 'P' );
          if ( display_string[Expo_point] == '.' ) {
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
          if ( Debug_Level == 5 ) {
            Serial.print("FIX_");
            Serial.println(fix_extra_test);
          }
          if ( Start_input < Input_Memory ) {  // Input Number
            Get_Number( 0 );
          }
          mem_pointer = 0;                           // Display Number
          display_digit = fix_extra_test;
          Display_Number();
          display_string[Memory_1] = Display_Memory_1[5];
          display_string[Memory_0] = Display_Memory_0[5];
          Beep__on();
          break;

        case 102:                //    FIX_E24
          if ( Debug_Level == 5 ) {
            Serial.println("FIX_E24");
          }
          Beep__on();
          break;

        case 103:                //    M_plus(1)
        case 104:                //    M_plus(2)
        case 105:                //    M_plus(3)
        case 106:                //    M_plus(4)
        case 107:                //    M_plus(5)
        case 108:                //    M_plus(6)
        case 109:                //    M_plus(7)
        case 110:                //    M_plus(8)
        case 111:                //    M_plus(9)
          mem_extra_test = Switch_Code - M_plus_0;
          if ( Debug_Level == 5 ) {
            Serial.print("M_plus(");
            Serial.print(mem_extra_test);
            Serial.println(")");
          }
          if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) ) {
            mem_stack_count = 1;
            mem_pointer = mem_stack_count;
            Start_input = Input_Operation_0;
            left_right_mem_extra( mem_pointer, 0 );
          }
          if ( Start_input == Input_Operation_0 ) {
            mem_pointer = 0;
            left_right_mem_extra( mem_pointer, 0 );

            mem_stack_input[ mem_pointer ] = add(mem_extra_stack[ mem_extra_test ], mem_extra_stack[ 0 ], 1);
            Error_Test();
            if ( Start_input != Display_Error ) {
              mem_extra_left_right( mem_extra_test, mem_pointer );
              Display_Number();
              Display_Memory_Plus();
            }
            else {
              mem_pointer = mem_stack_count;
            }
            Beep__on();

            if ( Debug_Level == 12 ) {
              Serial.print("= ___M+ 64bit___ ");
              Serial.print(mem_extra_stack[ mem_extra_test ].num);
              Serial.print(" / ");
              Serial.print(mem_extra_stack[ mem_extra_test ].denom);
              Serial.print(" x 10^ ");
              Serial.println(mem_extra_stack[ mem_extra_test ].expo);
            }
          }
          break;

        case 112:                //    ln(x)
          if ( Debug_Level == 5 ) {
            Serial.println("ln(x)");
          }
          Beep__on();
          break;

        case 113:                //    e^x
          if ( Debug_Level == 5 ) {
            Serial.println("e^x");
          }
          Beep__on();
          break;

        case 114:                //    2^x
          if ( Debug_Level == 5 ) {
            Serial.println("2^x");
          }
          Beep__on();
          break;

        case 115:                //    log(x)
          if ( Debug_Level == 5 ) {
            Serial.println("log(x)");
          }
          Beep__on();
          break;

        case 116:                //    10^x
          if ( Debug_Level == 5 ) {
            Serial.println("10^x");
          }
          Beep__on();
          break;

        case 117:                //    _x^2_
          if ( Debug_Level == 5 ) {
            Serial.println("x^2");
          }
          Function_1_number();
          break;

        case 118:                //    _sqrt()_
          if ( Debug_Level == 5 ) {
            Serial.println("sqrt()");
          }
          Function_1_number();
          break;

        case 119:                //    x!
          if ( Debug_Level == 5 ) {
            Serial.println("x!");
          }
          Beep__on();
          break;

        case 120:                //    _EE_
          if ( Debug_Level == 5 ) {
            Serial.println("EE");
          }
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
              Beep__on();
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
              display_string[Cursor_pos] = '#';
              display_string[Expo_point] = '.';
              Start_input = Input_Expo;
              if (Init_expo == true) {
                Init_expo = false;
                display_string[Expo_1] = '0';
                display_string[Expo_0] = '0';
              }
              Beep__on();
              break;
            }
            if ( Start_input == Input_Expo ) {
              display_string[Cursor_pos] = Pointer_memory;
              display_string[Expo_point] = ' ';
              Start_input = Input_Mantisse;
              Beep__on();
              break;
            }
          }
          break;

        case 124:                //    _HM_
          if ( Debug_Level == 5 ) {
            Serial.println("HM");
          }
          add_operation_to_mem( 4, 'H' );
          if ( display_string[Expo_point] == '.' ) {
            temp_operation = 219;
          }
          break;

        case 127:                 //    -->
          if ( Debug_Level == 5 ) {
            Serial.println("->>");
          }
          Beep__on();
          break;

        case 128:                //    _AM_
          if ( Debug_Level == 5 ) {
            Serial.println("AM");
          }
          add_operation_to_mem( 4, 'f' );
          if ( display_string[Expo_point] == '.' ) {
            temp_operation = 220;
          }
          break;

        case 129:                //    _GM_
          if ( Debug_Level == 5 ) {
            Serial.println("GM");
          }
          add_operation_to_mem( 4, 'q' );
          if ( display_string[Expo_point] == '.' ) {
            temp_operation = 221;
          }
          break;

        case 148:                 //    <<-->>
          if ( Debug_Level == 5 ) {
            Serial.println("<<-->>");
          }
          Beep__on();
          break;

        case 149:                 //    __/
          Print_Statepoint();
          if ( Debug_Level == 5 ) {
            Serial.println("__/");
          }
          Beep__on();
          break;

        case 150:                 //    ° ' ''
          if ( Debug_Level == 5 ) {
            Serial.println("o_-_=");
          }
          Beep__on();
          break;

        case 151:                 //    _AGM_
          if ( Debug_Level == 5 ) {
            Serial.println("AGM");
          }
          add_operation_to_mem( 4, 'A' );
          if ( display_string[Expo_point] == '.' ) {
            temp_operation = 222;
          }
          break;

        case 160:                 //   SM(MCs)
          if ( Debug_Level == 5 ) {
            Serial.println("MCs");
          }
          for ( index_mem = 1; index_mem < 10; index_mem += 1 ) {
            mem_extra_stack[ index_mem ].num = int32_max;
            mem_extra_stack[ index_mem ].denom = int32_max;
            mem_extra_stack[ index_mem ].expo = expo_min_input;
            mem_extra_stack[ index_mem ].op = temp_op;
          }
          Beep__on();
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
          if ( Debug_Level == 5 ) {
            Serial.print("Min(");
            Serial.print(mem_extra_test);
            Serial.println(")");
          }
          if ( Start_input < Input_Memory ) {      // Input Number
            if ( Number_count != Zero_count ) {
              if ( Mantisse_change == true ) {
                Get_Mantisse();
              }
              else {
                if ( Expo_change == true ) {
                  Get_Expo_change();
                }
              }
              if ( Start_input != Display_Error ) {
                copy_input_left_right( 0, mem_pointer );
                Start_input = Input_Memory;
              }
            }
          }
          if ( Start_input == Display_Result ) {
            mem_stack_count = 1;
            mem_pointer = mem_stack_count;
          }
          if ( (Start_input == Display_Result) || (Start_input == Input_Operation_0) || (Start_input == Input_Memory) ) {
            if ( Start_input == Display_Result ) {
              left_right_mem_extra( mem_pointer, 0 );
              mem_pointer = 0;
            }
            mem_extra_left_right( mem_extra_test, mem_pointer );
            mem_stack_input[ mem_pointer ].op = temp_op_;
            if ( max_input == false ) {
              mem_stack_input[ mem_pointer ].op += '_';
            }
            else {
              mem_stack_input[ mem_pointer ].op += 'x';
            }
            Start_input = Input_Memory;
            Display_Number();
            mem_save = true;
            display_string[Memory_1] = Display_Memory_1[3];  // m
            display_string[Memory_0] = '0' + mem_extra_test;
          }
          if ( Start_input > Input_Operation_0 ) {
            if ( Start_input < Display_Input_Error ) {
              display_string[Memory_1] = '_';
              display_string[Memory_0] = '_';
            }
          }
          Beep__on();
          break;

        case 170:                //    _Int_
          if ( Debug_Level == 5 ) {
            Serial.println("Int");
          }
          Function_1_number();
          break;

        case 171:                //    _Frac_
          if ( Debug_Level == 5 ) {
            Serial.println("Frac");
          }
          Function_1_number();
          break;

        case 172:                //    _x^3_
          if ( Debug_Level == 5 ) {
            Serial.println("x^3");
          }
          Function_1_number();
          break;

        case 173:                //    _cbrt()_
          if ( Debug_Level == 5 ) {
            Serial.println("cbrt()");
          }
          Beep__on();
          break;

        case 174:                //    _EE+1_
          if ( Debug_Level == 5 ) {
            Serial.println("EE+1");
          }
          if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) ) {
            if ( display_string[Point_pos - 1] > '/' ) {
              expo_temp_16 = Get_Expo();
              if ( expo_temp_16 < 99 ) {
                Init_expo = false;
                ++expo_temp_16;
                Put_Expo();
                display_string[Point_pos] = display_string[Point_pos - 1];
                --Point_pos;
                display_string[Point_pos] = '.';
                Beep__on();
              }
            }
          }
          break;

        case 175:                //    EE-1
          if ( Debug_Level == 5 ) {
            Serial.println("EE-1");
          }
          if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) ) {
            if ( display_string[Point_pos + 1] > '/' ) {
              expo_temp_16 = Get_Expo();
              if ( expo_temp_16 > -99 ) {
                Init_expo = false;
                --expo_temp_16;
                Put_Expo();
                display_string[Point_pos] = display_string[Point_pos + 1];
                ++Point_pos;
                display_string[Point_pos] = '.';
                Beep__on();
              }
            }
          }
          break;

        case 176:                //    Dis_Cha_Dir_on
          if ( Debug_Level == 5 ) {
            Serial.println("Dis_Cha_Dir_on");
          }
          Beep__on();
          Print_Statepoint_after();
          break;

        case 177:                //    Dis_Cha_Dir_off
          if ( Debug_Level == 5 ) {
            Serial.println("Dis_Cha_Dir_off");
          }
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
          if ( (Number_count != Zero_count) || (mem_save == true) || (mem_exchange == true) ) {
            mem_extra_test = Switch_Code - M_xch_0;
            if ( Debug_Level == 5 ) {
              Serial.print("M-xch(");
              Serial.print(mem_extra_test);
              Serial.println(")");
            }
            temp_num = mem_extra_stack[ mem_extra_test ].num;
            temp_denom = mem_extra_stack[ mem_extra_test ].denom;
            temp_expo = mem_extra_stack[ mem_extra_test ].expo;
            if ( Start_input < Input_Memory ) {      // Input Number
              if ( Mantisse_change == true ) {
                Get_Mantisse();
              }
              else {
                if ( Expo_change == true ) {
                  Get_Expo_change();
                }
              }
              if ( Start_input != Display_Error ) {
                copy_input_left_right( 0, mem_pointer );
                Start_input = Input_Memory;
              }
            }
            if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) ) {
              mem_stack_count = 1;
              mem_pointer = mem_stack_count;
              left_right_mem_extra( mem_pointer, 0 );
              Start_input = Input_Operation_0;
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
              mem_stack_input[ mem_pointer ].op = temp_op_;
              if ( max_input == false ) {
                mem_stack_input[ mem_pointer ].op += '_';
              }
              else {
                mem_stack_input[ mem_pointer ].op += 'x';
              }
              Display_Number();
              mem_exchange = true;
              mem_save = false;
              display_string[Memory_1] = Display_Memory_1[4];  // E
              display_string[Memory_0] = '0' + mem_extra_test;
              Beep__on();
            }
          }
          break;

        case 188:                //    Dis_Memory_X_off
          if ( Debug_Level == 5 ) {
            Serial.println("Dis_Memory_X_off");
          }
          break;

        case 190:                //    rnd(x)
          if ( Debug_Level == 5 ) {
            Serial.println("rnd(x)");
          }
          Beep__on();
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
          if ( Debug_Level == 5 ) {
            Serial.print("to_");
            Serial.println(to_extra_test);
          }
          if ( Start_input < Input_Memory ) {      // Input Number
            Get_Number( 0 );
          }
          if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) ) {
            mem_stack_count = 1;
            mem_pointer = mem_stack_count;
            copy_input_left_right( mem_pointer, 0 );
            Start_input = Input_Operation_0;
          }
          if ( (Start_input == Input_Operation_0) || (Start_input == Input_Memory) ) {
            Start_input = Input_Operation_0;
            Error_Test();
            mem_pointer = mem_stack_count;
            if ( Start_input != Display_Error ) {
              mem_pointer = 0;
              if ( to_extra_test == 9 ) {
                mem_stack_input[ mem_pointer ] = add(mem_stack_input[ mem_pointer ], to_xx[ 11 ], 1);
              }
              if ( to_extra_test == 8 ) {
                mem_stack_input[ mem_pointer ] = add(mem_stack_input[ mem_pointer ], to_xx[ 10 ], 1);
              }
              mem_stack_input[ mem_pointer ] = mul(mem_stack_input[ mem_pointer ], to_xx[ to_extra_test ]);
              Error_Test();
              mem_pointer = mem_stack_count;
              if ( Start_input != Display_Error ) {
                mem_pointer = 0;
                mem_stack_input[ mem_pointer ].op = temp_op_;
                if ( max_input == false ) {
                  mem_stack_input[ mem_pointer ].op += '_';
                }
                else {
                  mem_stack_input[ mem_pointer ].op += 'x';
                }
                Display_Number();
                display_string[Memory_1] = Display_Memory_1[12];  // =
                display_string[Memory_0] = '0' + to_extra_test;
                to_extra = true;
              }
            }
          }
          if ( Start_input > Input_Operation_0 ) {
            if ( Start_input < Display_Input_Error ) {
              display_string[Memory_1] = '_';
              display_string[Memory_0] = '_';
            }
          }
          Beep__on();
          break;

        case 201:                //    Beep
          if ( Debug_Level == 5 ) {
            Serial.println("Beep");
          }
          Switch_Code = 0;
          Beep__on();
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
          if ( Debug_Level == 5 ) {
            Serial.print("EE(");
            Serial.print(to_extra_test);
            Serial.println(")");
          }
          if ( Start_input == Input_Expo ) {
            display_string[Plus_Minus_Expo] = '#';
            display_string[Expo_1] = '0';
            switch (to_extra_test) {

              case 1:
                display_string[Plus_Minus_Expo] = '-';
              case 9:
                display_string[Expo_1] = '1';
                display_string[Expo_0] = '2';
                break;

              case 2:
                display_string[Plus_Minus_Expo] = '-';
              case 8:
                display_string[Expo_0] = '9';
                break;

              case 3:
                display_string[Plus_Minus_Expo] = '-';
              case 7:
                display_string[Expo_0] = '6';
                break;

              case 4:
                display_string[Plus_Minus_Expo] = '-';
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
            Beep__on();
          }
          break;

        case 211:                 //    _/p/ /p/_
          if ( Debug_Level == 5 ) {
            Serial.println("_/p/ /p/_");
          }
          Switch_Code = 0;
          break;

        case 212:                 //    _- -_
          if ( Debug_Level == 5 ) {
            Serial.println("_- -_");
          }
          Switch_Code = 0;
          break;

        case 213:                 //    _* *_
          if ( Debug_Level == 5 ) {
            Serial.println("_* *_");
          }
          Switch_Code = 0;
          break;

        case 214:                 //    _/ /_
          if ( Debug_Level == 5 ) {
            Serial.println("_/ /_");
          }
          Switch_Code = 0;
          break;

        case 215:                 //    _// //_
          if ( Debug_Level == 5 ) {
            Serial.println("_// //_");
          }
          Switch_Code = 0;
          break;

        case 216:                 //    _+ +_
          if ( Debug_Level == 5 ) {
            Serial.println("_+ +_");
          }
          Switch_Code = 0;
          break;

        case 217:                 //    _y_expo y_expo_
          if ( Debug_Level == 5 ) {
            Serial.println("_y_expo y_expo_");
          }
          Switch_Code = 0;
          break;

        case 218:                 //    _y_root y_root_
          if ( Debug_Level == 5 ) {
            Serial.println("_y_root y_root_");
          }
          Switch_Code = 0;
          break;

        case 219:                 //    _HM HM_
          if ( Debug_Level == 5 ) {
            Serial.println("_HM HM_");
          }
          Switch_Code = 0;
          break;

        case 220:                 //    _AM AM_
          if ( Debug_Level == 5 ) {
            Serial.println("_AM AM_");
          }
          Switch_Code = 0;
          break;

        case 221:                 //    _GM GM_
          if ( Debug_Level == 5 ) {
            Serial.println("_GM GM_");
          }
          Switch_Code = 0;
          break;

        case 222:                 //    _AGM AGM_
          if ( Debug_Level == 5 ) {
            Serial.println("_AGM AGM_");
          }
          Switch_Code = 0;
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
        }
        else {
          Switch_Code = 0;
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

      Beep__on();
      Print_Statepoint_after();
    }
  }

  if ( Display_new == true ) { // Display refresh
    index_LED = 0;
    for ( index_a = 0; index_a < Digit_Count; index_a += 1 ) {
      if ( display_string[index_LED] == '.' ) {
        --index_a;
        display_b[index_a] = display_b[index_a] + led_font[( display_string[index_LED] - start_ascii )] ;
      }
      else {
        display_b[index_a] = led_font[( display_string[index_LED] - start_ascii )] ;
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
          --index_a;
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

  if ( time_10ms == true ) {   // here al that will make in 95 Hz
    time_10ms = false;
    ++count_10ms;              // (10 + 30/57) ms
    ++index_10ms;

    if ( Countdown_OFF > 0 ) {
      --Countdown_OFF;


      switch (Countdown_OFF) { // Off

        case Countdown_Off_0 - 1:
          reboot();
          break;

        case Countdown_Off_0:
          pinMode(On_Off_PIN, OUTPUT);
          break;

        case Countdown_Off_0 + 1:
          pinMode(On_Off_PIN, INPUT_PULLUP);
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
          Display_new = true;
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
          Display_new = false;
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
          Display_new = false;
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
      Serial.println(taste[4]);
    }

    if ( Display_Status_new != Display_Status_old ) {
      if ( Switch_down == 0 ) {
        Display_Status_new = 0;
        Mr_0_test = false;
      }

      display_string[Memory_1] = mem_str_1[mem_pointer];
      display_string[Memory_0] = mem_str_0[mem_pointer];

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

      switch (Display_Status_new) {   // Display --> Display_Status_new

        case 24:       // MR
          if ( (Start_input < Input_Operation_0) || (Display_Error < Start_input) ) {
            display_string[Memory_1] = Display_Memory_1[2];  // MR
            display_string[Memory_0] = Display_Memory_0[2];
            if ( Mr_0_test == true ) {
              display_string[Memory_0] = '0';
            }
          }
          else {
            display_string[Memory_1] = '_';
            display_string[Memory_0] = '_';
          }
          Beep__on();
          break;

        case 44:       // MCs
          display_string[Memory_1] = Display_Memory_1[2];      // MR
          display_string[Memory_0] = 'c';
          Beep__on();
          break;

        case 48:       // MS
          if ( (Number_count != Zero_count) || (mem_save == true) ) {
            display_string[Memory_1] = Display_Memory_1[3];      // MS
            display_string[Memory_0] = Display_Memory_0[3];
          }
          else {
            display_string[Memory_1] = '_';
            display_string[Memory_0] = '_';
          }

          if ( Start_input == Display_M_Plus ) {
            display_string[Memory_1] = Display_Memory_1[3];      // MS
            display_string[Memory_0] = Display_Memory_0[3];
            Start_input = Display_Result;
            mem_extra_left_right( 0, 0 );
          }
          Beep__on();
          break;

        case 2:       // MEx
          if ( Start_input == Display_M_Plus ) {
            Start_input = Display_Result;
          }
          if ( Start_input == Display_Result ) {
            mem_save = true;
          }
          if ( (Number_count != Zero_count) || (mem_save == true) || (mem_exchange == true) ) {
            display_string[Memory_1] = Display_Memory_1[4];
            display_string[Memory_0] = Display_Memory_0[4];
          }
          else {
            display_string[Memory_1] = '_';
            display_string[Memory_0] = '_';
          }
          if ( mem_exchange == true ) {
            display_string[Memory_0] = '0' + mem_extra_test;
          }
          Beep__on();
          break;

        case 1:       // EE to ..
          if ( Start_input == Input_Expo ) {
            display_string[Memory_1] = Display_Memory_1[15];
            display_string[Memory_0] = Display_Memory_0[15];
          }
          else {
            display_string[Memory_1] = '_';
            display_string[Memory_0] = '_';
          }
          if ( expo_exchange == true ) {
            display_string[Memory_0] = '0' + to_extra_test;
          }
          Beep__on();
          break;


        case 40:      // Display
          display_string[Memory_1] = Display_Memory_1[5];
          display_string[Memory_0] = Display_Memory_0[5];
          Beep__on();
          break;

        case 8:       // Invers
          display_string[Memory_1] = Display_Memory_1[6];
          display_string[Memory_0] = Display_Memory_0[6];
          if ( Switch_down == 3) {
            Switch_Code = 120;
          }
          if ( (Start_input < Input_Operation_0) || (Display_Error < Start_input) ) {
            if ( mem_save == true ) {
              display_string[Memory_1] = Display_Memory_1[2];    // MR
              display_string[Memory_0] = '0' + mem_extra_test;
            }
          }
          Beep__on();
          break;

        case 16:      // Fn
          display_string[Memory_1] = Display_Memory_1[7];
          display_string[Memory_0] = Display_Memory_0[7];
          if ( (Start_input < Input_Operation_0) || (Display_Error < Start_input) ) {
            if ( mem_save == true ) {
              display_string[Memory_1] = Display_Memory_1[2];    // MR
              display_string[Memory_0] = '0' + mem_extra_test;
            }
          }
          Beep__on();
          break;

        case 32:      // =
          display_string[Memory_1] = Display_Memory_1[8];
          display_string[Memory_0] = Display_Memory_0[8];
          if ( Start_input < Display_Result ) {
            display_string[Memory_1] = mem_str_1[mem_pointer];
            display_string[Memory_0] = mem_str_0[mem_pointer];
          }
          if ( Start_input == Display_Result ) {
            display_string[Memory_1] = Display_Memory_1[9];
            display_string[Memory_0] = Display_Memory_0[9];
          }
          if ( Start_input == Display_M_Plus ) {
            display_string[Memory_1] = Display_Memory_1[3];      // MS
            display_string[Memory_0] = '0' + mem_extra_test;
          }
          Beep__on();
          break;

        case 152:     // Off
          display_string[Memory_1] = Display_Memory_1[10];
          display_string[Memory_0] = Display_Memory_0[10];
          Beep__on();
          break;

        case 4:       // -->]  to ..
          display_string[Memory_1] = Display_Memory_1[12];
          display_string[Memory_0] = Display_Memory_0[12];
          if ( to_extra == true ) {
            display_string[Memory_0] = '0' + to_extra_test;
          }
          Beep__on();
          break;

        case 3:       // °"
          display_string[Memory_1] = Display_Memory_1[13];
          display_string[Memory_0] = Display_Memory_0[13];
          Beep__on();
          break;

        case 6:       // _,_/
          display_string[Memory_1] = Display_Memory_1[14];
          display_string[Memory_0] = Display_Memory_0[14];
          Beep__on();
          break;

        default:
          if ( Start_input == Display_Result ) {
            display_string[Memory_1] = Display_Memory_1[8];
            display_string[Memory_0] = Display_Memory_0[8];
            Beep__on();
          }
          if ( Start_input == Display_M_Plus ) {
            display_string[Memory_1] = Display_Memory_1[3];      // MS
            display_string[Memory_0] = '0' + mem_extra_test;
            Beep__on();
          }
          if ( Start_input == Input_Memory ) {
            display_string[Memory_1] = Display_Memory_1[2];     // MR
            display_string[Memory_0] = '0' + mem_extra_test;
            Beep__on();
          }

      }

      if ( Start_input > Input_Operation_0 ) {
        if ( Start_input < Display_Input_Error ) {
          if  ( (Display_Status_new !=  8)
             && (Display_Status_new != 16)
             && (Display_Status_new != 40) ) {   // EE  FN  ][
            display_string[Memory_1] = '_';
            display_string[Memory_0] = '_';
            Beep__on();
            if ( Display_Status_new == 24 ) {
              display_string[Memory_1] = Display_Memory_1[2];   // MR
              display_string[Memory_0] = Display_Memory_0[2];
            }
          }
        }
      }

      if ( Start_input == Off_Status ) {
        display_string[Memory_1] = Display_Memory_1[10];     // Off
        display_string[Memory_0] = Display_Memory_0[10];
      }

      if ( Display_Status_old == 255 ) {
        Beep_on = false;
      }
      if ( Display_Status_old > Display_Status_new ) {
        Beep_on = false;

        switch (Display_Status_old) {

          case 12:
          case  9:
          case  5:
            if ( to_extra == true ) {
              if ( Display_Status_new == 0 ) {
                to_extra = false;
              }
            }
            if ( expo_exchange == true ) {
              if ( Display_Status_new == 0 ) {
                expo_exchange = false;
              }
            }
            break;

          case 1:    // EE to ..
            display_string[Memory_1] = mem_str_1[mem_pointer];
            display_string[Memory_0] = mem_str_0[mem_pointer];
            if ( Display_Status_new == 0 ) {
              if ( expo_exchange == true ) {
                display_string[Memory_1] = Display_Memory_1[15];  // X;
                display_string[Memory_0] = '0' + to_extra_test;
                expo_exchange = false;
              }
            }
            break;

          case 2:   //  MEx
            if ( Start_input == Input_Memory ) {
              Start_input = Input_Operation_0;
              if ( Display_Status_new == 0 ) {
                display_string[Memory_1] = Display_Memory_1[4];  // E;
                if ( mem_exchange == false ) {
                  display_string[Memory_1] = ' ';    //
                  display_string[Memory_0] = ' ';    //
                }
              }
            }
            else {
              display_string[Memory_1] = mem_str_1[mem_pointer];
              display_string[Memory_0] = mem_str_0[mem_pointer];
            }
            break;

          case  4:   // -->]  to ..
            display_string[Memory_1] = mem_str_1[mem_pointer];
            display_string[Memory_0] = mem_str_0[mem_pointer];
            if ( Display_Status_new == 0 ) {
              if ( to_extra == true ) {
                display_string[Memory_1] = Display_Memory_1[12];  // =
                display_string[Memory_0] = '0' + to_extra_test;
                to_extra = false;
              }
            }
            break;

          case  8:   //  MR
          case 16:   //  MS + MR
          case 24:   //  MR
          case 32:   //  MS
          case 48:   //  MS
            display_string[Memory_1] = mem_str_1[mem_pointer];
            display_string[Memory_0] = mem_str_0[mem_pointer];

            if ( Display_Status_new == 32 ) {
              if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) ) {
                display_string[Memory_1] = Display_Memory_1[3];      // MS
                display_string[Memory_0] = Display_Memory_0[3];
              }
            }

            if ( mem_save == true ) {
              display_string[Memory_1] = Display_Memory_1[3];  // m
              display_string[Memory_0] = '0' + mem_extra_test;
            }
            else {
              Display_Status_new = 0;
            }

            if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) ) {
              display_string[Memory_1] = Display_Memory_1[3];  // m
              display_string[Memory_0] = '0' + mem_extra_test;
            }

            break;
        }

        if ( Start_input == Display_Result ) {
          display_string[Memory_1] = Display_Memory_1[8];  // =
          display_string[Memory_0] = Display_Memory_0[8];  // =
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

/// --------------------------
/// Timer ISR Timer Routine
/// --------------------------
void timerIsr() {
uint16_t temp_pwm = test_pwm;

  ++index_Switch;
  ++index_TIME;

  switch (index_TIME) {

    case 2:       // +32
    case 34:      // +32
    case 65:      // +31
      Timer1.setPeriod(Time_HIGH);  // sets timer1 to a period of 264 microseconds
      break;

    case 7:       //  5x
    case 39:      //  5x
    case 70:      //  5x
      Timer1.setPeriod(Time_LOW);   // sets timer1 to a period of 263 microseconds
      break;

    case 94:
      index_TIME = 255;
      break;
  }

  index_Display_old = index_Display;

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
      digitalWrite(Out_A, HIGH);
      index_Display = 1;
      Display_on();
    case 1:
    case 4:
    case 43:
    case 82:
      Switch_number = 0;
      Switch_read = analogRead(A_0);
      break;

    case 5:             //  1
      digitalWrite(Out_A, HIGH);
      index_Display = 1;
      Display_on();
    case 2:
    case 41:
    case 44:
    case 83:
      Switch_number = 1;
      Switch_read = analogRead(A_1);
      break;

    case 45:            //  1
      digitalWrite(Out_A, HIGH);
      index_Display = 1;
      Display_on();
    case 3:
    case 42:
    case 81:
    case 84:
      Switch_number = 2;
      Switch_read = analogRead(A_2);
      break;

    case 10:            //  2
      digitalWrite(Out_B, HIGH);
      index_Display = 2;
      Display_on();
    case 7:
    case 46:
    case 49:
    case 88:
      Switch_number = 3;
      Switch_read = analogRead(A_0);
      break;

    case 50:            //  2
      digitalWrite(Out_B, HIGH);
      index_Display = 2;
      Display_on();
    case 8:
    case 47:
    case 86:
    case 89:
      Switch_number = 4;
      Switch_read = analogRead(A_1);
      break;

    case 90:            //  2
      digitalWrite(Out_B, HIGH);
      index_Display = 2;
      Display_on();
    case 6:
    case 9:
    case 48:
    case 87:
      Switch_number = 5;
      Switch_read = analogRead(A_2);
      break;

    case 55:            //  3
      digitalWrite(Out_A, LOW);
      index_Display = 3;
      Display_on();
    case 13:
    case 52:
    case 91:
    case 94:
      Switch_number = 6;
      Switch_read = analogRead(A_0);
      break;

    case 95:            //  3
      digitalWrite(Out_A, LOW);
      index_Display = 3;
      Display_on();
    case 11:
    case 14:
    case 53:
    case 92:
      Switch_number = 7;
      Switch_read = analogRead(A_1);
      break;

    case 15:            //  3
      digitalWrite(Out_A, LOW);
      index_Display = 3;
      Display_on();
    case 12:
    case 51:
    case 54:
    case 93:
      Switch_number = 8;
      Switch_read = analogRead(A_2);
      break;

    case 100:           //  4
      digitalWrite(Out_C, HIGH);
      index_Display = 4;
      Display_on();
    case 16:
    case 19:
    case 58:
    case 97:
      Switch_number = 9;
      Switch_read = analogRead(A_0);
      break;

    case 20:            //  4
      digitalWrite(Out_C, HIGH);
      index_Display = 4;
      Display_on();
    case 17:
    case 56:
    case 59:
    case 98:
      Switch_number = 10;
      Switch_read = analogRead(A_1);
      break;

    case 60:            //  4
      digitalWrite(Out_C, HIGH);
      index_Display = 4;
      Display_on();
    case 18:
    case 57:
    case 96:
    case 99:
      Switch_number = 11;
      Switch_read = analogRead(A_2);
      break;

    case 25:            //  5
      digitalWrite(Out_A, HIGH);
      index_Display = 5;
      Display_on();
    case 22:
    case 61:
    case 64:
    case 103:
      Switch_number = 12;
      Switch_read = analogRead(A_0);
      break;

    case 65:            //  5
      digitalWrite(Out_A, HIGH);
      index_Display = 5;
      Display_on();
    case 23:
    case 62:
    case 101:
    case 104:
      Switch_number = 13;
      Switch_read = analogRead(A_1);
      break;

    case 105:           //  5
      digitalWrite(Out_A, HIGH);
      index_Display = 5;
      Display_on();
    case 21:
    case 24:
    case 63:
    case 102:
      Switch_number = 14;
      Switch_read = analogRead(A_2);
      break;

    case 70:            //  6
      digitalWrite(Out_B, LOW);
      index_Display = 6;
      Display_on();
    case 28:
    case 67:
    case 106:
    case 109:
      Switch_number = 15;
      Switch_read = analogRead(A_0);
      break;

    case 110:           //  6
      digitalWrite(Out_B, LOW);
      index_Display = 6;
      Display_on();
    case 26:
    case 29:
    case 68:
    case 107:
      Switch_number = 16;
      Switch_read = analogRead(A_1);
      break;

    case 30:            //  6
      digitalWrite(Out_B, LOW);
      index_Display = 6;
      Display_on();
    case 27:
    case 66:
    case 69:
    case 108:
      Switch_number = 17;
      Switch_read = analogRead(A_2);
      break;

    case 115:           //  7
      digitalWrite(Out_A, LOW);
      index_Display = 7;
      Display_on();
    case 31:
    case 34:
    case 73:
    case 112:
      Switch_number = 18;
      Switch_read = analogRead(A_0);
      break;

    case 35:            //  7
      digitalWrite(Out_A, LOW);
      index_Display = 7;
      Display_on();
    case 32:
    case 71:
    case 74:
    case 113:
      Switch_number = 19;
      Switch_read = analogRead(A_1);
      break;

    case 75:            //  7
      digitalWrite(Out_A, LOW);
      index_Display = 7;
      Display_on();
    case 33:
    case 72:
    case 111:
    case 114:
      Switch_number = 20;
      Switch_read = analogRead(A_2);
      break;

    case 40:            //  0
      digitalWrite(Out_C, LOW);
      index_Display = 0;
      Display_on();
      time_10ms = true;
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
      digitalWrite(Out_C, LOW);
      index_Display = 0;
      Display_on();
      time_10ms = true;
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
      digitalWrite(Out_C, LOW);
      index_Display = 0;
      Display_on();
      time_10ms = true;
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

  taste[Switch_number] -= taste[Switch_number] / Average;
  taste[Switch_number] += Switch_read / Average;

  if ( bitRead(Switch_new, Switch_number) == 0 ) {
    if ( bitRead(Switch_up, Switch_number) == 0 ) {  //  Pos. 0
      if (taste[Switch_number] < Switch_up_b) {
        bitWrite(Switch_up, Switch_number, 1);
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
      if ( Beep_count <= min_Beep_count ) {
        pinMode(Beep_m, INPUT_PULLUP); // Pin A3
        pinMode(Beep_p, INPUT_PULLUP); // Pin A7
        Beep_on = false;
        Beep_count = max_Beep_count;
      } 
    }
    else {      //  0 .. 63
      if ( Beep_on_off == true ) {
        if ( Beep_count > 63 ) {
          pinMode(Beep_m, OUTPUT);     // Pin A3
          digitalWrite(Beep_m, HIGH);
          pinMode(Beep_p, OUTPUT);     // Pin A7
          digitalWrite(Beep_p, HIGH);
        }
        else {
          digitalWrite( Beep_m, bitRead(Beep_patt_m, Beep_count) );    // Toggle Beep
          digitalWrite( Beep_p, bitRead(Beep_patt_p, Beep_count) );    // Toggle Beep
        }
      }
    }
  }
} 

