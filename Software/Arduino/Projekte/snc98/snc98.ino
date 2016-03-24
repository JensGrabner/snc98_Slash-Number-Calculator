/********************************************************************** 

   Project:   snc98 - Slash Number Calculator

   Developer: Jens Grabner
   Email:     jens@grabner-online.org
   Date:      March 2016

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

/*													HEADER

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

#include <stdlib.h>	      // for itoa(); ltoa(); call
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <TimerOne.h>
// #include <DigitalIO.h>

// #define Standard1284GpioPinMap_h

#define Debug_Level  0 //  0 - not Debug
                       //  1 - Test intern 1ms - Task by 100 ms
                       //  2 - Test intern 1ms - Task by 1000 ms
                       //  3 - Test Switch "=" up / down (analog)
                       //  4 - Test Switchnumber down (digital)
                       //  5 - Monitor Switch_Code (Text) 105 - Functions
                       //  6 - Test Pendulum Time - Sinus 0.5 Hz
                       //  7 - get_Expo
                       //  8 - get mem_stack "=" Get_Mantisse(); 
                       //  9 - Display_Number();
                       // 10 - x^2 Test;
                       // 11 - Error_Test;
                       // 12 - Memory_Plus Test;
                       
#define operation_test_max 4     //  0 .. 3  Stacktiefe
                                 //                     74HCF4053 (1 kOhm)
#define Switch_down_start  1020  //  1020  ... 1020  100  %   ... 1020  <<--
#define Switch_up_b         862  //   870             75  %   ...  862
#define Switch_down_b       516  //   540             20  %   ...  516
#define Switch_up_start     327  //   360  ...  420    0  %   ...  390  <<--
                                 //                  -10  %   ...  327
#define Average               5  //     5
	
#define Beep A0        //     Pin A7
                       //     Sanguino  Pin 13 --> Pin 5 (PD5)
#define PWM 13         //     Sanguino  Pin 12 --> Pin 4 (PD4)
#define DCF77 7        //     Pin 7

#define A_0 A7         //     Pin A0
#define A_1 A6         //     Pin A1
#define A_2 A5         //     Pin A2
#define Out_0 A4       //     Pin A3 not used

#define Out_A A3       //     Pin A4
#define Out_B A2       //     Pin A5
#define Out_C A1       //     Pin A6

#define On_Off_PIN    18

#define Min_Out        2   // D2 and D3 not usesd
#define Max_Out       23

#define Digit_Count   15
#define Switch_Count  24

#define count_ascii  112
#define start_ascii   16
#define ascii_count   27

uint16_t taste[Switch_Count] = {
  Switch_down_start
};

// ... up to 32 Switch possible - per bit a switch
uint32_t Switch_up     = 0;  // change Low --> High
uint32_t Switch_down   = 0;  // change High --> Low
uint32_t Switch_old    = 0;
uint32_t Switch_delta  = 0;

uint16_t Switch_test   = Switch_down_start;
uint16_t Switch_read   = Switch_down_start;  // analog
uint8_t  Switch_number = 0;

uint32_t time          = 0;
uint32_t time_down     = 0;
uint32_t time_old      = 0;

#define expo_max_input  99  
#define expo_min_input -99  

#define expo_max_in_3   33  
#define expo_min_in_3  -33  

#define expo_max       102  
#define expo_min       -99  

#define _PI_        36   
#define _e_         44  

#define SM_0      160 
#define RM_0        77 
#define FIX_0       93
#define M_plus_0   102
#define Mem_0      128 

/*
 * rational number "numerator / denominator"
 */
struct AVRational{  //     0.3 ... 3 x 10^expo
  int8_t  expo;     // <-- expo
  int32_t num;      // <-- numerator
  int32_t denom;    // <-- denominator
};

const uint32_t expo_10[10] = {
  1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000 };

#define expo_10_10           0x2540BE400ULL   // 10000000000
#define expo_10_11          0x174876E800ULL   // 100000000000
#define expo_10_12          0xE8D4A51000ULL   // 1000000000000
#define expo_10_13         0x9184E72A000ULL   // 10000000000000
#define expo_10_14        0x5AF3107A4000ULL   // 100000000000000
#define expo_10_15       0x38D7EA4C68000ULL   // 1000000000000000
#define expo_10_16      0x2386F26FC10000ULL   // 10000000000000000
#define expo_10_17     0x16345785D8A0000ULL   // 100000000000000000
#define expo_10_18     0xDE0B6B3A7640000ULL   // 1000000000000000000

const uint64_t expo_10_[9] = {
  expo_10_10, expo_10_11, expo_10_12, expo_10_13, expo_10_14, 
  expo_10_15, expo_10_16, expo_10_17, expo_10_18 };
                                              // 9223372036854775807
#define expo_test_9          0x2241B46A1ULL   //          9195701921 x 0.000000000997
#define expo_test_6        0x85D0A8BE3E8ULL   //       9195701920744 x 0.000000997
#define expo_test_3     0x20AB7132724313ULL   //    9195701920744211 x 0.000997
#define expo_test_2a    0x50EFDC9C4DA900ULL   //   22781728931031296 x 0.00247
#define expo_test_2    0x146B26BF8769EC3ULL   //   91957019207442115 x 0.00997
#define expo_test_1a   0x3295E9E1B089A02ULL   //  227817289310312962 x 0.0247
#define expo_test_1    0xCC2F837B4A2339CULL   //  919570192074421148 x 0.0997

#define int32_max    2147483647
#define int32_2_max       46341     // sqrt(int32_max)
#define int15_max         32767

// ---  sqrt(10)_Konstante  --- 
#define sqrt_10_expo            0;
#define sqrt_10_num    1499219281;
#define sqrt_10_denom   474094764;  // Fehler ..  7.03e-19

// ---  Pi_Konstante  --- 
#define Pi_expo                 0;
#define Pi_num         1068966896;
#define Pi_denom        340262731;  // Fehler .. -3.07e-18

// ---  e_Konstante  --- 
#define e_expo                  0;
#define e_num           848456353;
#define e_denom         312129649;  // Fehler .. -6.03e-19

// ---  _180_Pi_Konstante  --- 
#define _180_Pi_expo            2;
#define _180_Pi_num     853380389;
#define _180_Pi_denom  1489429756;  // Fehler ..  5.75e-20

char    expo_temp_str[]    = "#00";
int8_t  expo_temp_8        =  1;
/*
  union
  {
    uint64_t n64;
    struct
    {
 	    uint32_t denom_temp_u64_low;
 	    uint32_t denom_temp_u64_high;
      //  uint32_t a[2];  // 0 = LOW byte  1 = HIGH byte of the uint64_t
    }
  } nn;
*/
char    Expo_string_temp[] = "###" ;
int16_t expo_temp_16       = 0;
int16_t expo_temp_16_a     = 0;
int16_t expo_temp_16_b     = 0;
int16_t expo_temp_16_diff  = 0;
int16_t expo_temp_16_diff_abs  = 0;
int64_t num_temp_64_a      = 0;
int64_t num_temp_64_b      = 0;
int64_t denom_temp_64      = 1;
uint64_t num_temp_u64      = 1;
uint64_t num_temp_u64_0    = 1;
uint64_t denom_temp_u64    = 1;
uint64_t denom_temp_u64_0  = 1;
uint64_t p0 = 0;
uint64_t p1 = 1;
uint64_t q0 = 1;
uint64_t q1 = 0;
uint64_t a0 = 1;
int64_t gcd_temp_64        = 1;

int64_t calc_temp_64_a     = 1;
int64_t calc_temp_64_b     = 1;
int64_t calc_temp_64_c     = 1;
int64_t calc_temp_64_d     = 1;

uint64_t calc_temp_u64_0   = 1;
uint64_t calc_temp_u64_1   = 1;
uint64_t calc_temp_u64_2   = 1;

uint32_t num_temp_u32      = 1;
uint32_t denom_temp_u32    = 1;
uint32_t mul_temp_u32      = 1;
uint32_t calc_temp_u32     = 1;
int32_t mul_temp_32        = 1;
int32_t gcd_temp_32        = 1;
int32_t calc_temp_32_0     = 1;
int32_t calc_temp_32_1     = 1;
int32_t calc_temp_32_2     = 1;

int8_t  temp_expo      = 1;
int32_t temp_num       = 1;
int32_t temp_denom     = 1;

int16_t calc_temp_16_0 = 1;
int16_t calc_temp_16_1 = 1;
int16_t calc_temp_16_2 = 1;

int8_t  calc_temp_8_0  = 1;
int8_t  calc_temp_8_1  = 1;
int8_t  calc_temp_8_2  = 1;

int8_t  test_temp_8    = 0;
int8_t  test_signum_8  = 0;

// uint8_t operation_pointer    =   1;
// #define operation_stack_max    100
// uint8_t operation_stack[operation_stack_max] = {
//  0
// };

uint8_t display_digit      =  5;
uint8_t display_digit_temp =  5;

uint8_t mem_pointer        =  1;   // mem_stack 0 .. 19
#define mem_stack_max         20   // Variable in calculate
struct  AVRational
        mem_stack[mem_stack_max] = {
  { 0, int32_max, int32_max  },
  { 0, int32_max, int32_max  }
};
const char mem_str_1[]     = "##########1111111111";
const char mem_str_0[]     = "#1234567890123456789";

uint8_t mem_extra_pointer  =  0;   // mem_extra  RM 0 .. RM 9
uint8_t mem_extra_test     =  0;   // mem_extra  RM 0 .. RM 9
#define mem_extra_max        10    // mem_extra  SM 0 .. SM 9            
struct  AVRational
        mem_extra_stack[mem_extra_max] = {
  { 0, int32_max, int32_max },
  { 0, int32_max, int32_max },
  { 0, int32_max, int32_max },
  { 0, int32_max, int32_max },
  { 0, int32_max, int32_max },
  { 0, int32_max, int32_max },
  { 0, int32_max, int32_max },
  { 0, int32_max, int32_max },
  { 0, int32_max, int32_max },
  { 0, int32_max, int32_max }
};

#define led_bright_min    2
#define led_bright_max   10
#define led_bright_start  6  //  1 -  39 mA / 4,03 V
                             //  2 -  47 mA / 4,03 V   +  8 mA
                             //  3 -  58 mA / 4,03 V   + 11 mA   + 3 mA
                             //  4 -  72 mA / 4,02 V   + 14 mA   + 3 mA
                             //  5 -  89 mA / 4,02 V   + 17 mA   + 3 mA
                             //  6 - 108 mA / 4,01 V   + 19 mA   + 2 mA
                             //  7 - 126 mA / 4,00 V   + 18 mA   - 1 mA
                             //  8 - 144 mA / 3.99 V   + 18 mA     0 mA
                             //  9 - 162 mA / 3.98 V   + 18 mA     0 mA

uint8_t led_bright_index = led_bright_start;
uint16_t test_pwm = 117;

const uint16_t led_bright_plus[led_bright_max + 3] = {
  0, 76,  84, 100, 126,  165,  221,  318,  429,  562,  721,  909, 1128  };
//      8   16   26   39    56    97   111   133   159   188    219
//        8    10   13   17    41    14    22    26    29    31
//           2    3    4    24   -27     8      4    3    2
const uint16_t led_bright[led_bright_max + 3] = {
  0, 30,  42,  59,  83,  117,  165,  243,  339,  461,  613,  799, 1023  };
//     12   17   24   34    48    78    96   122   152   186    224
//        5    7   10    14    30    18    26    30    34    38
//           2    3    4    16   -12     8     4     4    4

#define beep_patt 0x993264C993264C99ULL   // 16.8421 ms -- 1128.125 Hz -- 19x Peak
//      1001100100110010011001001100100110010011001001100100110010011001 -- binaer
#define max_Beep_count 65

uint8_t Countdown_OFF = 0;
#define Countdown_Start      27   // 27
#define Countdown_Start_Off  39   // 39

uint8_t index_Switch  = 255;      // counter Switch-digit
uint8_t index_LED = 0;            // counter LED-digit
uint8_t index_Display = 0;        // counter Display-digit
uint8_t index_Display_old = 0;    // counter Display-digit_old
uint8_t index_a = 0;
uint8_t index_pendel_a = 0;       // 0 .. 189
uint8_t index_TIME = 255;         // counter Time
#define Time_LOW     263          // 263    t = (263 + 3/19) µs
#define Time_HIGH    264          // 264  1/t = 3800 Hz

boolean Beep_on = false;
boolean Beep_on_off = true;
int8_t Beep_count = max_Beep_count;

boolean Pendular_on = false;
boolean EE_FN_xx = false;
boolean SM_on = false;
boolean hyp_on = false;
boolean disp_on = false;

uint64_t beep_pattern = beep_patt;

uint16_t display_bright = led_bright_max;

const uint8_t led_font[count_ascii] = {
    0,  64,  68,  70,  71, 103, 119, 127, 255, 191, 187, 185, 184, 152, 136, 128,     //  ¦                ¦
    0, 107,  34,   0, 109,  18, 125,   2,  57,  15,  92,  66,  12,  64, 128,  82,     //  ¦ !"#$%&'()*+,-./¦
   63,   6,  91,  79, 102, 109, 124,   7, 127, 103,   9,  13,  88,  72,  76,  83,     //  ¦0123456789:;<=>?¦
  123, 119, 127,  57,  15, 121, 113,  61, 118,   6,  30, 122,  56,  85,  55,  27,     //  ¦@ABCDEFGHIJKLMNO¦
  115, 103,  49,  45,   7,  62,  42,  60,  73, 110,  27,  57, 100,  15,  35,   8,     //  ¦PQRSTUVWXYZ[\]^_¦
   32,  95, 124,  88,  94, 123, 113, 111, 116,   5,  14, 120,  24,  21,  84,  92,     //  ¦`abcdefghijklmno¦
  115, 103,  80, 108,  70,  28,  42, 106,  73, 102,  82,  57,  20,  15,   1,  54};    //  ¦pqrstuvwxyz{|}~ ¦

uint8_t index_display[Digit_Count] = {
  14, 15, 8, 9, 6, 5, 10, 11, 12, 13, 23, 22, 21, 20, 19
};

uint8_t display_a[Digit_Count] = {
  255
};

uint8_t display_b[Digit_Count] = {
  255
};
int8_t Digit = 0;
boolean Digit_Test = false;

boolean Deg_in_out = true;
boolean Rad_in_out = false;

char display_string[ ascii_count ]     = " -1.2345678#- 1 2 # # =.  " ;
const char string_start[ ascii_count ] = "  _########## # # # # #   " ;
// char display_string[]                  = " 8.8.8.8.8.8.8.8.8.8.8.8.8.8.8." ;
#define Plus_Minus           1
#define Mantisse_0           2
#define Mantisse_1           3
#define Plus_Minus_Expo__   10
#define Plus_Minus_Expo_    11
#define Plus_Minus_Expo     12
#define inv_point           13
#define Expo_1              14
#define STO_point           15
#define Expo_0              16     // 16
#define Expo_point          17     // 17
#define Operation           18     // 18
#define DCF77_point         19     // 19
#define Memory_1            20     // 20
#define Rad_point           21     // 21
#define Memory_0            22     // 22
#define Deg_point           23     // 23

char char_test;               // 0.. 127

uint8_t     index_mem = 255;  // 1..9  ... for CMs (Clear Memory's)

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

boolean Init_expo = true;
boolean Display_new = true;
boolean Display_change = false;
// boolean RM_0_change = false;

uint8_t Cursor_pos = 2;       //
uint8_t Point_pos = 0;        //
uint8_t Null_count = 0;       //
uint8_t Number_count_temp = 0;    
uint8_t Number_count = 0;     //   Anzahl der Ziffern  --  maximal 8
uint8_t Zero_count = 0;       //   Anzahl der Nullen   --  maximal 8
uint8_t Zero_after_Point = 0; //   Anzahl der Ziffern  --  maximal 7
uint8_t Zero_index = 0;
uint8_t Zero_index_a = 0;
char First_char;              //   0.. 127
char Temp_char[12] = "           ";
char Temp_char_expo[5] = "    ";

uint8_t PWM_Pin = PWM;

// Define various ADC prescaler
const unsigned char PS_16  = (1 << ADPS2);
const unsigned char PS_32  = (1 << ADPS2) |                (1 << ADPS0);
const unsigned char PS_64  = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

uint8_t Switch_Code = 0;
uint8_t Start_mem = 0;

boolean Mantisse_change = false;
boolean Expo_change = false;

#define First_Display       0
#define Start_Mode          1
#define Input_Mantisse      2    //    Input Number
#define Input_Expo          3    //    Input Number
#define Input_dms           4    //    Input Number
#define Input_Fraction      5    //    Input Number
#define Input_Operation_0   6    //   Input Operation  no Input
#define Input_Operation_1   7    //   Input Operation  "+"  "-"
#define Input_Operation_2   8    //   Input Operation  "*"  "/"
#define Input_Operation_3   9    //   Input Operation  "//"  "/p/"
#define Input_Operation_4   10   //   Input Operation  "sqrt(x,y)"  "x^y"  "AGM"
#define Input_Operation_5   11   //   Input Operation  "("  ")"
#define Display_Error       12   //  Display Number
#define Display_Result      13   //  Display Number
#define Display_M_Plus      14   //  Display Number
#define Display_Fraction_a_b    15   //  Display Number
#define Display_Fraction_a_b_c  15   //  Display Number
#define Display_dms             16   //  Display Number

uint8_t Start_input = First_Display;
char Input_Operation_char = '_';
char Pointer_memory = '_';

void Memory_to_Input_Mantisse() {
  if ( Start_input == Display_Result ) {
    Result_to_Start_Mode();
  }
  if ( Start_input == Display_M_Plus ) {
    Result_to_Start_Mode();
  }
  if ( Start_input < Input_Operation_0 ) {      // Input Number
    mem_stack[ 0 ].expo =  temp_expo;
    mem_stack[ 0 ].num = temp_num;
    mem_stack[ 0 ].denom = temp_denom;
    Start_input = Input_Operation_0;
  }
  if ( Start_input == Input_Operation_0 ) {
    if ( mem_pointer > 0 ) {
      mem_stack[ mem_pointer ].expo  = temp_expo;
      mem_stack[ mem_pointer ].num = temp_num;
      mem_stack[ mem_pointer ].denom = temp_denom;
      Start_input = Input_Mantisse;
      Display_Number();
      if ( display_digit == 8 ) {
        display_string[Cursor_pos] = '.'; 
      }
      else {
        display_string[Cursor_pos] = '_';
      }
      Mantisse_change = false;
      Expo_change = false;
      Init_expo = false;
      Beep_on = true;
    }
  }
}

void Result_to_Start_Mode() {
  mem_pointer = 1;
  Clear_String();
}

void Display_on() {     // Segmente einschalten --> Segment "a - f - point"
  for (Digit = 0; Digit < Digit_Count; ++Digit) {
    digitalWrite(index_display[Digit], !(bitRead(display_a[Digit], index_Display)));
  //  fastDigitalWrite(index_display[Digit], !(bitRead(display_a[Digit], index_Display)));
  }
  if ( index_Display == 7 ) {
    Timer1.pwm(PWM_Pin, led_bright[led_bright_index + 2]);    // duty cycle goes from 0 to 1023
  }
  if ( index_Display == 0 ) {
    Timer1.pwm(PWM_Pin, test_pwm);    // duty cycle goes from 0 to 1023
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
  while (v_0 != 0) {
    t_0 = u_0; 
    u_0 = v_0; 
    v_0 = t_0 % v_0;
  }                     // return u < 0 ? -u : u; /* abs(u) */
  return abs(u_0);
}

int64_t gcd_iter_64(int64_t u_1, int64_t v_1) {
  int64_t t_1;
  while (v_1 != 0) {
    t_1 = u_1; 
    u_1 = v_1; 
    v_1 = t_1 % v_1;
  }                     // return u < 0 ? -u : u; /* abs(u) */
  return abs(u_1);
}

void Error_String() {
  strcpy( display_string, string_start );
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
}

void Clear_String() {   // String loeschen -- Eingabe Mantisse
  strcpy( display_string, string_start );
  // mem_pointer = 1;
  display_string[Memory_1] = mem_str_1[mem_pointer];
  display_string[Memory_0] = mem_str_0[mem_pointer];

  if ( Rad_in_out == true ) {
    display_string[Rad_point] = '.';
  }
 
  if ( Deg_in_out == true ) {
    display_string[Deg_point] = '.';
  }

  Cursor_pos = 2;
  Point_pos = 0;
  Number_count = 0;
  Zero_count = 0;
  Zero_after_Point = 0;
  Zero_index = 0;
  Zero_index_a = 0;
  Init_expo = true;

  Display_new = true;
  Mantisse_change = false;
  Expo_change = false;
  Start_input = Input_Mantisse;
  
  mem_stack[mem_pointer].expo = 0;
  mem_stack[mem_pointer].num = int32_max;
  mem_stack[mem_pointer].denom = int32_max;
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
  Display_new = true;
}

void Error_Test() {
	if ( Debug_Level == 11 ) {
    Serial.print("= ");
    Serial.print(mem_stack[mem_pointer].num);
    Serial.print(" / ");
    Serial.print(mem_stack[mem_pointer].denom);
    Serial.print(" x 10^ ");
    Serial.println(mem_stack[mem_pointer].expo);
  }
  if ( mem_stack[mem_pointer].expo > expo_max ) {
    Clear_String();
    Error_String();
    Start_input = Display_Error;
    display_string[2] = '^';
    display_string[13] = '^';
  }
  if ( mem_stack[mem_pointer].expo == expo_max ) {
  	num_temp_u32   = abs(mem_stack[mem_pointer].num) / 9;
  	denom_temp_u32 = abs(mem_stack[mem_pointer].denom) / 10;
  	if ( num_temp_u32 > denom_temp_u32 ) {
      Clear_String();
      Error_String();
      Start_input = Display_Error;
      display_string[2] = '^';
      display_string[13] = '^';
    }
  }
  if ( mem_stack[mem_pointer].expo == expo_min ) {
  	if ( abs(mem_stack[mem_pointer].num) < abs(mem_stack[mem_pointer].denom) ) {
      Clear_String();
      Error_String();
      Start_input = Display_Error;
      display_string[2] = 'u';
      display_string[13] = 'u';
    }
  }
  if ( mem_stack[mem_pointer].expo < expo_min ) {
    Clear_String();
    Error_String();
    Start_input = Display_Error;
    display_string[2] = 'u';
    display_string[13] = 'u';
  }
}

void Get_Number() {
  if ( Number_count != Zero_count ) {
    if ( Mantisse_change == true ) {
      Get_Mantisse();
    }
    else {
      if ( Expo_change = true ) {
        Get_Expo_change();
      }
    }
  }
  else {
    mem_stack[ mem_pointer ].expo = -99;
    mem_stack[ mem_pointer ].num = int32_max;
    mem_stack[ mem_pointer ].denom = int32_max;
    if (display_string[1] == '-') {
      mem_stack[mem_pointer].num *= -1;
    }
  }
  if ( Start_input != Display_Error ) {
    mem_stack[ 0 ].num = mem_stack[ mem_pointer ].num;
    mem_stack[ 0 ].denom = mem_stack[ mem_pointer ].denom;
    mem_stack[ 0 ].expo = mem_stack[ mem_pointer ].expo;
    Start_input = Input_Operation_0;
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
  mem_stack[mem_pointer].expo = expo_temp_16;
 
  switch (Point_pos) {
  	
    case 0:
      mem_stack[mem_pointer].expo = mem_stack[mem_pointer].expo + Number_count;
      break;

    case 1:
      break;

    case 2:
      mem_stack[mem_pointer].expo = mem_stack[mem_pointer].expo - Zero_after_Point;
      break;
  	
    default:
      mem_stack[mem_pointer].expo = mem_stack[mem_pointer].expo + Point_pos - 2;
      break;
  }

  num_temp_u32 = atol(Temp_char);
  num_temp_u32 *= expo_10[Zero_after_Point];
  denom_temp_u32 = expo_10[Number_count];

  if ( First_char < '3' ) {
    num_temp_u32 *= expo_10[1];
    --mem_stack[mem_pointer].expo;
  }
  
  gcd_temp_32 = gcd_iter_32(num_temp_u32, denom_temp_u32);
  num_temp_u32 /= gcd_temp_32;
  denom_temp_u32 /= gcd_temp_32;

  Expand_Number();
  
  mem_stack[mem_pointer].num = num_temp_u32;
  if (display_string[1] == '-') {
    mem_stack[mem_pointer].num *= -1;
  }
  mem_stack[mem_pointer].denom = denom_temp_u32;

  Error_Test();  

  if ( Debug_Level == 8 ) {
    Serial.print("= ");
    Serial.print(mem_stack[mem_pointer].num);
    Serial.print(" / ");
    Serial.print(mem_stack[mem_pointer].denom);
    Serial.print(" x 10^ ");
    Serial.println(mem_stack[mem_pointer].expo);
  }
}

void Reduce_Number() {
  if ( num_temp_u64 > denom_temp_u64 ) {
    test_temp_8 =  1;
  }
  else {
    test_temp_8 = -1;               // num_temp_u64 <--> denom_temp_u64
    calc_temp_u64_0 = num_temp_u64;
    num_temp_u64 = denom_temp_u64;
    denom_temp_u64 = calc_temp_u64_0;
  }
              
  // https://hg.python.org/cpython/file/3.5/Lib/fractions.py#l252
  // https://hg.python.org/cpython/file/3.1/Lib/fractions.py#l167
              
  num_temp_u64_0 = num_temp_u64;
  denom_temp_u64_0 = denom_temp_u64;
  p0 = 0;
  p1 = 1;
  q0 = 1;
  q1 = 0;
  while ( p1 < int32_max ) {
    a0 = num_temp_u64_0 / denom_temp_u64_0;

    calc_temp_u64_0 = p0;
    p0 =  p1;
    p1 *= a0;
    p1 += calc_temp_u64_0;

    calc_temp_u64_0 = q0;
    q0 =  q1;
    q1 *= a0;
    q1 += calc_temp_u64_0;

    calc_temp_u64_0 = num_temp_u64_0;
    num_temp_u64_0 = denom_temp_u64_0;
    denom_temp_u64_0 = calc_temp_u64_0 - (a0 * denom_temp_u64_0);
  }

  if ( test_temp_8 > 0 ) {
    num_temp_u32 = p0;
    denom_temp_u32 = q0;
  }
  else {
    num_temp_u32 = q0;
    denom_temp_u32 = p0;
  }

  Expand_Number();
}

void Expand_Number() {
  if ( num_temp_u32 > denom_temp_u32 ) {
    mul_temp_u32 = int32_max / num_temp_u32;
  }
  else { 
    mul_temp_u32 = int32_max / denom_temp_u32;              
  }
  num_temp_u32 *= mul_temp_u32;
  denom_temp_u32 *= mul_temp_u32;
}

void Get_Expo_change() {
  calc_temp_32_0 = mem_stack[ mem_pointer ].num;
  calc_temp_32_1 = mem_stack[ mem_pointer ].denom;
  Get_Mantisse();
  mul_temp_u32 = abs(calc_temp_32_0 / calc_temp_32_1);
  mem_stack[ mem_pointer ].num = calc_temp_32_0;
  mem_stack[ mem_pointer ].denom = calc_temp_32_1;
  if ( mul_temp_u32 > 2 ) {
    --mem_stack[ mem_pointer ].expo;
  }
}

int16_t display_expo = 0;
int64_t display_big = 1;
int32_t display_number = 1;
#define digit_count_max 8
int16_t display_expo_mod = 0;

void Display_Number() {
/*
 *   Round "half towards zero"
 *   https://en.wikipedia.org/wiki/Rounding#Round_half_towards_zero
 *   23.5     -->  23
 *  -23.5     --> -23
 *   23.50001 -->  24
 *  -23.50001 --> -24
 */
  if ( Debug_Level == 9 ) {
	  Serial.print("'");
	  Serial.print(display_string);
	  Serial.println("'");
  }

  if ( mem_stack[mem_pointer].denom < 0 ) {
  	mem_stack[mem_pointer].num *= -1;
  	mem_stack[mem_pointer].denom *= -1;
  }
  display_expo = mem_stack[mem_pointer].expo;
  if ( abs(mem_stack[mem_pointer].num) != 0 ) {            
    display_big = abs(mem_stack[mem_pointer].num);
  }
  else {
  	display_big = abs(mem_stack[mem_pointer].denom);
  }

  if ( display_big > abs(mem_stack[mem_pointer].denom) ) {
    display_big = expo_10[display_digit -1] * display_big;
    ++display_expo;
  }
  else {
    display_big = expo_10[display_digit] * display_big;
  }  
  display_big = display_big + (abs(mem_stack[mem_pointer].denom) / 2) - 1;
  display_number = display_big / mem_stack[mem_pointer].denom;

  if (display_number == expo_10[display_digit]) {
    display_number = expo_10[display_digit - 1];
    ++display_expo;
  }
  strcpy( display_string, string_start );

  if (mem_stack[mem_pointer].num < 0) {
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

  display_string[Plus_Minus_Expo_] = '#';
  if ( expo_temp_16 >= 0 ) {
    display_string[Plus_Minus_Expo] = '#';
    display_string[Plus_Minus_Expo__] = ' ';
  }
  display_string[display_digit + 2] = '.';
  // display_string[display_digit + 3] = '#';

  if ( display_expo_mod <= 0 ) {
    display_expo_mod = display_expo_mod + 3;
  }
  
  if ( Debug_Level == 9 ) {
	  Serial.print("'");
	  Serial.print(display_string);
	  Serial.println("'");
  }

  if ( display_expo_mod < display_digit ) {
    for ( index_a = display_digit + 1; index_a > (display_expo_mod + 2); --index_a ) {
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
 
  if ( Deg_in_out == true ) {
    display_string[Deg_point] = '.';
  }
  
  if ( display_string[Plus_Minus_Expo__] == ' ' ) {
    display_string[Plus_Minus_Expo__] = '#';
  }

  if ( Start_input >= Input_Operation_0 ) {       // Input / Operation Display
    if ( Start_input < Display_Error ) {	
      display_string[Operation] = Input_Operation_char;
    }
  }

  if ( Start_input == Display_Result ) {
    display_string[Memory_0] = '=';
  }

  Point_pos = display_expo_mod + 2;
  Number_count = display_digit;
  Cursor_pos = display_digit + 3;
  
  if ( display_string[Cursor_pos] == '.' ) {
  	++Number_count;
    ++Cursor_pos;
  }

  if ( abs(mem_stack[mem_pointer].num) == 0 ) {            
    display_string[Mantisse_0] = '.';
    display_string[Mantisse_1] = '0';
    Zero_count = display_digit;
    --Point_pos;
  }

  Display_new = true;

  if ( abs(mem_stack[mem_pointer].denom) == 0 ) {            
    Clear_String();
    Error_String();
    Start_input = Display_Error;
    display_string[2] = '0';
    display_string[13] = '0';
  }
  
  Zero_count = 0;
  Zero_index = 0;
  while (display_string[Zero_index] != '#') {
    if ( display_string[Zero_index]  == '0' ) {
      ++Zero_count;
    }
    ++Zero_index;
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

  pinMode(PWM, OUTPUT);           // Pin 22
  digitalWrite(PWM, HIGH);

  pinMode(On_Off_PIN, OUTPUT);    // Power "ON"
  digitalWrite(On_Off_PIN, LOW);

  uint8_t pin;
  for (pin = Min_Out; pin <= Max_Out; ++pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }
 /*
  for (pin = 0; pin < Digit_Count; ++pin) { 
    fastPinMode(index_display[pin], OUTPUT);  // fastDigital function
  }
 */
  digitalWrite(On_Off_PIN, HIGH);

  pinMode(SDA, INPUT);           // Pin 17
  pinMode(SCL, OUTPUT);          // Pin 16
  pinMode(DCF77, INPUT_PULLUP);  // Pin 7

  pinMode(A_0, INPUT);           // set pin to input
  pinMode(A_1, INPUT);           // set pin to input
  pinMode(A_2, INPUT);           // set pin to input

  pinMode(Out_0, OUTPUT);        // Pin A3 not used
  digitalWrite(Out_0, LOW);
  pinMode(Out_A, OUTPUT);        // Pin A4
  digitalWrite(Out_A, LOW);
  pinMode(Out_B, OUTPUT);        // Pin A5
  digitalWrite(Out_B, LOW);
  pinMode (Out_C, OUTPUT);       // Pin A6
  digitalWrite(Out_C, LOW);

  pinMode(Beep, OUTPUT);         // Pin A7
  digitalWrite(Beep, LOW);

  analogReference(EXTERNAL);

  Timer1.initialize(Time_LOW);  // sets timer1 to a period of 263 microseconds
  TCCR1A |= (1 << COM1B1) | (1 << COM1B0);	// inverting mode for Pin OC1B --> D4
  Timer1.attachInterrupt( timerIsr );  // attach the service routine here
  Timer1.pwm(PWM_Pin, led_bright[led_bright_index]);  // duty cycle goes from 0 to 1023

  // start serial port at 115200 bps and wait for port to open:
  Serial.begin(115200);
  
  // operation_stack[0] = Mem_0;

}

// the loop routine runs over and over again forever:
void loop() {

  if ( Switch_Code > 0 ) {     //    Main Responce about Switch

    switch (Switch_Code) {

      case 91:                 //    FN - Light up
        if ( Debug_Level == 5 ) {
          Serial.println("Light_up");
        }
        ++led_bright_index;
        if ( led_bright_index > led_bright_max ) {
          led_bright_index = led_bright_max;
        }
        else {
          Beep_on = true;
        }
        break;

      case 93:                 //    FN - Light down
        if ( Debug_Level == 5 ) {
          Serial.println("Light_down");
        }
        --led_bright_index;
        if ( led_bright_index < led_bright_min ) {
          led_bright_index = led_bright_min;
        }
        else {
          Beep_on = true;
        }
        break;

      case 121:                //    clock
        if ( Debug_Level == 5 ) {
          Serial.println("clock");
        }
        Beep_on = true;

        if (Pendular_on == false) {
          Pendular_on = true;
          Start_mem = Start_input;
        }
        else {
          Pendular_on = false;
          Start_input = Start_mem;
          Display_new = true;
        }
        break;

      case 122:                //    beep
        if ( Debug_Level == 5 ) {
          Serial.println("beep");
        }
        Beep_on = true;

        Beep_on_off = !Beep_on_off;     // (toggle)
        break;

      case 123:                //    Off
        if ( Debug_Level == 5 ) {
          Serial.println("OFF");
        }
        Countdown_OFF = Countdown_Start;      // Switch Off  2x Beep
        break;

      case 125:                //    Off after  5min
        if ( Debug_Level == 5 ) {
          Serial.println("OFF_5min");
        }
        Countdown_OFF = Countdown_Start_Off;  // Switch Off  3x Beep
        break;
        
      case 90:                 //    _CE_
      	if ( Start_input == Display_Error ) {
          Beep_on = true;
          mem_pointer = 1;
          Start_input = Start_Mode;     
        }
        break;

    }

    if ( (Pendular_on == false) && (Start_input != Display_Error) ) {

      switch (Switch_Code) {

        case 30:                 //    <--
          if ( Debug_Level == 5 ) {
            Serial.println("<<-");
          }
          Beep_on = true;
          break;

        case 31:                 //    SM_on
          if ( Debug_Level == 5 ) {
          	Serial.println("_SM_on");
          }
          if ( SM_on == false ) {
            SM_on = true;
            disp_on = false;
            display_string[STO_point] = '.';
            hyp_on = false;
            display_string[inv_point] = ' ';
            Beep_on = true;
            Display_new = true;
          }
          break;

        case 32:                 //    FIX_dms
          if ( Debug_Level == 5 ) {
            Serial.println("FIX_dms");
          }
          Beep_on = true;
          break;

        case 33:                 //    _1/x_
          if ( Debug_Level == 5 ) {
            Serial.println("1/x");
          }
          Beep_on = true;
          if ( Start_input < Input_Operation_0 ) {      // Input Number
            Get_Number();
          }
          if ( Start_input == Display_M_Plus ) {
            mem_pointer = 1;
            mem_stack[ mem_pointer ].expo = mem_stack[ 0 ].expo;
            mem_stack[ mem_pointer ].num = mem_stack[ 0 ].num;
            mem_stack[ mem_pointer ].denom = mem_stack[ 0 ].denom;
            Start_input = Input_Operation_0;
          }
          if ( Start_input == Display_Result ) {
            mem_pointer = 1;
            mem_stack[ mem_pointer ].expo = mem_extra_stack[ 0 ].expo;
            mem_stack[ mem_pointer ].num = mem_extra_stack[ 0 ].num;
            mem_stack[ mem_pointer ].denom = mem_extra_stack[ 0 ].denom;
            Start_input = Input_Operation_0;
          }
          if ( Start_input == Input_Operation_0 ) {
            mem_pointer = 0;
            calc_temp_32_0 = mem_stack[ mem_pointer ].num;
            mem_stack[ mem_pointer ].num = mem_stack[ mem_pointer ].denom;
            mem_stack[ mem_pointer ].denom = calc_temp_32_0;
            mem_stack[ mem_pointer ].expo *= -1;

            Error_Test();            
            if ( Start_input != Display_Error ) {
              Display_Number();
            }
            else {
              mem_pointer = 1;
            }
          }
          break;

        case 34:                 //    lg2
          if ( Debug_Level == 5 ) {
            Serial.println("lg2");
          }
          Beep_on = true;
          break;

        case 35:                 //    +/-
          if ( Debug_Level == 5 ) {
            Serial.println("+/-");
          }
          if ( Start_input == Input_Operation_0 ) {
          	if ( mem_pointer > 0 ) {
              mem_stack[ 0 ].num = mem_stack[ mem_pointer ].num;
              mem_stack[ 0 ].denom = mem_stack[ mem_pointer ].denom;
              mem_stack[ 0 ].expo = mem_stack[ mem_pointer ].expo;
            }
          	mem_pointer = 0;
          	mem_stack[ mem_pointer ].num *= -1;
            Display_Number();
          }
          if ( Start_input == Display_M_Plus ) {
          	Result_to_Start_Mode();
          }
          if ( Start_input == Display_Result ) {
          	Result_to_Start_Mode();
          }
          if ( Start_input == Input_Mantisse ) {
            mem_stack[ mem_pointer ].num *= -1;
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
          Display_new = true;
          Beep_on = true;
          break;

        case 36:                 //    PI()
          if ( Debug_Level == 5 ) {
            Serial.println("PI()");
          }
          temp_expo  = Pi_expo;
          temp_num   = Pi_num;
          temp_denom = Pi_denom;
          Memory_to_Input_Mantisse();
          break;

        case 37:                 //    Deg
          if ( Debug_Level == 5 ) {
            Serial.println("Deg");
          }
          if ( Deg_in_out == false ) {
            Deg_in_out = true;
            Rad_in_out = false;
            display_string[Deg_point] = '.';
            display_string[Rad_point] = ' ';
            Display_new = true;
            Beep_on = true;
          }
          break;

        case 38:                 //    _EE+3_
          if ( Debug_Level == 5 ) {
            Serial.println("EE+3");
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
                Display_new = true;
                Beep_on = true;
              }
            }
          }
          if ( Start_input < Input_Operation_0 ) {      // Input Number
            if (Number_count != Zero_count) {
            	Init_expo = false;
              expo_temp_16 = Get_Expo() + 102;
              expo_temp_16 = expo_temp_16 / 3;
              if ( expo_temp_16 < (expo_max_in_3 + 34) ) {
                Beep_on = true;
                Expo_change = true;
                ++expo_temp_16;
                expo_temp_16 = (expo_temp_16 * 3) - 102;
                Put_Expo();
              }
            }
          }
          break;

        case 39:                 //    _EE-3_
          if ( Debug_Level == 5 ) {
            Serial.println("EE-3");
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
                Display_new = true;
                Beep_on = true;
              }
            }
          }
          if ( Start_input < Input_Operation_0 ) {      // Input Number
            if (Number_count != Zero_count) {
            	Init_expo = false;
              expo_temp_16 = Get_Expo() + 102;
              if ( (expo_temp_16 % 3) != 0 ) {
                expo_temp_16 = expo_temp_16 + 3;
              }
              expo_temp_16 = expo_temp_16 / 3;
              if ( expo_temp_16 > (expo_min_in_3 + 34) ) {
                Beep_on = true;
                Expo_change = true;
                --expo_temp_16;
                expo_temp_16 = (expo_temp_16 * 3) - 102;
                Put_Expo();
              }
            }
          }
          break;

        case 40:                 //    _(_
          if ( Debug_Level == 5 ) {
            Serial.println("_(_");
          }
          Beep_on = true;
          break;

        case 41:                 //    _)_
          if ( Debug_Level == 5 ) {
            Serial.println("_)_");
          }
          Beep_on = true;
          break;

        case 42:                 //    _*_
          if ( Debug_Level == 5 ) {
            Serial.println("_*_");
          }
          Beep_on = true;
          break;

        case 43:                 //    _+_
          if ( Debug_Level == 5 ) {
            Serial.println("_+_");
          }
          Beep_on = true;
          break;

        case 44:                 //    e()
          if ( Debug_Level == 5 ) {
            Serial.println("e()");
          }
          temp_expo  = e_expo;
          temp_num   = e_num;
          temp_denom = e_denom;
          Memory_to_Input_Mantisse();
          break;

        case 45:                 //    _-_
          if ( Debug_Level == 5 ) {
            Serial.println("_-_");
          }
          Beep_on = true;
          break;

        case 46:                 //    .
          if ( Debug_Level == 5 ) {
            Serial.println(".");
          }
          if ( Start_input == Display_M_Plus ) {
          	Result_to_Start_Mode();
          }
          if ( Start_input == Display_Result ) {
          	Result_to_Start_Mode();
          }
          if ( Start_input == Input_Operation_0 ) {
            if ( mem_pointer > 0 ) {
              Clear_String();
            }
          }
          if ( Start_input == Input_Mantisse ) {
            if ( Point_pos == 0 ) {
              if ( Number_count < 8 ) {
                Beep_on = true;
                Point_pos = Cursor_pos;
                display_string[Cursor_pos] = '.';
                ++Cursor_pos;
                display_string[Cursor_pos] = '_';
                Display_new = true;
              }
            }
          }
          break;

        case 47:                 //    _/_
          if ( Debug_Level == 5 ) {
            Serial.println("_/_");
          }
          Beep_on = true;
          break;

        case 48:                 //    0
          if ( Debug_Level == 5 ) {
            Serial.println(char(Switch_Code));
          }
          if ( Start_input == Input_Mantisse ) {
            if ( Number_count < 8 ) {
            	if ( Zero_count < 7 ) {
                if (( Number_count > 0 ) or ( Point_pos > 0 )) {
                 	Beep_on = true;
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
                  Display_new = true;
                }
              }
            }
          }
          if ( Start_input == Input_Expo ) {
          	Beep_on = true;
          	Expo_change = true;
            display_string[Expo_1] = display_string[Expo_0];
            display_string[Expo_0] = Switch_Code;
            Display_new = true;
          }
          break;

        case 49:                 //    1
        case 50:                 //    2
        case 51:                 //    3
        case 52:                 //    4
        case 53:                 //    5
        case 54:                 //    6
        case 55:                 //    7
        case 56:                 //    8
        case 57:                 //    9
          if ( Debug_Level == 5 ) {
            Serial.println(char(Switch_Code));
          }
          if ( Start_input == Display_M_Plus ) {
          	Result_to_Start_Mode();
          }
          if ( Start_input == Display_Result ) {
          	Result_to_Start_Mode();
          }
          if ( Start_input == Input_Operation_0 ) {
            if ( mem_pointer > 0 ) {
              Clear_String();
            }
          }
          if ( Start_input == Input_Mantisse ) {
            if ( Number_count < 8 ) {
              Beep_on = true;
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
              Display_new = true;
            }
          }
          if ( Start_input == Input_Expo ) {
          	Beep_on = true;
          	Expo_change = true;
            display_string[Expo_1] = display_string[Expo_0];
            display_string[Expo_0] = Switch_Code;
            Display_new = true;
          }
          break;

        case 58:                 //    DISP_on
          if ( Debug_Level == 5 ) {
          	Serial.println("_DISP_on");
          }
          if ( disp_on == false ) {
            disp_on = true;
            display_string[STO_point] = '.';
            hyp_on = false;
            display_string[inv_point] = '.';
            SM_on = false;
            Beep_on = true;
            Display_new = true;
          }
          break;

        case 59:                 //    EE_FN_=
          if ( Debug_Level == 5 ) {
            Serial.println("EE_FN_=");
          }
          EE_FN_xx = true;
          disp_on = false;
          display_string[STO_point] = ' ';
          hyp_on = false;
          display_string[inv_point] = ' ';
          SM_on = false;
          Beep_on = true;
          Display_new = true;
          break;

        case 60:                 //    <--
          if ( Debug_Level == 5 ) {
            Serial.println("<--");
          }
          if ( Start_input == Input_Mantisse ) {
            if (Cursor_pos > 2) {
              Beep_on = true;
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
              Display_new = true;
            }
          }
          break;

        case 61:                 //    _=_
          if ( Debug_Level == 5 ) {
            Serial.println("_=_");
          }
          if ( Start_input < Input_Operation_0 ) {      // Input Number
            Get_Number();
          }
          if ( Start_input != Display_Result ) {      
            if ( Start_input != Display_Error ) {
              Start_input = Display_Result;
              mem_pointer = 0;
              mem_extra_stack[ 0 ].expo = mem_stack[ 0 ].expo;
              mem_extra_stack[ 0 ].num = mem_stack[ 0 ].num;
              mem_extra_stack[ 0 ].denom = mem_stack[ 0 ].denom;
              Display_Number();
              Beep_on = true;
            }
          }
          break;

        case 62:                 //    hyp_on
          if ( Debug_Level == 5 ) {
            Serial.println("_hyp_on");
          }
          if ( hyp_on == false ) {
            hyp_on = true;
            display_string[inv_point] = '.';
            disp_on = false;
            display_string[STO_point] = ' ';
            SM_on = false;
            Beep_on = true;
            Display_new = true;
          }
          break;

        case 63:                 //    DISP_hyp_off
          if ( Debug_Level == 5 ) {
            Serial.println("_DISP_hyp_off");
          }
          if ( hyp_on == true ) {
            hyp_on = false;
            Beep_on = true;
            display_string[inv_point] = ' ';
            Display_new = true;
          }
          if ( disp_on == true  ) {
            disp_on = false;
            Beep_on = true;
            display_string[inv_point] = ' ';
          	display_string[STO_point] = ' ';
            Display_new = true;
          }
          if ( SM_on == true  ) {
            SM_on = false;
            Beep_on = true;
          	display_string[STO_point] = ' ';
            Display_new = true;
          }
          break;

        case 64:                 //    Rad
          if ( Debug_Level == 5 ) {
            Serial.println("Rad");
          }
          if ( Rad_in_out == false ) {
            Rad_in_out = true;
            Deg_in_out = false;
            display_string[Deg_point] = ' ';
            display_string[Rad_point] = '.';
            Display_new = true;
            Beep_on = true;
          }
          break;

        case 65:                 //    sin(x)
          if ( Debug_Level == 5 ) {
            Serial.println("sin(x)");
          }
          Beep_on = true;
          break;

        case 66:                 //    cos(x)
          if ( Debug_Level == 5 ) {
            Serial.println("cos(x)");
          }
          Beep_on = true;
          break;

        case 67:                 //    tan(x)
          if ( Debug_Level == 5 ) {
            Serial.println("tan(x)");
          }
          Beep_on = true;
          break;

        case 68:                 //    asin(x)
          if ( Debug_Level == 5 ) {
            Serial.println("asin(x)");
          }
          Beep_on = true;
          break;

        case 69:                 //    acos(x)
          if ( Debug_Level == 5 ) {
            Serial.println("acos(x)");
          }
          Beep_on = true;
          break;

        case 70:                 //    atan(x)
          if ( Debug_Level == 5 ) {
            Serial.println("atan(x)");
          }
          Beep_on = true;
          break;

        case 71:                 //    sinh(x)
          if ( Debug_Level == 5 ) {
            Serial.println("sinh(x)");
          }
          Beep_on = true;
          break;

        case 72:                 //    cosh(x)
          if ( Debug_Level == 5 ) {
            Serial.println("cosh(x)");
          }
          Beep_on = true;
          break;

        case 73:                 //    tanh(x)
          if ( Debug_Level == 5 ) {
            Serial.println("tanh(x)");
          }
          Beep_on = true;
          break;

        case 74:                 //    asinh(x)
          if ( Debug_Level == 5 ) {
            Serial.println("asinh(x)");
          }
          Beep_on = true;
          break;

        case 75:                 //    acosh(x)
          if ( Debug_Level == 5 ) {
            Serial.println("acosh(x)");
          }
          Beep_on = true;
          break;

        case 76:                 //    atanh(x)
          if ( Debug_Level == 5 ) {
            Serial.println("atanh(x)");
          }
          Beep_on = true;
          break;

        case 77:                 //    RM(0)
        case 78:                 //    RM(1)
        case 79:                 //    RM(2)
        case 80:                 //    RM(3)
        case 81:                 //    RM(4)
        case 82:                 //    RM(5)
        case 83:                 //    RM(6)
        case 84:                 //    RM(7)
        case 85:                 //    RM(8)
        case 86:                 //    RM(9)
        	mem_extra_test = Switch_Code - RM_0;
          if ( Debug_Level == 5 ) {
            Serial.print("RM(");
            Serial.print(mem_extra_test);
            Serial.println(")");
          }
          temp_expo  = mem_extra_stack[ mem_extra_test ].expo;
          temp_num   = mem_extra_stack[ mem_extra_test ].num;
          temp_denom = mem_extra_stack[ mem_extra_test ].denom;
          Memory_to_Input_Mantisse();
          display_string[Memory_1] = 'm';
          display_string[Memory_0] = 48 + mem_extra_test;
          break;

        case 87:                 //    FIX_a_b/c
          if ( Debug_Level == 5 ) {
            Serial.println("FIX_a_b/c");
          }
          Beep_on = true;
          break;

        case 88:                 //    y_expo
          if ( Debug_Level == 5 ) {
            Serial.println("y_expo");
          }
          Beep_on = true;
          break;

        case 89:                 //    y_root
          if ( Debug_Level == 5 ) {
            Serial.println("y_root");
          }
          Beep_on = true;
          break;

        case 90:                 //    _CE_
          if ( Debug_Level == 5 ) {
            Serial.println("CE");
          }
          if ( Start_input < Input_Operation_0 ) {    // Input Number
            if ( (Number_count > 0) || (Point_pos > 0) ) {
              Beep_on = true;
              Start_input = Start_Mode;
            }
          }
          if ( (Start_input == Display_Result) || (Start_input == Display_M_Plus) ) {
            // operation_pointer = 1;
            Mantisse_change = false;
            Start_input = Input_Mantisse;
            Init_expo = false;
            mem_pointer = 1;
            mem_stack[ mem_pointer ].num = mem_stack[ 0 ].num;
            mem_stack[ mem_pointer ].denom = mem_stack[ 0 ].denom;
            mem_stack[ mem_pointer ].expo = mem_stack[ 0 ].expo;
            display_digit_temp = display_digit;
            display_digit = 8;
            Display_Number();
            display_string[Cursor_pos] = '.';
            // display_string[Memory_0] = '1';
            // display_string[Cursor_pos] = '_';
            // if ( Cursor_pos == Plus_Minus_Expo_ ) {
            // 	display_string[Cursor_pos] = '.';
            // }
            Beep_on = true;
            display_digit = display_digit_temp;
          //  Display_new = true;
          }
          if ( Start_input == Input_Operation_0 ) {
            // --operation_pointer;
            Mantisse_change = false;
            Start_input = Input_Mantisse;
            Init_expo = false;
            mem_pointer = 1;
            display_digit_temp = display_digit;
            display_digit = 8;
            Display_Number();
            // display_string[Memory_0] = '1';
            // display_string[Cursor_pos] = '_';
            // display_string[Operation] = '#';
            // if ( Cursor_pos == Plus_Minus_Expo_ ) {
            display_string[Cursor_pos] = '.';
            while ( display_string[Cursor_pos - 1] == '0' ) {
              display_string[Cursor_pos] = '#';
            	--Cursor_pos;
              --Number_count;
              --Zero_count;
              display_string[Cursor_pos] = '_';
            }
            Beep_on = true;
            // Display_new = true;
            display_digit = display_digit_temp;
          }
          break;

        case 92:                 //    _//_
          if ( Debug_Level == 5 ) {
            Serial.println("_//_");
          }
          Beep_on = true;
          break;

        case 94:                 //    _/p/_  Phytagoras
          if ( Debug_Level == 5 ) {
            Serial.println("_/p/_");
          }
          Beep_on = true;
          break;

        case 95:          // FIX_2        10 ..       99 -->  Display
        case 96:          // FIX_3       100 ..      999 -->  Display
        case 97:          // FIX_4      1000 ..     9999 -->  Display
        case 98:          // FIX_5     10000 ..    99999 -->  Display
        case 99:          // FIX_6    100000 ..   999999 -->  Display
        case 100:         // FIX_7   1000000 ..  9999999 -->  Display
        case 101:         // FIX_8  10000000 .. 99999999 -->  Display
         	mem_extra_test = Switch_Code - FIX_0;
          if ( Debug_Level == 5 ) {
            Serial.print("FIX_");
            Serial.println(mem_extra_test);
          }
          if ( Start_input < Input_Operation_0 ) {  // Input Number
            Get_Number();
          }
          else {                                    // Display Number
            Beep_on = true;
            display_digit = mem_extra_test; 
            Display_Number();
          }
          break;

        case 102:                //    FIX_E24
          if ( Debug_Level == 5 ) {
            Serial.println("FIX_E24");
          }
          Beep_on = true;
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
            mem_pointer = 1;
            mem_stack[ mem_pointer ].expo = mem_extra_stack[ 0 ].expo;
            mem_stack[ mem_pointer ].num = mem_extra_stack[ 0 ].num;
            mem_stack[ mem_pointer ].denom = mem_extra_stack[ 0 ].denom;
            Start_input = Input_Operation_0;
          }
          if ( Start_input == Input_Operation_0 ) {
            mem_pointer = 0;
            expo_temp_16_a = mem_extra_stack[ 0 ].expo;
            expo_temp_16_b = mem_extra_stack[ mem_extra_test ].expo;         
            expo_temp_16_diff = expo_temp_16_a - expo_temp_16_b;
            expo_temp_16_diff_abs = abs(expo_temp_16_diff); 
            calc_temp_64_a = mem_extra_stack[ 0 ].num;
            calc_temp_64_a *= mem_extra_stack[ mem_extra_test ].denom;
            calc_temp_64_b = mem_extra_stack[ mem_extra_test ].num;
            calc_temp_64_b *= mem_extra_stack[ 0 ].denom;
            calc_temp_64_c = mem_extra_stack[ 0 ].denom;
            calc_temp_64_c *= mem_extra_stack[ mem_extra_test ].denom;
            expo_temp_16 = expo_temp_16_a; 
              if ( Debug_Level == 12 ) {
                Serial.print("= _M+ 32bit_ ");
                Serial.print(num_temp_u32);
                Serial.print(" / ");
                Serial.print(denom_temp_u32);
                Serial.print(" x 10^ ");
                Serial.println(expo_temp_16);
              }
            if ( expo_temp_16_diff > 0 ) {
              expo_temp_16 = expo_temp_16_a;
              if ( expo_temp_16_diff_abs > 18 ) {
                mem_extra_stack[ mem_extra_test ].expo = mem_stack[ mem_pointer ].expo;
                mem_extra_stack[ mem_extra_test ].num = mem_stack[ mem_pointer ].num;
                mem_extra_stack[ mem_extra_test ].denom = mem_stack[ mem_pointer ].denom;
                Display_Number();
                display_string[Memory_1] = 'm';
                display_string[Memory_0] = 48 + mem_extra_test;
                Beep_on = true;
                break;
              }
              if ( expo_temp_16_diff_abs < 10 ) {
              	calc_temp_64_b /= expo_10[expo_temp_16_diff_abs];
              }
              else {
                calc_temp_64_b /= expo_10_[expo_temp_16_diff_abs - 10];
              }
            }
            if ( expo_temp_16_diff < 0 ) {
              expo_temp_16 = expo_temp_16_b;              
              if ( expo_temp_16_diff_abs > 18 ) {
                mem_stack[ mem_pointer ].expo = mem_extra_stack[ mem_extra_test ].expo;
                mem_stack[ mem_pointer ].num = mem_extra_stack[ mem_extra_test ].num;
                mem_stack[ mem_pointer ].denom = mem_extra_stack[ mem_extra_test ].denom;
                Display_Number();
                display_string[Memory_1] = 'm';
                display_string[Memory_0] = 48 + mem_extra_test;
                Beep_on = true;
                break;
              }
              if ( expo_temp_16_diff_abs < 10 ) {
              	calc_temp_64_a /= expo_10[expo_temp_16_diff_abs];
              }
              else {
                calc_temp_64_a /= expo_10_[expo_temp_16_diff_abs - 10];
              }
            }
            calc_temp_64_d = calc_temp_64_a;
            calc_temp_64_d += calc_temp_64_b;
            if ( calc_temp_64_d > 0 ) {
            	test_signum_8 = 1;
            }
            else {
            	test_signum_8 = -1;
            }

          	if ( calc_temp_64_b > calc_temp_64_a ) {
              calc_temp_u64_0 = calc_temp_64_b / int32_max;
            } else {
              calc_temp_u64_0 = calc_temp_64_a / int32_max;
            }
            ++calc_temp_u64_0;

            num_temp_u64 = abs(calc_temp_64_d);  // max:  9223372036854775807
            denom_temp_u64 = calc_temp_64_c;

            if ( num_temp_u64 <= expo_test_9 ) {
              num_temp_u64 *= expo_10[9];
              expo_temp_16 -= 9;
            }
            if ( num_temp_u64 <= expo_test_6 ) {
              num_temp_u64 *= expo_10[6];
              expo_temp_16 -= 6;
            }
            if ( num_temp_u64 <= expo_test_3 ) {
              num_temp_u64 *= expo_10[3];
              expo_temp_16 -= 3;
            }
            if ( num_temp_u64 <= expo_test_2 ) {
              num_temp_u64 *= expo_10[2];
              expo_temp_16 -= 2;
            }
            if ( num_temp_u64 <= expo_test_1 ) {
              num_temp_u64 *= expo_10[1];
              expo_temp_16 -= 1;
            }

            if ( denom_temp_u64 <= expo_test_2a ) {
              denom_temp_u64 *= expo_10[2];
              expo_temp_16 += 2;
            }

            if ( denom_temp_u64 <= expo_test_1a ) {
              denom_temp_u64 *= expo_10[1];
              expo_temp_16 += 1;
            }

          	if ( num_temp_u64 > denom_temp_u64 ) {
          	  calc_temp_u32 = num_temp_u64 / denom_temp_u64;
          	  if ( calc_temp_u32 > 2 ) {
          	    num_temp_u64 /= 2;
          	    denom_temp_u64 *= 5;
          	    ++expo_temp_16;
           	  }
          	}

          	if ( denom_temp_u64 > num_temp_u64 ) {
          	  calc_temp_u32 = denom_temp_u64 / num_temp_u64;
          	  if ( calc_temp_u32 > 2 ) {
           	    num_temp_u64 *= 5;
           	    denom_temp_u64 /= 2;
           	    --expo_temp_16;
           	  } 
           	}

           	if ( num_temp_u64 > denom_temp_u64 ) {
              calc_temp_u64_0 = num_temp_u64 / int32_max;
            }
            else {
              calc_temp_u64_0 = denom_temp_u64 / int32_max;
            }
            ++calc_temp_u64_0;
            num_temp_u32 = num_temp_u64 / calc_temp_u64_0;
            denom_temp_u32 = denom_temp_u64 / calc_temp_u64_0;

            if ( num_temp_u32 == 0 ) {
            	expo_temp_16 = 0;
            }

            mem_stack[ mem_pointer ].expo = expo_temp_16;
            if ( expo_temp_16 >= 115 ) {
            	mem_stack[ mem_pointer ].expo = 115;
            }
            if ( expo_temp_16 <= -115 ) {
            	mem_stack[ mem_pointer ].expo = -115;
            }

            Reduce_Number();
                        
            mem_stack[ mem_pointer ].num = num_temp_u32;
            mem_stack[ mem_pointer ].num *= test_signum_8;
            mem_stack[ mem_pointer ].denom = denom_temp_u32;
            
            Error_Test();            
            if ( Start_input != Display_Error ) {
              mem_extra_stack[ mem_extra_test ].expo = mem_stack[ mem_pointer ].expo;
              mem_extra_stack[ mem_extra_test ].num = mem_stack[ mem_pointer ].num;
              mem_extra_stack[ mem_extra_test ].denom = mem_stack[ mem_pointer ].denom;
              Display_Number();
              display_string[Memory_1] = 'm';
              display_string[Memory_0] = 48 + mem_extra_test;
            }
            else {
              mem_pointer = 1;
            }
            
            Start_input = Display_M_Plus;
            Beep_on = true;
            
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
          Beep_on = true;
          break;

        case 113:                //    e^x
          if ( Debug_Level == 5 ) {
            Serial.println("e^x");
          }
          Beep_on = true;
          break;

        case 114:                //    2^x
          if ( Debug_Level == 5 ) {
            Serial.println("2^x");
          }
          Beep_on = true;
          break;

        case 115:                //    log(x)
          if ( Debug_Level == 5 ) {
            Serial.println("log(x)");
          }
          Beep_on = true;
          break;

        case 116:                //    10^x
          if ( Debug_Level == 5 ) {
            Serial.println("10^x");
          }
          Beep_on = true;
          break;

        case 117:                //    _x^2_
          if ( Debug_Level == 5 ) {
            Serial.println("x^2");
          }
          Beep_on = true;
          if ( Start_input < Input_Operation_0 ) {      // Input Number
            Get_Number();
          }
          if ( Start_input == Display_M_Plus ) {
            mem_pointer = 1;
            mem_stack[ mem_pointer ].expo = mem_stack[ 0 ].expo;
            mem_stack[ mem_pointer ].num = mem_stack[ 0 ].num;
            mem_stack[ mem_pointer ].denom = mem_stack[ 0 ].denom;
            Start_input = Input_Operation_0;
          }
          if ( Start_input == Display_Result ) {
            mem_pointer = 1;
            mem_stack[ mem_pointer ].expo = mem_extra_stack[ 0 ].expo;
            mem_stack[ mem_pointer ].num = mem_extra_stack[ 0 ].num;
            mem_stack[ mem_pointer ].denom = mem_extra_stack[ 0 ].denom;
            Start_input = Input_Operation_0;
          }
          if ( Start_input == Input_Operation_0 ) {
            mem_pointer = 0;
            expo_temp_16 =  mem_stack[ mem_pointer ].expo;
            expo_temp_16 *= 2;
            mem_stack[ mem_pointer ].num = abs( mem_stack[ mem_pointer ].num );
            mem_stack[ mem_pointer ].denom = abs( mem_stack[ mem_pointer ].denom);

            gcd_temp_32 = gcd_iter_32(mem_stack[mem_pointer].num, mem_stack[mem_pointer].denom);
            mem_stack[mem_pointer].num /= gcd_temp_32;
            mem_stack[mem_pointer].denom /= gcd_temp_32;

            if ( Debug_Level == 10 ) {
              Serial.print("= ");
              Serial.print(mem_stack[mem_pointer].num);
              Serial.print(" / ");
              Serial.print(mem_stack[mem_pointer].denom);
              Serial.print(" x 10^ ");
              Serial.println(mem_stack[mem_pointer].expo);
            }

            if ( (mem_stack[mem_pointer].num < int15_max) || (mem_stack[mem_pointer].denom < int15_max) ) {
            	num_temp_u32 =  mem_stack[mem_pointer].num;
            	num_temp_u32 *= mem_stack[mem_pointer].num;
            	num_temp_u32 *= 2;
            	denom_temp_u32 =  mem_stack[mem_pointer].denom;
            	denom_temp_u32 *= mem_stack[mem_pointer].denom;
            	denom_temp_u32 *= 2;
            	
            	if ( num_temp_u32 > denom_temp_u32 ) {
            	  calc_temp_16_0 = num_temp_u32 / denom_temp_u32;
            	  if ( calc_temp_16_0 > 2 ) {
            	    num_temp_u32 /= 2;
            	    denom_temp_u32 *= 5;
            	    ++expo_temp_16;
            	  }
            	}

            	if ( denom_temp_u32 > num_temp_u32 ) {
            	  calc_temp_16_0 = denom_temp_u32 / num_temp_u32;
            	  if ( calc_temp_16_0 > 2 ) {
            	    num_temp_u32 *= 5;
            	    denom_temp_u32 /= 2;
            	    --expo_temp_16;
            	  } 
            	}
            	
              Expand_Number();
            	
              if ( Debug_Level == 10 ) {
                Serial.print("= ");
                Serial.print(num_temp_u32);
                Serial.print(" / ");
                Serial.print(denom_temp_u32);
                Serial.print(" x 10^ ");
                Serial.println(expo_temp_16);    
              }
  
              mem_stack[ mem_pointer ].num = num_temp_u32;
              mem_stack[ mem_pointer ].denom = denom_temp_u32;
  
            } else {
            	num_temp_u64 =  mem_stack[mem_pointer].num;
            	num_temp_u64 *= mem_stack[mem_pointer].num;
            	num_temp_u64 *= 2;
            	denom_temp_u64 =  mem_stack[mem_pointer].denom;
            	denom_temp_u64 *= mem_stack[mem_pointer].denom;
            	denom_temp_u64 *= 2;
              
            	if ( num_temp_u64 > denom_temp_u64 ) {
            	  calc_temp_32_0 = num_temp_u64 / denom_temp_u64;
            	  if ( calc_temp_32_0 > 2 ) {
            	    num_temp_u64 /= 2;
            	    denom_temp_u64 *= 5;
            	    ++expo_temp_16;
            	  }
            	}

            	if ( denom_temp_u64 > num_temp_u64 ) {
            	  calc_temp_32_0 = denom_temp_u64 / num_temp_u64;
            	  if ( calc_temp_32_0 > 2 ) {
            	    num_temp_u64 *= 5;
            	    denom_temp_u64 /= 2;
            	    --expo_temp_16;
            	  } 
            	}

            	if ( num_temp_u64 > denom_temp_u64 ) {
                calc_temp_u64_0 = num_temp_u64 / int32_max;
              } else {
                calc_temp_u64_0 = denom_temp_u64 / int32_max;
              }
              ++calc_temp_u64_0;
              num_temp_u32 = num_temp_u64 / calc_temp_u64_0;
              denom_temp_u32 = denom_temp_u64 / calc_temp_u64_0;

              if ( Debug_Level == 10 ) {
                Serial.print("= __ ");
                Serial.print(num_temp_u32);
                Serial.print(" / ");
                Serial.print(denom_temp_u32);
                Serial.print(" x 10^ ");
                Serial.println(expo_temp_16);
              }

              Reduce_Number();
              
              if ( Debug_Level == 10 ) {
                Serial.print("= ____ ");
                Serial.print(num_temp_u32);
                Serial.print(" / ");
                Serial.print(denom_temp_u32);
                Serial.print(" x 10^ ");
                Serial.println(expo_temp_16);
              }
              
              mem_stack[ mem_pointer ].num = num_temp_u32;
              mem_stack[ mem_pointer ].denom = denom_temp_u32;
            }

            mem_stack[ mem_pointer ].expo = expo_temp_16;
            if ( expo_temp_16 >= 115 ) {
            	mem_stack[ mem_pointer ].expo = 115;
            }
            if ( expo_temp_16 <= -115 ) {
            	mem_stack[ mem_pointer ].expo = -115;
            }
            Error_Test();            
            if ( Start_input != Display_Error ) {
              Display_Number();
            }
            else {
              mem_pointer = 1;
            }
          }
          break;

        case 118:                //    sqrt()
          if ( Debug_Level == 5 ) {
            Serial.println("sqrt()");
          }
          Beep_on = true;
          break;

        case 119:                //    x!
          if ( Debug_Level == 5 ) {
            Serial.println("x!");
          }
          Beep_on = true;
          break;

        case 120:                //    _EE_
          if ( Debug_Level == 5 ) {
            Serial.println("EE");
          }
          if ( Number_count != Zero_count ) {
            if ( Start_input == Input_Mantisse ) {
            	Pointer_memory = display_string[Cursor_pos];
            	display_string[Cursor_pos] = '#';
              display_string[Expo_point] = '.';
              Start_input = Input_Expo;
              Beep_on = true;
              if (Init_expo == true) {
                Init_expo = false;
                display_string[Expo_1] = '0';
                display_string[Expo_0] = '0';
              }
              Display_new = true;
              break;
            }
            if ( Start_input == Input_Expo ) {
            	display_string[Cursor_pos] = Pointer_memory;
              display_string[Expo_point] = ' ';
              Start_input = Input_Mantisse;
              Beep_on = true;
              Display_new = true;
              break;
            }
          }
          break;

        case 124:                //    AGM
          if ( Debug_Level == 5 ) {
            Serial.println("AGM");
          }
          Beep_on = true;
          break;

        case 127:                 //    -->
          if ( Debug_Level == 5 ) {
            Serial.println("->>");
          }
          Beep_on = true;
          break;

        case 148:                 //    <<-->>
          if ( Debug_Level == 5 ) {
            Serial.println("<<-->>");
          }
          Beep_on = true;
          break;

        case 149:                 //    __/
          if ( Debug_Level == 5 ) {
            Serial.println("__/");
          }
          Beep_on = true;
          break;

        case 150:                 //    ° ' ''
          if ( Debug_Level == 5 ) {
            Serial.println("o_-_=");
          }
          Beep_on = true;
          break;

        case 160:                 //   SM(CMs)
          if ( Debug_Level == 5 ) {
            Serial.println("CMs");
          }
          Beep_on = true;
          for (index_mem = 1; index_mem <= 9; ++index_mem) {
            mem_extra_stack[ index_mem ].expo = -99;
            mem_extra_stack[ index_mem ].num = int32_max;
            mem_extra_stack[ index_mem ].denom = int32_max;
  	      }
          break;

        case 161:                 //    SM(1)
        case 162:                 //    SM(2)
        case 163:                 //    SM(3)
        case 164:                 //    SM(4)
        case 165:                 //    SM(5)
        case 166:                 //    SM(6)
        case 167:                 //    SM(7)
        case 168:                 //    SM(8)
        case 169:                 //    SM(9)
        	mem_extra_test = Switch_Code - SM_0;
          if ( Debug_Level == 5 ) {
            Serial.print("SM(");
            Serial.print(mem_extra_test);
            Serial.println(")");
          }
          Beep_on = true;
          if ( Start_input < Input_Operation_0 ) {      // Input Number
            if ( Number_count != Zero_count ) {
            	if ( Mantisse_change == true ) {
                Get_Mantisse();
              }
              else {
                if ( Expo_change = true ) {
                	Get_Expo_change();
                }
              }
              if ( Start_input != Display_Error ) {
                mem_stack[ 0 ].num = mem_stack[ mem_pointer ].num;
                mem_stack[ 0 ].denom = mem_stack[ mem_pointer ].denom;
                mem_stack[ 0 ].expo = mem_stack[ mem_pointer ].expo;
                Start_input = Input_Operation_0;
              }
            }
          }
          if ( Start_input == Display_Result ) {
            mem_pointer = 1;
            mem_stack[ mem_pointer ].expo = mem_extra_stack[ 0 ].expo;
            mem_stack[ mem_pointer ].num = mem_extra_stack[ 0 ].num;
            mem_stack[ mem_pointer ].denom = mem_extra_stack[ 0 ].denom;
            Start_input = Input_Operation_0;
          }
          if ( Start_input == Input_Operation_0 ) {
            mem_extra_stack[ mem_extra_test ].expo = mem_stack[ mem_pointer ].expo;
            mem_extra_stack[ mem_extra_test ].num = mem_stack[ mem_pointer ].num;
            mem_extra_stack[ mem_extra_test ].denom = mem_stack[ mem_pointer ].denom;
            Display_Number();
            display_string[Memory_1] = 'm';
            display_string[Memory_0] = 48 + mem_extra_test;
            Beep_on = true;
          }
          break;

        case 170:                //    x^3
          if ( Debug_Level == 5 ) {
            Serial.println("INT");
          }
          Beep_on = true;
          break;

        case 171:                //    cbrt()
          if ( Debug_Level == 5 ) {
            Serial.println("FRC");
          }
          Beep_on = true;
          break;

      }
    }

    Switch_Code =   0;
    index_5min  = 255;
  }

  if ( time_7500ms == true ) { // here al that will make in 7.5 sec
    time_7500ms = false;
    ++index_5min;              // Switch Off in 5min
    if ( index_5min == 23 ) {  // 23 - after 3min
      if (Pendular_on == false) {
        Pendular_on = true;
        Start_mem = Start_input;
        Beep_on_off = false;
      }
    }
    if ( index_5min == 39 ) {  // 39
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
      Serial.print("\t");
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
      Serial.print("\t");
      Serial.println(time);
    }

    if ( count_100ms == 8 ) {  // after 3 Second --> Input_Cursor
      Start_input = Start_Mode;
      Beep_on = true;
    }
  }

  if ( Display_new == true ) { // Display refresh
    Display_new = false;
    index_LED = 0;
    for (index_a = 0; index_a <= Digit_Count; ++index_a) {
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
    Display_change = true;
  }

  if ( time_10ms == true ) {   // here al that will make in 95 Hz
    time_10ms = false;
    ++count_10ms;              // (10 + 30/57) ms
    ++index_10ms;

    if ( Countdown_OFF > 0 ) {
      --Countdown_OFF;

      switch (Countdown_OFF) { // Off

      	case 0:
      	case 1:
       	case 2:
      	case 3:
      	case 4:
      	case 5:
      		digitalWrite(On_Off_PIN, LOW);
      	  break;

      	case 14:
      	case 25:
      	case 36:
          Beep_on = true;
      	  break;
      }
    }

    if ( Start_input == Start_Mode ) {
      Clear_String();
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
            Serial.println("\t-15.0");
          }
          break;

        case 8:
        case 175:
          display_b[0] = 34;
          display_b[1] = 0;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t-14.0");
          }
          break;

        case 14:
        case 171:
          display_b[0] = 2;
          display_b[1] = 32;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t-13.0");
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
            Serial.println("\t-12.0");
          }
          break;

        case 21:
        case 165:
          display_b[1] = 2;
          display_b[2] = 48;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t-11.0");
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
            Serial.println("\t-10.0");
          }
          break;

        case 27:
        case 159:
          display_b[2] = 6;
          display_b[3] = 48;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t-9.0");
          }
          break;

        case 30:
          Beep_on = true;
        case 157:
          display_b[2] = 0;
          display_b[3] = 54;
          display_b[4] = 0;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t-8.0");
          }
          break;

        case 32:
        case 155:
          display_b[3] = 6;
          display_b[4] = 16;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t-7.0");
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
            Serial.println("\t-6.0");
          }
          break;

        case 36:
        case 151:
          display_b[4] = 4;
          display_b[5] = 16;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t-5.0");
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
            Serial.println("\t-4.0");
          }
          break;

        case 40:
        case 147:
          display_b[5] = 4;
          display_b[6] = 16;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t-3.0");
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
            Serial.println("\t-2.0");
          }
          break;

        case 44:
        case 143:
          display_b[6] = 4;
          display_b[7] = 16;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t-1.0");
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
            Serial.println("\t0.0");
          }
          break;

        case 48:
        case 139:
          display_b[7] = 4;
          display_b[8] = 16;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t1.0");
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
            Serial.println("\t2.0");
          }
          break;

        case 52:
        case 135:
          display_b[8] = 4;
          display_b[9] = 16;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t3.0");
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
            Serial.println("\t4.0");
          }
          break;

        case 56:
        case 131:
          display_b[9] = 4;
          display_b[10] = 16;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t5.0");
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
            Serial.println("\t6.0");
          }
          break;

        case 60:
        case 127:
          display_b[10] = 4;
          display_b[11] = 48;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t7.0");
          }
          break;

        case 125:
          Beep_on = true;
        case 62:
          display_b[10] = 0;
          display_b[11] = 54;
          display_b[12] = 0;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t8.0");
          }
          break;

        case 64:
        case 122:
          display_b[11] = 6;
          display_b[12] = 48;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t9.0");
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
            Serial.println("\t10.0");
          }
          break;

        case 70:
        case 116:
          display_b[12] = 6;
          display_b[13] = 32;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t11.0");
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
            Serial.println("\t12.0");
          }
          break;

        case 76:
        case 109:
          display_b[13] = 2;
          display_b[14] = 32;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t13.0");
          }
          break;

        case 80:
        case 103:
          display_b[13] = 0;
          display_b[14] = 34;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t14.0");
          }
          break;

        case 86:
        case 94:
          display_b[14] = 2;

          if ( Debug_Level == 6 ) {
            time = millis();
            Serial.print(time);
            Serial.println("\t15.0");
          }
          break;

      }
      Display_change = true;
    }

    if ( index_pendel_a == 189 ) {
      index_pendel_a = 255;
    }
    ++index_pendel_a;

    if ( Switch_old > Switch_down ) {
    	Switch_delta = Switch_old - Switch_down;

      switch (Switch_delta) {

      	case 1:
      		if ( SM_on == false ) {
            Switch_Code = 63;   //                DISP_hyp_off
          }
          if ( EE_FN_xx == true ) {
          	if (Switch_down > 4) {
             EE_FN_xx = false;
              Switch_Code = 31;   //              SM_on
            }
          }
          break;

      	case 4:
      		if ( hyp_on == false ) {
            Switch_Code = 63;   //                DISP_hyp_off
          }
          if ( EE_FN_xx == true ) {
          	if (Switch_down > 2) {
              EE_FN_xx = false;
              Switch_Code = 62;   //              hyp_on
            }
          }
          break;

      	case 2:
      		if ( disp_on == false ) {
            Switch_Code = 63;   //                DISP_hyp_off
          }
          if ( EE_FN_xx == true ) {
          	if (Switch_down > 4) {
            	EE_FN_xx = false;
              Switch_Code = 58;   //              disp_on
            }
          }
          break;

      	case 3:
      	case 5:
      	case 6:
      	case 7:
          Switch_Code = 63;   //                  DISP_hyp_off
          break;
      }
    }

    if ( Switch_down > Switch_old ) {
      switch (Switch_down) {

        case 7:        //   "EE" + "FN"    + "="  three Switch pressed
          Switch_Code = 59;   //                  EE_FN_=
          break;

        case 35:       //   "EE" + "FN"    + ")"  three Switch pressed
          Switch_Code = 64;   //                  rad
          break;

        case 70:       //   "FN" +  "="    + "<"  three Switch pressed
          Switch_Code = 34;   //                  lg2
          break;

        case 259:  // "EE" + "FN" + "Light down"  three Switch pressed
          Switch_Code = 37;   //                  grd
          break;

        case 518:       //   "FN" +  "="  + "CE"  three Switch pressed
          Switch_Code = 112;    //                ln(x)
          break;

        case 2051:   // "EE" + "FN" + "Light up"  three Switch pressed
          Switch_Code = 123;  //                  Off
          break;

        case 262150:   //     "FN" + "="  +  "4"  three Switch pressed
          Switch_Code = 74;  //                   asinh(x)
          break;

        case 524294:   //     "FN" + "="  +  "5"  three Switch pressed
          Switch_Code = 75;  //                   acosh(x)
          break;

        case 1048582:  //     "FN" + "="  +  "6"  three Switch pressed
          Switch_Code = 76;  //                   atanh(x)
          break;

        case 2097158:   //    "FN" + "="  +  "7"  three Switch pressed
          Switch_Code = 68;  //                   asin(x)
          break;

        case 4194310:  //     "FN" + "="  +  "8"  three Switch pressed
          Switch_Code = 69;  //                   acos(x)
          break;

        case 8388614:  //     "FN" + "="  +  "9"  three Switch pressed
          Switch_Code = 70;  //                   atan(x)
          break;

        case 3:  //           "EE" + "FN"         two Switch pressed
          Switch_Code = 62;   //                  _hyp_on_
          break;

        case 5:  //           "EE" + "="          two Switch pressed
          Switch_Code = 58;   //                  _DISP_on_
          break;

        case 6:  //           "FN" + "="          two Switch pressed
          Switch_Code = 31;   //                  _SM_on_
          break;

        case 9:        //     "EE" + "1/x"        two Switch pressed
        case 27:       //     "EE" + "FN" + "1/x" three Switch pressed
          Switch_Code = 44;   //                  _e_
          break;

        case 65:       //     "EE" + "<"          two Switch pressed
        case 195:      //     "EE" + "FN" + "<"   three Switch pressed
        //  Switch_Code = 34;   //                  _lg2_
          break;

        case 260:      //     "="  +  "/"         two Switch pressed
        case 257:      //     "EE" +  "/"         two Switch pressed
          Switch_Code = 39;   //                  _EE-3_
          break;

        case 2052:     //     "="  +  ")"         two Switch pressed
        case 2049:     //     "EE" +  ")"         two Switch pressed
          Switch_Code = 38;   //                  _EE+3_
          break;

        case 32770:    //     "FN" +  "1"         two Switch pressed
          Switch_Code = 88;   //                  _y_expo_
          break;

        case 32774:    //     "FN" +  "=" + "1"   three Switch pressed
          Switch_Code = 89;   //                  _y_root_
          break;

        case 65538:    //     "FN" +  "2"         two Switch pressed
          Switch_Code = 117;  //                  x^2
          break;

        case 65542:    //     "FN" +  "=" + "2"   three Switch pressed
          Switch_Code = 118;  //                  sqrt()
          break;

        case 131074:   //     "FN" +  "3"         two Switch pressed
          Switch_Code = 170;  //                  INT
          break;

        case 131078:    //    "FN" +  "=" + "3"   three Switch pressed
          Switch_Code = 171;  //                  FRC
          break;

        case 262146:   //     "FN" +  "4"         two Switch pressed
          Switch_Code = 71;  //                   sinh(x)
          break;

        case 524290:   //     "FN" +  "5"         two Switch pressed
          Switch_Code = 72;  //                   cosh(x)
          break;

        case 1048578:  //     "FN" +  "6"         two Switch pressed
          Switch_Code = 73;  //                   tanh(x)
          break;

        case 2097154:   //    "FN" +  "7"         two Switch pressed
          Switch_Code = 65;  //                   sin(x)
          break;

        case 4194306:  //     "FN" +  "8"         two Switch pressed
          Switch_Code = 66;  //                   cos(x)
          break;

        case 8388610:  //     "FN" +  "9"         two Switch pressed
          Switch_Code = 67;  //                   tan(x)
          break;

        case 10:       //     FN - 1/x            two Switch pressed
          Switch_Code = 36;   //                  Pi()
          break;

        case 18:       //     FN - _+_            two Switch pressed
          Switch_Code = 92;   //                  //
          break;

        case 34:       //     FN - _-_            two Switch pressed
          Switch_Code = 94;   //                  _/p/_  Phytagoras
          break;

        case 66:       //     FN - _<_            two Switch pressed
          Switch_Code = 114;  //                  _2^x_
          break;

        case 130:      //     FN - _*_            two Switch pressed
          Switch_Code = 124;   //                 _AGM_
          break;

        case 258:      //     FN - _/_            two Switch pressed
          Switch_Code = 93;   //                  Light down
          break;

        case 513:      //     EE - _CE_           two Switch pressed
            Switch_Code = 30;  //                 <--
          break;

        case 514:      //     FN - _CE_           two Switch pressed
          Switch_Code = 113;  //                  e^x
          break;

        case 1025:     //     EE - _(_            two Switch pressed
          Switch_Code = 127;  //                  -->
          break;

        case 1026:     //     FN - _(_            two Switch pressed
          Switch_Code = 116;  //                  10^x
          break;

        case 1540:     //    "=" + "CE"  +  "("   three Switch pressed
          Switch_Code = 148;  //                  <<-->>
          break;

        case 1028:     //    "="  + "("           two Switch pressed
        case 3078:     //    "FN" + "="  +  "("   three Switch pressed
          Switch_Code = 115;  //                  log(x)
          break;

        case 2050:     //     FN - _)_            two Switch pressed
          Switch_Code = 91;   //                  Light up
          break;

        case 4098:     //     FN  +  "0"          two Switch pressed
          Switch_Code = 119;  //                  x!
          break;

        case 8194:     //     FN - "."            two Switch pressed
          Switch_Code = 122; //                   beep
          break;

        case 12288:     //    "0" + "."           two Switch pressed
          Switch_Code = 150; //                   "° ' ''" hh:mm:ss  Input
          break;

        case 16386:    //     FN - "+/-"          two Switch pressed
          Switch_Code = 121; //                   clock
          break;

        case 24576:     //    "." + "+/-"         two Switch pressed
          Switch_Code = 149; //                   Fraction a_b/c     Input
          break;

        case 4097:     //     EE - "0"            two Switch pressed
          Switch_Code = 77;  //                   RM(0)
          break;

        case 32769:    //     EE - "1"            two Switch pressed
          Switch_Code = 78;  //                   RM(1)  
          break;

        case 65537:    //     EE - "2"            two Switch pressed
          Switch_Code = 79;  //                   RM(2)
          break;

        case 131073:   //     EE - "3"            two Switch pressed
          Switch_Code = 80;  //                   RM(3)
          break;

        case 262145:   //     EE - "4"            two Switch pressed
          Switch_Code = 81;  //                   RM(4)
          break;

        case 524289:   //     EE - "5"            two Switch pressed
          Switch_Code = 82;  //                   RM(5)
          break;

        case 1048577:  //     EE - "6"            two Switch pressed
          Switch_Code = 83;  //                   RM(6)
          break;

        case 2097153:  //     EE - "7"            two Switch pressed
          Switch_Code = 84;  //                   RM(7)
          break;

        case 4194305:  //     EE - "8"            two Switch pressed
          Switch_Code = 85;  //                   RM(8)
          break;

        case 8388609:  //     EE - "9"            two Switch pressed
          Switch_Code = 86;  //                   RM(9)
          break;

        case 4101:     //     "EE" +  "=" +  "0"  three Switch pressed
          Switch_Code = 32;  //                   FIX_dms
          break;

        case 32773:    //     "EE" +  "=" +  "1"  three Switch pressed
          Switch_Code = 87;  //                   FIX_a_b/c
          break;

        case 65541:    //     "EE" +  "=" +  "2"  three Switch pressed
          Switch_Code = 95;  //                   FIX_2
          break;

        case 131077:    //    "EE" +  "=" +  "3"  three Switch pressed
          Switch_Code = 96;  //                   FIX_3
          break;

        case 262149:    //    "EE" +  "=" +  "4"  three Switch pressed
          Switch_Code = 97;  //                   FIX_4
          break;

        case 524293:    //    "EE" +  "=" +  "5"  three Switch pressed
          Switch_Code = 98;  //                   FIX_5
          break;

        case 1048581:   //    "EE" +  "=" +  "6"  three Switch pressed
          Switch_Code = 99;  //                   FIX_6
          break;

        case 2097157:   //    "EE" +  "=" +  "7"  three Switch pressed
          Switch_Code = 100;  //                  FIX_7
          break;

        case 4194309:   //    "EE" +  "=" +  "8"  three Switch pressed
          Switch_Code = 101;  //                  FIX_8
          break;

        case 8388613:   //    "EE" +  "=" +  "9"  three Switch pressed
          Switch_Code = 102;  //                  FIX_E24
          break;

        case 32772:    //     "=" + M_plus(1)     two Switch pressed
          Switch_Code = 103;
          break;

        case 65540:    //     "=" + M_plus(2)     two Switch pressed
          Switch_Code = 104;
          break;

        case 131076:   //     "=" + M_plus(3)     two Switch pressed
          Switch_Code = 105;
          break;

        case 262148:   //     "=" + M_plus(4)     two Switch pressed
          Switch_Code = 106;
          break;

        case 524292:   //     "=" + M_plus(5)     two Switch pressed
          Switch_Code = 107;
          break;

        case 1048580:  //     "=" + M_plus(6)     two Switch pressed
          Switch_Code = 108;
          break;

        case 2097156:  //     "=" + M_plus(7)     two Switch pressed
          Switch_Code = 109;
          break;

        case 4194308:  //     "=" + M_plus(8)     two Switch pressed
          Switch_Code = 110;
          break;

        case 8388612:  //     "=" + M_plus(9)     two Switch pressed
          Switch_Code = 111;
          break;

        case 16390:    //     "=" + SM(CMs)      three Switch pressed
          Switch_Code = 160;  //                  CMs
          break;

        case 32771:    //     "EE" + "FN" + "1"   three Switch pressed
          Switch_Code = 161;  //                  x->M(1)
          break;

        case 65539:    //     "EE" + "FN" + "2"   three Switch pressed
          Switch_Code = 162;  //                  x->M(2)
          break;

        case 131075:   //     "EE" + "FN" + "3"   three Switch pressed
          Switch_Code = 163;  //                  x->M(3)
          break;

        case 262147:   //     "EE" + "FN" + "4"   three Switch pressed
          Switch_Code = 164;  //                  x->M(4)
          break;

        case 524291:   //     "EE" + "FN" + "5"   three Switch pressed
          Switch_Code = 165;  //                  x->M(5)
          break;

        case 1048579:  //     "EE" + "FN" + "6"   three Switch pressed
          Switch_Code = 166;  //                  x->M(6)
          break;

        case 2097155:  //     "EE" + "FN" + "7"   three Switch pressed
          Switch_Code = 167;  //                  x->M(7)
          break;

        case 4194307:  //     "EE" + "FN" + "8"   three Switch pressed
          Switch_Code = 168;  //                  x->M(8)
          break;

        case 8388611:    //   "EE" + "FN" + "9"   three Switch pressed
          Switch_Code = 169;  //                  x->M(9)
          break;

        case 11:       //    "pi()"               defect
        case 19:       //    "pi()"               defect
        case 67:       //    "<"                  defect          
        case 72:       //    "<--"                defect          
        case 131:      //    "<"                  defect          
        case 194:      //    "<"                  defect          
        case 585:      //    "e()"                defect
        case 515:      //    "CE"                 defect
        case 576:      //    "CE"                 defect
        case 1027:     //    "("                  defect
        case 1030:     //    "("                  defect
        case 1537:     //    "("                  defect
        case 1538:     //    "("                  defect
        case 2054:     //    "("                  defect
        case 196614:   //	   "2" + "3" + "SM_on" defect
        case 1572870:  //	   "5" + "6" + "SM_on" defect
        case 12582918: //	   "8" + "9" + "SM_on" defect
          break;

        default:
          Switch_delta = Switch_down - Switch_old;

          switch (Switch_delta) {

            case 1:          //    EE
            case 3:
              Switch_Code = 120;
              break;

            case 4:          //    _=_
            case 6:
              Switch_Code = 61;
              break;

            case 8:          //    _1/x_
            case 24:
              Switch_Code = 33;
              break;

            case 16:         //    _+_
              Switch_Code = 43;
              break;

            case 32:         //    _-_
            case 48:
              Switch_Code = 45;
              break;

            case 64:         //    <--
            case 192:
              Switch_Code = 60;
              break;

            case 128:        //    _*_
              Switch_Code = 42;
              break;

            case 256:        //    _/_
            case 384:
              Switch_Code = 47;
              break;

            case 512:        //    _CE_
            case 1536:
              Switch_Code = 90;
              break;

            case 1024:       //    (
              Switch_Code = 40;
              break;

            case 2048:       //    )
            case 3072:
              Switch_Code = 41;
              break;

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

            case 32768:      //    1
            case 98304:
              Switch_Code = 49;
              break;

            case 65536:      //    2
              Switch_Code = 50;
              break;

            case 131072:     //    3
            case 196608:
              Switch_Code = 51;
              break;

            case 262144:     //    4
            case 786432:
              Switch_Code = 52;
              break;

            case 524288:     //    5
              Switch_Code = 53;
              break;

            case 1048576:    //    6
            case 1572864:
              Switch_Code = 54;
              break;

            case 2097152:    //    7
            case 6291456:
              Switch_Code = 55;
              break;

            case 4194304:    //    8
              Switch_Code = 56;
              break;

            case 8388608:    //    9
            case 12582912:
              Switch_Code = 57;
              break;
          }
      }
    }

    time_down = millis();
    time = time_down - time_old;

    if ( Debug_Level == 3 ) {
      Serial.print(time);
      Serial.print("\t");
      Serial.println(taste[2]);
    }

    if ( Switch_down != Switch_old ) {
      if ( Debug_Level == 4 ) {
        Serial.print(time);
        Serial.print("\t");
        Serial.println(Switch_down);
      }
    }

    Switch_old = Switch_down;
    time_old = time_down;

  }
}

/// --------------------------
/// CuSMm ISR Timer Routine
/// --------------------------
void timerIsr() {

  ++index_Switch;
  ++index_TIME;

  switch (index_TIME) {

    case 2:       // +15
    case 16:      // +14
    case 30:      // +14
    case 44:      // +14
      Timer1.setPeriod(Time_HIGH);  // sets timer1 to a period of 264 microseconds
      break;

    case 4:       //  2x
    case 18:      //  2x
    case 32:      //  2x
    case 47:      //  3x
      Timer1.setPeriod(Time_LOW);   // sets timer1 to a period of 263 microseconds
      break;

    case 56:
      index_TIME = 255;
      break;
  }

  index_Display_old = index_Display;

  if ( index_Switch % 5 == 0 ) {   // Segmente ausschalten --> Segment "a - f - point"
    for (Digit = 0; Digit < Digit_Count; ++Digit) {
      digitalWrite(index_display[Digit], HIGH);
    //  fastDigitalWrite(index_display[Digit], HIGH);
    }
    if (Display_change == true) {
      for (Digit = 0; Digit < Digit_Count; ++Digit) {
        display_a[Digit] = display_b[Digit];
      }
      Display_change = false;
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

  if ( bitRead(Switch_down, Switch_number) == 0 ) {
    if (bitRead(Switch_up, Switch_number) == 0) {    //  Pos. 0
      if (taste[Switch_number] < Switch_up_b) {
        bitWrite(Switch_up, Switch_number, 1);
      }
    }
    else {                                           //  Pos. 1
      if (taste[Switch_number] < Switch_down_b) {
        bitWrite(Switch_down, Switch_number, 1);
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
        bitWrite(Switch_down, Switch_number, 0);
        taste[Switch_number] = Switch_down_start;
      }
    }
  }

  if ( Beep_on == true ) {
    --Beep_count;
    if (Beep_count > 0) {
      if ( Beep_on_off == true ) {
        digitalWrite( Beep, bitRead(beep_patt, Beep_count) );    // Toggle BEEP
      }
    }
    else {
      digitalWrite(Beep, LOW);
      Beep_on = false;
      Beep_count = max_Beep_count;
    }
  }
  else {
    digitalWrite(Beep, LOW);
  }

}

