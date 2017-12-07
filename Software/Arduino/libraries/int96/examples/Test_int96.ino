#include <int96.h>
// Original:  http://www.naughter.com/int96.html
// https://github.com/JensGrabner/snc98_Slash-Number-Calculator/tree/master/Software/Arduino/libraries/int96

#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include <itoa_ljust.h>
// https://github.com/JensGrabner/snc98_Slash-Number-Calculator/tree/master/Software/Arduino/libraries/itoa_ljust

// uint64_t my_xx_bit;
// uint64_t add;
 int96_a my_xx_bit;
 int96_a add = 1;
 int96_a mul = 1;
 int96_a mul_10 = 10;

 int96_a my_start_bit;
 int96_a test;
 
char  display_string[33];

uint32_t time_start;
uint32_t time_end;
uint32_t time;

// ------------------------------------------------

void setup() {


  Serial.begin(115200);
	
		// 39614081257 = 2^95 / mul
  my_start_bit.hi  = 0x7FFFFFFF;
  my_start_bit.mid = 0xFFFFFFFF;
  my_start_bit.lo  = 0xFFFFFFFF;



} // end setup

// ------------------------------------------------

void loop() {
  delay(10);

  time_start = millis();
	
  
//  itoa_(my_start_bit, display_string);
//  Serial.println(display_string);



for ( uint8_t ii_a = 0; ii_a < 27 ; ii_a += 1 ) {
  delay(10);
  mul *= mul_10;

  my_xx_bit = my_start_bit / mul;
  itoa_(my_xx_bit, display_string);
  Serial.println(display_string);

  my_start_bit.mul_div95(my_xx_bit,test);  // const my_start_bit. * my_xx_bit = test;
  itoa_(test, display_string);
  Serial.println(display_string);
}  

  time_end = millis();
  time = time_end - time_start;
  Serial.print("Time: ");
  Serial.print(time);
  Serial.print(" ms ");
  Serial.println(display_string);
 
delay(20000000);

}


