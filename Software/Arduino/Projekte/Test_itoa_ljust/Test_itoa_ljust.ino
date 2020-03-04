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
 int96_a my_test_pos;
 int96_a my_test_neg;
 int96_a add;
 int96_a mul_10 = 10;
 int96_a my_start_bit;
 
char  display_string[33];

uint32_t time_start;
uint32_t time_end;
uint32_t time;

uint16_t ii_end = 900;
// ------------------------------------------------

void setup() {
	
	my_xx_bit = 100;
	add       =   1;

  Serial.begin(115200);
  delay(120);
 
} // end setup

// ------------------------------------------------

void loop() {

  delay(80);

for ( uint8_t ii_a = 0; ii_a < 27 ; ii_a += 1 ) {
  if ( ii_a == 26 ) ii_end = 297;
  time_start = millis();

  for ( uint16_t ii_b = 0; ii_b < ii_end; ii_b += 1 ) {
  	 my_test_pos  = my_xx_bit;
  	 my_test_neg  = 0;
  	 my_test_neg -= my_test_pos;
  	 
  	 my_test_pos.cbrt(my_test_pos);
     itoa_(my_test_pos, display_string);
     Serial.println(display_string);

  	 my_test_neg.cbrt(my_test_neg);
     itoa_(my_test_neg, display_string);
     Serial.println(display_string);

     my_xx_bit += add;
  }
  /*
  itoa_(my_xx_bit, display_string);
  
  time_end = millis();
  time = time_end - time_start;
  Serial.print("Time: ");
  Serial.print(time);
  Serial.print(" ms ");
  Serial.println(display_string);
  */
  add *= mul_10;
}  
 
delay(20000000);

}


