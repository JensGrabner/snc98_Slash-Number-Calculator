#include <int96.h>
// Original:  http://www.naughter.com/int96.html
// https://github.com/JensGrabner/snc98_Slash-Number-Calculator/tree/master/Software/Arduino/libraries/int96

#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include <itoa_ljust.h>
// https://github.com/JensGrabner/snc98_Slash-Number-Calculator/tree/master/Software/Arduino/libraries/itoa_ljust

 /*
 uint64_t my_xx_bit;
 uint64_t add;
 uint64_t mul_10 = 10;
 uint8_t  end    = 18;
 */
 int96_a my_xx_bit;
 int96_a add;
 int96_a mul_10 = 10;
 uint8_t end    = 27;

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
  delay(10);
 
} // end setup

// ------------------------------------------------

void loop() {

  delay(10);

for ( uint8_t ii_a = 0; ii_a < end ; ii_a += 1 ) {
  delay(10);
  if ( ii_a == 26 ) ii_end = 296;
  time_start = millis();

  for ( uint16_t ii_b = 0; ii_b < ii_end; ii_b += 1 ) {
     itoa_(my_xx_bit, display_string);
     my_xx_bit += add;
     // Serial.println(display_string);
  }
  
  itoa_(my_xx_bit, display_string);

  time_end = millis();
  time = time_end - time_start;
  Serial.print("Time: ");
  Serial.print(time);
  Serial.print(" ms ");
  Serial.println(display_string);
  
  add *= mul_10;
}  
 
delay(20000000);

}


