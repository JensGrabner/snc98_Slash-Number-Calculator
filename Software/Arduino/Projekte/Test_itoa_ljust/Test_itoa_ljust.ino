
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include <itoa_ljust.h>

uint64_t my64bit;
uint64_t add;
 
char  display_string[21];

uint32_t time_start;
uint32_t time_end;
uint32_t time;

// ------------------------------------------------

void setup() {
	
	my64bit = 100;
	add     =   1;

  Serial.begin(115200);
  delay(10);

} // end setup

// ------------------------------------------------

void loop() {

  delay(10);

for ( uint16_t ii = 0; ii < 17 ; ii += 1 ) {
  delay(10);
  time_start = millis();

  for ( uint16_t ii = 0; ii < 900 ; ii += 1 ) {
    itoa_(my64bit, display_string);
    my64bit += add;
    // Serial.println(display_string);
  }

  time_end = millis();
  time = time_end - time_start;
  Serial.print("Time: ");
  Serial.print(time);
  Serial.print(" ms ");
  Serial.println(display_string);

  add *= 10;
}  

delay(2000000);

}


