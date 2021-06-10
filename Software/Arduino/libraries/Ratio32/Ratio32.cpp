// Copyright (c) 2021 Jens Grabner
// Email: jens@grabner-online.org
// https://github.com/JensGrabner/snc98_Slash-Number-Calculator/tree/master/Software/Arduino/libraries/Ratio32


///////////////////////////////// Includes //////////////////////////////////

#include <Ratio32.h>

///////////////////////////////// Definition //////////////////////////////////


//////////////////////////////// Implementation /////////////////////////////

Ratio_32 min_x(Ratio_32 a) {
	if ( a.denom < 0 ) {
		a.denom *= -1;
	}
	else {
		a.num   *= -1;
	}

	return a;
}

Ratio_32 div_x(Ratio_32 a) {
	a.expo      *= -1;

	int32_t temp = a.denom;
	a.denom      = a.num;
	a.num        = temp;

	if ( a.denom < 0 ) {
		a.num   *= -1;
		a.denom *= -1;
	}

	return a;
}

Ratio_32 abs_x(Ratio_32 a) {
	if ( a.num < 0 ) {
		a.num *= -1;
	}
	if ( a.denom < 0 ) {
		a.denom *= -1;
	}
	return a;
}

Ratio_32 clone(Ratio_32 a) {
	// Creates a copy of the actual Fraction object
	return a;
}

int8_t compare(Ratio_32 a, Ratio_32 b) {
	if ( a.num < 0 ) {
		a.num *= -1;
	}
	if ( b.num < 0 ) {
		b.num *= -1;
	}
	uint64_t test_a = a.num;
	uint64_t test_b = b.num;

	test_a *= b.denom;
	test_b *= a.denom;

	if ( a.expo > b.expo ) {
		return  1;
	}

	if ( a.expo < b.expo ) {
		return -1;
	}

	if ( test_a > test_b ) {
		return 1;
	}

	if ( test_a < test_b ) {
		return -1;
	}

	return 0;
}

int8_t compare(R_uint32 a, R_uint32 b) {
	uint64_t test_a = a.num;
	uint64_t test_b = b.num;

	test_a *= b.denom;
	test_b *= a.denom;

	if ( test_a > test_b ) {
		return 1;
	}

	if ( test_a < test_b ) {
		return -1;
	}

	return 0;
}

int8_t compare(R_uint64 a, R_uint64 b) {
		int8_t comp    = 0;
	uint64_t test_a  = 0;   // Factor
	uint64_t test_b  = 0;   // Factor

	while ( comp == 0 ) {
		test_a  = a.num;      // Factor
		test_a /= a.denom;
		test_b  = b.num;      // Factor
		test_b /= b.denom;

		if ( test_a > test_b ) {
			return 1;
		}

		if ( test_a < test_b ) {
			return -1;
		}

		test_a *= a.denom;
		a.num  -= test_a;

		if ( a.num == 0 ) {
			if ( a.denom > b.denom ) {
				return 1;
			}

			if ( a.denom < b.denom ) {
				return -1;
			}

			return comp;
		}

		test_b *= b.denom;
		b.num  -= test_b;

		test_a  = a.denom;   // Factor
		test_a /= a.num;
		test_b  = b.denom;   // Factor
		test_b /= b.num;

		if ( test_a > test_b ) {
			return -1;
		}

		if ( test_a < test_b ) {
			comp = 1;
			return comp;
		}

		test_a  *= a.num;
		a.denom -= test_a;

		if ( a.denom == 0 ) {
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

R_uint32 compare_extra(R_uint32 a, R_uint32 b, uint64_t num_u64, uint64_t denom_u64) {
	R_uint64 calc;
	R_uint64 input;
	int8_t   comp    = 0;
	int8_t   test_32 = 0;
	uint64_t test    = 0;

	input.num   = num_u64;
	input.denom = denom_u64;

	test_32 = compare(b, a);

	calc.num  = a.num;
	calc.num *= b.denom;
	test      = b.num;
	test     *= a.denom;

	calc.num += test;

	calc.denom  = a.denom;
	calc.denom *= b.denom;
	calc.denom *= 2;

	comp    = compare(calc, input);

	if ( comp > 0 ) {
		if ( test_32 > 0 ) {
			return a;
		}
		else {
			return b;
		}
	}

	if ( comp < 0 ) {
		if ( test_32 > 0 ) {
			return b;
		}
		else {
			return a;
		}
	}

	if ( a.num > b.num ) {
		if ( test_32 > 0 ) {
			return a;
		}
		else {
			return b;
		}
	}
	else {
		if ( test_32 > 0 ) {
			return b;
		}
		else {
			return a;
		}
	}
}

Ratio_32 Reduce_Number(int8_t expo, uint64_t num_u64, uint64_t denom_u64) {
 /*
  * https://dspace.library.uu.nl/bitstream/handle/1874/325967/Ramanujan.pdf
  * Page 4  a(1) = 1; a(2) = -1; a(3) = 1; a(4) = -1; a(5) = 1; ...
  * "odd-odd continued fraction" - Rieger 
  * 
  * https://faculty.math.illinois.edu/~emerrim2/smp.pdf
  * Page 4 and 5 "even continued fraction"
  *
  * https://www.asc.ohio-state.edu/merriman.72/slides/MerrimanExpandingDynamics.pdf
  * http://eventos.cmm.uchile.cl/sdynamics20207/wp-content/uploads/sites/110/2020/12/MerrimanExpandingDynamics.pdf
  * https://www.ideals.illinois.edu/bitstream/handle/2142/105632/MERRIMAN-DISSERTATION-2019.pdf
  *
  * https://www.math.leidenuniv.nl/scripties/masarotto.pdf
  */
 /*
	*  https://hg.python.org/cpython/file/3.5/Lib/fractions.py#l252
	*  https://ffmpeg.org/doxygen/2.8/rational_8c_source.html#l00035
	*  https://github.com/FFmpeg/FFmpeg/blob/master/libavutil/rational.c
	*  http://link.springer.com/article/10.1007%2Fs00607-008-0013-8
	*  https://math.boku.ac.at/udt/vol05/no2/3zhabitsk10-2.pdf  Page_5
	*/
 /*
	*  Thill M. - A more precise rounding algorithm for natural numbers.
	*  .. Computing Jul 2008, Volume 82, Issue 2, pp 189–198
	*  http://link.springer.com/article/10.1007/s00607-008-0006-7
	*  =====
	*  Thill M. - A more precise rounding algorithm for natural numbers.
	*  .. Computing Sep 2008, Volume 82, Issue 4, pp 261–262
	*  http://link.springer.com/article/10.1007/s00607-008-0013-8
	*  http://link.springer.com/content/pdf/10.1007/s00607-008-0013-8.pdf
	*/
	Ratio_32 Reduce;
	uint32_t test_max   = int32_max;

	int32_t  num_i32    = 0;
	int32_t  denom_i32  = 0;

	int8_t   test_1_2_4 = 0;
	uint64_t temp_u64   = 0;
	uint64_t temp_m_u64 = 0;
	bool     calc       = true;
	uint8_t  count      = 2;
	uint8_t  count_x    = 200;
	uint64_t g_x        = 0;
	uint64_t num_x[4]   = {1, 0, 1, 0};
	uint64_t denom_x[4] = {0, 1, 0, 1};
	
	Reduce.expo  = expo;
 /*
	if ( Debug_Level == 62 ) {
		itoa_(num_u64, display_string_itoa_);
		Serial.print(display_string_itoa_);
		Serial.print(" / ");
		itoa_(denom_u64, display_string_itoa_);
		Serial.println(display_string_itoa_);
		Serial.println(" === ");
	}
 */
	if ( denom_u64 == num_u64 ) {
		Reduce.num   = int32_max;
		Reduce.denom = int32_max;
	
		return Reduce;	
	}

	if ( denom_u64 > num_u64 ) {
		test_1_2_4 = -1;                            // 0.5  .. 1.0
		if ( (denom_u64 - num_u64) > num_u64 ) {    // 0.25 .. 0.5
			test_1_2_4  = -2;
			num_u64    *= 2;
			test_max   /= 2;
		}
	}
	
	if ( num_u64 > denom_u64 ) {
		test_1_2_4 = 1;                             // 1.0 .. 2.0
		if ( (num_u64 - denom_u64) > denom_u64 ) {  // 2.0 .. 4.0 
			test_1_2_4  = 2;
			denom_u64  *= 2;
			test_max   /= 2;
		}
		temp_u64  = num_u64;
		num_u64   = denom_u64;
		denom_u64 = temp_u64;
	}
 /*
	if ( Debug_Level == 62 ) {
		itoa_(num_u64, display_string_itoa_);
		Serial.print(display_string_itoa_);
		Serial.print(" / ");
		itoa_(denom_u64, display_string_itoa_);
		Serial.println(display_string_itoa_);
	}
 */
	while ( calc == true ) {
		g_x    = denom_u64;
		g_x   /= num_u64;

		temp_u64  = num_u64;

		if ( (count % 2) == 0 ) {
			g_x     += 1;
			num_u64 *= g_x;
			num_u64 -= denom_u64;
			denom_x[ (count % 4) ]  = denom_x[ ((count - 1) % 4) ];
			denom_x[ (count % 4) ] *= g_x;
			denom_x[ (count % 4) ] += denom_x[ ((count - 2) % 4) ];
		}
		else {
			temp_m_u64  = num_u64;
			temp_m_u64 *= g_x;
			num_u64     = denom_u64;
			num_u64    -= temp_m_u64;
		}

		num_x[ (count % 4) ]    = num_x[ ((count - 1) % 4) ];
		num_x[ (count % 4) ]   *= g_x;
		denom_x[ (count % 4) ]  = denom_x[ ((count - 1) % 4) ];
		denom_x[ (count % 4) ] *= g_x;
		if ( (count % 2) == 0 ) {
			num_x[ (count % 4) ]   += num_x[ ((count - 2) % 4) ];
			denom_x[ (count % 4) ] += denom_x[ ((count - 2) % 4) ];
		}
		else {
			num_x[ (count % 4) ]   -= num_x[ ((count - 2) % 4) ];
			denom_x[ (count % 4) ] -= denom_x[ ((count - 2) % 4) ];
		}

		denom_u64 = temp_u64;
   /*
		if ( Debug_Level == 62 ) {
			itoa_(num_x[ (count % 4) ], display_string_itoa_);
			Serial.print(display_string_itoa_);
			Serial.print(" / ");
			itoa_(denom_x[ (count % 4) ], display_string_itoa_);
			Serial.println(display_string_itoa_);
			Serial.print("g = ");
			itoa_(g_x, display_string_itoa_);
			Serial.println(display_string_itoa_);
			itoa_(num_u64, display_string_itoa_);
			Serial.print(display_string_itoa_);
			Serial.print(" / ");
			itoa_(denom_u64, display_string_itoa_);
			Serial.println(display_string_itoa_);
		}
   */
		if ( g_x > test_max ) {
			calc   = false;
		}

		if ( count > count_x ) {
			calc   = false;
		}

		if ( denom_x[ (count % 4) ] > test_max ) {
			count_x   = count;
		}

		if ( num_u64 == 0 ) {
			calc   = false;
		}
		count += 1;
	}
	count -= 1;
	
	// Serial.println(count);

	for ( uint8_t ind_a = 0; ind_a < 4; ind_a += 1 ) {
		if ( denom_x[ (count % 4) ] < test_max ) {
			num_i32    = num_x[ (count % 4) ];
			denom_i32  = denom_x[ (count % 4) ];
			ind_a      = 4;
		}
		count -= 1;
	}

	test_max  /= denom_i32;
		
	num_i32   *= test_max;
	denom_i32 *= test_max;

	if ( test_1_2_4 > 0 ) {
		Reduce.num   = denom_i32;
		Reduce.denom = num_i32;
		if ( test_1_2_4 > 1 ) {
			Reduce.num  *= 2;
		}
	}
	else {
		Reduce.num   = num_i32;
		Reduce.denom = denom_i32;			
		
		if ( test_1_2_4 < -1 ) {
			Reduce.denom  *= 2;
		}
	}
 /*
	if ( Debug_Level == 62 ) {
		Serial.print(Reduce.num);
		Serial.print(" / ");
		Serial.println(Reduce.denom);
		Serial.println("=====");
	}
 */	
	return Reduce;	
}

