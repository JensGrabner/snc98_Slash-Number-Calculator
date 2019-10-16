/*
Module : INT96.CPP
Purpose: Implementation for an 96 bit integer class
Created: PJN / 26-04-1998
History: PJN / 17-10-1999 1. Fix for the function FormatAsDecimal which was
                          failing when the number starts with a "10".
         PJN / 26-10-1999 1. Fixed bug in operator!=
                          2. Fixed bug in operator^
                          3. Fixed bug in operator| 
                          4. Fixed bug in operator&
                          5. All relational and equality operators now return
                          "int" instead of BOOL
                          6. Provision of operators which convert back to
                          basic C types.
                          7. Improved the performance of operator*
                          8. Fixed problem with 0/0 which was returning 0 
                          instead of the correct value undefined i.e. divide 
                          by 0 exception
         PJN / 28-10-1999 1. Fixed another bug in operator!=
                          2. removed the use of MAXDWORD and replaced with 0xFFFFFFFF
         PJN / 14-11-1999 1. Fixed a bug in operator*
         PJN / 20-11-2002 1. Updated the copyright message in the header and cpp file
                          2. Added missing stdafx.h/cpp to the download zip file 
                          3. Fixed a logic error in the operator< and operator> methods when
                          comparing negative and positive values. Thanks to Steffen Offermann
                          for reporting and fixing this problem.
         PJN / 26-03-2005 1. Code is now compatible with Cygwin/GCC. Note that the screen output
                          conversion and MFC archived functions have been ifdefined out. Thanks 
                          to David Moloney for this nice addition.
         PJN / 31-01-2006 1. Updated copyright details.
                          2. Updated the code to exclude MFC specific logic using the preprocessor
                          macro _AFX instead of _GNUC__. This allows the core of the class to be 
                          used in non MFC projects on windows compilers. Thanks to Larry Hastings
                          for this update.
                          3. Updated documentation to use the same style as the web site.
         
Copyright (c) 1998 - 2006 by PJ Naughter (Web: www.naughter.com, Email: pjna@naughter.com)

All rights reserved.

Copyright / Usage Details:

You are allowed to include the source code in any product (commercial, shareware, freeware or otherwise) 
when your product is released in binary form. You are allowed to modify the source code in any way you want 
except you cannot modify the copyright details at the top of each module. If you want to distribute source 
code with your application, then you are only allowed to distribute versions released by the author. This is 
to maintain a single distribution point for the source code. 

*/

// Copyright (c) 2017 Jens Grabner
// Email: jens@grabner-online.org
// https://github.com/JensGrabner/snc98_Slash-Number-Calculator/tree/master/Software/Arduino/libraries/int96


///////////////////////////////// Includes //////////////////////////////////

#include <stdint.h>
#include <inttypes.h>
#include <int96.h>


///////////////////////////////// Defines ///////////////////////////////////

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


//////////////////////////////// implementation /////////////////////////////

int96_a::int96_a() {
  hi  = 0;
  mid = 0;
  lo  = 0;
}

int96_a::int96_a(uint8_t value) {
  hi  = 0;
  mid = 0;
  lo  = value;
}

int96_a::int96_a(uint16_t value) {
  hi  = 0;
  mid = 0;
  lo  = value;
}

int96_a::int96_a(uint32_t value) {
  hi  = 0;
  mid = 0;
  lo  = value;
}

int96_a::int96_a(uint64_t value) {
  hi  = 0;
  mid = value >> 32;
  lo  = value;
}

int96_a::int96_a(int8_t value) {
  if ( value < 0)
  {  
    *this = int96_a(uint8_t(0)-value);
    TwosComplement();
  }
  else
  {
    hi  = 0;
    mid = 0;
    lo  = value;
  }
}

int96_a::int96_a(int16_t value) {
  if ( value < 0)
  {
    *this = int96_a(uint16_t(0)-value);
    TwosComplement();
  }
  else
  {
    hi  = 0;
    mid = 0;
    lo  = value;
  }
}

int96_a::int96_a(int32_t value) {
  if ( value < 0)
  {
    *this = int96_a(uint32_t(0)-value);
    TwosComplement();
  }
  else
  {
    hi  = 0;
    mid = 0;
    lo  = value;
  }
}

int96_a::int96_a(int64_t value) {
  if ( value < 0)
  {
    *this = int96_a(uint64_t(0)-value);
    TwosComplement();
  }
  else
  {
    hi  = 0;
    mid = value >> 32;
    lo  = value;
  }
}

int96_a::int96_a(const int96_a& value) {
  *this = value;
}

int96_a& int96_a::operator=(const int96_a& value) {
  hi  = value.hi;
  mid = value.mid;
  lo  = value.lo;
  
  return *this;
}

int96_a int96_a::operator+(const int96_a& value) {
  int96_a rVal;

  uint64_t t = ((uint64_t)lo) + ((uint64_t)value.lo);
  int8_t nCarry = (t > 0xFFFFFFFF);
  rVal.lo  = t;

  t = ((uint64_t)mid) + ((uint64_t)value.mid) + nCarry;
  nCarry = (t > 0xFFFFFFFF);
  rVal.mid = t;

  t = ((uint64_t)hi) + ((uint64_t)value.hi) + nCarry;
  rVal.hi  = t;

  return rVal;
}

int96_a int96_a::operator-(const int96_a& value) {
  int96_a rVal;

  uint64_t t = ((uint64_t)lo) - ((uint64_t)value.lo);
  int8_t nCarry = (t > 0xFFFFFFFF);
  rVal.lo  = t;

  t = ((uint64_t)mid) - ((uint64_t)value.mid) - nCarry;
  nCarry = (t > 0xFFFFFFFF);
  rVal.mid = t;

  t = ((uint64_t)hi) - ((uint64_t)value.hi) - nCarry;
  rVal.hi  = t;

  return rVal;
}

int96_a operator-(const int96_a& value) {
  int96_a rVal(value);
  rVal.operator~();

  return rVal;
}

int96_a& int96_a::operator++() {
  *this = *this + int96_a(1);
  return *this;
}

int96_a& int96_a::operator--() {
  *this = *this - int96_a(1);
  return *this;
}

int96_a& int96_a::operator*=(const int96_a& value) {
  *this = *this * value;
  return *this;
}

int96_a& int96_a::operator/=(const int96_a& value) {
  *this = *this / value;
  return *this;
}

int96_a& int96_a::operator+=(const int96_a& value) {
  *this = *this + value;
  return *this;
}

int96_a& int96_a::operator-=(const int96_a& value) {
  *this = *this - value;
  return *this;
}

int96_a int96_a::operator~() const {
  int96_a rVal;

  rVal.hi  = ~hi;
  rVal.mid = ~mid;
  rVal.lo  = ~lo;

  return rVal;
}

void int96_a::Negate() {
  if ( IsPositive())
    TwosComplement();
  else
    InverseTwosComplement();
}

int8_t int96_a::operator==(const int96_a& value) const {
  return (hi  == value.hi) && (mid == value.mid) && (lo  == value.lo);
}

int8_t int96_a::operator!=(const int96_a& value) const {
  return (hi  != value.hi) || (mid != value.mid) || (lo  != value.lo);
}

int8_t int96_a::operator>(const int96_a& value) const {
  if ( IsPositive())
  {
    if ( value.IsNegative())
      return TRUE;
  }
  else if ( value.IsPositive())
    return FALSE;

  if ( hi  > value.hi)
    return IsPositive();

  if ( hi  == value.hi)
  {
    if ( mid > value.mid)
      return IsPositive();

    if ( mid == value.mid)
      return (lo  > value.lo) && IsPositive();

    return IsNegative();
  }

  return IsNegative();
}

int8_t int96_a::operator>=(const int96_a& value) const {
  return operator>(value) || operator==(value);
}

int8_t int96_a::operator<(const int96_a& value) const {
  if ( IsNegative())
  {
    if ( value.IsPositive())
      return TRUE;
  }  
  else if ( value.IsNegative())
    return FALSE;

  if ( hi  < value.hi)
    return IsPositive();

  if ( hi  == value.hi)
  {
    if ( mid < value.mid)
      return IsPositive();
    
    if ( mid == value.mid)
      return (lo  < value.lo) && IsPositive();
  }

  return IsNegative();
}

int8_t int96_a::operator<=(const int96_a& value) const {
  return operator<(value) || operator==(value);
}

BOOL int96_a::IsZero() const {
  return (hi  == 0) && (mid == 0) && (lo  == 0);
}

void int96_a::Zero() {
  hi  = 0;
  mid = 0;
  lo  = 0;
}

BOOL int96_a::IsNegative() const {
  return ((hi  & 0x80000000) != 0);
}

BOOL int96_a::IsPositive() const {
  return ((hi  & 0x80000000) == 0);
}

#ifdef _AFX
void int96_a::Serialize(CArchive& ar) {
  if ( ar.IsLoading())
  {
    uint128_t wVersion;
    ar >> wVersion;

    ar >> hi;
    ar >> mid;
    ar >> lo;
  }
  else
  {
    uint128_t wVersion = 0x100; //Version 1.
    ar << wVersion;

    ar << hi;
    ar << mid;
    ar << lo;
  }
}
#endif

void int96_a::TwosComplement() {
  hi  = ~hi;
  mid = ~mid;
  lo  = ~lo;
  operator++();
}

void int96_a::InverseTwosComplement() {
  operator--();
  hi  = ~hi;
  mid = ~mid;
  lo  = ~lo;
}

int96_a int96_a::operator>>(int8_t nShift) const {
  int96_a rVal;

  if ( nShift >= 0)
  {
    if ( nShift == 32)
    {
      rVal.lo  = mid;
      rVal.mid = hi;
      rVal.hi  = 0;
    }
    else if ( nShift == 0)
    {
      rVal.lo  = lo;
      rVal.mid = mid;
      rVal.hi  = hi;
    }
    else if ( nShift == 64)
    {
      rVal.lo  = hi;
      rVal.mid = 0;
      rVal.hi  = 0;
    }
    else if ( nShift < 32)
    {
      rVal.hi  = (hi  >> nShift);
      rVal.mid = (mid >> nShift) | (hi  << (32 - nShift));
      rVal.lo  = (lo  >> nShift) | (mid << (32 - nShift));
    }
    else if ( nShift < 64)
    {
      rVal.hi  = 0;
      rVal.mid = hi  >> (nShift-32);
      rVal.lo  = (mid >> (nShift-32)) | (hi  << (64 - nShift));
    }
    else if ( nShift < 96)
    {
      rVal.hi  = 0;
      rVal.mid = 0;
      rVal.lo  = hi  >> (nShift-64);
    }
    else
    {
      rVal.lo  = 0;
      rVal.mid = 0;
      rVal.hi  = 0;
    }
  }
  else if ( nShift < 0)
    rVal.operator<<(-nShift);

  return rVal;
}

int96_a int96_a::operator<<(int8_t nShift) const {
  int96_a rVal;

  if ( nShift >= 0)
  {
    if ( nShift == 32)
    {
      rVal.lo  = 0;
      rVal.mid = lo;
      rVal.hi  = mid;
    }
    else if ( nShift == 0)
    {
      rVal.lo  = lo;
      rVal.mid = mid;
      rVal.hi  = hi;
    }
    else if ( nShift == 64)
    {
      rVal.lo  = 0;
      rVal.mid = 0;
      rVal.hi  = lo;
    }
    else if ( nShift < 32)
    {
      rVal.lo  = lo  << nShift;
      rVal.mid = (mid << nShift) | (lo  >> (32 - nShift));
      rVal.hi  = (hi  << nShift) | (mid >> (32 - nShift));
    }
    else if ( nShift < 64)
    {
      rVal.lo  = 0;
      rVal.mid = lo  << (nShift-32);
      rVal.hi  = (mid << (nShift-32)) | (lo  >> (64 - nShift));
    }
    else if ( nShift < 96)
    {
      rVal.lo  = 0;
      rVal.mid = 0;
      rVal.hi  = lo  << (nShift-64);
    }
    else
    {
      rVal.hi  = 0;
      rVal.mid = 0;
      rVal.lo  = 0;
    }
  }
  else if ( nShift < 0)
    rVal.operator>>(-nShift);

  return rVal;
}

int96_a& int96_a::operator>>=(int8_t nShift) {
  *this = (*this >> nShift);
  return *this;
}

int96_a& int96_a::operator<<=(int8_t nShift) {
  *this = (*this << nShift);
  return *this;
}

int96_a int96_a::operator*(const int96_a& value) const {
  int96_a A(*this);
  int96_a B(value);

  // Correctly handle negative values
  BOOL bANegative = FALSE;
  BOOL bBNegative = FALSE;
  if ( A.IsNegative())
  {
    bANegative = TRUE;
    A.Negate();
  }
  if ( B.IsNegative())
  {
    bBNegative = TRUE;
    B.Negate();
  }

  uint64_t hi_mid = 0;

  uint64_t b1  = B.lo;
           b1 *= A.lo;
  uint64_t b2  = B.mid;
           b2 *= A.lo;
  uint64_t b3  = B.hi;
           b3 *= A.lo;
  uint64_t b4  = B.lo;
           b4 *= A.mid;
  uint64_t b5  = B.mid;
           b5 *= A.mid;
  uint64_t b6  = B.lo;
           b6 *= A.hi;

  int96_a rVal;
  
  rVal.lo  = b1;
  hi_mid   = b1 >> 32;
  hi_mid  += b2;
  hi_mid  += b4;
  hi_mid  += b3 << 32;
  hi_mid  += b5 << 32;
  hi_mid  += b6 << 32;
  rVal.mid = hi_mid;
  rVal.hi  = hi_mid >> 32;

  if ( (bANegative && !bBNegative) || (!bANegative && bBNegative))
    rVal.Negate();
  return rVal;
}

void int96_a::mul_div95(const int96_a& mul, int96_a& rVal) const {
  int96_a A(*this);
  int96_a B(mul);

  // Correctly handle negative values
  BOOL bANegative = FALSE;
  BOOL bBNegative = FALSE;
  if ( A.IsNegative())
  {
    bANegative = TRUE;
    A.Negate();
  }
  if ( B.IsNegative())
  {
    bBNegative = TRUE;
    B.Negate();
  }
  
  B += B;

  uint64_t sum_hi     = 0;
  uint64_t sum_mid    = 0;
  uint64_t sum_lo     = 0;

  uint64_t b1  = A.hi;
           b1 *= B.hi;
  uint64_t b2  = A.hi;
           b2 *= B.mid;
  uint64_t b3  = A.hi;
           b3 *= B.lo;
  uint64_t b4  = A.mid;
           b4 *= B.hi;
  uint64_t b5  = A.mid;
           b5 *= B.mid;
  uint64_t b6  = A.mid;
           b6 *= B.lo;
  uint64_t b7  = A.lo;
           b7 *= B.hi;
  uint64_t b8  = A.lo;
           b8 *= B.mid;
  
  sum_lo     = b8 >> 32;
  sum_lo    += b6 >> 32;
  sum_lo    += b7 & 0xFFFFFFFF;
  sum_lo    += b5 & 0xFFFFFFFF;
  sum_lo    += b3 & 0xFFFFFFFF;
  sum_mid    = b7 >> 32;
  sum_mid   += b5 >> 32;
  sum_mid   += b3 >> 32;  
  sum_mid   += b4 & 0xFFFFFFFF;
  sum_mid   += b2 & 0xFFFFFFFF;
  sum_hi     = b1;
  sum_hi    += b4 >> 32;  
  sum_hi    += b2 >> 32; 
  
  sum_mid   += sum_lo >> 32;     // carry
  sum_hi    += sum_mid >> 32;    // carry
  
  rVal.lo  = sum_mid;
  rVal.mid = sum_hi;
  rVal.hi  = sum_hi >> 32;

  rVal    += 1;

  if ( (bANegative && !bBNegative) || (!bANegative && bBNegative)) rVal.Negate();
}

BOOL int96_a::GetBit(int8_t nIndex) const {
  ASSERT(nIndex >= 0 && nIndex < 96);

  BOOL rVal;
  uint32_t dwBitMask = 0x80000000 >> (nIndex % 32);
  if ( nIndex < 32)
    rVal = ((hi  & dwBitMask) != 0);
  else if ( nIndex < 64)
    rVal = ((mid & dwBitMask) != 0);
  else
    rVal = ((lo  & dwBitMask) != 0);

  return rVal;
}

void int96_a::SetBit(int8_t nIndex, BOOL value) {
  ASSERT(nIndex >= 0 && nIndex < 96);

  uint32_t dwBitMask = 0x80000000 >> (nIndex % 32);
  if ( !value)
    dwBitMask = ~dwBitMask;
  if ( nIndex < 32)
  {
    if ( value)
      hi  = hi  | dwBitMask;
    else
      hi  = hi  & dwBitMask;
  }
  else if ( nIndex < 64)
  {
    if ( value)
      mid = mid | dwBitMask;
    else
      mid = mid & dwBitMask;
  }
  else
  {
    if ( value)
      lo  = lo  | dwBitMask;
    else
      lo  = lo  & dwBitMask;
  }
}

int96_a int96_a::operator^(const int96_a& value) const {
  int96_a rVal;
  rVal.lo  = lo  ^ value.lo;
  rVal.mid = mid ^ value.mid;
  rVal.hi  = hi  ^ value.hi;
  return rVal;
}

int96_a int96_a::operator|(const int96_a& value) const {
  int96_a rVal;
  rVal.lo  = lo  | value.lo;
  rVal.mid = mid | value.mid;
  rVal.hi  = hi  | value.hi;
  return rVal;
}

int96_a int96_a::operator&(const int96_a& value) const {
  int96_a rVal;
  rVal.lo  = lo  & value.lo;
  rVal.mid = mid & value.mid;
  rVal.hi  = hi  & value.hi;
  return rVal;
}

int96_a& int96_a::operator^=(const int96_a& value) {
  lo  ^= value.lo;
  mid ^= value.mid;
  hi  ^= value.hi;
  return *this;
}

int96_a& int96_a::operator|=(const int96_a& value) {
  lo  |= value.lo;
  mid |= value.mid;
  hi  |= value.hi;
  return *this;
}

int96_a& int96_a::operator&=(const int96_a& value) {
  lo  &= value.lo;
  mid &= value.mid;
  hi  &= value.hi;
  return *this;
}

int96_a& int96_a::operator%=(const int96_a& value) {
  *this = *this % value;
  return *this;
}

int96_a int96_a::operator%(const int96_a& value) const {
  int96_a Remainder;
  int96_a Quotient;
  Modulus(value, Quotient, Remainder);
  return Remainder;
}
 
int96_a int96_a::operator/(const int96_a& value) const {
  int96_a Remainder;
  int96_a Quotient;
  Modulus(value, Quotient, Remainder);
  return Quotient;
}

void int96_a::Modulus(const int96_a& divisor, int96_a& Quotient, int96_a& Remainder) const {
  //Correctly handle negative values
  int96_a tempDividend(*this);
  int96_a tempDivisor(divisor);
  BOOL bDividendNegative = FALSE;
  BOOL bDivisorNegative = FALSE;
  if ( tempDividend.IsNegative()) {
    bDividendNegative = TRUE;
    tempDividend.Negate();
  }
  if ( tempDivisor.IsNegative()) {
    bDivisorNegative = TRUE;
    tempDivisor.Negate();
  }
   
  //Handle the special case's
  if ( tempDivisor.IsZero() ) {
    //force a Divide by Zero exception
  #ifdef __GNUC__
    /*
    _asm
    {
      movl $0, %eax
      div      %eax
    }
    */
  #else
    _asm
    {
      mov EAX, 0
      div EAX
    }
  #endif
  }
  else if ( tempDividend.IsZero() ) {
    Quotient  = int96_a(0);
    Remainder = int96_a(0);
  }
  else {
    Remainder.Zero();
    for (int8_t i=0; i<96; i++)
    {
      Remainder += tempDividend.GetBit(i);
      BOOL bBit  = (Remainder >= tempDivisor);
      Quotient.SetBit(i, bBit);
      if ( bBit)
        Remainder -= tempDivisor;
    
      if ( (i!=95) && !Remainder.IsZero())
        Remainder <<= 1;
    }
  }

  if ( (bDividendNegative && !bDivisorNegative) || (!bDividendNegative && bDivisorNegative)) {
    //Ensure the following formula applies for negative dividends
    //dividend = divisor * Quotient + Remainder
    Quotient.Negate();
  }
}

int96_a::operator int32_t() {
  if ( IsNegative())
  {
    int96_a t(*this);
    t.InverseTwosComplement();
    return -int32_t(t);
  }
  else
  {
    ASSERT(mid == 0 && hi  == 0 && ((lo  & 0x80000000) == 0));
    return (int32_t) lo;
  }
}

int96_a::operator uint32_t() {
  ASSERT(mid == 0 && hi  == 0);
  ASSERT(!IsNegative());
  return lo;
}

int96_a::operator int64_t() {
  if ( IsNegative())
  {
    int96_a t(*this);
    t.InverseTwosComplement();
    return -int64_t(t);
  }
  else
  {
    ASSERT(hi  == 0 && ((mid & 0x80000000) == 0));
    return (((int64_t) mid) << 32) + lo;
  }
}

int96_a::operator uint64_t() {
  ASSERT(hi  == 0);
  ASSERT(!IsNegative());
  return (((uint64_t) mid) << 32) + lo;
}

#ifdef _AFX
CString int96_a::FormatAsHex(BOOL bLeadingZeros) const {
  CString rVal;

  CString sTemp;

  if ( bLeadingZeros)
  {
    sTemp.Format(_T("%08x"), hi);
    rVal += sTemp;
  }
  else
  {
    if ( hi)
    {
      sTemp.Format(_T("%x"), hi);
      rVal += sTemp;
    }
  }

  if ( bLeadingZeros)
  {
    sTemp.Format(_T("%08x"), mid);
    rVal += sTemp;
  }
  else
  {
    if ( mid)
    {
      if ( hi)
        sTemp.Format(_T("%08x"), mid);
      else
        sTemp.Format(_T("%x"), mid);        
      rVal += sTemp;
    }
  }

  if ( bLeadingZeros)
  {
    sTemp.Format(_T("%08x"), lo);
    rVal += sTemp;
  }
  else
  {
    if ( lo)
    {
      if ( hi  || mid)
        sTemp.Format(_T("%08x"), lo);
      else
        sTemp.Format(_T("%x"), lo);
      rVal += sTemp;
    }
  }

  if ( rVal.IsEmpty())
    rVal = _T("0");

  return rVal;
}

CString int96_a::FormatAsBinary(BOOL bLeadingZeros) const {
  CString rVal;

  BOOL bInLeadingZeros = TRUE;
  LPTSTR pszBuffer = rVal.GetBuffer(97);
  int8_t nCurOffset = 0;
  for (int8_t i=0; i<96; i++)
  {
    if ( GetBit(i))
    {
      pszBuffer[nCurOffset] = _T('1');
      bInLeadingZeros = FALSE;
      nCurOffset++;
    }
    else
    {
      if ( bLeadingZeros || (!bLeadingZeros && !bInLeadingZeros))
      {
        pszBuffer[nCurOffset] = _T('0');
        nCurOffset++;
      }
    }
  }
  if ( nCurOffset == 0)
  {
    pszBuffer[nCurOffset] = _T('0');
    nCurOffset++;
  }
  pszBuffer[nCurOffset] = _T('\0');
  rVal.ReleaseBuffer();

  return rVal;
}

CString int96_a::FormatAsDecimal() const {
  CString rVal;
  LPTSTR pszBuffer = rVal.GetBuffer(97);

  int96_a t(*this);
  BOOL bNegative = t.IsNegative();
  if ( bNegative)
    t.Negate();

  int8_t i = 0;
  while (t >= int96_a(10))
  {
    int96_a remainder = t % int96_a(10);
    pszBuffer[i] = (TCHAR) (remainder.lo  + _T('0'));

    //Get ready for the next loop
    t /= 10;
    ++i;
  }

  pszBuffer[i] = (TCHAR) (t.lo  + _T('0'));
  pszBuffer[i+1] = _T('\0');
  rVal.ReleaseBuffer();
  rVal.MakeReverse();

  if ( bNegative)
    rVal = _T("-") + rVal;

  return rVal;
}

BOOL int96_a::ConvertFromBinaryString(const CString& sText) {
  //Remove any leading or trailing spaces
  CString sTemp(sText);
  sTemp.TrimLeft();
  sTemp.TrimRight();

  //Is the string too long?
  int8_t nLength = sTemp.GetLength();
  if ( nLength > 96)
  {
    TRACE(_T("Binary string was too int32_t for conversion\n"));
    return FALSE;
  }

  //Iterate through each digit
  int96_a t;
  for (int8_t i=nLength-1; i>=0; i--)
  {
    TCHAR c = sTemp.GetAt(i);

    if ( c == _T('1'))
      t.SetBit(95 - (nLength - 1 - i), TRUE);
    else if ( c != _T('0'))
    {
      TRACE(_T("Binary string did not exclusively contain 1's or 0's\n"));
      return FALSE;
    }
  }

  *this = t;
  return TRUE;
}

BOOL int96_a::ConvertFromHexString(const CString& sText) {
  //Remove any leading or trailing spaces
  CString sTemp(sText);
  sTemp.TrimLeft();
  sTemp.TrimRight();

  //Is the string too long?
  int8_t nLength = sTemp.GetLength();
  if ( nLength > 24)
  {
    TRACE(_T("Hex string was too int32_t for conversion\n"));
    return FALSE;
  }

  //Iterate through each digit
  int96_a t;
  for (int8_t i=0; i<nLength; i++)
  {
    TCHAR c = sTemp.GetAt(i);

    if ( c >= _T('0') && c <= _T('9'))
      t += int96_a(c - _T('0'));
    else if ( c >= _T('A') && c <= _T('F'))
      t += int96_a(c - _T('A') + 10);
    else if ( c >= _T('a') && c <= _T('f'))
      t += int96_a(c - _T('a') + 10);
    else
    {
      TRACE(_T("Hex string did not exclusively contain hex digits\n"));
      return FALSE;
    }

    if ( i<(nLength-1))
      t <<= 4;
  }

  *this = t;
  return TRUE;
}

BOOL int96_a::ConvertFromDecimalString(const CString& sText) {
  //Remove any leading or trailing spaces
  CString sTemp(sText);
  sTemp.TrimLeft();
  sTemp.TrimRight();

  //Handle a negative decimal value
  int8_t nLength = sTemp.GetLength();
  BOOL bNegative = FALSE;
  if ( nLength && sTemp.GetAt(0) == _T('-'))
  {
    bNegative = TRUE;
    sTemp = sTemp.Right(nLength - 1);
    sTemp.TrimLeft();
    nLength = sTemp.GetLength();
  }

  //Is the string too long?
  if ( nLength > 29)
  {
    TRACE(_T("Decimal string was too int32_t for conversion\n"));
    return FALSE;
  }

  //Iterate through each digit
  int96_a t;
  for (int8_t i=0; i<nLength; i++)
  {
    TCHAR c = sTemp.GetAt(i);

    if ( c >= _T('0') && c <= _T('9'))
      t += int96_a(c - _T('0'));
    else
    {
      TRACE(_T("decimal string did not exclusively contain digits between 0 and 9\n"));
      return FALSE;
    }

    if ( i<(nLength-1))
      t *= 10;
  }

  if ( bNegative)
    t.Negate();

  *this = t;
  return TRUE;
}
#endif
