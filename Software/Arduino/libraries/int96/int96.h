/*
Module : INT96.H
Purpose: Interface for a 96 bit integer class
Created: PJN / 24-04-1998

Copyright (c) 1998 - 2006 by PJ Naughter.  (Web: www.naughter.com, Email: pjna@naughter.com)

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

////////// Defines ///////////////////////

#ifndef __INT96_H__
#define __INT96_H__


#ifdef __GNUC__
  #include "assert.h"
  #include "inttypes.h"
  #define ASSERT             assert
  #define BOOL               bool
  #define TRUE               1
  #define FALSE              0
#else
  typedef __int64           int64_t;
  typedef unsigned __int64 uint64_t;
#endif



////////// Classes ///////////////////////

class  int96_a {

public:
//constructors / destructors
//Actual member data variables
  uint32_t hi;
  uint32_t mid;
  uint32_t lo;

  int96_a();
  int96_a(uint8_t value);
  int96_a(uint16_t value);
  int96_a(uint32_t value);
  int96_a(uint64_t value);
  int96_a(int8_t value);
  int96_a(int16_t value);
  int96_a(int32_t value);
  int96_a(int64_t value);
  int96_a(const int96_a& value);

//assignment operator
  int96_a& operator=(const  int96_a& value);

//arithmetic operators
  int96_a operator+(const  int96_a& value);
  int96_a operator-(const  int96_a& value);
  friend  int96_a operator-(const  int96_a& value);
  int96_a& operator++();
  int96_a& operator--();
  int96_a  operator*(const  int96_a& value) const;
  int96_a  operator/(const  int96_a& value) const;
 // int96_a  operator%(const  int96_a& value) const;
  int96_a& operator*=(const  int96_a& value);
  int96_a& operator/=(const  int96_a& value);
  int96_a& operator+=(const  int96_a& value);
  int96_a& operator-=(const  int96_a& value);
 // int96_a& operator%=(const  int96_a& value);
  int96_a  operator~() const;

//equality operators
  int8_t operator==(const  int96_a& value) const;
  int8_t operator!=(const  int96_a& value) const;
  int8_t operator>(const  int96_a& value) const;
  int8_t operator>=(const  int96_a& value) const;
  int8_t operator<(const  int96_a& value) const;
  int8_t operator<=(const  int96_a& value) const;

//Misc operators

  int96_a operator>>(uint8_t nShift) const;
  int96_a operator<<(uint8_t nShift) const;
  int96_a& operator>>=(uint8_t nShift);
  int96_a& operator<<=(uint8_t nShift);

  int96_a  operator^(const  int96_a& value) const;
  int96_a  operator|(const  int96_a& value) const;
  int96_a  operator&(const  int96_a& value) const;
  int96_a& operator^=(const  int96_a& value);
  int96_a& operator|=(const  int96_a& value);
  int96_a& operator&=(const  int96_a& value);

//Operators to convert back to basic types
  operator  int32_t();
  operator uint32_t();
  operator  int64_t();
  operator uint64_t();

//String conversion functions
#ifdef _AFX //Note string conversion functions are currently only supported when using MFC
  CString FormatAsHex(BOOL bLeadingZeros = TRUE) const;
  CString FormatAsBinary(BOOL bLeadingZeros = TRUE) const;
  CString FormatAsDecimal() const;
  BOOL ConvertFromBinaryString(const CString& sText);
  BOOL ConvertFromHexString(const CString& sText);
  BOOL ConvertFromDecimalString(const CString& sText);
#endif

//Misc. Functions
  BOOL GetBit(int8_t nIndex) const;
  void SetBit(int8_t nIndex, BOOL value);
  BOOL IsZero() const;
  void Zero();
  void Negate();
  void cbrt(int96_a& test);
  BOOL IsNegative() const;
  BOOL IsPositive() const;
  void Modulus(const int96_a& divisor, int96_a& Quotient) const;
  void mul_div95(const int96_a& mul, int96_a& rVal) const;

//Serialization
#ifdef _AFX //Nore serialization is MFC specific
  virtual void Serialize(CArchive& ar);
#endif

protected:
  void TwosComplement();
  void InverseTwosComplement();

};

#endif //__INT96_H__
