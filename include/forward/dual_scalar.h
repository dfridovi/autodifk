/*
 * Copyright (c) 2018, David Fridovich-Keil
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */
////////////////////////////////////////////////////////////////////////////////
//
// Dual scalar implementation. Each DualScalar stores a value and the derivative
// of a function at that value. The precise function will be constructed
// automatically on-the-fly by operator overloading. That is, any function of
// DualScalars will automatically update both the value and derivative by
// forward-propagation.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef AUTODIFK_FORWARD_DUAL_SCALAR_H
#define AUTODIFK_FORWARD_DUAL_SCALAR_H

#include <glog/logging.h>
#include <iostream>

namespace autodifk {
namespace forward {

struct DualScalar {
  double value;
  double derivative;

  // Provide a default constructor and a non-explicit constructor from double,
  // in addition to a full constructor.
  DualScalar() : value(0.0), derivative(0.0) {}
  DualScalar(double v) : value(v), derivative(0.0) {}
  DualScalar(double v, double d) : value(v), derivative(d) {}

  // Unary arithmetic operators.
  DualScalar& operator+=(const DualScalar& rhs) {
    value += rhs.value;
    derivative += rhs.derivative;
    return *this;
  }

  DualScalar& operator-=(const DualScalar& rhs) {
    value -= rhs.value;
    derivative -= rhs.derivative;
    return *this;
  }

  DualScalar& operator*=(const DualScalar& rhs) {
    derivative = derivative * rhs.value + value * rhs.derivative;
    value *= rhs.value;
    return *this;
  }

  DualScalar& operator/=(const DualScalar& rhs) {
    CHECK_NE(rhs.value, 0.0);
    derivative = (derivative * rhs.value - value * rhs.derivative) /
                 (rhs.value * rhs.value);
    value /= rhs.value;
    return *this;
  }

  // Binary arithmetic operators.
  friend DualScalar operator+(DualScalar lhs, const DualScalar& rhs) {
    lhs += rhs;
    return lhs;
  }

  friend DualScalar operator-(DualScalar lhs, const DualScalar& rhs) {
    lhs -= rhs;
    return lhs;
  }

  friend DualScalar operator*(DualScalar lhs, const DualScalar& rhs) {
    lhs *= rhs;
    return lhs;
  }

  friend DualScalar operator/(DualScalar lhs, const DualScalar& rhs) {
    lhs /= rhs;
    return lhs;
  }

  // Relational operators are defined only on values and not derivatives
  // to provide the same interface as a normal scalar.
  friend bool operator==(const DualScalar& lhs, const DualScalar& rhs) {
    return lhs.value == rhs.value;
  }

  friend bool operator!=(const DualScalar& lhs, const DualScalar& rhs) {
    return !(lhs == rhs);
  }

  friend bool operator<(const DualScalar& lhs, const DualScalar& rhs) {
    return lhs.value < rhs.value;
  }

  friend bool operator<=(const DualScalar& lhs, const DualScalar& rhs) {
    return lhs.value <= rhs.value;
  }

  friend bool operator>(const DualScalar& lhs, const DualScalar& rhs) {
    return !(lhs <= rhs);
  }

  friend bool operator>=(const DualScalar& lhs, const DualScalar& rhs) {
    return !(lhs < rhs);
  }

  // Output stream operator.
  friend std::ostream& operator<<(std::ostream& out, const DualScalar& rhs) {
    return out << "( " << rhs.value << ", " << rhs.derivative << " )";
  }
};  //\struct DualScalar

}  // namespace forward
}  // namespace autodifk

#endif
