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
// Common functions implemented on the DualScalar type.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef AUTODIFK_FORWARD_UTILS_H
#define AUTODIFK_FORWARD_UTILS_H

#include "../forward/dual_scalar.h"
#include "../utils/sign.h"

#include <glog/logging.h>
#include <math.h>

namespace autodifk {
namespace forward {

inline DualScalar sin(const DualScalar& x) {
  return DualScalar(std::sin(x.value), x.derivative * std::cos(x.value));
}

inline DualScalar cos(const DualScalar& x) {
  return DualScalar(std::cos(x.value), -x.derivative * std::sin(x.value));
}

inline DualScalar tan(const DualScalar& x) {
  const double secant_x = 1.0 / std::cos(x.value);
  return DualScalar(std::tan(x.value), x.derivative * secant_x * secant_x);
}

inline DualScalar exp(const DualScalar& x) {
  const double exp_x = std::exp(x.value);
  return DualScalar(exp_x, x.derivative * exp_x);
}

inline DualScalar log(const DualScalar& x) {
  CHECK_GT(x.value, 0.0);
  return DualScalar(std::log(x.value), x.derivative / x.value);
}

inline DualScalar pow(const DualScalar& x, int k) {
  if (k == 0) return DualScalar(1.0, 0.0);
  if (k == 1) return x;

  const double pow_x_km1 = std::pow(x.value, k - 1);
  return DualScalar(x.value * pow_x_km1,
                    x.derivative * static_cast<double>(k) * pow_x_km1);
}

inline DualScalar abs(const DualScalar& x) {
  return DualScalar(std::abs(x), x.derivative * sign(x.value));
}

}  // namespace forward
}  // namespace autodifk

#endif
