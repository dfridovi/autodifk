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
// Commonly-used expressions that derive from ScalarExpression.
//
////////////////////////////////////////////////////////////////////////////////

#include <reverse/utils.h>

#include <glog/logging.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <numeric>
#include <vector>

namespace autodifk {
namespace reverse {

// Sum.
double Sum::ForwardPropagateValue() {
  value =
      std::accumulate(subexpressions_.begin(), subexpressions_.end(), 0.0,
                      [](double s, const ScalarExpression::Ptr& expression) {
                        return s + expression->value;
                      });
  return value;
}

void Sum::BackwardPropagateDerivative() {
  for (auto& sub : subexpressions_) sub->derivative += derivative;
}

// Product.
double Product::ForwardPropagateValue() {
  value =
      std::accumulate(subexpressions_.begin(), subexpressions_.end(), 1.0,
                      [](double s, const ScalarExpression::Ptr& expression) {
                        return s * expression->value;
                      });
  return value;
}

void Product::BackwardPropagateDerivative() {
  for (auto& sub : subexpressions_)
    sub->derivative += derivative * value / sub->value;
}

// Sine.
double Sin::ForwardPropagateValue() {
  value = std::sin(subexpressions_[0]->value);
  return value;
}

void Sin::BackwardPropagateDerivative() {
  subexpressions_[0]->derivative +=
      derivative * std::cos(subexpressions_[0]->value);
}

// Cosine.
double Cos::ForwardPropagateValue() {
  value = std::cos(subexpressions_[0]->value);
  return value;
}

void Cos::BackwardPropagateDerivative() {
  subexpressions_[0]->derivative +=
      -derivative * std::sin(subexpressions_[0]->value);
}

// Tangent.
double Tan::ForwardPropagateValue() {
  value = std::tan(subexpressions_[0]->value);
  return value;
}

void Tan::BackwardPropagateDerivative() {
  const double cosine = std::cos(subexpressions_[0]->value);
  subexpressions_[0]->derivative += derivative / (cosine * cosine);
}

// Exponential.
double Exp::ForwardPropagateValue() {
  value = std::exp(subexpressions_[0]->value);
  return value;
}

void Exp::BackwardPropagateDerivative() {
  subexpressions_[0]->derivative += derivative * value;
}

// Natural logarithm.
double Log::ForwardPropagateValue() {
  value = std::log(subexpressions_[0]->value);
  return value;
}

void Log::BackwardPropagateDerivative() {
  subexpressions_[0]->derivative += derivative / subexpressions_[0]->value;
}

// Absolute value.
double Abs::ForwardPropagateValue() {
  value = std::abs(subexpressions_[0]->value);
  return value;
}

void Abs::BackwardPropagateDerivative() {
  subexpressions_[0]->derivative +=
      derivative * ((subexpressions_[0]->value >= 0.0) ? 1.0 : -1.0);
}

// Rectified linear unit.
double ReLU::ForwardPropagateValue() {
  value = (subexpressions_[0]->value >= 0.0) ? subexpressions_[0]->value : 0.0;
  return value;
}

void ReLU::BackwardPropagateDerivative() {
  subexpressions_[0]->derivative +=
      derivative * ((subexpressions_[0]->value >= 0.0) ? 1.0 : 0.0);
}

}  // namespace reverse
}  // namespace autodifk
