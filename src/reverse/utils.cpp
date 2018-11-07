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

#include <autodifk/reverse/utils.h>

#include <glog/logging.h>
#include <iostream>
#include <memory>
#include <vector>

namespace autodifk {
namespace reverse {

double Sum::ForwardPropagateValue() {
  value = std::accumulate(subexpressions_.begin(), subexpressions_.end(), 0.0,
                          [](const ScalarExpression& expression, double s) {
                            return s + expression.value;
                          });
  return value;
}

void Sum::BackwardPropagateDerivative() {
  for (auto& sub : subexpressions_) sub.derivative = derivative;
}

double Product::ForwardPropagateValue() {
  value = std::accumulate(subexpressions_.begin(), subexpressions_.end(), 0.0,
                          [](const ScalarExpression& expression, double s) {
                            return s * expression.value;
                          });
  return value;
}

void Product::BackwardPropagateDerivative() {
  // NOTE: assumes that we've got an up-to-date value (i.e. we just did a
  // full forward pass).
  for (auto& sub : subexpressions_)
    sub.derivative = derivative * value / sub.value;
}

double Sin::ForwardPropagateValue() {}
void Sin::BackwardPropagateDerivative() {}

double Cos::ForwardPropagateValue() {}
void Cos::BackwardPropagateDerivative() {}

double Tan::ForwardPropagateValue() {}
void Tan::BackwardPropagateDerivative() {}

double Exp::ForwardPropagateValue() {}
void Exp::BackwardPropagateDerivative() {}

double Log::ForwardPropagateValue() {}
void Log::BackwardPropagateDerivative() {}

double Abs::ForwardPropagateValue() {}
void Abs::BackwardPropagateDerivative() {}

double ReLU::ForwardPropagateValue() {}
void ReLU::BackwardPropagateDerivative() {}

}  // namespace reverse
}  // namespace autodifk
