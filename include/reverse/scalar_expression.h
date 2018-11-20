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
// Atomic representation of an expression. These may be connected to one another
// to form a graph on which derivatives may be propagated backward via the
// chain rule.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef AUTODIFK_REVERSE_SCALAR_EXPRESSION_H
#define AUTODIFK_REVERSE_SCALAR_EXPRESSION_H

#include <glog/logging.h>
#include <iostream>
#include <memory>
#include <vector>

namespace autodifk {
namespace reverse {

class ScalarExpression {
 public:
  // Value of the expression and of the derivative of the final expression
  // against this expression.
  double value;
  double derivative;

  // Builtin constructors.
  ScalarExpression() : value(0.0), derivative(0.0) {}
  ScalarExpression(double v) : value(v), derivative(0.0) {}
  ScalarExpression(double v, derivative d) : value(v), derivative(d) {}

  // Construct from subexpressions. Automatically calls a virtual  method to
  // check any properties of the subexpressions.
  ScalarExpression(const std::vector<ScalarExpression>& subexpressions)
      : ScalarExpression(), subexpressions_(subexpressions) {
    CHECK(CheckSubexpressions());
  }

  // Forward and backward passes.
  double ForwardPass() {
    if (subexpressions_.empty()) return value;

    for (auto& sub : subexpressions_) sub.ForwardPass();
    return ForwardPropagateValue();
  }
  void BackwardPass() {
    BackwardPropagateDerivative();
    for (auto& sub : subexpressions_) sub.BackwardPass();
  }

 private:
  // Check any specific properties of the subexpressions.
  virtual bool CheckSubexpressions() const { return true; }

  // Pure virtual functions.
  // Compute the value of this expression from the values of each subexpression.
  // Returns the value and also sets the public member variable 'value'.
  virtual double ForwardPropagateValue() = 0;

  // Compute the gradient of the final expression against all subexpressions.
  // Sets the 'derivative' field of all subexpressions.
  virtual void BackwardPropagateDerivative() = 0;

  // All the expressions that this expression depends upon.
  std::vector<ScalarExpression> subexpressions_;
};  // class ScalarExpression

}  // namespace reverse
}  // namespace autodifk

#endif
