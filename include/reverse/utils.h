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

#ifndef AUTODIFK_REVERSE_UTILS_H
#define AUTODIFK_REVERSE_UTILS_H

#include "../reverse/scalar_expression.h"

#include <glog/logging.h>
#include <iostream>
#include <memory>
#include <vector>

namespace autodifk {
namespace reverse {

class Sum : public ScalarExpression {
 public:
  using ScalarExpression::ScalarExpression;

 private:
  double ForwardPropagateValue();
  void BackwardPropagateDerivative();
};  // class Sum

class Product : public ScalarExpression {
 public:
  using ScalarExpression::ScalarExpression;

 private:
  double ForwardPropagateValue();
  void BackwardPropagateDerivative();
};  // class Product

class Sin : public UnaryScalarExpression {
 public:
  using UnaryScalarExpression::UnaryScalarExpression;

 private:
  double ForwardPropagateValue();
  void BackwardPropagateDerivative();
};  // class Sin

class Cos : public UnaryScalarExpression {
 public:
  using UnaryScalarExpression::UnaryScalarExpression;

 private:
  double ForwardPropagateValue();
  void BackwardPropagateDerivative();
};  // class Cos

class Tan : public UnaryScalarExpression {
 public:
  using UnaryScalarExpression::UnaryScalarExpression;

 private:
  double ForwardPropagateValue();
  void BackwardPropagateDerivative();
};  // class Tan

class Exp : public UnaryScalarExpression {
 public:
  using UnaryScalarExpression::UnaryScalarExpression;

 private:
  double ForwardPropagateValue();
  void BackwardPropagateDerivative();
};  // class Exp

class Log : public UnaryScalarExpression {
 public:
  using UnaryScalarExpression::UnaryScalarExpression;

 private:
  double ForwardPropagateValue();
  void BackwardPropagateDerivative();
};  // class Log

class Abs : public UnaryScalarExpression {
 public:
  using UnaryScalarExpression::UnaryScalarExpression;

 private:
  double ForwardPropagateValue();
  void BackwardPropagateDerivative();
};  // class Abs

class ReLU : public UnaryScalarExpression {
 public:
  using UnaryScalarExpression::UnaryScalarExpression;

 private:
  double ForwardPropagateValue();
  void BackwardPropagateDerivative();
};  // class ReLU

}  // namespace reverse
}  // namespace autodifk

#endif
