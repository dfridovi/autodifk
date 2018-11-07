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

template <size_t N>
struct Sum : public ScalarExpression<N> {
  double ForwardPropagateValue();
  void BackwardPropagateDerivative();
};  // struct Sum

template <size_t N>
struct Product : public ScalarExpression<N> {
  double ForwardPropagateValue();
  void BackwardPropagateDerivative();
};  // struct Product

struct Sin : public ScalarExpression<1> {
  double ForwardPropagateValue();
  void BackwardPropagateDerivative();
};  // struct Sin

struct Cos : public ScalarExpression<1> {
  double ForwardPropagateValue();
  void BackwardPropagateDerivative();
};  // struct Cos

struct Tan : public ScalarExpression<1> {
  double ForwardPropagateValue();
  void BackwardPropagateDerivative();
};  // struct Tan

struct Exp : public ScalarExpression<1> {
  double ForwardPropagateValue();
  void BackwardPropagateDerivative();
};  // struct Exp

struct Log : public ScalarExpression<1> {
  double ForwardPropagateValue();
  void BackwardPropagateDerivative();
};  // struct Log

struct Abs : public ScalarExpression<1> {
  double ForwardPropagateValue();
  void BackwardPropagateDerivative();
};  // struct Abs

struct ReLU : public ScalarExpression<1> {
  double ForwardPropagateValue();
  void BackwardPropagateDerivative();
};  // struct ReLU

}  // namespace reverse
}  // namespace autodifk

#endif
