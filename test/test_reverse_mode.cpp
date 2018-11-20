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

#include <reverse/utils.h>

#include <utils/numerical_diff.h>

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <random>

namespace autodifk {
namespace reverse {

// Test reverse mode autodifferentiator on an arbitrary scalar function.
TEST(ReverseAutodiff, TestScalar) {
  const ScalarFunctor functor;
  ForwardAutodiff<ScalarFunctor, 1, 1> automatic_diff(functor);
  NumericalDiff<ScalarFunctor, 1, 1> numerical_diff(functor);

  // Check that gradients match at a bunch of random points.
  std::random_device rd;
  std::default_random_engine rng(rd());
  std::uniform_real_distribution<double> unif(0.0, 1.0);

  constexpr size_t kNumRandomTests = 100;
  Eigen::Matrix<double, 1, 1> x, automatic_derivative, numerical_derivative;
  for (size_t ii = 0; ii < kNumRandomTests; ii++) {
    x(0) = unif(rng);

    // Evaluate and get derivatives.
    automatic_diff.Evaluate(x, &automatic_derivative);
    numerical_diff.Evaluate(x, &numerical_derivative);

    // Make sure that the derivatives are close.
    constexpr double kSmallNumber = 1e-3;
    EXPECT_LT(std::abs(automatic_derivative(0) - numerical_derivative(0)),
              kSmallNumber);
  }
}

}  // namespace reverse
}  // namespace autodifk
