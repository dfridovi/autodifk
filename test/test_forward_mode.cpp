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

#include <forward/dual_scalar.h>
#include <forward/forward_autodiff.h>
#include <forward/utils.h>

#include <utils/numerical_diff.h>

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <random>

namespace autodifk {
namespace forward {

namespace {
// Make sure to specify that we might be using 'tan' to mean 'std::tan' OR our
// own tangent function defined on DualScalars. Same for other functions.
using std::cos;
using std::sin;
using std::tan;

// Scalar-valued functor to test gradients on. Templates the () operator.
struct ScalarFunctor {
  template <typename T>
  Eigen::Matrix<T, 1, 1> operator()(const Eigen::Matrix<T, 1, 1>& x) const {
    Eigen::Matrix<T, 1, 1> output;
    output(0) = -x(0) * (x(0) - 1.0) * tan(x(0));

    return output;
  }
};  // struct ScalarFunctor

// Vector-valued functor to test gradients on. Templates the () operator.
struct VectorFunctor {
  template <typename T>
  Eigen::Matrix<T, 2, 1> operator()(const Eigen::Matrix<T, 2, 1>& x) const {
    Eigen::Matrix<T, 2, 1> output;
    output(0) = -x(0) * (x(1) - 1.0) * cos(x(1)) + x(1) * sin(x(0));
    output(1) = x(1) * (x(0) - 1.0) * cos(x(1) - x(1) * sin(x(0)));

    return output;
  }
};  // struct VectorFunctor

}  // namespace

// Test forward mode autodifferentiator on an arbitrary scalar function.
TEST(ForwardAutodiff, TestScalar) {
  const ScalarFunctor functor;
  ForwardAutodiff<ScalarFunctor, 1, 1> automatic_diff(functor);
  NumericalDiff<ScalarFunctor, 1, 1> numerical_diff(functor);

  // Check that gradients match at a bunch of random points.
  std::random_device rd;
  std::default_random_engine rng(rd());
  std::uniform_real_distribution<double> unif(0.0, 1.0);

  constexpr size_t kNumRandomTests = 100;
  Eigen::Matrix<double, 1, 1> x, automatic_jacobian, numerical_jacobian;
  for (size_t ii = 0; ii < kNumRandomTests; ii++) {
    x(0) = unif(rng);

    // Evaluate and get derivatives.
    automatic_diff.Evaluate(x, &automatic_jacobian);
    numerical_diff.Evaluate(x, &numerical_jacobian);

    // Make sure that the Jacobians are close.
    constexpr double kSmallNumber = 1e-3;
    EXPECT_NEAR(automatic_jacobian(0), numerical_jacobian(0), kSmallNumber);
  }
}

// Test forward mode autodifferentiator on an arbitrary vector function.
TEST(ForwardAutodiff, TestVector) {
  const VectorFunctor functor;
  ForwardAutodiff<VectorFunctor, 2, 2> automatic_diff(functor);
  NumericalDiff<VectorFunctor, 2, 2> numerical_diff(functor);

  // Check that gradients match at a bunch of random points.
  std::random_device rd;
  std::default_random_engine rng(rd());
  std::uniform_real_distribution<double> unif(0.0, 1.0);

  constexpr size_t kNumRandomTests = 100;
  Eigen::Matrix<double, 2, 1> x;
  Eigen::Matrix<double, 2, 2> automatic_jacobian, numerical_jacobian;
  for (size_t ii = 0; ii < kNumRandomTests; ii++) {
    x(0) = unif(rng);
    x(1) = unif(rng);

    // Evaluate and get derivatives.
    automatic_diff.Evaluate(x, &automatic_jacobian);
    numerical_diff.Evaluate(x, &numerical_jacobian);

    // Make sure that the Jacobians are close.
    constexpr double kSmallNumber = 1e-3;
    EXPECT_LT(
        (automatic_jacobian - numerical_jacobian).lpNorm<Eigen::Infinity>(),
        kSmallNumber);
  }
}

}  // namespace forward
}  // namespace autodifk
