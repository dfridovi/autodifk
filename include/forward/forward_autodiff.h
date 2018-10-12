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
// Forward-mode autodifferentiator. Templated on a functor and the input/output
// dimensions of that functor. The functor's operator() must be templated so
// that it may be used with DualScalars.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef AUTODIFK_FORWARD_FORWARD_AUTODIFF_H
#define AUTODIFK_FORWARD_FORWARD_AUTODIFF_H

#include "../forward/dual_scalar.h"

#include <glog/logging.h>
#include <Eigen/Core>
#include <iostream>
#include <vector>

namespace autodifk {
namespace forward {

template <typename F, size_t N_INPUTS, size_t N_OUTPUTS>
class ForwardAutodiff {
 public:
  ~ForwardAutodiff() {}
  explicit ForwardAutodiff(const F& functor) : functor_(functor) {}

  // Evaluate the functor and optionally populate the Jacobian of the functor.
  Eigen::Matrix<double, N_OUTPUTS, 1> Evaluate(
      const Eigen::Matrix<double, N_INPUTS, 1>& x,
      Eigen::Matrix<double, N_OUTPUTS, N_INPUTS>* jacobian);

 private:
  const F functor_;
};  // class ForwardAutodiff

// ------------------------------ IMPLEMENTATION ---------------------------- //

template <typename F, size_t N_INPUTS, size_t N_OUTPUTS>
Eigen::Matrix<double, N_OUTPUTS, 1>
ForwardAutodiff<F, N_INPUTS, N_OUTPUTS>::Evaluate(
    const Eigen::Matrix<double, N_INPUTS, 1>& x,
    Eigen::Matrix<double, N_OUTPUTS, N_INPUTS>* jacobian) {
  // If no Jacobian is required, just evaluate the functor.
  if (!jacobian) return functor_(x);

  // Convert input 'x' to DualScalar format. By default, all derivatives stored
  // in dual numbers are zero.
  Eigen::Matrix<DualScalar, N_INPUTS, 1> x_dual;
  for (size_t ii = 0; ii < N_INPUTS; ii++) x_dual(ii) = x(ii);

  // Create return container.
  Eigen::Matrix<double, N_OUTPUTS, 1> output;

  // Evaluate the functor once for each input to compute the corresponding
  // column of the Jacobian.
  for (size_t jj = 0; jj < N_INPUTS; jj++) {
    // Set input partial derivative in this dimension to 1.0.
    x_dual(jj).derivative = 1.0;

    // Evaluate functor on DualScalars.
    const auto output_dual = functor_(x_dual);
    CHECK_EQ(output_dual.size(), N_OUTPUTS);

    // If this is the first time through, fill in the output container.
    if (jj == 0) {
      for (size_t ii = 0; ii < N_OUTPUTS; ii++) output(ii) = output_dual(ii).value;
    }

    // Set corresponding column of Jacobian.
    for (size_t ii = 0; ii < N_OUTPUTS; ii++)
      (*jacobian)(ii, jj) = output_dual(ii).derivative;

    // Reset input partial derivative to 0.0 in this dimension.
    x_dual(jj).derivative = 0.0;
  }

  return output;
}

}  // namespace forward
}  // namespace autodifk

#endif
