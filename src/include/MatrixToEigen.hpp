/******************************************************************************
 *
 * Slamcore Confidential
 * ---------------------
 *
 * Slamcore Limited
 * All Rights Reserved.
 * (C) Copyright 2020
 *
 * NOTICE:
 *
 * All information contained herein is, and remains the property of Slamcore
 * Limited and its suppliers, if any. The intellectual and technical concepts
 * contained herein are proprietary to Slamcore Limited and its suppliers and
 * may be covered by patents in process, and are protected by trade secret or
 * copyright law. Dissemination of this information or reproduction of this
 * material is strictly forbidden unless prior written permission is obtained
 * from Slamcore Limited.
 *
 ******************************************************************************/

/**
 * @file
 * @brief Adaptor functions to instantiate Eigen vectors/matrices/transforms from Slamcore interface objects.
 */

#pragma once

#include "slamcore/objects/matrix.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace slamcore
{

namespace detail
{
namespace
{
// use unaligned for now (as customers may give us unaligned data)
/// @todo look into this!
constexpr auto EigenMapAlignment = Eigen::AlignmentType::Unaligned;
} // namespace
} // namespace detail

template <typename Scalar, int Rows, int Cols>
using EigenMapT = Eigen::Map<Eigen::Matrix<Scalar, Rows, Cols>, detail::EigenMapAlignment>;
template <typename Scalar, int Rows, int Cols>
using ConstEigenMapT = Eigen::Map<const Eigen::Matrix<Scalar, Rows, Cols>, detail::EigenMapAlignment>;

template <typename Scalar>
using QuaternionMapT = Eigen::Map<Eigen::Quaternion<Scalar>, detail::EigenMapAlignment>;
template <typename Scalar>
using ConstQuaternionMapT =
  const Eigen::Map<const Eigen::Quaternion<Scalar>, detail::EigenMapAlignment>;

template <typename Scalar, int Rows, int Cols>
EigenMapT<Scalar, Rows, Cols> makeEigenMap(Matrix<Scalar, Rows, Cols>& matrix)
{
  return EigenMapT<Scalar, Rows, Cols>(matrix.data(), matrix.rows(), matrix.cols());
}

template <typename Scalar, int Rows, int Cols>
ConstEigenMapT<Scalar, Rows, Cols> makeEigenMap(const Matrix<Scalar, Rows, Cols>& matrix)
{
  return ConstEigenMapT<Scalar, Rows, Cols>(matrix.data(), matrix.rows(), matrix.cols());
}

template <typename Scalar>
QuaternionMapT<Scalar> makeEigenQuaternionMap(Vector<Scalar, 4>& vector)
{
  return QuaternionMapT<Scalar>(vector.data());
}

template <typename Scalar>
ConstQuaternionMapT<Scalar> makeEigenQuaternionMap(const Vector<Scalar, 4>& vector)
{
  return ConstQuaternionMapT<Scalar>(vector.data());
}

template <typename EigenDerived>
auto fromEigen(const Eigen::MatrixBase<EigenDerived>& matrix)
{
  using Scalar = typename EigenDerived::Scalar;
  constexpr auto Rows = EigenDerived::RowsAtCompileTime;
  constexpr auto Cols = EigenDerived::ColsAtCompileTime;

  Matrix<Scalar, Rows, Cols> result;
  result.resize(matrix.rows(), matrix.cols());
  makeEigenMap(result) = matrix;
  return result;
}

template <int Rows, int Cols, typename Scalar, int OtherRows, int OtherCols>
auto toStaticMatrix(const Matrix<Scalar, OtherRows, OtherCols>& other)
{
  using ResultT = Matrix<Scalar, Rows, Cols>;

  static_assert(!ResultT::IsDynamic, "Use to construct static matrices only!");
  assert(Rows == other.rows() && "Rows dont match");
  assert(Cols == other.cols() && "Columns dont match");

  ResultT result;

  makeEigenMap(result) = makeEigenMap(other);

  return result;
}

template <int Rows, typename Scalar>
auto toStaticVector(const Vector<Scalar, Dynamic>& other)
{
  return toStaticMatrix<Rows, 1>(other);
}

} // namespace slamcore
