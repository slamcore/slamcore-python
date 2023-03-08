/******************************************************************************
 *
 * Slamcore Confidential
 * ---------------------
 *
 * Slamcore Limited
 * All Rights Reserved.
 * (C) Copyright 2021
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

#include "MatrixToEigen.hpp"
#include "slamcore/objects/matrix.hpp"

#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <type_traits>

template <typename>
struct isMatrix : public std::false_type
{
};

template <typename Scalar, int Rows, int Cols>
struct isMatrix<slamcore::Matrix<Scalar, Rows, Cols>> : public std::true_type
{
};

namespace pybind11::detail
{
template <typename Type>
struct type_caster<Type, std::enable_if_t<isMatrix<Type>::value>>
{
public:
  using ScalarT = typename Type::Scalar;
  using EigenRefT = Eigen::Ref<Eigen::Matrix<ScalarT, Type::CompileTimeRows, Type::CompileTimeCols>>;

  PYBIND11_TYPE_CASTER(Type, type_caster<EigenRefT>::name);

  bool load(handle src, bool convert)
  {
    type_caster<EigenRefT> eigenCaster;
    if (!eigenCaster.load(src, convert))
    {
      return false;
    }

    const EigenRefT& eigenRef = eigenCaster;
    value.resize(eigenRef.rows(), eigenRef.cols());
    slamcore::makeEigenMap(value) = eigenRef;

    return true;
  }

  static handle cast(const Type& matrix, return_value_policy policy, handle parent)
  {
    auto thisMap = slamcore::makeEigenMap(matrix);
    return type_caster<decltype(thisMap)>::cast(thisMap, policy, parent);
  }
};
} // namespace pybind11::detail
