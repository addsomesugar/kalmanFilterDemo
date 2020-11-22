//
// Created by hdf on 2020/11/22.
//

#ifndef KALMANFILTER_TYPES_H
#define KALMANFILTER_TYPES_H

#include "Matrix.h"

namespace Kalman
{
    template <typename T, int N>
    using SquareMatrix = Eigen::Matrix<T, N, N>;

    template <class Type>
    using Covariance = SquareMatrix<typename Type::Scalar, Type::RowsAtCompileTime>;

    template <class State, class Measurement>
    using KalmanGain = Eigen::Matrix<typename State::Scalar,
    State::RowsAtCompileTime, Measurement::RowsAtCompileTime>;

    template <class A, class B>
    using Jacobian = Eigen::Matrix<typename A::Scalar, A::RowsAtCompileTime, B::RowsAtCompileTime>;

}

#endif //KALMANFILTER_TYPES_H
