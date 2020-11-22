//
// Created by hdf on 2020/11/22.
//

#ifndef KALMANFILTER_MATRIX_H
#define KALMANFILTER_MATRIX_H

#include <Eigen/Dense>

namespace Kalman
{
    template <typename T, int N>
    class Vector : public Eigen::Matrix<T, N, 1>
    {
    public:
        Vector():Eigen::Matrix<T,N,1>(){}

        template<typename OtherDerived>
        Vector(const Eigen::MatrixBase<OtherDerived> &other):Eigen::Matrix<T,N,1>(other){}

        template<typename OtherDerived>
        Vector& operator=(const Eigen::MatrixBase<OtherDerived> &other) {
            this->Eigen::Matrix<T, N, 1>::operator=(other);
            return *this;
        }
    };
}

#endif //KALMANFILTER_MATRIX_H
