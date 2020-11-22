//
// Created by hdf on 2020/11/22.
//

#ifndef KALMANFILTER_SELF_MEASUREMENT_H
#define KALMANFILTER_SELF_MEASUREMENT_H
#include "kalman/LinearizedMeasurementModel.h"

namespace KalmanExamples
{
    namespace Robot1
    {
        template <typename T>
        class PositionMeasurement: public Kalman::Vector<T, 2>
        {
        public:
            PositionMeasurement() : Kalman::Vector<T, 2>() {}

            template<typename OtherDerived>
            PositionMeasurement(const Eigen::MatrixBase<OtherDerived> &other) : Kalman::Vector<T,2>(other){}

            template<typename OtherDerived>
            PositionMeasurement& operator= (const Eigen::MatrixBase<OtherDerived> &other) {
                this->swap(other);
                return *this;
            }

            static constexpr size_t X = 0;
            static constexpr size_t Y = 1;

            T xx()    const { return (*this)[X];}
            T yy()    const { return (*this)[Y];}

            T& xx()     { return (*this)[X];}
            T& yy()     { return (*this)[Y];}
        };

        template <typename T, template<class> class CovarianceBase = Kalman::StandardBase>
        class PositionMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, PositionMeasurement<T>, CovarianceBase>
        {
        public:
            //! State type shortcut definition
            using S = State<T>;

            //! Measurement type shortcut definition
            using M = PositionMeasurement<T>;
        public:
            PositionMeasurementModel() {
                this->H(M::X, S::X) = 1;
                this->H(M::Y, S::Y) = 1;
                this->V.setIdentity();
            }

            PositionMeasurement<T> h(const State<T> &x) const {
                PositionMeasurement<T> measurement;
                auto position = x.template head<4>();

                measurement.xx() = position[0];
                measurement.yy() = position[1];
                return measurement;
            }

        protected:
            void updateJacobians(const State<T> &x) {}
        };
    }
}

#endif //KALMANFILTER_SELF_MEASUREMENT_H
