//
// Created by hdf on 2020/11/22.
//

#ifndef KALMANFILTER_LINEARIZEDMEASUREMENTMODEL_H
#define KALMANFILTER_LINEARIZEDMEASUREMENTMODEL_H

#include "MeasurementModel.h"

namespace Kalman
{
    template <class StateType>
    class LinearizedKalmanFilter;

    template<class StateType, class MeasurementType, template<class> class CovarianceBase = StandardBase>
    class LinearizedMeasurementModel : public MeasurementModel<StateType, MeasurementType, CovarianceBase>
    {
        friend class LinearizedKalmanFilter<StateType>;

    protected:
        Jacobian<MeasurementType, StateType> H;
        Jacobian<MeasurementType, MeasurementType> V;

        virtual void updateJacobians(const StateType& x) {}

    protected:
        LinearizedMeasurementModel() {
            H.setIdentity();
            V.setIdentity();
        }
        ~LinearizedMeasurementModel() {}
    };
}

#endif //KALMANFILTER_LINEARIZEDMEASUREMENTMODEL_H
