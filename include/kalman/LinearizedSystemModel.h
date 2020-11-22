//
// Created by hdf on 2020/11/22.
//

#ifndef KALMANFILTER_LINEARIZEDSYSTEMMODEL_H
#define KALMANFILTER_LINEARIZEDSYSTEMMODEL_H

#include "SystemModel.h"

namespace Kalman
{
    template <class StateType>
    class LinearizedKalmanFilter;

    template <class StateType, class ControlType=Vector<typename StateType::Scalar, 0>, template<class> class CovarianceBase = StandardBase>
    class LinearizedSystemModel : public SystemModel<StateType, ControlType, CovarianceBase>
    {
        friend class LinearizedKalmanFilter<StateType>;

    protected:
        Jacobian<StateType, StateType> F;
        Jacobian<StateType, StateType> W;

        virtual void updateJacobians(const StateType& x, const ControlType& u) {}

    protected:
        LinearizedSystemModel() {
            F.setIdentity();
            W.setIdentity();
        }
        ~LinearizedSystemModel(){}
    };
}


#endif //KALMANFILTER_LINEARIZEDSYSTEMMODEL_H
