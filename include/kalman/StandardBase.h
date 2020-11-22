//
// Created by hdf on 2020/11/22.
//

#ifndef KALMANFILTER_STANDARDBASE_H
#define KALMANFILTER_STANDARDBASE_H

#include "Types.h"

namespace Kalman
{
    template <class StateType>
    class StandardBase
    {
    protected:
        Covariance<StateType> P;

    public:

        const Covariance<StateType> &getCovariance() const {
            return P;
        }

        bool setCovariance(const Covariance<StateType> &covariance){
            P = covariance;
            return true;
        }

    protected:
        StandardBase()
        {
            P.setIdentity();
        }
    };
}

#endif //KALMANFILTER_STANDARDBASE_H
