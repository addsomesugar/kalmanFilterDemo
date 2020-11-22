//
// Created by hdf on 2020/11/22.
//

#ifndef KALMANFILTER_KALMANFILTERBASE_H
#define KALMANFILTER_KALMANFILTERBASE_H

#include "Types.h"

namespace Kalman
{
    template <class StateType>
    class KalmanFilterBase
    {
    public:
        static_assert(/*StateType::RowsAtCompileTime == Dynamic ||*/StateType::RowsAtCompileTime > 0,
                                                                    "State vector must contain at least 1 element" /* or be dynamic */);
        static_assert(StateType::ColsAtCompileTime == 1, "State type must be a column vector");

    protected:
        StateType x;

    public:
        const StateType& getState() {
            return x;
        }

        void init(const StateType& initialState) {
            x = initialState;
        }

    protected:
        KalmanFilterBase() {}
    };
}



#endif //KALMANFILTER_KALMANFILTERBASE_H
