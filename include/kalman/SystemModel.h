//
// Created by hdf on 2020/11/22.
//

#ifndef KALMANFILTER_SYSTEMMODEL_H
#define KALMANFILTER_SYSTEMMODEL_H

#include "StandardBase.h"

namespace Kalman
{
    template <class StateType, class ControlType=Vector<typename StateType::Scalar, 0>, template<class> class CovarianceBase = StandardBase>
    class SystemModel : public CovarianceBase<StateType>
    {
        static_assert(/*StateType::RowsAtCompileTime == Dynamic ||*/ StateType::RowsAtCompileTime > 0,
                                                                     "State vector must contain at least 1 element" /* or be dynamic */);
        static_assert(/*ControlType::RowsAtCompileTime == Dynamic ||*/ ControlType::RowsAtCompileTime >= 0,
                                                                       "Control vector must contain at least 0 elements" /* or be dynamic */);
        static_assert(std::is_same<typename StateType::Scalar, typename ControlType::Scalar>::value,
                      "State and Control scalar types must be identical");

    public:
        virtual StateType f(const StateType& x, const ControlType& u) const = 0;

    protected:
        SystemModel() {}
        virtual ~SystemModel() {}
    };
}

#endif //KALMANFILTER_SELF_SYSTEMMODEL_H
