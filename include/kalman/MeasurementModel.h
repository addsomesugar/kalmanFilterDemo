//
// Created by hdf on 2020/11/22.
//

#ifndef KALMANFILTER_MEASUREMENTMODEL_H
#define KALMANFILTER_MEASUREMENTMODEL_H

#include "StandardBase.h"

namespace Kalman
{
    template<class StateType, class MeasurementType, template<class> class CovarianceBase = StandardBase>
    class MeasurementModel : public CovarianceBase<MeasurementType>
    {
        static_assert(/*StateType::RowsAtCompileTime == Dynamic ||*/StateType::RowsAtCompileTime > 0,
                                                                    "State vector must contain at least 1 element" /* or be dynamic */);
        static_assert(/*MeasurementType::RowsAtCompileTime == Dynamic ||*/MeasurementType::RowsAtCompileTime > 0,
                                                                          "Measurement vector must contain at least 1 element" /* or be dynamic */);
        static_assert(std::is_same<typename StateType::Scalar, typename MeasurementType::Scalar>::value,
                      "State and Measurement scalar types must be identical");
    public:
        virtual MeasurementType h(const StateType& x) const = 0;

    protected:
        MeasurementModel() {}
        virtual ~MeasurementModel() {}
    };
}

#endif //KALMANFILTER_MEASUREMENTMODEL_H
