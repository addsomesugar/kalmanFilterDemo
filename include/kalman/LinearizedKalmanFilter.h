//
// Created by hdf on 2020/11/22.
//

#ifndef KALMANFILTER_LINEARIZEDKALMANFILTER_H
#define KALMANFILTER_LINEARIZEDKALMANFILTER_H

#include "KalmanFilterBase.h"
#include "StandardFilterBase.h"
#include "LinearizedSystemModel.h"
#include "LinearizedMeasurementModel.h"

namespace Kalman
{
    template <class StateType>
    class LinearizedKalmanFilter : public KalmanFilterBase<StateType>, public StandardFilterBase<StateType>
    {
    public:
        //! Linearized Measurement Model Type
        template<class Measurement, template<class> class CovarianceBase>
        using MeasurementModelType = LinearizedMeasurementModel<StateType, Measurement, CovarianceBase>;

        //! Linearized System Model Type
        template<class Control, template<class> class CovarianceBase>
        using SystemModelType = LinearizedSystemModel<StateType, Control, CovarianceBase>;
    protected:
        using KalmanFilterBase<StateType>::x;
        using StandardFilterBase<StateType>::P;

    public:
        LinearizedKalmanFilter(){
            P.setIdentity();
        }

        template<class Control, template<class> class CovarianceBase>
        const StateType& predict(SystemModelType<Control,CovarianceBase>& s) {
            Control u;
            u.setZero();
            return predict(s, u);
        }

        template<class Control, template<class> class CovarianceBase>
        const StateType& predict( SystemModelType<Control, CovarianceBase>& s, const Control& u )
        {
            // predict state
            x = s.f(x, u);

            // predict covariance
            P  = ( s.F * P * s.F.transpose() ) + ( s.W * s.getCovariance() * s.W.transpose() );

            // return state prediction
            return this->getState();
        }

        template<class Measurement, template<class> class CovarianceBase>
        const StateType& update( MeasurementModelType<Measurement, CovarianceBase>& m, const Measurement& z )
        {
            Covariance<Measurement> S = ( m.H * P * m.H.transpose() ) + ( m.V * m.getCovariance() * m.V.transpose() );

            // compute kalman gain
            Kalman::KalmanGain<StateType, Measurement> K = P * m.H.transpose() * S.inverse();

            // UPDATE STATE ESTIMATE AND COVARIANCE
            // Update state using computed kalman gain and innovation
            x += K * ( z - m.h( x ) );

            // Update covariance
            P -= K * m.H * P;

            // return updated state estimate
            return this->getState();
        }
    };
}


#endif //KALMANFILTER_LINEARIZEDKALMANFILTER_H
