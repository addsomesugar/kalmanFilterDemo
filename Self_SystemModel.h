//
// Created by hdf on 2020/11/22.
//

#ifndef KALMANFILTER_SELF_SYSTEMMODEL_H
#define KALMANFILTER_SELF_SYSTEMMODEL_H

#include "kalman/LinearizedSystemModel.h"

namespace KalmanExamples
{
    namespace Robot1
    {
        template <typename T>
        class State: public Kalman::Vector<T, 4>
        {
        public:
            State() : Kalman::Vector<T, 4>() {}

            template<typename OtherDerived>
            State(const Eigen::MatrixBase<OtherDerived> &other) : Kalman::Vector<T,4>(other){}

            template<typename OtherDerived>
            State& operator= (const Eigen::MatrixBase<OtherDerived> &other) {
                this->Kalman::Vector<T, 4>::operator=(other);
                return *this;
            }

            static constexpr size_t X = 0;
            static constexpr size_t Y = 1;
            static constexpr size_t V_X = 2;
            static constexpr size_t V_Y = 3;

            T xx() const {return (*this)[X];}
            T yy() const {return (*this)[Y];}
            T vx() const {return (*this)[V_X];}
            T vy() const {return (*this)[V_Y];}

            T& xx()      {return (*this)[X];}
            T& yy()      {return (*this)[Y];}
            T& vx()      {return (*this)[V_X];}
            T& vy()      {return (*this)[V_Y];}
        };



        template <class T, class ControlType=Kalman::Vector<T, 0>, template<class> class CovarianceBase = Kalman::StandardBase>
        class VelocitySystemModel : public Kalman::LinearizedSystemModel<State<T>, ControlType, CovarianceBase>
        {
        public:
            //! State type shortcut definition
            using S = State<T>;

            VelocitySystemModel(){
                this->F.setZero();
                this->F(S::X, S::X) = 1;
                this->F(S::Y, S::Y) = 1;
                this->F(S::V_X, S::V_X) = 1;
                this->F(S::V_Y, S::V_Y) = 1;
                this->F(S::V_X, S::X) = 1;
                this->F(S::V_Y, S::Y) = 1;

                this->W.setIdentity();
            }

            State<T> f(const State<T> &x, const ControlType &u) const {
                State<T> x_;
                x_.xx() = x.xx() + x.vx();
                x_.yy() = x.yy() + x.vy();
                x_.vx() = x.vx();
                x_.vy() = x.vy();
                return x_;
            }

        protected:
            void updateJacobians(const State<T>& x, const ControlType& u) override {}
        };
    }
}


#endif //KALMANFILTER_SELF_SYSTEMMODEL_H
