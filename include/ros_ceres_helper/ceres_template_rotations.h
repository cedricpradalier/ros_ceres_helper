#ifndef CERES_TEMPLATE_ROTATIONS_H
#define CERES_TEMPLATE_ROTATIONS_H
#include <stdio.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace cerise{ 
    template <typename T>
        void invertRotation(const T* const Q,
                T* Qinv) {
            Qinv[0]=Q[0];
            Qinv[1]=-Q[1];
            Qinv[2]=-Q[2];
            Qinv[3]=-Q[3];
        }

    template <typename T>
        void applyRotation(const T* const Q,  const T* const Pin, T* Pout) {
            ceres::QuaternionRotatePoint(Q, Pin, Pout);
        }

    template <typename T>
        void composeRotation(const T* const Q1, 
                const T* const Q2, 
                T* Q) {
            ceres::QuaternionProduct(Q1,Q2,Q);
        }


    template <typename DT>
    struct TRotation {
        DT Q[4];
        TRotation() {
            DT T[3];
            T[0]=T[1]=T[2]=DT(0);
            ceres::AngleAxisToQuaternion<DT>(T,Q);
        }

        template <typename DTin>
            TRotation(const DTin * const  q) {
                    Q[0]=DT(q[0]);
                    Q[1]=DT(q[1]);
                    Q[2]=DT(q[2]);
                    Q[3]=DT(q[3]);
            }

        template <typename DTin>
            TRotation(const TRotation<DTin> & T) {
                    Q[0]=DT(T.Q[0]);
                    Q[1]=DT(T.Q[1]);
                    Q[2]=DT(T.Q[2]);
                    Q[3]=DT(T.Q[3]);
            }

        template <typename DTin>
            void setFromAngleAxis(DTin x, DTin y, DTin z) {
                DT aa[3] = {DT(x),DT(y),DT(z)};
                ceres::AngleAxisToQuaternion(aa, Q);
            }

        template <typename DTin>
            void setFromAngleAxis(const DTin *aain) {
                DT aa[3] = {DT(aain[0]),DT(aain[1]),DT(aain[2])};
                ceres::AngleAxisToQuaternion(aa, Q);
            }

        template <typename DTout>
            void getAngleAxis(DTout *aaout) const {
                DT aa[3]={DT(0),DT(0),DT(0)};
                ceres::QuaternionToAngleAxis(Q,aa);
                aaout[0]=DTout(aa[0]);
                aaout[1]=DTout(aa[1]);
                aaout[2]=DTout(aa[2]);
            }

        template <typename DTin>
            void setFromEuler(DTin roll, DTin pitch, DTin yaw) {
                DT Q0[4] = { DT(1), DT(0), DT(0), DT(0) };
                DT Q1[4];
                DT Qroll[4], Qpitch[4], Qyaw[4];
                DT aaroll[3]={DT(roll),DT(0),DT(0)};
                ceres::AngleAxisToQuaternion(aaroll, Qroll);
                DT aapitch[3]={DT(0),DT(pitch),DT(0)};
                ceres::AngleAxisToQuaternion(aapitch, Qpitch);
                DT aayaw[3]={DT(0),DT(0),DT(yaw)};
                ceres::AngleAxisToQuaternion(aayaw, Qyaw);
                ceres::QuaternionProduct(Qroll,Q0,Q1);
                ceres::QuaternionProduct(Qpitch,Q1,Q0);
                ceres::QuaternionProduct(Qyaw,Q0,Q);
            }
        

        TRotation<DT> inverse() const {
            TRotation<DT> out;
            invertRotation(Q,out.Q);
            return out;
        }

        TRotation<DT> operator*(const TRotation<DT> & P2) const {
            TRotation<DT> out;
            composeRotation(Q,P2.Q,out.Q);
            return out;
        }

        void apply(const DT* const Pin, DT* Pout) const {
            applyRotation(Q,Pin,Pout);
        }

        template <typename DTout>
            TRotation<DTout> cast() const {
                TRotation<DTout> out;
                cast<DTout>(out.Q);
                return out;
            }

        template <typename DTout>
            void cast(DTout * qout) const {
                qout[0]=DTout(Q[0]);
                qout[1]=DTout(Q[1]);
                qout[2]=DTout(Q[2]);
                qout[3]=DTout(Q[3]);
            }
    };

}

#endif // CERES_TEMPLATE_ROTATIONS_H
