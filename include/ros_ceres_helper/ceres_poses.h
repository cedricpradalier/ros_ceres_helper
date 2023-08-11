#ifndef CERES_POSES_H
#define CERES_POSES_H
#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace cerise{ 
    template <typename T>
        void invertTransform(const T* const Q, const T* const t,
                T* Qinv, T* tinv) {
            Qinv[0]=Q[0];
            Qinv[1]=-Q[1];
            Qinv[2]=-Q[2];
            Qinv[3]=-Q[3];
            ceres::QuaternionRotatePoint(Qinv, t, tinv);
            tinv[0]=-tinv[0]; 
            tinv[1]=-tinv[1]; 
            tinv[2]=-tinv[2];
        }

    template <typename T>
        void applyTransform(const T* const Q, const T* const t, const T* const Pin, T* Pout) {
            ceres::QuaternionRotatePoint(Q, Pin, Pout);
            Pout[0]+=t[0]; 
            Pout[1]+=t[1]; 
            Pout[2]+=t[2];
        }

    template <typename T>
        void composeTransform(const T* const Q1, const T* const t1,
                const T* const Q2, const T* const t2,
                T* Q, T* t) {
            ceres::QuaternionProduct(Q1,Q2,Q);
            ceres::QuaternionRotatePoint(Q1, t2, t);
            t[0]+=t1[0]; 
            t[1]+=t1[1]; 
            t[2]+=t1[2];
        }


    template <typename DT>
    struct TPose {
        DT T[3];
        DT Q[4];
        TPose() {
            T[0]=T[1]=T[2]=DT(0);
            ceres::AngleAxisToQuaternion<DT>(T,Q);
        }

        template <typename DTin>
            TPose(const DTin * const  q, const DTin * const t) {
                    Q[0]=DT(q[0]);
                    Q[1]=DT(q[1]);
                    Q[2]=DT(q[2]);
                    Q[3]=DT(q[3]);
                    T[0]=DT(t[0]);
                    T[1]=DT(t[1]);
                    T[2]=DT(t[2]);
            }

        TPose<DT> inverse() const {
            TPose<DT> out;
            invertTransform(Q,T,out.Q,out.T);
            return out;
        }

        TPose<DT> operator*(const TPose<DT> & P2) const {
            TPose<DT> out;
            composeTransform(Q,T,P2.Q,P2.T,out.Q,out.T);
            return out;
        }

        void apply(const DT* const Pin, DT* Pout) const {
            applyTransform(Q,T,Pin,Pout);
        }

        template <typename DTout>
            TPose<DTout> cast() const {
                TPose<DTout> out;
                cast<DTout>(out.Q,out.T);
                return out;
            }

        template <typename DTout>
            void cast(DTout * qout, DTout * tout) const {
                qout[0]=DTout(Q[0]);
                qout[1]=DTout(Q[1]);
                qout[2]=DTout(Q[2]);
                qout[3]=DTout(Q[3]);
                tout[0]=DTout(T[0]);
                tout[1]=DTout(T[1]);
                tout[2]=DTout(T[2]);
            }
    };

    typedef TPose<double> Pose;

}

#endif // CERES_POSES_H
