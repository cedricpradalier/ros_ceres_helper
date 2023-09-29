#ifndef CERES_TEMPLATE_POSES_H
#define CERES_TEMPLATE_POSES_H
#include <stdio.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>



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

        template <typename DTin>
            TPose(const TPose<DTin> & T) {
                    Q[0]=DT(T.Q[0]);
                    Q[1]=DT(T.Q[1]);
                    Q[2]=DT(T.Q[2]);
                    Q[3]=DT(T.Q[3]);
                    T[0]=DT(T.T[0]);
                    T[1]=DT(T.T[1]);
                    T[2]=DT(T.T[2]);
            }

        template <typename DTin>
            void setFromAngleAxis(DTin tx, DTin ty, DTin tz,
                    DTin rx, DTin ry, DTin rz) {
                T[0]=DT(tx); T[1]=DT(ty); T[2]=DT(tz);
                DT aa[3] = {DT(rx),DT(ry),DT(rz)};
                ceres::AngleAxisToQuaternion(aa, Q);
            }

        template <typename DTin>
            void setFromAngleAxis(const DTin *tt, const DTin *aain) {
                T[0]=DT(tt[0]); T[1]=DT(tt[1]); T[2]=DT(tt[2]);
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
            void setFromEuler(DTin roll, DTin pitch, DTin yaw,
                    DTin x=DTin(0), DTin y=DTin(0), DTin z=DTin(0)) {
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

                T[0]=DT(x);
                T[1]=DT(y);
                T[2]=DT(z);
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

}

#endif // CERES_TEMPLATE_POSES_H
