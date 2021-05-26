#ifndef MADGWICK_AHRS_H_
#define MADGWICK_AHRS_H_

#include <linalg/linalg.h>
#include <math.h>

#include <igris/math/fast_invsqrt.h>
#include <igris/math/defs.h>

#define RAD_TO_DEG 180.0 / M_PI
#define SAMPLE_FREQ 1000.0f     // sample frequency in Hz
#define BETA_DEF    0.5f        // 2 * proportional gain

namespace ralgo
{
    class madgwick
    {

    public:
        madgwick();

        linalg::vec<float,4> quat()
        { return linalg::vec<float,4>(q1,q2,q3,q0); };
        
        void reset();
        void reset(float _q0,float _q1,float _q2,float _q3) 
        { q0 = _q0; q1 = _q1; q2 = _q2; q3 = _q3; }

        void reset(const linalg::vec<float,4> & q) 
        { q0 = q.w; q1 = q.x; q2 = q.y; q3 = q.z; }

        void setKoeff(float sampleFreq, float beta);

        void update_magnetic_reference_direction(float umx, float umy, float umz);
        void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
        void update(float gx, float gy, float gz, float ax, float ay, float az);
        void update(float gx, float gy, float gz);

//        void update(linalg::vec<float,3> g, linalg::vec<float,3> a, linalg::vec<float,3> m);
//        void update(linalg::vec<float,3> g, linalg::vec<float,3> a);
//        void update(linalg::vec<float,3> g);


        void apply(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
        void apply(float gx, float gy, float gz, float ax, float ay, float az);
        void apply(float gx, float gy, float gz);

        linalg::vec<float, 3> gravity_direction();

        float getPitchRad();
        float getRollRad();
        float getYawRad();
        float getPitchDeg();
        float getRollDeg();
        float getYawDeg();

        void ZYX(float *z, float *y, float *x);
        void ZYZ(float *z, float *y, float *z2);
        void ZYZ_u(float *z, float *y, float *z2);

        float magnetic_reference_x() { return hx; }
        float magnetic_reference_y() { return hy; }

    public:
        static float invSqrt(float x);

        float beta;                // algorithm gain
        float sampleFreq;
        float invSampleFreq;

        linalg::vec<float,4> q = {0,0,0,1};

        float hx, hy;
    
        float q0=1;
        float q1=0;
        float q2=0;
        float q3=0;
    };
}

#endif