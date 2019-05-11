#include "ralgo/madgwick.h"
#include <igris/math/fast_invsqrt.h>

madgwick::madgwick() {

}

void madgwick::setKoeff(float _sampleFreq, float _beta) {
    beta = _beta;
    sampleFreq = _sampleFreq;
    invSampleFreq = (1.0f / _sampleFreq);
}

void madgwick::reset() 
{
    q = linalg::identity;
}

/*void madgwick::readQuaternions(float *_q0, float *_q1, float *_q2, float *_q3) {
    *_q0 = q.w;
    *_q1 = q.x;
    *_q2 = q.y;
    *_q3 = q.z;
}*/

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void madgwick::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        update(gx, gy, gz, ax, ay, az);
        return;
    }

    // Convert gyroscope degrees/sec to radians/sec
    /*gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;*/

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q.x * gx - q.y * gy - q.z * gz);
    qDot2 = 0.5f * (q.w * gx + q.y * gz - q.z * gy);
    qDot3 = 0.5f * (q.w * gy - q.x * gz + q.z * gx);
    qDot4 = 0.5f * (q.w * gz + q.x * gy - q.y * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q.w * mx;
        _2q0my = 2.0f * q.w * my;
        _2q0mz = 2.0f * q.w * mz;
        _2q1mx = 2.0f * q.x * mx;
        _2q0 = 2.0f * q.w;
        _2q1 = 2.0f * q.x;
        _2q2 = 2.0f * q.y;
        _2q3 = 2.0f * q.z;
        _2q0q2 = 2.0f * q.w * q.y;
        _2q2q3 = 2.0f * q.y * q.z;
        q0q0 = q.w * q.w;
        q0q1 = q.w * q.x;
        q0q2 = q.w * q.y;
        q0q3 = q.w * q.z;
        q1q1 = q.x * q.x;
        q1q2 = q.x * q.y;
        q1q3 = q.x * q.z;
        q2q2 = q.y * q.y;
        q2q3 = q.y * q.z;
        q3q3 = q.z * q.z;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q.z + _2q0mz * q.y + mx * q1q1 + _2q1 * my * q.y + _2q1 * mz * q.z - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q.z + my * q0q0 - _2q0mz * q.x + _2q1mx * q.y - my * q1q1 + my * q2q2 + _2q2 * mz * q.z - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * q.y + _2q0my * q.x + mz * q0q0 + _2q1mx * q.z - mz * q1q1 + _2q2 * my * q.z - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q.y * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q.z + _2bz * q.x) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q.y * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q.x * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q.z * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q.y + _2bz * q.w) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q.z - _4bz * q.x) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q.y * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q.y - _2bz * q.w) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q.x + _2bz * q.z) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q.w - _4bz * q.y) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q.z + _2bz * q.x) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q.w + _2bz * q.y) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q.x * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q.w += qDot1 * invSampleFreq;
    q.x += qDot2 * invSampleFreq;
    q.y += qDot3 * invSampleFreq;
    q.z += qDot4 * invSampleFreq;

    // Normalise quaternion
    recipNorm = invSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;
    //anglesComputed = 0;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void madgwick::update(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Convert gyroscope degrees/sec to radians/sec
    /*gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;*/

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q.x * gx - q.y * gy - q.z * gz);
    qDot2 = 0.5f * (q.w * gx + q.y * gz - q.z * gy);
    qDot3 = 0.5f * (q.w * gy - q.x * gz + q.z * gx);
    qDot4 = 0.5f * (q.w * gz + q.x * gy - q.y * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q.w;
        _2q1 = 2.0f * q.x;
        _2q2 = 2.0f * q.y;
        _2q3 = 2.0f * q.z;
        _4q0 = 4.0f * q.w;
        _4q1 = 4.0f * q.x;
        _4q2 = 4.0f * q.y;
        _8q1 = 8.0f * q.x;
        _8q2 = 8.0f * q.y;
        q0q0 = q.w * q.w;
        q1q1 = q.x * q.x;
        q2q2 = q.y * q.y;
        q3q3 = q.z * q.z;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q.x - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q.y + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q.z - _2q1 * ax + 4.0f * q2q2 * q.z - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q.w += qDot1 * invSampleFreq;
    q.x += qDot2 * invSampleFreq;
    q.y += qDot3 * invSampleFreq;
    q.z += qDot4 * invSampleFreq;

    // Normalise quaternion
    recipNorm = invSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;
}

void madgwick::update(float gx, float gy, float gz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q.x * gx - q.y * gy - q.z * gz);
    qDot2 = 0.5f * (q.w * gx + q.y * gz - q.z * gy);
    qDot3 = 0.5f * (q.w * gy - q.x * gz + q.z * gx);
    qDot4 = 0.5f * (q.w * gz + q.x * gy - q.y * gx);

    // Integrate rate of change of quaternion to yield quaternion
    q.w += qDot1 * invSampleFreq;
    q.x += qDot2 * invSampleFreq;
    q.y += qDot3 * invSampleFreq;
    q.z += qDot4 * invSampleFreq;

    // Normalise quaternion
    recipNorm = invSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float madgwick::invSqrt(float x) 
{
    return quick_invsqrt(x);
}

float madgwick::getYawRad() {
    return atan2(2 * q.x * q.y - 2 * q.w * q.z, 2 * q.w * q.w + 2 * q.x * q.x - 1);
}

float madgwick::getPitchRad() {
    return atan2(2 * q.y * q.z - 2 * q.w * q.x, 2 * q.w * q.w + 2 * q.z * q.z - 1);
}

float madgwick::getRollRad() {
    return -1 * atan2(2.0f * (q.w * q.y - q.x * q.z), 1.0f - 2.0f * (q.y * q.y + q.x *q.x ));
}

float madgwick::getYawDeg() {
    return getYawRad() * RAD_TO_DEG;
}

float madgwick::getPitchDeg() {
    return getPitchRad() * RAD_TO_DEG;
}

float madgwick::getRollDeg() {
    return getRollRad() * RAD_TO_DEG;
}

#include <gxx/panic.h> 

void madgwick::ZYX(float *z, float *y, float *x) {
    *z = atan2(q.x*q.y+q.z*q.w, 0.5 - q.y*q.y-q.z*q.z);
    *y = - asin(2*(q.x*q.z - q.y*q.w));
    *x = atan2(q.y*q.z+q.x*q.w, 0.5-q.x*q.x-q.y*q.y);
}


void madgwick::ZYZ(float *z, float *y, float *z2) {
    *z = atan2(q.y*q.z-q.x*q.w, q.x*q.z+q.y*q.w);
    *y = acos(1-2*(q.x*q.x + q.y*q.y));
    *z2 = -atan2(q.y*q.z+q.x*q.w, q.x*q.z-q.y*q.w);
}

void madgwick::ZYZ_u(float *z, float *y, float *z2) {
    *z = atan2(q.x*q.y+q.z*q.w, 0.5 - (q.y*q.y+q.z*q.z));
    *y = acos(1-2*(q.x*q.x + q.y*q.y));
    *z2 = -atan2(q.y*q.z+q.x*q.w, q.x*q.z-q.y*q.w);
}