// 3 x 1 Vector + Quaternion Utility Library for BMA400 Accelerometer Data

#ifndef QUAT_VEC3_H
#define QUAT_VEC3_H

#include <math.h>
#include <stdint.h>

static constexpr float PI_F = 3.14159265358979323846f;

// ====== VEC3 ======

struct Vec3{
    float x, y, z;
    // Vector addition
    Vec3 operator+(const Vec3 &b) const{
        return {x + b.x, y + b.y, z + b.z};
    }
    // Vector subtraction
    Vec3 operator-(const Vec3 &b) const{
        return {x - b.x, y - b.y, z - b.z};
    }
    // Scalar multiplication
    Vec3 operator*(float s) const{
        return {x * s, y * s, z * s};
    }
    // Scalar division
    Vec3 operator/(float s) const{
        return {x / s, y / s, z / s};
    }
};

// 3D DOT PRODUCT
static inline float dot(const Vec3 &a, const Vec3 &b){ // a dot b
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

// 3D CROSS PRODUCT
static inline Vec3 cross(const Vec3 &a, const Vec3 &b){ // a x b
    return {
        (a.y * b.z) - (a.z * b.y),
        (a.z * b.x) - (a.x * b.z),
        (a.x * b.y) - (a.y * b.x)};
}

// SQUARED NORM
static inline float norm2(const Vec3 &v){
    return dot(v, v);
}

// NORM (LENGTH, MAGNITUDE)
static inline float norm(const Vec3 &v){
    return sqrtf(norm2(v));
}

// NORMALIZE VECTOR
static inline Vec3 normalize(const Vec3 &v, float eps = 1e-6f){ // NO division by zero
    float n = norm(v);
    if (n < eps)
        return {0.0f, 0.0f, 0.0f}; // return zero vector if norm too small
    return v / n;
}

// CLAMP (FLOAT VALUE)
static inline float clampf(float x, float low, float high){
    return (x < low) ? low : ((x > high) ? high : x);
}

// === QUATERNION ===

// Unit quaternion q = (w, x, y, z) represents a rotation around the axis (x,y,z) by an angle theta, where:
// w = cos(theta/2), (x,y,z) = sin(theta/2) * (axis_x, axis_y, axis_z)

// QUATERNION STRUCTURE + OPERATIONS
struct Quat{
    float w, x, y, z; // (w, xyz)

    // Identity Quaternion
    static inline Quat identity(){
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }

    // Quaternion addition
    Quat operator+(const Quat &b) const{ // allow usage with const objects
        return {w + b.w, x + b.x, y + b.y, z + b.z};
    }

    // Quaternion multiplication
    Quat operator*(float s) const{
        return {w * s, x * s, y * s, z * s};
    }
};

// QUATERNION NORM SQUARED
static inline float quatNorm2(const Quat& q) {
    return (q.w * q.w) + (q.x * q.x) + (q.y * q.y) + (q.z * q.z);
}

// NORMALIZE QUATERNION
static inline Quat quatNormalize(const Quat& q, float eps = 1e-6f){
    float n2 = quatNorm2(q);
    
    if (n2 < eps * eps) return Quat::identity();
    float inv = 1.0f / sqrtf(n2);
    
    return {q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

// QUATERNION CONJUGATE
// q_conj = (w, -x, -y, -z), inverse rotation for unit quaternions
static inline Quat quatConj(const Quat& q){
    return {q.w, -q.x, -q.y, -q.z};
}

// QUATERNION MULTIPLICATION
static inline Quat quatMul(const Quat& a, const Quat& b){
    // Hamilton product, non-commutative: q = a * b applies b's rotation, then a's rotation
    return { 
        (a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z),
        (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y),
        (a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x),
        (a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w)
    };
}

// ROTATE VECTOR BY UNIT QUATERNION
static inline Vec3 rotateVec(const Quat& q_unit, const Vec3& v){
    // v' = q * (0,v) * q_conj
    // Treat vector as quaternion with w = 0, then apply rotation

    Quat p{0.0f, v.x, v.y, v.z};
    Quat q = quatNormalize(q_unit);
    Quat qc = quatConj(q);
    Quat r = quatMul(quatMul(q, p), qc);
    return {r.x, r.y, r.z};
}

// QUATERNION FROM TWO UNIT VECTORS
static inline Quat quatFrom2UnitVectors(const Vec3& u_unit, const Vec3& v_unit)
{
    const float eps = 1e-6f;

    float d = clampf(dot(u_unit, v_unit), -1.0f, 1.0f);

    // Nearly identical (if vectors nearly the same, return identity quaternion (no rotation))
    if (d > 1.0f - eps) {
        return Quat::identity();
    }

    // Nearly opposite (if vectors nearly opposite, return 180° rotation around any orthogonal axis)
    if (d < -1.0f + eps) {
        float ax = fabsf(u_unit.x);
        float ay = fabsf(u_unit.y);
        float az = fabsf(u_unit.z);

        // Pick axis from u_unit that's orthogonal to v_unit (cross product ~0 if parallel)
        Vec3 ref;
        if (ax <= ay && ax <= az)      ref = {1.0f, 0.0f, 0.0f};
        else if (ay <= ax && ay <= az) ref = {0.0f, 1.0f, 0.0f};
        else                           ref = {0.0f, 0.0f, 1.0f};

        Vec3 axis = normalize(cross(u_unit, ref));
        // 180° rotation: w = 0, xyz = axis
        return {0.0f, axis.x, axis.y, axis.z};
    }

    // General case
    // Use half-angle formula: q = (1 + dot(u,v), cross(u,v))
    Vec3 c = cross(u_unit, v_unit);
    Quat q = {1.0f + d, c.x, c.y, c.z};
    return quatNormalize(q);
}

// ANGLE CONVERSIONS
// Radians --> Degrees
static inline float rad2deg(float r){
    return r * (180.0f / (float)PI_F);
}

static inline float deg2rad(float d){
    return d * ((float)PI_F / 180.0f);
}

// Quaternion Angle --> Radians
static inline float quatAngleRad(const Quat& q_unit){
    Quat q = quatNormalize(q_unit);
    float w = clampf(q.w, -1.0f, 1.0f);
    w = fabsf(w); // q and -q represent the same rotation
    return 2.0f * acosf(w);
}

// Quaternion Angle in Degrees
static inline float quatAngleDeg(const Quat& q_unit){
  return rad2deg(quatAngleRad(q_unit));
}

// EXTRACT ROLL + PITCH IN DEGREES FROM UNIT QUATERNION
static inline void quatToRollPitchDeg(const Quat& q_unit, float* roll_deg, float* pitch_deg){

    Quat q = quatNormalize(q_unit);
    float w = q.w, x = q.x, y = q.y, z = q.z;

    // roll (x-axis rotation)
    float sinr_cosp = 2.0f*((w*x) + (y*z));
    float cosr_cosp = 1.0f - 2.0f*((x*x) + (y*y));
    float roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2.0f*((w*y) - (z*x));
    sinp = clampf(sinp, -1.0f, 1.0f);
    float pitch = asinf(sinp);

    *roll_deg  = rad2deg(roll);
    *pitch_deg = rad2deg(pitch);
}

#endif // QUAT_VEC3_H