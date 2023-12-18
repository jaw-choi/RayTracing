#pragma once

#include "geom.h"
#include "Ray.h"
#include "Intersection.h"

const float INF = 1000.f;

struct Slab
{
    float d0, d1;
    vec3 N;//(a,b,c)
};
class Interval
{
public:
    float t0, t1;
    vec3 N0, N1;
public:
    Interval();
    Interval(float t0_, float t1_);
    Interval(float t0_, float t1_, vec3 N0_, vec3 N1_);

    bool Intersect(Ray ray, Slab slab);
};

inline Interval::Interval() { t0 = 0; t1 = INF; }

inline Interval::Interval(float t0_, float t1_, vec3 N0_, vec3 N1_)
{
    if (t0_ <= t1) {
        t0 = t0_; t1 = t1_; N0 = N0_; N1 = N1_;
    }
    else {
        t0 = 0; t1 = -1;
    }
}

inline Interval::Interval(float _t0, float _t1)
{
    t0 = _t0;
    t1 = _t1;

    N0 = vec3(0);
    N1 = vec3(0);
}
inline bool Interval::Intersect(Ray ray, Slab slab)
{
    float result = dot(slab.N, ray.direction);
    if (result > 0.001f || result < -0.001f)//Ray intersects both slabe planes // result != 0 (float)
    {
        //plane 0
        t0 = -(slab.d0 + dot(slab.N, ray.origin)) / dot(slab.N, ray.direction);
        //plane 1
        t1 = -(slab.d1 + dot(slab.N, ray.origin)) / dot(slab.N, ray.direction);

        if (t0 > t1)
            std::swap(t0, t1);

        if (slab.d0 > slab.d1) {
            N0 = -slab.N;
            N1 = slab.N;
        }
        else {
            N0 = slab.N;
            N1 = -slab.N;
        }

        return true;
    }

    //else
      //ray is parallel to slab planes

    float s0 = dot(slab.N, ray.origin) + slab.d0;
    float s1 = dot(slab.N, ray.origin) + slab.d1;
    if ((s0 < 0 && s1 < 0) || (s0 > 0 && s1 > 0))//signs is same
    {
        t0 = 1.f; t1 = 0.f;
        return false;

    }

        t0 = 0; t1 = INF;
    
    return true;


}
