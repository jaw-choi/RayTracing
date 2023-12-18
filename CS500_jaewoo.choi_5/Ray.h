#pragma once

class Ray {
public:
    vec3 origin, direction;

    Ray(const vec3 _origin, const vec3 _direction)
        : origin(_origin), direction(_direction) {}

    vec3 Eval(float t)
    {
        return origin + t * direction;
    }
};