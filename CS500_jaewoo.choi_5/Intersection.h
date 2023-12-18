#pragma once

class Shape;
class Intersection {
public:
    float t=1000.f;
    Shape* intersectedObj = nullptr;
    vec3 intersectPoint = vec3{ 0,0,0 };
    vec3 intersectNormal = vec3{ 0,0,0 };
    vec3 matKd = vec3{ 0,0,0 };
    float distance() const { return t; }  // A function the BVH traversal needs to be supplied.

};