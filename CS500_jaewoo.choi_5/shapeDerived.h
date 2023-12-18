#pragma once

#include "acceleration.h"
#include "geom.h"
#include "Interval.h"
#include "realtime.h"
enum choiceType
{
    Phong = 0,
    Beckman,
    GGX
};
class Shape
{

public:
    Shape(MeshData* md, mat4 mTR, Material* mat)
        : meshdata(md), modelTR(mTR), material(mat) {};

    MeshData* meshdata;
    mat4 modelTR;
    Material* material;

    vec3 pos;
    vec3 diagonal;
    float area;
    ivec3 ivec;


    vec3 boxMax = vec3{ -1,-1,-1 };
    vec3 boxMin = vec3{ 1,1,1 };
    vec3 center = vec3{ 0,0,0 };

    float radius = 0.f;
    vec3 cylaxis{ 0.f, 0.f, 0.f };
    bool IsLight() { return material->isLight(); }
    virtual bool intersect(Ray ray, Intersection& intersec) { return false; }
    float Sign(float x);
    vec3 BeersLaw(vec3 omegaOut, vec3 Normal, const Intersection& i);
    vec3 SampleBrdf(vec3 omegaOut, vec3 Normal, float p_d, float p_r, float ior, choiceType type_);
    vec3 SampleLobe(vec3 Normal, float ran1, float ran2);
    float PdfBrdf(vec3 omegaOut, vec3 Normal, vec3 omegaIn, float p_d, float p_r, float p_t, float iorIn, float iorOut, float ior, choiceType type_);
    vec3 EvalScattering(vec3 omegaOut, vec3 Normal, vec3 omegaIn, const Intersection& i,float iorIn, float iorOut, float ior, choiceType type_);
    float CharacteristiceFactor(float d);
    vec3 Fresnel(float d,const Intersection& i);
    float Distribution(float dot_m_n, choiceType type_);
    float GFactor(vec3 omegaOut, vec3 omegaIn, vec3 m, vec3 normal, choiceType type_);
    float G1Factor(vec3 v, vec3 m, vec3 normal, choiceType type_);
    float GetIndexOfRefraction();
};

class Sphere : public Shape
{
public:
    Sphere(MeshData* md, mat4 mTR, Material* mat);
    vec3 boxMax = vec3{ -1,-1,-1 };
    vec3 boxMin = vec3{ 1,1,1 };
    virtual bool intersect(Ray ray, Intersection& intersec) ;



};

class Box : public Shape
{
public:
    Box(MeshData* md, mat4 mTR, Material* mat);
    vec3 boxMax = vec3{ -1,-1,-1 };
    vec3 boxMin = vec3{ 1,1,1 };
    virtual bool intersect(Ray ray, Intersection& intersec) ;

};

class Cylinder : public Shape
{
public:
    Cylinder(MeshData* md, mat4 mTR, Material* mat);
    vec3 boxMax = vec3{ -1,-1,-1 };
    vec3 boxMin = vec3{ 1,1,1 };
    virtual bool intersect(Ray ray, Intersection& intersec) ;

};

class Triangle : public Shape
{
public:
    Triangle(MeshData* md, mat4 mTR, Material* mat);
    vec3 boxMax = vec3{ -1,-1,-1 };
    vec3 boxMin = vec3{ 1,1,1 };
    virtual bool intersect(Ray ray, Intersection& intersec) ;

};
