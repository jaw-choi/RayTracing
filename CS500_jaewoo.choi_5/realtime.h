#pragma once
////////////////////////////////////////////////////////////////////////////////
// Temporary code.  Remove this from your raytracer.  This displays
// the contents of a scene file in realtime in a GLFW window.
////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <fstream>
#include <vector>

#define GLFW_INCLUDE_NONE
#include <glbinding/Binding.h>
#include <glbinding/gl/gl.h>
using namespace gl;

#include <GLFW/glfw3.h>
#include "geom.h"
#include "raytrace.h"
#include "Interval.h"
#include "shapeDerived.h"


////////////////////////////////////////////////////////////////////////
// Shader programming class;  Encapsulates an OpenGL Shader.
////////////////////////////////////////////////////////////////////////
class ShaderProgram
{
public:
    int program;
    
    void CreateProgram() { program = glCreateProgram(); }
    void Use() { glUseProgram(program); }
    void Unuse() { glUseProgram(0); }
    void CreateShader(const std::string fname, const GLenum type);
    void LinkProgram();

};

////////////////////////////////////////////////////////////////////////
// Obj: encapsulates objects to be drawn; uses OpenGL's VAOs
////////////////////////////////////////////////////////////////////////


class Obj
{
public:
    //bool IsIntersect(Ray ray, Intersection intersection);
    MeshData* meshdata;
    mat4 modelTR;
    Material* material;
    vec3 center;
    float area;
    unsigned int vao;



    Obj(MeshData* m, const mat4& tr, Material* b);
    vec3 Center() { return center; }
};

////////////////////////////////////////////////////////////////////////
// Realtime handles all realtime drawing/interaction
////////////////////////////////////////////////////////////////////////
class RayCast
{
public:

    bool nav;
    char motionkey;
    float speed;

    // Camera/viewing parameters
    vec3 ambient;
    vec3 eye;      // Position of eye for viewing scene
    quat orient;   // Represents rotation of -Z to view direction
    float ry;
    float front, back;
    float spin, tilt;
    float cDist;              // Distance from eye to center of scene
    //float lightSpin, lightTilt, lightDist;
    

    MeshData* sphMesh;
    MeshData* boxMesh;
    MeshData* cylMesh;
    std::vector<Shape*> triangles;


    int width, height;
    std::vector<Shape*> objs;
    std::vector<Shape*> lights;
    
    vec3 lightEmit[8];
    vec3 lightPosn[8];

    quat ViewQuaternion() {
        quat q = angleAxis((tilt-90.0f)*Radians, vec3(1,0,0))
            *conjugate(orient)
            *angleAxis(spin*Radians, vec3(0,0,1));
        return conjugate(q);
    }

    vec3 ViewDirection() {
        vec4 v = toMat4(ViewQuaternion()) * vec4(0.0f, 0.0f, -1.0f, 1.0f);
        return vec3(v[0], v[1], v[2]);
    }

    // The following methods are called from raytrace.cpp.  They will
    // need to be replaced when this realtime code is eliminated.
    
    RayCast();
    
    void setScreen(const int _width, const int _height) 
        { width = _width;  height = _height; }
    
    void setCamera(const vec3& _eye, const quat& _o, const float _ry)
        { eye=_eye; orient=_o; ry=_ry; }
    void setAmbient(const vec3& _a) { ambient = _a; }
    
    void sphere(const vec3 center, const float r, Material* mat);
    void box(const vec3 base, const vec3 diag, Material* mat);
    void cylinder(const vec3 base, const vec3 axis, const float radius, Material* mat);

    void triangleMesh(MeshData* meshdata, Material* mat_);
    

};


