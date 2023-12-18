////////////////////////////////////////////////////////////////////////////////
// Temporary code.  Remove this from your raytracer.  This displays
// the contents of a scene file in realtime in a GLFW window.
////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <fstream>
#include <vector>

#include "geom.h"
#include "raytrace.h"
#include "realtime.h"
#include "Interval.h"
#include "shapeDerived.h"


//#include <bvh/ray.hpp>

MeshData* SphMesh()
{
    MeshData* meshdata = new MeshData();
    unsigned int n = 20;
    float d = 2.0f*PI/float(n*2);
    for (unsigned int i=0;  i<=n*2;  i++) {
        float s = i*2.0f*PI/float(n*2);
        for (unsigned int j=0;  j<=n;  j++) {
            float t = j*PI/float(n);
            float x = cos(s)*sin(t);
            float y = sin(s)*sin(t);
            float z = cos(t);
            meshdata->vertices.push_back(VertexData(vec3(x,y,z),
                                                    vec3(x,y,z),
                                                    vec2(s/(2*PI), t/PI),
                                                    vec3(sin(s), cos(s), 0.0)));
            if (i>0 && j>0) {
                meshdata->triangles.push_back(ivec3((i-1)*(n+1) + (j-1), 
                                                      (i-1)*(n+1) + (j  ), 
                                                      (i  )*(n+1) + (j  )));
                meshdata->triangles.push_back(ivec3((i-1)*(n+1) + (j-1),
                                                      (i  )*(n+1) + (j  ),
                                                      (i  )*(n+1) + (j-1))); } } }
    return meshdata;
}

MeshData* BoxMesh()
{
    mat4 face[6] = {
        Identity(),
        rotate(180.0f*Radians, vec3(1.0f, 0.0f, 0.0f)),
        rotate( 90.0f*Radians, vec3(1.0f, 0.0f, 0.0f)),
        rotate(-90.0f*Radians, vec3(1.0f, 0.0f, 0.0f)),
        rotate( 90.0f*Radians, vec3(0.0f, 1.0f, 0.0f)),
        rotate(-90.0f*Radians, vec3(0.0f, 1.0f, 0.0f))};
       
    mat4 half = translate(vec3(0.5f, 0.5f, 0.5f))*scale(vec3(0.5f, 0.5f, 0.5f));
    MeshData* meshdata = new MeshData();
    for (unsigned int f=0;  f<6;  f++) {
        mat4 m4 = half*face[f];
        mat3 m3 = mat3(m4); // Extracts 3x3 from a 4x4
        for (unsigned int i=0;  i<2;  i++) {
            for (unsigned int j=0;  j<2;  j++) {
              vec4 p = m4*vec4(float(2*i)-1.0f, float(2*j)-1.0f, 1.0f, 1.0f);
              vec3 tnrm = m3*vec3(0.0f, 0.0f, 1.0f);
              vec3 ttan = m3*vec3(1.0, 0.0, 0.0);
              meshdata->vertices.push_back(VertexData(vec3(p[0], p[1], p[2]),
                                                      vec3(tnrm[0], tnrm[1], tnrm[2]),
                                                      vec2(float(i), float(j)),
                                                      vec3(ttan[0], ttan[1], ttan[2])));
              meshdata->triangles.push_back(ivec3(4*f+0, 4*f+1, 4*f+3));
              meshdata->triangles.push_back(ivec3(4*f+0, 4*f+3, 4*f+2)); } } }
    return meshdata;
}

MeshData* CylMesh()
{
    MeshData* meshdata = new MeshData();
    unsigned int n = 20;
    float d = 2.0f*PI/float(n*2);
    for (unsigned int i=0;  i<=n;  i++) {
        float s = i*2.0f*PI/float(n);
        float x = cos(s);
        float y = sin(s);
        
        meshdata->vertices.push_back(VertexData(vec3(x, y, 0.0f),
                                                vec3(x, y, 0.0f),
                                                vec2(s/(2*PI), 0.0f),
                                                vec3(-sin(s), cos(s), 0.0f)));

        meshdata->vertices.push_back(VertexData(vec3(x, y, 1.0f),
                                                vec3(x, y, 0.0f),
                                                vec2(s/(2*PI), 0.0f),
                                                vec3(-sin(s), cos(s), 0.0f)));

        if (i>0) {
            meshdata->triangles.push_back(ivec3((i-1)*2+1, (i-1)*2, (i  )*2));
            meshdata->triangles.push_back(ivec3((i-1)*2+1, (i  )*2, (i  )*2+1)); } }
    return meshdata;
}


////////////////////////////////////////////////////////////////////////
// Obj: encapsulates objects to be drawn; uses OpenGL's VAOs
////////////////////////////////////////////////////////////////////////
Obj::Obj(MeshData* m, const mat4& tr, Material* b)
    : meshdata(m), modelTR(tr), material(b)
{

}



RayCast::RayCast()
{   
    // Several generic meshes which can be transfofrmed to *any* sphere, box, or cylinder.
    sphMesh = SphMesh();
    boxMesh = BoxMesh();
    cylMesh = CylMesh();

    // Initialize various member attributes
    nav = true;
    spin = 0.0f;
    tilt = 90.0f;
    speed = 0.05f;
    front = 0.1f;
    back = 1000.0f;

    motionkey = 0;

    ambient = vec3(0.2, 0.2, 0.2); 
}


void RayCast::sphere(const vec3 center_, const float r, Material* mat)
{
    mat4 m = translate(center_) * scale(vec3(r,r,r));
    vec3 rrr(r,r,r);
    Shape* obj = new Sphere(sphMesh, m, mat);
    obj->center = center_;
    obj->radius = r;
    obj->area = 4*PI*r*r;

    obj->boxMin = center_ - rrr;
    obj->boxMax = center_ + rrr;

    objs.push_back(obj);
    if (mat->isLight())
        lights.push_back(obj);
}

void RayCast::box(const vec3 base, const vec3 diag, Material* mat)
{
    mat4 m = translate(base) * scale(vec3(diag[0],diag[1],diag[2]));
    Shape* obj = new Box(boxMesh, m, mat);

    obj->pos = base;
    obj->diagonal = diag;

    vec3 point1 = base;
    vec3 point2 = base + diag;


    obj->boxMin = vec3(std::min(point1.x, point2.x), std::min(point1.y, point2.y), std::min(point1.z, point2.z));
    obj->boxMax = vec3(std::max(point1.x, point2.x), std::max(point1.y, point2.y), std::max(point1.z, point2.z));
    objs.push_back(obj);
    if (mat->isLight())
        lights.push_back(obj);
}


void RayCast::cylinder(const vec3 base, const vec3 axis, const float radius, Material* mat)
{
    vec3 Z(0.0f, 0.0f, 1.0f);
    vec3 C = normalize(axis);
    vec3 B = cross(C,Z);
    if (length(B) <1e-8)
        B = vec3(0,1,0);
    else
        B = normalize(B);
    vec3 A = normalize(cross(B,C));

    mat4 R(A[0], A[1], A[2], 0.0f,
           B[0], B[1], B[2], 0.0f,
           C[0], C[1], C[2], 0.0f,
           0.0f, 0.0f, 0.0f, 1.0f);           

    mat4 m = translate(base)*R*scale(vec3(radius, radius, length(axis)));
    vec3 rrr(radius,radius,radius);
    Shape* obj = new Cylinder(cylMesh, m, mat);

    vec3 point1 = base + rrr;
    vec3 point2 = base - rrr;
    vec3 point3 = (base + axis) + rrr;
    vec3 point4 = (base + axis) - rrr;

    float xMin = std::min(std::min(point1.x, point2.x), std::min(point3.x, point4.x));
    float yMin = std::min(std::min(point1.y, point2.y), std::min(point3.y, point4.y));
    float zMin = std::min(std::min(point1.z, point2.z), std::min(point3.z, point4.z));
    float xMax = std::max(std::max(point1.x, point2.x), std::max(point3.x, point4.x));
    float yMax = std::max(std::max(point1.y, point2.y), std::max(point3.y, point4.y));
    float zMax = std::max(std::max(point1.z, point2.z), std::max(point3.z, point4.z));

    obj->boxMin = vec3(xMin, yMin, zMin);
    obj->boxMax = vec3(xMax, yMax, zMax);

    obj->cylaxis = axis;
    obj->center = base;

    objs.push_back(obj);
    if (mat->isLight())
        lights.push_back(obj);
}

void RayCast::triangleMesh(MeshData* meshdata, Material* mat_)
{
    Shape* obj = new Shape(meshdata, Identity(), mat_);
    //objs.push_back(obj);
    if (meshdata->mat->isLight())
        lights.push_back(obj);

    triangles.push_back(obj);
}

