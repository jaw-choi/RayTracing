//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <vector>

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
#else
    // Includes for Linux
#endif

#include "geom.h"
#include "raytrace.h"
#include "realtime.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_FAILURE_USERMSG
#include "stb_image.h"


#include "Ray.h"
#include "acceleration.h"
#include "shapeDerived.h"
#include "rgbe.h"
#include "HDRLoader.h"

// A good quality *thread-safe* Mersenne Twister random number generator.
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].

Scene::Scene() 
{ 
    raycast_ = new RayCast(); 
}

void Scene::Finit()
{

    for (int i = 0; i < raycast_->triangles.size(); ++i)
    {
        Shape* obj = raycast_->triangles[i];

        size_t meshTriangleCount = obj->meshdata->triangles.size();

        for (size_t i = 0; i < meshTriangleCount; ++i)
        {
            ivec3 i0 = obj->meshdata->triangles[i];
            vec3 v0 = obj->meshdata->vertices[i0.x].pnt;
            vec3 v1 = obj->meshdata->vertices[i0.y].pnt;
            vec3 v2 = obj->meshdata->vertices[i0.z].pnt;
            vec3 center = (v0 + v1 + v2) / 3.f;

            float xMin = std::min(std::min(v0.x, v1.x), std::min(v1.x, v2.x));
            float yMin = std::min(std::min(v0.y, v1.y), std::min(v1.y, v2.y));
            float zMin = std::min(std::min(v0.z, v1.z), std::min(v1.z, v2.z));

            float xMax = std::max(std::max(v0.x, v1.x), std::max(v1.x, v2.x));
            float yMax = std::max(std::max(v0.y, v1.y), std::max(v1.y, v2.y));
            float zMax = std::max(std::max(v0.z, v1.z), std::max(v1.z, v2.z));

            vec3 min(xMin, yMin, zMin);
            vec3 max(xMax, yMax, zMax);
            //vec3 min = glm::min(glm::min(v0, v1), glm::min(v1, v2));
            //vec3 max = glm::max(glm::max(v0, v1), glm::max(v1, v2));


            Shape* triangleObj = new Triangle(obj->meshdata, Identity(), obj->material);

            triangleObj->center = center;
            triangleObj->boxMin = min;
            triangleObj->boxMax = max;
            triangleObj->ivec = i0;

            shapes.push_back(triangleObj);
        }
    }

    for (int i = 0; i < raycast_->objs.size(); ++i)
    {
        shapes.push_back(raycast_->objs[i]);
    }

    acceleration = new AccelerationBvh(shapes);
}

void Scene::triangleMesh(MeshData* mesh) 
{ 
    raycast_->triangleMesh(mesh,currentMat); 
}

Texture::Texture(const std::string &bpath) : id(0)
{
    // Replace backslashes with forward slashes -- Good for Linux, and maybe Windows?
    std::string path=bpath;
    std::string bs="\\";
    std::string fs="/";
    while (path.find(bs) != std::string::npos) {
        path.replace(path.find(bs), 1, fs);}
    
    // Does the file exist?
    std::ifstream find_it(path.c_str());
    if (find_it.fail()) {
        std::cerr << "Texture file not found: "  << path << std::endl;
        exit(-1); }
    else {
        // Read image, and check for success
        stbi_set_flip_vertically_on_load(true);
        image = stbi_load(path.c_str(), &width, &height, &depth, 4);
        printf("%d %d %d %s\n", depth, width, height, path.c_str());
        if (!image) {
            printf("\nRead error on file %s:\n  %s\n\n", path.c_str(), stbi_failure_reason());
            exit(-1); } }
}

quat Orientation(int i, 
                        const std::vector<std::string>& strings,
                        const std::vector<float>& f)
{
    quat q(1,0,0,0); // Unit quaternion
    while (i<strings.size()) {
        std::string c = strings[i++];
        if (c == "x")  
            q *= angleAxis(f[i++]*Radians, Xaxis());
        else if (c == "y")  
            q *= angleAxis(f[i++]*Radians, Yaxis());
        else if (c == "z")  
            q *= angleAxis(f[i++]*Radians, Zaxis());
        else if (c == "q")  {
            q *= quat(f[i+0], f[i+1], f[i+2], f[i+3]);
            i+=4; }
        else if (c == "a")  {
            q *= angleAxis(f[i+0]*Radians, normalize(vec3(f[i+1], f[i+2], f[i+3])));
            i+=4; } }
    return q;
}

void Scene::Command(const std::vector<std::string>& strings,
                    const std::vector<float>& f)
{
    if (strings.size() == 0) return;
    std::string c = strings[0];
    
    if (c == "screen") {
        // syntax: screen width height
        raycast_->setScreen(int(f[1]),int(f[2]));
        width = int(f[1]);
        height = int(f[2]); }

    else if (c == "camera") {
        // syntax: camera x y z   ry   <orientation spec>
        // Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
        raycast_->setCamera(vec3(f[1],f[2],f[3]), Orientation(5,strings,f), f[4]); }

    else if (c == "ambient") {
        // syntax: ambient r g b
        // Sets the ambient color.  Note: This parameter is temporary.
        // It will be ignored once your raytracer becomes capable of
        // accurately *calculating* the true ambient light.
        raycast_->setAmbient(vec3(f[1], f[2], f[3])); }

    else if (c == "brdf")  {
        // syntax: brdf  r g b   r g b  alpha
        // later:  brdf  r g b   r g b  alpha  r g b ior
        // First rgb is Diffuse reflection, second is specular reflection.
        // third is beer's law transmission followed by index of refraction.
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Material(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], vec3(f[8], f[9], f[10]), f[11]);
    }

    else if (c == "light") {
        // syntax: light  r g b   
        // The rgb is the emission of the light
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Light(vec3(f[1], f[2], f[3])); }
   
    else if (c == "sphere") {
        // syntax: sphere x y z   r
        // Creates a Shape instance for a sphere defined by a center and radius
        raycast_->sphere(vec3(f[1], f[2], f[3]), f[4], currentMat); }

    else if (c == "box") {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector
        raycast_->box(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), currentMat); }

    else if (c == "cylinder") {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        raycast_->cylinder(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], currentMat); }


    else if (c == "mesh") {
        // syntax: mesh   filename   tx ty tz   s   <orientation>
        // Creates many Shape instances (one per triangle) by reading
        // model(s) from filename. All triangles are rotated by a
        // quaternion (qw qx qy qz), uniformly scaled by s, and
        // translated by (tx ty tz) .
        mat4 modelTr = translate(vec3(f[2],f[3],f[4]))
                          *scale(vec3(f[5],f[5],f[5]))
                          *toMat4(Orientation(6,strings,f));
        ReadAssimpFile(strings[1], modelTr);  }

    
    else {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}

void Scene::TraceImage(Color* image, const int pass, std::string hdr)
{

    //realtime->run();                          // Remove this (realtime stuff)
    //TODO
    hdrLoader = new HDRLoader();
    hdrLoaderResult = new HDRLoaderResult();
    hdrLoader->load("BackYard.hdr", *hdrLoaderResult);
    float radius = 0.3f;
    glm::vec3 pos = glm::vec3(0.5f, 1.1f, 0.7f);
    float rx = raycast_->ry * ((float)width / (float)height);
    vec3 X = rx * transformVector(raycast_->ViewQuaternion(), Xaxis());
    vec3 Y = raycast_->ry * transformVector(raycast_->ViewQuaternion(), Yaxis());
    vec3 Z = transformVector(raycast_->ViewQuaternion(), Zaxis());

    vec3 origin = raycast_->eye;
    float Distance = glm::distance(raycast_->eye, pos);
    for (int i{ 0 }; i < pass; i++)
    {
        //if(i%100==0)
        std::cout << "Pass: " << (i + 1) << "/" << pass << std::endl;
#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
        for (int y = 0; y < height; y++) {

            //fprintf(stderr, "Rendering %4d\r", y);
            for (int x = 0; x < width; x++) {

                float r = radius * sqrt(myrandom(RNGen));
                float theta = 2.f * PI * r * myrandom(RNGen);
                rx = r * cos(theta);
                float ry = r * sin(theta);
                float dx = (2 * (static_cast<float>(x) + myrandom(RNGen)) / static_cast<float>(width)) - 1;
                float dy = (2 * (static_cast<float>(y) + myrandom(RNGen)) / static_cast<float>(height)) - 1;
                //vec3 dir = normalize(dx * X + dy * Y - Z);
                vec3 dir = normalize((dx * Distance - rx) * X + (dy * Distance - ry) * Y - Distance * Z);
                Ray ray(origin + rx * X + ry*Y , dir);
                Color C = acceleration->TracePath(ray, raycast_, *hdrLoaderResult);

                if (!isnan(C.x) && !isinf(C.x) &&
                    !isnan(C.y) && !isinf(C.y) &&
                    !isnan(C.z) && !isinf(C.z))
                    image[y * width + x] += C;

            }
        }

        WriteHDRImage(image, i, hdr);
    }



    fprintf(stderr, "\n");
}

void Scene::WriteHDRImage(Color* image, int currentPass, std::string hdr)
{
    // Turn image from a 2D-bottom-up array of Vector3D to an top-down-array of floats
    float* data = new float[width * height * 3];
    float* dp = data;
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            Color pixel = image[y * width + x] / (currentPass/3.f);

            *dp++ = pixel[0];
            *dp++ = pixel[1];
            *dp++ = pixel[2];
        }
    }

    // Write image to file in HDR (a.k.a RADIANCE) format
    rgbe_header_info info;
    char errbuf[100] = { 0 };

    FILE* fp = fopen(hdr.c_str(), "wb");
    info.valid = false;
    int r = RGBE_WriteHeader(fp, width, height, &info, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);

    r = RGBE_WritePixels_RLE(fp, data, width, height, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);
    fclose(fp);

    delete data;
}