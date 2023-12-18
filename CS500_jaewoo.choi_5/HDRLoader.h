#pragma once
#include "geom.h"

class HDRLoaderResult {
public:
    int width, height;
    // each pixel takes 3 float32, each component can be of any value...
    float* cols;
    float angle;
    float* pBuffer;
    float* pUDist;
    float* image;
    float radius = 1000.f;
};

class HDRLoader {
public:
    static bool load(const char* fileName, HDRLoaderResult& res);
};