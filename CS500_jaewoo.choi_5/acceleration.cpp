
#include <vector>
#include "geom.h"
#include "raytrace.h"
#include "acceleration.h"

#include <bvh/sweep_sah_builder.hpp>
#include <bvh/single_ray_traverser.hpp>
#include <bvh/primitive_intersectors.hpp>
#include "realtime.h"
#include "shapeDerived.h"
#include "HDRLoader.h"

//#include "Interval.h"
/////////////////////////////
// Vector and ray conversions

// Pre-processing step: Marginal and conditional CDF



Ray RayFromBvh(const bvh::Ray<float> &r)
{
    return Ray(vec3FromBvh(r.origin), vec3FromBvh(r.direction));
}
bvh::Ray<float> RayToBvh(const Ray &r)
{
    return bvh::Ray<float>(vec3ToBvh(r.origin), vec3ToBvh(r.direction));
}


/////////////////////////////
// SimpleBox
bvh::Vector3<float> vec3ToBvh(const vec3& v)
{
    return bvh::Vector3<float>(v[0],v[1],v[2]);
}

vec3 vec3FromBvh(const bvh::Vector3<float>& v)
{
    return vec3(v[0],v[1],v[2]);
}

SimpleBox::SimpleBox(): bvh::BoundingBox<float>() {}
SimpleBox::SimpleBox(const vec3 v): bvh::BoundingBox<float>(vec3ToBvh(v)) {}

SimpleBox& SimpleBox::extend(const vec3 v)
{
    bvh::BoundingBox<float>::extend(vec3ToBvh(v));
    return *this;
}


/////////////////////////////
// BvhShape

BvhShape::BvhShape(Shape* s) : shape(s)
{

    BoxBounding = new SimpleBox();
    BoxBounding->min = vec3ToBvh(s->boxMin);
    BoxBounding->max = vec3ToBvh(s->boxMax);
}

SimpleBox BvhShape::bounding_box() const
{
    //  Return the shape's bounding box.
    return *BoxBounding; // FIX THIS-done
}

bvh::Vector3<float> BvhShape::center() const
{
    return bounding_box().center();
}
    
std::optional<Intersection> BvhShape::intersect(const bvh::Ray<float>& bvhray) const
{
    
    Intersection result;
    
    if (shape->intersect(RayFromBvh(bvhray), result) == false)
        return std::nullopt;
     if(result.t < bvhray.tmin || result.t > bvhray.tmax)
        return std::nullopt;
     
     result.matKd = shape->material->Kd;
     result.intersectedObj = shape;
    return result;  // FIX THIS 
}

AccelerationBvh::AccelerationBvh(std::vector<Shape*> &objs)
{
    raycast_ = new RayCast();
    // Wrap all Shape*'s with a bvh specific instance
    for (Shape* shape:  objs) {
        shapeVector.emplace_back(shape); }

    // Magic found in the bvh examples:
    auto [bboxes, centers] = bvh::compute_bounding_boxes_and_centers(shapeVector.data(),
                                                                     shapeVector.size());
    auto global_bbox = bvh::compute_bounding_boxes_union(bboxes.get(), shapeVector.size());

    bvh::SweepSahBuilder<bvh::Bvh<float>> builder(bvh);
    builder.build(global_bbox, bboxes.get(), centers.get(), shapeVector.size());
}

float GeometryFactor(const Intersection& a, const Intersection& b)
{
    vec3 difference = normalize(a.intersectPoint - b.intersectPoint);
    return abs((dot(a.intersectNormal, difference) * dot(b.intersectNormal, difference) / pow(dot(difference, difference), 2)));
}

float AccelerationBvh::PdfLight(const Intersection& L, RayCast* raycast)
{
    
    float AreaOfLightSphere = 1000.f;
    Sphere* light = dynamic_cast<Sphere*>(L.intersectedObj);
    if (light != nullptr)
    {
        AreaOfLightSphere = 4 * PI * light->radius * light->radius;
    }
    
    if (raycast->lights.size() == 0)
    {
        return 1.f / AreaOfLightSphere;
    }

    return 1.f / (raycast->lights.size() * AreaOfLightSphere);

}

Intersection SampleSphere(vec3 center, float radius)
{
    float x1 = myrandom(RNGen);
    float x2 = myrandom(RNGen);
    float z = 2 * x1 - 1;
    float r = static_cast<float>(sqrt(1 - pow(z, 2)));
    float a = 2 * PI * x2;

    Intersection res;
    res.intersectNormal = normalize(vec3(r * cos(a), r * sin(a), z));
    res.intersectPoint = center + (radius * res.intersectNormal);

    return res;
}

Intersection AccelerationBvh::SampleLight(RayCast* raycast)
{
    //choose randomly
        //int lightIndex = static_cast<int>(myrandom(RNGen) * (lights.size() - 1));
        //Sphere* light = dynamic_cast<Sphere*>(lights[lightIndex]);

    Sphere* light;
    if (raycast->lights.size() != 0)
        light = dynamic_cast<Sphere*>(raycast->lights[0]);
    Intersection res;
    if (light != nullptr)
    {
        res = SampleSphere(light->center, light->radius);
        res.intersectedObj = light;
    }

    return res;
}


Intersection AccelerationBvh::SampleAsLight(RayCast* raycast,HDRLoaderResult* ibl)
{
    Intersection B;
    Sphere* light;
    if (raycast->lights.size() != 0)
        light = dynamic_cast<Sphere*>(raycast->lights[0]);
    float u = myrandom(RNGen);
    float v = myrandom(RNGen);
    float maxUVal = ibl->pUDist[ibl->width - 1];
    float* pUPos = std::lower_bound(ibl->pUDist, ibl->pUDist + ibl->width,
        u * maxUVal);

    int iu = pUPos - ibl->pUDist;
    float* pVDist = &ibl->pBuffer[ibl->height * iu];
    float* pVPos = std::lower_bound(pVDist, pVDist + ibl->height,
        v * pVDist[ibl->height - 1]);
    int iv = pVPos - pVDist;
    double phi = ibl->angle - 2.f * PI * iu / ibl->width;
    double theta = PI * iv / ibl->height;
    B.intersectNormal = vec3(sin(theta) * cos(phi),
        sin(theta) * sin(phi),
        cos(theta));
    B.intersectPoint = B.intersectNormal * ibl->radius;
    B.intersectedObj = light;
    return B;
}
float AccelerationBvh::PdfAsLight(const Intersection& B, HDRLoaderResult* ibl) const
{
    vec3 P = normalize(B.intersectPoint);
    double fu = (ibl->angle - atan2(P[1], P[0])) / (2.f * PI);
    fu = fu - floor(fu); // Wrap to be within 0...1
    int u = floor(ibl->width * fu);
    int v = floor(ibl->height * acos(P[2]) / PI);
    float angleFrac = PI / float(ibl->height);
    float* pVDist = &ibl->pBuffer[ibl->height * u];
    float pdfU = (u == 0) ? (ibl->pUDist[0]) : (ibl->pUDist[u] - ibl->pUDist[u - 1]);
    pdfU /= ibl->pUDist[ibl->width - 1];
    pdfU *= ibl->width / (2.f * PI);
    float pdfV = (v == 0) ? (pVDist[0]) : (pVDist[v] - pVDist[v - 1]);
    pdfV /= pVDist[ibl->height - 1];
    pdfV *= ibl->height / PI;
    float theta = angleFrac * 0.5 + angleFrac * v;
    float pdf = pdfU * pdfV * sin(theta) / (4.0 * PI * ibl->radius * ibl->radius);
    //printf("(%f %f %f) %d %d %g\n", P[0], P[1], P[2], u, v, pdf);
    return pdf;
}
Color AccelerationBvh::Radiance(const Intersection& A, HDRLoaderResult* ibl)
{
    vec3 P = normalize(A.intersectPoint);
    double u = (ibl->angle - atan2(P[1], P[0])) / (2.f * PI);
    u = u - floor(u); // Wrap to be within 0...1
    double v = acos(P[2]) / PI;
    int i0 = floor(u * ibl->width);
    int j0 = floor(v * ibl->height);
    float uw[2], vw[2];
    uw[1] = u * ibl->width - i0; uw[0] = 1.0 - uw[1];
    vw[1] = v * ibl->height - j0; vw[0] = 1.0 - vw[1];
    Color r(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            int k = 3 * (((j0 + j) % ibl->height) * ibl->width + ((i0 + i) % ibl->width));
            for (int c = 0; c < 3; c++) {
                r[c] += uw[i] * vw[j] * ibl->cols[k + c];
            }
        }
    }
    if (r.x > 1.0f)
        r.x = 1.f;
    if (r.y > 1.0f)
        r.y = 1.f;
    if (r.z > 1.0f)
        r.z = 1.f;
    return r;
}
Color EvalRadiance(Light* L)
{
    return L->Kd;
}

vec3 EvalScattering(vec3 omegaOut, vec3 normal, vec3 omega, const Intersection& i)
{
    return abs(dot(normal, omega)) * i.intersectedObj->material->Kd / PI;

}

vec3 AccelerationBvh::ExplicitLightConnection(vec3 omegaOut,const Intersection& previousIntersection, RayCast* raycast, vec3 normal, vec3 weight,float q)
{
    const float epsilon = 0.001f;
    Intersection L = SampleLight(raycast);
    float p = PdfLight(L,raycast) / GeometryFactor(previousIntersection, L);

    vec3 omegaIn = normalize(L.intersectPoint - previousIntersection.intersectPoint);
    float w_mis = (p * p) / (p * p + q * q);

    Ray I(previousIntersection.intersectPoint, omegaIn);
    Intersection interFromPToL = intersect(I);

    if (interFromPToL.intersectedObj != nullptr && 
        p > epsilon &&
        L.intersectPoint == interFromPToL.intersectPoint)
    {
        vec3 f = EvalScattering(omegaOut,normal, omegaIn, previousIntersection);
        return (0.5f * weight * (f / p) * w_mis * L.intersectedObj->material->Kd);
    }
    return vec3(0.f);
}

float AccelerationBvh::CalculateIndexOfRefraction(float WoDotNormal, const Intersection& intersection, float& iorIn, float& iorOut)
{
    if (WoDotNormal > 0.0001f)
    {
        iorIn = 1.f;
        iorOut = intersection.intersectedObj->GetIndexOfRefraction();
        return iorIn / iorOut;
    }
    else if(WoDotNormal < 0.0001f)
    {
        iorIn = intersection.intersectedObj->GetIndexOfRefraction();
        iorOut = 1.f;

        return iorIn / iorOut;
    }
}



Color AccelerationBvh::TracePath(Ray ray,RayCast* raycast, HDRLoaderResult ibl)
{
    Color C(0.f);
    Color W(1.f);
    Intersection P = intersect(ray);
    vec3 N = P.intersectNormal;
    vec3 omegaOut(0.f);


    if (P.intersectedObj == nullptr)
        return C;

    //if (P.intersectedObj->material->isLight())
    //    return EvalRadiance(dynamic_cast<Light*>(P.intersectedObj->material));
    if (P.intersectedObj->material->isLight())
        return Radiance(P,&ibl);

    omegaOut = -ray.direction;
    float iorIn = 0.f;
    float iorOut = 0.f;
    float ior = CalculateIndexOfRefraction(dot(omegaOut, N), P, iorIn, iorOut);
    while (myrandom(RNGen) <= RussianRoulette)
    {
        Intersection L = SampleAsLight(raycast,&ibl);
        float s = length(P.intersectedObj->material->Kd) + length(P.intersectedObj->material->Ks) + length(P.intersectedObj->material->Kt);
        float p_d = length(P.intersectedObj->material->Kd) / s;
        float p_r = length(P.intersectedObj->material->Ks) / s;
        float p_t = length(P.intersectedObj->material->Kt) / s;

        vec3 omegaIn = normalize(L.intersectPoint - P.intersectPoint);
        float p = PdfAsLight(L, &ibl) / GeometryFactor(P, L);
        float q = L.intersectedObj->PdfBrdf(omegaOut, N, omegaIn, p_d, p_r, p_t, iorIn, iorOut, ior, choiceType::Phong) * RussianRoulette;
        float weightMIS = powf(p, 2) / (powf(p, 2) + powf(q, 2));

        Intersection I = intersect(Ray(P.intersectPoint, omegaIn));
        if (p > 0.0001f && I.intersectedObj != nullptr && I.intersectPoint == L.intersectPoint)
        {
            vec3 f = P.intersectedObj->EvalScattering(omegaOut, N, omegaIn, P, iorIn, iorOut, ior, choiceType::Phong);
            C += 0.5f * W* weightMIS * f / p * L.intersectedObj->material->Kd;
        }

        // Extend path
        omegaIn = P.intersectedObj->SampleBrdf(omegaOut, N, p_d, p_r, ior, choiceType::Phong);
        //w = normalize(w);

        Intersection Q = intersect(Ray(P.intersectPoint, omegaIn));
        if (Q.intersectedObj == nullptr)
            break;

        vec3 f = P.intersectedObj->EvalScattering(omegaOut, N, omegaIn, P, iorIn, iorOut, ior, choiceType::Phong);
        float p_ = P.intersectedObj->PdfBrdf(omegaOut, N, omegaIn, p_d, p_r, p_t, iorIn, iorOut, ior, choiceType::Phong) * RussianRoulette;
        // Explicit light connection
        //C += ExplicitLightConnection(omegaOut, P, raycast, P.intersectNormal, W, p_);

        if (p_ < 0.000001f)
            break;
        W *= f / p_;

        // Implicit light connection
        if (Q.intersectedObj->material->isLight())
        {
            q = PdfAsLight(Q, &ibl) / GeometryFactor(P, Q);
            weightMIS = (p_ * p_) / (p_ * p_ + q * q);
            C += W * weightMIS * Radiance(Q, &ibl);
            break;
        }
        //step forward
        P = Q;
        N = P.intersectNormal;
        omegaOut = -omegaIn;
        ior = CalculateIndexOfRefraction(dot(omegaOut, N), P, iorIn, iorOut);
    }


    return C;
}


Intersection AccelerationBvh::intersect(const Ray& ray)
{
    bvh::Ray<float> bvhRay = RayToBvh(ray);

    // Magic found in the bvh examples:
    bvh::ClosestPrimitiveIntersector<bvh::Bvh<float>, BvhShape> intersector(bvh, shapeVector.data());
    bvh::SingleRayTraverser<bvh::Bvh<float>> traverser(bvh);

    auto hit = traverser.traverse(bvhRay, intersector);
    if (hit) {
        return hit->intersection;
    }
    else
        return  Intersection();  // Return an IntersectionRecord which indicates NO-INTERSECTION


}

