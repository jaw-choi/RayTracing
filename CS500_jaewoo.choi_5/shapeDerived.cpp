#include "shapeDerived.h"



bool SphereIntersect(Ray ray, Intersection& intersection_, vec3 center, float radius)
{

    vec3 Qvec = ray.origin - center;
    float a = dot(ray.direction, ray.direction);
    float b = 2.f * dot(Qvec, ray.direction);
    float c = dot(Qvec, Qvec) - (radius * radius);

    float discriminant = (b * b) - (4.f * a * c);
    if (discriminant < 0.0001f)
        return false;

    float tm = (-b - (sqrt(discriminant))) / (2.f * a);
    float tp = (-b + (sqrt(discriminant))) / (2.f * a);

    if (tm < 0.0001f && tp < 0.0001f)
        return false;

    //if (tm > 0.0001f && tm < tp)
    //    intersection_.t = tm;
    //else if (tp > 0.0001f && tm > tp)
    //    intersection_.t = tp;

    //if (tm > 0.00001f && tp < 0.00001f)
    //    intersection_.t = tm;
    //else if (tm < 0.00001f && tp > 0.00001f)
    //    intersection_.t = tp;
    //else if (tm > 0.00001f && tp > 0.00001f)
    //    intersection_.t = (tm < tp) ? tm : tp;
    if (intersection_.t > tp || intersection_.t > tm)
    {
        if (tm < 0.0001f)
            intersection_.t = tp;
        else
            intersection_.t = tm;

        intersection_.intersectPoint = ray.Eval(intersection_.t);
        intersection_.intersectNormal = normalize(intersection_.intersectPoint - center);
        //intersection_.shape = this;

        return true;
    }

    intersection_.intersectPoint = ray.Eval(intersection_.t);
    intersection_.intersectNormal = normalize(intersection_.intersectPoint - center);

    return true;
}

bool BoxIntersect(Ray ray, Intersection& intersection_, vec3 min, vec3 diagonal)
{
    Slab s1;
    s1.N = vec3(1.0f, 0.0f, 0.0f);
    s1.d0 = -min.x;
    s1.d1 = -min.x - diagonal.x;

    Slab s2;
    s2.N = vec3(0.0f, 1.0f, 0.0f);
    s2.d0 = -min.y;
    s2.d1 = -min.y - diagonal.y;

    Slab s3;
    s3.N = vec3(0.0f, 0.0f, 1.0f);
    s3.d0 = -min.z;
    s3.d1 = -min.z - diagonal.z;

    Interval i1, i2, i3;

    intersection_.intersectPoint = ray.Eval(intersection_.t);
    if (i1.Intersect(ray, s1)
        && i2.Intersect(ray, s2)
        && i3.Intersect(ray, s3))
    {
        float min_t = std::max(std::max(i1.t0, i2.t0), std::max(i1.t0, i3.t0));
        float max_t = std::min(std::min(i1.t1, i2.t1), std::min(i1.t1, i3.t1));

        if (min_t > max_t)
            return false;

        if (min_t > 0.00001f && intersection_.t > min_t)
        {
            intersection_.t = min_t;
            intersection_.intersectPoint = ray.Eval(intersection_.t);

            if (fabs(intersection_.t - i1.t0) < 0.00001f)
            {
                intersection_.intersectNormal = s1.N;
            }
            else if (fabs(intersection_.t - i2.t0) < 0.00001f)
            {
                intersection_.intersectNormal = s2.N;
            }
            else if (fabs(intersection_.t - i3.t0) < 0.00001f)
            {
                intersection_.intersectNormal = s3.N;
            }

        }
        else if (max_t > 0.00001f && intersection_.t > max_t)
        {
            intersection_.t = max_t;
            intersection_.intersectPoint = ray.Eval(intersection_.t);

            if (fabs(intersection_.t - i1.t1) < 0.00001f)
            {
                intersection_.intersectNormal = -s1.N;
            }
            else if (fabs(intersection_.t - i2.t1) < 0.00001f)
            {
                intersection_.intersectNormal = -s2.N;
            }
            else if (fabs(intersection_.t - i3.t1) < 0.00001f)
            {
                intersection_.intersectNormal = -s3.N;
            }
        }
        else
            return false;

        return true;
    }
    return false;
}


bool TriangleIntersect(Ray ray, Intersection& intersection_,
    vec3 v0, vec3 v1, vec3 v2,
    vec3 n0, vec3 n1, vec3 n2)
{
    vec3 e1 = v1 - v0;
    vec3 e2 = v2 - v0;
    vec3 s = ray.origin - v0;

    float d = dot(cross(ray.direction, e2), e1);
    vec3 p = cross(ray.direction, e2);
    if (fabs(d - 0.00001f) < 0.00001f)
        return false;

    vec3 S = ray.origin - v0;
    float u = dot(p, s) / d;
    if (u < 0.00001f || u>1)
        return false;

    vec3 q = cross(s, e1);
    float v = dot(ray.direction, q) / d;
    if (v < 0.00001f || u + v>1)
        return false;
    float t = dot(e2, q) / d;
    if (t < 0.00001f)
        return false;


    intersection_.t = t;
    intersection_.intersectPoint = ray.Eval(t);
    vec3 normal = (1 - u - v) * n0 + u * n1 + v * n2;
    intersection_.intersectNormal = normal;
    return true;

}

bool CylinderIntersect(Ray ray, Intersection& intersection_, vec3 center, vec3 axis)
{
    float radius = 0.05f;
    vec3 A = normalize(axis);

    vec3 B = vec3(0, 0, 1);
    if (dot(A, B) == length(A) * length(B)) //PARALLEL
        B = vec3(1, 0, 0);

    B = normalize(cross(B, A));
    vec3 C = normalize(cross(A, B));

    mat3 R_inversed = glm::mat3(B, C, A);
    mat3 R = glm::transpose(R_inversed);

    Slab slab;
    slab.N = vec3(0.0f, 0.0f, 1.0f);
    slab.d0 = 0.f;
    slab.d1 = -length(axis);

    Ray mRay = ray;

    ray.direction = normalize(R * ray.direction);
    ray.origin = ray.origin - center;
    ray.origin = R * ray.origin;

    Interval interval;
    if (interval.Intersect(ray, slab))
    {

        float a = ray.direction.x * ray.direction.x + ray.direction.y * ray.direction.y;
        float b = 2 * (ray.direction.x * ray.origin.x + ray.direction.y * ray.origin.y);
        float c = (ray.origin.x * ray.origin.x) + (ray.origin.y * ray.origin.y) - (radius * radius);

        float discriminant = (b * b) - (4 * a * c);
        if (discriminant < 0.f)
            return false;

        float b0 = (-b - sqrt(discriminant)) / (2 * a);
        float b1 = (-b + sqrt(discriminant)) / (2 * a);

        float t0 = std::max(interval.t0, b0);
        float t1 = std::min(interval.t1, b1);

        if (t0 > t1)
            return false;
        if (t1 < 0.0001f)
            return false;

        if (t0 > 0.0001f && intersection_.t > t0)
        {
            intersection_.t = t0;
            if (fabs(t0 - interval.t0) <= 0.0001f)
            {
                intersection_.intersectNormal = vec3(0.f, 0.f, 1.f);
            }
            else
            {
                vec3 rayLine = ray.Eval(intersection_.t);

                vec3 normalHat = vec3(rayLine.x, rayLine.y, 0.f);
                normalHat = normalize(R_inversed * normalHat);
                intersection_.intersectNormal = normalHat;

                float largest = std::max(std::max(intersection_.intersectNormal.x, intersection_.intersectNormal.y),
                    std::max(intersection_.intersectNormal.x, intersection_.intersectNormal.z));

                intersection_.intersectNormal /= largest;
            }
        }
        else if (t1 > 0.0001f && intersection_.t > t1)
        {
            intersection_.t = t1;
            if (fabs(t1 - interval.t1) <= 0.0001f)
            {
                intersection_.intersectNormal = vec3(0.f, 0.f, -1.f);
            }
            else
            {
                vec3 rayLine = ray.Eval(intersection_.t);

                vec3 normalHat = vec3(rayLine.x, rayLine.y, 0.f);
                normalHat = normalize(R_inversed * normalHat);
                intersection_.intersectNormal = -normalHat;

                float largest = std::max(std::max(intersection_.intersectNormal.x, intersection_.intersectNormal.y),
                    std::max(intersection_.intersectNormal.x, intersection_.intersectNormal.z));

                intersection_.intersectNormal /= largest;
            }
        }
        intersection_.intersectPoint = mRay.Eval(intersection_.t);
    }
    return true;

}

Sphere::Sphere(MeshData* md, mat4 mTR, Material* mat) : Shape(md, mTR, mat) {}
Box::Box(MeshData* md, mat4 mTR, Material* mat) : Shape(md, mTR, mat) {}
Triangle::Triangle(MeshData* md, mat4 mTR, Material* mat) : Shape(md, mTR, mat) {}
Cylinder::Cylinder(MeshData* md, mat4 mTR, Material* mat) : Shape(md, mTR, mat) {}

bool Sphere::intersect(Ray r, Intersection& intersection)
{
    float a = dot(r.direction, r.direction);
    float b = 2.f * dot(r.origin - center, r.direction);
    float c = dot(r.origin - center, r.origin - center) - (radius * radius);

    float discriminant = (b * b) - (4.f * a * c);

    if (discriminant < 0.0001f)
        return false;

    float root1 = (-b + sqrt(discriminant)) / (2.f * a);	//t+
    float root2 = (-b - sqrt(discriminant)) / (2.f * a);	//t-

    if (root1 < 0.0001f && root2 < 0.0001f)
        return false;

    if (intersection.t > root1 || intersection.t > root2)
    {
        if (root2 < 0.0001f)
            intersection.t = root1;
        else
            intersection.t = root2;

        intersection.intersectPoint = r.Eval(intersection.t);
        intersection.intersectNormal = normalize(intersection.intersectPoint - center);
        intersection.intersectedObj = this;

        return true;
    }

    return false;
}

vec3 Shape::BeersLaw(vec3 omegaOut, vec3 Normal, const Intersection& i)
{
    float cx = log(i.intersectedObj->material->Kt.x);
    float cy = log(i.intersectedObj->material->Kt.y);
    float cz = log(i.intersectedObj->material->Kt.z);
    vec3 A_t(1.f,1.f,1.f);
    if (dot(omegaOut, Normal) < 0)
    {
        A_t.x = exp(i.t * cx);
        A_t.y = exp(i.t * cy);
        A_t.z = exp(i.t * cz);
    }
    return A_t;
}

vec3 Shape::SampleBrdf(vec3 omegaOut, vec3 Normal, float p_d, float p_r, float ior, choiceType type_)
{

    const float choice = myrandom(RNGen);
    const float x1 = myrandom(RNGen);
    const float x2 = myrandom(RNGen);

    float alphaBeck = abs(sqrt(2.f / (material->alpha + 2)));
    float alphaG = alphaBeck;
    vec3 omegaIn(0.f);
    vec3 m(0.f);
    float cosThetaM = 0;

    if (type_ == choiceType::Phong)
        cosThetaM = pow(x1, 1.f / (material->alpha + 1));
    else if (type_ == choiceType::Beckman)
        cosThetaM = cos(atan(sqrt(-(alphaBeck * alphaBeck * log(1.f - x1)))));
    else
        cosThetaM = cos(atan((alphaG * sqrt(x1)) / (sqrt(1.f - x1))));

    if (choice < p_d)//choice = diffuse
    {
        omegaIn = SampleLobe(Normal, sqrt(x1), 2.f * PI * x2);
    }
    else if (choice < p_d + p_r)//choice = reflection
    {
        m = SampleLobe(Normal, cosThetaM, 2.f * PI * x2);
        omegaIn = normalize(2.f * fabs(dot(omegaOut, m)) * m - omegaOut);
    }
    else//choice = transmission
    {
        m = SampleLobe(Normal, cosThetaM, 2.f * PI * x2);
        float radicand = 1.f - pow(ior,2) * (1.f - pow(dot(omegaOut, m),2));
        if(radicand < 0.0001f)
            omegaIn = normalize(2.f * fabs(dot(omegaOut, m)) * m - omegaOut);
        else
            omegaIn = normalize((ior * dot(omegaOut, m) - Sign(dot(omegaOut, Normal)) * sqrt(radicand)) * m - (ior * omegaOut));
    }


    return omegaIn;
}

vec3 Shape::SampleLobe(vec3 A, float ran1, float ran2)
{
    vec3 A_ = normalize(A);
    float s = sqrt(1 - ran1 * ran1);
    vec3 K(cos(ran2) * s, sin(ran2) * s, ran1);

    if (fabs(A_.z - 1.f) < 0.0001f)
        return K;
    else if (fabs(A_.z + 1.f) < 0.0001f)
        return vec3(K.x, -K.y, -K.z);

    vec3 B = normalize(vec3(-A_.y, A_.x, 0.f));
    vec3 C = cross(A_, B);

    return K.x * B + K.y * C + K.z * A_;
}

float Shape::PdfBrdf(vec3 omegaOut, vec3 Normal, vec3 omegaIn, float p_d, float p_r, float p_t, float iorIn, float iorOut, float ior, choiceType type_)
{
    float probablityP_d = fabs(dot(omegaIn, Normal)) / PI;
    vec3 m = normalize(omegaOut + omegaIn);

    float probablityP_r = Distribution(dot(m, Normal),type_) * fabs(dot(m, Normal)) * (1.f / (4.f * fabs(dot(omegaIn, m))));

    vec3 mTrans = -normalize(iorOut * omegaIn + iorIn * omegaOut);
    float radicand = 1.f - (ior * ior) * (1.f - (pow(dot(omegaOut, mTrans), 2)));

    float probablityP_t = 0;

    if (radicand < 0.0001f)
    {
        probablityP_t = probablityP_r;
    }
    else
    {
        probablityP_t = Distribution(dot(mTrans, Normal), type_)
                        * fabs(dot(mTrans, Normal))
                        * ((iorOut * iorOut * fabs(dot(omegaIn, mTrans))) / (pow((iorOut * (dot(omegaIn, mTrans)) + iorIn * (dot(omegaOut, mTrans))), 2)));
    }

    return p_d * probablityP_d + p_r * probablityP_r + p_t * probablityP_t;
}

float Shape::Sign(float x)
{
    return (x >= 0.0001f) ? 1.f : -1.f;
}

vec3 Shape::EvalScattering(vec3 omegaOut, vec3 Normal, vec3 omegaIn, const Intersection& i, float iorIn, float iorOut, float ior, choiceType type_)
{
    vec3 E_d = material->Kd / PI;
    vec3 m = normalize(omegaOut + omegaIn);
    vec3 E_r = (Distribution(dot(m, Normal),type_) * GFactor(omegaOut, omegaIn, m, Normal, type_) * Fresnel(dot(omegaIn, m), i)) / (4.f * fabs(dot(omegaIn, Normal) * fabs(dot(omegaOut, Normal))));
    vec3 E_t{};
    vec3 mTrans = -normalize((iorOut * omegaIn + iorIn * omegaOut));
    float radicand = 1 - (ior * ior) * (1 - (pow(dot(omegaOut, mTrans), 2)));

    float D = Distribution(dot(m, Normal), type_);
    float G = GFactor(omegaOut, omegaIn, m, Normal, type_);
    vec3 F = Fresnel(dot(omegaIn, m), i);
    float denominator = 4.f * fabs(dot(omegaIn, Normal) * fabs(dot(omegaOut, Normal)));

    float D_ = Distribution(dot(mTrans, Normal), type_);
    float G_ = GFactor(omegaOut, omegaIn, mTrans, Normal, type_);
    vec3 F_ = Fresnel(dot(omegaIn, mTrans), i);
    float denominator_ = fabs(dot(omegaIn, Normal)) * fabs(dot(omegaOut, Normal));
    float numeratorRight = (fabs(dot(omegaIn, mTrans)) * fabs(dot(omegaOut, mTrans)) * iorOut * iorOut);
    float denomiRight =  pow((iorOut * (dot(omegaIn, mTrans)) + iorIn * (dot(omegaOut, mTrans))), 2);

    vec3 A_t = BeersLaw(omegaOut,Normal, i);

    if (radicand <= 0.0001f)
    {
        E_t = A_t *( (D * G * F) / (denominator));
    }
    else
    {
        E_t = A_t * ((D_ * G_ * (1.f - F_)) / (denominator_)) * (numeratorRight / denomiRight);

    }
    return fabs(dot(Normal, omegaIn)) * (E_d + E_r + E_t);
}

float Shape::CharacteristiceFactor(float d)
{
    if (d > 0.0001f) 
        return 1.f;
    else
        return 0.f;
}

vec3 Shape::Fresnel(float d, const Intersection& i)
{
    const float scaler = pow((1 - fabs(d)), 5);
    vec3 K_s = i.intersectedObj->material->Ks;
    if (K_s.x > 0.f && K_s.x <= 1.f &&
        K_s.y > 0.f && K_s.y <= 1.f &&
        K_s.z > 0.f && K_s.z <= 1.f)
        return K_s + scaler * (vec3(1.f) - K_s);

    return K_s;
}

float Shape::Distribution(float dot_m_n, choiceType type_)
{
    float tanThetaM = sqrt(1.0 - (pow(dot_m_n, 2))) / dot_m_n;
    float alphaBeck = abs(sqrt(2.f / (material->alpha + 2)));
    float alphaG = alphaBeck;
    if (type_ == choiceType::Phong)
        return CharacteristiceFactor(dot_m_n) * ((material->alpha + 2.f) / (2.f * PI)) * pow((dot_m_n), material->alpha);
    else if (type_ == choiceType::Beckman)
        return CharacteristiceFactor(dot_m_n) * ((exp(-(tanThetaM * tanThetaM) / (alphaBeck * alphaBeck))) / (PI * alphaBeck * alphaBeck) * pow((dot_m_n), 4));

    return CharacteristiceFactor(dot_m_n) * ((alphaG * alphaG) / (PI * pow(dot_m_n, 4) * pow(alphaG * alphaG + tanThetaM * tanThetaM, 2)));
    
}

float Shape::GFactor(vec3 omegaOut, vec3 omegaIn, vec3 m, vec3 normal, choiceType type_)
{
    return G1Factor(omegaIn, m, normal, type_) * G1Factor(omegaOut, m, normal, type_);
}

float Shape::G1Factor(vec3 v, vec3 m, vec3 normal, choiceType type_)
{
    if (dot(v, normal) > 1.0f)
        return 1.0f;

    float tanThetaV = sqrt(1.0f - (pow(dot(v, normal), 2))) / dot(v, normal);
    if (abs(tanThetaV) < 0.001f)
        return 1.0f;

    float alphaBeck = abs(sqrt(2.f / (material->alpha + 2)));
    float alphaG = alphaBeck;
    float a = 0;
    if (type_ == choiceType::Phong)
        a = (sqrt((material->alpha / 2.f) + 1.f) / tanThetaV);
    else if (type_ == choiceType::Beckman)
        a = 1.f / (alphaBeck * tanThetaV);

    float result = CharacteristiceFactor(dot(v, m) / dot(v, normal));

    if (type_ != choiceType::GGX)
    {
        if (a < 1.6f)
            return result * ((3.535 * a + 2.181 * a * a) / (1.0 + 2.276 * a + 2.577 * a * a));
        else
            return result;
    }
    else
    {
        return result * ((2.f) / (1 + sqrt(1 + alphaG * alphaG * tanThetaV * tanThetaV)));
    }
}

float Shape::GetIndexOfRefraction()
{
     return material->ior; 
}

bool Box::intersect(Ray ray, Intersection& intersec)
{
    if (BoxIntersect(ray, intersec, pos, diagonal))
        return true;
}

bool Cylinder::intersect(Ray ray, Intersection& intersection_)
{
    float radius = 0.05f;
    vec3 A = normalize(cylaxis);

    vec3 B = vec3(0, 0, 1);
    if (dot(A, B) == length(A) * length(B))
        B = vec3(1, 0, 0);

    B = normalize(cross(B, A));
    vec3 C = normalize(cross(A, B));

    mat3 R_inversed = glm::mat3(B, C, A);
    mat3 R = glm::transpose(R_inversed);

    Slab slab;
    slab.N = vec3(0.0f, 0.0f, 1.0f);
    slab.d0 = 0.f;
    slab.d1 = -length(cylaxis);

    Ray mRay = ray;

    ray.direction = R * ray.direction;
    ray.origin = R * (ray.origin - center);

    Interval interval;
    if (interval.Intersect(ray, slab))
    {

        float a = ray.direction.x * ray.direction.x + ray.direction.y * ray.direction.y;
        float b = 2 * (ray.direction.x * ray.origin.x + ray.direction.y * ray.origin.y);
        float c = (ray.origin.x * ray.origin.x) + (ray.origin.y * ray.origin.y) - (radius * radius);

        float discriminant = (b * b) - (4 * a * c);
        if (discriminant < 0.f)
            return false;

        float b0 = (-b - sqrt(discriminant)) / (2 * a);
        float b1 = (-b + sqrt(discriminant)) / (2 * a);

        float t0 = std::max(interval.t0, b0);
        float t1 = std::min(interval.t1, b1);

        if (t0 > t1)
            return false;
        if (t1 < 0.0001f)
            return false;

        if (t0 > 0.0001f && intersection_.t > t0)
        {
            intersection_.t = t0;
            intersection_.intersectPoint = mRay.Eval(intersection_.t);

            if (t0 - interval.t0 <= 0.0001f && t0 - interval.t0 >= -0.0001f)
            {
                intersection_.intersectNormal = vec3(0.f, 0.f, 1.f);
            }
            else
            {
                vec3 temp = ray.Eval(t0);
                intersection_.intersectNormal = R_inversed * vec3(temp.x, temp.y, 0.f);
                intersection_.intersectNormal = normalize(intersection_.intersectNormal);
            }
            intersection_.intersectedObj = this;
            return true;
        }
        if (t1 > 0.0001f && intersection_.t > t1)
        {
            intersection_.t = t1;
            intersection_.intersectPoint = mRay.Eval(intersection_.t);

            if (t1 - interval.t1 < 0.0001f && t1 - interval.t1 > -0.0001f)
            {
                intersection_.intersectNormal = vec3(0.f, 0.f, -1.f);
            }
            else
            {
                vec3 temp = ray.Eval(t1);
                intersection_.intersectNormal = R_inversed * vec3(temp.x, temp.y, 0.f);
                intersection_.intersectNormal = normalize(intersection_.intersectNormal);
            }
            intersection_.intersectedObj = this;
            return true;
        }
    }
    return false;
}

bool Triangle::intersect(Ray ray, Intersection& intersec)
{
    vec3 v0 = meshdata->vertices[ivec.x].pnt;
    vec3 v1 = meshdata->vertices[ivec.y].pnt;
    vec3 v2 = meshdata->vertices[ivec.z].pnt;

    vec3 n0 = meshdata->vertices[ivec.x].nrm;
    vec3 n1 = meshdata->vertices[ivec.y].nrm;
    vec3 n2 = meshdata->vertices[ivec.z].nrm;

    if (TriangleIntersect(ray, intersec, v0, v1, v2, n0, n1, n2))
        return true;
}
