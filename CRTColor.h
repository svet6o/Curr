#pragma once

#include <algorithm>
#include <vector>
#include <cmath>
#include "CRTVector.h"
#include "CRTLight.h"
#include "CRTRay.h"
#include "CRTTriangle.h"

constexpr float PI = 3.14159265358979323846f;

struct CRTColor {
    int r, g, b;

    CRTColor() : r(0), g(0), b(0) {}
    CRTColor(int rr, int gg, int bb) : r(rr), g(gg), b(bb) {}

    CRTColor operator*(float s) const {
        float rf = r * s;
        float gf = g * s;
        float bf = b * s;
        int rc = static_cast<int>(std::max(0.0f, std::min(rf, 255.0f)));
        int gc = static_cast<int>(std::max(0.0f, std::min(gf, 255.0f)));
        int bc = static_cast<int>(std::max(0.0f, std::min(bf, 255.0f)));
        return CRTColor{rc, gc, bc};
    }

   
    CRTColor operator+(const CRTColor& o) const {
        return CRTColor{
            std::min(255, r + o.r),
            std::min(255, g + o.g),
            std::min(255, b + o.b)
        };
    }


    CRTColor operator-(const CRTColor& other) const {
        return CRTColor{
            std::max(0, r - other.r),
            std::max(0, g - other.g),
            std::max(0, b - other.b)
        };
    }

  
    CRTColor operator*(const CRTColor& other) const {
        return CRTColor{
            static_cast<int>((r * other.r) / 255),
            static_cast<int>((g * other.g) / 255),
            static_cast<int>((b * other.b) / 255)
        };
    }


    CRTColor& operator+=(const CRTColor& o) {
        r = std::min(255, r + o.r);
        g = std::min(255, g + o.g);
        b = std::min(255, b + o.b);
        return *this;
    }
};

inline CRTColor shade(
    const CRTVector& p,
    const CRTVector& triN,
    const CRTColor& a,
    const std::vector<CRTLight>& lights,
    const std::vector<CRTTriangle>& scene,
    const CRTTriangle* hitTri,
    float shadowBias = 1e-4f
) {
    CRTColor FinalColor(0, 0, 0);

    for (const auto& light : lights) {
        CRTVector lightDir = light.getPosition() - p;
        float sr = lightDir.length();
        lightDir = lightDir.normalize();

        float cosLaw = std::max(0.0f, lightDir.dot(triN));
        if (cosLaw <= 0.0f) continue;

        float li = light.getIntensity();
        float sa = 4.0f * PI * sr * sr + 1e-4f;

        CRTVector origin = p + triN * shadowBias;
        CRTRay shadowRay{origin, lightDir};

        bool intersection = false;
        for (const auto& tri : scene) {
            if (&tri == hitTri) continue;

            float tHit = 0.0f;
            float u = 0.0f, v = 0.0f;
            CRTVector normal;
            CRTVector dummyUV;

            if (tri.intersect(shadowRay, tHit, u, v, normal, dummyUV)) {
                if (tHit < sr) {
                    intersection = true;
                    break;
                }
            }
        }

        if (!intersection) {
            FinalColor += a * (li / sa * cosLaw);
        }
    }

    return FinalColor;
}