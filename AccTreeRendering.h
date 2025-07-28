#pragma once

#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>
#include <thread>
#include <mutex>
#include <fstream>

#include "CRTTriangle.h"
#include "CRTAABB.h"
#include "CRTRay.h"
#include "CRTColor.h"
#include "CRTMaterial.h"
#include "CRTLight.h"
#include "CRTCamera.h"
#include "CRTSettings.h"
#include "CRTAccTree.h"

// Accelerated ray tracing function
inline CRTColor traceRayWithBVH(
    const CRTRay& ray,
    const std::vector<CRTTriangle>& triangles,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTSettings& settings,
    const CRTAccTree& accTree,
    int depth = 0,
    bool isShadowRay = false,
    float shadowBias = 1e-4f,
    float refractionBias = 1e-4f
) {
    if (depth > 5) {
        return settings.backgroundColor;
    }

    float closestT;
    int hitTriangleIdx;
    float hitU, hitV;
    CRTVector hitNormal, hitUV;

    // Use acceleration tree for intersection testing
    if (!accTree.intersect(ray, triangles, closestT, hitTriangleIdx, hitU, hitV, hitNormal, hitUV)) {
        return settings.backgroundColor;
    }

    // Get the hit triangle and material
    const CRTTriangle& hitTriangle = triangles[hitTriangleIdx];
    const CRTMaterial& mat = materials[hitTriangle.getMaterialIndex()];
    
    CRTVector hitPoint = ray.getOrigin() + ray.getDirection() * closestT;
    CRTVector normalToUse = mat.smoothShading ? hitNormal : hitTriangle.getFaceNormal();

    // Sample texture
    CRTColor baseColor = (mat.texType != CRTMaterial::TexType::NONE)
                         ? mat.sampleTexture(hitUV, hitU, hitV)
                         : mat.albedo;

    // Material-specific shading logic
    switch (mat.type) {
        case CRTMaterial::Type::DIFFUSE: {
            CRTColor accum(0, 0, 0);
            for (const auto& light : lights) {
                CRTVector L = light.getPosition() - hitPoint;
                float dist = L.length();
                L = L.normalize();

                float NdotL = std::max(0.0f, normalToUse.dot(L));
                if (NdotL <= 0.0f) continue;

                // Shadow ray using acceleration tree
                CRTRay shadowRay(hitPoint + L * shadowBias, L);
                bool inShadow = accTree.intersectShadow(shadowRay, triangles, materials, dist, hitTriangleIdx);

                if (inShadow) continue;

                float Li = light.getIntensity();
                float attenuation = Li / (4.0f * 3.14159265f * dist * dist + 1e-4f);
                accum += baseColor * (attenuation * NdotL);
            }
            return accum;
        }
        case CRTMaterial::Type::REFLECTIVE: {
            CRTVector I = ray.getDirection().normalize();
            CRTVector R = I - normalToUse * (2.0f * I.dot(normalToUse));
            CRTRay reflectRay(hitPoint + normalToUse * shadowBias, R.normalize());
            return traceRayWithBVH(reflectRay, triangles, materials, lights, settings, accTree, depth + 1);
        }

        case CRTMaterial::Type::REFRACTIVE: {
            CRTVector I = ray.getDirection().normalize();
            CRTVector N = normalToUse;
            float eta1 = 1.0f, eta2 = mat.ior;
            if (I.dot(N) > 0.0f) {
                N = -N;
                std::swap(eta1, eta2);
            }

            CRTVector Rdir = I - N * (2.0f * I.dot(N));
            CRTRay reflectRay(hitPoint + N * shadowBias, Rdir.normalize());
            CRTColor C_reflect = traceRayWithBVH(reflectRay, triangles, materials, lights, settings, accTree, depth + 1);

            CRTVector Tdir;
            CRTColor C_refract(0, 0, 0);
            if (Refract(I, N, eta1, eta2, Tdir)) {
                CRTRay refractRay(hitPoint - N * refractionBias, Tdir);
                C_refract =  traceRayWithBVH(reflectRay, triangles, materials, lights, settings, accTree, depth + 1);
            }

            if (isShadowRay) {
                return (C_refract.r || C_refract.g || C_refract.b) ? C_refract : C_reflect;
            }

            float kr = (C_refract.r || C_refract.g || C_refract.b)
                     ? FresnelSchlick(I, N, mat.ior)
                     : 1.0f;
            return C_reflect * kr + C_refract * (1.0f - kr);
        }

        case CRTMaterial::Type::CONSTANT:
            return baseColor;

        default:
            return CRTColor(255, 0, 255); // Магента за грешка
    }
}


// Bucket structure for multithreaded rendering
struct BVHBucket {
    int x0, y0, x1, y1;
};

inline void renderBVHBucket(
    const BVHBucket& bucket,
    float* imageBuffer,
    int width, int height,
    const std::vector<CRTTriangle>& triangles,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTCamera& camera,
    const CRTSettings& settings,
    const CRTAccTree& accTree
) {
    for (int y = bucket.y0; y < bucket.y1; ++y) {
        for (int x = bucket.x0; x < bucket.x1; ++x) {
            CRTRay primary = CRTRay::generatePrimaryRay(x, y, width, height, camera);
            CRTColor color = traceRayWithBVH(primary, triangles, materials, lights, settings, accTree, 0, false, 1e-4f, 1e-4f);

            const int idx = (y * width + x) * 3;
            imageBuffer[idx + 0] = static_cast<float>(color.r);
            imageBuffer[idx + 1] = static_cast<float>(color.g);
            imageBuffer[idx + 2] = static_cast<float>(color.b);
        }
    }
}

inline void saveToPPMFromBufferBVH(const std::string& filename,
                                   const float* imageBuffer,
                                   int width, int height) {
    std::ofstream ppmFile(filename);
    ppmFile << "P3\n" << width << " " << height << "\n255\n";

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = (y * width + x) * 3;
            int r = std::min(255, std::max(0, static_cast<int>(std::round(imageBuffer[idx + 0]))));
            int g = std::min(255, std::max(0, static_cast<int>(std::round(imageBuffer[idx + 1]))));
            int b = std::min(255, std::max(0, static_cast<int>(std::round(imageBuffer[idx + 2]))));
            ppmFile << r << " " << g << " " << b << " ";
        }
        ppmFile << "\n";
    }
}

// Main BVH-accelerated rendering function
inline void renderTriangleSceneWithBVH(
    const std::vector<CRTTriangle>& triangles,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTSettings& settings,
    const CRTCamera& camera,
    const CRTAccTree& accTree,
    const std::string& filename
) {
    const int W = settings.resolutionWidth;
    const int H = settings.resolutionHeight;

    // Print acceleration tree statistics
    accTree.printStatistics();

    // Image buffer
    std::vector<float> imageBuffer(W * H * 3, 0.0f);

    // Prepare buckets
    const int bucketSize = (settings.bucketSize > 0) ? settings.bucketSize : 32;
    std::vector<BVHBucket> buckets;
    for (int y = 0; y < H; y += bucketSize) {
        for (int x = 0; x < W; x += bucketSize) {
            buckets.push_back({
                x,
                y,
                std::min(x + bucketSize, W),
                std::min(y + bucketSize, H)
            });
        }
    }

    // Multithreaded rendering
    const unsigned hw = std::thread::hardware_concurrency();
    const int numThreads = (hw == 0) ? 4 : static_cast<int>(hw);
    std::mutex bucketsMutex;

    auto worker = [&]() {
        for (;;) {
            BVHBucket bucket;
            {
                std::lock_guard<std::mutex> lock(bucketsMutex);
                if (buckets.empty()) return;
                bucket = buckets.back();
                buckets.pop_back();
            }

            renderBVHBucket(bucket, imageBuffer.data(), W, H,
                           triangles, materials, lights, camera, settings, accTree);
        }
    };

    std::vector<std::thread> threads;
    threads.reserve(numThreads);
    for (int i = 0; i < numThreads; ++i) {
        threads.emplace_back(worker);
    }

    for (auto& t : threads) {
        t.join();
    }

    saveToPPMFromBufferBVH(filename, imageBuffer.data(), W, H);
}
