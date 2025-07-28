#pragma once

#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>
#include <thread>
#include <mutex>
#include <fstream>
#include <cmath>

#include "CRTTriangle.h"
#include "CRTAABB.h"
#include "CRTRay.h"
#include "CRTColor.h"
#include "CRTMaterial.h"
#include "CRTLight.h"
#include "CRTCamera.h"
#include "CRTSettings.h"
#include "CRTAccTree.h"

// Forward declarations - these need to be implemented in your main .cpp file
static bool Refract(const CRTVector& I, const CRTVector& N, float eta1, float eta2, CRTVector& refracted);
static float FresnelSchlick(const CRTVector& I, const CRTVector& N, float ior);
void saveToPPM(const std::string &filename, const std::vector<std::vector<CRTColor>> &image);

template <typename T>
constexpr const T &clamp(const T &value, const T &min, const T &max);

// BVH-accelerated ray tracing function
inline CRTColor traceRayWithBVH(
    const CRTRay &ray,
    const std::vector<CRTTriangle> &scene,
    const std::vector<CRTMaterial> &materials,
    const std::vector<CRTLight> &lights,
    const CRTSettings &settings,
    const CRTAccTree& accTree,
    int depth,
    bool isShadowRay = false,
    float shadowBias = 1e-4f,
    float refractionBias = 1e-4f)
{
    if (depth > 5)
        return settings.backgroundColor;

    float closestT = std::numeric_limits<float>::infinity();
    int hitIdx = -1;
    float hitU = 0.0f, hitV = 0.0f;
    CRTVector hitNormal, hitUV;

    // Use acceleration tree instead of brute force loop
    if (!accTree.intersect(ray, scene, closestT, hitIdx, hitU, hitV, hitNormal, hitUV)) {
        return settings.backgroundColor;
    }

    CRTVector hitPoint = ray.getOrigin() + ray.getDirection() * closestT;
    const CRTMaterial &mat = materials[scene[hitIdx].getMaterialIndex()];
    CRTVector normalToUse = mat.smoothShading
                                ? hitNormal
                                : scene[hitIdx].getFaceNormal();

    // Sample texture (matching your original)
    CRTColor baseColor = (mat.texType != CRTMaterial::TexType::NONE)
                             ? mat.sampleTexture(hitUV, hitU, hitV)
                             : mat.albedo;

    // Material logic (exactly matching your original)
    switch (mat.type)
    {
    case CRTMaterial::Type::DIFFUSE:
    {
        CRTColor accum(0, 0, 0);
        for (const auto &light : lights)
        {
            CRTVector L = light.getPosition() - hitPoint;
            float dist = L.length();
            L = L.normalize();

            float NdotL = std::max(0.0f, normalToUse.dot(L));
            if (NdotL <= 0.0f)
                continue;

            // Shadow ray using acceleration tree instead of brute force
            CRTRay shadowRay(hitPoint + normalToUse * shadowBias, L);
            bool inShadow = accTree.intersectShadow(shadowRay, scene, materials, dist, hitIdx);

            if (inShadow)
                continue;

            float Li = light.getIntensity();
            float attenuation = Li / (4.0f * PI * dist * dist + 1e-4f);
            accum += baseColor * (attenuation * NdotL);
        }
        return accum;
    }

    case CRTMaterial::Type::REFLECTIVE:
    {
        CRTVector I = ray.getDirection().normalize();
        CRTVector R = I - normalToUse * (2.0f * I.dot(normalToUse));
        CRTRay reflectRay(hitPoint + normalToUse * shadowBias, R.normalize());
        return traceRayWithBVH(reflectRay, scene, materials, lights, settings, accTree, depth + 1);
    }

    case CRTMaterial::Type::REFRACTIVE:
    {
        CRTVector I = ray.getDirection().normalize();
        CRTVector N = normalToUse;
        float eta1 = 1.0f, eta2 = mat.ior;
        if (I.dot(N) > 0.0f)
        {
            N = -N;
            std::swap(eta1, eta2);
        }

        CRTVector Rdir = I - N * (2.0f * I.dot(N));
        CRTRay reflectRay(hitPoint + N * shadowBias, Rdir.normalize());
        CRTColor C_reflect = traceRayWithBVH(reflectRay, scene, materials, lights, settings, accTree, depth + 1);

        CRTVector Tdir;
        CRTColor C_refract(0, 0, 0);
        if (Refract(I, N, eta1, eta2, Tdir))
        {
            CRTRay refractRay(hitPoint - N * refractionBias, Tdir);
            C_refract = traceRayWithBVH(refractRay, scene, materials, lights, settings, accTree, depth + 1);
        }

        if (isShadowRay)
        {
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
        return CRTColor(255, 0, 255); // Magenta for error
    }
}

// Simple single-threaded BVH rendering (matching your original structure exactly)
inline void renderTriangleSceneWithBVH(
    const std::vector<CRTTriangle> &triangles,
    const std::vector<CRTMaterial> &materials,
    const std::vector<CRTLight> &lights,
    const CRTCamera &camera,
    const CRTSettings &settings,
    const CRTAccTree& accTree,
    const std::string &filename)
{
    int W = settings.resolutionWidth;
    int H = settings.resolutionHeight;
    CRTColor bg = settings.backgroundColor;

    // Print acceleration tree statistics
    accTree.printStatistics();

    std::vector<std::vector<CRTColor>> image(H, std::vector<CRTColor>(W, bg));

    for (int y = 0; y < H; ++y)
    {
        for (int x = 0; x < W; ++x)
        {
            CRTRay primary = CRTRay::generatePrimaryRay(x, y, W, H, camera);

            // Use BVH-accelerated tracing instead of regular tracing
            image[y][x] = traceRayWithBVH(primary, triangles, materials, lights, settings, accTree, 0, false, 1e-4f, 1e-4f);
        }
    }

    // Use your existing saveToPPM function
    saveToPPM(filename, image);
}

// Bucket structure for multithreaded rendering
struct BVHBucket {
    int x0, y0, x1, y1;
};

// Thread-safe multithreaded bucket renderer using flat buffer
inline void renderBVHBucket(
    const BVHBucket& bucket,
    CRTColor* imageBuffer,  // Changed to flat buffer for thread safety
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
            
            // Thread-safe write to flat buffer
            const int idx = y * width + x;
            imageBuffer[idx] = color;
        }
    }
}

// Fixed multithreaded BVH rendering function - thread-safe
inline void renderTriangleSceneWithBVHMultithreaded(
    const std::vector<CRTTriangle>& triangles,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTCamera& camera,
    const CRTSettings& settings,
    const CRTAccTree& accTree,
    const std::string& filename
) {
    const int W = settings.resolutionWidth;
    const int H = settings.resolutionHeight;
    const CRTColor bg = settings.backgroundColor;

    // Print acceleration tree statistics
    accTree.printStatistics();

    // Use thread-safe flat buffer instead of 2D vector
    std::vector<CRTColor> imageBuffer(W * H, bg);

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

    // Convert flat buffer back to 2D vector for saveToPPM
    std::vector<std::vector<CRTColor>> image(H, std::vector<CRTColor>(W));
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            image[y][x] = imageBuffer[y * W + x];
        }
    }

    // Use your existing saveToPPM function
    saveToPPM(filename, image);
}

// Alternative float buffer version (if you prefer the buffer approach)
inline void renderBVHBucketToBuffer(
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

inline void saveToPPMFromBuffer(const std::string& filename,
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

// Buffer-based multithreaded rendering (alternative approach)
inline void renderTriangleSceneWithBVHBuffer(
    const std::vector<CRTTriangle>& triangles,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTCamera& camera,
    const CRTSettings& settings,
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

            renderBVHBucketToBuffer(bucket, imageBuffer.data(), W, H,
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

    saveToPPMFromBuffer(filename, imageBuffer.data(), W, H);
}