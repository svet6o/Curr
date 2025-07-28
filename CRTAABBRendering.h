#pragma once

#include <vector>
#include <thread>
#include <mutex>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>
#include <map>

#include "CRTColor.h"
#include "CRTRay.h"
#include "CRTObject.h"
#include "CRTMaterial.h"
#include "CRTLight.h"
#include "CRTCamera.h"
#include "CRTSettings.h"

// Forward declarations for helper functions
static bool Refract(const CRTVector& I, const CRTVector& N, float eta1, float eta2, CRTVector& refracted);
static float FresnelSchlick(const CRTVector& I, const CRTVector& N, float ior);

// Forward declaration of the main tracing function
CRTColor traceRayWithAABB(
    const CRTRay& ray,
    const std::vector<CRTSceneObject>& sceneObjects,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTSettings& settings,
    int depth = 0,
    bool isShadowRay = false,
    float shadowBias = 1e-4f,
    float refractionBias = 1e-4f
);

// AABB-accelerated ray tracing function
inline CRTColor traceRayWithAABB(
    const CRTRay& ray,
    const std::vector<CRTSceneObject>& sceneObjects,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTSettings& settings,
    int depth,
    bool isShadowRay,
    float shadowBias,
    float refractionBias
) {
    if (depth > 5) {
        return settings.backgroundColor;
    }

    float closestT = std::numeric_limits<float>::infinity();
    int hitObjectIndex = -1;
    int hitTriangleIndex = -1;
    float hitU = 0.0f, hitV = 0.0f;
    CRTVector hitNormal, hitUV;

    // Test intersection with all scene objects
    for (size_t objIdx = 0; objIdx < sceneObjects.size(); ++objIdx) {
        const CRTSceneObject& obj = sceneObjects[objIdx];
        
        float t;
        int triIdx;
        float u, v;
        CRTVector normal, uv;
        
        // AABB test is performed inside the object's intersect method
        if (obj.intersect(ray, t, triIdx, u, v, normal, uv)) {
            if (t < closestT) {
                closestT = t;
                hitObjectIndex = static_cast<int>(objIdx);
                hitTriangleIndex = triIdx;
                hitU = u;
                hitV = v;
                hitNormal = normal;
                hitUV = uv;
            }
        }
    }

    if (hitObjectIndex < 0) {
        return settings.backgroundColor;
    }

    // Get the hit triangle and material
    const CRTSceneObject& hitObject = sceneObjects[hitObjectIndex];
    const CRTTriangle& hitTriangle = hitObject.getTriangle(hitTriangleIndex);
    const CRTMaterial& mat = materials[hitObject.getMaterialIndex()];
    
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

        // Shadow ray
        CRTRay shadowRay(hitPoint + L * shadowBias, L); // offset по посока на лъча (по-стабилно)
        bool inShadow = false;

        for (size_t objIdx = 0; objIdx < sceneObjects.size(); ++objIdx) {
            const auto& obj = sceneObjects[objIdx];

            // Игнорирай само същия триъгълник от същия обект
            int ignoreTri = (int)objIdx == hitObjectIndex ? hitTriangleIndex : -1;

            if (obj.intersectShadowRay(shadowRay, dist, ignoreTri)) {
                const auto& shadowMat = materials[obj.getMaterialIndex()];
                if (shadowMat.type != CRTMaterial::Type::REFRACTIVE) {
                    inShadow = true;
                    break;
                }
                // Ако искаш частични сенки през стъкло – тук добави attenuation вместо просто да пропускаш
            }
        }

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
            return traceRayWithAABB(reflectRay, sceneObjects, materials, lights, settings, depth + 1);
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
            CRTColor C_reflect = traceRayWithAABB(reflectRay, sceneObjects, materials, lights, settings, depth + 1);

            CRTVector Tdir;
            CRTColor C_refract(0, 0, 0);
            if (Refract(I, N, eta1, eta2, Tdir)) {
                CRTRay refractRay(hitPoint - N * refractionBias, Tdir);
                C_refract = traceRayWithAABB(refractRay, sceneObjects, materials, lights, settings, depth + 1);
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
            return CRTColor(255, 0, 255); // Magenta for error
    }
}

// AABB-accelerated bucket rendering
struct AABBBucket {
    int x0, y0, x1, y1;
};

inline void renderAABBBucket(
    const AABBBucket& bucket,
    float* imageBuffer,
    int width, int height,
    const std::vector<CRTSceneObject>& sceneObjects,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTCamera& camera,
    const CRTSettings& settings
) {
    for (int y = bucket.y0; y < bucket.y1; ++y) {
        for (int x = bucket.x0; x < bucket.x1; ++x) {
            CRTRay primary = CRTRay::generatePrimaryRay(x, y, width, height, camera);
            CRTColor color = traceRayWithAABB(primary, sceneObjects, materials, lights, settings, 0, false, 1e-4f, 1e-4f);

            const int idx = (y * width + x) * 3;
            imageBuffer[idx + 0] = static_cast<float>(color.r);
            imageBuffer[idx + 1] = static_cast<float>(color.g);
            imageBuffer[idx + 2] = static_cast<float>(color.b);
        }
    }
}

inline void saveToPPMFromBufferAABB(const std::string& filename,
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

// Main AABB-accelerated rendering function
inline void renderSceneWithAABB(
    const std::vector<CRTSceneObject>& sceneObjects,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTCamera& camera,
    const CRTSettings& settings,
    const std::string& filename
) {
    const int W = settings.resolutionWidth;
    const int H = settings.resolutionHeight;

    // Print AABB statistics
    std::cout << "AABB Acceleration Statistics:\n";
    for (size_t i = 0; i < sceneObjects.size(); ++i) {
        const auto& obj = sceneObjects[i];
        auto stats = obj.getStatistics();
        std::cout << "Object " << i << " (" << obj.name << "): "
                  << stats.triangleCount << " triangles, "
                  << "AABB volume: " << stats.boundingBoxVolume << "\n";
    }

    // Image buffer
    std::vector<float> imageBuffer(W * H * 3, 0.0f);

    // Prepare buckets
    const int bucketSize = (settings.bucketSize > 0) ? settings.bucketSize : 32;
    std::vector<AABBBucket> buckets;
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
            AABBBucket bucket;
            {
                std::lock_guard<std::mutex> lock(bucketsMutex);
                if (buckets.empty()) return;
                bucket = buckets.back();
                buckets.pop_back();
            }

            renderAABBBucket(bucket, imageBuffer.data(), W, H,
                           sceneObjects, materials, lights, camera, settings);
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

    saveToPPMFromBufferAABB(filename, imageBuffer.data(), W, H);
}

// Utility function to convert triangle list to scene objects
inline std::vector<CRTSceneObject> createSceneObjectsFromTriangles(
    const std::vector<CRTTriangle>& triangles,
    bool groupByMaterial = true
) {
    std::vector<CRTSceneObject> sceneObjects;
    
    if (groupByMaterial) {
        // Group triangles by material index
        std::map<int, std::vector<CRTTriangle>> materialGroups;
        
        for (const auto& triangle : triangles) {
            int matIdx = triangle.getMaterialIndex();
            materialGroups[matIdx].push_back(triangle);
        }
        
        // Create scene objects for each material group
        for (const auto& group : materialGroups) {
            std::string name = "Material_" + std::to_string(group.first);
            CRTSceneObject obj(group.second, name, group.first);
            sceneObjects.push_back(obj);
        }
    } else {
        // Each triangle becomes its own object (less efficient but more granular)
        for (size_t i = 0; i < triangles.size(); ++i) {
            std::vector<CRTTriangle> singleTriangle = { triangles[i] };
            std::string name = "Triangle_" + std::to_string(i);
            CRTSceneObject obj(singleTriangle, name, triangles[i].getMaterialIndex());
            sceneObjects.push_back(obj);
        }
    }
    
    return sceneObjects;
}