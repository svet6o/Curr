// AccTreeRendering.h - ПОПРАВЕНА ВЕРСИЯ
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
// Corrected traceRayWithBVH function - следва точно алгоритъма от изискванията
inline CRTColor traceRayWithBVH(
    const CRTRay& ray,
    const std::vector<CRTTriangle>& triangles,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTSettings& settings,
    const CRTAccTree& accTree,
    int depth = 0,
    float shadowBias = 1e-4f,
    float refractionBias = 1e-4f
) {
    // Check max ray depth from settings
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
            // SHADE ALGORITHM - точно както е описан в изискванията
            CRTColor finalColor(0, 0, 0);
            
            // For all lights in the scene
            for (const auto& light : lights) {
                // Compute direction from hitPoint to light position
                CRTVector lightDir = light.getPosition() - hitPoint;
                
                // Compute sphere radius (distance to light)
                float sr = lightDir.length();
                
                // Normalize lightDir
                lightDir = lightDir.normalize();
                
                // Calculate Cosine Law
                float cosLaw = std::max(0.0f, normalToUse.dot(lightDir));
                if (cosLaw <= 0.0f) continue; // Light behind surface
                
                // Light intensity
                float li = light.getIntensity();
                
                // Compute sphere area: sa = 4 * pi * sr * sr
                float sa = 4.0f * 3.14159265f * sr * sr;
                
                // Create shadow ray
                CRTRay shadowRay(hitPoint + normalToUse * shadowBias, lightDir);
                
                // Trace shadow ray to check for triangle intersection
                bool intersection = accTree.intersectShadow(shadowRay, triangles, materials, sr, hitTriangleIdx);
                
                // Light contribution: intersection ? 0 : Color(li / sa * albedo * cosLaw)
                if (!intersection) {
                    float attenuation = li / (sa + 1e-4f); // Add epsilon to prevent division by zero
                    CRTColor lightContribution = baseColor * (attenuation * cosLaw);
                    finalColor += lightContribution;
                }
            }
            return finalColor;
        }
        
        case CRTMaterial::Type::REFLECTIVE: {
            CRTVector I = ray.getDirection().normalize();
            CRTVector R = I - normalToUse * (2.0f * I.dot(normalToUse));
            CRTRay reflectRay(hitPoint + normalToUse * shadowBias, R.normalize());
            return traceRayWithBVH(reflectRay, triangles, materials, lights, settings, accTree, depth + 1, shadowBias, refractionBias);
        }

        case CRTMaterial::Type::REFRACTIVE: {
            CRTVector I = ray.getDirection().normalize();
            CRTVector N = normalToUse;
            float eta1 = 1.0f, eta2 = mat.ior;
            
            // Check if the incident ray leaves the transparent object
            if (I.dot(N) > 0.0f) {
                N = -N;
                std::swap(eta1, eta2);
            }
            
            // Compute the cosine between I and N
            float cosIN = -I.dot(N);
            
            // Check if angle(I, N) < critical angle
            float sinIN = std::sqrt(1.0f - cosIN * cosIN);
            float criticalSin = eta2 / eta1;
            
            if (sinIN < criticalSin) {
                // No total internal reflection - compute both reflection and refraction
                
                // Using Snell's Law find sin(R, -N)
                float sinRN = (sinIN * eta1) / eta2;
                float cosRN = std::sqrt(1.0f - sinRN * sinRN);
                
                // Compute vector R using vector addition: R = A + B
                // A = cos(β) * -N = cosRN * (-N)
                CRTVector A = (-N) * cosRN;
                
                // C = (I + cos(α) * N).normalize() where cos(α) = cosIN
                CRTVector C = (I + N * cosIN).normalize();
                
                // B = C * sin(β) = C * sinRN
                CRTVector B = C * sinRN;
                
                // R = A + B
                CRTVector R = A + B;
                R = R.normalize();
                
                // Construct refraction ray
                CRTRay refractionRay(hitPoint + (-N) * refractionBias, R);
                CRTColor refractionColor = traceRayWithBVH(refractionRay, triangles, materials, lights, settings, accTree, depth + 1, shadowBias, refractionBias);
                
                // Construct reflection ray
                CRTVector reflectionDir = I - 2.0f * I.dot(N) * N;
                CRTRay reflectionRay(hitPoint + N * shadowBias, reflectionDir.normalize());
                CRTColor reflectionColor = traceRayWithBVH(reflectionRay, triangles, materials, lights, settings, accTree, depth + 1, shadowBias, refractionBias);
                
                // Fresnel calculation: fresnel = 0.5 * (1.0 + dot(I, N))^5
                float fresnel = 0.5f * std::pow(1.0f + I.dot(N), 5.0f);
                
                return reflectionColor * fresnel + refractionColor * (1.0f - fresnel);
            }
            else {
                // Total internal reflection - construct reflection ray only
                CRTVector reflectionDir = I - 2.0f * I.dot(N) * N;
                CRTRay reflectionRay(hitPoint + N * shadowBias, reflectionDir.normalize());
                return traceRayWithBVH(reflectionRay, triangles, materials, lights, settings, accTree, depth + 1, shadowBias, refractionBias);
            }
        }

        case CRTMaterial::Type::CONSTANT:
            return baseColor;

        default:
            return CRTColor(255, 0, 255); // Magenta for error
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
            // ПОПРАВЕНО: Премахнах isShadowRay параметъра за консистенция с brute force
            CRTColor color = traceRayWithBVH(primary, triangles, materials, lights, settings, accTree, 0, 1e-4f, 1e-4f);

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