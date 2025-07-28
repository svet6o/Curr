#pragma once

#include "CRTAccTree.h"
#include "CRTRay.h"
#include "CRTCamera.h"
#include "CRTSettings.h"
#include "CRTMaterial.h"
#include "CRTLight.h"
#include "CRTColor.h"
#include "CRTTriangle.h"
#include <vector>
#include <fstream>
#include <algorithm>
#include <limits>

// Forward declarations
static bool Refract(const CRTVector& I, const CRTVector& N, float eta1, float eta2, CRTVector& refracted);
static float FresnelSchlick(const CRTVector& I, const CRTVector& N, float ior);

// BVH-accelerated ray tracing function
CRTColor traceRayWithBVH(
    const CRTRay& ray,
    const std::vector<CRTTriangle>& scene,
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

    // Use BVH for intersection
    float closestT;
    IntersectionData hitData;
    
    if (!accTree.intersect(ray, materials, closestT, hitData)) {
        return settings.backgroundColor;
    }

    // Get hit information
    CRTVector hitPoint = hitData.hitPoint;
    const CRTMaterial& mat = materials[hitData.materialIndex];
    CRTVector normalToUse = hitData.normal;

    // Sample texture or use base albedo
    CRTColor baseColor = (mat.texType != CRTMaterial::TexType::NONE) 
                        ? mat.sampleTexture(hitData.uv, hitData.u, hitData.v)
                        : mat.albedo;

    // Handle different material types
    switch (mat.type) {
        case CRTMaterial::Type::DIFFUSE: {
            CRTColor accum(0, 0, 0);
            
            for (const auto& light : lights) {
                CRTVector L = light.getPosition() - hitPoint;
                float dist = L.length();
                L = L.normalize();

                float NdotL = std::max(0.0f, normalToUse.dot(L));
                if (NdotL <= 0.0f) continue;

                // Shadow ray test using BVH
                CRTRay shadowRay(hitPoint + normalToUse * shadowBias, L);
                bool inShadow = accTree.intersectShadowRay(shadowRay, dist);

                if (!inShadow) {
                    float Li = light.getIntensity();
                    float attenuation = Li / (4.0f * PI * dist * dist + 1e-4f);
                    accum += baseColor * (attenuation * NdotL);
                }
            }
            return accum;
        }

        case CRTMaterial::Type::REFLECTIVE: {
            CRTVector I = ray.getDirection().normalize();
            CRTVector R = I - normalToUse * (2.0f * I.dot(normalToUse));
            CRTRay reflectRay(hitPoint + normalToUse * shadowBias, R.normalize());
            
            return traceRayWithBVH(reflectRay, scene, materials, lights, settings, accTree, depth + 1);
        }

        case CRTMaterial::Type::REFRACTIVE: {
            CRTVector I = ray.getDirection().normalize();
            CRTVector N = normalToUse;
            float eta1 = 1.0f, eta2 = mat.ior;
            
            if (I.dot(N) > 0.0f) {
                N = -N;
                std::swap(eta1, eta2);
            }

            // Reflection component
            CRTVector Rdir = I - N * (2.0f * I.dot(N));
            CRTRay reflectRay(hitPoint + N * shadowBias, Rdir.normalize());
            CRTColor C_reflect = traceRayWithBVH(reflectRay, scene, materials, lights, settings, accTree, depth + 1);

            // Refraction component
            CRTVector Tdir;
            CRTColor C_refract(0, 0, 0);
            
            if (Refract(I, N, eta1, eta2, Tdir)) {
                CRTRay refractRay(hitPoint - N * refractionBias, Tdir);
                C_refract = traceRayWithBVH(refractRay, scene, materials, lights, settings, accTree, depth + 1);
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

// Main rendering function with BVH acceleration
void renderTriangleSceneWithBVH(
    const std::vector<CRTTriangle>& triangles,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTSettings& settings,
    const CRTCamera& camera,
    const CRTAccTree& accTree,
    const std::string& filename
) {
    int W = settings.resolutionWidth;
    int H = settings.resolutionHeight;
    CRTColor bg = settings.backgroundColor;

    std::vector<std::vector<CRTColor>> image(H, std::vector<CRTColor>(W, bg));

    // Render each pixel
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            CRTRay primary = CRTRay::generatePrimaryRay(x, y, W, H, camera);
            image[y][x] = traceRayWithBVH(primary, triangles, materials, lights, settings, accTree, 0);
        }
        
        // Optional: Print progress
        if (y % (H / 10) == 0) {
            float progress = static_cast<float>(y) / H * 100.0f;
            std::cout << "Rendering progress: " << progress << "%" << std::endl;
        }
    }

    // Save to PPM format
    saveToPPM(filename, image);
}

// Bucket-based rendering with BVH acceleration
void renderTriangleSceneWithBVHBuckets(
    const std::vector<CRTTriangle>& triangles,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTSettings& settings,
    const CRTCamera& camera,
    const CRTAccTree& accTree,
    const std::string& filename
) {
    int W = settings.resolutionWidth;
    int H = settings.resolutionHeight;
    int bucketSize = settings.bucketSize;
    CRTColor bg = settings.backgroundColor;

    std::vector<std::vector<CRTColor>> image(H, std::vector<CRTColor>(W, bg));

    // Calculate number of buckets
    int bucketsX = (W + bucketSize - 1) / bucketSize;
    int bucketsY = (H + bucketSize - 1) / bucketSize;

    std::cout << "Rendering with " << bucketsX << "x" << bucketsY << " buckets of size " 
              << bucketSize << "x" << bucketSize << std::endl;

    // Render each bucket
    for (int bucketY = 0; bucketY < bucketsY; ++bucketY) {
        for (int bucketX = 0; bucketX < bucketsX; ++bucketX) {
            // Calculate bucket boundaries
            int startX = bucketX * bucketSize;
            int endX = std::min(startX + bucketSize, W);
            int startY = bucketY * bucketSize;
            int endY = std::min(startY + bucketSize, H);

            // Render pixels in this bucket
            for (int y = startY; y < endY; ++y) {
                for (int x = startX; x < endX; ++x) {
                    CRTRay primary = CRTRay::generatePrimaryRay(x, y, W, H, camera);
                    image[y][x] = traceRayWithBVH(primary, triangles, materials, lights, settings, accTree, 0);
                }
            }

            // Optional: Print bucket progress
            int currentBucket = bucketY * bucketsX + bucketX + 1;
            int totalBuckets = bucketsX * bucketsY;
            if (currentBucket % (totalBuckets / 20) == 0 || currentBucket == totalBuckets) {
                float progress = static_cast<float>(currentBucket) / totalBuckets * 100.0f;
                std::cout << "Bucket progress: " << progress << "% (" 
                          << currentBucket << "/" << totalBuckets << ")" << std::endl;
            }
        }
    }

    // Save to PPM format
    saveToPPM(filename, image);
}

// Utility function to save image in PPM format
void saveToPPM(const std::string& filename, const std::vector<std::vector<CRTColor>>& image) {
    if (image.empty() || image[0].empty()) {
        std::cerr << "Error: Empty image data" << std::endl;
        return;
    }

    int width = static_cast<int>(image[0].size());
    int height = static_cast<int>(image.size());

    std::ofstream ppmFile(filename);
    if (!ppmFile.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing" << std::endl;
        return;
    }

    ppmFile << "P3\n" << width << " " << height << "\n255\n";

    for (const auto& row : image) {
        for (const auto& pixel : row) {
            ppmFile << pixel.r << " " << pixel.g << " " << pixel.b << " ";
        }
        ppmFile << "\n";
    }

    ppmFile.close();
    std::cout << "Image saved to: " << filename << std::endl;
}


// BVH tree statistics and debugging
void printBVHStatistics(const CRTAccTree& accTree) {
    auto stats = accTree.getStatistics();
    
    std::cout << "\n=== BVH Tree Statistics ===" << std::endl;
    std::cout << "Total nodes: " << stats.totalNodes << std::endl;
    std::cout << "Leaf nodes: " << stats.leafNodes << std::endl;
    std::cout << "Internal nodes: " << (stats.totalNodes - stats.leafNodes) << std::endl;
    std::cout << "Maximum depth reached: " << stats.maxDepthReached << std::endl;
    std::cout << "Total triangles in leaves: " << stats.totalTriangles << std::endl;
    std::cout << "Average triangles per leaf: " << stats.averageTrianglesPerLeaf << std::endl;
    std::cout << "==========================\n" << std::endl;
}

// Performance comparison function
struct RenderingStats {
    double buildTime = 0.0;
    double renderTime = 0.0;
    int totalRayTests = 0;
    int totalIntersectionTests = 0;
};

// Function to compare BVH vs brute force rendering performance
void compareRenderingPerformance(
    const std::vector<CRTTriangle>& triangles,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTSettings& settings,
    const CRTCamera& camera
) {
    std::cout << "\n=== Performance Comparison ===" << std::endl;
    std::cout << "Scene complexity: " << triangles.size() << " triangles" << std::endl;
    
    // Build BVH and measure time
    auto buildStart = std::chrono::high_resolution_clock::now();
    CRTAccTree accTree;
    accTree.build(triangles, 32, 8);
    auto buildEnd = std::chrono::high_resolution_clock::now();
    
    double buildTime = std::chrono::duration<double>(buildEnd - buildStart).count();
    std::cout << "BVH build time: " << buildTime << " seconds" << std::endl;
    
    printBVHStatistics(accTree);
    
    std::cout << "Ready for rendering with BVH acceleration..." << std::endl;
}
