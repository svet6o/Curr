#pragma once
#include <vector>
#include <thread>
#include <mutex>
#include <fstream>
#include <algorithm>
#include <cmath>

#include "CRTColor.h"
#include "CRTRay.h"
#include "CRTTriangle.h"
#include "CRTMaterial.h"
#include "CRTLight.h"
#include "CRTCamera.h"
#include "CRTSettings.h"

CRTColor traceRay(
      const CRTRay& ray,
    const std::vector<CRTTriangle>& scene,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTSettings& settings,
    int depth,
    bool isShadowRay,
    float shadowBias,
    float refractionBias
);


struct Bucket {
    int x0, y0, x1, y1;
};


inline void saveToPPMFromBufferBuck(const std::string& filename,
                                const float* imageBuffer,
                                int width, int height)
{
    std::ofstream ppmFile(filename);
    ppmFile << "P3\n" << width << " " << height << "\n255\n";

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = (y * width + x) * 3;
            int r = std::min(255, std::max(0, (int)std::round(imageBuffer[idx + 0])));
            int g = std::min(255, std::max(0, (int)std::round(imageBuffer[idx + 1])));
            int b = std::min(255, std::max(0, (int)std::round(imageBuffer[idx + 2])));
            ppmFile << r << " " << g << " " << b << " ";
        }
        ppmFile << "\n";
    }
}

inline void renderBucket(
    const Bucket& b,
    float* imageBuffer,
    int width, int height,
    const std::vector<CRTTriangle>& triangles,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTCamera& camera,
    const CRTSettings& settings
) {
    for (int y = b.y0; y < b.y1; ++y) {
        for (int x = b.x0; x < b.x1; ++x) {
            CRTRay primary = CRTRay::generatePrimaryRay(x, y, width, height, camera);
            CRTColor color = traceRay(primary, triangles, materials, lights, settings, 0, false, 1e-4f, 1e-4f);

            const int idx = (y * width + x) * 3;
            imageBuffer[idx + 0] = (float)color.r;
            imageBuffer[idx + 1] = (float)color.g;
            imageBuffer[idx + 2] = (float)color.b;
        }
    }
}


inline void renderTriangleSceneBuckets(
    const std::vector<CRTTriangle>& triangles,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTCamera& camera,
    const CRTSettings& settings,
    const std::string& filename
) {
    const int W = settings.resolutionWidth;
    const int H = settings.resolutionHeight;

    // 1) Буфер за изображението
    std::vector<float> imageBuffer(W * H * 3, 0.0f);

    // 2) Подготовка на bucket-и
    const int bucketSize = (settings.bucketSize > 0) ? settings.bucketSize : 32;
    std::vector<Bucket> buckets;
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

    const unsigned hw = std::thread::hardware_concurrency();
    const int numThreads = (hw == 0) ? 4 : (int)hw;

    std::mutex bucketsMutex;

    auto worker = [&]() {
        for (;;) {
            Bucket bucket;
            {
                std::lock_guard<std::mutex> lock(bucketsMutex);
                if (buckets.empty())
                    return;
                bucket = buckets.back();
                buckets.pop_back();
            }
        
            renderBucket(bucket,
                         imageBuffer.data(),
                         W, H,
                         triangles, materials, lights,
                         camera, settings);
        }
    };

    std::vector<std::thread> threads;
    threads.reserve(numThreads);
    for (int i = 0; i < numThreads; ++i)
        threads.emplace_back(worker);

    for (auto& t : threads)
        t.join();

    saveToPPMFromBufferBuck(filename, imageBuffer.data(), W, H);
}
