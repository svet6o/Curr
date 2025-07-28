#pragma once

#include <vector>
#include <thread>
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

                  
inline void saveToPPMFromBufferReg(const std::string& filename, const float* imageBuffer, int width, int height) {
    std::ofstream ppmFile(filename);
    ppmFile << "P3\n" << width << " " << height << "\n255\n";

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = (y * width + x) * 3;
            int r = std::min(255, std::max(0, int(imageBuffer[idx] * 255.0f)));
            int g = std::min(255, std::max(0, int(imageBuffer[idx + 1] * 255.0f)));
            int b = std::min(255, std::max(0, int(imageBuffer[idx + 2] * 255.0f)));

            ppmFile << r << " " << g << " " << b << " ";
        }
        ppmFile << "\n";
    }
}

// Рендерира регион от изображението (xStart..xEnd, yStart..yEnd)
inline void renderRegion(
    float* imageBuffer,
    int width, int height,
    int xStart, int xEnd,
    int yStart, int yEnd,
    const std::vector<CRTTriangle>& triangles,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTCamera& camera,
    const CRTSettings& settings
) {
    for (int y = yStart; y < yEnd; ++y) {
        for (int x = xStart; x < xEnd; ++x) {
            CRTRay primary = CRTRay::generatePrimaryRay(x, y, width, height, camera);
            CRTColor color = traceRay(primary, triangles, materials, lights, settings, 0, false, 1e-4f, 1e-4f);

            int idx = (y * width + x) * 3;
            imageBuffer[idx]     = color.r / 255.0f;
            imageBuffer[idx + 1] = color.g / 255.0f;
            imageBuffer[idx + 2] = color.b / 255.0f;
        }
    }
}

// Главна функция за мултитрединг рендеринг
inline void renderTriangleSceneMultithreadedRegions(
    const std::vector<CRTTriangle>& triangles,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTCamera& camera,
    const CRTSettings& settings,
    const std::string& filename
) {
    int W = settings.resolutionWidth;
    int H = settings.resolutionHeight;
    int numThreads = std::thread::hardware_concurrency();
    if (numThreads == 0) numThreads = 4;

    int numRegionsPerDim = (int)std::sqrt(numThreads);
    if (numRegionsPerDim * numRegionsPerDim < numThreads) numRegionsPerDim++;

    std::vector<float> imageBuffer(H * W * 3, 0.0f);

    std::vector<std::thread> threads;

    int regionWidth = W / numRegionsPerDim;
    int regionHeight = H / numRegionsPerDim;

    for (int ry = 0; ry < numRegionsPerDim; ++ry) {
        for (int rx = 0; rx < numRegionsPerDim; ++rx) {
            int xStart = rx * regionWidth;
            int yStart = ry * regionHeight;
            int xEnd = (rx == numRegionsPerDim - 1) ? W : xStart + regionWidth;
            int yEnd = (ry == numRegionsPerDim - 1) ? H : yStart + regionHeight;

            threads.emplace_back(renderRegion,
                                 imageBuffer.data(),
                                 W, H,
                                 xStart, xEnd,
                                 yStart, yEnd,
                                 std::ref(triangles),
                                 std::ref(materials),
                                 std::ref(lights),
                                 std::ref(camera),
                                 std::ref(settings));
        }
    }

    for (auto& t : threads) {
        if (t.joinable())
            t.join();
    }

    saveToPPMFromBufferReg(filename, imageBuffer.data(), W, H);
}
