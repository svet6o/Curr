#pragma once

#include "CRTAccTree.h"
#include <cmath>
#include <algorithm>
#include <iostream>

// Generic clamp utility
template <typename T>
constexpr const T& clamp(const T& value, const T& low, const T& high);

class AutoAccTree : public CRTAccTree {
private:
    struct SceneAnalysis {
        size_t triangleCount = 0;
        float sceneVolume = 0.0f;
        float averageTriangleArea = 0.0f;
        float triangleDensity = 0.0f;
        CRTAABB sceneBounds;
        float aspectRatio = 0.0f;
        bool isLargeScene = false;
        bool isDenseScene = false;
    };

    SceneAnalysis analyzeScene(const std::vector<CRTTriangle>& triangles) const {
        SceneAnalysis analysis;
        analysis.triangleCount = triangles.size();
        if (triangles.empty()) return analysis;

        // Compute bounds and volume
        analysis.sceneBounds = CRTAABB::fromTriangles(triangles);
        CRTVector size = analysis.sceneBounds.getSize();
        analysis.sceneVolume = size.getX() * size.getY() * size.getZ();

        float maxDim = std::max({size.getX(), size.getY(), size.getZ()});
        float minDim = std::min({size.getX(), size.getY(), size.getZ()});
        analysis.aspectRatio = (minDim > 0) ? maxDim / minDim : 1.0f;

        // Average area
        float totalArea = 0.0f;
        for (const auto& tri : triangles) {
            CRTVector v0 = tri.getVertex(0);
            CRTVector v1 = tri.getVertex(1);
            CRTVector v2 = tri.getVertex(2);
            float area = 0.5f * (v1 - v0).cross(v2 - v0).length();
            totalArea += area;
        }
        analysis.averageTriangleArea = totalArea / triangles.size();

        // Density
        analysis.triangleDensity = (analysis.sceneVolume > 0.0f)
            ? static_cast<float>(triangles.size()) / analysis.sceneVolume
            : 0.0f;

        // Flags
        analysis.isLargeScene = (triangles.size() > 10000) || (analysis.sceneVolume > 1000.0f);
        analysis.isDenseScene = analysis.triangleDensity > 0.1f;
        return analysis;
    }

    int calculateOptimalMaxTriangles(const SceneAnalysis& A) const {
        int base = 8;
        if (A.triangleCount < 100) base = 16;
        else if (A.triangleCount < 1000) base = 12;
        else if (A.triangleCount < 10000) base = 8;
        else if (A.triangleCount < 100000) base = 6;
        else base = 4;
        if (A.isDenseScene) base = std::max(4, base - 2);
        if (A.aspectRatio > 5.0f) base = std::max(6, base + 2);
        return clamp(base, 2, 32);
    }

    int calculateOptimalMaxDepth(const SceneAnalysis& A) const {
        int depth = static_cast<int>(std::ceil(std::log2(static_cast<float>(A.triangleCount)))) + 4;
        if (A.isLargeScene) depth += 3;
        if (A.isDenseScene) depth += 2;
        if (A.aspectRatio > 10.0f) depth += 2;
        if (A.triangleCount > 1000000) depth = std::min(depth, 28);
        return clamp(depth, 8, 32);
    }

public:
    AutoAccTree() = default;

    void autoBuild(const std::vector<CRTTriangle>& triangles) {
        if (triangles.empty()) {
            std::cout << "AutoAccTree: Empty scene, skipping build\n";
            return;
        }
        auto A = analyzeScene(triangles);
        int maxT = calculateOptimalMaxTriangles(A);
        int maxD = calculateOptimalMaxDepth(A);
        setMaxTrianglesPerNode(maxT);
        setMaxDepth(maxD);
        std::cout << "AutoAccTree: maxTriangles=" << maxT
                  << " maxDepth=" << maxD << "\n";
        CRTAccTree::build(triangles, maxT, maxD);
    }
};
