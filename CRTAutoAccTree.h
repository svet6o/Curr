#pragma once

#include "CRTAccTree.h"
#include <cmath>
#include <algorithm>

template <typename T>
constexpr const T &clamp(const T &value, const T &min, const T &max);

class AutoAccTree : public CRTAccTree {
private:
    struct SceneAnalysis {
        size_t triangleCount;
        float sceneVolume;
        float averageTriangleArea;
        float triangleDensity;
        CRTAABB sceneBounds;
        float aspectRatio;
        bool isLargeScene;
        bool isDenseScene;
    };

    // Analyze scene characteristics
    SceneAnalysis analyzeScene(const std::vector<CRTTriangle>& triangles) const {
        SceneAnalysis analysis;
        analysis.triangleCount = triangles.size();
        
        if (triangles.empty()) {
            return analysis;
        }
        
        // Calculate scene bounds
        analysis.sceneBounds = CRTAABB::fromTriangles(triangles);
        CRTVector sceneSize = analysis.sceneBounds.getSize();
        
        // Calculate scene volume and aspect ratio
        analysis.sceneVolume = sceneSize.getX() * sceneSize.getY() * sceneSize.getZ();
        float maxDim = std::max({sceneSize.getX(), sceneSize.getY(), sceneSize.getZ()});
        float minDim = std::min({sceneSize.getX(), sceneSize.getY(), sceneSize.getZ()});
        analysis.aspectRatio = (minDim > 0) ? maxDim / minDim : 1.0f;
        
        // Calculate average triangle area
        float totalArea = 0.0f;
        for (const auto& triangle : triangles) {
            // Simple area calculation using cross product
            CRTVector v0 = triangle.getVertex(0);
            CRTVector v1 = triangle.getVertex(1);
            CRTVector v2 = triangle.getVertex(2);
            
            CRTVector edge1 = v1 - v0;
            CRTVector edge2 = v2 - v0;
            CRTVector cross = edge1.cross(edge2);
            totalArea += cross.length() * 0.5f;
        }
        analysis.averageTriangleArea = totalArea / static_cast<float>(triangles.size());
        
        // Calculate triangle density (triangles per unit volume)
        analysis.triangleDensity = (analysis.sceneVolume > 0) ? 
            static_cast<float>(triangles.size()) / analysis.sceneVolume : 0.0f;
        
        // Scene classification
        analysis.isLargeScene = (triangles.size() > 10000) || (analysis.sceneVolume > 1000.0f);
        analysis.isDenseScene = analysis.triangleDensity > 0.1f;
        
        return analysis;
    }
    
    // Determine optimal max triangles per node
    int calculateOptimalMaxTriangles(const SceneAnalysis& analysis) const {
        int baseTriangles = 8; // Default baseline
        
        // Scale based on triangle count
        if (analysis.triangleCount < 100) {
            baseTriangles = 16; // Allow more triangles per leaf for small scenes
        } else if (analysis.triangleCount < 1000) {
            baseTriangles = 12;
        } else if (analysis.triangleCount < 10000) {
            baseTriangles = 8;
        } else if (analysis.triangleCount < 100000) {
            baseTriangles = 6;
        } else {
            baseTriangles = 4; // Fewer triangles per leaf for very large scenes
        }
        
        // Adjust for scene density
        if (analysis.isDenseScene) {
            baseTriangles = std::max(4, baseTriangles - 2); // Reduce for dense scenes
        }
        
        // Adjust for aspect ratio (elongated scenes might benefit from different partitioning)
        if (analysis.aspectRatio > 5.0f) {
            baseTriangles = std::max(6, baseTriangles + 2); // Allow more triangles for elongated scenes
        }
        
        // Ensure reasonable bounds
        return clamp(baseTriangles, 2, 32);
    }
    
    // Determine optimal max depth
    int calculateOptimalMaxDepth(const SceneAnalysis& analysis) const {
        // Base depth calculation using logarithm of triangle count
        int baseDepth = static_cast<int>(std::ceil(std::log2(static_cast<float>(analysis.triangleCount)))) + 4;
        
        // Adjust for scene characteristics
        if (analysis.isLargeScene) {
            baseDepth += 3; // Allow deeper trees for large scenes
        }
        
        if (analysis.isDenseScene) {
            baseDepth += 2; // Need more subdivision for dense scenes
        }
        
        // Adjust for aspect ratio
        if (analysis.aspectRatio > 10.0f) {
            baseDepth += 2; // May need more depth for very elongated scenes
        }
        
        // Memory and performance considerations
        if (analysis.triangleCount > 1000000) {
            baseDepth = std::min(baseDepth, 28); // Cap depth for very large scenes to prevent memory issues
        }
        
        // Ensure reasonable bounds
        return clamp(baseDepth, 8, 32);
    }
    
    // Performance-based heuristics
    struct PerformanceHeuristics {
        float memoryConstraint;
        float buildTimeConstraint;
        bool favorMemory;
        bool favorSpeed;
        
        PerformanceHeuristics() : 
            memoryConstraint(1.0f), buildTimeConstraint(1.0f), 
            favorMemory(false), favorSpeed(true) {}
    };
    
    void applyPerformanceHeuristics(int& maxTriangles, int& maxDepth, 
                                   const SceneAnalysis& analysis,
                                   const PerformanceHeuristics& perf) const {
        if (perf.favorMemory) {
            // Reduce memory usage by allowing more triangles per leaf (fewer nodes)
            maxTriangles = std::min(maxTriangles + 4, 24);
            maxDepth = std::max(maxDepth - 2, 8);
        }
        
        if (perf.favorSpeed) {
            // Optimize for traversal speed
            if (analysis.triangleCount > 50000) {
                maxTriangles = std::max(maxTriangles - 2, 4);
                maxDepth = std::min(maxDepth + 1, 30);
            }
        }
    }

public:
    AutoAccTree() : CRTAccTree() {}
    
    // Auto-build with optimal parameters
    void autoBuild(const std::vector<CRTTriangle>& triangles) {
        if (triangles.empty()) {
            std::cout << "AutoAccTree: No triangles to build tree from\n";
            return;
        }
        
        // Analyze the scene
        SceneAnalysis analysis = analyzeScene(triangles);
        
        // Calculate optimal parameters
        int optimalMaxTriangles = calculateOptimalMaxTriangles(analysis);
        int optimalMaxDepth = calculateOptimalMaxDepth(analysis);
        
        // Apply performance heuristics
        PerformanceHeuristics perf; // Use default performance preferences
        applyPerformanceHeuristics(optimalMaxTriangles, optimalMaxDepth, analysis, perf);
        
        // Print analysis results
        printSceneAnalysis(analysis, optimalMaxTriangles, optimalMaxDepth);
        
        // Build the tree with calculated parameters
        build(triangles, optimalMaxTriangles, optimalMaxDepth);
    }
    
    // Auto-build with custom performance preferences
    void autoBuild(const std::vector<CRTTriangle>& triangles, 
                   bool favorMemoryOverSpeed, 
                   float memoryConstraint = 1.0f) {
        if (triangles.empty()) {
            std::cout << "AutoAccTree: No triangles to build tree from\n";
            return;
        }
        
        SceneAnalysis analysis = analyzeScene(triangles);
        int optimalMaxTriangles = calculateOptimalMaxTriangles(analysis);
        int optimalMaxDepth = calculateOptimalMaxDepth(analysis);
        
        // Custom performance heuristics
        PerformanceHeuristics perf;
        perf.favorMemory = favorMemoryOverSpeed;
        perf.favorSpeed = !favorMemoryOverSpeed;
        perf.memoryConstraint = memoryConstraint;
        
        applyPerformanceHeuristics(optimalMaxTriangles, optimalMaxDepth, analysis, perf);
        
        printSceneAnalysis(analysis, optimalMaxTriangles, optimalMaxDepth);
        build(triangles, optimalMaxTriangles, optimalMaxDepth);
    }
    
    // Get recommended parameters without building
    std::pair<int, int> getOptimalParameters(const std::vector<CRTTriangle>& triangles) const {
        if (triangles.empty()) {
            return {8, 20}; // Default fallback
        }
        
        SceneAnalysis analysis = analyzeScene(triangles);
        int optimalMaxTriangles = calculateOptimalMaxTriangles(analysis);
        int optimalMaxDepth = calculateOptimalMaxDepth(analysis);
        
        return {optimalMaxTriangles, optimalMaxDepth};
    }
    
    // Benchmark different parameter combinations
    struct BenchmarkResult {
        int maxTriangles;
        int maxDepth;
        size_t nodeCount;
        int avgTrianglesPerLeaf;
        int maxTrianglesInLeaf;
        float estimatedTraversalCost;
        float memoryUsage; // in MB
    };
    
    std::vector<BenchmarkResult> benchmarkParameters(const std::vector<CRTTriangle>& triangles,
                                                    const std::vector<std::pair<int, int>>& parameterSets) const {
        std::vector<BenchmarkResult> results;
        
        for (const auto& params : parameterSets) {
            // Create a temporary tree for testing
            AutoAccTree tempTree;
            tempTree.build(triangles, params.first, params.second);
            
            BenchmarkResult result;
            result.maxTriangles = params.first;
            result.maxDepth = params.second;
            result.nodeCount = tempTree.nodes.size();
            
            // Calculate statistics
            int leafNodes = 0;
            int totalTriangles = 0;
            int maxTrianglesInLeaf = 0;
            
            for (const auto& node : tempTree.nodes) {
                if (node.isLeaf()) {
                    leafNodes++;
                    int triCount = static_cast<int>(node.triangleIndices.size());
                    totalTriangles += triCount;
                    maxTrianglesInLeaf = std::max(maxTrianglesInLeaf, triCount);
                }
            }
            
            result.avgTrianglesPerLeaf = (leafNodes > 0) ? totalTriangles / leafNodes : 0;
            result.maxTrianglesInLeaf = maxTrianglesInLeaf;
            
            // Estimate traversal cost (simplified heuristic)
            float avgDepth = std::log2(static_cast<float>(leafNodes)) + 1;
            result.estimatedTraversalCost = avgDepth + static_cast<float>(result.avgTrianglesPerLeaf) * 0.1f;
            
            // Estimate memory usage (rough approximation)
            result.memoryUsage = static_cast<float>(result.nodeCount * sizeof(CRTAccNode)) / (1024.0f * 1024.0f);
            
            results.push_back(result);
        }
        
        return results;
    }

private:
    void printSceneAnalysis(const SceneAnalysis& analysis, int maxTriangles, int maxDepth) const {
        std::cout << "\n=== AutoAccTree Scene Analysis ===\n";
        std::cout << "Triangle count: " << analysis.triangleCount << "\n";
        std::cout << "Scene volume: " << analysis.sceneVolume << "\n";
        std::cout << "Average triangle area: " << analysis.averageTriangleArea << "\n";
        std::cout << "Triangle density: " << analysis.triangleDensity << "\n";
        std::cout << "Aspect ratio: " << analysis.aspectRatio << "\n";
        std::cout << "Large scene: " << (analysis.isLargeScene ? "Yes" : "No") << "\n";
        std::cout << "Dense scene: " << (analysis.isDenseScene ? "Yes" : "No") << "\n";
        std::cout << "\n=== Optimal Parameters ===\n";
        std::cout << "Max triangles per node: " << maxTriangles << "\n";
        std::cout << "Max tree depth: " << maxDepth << "\n";
        std::cout << "=====================================\n\n";
    }
};