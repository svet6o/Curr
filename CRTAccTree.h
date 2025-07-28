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

// Forward declaration of trace function
static bool Refract(const CRTVector& I, const CRTVector& N, float eta1, float eta2, CRTVector& refracted);
static float FresnelSchlick(const CRTVector& I, const CRTVector& N, float ior);

// Acceleration tree node
struct CRTAccNode {
    CRTAABB boundingBox;
    int child0Idx = -1;
    int child1Idx = -1;
    std::vector<int> triangleIndices; // Indices into the main triangle array
    
    CRTAccNode() = default;
    CRTAccNode(const CRTAABB& bbox) : boundingBox(bbox) {}
    
    bool isLeaf() const {
        return child0Idx == -1 && child1Idx == -1;
    }
};

// Main acceleration tree class
class CRTAccTree {

protected:
std::vector<CRTAccNode> nodes;
private:
    int maxDepth;
    int maxBoxTrianglesCount;
    
public:
    CRTAccTree() : maxDepth(20), maxBoxTrianglesCount(8) {}
    
    void setMaxDepth(int depth) { maxDepth = depth; }
    void setMaxTrianglesPerNode(int count) { maxBoxTrianglesCount = count; }
    
    // Add a new node and return its index
    int addNode(const CRTAABB& bbox, int child0, int child1, const std::vector<int>& triangles) {
        nodes.emplace_back(bbox);
        int idx = static_cast<int>(nodes.size() - 1);
        nodes[idx].child0Idx = child0;
        nodes[idx].child1Idx = child1;
        nodes[idx].triangleIndices = triangles;
        return idx;
    }
    
    struct SplitResult {
        CRTAABB leftAABB, rightAABB;
        std::vector<int> leftTriangles, rightTriangles;
        float cost;
    };
    
    SplitResult findBestSplit(const CRTAABB& nodeAABB, const std::vector<int>& triangleIndices,
                              const std::vector<CRTTriangle>& allTriangles) const {
        SplitResult bestSplit;
        bestSplit.cost = std::numeric_limits<float>::max();
        
        // Try all 3 axes
        for (int axis = 0; axis < 3; ++axis) {
            // Collect triangle centroids for this axis
            std::vector<std::pair<float, int>> centroids;
            centroids.reserve(triangleIndices.size());
            
            for (int triIdx : triangleIndices) {
                const CRTTriangle& tri = allTriangles[triIdx];
                CRTVector centroid = (tri.getVertex(0) + tri.getVertex(1) + tri.getVertex(2)) * (1.0f/3.0f);
                float coord = (axis == 0) ? centroid.getX() : (axis == 1) ? centroid.getY() : centroid.getZ();
                centroids.emplace_back(coord, triIdx);
            }
            
            // Sort by centroid coordinate
            std::sort(centroids.begin(), centroids.end());
            
            // Try different split positions
            for (size_t i = 1; i < centroids.size(); ++i) {
                // Split at position i (first i triangles go left)
                std::vector<int> leftTris, rightTris;
                leftTris.reserve(i);
                rightTris.reserve(centroids.size() - i);
                
                for (size_t j = 0; j < i; ++j) {
                    leftTris.push_back(centroids[j].second);
                }
                for (size_t j = i; j < centroids.size(); ++j) {
                    rightTris.push_back(centroids[j].second);
                }
                
                // Calculate AABBs for each side
                CRTAABB leftAABB, rightAABB;
                for (int idx : leftTris) {
                    leftAABB.expand(CRTAABB::fromTriangle(allTriangles[idx]));
                }
                for (int idx : rightTris) {
                    rightAABB.expand(CRTAABB::fromTriangle(allTriangles[idx]));
                }
                
                // Calculate SAH cost
                float leftArea = leftAABB.getSurfaceArea();
                float rightArea = rightAABB.getSurfaceArea();
                float totalArea = nodeAABB.getSurfaceArea();
                
                float cost = 1.0f + (leftArea / totalArea) * leftTris.size() + 
                                   (rightArea / totalArea) * rightTris.size();
                
                if (cost < bestSplit.cost) {
                    bestSplit.cost = cost;
                    bestSplit.leftAABB = leftAABB;
                    bestSplit.rightAABB = rightAABB;
                    bestSplit.leftTriangles = std::move(leftTris);
                    bestSplit.rightTriangles = std::move(rightTris);
                }
            }
        }
        
        return bestSplit;
    }
    
    // AABB-triangle intersection algorithm
    bool aabbTriangleIntersect(const CRTAABB& nodeAABB, const CRTAABB& triAABB) const {
        // If the boxes do not intersect on one of the axis, there isn't an intersection
        
        // X axis
        if (triAABB.minPoint.getX() > nodeAABB.maxPoint.getX()) return false;
        if (triAABB.maxPoint.getX() < nodeAABB.minPoint.getX()) return false;
        
        // Y axis
        if (triAABB.minPoint.getY() > nodeAABB.maxPoint.getY()) return false;
        if (triAABB.maxPoint.getY() < nodeAABB.minPoint.getY()) return false;
        
        // Z axis
        if (triAABB.minPoint.getZ() > nodeAABB.maxPoint.getZ()) return false;
        if (triAABB.maxPoint.getZ() < nodeAABB.minPoint.getZ()) return false;
        
        return true;
    }
    
    // Recursive tree building function
    void buildAccTree(int parentIdx, int depth, const std::vector<int>& triangleIndices, 
                      const std::vector<CRTTriangle>& allTriangles) {
        
        // Check termination conditions
        if (depth >= maxDepth || 
            static_cast<int>(triangleIndices.size()) <= maxBoxTrianglesCount ||
            triangleIndices.size() <= 1) {
            nodes[parentIdx].triangleIndices = triangleIndices;
            return;
        }
        
        // Find the best split
        SplitResult split = findBestSplit(nodes[parentIdx].boundingBox, triangleIndices, allTriangles);
        
        // Check if split is worthwhile (avoid bad splits)
        if (split.leftTriangles.empty() || split.rightTriangles.empty() ||
            split.cost >= triangleIndices.size()) {
            // Split didn't improve things, make this a leaf
            nodes[parentIdx].triangleIndices = triangleIndices;
            return;
        }
        
        // Create child nodes
        int child0Idx = addNode(split.leftAABB, -1, -1, std::vector<int>());
        int child1Idx = addNode(split.rightAABB, -1, -1, std::vector<int>());
        
        nodes[parentIdx].child0Idx = child0Idx;
        nodes[parentIdx].child1Idx = child1Idx;
        
        // Recursively build children
        buildAccTree(child0Idx, depth + 1, split.leftTriangles, allTriangles);
        buildAccTree(child1Idx, depth + 1, split.rightTriangles, allTriangles);
    }
    
    // Main build function
    void build(const std::vector<CRTTriangle>& triangles, int maxTriangles = 8, int maxTreeDepth = 20) {
        maxBoxTrianglesCount = maxTriangles;
        maxDepth = maxTreeDepth;
        nodes.clear();
        
        if (triangles.empty()) return;
        
        // Create AABB for the scene
        CRTAABB sceneAABB = CRTAABB::fromTriangles(triangles);
        
        // Gather all triangles in an array
        std::vector<int> allTriangleIndices;
        allTriangleIndices.reserve(triangles.size());
        for (size_t i = 0; i < triangles.size(); ++i) {
            allTriangleIndices.push_back(static_cast<int>(i));
        }
        
        // Create root node
        int rootIdx = addNode(sceneAABB, -1, -1, std::vector<int>());
        
        // Recursively build the acceleration tree
        buildAccTree(rootIdx, 0, allTriangleIndices, triangles);
        
        std::cout << "Acceleration tree built with " << nodes.size() << " nodes\n";
    }
    
    // Tree traversal for ray intersection
  bool intersect(const CRTRay& ray, const std::vector<CRTTriangle>& triangles,
                   float& closestT, int& hitTriangleIdx, float& hitU, float& hitV,
                   CRTVector& hitNormal, CRTVector& hitUV) const {
        if (nodes.empty()) return false;
        
        closestT = std::numeric_limits<float>::max();
        hitTriangleIdx = -1;
        bool hasHit = false;
        
        intersectNode(0, ray, triangles, closestT, hitTriangleIdx, hitU, hitV, hitNormal, hitUV, hasHit);
        
        return hasHit;
    }
    
private:
     void intersectNode(int nodeIdx, const CRTRay& ray, const std::vector<CRTTriangle>& triangles,
                       float& closestT, int& hitTriangleIdx, float& hitU, float& hitV,
                       CRTVector& hitNormal, CRTVector& hitUV, bool& hasHit) const {
        
        const CRTAccNode& node = nodes[nodeIdx];
        
        // Test ray against node's AABB with distance constraint
        float tNear, tFar;
        if (!node.boundingBox.intersect(ray, tNear, tFar)) {
            return;
        }
        
        // Early exit if AABB is farther than current closest hit
        if (tNear > closestT) {
            return;
        }
        
        if (node.isLeaf()) {
            // Test against all triangles in this leaf
            for (int triIdx : node.triangleIndices) {
                float t, u, v;
                CRTVector normal, uv;
                
                if (triangles[triIdx].intersect(ray, t, u, v, normal, uv)) {
                    if (t > 1e-4f && t < closestT) { // Add epsilon check
                        closestT = t;
                        hitTriangleIdx = triIdx;
                        hitU = u;
                        hitV = v;
                        hitNormal = normal;
                        hitUV = uv;
                        hasHit = true;
                    }
                }
            }
        } else {
            // Test children in order of distance (front-to-back traversal)
            bool traverseChild0First = true;
            
            if (node.child0Idx != -1 && node.child1Idx != -1) {
                // Calculate approximate distances to child AABBs
                float dist0 = (nodes[node.child0Idx].boundingBox.getCenter() - ray.getOrigin()).lengthSquared();
                float dist1 = (nodes[node.child1Idx].boundingBox.getCenter() - ray.getOrigin()).lengthSquared();
                traverseChild0First = (dist0 <= dist1);
            }
            
            if (traverseChild0First) {
                if (node.child0Idx != -1) {
                    intersectNode(node.child0Idx, ray, triangles, closestT, hitTriangleIdx, 
                                 hitU, hitV, hitNormal, hitUV, hasHit);
                }
                if (node.child1Idx != -1) {
                    intersectNode(node.child1Idx, ray, triangles, closestT, hitTriangleIdx, 
                                 hitU, hitV, hitNormal, hitUV, hasHit);
                }
            } else {
                if (node.child1Idx != -1) {
                    intersectNode(node.child1Idx, ray, triangles, closestT, hitTriangleIdx, 
                                 hitU, hitV, hitNormal, hitUV, hasHit);
                }
                if (node.child0Idx != -1) {
                    intersectNode(node.child0Idx, ray, triangles, closestT, hitTriangleIdx, 
                                 hitU, hitV, hitNormal, hitUV, hasHit);
                }
            }
        }
    }

public:
    // Shadow ray intersection (early termination)
    bool intersectShadow(const CRTRay& ray, const std::vector<CRTTriangle>& triangles,
                         const std::vector<CRTMaterial>& materials, float maxDistance,
                         int ignoreTriangleIdx = -1) const {
        if (nodes.empty()) return false;
        
        return intersectShadowNode(0, ray, triangles, materials, maxDistance, ignoreTriangleIdx);
    }
    
private:
    bool intersectShadowNode(int nodeIdx, const CRTRay& ray, const std::vector<CRTTriangle>& triangles,
                             const std::vector<CRTMaterial>& materials, float maxDistance,
                             int ignoreTriangleIdx) const {
        
        const CRTAccNode& node = nodes[nodeIdx];
        
        // Test ray against node's AABB with distance constraint
        float tNear, tFar;
        if (!node.boundingBox.intersect(ray, tNear, tFar)) {
            return false;
        }
        
        // If intersection is beyond the light, no shadow
        if (tNear > maxDistance) {
            return false;
        }
        
        if (node.isLeaf()) {
            // Test against all triangles in this leaf
            for (int triIdx : node.triangleIndices) {
                if (triIdx == ignoreTriangleIdx) continue;
                
                float t, u, v;
                CRTVector normal, uv;
                
                if (triangles[triIdx].intersect(ray, t, u, v, normal, uv)) {
                    if (t > 1e-4f && t < maxDistance) {
                        // Check if this is a refractive material (transparent)
                        int matIdx = triangles[triIdx].getMaterialIndex();
                        if (matIdx < static_cast<int>(materials.size()) && 
                            materials[matIdx].type == CRTMaterial::Type::REFRACTIVE) {
                            continue; // Skip transparent materials for shadows
                        }
                        return true; // Shadow ray blocked
                    }
                }
            }
        } else {
            // Recursively test child nodes
            if (node.child0Idx != -1) {
                if (intersectShadowNode(node.child0Idx, ray, triangles, materials, maxDistance, ignoreTriangleIdx)) {
                    return true;
                }
            }
            if (node.child1Idx != -1) {
                if (intersectShadowNode(node.child1Idx, ray, triangles, materials, maxDistance, ignoreTriangleIdx)) {
                    return true;
                }
            }
        }
        
        return false;
    }

public:
    // Get statistics
    void printStatistics() const {
        int leafNodes = 0;
        int totalTriangles = 0;
        int maxTrianglesInLeaf = 0;
        
        for (const auto& node : nodes) {
            if (node.isLeaf()) {
                leafNodes++;
                int triCount = static_cast<int>(node.triangleIndices.size());
                totalTriangles += triCount;
                maxTrianglesInLeaf = std::max(maxTrianglesInLeaf, triCount);
            }
        }
        
        std::cout << "Acceleration Tree Statistics:\n";
        std::cout << "  Total nodes: " << nodes.size() << "\n";
        std::cout << "  Leaf nodes: " << leafNodes << "\n";
        std::cout << "  Average triangles per leaf: " << (leafNodes > 0 ? totalTriangles / leafNodes : 0) << "\n";
        std::cout << "  Max triangles in leaf: " << maxTrianglesInLeaf << "\n";
    }
};
