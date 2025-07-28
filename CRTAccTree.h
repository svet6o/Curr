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
    
    // AABB split algorithm
    std::pair<CRTAABB, CRTAABB> splitAABB(const CRTAABB& aabbToSplit, int splitAxisIdx) const {
        // Calculate the middle point for the splitting
        float range = aabbToSplit.maxPoint.getX();
        float minVal = aabbToSplit.minPoint.getX();
        
        switch (splitAxisIdx) {
            case 0: // X axis
                range = aabbToSplit.maxPoint.getX() - aabbToSplit.minPoint.getX();
                minVal = aabbToSplit.minPoint.getX();
                break;
            case 1: // Y axis
                range = aabbToSplit.maxPoint.getY() - aabbToSplit.minPoint.getY();
                minVal = aabbToSplit.minPoint.getY();
                break;
            case 2: // Z axis
                range = aabbToSplit.maxPoint.getZ() - aabbToSplit.minPoint.getZ();
                minVal = aabbToSplit.minPoint.getZ();
                break;
        }
        
        float mid = range / 2.0f;
        float splitPlaneCoordinate = minVal + mid;
        
        // Create A and B to be the same as aabbToSplit
        CRTAABB A = aabbToSplit;
        CRTAABB B = aabbToSplit;
        
        // Update the maximum component of A for the splitting axis
        switch (splitAxisIdx) {
            case 0: // X axis
                A.maxPoint = CRTVector(splitPlaneCoordinate, A.maxPoint.getY(), A.maxPoint.getZ());
                B.minPoint = CRTVector(splitPlaneCoordinate, B.minPoint.getY(), B.minPoint.getZ());
                break;
            case 1: // Y axis
                A.maxPoint = CRTVector(A.maxPoint.getX(), splitPlaneCoordinate, A.maxPoint.getZ());
                B.minPoint = CRTVector(B.minPoint.getX(), splitPlaneCoordinate, B.minPoint.getZ());
                break;
            case 2: // Z axis
                A.maxPoint = CRTVector(A.maxPoint.getX(), A.maxPoint.getY(), splitPlaneCoordinate);
                B.minPoint = CRTVector(B.minPoint.getX(), B.minPoint.getY(), splitPlaneCoordinate);
                break;
        }
        
        return std::make_pair(A, B);
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
        if (depth >= maxDepth || static_cast<int>(triangleIndices.size()) <= maxBoxTrianglesCount) {
            nodes[parentIdx].triangleIndices = triangleIndices;
            return; // Stop the recursion
        }
        
        // Split the parent AABB box in two halves, alternating the split axis
        int splitAxis = depth % 3; // 3 axes: X, Y, Z
        auto [child0AABB, child1AABB] = splitAABB(nodes[parentIdx].boundingBox, splitAxis);
        
        std::vector<int> child0Triangles;
        std::vector<int> child1Triangles;
        
        // For each triangle in triangles
        for (int triIdx : triangleIndices) {
            const CRTTriangle& triangle = allTriangles[triIdx];
            CRTAABB triAABB = CRTAABB::fromTriangle(triangle);
            
            // If triangle AABB intersects child0AABB
            if (aabbTriangleIntersect(child0AABB, triAABB)) {
                child0Triangles.push_back(triIdx);
            }
            
            // If triangle AABB intersects child1AABB
            if (aabbTriangleIntersect(child1AABB, triAABB)) {
                child1Triangles.push_back(triIdx);
            }
        }
        
        // Create child nodes if they have triangles
        if (!child0Triangles.empty()) {
            int child0Idx = addNode(child0AABB, -1, -1, std::vector<int>());
            nodes[parentIdx].child0Idx = child0Idx;
            buildAccTree(child0Idx, depth + 1, child0Triangles, allTriangles);
        }
        
        if (!child1Triangles.empty()) {
            int child1Idx = addNode(child1AABB, -1, -1, std::vector<int>());
            nodes[parentIdx].child1Idx = child1Idx;
            buildAccTree(child1Idx, depth + 1, child1Triangles, allTriangles);
        }
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
        
        // Start traversal from root
        intersectNode(0, ray, triangles, closestT, hitTriangleIdx, hitU, hitV, hitNormal, hitUV, hasHit);
        
        return hasHit;
    }
    
private:
    void intersectNode(int nodeIdx, const CRTRay& ray, const std::vector<CRTTriangle>& triangles,
                       float& closestT, int& hitTriangleIdx, float& hitU, float& hitV,
                       CRTVector& hitNormal, CRTVector& hitUV, bool& hasHit) const {
        
        const CRTAccNode& node = nodes[nodeIdx];
        
        // Test ray against node's AABB
        if (!node.boundingBox.intersect(ray)) {
            return; // Ray doesn't hit this node's bounding box
        }
        
        if (node.isLeaf()) {
            // Test against all triangles in this leaf
            for (int triIdx : node.triangleIndices) {
                float t, u, v;
                CRTVector normal, uv;
                
                if (triangles[triIdx].intersect(ray, t, u, v, normal, uv)) {
                    if (t < closestT) {
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
            // Recursively test child nodes
            if (node.child0Idx != -1) {
                intersectNode(node.child0Idx, ray, triangles, closestT, hitTriangleIdx, 
                             hitU, hitV, hitNormal, hitUV, hasHit);
            }
            if (node.child1Idx != -1) {
                intersectNode(node.child1Idx, ray, triangles, closestT, hitTriangleIdx, 
                             hitU, hitV, hitNormal, hitUV, hasHit);
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
