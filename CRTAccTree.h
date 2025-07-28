#pragma once

#include "CRTAABB.h"
#include "CRTTriangle.h"
#include "CRTRay.h"
#include "CRTMaterial.h"
#include <vector>
#include <stack>
#include <limits>

// Intersection data structure to hold hit information
struct IntersectionData {
    int triangleIndex = -1;
    int materialIndex = -1;
    float u = 0.0f, v = 0.0f;
    CRTVector normal;
    CRTVector uv;
    CRTVector hitPoint;
    
    IntersectionData() = default;
};

// Node for the acceleration tree structure
struct CRTAccTreeNode {
    std::vector<CRTTriangle> triangles; ///< In case of a leaf node, list of triangles for intersection
    CRTAABB aabb; ///< Axis aligned bounding box for the sub space this node represents
    int children[2]; ///< The left and right indices for the node's children (-1 if no child)
    int parent; ///< The index of the parent node (-1 for root)
    
    // Constructor
    CRTAccTreeNode(const CRTAABB& nodeAABB, int parentIdx, int leftNodeIdx, int rightNodeIdx, 
                   const std::vector<CRTTriangle>& nodeTriangles)
        : aabb(nodeAABB), parent(parentIdx), triangles(nodeTriangles) {
        children[0] = leftNodeIdx;
        children[1] = rightNodeIdx;
    }
    
    // Default constructor
    CRTAccTreeNode() : parent(-1) {
        children[0] = children[1] = -1;
    }
    
    // Check if this is a leaf node
    bool isLeaf() const {
        return children[0] == -1 && children[1] == -1;
    }
    
    // Intersect the given ray with the triangles in this node (meaningful only for leaf nodes)
    void intersect(const CRTRay& ray, const float maxT, const std::vector<CRTMaterial>& materials,
                   float& t, float& minT, IntersectionData& data) const {
        
        if (!isLeaf()) return; // Only leaf nodes contain triangles
        
        for (size_t i = 0; i < triangles.size(); ++i) {
            float hitT, u, v;
            CRTVector normal, uv;
            
            if (triangles[i].intersect(ray, hitT, u, v, normal, uv)) {
                if (hitT > 1e-4f && hitT < t && hitT < maxT) {
                    t = hitT;
                    minT = hitT;
                    data.triangleIndex = static_cast<int>(i);
                    data.materialIndex = triangles[i].getMaterialIndex();
                    data.u = u;
                    data.v = v;
                    data.normal = normal;
                    data.uv = uv;
                    data.hitPoint = ray.pointAtParameter(hitT);
                }
            }
        }
    }
};

class CRTAccTree {
private:
    std::vector<CRTAccTreeNode> nodes;
    int rootIndex;
    int maxDepth;
    int maxBoxTrianglesCount;
    
public:
    CRTAccTree() : rootIndex(-1), maxDepth(8), maxBoxTrianglesCount(32) {}
    
    // Add a node to the tree and return its index
    int addNode(const CRTAABB& aabb, int parentIdx, int child0Idx, int child1Idx, 
                const std::vector<CRTTriangle>& triangles) {
        int nodeIndex = static_cast<int>(nodes.size());
        nodes.emplace_back(aabb, parentIdx, child0Idx, child1Idx, triangles);
        return nodeIndex;
    }
    
    // Build the acceleration tree
    void build(const std::vector<CRTTriangle>& triangles, int maxTrianglesPerLeaf = 32, int maxTreeDepth = 8) {
        maxBoxTrianglesCount = maxTrianglesPerLeaf;
        maxDepth = maxTreeDepth;
        nodes.clear();
        
        if (triangles.empty()) {
            rootIndex = -1;
            return;
        }
        
        // Create AABB for the entire scene
        CRTAABB sceneAABB = CRTAABB::fromTriangles(triangles);
        
        // Create root node
        rootIndex = addNode(sceneAABB, -1, -1, -1, std::vector<CRTTriangle>());
        
        // Recursively build the tree
        std::vector<CRTTriangle> allTriangles = triangles;
        buildAccTree(rootIndex, 0, allTriangles);
    }
    
    // Recursive tree building function
    void buildAccTree(int parentIdx, int depth, const std::vector<CRTTriangle>& triangles) {
        // Stop conditions: max depth reached or few enough triangles
        if (depth >= maxDepth || static_cast<int>(triangles.size()) <= maxBoxTrianglesCount) {
            nodes[parentIdx].triangles = triangles;
            return; // This becomes a leaf node
        }
        
        // Split the parent AABB box in two halves, alternating the split axis
        int splitAxis = depth % 3; // 3 axes: X, Y, Z
        auto [child0AABB, child1AABB] = nodes[parentIdx].aabb.split(splitAxis);
        
        // Distribute triangles to children based on AABB intersection
        std::vector<CRTTriangle> child0Triangles, child1Triangles;
        
        for (const auto& triangle : triangles) {
            if (child0AABB.intersectsTriangle(triangle)) {
                child0Triangles.push_back(triangle);
            }
            if (child1AABB.intersectsTriangle(triangle)) {
                child1Triangles.push_back(triangle);
            }
        }
        
        // Create child nodes if they have triangles
        if (!child0Triangles.empty()) {
            int child0Idx = addNode(child0AABB, parentIdx, -1, -1, std::vector<CRTTriangle>());
            nodes[parentIdx].children[0] = child0Idx;
            buildAccTree(child0Idx, depth + 1, child0Triangles);
        }
        
        if (!child1Triangles.empty()) {
            int child1Idx = addNode(child1AABB, parentIdx, -1, -1, std::vector<CRTTriangle>());
            nodes[parentIdx].children[1] = child1Idx;
            buildAccTree(child1Idx, depth + 1, child1Triangles);
        }
    }
    
    // Ray traversal using DFS with stack
    bool intersect(const CRTRay& ray, const std::vector<CRTMaterial>& materials,
                   float& closestT, IntersectionData& data) const {
        if (rootIndex == -1 || nodes.empty()) {
            return false;
        }
        
        closestT = std::numeric_limits<float>::infinity();
        bool hasHit = false;
        
        std::stack<int> nodeIndicesToCheck;
        nodeIndicesToCheck.push(rootIndex);
        
        while (!nodeIndicesToCheck.empty()) {
            int currentNodeIdxToCheck = nodeIndicesToCheck.top();
            nodeIndicesToCheck.pop();
            
            const CRTAccTreeNode& currentNode = nodes[currentNodeIdxToCheck];
            
            // Check if ray intersects with current node's AABB
            if (currentNode.aabb.intersect(ray)) {
                if (currentNode.isLeaf() && !currentNode.triangles.empty()) {
                    // Leaf node - intersect with triangles
                    float nodeT = closestT;
                    float minT = closestT;
                    IntersectionData nodeData;
                    
                    currentNode.intersect(ray, closestT, materials, nodeT, minT, nodeData);
                    
                    if (nodeT < closestT) {
                        closestT = nodeT;
                        data = nodeData;
                        hasHit = true;
                    }
                } else {
                    // Internal node - add children to stack for checking
                    if (currentNode.children[0] != -1) {
                        nodeIndicesToCheck.push(currentNode.children[0]);
                    }
                    if (currentNode.children[1] != -1) {
                        nodeIndicesToCheck.push(currentNode.children[1]);
                    }
                }
            }
        }
        
        return hasHit;
    }
    
    // Shadow ray intersection (early termination version)
    bool intersectShadowRay(const CRTRay& ray, float maxDistance) const {
        if (rootIndex == -1 || nodes.empty()) {
            return false;
        }
        
        std::stack<int> nodeIndicesToCheck;
        nodeIndicesToCheck.push(rootIndex);
        
        while (!nodeIndicesToCheck.empty()) {
            int currentNodeIdxToCheck = nodeIndicesToCheck.top();
            nodeIndicesToCheck.pop();
            
            const CRTAccTreeNode& currentNode = nodes[currentNodeIdxToCheck];
            
            // Check if ray intersects with current node's AABB
            float tNear, tFar;
            if (currentNode.aabb.intersect(ray, tNear, tFar)) {
                // If intersection is beyond the light, skip this node
                if (tNear > maxDistance) continue;
                
                if (currentNode.isLeaf() && !currentNode.triangles.empty()) {
                    // Leaf node - check triangles for shadow intersection
                    for (const auto& triangle : currentNode.triangles) {
                        float t, u, v;
                        CRTVector normal, uv;
                        
                        if (triangle.intersect(ray, t, u, v, normal, uv)) {
                            if (t > 1e-4f && t < maxDistance) {
                                return true; // Shadow ray blocked
                            }
                        }
                    }
                } else {
                    // Internal node - add children to stack for checking
                    if (currentNode.children[0] != -1) {
                        nodeIndicesToCheck.push(currentNode.children[0]);
                    }
                    if (currentNode.children[1] != -1) {
                        nodeIndicesToCheck.push(currentNode.children[1]);
                    }
                }
            }
        }
        
        return false; // No intersection found
    }
    
    // Get statistics about the tree
    struct TreeStats {
        int totalNodes = 0;
        int leafNodes = 0;
        int maxDepthReached = 0;
        int totalTriangles = 0;
        float averageTrianglesPerLeaf = 0.0f;
    };
    
    TreeStats getStatistics() const {
        TreeStats stats;
        if (rootIndex == -1) return stats;
        
        std::vector<bool> visited(nodes.size(), false);
        std::stack<std::pair<int, int>> nodeStack; // pair of (nodeIndex, depth)
        nodeStack.push(std::make_pair(rootIndex, 0));
        
        while (!nodeStack.empty()) {
            auto [nodeIdx, depth] = nodeStack.top();
            nodeStack.pop();
            
            if (visited[nodeIdx]) continue;
            visited[nodeIdx] = true;
            
            stats.totalNodes++;
            stats.maxDepthReached = std::max(stats.maxDepthReached, depth);
            
            const CRTAccTreeNode& node = nodes[nodeIdx];
            
            if (node.isLeaf()) {
                stats.leafNodes++;
                stats.totalTriangles += static_cast<int>(node.triangles.size());
            } else {
                if (node.children[0] != -1) {
                    nodeStack.push(std::make_pair(node.children[0], depth + 1));
                }
                if (node.children[1] != -1) {
                    nodeStack.push(std::make_pair(node.children[1], depth + 1));
                }
            }
        }
        
        if (stats.leafNodes > 0) {
            stats.averageTrianglesPerLeaf = static_cast<float>(stats.totalTriangles) / stats.leafNodes;
        }
        
        return stats;
    }
};
