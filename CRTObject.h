#pragma once

#include "CRTAABB.h"
#include "CRTTriangle.h"
#include "CRTRay.h"
#include "CRTMaterial.h"
#include <vector>
#include <string>
#include <limits>
#include <stdexcept>

class CRTSceneObject {
public:
    std::vector<CRTTriangle> triangles;
    CRTAABB boundingBox;
    std::string name;
    int materialIndex;

    static constexpr float EPSILON = 1e-4f;

    CRTSceneObject() : materialIndex(0) {}

    CRTSceneObject(const std::vector<CRTTriangle>& tris, const std::string& objName = "", int matIdx = 0)
        : triangles(tris), name(objName), materialIndex(matIdx) {
        updateBoundingBox();
    }

    // Update the bounding box to encompass all triangles
    void updateBoundingBox() {
        if (triangles.empty()) {
            boundingBox = CRTAABB(); // Invalid AABB
            return;
        }
        
        boundingBox = CRTAABB::fromTriangles(triangles);
    }

    // Add a triangle to this object
    void addTriangle(const CRTTriangle& triangle) {
        triangles.push_back(triangle);
        
        // Expand bounding box to include new triangle
        if (triangles.size() == 1) {
            boundingBox = CRTAABB::fromTriangle(triangle);
        } else {
            CRTAABB triangleBounds = CRTAABB::fromTriangle(triangle);
            boundingBox.expand(triangleBounds);
        }
    }

    // Fast intersection test using AABB first, then detailed triangle tests
    bool intersect(const CRTRay& ray, float& closestT, int& hitTriangleIndex, 
                   float& hitU, float& hitV, CRTVector& hitNormal, CRTVector& hitUV) const {
        
        // First, test against the bounding box
        if (!boundingBox.intersect(ray)) {
            return false; // Ray doesn't hit the bounding box, so it can't hit any triangles
        }

        // If AABB test passes, test individual triangles
        bool hasHit = false;
        closestT = std::numeric_limits<float>::max();
        hitTriangleIndex = -1;

        for (size_t i = 0; i < triangles.size(); ++i) {
            float t, u, v;
            CRTVector normal, uv;
            
            if (triangles[i].intersect(ray, t, u, v, normal, uv)) {
                if (t > EPSILON && t < closestT) { // Added epsilon check
                    closestT = t;
                    hitTriangleIndex = static_cast<int>(i);
                    hitU = u;
                    hitV = v;
                    hitNormal = normal;
                    hitUV = uv;
                    hasHit = true;
                }
            }
        }

        return hasHit;
    }

    // Unified shadow ray intersection test with optional triangle ignore
    bool intersectShadowRay(const CRTRay& ray, float maxDistance, int ignoreTriangle = -1) const {
        // First, test against the bounding box with distance constraint
        float tNear, tFar;
        if (!boundingBox.intersect(ray, tNear, tFar)) {
            return false;
        }
        
        // If AABB intersection is beyond the light, no shadow
        if (tNear > maxDistance) {
            return false;
        }

        // Test individual triangles
        for (size_t i = 0; i < triangles.size(); ++i) {
            if (static_cast<int>(i) == ignoreTriangle) continue;

            float t, u, v;
            CRTVector normal, uv;
            
            if (triangles[i].intersect(ray, t, u, v, normal, uv)) {
                // Check if intersection is within valid shadow range
                if (t > EPSILON && t < maxDistance - EPSILON) {
                    return true; // Shadow ray blocked
                }
            }
        }
        
        return false; // No blocking intersection found
    }

    // Get the triangle at the specified index with bounds checking
    const CRTTriangle& getTriangle(int index) const {
        if (index < 0 || index >= static_cast<int>(triangles.size())) {
            throw std::out_of_range("Triangle index out of range");
        }
        return triangles[index];
    }

    // Safe version that returns nullptr if index is invalid
    const CRTTriangle* getTriangleSafe(int index) const {
        if (index < 0 || index >= static_cast<int>(triangles.size())) {
            return nullptr;
        }
        return &triangles[index];
    }

    // Get number of triangles
    size_t getTriangleCount() const {
        return triangles.size();
    }

    // Check if object is valid (has triangles and valid bounding box)
    bool isValid() const {
        return !triangles.empty() && boundingBox.isValid();
    }

    // Get bounding box information
    const CRTAABB& getBoundingBox() const {
        return boundingBox;
    }

    // Set material index for all triangles in this object
    void setMaterialIndex(int matIdx) {
        materialIndex = matIdx;
        for (auto& triangle : triangles) {
            triangle.setMaterialIndex(matIdx);
        }
    }

    // Get material index
    int getMaterialIndex() const {
        return materialIndex;
    }

    // Transform all triangles and update bounding box
    void transform(const CRTMatrix& transformMatrix) {
        // Store original triangles data before transformation
        std::vector<CRTTriangle> transformedTriangles;
        transformedTriangles.reserve(triangles.size());
        
        for (const auto& triangle : triangles) {
            // Get current vertices
            CRTVector v0 = triangle.getVertex(0);
            CRTVector v1 = triangle.getVertex(1);
            CRTVector v2 = triangle.getVertex(2);
            
            // Transform vertices
            CRTVector tv0 = transformMatrix * v0;
            CRTVector tv1 = transformMatrix * v1;
            CRTVector tv2 = transformMatrix * v2;
            
            // Get current normals
            CRTVector n0 = triangle.getNormal(0);
            CRTVector n1 = triangle.getNormal(1);
            CRTVector n2 = triangle.getNormal(2);
            
            // Transform normals (for rotation/uniform scaling, matrix works)
            // For non-uniform scaling, you'd need inverse transpose
            CRTVector tn0 = (transformMatrix * n0).normalize();
            CRTVector tn1 = (transformMatrix * n1).normalize();
            CRTVector tn2 = (transformMatrix * n2).normalize();
            
            // UV coordinates remain the same during spatial transformation
            // Note: Assuming default UVs for now - you might want to preserve original UVs
            CRTVector uv0(0.0f, 0.0f, 0.0f);
            CRTVector uv1(1.0f, 0.0f, 0.0f);
            CRTVector uv2(0.0f, 1.0f, 0.0f);
            
            // Create new transformed triangle
            CRTTriangle transformedTri(
                tv0, tv1, tv2,                    // Transformed vertices
                tn0, tn1, tn2,                    // Transformed normals
                uv0, uv1, uv2,                    // UV coordinates
                triangle.getMaterialIndex()       // Preserve material
            );
            
            transformedTriangles.push_back(transformedTri);
        }
        
        // Replace triangles with transformed versions
        triangles = std::move(transformedTriangles);
        
        // Update bounding box after transformation
        updateBoundingBox();
    }
    
    // Simplified transform that recalculates normals automatically
    void transformSimple(const CRTMatrix& transformMatrix) {
        std::vector<CRTTriangle> transformedTriangles;
        transformedTriangles.reserve(triangles.size());
        
        for (const auto& triangle : triangles) {
            // Transform only vertices, let constructor calculate normals
            CRTVector tv0 = transformMatrix * triangle.getVertex(0);
            CRTVector tv1 = transformMatrix * triangle.getVertex(1);
            CRTVector tv2 = transformMatrix * triangle.getVertex(2);
            
            // Create triangle with auto-calculated normals (pass empty normals)
            CRTTriangle transformedTri(
                tv0, tv1, tv2,                    // Transformed vertices
                CRTVector(), CRTVector(), CRTVector(), // Empty = auto-calculate normals
                CRTVector(0,0,0), CRTVector(1,0,0), CRTVector(0,1,0), // Default UVs
                triangle.getMaterialIndex()
            );
            
            transformedTriangles.push_back(transformedTri);
        }
        
        triangles = std::move(transformedTriangles);
        updateBoundingBox();
    }

    // Enhanced statistics structure
    struct Statistics {
        size_t triangleCount;
        float boundingBoxVolume;
        CRTVector boundingBoxSize;
        CRTVector center;
        float averageTriangleArea;
        float totalSurfaceArea;
    };

    // Get enhanced statistics about this object
    Statistics getStatistics() const {
        Statistics stats;
        stats.triangleCount = triangles.size();
        stats.averageTriangleArea = 0.0f;
        stats.totalSurfaceArea = 0.0f;
        
        if (boundingBox.isValid()) {
            CRTVector size = boundingBox.getSize();
            stats.boundingBoxVolume = size.getX() * size.getY() * size.getZ();
            stats.boundingBoxSize = size;
            stats.center = boundingBox.getCenter();
        } else {
            stats.boundingBoxVolume = 0.0f;
            stats.boundingBoxSize = CRTVector(0, 0, 0);
            stats.center = CRTVector(0, 0, 0);
        }

        // Calculate surface area statistics
        if (!triangles.empty()) {
            float totalArea = 0.0f;
            for (const auto& triangle : triangles) {
                // Calculate triangle area using cross product
                CRTVector v0 = triangle.getVertex(0);
                CRTVector v1 = triangle.getVertex(1);
                CRTVector v2 = triangle.getVertex(2);
                
                CRTVector edge1 = v1 - v0;
                CRTVector edge2 = v2 - v0;
                float area = 0.5f * edge1.cross(edge2).length();
                totalArea += area;
            }
            stats.totalSurfaceArea = totalArea;
            stats.averageTriangleArea = totalArea / triangles.size();
        }
        
        return stats;
    }

    // Find degenerate triangles (very small area)
    std::vector<int> findDegenerateTriangles(float minArea = 1e-6f) const {
        std::vector<int> degenerateIndices;
        
        for (size_t i = 0; i < triangles.size(); ++i) {
            const auto& tri = triangles[i];
            CRTVector v0 = tri.getVertex(0);
            CRTVector v1 = tri.getVertex(1);
            CRTVector v2 = tri.getVertex(2);
            
            CRTVector edge1 = v1 - v0;
            CRTVector edge2 = v2 - v0;
            float area = 0.5f * edge1.cross(edge2).length();
            
            if (area < minArea) {
                degenerateIndices.push_back(static_cast<int>(i));
            }
        }
        
        return degenerateIndices;
    }

    // Remove degenerate triangles and return count removed
    int removeDegenerate() {
        auto degenerates = findDegenerateTriangles();
        
        // Remove in reverse order to maintain indices
        for (auto it = degenerates.rbegin(); it != degenerates.rend(); ++it) {
            triangles.erase(triangles.begin() + *it);
        }
        
        // Update bounding box after removal
        if (!degenerates.empty()) {
            updateBoundingBox();
        }
        
        return static_cast<int>(degenerates.size());
    }

    // Animation helper methods - updated for 3x3 matrix + translation vector
    void translate(const CRTVector& translation) {
        // For 3x3 matrices, we handle translation by directly adding to vertices
        std::vector<CRTTriangle> translatedTriangles;
        translatedTriangles.reserve(triangles.size());
        
        for (const auto& triangle : triangles) {
            // Translate vertices directly
            CRTVector tv0 = triangle.getVertex(0) + translation;
            CRTVector tv1 = triangle.getVertex(1) + translation;
            CRTVector tv2 = triangle.getVertex(2) + translation;
            
            // Normals don't change with translation
            CRTVector n0 = triangle.getNormal(0);
            CRTVector n1 = triangle.getNormal(1);
            CRTVector n2 = triangle.getNormal(2);
            
            // Create translated triangle
            CRTTriangle translatedTri(
                tv0, tv1, tv2,
                n0, n1, n2,
                CRTVector(0,0,0), CRTVector(1,0,0), CRTVector(0,1,0), // Default UVs
                triangle.getMaterialIndex()
            );
            
            translatedTriangles.push_back(translatedTri);
        }
        
        triangles = std::move(translatedTriangles);
        updateBoundingBox();
    }
    // Store original state for animation resets
    void storeOriginalState() {
        originalTriangles = triangles;
        originalBoundingBox = boundingBox;
    }
    
    void restoreOriginalState() {
        if (!originalTriangles.empty()) {
            triangles = originalTriangles;
            boundingBox = originalBoundingBox;
        }
    }
    
private:
    // Store original geometry for animation resets
    std::vector<CRTTriangle> originalTriangles;
    CRTAABB originalBoundingBox;
};