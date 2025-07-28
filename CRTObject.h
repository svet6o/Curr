#pragma once

#include "CRTAABB.h"
#include "CRTTriangle.h"
#include "CRTRay.h"
#include "CRTMaterial.h"
#include <vector>
#include <string>

class CRTSceneObject {
public:
    std::vector<CRTTriangle> triangles;
    CRTAABB boundingBox;
    std::string name;
    int materialIndex;

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
                if (t < closestT) {
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

    // Fast shadow ray intersection test (returns early on first hit)
    bool intersectShadowRay(const CRTRay& ray, float maxDistance) const {
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
        for (const auto& triangle : triangles) {
            float t, u, v;
            CRTVector normal, uv;
            
            if (triangle.intersect(ray, t, u, v, normal, uv)) {
                // Check if intersection is within valid shadow range
                if (t > 1e-4f && t < maxDistance) {
                    return true; // Shadow ray blocked
                }
            }
        }
        
        return false; // No blocking intersection found
    }

    // Fast shadow ray intersection test (returns early on first hit), with ability to ignore one triangle
bool intersectShadowRay(const CRTRay& ray,
                        float maxDistance,
                        int ignoreTriangle) const
{
    constexpr float kEps = 1e-4f;

    float tNear, tFar;
    if (!boundingBox.intersect(ray, tNear, tFar)) {
        return false;
    }

    // Ако пресичането с AABB започва след светлината – няма смисъл да тестваме
    if (tNear > maxDistance) {
        return false;
    }

    // Тествай триъгълниците
    for (size_t i = 0; i < triangles.size(); ++i) {
        if ((int)i == ignoreTriangle) continue; // <-- ключовата линия

        float t, u, v;
        CRTVector normal, uv;

        if (triangles[i].intersect(ray, t, u, v, normal, uv)) {
            if (t > kEps && t < maxDistance - kEps) {
                return true;
            }
        }
    }

    return false;
}


    // Get the triangle at the specified index
    const CRTTriangle& getTriangle(int index) const {
        return triangles[index];
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
    // Note: This is a placeholder - you'd implement actual transformation logic
    void transform(const CRTMatrix& transformMatrix) {
        // Transform all triangle vertices
        // This would require extending CRTTriangle to support transformation
        // For now, just update the bounding box
        updateBoundingBox();
    }

    // Get statistics about this object
    struct Statistics {
        size_t triangleCount;
        float boundingBoxVolume;
        CRTVector boundingBoxSize;
        CRTVector center;
    };

    Statistics getStatistics() const {
        Statistics stats;
        stats.triangleCount = triangles.size();
        
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
        
        return stats;
    }
};
