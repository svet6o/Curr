#pragma once

#include "CRTVector.h"
#include "CRTRay.h"
#include "CRTTriangle.h"
#include <vector>
#include <limits>
#include <algorithm>

class CRTAABB {
public:
    CRTVector minPoint;
    CRTVector maxPoint;

    // Default constructor - creates invalid AABB
    CRTAABB() 
        : minPoint(std::numeric_limits<float>::max(),
                   std::numeric_limits<float>::max(),
                   std::numeric_limits<float>::max()),
          maxPoint(std::numeric_limits<float>::lowest(),
                   std::numeric_limits<float>::lowest(),
                   std::numeric_limits<float>::lowest()) {}

    // Constructor with min/max points
    CRTAABB(const CRTVector& min, const CRTVector& max)
        : minPoint(min), maxPoint(max) {}

    // Build AABB from a collection of triangles
    static CRTAABB fromTriangles(const std::vector<CRTTriangle>& triangles) {
        CRTAABB box;
        
        for (const auto& triangle : triangles) {
            for (int i = 0; i < 3; ++i) {
                const CRTVector& vertex = triangle.getVertex(i);
                box.expand(vertex);
            }
        }
        
        return box;
    }

    // Build AABB from a single triangle
    static CRTAABB fromTriangle(const CRTTriangle& triangle) {
        CRTAABB box;
        
        for (int i = 0; i < 3; ++i) {
            const CRTVector& vertex = triangle.getVertex(i);
            box.expand(vertex);
        }
        
        return box;
    }

    // Expand the AABB to include a point
    void expand(const CRTVector& point) {
        minPoint = CRTVector(
            std::min(minPoint.getX(), point.getX()),
            std::min(minPoint.getY(), point.getY()),
            std::min(minPoint.getZ(), point.getZ())
        );
        
        maxPoint = CRTVector(
            std::max(maxPoint.getX(), point.getX()),
            std::max(maxPoint.getY(), point.getY()),
            std::max(maxPoint.getZ(), point.getZ())
        );
    }

    // Expand the AABB to include another AABB
    void expand(const CRTAABB& other) {
        expand(other.minPoint);
        expand(other.maxPoint);
    }

    // Check if this AABB is valid (min <= max for all components)
    bool isValid() const {
        return minPoint.getX() <= maxPoint.getX() &&
               minPoint.getY() <= maxPoint.getY() &&
               minPoint.getZ() <= maxPoint.getZ();
    }

    // Get the center of the AABB
    CRTVector getCenter() const {
        return CRTVector(
            (minPoint.getX() + maxPoint.getX()) * 0.5f,
            (minPoint.getY() + maxPoint.getY()) * 0.5f,
            (minPoint.getZ() + maxPoint.getZ()) * 0.5f
        );
    }

    // Get the size (extents) of the AABB
    CRTVector getSize() const {
        return CRTVector(
            maxPoint.getX() - minPoint.getX(),
            maxPoint.getY() - minPoint.getY(),
            maxPoint.getZ() - minPoint.getZ()
        );
    }

    // Get surface area of the AABB (useful for BVH construction)
    float getSurfaceArea() const {
        CRTVector size = getSize();
        return 2.0f * (size.getX() * size.getY() + 
                       size.getY() * size.getZ() + 
                       size.getZ() * size.getX());
    }

    // Ray-AABB intersection test using slab method
    bool intersect(const CRTRay& ray, float& tNear, float& tFar) const {
        const float epsilon = 1e-4f;
        const CRTVector& origin = ray.getOrigin();
        const CRTVector& direction = ray.getDirection();
        
        tNear = 0.0f;
        tFar = std::numeric_limits<float>::max();

        // Test intersection with each pair of parallel planes
        for (int axis = 0; axis < 3; ++axis) {
            float rayOrigin, rayDir, boxMin, boxMax;
            
            switch (axis) {
                case 0: // X axis
                    rayOrigin = origin.getX();
                    rayDir = direction.getX();
                    boxMin = minPoint.getX();
                    boxMax = maxPoint.getX();
                    break;
                case 1: // Y axis
                    rayOrigin = origin.getY();
                    rayDir = direction.getY();
                    boxMin = minPoint.getY();
                    boxMax = maxPoint.getY();
                    break;
                case 2: // Z axis
                    rayOrigin = origin.getZ();
                    rayDir = direction.getZ();
                    boxMin = minPoint.getZ();
                    boxMax = maxPoint.getZ();
                    break;
            }

            if (std::abs(rayDir) < epsilon) {
                // Ray is parallel to the slab
                if (rayOrigin < boxMin || rayOrigin > boxMax) {
                    return false; // Ray is outside the slab
                }
            } else {
                // Calculate intersection distances
                float t1 = (boxMin - rayOrigin) / rayDir;
                float t2 = (boxMax - rayOrigin) / rayDir;
                
                // Ensure t1 is the near intersection, t2 is the far
                if (t1 > t2) {
                    std::swap(t1, t2);
                }
                
                // Update the intersection interval
                tNear = std::max(tNear, t1);
                tFar = std::min(tFar, t2);
                
                // Check if the interval is valid
                if (tNear > tFar) {
                    return false; // No intersection
                }
            }
        }
        
        // For shadow rays, we need tNear to be positive and meaningful
        return tFar >= 0.0f && tNear <= tFar;
    }

    // Simple ray-AABB intersection test (returns only boolean)
    bool intersect(const CRTRay& ray) const {
        float tNear, tFar;
        return intersect(ray, tNear, tFar);
    }

    // Check if a point is inside the AABB
    bool contains(const CRTVector& point) const {
        return point.getX() >= minPoint.getX() && point.getX() <= maxPoint.getX() &&
               point.getY() >= minPoint.getY() && point.getY() <= maxPoint.getY() &&
               point.getZ() >= minPoint.getZ() && point.getZ() <= maxPoint.getZ();
    }

    // Check if this AABB overlaps with another AABB
    bool overlaps(const CRTAABB& other) const {
        return minPoint.getX() <= other.maxPoint.getX() && maxPoint.getX() >= other.minPoint.getX() &&
               minPoint.getY() <= other.maxPoint.getY() && maxPoint.getY() >= other.minPoint.getY() &&
               minPoint.getZ() <= other.maxPoint.getZ() && maxPoint.getZ() >= other.minPoint.getZ();
    }

    // Get the longest axis (0=X, 1=Y, 2=Z)
    int getLongestAxis() const {
        CRTVector size = getSize();
        if (size.getX() >= size.getY() && size.getX() >= size.getZ()) return 0;
        if (size.getY() >= size.getZ()) return 1;
        return 2;
    }

    // Generate the 8 corner points of the AABB
    std::vector<CRTVector> getCorners() const {
        return {
            CRTVector(minPoint.getX(), minPoint.getY(), minPoint.getZ()),
            CRTVector(maxPoint.getX(), minPoint.getY(), minPoint.getZ()),
            CRTVector(minPoint.getX(), maxPoint.getY(), minPoint.getZ()),
            CRTVector(maxPoint.getX(), maxPoint.getY(), minPoint.getZ()),
            CRTVector(minPoint.getX(), minPoint.getY(), maxPoint.getZ()),
            CRTVector(maxPoint.getX(), minPoint.getY(), maxPoint.getZ()),
            CRTVector(minPoint.getX(), maxPoint.getY(), maxPoint.getZ()),
            CRTVector(maxPoint.getX(), maxPoint.getY(), maxPoint.getZ())
        };
    }
};
