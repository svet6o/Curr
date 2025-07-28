#pragma once

#include "CRTVector.h"
#include "CRTRay.h"
#include "CRTTriangle.h"
#include <vector>
#include <limits>
#include <algorithm>

class CRTAABB {
public:
    CRTVector min, max;

    // Default constructor - creates invalid AABB
    CRTAABB() : min(std::numeric_limits<float>::max(), 
                    std::numeric_limits<float>::max(), 
                    std::numeric_limits<float>::max()),
                max(-std::numeric_limits<float>::max(), 
                    -std::numeric_limits<float>::max(), 
                    -std::numeric_limits<float>::max()) {}

    // Constructor with min/max points
    CRTAABB(const CRTVector& minPoint, const CRTVector& maxPoint) 
        : min(minPoint), max(maxPoint) {}

    // Check if AABB is valid (min <= max for all components)
    bool isValid() const {
        return min.getX() <= max.getX() && 
               min.getY() <= max.getY() && 
               min.getZ() <= max.getZ();
    }

    // Create AABB from a single triangle
    static CRTAABB fromTriangle(const CRTTriangle& triangle) {
        const CRTVector* vertices = triangle.getVertices();
        
        CRTVector minPoint = vertices[0];
        CRTVector maxPoint = vertices[0];
        
        for (int i = 1; i < 3; ++i) {
            minPoint = minPoint.componentMin(vertices[i]);
            maxPoint = maxPoint.componentMax(vertices[i]);
        }
        
        return CRTAABB(minPoint, maxPoint);
    }

    // Create AABB from multiple triangles
    static CRTAABB fromTriangles(const std::vector<CRTTriangle>& triangles) {
        if (triangles.empty()) {
            return CRTAABB(); // Invalid AABB
        }
        
        CRTAABB result = fromTriangle(triangles[0]);
        
        for (size_t i = 1; i < triangles.size(); ++i) {
            CRTAABB triangleBounds = fromTriangle(triangles[i]);
            result.expand(triangleBounds);
        }
        
        return result;
    }

    // Expand this AABB to include another AABB
    void expand(const CRTAABB& other) {
        if (!other.isValid()) return;
        
        if (!isValid()) {
            *this = other;
            return;
        }
        
        min = min.componentMin(other.min);
        max = max.componentMax(other.max);
    }

    // Expand this AABB to include a point
    void expand(const CRTVector& point) {
        if (!isValid()) {
            min = max = point;
            return;
        }
        
        min = min.componentMin(point);
        max = max.componentMax(point);
    }

    // Split AABB into two halves along the specified axis (0=X, 1=Y, 2=Z)
    std::pair<CRTAABB, CRTAABB> split(int axis) const {
        CRTAABB a = *this;
        CRTAABB b = *this;
        
        float minComponent, maxComponent;
        switch (axis) {
            case 0: // X axis
                minComponent = min.getX();
                maxComponent = max.getX();
                break;
            case 1: // Y axis
                minComponent = min.getY();
                maxComponent = max.getY();
                break;
            case 2: // Z axis
                minComponent = min.getZ();
                maxComponent = max.getZ();
                break;
            default:
                return std::make_pair(*this, *this); // No split
        }
        
        float mid = (maxComponent - minComponent) / 2.0f;
        float splitPlaneCoordinate = minComponent + mid;
        
        // Update A's max component for the splitting axis
        switch (axis) {
            case 0:
                a.max = CRTVector(splitPlaneCoordinate, a.max.getY(), a.max.getZ());
                b.min = CRTVector(splitPlaneCoordinate, b.min.getY(), b.min.getZ());
                break;
            case 1:
                a.max = CRTVector(a.max.getX(), splitPlaneCoordinate, a.max.getZ());
                b.min = CRTVector(b.min.getX(), splitPlaneCoordinate, b.min.getZ());
                break;
            case 2:
                a.max = CRTVector(a.max.getX(), a.max.getY(), splitPlaneCoordinate);
                b.min = CRTVector(b.min.getX(), b.min.getY(), splitPlaneCoordinate);
                break;
        }
        
        return std::make_pair(a, b);
    }

    // Check if this AABB intersects with another AABB
    bool intersects(const CRTAABB& other) const {
        // Check each axis
        if (min.getX() > other.max.getX() || max.getX() < other.min.getX()) return false;
        if (min.getY() > other.max.getY() || max.getY() < other.min.getY()) return false;
        if (min.getZ() > other.max.getZ() || max.getZ() < other.min.getZ()) return false;
        return true;
    }

    // Check if triangle AABB intersects with this node AABB
    bool intersectsTriangle(const CRTTriangle& triangle) const {
        CRTAABB triAABB = fromTriangle(triangle);
        return intersects(triAABB);
    }

    // Ray-AABB intersection test
    bool intersect(const CRTRay& ray) const {
        float tNear, tFar;
        return intersect(ray, tNear, tFar);
    }

    // Ray-AABB intersection test with t values
    bool intersect(const CRTRay& ray, float& tNear, float& tFar) const {
        const CRTVector& origin = ray.getOrigin();
        const CRTVector& direction = ray.getDirection();
        
        tNear = -std::numeric_limits<float>::infinity();
        tFar = std::numeric_limits<float>::infinity();
        
        // Check intersection for each axis
        for (int axis = 0; axis < 3; ++axis) {
            float rayOrigin, rayDir, boxMin, boxMax;
            
            switch (axis) {
                case 0:
                    rayOrigin = origin.getX();
                    rayDir = direction.getX();
                    boxMin = min.getX();
                    boxMax = max.getX();
                    break;
                case 1:
                    rayOrigin = origin.getY();
                    rayDir = direction.getY();
                    boxMin = min.getY();
                    boxMax = max.getY();
                    break;
                case 2:
                    rayOrigin = origin.getZ();
                    rayDir = direction.getZ();
                    boxMin = min.getZ();
                    boxMax = max.getZ();
                    break;
            }
            
            if (std::abs(rayDir) < 1e-8f) {
                // Ray is parallel to the slab
                if (rayOrigin < boxMin || rayOrigin > boxMax) {
                    return false;
                }
            } else {
                // Calculate intersection t values for this axis
                float t1 = (boxMin - rayOrigin) / rayDir;
                float t2 = (boxMax - rayOrigin) / rayDir;
                
                if (t1 > t2) std::swap(t1, t2);
                
                tNear = std::max(tNear, t1);
                tFar = std::min(tFar, t2);
                
                if (tNear > tFar) return false;
            }
        }
        
        return tFar > 0; // We want intersections in front of the ray
    }

    // Get center point of AABB
    CRTVector getCenter() const {
        return CRTVector(
            (min.getX() + max.getX()) * 0.5f,
            (min.getY() + max.getY()) * 0.5f,
            (min.getZ() + max.getZ()) * 0.5f
        );
    }

    // Get size (extent) of AABB
    CRTVector getSize() const {
        return CRTVector(
            max.getX() - min.getX(),
            max.getY() - min.getY(),
            max.getZ() - min.getZ()
        );
    }

    // Get surface area of AABB (useful for SAH)
    float getSurfaceArea() const {
        CRTVector size = getSize();
        return 2.0f * (size.getX() * size.getY() + 
                      size.getY() * size.getZ() + 
                      size.getZ() * size.getX());
    }

    // Get volume of AABB
    float getVolume() const {
        CRTVector size = getSize();
        return size.getX() * size.getY() * size.getZ();
    }
};
