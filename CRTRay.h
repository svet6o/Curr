#pragma once
#include "CRTVector.h"
#include "CRTCamera.h"

class CRTRay {
public:
    CRTRay() : origin(), direction() {}

    CRTRay(const CRTVector& origin, const CRTVector& direction) 
        : origin(origin), direction(direction.normalize()) {}

    CRTRay(const CRTRay& other) 
        : origin(other.origin), direction(other.direction) {}

    // Basic accessors (same as original)
    const CRTVector& getOrigin() const { return origin; }
    const CRTVector& getDirection() const { return direction; }

    void setOrigin(const CRTVector& newOrigin) { origin = newOrigin; }
    void setDirection(const CRTVector& newDirection) { direction = newDirection.normalize(); }

    // Point along ray (same as original)
    CRTVector pointAtParameter(float t) const {
        return origin + direction * t;
    }

 static CRTRay generatePrimaryRay(int x, int y, int imageWidth, int imageHeight, const CRTCamera& camera) {
        float pixelCenterX = static_cast<float>(x) + 0.5f;
        float pixelCenterY = static_cast<float>(y) + 0.5f;

        float ndcX = pixelCenterX / static_cast<float>(imageWidth);
        float ndcY = pixelCenterY / static_cast<float>(imageHeight);

        float screenX = 2.0f * ndcX - 1.0f;
        float screenY = 1.0f - 2.0f * ndcY;

        float aspectRatio = static_cast<float>(imageWidth) / static_cast<float>(imageHeight);
        screenX *= aspectRatio;

        CRTVector direction(screenX, screenY, -1.0f);
        direction.normalize();

        return CRTRay(camera.getPosition(), direction);
    }

private:
    CRTVector origin;    
    CRTVector direction; 
};