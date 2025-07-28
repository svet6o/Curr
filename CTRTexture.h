// texture.h
#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include "CRTVector.h"

template<typename T>
constexpr const T& clamp(const T& value, const T& min, const T& max) {
    return (value < min) ? min : (max < value) ? max : value;
}

class Texture {
public:
    virtual ~Texture() = default;
    virtual CRTVector sample(float u, float v) const = 0;
};

class BitmapTexture : public Texture {
public:
    // data:
    BitmapTexture(int w, int h, const unsigned char* rawData)
        : width(w), height(h), data(rawData) {}

    CRTVector sample(float u, float v) const override {
        int iu = clamp(int(u * (width - 1)), 0, width - 1);
        int iv = clamp(int((1.0f - v) * (height - 1)), 0, height - 1);
        int idx = (iv * width + iu) * 3;
        return CRTVector(
            data[idx]   / 255.0f,
            data[idx+1] / 255.0f,
            data[idx+2] / 255.0f
        );
    }

private:
    int width;
    int height;
    const unsigned char* data;
};


class CheckerTexture : public Texture {
public:
    CheckerTexture(const CRTVector& colorA, const CRTVector& colorB, float size)
        : A(colorA), B(colorB), squareSize(size) {}

    CRTVector sample(float u, float v) const override {
        int x = int(std::floor(u / squareSize));
        int y = int(std::floor(v / squareSize));
        bool even = ((x + y) & 1) == 0;
        return even ? A : B;
    }

private:
    CRTVector A, B;
    float squareSize;
};

// Edge
class EdgeTexture : public Texture {
public:
    EdgeTexture(const CRTVector& innerColor, const CRTVector& edgeColor, float edgeWidth)
        : inner(innerColor), edge(edgeColor), width(edgeWidth) {}

    CRTVector sample(float u, float v) const override {
        float minDist = std::min({u, v, 1.0f - u - v});
        return (minDist < width) ? edge : inner;
    }

private:
    CRTVector inner, edge;
    float width;
};
