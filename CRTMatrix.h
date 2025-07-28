#pragma once

#include "CRTVector.h"
#include <cmath>
#include <stdexcept>

class CRTMatrix {
public:
    CRTMatrix() {
        data[0][0] = 1.0f; data[0][1] = 0.0f; data[0][2] = 0.0f;
        data[1][0] = 0.0f; data[1][1] = 1.0f; data[1][2] = 0.0f;
        data[2][0] = 0.0f; data[2][1] = 0.0f; data[2][2] = 1.0f;
    }

    CRTMatrix(float m00, float m01, float m02,
              float m10, float m11, float m12,
              float m20, float m21, float m22) {
        data[0][0] = m00; data[0][1] = m01; data[0][2] = m02;
        data[1][0] = m10; data[1][1] = m11; data[1][2] = m12;
        data[2][0] = m20; data[2][1] = m21; data[2][2] = m22;
    }

    static CRTMatrix identity() {
        return CRTMatrix();
    }

    CRTMatrix operator*(const CRTMatrix& other) const {
        CRTMatrix result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.data[i][j] = 0.0f;
                for (int k = 0; k < 3; ++k) {
                    result.data[i][j] += data[i][k] * other.data[k][j];
                }
            }
        }
        return result;
    }

    CRTVector operator*(const CRTVector& vec) const {
        return CRTVector(
            data[0][0] * vec.getX() + data[0][1] * vec.getY() + data[0][2] * vec.getZ(),
            data[1][0] * vec.getX() + data[1][1] * vec.getY() + data[1][2] * vec.getZ(),
            data[2][0] * vec.getX() + data[2][1] * vec.getY() + data[2][2] * vec.getZ()
        );
    }

    float& at(int row, int col) {
        if (row < 0 || row > 2 || col < 0 || col > 2) {
            throw std::out_of_range("Matrix index out of range");
        }
        return data[row][col];
    }

    float at(int row, int col) const {
        if (row < 0 || row > 2 || col < 0 || col > 2) {
            throw std::out_of_range("Matrix index out of range");
        }
        return data[row][col];
    }

    CRTMatrix transpose() const {
        return CRTMatrix(
            data[0][0], data[1][0], data[2][0],
            data[0][1], data[1][1], data[2][1],
            data[0][2], data[1][2], data[2][2]
        );
    }

    float determinant() const {
        return data[0][0] * (data[1][1] * data[2][2] - data[1][2] * data[2][1]) -
               data[0][1] * (data[1][0] * data[2][2] - data[1][2] * data[2][0]) +
               data[0][2] * (data[1][0] * data[2][1] - data[1][1] * data[2][0]);
    }

    bool isOrthogonal() const {
        CRTMatrix transposed = transpose();
        CRTMatrix product = (*this) * transposed;
        
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                float expected = (i == j) ? 1.0f : 0.0f;
                if (std::abs(product.data[i][j] - expected) > 1e-4f) {
                    return false;
                }
            }
        }
        return true;
    }

private:
    float data[3][3];
};
