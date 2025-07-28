#pragma once
#include "CRTVector.h"
#include "CRTMatrix.h"
#include <cmath>
#include <vector>
#include <stdexcept>
#include "rapidjson/document.h"

class CRTCamera {
public:
    enum class CameraActionType {
        Pan,
        Tilt,
        Roll,
        Dolly,
        Truck,
        Boom
    };

    struct CameraAction {
        CameraActionType type;
        float value; // Angle (radians) or distance
    };

    CRTCamera() : position(0.0f, 0.0f, 0.0f), 
                 rotationMatrix(CRTMatrix::identity()) {}

    CRTCamera(const CRTVector& pos, const CRTMatrix& rot = CRTMatrix::identity())
        : position(pos), rotationMatrix(rot) {}

    // Movement methods
    void dolly(float distance) { position = position + forward() * distance; }
    void truck(float distance) { position = position + right() * distance; }
    void boom(float distance) { position = position + up() * distance; }

    // Rotation methods
    void pan(float angleRadians) {
        rotateAroundAxis(up(), angleRadians);
    }

    void tilt(float angleRadians) {
        rotateAroundAxis(right(), angleRadians);
    }

    void roll(float angleRadians) {
        rotateAroundAxis(forward(), angleRadians);
    }

    // Direction vectors
    CRTVector forward() const { return rotationMatrix * CRTVector(0, 0, -1); }
    CRTVector right() const { return rotationMatrix * CRTVector(1, 0, 0); }
    CRTVector up() const { return rotationMatrix * CRTVector(0, 1, 0); }

    // Getters
    CRTVector getPosition() const { return position; }
    CRTMatrix getRotation() const { return rotationMatrix; }

    // Setters
    void setPosition(const CRTVector& pos) { position = pos; }
    void setRotation(const CRTMatrix& rot) { rotationMatrix = rot; }

    void loadFromJSON(const rapidjson::Value& json) {
    if (!json.IsObject()) {
        throw std::runtime_error("Camera section is not an object");
    }

    if (json.HasMember("position") && json["position"].IsArray()) {
        const auto& pos = json["position"].GetArray();
        if (pos.Size() == 3) {
            position = CRTVector(pos[0].GetFloat(), pos[1].GetFloat(), pos[2].GetFloat());
        }
    } else {
        throw std::runtime_error("Missing or invalid 'position'");
    }

    if (json.HasMember("matrix") && json["matrix"].IsArray()) {
        const auto& m = json["matrix"].GetArray();
        if (m.Size() == 9) {
            rotationMatrix = CRTMatrix(
                m[0].GetFloat(), m[1].GetFloat(), m[2].GetFloat(),
                m[3].GetFloat(), m[4].GetFloat(), m[5].GetFloat(),
                m[6].GetFloat(), m[7].GetFloat(), m[8].GetFloat()
            );
        }
    } else {
        throw std::runtime_error("Missing or invalid 'matrix'");
    }
}

private:
    CRTVector position;
    CRTMatrix rotationMatrix;

    void rotateAroundAxis(const CRTVector& axis, float angle) {
        CRTVector a = axis.normalize();
        float x = a.getX(), y = a.getY(), z = a.getZ();
        float c = std::cos(angle);
        float s = std::sin(angle);
        float t = 1 - c;

        CRTMatrix rotation(
            t*x*x + c,   t*x*y - s*z, t*x*z + s*y,
            t*x*y + s*z, t*y*y + c,   t*y*z - s*x,
            t*x*z - s*y, t*y*z + s*x, t*z*z + c
        );

        rotationMatrix = rotation * rotationMatrix;
    }
};