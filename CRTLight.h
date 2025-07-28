#pragma once

#include "rapidjson/document.h"
#include  "CRTVector.h"

class CRTLight {
public:
     CRTLight()
        : position(CRTVector{0.0f, 0.0f, 0.0f}), intensity(1.0f) {}

    CRTLight(const CRTVector& position, float intensity = 1.0f)
        : position(position), intensity(intensity) {}

    const CRTVector& getPosition() const {
        return position;
    }

    float getIntensity() const {
        return intensity;
    }

    void loadFromJSON(const rapidjson::Value& json) {
        if (json.HasMember("position") && json["position"].IsArray()) {
            const auto& posArray = json["position"];
            if (posArray.Size() == 3 &&
                posArray[0].IsNumber() &&
                posArray[1].IsNumber() &&
                posArray[2].IsNumber()) {

                position = CRTVector(
                    posArray[0].GetFloat(),
                    posArray[1].GetFloat(),
                    posArray[2].GetFloat()
                );
            }
        }
        if (json.HasMember("intensity") && json["intensity"].IsNumber()) {
            intensity = json["intensity"].GetFloat();
        }
    }

private:
    CRTVector position;   
    float intensity;
};