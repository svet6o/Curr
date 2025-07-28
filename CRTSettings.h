#pragma once

#include "CRTColor.h"
#include <string>
#include <stdexcept>
#include <rapidjson/document.h>

class CRTSettings {
public:
    CRTColor backgroundColor;
    int resolutionWidth;
    int resolutionHeight;
    int bucketSize;

    CRTSettings()
        : resolutionWidth(1920)
        , resolutionHeight(1080)
        , bucketSize(32)       
    {}

    void loadFromJSON(const rapidjson::Value& json) {
        if (!json.IsObject()) {
            throw std::runtime_error("Settings section is not an object");
        }

        if (json.HasMember("background_color") && json["background_color"].IsArray()) {
            const auto& color = json["background_color"].GetArray();
            if (color.Size() == 3) {
                backgroundColor.r = static_cast<unsigned char>(color[0].GetFloat() * 255.0f);
                backgroundColor.g = static_cast<unsigned char>(color[1].GetFloat() * 255.0f);
                backgroundColor.b = static_cast<unsigned char>(color[2].GetFloat() * 255.0f);
            }
        } else {
            throw std::runtime_error("Missing or invalid 'background_color'");
        }

        if (json.HasMember("image_settings") && json["image_settings"].IsObject()) {
            const auto& img = json["image_settings"];
            if (img.HasMember("width") && img["width"].IsInt()) {
                resolutionWidth = img["width"].GetInt();
            }
            if (img.HasMember("height") && img["height"].IsInt()) {
                resolutionHeight = img["height"].GetInt();
            }
            if (img.HasMember("bucket_size") && img["bucket_size"].IsInt()) {
                bucketSize = img["bucket_size"].GetInt();
            }
        } else {
            throw std::runtime_error("Missing or invalid 'image_settings'");
        }
    }
};
