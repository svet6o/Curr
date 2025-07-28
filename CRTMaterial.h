#pragma once

#include "stb_image.h"
#include "CRTColor.h"
#include "CRTVector.h"

#include <string>
#include <vector>
#include <unordered_map>
#include <stdexcept>
#include <rapidjson/document.h>

struct TextureDef {
    enum class Type { ALBEDO, EDGES, CHECKER, BITMAP } type;
    CRTColor albedoColor;
    CRTColor edgeColor, innerColor;
    float edgeWidth = 0.04f;
    CRTColor colorA, colorB;
    float squareSize = 0.125f;
    std::string filePath;
};

class CRTMaterial {
public:
    enum class Type { DIFFUSE, REFLECTIVE, REFRACTIVE, CONSTANT };
    enum class TexType { NONE, CHECKER, BITMAP, EDGES };

    Type type = Type::DIFFUSE;
    bool smoothShading = false;
    float ior = 1.0f;

    // Albedo / texture
    CRTColor albedo = CRTColor(255, 255, 255);
    TexType texType = TexType::NONE;

    // For checker
    CRTColor checkerA, checkerB;
    float squareSize = 1.0f;

    // For edges
    CRTColor edgeColor, innerColor;
    float edgeWidth = 0.05f;

    // For bitmap
    std::string bitmapPath;
    std::vector<unsigned char> bitmapData;
    int width = 0, height = 0;

    CRTMaterial() = default;

    void loadFromJSON(
        const rapidjson::Value& json,
        const std::unordered_map<std::string, TextureDef>& textureDefs
    ) {
        if (json.HasMember("type") && json["type"].IsString()) {
            std::string t = json["type"].GetString();
            if (t == "diffuse") type = Type::DIFFUSE;
            else if (t == "reflective") type = Type::REFLECTIVE;
            else if (t == "refractive") type = Type::REFRACTIVE;
            else if (t == "constant") type = Type::CONSTANT;
        }

        if (json.HasMember("smooth_shading"))
            smoothShading = json["smooth_shading"].GetBool();

        if (json.HasMember("ior"))
            ior = json["ior"].GetFloat();

        // Load albedo — can be color array or string (texture name)
        if (json.HasMember("albedo")) {
            if (json["albedo"].IsArray()) {
                const auto& arr = json["albedo"].GetArray();
                if (arr.Size() == 3) {
                    albedo = CRTColor(
                        static_cast<unsigned char>(arr[0].GetFloat() * 255),
                        static_cast<unsigned char>(arr[1].GetFloat() * 255),
                        static_cast<unsigned char>(arr[2].GetFloat() * 255)
                    );
                    texType = TexType::NONE;
                }
            } else if (json["albedo"].IsString()) {
                std::string texName = json["albedo"].GetString();
                auto it = textureDefs.find(texName);
                if (it == textureDefs.end())
                    throw std::runtime_error("Unknown texture: " + texName);

                const TextureDef& def = it->second;
                switch (def.type) {
                    case TextureDef::Type::ALBEDO:
                        texType = TexType::NONE;
                        albedo = def.albedoColor;
                        break;

                    case TextureDef::Type::CHECKER:
                        texType = TexType::CHECKER;
                        checkerA = def.colorA;
                        checkerB = def.colorB;
                        squareSize = def.squareSize;
                        break;

                    case TextureDef::Type::EDGES:
                        texType = TexType::EDGES;
                        edgeColor = def.edgeColor;
                        innerColor = def.innerColor;
                        edgeWidth = def.edgeWidth;
                        break;

                    case TextureDef::Type::BITMAP:
                        texType = TexType::BITMAP;
                        bitmapPath = def.filePath;
                        loadImageData(bitmapPath, width, height, bitmapData);
                        break;
                }
            }
        }
    }

    CRTColor sampleTexture(const CRTVector& uv, float baryU = -1.0f, float baryV = -1.0f) const {
        switch (texType) {
            case TexType::CHECKER: {
                float u = uv.getX() - std::floor(uv.getX());
                float v = uv.getY() - std::floor(uv.getY());

                int squaresU = static_cast<int>(u / squareSize);
                int squaresV = static_cast<int>(v / squareSize);

                bool uEven = (squaresU % 2) == 0;
                bool vEven = (squaresV % 2) == 0;

                return (uEven == vEven) ? checkerA : checkerB;
            }

            case TexType::BITMAP:
                return sampleBitmap(uv);

            case TexType::EDGES: {
                float w = 1.0f - baryU - baryV;
                if (baryU < edgeWidth || baryV < edgeWidth || w < edgeWidth)
                    return edgeColor;
                else
                    return innerColor;
            }

            case TexType::NONE:
            default:
                return albedo;
        }
    }

private:
    void loadImageData(const std::string& path, int& outW, int& outH, std::vector<unsigned char>& outData) {
        stbi_set_flip_vertically_on_load(true);
        int channels;
        unsigned char* data = stbi_load(path.c_str(), &outW, &outH, &channels, 3);
        if (!data) {
            throw std::runtime_error("Failed to load bitmap: " + path);
        }

        outData.assign(data, data + outW * outH * 3);
        stbi_image_free(data);
    }

    CRTColor sampleBitmap(const CRTVector& uv) const {
        if (width <= 0 || height <= 0 || bitmapData.empty()) return albedo;

        float u = std::min(std::max(uv.getX(), 0.0f), 1.0f);
        float v = std::min(std::max(uv.getY(), 0.0f), 1.0f);

        int x = static_cast<int>(u * (width - 1));
        int y = static_cast<int>(v * (height - 1));
        int idx = (y * width + x) * 3;

        return CRTColor(
            bitmapData[idx],
            bitmapData[idx + 1],
            bitmapData[idx + 2]
        );
    }
};
