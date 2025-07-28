#pragma once
#include "CRTColor.h"
#include "CRTVector.h"
#include "CRTMaterial.h"
#include <string>
#include <vector>
#include <unordered_map>
#include <stdexcept>
#include <rapidjson/document.h>

class CRTMesh {
public:
    CRTMesh() = default;

    // Load mesh data (vertices, optional uvs, triangle indices) from JSON
    void loadFromJSON(const rapidjson::Value& json) {
        if (!json.IsObject())
            throw std::runtime_error("Object entry is not a valid JSON object");

        // Clear previous data
        vertices.clear();
        uvCoords.clear();
        triangleVertIndices.clear();
        triangleMaterialIndices.clear();

        // 1) Vertices
        if (json.HasMember("vertices") && json["vertices"].IsArray()) {
            const auto& verts = json["vertices"].GetArray();
            if (verts.Size() % 3 != 0)
                throw std::runtime_error("Vertex count is not a multiple of 3");

            for (size_t i = 0; i < verts.Size(); i += 3) {
                float x = verts[i + 0].GetFloat();
                float y = verts[i + 1].GetFloat();
                float z = verts[i + 2].GetFloat();
                vertices.emplace_back(x, y, z);
            }
        } else {
            throw std::runtime_error("Missing or invalid 'vertices'");
        }

        // 2) UV coordinates (optional)
        if (json.HasMember("uvs") && json["uvs"].IsArray()) {
            const auto& uvArr = json["uvs"].GetArray();
            if (uvArr.Size() % 2 != 0)
                throw std::runtime_error("'uvs' array size must be a multiple of 2 (u,v)");

            size_t uvCount = uvArr.Size() / 2;
            uvCoords.reserve(uvCount);
            for (size_t i = 0; i < uvArr.Size(); i += 2) {
                float u = uvArr[i + 0].GetFloat();
                float v = uvArr[i + 1].GetFloat();
                uvCoords.emplace_back(u, v, 0.0f);
            }
            // Pad or trim to match vertex count
            if (uvCoords.size() < vertices.size()) {
                uvCoords.resize(vertices.size(), CRTVector(0.0f, 0.0f, 0.0f));
            } else if (uvCoords.size() > vertices.size()) {
                uvCoords.resize(vertices.size());
            }
        } else {
            // No UVs provided -> fill with default (0,0)
            uvCoords.assign(vertices.size(), CRTVector(0.0f, 0.0f, 0.0f));
        }

        // 3) Triangle vertex indices
        if (json.HasMember("triangles") && json["triangles"].IsArray()) {
            const auto& tris = json["triangles"].GetArray();
            for (const auto& t : tris) {
                if (!t.IsInt()) throw std::runtime_error("Invalid triangle index");
                triangleVertIndices.push_back(t.GetInt());
            }
        } else {
            throw std::runtime_error("Missing or invalid 'triangles'");
        }

        // 4) Material indices per triangle (optional)
        if (json.HasMember("material_indices") && json["material_indices"].IsArray()) {
            const auto& mats = json["material_indices"].GetArray();
            for (const auto& m : mats) {
                if (!m.IsInt()) throw std::runtime_error("Invalid material index");
                triangleMaterialIndices.push_back(m.GetInt());
            }
            if (triangleMaterialIndices.size() != triangleVertIndices.size() / 3)
                throw std::runtime_error("Material indices count must match triangle count");
        }
    }

    // Convert mesh to triangles for rendering
    std::vector<CRTTriangle> toTriangles(bool smoothShading = false) const {
        std::vector<CRTTriangle> result;
        size_t triCount = triangleVertIndices.size() / 3;
        result.reserve(triCount);

        // Precompute normals if smooth shading
        std::vector<CRTVector> vertexNormals;
        if (smoothShading) {
            vertexNormals.assign(vertices.size(), CRTVector(0, 0, 0));
            for (size_t i = 0; i < triCount; ++i) {
                int i0 = triangleVertIndices[i*3 + 0];
                int i1 = triangleVertIndices[i*3 + 1];
                int i2 = triangleVertIndices[i*3 + 2];
                CRTVector e1 = vertices[i1] - vertices[i0];
                CRTVector e2 = vertices[i2] - vertices[i0];
                CRTVector fn = e1.cross(e2).normalize();
                vertexNormals[i0] += fn;
                vertexNormals[i1] += fn;
                vertexNormals[i2] += fn;
            }
            for (auto& n : vertexNormals) n = n.normalize();
        }

        for (size_t i = 0; i < triCount; ++i) {
            int i0 = triangleVertIndices[i*3 + 0];
            int i1 = triangleVertIndices[i*3 + 1];
            int i2 = triangleVertIndices[i*3 + 2];

            CRTVector n0, n1, n2;
            if (smoothShading) {
                n0 = vertexNormals[i0];
                n1 = vertexNormals[i1];
                n2 = vertexNormals[i2];
            }

            CRTTriangle tri(
                vertices[i0], vertices[i1], vertices[i2],
                n0, n1, n2,
                uvCoords[i0], uvCoords[i1], uvCoords[i2],
                (triangleMaterialIndices.empty() ? 0 : triangleMaterialIndices[i])
            );
            result.push_back(tri);
        }
        return result;
    }

    // Optionally set materials externally
    void setMaterials(const std::vector<CRTMaterial>& mats) {
        materials = mats;
    }

private:
    std::vector<CRTVector> vertices;
    std::vector<CRTVector> uvCoords;
    std::vector<int> triangleVertIndices;
    std::vector<int> triangleMaterialIndices;
    std::vector<CRTMaterial> materials;
};
