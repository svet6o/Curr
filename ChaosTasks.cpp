#define STB_IMAGE_IMPLEMENTATION

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <sstream>
#include <array>
#include <chrono>
#include <map>

#include "CRTColor.h"
#include "CRTRay.h"
#include "CRTVector.h"
#include "CRTTriangle.h"
#include "CRTCamera.h"
#include "CRTMesh.h"
#include "CRTSettings.h"
#include "CRTLight.h"
#include "CRTMaterial.h"

#include "CRTAABB.h"
#include "CRTObject.h"
#include "CRTAABBRendering.h"
#include "MultithreadedBuckets.h"
#include "MultithreadedRegions.h"
#include "AccTreeRendering.h"
#include "CRTAutoAccTree.h"

#include "rapidjson/document.h"

using namespace std::chrono;

template <typename T>
constexpr const T &clamp(const T &value, const T &min, const T &max)
{
    return (value < min) ? min : (max < value) ? max
                                               : value;
}

void saveToPPM(const std::string &filename, const std::vector<std::vector<CRTColor>> &image)
{
    int width = image[0].size();
    int height = image.size();

    std::ofstream ppmFile(filename);
    ppmFile << "P3\n"
            << width << " " << height << "\n255\n";

    for (const auto &row : image)
    {
        for (const auto &pixel : row)
        {
            ppmFile << pixel.r << " " << pixel.g << " " << pixel.b << " ";
        }
        ppmFile << "\n";
    }
}

static bool Refract(const CRTVector &I, const CRTVector &N, float eta1, float eta2,
                    CRTVector &refracted)
{
    float cosI = -I.dot(N);
    float n1 = eta1, n2 = eta2;
    CRTVector nN = N;

    if (cosI < 0.0f)
    {

        cosI = -cosI;
        std::swap(n1, n2);
        nN = -N;
    }

    float eta = n1 / n2;
    float sinT2 = eta * eta * (1.0f - cosI * cosI);
    if (sinT2 > 1.0f)
        return false;

    float cosT = std::sqrt(1.0f - sinT2);
    refracted = I * eta + nN * (eta * cosI - cosT);
    refracted = refracted.normalize();
    return true;
}

static float FresnelSchlick(const CRTVector &I, const CRTVector &N, float ior)
{
    float cosI = clamp(-I.dot(N), -1.0f, 1.0f);
    float eta1 = 1.0f, eta2 = ior;
    if (cosI > 0)
        std::swap(eta1, eta2);

    float sinT = eta1 / eta2 * sqrtf(1.0f - cosI * cosI);
    if (sinT >= 1.0f)
        return 1.0f; // Total internal reflection

    float cosT = sqrtf(1.0f - sinT * sinT);
    float rParallel = ((eta2 * cosI) - (eta1 * cosT)) / ((eta2 * cosI) + (eta1 * cosT));
    float rPerpendicular = ((eta1 * cosI) - (eta2 * cosT)) / ((eta1 * cosI) + (eta2 * cosT));
    return (rParallel * rParallel + rPerpendicular * rPerpendicular) / 2.0f;
}

CRTColor traceRay(
    const CRTRay &ray,
    const std::vector<CRTTriangle> &scene,
    const std::vector<CRTMaterial> &materials,
    const std::vector<CRTLight> &lights,
    const CRTSettings &settings,
    int depth,
    bool isShadowRay = false,
    float shadowBias = 1e-4f,
    float refractionBias = 1e-4f)
{
    if (depth > 5)
        return settings.backgroundColor;

    float closestT = std::numeric_limits<float>::infinity();
    int hitIdx = -1;
    float hitU = 0.0f, hitV = 0.0f;
    CRTVector hitNormal, hitUV;

    for (size_t i = 0; i < scene.size(); ++i)
    {
        float t, u, v;
        CRTVector localN, localUV;
        if (scene[i].intersect(ray, t, u, v, localN, localUV) && t < closestT)
        {
            closestT = t;
            hitIdx = int(i);
            hitNormal = localN;
            hitUV = localUV;
            hitU = u;
            hitV = v;
        }
    }

    if (hitIdx < 0)
        return settings.backgroundColor;

    CRTVector hitPoint = ray.getOrigin() + ray.getDirection() * closestT;
    const CRTMaterial &mat = materials[scene[hitIdx].getMaterialIndex()];
    CRTVector normalToUse = mat.smoothShading
                                ? hitNormal
                                : scene[hitIdx].getFaceNormal();

    // 4) Семплиране на текстура (или албедо)
    CRTColor baseColor = (mat.texType != CRTMaterial::TexType::NONE)
                             ? mat.sampleTexture(hitUV, hitU, hitV)
                             : mat.albedo;

    // 5) Логика по тип на материала
   switch (mat.type) {
        case CRTMaterial::Type::DIFFUSE: {
            CRTColor accum(0, 0, 0);
            for (const auto& light : lights) {
                CRTVector L = light.getPosition() - hitPoint;
                float dist = L.length();
                L = L.normalize();

                float NdotL = std::max(0.0f, normalToUse.dot(L));
                if (NdotL <= 0.0f) continue;

                // shadow ray
                CRTRay shadowRay(hitPoint + normalToUse * shadowBias, L);
                bool inShadow = false;
                for (const auto& tri : scene) {
                    if (&tri == &scene[hitIdx]) continue;
                    float t2, u2, v2;
                    CRTVector n2, uv2;
                    if (tri.intersect(shadowRay, t2, u2, v2, n2, uv2) && t2 < dist) {
                        int m2 = tri.getMaterialIndex();
                        if (materials[m2].type != CRTMaterial::Type::REFRACTIVE) {
                            inShadow = true;
                            break;
                        }
                    }
                }

                if (inShadow) continue;

                float Li = light.getIntensity();
                float attenuation = Li / (4.0f * PI * dist * dist + 1e-4f);
                accum += baseColor * (attenuation * NdotL);
            }
            return accum;
        }

        case CRTMaterial::Type::REFLECTIVE: {
            CRTVector I = ray.getDirection().normalize();
            CRTVector R = I - normalToUse * (2.0f * I.dot(normalToUse));
            CRTRay reflectRay(hitPoint + normalToUse * shadowBias, R.normalize());
            return traceRay(reflectRay, scene, materials, lights, settings, depth + 1);
        }

        case CRTMaterial::Type::REFRACTIVE: {
            CRTVector I = ray.getDirection().normalize();
            CRTVector N = normalToUse;
            float eta1 = 1.0f, eta2 = mat.ior;
            if (I.dot(N) > 0.0f) {
                N = -N;
                std::swap(eta1, eta2);
            }

            CRTVector Rdir = I - N * (2.0f * I.dot(N));
            CRTRay reflectRay(hitPoint + N * shadowBias, Rdir.normalize());
            CRTColor C_reflect = traceRay(reflectRay, scene, materials, lights, settings, depth + 1);

            CRTVector Tdir;
            CRTColor C_refract(0, 0, 0);
            if (Refract(I, N, eta1, eta2, Tdir)) {
                CRTRay refractRay(hitPoint - N * refractionBias, Tdir);
                C_refract = traceRay(refractRay, scene, materials, lights, settings, depth + 1);
            }

            if (isShadowRay) {
                return (C_refract.r || C_refract.g || C_refract.b) ? C_refract : C_reflect;
            }

            float kr = (C_refract.r || C_refract.g || C_refract.b)
                     ? FresnelSchlick(I, N, mat.ior)
                     : 1.0f;
            return C_reflect * kr + C_refract * (1.0f - kr);
        }

        case CRTMaterial::Type::CONSTANT:
            return baseColor;

        default:
            return CRTColor(255, 0, 255); // Магента за грешка
    }
}

void renderTriangleScene(const std::vector<CRTTriangle> &triangles,
                         const std::vector<CRTMaterial> &materials,
                         const std::vector<CRTLight> &lights,
                         const CRTCamera &camera,
                         const CRTSettings &settings,
                         const std::string &filename)
{
    int W = settings.resolutionWidth;
    int H = settings.resolutionHeight;
    CRTColor bg = settings.backgroundColor;

    std::vector<std::vector<CRTColor>> image(H, std::vector<CRTColor>(W, bg));

    for (int y = 0; y < H; ++y)
    {
        for (int x = 0; x < W; ++x)
        {
            CRTRay primary = CRTRay::generatePrimaryRay(x, y, W, H, camera);

            image[y][x] = traceRay(primary, triangles, materials, lights, settings, 0);
        }
    }

    saveToPPM(filename, image);
}


void loadSceneFromFile(
    const std::string& filePath,
    CRTSettings& settings,
    CRTCamera& camera,
    std::vector<CRTTriangle>& allTriangles,
    std::vector<CRTLight>& allLights,
    std::vector<CRTMaterial>& allMaterials
) {
    std::ifstream inputFile(filePath);
    if (!inputFile.is_open())
        throw std::runtime_error("Could not open scene file: " + filePath);

    std::stringstream buffer;
    buffer << inputFile.rdbuf();
    rapidjson::Document doc;
    doc.Parse(buffer.str().c_str());
    if (doc.HasParseError())
        throw std::runtime_error("Failed to parse JSON in scene file");

    // 1. Settings & Camera
    if (!doc.HasMember("settings")) throw std::runtime_error("Missing 'settings'");
    settings.loadFromJSON(doc["settings"]);

    if (!doc.HasMember("camera")) throw std::runtime_error("Missing 'camera'");
    camera.loadFromJSON(doc["camera"]);

    // 2. Load texture definitions
    std::unordered_map<std::string, TextureDef> textureDefs;
    if (doc.HasMember("textures") && doc["textures"].IsArray()) {
        for (const auto& t : doc["textures"].GetArray()) {
            if (!t.HasMember("name") || !t["name"].IsString())
                throw std::runtime_error("Texture missing 'name'");
            std::string name = t["name"].GetString();

            if (!t.HasMember("type") || !t["type"].IsString())
                throw std::runtime_error("Texture missing 'type'");
            std::string type = t["type"].GetString();

            TextureDef def;
            if (type == "albedo") {
                def.type = TextureDef::Type::ALBEDO;
                auto c = t["albedo"].GetArray();
                def.albedoColor = CRTColor(
                    static_cast<unsigned char>(c[0].GetFloat() * 255),
                    static_cast<unsigned char>(c[1].GetFloat() * 255),
                    static_cast<unsigned char>(c[2].GetFloat() * 255)
                );
            }
            else if (type == "checker") {
                def.type = TextureDef::Type::CHECKER;
                auto ca = t["color_A"].GetArray();
                auto cb = t["color_B"].GetArray();
                def.colorA = CRTColor(
                    static_cast<unsigned char>(ca[0].GetFloat() * 255),
                    static_cast<unsigned char>(ca[1].GetFloat() * 255),
                    static_cast<unsigned char>(ca[2].GetFloat() * 255)
                );
                def.colorB = CRTColor(
                    static_cast<unsigned char>(cb[0].GetFloat() * 255),
                    static_cast<unsigned char>(cb[1].GetFloat() * 255),
                    static_cast<unsigned char>(cb[2].GetFloat() * 255)
                );
                def.squareSize = t["square_size"].GetFloat();
            }
            else if (type == "edges") {
                def.type = TextureDef::Type::EDGES;
                auto ec = t["edge_color"].GetArray();
                auto ic = t["inner_color"].GetArray();
                def.edgeColor = CRTColor(
                    static_cast<unsigned char>(ec[0].GetFloat() * 255),
                    static_cast<unsigned char>(ec[1].GetFloat() * 255),
                    static_cast<unsigned char>(ec[2].GetFloat() * 255)
                );
                def.innerColor = CRTColor(
                    static_cast<unsigned char>(ic[0].GetFloat() * 255),
                    static_cast<unsigned char>(ic[1].GetFloat() * 255),
                    static_cast<unsigned char>(ic[2].GetFloat() * 255)
                );
                def.edgeWidth = t["edge_width"].GetFloat();
            }
            else if (type == "bitmap") {
                def.type = TextureDef::Type::BITMAP;
                def.filePath = t["file_path"].GetString();
            }
            else {
                throw std::runtime_error("Unsupported texture type: " + type);
            }

            textureDefs[name] = def;
        }
    }

    // 3. Materials (now passing textureDefs!)
    if (doc.HasMember("materials") && doc["materials"].IsArray()) {
        for (const auto& m : doc["materials"].GetArray()) {
            CRTMaterial mat;
            mat.loadFromJSON(m, textureDefs);
            allMaterials.push_back(std::move(mat));
        }
    }

    if (doc.HasMember("objects") && doc["objects"].IsArray()) {

    // Ако няма никакви материали – създаваме дефолтен
    if (allMaterials.empty()) {
        CRTMaterial defaultMat;
        defaultMat.albedo = CRTColor(200, 50, 200); // Отличителен цвят (лилав)
        defaultMat.type = CRTMaterial::Type::DIFFUSE;
        defaultMat.smoothShading = true;
        allMaterials.push_back(defaultMat);
        std::cerr << "[Info] No materials defined. Added default purple material.\n";
    }

    for (const auto& obj : doc["objects"].GetArray()) {
        CRTMesh mesh;
        mesh.loadFromJSON(obj);

        int matIdx = 0; // Дефолтен материал

        if (obj.HasMember("material_index") && obj["material_index"].IsInt()) {
            matIdx = obj["material_index"].GetInt();
            if (matIdx < 0 || matIdx >= (int)allMaterials.size()) {
                std::cerr << "[Warning] Invalid material_index " << matIdx
                          << ". Using default material (index 0).\n";
                matIdx = 0;
            }
        } else {
            std::cerr << "[Warning] Missing 'material_index' in object. Using default material (index 0).\n";
        }

        bool smooth = allMaterials[matIdx].smoothShading;
        std::vector<CRTTriangle> tris = mesh.toTriangles(smooth);

        for (auto& tri : tris)
            tri.setMaterialIndex(matIdx);

        allTriangles.insert(allTriangles.end(), tris.begin(), tris.end());
    }

}

    // 5. Lights
    if (doc.HasMember("lights") && doc["lights"].IsArray()) {
        for (const auto& lj : doc["lights"].GetArray()) {
            CRTLight light;
            light.loadFromJSON(lj);
            allLights.push_back(std::move(light));
        }
    }
}


int main() {

    CRTSettings settings;
    CRTCamera camera;
    std::vector<CRTTriangle> triangles;
    std::vector<CRTLight> lights;
    std::vector<CRTMaterial> materials;
    
    std::cout << "Loading scene..." << std::endl;
    loadSceneFromFile("scene1.crtscene", settings, camera, triangles, lights, materials);
    
    std::cout << "Scene loaded: " << triangles.size() << " triangles, "
              << materials.size() << " materials, " << lights.size() << " lights" << std::endl;

    std::cout << "Creating AutoAccTree acceleration structure..." << std::endl;
    AutoAccTree autoAccTree;
    
    high_resolution_clock::time_point buildStart = high_resolution_clock::now();
    
    autoAccTree.autoBuild(triangles,false);
    
    high_resolution_clock::time_point buildStop = high_resolution_clock::now();
    microseconds buildDuration = duration_cast<microseconds>(buildStop - buildStart);
    const double buildSeconds = buildDuration.count() / 1'000'000.0;
    
    std::cout << "AutoAccTree built in: " << buildSeconds << " seconds" << std::endl;
    
    // Start rendering timer
    high_resolution_clock::time_point renderStart = high_resolution_clock::now();
    
    // Render with AutoAccTree acceleration using BVH rendering
    std::cout << "Starting BVH-accelerated rendering..." << std::endl;
    renderTriangleSceneWithBVH(
        triangles,
        materials,
        lights,
        settings,
        camera,
        autoAccTree,  
        "scene1_auto.ppm"
    );
    
    high_resolution_clock::time_point renderStop = high_resolution_clock::now();
    microseconds renderDuration = duration_cast<microseconds>(renderStop - renderStart);
    const double renderSeconds = renderDuration.count() / 1'000'000.0;
    
    std::cout << "AutoAccTree-accelerated rendering completed in: " << renderSeconds << " seconds" << std::endl;
    std::cout << "Total time (build + render): " << (buildSeconds + renderSeconds) << " seconds" << std::endl;
    
    
    return 0;
}