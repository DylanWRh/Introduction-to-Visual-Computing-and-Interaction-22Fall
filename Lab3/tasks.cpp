#include "Labs/3-Rendering/tasks.h"

namespace VCX::Labs::Rendering {

    glm::vec4 GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const & texture, glm::vec2 const & uvCoord) {
        if (texture.GetSizeX() == 1 || texture.GetSizeY() == 1) return texture.At(0, 0);
        glm::vec2 uv      = glm::fract(uvCoord);
        uv.x              = uv.x * texture.GetSizeX() - .5f;
        uv.y              = uv.y * texture.GetSizeY() - .5f;
        std::size_t xmin  = std::size_t(glm::floor(uv.x) + texture.GetSizeX()) % texture.GetSizeX();
        std::size_t ymin  = std::size_t(glm::floor(uv.y) + texture.GetSizeY()) % texture.GetSizeY();
        std::size_t xmax  = (xmin + 1) % texture.GetSizeX();
        std::size_t ymax  = (ymin + 1) % texture.GetSizeY();
        float       xfrac = glm::fract(uv.x), yfrac = glm::fract(uv.y);
        return glm::mix(glm::mix(texture.At(xmin, ymin), texture.At(xmin, ymax), yfrac), glm::mix(texture.At(xmax, ymin), texture.At(xmax, ymax), yfrac), xfrac);
    }

    glm::vec4 GetAlbedo(Engine::Material const & material, glm::vec2 const & uvCoord) {
        glm::vec4 albedo       = GetTexture(material.Albedo, uvCoord);
        glm::vec3 diffuseColor = albedo;
        return glm::vec4(glm::pow(diffuseColor, glm::vec3(2.2)), albedo.w);
    }

    /******************* 1. Ray-triangle intersection *****************/
    bool IntersectTriangle(Intersection & output, Ray const & ray, glm::vec3 const & p1, glm::vec3 const & p2, glm::vec3 const & p3) {
        // your code here
        auto determinant = [](glm::vec3 a, glm::vec3 b, glm::vec3 c) -> float {
            return glm::dot(a, glm::cross(b, c));
        };

        float A = determinant(p1 - p2, p1 - p3, ray.Direction);

        float u = determinant(p1 - ray.Origin, p1 - p3, ray.Direction) / A;
        if (u > 1 || u < 0) return false;

        float v = determinant(p1 - p2, p1 - ray.Origin, ray.Direction) / A;
        if (v > 1 || v < 0) return false;
        if (u + v > 1 || u + v < 0) return false;

        float t = determinant(p1 - p2, p1 - p3, p1 - ray.Origin) / A;
        output.u = u;
        output.v = v;
        output.t = t;

        return true;
    }

    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, int maxDepth, bool enableShadow) {
        glm::vec3 color(0.0f);
        glm::vec3 weight(1.0f);

        for (int depth = 0; depth < maxDepth; depth++) {
            auto rayHit = intersector.IntersectRay(ray);
            if (! rayHit.IntersectState) return color;
            const glm::vec3 pos       = rayHit.IntersectPosition;
            const glm::vec3 n         = rayHit.IntersectNormal;
            const glm::vec3 kd        = rayHit.IntersectAlbedo;
            const glm::vec3 ks        = rayHit.IntersectMetaSpec;
            const float     alpha     = rayHit.IntersectAlbedo.w;
            const float     shininess = rayHit.IntersectMetaSpec.w * 256;

            glm::vec3 result(0.0f);
            /******************* 2. Whitted-style ray tracing *****************/
            // your code here
            result = kd * intersector.InternalScene->AmbientIntensity; // Ambient

            for (const Engine::Light & light : intersector.InternalScene->Lights) {
                glm::vec3 l;
                float     attenuation;
                /******************* 3. Shadow ray *****************/
                if (light.Type == Engine::LightType::Point) {
                    l           = light.Position - pos;
                    attenuation = 1.0f / glm::dot(l, l);
                    if (enableShadow) {
                        // your code here
                        glm::vec3 new_ray_pos   = light.Position;
                        float     eps           = 1e-3;
                        while (1) {
                            auto rayHit_shadow = intersector.IntersectRay(Ray(new_ray_pos, -l));
                            if (! rayHit.IntersectState) break;
                            const glm::vec3 pos_shadow = rayHit_shadow.IntersectPosition;
                            if (glm::length(pos_shadow - pos) < eps) break;
                            else {
                                if (rayHit_shadow.IntersectAlbedo.w >= 0.2) {
                                    attenuation = 0;
                                    break;
                                } else new_ray_pos = pos_shadow;
                            }
                        }
                    }
                } else if (light.Type == Engine::LightType::Directional) {
                    l           = light.Direction;
                    attenuation = 1.0f;
                    if (enableShadow) {
                        // your code here
                        glm::vec3 new_ray_pos   = pos;
                        float     eps           = 1e-3;
                        while (1) {
                            auto rayHit_shadow = intersector.IntersectRay(Ray(new_ray_pos, -l));
                            if (! rayHit.IntersectState) break;
                            const glm::vec3 pos_shadow = rayHit_shadow.IntersectPosition;
                            if (glm::length(pos_shadow - pos) < eps) break;
                            else {
                                if (rayHit_shadow.IntersectAlbedo.w >= 0.2) {
                                    attenuation = 0;
                                    break;
                                } else new_ray_pos = pos_shadow;
                            }
                        }
                    }
                }

                /******************* 2. Whitted-style ray tracing *****************/
                // your code here
                result += light.Intensity * attenuation * kd * std::max(0.0f, glm::dot(glm::normalize(l), glm::normalize(n)));
                glm::vec3 _half_dir = glm::normalize(glm::normalize(ray.Origin - pos) + glm::normalize(l));
                result += light.Intensity * attenuation * ks * pow(std::max(0.0f, glm::dot(_half_dir, glm::normalize(n))), shininess);
            }

            if (alpha < 0.9) {
                // refraction
                // accumulate color
                glm::vec3 R = alpha * glm::vec3(1.0f);
                color += weight * R * result;
                weight *= glm::vec3(1.0f) - R;

                // generate new ray
                ray = Ray(pos, ray.Direction);
            } else {
                // reflection
                // accumulate color
                glm::vec3 R = ks * glm::vec3(0.5f);
                color += weight * (glm::vec3(1.0f) - R) * result;
                weight *= R;

                // generate new ray
                glm::vec3 out_dir = ray.Direction - glm::vec3(2.0f) * n * glm::dot(n, ray.Direction);
                ray               = Ray(pos, out_dir);
            }
        }

        return color;
    }
} // namespace VCX::Labs::Rendering