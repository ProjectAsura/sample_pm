//-------------------------------------------------------------------------------------------------
// File : main.cpp
// Desc : Main Entry Point.
// Copyright(c) Project Asura. All right reserved.
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Inlcudes
//-------------------------------------------------------------------------------------------------
#include <r3d_math.h>
#include <r3d_camera.h>
#include <r3d_shape.h>
#include <vector>
#include <stb/stb_image_write.h>
#include "photonmap.h"


namespace {

//-------------------------------------------------------------------------------------------------
// Global Varaibles.
//-------------------------------------------------------------------------------------------------
const int     g_max_depth = 5;
const Vector3 g_back_ground (0.0,   0.0,    0.0);
const Sphere  g_spheres[] = {
    Sphere(1e5,     Vector3( 1e5 + 1.0,    40.8,          81.6), Vector3(0.25,  0.75,  0.25), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(-1e5 + 99.0,   40.8,          81.6), Vector3(0.25,  0.25,  0.75), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(50.0,          40.8,           1e5), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(50.0,          40.8,  -1e5 + 170.0), Vector3(0.01,  0.01,  0.01), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(50.0,           1e5,          81.6), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(50.0,   -1e5 + 81.6,          81.6), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(16.5,    Vector3(27.0,          16.5,          47.0), Vector3(0.75,  0.25,  0.25), ReflectionType::PerfectSpecular,  Vector3(0, 0, 0)),
    Sphere(16.5,    Vector3(73.0,          16.5,          78.0), Vector3(0.99,  0.99,  0.99), ReflectionType::Refraction,       Vector3(0, 0, 0)),
    Sphere(5.0,     Vector3(50.0,          81.6,          81.6), Vector3(),                   ReflectionType::Diffuse,          Vector3(12, 12, 12))
};
const int      g_lightId = 9;

//-------------------------------------------------------------------------------------------------
//      シーンとの交差判定を行います.
//-------------------------------------------------------------------------------------------------
inline bool intersect_scene(const Ray& ray, double* t, int* id)
{
    auto n = static_cast<int>(sizeof(g_spheres) / sizeof(g_spheres[0]));

    *t  = D_MAX;
    *id = -1;

    for (auto i = 0; i < n; ++i)
    {
        auto d = g_spheres[i].intersect(ray);
        if (d > D_HIT_MIN && d < *t)
        {
            *t  = d;
            *id = i;
        }
    }

    return (*t < D_HIT_MAX);
}

void generate_photon(Ray* ray, Vector3* flux, int count, Random* random)
{
    const auto r1 = D_2PI * random->get_as_double();
    const auto r2 = 1.0 - 2.0 * random->get_as_double();
    const auto t = sqrt(1.0 - r2 * r2);
    const auto light_pos = g_spheres[g_lightId].pos 
                        + (g_spheres[g_lightId].radius + D_HIT_MIN) * Vector3(t * cos(r1), t * sin(r1), r2);
    const auto normal = normalize(light_pos - g_spheres[g_lightId].pos);

    // 基底ベクトル生成.
    Vector3 u, v, w;
    w = normal;
    if (fabs(w.x) > 0.1)
    { u = normalize(cross(Vector3(0, 1, 0), w)); }
    else
    { u = normalize(cross(Vector3(1, 0, 1), w)); }

    v = cross(w, u);

    const auto u1 = D_2PI * random->get_as_double();
    const auto u2 = random->get_as_double();
    const auto u2s = sqrt(u2);

    const auto light_dir = normalize(u * cos(u1) * u2s + v * sin(u1) * u2s * w * sqrt(1.0 - u2));

    ray->pos = light_pos;
    ray->dir = light_dir;

    auto sr2 = g_spheres[g_lightId].radius * g_spheres[g_lightId].radius;
    *flux = g_spheres[g_lightId].emission * 4.0 * D_PI * sr2 * D_PI / count;
}

void create_photon_map(const int emit_count, PhotonMap& photon_map, Random* random)
{
    // 指定された数分フォトンを放出する.
    for (auto i = 0; i < emit_count; ++i)
    {
        Ray     ray;
        Vector3 flux;

        generate_photon(&ray, &flux, emit_count, random);
    }
}

//-------------------------------------------------------------------------------------------------
//      放射輝度を求めます.
//-------------------------------------------------------------------------------------------------
Vector3 radiance(const Ray& input_ray, int depth, Random* random)
{
    Vector3 L(0, 0, 0);
    Vector3 W(1, 1, 1);
    Ray ray(input_ray.pos, input_ray.dir);

    while(true)
    {
        double t;
        int   id;

        // シーンとの交差判定.
        if (!intersect_scene(ray, &t, &id))
        { break; }

        // 交差物体.
        const auto& obj = g_spheres[id];

        // 交差位置.
        const auto hit_pos = ray.pos + ray.dir * t;

        // 法線ベクトル.
        const auto normal  = normalize(hit_pos - obj.pos);

        // 物体からのレイの入出を考慮した法線ベクトル.
        const auto orienting_normal = (dot(normal, ray.dir) < 0.0) ? normal : -normal;

        auto p = max(obj.color.x, max(obj.color.y, obj.color.z));

        L += W * obj.emission;

        // 打ち切り深度に達したら終わり.
        if(depth > g_max_depth)
        {
            if (random->get_as_double() >= p)
            { break; }
        }
        else
        {
            p = 1.0;
        }

        switch (obj.type)
        {
        case ReflectionType::Diffuse:
            {
                // 基底ベクトル.
                Vector3 u, v, w;

                w = orienting_normal;
                if (abs(w.x) > 0.1)
                { u = normalize(cross(Vector3(0, 1, 0), w)); }
                else
                { u = normalize(cross(Vector3(1, 0, 0), w)); }
                v = cross(w, u);

                const auto r1 = D_2PI * random->get_as_double();
                const auto r2 = random->get_as_double();
                const auto r2s = sqrt(r2);

                auto dir = normalize(u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1.0 - r2));

                ray = Ray(hit_pos, dir);
                W *= (obj.color / p);
            }
            break;

        case ReflectionType::PerfectSpecular:
            {
                ray = Ray(hit_pos, reflect(ray.dir, normal));
                W *= (obj.color / p);
            }
            break;

        case ReflectionType::Refraction:
            {
                Ray reflect_ray = Ray(hit_pos, reflect(ray.dir, normal));
                auto into = dot(normal, orienting_normal) > 0.0;

                const auto nc = 1.0;
                const auto nt = 1.5;
                const auto nnt = (into) ? (nc / nt) : (nt / nc);
                const auto ddn = dot(ray.dir, orienting_normal);
                const auto cos2t = 1.0 - nnt * nnt * (1.0 - ddn * ddn);

                if (cos2t < 0.0)
                {
                    ray = reflect_ray;
                    W *= (obj.color / p);
                    break;
                }

                auto dir = normalize(ray.dir * nnt - normal * ((into) ? 1.0 : -1.0) * (ddn * nnt + sqrt(cos2t)));

                const auto a = nt - nc;
                const auto b = nt + nc;
                const auto R0 = (a * a) / (b * b);
                const auto c = 1.0 - ((into) ? -ddn : dot(dir, normal));
                const auto Re = R0 + (1.0 - R0) * pow(c, 5.0);
                const auto Tr = 1.0 - Re;
                const auto prob = 0.25 + 0.5 * Re;

                if (random->get_as_double() < prob)
                {
                    ray = reflect_ray;
                    W *= (obj.color * Re / prob) / p; 
                }
                else
                {
                    ray = Ray(hit_pos, dir);
                    W *= (obj.color * Tr / (1.0 - prob)) / p;
                }
            }
            break;
        }

        depth++;
    }

    return L;
}

//-------------------------------------------------------------------------------------------------
//      BMPファイルに保存します.
//-------------------------------------------------------------------------------------------------
void save_to_bmp(const char* filename, int width, int height, const double* pixels)
{
    std::vector<uint8_t> images;
    images.resize(width * height * 3);

    const double inv_gamma = 1.0 / 2.2;

    for(auto i=0; i<width * height * 3; i+=3)
    {
        auto r = pow(pixels[i + 0], inv_gamma);
        auto g = pow(pixels[i + 1], inv_gamma);
        auto b = pow(pixels[i + 2], inv_gamma);

        r = saturate(r);
        g = saturate(g);
        b = saturate(b);

        images[i + 0] = static_cast<uint8_t>( r * 255.0 + 0.5 );
        images[i + 1] = static_cast<uint8_t>( g * 255.0 + 0.5 );
        images[i + 2] = static_cast<uint8_t>( b * 255.0 + 0.5 );
    }

    stbi_write_bmp(filename, width, height, 3, images.data());
}

} // namespace


//-------------------------------------------------------------------------------------------------
//      メインエントリーポイントです.
//-------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    // レンダーターゲットのサイズ.
    int width   = 640;
    int height  = 480;
    int samples = 512;

    // カメラ用意.
    Camera camera(
        Vector3(50.0, 52.0, 295.6),
        normalize(Vector3(0.0, -0.042612, -1.0)),
        Vector3(0.0, 1.0, 0.0),
        0.5135,
        double(width) / double(height),
        130.0
    );

    // レンダーターゲット生成.
    std::vector<Vector3> image;
    image.resize(width * height);

    Random random(123456);

    // レンダーターゲットをクリア.
    for (size_t i = 0; i < image.size(); ++i)
    { image[i] = g_back_ground; }

    for(auto s = 0; s < samples; ++s)
    {
        printf_s("%.2lf%% complete\r", (double(s)/double(samples) * 100.0));

        for (auto y = 0; y < height; ++y)
        {
            for (auto x = 0; x < width; ++x)
            {   
                auto idx = y * width + x;

                auto fx = double(x) / double(width)  - 0.5;
                auto fy = double(y) / double(height) - 0.5;

                // Let's レイトレ！
                image[idx] += radiance(camera.emit(fx, fy), 0, &random) / samples;
            }
        }
    }

    // レンダーターゲットの内容をファイルに保存.
    save_to_bmp("image.bmp", width, height, &image.data()->x);

    // レンダーターゲットクリア.
    image.clear();

    return 0;
}