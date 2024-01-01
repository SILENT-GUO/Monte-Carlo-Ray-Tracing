#include <iostream>
#include <vector>
#include <thread>
#include "Common.h"
#include "Scene.h"
#include "Camera.h"
#include "Material.h"
#include "Hittable.h"
#include "Utils/lodepng.h"

const int kMaxTraceDepth = 5;

Color TraceRay(const Ray& ray, const std::vector<LightSource>& light_sources, const Hittable& scene, int trace_depth);
Color Shade(const std::vector<LightSource>& light_sources,
    const Hittable& hittable_collection,
    const HitRecord& hit_record,
    int trace_depth);


int main() {
    // TODO: Set your workdir (absolute path) here. Don't forget the last slash
    const std::string work_dir("/Users/silent/Documents/Year4/Semester 1/COMP3271/PAs/PA3_Release/");

    // Construct scene
    Scene scene(work_dir, "scene/teapot_area_light.toml");
    const Camera& camera = scene.camera_;
    int width = camera.width_;
    int height = camera.height_;

    std::vector<unsigned char> image(width * height * 4, 0);
    float* image_buffer = new float[width * height * 3];
    for (int i = 0; i < width * height * 3; ++i) {
		image_buffer[i] = 0.f;
	}
    // original value for spp is 128
    int spp = 72;
    int NUM_THREAD = 1;

    
    float progress = 0.f;
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            Color color(0.f, 0.f, 0.f);
            for (int i = 0; i < spp; i++)
            {
                float bias_x = get_random_float();
                float bias_y = get_random_float();
                Ray ray = camera.RayAt(float(x) + bias_x, float(y) + bias_y);
                color += TraceRay(ray, scene.light_sources_, scene.hittable_collection_, 1);
            }
            color /= float(spp);
            int idx = 3 * ((height - y - 1) * width + x);
            image_buffer[idx] += color.r;
            image_buffer[idx + 1] += color.g;
            image_buffer[idx + 2] += color.b;


            float curr_progress = float(x * height + y) / float(height * width);
            if (curr_progress > progress + 0.05f) {
                progress += 0.05f;
                std::cout << "Progress (thread " << 1 << "/" << NUM_THREAD << "): " << progress << std::endl;
            }
        }
    }

    // copy from image_buffer to image
    for (int i = 0; i < width * height; ++i) {
        for (int j = 0; j < 3; ++j) {
			image[4 * i + j] = (uint8_t)(glm::min(image_buffer[3 * i + j], 1.f - 1e-5f) * 256.f);
		}
		image[4 * i + 3] = 255;
	}
    

    std::vector<unsigned char> png;
    unsigned error = lodepng::encode(png, image, width, height);
    lodepng::save_file(png, work_dir + "outputs/teapot_area_light.png");
}


Vec toWorld(const Vec& localRay, const Vec& N) {
    // TODO: add your code here.
    // This function transforms a vector from local coordinate to world coordinate.
    Vec C;
    if(abs(N.x) > abs(N.y)){
        C = Vec(N.z, 0, -N.x) / sqrt(N.x * N.x + N.z * N.z);

    }else{
        // we can require C.x is zero
        C = Vec(0, N.z, -N.y) / sqrt(N.y * N.y + N.z * N.z);
    }
    Vec B = glm::cross(N, C);
    glm::mat3 M = glm::mat3(C, N, B);
    return glm::normalize(M * localRay);
}

Vec SampleHemisphere(const HitRecord& hit_record)
{
    // TODO: add your code here.
    // This function randomly samples a direction on the hemisphere.
    // It will calls toWorld() to transform the local coordinate to world coordinate.
    float angle_theta = get_random_float() * 2 * M_PI;
    float angle_phi = get_random_float() * M_PI / 2;
    Vec localRay = Vec(sin(angle_phi) * cos(angle_theta), sin(angle_phi) * sin(angle_theta), cos(angle_phi));

    // transform localRay to worldRay
    Vec normal = hit_record.normal;
    Vec worldRay = toWorld(localRay, normal);
    return worldRay;
}



Color Shade(const std::vector<LightSource>& light_sources,
            const Hittable& hittable_collection,
            const HitRecord& hit_record,
            int trace_depth) {
    // TODO: Add your code here.
    Color color(0.f, 0.f, 0.f);
    HitRecord record = hit_record;
    HitRecord dummy, sampleRecord, dummy_sample;
    float samplePdf;

    //ambient
    color = hit_record.material.ambient * hit_record.material.k_a;

    //diffuse terms dur to lighting objects
    hittable_collection.Sample(&sampleRecord, samplePdf);
    Vec shadow_sample = glm::normalize(sampleRecord.position - hit_record.position);
    if(glm::dot(hit_record.normal, shadow_sample) > 0){
        Ray temp_ray_sample(hit_record.position, shadow_sample);
        bool isHit = hittable_collection.Hit(temp_ray_sample, &dummy_sample);
        //if hit point is the emitted object, set it to true
        if(isHit && dummy_sample.position.x - sampleRecord.position.x < 1e-3f
        && dummy_sample.position.y - sampleRecord.position.y < 1e-3f
        && dummy_sample.position.z - sampleRecord.position.z < 1e-3f){
            isHit = false;
        }
        if(!isHit){
//            std::cout << "not hit" << std::endl;
            float cos_theta = glm::dot(hit_record.normal, glm::normalize(sampleRecord.position - hit_record.position));
            float cos_theta_prime = glm::dot(sampleRecord.normal, glm::normalize(hit_record.position - sampleRecord.position));
            float distance = glm::distance(hit_record.position, sampleRecord.position);
            float G = cos_theta * cos_theta_prime / (distance * distance);
            color += hit_record.material.diffuse * hit_record.material.k_d * G * sampleRecord.emission / samplePdf;
        }
    }


    //recurse through all light sources
    for (int i = 0; i < light_sources.size(); i++){
        //get light source
        LightSource light = light_sources[i];
        //get light direction
        Vec shadow_ray = glm::normalize(light.position - hit_record.position);
        if(glm::dot(hit_record.normal, shadow_ray) > 0){
            Ray temp_ray(hit_record.position, shadow_ray);
            if(!hittable_collection.Hit(temp_ray, &dummy)){
                Vec shadow_ray_reflection = glm::normalize(glm::reflect(-shadow_ray, hit_record.normal));
                color += 1.0f * light.intensity * (hit_record.material.diffuse * hit_record.material.k_d * glm::dot(hit_record.normal, shadow_ray)
                                                   + hit_record.material.specular * hit_record.material.k_s * glm::pow(glm::dot(shadow_ray_reflection, -glm::normalize(record.in_direction)), hit_record.material.sh));
            }
        }
    }
    //ray tracing for kMaxTraceDepth
    if(trace_depth < kMaxTraceDepth){
        if(hit_record.material.k_s > 0){
            //use sample hemisphere to get reflection ray by random sampling
            Ray reflection_ray = Ray(hit_record.position, SampleHemisphere(hit_record));
//            Ray reflection_ray(hit_record.position, record.reflection);
            color += hit_record.material.k_s * TraceRay(reflection_ray, light_sources, hittable_collection, trace_depth + 1);
        }
    }
    //clamping color
//    for (int i = 0; i < 3; i++){
//        if(color[i] > 1.0f){
//            color[i] = 1.0f;
//        }
//    }
    return color;
}

Color TraceRay(const Ray& ray,
               const std::vector<LightSource>& light_sources,
               const Hittable& hittable_collection,
               int trace_depth) {
    // TODO: Add your code here.
    HitRecord record;
    Color color(0.0f, 0.0f, 0.0f);
    if(hittable_collection.Hit(ray, &record)){
        color = Shade(light_sources, hittable_collection, record, trace_depth);
    }
    return color;
}