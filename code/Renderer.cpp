//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <thread>
#include <algorithm>
#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.



//use multiple threads
void Job(const int& step, const Scene& scene, const int thread_index, const float& scale, const float& imageAspectRatio, const Vector3f& eye_pos, std::vector<Vector3f>& framebuffer, const int& spp)
{
	uint32_t begin = step * thread_index;
	for (uint32_t j = begin; j < begin + step; ++j)
	{
		for (uint32_t i = 0; i < scene.width; ++i) {
			// generate primary ray direction
			float x = (2 * (i + 0.5) / (float)scene.width - 1) *
				imageAspectRatio * scale;
			float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

			Vector3f dir = normalize(Vector3f(-x, y, 1));
			for (int k = 0; k < spp; k++) {
				framebuffer[scene.width * (j - begin) + i] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
			}
		}
		UpdateProgress(j / (float)scene.height);
	}
}

void Renderer::Render(const Scene& scene)
{
	std::vector<Vector3f> framebuffer(scene.width * scene.height);

	float scale = tan(deg2rad(scene.fov * 0.5));
	float imageAspectRatio = scene.width / (float)scene.height;
	Vector3f eye_pos(278, 273, -800);

	// change the spp value to change sample ammount
	int spp = 16;
	std::cout << "SPP: " << spp << "\n";
	uint32_t thread_num = 4;
	std::vector<std::thread> threads;
	threads.reserve(thread_num);
	uint32_t step = (uint32_t)(scene.height / thread_num);
	std::vector<std::vector<Vector3f> > membuffer;
	membuffer.reserve(thread_num);
	for (int i = 0; i < thread_num; i++)
		membuffer.emplace_back(std::vector<Vector3f>(scene.width * step));

	for (uint32_t j = 0; j < thread_num; j++) {
		threads.emplace_back(std::thread(Job, std::ref(step), std::ref(scene), j, std::ref(scale), std::ref(imageAspectRatio), std::ref(eye_pos), std::ref(membuffer[j]), std::ref(spp)));
	}
	for (uint32_t j = 0; j < thread_num; j++) {
		threads[j].join();
	}
	for (int i = 0; i < thread_num; i++) {
		std::copy(membuffer[i].begin(), membuffer[i].end(), framebuffer.begin() + i * scene.width * step);
	}
	UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}


//use single thread
/*void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    int spp = 1;
    std::cout << "SPP: " << spp << "\n";
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                      imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            Vector3f dir = normalize(Vector3f(-x, y, 1));
            for (int k = 0; k < spp; k++){
                framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
            }
            m++;
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}*/
