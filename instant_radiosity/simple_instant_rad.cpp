// Copyright (c) 2014 hole
// This software is released under the MIT License (http://kagamin.net/hole/license.txt).
// A part of this software is based on smallpt (http://www.kevinbeason.com/smallpt/) and
// released under the MIT License (http://kagamin.net/hole/smallpt-license.txt).
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include <queue>
#include <vector>
#include <map>

const double PI = 3.14159265358979323846;
const double INF = 1e20;
const double EPS = 1e-6;
const double MaxDepth = 5;

// *** Other functions ***
inline double clamp(double x) { return x < 0 ? 0 : x > 1 ? 1 : x; }
inline int toInt(double x) { return int(pow(clamp(x), 1 / 2.2) * 255 + .5); }
inline double rand01() { return (double)rand() / RAND_MAX; }

// *** data structure ***
struct Vec {
	double x, y, z;
	Vec(const double x_ = 0, const double y_ = 0, const double z_ = 0) : x(x_), y(y_), z(z_) {}
	inline Vec operator+(const Vec &b) const {return Vec(x + b.x, y + b.y, z + b.z);}
	inline Vec operator-(const Vec &b) const {return Vec(x - b.x, y - b.y, z - b.z);}
	inline Vec operator*(const double b) const {return Vec(x * b, y * b, z * b);}
	inline Vec operator/(const double b) const {return Vec(x / b, y / b, z / b);}
	inline const double LengthSquared() const { return x * x + y * y + z * z; }
	inline const double Length() const { return sqrt(LengthSquared()); }
};
inline Vec operator*(double f, const Vec &v) { return v * f; }
inline Vec Normalize(const Vec &v) { return v / v.Length(); }
// Take the product of each element
inline const Vec Multiply(const Vec &v1, const Vec &v2) {
	return Vec(v1.x * v2.x, v1.y * v2.y, v1.z * v2.z);
}
inline const double Dot(const Vec &v1, const Vec &v2) {
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}
inline const Vec Cross(const Vec &v1, const Vec &v2) {
	return Vec((v1.y * v2.z) - (v1.z * v2.y), (v1.z * v2.x) - (v1.x * v2.z), (v1.x * v2.y) - (v1.y * v2.x));
}
typedef Vec Color;
const Color BackgroundColor(0.0, 0.0, 0.0);

struct Ray {
	Vec org, dir;
	Ray(const Vec org_, const Vec &dir_) : org(org_), dir(dir_) {}
};

enum ReflectionType {
	DIFFUSE,    // Complete diffusion surface. The so-called Lambertian surface.
	SPECULAR,   // Ideal mirror surface.
	REFRACTION, // An ideal glassy substance.
};

struct Sphere {
	double radius;
	Vec position;
	Color emission, color;
	ReflectionType ref_type;

	Sphere(const double radius_, const Vec &position_, const Color &emission_, const Color &color_, const ReflectionType ref_type_) :
		radius(radius_), position(position_), emission(emission_), color(color_), ref_type(ref_type_) {}
	// Returns the distance to the intersection with respect to the input ray. Returns 0 if it does not intersect.
	const double intersect(const Ray &ray) {
		Vec o_p = position - ray.org;
		const double b = Dot(o_p, ray.dir), det = b * b - Dot(o_p, o_p) + radius * radius;
		if (det >= 0.0) {
			const double sqrt_det = sqrt(det);
			const double t1 = b - sqrt_det, t2 = b + sqrt_det;
			if (t1 > EPS)		return t1;
			else if (t2 > EPS)	return t2;
		}
		return 0.0;
	}
};

// *** Scene data to render ****
// from smallpt
Sphere spheres[] = {
	Sphere(5.0, Vec(50.0, 75.0, 81.6), Color(12, 12, 12), Color(), DIFFUSE), // Illumination
	Sphere(1e5, Vec( 1e5 + 1, 40.8, 81.6), Color(), Color(0.75, 0.25, 0.25), DIFFUSE), // Left
	Sphere(1e5, Vec(-1e5 + 99, 40.8, 81.6), Color(), Color(0.25, 0.25, 0.75), DIFFUSE), // Right
	Sphere(1e5, Vec(50, 40.8, 1e5),     Color(), Color(0.75, 0.75, 0.75), DIFFUSE), // Background
	Sphere(1e5, Vec(50, 40.8, -1e5 + 170), Color(), Color(), DIFFUSE), // Foreground
	Sphere(1e5, Vec(50, 1e5, 81.6),    Color(), Color(0.75, 0.75, 0.75), DIFFUSE), // Floor
	Sphere(1e5, Vec(50, -1e5 + 81.6, 81.6), Color(), Color(0.75, 0.75, 0.75), DIFFUSE), // Ceiling
	Sphere(16.5, Vec(27, 16.5, 47),       Color(), Color(1, 1, 1)*.99, SPECULAR), // Mirror
	Sphere(16.5, Vec(73, 16.5, 78),       Color(), Color(1, 1, 1)*.99, REFRACTION), // Glass
	Sphere(16.5, Vec(20, 16.5, 98),       Color(), Color(0.75, 0.75, 0.75), DIFFUSE),
};
const int LightID = 0;

// *** Rendering function ***
// Crossing judgment function with the scene
inline bool intersect_scene(const Ray &ray, double *t, int *id) {
	const double n = sizeof(spheres) / sizeof(Sphere);
	*t  = INF;
	*id = -1;
	for (int i = 0; i < int(n); i ++) {

		double d = spheres[i].intersect(ray);
		if (d > 0.0 && d < *t) {
			*t  = d;
			*id = i;
		}
	}
	return *t < INF;
}

struct PointLight {
	Vec position;
	Color power;
	Vec normal;

	PointLight(const Vec& position_, const Vec& normal_, const Color& power_) :
		position(position_), normal(normal_), power(power_) {}
};

// VPL launch
void emit_vpl(const int vpl_num, std::vector<PointLight> *point_lights) {

	for (int i = 0; i < vpl_num; i ++) {
		// Fire from a light source
		// Sampling a point on the light source
		const double r1 = 2 * PI * rand01();
		const double r2 = 1.0 - 2.0 * rand01() ;

		const Vec light_pos = spheres[LightID].position + ((spheres[LightID].radius + EPS) * Vec(sqrt(1.0 - r2 * r2) * cos(r1), sqrt(1.0 - r2 * r2) * sin(r1), r2));

		const Vec normal = Normalize(light_pos - spheres[LightID].position);
		// Hemispherical sampling from a point on the light source
		Vec w, u, v;
		w = normal;
		if (fabs(w.x) > 0.1)
			u = Normalize(Cross(Vec(0.0, 1.0, 0.0), w));
		else
			u = Normalize(Cross(Vec(1.0, 0.0, 0.0), w));
		v = Cross(w, u);
		const double u1 = 2 * PI * rand01();
		const double u2 = rand01(), u2s = sqrt(u2);
		Vec light_dir = Normalize((u * cos(u1) * u2s + v * sin(u1) * u2s + w * sqrt(1.0 - u2)));

		Ray now_ray(light_pos, light_dir);
		Color now_flux = spheres[LightID].emission * 4.0 * PI * pow(spheres[LightID].radius, 2.0) * PI / vpl_num;

		point_lights->push_back(PointLight(light_pos, normal, now_flux));

		bool trace_end = false;
		for (; !trace_end;) {
			if (std::max(now_flux.x, std::max(now_flux.y, now_flux.z)) <= 0.0)
				break;

			double t; // Distance from Ray to the intersection of the scene
			int id;   // ID of objects in the intersecting scene
			if (!intersect_scene(now_ray, &t, &id))
				break;
			const Sphere &obj = spheres[id];
			const Vec hitpoint = now_ray.org + t * now_ray.dir; // Crossing position
			const Vec normal  = Normalize(hitpoint - obj.position); // Intersection normal
			const Vec orienting_normal = Dot(normal, now_ray.dir) < 0.0 ? normal : (-1.0 * normal); // Crossing normals (considering the entry and exit of rays from objects)

			switch (obj.ref_type) {
			case DIFFUSE: {
				point_lights->push_back(PointLight(hitpoint, orienting_normal, now_flux));

				// Russian roulette decides whether to reflect
				// As usual, the probability is arbitrary. This time we will use the average RGB reflectance according to the photon map book
				const double probability = (obj.color.x + obj.color.y + obj.color.z) / 3;
				if (probability > rand01()) { // 反射
					// Create an orthonormal basis (w, u, v) relative to the direction of orienting_normal. Fly the next ray within the hemisphere to this basis.
					Vec w, u, v;
					w = orienting_normal;
					if (fabs(w.x) > 0.1)
						u = Normalize(Cross(Vec(0.0, 1.0, 0.0), w));
					else
						u = Normalize(Cross(Vec(1.0, 0.0, 0.0), w));
					v = Cross(w, u);
					// Focused sampling using cosine terms
					const double r1 = 2 * PI * rand01();
					const double r2 = rand01(), r2s = sqrt(r2);
					Vec dir = Normalize((u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1.0 - r2)));

					now_ray = Ray(hitpoint, dir);
					now_flux = Multiply(now_flux, obj.color) / probability;

					continue;
				} else { // Absorption (i.e. end tracking here)
					trace_end = true;
					continue;
				}
			} break;
			case SPECULAR: {
				now_ray = Ray(hitpoint, now_ray.dir - normal * 2.0 * Dot(normal, now_ray.dir));
				now_flux = Multiply(now_flux, obj.color);
				continue;
			} break;
			case REFRACTION: {
				Ray reflection_ray = Ray(hitpoint, now_ray.dir - normal * 2.0 * Dot(normal, now_ray.dir));
				bool into = Dot(normal, orienting_normal) > 0.0; // Whether the ray leaves or enters the object

				// Snell's law
				const double nc = 1.0; // Refractive index of vacuum
				const double nt = 1.5; // Refractive index of the object
				const double nnt = into ? nc / nt : nt / nc;
				const double ddn = Dot(now_ray.dir, orienting_normal);
				const double cos2t = 1.0 - nnt * nnt * (1.0 - ddn * ddn);

				if (cos2t < 0.0) { // Total internal reflection
					now_ray = reflection_ray;
					now_flux = Multiply(now_flux, obj.color);
					continue;
				}
				// Direction of refraction
				Vec tdir = Normalize(now_ray.dir * nnt - normal * (into ? 1.0 : -1.0) * (ddn * nnt + sqrt(cos2t)));
				const double probability  = 0.5;

				// Track either refraction or reflection.
				// Determined by Russian roulette.
				if (rand01() < probability) { // reflection
					now_ray = Ray(hitpoint, tdir);
					now_flux = Multiply(now_flux, obj.color);
					continue;
				} else { // refraction
					now_ray = reflection_ray;
					now_flux = Multiply(now_flux, obj.color);
					continue;
				}
			} break;
			}
		}
	}
	std::cout << "VPL: " << point_lights->size() << std::endl;
}


// Find the radiance from the ray direction
Color radiance(const Ray &ray, const int depth, std::vector<PointLight> *point_lights, const double bias) {
	double t; // Distance from Ray to the intersection of the scene
	int id;   // ID of objects in the intersecting scene
	if (!intersect_scene(ray, &t, &id))
		return BackgroundColor;

	const Sphere &obj = spheres[id];
	const Vec hitpoint = ray.org + t * ray.dir; // 交差位置
	const Vec normal  = Normalize(hitpoint - obj.position); // 交差位置の法線
	const Vec orienting_normal = Dot(normal, ray.dir) < 0.0 ? normal : (-1.0 * normal); // Crossing normals (considering the entry and exit of rays from objects)

	// Get the one with the maximum color reflectance. Used in Russian roulette.
	// Russian roulette threshold is arbitrary, but it is better to use color reflectance etc.
	double russian_roulette_probability = std::max(obj.color.x, std::max(obj.color.y, obj.color.z));
	// After tracking Ray over a certain amount, run Russian roulette and decide whether to stop tracking
	if (depth > MaxDepth) {
		if (rand01() >= russian_roulette_probability)
			return obj.emission;
	} else
		russian_roulette_probability = 1.0; // Russian roulette did not run

	switch (obj.ref_type) {
	case DIFFUSE: {
		if (id == LightID)
			return obj.emission;

		// Calculate the impact from VPL
		Color accum;

		for (int i = 0; i < point_lights->size(); i ++) {
			int id;
			double t;
			const double dist = (hitpoint - (*point_lights)[i].position).LengthSquared();
			const Vec to0 = Normalize(hitpoint - (*point_lights)[i].position);
			const Vec to1 = Normalize((*point_lights)[i].position - hitpoint);
			intersect_scene(Ray((*point_lights)[i].position, to0), &t, &id);
			const double c0 = Dot((*point_lights)[i].normal, to0);
			const double c1 = Dot(orienting_normal, to1);
			const double dist2 = (hitpoint - (*point_lights)[i].position).LengthSquared();
			if (c0 >= 0 &&  c1 >= 0 && fabs(sqrt(dist2) - t) < EPS) {
				accum = accum + (c0 * c1 / (dist2 + bias)) * Multiply(obj.color / PI, (*point_lights)[i].power) / PI;
			}
		}

		return accum;
	} break;
	case SPECULAR: {
		// If it hits a perfect mirror surface, it will receive radiance from the direction of reflection.
		return obj.emission + radiance(Ray(hitpoint, ray.dir - normal * 2.0 * Dot(normal, ray.dir)), depth + 1, point_lights, bias);
	} break;
	case REFRACTION: {
		Ray reflection_ray = Ray(hitpoint, ray.dir - normal * 2.0 * Dot(normal, ray.dir));
		bool into = Dot(normal, orienting_normal) > 0.0; // レイがオブジェクトから出るのか、入るのか

		// Snell's law
		const double nc = 1.0; // Refractive index of vacuum
		const double nt = 1.5; // Refractive index of the object
		const double nnt = into ? nc / nt : nt / nc;
		const double ddn = Dot(ray.dir, orienting_normal);
		const double cos2t = 1.0 - nnt * nnt * (1.0 - ddn * ddn);

		if (cos2t < 0.0) { // Total internal reflection
			// Receives radiance from the reflection direction
			return obj.emission + Multiply(obj.color,
			                               radiance(Ray(hitpoint, ray.dir - normal * 2.0 * Dot(normal, ray.dir)), depth + 1, point_lights, bias)) / russian_roulette_probability;
		}
		// Direction of refraction
		Vec tdir = Normalize(ray.dir * nnt - normal * (into ? 1.0 : -1.0) * (ddn * nnt + sqrt(cos2t)));

		// Approximation of Fresnel's reflectance coefficient by Schlick
		const double a = nt - nc, b = nt + nc;
		const double R0 = (a * a) / (b * b);
		const double c = 1.0 - (into ? -ddn : Dot(tdir, normal));
		const double Re = R0 + (1.0 - R0) * pow(c, 5.0);
		const double Tr = 1.0 - Re; // Amount of light carried by refracted light
		const double probability  = 0.25 + 0.5 * Re;

		// If you track a ray above a certain level, track either refraction or reflection. (Otherwise, Ray will increase exponentially)
		// Determined by Russian roulette.
		if (depth > 2) {
			if (rand01() < probability) { // reflection
				return obj.emission +
				       Multiply(obj.color, radiance(reflection_ray, depth + 1, point_lights, bias) * Re)
				       / probability
				       / russian_roulette_probability;
			} else { // refraction
				return obj.emission +
				       Multiply(obj.color, radiance(Ray(hitpoint, tdir), depth + 1, point_lights, bias) * Tr)
				       / (1.0 - probability)
				       / russian_roulette_probability;
			}
		} else { // Track both refraction and reflection
			return obj.emission +
			       Multiply(obj.color, radiance(reflection_ray, depth + 1, point_lights, bias) * Re
			                + radiance(Ray(hitpoint, tdir), depth + 1, point_lights, bias) * Tr) / russian_roulette_probability;
		}
	} break;
	}

	return Color();
}


// *** Function to output in .hdr format ***
struct HDRPixel {
	unsigned char r, g, b, e;
	HDRPixel(const unsigned char r_ = 0, const unsigned char g_ = 0, const unsigned char b_ = 0, const unsigned char e_ = 0) :
		r(r_), g(g_), b(b_), e(e_) {};
	unsigned char get(int idx) {
		switch (idx) {
		case 0: return r;
		case 1: return g;
		case 2: return b;
		case 3: return e;
		} return 0;
	}

};

// Convert double RGB elements for .hdr format
HDRPixel get_hdr_pixel(const Color &color) {
	double d = std::max(color.x, std::max(color.y, color.z));
	if (d <= 1e-32)
		return HDRPixel();
	int e;
	double m = frexp(d, &e); // d = m * 2^e
	d = m * 256.0 / d;
	return HDRPixel(color.x * d, color.y * d, color.z * d, e + 128);
}

// Export function
void save_hdr_file(const std::string &filename, const Color* image, const int width, const int height) {
	FILE *fp = fopen(filename.c_str(), "wb");
	if (fp == NULL) {
		std::cerr << "Error: " << filename << std::endl;
		return;
	}
	// Export data according to .hdr format
	// Header
	unsigned char ret = 0x0a;
	fprintf(fp, "#?RADIANCE%c", (unsigned char)ret);
	fprintf(fp, "# Made with 100%% pure HDR Shop%c", ret);
	fprintf(fp, "FORMAT=32-bit_rle_rgbe%c", ret);
	fprintf(fp, "EXPOSURE=1.0000000000000%c%c", ret, ret);

	// Luminance value export
	fprintf(fp, "-Y %d +X %d%c", height, width, ret);
	for (int i = height - 1; i >= 0; i --) {
		std::vector<HDRPixel> line;
		for (int j = 0; j < width; j ++) {
			HDRPixel p = get_hdr_pixel(image[j + i * width]);
			line.push_back(p);
		}
		fprintf(fp, "%c%c", 0x02, 0x02);
		fprintf(fp, "%c%c", (width >> 8) & 0xFF, width & 0xFF);
		for (int i = 0; i < 4; i ++) {
			for (int cursor = 0; cursor < width;) {
				const int cursor_move = std::min(127, width - cursor);
				fprintf(fp, "%c", cursor_move);
				for (int j = cursor;  j < cursor + cursor_move; j ++)
					fprintf(fp, "%c", line[j].get(i));
				cursor += cursor_move;
			}
		}
	}

	fclose(fp);
}

int main(int argc, char **argv) {
	int width = 320;
	int height = 240;
	int vpl = 128;
	double bias = 128;
	auto startTime = std::chrono::high_resolution_clock::now();
	// Camera position
	Ray camera(Vec(50.0, 52.0, 295.6), Normalize(Vec(0.0, -0.042612, -1.0)));
	// Vector in x, y direction of the screen in the scene
	Vec cx = Vec(width * 0.5135 / height);
	Vec cy = Normalize(Cross(cx, camera.dir)) * 0.5135;
	Color *image = new Color[width * height];

	std::vector<PointLight> point_lights;

	emit_vpl(vpl, &point_lights);

//#pragma omp parallel for schedule(dynamic, 1)
	for (int y = 0; y < height; y ++) {
		std::cerr << "Rendering " << (100.0 * y / (height - 1)) << "%" << std::endl;
		srand(y * y * y);
		for (int x = 0; x < width; x ++) {
			int image_index = y * width + x;
			image[image_index] = Color();

			// 2x2 subpixel sampling
			for (int sy = 0; sy < 2; sy ++) {
				for (int sx = 0; sx < 2; sx ++) {
					Color accumulated_radiance = Color();
					// Sampling by tent filter
					// Rather than sampling uniformly over the pixel range, create a bias so that many samples are gathered near the center of the pixel.
					const double r1 = 2.0 * rand01(), dx = r1 < 1.0 ? sqrt(r1) - 1.0 : 1.0 - sqrt(2.0 - r1);
					const double r2 = 2.0 * rand01(), dy = r2 < 1.0 ? sqrt(r2) - 1.0 : 1.0 - sqrt(2.0 - r2);
					Vec dir = cx * (((sx + 0.5 + dx) / 2.0 + x) / width - 0.5) +
					          cy * (((sy + 0.5 + dy) / 2.0 + y) / height - 0.5) + camera.dir;
					accumulated_radiance = accumulated_radiance +
					                       radiance(Ray(camera.org + dir * 130.0, Normalize(dir)), 0, &point_lights, bias);
					image[image_index] = image[image_index] + accumulated_radiance;
				}
			}
		}
	}
	auto endTime = std::chrono::high_resolution_clock::now();
	auto timeDiff = endTime - startTime;
	std::cout << std::chrono::duration <double, std::milli> (timeDiff).count() << " ms" << std::endl;
	// Output in .hdr format
	save_hdr_file(std::string("image.hdr"), image, width, height);
}
