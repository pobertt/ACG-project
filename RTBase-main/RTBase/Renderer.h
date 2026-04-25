#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>

struct VPL {
	Vec3 position;
	Vec3 normal;
	Colour intensity;
};

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	std::thread **threads;
	int numProcs;

	std::vector<VPL> vpls;

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread*[numProcs];
		samplers = new MTRandom[numProcs];
		clear();
	}
	void clear()
	{
		film->clear();
	}
	void connectToCamera(ShadingData& sd, Colour col, float totalPaths)
	{
		float x, y;

		// safety checks
		if (!scene->camera.projectOntoCamera(sd.x, x, y)) return;
		if (x < 0 || x >= film->width || y < 0 || y >= film->height) return;
		if (!scene->visible(sd.x, scene->camera.origin)) return;

		// dist/direction
		Vec3 v = scene->camera.origin - sd.x;
		float distSq = v.lengthSq();
		Vec3 dir = v / sqrtf(distSq);

		Colour f = sd.bsdf->evaluate(sd, dir);

		float cosThetaS = Dot(sd.sNormal, dir);
		float cosThetaC = Dot(scene->camera.viewDirection, -dir);

		if (cosThetaS > 0.0f && cosThetaC > 0.0f) {
			float g = (cosThetaS * cosThetaC) / std::max(0.01f, distSq);
			film->splat(x, y, (col * f * g) / totalPaths);
		}
	}
	
	void lightTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler, int depth, float totalPaths)
	{
		if (depth > 32) return;

		// fire 
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t >= FLT_MAX) return;

		ShadingData sd = scene->calculateShadingData(intersection, r);

		// connect to cam (only for diffuse/glossy)
		if (!sd.bsdf->isPureSpecular()) {
			connectToCamera(sd, pathThroughput * Le, totalPaths);
		}

		// find next direction
		Colour indirect;
		float pdf;
		Vec3 nextDir = sd.bsdf->sample(sd, sampler, indirect, pdf);

		if (pdf <= 0.0f) return;

		float cosTheta = Dot(sd.sNormal, nextDir);
		if (cosTheta <= 0.0f && !sd.bsdf->isPureSpecular()) return;

		if (sd.bsdf->isPureSpecular()) {
			// glass/mirror
			pathThroughput = pathThroughput * indirect;
		}
		else {
			// diffuse
			Colour f = sd.bsdf->evaluate(sd, nextDir);
			pathThroughput = pathThroughput * (f * fabsf(cosTheta)) / pdf;
		}

		// russian roulette 
		float prob = std::min(1.0f, std::max(pathThroughput.r, std::max(pathThroughput.g, pathThroughput.b)));
		if (prob < 0.05f || sampler->next() > prob) return;

		pathThroughput = pathThroughput / prob;

		// fire next ray
		Vec3 offsetNormal = Dot(nextDir, sd.sNormal) > 0.0f ? sd.sNormal : -sd.sNormal;
		Ray nextRay(sd.x + (offsetNormal * EPSILON), nextDir);

		// recursion 
		lightTracePath(nextRay, pathThroughput, Le, sampler, depth + 1, totalPaths);
	}

	void lightTrace(Sampler* sampler, float totalPaths)
	{
		// pick a random light
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);

		if (light == NULL || pmf <= 0.0f) return;

		// random starting pos and direction
		float posPdf;
		float dirPdf;
		Vec3 pos = light->samplePositionFromLight(sampler, posPdf);
		Vec3 dir = light->sampleDirectionFromLight(sampler, dirPdf);

		if (posPdf <= 0.0f || dirPdf <= 0.0f) return;

		Colour Le(0.0f, 0.0f, 0.0f);
		if (light->isArea()) {
			Le = ((AreaLight*)light)->emission;
		}
		else {
			Le = light->evaluate(dir);
		}

		// monte carlo weighting
		Colour pathThroughput = Colour(1.0f, 1.0f, 1.0f) / (pmf * posPdf * dirPdf);

		// lambert cosine law
		if (light->isArea()) {
			pathThroughput = pathThroughput * std::max(0.0f, Dot(((AreaLight*)light)->triangle->gNormal(), dir));
		}

		// create ray obj
		Ray r(pos + (dir * EPSILON), dir);

		lightTracePath(r, pathThroughput, Le, sampler, 0, totalPaths);
	}
	void genVPLs(int paths, int maxDepth) {
		vpls.clear();

		std::cout << "generating: " << paths << " VPLs" << std::endl;

		// shoot ray from light source
		for (int i = 0; i < paths; i++) {
			float pmf;
			// randomly choose 1 light to shoot a ray from
			Light* light = scene->sampleLight(&samplers[0], pmf);
			if (light == NULL || pmf <= 0.0f) continue;

			// random starting pos and direction
			float posPdf;
			float dirPdf;
			Vec3 pos = light->samplePositionFromLight(&samplers[0], posPdf);
			Vec3 dir = light->sampleDirectionFromLight(&samplers[0], dirPdf);

			// colour being emitted
			Colour intensity(0.0f, 0.0f, 0.0f);
			if (light->isArea()) {
				intensity = ((AreaLight*)light)->emission;
			}
			else {
				intensity = light->evaluate(dir);
			}

			// monte carlo weightuing 
			intensity = intensity / (pmf * paths * posPdf * dirPdf);

			// if area light emission is affected by lambert cosine law
			if (light->isArea()) {
				intensity = intensity * std::max(0.0f, Dot(((AreaLight*)light)->triangle->gNormal(), dir));
			}

			// create ray
			Ray r(pos + (dir * EPSILON), dir);

			// ray bouncing 
			for (int depth = 0; depth < maxDepth; depth++) {
				// fire ray
				IntersectionData intersection = scene->traverse(r);

				// ray missed
				if (intersection.t >= FLT_MAX) {
					break;
				}

				// hit = get data 
				ShadingData sd = scene->calculateShadingData(intersection, r);

				// vpl
				if (!sd.bsdf->isPureSpecular()) {
					VPL vpl;
					vpl.position = sd.x;
					vpl.normal = sd.sNormal;
					vpl.intensity = intensity;

					if (std::isnan(intensity.r) || std::isnan(intensity.g) || std::isnan(intensity.b) ||
						std::isinf(intensity.r) || std::isinf(intensity.g) || std::isinf(intensity.b)) {
						break;
					}

					vpls.push_back(vpl);
				}

				Colour indirect;
				float pdf;

				// where ray goes next
				Vec3 nextDir = sd.bsdf->sample(sd, &samplers[0], indirect, pdf);
				float cosTheta = Dot(sd.sNormal, nextDir);
				// kill if failed
				if (pdf <= 0.0f) {
					break;
				}
				else if (cosTheta <= 0.0f && !sd.bsdf->isPureSpecular()) {
					break;
				}

				// update ray for next bounce
				if (sd.bsdf->isPureSpecular()) {
					// glass/mirrors
					intensity = intensity * indirect;
				}
				else {
					Colour f = sd.bsdf->evaluate(sd, nextDir);
					intensity = intensity * (f * fabsf(cosTheta)) / pdf;
				}

				// russian roulette
				float prob = std::min(1.0f, std::max(intensity.r, std::max(intensity.g, intensity.b)));
				if (prob < 0.05f) break;
				if (samplers[0].next() > prob) break;
				intensity = intensity / prob;

				// ray for next loop
				r = Ray(sd.x + ((Dot(nextDir, sd.sNormal) > 0.0f ? sd.sNormal : -sd.sNormal) * EPSILON), nextDir);
			}
		}
	}
	Colour computeDirectVPL(ShadingData shadingData, Sampler* sampler)
	{
		// glass/mirrors
		if (shadingData.bsdf->isPureSpecular() == true) {
			return Colour(0.0f, 0.0f, 0.0f);
		}
		if (vpls.empty()) return Colour(0.0f, 0.0f, 0.0f);

		Colour light(0.0f, 0.0f, 0.0f);

		int randomVplIndex = std::min((int)(sampler->next() * vpls.size()), (int)vpls.size() - 1);
		VPL vpl = vpls[randomVplIndex];

		// distance/direction
		Vec3 wi = vpl.position - shadingData.x;
		float dist = wi.length();
		wi = wi.normalize();

		// shadow ray
		if (!scene->visible(shadingData.x, vpl.position)) {
			return Colour(0.0f, 0.0f, 0.0f);
		}

		// lamberts cosine law
		float cosTheta = Dot(shadingData.sNormal, wi);
		float cosThetaL = Dot(vpl.normal, -wi);

		// accumulate clamped VPL light.
		if (cosTheta > 0.0f && cosThetaL > 0.0f) {
			float distSq = std::max(0.1f, dist * dist);
			float g = (cosTheta * cosThetaL) / distSq;

			// eval how mat reacts to light
			Colour f = shadingData.bsdf->evaluate(shadingData, wi);

			light = light + (vpl.intensity * f * g) * (float)vpls.size();
		}
			
		return light;
	}
	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Compute direct lighting here
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);

		if (light == NULL || pmf <= 0.0f) { return Colour(0.0f, 0.0f, 0.0f); }

		Colour emittedColour;
		float pdf;
		Vec3 sampleVec = light->sample(shadingData, sampler, emittedColour, pdf);

		if (pdf <= 0.0f) { return Colour(0.0f, 0.0f, 0.0f); }

		Vec3 wi = sampleVec - shadingData.x;
		float dist = wi.length();
		wi = wi.normalize();

		if (!scene->visible(shadingData.x, sampleVec)) { return Colour(0.0f, 0.0f, 0.0f); };

		float cosThetaOut = Dot(shadingData.sNormal, wi);
		float g = 0.0f;

		// Light pdf solid angle 
		float pdfSA = pdf;

		if (light->isArea()) {
			AreaLight* areaLight = (AreaLight*)light;
			Vec3 lightNormal = areaLight->triangle->gNormal();
			float cosThetaLight = Dot(lightNormal, -wi);
			
			if (cosThetaOut > 0 && cosThetaLight > 0) {
				float distSquared = std::max(0.1f, dist * dist);
				g = (cosThetaOut * cosThetaLight) / distSquared;

				pdfSA = (pdf * distSquared) / cosThetaLight;
			}
		}
		else {
			if (cosThetaOut > 0.0f) {
				g = cosThetaOut;
			}
		}
		if (g < 0.0f) {
			return Colour(0.0f, 0.0f, 0.0f);
		}

		Colour f = shadingData.bsdf->evaluate(shadingData, wi);

		float bsdfPdf = shadingData.bsdf->PDF(shadingData, wi);
		float totalPdf = pdfSA * pmf;

		float misWeight = 1.0f;
		if (totalPdf > 0.0f) {
			float PdfSq = totalPdf * totalPdf;
			float bsdfPdfSq = bsdfPdf * bsdfPdf;
			misWeight = PdfSq / (PdfSq + bsdfPdfSq);
		}


		return (emittedColour * f * g * misWeight) / (pdf * pmf);
	}
	Colour computeIndirect(ShadingData shadingData, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		// russian roulette
		float prob = std::max(pathThroughput.r, std::max(pathThroughput.g, pathThroughput.b));
		if (sampler->next() > prob) {
			return Colour(0.0f, 0.0f, 0.0f);
		}
		pathThroughput = pathThroughput / prob;

		Colour indirect;
		float pdf;
		Vec3 nextDir = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);

		float cosTheta = Dot(shadingData.sNormal, nextDir);

		if (shadingData.bsdf->isPureSpecular()) {
			// glass/mirrors
			float absCos = fabsf(cosTheta);
			pathThroughput = pathThroughput * (indirect * absCos) / pdf;
		}
		else {
			// diffuse/conductors
			if (cosTheta <= 0.0f) return Colour(0.0f, 0.0f, 0.0f);
			Colour f = shadingData.bsdf->evaluate(shadingData, nextDir);
			pathThroughput = pathThroughput * (f * cosTheta) / pdf;
		}

		// fires
		Vec3 offsetNormal = Dot(nextDir, shadingData.sNormal) > 0.0f ? shadingData.sNormal : -shadingData.sNormal;
		Ray nextRay(shadingData.x + (offsetNormal * EPSILON), nextDir);
		nextRay.specularBounce = shadingData.bsdf->isPureSpecular();
		nextRay.prevPdf = pdf;

		return pathTrace(nextRay, pathThroughput, depth + 1, sampler);
	}
	Colour computeIndirectVPL(ShadingData shadingData, Colour pathThroughput, int depth, Sampler* sampler)
	{
		if (shadingData.bsdf->isPureSpecular()) {
			// glass/water
			Colour indirect;
			float pdf;
			Vec3 nextDir = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);

			if (pdf <= 0.0f) return Colour(0.0f, 0.0f, 0.0f);

			float cosTheta = Dot(shadingData.sNormal, nextDir);
			float absCos = fabsf(cosTheta);

			pathThroughput = pathThroughput * (indirect * absCos) / pdf;

			Vec3 offsetNormal = Dot(nextDir, shadingData.sNormal) > 0.0f ? shadingData.sNormal : -shadingData.sNormal;
			Ray nextRay(shadingData.x + (offsetNormal * EPSILON), nextDir);
			nextRay.specularBounce = true;
			nextRay.prevPdf = pdf;

			return pathTrace(nextRay, pathThroughput, depth + 1, sampler);
		}
		else {
			// solid
			Colour vplLight(0.0f, 0.0f, 0.0f);
			if (vpls.size() > 0) {
				vplLight = computeDirectVPL(shadingData, sampler);
			}

			return (vplLight * pathThroughput);
		}
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (depth > 64) {
			return Colour(0.0f, 0.0f, 0.0f);
		}

		if (intersection.t >= FLT_MAX) {
			if (scene->background != NULL) {
				Colour sky = scene->background->evaluate(r.dir);
				float misWeight = 1.0f;

				if (depth > 0 && !r.specularBounce) {
					misWeight = ShadingHelper::powerHeuristic(r.prevPdf, scene->background->PDF(shadingData, r.dir));
				}

				return sky * misWeight * pathThroughput;
			}
			return Colour(0.0f, 0.0f, 0.0f);
		}
		if (shadingData.bsdf->isLight()) {
			if (depth == 0 || r.specularBounce == true) {

				return shadingData.bsdf->emit(shadingData, shadingData.wo) * pathThroughput;
			}
			
			return Colour(0.0f, 0.0f, 0.0f);
		}

		Colour directLight = computeDirect(shadingData, sampler);
		directLight = directLight * pathThroughput;

		Colour l = computeIndirect(shadingData, pathThroughput, depth, sampler);
		//Colour l = computeIndirectVPL(shadingData, pathThroughput, depth, sampler);
		//Colour l = computeIndirect(shadingData, pathThroughput, depth, sampler);

		return directLight + l;

	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return scene->background->evaluate(r.dir);
	}
	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	void render()
	{
		// Add multi-threading
		film->incrementSPP();
		for (int y = 0; y < film->height; y++) {
			for (int x = 0; x < film->width; x++) {
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);

				//Colour col = viewNormals(ray);
				//Colour col = albedo(ray);
				//Colour col = direct(ray, &samplers[0]);

				Colour startingThroughput(1.0f, 1.0f, 1.0f);
				Colour col = pathTrace(ray, startingThroughput, 0, &samplers[0]);

				film->splat(px, py, col);
				unsigned char r, g, b;
				film->tonemap(x, y, r, g, b, 1.0f);
				canvas->draw(x, y, r, g, b);
			}
		}
	}
	void renderMT()
	{

		if (film->SPP == 0) {
			genVPLs(1000, 5);
		}


		int threadsToUse = numProcs;
		film->incrementSPP();

		const int size = 32;
		int tileX = (film->width + size - 1) / size;
		int tileY = (film->height + size - 1) / size;
		int total = tileX * tileY;

		int pathsPerTile = 1;
		float totalLightPaths = (float)(tileX * tileY * pathsPerTile);

		std::atomic<int> tileIndex(0);

		for (int i = 0; i < threadsToUse; ++i)
		{
			threads[i] = new std::thread([this, total, tileX, size, &tileIndex, i, totalLightPaths, pathsPerTile]() {
				int tileIdx;

				while ((tileIdx = tileIndex++) < total)
				{
					// tile coordinates
					int gridX = tileIdx % tileX;
					int gridY = tileIdx / tileX;

					int startX = gridX * size;
					int startY = gridY * size;
					int endX = std::min<int>(startX + size, static_cast<int>(film->width));
					int endY = std::min<int>(startY + size, static_cast<int>(film->height));

					// render pixels in tile
					for (int y = startY; y < endY; ++y) {
						for (int x = startX; x < endX; ++x) {
							float px = x + 0.5f;
							float py = y + 0.5f;
							Ray ray = scene->camera.generateRay(px, py);

							Ray gBufferRay = ray;
							IntersectionData firstHit = scene->traverse(gBufferRay);


							if (firstHit.t < FLT_MAX) {
								ShadingData sd = scene->calculateShadingData(firstHit, gBufferRay);
								film->splatAlbedo(px, py, sd.bsdf->getAlbedo(sd));
								film->splatNormal(px, py, sd.sNormal);
							}
							else {
								if (scene->background != NULL) {
									film->splatAlbedo(px, py, scene->background->evaluate(ray.dir));
								}
								else {
									film->splatAlbedo(px, py, Colour(0.0f, 0.0f, 0.0f));
								}
								film->splatNormal(px, py, Vec3(0.0f, 0.0f, 0.0f));
							}

							Colour startingThroughput(1.0f, 1.0f, 1.0f);
							Colour col = pathTrace(ray, startingThroughput, 0, &samplers[i]);

							film->splat(px, py, col);

						}
					}
					/*for (int p = 0; p < pathsPerTile; p++) {
						lightTrace(&samplers[i], totalLightPaths);
					}*/
				}
			});
		}

		for (int i = 0; i < threadsToUse; ++i) {
			threads[i]->join();
			delete threads[i];
		}

		//film->denoise();

		for (int y = 0; y < film->height; ++y) {
			for (int x = 0; x < film->width; ++x) {
				unsigned char r, g, b;
				film->tonemap(x, y, r, g, b, 1.0f);
				
				canvas->draw(x, y, r, g, b);
				
			}
		}

		
	}

	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};