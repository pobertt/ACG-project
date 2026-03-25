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

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	std::thread **threads;
	int numProcs;
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
		float distance = wi.length();
		wi = wi.normalize();

		if (!scene->visible(shadingData.x, sampleVec)) { return Colour(0.0f, 0.0f, 0.0f); };

		float cosThetaOut = Dot(shadingData.sNormal, wi);
		float g = 0.0f;

		if (light->isArea()) {
			AreaLight* areaLight = (AreaLight*)light;
			Vec3 lightNormal = areaLight->triangle->gNormal();
			float cosThetaLight = Dot(lightNormal, -wi);
			
			if (cosThetaOut > 0 && cosThetaLight > 0) {
				g = (cosThetaOut * cosThetaLight) / (distance * distance);
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
		return (emittedColour * f * g) / (pdf * pmf);
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		// Add pathtracer code here
		return Colour(0.0f, 0.0f, 0.0f);
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
		// Add multi-threading here for the kitchen scene
		film->incrementSPP();
		for (int y = 0; y < film->height; y++) {
			for (int x = 0; x < film->width; x++) {
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);
				//Colour col = viewNormals(ray);
				Colour col = direct(ray, &samplers[0]);
				//Colour col = albedo(ray);
				
				film->splat(px, py, col);
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