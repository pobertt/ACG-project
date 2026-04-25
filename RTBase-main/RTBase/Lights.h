#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

class SceneBounds
{
public:
	Vec3 sceneCentre;
	float sceneRadius;
};

class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}
	Colour evaluate(const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) < 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 1.0f / triangle->area;
	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Add code to sample a direction from the light
		/*Vec3 wi = Vec3(0, 0, 1);
		pdf = 1.0f;
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wi);*/

		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wi);

		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wi);
	}
};

class BackgroundColour : public Light
{
public:
	Colour emission;
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 4 * M_PI * use<SceneBounds>().sceneRadius * use<SceneBounds>().sceneRadius;
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};

class EnvironmentMap : public Light
{
public:
	Texture* env;

	std::vector<float> marginalCDF;
	std::vector<std::vector<float>> conditionalCDF;
	float mapIntegral;

	EnvironmentMap(Texture* _env)
	{
		env = _env;

		int w = env->width;
		int h = env->height;

		marginalCDF.resize(h, 0.0f);
		conditionalCDF.resize(h, std::vector<float>(w, 0.0f));

		// build
		for (int y = 0; y < h; ++y) {
			float v = (float(y) + 0.5f) / float(h);
			float sinTheta = sinf(M_PI * v); 

			float rowSum = 0.0f;
			for (int x = 0; x < w; ++x) {
				float lum = env->texels[y * w + x].Lum();
				rowSum += lum * sinTheta;
				conditionalCDF[y][x] = rowSum;
			}

			// normalise row
			if (rowSum > 0.0f) {
				for (int x = 0; x < w; ++x) {
					conditionalCDF[y][x] /= rowSum;
				}
			}
			else {
				// fallback if black
				for (int x = 0; x < w; ++x) {
					conditionalCDF[y][x] = float(x + 1) / float(w);
				}
			}

			marginalCDF[y] = (y == 0) ? rowSum : marginalCDF[y - 1] + rowSum;
		}

		mapIntegral = marginalCDF[h - 1];

		// normalise column
		if (mapIntegral > 0.0f) {
			for (int y = 0; y < h; ++y) {
				marginalCDF[y] /= mapIntegral;
			}
		}
		else {
			for (int y = 0; y < h; ++y) {
				marginalCDF[y] = float(y + 1) / float(h);
			}
		}
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		float r1 = sampler->next();
		float r2 = sampler->next();

		// pick random y
		auto itY = std::lower_bound(marginalCDF.begin(), marginalCDF.end(), r1);
		int y = std::min(int(itY - marginalCDF.begin()), env->height - 1);

		// pick random x
		auto itX = std::lower_bound(conditionalCDF[y].begin(), conditionalCDF[y].end(), r2);
		int x = std::min(int(itX - conditionalCDF[y].begin()), env->width - 1);

		// don't hit same pixel every time
		float u = (float(x) + sampler->next()) / float(env->width);
		float v = (float(y) + sampler->next()) / float(env->height);

		// convert 2d to 3d
		float phi = u * 2.0f * M_PI;
		float theta = v * M_PI;

		Vec3 wi;
		wi.y = cosf(theta);
		wi.x = sinf(theta) * cosf(phi);
		wi.z = sinf(theta) * sinf(phi);

		pdf = PDF(shadingData, wi);
		reflectedColour = evaluate(wi);

		return wi;
	}

	Colour evaluate(const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);

		// clamp to prevent nan errors
		float v = acosf(std::max(-1.0f, std::min(1.0f, wi.y))) / M_PI;

		return env->sample(u, v);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Assignment: Update this code to return the correct PDF of luminance weighted importance sampling

		if (mapIntegral <= 0.0f) return 1.0f / (4.0f * M_PI); 

		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(std::max(-1.0f, std::min(1.0f, wi.y))) / M_PI;

		float lum = env->sample(u, v).Lum();

		// return probability
		float pdf = (lum / mapIntegral) * (env->width * env->height) / (2.0f * M_PI * M_PI);

		return pdf;
	}

	bool isArea()
	{
		return false;
	}

	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}

	float totalIntegratedPower()
	{
		/*float total = 0;
		for (int i = 0; i < env->height; i++)
		{
			float st = sinf(((float)i / (float)env->height) * M_PI);
			for (int n = 0; n < env->width; n++)
			{
				total += (env->texels[(i * env->width) + n].Lum() * st);
			}
		}
		total = total / (float)(env->width * env->height);
		return total * 4.0f * M_PI;*/

		return mapIntegral * (4.0f * M_PI);
	}

	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		// Samples a point on the bounding sphere of the scene. Feel free to improve this.
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
		return p;
	}

	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Replace this tabulated sampling of environment maps
		/*Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;*/
		Colour reflectedColor;
		return sample(ShadingData(), sampler, reflectedColor, pdf);
	}
};