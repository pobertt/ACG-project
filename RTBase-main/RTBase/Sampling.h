#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		return dist(generator);
	}
	// This is code from slide 127 monte carlo slides
	/*MTRandom* samplers;
	SYSTEM_INFO sysInfo;
	GetSystemInfo(&sysInfo);
	numProcs = sysInfo.dwNumberOfProcessors;
	samplers = new MTRandom[numProcs];
	samplers[threadID].next();*/
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:

	// maybe use sqrtf(std::max(0.0f, 1.0f - (z * z))) for sqrtf(1 - (z * z)

	static Vec3 uniformSampleHemisphere(float r1, float r2)
	{
		// Add code here
		float z = r1;
		float phi = 2.0f * (M_PI * r2);
		float x = cosf(phi) * sqrtf(1.0f - (z * z));
		float y = sinf(phi) * sqrtf(1.0f - (z * z));
		return Vec3(x, y, z);
	}
	static float uniformHemispherePDF(const Vec3 wi)
	{
		// Add code here
		return 1.0f / (2.0f * M_PI);
	}
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		// Add code here
		float z = sqrtf(r1);
		float phi = 2.0f * (M_PI * r2);
		float r = sqrtf(1.0f - (z * z));
		float x = r * cosf(phi);
		float y = r * sinf(phi);
		return Vec3(x, y, z);
	}
	static float cosineHemispherePDF(const Vec3 wi)
	{
		// Add code here
		return wi.z / M_PI;
	}
	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		// Add code here
		float z = 1.0f - (2.0f * r1);
		float phi = (2.0f * M_PI) * r2;
		float r = sqrtf(1.0f - (z * z));
		float x = r * cosf(phi);
		float y = r * sinf(phi);

		return Vec3(x, y, z);
	}
	static float uniformSpherePDF(const Vec3& wi)
	{
		// Add code here
		return 1.0f / (4.0f * M_PI);
	}
};