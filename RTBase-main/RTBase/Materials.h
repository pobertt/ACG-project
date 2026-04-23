#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#pragma warning( disable : 4244)
#pragma warning( disable : 4305) // Double to float

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

class ShadingHelper
{
public:
	static float fresnelDielectric(float cosTheta, float iorInt, float iorExt)
	{
		// swap iors depending on which side of the surface we are on
		float iorIncident = cosTheta > 0.0f ? iorExt : iorInt;
		float iorTransmitted = cosTheta > 0.0f ? iorInt : iorExt;

		cosTheta = fabsf(cosTheta);

		// snells law
		float iorRatio = iorIncident / iorTransmitted;
		float sin2T = (iorRatio * iorRatio) * std::max(0.0f, 1.0f - (cosTheta * cosTheta));

		// total internal reflection
		if (sin2T >= 1.0f) return 1.0f;

		float cosT = sqrtf(1.0f - sin2T);

		// fresnel equations
		float parlReflectance = ((iorTransmitted * cosTheta) - (iorIncident * cosT)) / ((iorTransmitted * cosTheta) + (iorIncident * cosT));
		float perpReflectance = ((iorIncident * cosTheta) - (iorTransmitted * cosT)) / ((iorIncident * cosTheta) + (iorTransmitted * cosT));

		return ((parlReflectance * parlReflectance) + (perpReflectance * perpReflectance)) * 0.5f;
	}
	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k)
	{
		float cosI = fabsf(cosTheta);
		float cos2 = cosI * cosI;	

		Colour iorSQ = (ior * ior) + (k * k);
		Colour iorCos = iorSQ * cos2;
		Colour twoIorCos = ior * (2.0f * cosI);

		Colour cosSQ(cos2, cos2, cos2);
		Colour vec(1.0f, 1.0f, 1.0f);

		Colour perpReflectance = (iorSQ - twoIorCos + cosSQ) / (iorSQ + twoIorCos + cosSQ);
		Colour parlReflectance = (iorCos - twoIorCos + vec) / (iorCos + twoIorCos + vec);

		return (perpReflectance + parlReflectance) * 0.5f;
	}
	static float lambdaGGX(Vec3 wi, float alpha)
	{
		float cosTheta = fabsf(wi.z);

		if (cosTheta < 0.0001f) return 0.0f;

		float cos2Theta = cosTheta * cosTheta;
		float sin2Theta = std::max(0.0f, 1.0f - cos2Theta);
		float tan2Theta = sin2Theta / cos2Theta;
		float alpha2 = alpha * alpha;

		return (-1.0f + sqrtf(1.0f + (alpha2 * tan2Theta))) * 0.5f;
	}
	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		if (wi.z <= 0.0f || wo.z <= 0.0f) return 0.0f;

		return 1.0f / (1.0f + lambdaGGX(wi, alpha) + lambdaGGX(wo, alpha));
	}
	static float Dggx(Vec3 h, float alpha)
	{
		float cosTheta = h.z;

		if (cosTheta <= 0.0f) return 0.0f;

		float cos2Theta = cosTheta * cosTheta;
		float alpha2 = alpha * alpha;

		float root = (cos2Theta * (alpha2 - 1.0f)) + 1.0f;

		return alpha2 / (M_PI * root * root);
	}
};

class BSDF
{
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
	virtual Colour getAlbedo(const ShadingData& shadingData) {
		return Colour(0.5f, 0.5f, 0.5f); // Default fallback
	}
};


class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add correct sampling code here
		/*Vec3 wi = Vec3(0, 1, 0);
		pdf = 1.0f;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);*/
		//return wi;

		float r1 = sampler->next();
		float r2 = sampler->next();
		Vec3 local_space = SamplingDistributions::cosineSampleHemisphere(r1, r2);
		Vec3 world_space = shadingData.frame.toWorld(local_space);
		pdf = PDF(shadingData, world_space);
		reflectedColour = evaluate(shadingData, world_space);

		return world_space;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add correct PDF code here
		/*float cosTheta = Dot(shadingData.sNormal, wi);
		if (cosTheta <= 0.0) {
			return 0.0f;
		}
		float pdf = cosTheta / M_PI;
		return pdf;*/

		Vec3 wiLocal = shadingData.frame.toLocal(wi);  // Local to world

		if (wiLocal.z <= 0.0f)
		{
			return 0.0f;
		}

		return wiLocal.z / M_PI;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	Colour getAlbedo(const ShadingData& shadingData) override
	{
		if (albedo) return albedo->sample(shadingData.tu, shadingData.tv);
		return Colour(0.0f, 0.0f, 0.0f);
	}
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Mirror sampling code
		Vec3 local_wo = shadingData.frame.toLocal(shadingData.wo);

		// Perfect reflection in local space
		Vec3 local_wi = Vec3(-local_wo.x, -local_wo.y, local_wo.z);

		pdf = 1.0f;
		float cosTheta = local_wi.z;

		if (cosTheta <= 0.0001f) {
			reflectedColour = Colour(0, 0, 0);
		}
		else {
			Colour mirror_albedo(1.0f, 1.0f, 1.0f);
			reflectedColour = mirror_albedo / cosTheta;
		}

		return shadingData.frame.toWorld(local_wi);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror evaluation code
		/*return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;*/
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror PDF
		/*Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);*/
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	Colour getAlbedo(const ShadingData& shadingData) override
	{
		if (albedo) return albedo->sample(shadingData.tu, shadingData.tv);
		return Colour(0.0f, 0.0f, 0.0f);
	}
};


class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Conductor sampling code
		/*Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;*/
		float safeAlpha = std::max(0.001f, alpha);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		float u1 = sampler->next();
		float u2 = sampler->next();

		float alpha2 = safeAlpha * safeAlpha;
		float phi = 2.0f * M_PI * u1;

		float denom = 1.0f + (alpha2 - 1.0f) * u2;
		if (denom <= 0.0001f) {
			pdf = 1.0f;
			reflectedColour = Colour(0, 0, 0);
			return Vec3(0, 0, 1);
		}

		float cosTheta = sqrtf(std::max(0.0f, (1.0f - u2) / denom));
		float sinTheta = sqrtf(std::max(0.0f, 1.0f - (cosTheta * cosTheta)));

		Vec3 hLocal = Vec3(sinTheta * cosf(phi), sinTheta * sinf(phi), cosTheta).normalize();

		float woDotH = Dot(woLocal, hLocal);
		if (woDotH <= 0.0001f) {
			pdf = 1.0f;
			reflectedColour = Colour(0, 0, 0);
			return Vec3(0, 0, 1);
		}

		Vec3 wiLocal = (hLocal * (2.0f * woDotH)) - woLocal;
		if (wiLocal.z <= 0.0001f) {
			pdf = 1.0f;
			reflectedColour = Colour(0, 0, 0);
			return Vec3(0, 0, 1);
		}

		Vec3 wiWorld = shadingData.frame.toWorld(wiLocal).normalize();

		pdf = PDF(shadingData, wiWorld);

		if (pdf <= 0.0001f) {
			pdf = 1.0f;
			reflectedColour = Colour(0, 0, 0);
			return wiWorld;
		}

		reflectedColour = evaluate(shadingData, wiWorld);

		return wiWorld;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor evaluation code
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		
		float safeAlpha = std::max(0.001f, alpha);

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		if (wiLocal.z <= 0.0001f || woLocal.z <= 0.0001f) return Colour(0.0f, 0.0f, 0.0f);

		Vec3 h = woLocal + wiLocal;
		if (h.length() < 0.0001f) return Colour(0.0f, 0.0f, 0.0f);
		h = h.normalize();

		float D = ShadingHelper::Dggx(h, safeAlpha);
		float G = ShadingHelper::Gggx(wiLocal, woLocal, safeAlpha);

		float woDotH = std::max(0.0f, Dot(woLocal, h));
		Colour F = ShadingHelper::fresnelConductor(woDotH, eta, k);

		Colour texColor = albedo->sample(shadingData.tu, shadingData.tv);

		return texColor * F * ((D * G) / (4.0f * wiLocal.z * woLocal.z));
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor PDF
		/*Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);*/
		float safeAlpha = std::max(0.001f, alpha);

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		if (wiLocal.z <= 0.0001f || woLocal.z <= 0.0001f) return 0.0f;

		Vec3 h = woLocal + wiLocal;
		if (h.length() < 0.0001f) return 0.0f;
		h = h.normalize();

		float woDotH = Dot(woLocal, h);
		if (woDotH <= 0.0001f) return 0.0f;

		float D = ShadingHelper::Dggx(h, safeAlpha);

		return (D * h.z) / (4.0f * woDotH);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	Colour getAlbedo(const ShadingData& shadingData) override
	{
		if (albedo) return albedo->sample(shadingData.tu, shadingData.tv);
		return Colour(0.0f, 0.0f, 0.0f);
	}
};

class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Glass sampling code
		/*Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;*/

		pdf = 1.0f;

		Colour texColor = albedo->sample(shadingData.tu, shadingData.tv);

		Vec3 normal = shadingData.sNormal;
		float n1 = extIOR;
		float n2 = intIOR;
		float cosI = Dot(shadingData.wo, normal);

		// hitting inside of glass or not
		if (cosI < 0.0f) {
			normal = -shadingData.sNormal; 
			n1 = intIOR;                 
			n2 = extIOR;
			cosI = -cosI;                  
		}

		// snells law
		float eta = n1 / n2;
		float sin2I = std::max(0.0f, 1.0f - (cosI * cosI));
		float sin2T = (eta * eta) * sin2I;

		Vec3 wi;
		float reflectionProb = ShadingHelper::fresnelDielectric(Dot(shadingData.wo, shadingData.sNormal), intIOR, extIOR);

		if (sampler->next() < reflectionProb) {
			wi = (normal * 2.0f * cosI) - shadingData.wo;
			reflectedColour = texColor / cosI;
		}
		else {
			// refraction 
			float cosT = sqrtf(1.0f - sin2T);
			wi = (shadingData.wo * -eta) + (normal * ((eta * cosI) - cosT));
			reflectedColour = texColor / cosT;
		}

		return wi.normalize();
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Glass evaluation code
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with GlassPDF
		/*Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);*/
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	Colour getAlbedo(const ShadingData& shadingData) override
	{
		if (albedo) return albedo->sample(shadingData.tu, shadingData.tv);
		return Colour(0.0f, 0.0f, 0.0f);
	}
};

class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Dielectric sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	Colour getAlbedo(const ShadingData& shadingData) override
	{
		if (albedo) return albedo->sample(shadingData.tu, shadingData.tv);
		return Colour(0.0f, 0.0f, 0.0f);
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with OrenNayar sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	Colour getAlbedo(const ShadingData& shadingData) override
	{
		if (albedo) return albedo->sample(shadingData.tu, shadingData.tv);
		return Colour(0.0f, 0.0f, 0.0f);
	}
};

class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Plastic sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	Colour getAlbedo(const ShadingData& shadingData) override
	{
		if (albedo) return albedo->sample(shadingData.tu, shadingData.tv);
		return Colour(0.0f, 0.0f, 0.0f);
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code for evaluation of layer
		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
	Colour getAlbedo(const ShadingData& shadingData) override
	{
		return base->getAlbedo(shadingData);
	}
};