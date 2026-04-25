#pragma once

#include "Core.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define __STDC_LIB_EXT1__
#include "stb_image_write.h"
#include <OpenImageDenoise/oidn.hpp>
#include <vector>
#include <iostream>

// Stop warnings about buffer overruns if size is zero. Size should never be zero and if it is the code handles it.
#pragma warning( disable : 6386)

constexpr float texelScale = 1.0f / 255.0f;

class Texture
{
public:
	Colour* texels;
	float* alpha;
	int width;
	int height;
	int channels;
	void loadDefault()
	{
		width = 1;
		height = 1;
		channels = 3;
		texels = new Colour[1];
		texels[0] = Colour(1.0f, 1.0f, 1.0f);
	}
	void load(std::string filename)
	{
		alpha = NULL;
		if (filename.find(".hdr") != std::string::npos)
		{
			float* textureData = stbi_loadf(filename.c_str(), &width, &height, &channels, 0);
			if (width == 0 || height == 0)
			{
				loadDefault();
				return;
			}
			texels = new Colour[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				texels[i] = Colour(textureData[i * channels], textureData[(i * channels) + 1], textureData[(i * channels) + 2]);
			}
			stbi_image_free(textureData);
			return;
		}
		unsigned char* textureData = stbi_load(filename.c_str(), &width, &height, &channels, 0);
		if (width == 0 || height == 0)
		{
			loadDefault();
			return;
		}
		texels = new Colour[width * height];
		for (int i = 0; i < (width * height); i++)
		{
			texels[i] = Colour(textureData[i * channels] / 255.0f, textureData[(i * channels) + 1] / 255.0f, textureData[(i * channels) + 2] / 255.0f);
		}
		if (channels == 4)
		{
			alpha = new float[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				alpha[i] = textureData[(i * channels) + 3] / 255.0f;
			}
		}
		stbi_image_free(textureData);
	}
	Colour sample(const float tu, const float tv) const
	{
		Colour tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		Colour s[4];
		s[0] = texels[y * width + x];
		s[1] = texels[y * width + ((x + 1) % width)];
		s[2] = texels[((y + 1) % height) * width + x];
		s[3] = texels[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	float sampleAlpha(const float tu, const float tv) const
	{
		if (alpha == NULL)
		{
			return 1.0f;
		}
		float tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		float s[4];
		s[0] = alpha[y * width + x];
		s[1] = alpha[y * width + ((x + 1) % width)];
		s[2] = alpha[((y + 1) % height) * width + x];
		s[3] = alpha[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	~Texture()
	{
		delete[] texels;
		if (alpha != NULL)
		{
			delete alpha;
		}
	}
};

class ImageFilter
{
public:
	virtual float filter(const float x, const float y) const = 0;
	virtual int size() const = 0;
};

class BoxFilter : public ImageFilter
{
public:
	float filter(float x, float y) const
	{
		if (fabsf(x) < 0.51f && fabs(y) < 0.51f)
		{
			return 1.0f;
		}
		return 0;
	}
	int size() const
	{
		return 1;
	}
};

class Film
{
public:
	Colour* film;

	Colour* albedoBuffer;
	Vec3* normalBuffer;
	Colour* outputBuffer;

	unsigned int width;
	unsigned int height;
	int SPP;
	ImageFilter* filter;
	
	void splat(const float x, const float y, const Colour& L)
	{
		if (std::isnan(L.r) || std::isnan(L.g) || std::isnan(L.b) ||
			std::isinf(L.r) || std::isinf(L.g) || std::isinf(L.b) ||
			L.r < 0.0f || L.g < 0.0f || L.b < 0.0f) {
			return;
		}
		// Code to splat a smaple with colour L into the image plane using an ImageFilter
		float filterWeights[25]; // Storage to cache weights
		unsigned int indices[25]; // Store indices to minimize computations
		unsigned int used = 0;
		float total = 0;
		int size = filter->size();
		for (int i = -size; i <= size; i++) {
			for (int j = -size; j <= size; j++) {
				int px = (int)x + j;
				int py = (int)y + i;
				if (px >= 0 && px < width && py >= 0 && py < height) {
					indices[used] = (py * width) + px;
					filterWeights[used] = filter->filter(px - x, py - y);
					total += filterWeights[used];
					used++;
				}
			}
		}
		if (total > 0.0f) {
			for (int i = 0; i < used; i++) {
				film[indices[i]] = film[indices[i]] + (L * filterWeights[i] / total);
			}
		}
	}

	void tonemap(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, float exposure = 1.0f)
	{
		Colour c = film[y * width + x] / (float)SPP;
		//Colour c = outputBuffer[y * width + x];

		// exposure
		c.r = c.r * exposure;
		c.g = c.g * exposure;
		c.b = c.b * exposure;

		// Reinhard Curve 
		c.r = c.r / (1.0f + c.r);
		c.g = c.g / (1.0f + c.g);
		c.b = c.b / (1.0f + c.b);

		r = (unsigned char)(std::min(powf(std::max(c.r, 0.0f), 1.0f / 2.2f) * 255.0f, 255.0f));
		g = (unsigned char)(std::min(powf(std::max(c.g, 0.0f), 1.0f / 2.2f) * 255.0f, 255.0f));
		b = (unsigned char)(std::min(powf(std::max(c.b, 0.0f), 1.0f / 2.2f) * 255.0f, 255.0f));
	}
	// Do not change any code below this line
	void init(int _width, int _height, ImageFilter* _filter)
	{
		width = _width;
		height = _height;
		film = new Colour[width * height];

		albedoBuffer = new Colour[width * height];
		normalBuffer = new Vec3[width * height];
		outputBuffer = new Colour[width * height];

		clear();
		filter = _filter;
	}
	void clear()
	{
		memset(film, 0, width * height * sizeof(Colour));

		memset(albedoBuffer, 0, width * height * sizeof(Colour));
		memset(normalBuffer, 0, width * height * sizeof(Vec3));
		memset(outputBuffer, 0, width * height * sizeof(Colour));

		SPP = 0;
	}
	void incrementSPP()
	{
		SPP++;
	}
	void save(std::string filename)
	{
		Colour* hdrpixels = new Colour[width * height];
		for (unsigned int i = 0; i < (width * height); i++)
		{
			hdrpixels[i] = outputBuffer[i];
		}
		stbi_write_hdr(filename.c_str(), width, height, 3, (float*)hdrpixels);
		delete[] hdrpixels;
	}
	void splatAlbedo(int x, int y, const Colour& a) {
		if (x >= 0 && x < width && y >= 0 && y < height)
			albedoBuffer[y * width + x] = a;
	}

	void splatNormal(int x, int y, const Vec3& n) {
		if (x >= 0 && x < width && y >= 0 && y < height)
			normalBuffer[y * width + x] = n;
	}
	void denoise() {
		int numPixels = width * height;

		std::vector<float> colorData(numPixels * 3);
		std::vector<float> albedoData(numPixels * 3);
		std::vector<float> normalData(numPixels * 3);
		std::vector<float> outputData(numPixels * 3);

		for (int i = 0; i < numPixels; ++i) {
			Colour c = film[i] / (float)SPP;

			// 1. Sanitize Color Data
			if (std::isnan(c.r) || std::isnan(c.g) || std::isnan(c.b) ||
				std::isinf(c.r) || std::isinf(c.g) || std::isinf(c.b)) {
				c = Colour(0.0f, 0.0f, 0.0f);
			}
			colorData[i * 3 + 0] = c.r;
			colorData[i * 3 + 1] = c.g;
			colorData[i * 3 + 2] = c.b;

			// 2. Sanitize Albedo Data
			Colour a = albedoBuffer[i];
			if (std::isnan(a.r) || std::isnan(a.g) || std::isnan(a.b) ||
				std::isinf(a.r) || std::isinf(a.g) || std::isinf(a.b)) {
				a = Colour(0.5f, 0.5f, 0.5f);
			}
			albedoData[i * 3 + 0] = a.r;
			albedoData[i * 3 + 1] = a.g;
			albedoData[i * 3 + 2] = a.b;

			// 3. Sanitize Normal Data
			Vec3 n = normalBuffer[i];
			if (std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z) ||
				std::isinf(n.x) || std::isinf(n.y) || std::isinf(n.z)) {
				n = Vec3(0.0f, 1.0f, 0.0f);
			}
			normalData[i * 3 + 0] = n.x;
			normalData[i * 3 + 1] = n.y;
			normalData[i * 3 + 2] = n.z;
		}

		std::cout << "Intel OIDN: Initializing..." << std::endl;
		oidn::DeviceRef device = oidn::newDevice(oidn::DeviceType::CPU);
		device.commit();

		oidn::FilterRef filter = device.newFilter("RT");
		filter.setImage("color", colorData.data(), oidn::Format::Float3, width, height);
		filter.setImage("albedo", albedoData.data(), oidn::Format::Float3, width, height);
		filter.setImage("normal", normalData.data(), oidn::Format::Float3, width, height);
		filter.setImage("output", outputData.data(), oidn::Format::Float3, width, height);
		filter.set("hdr", true);
		filter.commit();

		std::cout << "Intel OIDN: Executing filter..." << std::endl;
		filter.execute();

		// NEW: Catch and print any internal OIDN crashes!
		const char* errorMessage;
		if (device.getError(errorMessage) != oidn::Error::None) {
			std::cout << "\n>>> OIDN FATAL ERROR: " << errorMessage << " <<<\n" << std::endl;
		}
		else {
			std::cout << "Intel OIDN: Success!" << std::endl;
		}

		for (int i = 0; i < numPixels; i++) {
			outputBuffer[i].r = outputData[i * 3 + 0];
			outputBuffer[i].g = outputData[i * 3 + 1];
			outputBuffer[i].b = outputData[i * 3 + 2];
		}
	}
};