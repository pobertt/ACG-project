#pragma once

#include "Core.h"
#include "Sampling.h"

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		// Change this so it uses the built in dot product
		float denom = (n.x * r.dir.x) + (n.y * r.dir.y) + (n.z * r.dir.z);
		float dotno = (n.x * r.o.x) + (n.y * r.o.y) + (n.z * r.o.z);
		float numerator = -(dotno + d);
		t = numerator / denom;

		if (t < 0.0f) {
			return false;
		}
		return true;
	}
};

#define EPSILON 0.001f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	// Add code here
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		// Bayocentric Coords
		float denom = Dot(n, r.dir);
		if (denom == 0) { return false; }
		t = (d - Dot(n, r.o)) / denom;
		if (t < 0) { return false; }
		Vec3 p = r.at(t);
		float invArea = 1.0f / Dot(e1.cross(e2), n);
		u = Dot(e1.cross(p - vertices[1].p), n) * invArea;
		if (u < 0 || u > 1.0f) { return false; }
		v = Dot(e2.cross(p - vertices[2].p), n) * invArea;
		if (v < 0 || (u + v) > 1.0f) { return false; }
		return true;
		
		/*
		// Moller Trumbore
		Vec3 edge1 = vertices[1].p - vertices[0].p;
		Vec3 edge2 = vertices[2].p - vertices[0].p;
		Vec3 p = r.dir.cross(edge2);
		if (edge1.dot(p) < 0.000001f) { return false; };
		float invdet = 1.0f / edge1.dot(p);

		Vec3 T = r.o - vertices[0].p;
		u = T.dot(p) * invdet;
		if (u < 0.0f || u > 1.0f) { return false; };

		Vec3 q = T.cross(edge1);
		v = r.dir.dot(q) * invdet;
		if (v < 0.0f || u + v > 1.0f) { return false; }

		t = edge2.dot(q) * invdet;

		return true;
		*/
	}
	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		// Implement uniformly sampling triangle area (monte carlo slides)
		float r1 = sampler->next();
		float r2 = sampler->next();

		float sqrtR1 = sqrtf(r1);
		float u = 1.0f - sqrtR1;
		float v = r2 * sqrtR1;
		float w = 1.0f - (u + v);

		Vec3 point = (vertices[0].p * w) + (vertices[1].p * u) + (vertices[2].p * v);

		pdf = 1.0f / area;

		return point;
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		Vec3 Tmin = (min - r.o) * r.invDir;
		Vec3 Tmax = (max - r.o) * r.invDir;
		Vec3 Tentry = Min(Tmin, Tmax);
		Vec3 Texit = Max(Tmin, Tmax);
		float tentry = std::max(Tentry.x, std::max(Tentry.y, Tentry.z));
		float texit = std::min(Texit.x, std::min(Texit.y, Texit.z));
		t = std::min(tentry, texit);
		return (tentry <= texit && texit > 0);

	}
	// Add code here
	bool rayAABB(const Ray& r)
	{
		Vec3 s = (min - r.o) * r.invDir;
		Vec3 l = (max - r.o) * r.invDir;
		Vec3 s1 = Min(s, l);
		Vec3 l1 = Max(s, l);
		float ts = std::max(s1.x, std::max(s1.y, s1.z));
		float tl = std::min(l1.x, std::min(l1.y, l1.z));
		return (ts <= tl && tl > 0);
	}
	// Add code here
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		return false;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	unsigned int offset;
	unsigned char num;
	BVHNode()
	{
		r = NULL;
		l = NULL;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles, std::vector<Triangle>& outputTriangles)
	{
		// marks for getting the BVH working and then more marks for it to be better (impress tom, get more marks)
		// Add BVH building code here
		this->bounds.reset();
		int axis;
		for (int i = 0; i < inputTriangles.size(); i++) {
			this->bounds.extend(inputTriangles[i].vertices[0].p);
			this->bounds.extend(inputTriangles[i].vertices[1].p);
			this->bounds.extend(inputTriangles[i].vertices[2].p);
		}

		if (inputTriangles.size() <= MAXNODE_TRIANGLES) {
			// if true = leaf
			this->offset = outputTriangles.size();				
			this->num = inputTriangles.size();
			for (int j = 0; j < inputTriangles.size(); j++) {
				outputTriangles.push_back(inputTriangles[j]);
			}
			return;
		}
		else if (inputTriangles.size() >= MAXNODE_TRIANGLES) {
			Vec3 size = this->bounds.max - this->bounds.min;

			if (size.x > size.y && size.x > size.z) {
				axis = 0;
			}
			else if (size.y > size.x && size.y > size.z) {
				axis = 1;
			}
			else {
				axis = 2;
			}
		}

		std::sort(inputTriangles.begin(), inputTriangles.end(), [axis](const Triangle& a, const Triangle& b) {
			Vec3 centerA = a.centre();
			Vec3 centerB = b.centre();
			if (axis == 0) return centerA.x < centerB.x;
			if (axis == 1) return centerA.y < centerB.y;
			return centerA.z < centerB.z;
		});

		size_t mid = inputTriangles.size() / 2;
		std::vector<Triangle> leftTriangles(inputTriangles.begin(), inputTriangles.begin() + mid);
		std::vector<Triangle> rightTriangles(inputTriangles.begin() + mid, inputTriangles.end());

		this->l = new BVHNode();
		this->r = new BVHNode();
		this->l->build(leftTriangles, outputTriangles);
		this->r->build(rightTriangles, outputTriangles);
	}
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// marks for getting the BVH working and then more marks for it to be better (impress tom, get more marks)
		// Add BVH Traversal code here
		float t;
		if (!this->bounds.rayAABB(ray, t) || t >= intersection.t) {
			return;
		}
		else {
			if (this->l == NULL && this->r == NULL) {
				for (int i = this->offset; i < this->offset + this->num; i++) {
					float triT, u, v;
					if (triangles[i].rayIntersect(ray, triT, u, v) && triT < intersection.t) {
						intersection.t = triT;
						intersection.ID = i;
						intersection.alpha = u; 
						intersection.beta = v; 
						intersection.gamma = 1.0f - (u + v);
					}
				}
			}
			else {
				this->l->traverse(ray, triangles, intersection);
				this->r->traverse(ray, triangles, intersection);
			}
		}
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}

	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		// Add visibility code here
		return true;
	}
};
