#ifndef FACE_H
#define FACE_H

#include "stdbool.h"
#include "vector.h"

typedef struct LMFacePoints {
	vec3 v0;
	vec3 v1;
	vec3 v2;
} LMFacePoints;

typedef struct LMStandardUV {
	double u = 0.0;
	double v = 0.0;
} LMStandardUV;

typedef struct LMValveTextureAxis {
	vec3 axis;
	double offset = 0.0;
} LMValveTextureAxis;

typedef struct LMValveUV {
	LMValveTextureAxis u;
	LMValveTextureAxis v;
} LMValveUV;

typedef struct LMFaceUVExtra {
	double rot = 0.0;
	double scale_x = 0.0;
	double scale_y = 0.0;
} LMFaceUVExtra;

typedef struct LMFace {
	LMFacePoints plane_points;
	vec3 plane_normal;
	double plane_dist = 0.0;

	int texture_idx = 0;

	bool is_valve_uv = false;
	LMStandardUV uv_standard;
	LMValveUV uv_valve;
	LMFaceUVExtra uv_extra;
} face;

#endif
