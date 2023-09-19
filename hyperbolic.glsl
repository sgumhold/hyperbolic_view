#version 330 core

uniform float hyperbolic_lambda = 0.0;
uniform vec3  hyperbolic_translation = vec3(0.0);
uniform bool  simple_translation;
uniform bool  skip_first;
uniform int   dimension;
/*
The following interface is implemented in this shader:
//***** begin interface of hyperbolic.glsl ***********************************
vec4 hyperbolic_project(in vec4 p);
vec4 hyperbolic_transform(in vec4 x);
vec4 hyperbolic_mixture(in vec4 x, float t, in vec4 y);
//***** end interface of hyperbolic.glsl ***********************************
*/

vec4 hyperbolic_project(in vec4 p)
{
	float x2 = p[1], x3 = p[2], x4 = p[3];
	float x1 = sqrt(x2 * x2 + x3 * x3+ x4 * x4 + 1.0);
	float z1 = x2 / (x1 + 1.0f);
	float z2 = x3 / (x1 + 1.0f);
	float z3 = x4 / (x1 + 1.0f);
	vec4 q = vec4(0.0f, z1, z2, z3);
	vec4 m = mix(p,q,hyperbolic_lambda);
	float temp = 1.0-(z1*z1+z2*z2+z3*z3);
	float sz = pow(temp*temp,1.0/float(dimension));
	float sx = pow(x1,1.0/float(dimension));
	float size = mix(sx,sz,hyperbolic_lambda);
	if (skip_first)
		return vec4(m.yzw,size);
	else
		return vec4(m.xyz,size);
}

vec4 hyperbolic_transform(in vec4 x)
{
	vec3 xb = hyperbolic_translation;
	float c = dot(xb, xb);
	mat3 M = mat3(1.0);
	if (abs(c) > 1e-6) {
		mat3 M1;
		M1[0] = vec3(xb.x*xb.x, xb.y*xb.x, xb.z*xb.x);
		M1[1] = vec3(xb.x*xb.y, xb.y*xb.y, xb.z*xb.y);
		M1[2] = vec3(xb.x*xb.z, xb.y*xb.z, xb.z*xb.z);
		M += (sqrt(c + 1.0) - 1.0) / c * M1;
	}
	mat4 T;
	float x0 = sqrt(dot(hyperbolic_translation, hyperbolic_translation)+1.0);
	T[0] = vec4(x0,-hyperbolic_translation[0],-hyperbolic_translation[1],-hyperbolic_translation[2]);
	T[1] = vec4(-hyperbolic_translation[0], M[0][0], M[1][0], M[2][0]);
	T[2] = vec4(-hyperbolic_translation[1], M[0][1], M[1][1], M[2][1]);
	T[3] = vec4(-hyperbolic_translation[2], M[0][2], M[1][2], M[2][2]);
	vec4 y;
	if (simple_translation)
		y = x - vec4(0.0,hyperbolic_translation.x,hyperbolic_translation.y,hyperbolic_translation.z);
	else
		y = T * x;
	y[0] = sqrt(y[1] * y[1] + y[2] * y[2] + y[3] * y[3] + 1.0);
	return y;
}

vec4 hyperbolic_mixture(in vec4 x, float t, in vec4 y)
{
	float p = x[0] * y[0] - (x[1] * y[1] + x[2] * y[2] + x[3] * y[3]);
	vec4 v = (y - p * x) / sqrt(p * p - 1);
	t *= acosh(p);
	return cosh(t) * x + sinh(t) * v;
}

