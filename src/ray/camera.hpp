#pragma once

#include <math.h>
#include "basic.hpp"
#include "basic_fixed.hpp"
#include "util.hpp"

class Camera
{
public:
	Eigen::Vector3f m_pos{ 0.0f, 0.0f, 0.0f };
	float getWidth() const { return m_size[0]; }
	float getHeight() const { return m_size[1]; }
	void setWidth(float f)
	{
		m_size[0] = f;
		recalc();
	}
	void setHeight(float f)
	{
		m_size[1] = f;
		recalc();
	}
	void setFov(float f)  // in radians
	{
		m_focus = 1.0f / tan(f / 2.0f);
	}
	Ray getRay(int32_t x, int32_t y) const
	{
		// todo: remove
		auto xx = util::clamp(static_cast<float>(x), 0.0f, getWidth());
		auto yy = util::clamp(static_cast<float>(y), 0.0f, getHeight());

		float xr = util::ratio(static_cast<float>(xx), 0.0f, getWidth()) - 0.5f;
		float yr = util::ratio(static_cast<float>(yy), 0.0f, getHeight()) - 0.5f;

		Eigen::Vector3f dir(xr, yr, m_focus);
		dir.normalize();
		
		return Ray{ m_pos, dir };
	}
private:
	void recalc()
	{

	}
	Eigen::Vector2f m_size{ 100.0f, 100.0f };
	float m_focus = 1.0f;
};

class Camera_fi
{
public:
	vec3fi m_pos{ 0.0f, 0.0f, 0.0f };
	fixScalar getWidth() const { return m_size_x; }
	fixScalar getHeight() const { return m_size_y; }
	void setWidth(fixScalar f)
	{
		m_size_x = f;
		recalc();
	}
	void setHeight(fixScalar f)
	{
		m_size_y = f;
		recalc();
	}
	void setFov(float f)  // in radians
	{
		float focus = 1.0f / tan(f / 2.0f);
		m_focus = fixScalar(focus);
	}
	Ray_fi getRay(int32_t x, int32_t y) const
	{
		// todo: remove
		auto xx = util::clamp(fixScalar(x), fixScalar(0), getWidth());
		auto yy = util::clamp(fixScalar(y), fixScalar(0), getHeight());

		fixScalar xr = util::ratio(fixScalar(xx), fixScalar(0), getWidth()) - fixScalar(0.5);
		fixScalar yr = util::ratio(fixScalar(yy), fixScalar(0), getHeight()) - fixScalar(0.5);

		vec3fi dir(xr, yr, m_focus);
		dir = normalize(dir);
		
		return Ray_fi{ m_pos, dir };
	}
private:
	void recalc()
	{

	}
	fixScalar m_size_x = 100;
	fixScalar m_size_y = 100;
	fixScalar m_focus = 1;
};