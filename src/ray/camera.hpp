#pragma once

#include <math.h>
#include "basic.hpp"
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
	Ray getRay(float x, float y)
	{
		// todo: remove
		x = util::clamp(x, 0.0f, getWidth());
		y = util::clamp(y, 0.0f, getHeight());

		float xr = util::ratio(x, 0.0f, getWidth()) - 0.5f;
		float yr = util::ratio(y, 0.0f, getHeight()) - 0.5f;

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