#pragma once

#include "Eigen/Dense"

class Camera
{
	Camera() {}
	~Camera() {}
	Eigen::Vector3f m_pos{ 0.0f, 0.0f, 0.0f };
};