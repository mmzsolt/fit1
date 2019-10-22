#pragma once

namespace util
{
	template <typename T>
	inline T clamp(T v, T a, T b)
	{
		return v < a ? a : (v > b ? b : v);
	}

	template <typename T>
	inline T lerp(T r, T a, T b)
	{
		return (1 - r) * a + r * b;
	}

	template <typename T>
	inline T ratio(T v, T a, T b)
	{
		return (v - a) / (b - a);
	}

	static std::string to_string(Eigen::Vector3f v)
	{
		return std::string() + std::to_string(v.x()) + "," + std::to_string(v.y()) + "," + std::to_string(v.z());
	}

	static std::string to_string(vec3fi v)
	{
		return std::string() + std::to_string(static_cast<float>(v.x)) + "," + std::to_string(static_cast<float>(v.y)) + "," + std::to_string(static_cast<float>(v.z));
	}
}

