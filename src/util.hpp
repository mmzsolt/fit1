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

	inline Eigen::Vector3f min(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
	{
		return Eigen::Vector3f
		(
			a.x() < b.x() ? a.x() : b.x(),
			a.y() < b.y() ? a.y() : b.y(),
			a.z() < b.z() ? a.z() : b.z()
		);
	}

	inline Eigen::Vector3f max(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
	{
		return Eigen::Vector3f
		(
			a.x() > b.x() ? a.x() : b.x(),
			a.y() > b.y() ? a.y() : b.y(),
			a.z() > b.z() ? a.z() : b.z()
		);
	}

	inline float max_component(const Eigen::Vector3f& a)
	{
		float tempMax = std::max(a.x(), a.y());
		return std::max(tempMax, a.z());
	}

	inline float min_component(const Eigen::Vector3f& a)
	{
		float tempMin = std::min(a.x(), a.y());
		return std::min(tempMin, a.z());
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

