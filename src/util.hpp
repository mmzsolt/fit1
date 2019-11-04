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

	inline Eigen::Vector3f direction(const Eigen::Vector3f& from, const Eigen::Vector3f& to)
	{
		Eigen::Vector3f dir = to - from;
		dir.normalize();
		return dir;
	}

	inline std::pair<float, Eigen::Vector3f> directionAndDistance(const Eigen::Vector3f& from, const Eigen::Vector3f& to)
	{
		Eigen::Vector3f dir = to - from;
		float length = dir.lpNorm<2>();
		dir = (1.0f / length) * dir;
		return std::make_pair(length, dir);
	}

	inline Eigen::Vector3f reflect(const Eigen::Vector3f& normal, const Eigen::Vector3f& vec)
	{
		auto ret = 2.0f * normal.dot(vec) * normal - vec;
		return ret;
	}

	inline uint32_t floatToColorInt(float f)
	{
		uint32_t res = static_cast<uint32_t>(std::round(clamp(f, 0.0f, 1.0f) * 255.0f));
		return res;
	}

	inline uint32_t colorToRGB(const Eigen::Vector3f& color)
	{
		uint32_t r = floatToColorInt(color.x());
		uint32_t g = floatToColorInt(color.y());
		uint32_t b = floatToColorInt(color.z());
		uint32_t res = r | (g << 8) | (b << 16);
		return res;
	}

	inline Eigen::Vector3f calcNormal(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c)
	{
		Eigen::Vector3f ab = a - b;
		Eigen::Vector3f ac = a - c;
		ab.normalize();
		ac.normalize();
		auto ret = ab.cross(ac);
		ret.normalize();
		return ret;
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

