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

	inline Eigen::Vector3f abs(const Eigen::Vector3f& a)
	{
		return Eigen::Vector3f
		(
			a.x() > 0.0f ? a.x() : -a.x(),
			a.y() > 0.0f ? a.y() : -a.y(),
			a.z() > 0.0f ? a.z() : -a.z()
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

	inline std::pair<float, int> max_component_index(const Eigen::Vector3f& a)
	{
		float max_v = a.x();
		int max_i = 0;
		if (a.y() > max_v)
		{
			max_v = a.y();
			max_i = 1;
		}
		if (a.z() > max_v)
		{
			max_v = a.z();
			max_i = 2;
		}
		return {max_v, max_i};
	}

	inline std::pair<float, int> min_component_index(const Eigen::Vector3f& a)
	{
		float min_v = a.x();
		int min_i = 0;
		if (a.y() < min_v)
		{
			min_v = a.y();
			min_i = 1;
		}
		if (a.z() < min_v)
		{
			min_v = a.z();
			min_i = 2;
		}
		return {min_v, min_i};
	}

	inline std::pair<float, int> max_component_index(float x, float y, float z)
	{
		float max_v = x;
		int max_i = 0;
		if (y > max_v)
		{
			max_v = y;
			max_i = 1;
		}
		if (z > max_v)
		{
			max_v = z;
			max_i = 2;
		}
		return {max_v, max_i};
	}

	inline std::pair<float, int> min_component_index(float x, float y, float z)
	{
		float min_v = x;
		int min_i = 0;
		if (y < min_v)
		{
			min_v = y;
			min_i = 1;
		}
		if (z < min_v)
		{
			min_v = z;
			min_i = 2;
		}
		return {min_v, min_i};
	}

	inline std::pair<float, int> max_component_index(float x, float y)
	{
		float max_v = x;
		int max_i = 0;
		if (y > max_v)
		{
			max_v = y;
			max_i = 1;
		}
		return {max_v, max_i};
	}

	inline std::pair<float, int> min_component_index(float x, float y)
	{
		float min_v = x;
		int min_i = 0;
		if (y < min_v)
		{
			min_v = y;
			min_i = 1;
		}
		return {min_v, min_i};
	}

	inline Eigen::Vector3f midpoint(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
	{
		return (a + b).array() * 0.5f;
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

	inline float distance(const Eigen::Vector3f& from, const Eigen::Vector3f& to)
	{
		Eigen::Vector3f dir = to - from;
		float length = dir.lpNorm<2>();
		return length;
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

	inline float sign(float f)
	{
		return std::signbit(f) ? -1.0f : 1.0f;
	}

	inline bool isNegative(float f)
	{
		return f <= FLT_EPSILON;
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

