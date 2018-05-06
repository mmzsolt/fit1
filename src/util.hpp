#pragma once

namespace util
{
	template <typename T>
	T clamp(T v, T a, T b)
	{
		return v < a ? a : (v > b ? b : v);
	}

	template <typename T>
	T lerp(T r, T a, T b)
	{
		return (1 - r) * a + r * b;
	}

	template <typename T>
	T ratio(T v, T a, T b)
	{
		return (v - a) / (b - a);
	}
}

