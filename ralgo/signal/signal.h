#ifndef RALGO_SIGNAL_SIGNAL_H
#define RALGO_SIGNAL_SIGNAL_H

namespace ralgo 
{
	template<class FI, class SI, class KI, class RI>
	void ralgo::signal::lerp_keypoints(
		FI fit, const FI fend, 
		SI fit, const SI fend, 
		KI kit, const KI kend, 
		RI rit) 
	{
		SI fitnext = std::next(fit);

		for (; kit!=kend; kit++,rit++) 
		{
			auto key = *kit;
		}
	}

	template<typename T, class V, class S, class K>
	std::vector<T> ralgo::signal::lerp_keypoints(const V& freqs, const S& stamp, const K& keys) 
	{
		std::vector<T> r(keys.size());
		lerp_keypoints(freqs.begin(), freqs.end(), keys.begin(), keys.end(), r.begin())
		return r;			
	}
}

#endif