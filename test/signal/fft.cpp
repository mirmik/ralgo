#include <doctest/doctest.h>
#include <ralgo/signal/fft.h>

#include <vector>
#include <nos/print.h>

TEST_CASE("fft") 
{
	std::vector<double> freqs;

	 auto tim = ralgo::vecops::linspace(0., 12 * M_PI * 2., 1<<8);
	 auto sig = ralgo::vecops::sin(tim);

	 auto ret = ralgo::signal::spectre(sig.data(), sig.size(), sig.size());
	 
	 freqs.resize((1<<8) / 2);
	 ralgo::signal::rfftfreq(freqs.data(), 1<<8, (1.)/(1<<8));


	 nos::println("values:");
	 for (unsigned int i = 0; i < ret.size()/2; ++i) 
	 {
	 	nos::println(ret[i], freqs[i]);
	 }
}