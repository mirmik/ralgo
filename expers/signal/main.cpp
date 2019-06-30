#include <ralgo/signal/fft.h>
#include <ralgo/signal/voice.h>
#include <nos/print.h>

#include <math.h>

#include <ralgo/vecops.h>

int main() 
{	
	std::vector<std::complex<double>> arr(8);
	std::vector<double> arr0 {1,2,1,2,1,2,1,2};

	for (int i = 0; i < arr0.size(); ++i) 
	{
		//arr0[i] = 42;
		//arr0[i] = cos((double)i * M_PI / 4) * 3;
	}

	for (int i = 0; i < arr.size(); ++i) 
	{
		arr[i] = arr0[i];
	}

	arr0 = ralgo::vecops::arange<std::vector<double>>(8);
	ralgo::vecops::inplace::sin(arr0);

	//nos::println(ralgo::vecops::norm(arr0));
	//nos::println(1./ralgo::vecops::norm(arr0));

	nos::print("Source array:\n\t");
	nos::println(arr0);


	//nos::print("Spectre:\n\t");
	//auto spectre_arr = ralgo::signal::spectre(arr0.data(), std::size(arr0), std::size(arr0));
	//nos::println(spectre_arr);

	//nos::print("Swaper spectre:\n\t");
	//auto reverse_spectre_arr = ralgo::vecops::reverse(spectre_arr);
	//nos::println(reverse_spectre_arr);

	nos::print("FFT:\n\t");
	for (int i = 0; i < arr.size(); ++i) { arr[i] = arr0[i]; }
	ralgo::signal::fft(arr);
	//ralgo::vecops::inplace::div(arr, 8);
	nos::println(ralgo::vecops::real(arr));
	nos::print("\t");
	nos::println(ralgo::vecops::imag(arr));
	nos::print("\t");
	nos::println(ralgo::vecops::abs(arr));

	nos::print("Reverse FFT:\n\t");
	ralgo::signal::ifft(arr);
	//ralgo::vecops::inplace::mul(arr, 8);
	nos::println(ralgo::vecops::real(arr));
	nos::print("\t");
	nos::println(ralgo::vecops::imag(arr));

	nos::print("FREQS:\n\t");
	nos::println(ralgo::signal::fftfreq(arr.size(), .1/8));
	nos::print("\t");
	nos::println(ralgo::signal::rfftfreq(arr.size(), .1/8));

	nos::println("MELS");
	auto freqs = ralgo::signal::rfftfreq(arr.size(), .1/8);
	nos::println("\t", freqs);
	nos::println("\t", freqs = ralgo::vectorize(freqs, ralgo::hz2mel));
	nos::println("\t", freqs = ralgo::vectorize(freqs, ralgo::mel2hz));
}