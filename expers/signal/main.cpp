#include <ralgo/signal/fft.h>
#include <ralgo/signal/voice.h>
#include <nos/print.h>

#include <math.h>

#include <ralgo/vecops.h>
#include <ralgo/signal/convolution.h>

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

	constexpr auto hz2mel = ralgo::vectorize<std::vector<double>>(&ralgo::hz2mel);
	constexpr auto mel2hz = ralgo::vectorize<std::vector<double>>(&ralgo::mel2hz);

	nos::println("\t", ralgo::hz2mel_vi(freqs));
	nos::println("\t", ralgo::mel2hz_vi(freqs));

	ralgo::signal::triangle_window w(15, 25);

	std::vector<double> keys;

	nos::println("keypoints:", keys = w.keypoints_map(freqs));
	nos::println("keypoints:", keys = ralgo::signal::lerp_keypoints(freqs, keys));
}