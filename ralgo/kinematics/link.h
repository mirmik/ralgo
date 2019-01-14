#ifndef CYNEMATIC_LINK_H
#define CYNEMATIC_LINK_H

#include <linalg.h>
#include <linalg-ext.h>
#include <linalg-add.h>

#include <nos/trace.h>
#include <gxx/panic.h>

#include <initializer_list>
#include <algorithm>

#include <gxx/util/iteration_counter.h>

#include <thread>
#include <chrono>

#include <numeric>

#include <nos/input.h>

#include <malgo/vector.h>
#include <malgo/matrix.h>
#include <malgo/nrecipes/svd.h>

#define EPSILON 	0.000001
#define EPSILON2 	0.0000001

namespace cynematic
{
	using namespace linalg;
	using namespace linalg::aliases;
	using namespace linalg::ostream_overloads;

	template<typename T, int M>
	malgo::vector<T> backpack(vec<T, M> need, std::vector<vec<T, M>> sens) 
	{
		malgo::matrix<T> m(sens);
		m = transpose(m);
		malgo::vecview<T> b(&need[0], 6);
		malgo::vector<T> x(sens.size());
		malgo::SVD<malgo::matrix<T>> svd(m);

		//PRINT(m);
		//PRINT(svd.u);
		//PRINT(svd.v);
		//PRINT(svd.w);

		try {
		svd.solve(b, x);
		}
		catch (const char * c) {
		//	PRINT(b.size());
		//	PRINT(x.size());
		//	PRINT(m.size1());
		//	PRINT(m.size2());
		//	PRINT(c);
		//	exit(0);
		}
		//PRINT(b);
		//PRINT(x);

		return x;
	}
	

	///Функция выбирает коэфициенты линейной комбинации sens таким образом, чтобы образовать вектор need.
/*	template<typename T, int M>
	malgo::vector<T> backpack(vec<T, M> need, std::vector<vec<T, M>> sens)
	{
		T last_errlen;
		vec<T, M> curvec;
		malgo::vector<T> koeffs(sens.size());
		malgo::vector<T> sens_length2(sens.size());

		for (int i = 0; i < sens.size(); ++i) 
		{
			sens_length2[i] = length2(sens[i]);
		}

		auto error = need - curvec;
		auto errlen = pseudolen1(error);

		do {
			//В цикле покоординатно корректируем вектор. 
			for (int i = 0; i < sens.size(); ++i)
			{
				//Запоминаем то, что получили на последней итерации.
				last_errlen = errlen;

				//Проецируем sens на недостающий кусок вектора. Делим на квадрат sens.
				//Первое деление даёт норму.
				//Таким образом мы получаем длину проекции на sens.
				//Второе деление учитывает масштаб. В конечном итоге получается минимизация расстояния need - curvec. 
				auto koeffadd = linalg::dot(error, sens[i]) / sens_length2[i];
				koeffs[i] += koeffadd;

				//Вычисляем новый curvec
				curvec += sens[i] * koeffadd;
				//curvec = vec<T, M> {};
				//for (int j = 0; j < sens.size(); ++j)
				//{
				//	curvec += sens[j] * koeffs[j];
				//}

				error = need - curvec;
				errlen = pseudolen1(error);

				/*nos::println();
				PRINT(i);
				PRINT(koeffadd);
				PRINT(koeffs);
				PRINT(need);
				PRINT(sens);
				PRINT(curvec);
				PRINT(error);
				PRINT(errlen);*/

				//Условие выхода - минимизация длины ошибки.
/*				if (errlen == 0) break;
			}
		} 
		//Если результат не может быть достигнут, покидаем алгоритм по этому условию.
		while (abs(errlen - last_errlen) > 0.0000000001);

		return koeffs;
	}*/

	template <typename T>
	struct abstract_link
	{
		using ax_t = linalg::vec<T, 3>;

		virtual mtrans<T> get(const malgo::vector<T>& coords,
		                      uint8_t pos) = 0;

		virtual uint8_t count_of_coords() = 0;
	};

	template <typename T>
	struct onedof_link : public abstract_link<T>
	{
		virtual mtrans<T> get(T coord) = 0;

		mtrans<T> get(const malgo::vector<T>& coords, uint8_t pos) override
		{
			return get(coords[pos]);
		}

		virtual bivec<T, 3> d1_bivec() = 0;
	};

	template <typename T>
	struct constant_link : public abstract_link<T>
	{
		mtrans<T> mat;

		mtrans<T> get()
		{
			return mat;
		}

		mtrans<T> get(const malgo::vector<T>& coords, uint8_t pos) override
		{
			return mat;
		}

		uint8_t count_of_coords() override
		{
			return 0;
		}

		constant_link(mtrans<T> _mat) : mat(_mat) {};
	};

	template<typename T>
	struct rotation_link : public onedof_link<T>
	{
		using ax_t = linalg::vec<T, 3>;
		ax_t axvec;

		rotation_link(ax_t _axvec) : axvec(_axvec) {}

		mtrans<T> get(T coord) override
		{
			return mtrans<T>::rotation(axvec, coord);
		}

		uint8_t count_of_coords() override
		{
			return 1;
		}

		bivec<T, 3> d1_bivec() override
		{
			return { axvec, ax_t() };
		}
	};

	template<typename T>
	struct translation_link : public onedof_link<T>
	{
		using ax_t = linalg::vec<T, 3>;
		ax_t axvec;

		translation_link(ax_t _axvec) : axvec(_axvec) {}

		mtrans<T> get(T coord) override
		{
			return mtrans<T>::translation(axvec * coord);
		}

		uint8_t count_of_coords() override { return 1; }

		bivec<T, 3> d1_bivec() override
		{
			return { ax_t(), axvec };
		}
	};

	template <typename T>
	struct chain
	{
		chain() {}
		chain(std::initializer_list<abstract_link<T>*> lst) : links(lst)
		{
			for (auto* r : links) { coords_total += r->count_of_coords(); }
		}

		void add_link(abstract_link<T>* lnk) { links.push_back(lnk); coords_total += lnk->count_of_coords(); }

		mtrans<T> get(const malgo::vector<T>& coords)
		{
			mtrans<T> result {};
			int8_t coord_pos = coords.size() - 1;

			for (int i = links.size() - 1; i >= 0; --i)
			{
				uint8_t count_of_coords = links[i]->count_of_coords();

				if (coord_pos - count_of_coords + 1 < 0)
					return mtrans<T>();

				mtrans<T> nmat = links[i]->get(coords, coord_pos);
				result = nmat * result;
				coord_pos -= count_of_coords;
			}

			return result;
		}

		std::vector<vec<T, 6>> get_speed_transes(const malgo::vector<T>& coords)
		{
			std::vector<vec<T, 6>> result;

			mtrans<T> curtrans {};
			int8_t coord_pos = coords.size() - 1;

			for (int i = links.size() - 1; i >= 0; --i)
			{
				uint8_t count_of_coords = links[i]->count_of_coords();

				//Проверка корректности вектора координат.
				if (coord_pos - count_of_coords + 1 < 0)
					return std::vector<vec<T, 6>>();

				if (count_of_coords > 0)
				{
					if (count_of_coords == 1)
					{
						result.emplace_back(speed_trans(((onedof_link<T>*)links[i])->d1_bivec(), curtrans).concat());
					}
					else
					{
						PANIC_TRACED();
					}
				}

				//перематываем на следующий линк.
				mtrans<T> nmat = links[i]->get(coords, coord_pos);
				curtrans = nmat * curtrans;

				coord_pos -= count_of_coords;
			}

			std::reverse(result.begin(), result.end());
			return result;
		}

		bivec<T, 3> get_sens(int idx, const malgo::vector<T>& coords)
		{
			uint8_t coordpos = coords_total - 1;
			mtrans<T> curtrans {};

			int linkidx = links.size() - 1;

			while (coordpos != idx)
			{
				uint8_t count_of_coords = links[linkidx]->count_of_coords();
				auto nmat = links[linkidx]->get(coords, coordpos);

				curtrans = nmat * curtrans;
				coordpos -= count_of_coords;
				linkidx--;
			}

			return speed_trans(((onedof_link<T>*)links[linkidx])->d1_bivec(), curtrans);
		}

		bivec<T, 3> get_sens_base(int idx, const malgo::vector<T>& coords)
		{
			uint8_t coordpos = coords_total - 1;
			mtrans<T> curtrans {};

			int linkidx = links.size() - 1;

			while (coordpos != idx)
			{
				uint8_t count_of_coords = links[linkidx]->count_of_coords();
				auto nmat = links[linkidx]->get(coords, coordpos);
				curtrans = nmat * curtrans;
				coordpos -= count_of_coords;
				linkidx--;
			}

			auto bi = ((onedof_link<T>*)links[linkidx])->d1_bivec();
			auto w = bi.a;
			auto v = cross(w, curtrans.center) + bi.b;

			mtrans<T> basetrans {};

			while (linkidx >= 0)
			{
				uint8_t count_of_coords = links[linkidx]->count_of_coords();
				auto nmat = links[linkidx]->get(coords, coordpos);
				basetrans = nmat * basetrans;
				coordpos -= count_of_coords;
				linkidx--;
			}

			return { basetrans.rotate(w), basetrans.rotate(v) };
		}


		malgo::vector<T> solve_inverse_cynematic(const mtrans<T>& target, const malgo::vector<T>& _reference, 
			T maxstep = 1)
		{
			assert(_reference.size() == coords_total);
			malgo::vector<T> reference = _reference;

			mtrans<T> itr;
			T lastlen = std::numeric_limits<T>::max();

			while (1)
			{
				auto curtrans = get(reference);
				auto rrr = get_speed_transes(reference);
				auto iv6 = curtrans.vector6_to(target);

				if (pseudolen1(iv6) < 0.0000001) { 
					nos::println("RESULT:", reference);
					break; 
				}
				auto koeffs = backpack(iv6, rrr);


				T koeffs_length = length(koeffs);
				//for (auto f : koeffs) koeffs_length += f * f;
				//koeffs_length = sqrt(koeffs_length);///= (T)koeffs.size();
				//for (auto f : koeffs) koeffs_length += abs(f);
				//koeffs_length /= koeffs.size();


				/*nos::println();
				PRINT(target);
				PRINT(curtrans);
				PRINT(curtrans.inverse() * target);
				PRINT(iv6);
				PRINT(rrr);
				PRINT(koeffs);
				PRINT(koeffs_length);
				PRINT(reference);

				nos::readline();*/

				for (int i = 0; i < coords_total; ++i)
				{
					reference[i] += koeffs[i] * (koeffs_length > maxstep ? (maxstep / koeffs_length) : 1);
				}
			}

			return reference;
		}





		/*std::vector<mtrans<T>> sensivity_matrices(const std::vector<T>& coords)
		{
			TRACE();
			std::vector<mtrans<T>> result;
			mtrans<T> curtrans = identity;
			int8_t coord_pos = coords.size() - 1;

			for (int i = links.size() - 1; i >= 0; --i)
			{
				uint8_t count_of_coords = links[i]->count_of_coords();

				if (coord_pos - count_of_coords + 1 < 0)
					return std::vector<mtrans<T>>();

				if (count_of_coords > 0)
				{
					if (count_of_coords == 1)
					{
						auto sensmat = links[i]->sensmat();
						result.emplace_back(curtrans * sensmat);
					}
					else
					{
						return std::vector<mtrans<T>>();
					}
				}

				mtrans<T> nmat = links[i]->get(coords, coord_pos);
				curtrans = nmat * curtrans;

				coord_pos -= count_of_coords;
			}

			return result;

		}

		std::vector<T> sensivity(std::vector<T> curcoords, mtrans<T> target)
		{
			auto curmat = get(curcoords);
			auto needmat = target * inverse(curmat);

			auto sensmats = sensivity_matrices(curcoords);

			std::cout << "needmat: " << needmat << std::endl;
			for (auto m : sensmats)
			{
				std::cout << "smat: " << m << std::endl;
			}
		}*/
	private:
		std::vector<abstract_link<T> *> links;
		size_t coords_total = 0;
	};
	/*
		struct dynamic_chain : public chain
		{
			~dynamic_chain()
			{
				for (int i = 0; i < links.size(); ++i) delete links[i];
			}
		};*/
}

#endif