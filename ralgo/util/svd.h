#ifndef RALGO_SVD_H
#define RALGO_SVD_H

#include <limits>
#include <ralgo/vecops.h>
#include <ralgo/matops.h>
#include <ralgo/util/math.h>
#include <ralgo/fault.h>
// Based on numerical recipes.
//Attention Row Order.

#include <nos/print.h>

namespace ralgo
{


	template <class T, class MU, class MV, class V>
	struct SVD
	{
		int m, n;
		MU u;
		MV v;
		V  w;
		T  eps;
		T  tsh;

		template<class MA>
		SVD(const MA & a, MU& u, MV& v, V& w) :
			m(a.size1()), n(a.size2()), // берём размерность.
			u(u), v(v), w(w)
			//u(a), v(n, n), w(n)
		{
//			if (u.size1() != a.size1()
//			        || u.size2() != a.size2()
//			        || v.size1() != n
//			        || v.size2() != n
//			        || w.size() != n)
//				ralgo::fault("incompatible matrices");

			assert(u.size1() == (unsigned)m);
			assert(u.size2() == (unsigned)n);
			assert(v.size1() == (unsigned)n);
			assert(v.size2() == (unsigned)n);
			assert(w.size() == (unsigned)n);

				//ralgo::vecops::copy(u, a); // Копируем данные.
			ralgo::matops::copy(u, a); // Копируем данные.
			ralgo::vecops::inplace::clean(v);
			ralgo::vecops::inplace::clean(w);

			eps = std::numeric_limits<T>::epsilon();

			decompose();
			
			reorder();
			tsh = 0.5 * sqrt(m + n + 1.) * w[0] * eps;
		}

		/*SVD(mroot<M> &a) : m(a.size1()), n(a.size2()), u(a), v(n, n), w(n)
		{
			eps = std::numeric_limits<type_t<M>>::epsilon();
			decompose();
			reorder();
			tsh = 0.5 * sqrt(m + n + 1.) * w[0] * eps;
		}*/

		template<class A, class B> 
		void solve(const A &b, B &x, T thresh = -1.);
		//void solve(const mroot<M> &b, mroot<M> &x, type_t<M> thresh = -1.);
		/*
		int 			rank(type_t<M> thresh = -1.);
		int 			nullity(type_t<M> thresh = -1.);
		matrix_t<M> 	range(type_t<M> thresh = -1.);
		matrix_t<M> 	nullspace(type_t<M> thresh = -1.);

		type_t<M> inv_condition()
		{
			return (w[0] <= 0. || w[n - 1] <= 0.) ? 0. : w[n - 1] / w[0];
		}
		*/
		void decompose();
		void reorder();

		T pythag(const T a, const T b)
		{
			T absa = abs(a), absb = abs(b);
			return (absa > absb ? absa * sqrt(1.0 + ralgo::sqr(absb / absa)) :
			        (absb == 0.0 ? 0.0 : absb * sqrt(1.0 + ralgo::sqr(absa / absb))));
		}
	};
	
	template<class T, class MU, class MV, class V>
	template<class A, class B>
	void SVD<T, MU, MV, V>::solve(const A &b, B &x, T thresh)
	{
		int i, j, jj;
		T s;
		if (b.size() != (unsigned)m || x.size() != (unsigned)n) ralgo::fault("SVD::solve bad sizes");
		//vector_t<M> tmp(n);
		T tmp[n];
		tsh = (thresh >= 0. ? thresh : 0.5 * sqrt(m + n + 1.) * w[0] * eps);
		for (j = 0; j < n; j++)
		{
			s = 0.0;
			if (w[j] > tsh)
			{
				for (i = 0; i < m; i++) s += u[i][j] * b[i];
				s /= w[j];
			}
			tmp[j] = s;
		}
		for (j = 0; j < n; j++)
		{
			s = 0.0;
			for (jj = 0; jj < n; jj++) s += v[j][jj] * tmp[jj];
			x[j] = s;
		}
	}
/*
	template <typename M>
	void SVD<M>::solve(const mroot<M> &b, mroot<M> &x, type_t<M> thresh)
	{
		int i, j, p = b.ncols();
		if (b.nrows() != m || x.nrows() != n || x.ncols() != p)
			throw("SVD::solve bad sizes");
		vector_t<M> xx(n), bcol(m);
		for (j = 0; j < p; j++)
		{
			for (i = 0; i < m; i++) bcol[i] = b[i][j];
			solve(bcol, xx, thresh);
			for (i = 0; i < n; i++) x[i][j] = xx[i];
		}
	}

	template <typename M>
	int SVD<M>::rank(type_t<M> thresh)
	{
		int j, nr = 0;
		tsh = (thresh >= 0. ? thresh : 0.5 * sqrt(m + n + 1.) * w[0] * eps);
		for (j = 0; j < n; j++) if (w[j] > tsh) nr++;
		return nr;
	}

	template <typename M>
	int SVD<M>::nullity(type_t<M> thresh)
	{
		int j, nn = 0;
		tsh = (thresh >= 0. ? thresh : 0.5 * sqrt(m + n + 1.) * w[0] * eps);
		for (j = 0; j < n; j++) if (w[j] <= tsh) nn++;
		return nn;
	}

	template <typename M>
	matrix_t<M> SVD<M>::range(type_t<M> thresh)
	{
		int i, j, nr = 0;
		matrix_t<M> rnge(m, rank(thresh));
		for (j = 0; j < n; j++)
		{
			if (w[j] > tsh)
			{
				for (i = 0; i < m; i++) rnge[i][nr] = u[i][j];
				nr++;
			}
		}
		return rnge;
	}

	template <typename M>
	matrix_t<M> SVD<M>::nullspace(type_t<M> thresh)
	{
		int j, jj, nn = 0;
		matrix_t<M> nullsp(n, nullity(thresh));
		for (j = 0; j < n; j++)
		{
			if (w[j] <= tsh)
			{
				for (jj = 0; jj < n; jj++) nullsp[jj][nn] = v[jj][j];
				nn++;
			}
		}
		return nullsp;
	}
	*/
	template <class T, class MU, class MV, class V>
	void SVD<T, MU, MV, V>::decompose()
	{
		bool flag;
		int i, its, j, jj, k, l, nm;
		T anorm, c, f, g, h, s, scale, x, y, z;
		//vector_t<M> rv1(n);
		T rv1[n];

		g = scale = anorm = 0.0;
		for (i = 0; i < n; i++)
		{
			l = i + 2;
			rv1[i] = scale * g;
			g = s = scale = 0.0;
			if (i < m)
			{
				for (k = i; k < m; k++) scale += abs(u[k][i]);
				if (scale != 0.0)
				{
					for (k = i; k < m; k++)
					{
						u[k][i] /= scale;
						s += u[k][i] * u[k][i];
					}
					f = u[i][i];
					g = -ralgo::sign(sqrt(s), f);
					h = f * g - s;
					u[i][i] = f - g;
					for (j = l - 1; j < n; j++)
					{
						for (s = 0.0, k = i; k < m; k++) s += u[k][i] * u[k][j];
						f = s / h;
						for (k = i; k < m; k++) u[k][j] += f * u[k][i];
					}
					for (k = i; k < m; k++) u[k][i] *= scale;
				}
			}
			w[i] = scale * g;
			g = s = scale = 0.0;
			if (i + 1 <= m && i + 1 != n)
			{
				for (k = l - 1; k < n; k++) scale += abs(u[i][k]);
				if (scale != 0.0)
				{
					for (k = l - 1; k < n; k++)
					{
						u[i][k] /= scale;
						s += u[i][k] * u[i][k];
					}
					f = u[i][l - 1];
					g = -ralgo::sign(sqrt(s), f);
					h = f * g - s;
					u[i][l - 1] = f - g;
					for (k = l - 1; k < n; k++) rv1[k] = u[i][k] / h;
					for (j = l - 1; j < m; j++)
					{
						for (s = 0.0, k = l - 1; k < n; k++) s += u[j][k] * u[i][k];
						for (k = l - 1; k < n; k++) u[j][k] += s * rv1[k];
					}
					for (k = l - 1; k < n; k++) u[i][k] *= scale;
				}
			}
			anorm = ralgo::max(anorm, (abs(w[i]) + abs(rv1[i])));
		}

		for (i = n - 1; i >= 0; i--)
		{
			if (i < n - 1)
			{
				if (g != 0.0)
				{
					for (j = l; j < n; j++)
						v[j][i] = (u[i][j] / u[i][l]) / g;
					for (j = l; j < n; j++)
					{
						for (s = 0.0, k = l; k < n; k++) s += u[i][k] * v[k][j];
						for (k = l; k < n; k++) v[k][j] += s * v[k][i];
					}
				}
				for (j = l; j < n; j++) v[i][j] = v[j][i] = 0.0;
			}
			v[i][i] = 1.0;
			g = rv1[i];
			l = i;
		}

		for (i = ralgo::min(m, n) - 1; i >= 0; i--)
		{
			l = i + 1;
			g = w[i];
			for (j = l; j < n; j++) u[i][j] = 0.0;
			if (g != 0.0)
			{
				g = 1.0 / g;
				for (j = l; j < n; j++)
				{
					for (s = 0.0, k = l; k < m; k++) s += u[k][i] * u[k][j];
					f = (s / u[i][i]) * g;
					for (k = i; k < m; k++) u[k][j] += f * u[k][i];
				}
				for (j = i; j < m; j++) u[j][i] *= g;
			}
			else for (j = i; j < m; j++) u[j][i] = 0.0;
			++u[i][i];
		}
		for (k = n - 1; k >= 0; k--)
		{
			for (its = 0; its < 30; its++)
			{
				flag = true;
				for (l = k; l >= 0; l--)
				{
					nm = l - 1;
					if (l == 0 || abs(rv1[l]) <= eps * anorm)
					{
						flag = false;
						break;
					}
					if (abs(w[nm]) <= eps * anorm) break;
				}
				if (flag)
				{
					c = 0.0;
					s = 1.0;
					for (i = l; i < k + 1; i++)
					{
						f = s * rv1[i];
						rv1[i] = c * rv1[i];
						if (abs(f) <= eps * anorm) break;
						g = w[i];
						h = pythag(f, g);
						w[i] = h;
						h = 1.0 / h;
						c = g * h;
						s = -f * h;
						for (j = 0; j < m; j++)
						{
							y = u[j][nm];
							z = u[j][i];
							u[j][nm] = y * c + z * s;
							u[j][i] = z * c - y * s;
						}
					}
				}
				z = w[k];
				if (l == k)
				{
					if (z < 0.0)
					{
						w[k] = -z;
						for (j = 0; j < n; j++) v[j][k] = -v[j][k];
					}
					break;
				}
				if (its == 29) 
					ralgo::fault("no convergence in 30 svdcmp iterations");
				x = w[l];
				nm = k - 1;
				y = w[nm];
				g = rv1[nm];
				h = rv1[k];
				f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
				g = pythag(f, 1.0);
				f = ((x - z) * (x + z) + h * ((y / (f + ralgo::sign(g, f))) - h)) / x;
				c = s = 1.0;
				for (j = l; j <= nm; j++)
				{
					i = j + 1;
					g = rv1[i];
					y = w[i];
					h = s * g;
					g = c * g;
					z = pythag(f, h);
					rv1[j] = z;
					c = f / z;
					s = h / z;
					f = x * c + g * s;
					g = g * c - x * s;
					h = y * s;
					y *= c;
					for (jj = 0; jj < n; jj++)
					{
						x = v[jj][j];
						z = v[jj][i];
						v[jj][j] = x * c + z * s;
						v[jj][i] = z * c - x * s;
					}
					z = pythag(f, h);
					w[j] = z;
					if (z)
					{
						z = 1.0 / z;
						c = f * z;
						s = h * z;
					}
					f = c * g + s * y;
					x = c * y - s * g;
					for (jj = 0; jj < m; jj++)
					{
						y = u[jj][j];
						z = u[jj][i];
						u[jj][j] = y * c + z * s;
						u[jj][i] = z * c - y * s;
					}
				}
				rv1[l] = 0.0;
				rv1[k] = f;
				w[k] = x;
			}
		}
	}

	template <class T, class MU, class MV, class V>
	void SVD<T,MU,MV,V>::reorder()
	{
		int i, j, k, s, inc = 1;
		T sw;
		//vector_t<M> su(m), sv(n);
		T su[m];
		T sv[n];
		do { inc *= 3; inc++; }
		while (inc <= n);
		do
		{
			inc /= 3;
			for (i = inc; i < n; i++)
			{
				sw = w[i];
				for (k = 0; k < m; k++) su[k] = u[k][i];
				for (k = 0; k < n; k++) sv[k] = v[k][i];
				j = i;
				while (w[j - inc] < sw)
				{
					w[j] = w[j - inc];
					for (k = 0; k < m; k++) u[k][j] = u[k][j - inc];
					for (k = 0; k < n; k++) v[k][j] = v[k][j - inc];
					j -= inc;
					if (j < inc) break;
				}
				w[j] = sw;
				for (k = 0; k < m; k++) u[k][j] = su[k];
				for (k = 0; k < n; k++) v[k][j] = sv[k];

			}
		}
		while (inc > 1);
		for (k = 0; k < n; k++)
		{
			s = 0;
			for (i = 0; i < m; i++) if (u[i][k] < 0.) s++;
			for (j = 0; j < n; j++) if (v[j][k] < 0.) s++;
			if (s > (m + n) / 2)
			{
				for (i = 0; i < m; i++) u[i][k] = -u[i][k];
				for (j = 0; j < n; j++) v[j][k] = -v[j][k];
			}
		}
	}



	template <class MA, class MU, class MV, class V>
	SVD<typename MA::value_type, MU, MV, V>
	make_SVD(const MA & a, MU& u, MV& v, V& w)
	{
		return SVD<typename MA::value_type, MU, MV, V>(a, u, v, w);
	}
}


#endif