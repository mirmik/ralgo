#ifndef RALGO_LINFILTERS_H
#define RALGO_LINFILTERS_H

#include <linalg.h>

namespace ralgo 
{
	namespace lintrans 
	{
		template <class V> struct inout { 
			virtual V operator()(V in) = 0; 
		};
		
		//W(s)=k
		template <class V, class K=float> struct koeff : public inout<V>
		{
			K k;
			koeff(K _k) : k(_k) {}
			V operator()(V g) override { return k*g; }	
		};

		//W(s)=1/(T+s); a=t/(T+t); t=delta;
		template <class V, class K=float> struct aperiodic : public inout<V>
		{
			K a;
			V x;
			aperiodic(K _a, V init = V()) : a(_a), x(init) {}
			V operator()(V g) override { return x = x + (g - x) * a; }	
		};

		//W(s)=1/(T1*T2*s**2+T1*s+1); a=t/(T1*T2+T1*t); b=T1; t=delta;
		template <class V, class K=float> struct colleb : public inout<V>
		{
			K a, b, c, d, e;
			V x, v;
			colleb(K T, K ksi, K t, V init_x = V(), V init_v = V()) 
				: x(init_x), v(init_v)  
			{
				K T_T = T*T;
				K T_T_t = T*T*t;
				K t_t = t*t;
				K T_ksi_t_2 = 2 * T * ksi * t; 
				K T_T__T_ksi_t_2 = T_T + T_ksi_t_2; 
				
				K znam = T_T__T_ksi_t_2 + t_t;
				a = T_T__T_ksi_t_2 / znam;
				b = T_T_t / znam;
				c = t_t / znam;
				d = T_T / znam;
				e = t / znam;
				PRINT(a);PRINT(b);PRINT(c);
				PRINT(e);PRINT(d);
					}

			V operator()(V g) override 
			{ 
				v = (g-x)*e + d*v;
				return x = a*x + b*v + c*g;
			}	
		};

		template <class V, class K=float> struct zv1 : public inout<V>
		{
			K pp, pg;
			V p;
			zv1(K a, K t, V pi=V()) : p(pi) { K q=a+t; pp=a/q; pg=t/q; }
			V operator()(V g) override { return p = pp*p + pg*g; }	
		};

		template <class V, class K=float> struct zv2 : public inout<V>
		{
			//linalg::mat<V,2> m;
			//linalg::vec<V,2> v;
			
			K pp, pr, pg, rp, rr, rg;
			V p, r;
			zv2(K a, K b, K t, V pi=V(), V ri=V()) : p(pi), r(ri) { 
			//	K q=a+b*t+t*t; 
			//	pp=(a+b*t)/q; pr=(a*t)/q; pg=(t*t)/q;
			//	rp=(-t)/q; rr=(a)/q; rg=(t)/q;

				auto D = sqrt(4*a+b*b)/2;
				auto D1 = D+b/2;
				auto D2 = -D+b/2;
				auto DD = -1/D2+1/D1;
				auto DDD = D1*D2*DD;
				auto DD1 = D1*DD;
				auto DD2 = D2*DD;

				auto ED1 = exp(D1*t);
				auto ED2 = exp(D2*t);

				//m = linalg::mat<V,2> {
				pp = -ED2/DD2+ED1/DD1;
				pr = -ED1/DDD+ED2/DDD;
				rp = ED1/DD-ED2/DD;
				rr = -ED1/DD2+ED2/DD1;
				//};

				rg = 0.9;
				pg = 0;

				PRINT(pp);
				PRINT(rp);
				PRINT(pp);
				PRINT(rr);
			}

			V operator()(V g) override 
			{ 
				V _r = 	rp*p + rr*r + rg*g; 
				p = 	pp*p + pr*r + pg*g; 
				r = _r;
				return p;
			}
		};		
	}
}

#endif