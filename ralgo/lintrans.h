#ifndef RALGO_LINFILTERS_H
#define RALGO_LINFILTERS_H

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
			zv1(K a, K t) { K q=a+t; pp=a/q; pg=t/q; }
			V operator()(V g) override { return p = pp*p + pg*g; }	
		};

		template <class V, class K=float> struct zv2 : public inout<V>
		{
			K pp, pr, pg, rp, rr, rg;
			V p, r;
			zv2(K a, K b, K t) { 
				K q=a+b*t+t*t; 
				pp=(a+b*t)/q; pr=(a*t)/q; pg=t*t;
				rp=(-t)/q; rr=(a)/q; rg=(t)/q;
			}

			V operator()(V g) override { r = rp*p + rr*r + rg*g; return p = pp*p + pr*r + pg*g; }
		};		
	}
}

#endif