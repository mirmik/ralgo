#ifndef RALGO_LINFILTERS_H
#define RALGO_LINFILTERS_H

#include <linalg.h>
#include <linalg-add.h>

using namespace linalg::ostream_overloads;

namespace ralgo 
{
	namespace lintrans 
	{
		template <class V> struct inout { 
			virtual V operator()(V in) = 0;
			virtual void print_internal() { nos::println("TODO"); };
		};
		
		template <class V> struct inout_state : public inout<V> { 
			virtual V output() = 0; 
		};
		
		//W(s)=k
		template <class V, class K=float> struct koeff : public inout<V>
		{
			K k;
			koeff(K _k) : k(_k) {}
			V operator()(V g) override { return k*g; }	
		};

		//W(s)=1/(T+s); a=t/(T+t); t=delta;
		/*template <class V, class K=float> struct aperiodic : public inout<V>
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
		};*/

		template <class V, class K=float> struct integrator : public inout_state<V>
		{
			V integral;
			integrator(V init=V()) : integral(init) { }
			V operator()(V g) override { return integral+=g; }	
			V output() { return integral; }
		};
	
		template <class V, class K=float> struct aperiodic : public inout_state<V>
		{
			V _a;
			K pp, pg;
			V p;
			V a() { return _a; }
			aperiodic(K a, K t, V pi=V()) : p(pi), _a(a) { K q=a+t; pp=a/q; pg=t/q; }
			V operator()(V g) override { return p = pp*p + pg*g; }	
			V output() { return p; }

			void print_internal()  override
			{
				PRINT(_a);
			}
		};

		template <class V, class K=float> struct oscilator : public inout_state<V>
		{
			V _a, _b;
			linalg::mat<V,2,2> A;
			linalg::vec<V,2> B;
			linalg::vec<V,2> x;
			
			V a() { return _a; }
			V b() { return _b; }

			oscilator(K a, K b, K t) : x{0,0}, B{0,1}, A{{0,-a},{1,-b}}, _a(a), _b(b)
			{ 
				auto _A = exponent(A * t);
				auto I = linalg::mat<V,2,2>{linalg::identity};
				auto _B = inverse(A) * ((_A - I) * B);
				A = _A; B = _B;
			}

			V operator()(V g) override 
			{ 
				x = A*x + B*g;
				return x[0];
			}

			V output() { return x[0]; }

			void print_internal()  override
			{
				PRINT(_a);
				PRINT(_b);
				PRINT(A);
				PRINT(B);
			}
		};		


		template <class T, class K=float>
		struct pi : public inout<T>
		{
			K _kp;
			K ki_discr;
			K delta;
	
			T integral = 0;

			T kp() const { return _kp; }
			T ki() const { return ki_discr / delta; }
			T kip() const { return kip() / _kp; }
	
			pi(K kp, K ki, K delta) : _kp(kp), ki_discr(ki * delta), delta(delta)
			{
				//nos::println("pi");	
			}
	
			T operator()(T error) override
			{
				//nos::println("here");
				integral += error;
				return _kp * error + ki_discr * integral;
			}

			void print_internal() override
			{
				PRINT(_kp);
				PRINT(ki_discr);
				PRINT(delta);
			}
		};
	}
}

#endif