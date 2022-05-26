#ifndef RALGO_GEOM_ALGEBRA_H
#define RALGO_GEOM_ALGEBRA_H

#include <iostream>

namespace ralgo 
{
    namespace geom 
    {
        template <class T>
        class scalar 
        {
        public:
            T e;
            scalar(T e) : e(e) {}
            scalar(const scalar& s) = default;
            bool operator<=>(const scalar&) const = default;            
        };

        template <class T>
        class vector 
        {
        public:
            T e1, e2, e3;
            vector(T e1, T e2, T e3) : e1(e1), e2(e2), e3(e3) {}
            vector(const vector&) = default;
            bool operator<=>(const vector&) const = default;
        };

        template <class T>
        class bivector 
        {
        public:
            T e23, e31, e12;
            bivector(T e23, T e31, T e12) : e23(e23), e31(e31), e12(e12) {}
            bivector(const bivector&) = default;
            bool operator<=>(const bivector&) const = default;
        };

        template <class T>
        class trivector 
        {
        public:
            T e123;
            trivector(T e123) : e123(e123) {}
            trivector(const trivector&) = default;
            bool operator<=>(const trivector&) const = default;
        };

        /*template <class T>
        class multivector
        {
            scalar<T> s;
            vector<T> v;
            bivector<T> b;
            trivector<T> t;
        };*/

        template <class T>
        class scalar_bivector : public scalar<T>, public bivector<T>
        {
        public:
            scalar_bivector(T e, T e23, T e31, T e12) : scalar<T>(e), bivector<T>(e23, e31, e12) {}
            scalar_bivector(scalar<T> e, bivector<T> b) : scalar<T>(e), bivector<T>(b) {}
            scalar_bivector(const scalar_bivector&) = default;
            bool operator<=>(const scalar_bivector&) const = default;
        };

        
        template <class T> scalar<T> s_ss(const scalar<T>& a, const scalar<T>& b) { return scalar(a.e*b.e); } 
        template <class T> scalar<T> s_vv(const vector<T>& a, const vector<T>& b) { return scalar(a.e1*b.e1 + a.e2*b.e2 + a.e3*b.e3); }
        template <class T> scalar<T> s_bb(const bivector<T>& a, const bivector<T>& b) { return scalar( - a.e12 * b.e12 - a.e23 * b.e23 - a.e31 * b.e31); }
        template <class T> scalar<T> s_tt(const trivector<T>& a, const trivector<T>& b) { return scalar(-a.e123*b.e123); }        
        
        template <class T> vector<T> v_sv(const scalar<T>& a, const vector<T>& b) { return vector(a.e*b.e1, a.e*b.e2, a.e*b.e3); }
        template <class T> vector<T> v_vs(const vector<T>& a, const scalar<T>& b) { return vector(a.e1*b.e, a.e2*b.e, a.e3*b.e); }
        template <class T> vector<T> v_vb(const vector<T>& a, const bivector<T>& b) { return vector(- a.e2 * b.e12 + a.e3 * b.e31, - a.e3 * b.e23 + a.e1 * b.e12, - a.e1 * b.e31 + a.e2 * b.e23); }
        template <class T> vector<T> v_bv(const bivector<T>& a, const vector<T>& b) { return vector(- a.e31 * b.e3 + a.e12 * b.e2, - a.e12 * b.e1 + a.e23 * b.e3, - a.e23 * b.e2 + a.e31 * b.e1);}

        template <class T> bivector<T> b_bs(const bivector<T>& a, const scalar<T>& b) { return bivector(a.e23*b.e, a.e31*b.e, a.e12*b.e); }
        template <class T> bivector<T> b_sb(const scalar<T>& a, const bivector<T>& b) { return bivector(a.e*b.e23, a.e*b.e31, a.e*b.e12); }
        template <class T> bivector<T> b_vv(const vector<T>& a, const vector<T>& b) { return bivector(a.e2*b.e3 - a.e3*b.e2, a.e3*b.e1 - a.e1*b.e3, a.e1*b.e2 - a.e2*b.e1); }
        template <class T> bivector<T> b_bb(const bivector<T>& a, const bivector<T>& b) { return bivector(- a.e31*b.e12 + a.e12*b.e31, - a.e12*b.e23 + a.e23*b.e12, - a.e23*b.e31 + a.e31*b.e23); }

        template <class T> trivector<T> t_ts(const trivector<T>& a, const scalar<T>& b) { return trivector(a.e123*b.e); }
        template <class T> trivector<T> t_st(const scalar<T>& a, const trivector<T>& b) { return trivector(a.e*b.e123); }
        template <class T> trivector<T> t_vb(const vector<T>& a, const bivector<T>& b) { return trivector(a.e1*b.e23 + a.e2*b.e31 + a.e3*b.e12); }
        template <class T> trivector<T> t_bv(const bivector<T>& a, const vector<T>& b) { return trivector(a.e23*b.e1 + a.e31*b.e2 + a.e12*b.e3); }

        template <class T> vector<T> sv_vs_v(const scalar<T>& as, const vector<T>& bv, const vector<T>& av, const scalar<T>& bs) { return sv_v(as, bv) + vs_v(av, bs); }
        template <class T> bivector<T> sb_bs_b(const scalar<T>& as, const bivector<T>& bb, const bivector<T>& ab, const scalar<T>& bs) { return sb_b(as, bb) + bs_b(ab, bs); }

        template <class T> scalar<T> operator+(const scalar<T>& a, const scalar<T>& b) { return scalar(a.e + b.e); }
        template <class T> scalar<T> operator-(const scalar<T>& a, const scalar<T>& b) { return scalar(a.e - b.e); }
        template <class T> vector<T> operator+(const vector<T>& a, const vector<T>& b) { return vector(a.e1 + b.e1, a.e2 + b.e2, a.e3 + b.e3); }
        template <class T> vector<T> operator-(const vector<T>& a, const vector<T>& b) { return vector(a.e1 - b.e1, a.e2 - b.e2, a.e3 - b.e3); }
        template <class T> bivector<T> operator+(const bivector<T>& a, const bivector<T>& b) { return bivector(a.e23 + b.e23, a.e31 + b.e31, a.e12 + b.e12); }
        template <class T> bivector<T> operator-(const bivector<T>& a, const bivector<T>& b) { return bivector(a.e23 - b.e23, a.e31 - b.e31, a.e12 - b.e12); }
        template <class T> trivector<T> operator+(const trivector<T>& a, const trivector<T>& b) { return trivector(a.e123 + b.e123); }
        template <class T> trivector<T> operator-(const trivector<T>& a, const trivector<T>& b) { return trivector(a.e123 - b.e123); }

        template <class T>
        scalar_bivector<T> geommul(const scalar_bivector<T>& a, const scalar_bivector<T>& b) 
        {
            scalar<T> s = s_ss(a, b) + s_bb(a, b);
            bivector<T> B = b_sb(a, b) + b_bs(a, b) + b_bb(a, b);
            return scalar_bivector(s, B);
        }

        /*template <class T>
        multivector geommul(const multivector& a, const multivector& b) 
        {
            scalar s =    ss_s(a.s, b.s) + vv_s(a.v, b.v) + bb_s(a.b, b.b) + tt_s(a.t, b.t); 
            vector v =    sv_vs_v(a.s, b.v, a.v, b.s) + vb_bv_v(a.v, b.b, a.b, b.v) + bt_tb_v(a.b, b.t, a.t, b.b);
            bivector b =  sb_bs_b(a.s, b.b, a.b, b.s) + sb_bs_b(a.s, b.b, a.b, b.s) + vt_tv_b(a.v, b.t, a.t, b.v) + bb_b(a.b, b.b);
            trivector t = st_ts_t(a.s, b.t, a.t, b.s) + vb_bv_t(a.v, b.b, a.b, b.v);
            return multivector(s, v, b, t);
        }*/

        template <class T>
        std::ostream& operator<<(std::ostream& os, const scalar<T>& a)
        {
            os << a.e;
            return os;
        }

        template <class T>
        std::ostream& operator<<(std::ostream& os, const vector<T>& a)
        {
            os << a.e1 << " " << a.e2 << " " << a.e3;
            return os;
        }

        template <class T>
        std::ostream& operator<<(std::ostream& os, const bivector<T>& a)
        {
            os << a.e23 << " " << a.e31 << " " << a.e12;
            return os;
        }

        template <class T>
        std::ostream& operator<<(std::ostream& os, const trivector<T>& a)
        {
            os << a.e123;
            return os;
        }

        template <class T>
        std::ostream& operator<<(std::ostream& os, const scalar_bivector<T>& a)
        {
            os << (scalar<T>&)a << " " << (bivector<T>&)a;
            return os;
        }

    }
}

#endif