#ifndef RALGO_GEOM_ALGEBRA_H
#define RALGO_GEOM_ALGEBRA_H

#include <iostream>
#include <ralgo/linalg/linalg.h>

namespace ralgo 
{
    namespace geom 
    {
        template <class U>
        class scalar 
        {
        public:
            U e;
            scalar(U e) : e(e) {}
            scalar(const scalar& s) = default;
            bool operator<=>(const scalar&) const = default;            
        };

        template <class U>
        class vector
        {
        public:
            U e1, e2, e3;
            vector(U e1, U e2, U e3) : e1(e1), e2(e2), e3(e3) {}
            vector(const linalg::vec<U,3>& v) : e1(v[0]), e2(v[1]), e3(v[2]) {}
            vector(const vector&) = default;
            bool operator<=>(const vector&) const = default;
            const linalg::vec<U,3>& xyz() const { return *(linalg::vec<U,3>*)this; }
        };

        template <class U>
        class bivector 
        {
        public:
            U e23, e31, e12;
            bivector(U e23, U e31, U e12) : e23(e23), e31(e31), e12(e12) {}
            bivector(const linalg::vec<U,3>& v) : e23(v[0]), e31(v[1]), e12(v[2]) {}
            bivector(const bivector&) = default;
            bool operator<=>(const bivector&) const = default;
            const linalg::vec<U,3>& xyz() const { return *(linalg::vec<U,3>*)this; }
        };

        template <class U>
        class dual_scalar 
        {
        public:
            U e4;
            dual_scalar(U e4) : e4(e4) {}
            dual_scalar(const dual_scalar& s) = default;
            bool operator==(const dual_scalar& oth) { return e4 == oth.e4; }
            bool operator!=(const dual_scalar& oth) { return e4 != oth.e4; }
        };

        template <class U>
        class dual_vector 
        {
        public:
            U e14, e24, e34;
            dual_vector(U e14, U e24, U e34) : e14(e14), e24(e24), e34(e34) {}
            dual_vector(const linalg::vec<U,3>& v) : e14(v[0]), e24(v[1]), e34(v[2]) {}
            dual_vector(const dual_vector&) = default;
            bool operator==(const dual_vector& oth) { return e14 == oth.e14 && e24 == oth.e24 && e34 == oth.e34; }
            bool operator!=(const dual_vector& oth) { return e14 != oth.e14 || e24 != oth.e24 || e34 != oth.e34; }
            const linalg::vec<U,3>& xyz() const { return *(linalg::vec<U,3>*)this; }
        };

        template <class U>
        class dual_bivector 
        {
        public:
            U e234, e314, e124;
            dual_bivector(U e234, U e314, U e124) : e234(e234), e314(e314), e124(e124) {}
            dual_bivector(const linalg::vec<U,3>& v) : e234(v[0]), e314(v[1]), e124(v[2]) {}
            dual_bivector(const dual_bivector&) = default;
            bool operator==(const dual_bivector& oth) { return e234 == oth.e234 && e314 == oth.e314 && e124 == oth.e124; }
            bool operator!=(const dual_bivector& oth) { return e234 != oth.e234 || e314 != oth.e314 || e124 != oth.e124; }
            const linalg::vec<U,3>& xyz() const { return *(linalg::vec<U,3>*)this; }
        };

        template <class U>
        class trivector 
        {
        public:
            U e123;
            trivector(U e123) : e123(e123) {}
            trivector(const trivector&) = default;
            bool operator<=>(const trivector&) const = default;
        };

        template <class U>
        class dual_trivector 
        {
        public:
            U e1234;
            dual_trivector(U e1234) : e1234(e1234) {}
            dual_trivector(const dual_trivector&) = default;
            bool operator<=>(const dual_trivector&) const = default;
        };

        template <class type> using S = scalar<type>;
        template <class type> using V = vector<type>;
        template <class type> using B = bivector<type>;
        template <class type> using T = trivector<type>;
        template <class type> using E = dual_scalar<type>;
        template <class type> using D = dual_vector<type>;
        template <class type> using F = dual_bivector<type>;
        template <class type> using P = dual_trivector<type>;

        template <class U>
        class BD : public B<U>, public D<U>
        {            
        public:
            BD(U e23, U e31, U e12, U e14, U e24, U e34) : B<U>(e23, e31, e12), D<U>(e14, e24, e34) {}
            BD(const BD&) = default;
            bool operator<=>(const BD&) const = default;
        };

        template <class U>
        class SBD : public S<U>, public B<U>, public D<U>
        {
        public:
            SBD(U e, U e23, U e31, U e12, U e14, U e24, U e34) : S<U>(e), B<U>(e23, e31, e12), D<U>(e14, e24, e34) {}
            SBD(const SBD&) = default;
            SBD(const S<U>& s, const B<U>& b, const D<U>& d) : S<U>(s), B<U>(b), D<U>(d) {}
            bool operator<=>(const SBD&) const = default;
        };
    
        template <class U>
        class SB : public S<U>, public B<U>
        {
        public:
            SB(U e, U e23, U e31, U e12) : S<U>(e), B<U>(e23, e31, e12) {}
            SB(const S<U>& s, const B<U>& b) : S<U>(s), B<U>(b) {}
            SB(const SB&) = default;
            linalg::vec<U,3> bivector_xyz() const { return B<U>::xyz(); }
            bool operator<=>(const SB&) const = default;
        };
        
        template <class U> S<U> s_ss(const S<U>& a, const S<U>& b) { return S(a.e*b.e); } 
        template <class U> S<U> s_vv(const V<U>& a, const V<U>& b) { return S(a.e1*b.e1 + a.e2*b.e2 + a.e3*b.e3); }
        template <class U> S<U> s_bb(const B<U>& a, const B<U>& b) { return S( - a.e12 * b.e12 - a.e23 * b.e23 - a.e31 * b.e31); }
        template <class U> S<U> s_tt(const T<U>& a, const T<U>& b) { return S(-a.e123*b.e123); }        
        
        template <class U> V<U> v_sv(const S<U>& a, const V<U>& b) { return V(a.e*b.e1, a.e*b.e2, a.e*b.e3); }
        template <class U> V<U> v_vs(const V<U>& a, const S<U>& b) { return V(a.e1*b.e, a.e2*b.e, a.e3*b.e); }
        template <class U> V<U> v_vb(const V<U>& a, const B<U>& b) { return V(- a.e2 * b.e12 + a.e3 * b.e31, - a.e3 * b.e23 + a.e1 * b.e12, - a.e1 * b.e31 + a.e2 * b.e23); }
        template <class U> V<U> v_bv(const B<U>& a, const V<U>& b) { return V(- a.e31 * b.e3 + a.e12 * b.e2, - a.e12 * b.e1 + a.e23 * b.e3, - a.e23 * b.e2 + a.e31 * b.e1);}

        template <class U> B<U> b_bs(const B<U>& a, const S<U>& b) { return B(a.e23*b.e, a.e31*b.e, a.e12*b.e); }
        template <class U> B<U> b_sb(const S<U>& a, const B<U>& b) { return B(a.e*b.e23, a.e*b.e31, a.e*b.e12); }
        template <class U> B<U> b_vv(const V<U>& a, const V<U>& b) { return B(a.e2*b.e3 - a.e3*b.e2, a.e3*b.e1 - a.e1*b.e3, a.e1*b.e2 - a.e2*b.e1); }
        template <class U> B<U> b_bb(const B<U>& a, const B<U>& b) { return B(- a.e31*b.e12 + a.e12*b.e31, - a.e12*b.e23 + a.e23*b.e12, - a.e23*b.e31 + a.e31*b.e23); }

        template <class U> T<U> t_ts(const T<U>& a, const S<U>& b) { return T(a.e123*b.e); }
        template <class U> T<U> t_st(const S<U>& a, const T<U>& b) { return T(a.e*b.e123); }
        template <class U> T<U> t_vb(const V<U>& a, const B<U>& b) { return T(a.e1*b.e23 + a.e2*b.e31 + a.e3*b.e12); }
        template <class U> T<U> t_bv(const B<U>& a, const V<U>& b) { return T(a.e23*b.e1 + a.e31*b.e2 + a.e12*b.e3); }

        template <class U> V<U> sv_vs_v(const S<U>& as, const V<U>& bv, const V<U>& av, const S<U>& bs) { return sv_v(as, bv) + vs_v(av, bs); }
        template <class U> B<U> sb_bs_b(const S<U>& as, const B<U>& bb, const B<U>& ab, const S<U>& bs) { return sb_b(as, bb) + bs_b(ab, bs); }

        template <class U> S<U> operator+(const S<U>& a, const S<U>& b) { return S(a.e + b.e); }
        template <class U> S<U> operator-(const S<U>& a, const S<U>& b) { return S(a.e - b.e); }
        template <class U> V<U> operator+(const V<U>& a, const V<U>& b) { return V(a.e1 + b.e1, a.e2 + b.e2, a.e3 + b.e3); }
        template <class U> V<U> operator-(const V<U>& a, const V<U>& b) { return V(a.e1 - b.e1, a.e2 - b.e2, a.e3 - b.e3); }
        template <class U> B<U> operator+(const B<U>& a, const B<U>& b) { return B(a.e23 + b.e23, a.e31 + b.e31, a.e12 + b.e12); }
        template <class U> B<U> operator-(const B<U>& a, const B<U>& b) { return B(a.e23 - b.e23, a.e31 - b.e31, a.e12 - b.e12); }
        template <class U> T<U> operator+(const T<U>& a, const T<U>& b) { return T(a.e123 + b.e123); }
        template <class U> T<U> operator-(const T<U>& a, const T<U>& b) { return T(a.e123 - b.e123); }
        template <class U> B<U> operator/(const B<U>& a, const S<U>& b) { return B(a.e23 / b.e, a.e31 / b.e, a.e12 / b.e); }
        template <class U> B<U> operator/(const B<U>& a, double s) { return B(a.e23 / s, a.e31 / s, a.e12 / s); }
        template <class U> B<U> operator*(const B<U>& a, const S<U>& b) { return B(a.e23 * b.e, a.e31 * b.e, a.e12 * b.e); }
        template <class U> B<U> operator*(const B<U>& a, double s) { return B(a.e23 * s, a.e31 * s, a.e12 * s); }

        template <class U>
        SB<U> geommul(const SB<U>& a, const SB<U>& b) 
        {
            
            S<U> s = s_ss(a, b) + s_bb(a, b);
            B<U> bv = b_sb(a, b) + b_bs(a, b) + b_bb(a, b);
            return SB(s, bv);
        }

        /*template <class U>
        multivector geommul(const multivector& a, const multivector& b) 
        {
            scalar s =    ss_s(a.s, b.s) + vv_s(a.v, b.v) + bb_s(a.b, b.b) + tt_s(a.t, b.t); 
            vector v =    sv_vs_v(a.s, b.v, a.v, b.s) + vb_bv_v(a.v, b.b, a.b, b.v) + bt_tb_v(a.b, b.t, a.t, b.b);
            bivector b =  sb_bs_b(a.s, b.b, a.b, b.s) + sb_bs_b(a.s, b.b, a.b, b.s) + vt_tv_b(a.v, b.t, a.t, b.v) + bb_b(a.b, b.b);
            trivector t = st_ts_t(a.s, b.t, a.t, b.s) + vb_bv_t(a.v, b.b, a.b, b.v);
            return multivector(s, v, b, t);
        }*/

        template <class U>
        std::ostream& operator<<(std::ostream& os, const scalar<U>& a)
        {
            os << a.e;
            return os;
        }

        template <class U>
        std::ostream& operator<<(std::ostream& os, const vector<U>& a)
        {
            os << a.e1 << " " << a.e2 << " " << a.e3;
            return os;
        }

        template <class U>
        std::ostream& operator<<(std::ostream& os, const bivector<U>& a)
        {
            os << a.e23 << " " << a.e31 << " " << a.e12;
            return os;
        }

        template <class U>
        std::ostream& operator<<(std::ostream& os, const trivector<U>& a)
        {
            os << a.e123;
            return os;
        }

        template <class U>
        std::ostream& operator<<(std::ostream& os, const SB<U>& a)
        {
            os << (scalar<U>&)a << " " << (bivector<U>&)a;
            return os;
        }


        template <class U>
        SB<U> operator*(const SB<U>& a, const SB<U>& b)
        {
            return geommul(a, b);
        }

        template <class U>
        SB<U> exp(const bivector<U>& a)
        {
            U theta = linalg::length(a.xyz());
            scalar<U> s = cos(theta);
            bivector<U> B = sin(theta) * (a.xyz() / theta);
            return SB<U>(s, B);
        }

        template <class U>
        SBD<U> exp(const BD<U>& a) 
        {
            U theta = linalg::length(a.xyz());
            scalar<U> s = cos(theta);
            bivector<U> B = sin(theta) * (a.xyz() / theta);
            return SBD<U>(s, B, a.D);
        }

        template <class U>
        SB<U> rotor(const bivector<U>& a) 
        {
            return exp(a/2);
        }     

        template <class U>
        SBD<U> motor(const BD<U>& a) 
        {
            return exp(a/2);
        }   

        template <class U>
        bivector<U> log(const SB<U>& a)
        {
            U theta = acos(a.e);
            bivector<U> B = theta * (a.bivector_xyz() / sin(theta));
            return B; 
        }

        template <class U>
        BD<U> log(const SBD<U>& a)
        {
            U theta = acos(a.e);
            bivector<U> B = theta * (a.bivector_xyz() / sin(theta));
            return BD<U>(B, a.D);
        }

        template <class U>
        bivector<U> unrotor(const SB<U>& a)
        {
            return log(a) * 2.;
        }

        template <class U>
        BD<U> unmotor(const SBD<U>& a)
        {
            return log(a) * 2.;
        }

        template <class U>
        scalar<U> dual(const dual_scalar<U>& a)
        {
            return a.e;
        }

        template <class U>
        bivector<U> dual(const dual_bivector<U>& a)
        {
            return a.xyz();
        }

        template <class U>
        vector<U> dual(const dual_vector<U>& a)
        {
            return a.xyz();
        }

        template <class U>
        dual_scalar<U> dual(const scalar<U>& a)
        {
            return dual_scalar<U>(a.e);
        }

        template <class U>
        dual_bivector<U> dual(const bivector<U>& a)
        {
            return dual_bivector<U>(a.xyz());
        }

        template <class U>
        dual_vector<U> dual(const vector<U>& a)
        {
            return dual_vector<U>(a.xyz());
        }

        template <class U>
        trivector<U> dual(const trivector<U>& a)
        {
            return trivector<U>(a.e123);
        }

        template <class U>
        dual_trivector<U> dual(const trivector<U>& a)
        {
            return dual_trivector<U>(a.e123);
        }
    }
}

#endif