#include <ralgo/linalg/vecops.h>

using namespace ralgo;

LT_BEGIN_TEST(ralgo_test_suite, vecops)
{
	LT_CHECK(vecops::all(bvec{true,true,true,false}) == false);	
	LT_CHECK(vecops::all(bvec{true,true,true,true}) == true);	

	LT_CHECK(vecops::any(bvec{true,false,false,false}) == true);		
	LT_CHECK(vecops::any(bvec{false,false,false,false}) == false);	

	LT_CHECK(vecops::equal_all(dvec{1,2,3},dvec{1,2,3}));
	LT_CHECK(vecops::equal_all(dvec{1,2,3},dvec{1,3,4})==false);		
	LT_CHECK(vecops::equal_any(dvec{1,2,3},dvec{1,3,4}));		
	LT_CHECK(vecops::equal_any(dvec{1,2,3},dvec{2,3,4})==false);	

	LT_CHECK(vecops::equal_all(vecops::add_vv(dvec{1,2,3},dvec{2,3,4}), dvec{3,5,7})==true);
	LT_CHECK(vecops::equal_all(vecops::sub_vv(dvec{1,2,3},dvec{2,3,5}), dvec{-1,-1,-2})==true);
}
LT_END_TEST(vecops)



