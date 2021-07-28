#include <ralgo/linalg/vecops.h>
#include <ralgo/util/gen.h>
#include <nos/print.h>

#include <main.h>

using namespace ralgo;

LT_BEGIN_TEST(ralgo_test_suite, vecops)
{
	CHECK(vecops::all(bvec{true,true,true,false}) == false);	
	CHECK(vecops::all(bvec{true,true,true,true}) == true);	

	CHECK(vecops::any(bvec{true,false,false,false}) == true);		
	CHECK(vecops::any(bvec{false,false,false,false}) == false);	

	CHECK(vecops::equal_all(dvec{1,2,3},dvec{1,2,3}));
	CHECK(vecops::equal_all(dvec{1,2,3},dvec{1,3,4})==false);		
	CHECK(vecops::equal_any(dvec{1,2,3},dvec{1,3,4}));		
	CHECK(vecops::equal_any(dvec{1,2,3},dvec{2,3,4})==false);	

	CHECK(vecops::equal_all(vecops::add_vv(dvec{1,2,3},dvec{2,3,4}), dvec{3,5,7})==true);
	CHECK(vecops::equal_all(vecops::sub_vv(dvec{1,2,3},dvec{2,3,5}), dvec{-1,-1,-2})==true);

	CHECK(vecops::equal_all(ralgo::gen::range(4),ralgo::vecops::arange(4)));		
	CHECK(vecops::equal_all(ralgo::vecops::list(ralgo::gen::range(4)),ralgo::vecops::arange(4)));		
}
LT_END_TEST(vecops)


LT_BEGIN_TEST(ralgo_test_suite, range_gen)
{
	auto rng0 = ralgo::gen::range(1,7,3);
	auto rng1 = ralgo::gen::range(1,7);
	auto rng2 = ralgo::gen::range(7);

	CHECK(rng0.size() == 2);
	CHECK(rng1.size() == 6);
	CHECK(rng2.size() == 7);

	CHECK(ralgo::vecops::equal_all(rng0, std::vector<int>{1,4}));
	CHECK(ralgo::vecops::equal_all(rng1, std::vector<int>{1,2,3,4,5,6}));
	CHECK(ralgo::vecops::equal_all(rng2, std::vector<int>{0,1,2,3,4,5,6}));
}
LT_END_TEST(range_gen)


