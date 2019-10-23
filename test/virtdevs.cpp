#include <gtest/gtest.h>
#include <ralgo/virtdevs/device.h>

class test_device : public virtdevs::device
{
	std::vector<virtdevs::device*> devs;

public:
	test_device(const char* name) : virtdevs::device(name)
	{}

	igris::array_view<device*> dependence() override
	{ 
		return { devs.data(), devs.size() }; 
	}

	void add_dep(device& dev) { devs.push_back(&dev); }
};

TEST(virtdevs, ctr)
{
	test_device a("a");
	test_device b("b");
	test_device c("c");

	a.add_dep(b);
	a.add_dep(c);

//	a.vd_begin();
//	b.vd_begin();
//	c.vd_begin();

	b.allgood();
}
