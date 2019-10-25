#include <gtest/gtest.h>
#include <ralgo/virtdevs/device.h>

#include <nos/print.h>
#include <nos/print/stdtype.h>

class test_device : public virtdevs::device
{
	std::vector<virtdevs::device*> devs;

public:
	test_device(const char* name, bool virtdev=false) : virtdevs::device(name)
	{
		if (virtdev)
			allgood();
	}

	igris::array_view<virtdevs::device*> dependence() override
	{ 
		return { devs.data(), devs.size() }; 
	}

	void add_dep(device& dev) { devs.push_back(&dev); }

	int ready_counter = 0;
	void ready_handle() 
	{
		ready_counter++;
	}
};

TEST(virtdevs, ctr)
{
	int sts;

	test_device a("a", true);
	test_device b("b");
	test_device c("c");

	a.add_dep(b);
	a.add_dep(c);

	EXPECT_EQ(a.dependence().size(), 2);
	EXPECT_EQ(b.dependence().size(), 0);
	EXPECT_EQ(c.dependence().size(), 0);

	sts = a.activate_device();
	EXPECT_EQ(sts, -1);

	sts = b.activate_device();
	EXPECT_EQ(sts, 0);

	sts = c.activate_device();
	EXPECT_EQ(sts, 0);

	c.allgood();
	EXPECT_EQ(b.is_ready(), 0);
	EXPECT_EQ(c.is_ready(), 1);
	
	sts = a.activate_device();
	EXPECT_EQ(sts, 0);
	EXPECT_EQ(a.get_ok_deps_counter(), 1);
	
	EXPECT_EQ(a.is_alarmed(), true);
	EXPECT_EQ(b.is_alarmed(), true);
	EXPECT_EQ(c.is_alarmed(), false);

	b.allgood();
	EXPECT_EQ(a.get_ok_deps_counter(), 2);

	EXPECT_EQ(a.is_alarmed(), false);
	EXPECT_EQ(b.is_alarmed(), false);
	EXPECT_EQ(c.is_alarmed(), false);

	EXPECT_EQ(a.is_ready(), true);
	EXPECT_EQ(b.is_ready(), true);
	EXPECT_EQ(c.is_ready(), true);

	b.alarm({25,1});

	EXPECT_EQ(a.is_ready(), false);
	EXPECT_EQ(b.is_ready(), false);
	EXPECT_EQ(c.is_ready(), true);

	EXPECT_EQ(a.is_alarmed(), true);
	EXPECT_EQ(b.is_alarmed(), true);
	EXPECT_EQ(c.is_alarmed(), false);

	EXPECT_EQ(a.get_ok_deps_counter(), 1);

	b.allgood();

	EXPECT_EQ(a.is_ready(), true);
	EXPECT_EQ(b.is_ready(), true);
	EXPECT_EQ(c.is_ready(), true);

	nos::println(a.is_alarmed(), a.alarm_message());
	nos::println(b.is_alarmed(), b.alarm_message());
	nos::println(c.is_alarmed(), c.alarm_message());
}

TEST(virtdevs, handle)
{
	test_device a("a", true);
	test_device b("b");
	test_device c("c");

	a.add_dep(b);
	a.add_dep(c);

	b.activate_device();
	c.activate_device();
	a.activate_device();

	EXPECT_EQ(b.ready_counter, 0);
	b.allgood();
	EXPECT_EQ(b.ready_counter, 1);
}