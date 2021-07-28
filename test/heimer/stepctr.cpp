#include <doctest/doctest.h>
#include <ralgo/heimer/stepctr.h>

using namespace heimer;

static int setted_state = 0; 

void set_state(void * priv, uint8_t state) 
{
	struct stepctr_controller * stepctr = (struct stepctr_controller *) priv;
	setted_state = state;
}

TEST_CASE("stepctr")
{
	stepctr_controller stepctr;

	stepctr.init(set_state, nullptr, 2000, 0.7);
	CHECK_EQ(setted_state, 0);

	stepctr.shift(1250);
	CHECK_EQ(setted_state, 0);

	stepctr.shift(1250);
	CHECK_EQ(setted_state, 1);

	stepctr.shift(-1250);
	CHECK_EQ(setted_state, 1);

	stepctr.shift(-1250);
	CHECK_EQ(setted_state, 0);
}