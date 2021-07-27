#include <doctest/doctest.h>
#include <ralgo/heimer/stepctr.h>

static int setted_state = 0; 

void set_state(void * priv, uint8_t state) 
{
	struct stepctr_controller * stepctr = (struct stepctr_controller *) priv;
	setted_state = state;
}

TEST_CASE("stepctr")
{
	struct stepctr_controller stepctr;

	stepctr_controller_init(&stepctr, set_state, nullptr, 2000, 0.7);
	CHECK_EQ(setted_state, 0);

	stepctr_controller_shift(&stepctr, 1250);
	CHECK_EQ(setted_state, 0);

	stepctr_controller_shift(&stepctr, 1250);
	CHECK_EQ(setted_state, 1);

	stepctr_controller_shift(&stepctr, -1250);
	CHECK_EQ(setted_state, 1);

	stepctr_controller_shift(&stepctr, -1250);
	CHECK_EQ(setted_state, 0);
}