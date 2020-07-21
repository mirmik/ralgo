#include <ralgo/heimer/axisctr.h>
#include <ralgo/heimer/phaser_axis.h>

#include <ralgo/heimer/tandem.h>
#include <ralgo/heimer/xyalpha.h>

void loop();

heimer::phaser_emulator<float, float> phs0;
heimer::phaser_emulator<float, float> phs1;
heimer::phaser_emulator<float, float> phs2;
heimer::phaser_emulator<float, float> phs3;
heimer::phaser_emulator<float, float> phs4;

heimer::phaser_axis<float, float, float> ax0("ax0", &phs0);
heimer::phaser_axis<float, float, float> ax1("ax1", &phs1);

heimer::phaser_axis<float, float, float> ax2("ax2", &phs2);
heimer::phaser_axis<float, float, float> ax3("ax3", &phs3);
heimer::phaser_axis<float, float, float> ax4("ax4", &phs4);

heimer::tandem<float, float, 2> tand(
    "tand", "tand.x", "tand.y",
    &ax0, &ax1, 0.5);

heimer::xyalpha_chain2d_controller<float, float> xya(
	"xya", "xya.x", "xya.y", "xya.a",
	&ax2, &ax3, &ax4
);

heimer::axisctr<float, float> axctr0("axctr0", &ax0);
heimer::axisctr<float, float> axctr1("axctr1", &xya.x_axis);
heimer::axisctr<float, float> axctr2("axctr2", &xya.a_axis);

void _exit(void*) 
{
	exit(0);
}

void init_xyalpha()
{
	xya.x_link.relocate({0, {0, 0}});
	xya.y_link.relocate({0, {0, 0}});
	xya.a_link.relocate({0, {0, 0}});
	xya.output_link.relocate({0, { -58.5, 0}});

	xya.nullpos = ralgo::htrans2<float>(0, { -58.5, 0});
	xya.invnullpos = xya.nullpos.inverse();
}

int main(int argc, char* argv[])
{
	init_xyalpha();
	axctr1.operation_finish_signal = _exit;
	axctr1.incmove(10);
	axctr2.incmove(10);
	while (1) loop();
}

void loop()
{
	ax0.feedback();
	ax1.feedback();
	ax2.feedback();
	ax3.feedback();
	ax4.feedback();

	tand.feedback();
	xya.feedback();

	axctr1.serve();
	axctr2.serve();

	xya.serve();
	ax2.serve();
	ax3.serve();
	ax4.serve();

	phs2.serve();
	phs3.serve();
	phs4.serve();

//	PRINT();
	nos::println(phs2.target_position(), phs3.target_position(), phs4.target_position());
//	DPRINT(axctr0.feedback_position());
//	DPRINT(axctr0.target_position());
}