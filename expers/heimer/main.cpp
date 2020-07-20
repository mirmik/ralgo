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

heimer::axisctr<float, float> axctr0("axctr0", &ax0);

void _exit(void*) 
{
	exit(0);
}

int main(int argc, char* argv[])
{
	axctr0.operation_finish_signal = _exit;
	axctr0.incmove(10);
	while (1) loop();
}

void loop()
{
	ax0.feedback();

	axctr0.serve();
	ax0.serve();
	phs0.serve();

	DPRINT(phs0.feedback_position());
	DPRINT(phs0.target_position());
	DPRINT(axctr0.feedback_position());
	DPRINT(axctr0.target_position());
}