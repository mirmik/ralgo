#include <ralgo/heimer/axisctr.h>
#include <ralgo/heimer/phaser_axis.h>

#include <ralgo/heimer/tandem.h>
#include <ralgo/heimer/xyalpha.h>

void loop();

heimer::phaser_emulator<float, int64_t, float> phs0;
heimer::phaser_emulator<float, int64_t, float> phs1;
heimer::phaser_emulator<float, int64_t, float> phs2;
heimer::phaser_emulator<float, int64_t, float> phs3;
heimer::phaser_emulator<float, int64_t, float> phs4;

heimer::phaser_axis<float, int64_t, float> ax0("ax0", &phs0);
heimer::phaser_axis<float, int64_t, float> ax1("ax1", &phs1);

heimer::phaser_axis<float, int64_t, float> ax2("ax2", &phs2);
heimer::phaser_axis<float, int64_t, float> ax3("ax3", &phs3);
heimer::phaser_axis<float, int64_t, float> ax4("ax4", &phs4);

heimer::tandem<float, float, 2> tand(
    "tand", "tand.x", "tand.y",
    &ax0, &ax1, 0.5);

heimer::axisctr<float, float> axctr0("axctr0", &ax0);

int main(int argc, char* argv[])
{
	axctr0.incmove(10);
	while (1) loop();
}

void loop()
{

}