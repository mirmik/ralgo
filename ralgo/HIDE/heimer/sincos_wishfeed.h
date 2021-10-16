/** @file */

#ifndef HEIMER_SINCOS_WISHFEED_H
#define HEIMER_SINCOS_WISHFEED_H

/*
	x1 = x0 + R * cos(alpha) 
	y1 = y0 + R * sin(alpha) 
*/


namespace heimer 
{
	class sincos_wfnode
	{
		double radius;
		
	public:

		void serve_feed()
		{
			process(
			    left_signals(),
			    right_signals(),
			    &wishfeed::feed,
			    lr_transform_buffer);
		}

		void serve_wish()
		{
			process(
			    right_signals(),
			    left_signals(),
			    &wishfeed::wish,
			    rl_transform_buffer);
		}
	}
}

#endif
