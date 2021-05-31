#ifndef HEIMER_COMMAND_CENTER_2_H
#define HEIMER_COMMAND_CENTER_2_H

#include <vector>

#include <ralgo/heimer/datanode.h>
#include <ralgo/heimer/node.h>
#include <ralgo/heimer/linear_servowf_node.h>
#include <ralgo/heimer/types.h>

#include <nos/print.h>

namespace heimer
{
	class command_center_2
	{
		std::vector<heimer::datanode> datanodes;
		std::vector<heimer::node *> nodes;		

	public:
		datanode * create_datanode(const char * name, int typehint);
	
		linear_servowf_node * create_linear_servowf_node(
		    const char * name,
		    ralgo::matrix_view_co<real> & co,
		    const igris::array_view<const char *> & lsigs,
		    const igris::array_view<const char *> & rsigs	
		);

		datanode * find_datanode(const char * name);
		node *     find_node    (const char * name);

		~command_center_2();
	};
}

#endif