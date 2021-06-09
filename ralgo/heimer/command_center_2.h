#ifndef HEIMER_COMMAND_CENTER_2_H
#define HEIMER_COMMAND_CENTER_2_H

/**
	@file
*/

#include <vector>

#include <ralgo/heimer/datanode.h>
#include <ralgo/heimer/node.h>
#include <ralgo/heimer/linear_servowf_node.h>
#include <ralgo/heimer/types.h>
#include <ralgo/heimer/errcode.h>

#include <nos/print.h>

namespace heimer
{
	class command_center_2
	{
		std::list<heimer::datanode> datanodes;
		std::vector<heimer::node *> nodes;		

	public:
		datanode * create_datanode(const char * name, int typehint);
		node * create_node(const char * name, int typehint) { return nullptr; }

		linear_servowf_node * create_linear_servowf_node(
		    const char * name,
		    ralgo::matrix_view_co<real> & co,
		    const igris::array_view<const char *> & lsigs,
		    const igris::array_view<const char *> & rsigs	
		);

		bool datanode_is_used(datanode * dn);

		datanode_ptr find_datanode(const char * name);
		node *     find_node    (const char * name);

		heimer::errcode remove_datanode(const char * name);
		heimer::errcode remove_node(const char * name);

		~command_center_2();

		const char * error_to_string(heimer::errcode errcode);
	};
}

#endif