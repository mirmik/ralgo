#include <ralgo/heimer/command_center_2.h>

heimer::datanode * heimer::command_center_2::create_datanode(
	const char * name, 
	int typehint
) 
{
	datanodes.emplace_back(name, typehint);

	if (datanodes.back().typehint() == DATANODE_TYPEHINT_UNDEFINED) 
	{
		datanodes.pop_back();
		return nullptr;
	}

	return & datanodes.back();
}

heimer::linear_servowf_node *  heimer::command_center_2::create_linear_servowf_node(
	const char * name, 
	ralgo::matrix_view_co<real> & co,
	const igris::array_view<const char *> & lsigs,
	const igris::array_view<const char *> & rsigs	
) 
{
	int sts;

	heimer::datanode * lnodes[SERVO_WISHFEED_MAXDIM];
	heimer::datanode * rnodes[SERVO_WISHFEED_MAXDIM];

	for (int i = 0; i < lsigs.size(); ++i) 
		lnodes[i] = find_datanode(lsigs[i]);

	for (int i = 0; i < rsigs.size(); ++i) 
		rnodes[i] = find_datanode(rsigs[i]);

	igris::array_view<heimer::datanode *> lnodes_view(lnodes, lsigs.size());
	igris::array_view<heimer::datanode *> rnodes_view(rnodes, rsigs.size());

	auto * node = new linear_servowf_node;
	sts = node->init(name, co, lnodes_view, rnodes_view);
	
	if (sts) 
	{
		delete node;
		return nullptr;
	}

	nodes.push_back(node);

	return node;
}


heimer::command_center_2::~command_center_2() 
{
	for (auto * n : nodes) 
	{
		delete n;
	}
}

heimer::datanode * heimer::command_center_2::find_datanode(const char * name) 
{
	for (auto & dn : datanodes) 
	{
		if (dn.compare_name(name) == 0)
			return &dn;
	}
	return nullptr;
}