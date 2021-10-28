/** @file */

#include <ralgo/heimer/command_center_2.h>

heimer::datanode *heimer::command_center_2::create_datanode(const char *name,
                                                            int typehint)
{
    datanodes.emplace_back(name, typehint);

    if (datanodes.back().typehint() == DATANODE_TYPEHINT_UNDEFINED)
    {
        datanodes.pop_back();
        return nullptr;
    }

    return &datanodes.back();
}

heimer::linear_servowf_node *
heimer::command_center_2::create_linear_servowf_node(
    const char *name, ralgo::matrix_view_co<real> &co,
    const igris::array_view<const char *> &lsigs,
    const igris::array_view<const char *> &rsigs)
{
    int sts;

    heimer::datanode_ptr lnodes[SERVO_WISHFEED_MAXDIM];
    heimer::datanode_ptr rnodes[SERVO_WISHFEED_MAXDIM];

    for (int i = 0; i < lsigs.size(); ++i)
        lnodes[i] = find_datanode(lsigs[i]);

    for (int i = 0; i < rsigs.size(); ++i)
        rnodes[i] = find_datanode(rsigs[i]);

    igris::array_view<heimer::datanode_ptr> lnodes_view(lnodes, lsigs.size());
    igris::array_view<heimer::datanode_ptr> rnodes_view(rnodes, rsigs.size());

    auto *node = new linear_servowf_node;
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
    for (auto *n : nodes)
    {
        delete n;
    }
}

heimer::datanode_ptr heimer::command_center_2::find_datanode(const char *name)
{
    for (auto &dn : datanodes)
    {
        if (dn.compare_name(name) == 0)
            return &dn;
    }
    return nullptr;
}

heimer::errcode heimer::command_center_2::remove_datanode(const char *name)
{
    auto beg = datanodes.begin();
    auto end = datanodes.end();
    auto it = std::find_if(beg, end, [&name](const datanode &ref) {
        return ref.compare_name(name) == 0;
    });

    if (datanode_is_used(&*it))
    {
        return errcode::DATANODE_IS_USED_ERROR;
    }

    if (it != end)
    {
        datanodes.erase(it);
        return errcode::OK;
    }
    else
        return errcode::DATANODE_NOT_FOUND_ERROR;
}

bool heimer::command_center_2::datanode_is_used(datanode *ptr)
{
    return ptr->is_used();
}
