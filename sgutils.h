#ifndef SGUTILS_H
#define SGUTILS_H

#include <vector>

#include "scenegraph.h"

struct RbtNodesScanner : public SgNodeVisitor {
    typedef std::vector<SgRbtNode*> SgRbtNodes;

    SgRbtNodes &m_nodes;

    explicit RbtNodesScanner(SgRbtNodes &nodes) : m_nodes(nodes) {}

    bool visit(SgTransformNode &node) override {
        using namespace std;
        auto rbtPtr = dynamic_cast<SgRbtNode*>(&node);
        if (rbtPtr)
            m_nodes.push_back(rbtPtr);
        return true;
    }
};

[[nodiscard]] inline std::vector<SgRbtNode *> dumpSgRbtNodes(std::shared_ptr<SgNode> root) {
    std::vector<SgRbtNode *> rbt_nodes;
    RbtNodesScanner scanner(rbt_nodes);
    root->accept(scanner);
    return rbt_nodes;
}


#endif