#include <algorithm>

#include "scenegraph.h"


bool SgTransformNode::accept(SgNodeVisitor &visitor) {
    if (!visitor.visit(*this))
        return false;
    for (int i = 0, n = children_.size(); i < n; ++i) {
        if (!children_[i]->accept(visitor))
            return false;
    }
    return visitor.postVisit(*this);
}

void SgTransformNode::addChild(std::shared_ptr<SgNode> child) {
    children_.push_back(child);
}

void SgTransformNode::removeChild(std::shared_ptr<SgNode> child) {
    children_.erase(find(children_.begin(), children_.end(), child));
}

bool SgShapeNode::accept(SgNodeVisitor &visitor) {
    if (!visitor.visit(*this))
        return false;
    return visitor.postVisit(*this);
}

class RbtAccumVisitor : public SgNodeVisitor {
protected:
    std::vector<RigTForm> m_rbt_stack{1};
    SgTransformNode &m_target;
    bool m_found;
public:
    explicit RbtAccumVisitor(SgTransformNode &target)
            : m_target(target), m_found(false) {}

    RigTForm getAccumulatedRbt(int offsetFromStackTop = 0) {
        return m_rbt_stack.at(m_rbt_stack.size()-offsetFromStackTop-1);
    }

    bool visit(SgTransformNode &node) override {
        m_rbt_stack.push_back(m_rbt_stack.back()*node.getRbt());
        if (node == m_target){
            m_found = true;
            return false;
        }
        return true;
    }

    bool postVisit(SgTransformNode &node) override {
        m_rbt_stack.pop_back();
        return true;
    }
};

RigTForm getPathAccumRbt(
        SgTransformNode *source,
        SgTransformNode *destination,
        int offsetFromDestination) {

    RbtAccumVisitor accum(*destination);
    source->accept(accum);
    return accum.getAccumulatedRbt(offsetFromDestination);
}
