#ifndef DRAWER_H
#define DRAWER_H

#include <vector>

#include "scenegraph.h"
#include "asstcommon.h"

class Drawer : public SgNodeVisitor {
protected:
    std::vector<RigTForm> rbtStack_;
    const ShaderState &curSS_;
public:
    Drawer(const RigTForm &initialRbt, const ShaderState &curSS)
            : rbtStack_(1, initialRbt), curSS_(curSS) {}

    bool visit(SgTransformNode &node) override {
        rbtStack_.push_back(rbtStack_.back() * node.getRbt());
        return true;
    }

    bool postVisit(SgTransformNode &node) override {
        rbtStack_.pop_back();
        return true;
    }

    bool visit(SgShapeNode &shapeNode) override {
        const Matrix4 MVM = rigTFormToMatrix(rbtStack_.back()) * shapeNode.getAffineMatrix();
        sendModelViewNormalMatrix(curSS_, MVM, normalMatrix(MVM));
        shapeNode.draw(curSS_);
        return true;
    }

    bool postVisit(SgShapeNode &shapeNode) override {
        return true;
    }

    [[nodiscard]] const ShaderState &getCurSS() const {
        return curSS_;
    }
};

#endif



