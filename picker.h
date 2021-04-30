#ifndef PICKER_H
#define PICKER_H

#include <vector>
#include <map>
#include <memory>
#include <stdexcept>

#include "cvec.h"
#include "scenegraph.h"
#include "asstcommon.h"
#include "ppm.h"
#include "drawer.h"

class Picker : public SgNodeVisitor {
    std::vector<std::reference_wrapper<SgRbtNode>> m_node_stack;
    std::vector<SgRbtNode*> m_id_to_rbt_node{nullptr};

    bool srgbFrameBuffer_;

    Drawer drawer_;

    Cvec3 idToColor(int id);

    int colorToId(const PackedPixel &p);

public:
    Picker(const RigTForm &initialRbt, const ShaderState &curSS);

    bool visit(SgTransformNode &node) override;

    bool postVisit(SgTransformNode &node) override;

    bool visit(SgShapeNode &node) override;

    bool postVisit(SgShapeNode &node) override;

    SgRbtNode * getRbtNodeAtXY(int x, int y);
};


#endif
