#include <GL/glew.h>

#include "picker.h"


Picker::Picker(const RigTForm &initialRbt, const ShaderState &curSS)
        : drawer_(initialRbt, curSS), srgbFrameBuffer_(!g_Gl2Compatible) {}

bool Picker::visit(SgTransformNode &node) {
    if (auto p = dynamic_cast<SgRbtNode *>(&node)) {
        m_node_stack.emplace_back(*p);
    }
    return drawer_.visit(node);
}

bool Picker::postVisit(SgTransformNode &node) {
    m_node_stack.pop_back();
    return drawer_.postVisit(node);
}

bool Picker::visit(SgShapeNode &node) {
    auto id = static_cast<int>(m_id_to_rbt_node.size());
    m_id_to_rbt_node.push_back(&m_node_stack.back().get());
    auto color = idToColor(id);
    safe_glUniform3f(drawer_.getCurSS().h_uIdColor, color[0], color[1], color[2]);
    return drawer_.visit(node);
}

bool Picker::postVisit(SgShapeNode &node) {
    return drawer_.postVisit(node);
}

SgRbtNode *Picker::getRbtNodeAtXY(int x, int y) {
    auto image = std::array<unsigned char, 3>{};
    glReadPixels(0, 0, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, &image[0]);
    auto id = colorToId(PackedPixel{image[0], image[1], image[2]});

    return m_id_to_rbt_node.at(id); // return null for now
}

//------------------
// Helper functions
//------------------
//

// encode 2^4 = 16 IDs in each of R, G, B channel, for a total of 16^3 number of objects
static const int NBITS = 4, N = 1 << NBITS, MASK = N - 1;

Cvec3 Picker::idToColor(int id) {
    assert(id > 0 && id < N * N * N);
    Cvec3 framebufferColor = Cvec3(id & MASK, (id >> NBITS) & MASK, (id >> (NBITS + NBITS)) & MASK);
    framebufferColor = framebufferColor / N + Cvec3(0.5 / N);

    if (!srgbFrameBuffer_)
        return framebufferColor;
    else {
        // if GL3 is used, the framebuffer will be in SRGB format, and the color we supply needs to be in linear space
        Cvec3 linearColor;
        for (int i = 0; i < 3; ++i) {
            linearColor[i] = framebufferColor[i] <= 0.04045 ? framebufferColor[i] / 12.92 : pow(
                    (framebufferColor[i] + 0.055) / 1.055, 2.4);
        }
        return linearColor;
    }
}

int Picker::colorToId(const PackedPixel &p) {
    const int UNUSED_BITS = 8 - NBITS;
    int id = p.r >> UNUSED_BITS;
    id |= ((p.g >> UNUSED_BITS) << NBITS);
    id |= ((p.b >> UNUSED_BITS) << (NBITS + NBITS));
    return id;
}
