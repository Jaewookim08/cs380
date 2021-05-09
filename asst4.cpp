////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////

#include <memory>
#include <vector>
#include <string>
#include <memory>
#include <stdexcept>

#include <GL/glew.h>

#ifdef __APPLE__
#   include <GLUT/glut.h>
#else

#   include <GL/glut.h>
#include <list>
#include <fstream>

#endif

#include "cvec.h"
#include "matrix4.h"
#include "geometrymaker.h"
#include "ppm.h"
#include "glsupport.h"
#include "rigtform.h"
#include "arcball.h"

#include "asstcommon.h"
#include "scenegraph.h"
#include "drawer.h"
#include "picker.h"
#include "sgutils.h"


// G L O B A L S ///////////////////////////////////////////////////

// --------- IMPORTANT --------------------------------------------------------
// Before you start working on this assignment, set the following variable
// properly to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or
// OpenGL 3.x+ with GLSL 1.3.
//
// Set g_Gl2Compatible = true to use GLSL 1.0 and g_Gl2Compatible = false to
// use GLSL 1.3. Make sure that your machine supports the version of GLSL you
// are using. In particular, on Mac OS X currently there is no way of using
// OpenGL 3.x with GLSL 1.3 when GLUT is used.
//
// If g_Gl2Compatible=true, shaders with -gl2 suffix will be loaded.
// If g_Gl2Compatible=false, shaders with -gl3 suffix will be loaded.
// To complete the assignment you only need to edit the shader files that get
// loaded
// ----------------------------------------------------------------------------

const bool g_Gl2Compatible = false;

static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;    // near plane
static const float g_frustFar = -50.0;    // far plane
static const float g_groundY = -2.0;      // y coordinate of the ground
static const float g_groundSize = 10.0;   // half the ground length

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static bool g_mouseClickDown = false;    // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
static int g_activeShader = 0;

static const int PICKING_SHADER = 2; // index of the picking shader is g_shaerFiles
static const int g_numShaders = 3; // 3 shaders instead of 2
static const char* const g_shaderFiles[g_numShaders][2] = {{"./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader"},
                                                           {"./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader"},
                                                           {"./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"}};
static const char* const g_shaderFilesGl2[g_numShaders][2] = {{"./shaders/basic-gl2.vshader", "./shaders/diffuse-gl2.fshader"},
                                                              {"./shaders/basic-gl2.vshader", "./shaders/solid-gl2.fshader"},
                                                              {"./shaders/basic-gl2.vshader", "./shaders/pick-gl2.fshader"}};
static std::vector<std::shared_ptr<ShaderState>> g_shaderStates; // our global shader states

// --------- Geometry

// Macro used to obtain relative offset of a field within a struct
#define FIELD_OFFSET(StructType, field) &(((StructType *)0)->field)

// A vertex with floating point position and normal
struct VertexPN {
    Cvec3f p, n;

    VertexPN() {}

    VertexPN(float x, float y, float z, float nx, float ny, float nz) : p(x, y, z), n(nx, ny, nz) {}

    // Define copy constructor and assignment operator from GenericVertex so we can
    // use make* functions from geometrymaker.h
    VertexPN(const GenericVertex& v) {
        *this = v;
    }

    VertexPN& operator=(const GenericVertex& v) {
        p = v.pos;
        n = v.normal;
        return *this;
    }
};

struct Geometry {
    GlBufferObject vbo, ibo;
    int vboLen, iboLen;

    Geometry(VertexPN* vtx, unsigned short* idx, int vboLen, int iboLen) {
        this->vboLen = vboLen;
        this->iboLen = iboLen;

        // Now create the VBO and IBO
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPN) * vboLen, vtx, GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short) * iboLen, idx, GL_STATIC_DRAW);
    }

    void draw(const ShaderState& curSS) {
        // Enable the attributes used by our shader
        safe_glEnableVertexAttribArray(curSS.h_aPosition);
        safe_glEnableVertexAttribArray(curSS.h_aNormal);

        // bind vbo
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        safe_glVertexAttribPointer(curSS.h_aPosition, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN),
                                   FIELD_OFFSET(VertexPN, p));
        safe_glVertexAttribPointer(curSS.h_aNormal, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, n));

        // bind ibo
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);

        // draw!
        glDrawElements(GL_TRIANGLES, iboLen, GL_UNSIGNED_SHORT, 0);

        // Disable the attributes used by our shader
        safe_glDisableVertexAttribArray(curSS.h_aPosition);
        safe_glDisableVertexAttribArray(curSS.h_aNormal);
    }
};

namespace asd {
    struct frame {
        std::vector<RigTForm> rbt_states;
    };
    using animation = std::list<frame>;
}

static asd::animation animation;
static decltype(animation)::iterator current_frame_iter;

static int ms_between_keyframes = 2000;
static int animated_frames_per_sec = 60;

static bool show_animation_at_time(float t);

static void animate_timer_callback(int ms);
static void load_frame(asd::frame&);

static void load_current_frame_if_defined() {
    if (::current_frame_iter != animation.end()) {
        ::load_frame(*::current_frame_iter);
        std::cout << "loaded frame" << std::endl;
    }
}

[[nodiscard]] static asd::frame save_frame();

static void save_animation_to_file(const std::string& filename) {
    auto file = std::ofstream{filename};
    auto frame_count = animation.size();
    auto rbt_count = animation.empty() ? 0 : animation.front().rbt_states.size();
    file << frame_count << " " << rbt_count << "\n";

    for (auto&& saving_frame: animation) {
        for (auto&& rbt: saving_frame.rbt_states) {
            auto t = rbt.getTranslation();
            for (int i = 0; i < 3; i++)
                file << t[i] << " ";
            file << "\n";
            auto r = rbt.getRotation();
            for (int i = 0; i < 4; i++)
                file << r[i] << " ";
            file << "\n";
        }
    }
    file.close();
    std::cout << "saved animation to " << filename << std::endl;
}

static void load_animation_from_file(const std::string& filename) {
    auto file = std::ifstream{filename};
    if (!file) {
        std::cerr << "Failed to open file" << std::endl;
        return;
    }

    decltype(animation.size()) frame_count;
    file >> frame_count;
    decltype(asd::frame::rbt_states.size()) rbt_count;
    file >> rbt_count;

    asd::animation new_animation;
    for (auto frame_index = 0; frame_index < frame_count; frame_index++) {
        asd::frame new_frame;
        for (auto rbt_index = 0; rbt_index < rbt_count; rbt_index++) {
            RigTForm rbt;

            Cvec3 t;
            for (int i = 0; i < 3; i++)
                file >> t[i];
            rbt.setTranslation(t);

            Quat r;
            for (int i = 0; i < 4; i++)
                file >> r[i];
            rbt.setRotation(r);

            new_frame.rbt_states.push_back(rbt);
        }
        new_animation.push_back(new_frame);
    }

    std::cout << "loaded animation from" << filename << std::endl;

    ::animation = new_animation;
    ::current_frame_iter = animation.begin();
    ::load_current_frame_if_defined();
}


using MyShapeNode = SgGeometryShapeNode<Geometry>;

static bool waiting_pick = false;
static std::shared_ptr<SgRootNode> g_world;
static std::shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node;
static SgRbtNode* g_currentPickedRbtNode; // used later when you do picking
static SgRbtNode* g_eye_node;

// Vertex buffer and index buffer associated with the ground and cube geometry
static std::shared_ptr<Geometry> g_ground, g_cube, g_arcball;

// --------- Scene

static const Cvec3 g_light1(2.0, 3.0, 14.0), g_light2(-2, -3.0, -5.0);  // define two lights positions in world space

static std::optional<RigTForm> g_arcballRbt;

static int camera_index = 0;


namespace asd {
    enum class object_enum {
        sky_camera, COUNT
    };
    struct manipulation_setting {
        bool can_manipulate = false;
        RigTForm respect_frame;
    };
}

static bool do_skysky = false;


static double g_arcballScreenRadius = 1.f;
static double g_arcballScale = 0.001f;
///////////////// END OF G L O B A L S //////////////////////////////////////////////////


SgRbtNode* current_manipulating() {
    if (g_currentPickedRbtNode == nullptr)
        return g_skyNode.get();
    return g_currentPickedRbtNode;
}

asd::manipulation_setting get_manipulation_setting() {
    if (g_currentPickedRbtNode == nullptr) {
        if (g_eye_node != g_skyNode.get())
            return asd::manipulation_setting{false, RigTForm{}};
        auto sky_rbt_ref_world = getPathAccumRbt(g_world.get(), g_skyNode.get());
        return asd::manipulation_setting{true, do_skysky ? sky_rbt_ref_world : linFact(sky_rbt_ref_world)};
    }
    else {
        return asd::manipulation_setting{true, transFact(getPathAccumRbt(g_world.get(), ::current_manipulating())) *
                                               linFact(getPathAccumRbt(g_world.get(), ::g_eye_node))};
    }
}


static void initGround() {
    // A x-z plane at y = g_groundY of dimension [-g_groundSize, g_groundSize]^2
    VertexPN vtx[4] = {VertexPN(-g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
                       VertexPN(-g_groundSize, g_groundY, g_groundSize, 0, 1, 0),
                       VertexPN(g_groundSize, g_groundY, g_groundSize, 0, 1, 0),
                       VertexPN(g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),};
    unsigned short idx[] = {0, 1, 2, 0, 2, 3};
    g_ground.reset(new Geometry(&vtx[0], &idx[0], 4, 6));
}

static void initCubes() {
    int ibLen, vbLen;
    getCubeVbIbLen(vbLen, ibLen);

    // Temporary storage for cube geometry
    std::vector<VertexPN> vtx(vbLen);
    std::vector<unsigned short> idx(ibLen);

    makeCube(1, vtx.begin(), idx.begin());
    g_cube.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initArcball() {
    std::vector<VertexPN> vtx;
    std::vector<unsigned short> idx;
    makeSphere(1, 15, 15, std::back_inserter(vtx), std::back_inserter(idx));
    g_arcball = std::make_shared<Geometry>(&vtx[0], &idx[0], vtx.size(), idx.size());
}

// takes a projection matrix and send to the the shaders
static void sendProjectionMatrix(const ShaderState& curSS, const Matrix4& projMatrix) {
    GLfloat glmatrix[16];
    projMatrix.writeToColumnMajorMatrix(glmatrix); // send projection matrix
    safe_glUniformMatrix4fv(curSS.h_uProjMatrix, glmatrix);
}

// update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
static void updateFrustFovY() {
    if (g_windowWidth >= g_windowHeight)
        g_frustFovY = g_frustMinFov;
    else {
        const double RAD_PER_DEG = 0.5 * CS175_PI / 180;
        g_frustFovY = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight / g_windowWidth,
                            cos(g_frustMinFov * RAD_PER_DEG)) / RAD_PER_DEG;
    }
}

static Matrix4 makeProjectionMatrix() {
    return Matrix4::makeProjection(g_frustFovY, g_windowWidth / static_cast <double> (g_windowHeight),
                                   g_frustNear,
                                   g_frustFar);
}

static void drawStuff(const ShaderState& curSS, bool picking) {
    // short hand for current shader state

    // build & send proj. matrix to vshader
    const Matrix4 projmat = makeProjectionMatrix();
    sendProjectionMatrix(curSS, projmat);

    // get eyeRbt
    auto eye_rbt = getPathAccumRbt(g_world.get(), g_eye_node);

    // update arcballRbt
    g_arcballRbt = []() -> std::optional<RigTForm> {
        if (::g_eye_node == g_skyNode.get()
            && ::current_manipulating() == g_skyNode.get()
            && !::do_skysky)
            return RigTForm{};
        if (::current_manipulating() != g_skyNode.get() && ::current_manipulating() != ::g_eye_node)
            return getPathAccumRbt(g_world.get(), ::current_manipulating());
        // 위 경우를 제외하고는 rigTForm optional의 값은 없음.
        return {};
    }();


    const auto invEyeRbt = inv(eye_rbt);

    const Cvec3 eyeLight1 = Cvec3(invEyeRbt * Cvec4(g_light1, 1)); // g_light1 position in eye coordinates
    const Cvec3 eyeLight2 = Cvec3(invEyeRbt * Cvec4(g_light2, 1)); // g_light2 position in eye coordinates
    safe_glUniform3f(curSS.h_uLight, eyeLight1[0], eyeLight1[1], eyeLight1[2]);
    safe_glUniform3f(curSS.h_uLight2, eyeLight2[0], eyeLight2[1], eyeLight2[2]);


    if (!picking) {
        Drawer drawer(invEyeRbt, curSS);
        g_world->accept(drawer);

        if (g_arcballRbt.has_value()) {
            const auto arcballRbt = g_arcballRbt.value();

            // if not translating update arcballScale
            if (!(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)))
                g_arcballScale = getScreenToEyeScale((invEyeRbt * arcballRbt).getTranslation()[2], g_frustFovY,
                                                     g_windowHeight);

            // draw arcball
            Matrix4 MVM = rigTFormToMatrix(invEyeRbt * arcballRbt)
                          * Matrix4::makeScale(Cvec3{g_arcballScale * g_arcballScreenRadius});
            Matrix4 NMVM = normalMatrix(MVM);

            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            sendModelViewNormalMatrix(curSS, MVM, NMVM);
            safe_glUniform3f(curSS.h_uColor, 0., 0.7, 0.);
            g_arcball->draw(curSS);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }
    }
    else {
        Picker picker(invEyeRbt, curSS);
        g_world->accept(picker);

        glFlush();

        auto selected = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);
        if (selected == g_groundNode.get()) {
            g_currentPickedRbtNode = nullptr;   // set to NULL
        }
        else {
            g_currentPickedRbtNode = selected;
        }

        if (g_currentPickedRbtNode == nullptr) {
            std::cout << "No part picked\n";
        }
        else {
            std::cout << "Part picked\n";
        }
    }
}

static void pick() {
    // We need to set the clear color to black, for pick rendering.
    // so let's save the clear color
    GLdouble clearColor[4];
    glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);

    glClearColor(0, 0, 0, 0);

    // using PICKING_SHADER as the shader
    glUseProgram(g_shaderStates[PICKING_SHADER]->program);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawStuff(*g_shaderStates[PICKING_SHADER], true);

    // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
    // to see result of the pick rendering pass
//     glutSwapBuffers();

    //Now set back the clear color
    glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

    checkGlErrors();
}

static void display() {
    glUseProgram(g_shaderStates[g_activeShader]->program);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                   // clear framebuffer color&depth

    drawStuff(*g_shaderStates[g_activeShader], false);

    glutSwapBuffers();                                    // show the back buffer (where we rendered stuff)

    checkGlErrors();
}

static void reshape(const int w, const int h) {
    g_windowWidth = w;
    g_windowHeight = h;
    g_arcballScreenRadius = 0.25 * std::fmin(g_windowWidth, g_windowHeight);
    glViewport(0, 0, w, h);
    std::cerr << "Size of window is now " << w << "x" << h << std::endl;
    updateFrustFovY();
    glutPostRedisplay();
}

static void motion(const int x, const int y) {
    const double dx = x - g_mouseClickX;
    const double dy = g_windowHeight - y - 1 - g_mouseClickY;

    RigTForm rigT;
    const auto translation_scale = g_arcballRbt.has_value() ? g_arcballScale : 0.01;
    if (g_mouseLClickButton && !g_mouseRClickButton) { // left button down?
        if (g_arcballRbt.has_value()) {
            Cvec2 arcball_coord = getScreenSpaceCoord(
                    (inv(g_eye_node->getRbt()) * g_arcballRbt.value()).getTranslation(),
                    makeProjectionMatrix(), g_frustNear, g_frustFovY,
                    g_windowWidth, g_windowHeight);
//            std::cout << arcball_coord[0] << " " << arcball_coord[1] << std::endl;
//            std::cout << x << " " << y << std::endl;
            auto get_vec3 = [](Cvec2 vec2) {
                auto sqr = [](double d) { return d * d; };
                auto ret = Cvec3{};
                ret[0] = vec2[0];
                ret[1] = vec2[1];
                auto cc = sqr(g_arcballScreenRadius) - sqr(vec2[0]) - sqr(vec2[1]);
                ret[2] = cc > 0 ? std::sqrt(cc) : 0;
                return ret.normalize();
            };

            auto v2 = get_vec3(Cvec2{x, g_windowHeight - y - 1} - arcball_coord);
            auto v1 = get_vec3(Cvec2{g_mouseClickX, g_mouseClickY} - arcball_coord);

            rigT = RigTForm{Quat{dot(v1, v2), cross(v1, v2)}};
//            rigT = RigTForm{Quat::makeXRotation(-dy) * Quat::makeYRotation(dx)};

        }
        else
            rigT = RigTForm{Quat::makeXRotation(-dy) * Quat::makeYRotation(dx)};
    }
    else if (g_mouseRClickButton && !g_mouseLClickButton) { // right button down?
        rigT = RigTForm{Cvec3(dx, dy, 0) * translation_scale};
    }
    else if (g_mouseMClickButton ||
             (g_mouseLClickButton && g_mouseRClickButton)) {  // middle or (left and right) button down?
        rigT = RigTForm{Cvec3(0, 0, -dy) * translation_scale};
    }

    if (g_mouseClickDown) {
        const auto& settings = ::get_manipulation_setting();
        if (settings.can_manipulate) {
            const auto& target_rbt = ::current_manipulating()->getRbt();
            bool invert_translation = false;
            bool invert_linear = false;

            if (::current_manipulating() == ::g_skyNode.get()) {
                if (::g_eye_node == ::g_skyNode.get() && !do_skysky)
                    invert_translation = true;
                invert_linear = true;
            }
            else if (::g_eye_node == ::current_manipulating()) {
                invert_linear = true;
//                invert_translation = true;
            }

            {
                auto trans_part = transFact(rigT);
                auto lin_part = linFact(rigT);
                if (invert_translation)
                    trans_part = inv(trans_part);
                if (invert_linear)
                    lin_part = inv(lin_part);
                rigT = trans_part * lin_part;
            }

            auto rbt_parent_frame_ref_world = getPathAccumRbt(g_world.get(), ::current_manipulating(), 1);
            ::current_manipulating()->setRbt(
                    inv(rbt_parent_frame_ref_world) * settings.respect_frame * rigT * inv(settings.respect_frame) *
                    rbt_parent_frame_ref_world * target_rbt);
            glutPostRedisplay(); // we always redraw if we changed the scene
        }
    }

    g_mouseClickX = x;
    g_mouseClickY = g_windowHeight - y - 1;
}


static void mouse(const int button, const int state, const int x, const int y) {
    g_mouseClickX = x;
    g_mouseClickY =
            g_windowHeight - y -
            1;  // conversion from GLUT window-coordinate-system to OpenGL window-coordinate-system

    g_mouseLClickButton |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
    g_mouseRClickButton |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
    g_mouseMClickButton |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

    g_mouseLClickButton &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
    g_mouseRClickButton &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
    g_mouseMClickButton &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);

    g_mouseClickDown = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;

    if (::waiting_pick && button == GLUT_LEFT_BUTTON) {
        pick();
        std::cout << "Picking mode is off" << std::endl;
        ::waiting_pick = false;
    }
    glutPostRedisplay();
}


static void keyboard(const unsigned char key, const int x, const int y) {
    auto add_current_state_to_animation = []() {
        auto pos = ::current_frame_iter;
        if (pos != animation.end()) {
            pos++;
        }
        ::current_frame_iter = animation.insert(pos, save_frame());
        std::cout << "created a frame" << std::endl;
    };

    const auto save_filename = std::string{"animation.txt"};

    switch (key) {
        case 27:
            exit(0);                                  // ESC
        case 'h':
            std::cout << " ============== H E L P ==============\n\n" << "h\t\thelp menu\n"
                      << "s\t\tsave screenshot\n"
                      << "f\t\tToggle flat shading on/off.\n" << "o\t\tCycle object to edit\n"
                      << "v\t\tCycle view\n"
                      << "drag left mouse to rotate\n" << std::endl;
            break;
        case 's':
            glFlush();
            writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
            break;
        case 'f':
            g_activeShader ^= 1;
            break;
        case 'v': {
            constexpr int candidates_count = 3;
            static const std::array<SgRbtNode*, candidates_count> candidates{::g_skyNode.get(), ::g_robot1Node.get(),
                                                                             ::g_robot2Node.get()};
            camera_index = (camera_index + 1) % candidates_count;
            g_eye_node = candidates[camera_index];
            break;
        }
//        case 'o': {
//            current_manipulating = asd::object_enum(
//                    (static_cast<int>(current_manipulating) + 1) % static_cast<int>(asd::object_enum::COUNT));
//            std::cout << "Active object is " << object_to_name(current_manipulating) << std::endl;
//            break;
//        }
        case 'm': {
            ::do_skysky = !do_skysky;
            std::cout << "Editing sky eye w.r.t. " << (::do_skysky ? "sky-sky" : "world-sky")
                      << " frame" << std::endl;
            break;
        }
        case 'p': {
            ::waiting_pick = !::waiting_pick;
            std::cout << "Picking mode is " << (::waiting_pick ? "on" : "off") << "\n";
            break;
        }
        case ' ': {
            load_current_frame_if_defined();
            break;
        }
        case 'u': {
            if (::current_frame_iter != ::animation.end()) {
                *::current_frame_iter = ::save_frame();
                std::cout << "updated frame" << std::endl;
            }
            else {
                add_current_state_to_animation();
            }
            break;
        }
        case '.':
        case '>': {
            auto temp_iter = ::current_frame_iter;
            temp_iter++;
            if (temp_iter != ::animation.end()) {
                ::current_frame_iter++;
                load_current_frame_if_defined();
            }
            break;
        }
        case ',':
        case '<': {
            if (::current_frame_iter != ::animation.begin()) {
                ::current_frame_iter--;
                load_current_frame_if_defined();
            }
            break;
        }
        case 'd': {
            if (::current_frame_iter != ::animation.end()) {
                auto curr = ::animation.erase(::current_frame_iter);
                std::cout << "deleted frame" << std::endl;
                if (curr != ::animation.begin()) {
                    curr--;
                }
                ::current_frame_iter = curr;
                load_current_frame_if_defined();
            }
            break;
        }
        case 'n': {
            add_current_state_to_animation();
            break;
        }
        case 'i': {
            load_animation_from_file(save_filename);
            break;
        }
        case 'w': {
            save_animation_to_file(save_filename);
            break;
        }
        case 'y': {
            ::animate_timer_callback(0);
        }
    }
    glutPostRedisplay();
}

static void initGlutState(int argc, char* argv[]) {
    glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);  //  RGBA pixel channels and double buffering
    glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
    glutCreateWindow("Assignment 5");                       // title the window

    glutDisplayFunc(display);                               // display rendering callback
    glutReshapeFunc(reshape);                               // window reshape callback
    glutMotionFunc(motion);                                 // mouse movement callback
    glutMouseFunc(mouse);                                   // mouse click callback
    glutKeyboardFunc(keyboard);
}

static void initGLState() {
    glClearColor(128. / 255., 200. / 255., 255. / 255., 0.);
    glClearDepth(0.);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_GREATER);
    glReadBuffer(GL_BACK);
    if (!g_Gl2Compatible)
        glEnable(GL_FRAMEBUFFER_SRGB);
}

static void initShaders() {
    g_shaderStates.resize(g_numShaders);
    for (int i = 0; i < g_numShaders; ++i) {
        if (g_Gl2Compatible)
            g_shaderStates[i].reset(new ShaderState(g_shaderFilesGl2[i][0], g_shaderFilesGl2[i][1]));
        else
            g_shaderStates[i].reset(new ShaderState(g_shaderFiles[i][0], g_shaderFiles[i][1]));
    }
}

static void initGeometry() {
    initGround();
    initCubes();
    initArcball();
}

static void constructRobot(std::shared_ptr<SgTransformNode> base, const Cvec3& color) {

    const float ARM_LEN = 0.7,
            ARM_THICK = 0.25,
            TORSO_LEN = 1.5,
            TORSO_THICK = 0.25,
            TORSO_WIDTH = 1;
    const int NUM_JOINTS = 10,
            NUM_SHAPES = 10;

    struct JointDesc {
        int parent;
        float x, y, z;
    };

    JointDesc jointDesc[NUM_JOINTS] = {
            {-1}, // torso
            {0, TORSO_WIDTH / 2,  (TORSO_LEN / 2),  0}, // upper right arm
            {1, ARM_LEN + 0.05f,  0,                0}, // lower right arm
            {0, -TORSO_WIDTH / 2, (TORSO_LEN / 2),  0}, // upper left arm
            {3, -ARM_LEN - 0.05f, 0,                0}, // lower left arm
            {0, 0,                TORSO_LEN / 2,    0}, // head
            {0, TORSO_WIDTH / 2,  -(TORSO_LEN / 2), 0}, // upper right leg
            {6, 0,                -ARM_LEN - 0.05f, 0}, // lower right leg
            {0, -TORSO_WIDTH / 2, -(TORSO_LEN / 2), 0}, // upper left leg
            {8, 0,                -ARM_LEN - 0.05f, 0}, // lower left leg
    };

    struct ShapeDesc {
        int parentJointId;
        float x, y, z, sx, sy, sz;
        std::shared_ptr<Geometry> geometry;
    };

    ShapeDesc shapeDesc[NUM_SHAPES] = {
            {0, 0,              0,              0, (TORSO_WIDTH), (TORSO_LEN), (TORSO_THICK), g_cube}, // torso
            {1, (ARM_LEN /
                 2),            0,              0, (ARM_LEN),     (ARM_THICK), (ARM_THICK),   g_cube}, // upper right arm
            {2, (ARM_LEN /
                 2),            0,              0, (ARM_LEN),     (ARM_THICK), (ARM_THICK),   g_cube}, // lower right arm
            {3, -(ARM_LEN / 2), 0,              0, (ARM_LEN),     (ARM_THICK), (ARM_THICK),   g_cube}, // upper left arm
            {4, -(ARM_LEN / 2), 0,              0, (ARM_LEN),     (ARM_THICK), (ARM_THICK),   g_cube}, // lower left arm
            {5, 0,              0.5,            0, 0.3,           0.3,         0.3,           g_arcball}, // head
            {6, 0,              -(ARM_LEN /
                                  2),           0, (ARM_THICK),   (ARM_LEN),   (ARM_THICK),   g_cube}, // upper right arm
            {7, 0,              -(ARM_LEN /
                                  2),           0, (ARM_THICK),   (ARM_LEN),   (ARM_THICK),   g_cube}, // lower right arm
            {8, 0,              -(ARM_LEN / 2), 0, (ARM_THICK),   (ARM_LEN),   (ARM_THICK),   g_cube}, // upper left arm
            {9, 0,              -(ARM_LEN / 2), 0, (ARM_THICK),   (ARM_LEN),   (ARM_THICK),   g_cube}, // lower left arm
    };

    std::shared_ptr<SgTransformNode> jointNodes[NUM_JOINTS];

    for (int i = 0; i < NUM_JOINTS; ++i) {
        if (auto parent = jointDesc[i].parent; parent == -1)
            jointNodes[i] = base;
        else {
            jointNodes[i].reset(new SgRbtNode(RigTForm(Cvec3(jointDesc[i].x, jointDesc[i].y, jointDesc[i].z))));
            jointNodes[parent]->addChild(jointNodes[i]);
        }
    }

    for (auto& i : shapeDesc) {
        std::shared_ptr<MyShapeNode> shape{
                new MyShapeNode(i.geometry,
                                color,
                                Cvec3(i.x, i.y, i.z),
                                Cvec3(0, 0, 0),
                                Cvec3(i.sx, i.sy, i.sz))};
        jointNodes[i.parentJointId]->addChild(shape);
    }
}

static void initScene() {
    g_world.reset(new SgRootNode());

    g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.25, 4.0))));

    g_groundNode.reset(new SgRbtNode());
    g_groundNode->addChild(std::make_shared<MyShapeNode>(
            g_ground, Cvec3(0.1, 0.95, 0.1)));

    g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
    g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

    constructRobot(g_robot1Node, Cvec3(1, 0, 0)); // a Red robot
    constructRobot(g_robot2Node, Cvec3(0, 0, 1)); // a Blue robot

    g_world->addChild(g_skyNode);
    g_world->addChild(g_groundNode);
    g_world->addChild(g_robot1Node);
    g_world->addChild(g_robot2Node);
}


static void load_frame(asd::frame& frame) {
    auto nodes_list = dumpSgRbtNodes(::g_world);
    for (int i = 0; i < nodes_list.size(); i++) {
        nodes_list[i]->setRbt(frame.rbt_states[i]);
    }
    glutPostRedisplay();
}


static asd::frame save_frame() {
    auto ret = asd::frame{};
    for (auto&& node: dumpSgRbtNodes(::g_world)) {
        ret.rbt_states.push_back(node->getRbt());
    }
    return ret;
}


int main(int argc, char* argv[]) {
    try {
        initGlutState(argc, argv);

        glewInit(); // load the OpenGL extensions

        std::cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.3")
                  << std::endl;
        if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
            throw std::runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
        else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
            throw std::runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");

        initGLState();
        initShaders();
        initGeometry();
        initScene();

        // My initializations
        g_eye_node = g_skyNode.get();
        ::current_frame_iter = ::animation.begin();

        glutMainLoop();

        return 0;
    } catch (const std::runtime_error& e) {
        std::cout << "Exception caught: " << e.what() << std::endl;
        return -1;
    }
}


static bool show_animation_at_time(float t) {
    auto prev_frame_index = static_cast<int>(t) + 1;
    if (prev_frame_index > static_cast<int>(animation.size()) - 3) {// 0 ~ n-1까지 있으나 1~n-2만 표시.
        // end animation
        return true;
    }
    else {
        auto alpha = t - static_cast<float>(prev_frame_index - 1);
        const auto&[prev_frame, next_frame] = [&]() {
            auto it = animation.begin();
            for (int i = 0; i < prev_frame_index; i++) {
                it++;
            }
            auto next_it = it;
            next_it++;
            return std::tie(std::as_const(*it), std::as_const(*next_it));
        }();
        const auto& prev_rbts = prev_frame.rbt_states;
        const auto& next_rbts = next_frame.rbt_states;
        auto interpolated_frame = asd::frame{};

        for (auto i = 0; i < prev_rbts.size(); i++) {
            auto& rbt0 = prev_rbts[i];
            auto& rbt1 = next_rbts[i];

            auto interpolated_translation = rbt0.getTranslation() * (1 - alpha) + rbt1.getTranslation() * alpha;
            auto interpolated_rotation = slerp(rbt0.getRotation(), rbt1.getRotation(), alpha);
            interpolated_frame.rbt_states.emplace_back(interpolated_translation,
                                                       interpolated_rotation); // RigTForm(Cvec3, Quat)
        }
        load_frame(interpolated_frame);
        return false;
    }
}


static void animate_timer_callback(int ms) {
    float t = static_cast<float>(ms) / static_cast<float>(ms_between_keyframes);

    bool endReached = show_animation_at_time(t);
    if (!endReached) {
        auto dt = 1000 / animated_frames_per_sec;
        glutTimerFunc(dt, animate_timer_callback, ms + dt);
    }
    else {
        std::cout << "Animation ended" << std::endl;
        ::current_frame_iter = [&](){
            auto it = animation.end();
            for (int i = 0; i < 2; i++)
                it--;
            return it;
        }();
        ::load_current_frame_if_defined();
    }
}

