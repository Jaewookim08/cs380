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

static const int PICKING_SHADER = 2; // index of the picking shader is g_shaerFiles

static std::shared_ptr<Material> g_redDiffuseMat,
        g_blueDiffuseMat,
        g_bumpFloorMat,
        g_arcballMat,
        g_pickingMat,
        g_lightMat;

std::shared_ptr<Material> g_overridingMaterial;


// --------- Geometry


namespace asd {
    struct frame {
        std::vector<RigTForm> rbt_states;
    };
    using animation = std::list<frame>;


    template<class Container, class Func>
    auto transformed_tuple(const Container& container, Func func) {
        auto transform_parameters_to_tuple = [&func](auto& ... args) {
//            decltype((..., func(args)))::asdf;
            return std::forward_as_tuple(func(args)...);
        };

        return std::apply(transform_parameters_to_tuple, container);
    }
}

static asd::animation animation;
static decltype(animation)::iterator current_frame_iter;

static bool is_animation_playing = false;
static bool animation_stop_requested = false;


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


using MyShapeNode = SgGeometryShapeNode;

static bool waiting_pick = false;
static std::shared_ptr<SgRootNode> g_world;
static std::shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node;
static SgRbtNode* g_currentPickedRbtNode; // used later when you do picking
static SgRbtNode* g_eye_node;

// Vertex buffer and index buffer associated with the ground and cube geometry
static std::shared_ptr<Geometry> g_ground, g_cube, g_sphere;

// --------- Scene

static std::shared_ptr<SgRbtNode> g_light1Node, g_light2Node;

static std::unique_ptr<RigTForm> g_arcballRbt;

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
    using namespace std;
    int ibLen, vbLen;
    getPlaneVbIbLen(vbLen, ibLen);

    // Temporary storage for cube Geometry
    vector<VertexPNTBX> vtx(vbLen);
    vector<unsigned short> idx(ibLen);

    makePlane(g_groundSize * 2, vtx.begin(), idx.begin());
    g_ground.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initCubes() {
    using namespace std;
    int ibLen, vbLen;
    getCubeVbIbLen(vbLen, ibLen);

    // Temporary storage for cube Geometry
    vector<VertexPNTBX> vtx(vbLen);
    vector<unsigned short> idx(ibLen);

    makeCube(1, vtx.begin(), idx.begin());
    g_cube.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initSphere() {
    using namespace std;
    int ibLen, vbLen;
    getSphereVbIbLen(20, 10, vbLen, ibLen);

    // Temporary storage for sphere Geometry
    vector<VertexPNTBX> vtx(vbLen);
    vector<unsigned short> idx(ibLen);
    makeSphere(1, 20, 10, vtx.begin(), idx.begin());
    g_sphere.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}

// takes a projection matrix and send to the the shaders
inline void sendProjectionMatrix(Uniforms& uniforms, const Matrix4& projMatrix) {
    uniforms.put("uProjMatrix", projMatrix);
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

static void drawStuff(bool picking) {
    Uniforms uniforms;

    // build & send proj. matrix to vshader
    const Matrix4 projmat = makeProjectionMatrix();
    sendProjectionMatrix(uniforms, projmat);

    // get SeyeRbt
    auto eye_rbt = getPathAccumRbt(g_world.get(), g_eye_node);

    // update arcballRbt
    g_arcballRbt = []() -> std::unique_ptr<RigTForm> {
        if (::g_eye_node == g_skyNode.get()
            && ::current_manipulating() == g_skyNode.get()
            && !::do_skysky)
            return std::make_unique<RigTForm>();
        if (::current_manipulating() != g_skyNode.get() && ::current_manipulating() != ::g_eye_node)
            return std::make_unique<RigTForm>(getPathAccumRbt(g_world.get(), ::current_manipulating()));
        return nullptr;
    }();


    const auto invEyeRbt = inv(eye_rbt);

    Cvec3 light1 = getPathAccumRbt(g_world.get(), g_light1Node.get()).getTranslation();
    Cvec3 light2 = getPathAccumRbt(g_world.get(), g_light2Node.get()).getTranslation();

    const Cvec3 eyeLight1 = Cvec3(invEyeRbt * Cvec4(light1, 1)); // g_light1 position in eye coordinates
    const Cvec3 eyeLight2 = Cvec3(invEyeRbt * Cvec4(light2, 1)); // g_light2 position in eye coordinates
    uniforms.put("uLight", eyeLight1);
    uniforms.put("uLight2", eyeLight2);


    if (!picking) {
        Drawer drawer(invEyeRbt, uniforms);
        g_world->accept(drawer);

        if (g_arcballRbt != nullptr) {
            const auto& arcballRbt = *g_arcballRbt;

            // if not translating update arcballScale
            if (!(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)))
                g_arcballScale = getScreenToEyeScale((invEyeRbt * arcballRbt).getTranslation()[2], g_frustFovY,
                                                     g_windowHeight);

            // draw arcball
            Matrix4 MVM = rigTFormToMatrix(invEyeRbt * arcballRbt)
                          * Matrix4::makeScale(Cvec3{g_arcballScale * g_arcballScreenRadius});
            Matrix4 NMVM = normalMatrix(MVM);

            sendModelViewNormalMatrix(uniforms, MVM, NMVM);
            g_arcballMat->draw(*g_sphere, uniforms);
        }
    }
    else {
        Picker picker(invEyeRbt, uniforms);

        g_overridingMaterial = g_pickingMat;
        g_world->accept(picker);
        g_overridingMaterial.reset();

        glFlush();

        auto* selected = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY).get();
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

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    drawStuff(true);

    // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
    // to see result of the pick rendering pass
//     glutSwapBuffers();

    //Now set back the clear color
    glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

    checkGlErrors();
}

static void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                   // clear framebuffer color&depth

    drawStuff(false);

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
    const auto translation_scale = (g_arcballRbt != nullptr) ? g_arcballScale : 0.01;
    if (g_mouseLClickButton && !g_mouseRClickButton) { // left button down?
        if (g_arcballRbt != nullptr) {
            Cvec2 arcball_coord = getScreenSpaceCoord(
                    (inv(g_eye_node->getRbt()) * (*g_arcballRbt)).getTranslation(),
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
            if (::is_animation_playing) {
                std::cout << "Cannot operate when playing animation" << std::endl;
            }
            else {
                load_current_frame_if_defined();
            }
            break;
        }
        case 'u': {
            if (::is_animation_playing) {
                std::cout << "Cannot operate when playing animation" << std::endl;
            }
            else {
                if (::current_frame_iter != ::animation.end()) {
                    *::current_frame_iter = ::save_frame();
                    std::cout << "updated frame" << std::endl;
                }
                else {
                    add_current_state_to_animation();
                }
            }
            break;
        }
        case '.':
        case '>': {
            if (::is_animation_playing) {
                std::cout << "Cannot operate when playing animation" << std::endl;
            }
            else {
                auto temp_iter = ::current_frame_iter;
                temp_iter++;
                if (temp_iter != ::animation.end()) {
                    ::current_frame_iter++;
                    load_current_frame_if_defined();
                }
            }
            break;
        }
        case ',':
        case '<': {
            if (::is_animation_playing) {
                std::cout << "Cannot operate when playing animation" << std::endl;
            }
            else {
                if (::current_frame_iter != ::animation.begin()) {
                    ::current_frame_iter--;
                    load_current_frame_if_defined();
                }
            }
            break;
        }
        case 'd': {
            if (::is_animation_playing) {
                std::cout << "Cannot operate when playing animation" << std::endl;
            }
            else {
                if (::current_frame_iter != ::animation.end()) {
                    auto curr = ::animation.erase(::current_frame_iter);
                    std::cout << "deleted frame" << std::endl;
                    if (curr != ::animation.begin()) {
                        curr--;
                    }
                    ::current_frame_iter = curr;
                    load_current_frame_if_defined();
                }
            }

            break;
        }
        case 'n': {
            if (::is_animation_playing) {
                std::cout << "Cannot operate when playing animation" << std::endl;
            }
            else {
                add_current_state_to_animation();
            }
            break;
        }
        case 'i': {
            if (::is_animation_playing) {
                std::cout << "Cannot operate when playing animation" << std::endl;
            }
            else {
                load_animation_from_file(save_filename);
            }
            break;
        }
        case 'w': {
            if (::is_animation_playing) {
                std::cout << "Cannot operate when playing animation" << std::endl;
            }
            else {
                save_animation_to_file(save_filename);
            }
            break;
        }
        case 'y': {
            if (::is_animation_playing) {
                if (::animation_stop_requested) {
                    std::cout << "Animation stop already requested. Please wait..." << std::endl;
                }
                else {
                    ::animation_stop_requested = true;
                    std::cout << "Stopping animation..." << std::endl;
                }
            }
            else {
                std::cout << "Playing animation..." << std::endl;
                ::is_animation_playing = true;
                ::animate_timer_callback(0);
            }
            break;
        }
        case '-':
        case '_': {
            ms_between_keyframes += 100;
            std::cout << ms_between_keyframes << " ms between keyframes" << std::endl;
            break;
        }
        case '=':
        case '+': {
            ms_between_keyframes = std::max(100, ms_between_keyframes - 100);
            std::cout << ms_between_keyframes << " ms between keyframes" << std::endl;
            break;
        }
    }
    glutPostRedisplay();
}

static void initGlutState(int argc, char* argv[]) {
    glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);  //  RGBA pixel channels and double buffering
    glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
    glutCreateWindow("Assignment 7");                       // title the window

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

static void initMaterials() {
    using namespace std;
    // Create some prototype materials
    Material diffuse("./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader");
    Material solid("./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader");

    // copy diffuse prototype and set red color
    g_redDiffuseMat.reset(new Material(diffuse));
    g_redDiffuseMat->getUniforms().put("uColor", Cvec3f(1, 0, 0));

    // copy diffuse prototype and set blue color
    g_blueDiffuseMat.reset(new Material(diffuse));
    g_blueDiffuseMat->getUniforms().put("uColor", Cvec3f(0, 0, 1));

    // normal mapping material
    g_bumpFloorMat.reset(new Material("./shaders/normal-gl3.vshader", "./shaders/normal-gl3.fshader"));
    g_bumpFloorMat->getUniforms().put("uTexColor", shared_ptr<ImageTexture>(new ImageTexture("Fieldstone.ppm", true)));
    g_bumpFloorMat->getUniforms().put("uTexNormal",
                                      shared_ptr<ImageTexture>(new ImageTexture("FieldstoneNormal.ppm", false)));

    // copy solid prototype, and set to wireframed rendering
    g_arcballMat.reset(new Material(solid));
    g_arcballMat->getUniforms().put("uColor", Cvec3f(0.27f, 0.82f, 0.35f));
    g_arcballMat->getRenderStates().polygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // copy solid prototype, and set to color white
    g_lightMat.reset(new Material(solid));
    g_lightMat->getUniforms().put("uColor", Cvec3f(1, 1, 1));

    // pick shader
    g_pickingMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"));
};

static void initGeometry() {
    initGround();
    initCubes();
    initSphere();
}

static void constructRobot(std::shared_ptr<SgTransformNode> base, std::shared_ptr<Material> material) {

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
            {5, 0,              0.5,            0, 0.3,           0.3,         0.3,           g_sphere}, // head
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
                                material,
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
            g_ground, g_bumpFloorMat, Cvec3(0, g_groundY, 0)));

    g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
    g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

    constructRobot(g_robot1Node, g_redDiffuseMat); // a Red robot
    constructRobot(g_robot2Node, g_blueDiffuseMat); // a Blue robot

//    static const Cvec3 g_light1(2.0, 3.0, 14.0), g_light2(-2, -3.0, -5.0);  // define two lights positions in world space
    g_light1Node.reset(new SgRbtNode{RigTForm{Cvec3{4., 3., 3.}}});
    g_light2Node.reset(new SgRbtNode{RigTForm{Cvec3{-4., 1.5, -3.}}});
    g_light1Node->addChild(std::make_shared<MyShapeNode>(g_sphere, g_lightMat, Cvec3{0, 0, 0}, Cvec3{0}, Cvec3{0.5}));
    g_light2Node->addChild(std::make_shared<MyShapeNode>(g_sphere, g_lightMat, Cvec3{0, 0, 0}, Cvec3{0}, Cvec3{0.5}));

    g_world->addChild(g_skyNode);
    g_world->addChild(g_groundNode);
    g_world->addChild(g_robot1Node);
    g_world->addChild(g_robot2Node);
    g_world->addChild(g_light1Node);
    g_world->addChild(g_light2Node);
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
        initMaterials();
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

namespace asd {
    template<class Container, class Func>
    struct ci_tup {
        explicit ci_tup(Container& container, std::size_t ind, Func func) : m_container{container}, m_ind{ind},
                                                                            m_func(func) {}

        Container& m_container;
        Func m_func;

        const std::size_t m_ind;

        template<int relative>
        auto get() {
            return m_func(std::get<relative + 1>(m_container)[m_ind]);
        }

        template<int relative>
        [[nodiscard]] auto get() const {
            return m_func(std::get<relative + 1>(m_container)[m_ind]);
        }
    };
}


static bool show_animation_at_time(float t) {
    auto prev_frame_index = static_cast<int>(t) + 1;
    if (prev_frame_index > static_cast<int>(animation.size()) - 3) {// 0 ~ n-1까지 있으나 1~n-2만 표시.
        // end animation
        return true;
    }
    else {
        auto alpha = t - static_cast<float>(prev_frame_index - 1);
        const auto& surrounding_frames_tuple = [&]() {
            auto it = animation.begin();
            for (int i = 0; i < prev_frame_index - 1; i++) {
                it++;
            }

            const auto& a = *it++;
            const auto& b = *it++;
            const auto& c = *it++;
            const auto& d = *it;
            return std::tie(a, b, c, d);
        }();
        const auto& surrounding_rbts = asd::transformed_tuple(surrounding_frames_tuple,
                                                       [](const asd::frame& frame) -> auto& { return frame.rbt_states; });

        const auto rbt_state_size = std::get<0>(surrounding_rbts).size();


        auto interpolated_frame = asd::frame{std::vector<RigTForm>(rbt_state_size)};


        for (std::size_t i = 0; i < rbt_state_size; i++) {
            const auto& interpolated_rotation = [&]() {
                const auto ci = asd::ci_tup{surrounding_rbts, i, [](const RigTForm& rbt) { return rbt.getRotation(); }};
//            auto d0 = (1./6.)*(ci.get<1>().getTranslation() - ci.get<0>());

                const auto& c0 = ci.get<0>();
                const auto& c1 = ci.get<1>();

                const auto& d0 = pow(ci.get<1>() * inv(ci.get<-1>()), 1. / 6.) * ci.get<0>();
                const auto& e0 = pow(ci.get<2>() * inv(ci.get<0>()), -1. / 6.) * ci.get<1>();

                const auto& f = slerp(c0, d0, alpha);
                const auto& g = slerp(d0, e0, alpha);
                const auto& h = slerp(e0, c1, alpha);
                const auto& m = slerp(f, g, alpha);
                const auto& n = slerp(g, h, alpha);
                const auto& ct = slerp(m, n, alpha);
                return ct;
            }();

            const auto& interpolated_translation = [&]() {
                const auto ci = asd::ci_tup{surrounding_rbts, i,
                                            [](const RigTForm& rbt) { return rbt.getTranslation(); }};

                const auto& d0 = (ci.get<1>() - ci.get<-1>()) * (1. / 6.) + ci.get<0>();
                const auto& e0 = (ci.get<2>() - ci.get<0>()) * (-1. / 6.) + ci.get<1>();

                return ci.get<0>() * std::pow(1 - alpha, 3) + d0 * (std::pow(1 - alpha, 2) * alpha * 3) +
                       e0 * (std::pow(alpha, 2) * (1 - alpha) * 3) + ci.get<1>() * std::pow(alpha, 3);
            }();

//            const auto ci = asd::ci_tup{surrounding_rbts, i,
//                                        [](const RigTForm& rbt) { return rbt.getTranslation(); }};
//
//            const auto& interpolated_translation = ci.get<0>() * (1 - alpha) + ci.get<1>() * alpha;
//
//            const auto ci_tmp = asd::ci_tup{surrounding_rbts, i, [](const RigTForm& rbt) { return rbt.getRotation(); }};
//            auto interpolated_rotation = slerp(ci_tmp.get<0>(), ci_tmp.get<1>(), alpha);
            interpolated_frame.rbt_states[i] = {interpolated_translation,
                                                interpolated_rotation}; // RigTForm(Cvec3, Quat)
        }
        load_frame(interpolated_frame);
        return false;
    }
}


static void animate_timer_callback(int ms) {
    float t = static_cast<float>(ms) / static_cast<float>(ms_between_keyframes);

    bool endReached = show_animation_at_time(t);
    if (!::animation_stop_requested && !endReached) {
        auto dt = 1000 / animated_frames_per_sec;
        glutTimerFunc(dt, animate_timer_callback, ms + dt);
    }
    else {
        is_animation_playing = false;
        animation_stop_requested = false;
        std::cout << "Finished playing animation." << std::endl;
        ::current_frame_iter = [&]() {
            if (animation.size() < 2)
                return animation.begin();
            auto it = animation.end();
            for (int i = 0; i < 2; i++)
                it--;
            return it;
        }();
        ::load_current_frame_if_defined();
    }
}

