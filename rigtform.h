#ifndef RIGTFORM_H
#define RIGTFORM_H

#include <iostream>
#include <cassert>

#include "matrix4.h"
#include "quat.h"

static Cvec3 operator*(const Quat &quat, const Cvec3 &vec3) {
    return static_cast<Cvec3>(quat * static_cast<Cvec4>(vec3));
}

class RigTForm {
    Cvec3 t_; // translation component
    Quat r_;  // rotation component represented as a quaternion

public:
    RigTForm() : RigTForm(Cvec3{0}, Quat{}) {
        assert(norm2(Quat(1, 0, 0, 0) - r_) < CS175_EPS2);
    }

    RigTForm(const Cvec3 &t, const Quat &r) : t_{t}, r_{r} {}

    explicit RigTForm(const Cvec3 &t) : RigTForm(t, Quat{}) {}

    explicit RigTForm(const Quat &r) : RigTForm{Cvec3{0}, r} {}

    [[nodiscard]] Cvec3 getTranslation() const {
        return t_;
    }

    [[nodiscard]] Quat getRotation() const {
        return r_;
    }

    RigTForm &setTranslation(const Cvec3 &t) {
        t_ = t;
        return *this;
    }

    RigTForm &setRotation(const Quat &r) {
        r_ = r;
        return *this;
    }

    Cvec4 operator*(const Cvec4 &a) const {
        return r_ * a + ((std::abs(a[3]) < CS175_EPS) ? Cvec4{0} : Cvec4{t_, 0});
    }

    RigTForm operator*(const RigTForm &a) const {
        auto ret = RigTForm{
                t_ + r_ * a.t_,
                r_ * a.r_,
        };
        return ret;
    }
};

inline RigTForm inv(const RigTForm &tform) {
    auto ret = RigTForm{
            -(inv(tform.getRotation()) * tform.getTranslation()),
            inv(tform.getRotation())
    };
    return ret;
}

inline RigTForm transFact(const RigTForm &tform) {
    return RigTForm(tform.getTranslation());
}

inline RigTForm linFact(const RigTForm &tform) {
    return RigTForm(tform.getRotation());
}

inline Matrix4 rigTFormToMatrix(const RigTForm &rbt) {
    auto T = Matrix4::makeTranslation(rbt.getTranslation());
    auto R = quatToMatrix(rbt.getRotation());
    return T * R;
}

#endif
