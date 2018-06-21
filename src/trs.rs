use cgmath;
use std::fmt;

use approx::{AbsDiffEq,RelativeEq,UlpsEq};
use {DQuat, DMat4, DVec3, Quat, Mat4, Vec3};

/// Single-precision translation + rotation + non-uniform scale transform.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Trs {
    /// Translation vector.
    pub t: Vec3,

    /// Rotation quaternion.
    pub r: Quat,

    /// *Non-uniform* scale factor.
    pub s: Vec3,
}

impl fmt::Display for Trs {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", (self.t, self.r, self.s))
    }
}

impl Default for Trs {
    fn default() -> Self {
        Self::identity()
    }
}

impl Trs {
    /// Full constructor.
    pub fn new(t: Vec3, r: Quat, s: Vec3) -> Self {
        Trs { t, r, s }
    }

    /// Identity constructor.
    pub fn identity() -> Self {
        Trs {
            t: vec3!(),
            r: quat!(),
            s: vec3!(1.0),
        }
    }

    /// Returns the equivalent matrix representation for this transform.
    pub fn matrix(&self) -> Mat4 {
        let t = cgmath::Matrix4::from_translation(cgmath::Vector3::new(self.t.x, self.t.y, self.t.z));
        let r = cgmath::Matrix4::from(cgmath::Quaternion::new(self.r.s, self.r.x,  self.r.y, self.r.z));
        let s = cgmath::Matrix4::from_nonuniform_scale(self.s.x, self.s.y, self.s.z);
        let m: [[f32; 4]; 4] = (t * r * s).into();
        Mat4::from(m)
    }
}

impl RelativeEq for Trs {
    fn default_max_relative() -> Self::Epsilon {
        <f32 as RelativeEq>::default_max_relative()
    }

    fn relative_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_relative: Self::Epsilon,
    ) -> bool {
        self.t.relative_eq(&other.t, epsilon, max_relative)
            &&    
        self.r.relative_eq(&other.r, epsilon, max_relative)
            &&
        self.s.relative_eq(&other.s, epsilon, max_relative)
    }
}

impl UlpsEq for Trs {
    fn default_max_ulps() -> u32 {
        <f32 as UlpsEq>::default_max_ulps()
    }

    fn ulps_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_ulps: u32,
    ) -> bool {
        self.t.ulps_eq(&other.t, epsilon, max_ulps)
            &&
        self.r.ulps_eq(&other.r, epsilon, max_ulps)
            &&
        self.s.ulps_eq(&other.s, epsilon, max_ulps)
    }
}

impl AbsDiffEq for Trs {
    type Epsilon = <f32 as AbsDiffEq>::Epsilon;

    fn default_epsilon() -> Self::Epsilon {
        <f32 as AbsDiffEq>::default_epsilon()
    }

    fn abs_diff_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
    ) -> bool {
        self.t.abs_diff_eq(&other.t, epsilon, max_ulps)
            &&
        self.r.abs_diff_eq(&other.r, epsilon, max_ulps)
            &&
        self.s.abs_diff_eq(&other.s, epsilon, max_ulps)
    }
}

/// Double-precision translation + rotation + non-uniform scale transform.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DTrs {
    /// Translation vector.
    pub t: DVec3,

    /// Rotation quaternion.
    pub r: DQuat,

    /// *Non-uniform* scale factor.
    pub s: DVec3,
}

impl fmt::Display for DTrs {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", (self.t, self.r, self.s))
    }
}

impl Default for DTrs {
    fn default() -> Self {
        Self::identity()
    }
}

impl DTrs {
    /// Full constructor.
    pub fn new(t: DVec3, r: DQuat, s: DVec3) -> Self {
        DTrs { t, r, s }
    }

    /// Identity constructor.
    pub fn identity() -> Self {
        DTrs {
            t: dvec3!(),
            r: dquat!(),
            s: dvec3!(1.0),
        }
    }

    /// Returns the equivalent matrix representation for this transform.
    pub fn matrix(&self) -> DMat4 {
        let t = cgmath::Matrix4::from_translation(cgmath::Vector3::new(self.t.x, self.t.y, self.t.z));
        let r = cgmath::Matrix4::from(cgmath::Quaternion::new(self.r.s, self.r.x,  self.r.y, self.r.z));
        let s = cgmath::Matrix4::from_nonuniform_scale(self.s.x, self.s.y, self.s.z);
        let m: [[f64; 4]; 4] = (t * r * s).into();
        DMat4::from(m)
    }
}

impl RelativeEq for DTrs {
    fn default_max_relative() -> Self::Epsilon {
        <f64 as RelativeEq>::default_max_relative()
    }

    fn relative_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_relative: Self::Epsilon,
    ) -> bool {
        self.t.relative_eq(&other.t, epsilon, max_relative)
            &&    
        self.r.relative_eq(&other.r, epsilon, max_relative)
            &&
        self.s.relative_eq(&other.s, epsilon, max_relative)
    }
}

impl UlpsEq for DTrs {
    fn default_max_ulps() -> u32 {
        <f64 as UlpsEq>::default_max_ulps()
    }

    fn ulps_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_ulps: u32,
    ) -> bool {
        self.t.ulps_eq(&other.t, epsilon, max_ulps)
            &&
        self.r.ulps_eq(&other.r, epsilon, max_ulps)
            &&
        self.s.ulps_eq(&other.s, epsilon, max_ulps)
    }
}

impl AbsDiffEq for DTrs {
    type Epsilon = <f64 as AbsDiffEq>::Epsilon;

    fn default_epsilon() -> Self::Epsilon {
        <f64 as AbsDiffEq>::default_epsilon()
    }

    fn abs_diff_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
    ) -> bool {
        self.t.abs_diff_eq(&other.t, epsilon, max_ulps)
            &&
        self.r.abs_diff_eq(&other.r, epsilon, max_ulps)
            &&
        self.s.abs_diff_eq(&other.s, epsilon, max_ulps)
    }
}
