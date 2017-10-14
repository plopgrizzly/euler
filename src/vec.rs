use approx::ApproxEq;
use cgmath;
use std::{fmt, mem, ops};

/// Single-precision 2D vector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    /// Full constructor.
    pub fn new(x: f32, y: f32) -> Self {
        Vec2 { x, y }
    }

    /// Zero constructor.
    pub fn zero() -> Self {
        Default::default()
    }
}

impl From<f32> for Vec2 {
    fn from(arg: f32) -> Self {
        Self::new(arg, arg)
    }
}

impl From<f64> for Vec2 {
    fn from(arg: f64) -> Self {
        Self::from(arg as f32)
    }
}

impl From<DVec2> for Vec2 {
    fn from(arg: DVec2) -> Self {
        Self::new(arg.x as f32, arg.y as f32)
    }
}

impl fmt::Display for Vec2 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", (self.x, self.y))
    }
}

/// Single-precision 3D vector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    /// Full constructor.
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Vec3 { x, y, z }
    }

    /// Zero constructor.
    pub fn zero() -> Self {
        Default::default()
    }
}

impl From<f32> for Vec3 {
    fn from(arg: f32) -> Self {
        Self::new(arg, arg, arg)
    }
}

impl From<f64> for Vec3 {
    fn from(arg: f64) -> Self {
        Self::from(arg as f32)
    }
}

impl From<DVec3> for Vec3 {
    fn from(arg: DVec3) -> Self {
        Self::new(arg.x as f32, arg.y as f32, arg.z as f32)
    }
}

impl From<(Vec2, f32)> for Vec3 {
    fn from(arg: (Vec2, f32)) -> Self {
        Self::new(arg.0.x, arg.0.y, arg.1)
    }
}

impl fmt::Display for Vec3 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", (self.x, self.y, self.z))
    }
}

/// Single-precision 4D vector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Vec4 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl Vec4 {
    /// Full constructor.
    pub fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Vec4 { x, y, z, w }
    }

    /// Zero constructor.
    pub fn zero() -> Self {
        Default::default()
    }
}

impl From<f32> for Vec4 {
    fn from(arg: f32) -> Self {
        Self::new(arg, arg, arg, arg)
    }
}

impl From<f64> for Vec4 {
    fn from(arg: f64) -> Self {
        Self::from(arg as f32)
    }
}

impl From<DVec4> for Vec4 {
    fn from(arg: DVec4) -> Self {
        Self::new(arg.x as f32, arg.y as f32, arg.z as f32, arg.w as f32)
    }
}

impl From<(Vec2, f32, f32)> for Vec4 {
    fn from(arg: (Vec2, f32, f32)) -> Self {
        Self::new(arg.0.x, arg.0.y, arg.1, arg.2)
    }
}

impl From<(Vec3, f32)> for Vec4 {
    fn from(arg: (Vec3, f32)) -> Self {
        Self::new(arg.0.x, arg.0.y, arg.0.z, arg.1)
    }
}

impl fmt::Display for Vec4 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", (self.x, self.y, self.z, self.w))
    }
}

/// Double-precision 2D vector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct DVec2 {
    pub x: f64,
    pub y: f64,
}

impl DVec2 {
    /// Full constructor.
    pub fn new(x: f64, y: f64) -> Self {
        DVec2 { x, y }
    }

    /// Zero constructor.
    pub fn zero() -> Self {
        Default::default()
    }
}

impl From<f32> for DVec2 {
    fn from(arg: f32) -> Self {
        Self::from(arg as f64)
    }
}

impl From<f64> for DVec2 {
    fn from(arg: f64) -> Self {
        Self::new(arg, arg)
    }
}

impl From<Vec2> for DVec2 {
    fn from(arg: Vec2) -> Self {
        Self::new(arg.x as f64, arg.y as f64)
    }
}

impl fmt::Display for DVec2 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", (self.x, self.y))
    }
}

/// Double-precision 3D vector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct DVec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl DVec3 {
    /// Full constructor.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        DVec3 { x, y, z }
    }

    /// Zero constructor.
    pub fn zero() -> Self {
        Default::default()
    }
}

impl From<f32> for DVec3 {
    fn from(arg: f32) -> Self {
        Self::from(arg as f64)
    }
}

impl From<f64> for DVec3 {
    fn from(arg: f64) -> Self {
        Self::new(arg, arg, arg)
    }
}

impl From<Vec3> for DVec3 {
    fn from(arg: Vec3) -> Self {
        Self::new(arg.x as f64, arg.y as f64, arg.z as f64)
    }
}

impl From<(DVec2, f64)> for DVec3 {
    fn from(arg: (DVec2, f64)) -> Self {
        Self::new(arg.0.x, arg.0.y, arg.1)
    }
}

impl fmt::Display for DVec3 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", (self.x, self.y, self.z))
    }
}

/// Double-precision 4D vector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct DVec4 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl DVec4 {
    /// Full constructor.
    pub fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        DVec4 { x, y, z, w }
    }

    /// Zero constructor.
    pub fn zero() -> Self {
        Default::default()
    }
}

impl From<f32> for DVec4 {
    fn from(arg: f32) -> Self {
        Self::from(arg as f64)
    }
}

impl From<f64> for DVec4 {
    fn from(arg: f64) -> Self {
        Self::new(arg, arg, arg, arg)
    }
}

impl From<Vec4> for DVec4 {
    fn from(arg: Vec4) -> Self {
        Self::new(arg.x as f64, arg.y as f64, arg.z as f64, arg.w as f64)
    }
}

impl From<(DVec2, f64, f64)> for DVec4 {
    fn from(arg: (DVec2, f64, f64)) -> Self {
        Self::new(arg.0.x, arg.0.y, arg.1, arg.2)
    }
}

impl From<(DVec3, f64)> for DVec4 {
    fn from(arg: (DVec3, f64)) -> Self {
        Self::new(arg.0.x, arg.0.y, arg.0.z, arg.1)
    }
}

impl fmt::Display for DVec4 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", (self.x, self.y, self.z, self.w))
    }
}

impl DVec3 {
    /// Returns the cross product of two vectors.
    pub fn cross(self, rhs: Self) -> Self {
        let a: &cgmath::Vector3<f64> = self.as_ref().into();
        let b: &cgmath::Vector3<f64> = rhs.as_ref().into();
        let v: [f64; 3] = a.cross(*b).into();
        v.into()
    }
}

impl Vec3 {
    /// Returns the cross product of two vectors.
    pub fn cross(self, rhs: Self) -> Self {
        let a: &cgmath::Vector3<f32> = self.as_ref().into();
        let b: &cgmath::Vector3<f32> = rhs.as_ref().into();
        let v: [f32; 3] = a.cross(*b).into();
        v.into()
    }
}

macro_rules! impl_vector {
    ($self:ty, $base:ty, $inner:ty, $array:ty) => {
        impl $self {
            /// Returns the dot product of two vectors.
            pub fn dot(self, rhs: $self) -> $base {
                use cgmath::InnerSpace;
                let a: &$inner = self.as_ref().into();
                let b: &$inner = rhs.as_ref().into();
                a.dot(*b)
            }
        }

        impl ops::Add<$self> for $self {
            type Output = $self;
            fn add(self, rhs: $self) -> Self::Output {
                let a: &$inner = self.as_ref().into();
                let b: &$inner = rhs.as_ref().into();
                let v: $array = (a + b).into();
                v.into()
            }
        }

        impl ops::Sub<$self> for $self {
            type Output = $self;
            fn sub(self, rhs: $self) -> Self::Output {
                let a: &$inner = self.as_ref().into();
                let b: &$inner = rhs.as_ref().into();
                let v: $array = (a - b).into();
                v.into()
            }
        }

        impl ops::Mul<$base> for $self {
            type Output = $self;
            fn mul(self, arg: $base) -> Self::Output {
                let a: &$inner = self.as_ref().into();
                let v: $array = (arg * a).into();
                v.into()
            }
        }

        impl AsRef<$array> for $self {
            fn as_ref(&self) -> &$array {
                unsafe {
                    mem::transmute(self)
                }
            }
        }

        impl From<$array> for $self {
            fn from(array: $array) -> Self {
                unsafe {
                    mem::transmute(array)
                }
            }
        }

        impl Into<$array> for $self {
            fn into(self) -> $array {
                unsafe {
                    mem::transmute(self)
                }
            }
        }

        impl ApproxEq for $self {
            type Epsilon = <$inner as ApproxEq>::Epsilon;

            fn default_epsilon() -> Self::Epsilon {
                <$inner as ApproxEq>::default_epsilon()
            }

            fn default_max_relative() -> Self::Epsilon {
                <$inner as ApproxEq>::default_max_relative()
            }

            fn default_max_ulps() -> u32 {
                <$inner as ApproxEq>::default_max_ulps()
            }

            fn relative_eq(
                &self,
                other: &Self,
                epsilon: Self::Epsilon,
                max_relative: Self::Epsilon,
            ) -> bool {
                let a: &$inner = self.as_ref().into();
                let b: &$inner = other.as_ref().into();
                a.relative_eq(&b, epsilon, max_relative)
            }

            fn ulps_eq(
                &self,
                other: &Self,
                epsilon: Self::Epsilon,
                max_ulps: u32,
            ) -> bool {
                let a: &$inner = self.as_ref().into();
                let b: &$inner = other.as_ref().into();
                a.ulps_eq(b, epsilon, max_ulps)
            }
        }
    };
}

impl_vector!(Vec2, f32, cgmath::Vector2<f32>, [f32; 2]);
impl_vector!(Vec3, f32, cgmath::Vector3<f32>, [f32; 3]);
impl_vector!(Vec4, f32, cgmath::Vector4<f32>, [f32; 4]);

impl_vector!(DVec2, f64, cgmath::Vector2<f64>, [f64; 2]);
impl_vector!(DVec3, f64, cgmath::Vector3<f64>, [f64; 3]);
impl_vector!(DVec4, f64, cgmath::Vector4<f64>, [f64; 4]);
