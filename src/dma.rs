use core::marker::*;
use core::sync::atomic::*;
use Ordering::*;

mod private {
    use super::*;
    pub trait Sealed {}

    /// Trait to get the equivalent type with shared mutability suitable for dma
    pub trait ToDmaCellInner: Sized {
        type Inner: DmaCellInner<Self> + Sync + Send;
    }

    /// Abstraction to manipulate content of DmaCell.
    pub trait DmaCellInner<OUTER>: ConstDefault {
        fn read(&self) -> OUTER;
        fn write(&self, val: OUTER);
    }
}
use private::*;

/// Trait to initialize with a default value in a const or static context.
pub trait ConstDefault {
    const DEFAULT: Self;
}

#[allow(clippy::declare_interior_mutable_const)]
impl ConstDefault for AtomicU8 {
    const DEFAULT: Self = Self::new(0);
}
#[allow(clippy::declare_interior_mutable_const)]
impl ConstDefault for AtomicU16 {
    const DEFAULT: Self = Self::new(0);
}
#[allow(clippy::declare_interior_mutable_const)]
impl ConstDefault for AtomicU32 {
    const DEFAULT: Self = Self::new(0);
}
#[allow(clippy::declare_interior_mutable_const)]
impl ConstDefault for AtomicI8 {
    const DEFAULT: Self = Self::new(0);
}
#[allow(clippy::declare_interior_mutable_const)]
impl ConstDefault for AtomicI16 {
    const DEFAULT: Self = Self::new(0);
}
#[allow(clippy::declare_interior_mutable_const)]
impl ConstDefault for AtomicI32 {
    const DEFAULT: Self = Self::new(0);
}

impl ToDmaCellInner for u8 {
    type Inner = AtomicU8;
}
impl ToDmaCellInner for u16 {
    type Inner = AtomicU16;
}
impl ToDmaCellInner for u32 {
    type Inner = AtomicU32;
}
impl ToDmaCellInner for i8 {
    type Inner = AtomicI8;
}
impl ToDmaCellInner for i16 {
    type Inner = AtomicI16;
}
impl ToDmaCellInner for i32 {
    type Inner = AtomicI32;
}

impl DmaCellInner<u8> for AtomicU8 {
    fn read(&self) -> u8 {
        self.load(Acquire)
    }
    fn write(&self, val: u8) {
        self.store(val, Release)
    }
}
impl DmaCellInner<u16> for AtomicU16 {
    fn read(&self) -> u16 {
        self.load(Acquire)
    }
    fn write(&self, val: u16) {
        self.store(val, Release)
    }
}
impl DmaCellInner<u32> for AtomicU32 {
    fn read(&self) -> u32 {
        self.load(Acquire)
    }
    fn write(&self, val: u32) {
        self.store(val, Release)
    }
}
impl DmaCellInner<i8> for AtomicI8 {
    fn read(&self) -> i8 {
        self.load(Acquire)
    }
    fn write(&self, val: i8) {
        self.store(val, Release)
    }
}
impl DmaCellInner<i16> for AtomicI16 {
    fn read(&self) -> i16 {
        self.load(Acquire)
    }
    fn write(&self, val: i16) {
        self.store(val, Release)
    }
}
impl DmaCellInner<i32> for AtomicI32 {
    fn read(&self) -> i32 {
        self.load(Acquire)
    }
    fn write(&self, val: i32) {
        self.store(val, Release)
    }
}

#[repr(transparent)]
#[derive(Debug)]
/// Cell allowing to construct a mutably shared dma buffer
pub struct DmaCell<T: ToDmaCellInner>(T::Inner);

impl<T: ToDmaCellInner> DmaCell<T> {
    pub fn read(&self) -> T {
        self.0.read()
    }

    pub fn write(&self, val: T) {
        self.0.write(val);
    }
}

impl<T: ToDmaCellInner> ConstDefault for DmaCell<T> {
    const DEFAULT: Self = Self(T::Inner::DEFAULT);
}

impl<T: ConstDefault, const N: usize> ConstDefault for [T; N] {
    const DEFAULT: Self = [T::DEFAULT; N];
}

/// Marker trait to indicate type that can be uses as dma buffer
pub trait DmaBuffer: Sealed {
    /// Number of dma transfers required to run throught all the buffer
    fn nb_transfer(&self) -> usize {
        //core::mem::size_of_val(self);
        todo!()
    }
}

impl<T: ToDmaCellInner> Sealed for [DmaCell<T>] {}
impl<T: ToDmaCellInner> DmaBuffer for [DmaCell<T>] {}

impl<T: DmaBuffer> Sealed for [T] {}
impl<T: DmaBuffer> DmaBuffer for [T] {}

impl<T: ToDmaCellInner, const N: usize> Sealed for [DmaCell<T>; N] {}
impl<T: ToDmaCellInner, const N: usize> DmaBuffer for [DmaCell<T>; N] {}

impl<T: DmaBuffer, const N: usize> Sealed for [T; N] {}
impl<T: DmaBuffer, const N: usize> DmaBuffer for [T; N] {}

/// Function helping to instanciate a array suitable for dma usage.
pub const fn new_dma_array<T: ConstDefault, const N: usize>() -> [T; N]
where
    [T; N]: DmaBuffer,
{
    [T::DEFAULT; N]
}
