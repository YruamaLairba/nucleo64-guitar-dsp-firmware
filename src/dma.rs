use core::marker::*;
use core::sync::atomic::*;
use Ordering::*;

use stm32f4xx_hal::dma::*;

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

impl Sealed for u8 {}
impl Sealed for u16 {}
impl Sealed for u32 {}
impl Sealed for i8 {}
impl Sealed for i16 {}
impl Sealed for i32 {}

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

/// Transfer size of one value.
pub trait DataSize: Sealed {
    const DMA_DATA_SIZE: DmaDataSize;
    const USIZE: usize;
}

impl DataSize for u8 {
    const DMA_DATA_SIZE: DmaDataSize = DmaDataSize::Byte;
    const USIZE: usize = 1;
}
impl DataSize for u16 {
    const DMA_DATA_SIZE: DmaDataSize = DmaDataSize::HalfWord;
    const USIZE: usize = 2;
}
impl DataSize for u32 {
    const DMA_DATA_SIZE: DmaDataSize = DmaDataSize::Word;
    const USIZE: usize = 4;
}
impl DataSize for i8 {
    const DMA_DATA_SIZE: DmaDataSize = DmaDataSize::Byte;
    const USIZE: usize = 1;
}
impl DataSize for i16 {
    const DMA_DATA_SIZE: DmaDataSize = DmaDataSize::HalfWord;
    const USIZE: usize = 2;
}
impl DataSize for i32 {
    const DMA_DATA_SIZE: DmaDataSize = DmaDataSize::Word;
    const USIZE: usize = 4;
}

/// Marker trait to indicate type that can be uses as dma buffer
pub trait DmaBuffer: Sealed + DataSize {
    /// Number of dma transfers required to run throught all the buffer
    fn nb_transfer(&self) -> usize {
        core::mem::size_of_val(self) / Self::USIZE
    }
}

impl<T: ToDmaCellInner + DataSize> Sealed for [DmaCell<T>] {}
impl<T: ToDmaCellInner + DataSize> DataSize for [DmaCell<T>] {
    const DMA_DATA_SIZE: DmaDataSize = T::DMA_DATA_SIZE;
    const USIZE: usize = T::USIZE;
}
impl<T: ToDmaCellInner + DataSize> DmaBuffer for [DmaCell<T>] {}

impl<T: DmaBuffer> Sealed for [T] {}
impl<T: DmaBuffer> DataSize for [T] {
    const DMA_DATA_SIZE: DmaDataSize = T::DMA_DATA_SIZE;
    const USIZE: usize = T::USIZE;
}
impl<T: DmaBuffer> DmaBuffer for [T] {}

impl<T: ToDmaCellInner, const N: usize> Sealed for [DmaCell<T>; N] {}
impl<T: ToDmaCellInner + DataSize, const N: usize> DataSize for [DmaCell<T>; N] {
    const DMA_DATA_SIZE: DmaDataSize = T::DMA_DATA_SIZE;
    const USIZE: usize = T::USIZE;
}
impl<T: ToDmaCellInner + DataSize, const N: usize> DmaBuffer for [DmaCell<T>; N] {}

impl<T: DmaBuffer, const N: usize> Sealed for [T; N] {}
impl<T: DmaBuffer, const N: usize> DataSize for [T; N] {
    const DMA_DATA_SIZE: DmaDataSize = T::DMA_DATA_SIZE;
    const USIZE: usize = T::USIZE;
}
impl<T: DmaBuffer, const N: usize> DmaBuffer for [T; N] {}

/// Function helping to instanciate a array suitable for dma usage.
pub const fn new_dma_array<T: ConstDefault, const N: usize>() -> [T; N]
where
    [T; N]: DmaBuffer,
{
    [T::DEFAULT; N]
}
