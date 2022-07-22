use bitvec::prelude::*;
use deku::{ctx::Endian, prelude::*};

/// Define signed/unsigned integer
macro_rules! def_tuple_struct {
    ($type_name:ident, $bits:expr, $typ:ty) => {
        #[derive(
            Debug, PartialEq, deku::DekuRead, deku::DekuWrite, serde::Serialize, Clone, Copy,
        )]
        #[deku(endian = "_endian", ctx = "_endian: Endian")]
        pub struct $type_name(#[deku(bits = $bits, reader = "Self::reader(deku::rest)")] pub $typ);

        impl $type_name {
            pub fn reader<'a>(
                rest: &'a BitSlice<Msb0, u8>,
            ) -> Result<(&'a BitSlice<Msb0, u8>, $typ), DekuError> {
                if cfg!(feature = "debug_deku")
                    && cfg!(debug_assertions)
                    && std::any::type_name::<$type_name>().contains("")
                {
                    println!("{:x?}", rest.as_raw_slice());
                    println!("{}", std::any::type_name::<$type_name>());
                    let (rest, value) =
                        <$typ>::read(rest, (deku::ctx::Endian::Big, deku::ctx::Size::Bits($bits)))?;
                    println!("{:?}", value);
                    return Ok((rest, value));
                }
                <$typ>::read(rest, (deku::ctx::Endian::Big, deku::ctx::Size::Bits($bits)))
            }
        }
    };
}

/// Define sign-magnitude integer
macro_rules! def_tuple_struct_sm {
    ($type_name:ident, $bits:expr, $typ:ty, $dec_typ:ty) => {
        #[derive(
            Debug, PartialEq, deku::DekuRead, deku::DekuWrite, serde::Serialize, Clone, Copy,
        )]
        #[deku(endian = "_endian", ctx = "_endian: Endian")]
        pub struct $type_name(#[deku(bits = $bits, reader = "Self::reader(deku::rest)", writer = "Self::writer(deku::output, self.0)")] pub $typ);

        impl $type_name {
            pub fn reader<'a>(
                rest: &'a BitSlice<Msb0, u8>,
            ) -> Result<(&'a BitSlice<Msb0, u8>, $typ), DekuError> {
                if cfg!(feature = "debug_deku") && cfg!(debug_assertions) && std::any::type_name::<$type_name>().contains("")
                {
                    println!("{:x?}", rest.as_raw_slice());
                    println!("{}", std::any::type_name::<$type_name>());
                    let (rest, value) = Self::_reader(rest)?;
                    println!("{:?}", value);
                    return Ok((rest, value));
                }
                Self::_reader(rest)
            }

            pub fn writer(
                output: &mut BitVec<Msb0, u8>,
                mut var: $typ,
            ) -> Result<(), DekuError> {
                if var < 0 {
                    var *= -1;
                    let ret = var.write(output, (deku::ctx::Endian::Big, deku::ctx::Size::Bits($bits)));
                    let len = output.len();
                    if let Some(mut bit) = output.get_mut(len - $bits) {
                        *bit = true;
                    }
                    return ret;
                }
                var.write(output, (deku::ctx::Endian::Big, deku::ctx::Size::Bits($bits)))
            }

            pub fn _reader<'a>(
                rest: &'a BitSlice<Msb0, u8>,
            ) -> Result<(&'a BitSlice<Msb0, u8>, $typ), DekuError> {
                let (rest, mut value) =
                    <$dec_typ>::read(rest, (deku::ctx::Endian::Big, deku::ctx::Size::Bits($bits)))?;
                if (value >> ($bits - 1)) & 1 == 1 {
                    value &= !(1 << $bits - 1);
                    return Ok((rest, -(value as $typ)));
                }
                Ok((rest, value as $typ))
            }
        }
    };
}

def_tuple_struct!(U2, 2, u8);
def_tuple_struct!(U3, 3, u8);
def_tuple_struct!(U4, 4, u8);
def_tuple_struct!(U5, 5, u8);
def_tuple_struct!(U6, 6, u8);
def_tuple_struct!(U7, 7, u8);
def_tuple_struct!(U8, 8, u8);
def_tuple_struct!(U9, 9, u16);
def_tuple_struct!(U10, 10, u16);
def_tuple_struct!(U11, 11, u16);
def_tuple_struct!(U12, 12, u16);
def_tuple_struct!(U13, 13, u16);
def_tuple_struct!(U14, 14, u16);
def_tuple_struct!(U16, 16, u16);
def_tuple_struct!(U17, 17, u32);
def_tuple_struct!(U20, 20, u32);
def_tuple_struct!(U24, 24, u32);
def_tuple_struct!(U25, 25, u32);
def_tuple_struct!(U27, 27, u32);
def_tuple_struct!(U30, 30, u32);
def_tuple_struct!(U32, 32, u32);
def_tuple_struct!(U64, 64, u64);

def_tuple_struct!(I6, 6, i8);
def_tuple_struct!(I8, 8, i8);
def_tuple_struct!(I10, 10, i16);
def_tuple_struct!(I11, 11, i16);
def_tuple_struct!(I14, 14, i16);
def_tuple_struct!(I15, 15, i16);
def_tuple_struct!(I16, 16, i16);
def_tuple_struct!(I18, 18, i32);
def_tuple_struct!(I19, 19, i32);
def_tuple_struct!(I20, 20, i32);
def_tuple_struct!(I21, 21, i32);
def_tuple_struct!(I22, 22, i32);
def_tuple_struct!(I24, 24, i32);
def_tuple_struct!(I27, 27, i32);
def_tuple_struct!(I31, 31, i32);
def_tuple_struct!(I32, 32, i32);
def_tuple_struct!(I38, 38, i64);

def_tuple_struct_sm!(IS5, 5, i8, u8);
def_tuple_struct_sm!(IS11, 11, i16, u16);
def_tuple_struct_sm!(IS22, 22, i32, u32);
def_tuple_struct_sm!(IS24, 24, i32, u32);
def_tuple_struct_sm!(IS27, 27, i32, u32);
def_tuple_struct_sm!(IS32, 32, i32, u32);

def_tuple_struct!(Bit1, 1, bool);

#[cfg(test)]
mod tests {
    use super::*;
    use bitvec::slice::BitSlice;

    #[test]
    fn test_sign_magnitude_integer() -> Result<(), DekuError> {
        def_tuple_struct_sm!(S4, 4, i8, u8);

        assert_eq!(
            S4::read(bits![Msb0, u8; 1, 0, 1, 1, 1, 0], deku::ctx::Endian::Big)?,
            (bits![Msb0, u8; 1, 0], S4(-3))
        );
        assert_eq!(
            S4::read(bits![Msb0, u8; 0, 0, 1, 1, 1, 0], deku::ctx::Endian::Big)?,
            (bits![Msb0, u8; 1, 0], S4(3))
        );

        let mut output = bitvec![Msb0, u8; 1, 0];
        S4(-3).write(&mut output, deku::ctx::Endian::Big)?;
        assert_eq!(output, bitvec![Msb0, u8; 1, 0, 1, 0, 1, 1]);

        let mut output = bitvec![Msb0, u8; 1, 0];
        S4(3).write(&mut output, deku::ctx::Endian::Big)?;
        assert_eq!(output, bitvec![Msb0, u8; 1, 0, 0, 0, 1, 1]);

        let mut output = bitvec![Msb0, u8; 1, 0];
        S4(0).write(&mut output, deku::ctx::Endian::Big)?;
        assert_eq!(output, bitvec![Msb0, u8; 1, 0, 0, 0, 0, 0]);

        Ok(())
    }

    #[test]
    fn test_signed_integer() -> Result<(), DekuError> {
        def_tuple_struct!(I4, 4, i8);

        assert_eq!(
            I4::read(bits![Msb0, u8; 1, 0, 1, 1, 1, 0], deku::ctx::Endian::Big)?,
            (bits![Msb0, u8;  1, 0], I4(-5))
        );
        assert_eq!(
            I4::read(bits![Msb0, u8; 0, 0, 1, 1, 1, 0], deku::ctx::Endian::Big)?,
            (bits![Msb0, u8;  1, 0], I4(3))
        );

        let mut output = bitvec![Msb0, u8; 1, 0];
        I4(-5).write(&mut output, deku::ctx::Endian::Big)?;
        assert_eq!(output, bitvec![Msb0, u8; 1, 0, 1, 0, 1, 1]);

        let mut output = bitvec![Msb0, u8; 1, 0];
        I4(3).write(&mut output, deku::ctx::Endian::Big)?;
        assert_eq!(output, bitvec![Msb0, u8; 1, 0, 0, 0, 1, 1]);

        Ok(())
    }

    #[test]
    fn test_unsigned_integer() -> Result<(), DekuError> {
        def_tuple_struct!(U4, 4, u8);

        assert_eq!(
            U4::read(bits![Msb0, u8; 1, 0, 1, 1, 1, 0], deku::ctx::Endian::Big)?,
            (bits![Msb0, u8;  1, 0], U4(11))
        );
        assert_eq!(
            U4::read(bits![Msb0, u8; 0, 0, 1, 1, 1, 0], deku::ctx::Endian::Big)?,
            (bits![Msb0, u8;  1, 0], U4(3))
        );

        let mut output = bitvec![Msb0, u8; 1, 0];
        U4(11).write(&mut output, deku::ctx::Endian::Big)?;
        assert_eq!(output, bitvec![Msb0, u8; 1, 0, 1, 0, 1, 1]);

        let mut output = bitvec![Msb0, u8; 1, 0];
        U4(3).write(&mut output, deku::ctx::Endian::Big)?;
        assert_eq!(output, bitvec![Msb0, u8; 1, 0, 0, 0, 1, 1]);

        Ok(())
    }
}
