use core::f32;

pub const fn val_from_bytes(bytes: [u8; 2], resolution: f32) -> f32 {
    i16::from_be_bytes(bytes) as f32 * resolution
}

pub fn temperature_from_bytes(bytes: [u8; 2]) -> f32 {
    i16::from_be_bytes(bytes) as f32 / 333.87 + 21.0
}

pub const fn deg_to_rad(deg: f32) -> f32 {
    deg * f32::consts::PI / 180.0
}

#[cfg(test)]
mod unit_tests {
    extern crate std;
    use crate::mpu6500::config::GyroFullScaleSelect;

    use super::*;
    const TOLLERANCE: f32 = 1e-5;
    const NO_SO_PRECISE: f32 = 1e-2;
    fn assert_almost_eq(original: f32, expected: f32, tollerance: f32) {
        let diff = (original - expected).abs();
        assert!(diff <= tollerance)
    }

    #[test]
    fn deg_to_rad_test() {
        let rad = deg_to_rad(0.0);
        assert_almost_eq(rad, 0.0, TOLLERANCE);
        let rad = deg_to_rad(45.0);
        assert_almost_eq(rad, f32::consts::PI / 4.0, TOLLERANCE);
        let rad = deg_to_rad(90.0);
        assert_almost_eq(rad, f32::consts::PI / 2.0, TOLLERANCE);
        let rad = deg_to_rad(180.0);
        assert_almost_eq(rad, f32::consts::PI, TOLLERANCE);
    }
    #[test]
    fn temperature_from_bytes_test() {
        let bytes = 1336i16.to_be_bytes(); //  25 degree C
        assert_almost_eq(temperature_from_bytes(bytes), 25.00, NO_SO_PRECISE);
        let bytes = (-8680i16).to_be_bytes(); //  -5 degree C
        assert_almost_eq(temperature_from_bytes(bytes), -5.00, NO_SO_PRECISE);
         let bytes = (-7011i16).to_be_bytes(); //  0 degree C
        assert_almost_eq(temperature_from_bytes(bytes), 0.00, NO_SO_PRECISE);
    }

        #[test]
    fn val_from_bytes_test() {
        let resolution = GyroFullScaleSelect::Dps500.get_resolution(); // 500.0 / 32768.0,
        let bytes = 65i16.to_be_bytes();
        assert_almost_eq(val_from_bytes(bytes,resolution), 1.0, NO_SO_PRECISE);
    }
}
