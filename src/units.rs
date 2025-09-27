use core::f32;

const G_UNIT:f32 = 9.80665;


/// parses some value from be bytes : like acceleration and angular velocity
pub const fn val_from_bytes(bytes: [u8; 2], resolution: f32) -> f32 {
    i16::from_be_bytes(bytes) as f32 * resolution
}
/// converts be bytes to degreess of  C  1e-2 precision
pub fn temperature_from_bytes(bytes: [u8; 2]) -> f32 {
    i16::from_be_bytes(bytes) as f32 / 333.87 + 21.0
}
/// the default output of the IMU is deg/s, but it is easy to convert the value to rad/s
pub const fn deg_to_rad(deg: f32) -> f32 {
    deg * f32::consts::PI / 180.0
}

// the default output of the IMU is deg/s, but it is easy to convert the value to rad/s
pub const fn  g_to_m_s2(acc_g: f32) -> f32 {
   acc_g*G_UNIT
}



#[cfg(test)]
mod unit_tests {
    extern crate std;
    use crate::config::GyroFullScaleSelect;

    use super::*;
    const TOLLERANCE: f32 = 1e-5;
    const NO_SO_PRECISE: f32 = 1e-2;
    fn assert_almost_eq(original: f32, expected: f32, tollerance: f32) {
        let diff = (original - expected).abs();
        assert!(diff <= tollerance)
    }

    #[test]
    fn g_to_m_s2_test(){
         let g = 1.0;
         let expected = G_UNIT;
         let diff = (g_to_m_s2(g) - expected).abs();
         assert!(diff <= TOLLERANCE)
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
