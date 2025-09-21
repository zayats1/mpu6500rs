#![no_std]

use nalgebra::Vector3;
pub mod config;
pub mod blocking_driver;
pub mod units;

pub(crate) mod register;

#[derive(Debug,Clone, Copy)]
pub struct IMUmeasruments {
    acceleration: Vector3<f32>,
    angular_velocity: Vector3<f32>,
    temperature: f32,
}

impl defmt::Format for IMUmeasruments{
    fn format(&self, fmt: defmt::Formatter) {
          defmt::write!(fmt, "IMUmeasurments: acceleration {:?}, angular_velocity: {:?}, temperature: {}*C",
           self.acceleration.as_slice(),self.angular_velocity.as_slice(),self.temperature);
    }
}



impl IMUmeasruments {
    pub fn acceleration(&self) -> Vector3<f32> {
        self.acceleration
    }

    pub fn angular_velocity(&self) -> Vector3<f32> {
        self.angular_velocity
    }

    pub fn temperature(&self) -> f32 {
        self.temperature
    }
}
