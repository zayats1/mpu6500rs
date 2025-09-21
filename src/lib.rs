#![no_std]

use nalgebra::Vector3;
pub mod config;
pub mod sync_driver;
pub mod units;

pub(crate) mod register;

#[derive(Debug,Clone, Copy)]
pub struct IMUmeasruments {
    acceleration: Vector3<f32>,
    angular_velocity: Vector3<f32>,
    temperature: f32,
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
