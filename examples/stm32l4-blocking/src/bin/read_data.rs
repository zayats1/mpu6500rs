#![no_std]
#![no_main]
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::i2c::I2c;

use embassy_time::{Delay, Timer};
use heapless::Vec;

use mpu6500rs::blocking_driver::sensor::Mpu6500;

use stm32l4_blocking as _;
use stm32l4_blocking ::round_vec3;

const ADDRESS: u8 = 0b1101000;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut i2c = I2c::new_blocking(p.I2C2, p.PB13, p.PB14, Default::default());

    let mut delay = Delay;
    let mpu6500 = Mpu6500::new(ADDRESS, &mut delay, &mut i2c);
    let mut mpu6500 = match mpu6500 {
        Ok(device) => device,
        Err(e) => {
            error!("No device: {:?}", e);
            core::panic!("No device")
        }
    };
    loop {
        let measurements = mpu6500.read(&mut i2c);
        debug!("Measurements {:?}", measurements);
        match measurements {
            Ok(val) => {
                let acc = val.acceleration();
                let vec: Vec<f32, 3> = round_vec3(&acc);

                debug!("acc {:?}", vec.as_slice());

                let vel = val.angular_velocity();
                let vec: Vec<f32, 3> = round_vec3(&vel);

                debug!("vel {:?}", vec.as_slice());
                let temp = val.temperature();
                debug!("temp {}", temp);
                Timer::after_millis(250).await;
            }
            Err(e) => {
               error!("Error: {:?}", e);
            }
        }
    }
}
