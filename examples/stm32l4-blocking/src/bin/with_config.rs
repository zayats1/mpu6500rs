#![no_main]
#![no_std]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::i2c::I2c;

use embassy_time::{Delay, Timer};
use heapless::Vec;

use stm32l4_blocking as _;
use stm32l4_blocking::round_vec3;
use mpu6500rs::{
    blocking_driver::sensor::Mpu6500,
    config::{self, Config},
};

const ADDRESS: u8 = 0b1101000;
const WHOAMI: u8 = 0x75;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut i2c = I2c::new_blocking(p.I2C2, p.PB13, p.PB14, Default::default());

    let mut data = [0u8; 1];
    info!("Hello");
    i2c.blocking_write(ADDRESS, &[WHOAMI]).ok();
    match i2c.blocking_read(ADDRESS, &mut data) {
        Ok(_) => {
            debug!("Whoami: {}", data[0]);
        }
        Err(e) => error!("err {:?}", e),
    }

    let mut delay = Delay;
    let mpu6500 = Mpu6500::with_configuration(
        ADDRESS,
        &mut i2c,
        &mut delay,
        Config::default()
            .accel_fs_sel(config::AccelFullScaleSelect::G16)
            .gyro_fs_sel(config::GyroFullScaleSelect::Dps250)
            .accel_dlpf_cfg(config::AccelDlpfCfg::Dlpf21Hz)
            .gyro_dlpf_cfg(config::GyroDlpfCfg::Dlpf5Hz)
            .fifo_sample_rate(config::FifoSampleRate::Smpl500Hz),
    );
    let mut mpu6500 = match mpu6500 {
        Ok(device) => device,
        Err(e) => {
            error!("err {:?}", e);
            core::panic!("err")
        }
    };
    // mpu6500.calibrate(&mut i2c, &mut delay).unwrap(); // does not work

    loop {
        let val = mpu6500.read(&mut i2c);
        debug!("Measurements {:?}", val);

        loop {
            let measurements = mpu6500.read(&mut i2c);
            debug!("Measurements {:?}", measurements);
            if let Ok(val) = measurements {
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
        }
    }
}
