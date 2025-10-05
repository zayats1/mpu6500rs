#![no_main]
#![no_std]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::i2c::I2c;

use embassy_time::{Delay, Timer};
use heapless::Vec;
use libm::round;

use mpu6500rs::{
    blocking_driver::sensor::Mpu6500,
    config::{self, Config},
};
use mpu6500test as _;

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
            .accel_dlpf_cfg(mpu6500rs::config::AccelDlpfCfg::Dlpf21Hz)
            .gyro_dlpf_cfg(mpu6500rs::config::GyroDlpfCfg::Dlpf5Hz)
            .fifo_sample_rate(mpu6500rs::config::FifoSampleRate::Smpl500Hz),
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
        debug!("Measurments {:?}", val);

        loop {
            let measurments = mpu6500.read(&mut i2c);
            debug!("Measurments {:?}", measurments);
            if let Ok(val) = measurments {
                let acc = val.acceleration();
                let vec: Vec<f32, 3> = acc
                    .iter()
                    .map(|num| round((num * 100.0).into()) as f32 / 100.0)
                    .collect();
                debug!("acc {:?}", vec.as_slice());

                let vel = val.angular_velocity();
                let vec: Vec<f32, 3> = vel
                    .iter()
                    .map(|num| round((num * 100.0).into()) as f32 / 100.0)
                    .collect();
                debug!("vel {:?}", vec.as_slice());
                let temp = val.temperature();
                debug!("temp {}", temp);
                Timer::after_millis(250).await;
            }
        }
    }
}
