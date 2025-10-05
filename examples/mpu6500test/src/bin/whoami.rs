#![no_main]
#![no_std]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::i2c::I2c;

use embassy_time::{Delay};


use mpu6500rs::blocking_driver::sensor::Mpu6500;
use mpu6500test as _;

const ADDRESS: u8 = 0b1101000;


#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut i2c = I2c::new_blocking(p.I2C2, p.PB13, p.PB14, Default::default());


    let mut delay = Delay;
    let mpu6500 = Mpu6500::new(ADDRESS, &mut delay, &mut i2c);
    let mpu6500 = match mpu6500 {
        Ok(device) => device,
        Err(e) => {
            error!("err {:?}", e);
            core::panic!("err")
        }
    };

    let whoami = mpu6500.whoami(&mut i2c);
    match whoami {
    Ok(i_am) => {
        info!("Whoami: {}",i_am);
        return;
    }
    Err(e) => error!("{:?}",e),
   } 
}
