use core::error::Error;
use core::fmt;

use core::marker::PhantomData;

use embedded_hal::delay;
use embedded_hal::i2c;
use nalgebra::Vector3;

use crate::config;

use crate::config::Config;
use crate::register::Register;
use crate::units::temperature_from_bytes;
use crate::units::val_from_bytes;

use crate::sync_driver::mpu6500_sys::accel_cfg;
use super::mpu6500_sys::accel_cfg_2;
use super::mpu6500_sys::configure;
use super::mpu6500_sys::enable_interrupt;
use super::mpu6500_sys::fifo_count_h;
use super::mpu6500_sys::fifo_enable;
use super::mpu6500_sys::fifo_read;
use super::mpu6500_sys::gyro_cfg;
use super::mpu6500_sys::i2c_mst_ctrl;
use super::mpu6500_sys::int_pin_bypass_enable_interrupt_cfg;
use super::mpu6500_sys::pwr_mgmt_1;
use super::mpu6500_sys::pwr_mgmt_2;
use super::mpu6500_sys::read_accel_offset;
use super::mpu6500_sys::read_register;
use super::mpu6500_sys::sample_rate_div;
use super::mpu6500_sys::user_ctrl;
use super::mpu6500_sys::who_am_i;
use super::mpu6500_sys::write_accel_offset;
use super::mpu6500_sys::write_gyro_offset;

pub const I2C_ADDR_AL: u8 = 0x68;
pub const I2C_ADDR_AH: u8 = 0x69;

pub const DEV_ID_MPU6500: u8 = 0x70;
pub const DEV_ID_MPU9250: u8 = 0x71;
pub const DEV_ID_MPU9255: u8 = 0x73;

// const CALIB_GYRO_SENSITIVITY: u16 = 131; // LSB/deg/s
const CALIB_ACCEL_SENSITIVITY: u16 = 16384; // LSB/g


#[derive(Debug, defmt::Format)]
pub enum Mpu6500Err<I2C>
where
    I2C: i2c::I2c,
{
    I2cErr(<I2C as embedded_hal::i2c::ErrorType>::Error),
    InvalidDevice(u8),
}

impl<I2C> fmt::Display for Mpu6500Err<I2C>
where
    I2C: i2c::I2c,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Mpu6500Err::I2cErr(e) => write!(f, "I2C error: {:?}", e),
            Mpu6500Err::InvalidDevice(dev_id) => {
                write!(f, "The device is not a MPU6500: {}", dev_id)
            }
        }
    }
}

impl<I2C> Error for Mpu6500Err<I2C> where I2C: i2c::I2c + core::fmt::Debug {}

pub struct Mpu6500<I2C>
where
    I2C: i2c::I2c,
{
    addr: u8,
    cfg: Config,
    pub(crate) acceleration: Vector3<f32>,
    pub(crate) angular_velocity: Vector3<f32>,
    pub(crate) temperature: f32,
    accel_bias: Vector3<f32>,
    gyro_bias: Vector3<f32>,
    accel_resolution: f32,
    gyro_resolution: f32,
    _i2c: PhantomData<I2C>,
}

impl<I2C> Mpu6500<I2C>
where
    I2C: i2c::I2c,
{
    pub fn new<DELAY>(addr: u8, delay: &mut DELAY, i2c: &mut I2C) -> Result<Self, Mpu6500Err<I2C>>
    where
        DELAY: delay::DelayNs,
    {
        Self::with_configuration(addr, i2c, delay, Config::default())
    }

    pub fn with_configuration<DELAY>(
        addr: u8,
        i2c: &mut I2C,
        delay: &mut DELAY,
        cfg: Config,
    ) -> Result<Self, Mpu6500Err<I2C>>
    where
        DELAY: delay::DelayNs,
    {
        let dev_id = who_am_i(addr, i2c).map_err(|e| Mpu6500Err::I2cErr(e))?;

        if dev_id != DEV_ID_MPU6500 && dev_id != DEV_ID_MPU9250 && dev_id != DEV_ID_MPU9255 {
            return Err(Mpu6500Err::InvalidDevice(dev_id));
        }

        let mut mpu = Self {
            addr,
            acceleration: Vector3::default(),
            angular_velocity: Vector3::default(),
            temperature: 0.0,
            accel_bias: Vector3::default(),
            gyro_bias: Vector3::default(),
            accel_resolution: cfg.accel_fs_sel.get_resolution(),
            gyro_resolution: cfg.gyro_fs_sel.get_resolution(),
            cfg,
            _i2c: PhantomData,
        };

        mpu.init(i2c, delay).map_err(|e| Mpu6500Err::I2cErr(e))?;

        Ok(mpu)
    }

    fn init<DELAY>(
        &mut self,
        i2c: &mut I2C,
        delay: &mut DELAY,
    ) -> Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error>
    where
        DELAY: delay::DelayNs,
    {
        pwr_mgmt_1(self.addr, i2c, false, false, false, false, false, 0x00)?;
        delay.delay_ms(100);

        pwr_mgmt_1(self.addr, i2c, false, false, false, false, false, 0x01)?;
        delay.delay_ms(200);

        configure(self.addr, i2c, false, self.cfg.gyro_dlpf_cfg)?;
        sample_rate_div(self.addr, i2c, self.cfg.fifo_sample_rate)?;

        gyro_cfg(self.addr, i2c, self.cfg.gyro_fs_sel, self.cfg.gyro_fchoice)?;

        accel_cfg(self.addr, i2c, self.cfg.accel_fs_sel)?;
        accel_cfg_2(
            self.addr,
            i2c,
            (self.cfg.accel_fchoice & 0x01) == 1,
            self.cfg.accel_dlpf_cfg,
        )?;
        int_pin_bypass_enable_interrupt_cfg(
            self.addr, i2c, false, false, false, false, false, false, true,
        )?;
        enable_interrupt(self.addr, i2c, false, false, false, true)?;
        delay.delay_ms(100);

        Ok(())
    }

    pub(crate) fn read_imu(
        &mut self,
        i2c: &mut I2C,
    ) -> Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error> {
        let mut buf = [0; 14];
        read_register(self.addr, i2c, Register::ACCEL_XOUT_H, &mut buf)?;


        self.acceleration = Vector3::new(
            val_from_bytes([buf[0], buf[1]], self.accel_resolution),
            val_from_bytes([buf[2], buf[3]], self.accel_resolution),
            val_from_bytes([buf[4], buf[5]], self.accel_resolution),
        );

        self.temperature = temperature_from_bytes([buf[6], buf[7]]);

        self.angular_velocity = Vector3::new(
            val_from_bytes([buf[8], buf[9]], self.gyro_resolution),
            val_from_bytes([buf[10], buf[11]], self.gyro_resolution),
            val_from_bytes([buf[12], buf[13]], self.gyro_resolution),
        );

        Ok(())
    }

    pub fn read(
        &mut self,
        i2c: &mut I2C,
    ) -> Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error> {
        self.read_imu(i2c)?;

        Ok(())
    }
    /// the function either shuts down the gyro or makes it inacurate
    pub fn calibrate<DELAY>(
        &mut self,
        i2c: &mut I2C,
        delay: &mut DELAY,
    ) -> Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error>
    where
        DELAY: delay::DelayNs,
    {
        pwr_mgmt_1(self.addr, i2c, false, false, false, false, false, 0x01)?;
        pwr_mgmt_2(self.addr, i2c, false, false, false, false, false, false)?;
        delay.delay_ms(200);

        enable_interrupt(self.addr, i2c, false, false, false, false)?;
        fifo_enable(
            self.addr, i2c, false, false, false, false, false, false, false, false,
        )?;
        pwr_mgmt_1(self.addr, i2c, false, false, false, false, false, 0x00)?;
        i2c_mst_ctrl(self.addr, i2c, false, false, false, false, 0x00)?;
        user_ctrl(self.addr, i2c, false, false, false, false, false, false)?;
        user_ctrl(self.addr, i2c, false, false, true, true, false, false)?;
        delay.delay_ms(15);

        configure(self.addr, i2c, false, config::GyroDlpfCfg::Dlpf184Hz)?;
        sample_rate_div(self.addr, i2c, config::FifoSampleRate::Smpl1000Hz)?;
        gyro_cfg(self.addr, i2c, config::GyroFullScaleSelect::Dps250, 0x00)?;
        accel_cfg(self.addr, i2c, config::AccelFullScaleSelect::G2)?;

        user_ctrl(self.addr, i2c, true, false, false, false, false, false)?;
        fifo_enable(
            self.addr, i2c, false, true, true, true, true, false, false, false,
        )?;
        delay.delay_ms(40);

        fifo_enable(
            self.addr, i2c, false, false, false, false, false, false, false, false,
        )?;

        let fifo_count = fifo_count_h(self.addr, i2c)?;
        let packet_count = fifo_count / 12;

        for _ in 0..packet_count {
            let (accel, gyro) = fifo_read(self.addr, i2c)?;

            self.accel_bias.x += accel.x as f32;
            self.accel_bias.y += accel.y as f32;
            self.accel_bias.z += accel.z as f32;

            self.gyro_bias.x += gyro.x as f32;
            self.gyro_bias.y += gyro.y as f32;
            self.gyro_bias.z += gyro.z as f32;
        }

        if packet_count != 0 {
            self.accel_bias /= packet_count as f32;
            self.gyro_bias /= packet_count as f32;

            if self.accel_bias.z > 0.0 {
                self.accel_bias.z -= CALIB_ACCEL_SENSITIVITY as f32;
            } else {
                self.accel_bias.z += CALIB_ACCEL_SENSITIVITY as f32;
            }
        }

        let mut accel_offset = read_accel_offset(self.addr, i2c)?;
        let mut mask_bit = Vector3::new(1, 1, 1);

        // x

        if accel_offset.x % 2 != 0 {
            mask_bit.x = 0;
        }

        accel_offset.x -= (self.accel_bias.x as i16) >> 3;

        if mask_bit.x != 0 {
            accel_offset.x &= !mask_bit.x;
        } else {
            accel_offset.x |= 0b1;
        }

        // y

        if accel_offset.y % 2 != 0 {
            mask_bit.y = 0;
        }

        accel_offset.y -= (self.accel_bias.y as i16) >> 3;

        if mask_bit.y != 0 {
            accel_offset.y &= !mask_bit.y;
        } else {
            accel_offset.y |= 0b1;
        }

        // z

        if accel_offset.z % 2 != 0 {
            mask_bit.z = 0;
        }

        accel_offset.z -= (self.accel_bias.z as i16) >> 3;

        if mask_bit.z != 0 {
            accel_offset.z &= !mask_bit.z;
        } else {
            accel_offset.z |= 0b1;
        }

        let gyro_offset = Vector3::new(
            -self.gyro_bias.x as i16 / 4,
            -self.gyro_bias.y as i16 / 4,
            -self.gyro_bias.z as i16 / 4,
        );

        write_accel_offset(self.addr, i2c, accel_offset)?;
        write_gyro_offset(self.addr, i2c, gyro_offset)?;

        delay.delay_ms(100);

        self.init(i2c, delay)?;

        Ok(())
    }

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
