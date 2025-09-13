
use embedded_hal::i2c;
use nalgebra::Vector3;

use crate::sync_driver::config;
use crate::sync_driver::register::Register;

/// Registers 19 to 24 – Gyro Offset Registers
pub (crate)fn write_gyro_offset<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
    gyro_offset: Vector3<i16>,
) -> Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error> {
    let x = gyro_offset.x.to_be_bytes();
    write_register(addr, i2c, Register::XG_OFFSET_H, x[0])?;
    write_register(addr, i2c, Register::XG_OFFSET_L, x[1])?;

    let y = gyro_offset.y.to_be_bytes();
    write_register(addr, i2c, Register::YG_OFFSET_H, y[0])?;
    write_register(addr, i2c, Register::YG_OFFSET_L, y[1])?;

    let z = gyro_offset.z.to_be_bytes();
    write_register(addr, i2c, Register::ZG_OFFSET_H, z[0])?;
    write_register(addr, i2c, Register::ZG_OFFSET_L, z[1])?;

    Ok(())
}

/// Register 25 – Sample Rate Divider
pub (crate)fn sample_rate_div<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
    smplrt_div: config::FifoSampleRate,
) -> Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error> {
    write_register(addr, i2c, Register::SMPLRT_DIV, smplrt_div as u8)
}

/// Register 26 – Configuration
pub (crate)fn configure<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
    fifo_mode: bool,
    dlpf_cfg: config::GyroDlpfCfg,
) -> Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error> {
    write_register(
        addr,
        i2c,
        Register::CFG,
        (fifo_mode as u8) << 6 | (dlpf_cfg as u8) << 0,
    )
}

/// Register 27 – Gyroscope Configuration
pub (crate)fn gyro_cfg<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
    gyro_fs_sel: config::GyroFullScaleSelect,
    fchoice_b: u8,
) -> Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error> {
    let mut c = [0; 1];
    read_register(addr, i2c, Register::GYRO_CFG, &mut c)?;

    write_register(
        addr,
        i2c,
        Register::GYRO_CFG,
        c[0] & 0b11100111 | (gyro_fs_sel as u8) << 3 | (fchoice_b & 0b11) << 0,
    )
}

/// Register 28 – Accelerometer Configuration
pub (crate)fn accel_cfg<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
    accel_fs_sel: config::AccelFullScaleSelect,
) -> Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error> {
    let mut c = [0; 1];
    read_register(addr, i2c, Register::ACCEL_CFG_2, &mut c)?;

    write_register(
        addr,
        i2c,
        Register::ACCEL_CFG,
        c[0] & 0b11100111 | (accel_fs_sel as u8) << 3,
    )
}

/// Register 29 – Accelerometer Configuration 2
pub (crate)fn accel_cfg_2<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
    accel_fchoice_b: bool,
    a_dlpf_cfg: config::AccelDlpfCfg,
) -> Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error> {
    let mut c = [0; 1];
    read_register(addr, i2c, Register::ACCEL_CFG_2, &mut c)?;

    write_register(
        addr,
        i2c,
        Register::ACCEL_CFG_2,
        c[0] & 0b11110000 | (accel_fchoice_b as u8) << 3 | (a_dlpf_cfg as u8) << 0,
    )
}

/// Register 35 – FIFO Enable
pub (crate)fn fifo_enable<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
    temp_out: bool,
    gyro_xout: bool,
    gyro_yout: bool,
    gyro_zout: bool,
    accel: bool,
    slv_2: bool,
    slv_1: bool,
    slv_0: bool,
) -> Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error> {
    write_register(
        addr,
        i2c,
        Register::FIFO_EN,
        (temp_out as u8) << 7
            | (gyro_xout as u8) << 6
            | (gyro_yout as u8) << 5
            | (gyro_zout as u8) << 4
            | (accel as u8) << 3
            | (slv_2 as u8) << 2
            | (slv_1 as u8) << 1
            | (slv_0 as u8) << 0,
    )
}

/// Register 36 – I2C Master Control
pub (crate)fn i2c_mst_ctrl<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
    mult_mst_en: bool,
    wait_for_es: bool,
    slv_3_fifo_en: bool,
    i2c_mst_p_nsr: bool,
    i2c_mst_clk: u8,
) -> Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error> {
    write_register(
        addr,
        i2c,
        Register::I2C_MST_CTRL,
        (mult_mst_en as u8) << 7
            | (wait_for_es as u8) << 6
            | (slv_3_fifo_en as u8) << 5
            | (i2c_mst_p_nsr as u8) << 4
            | (i2c_mst_clk & 0b1111) << 0,
    )
}

/// Register 55 – INT Pin / Bypass Enable Configuration
pub (crate)fn int_pin_bypass_enable_interrupt_cfg<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
    actl: bool,
    open: bool,
    latch_int_en: bool,
    int_anyrd_2clear: bool,
    actl_fsync: bool,
    fsync_int_mode_en: bool,
    bypass_en: bool,
) -> Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error> {
    write_register(
        addr,
        i2c,
        Register::INT_PIN_CFG,
        (actl as u8) << 7
            | (open as u8) << 6
            | (latch_int_en as u8) << 5
            | (int_anyrd_2clear as u8) << 4
            | (actl_fsync as u8) << 3
            | (fsync_int_mode_en as u8) << 2
            | (bypass_en as u8) << 1,
    )
}

/// Register 56 – Interrupt Enable
pub (crate)fn enable_interrupt<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
    wom_en: bool,
    fifo_overflow_en: bool,
    fsync_int_en: bool,
    raw_rdy_en: bool,
) -> Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error> {
    write_register(
        addr,
        i2c,
        Register::INT_ENABLE,
        (wom_en as u8) << 6
            | (fifo_overflow_en as u8) << 4
            | (fsync_int_en as u8) << 3
            | (raw_rdy_en as u8) << 0,
    )
}

/// Register 106 – User Control
pub (crate)fn user_ctrl<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
    fifo_en: bool,
    i2c_mst_en: bool,
    i2c_if_dis: bool,
    fifo_rst: bool,
    i2c_mst_rst: bool,
    sig_cond_rst: bool,
) -> Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error> {
    write_register(
        addr,
        i2c,
        Register::USER_CTRL,
        (fifo_en as u8) << 6
            | (i2c_mst_en as u8) << 5
            | (i2c_if_dis as u8) << 4
            | (fifo_rst as u8) << 2
            | (i2c_mst_rst as u8) << 1
            | (sig_cond_rst as u8) << 0,
    )
}

/// Register 107 – Power Management 1
pub (crate)fn pwr_mgmt_1<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
    h_reset: bool,
    sleep: bool,
    cycle: bool,
    gyro_standby: bool,
    pd_ptat: bool,
    clksel: u8,
) -> Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error> {
    write_register(
        addr,
        i2c,
        Register::PWR_MGMT_1,
        (h_reset as u8) << 7
            | (sleep as u8) << 6
            | (cycle as u8) << 5
            | (gyro_standby as u8) << 4
            | (pd_ptat as u8) << 3
            | (clksel & 0b111) << 0,
    )
}

// Register 108 – Power Management 2
pub (crate)fn pwr_mgmt_2<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
    disable_xa: bool,
    disable_ya: bool,
    disable_za: bool,
    disable_xg: bool,
    disable_yg: bool,
    disable_zg: bool,
) -> Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error> {
    write_register(
        addr,
        i2c,
        Register::PWR_MGMT_2,
        (disable_xa as u8) << 5
            | (disable_ya as u8) << 4
            | (disable_za as u8) << 3
            | (disable_xg as u8) << 2
            | (disable_yg as u8) << 1
            | (disable_zg as u8) << 0,
    )
}

/// Register 114 – FIFO Count High
pub (crate)fn fifo_count_h<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
) -> Result<u16, <I2C as embedded_hal::i2c::ErrorType>::Error> {
    let mut buf = [0; 2];
    read_register(addr, i2c, Register::FIFO_COUNTH, &mut buf)?;
    Ok(u16::from_be_bytes(buf))
}

/// Register 116 – FIFO Read Write
pub (crate)fn fifo_read<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
) -> Result<(Vector3<i16>, Vector3<i16>), <I2C as embedded_hal::i2c::ErrorType>::Error> {
    let mut buf = [0; 12];
    read_register(addr, i2c, Register::FIFO_R_W, &mut buf)?;
    Ok((
        Vector3::new(
            i16::from_be_bytes([buf[0], buf[1]]),
            i16::from_be_bytes([buf[2], buf[3]]),
            i16::from_be_bytes([buf[4], buf[5]]),
        ),
        Vector3::new(
            i16::from_be_bytes([buf[6], buf[7]]),
            i16::from_be_bytes([buf[8], buf[9]]),
            i16::from_be_bytes([buf[10], buf[11]]),
        ),
    ))
}

/// Register 117 – Who Am I
pub (crate)fn who_am_i<I2C: i2c::I2c>(addr: u8, i2c: &mut I2C) -> Result<u8, <I2C as embedded_hal::i2c::ErrorType>::Error> {
    let mut buf = [0; 1];
    read_register(addr, i2c, Register::WHO_AM_I, &mut buf)?;
    Ok(buf[0])
}

/// Registers 119, 120, 122, 123, 125, 126 – Accelerometer Offset Registers
pub (crate)fn read_accel_offset<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
) -> Result<Vector3<i16>, <I2C as embedded_hal::i2c::ErrorType>::Error> {
    let mut buf = [0; 2];
    read_register(addr, i2c, Register::XA_OFFSET_H, &mut buf)?;
    let x = i16::from_be_bytes(buf);

    let mut buf = [0; 2];
    read_register(addr, i2c, Register::YA_OFFSET_H, &mut buf)?;
    let y = i16::from_be_bytes(buf);

    let mut buf = [0; 2];
    read_register(addr, i2c, Register::ZA_OFFSET_H, &mut buf)?;
    let z = i16::from_be_bytes(buf);

    Ok(Vector3::new(x, y, z))
}

/// Registers 119, 120, 122, 123, 125, 126 – Accelerometer Offset Registers
pub (crate)fn write_accel_offset<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
    accel_offset: Vector3<i16>,
) -> core::result::Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error> {
    let x = accel_offset.x.to_be_bytes();
    write_register(addr, i2c, Register::XA_OFFSET_H, x[0])?;
    write_register(addr, i2c, Register::XA_OFFSET_L, x[1])?;

    let y = accel_offset.y.to_be_bytes();
    write_register(addr, i2c, Register::YA_OFFSET_H, y[0])?;
    write_register(addr, i2c, Register::YA_OFFSET_L, y[1])?;

    let z = accel_offset.z.to_be_bytes();
    write_register(addr, i2c, Register::ZA_OFFSET_H, z[0])?;
    write_register(addr, i2c, Register::ZA_OFFSET_L, z[1])?;

    Ok(())
}

pub (crate)fn read_register<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
    reg: Register,
    buf: &mut [u8],
) -> core::result::Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error> {
    // i2c.write(addr, &[reg as u8])?;
    // i2c.read(addr, buf)?;
    i2c.write_read(addr, &[reg as u8], buf)?;
    Ok(())
}

pub (crate)fn write_register<I2C: i2c::I2c>(
    addr: u8,
    i2c: &mut I2C,
    reg: Register,
    cmd: u8,
) -> core::result::Result<(), <I2C as embedded_hal::i2c::ErrorType>::Error> {
    i2c.write(addr, &[reg as u8, cmd])
}
