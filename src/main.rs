#![no_std]
#![no_main]

use dport::{enable_peripheral, reset_peripheral};
use esp32_hal as hal;
use esp32_hal::target;
use hal::{
    dprintln,
    gpio::{InputPin, InputSignal, OutputPin, OutputSignal},
    prelude::*,
};
use panic_halt as _;
use target::I2C0;
use xtensa_lx_rt::entry;

const I2C_CMD_RESTART: u32 = 0;
const I2C_CMD_WRITE: u32 = 1 << 11;
const I2C_CMD_READ: u32 = 2 << 11;
const I2C_CMD_STOP: u32 = 3 << 11;
const I2C_CMD_END: u32 = 4 << 11;

fn init_pins<SCL: InputPin + OutputPin, SDA: InputPin + OutputPin>(scl: &mut SCL, sda: &mut SDA) {
    fn setup_pin<PIN: InputPin + OutputPin>(pin: &mut PIN) {
        pin.internal_pull_up(true);
        pin.set_output_high(true);
        pin.enable_output(true);
        pin.set_to_open_drain_output();
        pin.enable_input(true);
    }

    setup_pin(scl);
    scl.connect_peripheral_to_output(OutputSignal::I2CEXT0_SCL);
    scl.connect_input_to_peripheral(InputSignal::I2CEXT0_SCL);

    setup_pin(sda);
    sda.connect_peripheral_to_output(OutputSignal::I2CEXT0_SDA);
    sda.connect_input_to_peripheral(InputSignal::I2CEXT0_SDA);
}

fn program_i2c_timing(i2c: &mut I2C0, period: u16) {
    let half_period = period / 2;
    unsafe {
        i2c.scl_high_period.write(|w| w.period().bits(half_period));
        i2c.scl_low_period.write(|w| w.period().bits(half_period));
        i2c.scl_rstart_setup.write(|w| w.time().bits(half_period));
        i2c.scl_start_hold.write(|w| w.time().bits(half_period));
        i2c.scl_stop_hold.write(|w| w.time().bits(half_period));
        i2c.scl_stop_setup.write(|w| w.time().bits(half_period));
        i2c.sda_hold.write(|w| w.time().bits(half_period / 2));
        i2c.sda_sample.write(|w| w.time().bits(half_period / 2));
        i2c.scl_filter_cfg.write(|w| w.scl_filter_thres().bits(7));
        i2c.to.write(|w| w.time_out_reg().bits(period as u32 * 8));
    }

    i2c.scl_filter_cfg
        .modify(|_, w| w.scl_filter_en().set_bit());
}

fn setup_i2c<SCL: InputPin + OutputPin, SDA: InputPin + OutputPin>(
    i2c: &mut I2C0,
    period: u16,
    scl: &mut SCL,
    sda: &mut SDA,
) {
    init_pins(scl, sda);
    program_i2c_timing(i2c, period);
    i2c.fifo_conf.modify(|_, w| w.nonfifo_en().clear_bit());
    i2c.ctr.modify(|_, w| {
        w.rx_lsb_first().clear_bit();
        w.tx_lsb_first().clear_bit();
        w.ms_mode().set_bit();
        w.sample_scl_level().clear_bit();
        w.scl_force_out().set_bit();
        w.sda_force_out().set_bit();
        w
    });
}

fn send_i2c_data(i2c: &mut I2C0, slave_addr: u8, data: &[u8]) {
    // Reset TX FIFO
    i2c.fifo_conf.modify(|_, w| w.tx_fifo_rst().set_bit());
    i2c.fifo_conf.modify(|_, w| w.tx_fifo_rst().clear_bit());

    // Reset RX FIFO
    i2c.fifo_conf.modify(|_, w| w.rx_fifo_rst().set_bit());
    i2c.fifo_conf.modify(|_, w| w.rx_fifo_rst().clear_bit());

    let total_len = data.len() as u32 + 1;
    let commands = [
        I2C_CMD_RESTART,
        total_len | 1 << 8 | I2C_CMD_WRITE,
        I2C_CMD_STOP,
        I2C_CMD_END,
    ];
    let i2c_command_buffer = i2c.comd0.as_ptr();
    unsafe {
        for (idx, cmd) in commands.iter().enumerate() {
            core::ptr::write_volatile(i2c_command_buffer.offset(idx as isize), *cmd);
        }

        i2c.data.write(|w| w.fifo_rdata().bits(slave_addr << 1));
        for b in data.iter() {
            i2c.data.write(|w| w.fifo_rdata().bits(*b));
        }
    }

    i2c.int_clr.modify(|_, w| {
        w.master_tran_comp_int_clr().set_bit();
        w.trans_complete_int_clr().set_bit();
        w.end_detect_int_clr().set_bit();
        w
    });

    i2c.ctr.modify(|_, w| w.trans_start().clear_bit());
    i2c.ctr.modify(|_, w| w.trans_start().set_bit());

    while i2c.int_raw.read().trans_complete_int_raw().bit_is_clear() {}
}

fn send_ssd1306_cmd(i2c: &mut I2C0, cmd: u8) {
    let bytes = [0, cmd];

    send_i2c_data(i2c, 0b0111100, &bytes);
}

#[entry]
fn main() -> ! {
    use hal::dport::Split;
    use hal::serial::{self, Serial};

    let mut dp = target::Peripherals::take().expect("Failed to obtain Peripherals");

    let (_, dport_clock_control) = dp.DPORT.split();
    let mut pins = dp.GPIO.split();

    let clkcntrl = esp32_hal::clock_control::ClockControl::new(
        dp.RTCCNTL,
        dp.APB_CTRL,
        dport_clock_control,
        esp32_hal::clock_control::XTAL_FREQUENCY_AUTO,
    )
    .unwrap();
    let (clkcntrl_config, mut watchdog) = clkcntrl.freeze().unwrap();
    watchdog.disable();

    let (_, _, _, mut watchdog0) = esp32_hal::timer::Timer::new(dp.TIMG0, clkcntrl_config);
    let (_, _, _, mut watchdog1) = esp32_hal::timer::Timer::new(dp.TIMG1, clkcntrl_config);
    watchdog0.disable();
    watchdog1.disable();

    // Setup serial so dprint!() works.
    let _serial: Serial<_, _, _> = Serial::new(
        dp.UART0,
        serial::Pins {
            tx: pins.gpio1,
            rx: pins.gpio3,
            cts: None,
            rts: None,
        },
        serial::config::Config {
            baudrate: 115200.Hz(),
            ..serial::config::Config::default()
        },
        clkcntrl_config,
    )
    .unwrap();

    dprintln!("init done. APB freq: {}\r", clkcntrl_config.apb_frequency());

    enable_peripheral(Peripheral::I2C0);
    reset_peripheral(Peripheral::I2C0);

    let i2c_period = u32::from(clkcntrl_config.apb_frequency() / 100_000);
    use core::convert::TryInto;
    setup_i2c(
        &mut dp.I2C0,
        i2c_period.try_into().unwrap(),
        &mut pins.gpio4,
        &mut pins.gpio5,
    );

    send_i2c_data(&mut dp.I2C0, 0b0111100, &[0, 0xAE, 0xD5, 0x80, 0xA8]);
    send_ssd1306_cmd(&mut dp.I2C0, 63);

    send_i2c_data(&mut dp.I2C0, 0b0111100, &[0, 0xD3, 0, 0x40, 0x8D]);
    send_ssd1306_cmd(&mut dp.I2C0, 0x14);

    send_i2c_data(&mut dp.I2C0, 0b0111100, &[0, 0x20, 0, 0xa1, 0xc8]);

    send_ssd1306_cmd(&mut dp.I2C0, 0xDA);
    send_ssd1306_cmd(&mut dp.I2C0, 0x12);
    send_ssd1306_cmd(&mut dp.I2C0, 0x81);
    send_ssd1306_cmd(&mut dp.I2C0, 0xCF);

    send_ssd1306_cmd(&mut dp.I2C0, 0xD9);
    send_ssd1306_cmd(&mut dp.I2C0, 0xF1);

    send_i2c_data(
        &mut dp.I2C0,
        0b0111100,
        &[0, 0xDB, 0x40, 0xA4, 0xA7, 0x2E, 0xAF],
    );

    send_i2c_data(&mut dp.I2C0, 0b0111100, &[0, 0x22, 0, 0xff]);
    send_i2c_data(&mut dp.I2C0, 0b0111100, &[0, 0x21, 0, 127]);

    let rust_logo = include_bytes!("logo.raw");
    for y in (0..64).step_by(8) {
        for x in (0..16).step_by(2) {
            let mut line = [0u8; 17];
            line[0] = 0x40;
            if 4 <= x && x < 12 {
                for j in 0..16 {
                    for k in 0..8 {
                        use bit::BitIndex;
                        line[j + 1] |= if rust_logo[(k + y) * 8 + j / 8 + (x - 4)].bit(7 - j % 8) {
                            1 << k
                        } else {
                            0
                        };
                    }
                }
            }

            send_i2c_data(&mut dp.I2C0, 0b0111100, &line);
        }
    }

    dprintln!("done\r");
    loop {}
}
