//! Blinks an LED
//!
//! This assumes that a LED is connected to pc13 as is the case on the blue pill board.
//!
//! Note: Without additional hardware, PC13 should not be used to drive an LED, see page 5.1.2 of
//! the reference manual for an explanation. This is not an issue on the blue pill.
#![no_main]
// #![deny(unsafe_code)]
#![no_std]

// mod serial_connection;

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use cortex_m_rt::entry;
use cortex_m::asm::{wfi};
use stm32f1xx_hal::{pac, prelude::*};
use stm32f1xx_hal::usb::{Peripheral, UsbBus};
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

#[entry]
fn main() -> ! {
    // Init buffers for debug printing
    rtt_init_print!();
    rprintln!("Hello, Rust!");
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    // // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // // `clocks`
    // let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let clocks = rcc
        .cfgr
        .use_hse(16.MHz())
        .sysclk(48.MHz())
        .pclk1(24.MHz())
        .freeze(&mut flash.acr);

    assert!(clocks.usbclk_valid());

    // serial_connection::set_interrupt();

    let mut delay = cp.SYST.delay(&clocks);

    // // Acquire the GPIOC peripheral
    let mut gpioc = dp.GPIOC.split();

    let mut gpioa = dp.GPIOA.split();

    // // BluePill board has a pull-up resistor on the D+ line.
    // // Pull the D+ pin down to send a RESET condition to the USB bus.
    // // This forced reset is needed only for development, without it host
    // // will not reset your device when you upload new firmware.
    let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
    usb_dp.set_low();
    delay.delay_ms(100_u16);


    let usb = Peripheral {
        usb: dp.USB,
        pin_dm: gpioa.pa11,
        pin_dp: usb_dp.into_floating_input(&mut gpioa.crh),
    };
    let usb_bus = UsbBus::new(usb);

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Datastem")
        .product("Datastem Serial port")
        .serial_number("1")
        .device_class(USB_CLASS_CDC)
        .build();

    rprintln!("Hello, Rust!");
    // Wait for the timer to trigger an update and change the state of the LED
    let mut i = 0;
    let x: i16 = 42;

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut buf = [0u8; 64];

        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }

                let mut write_offset = 0;
                while write_offset < count {
                    match serial.write(&buf[write_offset..count]) {
                        Ok(len) if len > 0 => {
                            write_offset += len;
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }

    loop {
        wfi();
        delay.delay_ms(1_000_u16);
        wfi();
        delay.delay_ms(1_000_u16);
        i += 1;
        rprintln!("Hello again; I have blinked {} times. {}", i, x);
        if i == 100 {
            panic!("Yow, 100 times is enough!");
        }
    }
}
