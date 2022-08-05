
//! CDC-ACM serial port example using cortex-m-rtic.
//! Target board: Blue Pill
#![no_main]
#![no_std]
#![allow(non_snake_case)]
#![feature(alloc_error_handler)]

#[macro_use]
extern crate alloc;


// use panic_halt as _;
// use panic_semihosting as _;
mod usb_serial;
use rtic::app;
use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
mod app {
    use panic_halt as _;
    // use panic_rtt_target as _;
    // use rtt_target::{rprintln, rtt_init_print};
    use cortex_m::asm::{delay, wfi};
    use stm32f1xx_hal::pac;
    use stm32f1xx_hal::i2c::{BlockingI2c, Mode, DutyCycle};
    use stm32f1xx_hal::prelude::*;
    use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
    use stm32f1xx_hal::timer::{Event, CounterMs};
    use stm32f1xx_hal::gpio::{Alternate, OpenDrain};
    use stm32f1xx_hal::gpio::gpiob::{PB6, PB7};
    use stm32f1xx_hal::pac::{I2C1};
    use usb_device::prelude::*;
    use ds323x::{Ds323x, DateTimeAccess, interface::I2cInterface};

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: usbd_serial::SerialPort<'static, UsbBusType>,
    }


    #[local]
    struct Local {
        timer_handler: CounterMs<pac::TIM1>,
        rtc: Ds323x<I2cInterface<BlockingI2c<I2C1, (PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>)>>,ds323x::ic::DS3231>
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Init buffers for debug printing
        // rtt_init_print!();

        // Allocator setup
        {
            use core::mem::MaybeUninit;
            const HEAP_SIZE: usize = 1024;
            static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            unsafe { super::ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
        }      

        // USB Bus Setup
        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();
        let mut afio = cx.device.AFIO.constrain();

        // Set clocks for USB communication
        let clocks = rcc
            .cfgr
            .use_hse(16.MHz())
            .sysclk(48.MHz())
            .pclk1(24.MHz())
            .freeze(&mut flash.acr);

        // check clocks are configured correctly for USB
        assert!(clocks.usbclk_valid());

        // use GPIO A for USB, GPIO B for I2C_1
        let mut gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split();

        // Timer handler for periodic tasks
        // rprintln!("Setting timer function");

        // cx.device.TIM1.counter_ms
        let mut timer = cx.device.TIM1.counter_ms(&clocks);
        timer.start(60.secs()).unwrap();
        timer.listen(Event::Update);

        // I2c setup
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

        let i2c = BlockingI2c::i2c1(
            cx.device.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 400000.Hz(),
                duty_cycle: DutyCycle::Ratio16to9
            },
            clocks,
            5000,
            10,
            5000,
            5000
        );

        //Initialize the DS3231 I2C sensor
        let mut rtc = Ds323x::new_ds3231(i2c);
        // Optionally set Datetime
        // let datetime = NaiveDate::from_ymd(2022, 8, 5).and_hms(17, 41, 00);
        // rtc.set_datetime(&datetime).unwrap();
        rtc.enable().unwrap(); // set clock to run    
        // rprintln!("Init done!");

        // RESET USB_D+ to force reconnect
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low();
        delay(clocks.sysclk().raw() / 100);

        // SET USB pins
        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        let usb = Peripheral {
            usb: cx.device.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        unsafe {
            USB_BUS.replace(UsbBus::new(usb));
        }


        // Create Serial device
        let serial = usbd_serial::SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });

        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x16c0, 0x27dd),
        )
        .manufacturer("Datastem")
        .product("Datastem Serial port")
        .serial_number("1")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

        // return shared resources
        (Shared { usb_dev, serial}, Local {timer_handler: timer, rtc}, init::Monotonics())
    }

    // USB TRANSMIT
    #[task(binds = USB_HP_CAN_TX, shared = [usb_dev, serial])]
    fn usb_tx(cx: usb_tx::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;

        (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
            super::usb_serial::usb_poll(usb_dev, serial);
        });
    }

    // USB RECIEVE
    #[task(binds = USB_LP_CAN_RX0, shared = [usb_dev, serial])]
    fn usb_rx0(cx: usb_rx0::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;

        (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
            super::usb_serial::usb_poll(usb_dev, serial);
        });
    }

    // Period task
    #[task(binds = TIM1_UP, priority = 1, local = [rtc, timer_handler, count: u8 = 0], shared = [serial])]
    fn tick(cx: tick::Context) {
        // Count used to change the timer update frequency
        // rprintln!("Periodic Task #{}", *cx.local.count);
        *cx.local.count += 1;

        // let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;

        // DS3231 Measurement
        let rtc = cx.local.rtc;
        match rtc.temperature() {
            Ok(temperature) => {
                // let mut buf = [0u8; 64];
                (&mut serial).lock(|serial| {
                    let str_temp = format!("Current temperature = {}\r\n", temperature);
                    serial.write(str_temp.as_bytes()).ok();
                });
            }
            Err(_) => { }
        }
        match rtc.datetime() {
            Ok(datetime) => {
                (&mut serial).lock(|serial| {
                    let str_temp = format!("Current datetime = {}\r\n", datetime);
                    serial.write(str_temp.as_bytes()).ok();
                });
            }
            Err(_) => { }
        }

        // Clears the update flag
        cx.local.timer_handler.clear_interrupt(Event::Update);
    }

    #[idle(local = [x: u32 = 0])]
    fn idle(cx: idle::Context) -> ! {
        // Locals in idle have lifetime 'static
        // Now Wait For Interrupt is used instead of a busy-wait loop
        // to allow MCU to sleep between interrupts
        // https://developer.arm.com/documentation/ddi0406/c/Application-Level-Architecture/Instruction-Details/Alphabetical-list-of-instructions/WFI
        loop {
            wfi();
        }
    }
}
