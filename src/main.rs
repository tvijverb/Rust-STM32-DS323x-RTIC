
//! CDC-ACM serial port example using cortex-m-rtic.
//! Target board: Blue Pill
#![no_main]
#![no_std]
#![allow(non_snake_case)]


// use panic_halt as _;
// use panic_semihosting as _;
mod usb_serial;
use rtic::app;

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
mod app {
    use panic_rtt_target as _;
    use rtt_target::{rprintln, rtt_init_print};
    use cortex_m::asm::delay;
    use stm32f1xx_hal::pac;
    use stm32f1xx_hal::i2c::{I2c, BlockingI2c, DutyCycle, Mode};
    use stm32f1xx_hal::prelude::*;
    use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
    use stm32f1xx_hal::timer::{Event, CounterMs};
    use stm32f1xx_hal::gpio::{Alternate, OpenDrain};
    use stm32f1xx_hal::gpio::gpiob::{PB6, PB7};
    use stm32f1xx_hal::pac::I2C1;
    use usb_device::prelude::*;
    use bme280::i2c::{BME280};

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: usbd_serial::SerialPort<'static, UsbBusType>,
    }

    #[local]
    struct Local {
        timer_handler: CounterMs<pac::TIM1>,
        bme: BME280<BlockingI2c<I2C1, (PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>)>, cortex_m::delay::Delay>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Init buffers for debug printing
        rtt_init_print!();
        rprintln!("Rust RTIC USB Serial communication");

        // Get access to the device specific peripherals from the peripheral access crate
        let dp = pac::Peripherals::take().unwrap();
        // let dp = pac::Peripherals::take().unwrap();
        let cp = cortex_m::Peripherals::take().unwrap();

        // USB Bus Setup
        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();
        let mut afio = dp.AFIO.constrain();

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

        // RESET USB_D+ to force reconnect
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low();
        delay(clocks.sysclk().raw() / 100);
        // delay(clocks.sysclk().raw() / 100);

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

        // Timer handler for periodic tasks
        rprintln!("Setting timer function");

        // cx.device.TIM1.counter_ms
        let mut timer = cx.device.TIM1.counter_ms(&clocks);
        timer.start(1.secs()).unwrap();
        timer.listen(Event::Update);

        // BME280 I2C Pressure Sensor
        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

        let mut i2c = I2c::i2c1(
            dp.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 400_000.Hz(),
                duty_cycle: DutyCycle::Ratio2to1
            },
            clocks).blocking_default(clocks);

        //Initialize the sensor
        let new_delay = cortex_m::delay::Delay::new(cp.SYST, 1);
        let mut bme = BME280::new_secondary(i2c, new_delay);
        // let mut bme = BME280::new_primary(i2c); 
        
        _ = bme.init();

        rprintln!("Init done!");

        // return shared resources
        (Shared { usb_dev, serial}, Local {timer_handler: timer, bme: bme}, init::Monotonics())
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
    #[task(binds = TIM1_UP, priority = 1, local = [timer_handler, count: u8 = 0])]
    fn tick(cx: tick::Context) {
        // Depending on the application, you could want to delegate some of the work done here to
        // the idle task if you want to minimize the latency of interrupts with same priority (if
        // you have any). That could be done
        // Count used to change the timer update frequency
        rprintln!("Period task! {}", *cx.local.count);

        *cx.local.count += 1;

        // Clears the update flag
        cx.local.timer_handler.clear_interrupt(Event::Update);
    }
}
