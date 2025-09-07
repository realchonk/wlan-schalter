#![no_std]
#![no_main]
extern crate panic_halt;

use cyw43::JoinOptions;
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use embassy_executor::Spawner;
use embassy_net::{tcp::TcpSocket, StackResources};
use embassy_rp::{bind_interrupts, clocks::RoscRng, gpio::{Input, Level, Output, Pull}, peripherals::{DMA_CH0, PIO0, USB}, pio::{self, Pio}, usb, watchdog::Watchdog};
use embassy_time::{Duration, Timer, WithTimeout};
use embassy_usb_logger::{LoggerState, UsbLogger};
use static_cell::StaticCell;

const WIFI_NETWORK: &str = "<ssid>";
const WIFI_PASSWORD: &str = "<wpakey>";
const PORT: u16 = 1234;

bind_interrupts! {
    struct Irqs {
        PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
        USBCTRL_IRQ => usb::InterruptHandler<USB>;
    }
}

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<
        'static,
        Output<'static>,
        PioSpi<'static, PIO0, 0, DMA_CH0>
    >,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(
    mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn logger_task(driver: usb::Driver<'static, USB>) {
    static LOGGER: UsbLogger<1024, embassy_usb_logger::DummyHandler> = UsbLogger::with_custom_style(|record, writer| {
        use core::fmt::Write;
        let level = record.level().as_str();
        write!(writer, "[{level}] {}\r\n", record.args()).unwrap();
    });
    unsafe {
        log::set_logger_racy(&LOGGER).unwrap();
        log::set_max_level_racy(log::LevelFilter::Trace);
    }
    LOGGER.run(&mut LoggerState::new(), driver).await;
}

struct Switch<'a> {
    relays: [Output<'a>; 4],
    watchdog: Watchdog,
    led: Output<'a>,
    button: Input<'a>,
}

impl Switch<'_> {
    async fn run(&mut self, socket: &mut TcpSocket<'_>, b: u8) {
        match b {
            b'a' => self.relays[0].set_low(),
            b'b' => self.relays[1].set_low(),
            b'c' => self.relays[2].set_low(),
            b'd' => self.relays[3].set_low(),

            b'A' => self.relays[0].set_high(),
            b'B' => self.relays[1].set_high(),
            b'C' => self.relays[2].set_high(),
            b'D' => self.relays[3].set_high(),

            b'0' => {
                for r in &mut self.relays {
                    r.set_low();
                }
            },

            b'1' => {
                for r in &mut self.relays {
                    r.set_high();
                }
            },
            b'f' => {
                socket.abort();
                let _ = socket.flush().with_timeout(Duration::from_secs(1)).await;
                embassy_rp::rom_data::reset_to_usb_boot(0, 0);
            },
            b'r' => {
                socket.abort();
                let _ = socket.flush().with_timeout(Duration::from_secs(1)).await;
                self.watchdog.trigger_reset();
            },
            b'?' => {
                let mut buf = [0u8; 6];
                for (i, r) in self.relays.iter().enumerate() {
                    match r.get_output_level() {
                        Level::Low => buf[i] = b'a' + i as u8,
                        Level::High => buf[i] = b'A' + i as u8,
                    }
                }

                buf[4] = b'\r';
                buf[5] = b'\n';
                let _ = socket.write(&buf).await;
                return;
            },
            b'\r' | b'\n' => return,
            _ => {
                let _ = socket.write(b"?\r\n").await;
                return;
            }
        }

        let mut level = Level::Low;
        for r in &self.relays {
            if r.get_output_level() == Level::High {
                level = Level::High;
            }
        }
        self.led.set_level(level);
        let _ = socket.write(b".\r\n").await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());
    let usb_driver = usb::Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(usb_driver)).unwrap();

    let mut switch = Switch {
        relays: [
            Output::new(p.PIN_2, Level::Low),
            Output::new(p.PIN_3, Level::Low),
            Output::new(p.PIN_4, Level::Low),
            Output::new(p.PIN_5, Level::Low),
        ],
        watchdog: Watchdog::new(p.WATCHDOG),
        led: Output::new(p.PIN_6, Level::Low),
        button: Input::new(p.PIN_7, Pull::Down),
    };

    // wait for serial to come up
    Timer::after_secs(1).await;

    log::info!("Hello World");

    let mut rng = RoscRng;
    let fw = cyw43_firmware::CYW43_43439A0;
    let clm = cyw43_firmware::CYW43_43439A0_CLM;

    // Set up cyw43
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    log::info!("Starting network stack");

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    spawner.spawn(cyw43_task(runner)).unwrap();

    control.init(clm).await;
    control.set_power_management(cyw43::PowerManagementMode::PowerSave).await;

    let config = embassy_net::Config::dhcpv4(Default::default());
    let seed = rng.next_u64();
    
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let resources = RESOURCES.init(StackResources::new());
    let (stack, runner) = embassy_net::new(net_device, config, resources, seed);
    spawner.spawn(net_task(runner)).unwrap();

    let opts = JoinOptions::new(WIFI_PASSWORD.as_bytes());
    while let Err(e) = control.join(WIFI_NETWORK, opts.clone()).await {
        log::warn!("failed to join {WIFI_NETWORK:?}, status={}", e.status);
    }

    log::info!("waiting for link...");
    stack.wait_link_up().await;

    log::info!("waiting for DHCP...");
    stack.wait_config_up().await;

    let config = stack.config_v4().unwrap();
    log::info!("Acquired DHCP address: {:?}", config.address.address());

    let mut rxbuf = [0u8; 1024];
    let mut txbuf = [0u8; 1024];
    loop {
        let mut socket = TcpSocket::new(stack, &mut rxbuf, &mut txbuf);
        socket.set_keep_alive(Some(Duration::from_secs(3)));
        socket.set_timeout(Some(Duration::from_secs(10)));

        control.gpio_set(0, true).await;
        log::info!("Listening on TCP:{PORT}");
        if let Err(e) = socket.accept(PORT).await {
            log::warn!("accept error: {e:?}");
            continue;
        }

        log::info!("Received connection from {:?}", socket.remote_endpoint());
        control.gpio_set(0, false).await;

        loop {
            let mut buf = [0u8];
            match socket.read(&mut buf).await {
                Ok(0) => {
                    log::info!("end of file reached.");
                    break;
                },
                Ok(_) => {
                    let b = buf[0];
                    log::info!("received byte: {b:?}");
                    switch.run(&mut socket, b).await;
                },
                Err(e) => {
                    log::error!("failed to recv: {e:?}");
                    break;
                },
            }
        }

        socket.abort();
        let _ = socket.flush().await;
    }
}
