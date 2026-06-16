#![no_main]
#![no_std]

mod board;

use board::Board;
use defmt::info;
use defmt_semihosting as _;
use embassy_executor::Spawner;
use embassy_qemu_riscv::gpio::{Async as GpioAsync, Input};
use embassy_qemu_riscv::i2c::target::{Async as I2cAsync, I2c};
use embassy_qemu_riscv::uart::{buffered, Async};
use embedded_mcu_hal::i2c::target::{Request, WriteStatus};
use platform_common::board::BoardIo;
use platform_common::mock::MockOdpRelayHandler;
use semihosting as _; // Panic handler
use static_cell::StaticCell;

#[embassy_executor::task]
async fn uart_service(uart: buffered::Uart<'static, Async>, relay: MockOdpRelayHandler) {
    info!("Starting uart service");
    static UART_SERVICE: StaticCell<uart_service::DefaultService<MockOdpRelayHandler>> = StaticCell::new();
    let uart_service = uart_service::DefaultService::default_smbusespi(relay).unwrap();
    let uart_service = UART_SERVICE.init(uart_service);
    let Err(e) = uart_service::task::uart_service(uart_service, uart).await;
    panic!("uart-service error: {:?}", e);
}

// THROWAWAY: dump whatever the host pokes at us over I2C.
#[embassy_executor::task]
async fn i2c_debug(mut i2c: I2c<'static, I2cAsync>) {
    info!("Starting I2C debug listener on 0x{:02x}", board::EC_I2C_ADDR);
    let mut buf = [0u8; 64];
    loop {
        match i2c.listen().await {
            Request::Write(addr) => {
                let status = i2c.respond_to_write(&mut buf).await;
                let n = match status {
                    WriteStatus::Stopped(n) | WriteStatus::Restarted(n) | WriteStatus::BufferFull(n) => n,
                    _ => 0,
                };
                info!("I2C write to 0x{:02x}: {} bytes {:?}", addr, n, &buf[..n]);
            }
            Request::Read(addr) => {
                info!("I2C read from 0x{:02x}", addr);
                // Feed some filler so the controller doesn't stall.
                let _ = i2c.respond_to_read(&[0xFF; 4]).await;
            }
            Request::RepeatedStart(addr) => info!("I2C repeated-start (0x{:02x})", addr),
            Request::Stop(addr) => info!("I2C stop (0x{:02x})", addr),
            _ => info!("I2C other event"),
        }
    }
}

// THROWAWAY: dump GPIO0 (HID interrupt line) edges.
#[embassy_executor::task]
async fn gpio_debug(mut gpio: Input<'static, GpioAsync>) {
    info!("Starting GPIO debug listener on GPIO0, level={}", gpio.is_high());
    loop {
        gpio.wait_for_any_edge().await;
        info!("GPIO0 edge: level={}", gpio.is_high());
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_qemu_riscv::init();
    let board = Board::init(p);

    let relay = platform_common::mock::init(spawner).await;
    spawner.spawn(uart_service(board.uart, relay).expect("Failed to spawn UART service task"));
    spawner.spawn(i2c_debug(board.i2c).expect("Failed to spawn I2C debug task"));
    spawner.spawn(gpio_debug(board.gpio).expect("Failed to spawn GPIO debug task"));
}
