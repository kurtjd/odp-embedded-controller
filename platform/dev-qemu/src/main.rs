#![no_main]
#![no_std]

mod board;

use board::Board;
use defmt::info;
use defmt_semihosting as _;
use embassy_executor::Spawner;
use embassy_qemu_riscv::uart::{buffered, Async};
use platform_common::board::BoardIo;
use platform_common::OdpRelayHandler;
use semihosting as _; // Panic handler
use static_cell::StaticCell;

#[embassy_executor::task]
async fn uart_service(uart: buffered::Uart<'static, Async>, relay: OdpRelayHandler<'static>) {
    info!("Starting uart service");
    static UART_SERVICE: StaticCell<uart_service::Service<OdpRelayHandler>> = StaticCell::new();
    let uart_service = uart_service::Service::new(relay).unwrap();
    let uart_service = UART_SERVICE.init(uart_service);
    let Err(e) = uart_service::task::uart_service(uart_service, uart).await;
    panic!("uart-service error: {:?}", e);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_qemu_riscv::init();
    let board = Board::init(p);

    let relay = platform_common::mock::init(spawner).await;
    spawner.must_spawn(uart_service(board.uart, relay));
}
