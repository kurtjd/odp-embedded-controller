#![no_main]
#![no_std]

mod board;

use board::Board;
use defmt::info;
use defmt_semihosting as _;
use embassy_executor::Spawner;
use platform_common::board::BoardIo;
use semihosting as _; // Panic handler

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_qemu_riscv::init();
    let board = Board::init(p);

    let relay = platform_common::mock::init(spawner).await;

    info!("Starting uart service");
    let uart_service = uart_service::DefaultService::default_smbusespi(relay).unwrap();
    let Err(e) = uart_service::task::uart_service(uart_service, board.uart).await;
    panic!("uart-service error: {:?}", e);
}
