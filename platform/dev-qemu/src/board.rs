use embassy_qemu_riscv::uart::{buffered, Async};
use embassy_qemu_riscv::{bind_interrupts, peripherals, uart};
use platform_common::board::BoardIo;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    UART0 => uart::buffered::InterruptHandler<peripherals::UART0>;
});

/// Board IO for the dev-qemu platform.
///
/// This minimal development board provides a UART interface
/// for ODP service communication.
pub struct Board {
    /// UART for ODP service communication.
    pub uart: buffered::Uart<'static, Async>,
}

impl BoardIo for Board {
    type Peripherals = embassy_qemu_riscv::Peripherals;

    fn init(p: Self::Peripherals) -> Self {
        static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
        let rx_buf = RX_BUF.init([0u8; 256]);

        let uart =
            buffered::Uart::new_async(p.UART0, Irqs, rx_buf, Default::default()).expect("Failed to initialize UART");

        Board { uart }
    }
}
