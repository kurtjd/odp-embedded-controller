use embassy_qemu_riscv::gpio::{self, Async as GpioAsync, Input};
use embassy_qemu_riscv::i2c::target::{self, Async as I2cAsync, I2c};
use embassy_qemu_riscv::uart::{buffered, Async};
use embassy_qemu_riscv::{bind_interrupts, peripherals, uart};
use platform_common::board::BoardIo;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    UART0 => uart::buffered::InterruptHandler<peripherals::UART0>;
    I2C_TARGET => target::InterruptHandler<peripherals::I2C_TARGET>;
    GPIO => gpio::InterruptHandler;
});

/// 7-bit I2C address the host talks to (matches hid.asl).
pub const EC_I2C_ADDR: u8 = 0x2C;

/// Board IO for the dev-qemu platform.
///
/// This minimal development board provides a UART interface
/// for ODP service communication.
pub struct Board {
    /// UART for ODP service communication.
    pub uart: buffered::Uart<'static, Async>,
    /// I2C target (throwaway debug).
    pub i2c: I2c<'static, I2cAsync>,
    /// GPIO0 / HID interrupt line (throwaway debug).
    pub gpio: Input<'static, GpioAsync>,
}

impl BoardIo for Board {
    type Peripherals = embassy_qemu_riscv::Peripherals;

    fn init(p: Self::Peripherals) -> Self {
        static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
        let rx_buf = RX_BUF.init([0u8; 256]);

        let uart =
            buffered::Uart::new_async(p.UART0, Irqs, rx_buf, Default::default()).expect("Failed to initialize UART");

        let i2c = I2c::new_async(p.I2C_TARGET, Irqs, EC_I2C_ADDR);

        let gpio = Input::new_async(p.GPIO0, Irqs);

        Board { uart, i2c, gpio }
    }
}
