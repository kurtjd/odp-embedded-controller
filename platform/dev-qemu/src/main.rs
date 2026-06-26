//! HID-over-I2C keyboard driven by UART input.
//!
//! This instantiates the OpenDevicePartnership [`keyboard_service`] but, instead of scanning a
//! physical GPIO key matrix, it sources key presses from UART0. Connect a terminal (e.g. PuTTY)
//! to the UART and every byte you type is translated into a HID keyboard input report which is
//! served to a host over the HID-over-I2C transport (the `I2C_TARGET` peripheral).
//!
//! Architecture (mirrors a real GPIO keyboard, minus the matrix scanning):
//! - [`UartKeyboard`] implements [`keyboard_service::HidKeyboard`]; its `scan` awaits a UART byte,
//!   converts it to a HID usage code, and emits a "press" report followed by a "release" report
//!   (terminals only deliver key-down events, so we synthesise the key-up).
//! - [`HidI2cSlave`] adapts our HAL's I2C target driver to the [`hid_service::i2c::I2cSlaveAsync`]
//!   trait the HID service expects.
//! - Four service tasks are spawned: the keyboard scan loop, the report/interrupt pump, the HID
//!   device request handler, and the raw I2C host-request handler.
//!
//! A host (e.g. a separate QEMU instance) acts as the I2C controller / HID host and polls the
//! device for input reports; that side is out of scope for this example.//!
//! ## Modifier keys over a serial terminal
//!
//! A terminal cannot transmit a bare Windows key or a standalone Ctrl/Alt/Shift press, so
//! modifiers are derived from what the terminal *does* send:
//! - **Shift**: implicit in shifted ASCII (e.g. `A`, `!`).
//! - **Ctrl+letter**: the control bytes `0x01`–`0x1A` map to letter + Left-Ctrl
//!   (`Ctrl+H/I/J/M` are kept as Backspace/Tab/Enter for usability).
//! - **Alt+key**: the "meta" convention — terminals send `ESC` + key, decoded as key + Left-Alt.
//! - **GUI / Windows key**: typed via the `Ctrl+]` prefix — `Ctrl+]` then a key gives Win+key
//!   (e.g. Win+R), and pressing `Ctrl+]` twice sends a standalone Windows-key tap.
#![no_std]
#![no_main]

use defmt::info;
use defmt_semihosting as _;
use embassy_executor::Spawner;
use embassy_qemu_riscv::gpio::{Level, Output};
use embassy_qemu_riscv::i2c::target::{self, Async as I2cAsync, I2c};
use embassy_qemu_riscv::uart::buffered::{InterruptHandler as UartInterruptHandler, Uart, UartRx};
use embassy_qemu_riscv::uart::{Async as UartAsync, Config as UartConfig};
use embassy_qemu_riscv::{bind_interrupts, peripherals};
use embassy_sync::signal::Signal;
use embedded_mcu_hal::i2c::target::Request as TargetRequest;
use embedded_services::buffer::SharedRef;
use embedded_services::hid;
use embedded_services::GlobalRawMutex;
use keyboard_service::{impl_host_request_task, HidKeyboard, HidReportSlice, KeyboardError};
use semihosting as _;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    UART0 => UartInterruptHandler<peripherals::UART0>;
    I2C_TARGET => target::InterruptHandler<peripherals::I2C_TARGET>;
});

/// 7-bit I2C address the keyboard answers to as a HID-over-I2C device.
const KB_I2C_ADDR: u8 = 0x2C;

/// UART baud rate. Match this in your terminal (PuTTY: Serial, 115200 8N1).
const UART_BAUD: u32 = 115_200;

/// Vendor / product IDs reported in the HID descriptor.
const KB_VID: u16 = 0x045E;
const KB_PID: u16 = 0x0CC7;

// --- HID report layout -------------------------------------------------------

/// Report ID for the single keyboard input report.
const REPORT_ID: u8 = 1;

/// Size of the HID-over-I2C report header the backend prepends (2 byte length + 1 byte report ID).
const I2C_REPORT_HEADER_SZ: usize = 3;

/// One modifier byte.
const KEYMOD_SZ: usize = 1;

/// Number of simultaneous key usage codes (6-key rollover, matching the report descriptor).
const KRO: usize = 6;

/// The report payload we hand to the backend: `[modifiers, k0..k5]`.
const REPORT_SZ: usize = KEYMOD_SZ + KRO;

/// Max input report length advertised in the HID descriptor (header + payload).
const INPUT_MAX_LEN: usize = I2C_REPORT_HEADER_SZ + REPORT_SZ;

/// Max output report length (header + a single LED status byte).
const OUTPUT_MAX_LEN: usize = I2C_REPORT_HEADER_SZ + 1;

/// HID keyboard modifier bits (left-hand side modifiers).
const MOD_LCTRL: u8 = 0x01;
const MOD_LSHIFT: u8 = 0x02;
const MOD_LALT: u8 = 0x04;
const MOD_LGUI: u8 = 0x08;

/// Control byte used as a one-shot "apply Left-GUI (Windows key) to the next key" prefix.
///
/// A serial terminal cannot transmit a bare Windows key or standalone modifiers, so we expose the
/// GUI modifier through a prefix the user can type:
/// - `Ctrl+]` then a key  → that key with the Windows modifier (e.g. `Ctrl+]` then `r` = Win+R).
/// - `Ctrl+]` twice       → a standalone Windows-key tap (modifier only, no usage code).
const GUI_PREFIX: u8 = 0x1D; // Ctrl+]

/// Standard boot-compatible keyboard report descriptor: one input report (report ID 1) with an
/// 8-bit modifier field and six key usage codes, plus an LED output report. This matches the
/// format the keyboard service's HID-over-I2C backend expects.
#[rustfmt::skip]
const REPORT_DESCRIPTOR: &[u8] = &[
    // Usage Page (Generic Desktop Ctrls)
    0x05, 0x01,
    // Usage (Keyboard)
    0x09, 0x06,
    // Collection (Application)
    0xA1, 0x01,
    // Report ID (1)
    0x85, REPORT_ID,
    // Usage Page (Keyboard/Keypad)
    0x05, 0x07,
    // Usage Minimum (0xE0) .. Maximum (0xE7): the 8 modifier keys
    0x19, 0xE0,
    0x29, 0xE7,
    0x15, 0x00,
    0x25, 0x01,
    0x75, 0x01,
    0x95, 0x08,
    // Input (Data,Var,Abs)
    0x81, 0x02,
    // Six key usage codes
    0x19, 0x00,
    0x29, 0x91,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x06,
    // Input (Data,Array,Abs)
    0x81, 0x00,
    // LED output report (Num/Caps/Scroll lock)
    0x05, 0x08,
    0x19, 0x01,
    0x29, 0x03,
    0x75, 0x01,
    0x95, 0x03,
    0x25, 0x01,
    0x91, 0x02,
    // Padding to byte-align the LED report
    0x95, 0x05,
    0x91, 0x01,
    // End Collection
    0xC0,
];

// --- UART byte -> HID usage code translation ---------------------------------

/// A decoded key: the modifier byte plus a single HID usage code.
#[derive(Clone, Copy)]
struct Key {
    modifier: u8,
    usage: u8,
}

impl Key {
    const fn plain(usage: u8) -> Self {
        Self { modifier: 0, usage }
    }

    const fn shifted(usage: u8) -> Self {
        Self {
            modifier: MOD_LSHIFT,
            usage,
        }
    }
}

/// Map a single printable/control ASCII byte to a HID key, or `None` if unmapped.
///
/// `0x1B` (ESC) is handled by the escape-sequence parser, not here.
fn ascii_to_key(b: u8) -> Option<Key> {
    let key = match b {
        // Letters
        b'a'..=b'z' => Key::plain(0x04 + (b - b'a')),
        b'A'..=b'Z' => Key::shifted(0x04 + (b - b'A')),

        // Top-row digits
        b'1' => Key::plain(0x1E),
        b'2' => Key::plain(0x1F),
        b'3' => Key::plain(0x20),
        b'4' => Key::plain(0x21),
        b'5' => Key::plain(0x22),
        b'6' => Key::plain(0x23),
        b'7' => Key::plain(0x24),
        b'8' => Key::plain(0x25),
        b'9' => Key::plain(0x26),
        b'0' => Key::plain(0x27),

        // Shifted digits (symbols)
        b'!' => Key::shifted(0x1E),
        b'@' => Key::shifted(0x1F),
        b'#' => Key::shifted(0x20),
        b'$' => Key::shifted(0x21),
        b'%' => Key::shifted(0x22),
        b'^' => Key::shifted(0x23),
        b'&' => Key::shifted(0x24),
        b'*' => Key::shifted(0x25),
        b'(' => Key::shifted(0x26),
        b')' => Key::shifted(0x27),

        // Punctuation (unshifted / shifted pairs)
        b'-' => Key::plain(0x2D),
        b'_' => Key::shifted(0x2D),
        b'=' => Key::plain(0x2E),
        b'+' => Key::shifted(0x2E),
        b'[' => Key::plain(0x2F),
        b'{' => Key::shifted(0x2F),
        b']' => Key::plain(0x30),
        b'}' => Key::shifted(0x30),
        b'\\' => Key::plain(0x31),
        b'|' => Key::shifted(0x31),
        b';' => Key::plain(0x33),
        b':' => Key::shifted(0x33),
        b'\'' => Key::plain(0x34),
        b'"' => Key::shifted(0x34),
        b'`' => Key::plain(0x35),
        b'~' => Key::shifted(0x35),
        b',' => Key::plain(0x36),
        b'<' => Key::shifted(0x36),
        b'.' => Key::plain(0x37),
        b'>' => Key::shifted(0x37),
        b'/' => Key::plain(0x38),
        b'?' => Key::shifted(0x38),

        // Whitespace / control
        b' ' => Key::plain(0x2C),
        b'\r' | b'\n' => Key::plain(0x28), // Enter
        b'\t' => Key::plain(0x2B),         // Tab
        0x08 | 0x7F => Key::plain(0x2A),   // Backspace / DEL

        _ => return None,
    };
    Some(key)
}

/// Map a CSI/SS3 final byte (after `ESC [` or `ESC O`) with optional numeric parameter to a key.
fn escape_final_to_key(param: u16, final_byte: u8) -> Option<Key> {
    let usage = match final_byte {
        // Cursor keys (and SS3 application-mode cursor keys)
        b'A' => 0x52, // Up
        b'B' => 0x51, // Down
        b'C' => 0x4F, // Right
        b'D' => 0x50, // Left
        b'H' => 0x4A, // Home
        b'F' => 0x4D, // End

        // SS3 function keys F1-F4
        b'P' => 0x3A,
        b'Q' => 0x3B,
        b'R' => 0x3C,
        b'S' => 0x3D,

        // CSI `<param>~` sequences
        b'~' => match param {
            1 => 0x4A,  // Home
            2 => 0x49,  // Insert
            3 => 0x4C,  // Delete
            4 => 0x4D,  // End
            5 => 0x4B,  // Page Up
            6 => 0x4E,  // Page Down
            15 => 0x3E, // F5
            17 => 0x3F, // F6
            18 => 0x40, // F7
            19 => 0x41, // F8
            20 => 0x42, // F9
            21 => 0x43, // F10
            23 => 0x44, // F11
            24 => 0x45, // F12
            _ => return None,
        },

        _ => return None,
    };
    Some(Key::plain(usage))
}

/// State of the small ANSI escape-sequence parser.
#[derive(Clone, Copy)]
enum ParseState {
    /// Normal: bytes map directly to keys.
    Ground,
    /// Saw `ESC`; awaiting `[`, `O`, or a lone-Escape fallthrough.
    Esc,
    /// Inside a `ESC [ ...` (CSI) sequence; `param` accumulates the numeric argument.
    Csi { param: u16 },
    /// Inside a `ESC O ...` (SS3) sequence.
    Ss3,
}

/// Output of feeding one byte to the parser.
enum ParseOut {
    /// Need more bytes to decide.
    Incomplete,
    /// A key was decoded.
    Key(Key),
    /// A key was decoded, and the supplied byte must be reprocessed from [`ParseState::Ground`].
    KeyThenReprocess(Key, u8),
    /// The byte(s) did not map to anything.
    Ignored,
}

/// Incremental ANSI/VT key decoder. Holds all state internally so it is safe to drive one byte at
/// a time across cancellable `scan` calls.
struct EscParser {
    state: ParseState,
    /// Modifier bits to OR into the next emitted key (e.g. set by the GUI prefix). One-shot:
    /// cleared as soon as a key is produced.
    pending_mod: u8,
}

impl EscParser {
    const fn new() -> Self {
        Self {
            state: ParseState::Ground,
            pending_mod: 0,
        }
    }

    /// Take and clear any pending one-shot modifier bits.
    fn take_mod(&mut self) -> u8 {
        let m = self.pending_mod;
        self.pending_mod = 0;
        m
    }

    /// Emit a key, folding in (and clearing) any pending one-shot modifier.
    fn emit(&mut self, mut key: Key) -> ParseOut {
        key.modifier |= self.take_mod();
        ParseOut::Key(key)
    }

    /// Decode a ground-state byte (control codes and printable ASCII) into a key.
    ///
    /// `ESC` and the GUI prefix are control-flow bytes handled in [`Self::feed`], not here.
    fn decode_byte(b: u8) -> Option<Key> {
        match b {
            // Dedicated control keys take priority over their Ctrl-chord interpretation.
            0x08 | 0x7F => Some(Key::plain(0x2A)), // Backspace / DEL
            0x09 => Some(Key::plain(0x2B)),        // Tab    (Ctrl+I)
            0x0A | 0x0D => Some(Key::plain(0x28)), // Enter  (Ctrl+J / Ctrl+M)
            // Remaining Ctrl+letter chords: 0x01=Ctrl+A .. 0x1A=Ctrl+Z.
            0x01..=0x1A => Some(Key {
                modifier: MOD_LCTRL,
                usage: 0x04 + (b - 1),
            }),
            // Everything else: printable ASCII.
            _ => ascii_to_key(b),
        }
    }

    fn feed(&mut self, b: u8) -> ParseOut {
        match self.state {
            ParseState::Ground => match b {
                0x1B => {
                    self.state = ParseState::Esc;
                    ParseOut::Incomplete
                }
                GUI_PREFIX => {
                    if self.pending_mod & MOD_LGUI != 0 {
                        // Second prefix in a row: emit a standalone Windows-key tap.
                        self.take_mod();
                        ParseOut::Key(Key {
                            modifier: MOD_LGUI,
                            usage: 0,
                        })
                    } else {
                        // Arm the GUI modifier for the next key.
                        self.pending_mod |= MOD_LGUI;
                        ParseOut::Incomplete
                    }
                }
                _ => match Self::decode_byte(b) {
                    Some(key) => self.emit(key),
                    None => ParseOut::Ignored,
                },
            },
            ParseState::Esc => match b {
                b'[' => {
                    self.state = ParseState::Csi { param: 0 };
                    ParseOut::Incomplete
                }
                b'O' => {
                    self.state = ParseState::Ss3;
                    ParseOut::Incomplete
                }
                // `ESC ESC` is a literal Escape key.
                0x1B => {
                    self.state = ParseState::Ground;
                    self.emit(Key::plain(0x29))
                }
                // `ESC <printable>` is the meta convention: Alt + that key.
                0x20..=0x7E => {
                    self.state = ParseState::Ground;
                    match ascii_to_key(b) {
                        Some(mut key) => {
                            key.modifier |= MOD_LALT;
                            self.emit(key)
                        }
                        None => ParseOut::Ignored,
                    }
                }
                // Lone ESC was the Escape key; reprocess this byte as fresh input.
                _ => {
                    self.state = ParseState::Ground;
                    let mut esc = Key::plain(0x29);
                    esc.modifier |= self.take_mod();
                    ParseOut::KeyThenReprocess(esc, b)
                }
            },
            ParseState::Csi { param } => {
                if b.is_ascii_digit() {
                    let next = param.saturating_mul(10).saturating_add((b - b'0') as u16);
                    self.state = ParseState::Csi { param: next };
                    ParseOut::Incomplete
                } else if b == b';' {
                    // Ignore secondary parameters (e.g. modifier info); keep the first.
                    ParseOut::Incomplete
                } else {
                    self.state = ParseState::Ground;
                    match escape_final_to_key(param, b) {
                        Some(key) => self.emit(key),
                        None => ParseOut::Ignored,
                    }
                }
            }
            ParseState::Ss3 => {
                self.state = ParseState::Ground;
                match escape_final_to_key(0, b) {
                    Some(key) => self.emit(key),
                    None => ParseOut::Ignored,
                }
            }
        }
    }
}

// --- UART-backed HID keyboard ------------------------------------------------

/// A [`HidKeyboard`] whose key events come from UART bytes instead of a GPIO matrix.
struct UartKeyboard {
    rx: UartRx<'static, UartAsync>,
    parser: EscParser,
    /// Backing storage for the report slice returned from `scan`/`get_report`.
    report: [u8; REPORT_SZ],
    /// When set, the next `scan` emits an all-zero (key-up) report.
    pending_release: bool,
    /// A byte stashed by the parser to be reprocessed (lone-ESC fallthrough).
    pending_byte: Option<u8>,
    power_state: hid::PowerState,
    report_freq: hid::ReportFreq,
    /// Signalled when the host powers the keyboard on, waking a sleeping scan loop.
    power_on: Signal<GlobalRawMutex, ()>,
}

impl UartKeyboard {
    fn new(rx: UartRx<'static, UartAsync>) -> Self {
        Self {
            rx,
            parser: EscParser::new(),
            report: [0; REPORT_SZ],
            pending_release: false,
            pending_byte: None,
            // Default to On so the demo emits reports as soon as you type, even before a host
            // powers the device on. A host may still toggle power via SetPower commands.
            power_state: hid::PowerState::On,
            report_freq: hid::ReportFreq::Infinite,
            power_on: Signal::new(),
        }
    }

    /// Read exactly one byte from the UART. Cancel-safe: a byte is only consumed when this resolves.
    async fn read_byte(&mut self) -> u8 {
        let mut buf = [0u8; 1];
        loop {
            if self.rx.read(&mut buf).await >= 1 {
                return buf[0];
            }
        }
    }

    /// Store a decoded key into the report buffer.
    fn set_report(&mut self, key: Key) {
        self.report = [0; REPORT_SZ];
        self.report[0] = key.modifier;
        self.report[1] = key.usage;
    }
}

impl HidKeyboard for UartKeyboard {
    fn hid_descriptor(&self) -> hid::Descriptor {
        const VERSION: u16 = 0x0100;
        let regs = self.register_file();
        hid::Descriptor {
            w_hid_desc_length: hid::DESCRIPTOR_LEN as u16,
            bcd_version: VERSION,
            w_report_desc_length: REPORT_DESCRIPTOR.len() as u16,
            w_report_desc_register: regs.report_desc_reg,
            w_input_register: regs.input_reg,
            w_max_input_length: INPUT_MAX_LEN as u16,
            w_output_register: regs.output_reg,
            w_max_output_length: OUTPUT_MAX_LEN as u16,
            w_command_register: regs.command_reg,
            w_data_register: regs.data_reg,
            w_vendor_id: KB_VID,
            w_product_id: KB_PID,
            w_version_id: VERSION,
        }
    }

    fn report_descriptor(&self) -> &'static [u8] {
        REPORT_DESCRIPTOR
    }

    fn register_file(&self) -> hid::RegisterFile {
        hid::RegisterFile::default()
    }

    async fn scan(&mut self) -> Result<HidReportSlice<'_>, KeyboardError> {
        // Don't yield input until the host powers us on.
        if self.power_state == hid::PowerState::Sleep {
            self.power_on.wait().await;
        }

        // A press was reported last time: emit the matching key-up report now.
        if self.pending_release {
            self.pending_release = false;
            self.report = [0; REPORT_SZ];
            return Ok(HidReportSlice::new(&self.report));
        }

        // Decode bytes until one yields a key, emitting a "press" report.
        loop {
            let byte = match self.pending_byte.take() {
                Some(b) => b,
                None => self.read_byte().await,
            };

            match self.parser.feed(byte) {
                ParseOut::Key(key) => {
                    info!("key down: usage={=u8:#04x} mod={=u8:#04x}", key.usage, key.modifier);
                    self.set_report(key);
                    self.pending_release = true;
                    return Ok(HidReportSlice::new(&self.report));
                }
                ParseOut::KeyThenReprocess(key, reprocess) => {
                    info!("key down: usage={=u8:#04x} mod={=u8:#04x}", key.usage, key.modifier);
                    self.pending_byte = Some(reprocess);
                    self.set_report(key);
                    self.pending_release = true;
                    return Ok(HidReportSlice::new(&self.report));
                }
                ParseOut::Incomplete | ParseOut::Ignored => {}
            }
        }
    }

    async fn reset(&mut self) -> Result<(), KeyboardError> {
        self.report = [0; REPORT_SZ];
        self.pending_release = false;
        self.pending_byte = None;
        self.parser = EscParser::new();
        self.report_freq = hid::ReportFreq::Infinite;
        Ok(())
    }

    async fn set_power_state(&mut self, power_state: hid::PowerState) -> Result<(), KeyboardError> {
        self.power_state = power_state;
        if power_state == hid::PowerState::On {
            // Wake the scan loop if it was waiting.
            self.power_on.signal(());
        }
        Ok(())
    }

    async fn set_idle(&mut self, _report_id: hid::ReportId, report_freq: hid::ReportFreq) -> Result<(), KeyboardError> {
        self.report_freq = report_freq;
        Ok(())
    }

    fn get_idle(&self, _report_id: hid::ReportId) -> hid::ReportFreq {
        self.report_freq
    }

    async fn set_protocol(&mut self, _protocol: hid::Protocol) -> Result<(), KeyboardError> {
        // Only the Report protocol is supported; treat as a no-op.
        Ok(())
    }

    fn get_protocol(&self) -> hid::Protocol {
        hid::Protocol::Report
    }

    async fn vendor_cmd(&mut self) -> Result<(), KeyboardError> {
        Ok(())
    }

    async fn set_report(
        &mut self,
        _report_type: hid::ReportType,
        _report_id: hid::ReportId,
        _buf: &SharedRef<'static, u8>,
    ) -> Result<(), KeyboardError> {
        // We have no physical LEDs; accept and ignore output reports.
        Ok(())
    }

    fn get_report(&self, report_type: hid::ReportType, _report_id: hid::ReportId) -> HidReportSlice<'_> {
        match report_type {
            hid::ReportType::Input => HidReportSlice::new(&self.report),
            _ => HidReportSlice::new(&[0x00]),
        }
    }
}

// --- I2C target -> HID slave adapter -----------------------------------------

/// Adapts our HAL's I2C target driver to the [`hid_service::i2c::I2cSlaveAsync`] trait.
///
/// The HAL reports the transfer direction directly (`Read`/`Write`). It also surfaces bus-framing
/// events (START / repeated-START / STOP / general-call / SMBus alert) as their own `listen`
/// results *before* the direction of the next sub-transaction. The HID host loop in `hid-service`
/// expects `listen` to report a real data direction (its `send_response` path only does a single
/// `listen` and rejects anything that isn't a `Read`), so we swallow framing-only events here and
/// keep listening until the controller commits to a `Read` or `Write`.
struct HidI2cSlave(I2c<'static, I2cAsync>);

impl hid_service::i2c::I2cSlaveAsync for HidI2cSlave {
    type Error = core::convert::Infallible;

    async fn listen(&mut self) -> Result<hid_service::i2c::Command, Self::Error> {
        loop {
            match self.0.listen().await {
                TargetRequest::Read(_) => return Ok(hid_service::i2c::Command::Read),
                TargetRequest::Write(_) => return Ok(hid_service::i2c::Command::Write),
                // Framing-only events carry no data direction yet; wait for the next event.
                _ => continue,
            }
        }
    }

    async fn respond_to_write(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        let _ = self.0.respond_to_write(buf).await;
        Ok(())
    }

    async fn respond_to_read(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        let _ = self.0.respond_to_read(buf).await;
        Ok(())
    }
}

// --- Service tasks -----------------------------------------------------------
//
// The keyboard service's task helpers are generic and fallible, so each is wrapped in a concrete,
// non-generic Embassy task that panics if the (effectively infinite) loop ever returns an error.

#[embassy_executor::task]
async fn keyboard_task(kb: UartKeyboard) {
    let Err(e) = keyboard_service::task::keyboard_task(kb).await;
    panic!("keyboard task failed: {:?}", e);
}

#[embassy_executor::task]
async fn reports_task(kb_int: Output<'static>) {
    let Err(e) = keyboard_service::task::reports_task(kb_int).await;
    panic!("reports task failed: {:?}", e);
}

#[embassy_executor::task]
async fn device_requests_task(
    hid_descriptor: hid::Descriptor,
    report_descriptor: &'static [u8],
    reg_file: hid::RegisterFile,
) {
    let Err(e) =
        keyboard_service::task::init_and_recv_device_requests_task(hid_descriptor, report_descriptor, reg_file).await;
    panic!("device requests task failed: {:?}", e);
}

impl_host_request_task!(HidI2cSlave);

#[embassy_executor::task]
async fn host_requests_task_embassy(kb_i2c: HidI2cSlave) {
    host_requests_task(kb_i2c).await;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_qemu_riscv::init();
    embedded_services::init().await;

    static RX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    let rx_buf = RX_BUF.init([0; 64]);
    let uart = Uart::new_async(p.UART0, Irqs, rx_buf, UartConfig { baudrate: UART_BAUD })
        .expect("UART baud rate must be representable");
    let (rx, _tx) = uart.split();
    let kb = UartKeyboard::new(rx);

    // I2C target acting as the HID-over-I2C device.
    let i2c = I2c::new_async(p.I2C_TARGET, Irqs, KB_I2C_ADDR);
    let kb_i2c = HidI2cSlave(i2c);

    // Interrupt line the device drives to tell the host a report is ready.
    let kb_int = Output::new(p.GPIO0, Level::High);

    let descriptor = kb.hid_descriptor();
    let report_descriptor = kb.report_descriptor();
    let reg_file = kb.register_file();

    info!(
        "UART keyboard ready: HID-over-I2C device @ {=u8:#x}, UART {=u32} baud",
        KB_I2C_ADDR, UART_BAUD
    );
    info!("Hello world");

    spawner.spawn(
        device_requests_task(descriptor, report_descriptor, reg_file).expect("Failed to spawn device requests task"),
    );
    spawner.spawn(keyboard_task(kb).expect("Failed to spawn keyboard task"));
    spawner.spawn(reports_task(kb_int).expect("Failed to spawn reports task"));
    spawner.spawn(host_requests_task_embassy(kb_i2c).expect("Failed to spawn host requests task"));
}
