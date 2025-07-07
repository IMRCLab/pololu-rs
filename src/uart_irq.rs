use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use fugit::RateExtU32;
use heapless::spsc::Queue;
use rp2040_hal::{
    clocks::ClocksManager,
    gpio::{
        bank0::{Gpio28, Gpio29},
        FunctionUart, Pin, PullUp,
    },
    pac::{self, interrupt, UART0},
    uart::{DataBits, Reader, StopBits, UartConfig, UartPeripheral},
    Clock,
};

type UartReader = Reader<
    UART0,
    (
        Pin<Gpio28, FunctionUart, PullUp>,
        Pin<Gpio29, FunctionUart, PullUp>,
    ),
>;

pub static UART_READER: Mutex<RefCell<Option<UartReader>>> = Mutex::new(RefCell::new(None));
pub static UART_RX_QUEUE: Mutex<RefCell<Option<Queue<u8, 128>>>> = Mutex::new(RefCell::new(None));

pub fn init_uart_with_irq(
    uart0: UART0,
    tx: Pin<Gpio28, FunctionUart, PullUp>,
    rx: Pin<Gpio29, FunctionUart, PullUp>,
    resets: &mut pac::RESETS,
    clocks: &ClocksManager,
) {
    let pins = (tx, rx);

    let mut uart = UartPeripheral::new(uart0, pins, resets)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    uart.set_fifos(true);

    let (mut reader, _) = uart.split();
    reader.enable_rx_interrupt();

    cortex_m::interrupt::free(|cs| {
        *UART_READER.borrow(cs).borrow_mut() = Some(reader);
        *UART_RX_QUEUE.borrow(cs).borrow_mut() = Some(Queue::new());
    });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::UART0_IRQ);
    }
}

#[interrupt]
fn UART0_IRQ() {
    cortex_m::interrupt::free(|cs| {
        let mut reader_opt = UART_READER.borrow(cs).borrow_mut();
        let mut queue_opt = UART_RX_QUEUE.borrow(cs).borrow_mut();

        if let (Some(reader), Some(queue)) = (reader_opt.as_mut(), queue_opt.as_mut()) {
            let mut buf = [0u8, 64];
            if let Ok(count) = reader.read_raw(&mut buf) {
                for &b in &buf[..count] {
                    let _ = queue.enqueue(b);
                }
            }
        }
    });
}

pub fn try_read_byte() -> Option<u8> {
    cortex_m::interrupt::free(|cs| {
        UART_RX_QUEUE
            .borrow(cs)
            .borrow_mut()
            .as_mut()
            .and_then(|queue| queue.dequeue())
    })
}
