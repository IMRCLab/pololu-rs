//! Core 1 low-level UART driver.
//!
//! This module owns:
//! - UART0 IRQ handling
//! - timestamped RX bytes
//! - buffered TX packets
//! - the physical UART TX task
//!
//! Packet framing and mode-specific decoding do not belong here.

use embassy_rp::{
    interrupt::typelevel::{
        Handler,
        UART0_IRQ,
    },
    pac,
    uart::{
        Async,
        Uart,
    },
};
use embassy_sync::{
    blocking_mutex::raw::{
        CriticalSectionRawMutex,
        ThreadModeRawMutex,
    },
    channel::Channel,
};
use embassy_time::Instant;
use heapless::Vec;
use portable_atomic::{
    AtomicU32,
    Ordering,
};

pub const UART_TX_PACKET_SIZE: usize = 64;
const UART_TX_QUEUE_SIZE: usize = 128;
const UART_RX_QUEUE_SIZE: usize = 1024;

pub type UartTxPacket =
    Vec<u8, UART_TX_PACKET_SIZE>;

#[derive(Clone, Copy)]
pub struct TimestampedUartByte {
    pub byte: u8,
    pub stamp: Instant,
}

/// TX is only used by tasks running on the Core 1 thread-mode executor.
static UART_TX_CHANNEL: Channel<
    ThreadModeRawMutex,
    UartTxPacket,
    UART_TX_QUEUE_SIZE,
> = Channel::new();

/// RX is written from the UART interrupt and read by the Core 1 parser.
static UART_RX_CHANNEL: Channel<
    CriticalSectionRawMutex,
    TimestampedUartByte,
    UART_RX_QUEUE_SIZE,
> = Channel::new();

static UART_RX_DROPPED_BYTES: AtomicU32 =
    AtomicU32::new(0);

static UART_TX_DROPPED_PACKETS: AtomicU32 =
    AtomicU32::new(0);

/* ================= UART IRQ ================= */

pub struct UartRxIrqHandler;

impl Handler<UART0_IRQ> for UartRxIrqHandler {
    unsafe fn on_interrupt() {
        let uart = pac::UART0;

        while !uart.uartfr().read().rxfe() {
            let stamp = Instant::now();
            let data = uart.uartdr().read();

            let byte = TimestampedUartByte {
                byte: data.data(),
                stamp,
            };

            if UART_RX_CHANNEL
                .try_send(byte)
                .is_err()
            {
                UART_RX_DROPPED_BYTES.fetch_add(
                    1,
                    Ordering::Relaxed,
                );
            }
        }

        // Clear all RX-related interrupt conditions.
        uart.uarticr().write(|value| {
            value.set_rxic(true);
            value.set_rtic(true);
            value.set_oeic(true);
            value.set_beic(true);
            value.set_peic(true);
            value.set_feic(true);
        });
    }
}

pub fn init_uart_rx_irq() {
    let uart = pac::UART0;

    uart.uarticr().write(|value| {
        value.set_rxic(true);
        value.set_rtic(true);
        value.set_oeic(true);
        value.set_beic(true);
        value.set_peic(true);
        value.set_feic(true);
    });

    uart.uartifls().modify(|value| {
        // Lowest RX FIFO interrupt threshold.
        value.set_rxiflsel(0b000);
        value.set_txiflsel(0b000);
    });

    uart.uartimsc().modify(|value| {
        value.set_rxim(true);
        value.set_rtim(true);

        value.set_oeim(true);
        value.set_beim(true);
        value.set_peim(true);
        value.set_feim(true);
    });
}

/* ================= RX interface ================= */

/// Receive one timestamped UART byte.
///
/// Used by the Core 1 packet parser.
pub async fn receive_uart_byte() -> TimestampedUartByte {
    UART_RX_CHANNEL.receive().await
}

/// Remove bytes left from an incomplete or legacy packet.
pub fn drain_uart_rx() {
    while UART_RX_CHANNEL.try_receive().is_ok() {}
}

/* ================= TX interface ================= */

/// Queue a UART packet without waiting.
///
/// Suitable for periodic telemetry where dropping one packet is preferable to
/// delaying another Core 1 task.
pub fn try_send_uart(
    data: &[u8],
) -> Result<(), ()> {
    let mut packet = UartTxPacket::new();

    if packet.extend_from_slice(data).is_err() {
        UART_TX_DROPPED_PACKETS.fetch_add(
            1,
            Ordering::Relaxed,
        );

        return Err(());
    }

    if UART_TX_CHANNEL
        .try_send(packet)
        .is_err()
    {
        UART_TX_DROPPED_PACKETS.fetch_add(
            1,
            Ordering::Relaxed,
        );

        return Err(());
    }

    Ok(())
}

/// Queue a UART packet reliably.
///
/// This function may wait for TX queue space and therefore must never be
/// called by the Core 0 real-time control loop.
pub async fn send_uart(
    data: &[u8],
) -> Result<(), ()> {
    let mut packet = UartTxPacket::new();

    packet
        .extend_from_slice(data)
        .map_err(|_| ())?;

    UART_TX_CHANNEL.send(packet).await;

    Ok(())
}

/* ================= Hardware task ================= */

/// Permanent Core 1 UART TX task.
///
/// RX is handled directly by `UartRxIrqHandler`; this task owns the UART object
/// and performs asynchronous TX DMA operations.
#[embassy_executor::task]
pub async fn uart_hw_task(
    mut uart: Uart<'static, Async>,
) {
    defmt::info!("Core 1 UART hardware task started");

    loop {
        let packet =
            UART_TX_CHANNEL.receive().await;

        if uart.write(packet.as_slice()).await.is_err() {
            defmt::error!("UART TX failed");
        }
    }
}

/* ================= Metrics ================= */

pub fn uart_rx_dropped_bytes() -> u32 {
    UART_RX_DROPPED_BYTES.load(
        Ordering::Relaxed,
    )
}

pub fn uart_tx_dropped_packets() -> u32 {
    UART_TX_DROPPED_PACKETS.load(
        Ordering::Relaxed,
    )
}