//! Core 1 UART packet framing.
//!
//! Raw bytes come exclusively from `peripherals::uart`. This parser contains
//! no mode-specific business logic.

use embassy_futures::select::{
    Either,
    select,
};
use embassy_time::{
    Duration,
    Instant,
    Timer,
};
use heapless::Vec;

use crate::peripherals::uart::receive_uart_byte;

/// Packet lengths accepted by the current firmware protocol.
///
/// Length includes the `0x3C` header but excludes the preceding length byte.
const VALID_PACKET_LENGTHS: &[u8] = &[
    3,  // mode selection
    4,  // trajectory pause/resume
    8,  // parameter write
    9,  // TeleOp / ControlAction
    10, // legacy start packet
    18, // Mocap pose
];

pub struct ReceivedPacket {
    pub length: u8,

    /// Timestamp of the packet length byte.
    pub stamp: Instant,
}

/// Find and receive one complete framed packet.
///
/// Wire format:
///
/// ```text
/// [length, 0x3C, payload...]
/// ```
///
/// On return, `frame` contains:
///
/// ```text
/// [0x3C, payload...]
/// ```
///
/// The preceding length byte is not stored in `frame`.
pub async fn receive_packet<const N: usize>(
    frame: &mut Vec<u8, N>,
) -> ReceivedPacket {
    loop {
        /* ---------- Seek [length, 0x3C] ---------- */

        let (length, packet_stamp) = loop {
            let first = receive_uart_byte().await;

            if !VALID_PACKET_LENGTHS.contains(
                &first.byte,
            ) {
                continue;
            }

            let header = match select(
                receive_uart_byte(),
                Timer::after(
                    Duration::from_millis(5),
                ),
            )
            .await
            {
                Either::First(header) => header,
                Either::Second(_) => continue,
            };

            if header.byte != 0x3C {
                continue;
            }

            break (first.byte, first.stamp);
        };

        frame.clear();

        if frame.push(0x3C).is_err() {
            continue;
        }

        /* ---------- Read remaining bytes ---------- */

        let bytes_needed =
            (length as usize).saturating_sub(1);

        let mut received = 0usize;
        let mut framing_error = false;

        while received < bytes_needed {
            let byte = match select(
                receive_uart_byte(),
                Timer::after(
                    Duration::from_millis(10),
                ),
            )
            .await
            {
                Either::First(byte) => byte.byte,

                Either::Second(_) => {
                    framing_error = true;
                    break;
                }
            };

            if frame.push(byte).is_err() {
                framing_error = true;
                break;
            }

            received += 1;
        }

        if framing_error {
            frame.clear();
            continue;
        }

        return ReceivedPacket {
            length,
            stamp: packet_stamp,
        };
    }
}