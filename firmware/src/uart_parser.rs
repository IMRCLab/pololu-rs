/// Robust UART packet parser.
///
/// All packet formats use the framing: `[LEN, 0x3C, payload...]`
/// where `LEN` is the total number of bytes **including** the header byte 0x3C.
///
/// This parser seeks for a valid `[LEN, 0x3C]` preamble before reading the rest
/// of the payload. This makes it resilient to stale bytes, duplicate dongle sends,
/// and partial frames.

use embassy_futures::select::{Either, Either3, select, select3};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex as Raw, signal::Signal};
use embassy_time::{Duration, Timer};
use heapless::Vec;

use crate::uart::UART_RX_CHANNEL;

/// Result of a single `receive_packet` call.
pub enum RecvResult {
    /// Fully-received packet. `len` is the length byte, `frame` contains `[0x3C, payload...]`.
    Packet { len: u8 },
    /// The stop signal fired; the caller should exit.
    Stop,
}

/// The set of valid packet lengths for this firmware.
/// Any byte NOT in this list is treated as garbage and skipped.
const VALID_LENGTHS: &[u8] = &[
    3,  // LEN_FUNC_SELECT_CMD  (mode switch)
    4,  // LEN_STOP_RESUME_CMD  (traj start/stop)
    8,  // parameter write
    9,  // TELEOP_PACK_LEN
    10, // LEN_START (legacy)
    18, // LEN_POSE  (mocap pose)
];

/// Seek the `[LEN, 0x3C]` preamble then read the rest of the payload into `frame`.
///
/// `stop_sig` — a reference to the global `Signal` that fires when the task should exit.
/// The function calls `.wait()` on it repeatedly, so the signal must be a `Signal<Raw, ()>`.
///
/// Returns `RecvResult::Stop` when the stop signal fires.
/// Returns `RecvResult::Packet { len }` on success; `frame` is populated with
/// `[0x3C, byte1, byte2, ...]` (i.e. `len-1` additional bytes after the header).
///
/// On framing errors (timeout, bad header, etc.) the function loops back to seek.
pub async fn receive_packet<const N: usize>(
    frame: &mut Vec<u8, N>,
    stop_sig: &'static Signal<Raw, ()>,
) -> RecvResult {
    loop {
        // ---- Phase 1: seek a valid [LEN, 0x3C] preamble ----
        let len;
        'seek: loop {
            // Wait for a byte (or stop)
            let b = match select(UART_RX_CHANNEL.receive(), stop_sig.wait()).await {
                Either::First(b) => b,
                Either::Second(_) => return RecvResult::Stop,
            };

            // Is it a known length byte?
            if !VALID_LENGTHS.contains(&b) {
                continue 'seek;
            }

            // Peek at the next byte — must be 0x3C within 5 ms
            let header = match select(
                UART_RX_CHANNEL.receive(),
                Timer::after(Duration::from_millis(5)),
            )
            .await
            {
                Either::First(h) => h,
                Either::Second(_) => continue 'seek, // timeout — keep seeking
            };

            if header == 0x3C {
                len = b;
                frame.clear();
                frame.push(0x3C).ok();
                break 'seek;
            }
            // header wasn't 0x3C — restart seek.
            // The 3× retransmit from the dongle means we'll get another chance quickly.
        }

        // ---- Phase 2: read remaining payload bytes (len-1 because 0x3C already added) ----
        let need = (len as usize).saturating_sub(1);
        let mut got = 0usize;

        while got < need {
            let b_opt = match select3(
                UART_RX_CHANNEL.receive(),
                Timer::after(Duration::from_millis(10)),
                stop_sig.wait(),
            )
            .await
            {
                Either3::First(b) => Some(b),
                Either3::Second(_) => None,  // timeout → framing error, re-seek
                Either3::Third(_) => return RecvResult::Stop,
            };

            match b_opt {
                Some(b) => {
                    frame.push(b).ok();
                    got += 1;
                }
                None => break, // timeout → restart outer seek loop
            }
        }

        if got == need {
            return RecvResult::Packet { len };
        }
        // else: partial frame, restart seek
    }
}
