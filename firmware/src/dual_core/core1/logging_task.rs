//! Core 1 SD logging service.
//!
//! This is the only task allowed to own and operate `SdLogger` in the
//! dual-core firmware.

use embassy_futures::select::{
    Either,
    select,
};

use crate::{
    dual_core::{
        logging::receive_log_frame,
        messages::LoggerCommand,
        shared_state::receive_logger_command,
    },
    peripherals::sdcard::SdLogger,
};

/// Header written after the legacy SdLogger magic bytes.
const DUAL_CORE_LOG_MAGIC: &[u8; 4] = b"DCLG";

/// Binary schema version.
const DUAL_CORE_LOG_VERSION: u16 = 1;

/// Current serialized `UnifiedLogFrame` size:
///
/// sequence:       4
/// timestamp:      8
/// mode:           1
/// valid mask:     4
/// 25 f32 fields: 100
/// 3 u32 fields:  12
///
/// Total:         129 bytes
const DUAL_CORE_RECORD_SIZE: u16 = 129;

/// At 50 Hz, flush approximately every five seconds.
///
/// Full 512-byte internal buffers are still written automatically by
/// `SdLogger::write_buffered`.
const FLUSH_EVERY_FRAMES: u32 = 250;

fn open_log_session(
    logger: &mut SdLogger,
) {
    // Finish a previous session if one is still open.
    logger.flush();
    logger.close_file();

    logger.open_new_file();

    // SdLogger already writes its legacy AA BB CC DD magic. Append a
    // dual-core schema header so the decoder can identify this format.
    logger.write_buffered(DUAL_CORE_LOG_MAGIC);

    logger.write_buffered(
        &DUAL_CORE_LOG_VERSION.to_le_bytes(),
    );

    logger.write_buffered(
        &DUAL_CORE_RECORD_SIZE.to_le_bytes(),
    );

    logger.flush();
}

fn close_log_session(
    logger: &mut SdLogger,
) {
    // `close_file()` does not flush the software buffer, so flushing must
    // happen first.
    logger.flush();
    logger.close_file();
}

/// Permanent Core 1 SD logging task.
///
/// Commands:
/// - `StartSession`: open a new file
/// - `StopSession`: flush and close
/// - `Flush`: flush without closing
///
/// Frames received while no session is active are drained and discarded. This
/// prevents stale data from a previous mode being written into a later file.
#[embassy_executor::task]
pub async fn core1_logging_task(
    mut logger: Option<SdLogger>,
) {
    let mut session_active = false;
    let mut frames_since_flush = 0u32;

    if logger.is_some() {
        defmt::info!(
            "Core 1 logging service started"
        );
    } else {
        defmt::warn!(
            "Core 1 logging service started without SD card"
        );
    }

    loop {
        // Commands are placed first so mode transitions and stop requests have
        // priority when both futures are ready.
        match select(
            receive_logger_command(),
            receive_log_frame(),
        )
        .await
        {
            Either::First(command) => {
                match command {
                    LoggerCommand::StartSession(mode) => {
                        if let Some(logger) = logger.as_mut() {
                            open_log_session(logger);

                            defmt::info!(
                                "Core 1 log session started, mode={}",
                                mode.as_u8(),
                            );
                        }

                        session_active = true;
                        frames_since_flush = 0;
                    }

                    LoggerCommand::StopSession => {
                        session_active = false;
                        frames_since_flush = 0;

                        if let Some(logger) = logger.as_mut() {
                            close_log_session(logger);
                        }

                        defmt::info!(
                            "Core 1 log session stopped"
                        );
                    }

                    LoggerCommand::Flush => {
                        if let Some(logger) = logger.as_mut() {
                            logger.flush();
                        }

                        frames_since_flush = 0;
                    }
                }
            }

            Either::Second(frame) => {
                // Always drain frames, but only write during an active
                // logging session.
                if !session_active {
                    continue;
                }

                let Some(logger) = logger.as_mut() else {
                    continue;
                };

                let bytes = frame.to_bytes();

                logger.write_buffered(
                    bytes.as_slice(),
                );

                frames_since_flush =
                    frames_since_flush.saturating_add(1);

                if frames_since_flush >= FLUSH_EVERY_FRAMES {
                    logger.flush();
                    frames_since_flush = 0;
                }
            }
        }
    }
}