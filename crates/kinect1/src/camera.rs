//! Camera command protocol, with no I/O.
//!
//! Commands travel as vendor control transfers: the host sends a request (type 0x40, request 0)
//! starting with an 8-byte header `"GM"`, length in 16-bit words, command, tag, then polls with
//! type 0xC0 requests until the reply arrives with an `"RB"` header echoing command and tag.

use crate::{Error, Result};

pub const HEADER_LEN: usize = 8;
/// Size of the buffer the reply is polled into. A read of exactly this size means "not ready".
pub const REPLY_BUFFER_LEN: usize = 0x200;
const MAX_REQUEST_LEN: usize = 0x400;

const REQUEST_MAGIC: [u8; 2] = *b"GM";
const REPLY_MAGIC: [u8; 2] = *b"RB";

pub const READ_REGISTER: u16 = 0x02;
pub const WRITE_REGISTER: u16 = 0x03;
pub const GET_FIXED_PARAMS: u16 = 0x04;
pub const ALGORITHM_PARAMS: u16 = 0x16;

/// IR projector brightness, 1..=50.
pub const IR_BRIGHTNESS_REGISTER: u16 = 0x15;

fn u16_at(bytes: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([bytes[at], bytes[at + 1]])
}

/// Builds the request for `command` with `payload`, which must be a whole number of 16-bit words.
pub fn encode(command: u16, tag: u16, payload: &[u8]) -> Result<Vec<u8>> {
    if payload.len() % 2 != 0 || payload.len() > MAX_REQUEST_LEN - HEADER_LEN {
        return Err(Error::InvalidArgument(format!("camera command payload of {} bytes", payload.len())));
    }
    let header = [(payload.len() / 2) as u16, command, tag].into_iter().flat_map(u16::to_le_bytes);
    Ok(REQUEST_MAGIC.into_iter().chain(header).chain(payload.iter().copied()).collect())
}

/// Checks a reply to `command`/`tag` and returns its payload.
pub fn decode_reply(command: u16, tag: u16, reply: &[u8]) -> Result<&[u8]> {
    if reply.len() < HEADER_LEN || reply[..2] != REPLY_MAGIC {
        return Err(Error::Protocol(format!("malformed camera reply {:02x?}", &reply[..reply.len().min(16)])));
    }
    let payload = &reply[HEADER_LEN..];
    let (words, got_command, got_tag) = (u16_at(reply, 2), u16_at(reply, 4), u16_at(reply, 6));
    if got_command != command || got_tag != tag {
        return Err(Error::Protocol(format!(
            "camera replied to command {got_command:#x} tag {got_tag} instead of {command:#x} tag {tag}"
        )));
    }
    if usize::from(words) * 2 != payload.len() {
        return Err(Error::Protocol(format!("camera reply claims {words} words but has {} bytes", payload.len())));
    }
    Ok(payload)
}

pub fn read_register_payload(register: u16) -> [u8; 2] {
    register.to_le_bytes()
}

pub fn write_register_payload(register: u16, value: u16) -> [u8; 4] {
    let [r0, r1] = register.to_le_bytes();
    let [v0, v1] = value.to_le_bytes();
    [r0, r1, v0, v1]
}

/// The value from a register read reply: a status word, then the value.
pub fn parse_read_register(payload: &[u8]) -> Result<u16> {
    match payload {
        [0, 0, lo, hi] => Ok(u16::from_le_bytes([*lo, *hi])),
        _ => Err(Error::Protocol(format!("register read failed: {payload:02x?}"))),
    }
}

/// Checks a register write reply: a single zero status word.
pub fn check_write_register(payload: &[u8]) -> Result<()> {
    match payload {
        [0, 0] => Ok(()),
        _ => Err(Error::Protocol(format!("register write failed: {payload:02x?}"))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encodes_register_write() {
        let request = encode(WRITE_REGISTER, 5, &write_register_payload(0x105, 0)).unwrap();
        assert_eq!(request, [b'G', b'M', 2, 0, 3, 0, 5, 0, 0x05, 0x01, 0, 0]);
        assert!(encode(WRITE_REGISTER, 0, &[1, 2, 3]).is_err());
    }

    #[test]
    fn decodes_replies() {
        let reply = [b'R', b'B', 2, 0, 2, 0, 9, 0, 0, 0, 0x1e, 0];
        let payload = decode_reply(READ_REGISTER, 9, &reply).unwrap();
        assert_eq!(parse_read_register(payload).unwrap(), 0x1e);

        assert!(decode_reply(READ_REGISTER, 8, &reply).is_err(), "wrong tag");
        assert!(decode_reply(WRITE_REGISTER, 9, &reply).is_err(), "wrong command");
        assert!(decode_reply(READ_REGISTER, 9, &reply[..10]).is_err(), "length mismatch");
        assert!(decode_reply(READ_REGISTER, 9, b"GM\0\0\x02\0\x09\0").is_err(), "bad magic");
    }

    #[test]
    fn checks_register_statuses() {
        assert!(check_write_register(&[0, 0]).is_ok());
        assert!(check_write_register(&[1, 0]).is_err());
        assert!(parse_read_register(&[1, 0, 5, 0]).is_err());
    }
}
