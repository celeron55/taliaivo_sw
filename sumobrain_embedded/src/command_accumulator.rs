#![no_std]

extern crate arrayvec;

use arrayvec::{ArrayVec, ArrayString};

pub struct CommandAccumulator<const BUF_LEN: usize> {
    buf: ArrayString<BUF_LEN>,
}

impl<const BUF_LEN: usize> CommandAccumulator<BUF_LEN> {
    pub fn new() -> Self {
        CommandAccumulator {
            buf: ArrayString::new(),
        }
    }

    pub fn put(&mut self, c: char) -> Option<ArrayString<BUF_LEN>> {
        // TODO
        if c == '\n' || c == '\r' {
            if self.buf.len() > 0 {
                // NOTE: This is a bit wasteful, we could instead use a
                // swappable buffer and swap in an empty one, returning the
                // existing buffer as-is
                let command = self.buf.clone();
                self.buf.clear();
                return Some(command)
            }
        } else if c == '\u{7f}' {
            // Backspace: Edit the command
            if self.buf.len() > 0 {
                self.buf.truncate(self.buf.len() - 1)
            }
        } else {
            self.buf.push(c);
        }
        None
    }
}