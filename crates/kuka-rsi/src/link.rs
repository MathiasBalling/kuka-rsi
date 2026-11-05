//! UDP server side of the RSI link: the robot sends first, we answer the newest datagram.

use std::io::ErrorKind;
use std::net::{SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use crate::{Error, Result};

/// Metadata of a received datagram.
#[derive(Clone, Copy, Debug)]
pub struct Received {
    pub len: usize,
    pub from: SocketAddr,
    pub arrived: Instant,
    /// Older datagrams discarded in favour of this one.
    pub skipped: u32,
}

pub struct UdpLink {
    socket: UdpSocket,
    peer: Option<SocketAddr>,
    scratch: Vec<u8>,
    foreign: u64,
    read_timeout: Duration,
}

impl UdpLink {
    /// Bind exclusively (no `SO_REUSEADDR`, so a second process on the port fails loudly).
    pub fn bind(addr: SocketAddr, read_timeout: Duration) -> Result<UdpLink> {
        let domain = if addr.is_ipv4() { socket2::Domain::IPV4 } else { socket2::Domain::IPV6 };
        let sock = socket2::Socket::new(domain, socket2::Type::DGRAM, Some(socket2::Protocol::UDP))?;
        sock.set_reuse_address(false)?;
        sock.bind(&addr.into())
            .map_err(|e| Error::Io(std::io::Error::new(e.kind(), format!("cannot bind UDP {addr}: {e}"))))?;
        let socket: UdpSocket = sock.into();
        socket.set_read_timeout(Some(read_timeout))?;
        Ok(UdpLink { socket, peer: None, scratch: vec![0u8; 65536], foreign: 0, read_timeout })
    }

    pub fn local_addr(&self) -> Result<SocketAddr> {
        Ok(self.socket.local_addr()?)
    }

    pub fn peer(&self) -> Option<SocketAddr> {
        self.peer
    }

    /// Forget the locked peer so the next datagram from anyone starts a new session.
    pub fn reset_peer(&mut self) {
        self.peer = None;
    }

    /// Datagrams ignored because they came from an address other than the peer.
    pub fn foreign(&self) -> u64 {
        self.foreign
    }

    fn accept(&mut self, from: SocketAddr) -> bool {
        match self.peer {
            None => {
                self.peer = Some(from);
                true
            }
            Some(p) => p == from,
        }
    }

    /// Wait up to the read timeout for a datagram from the peer, then discard any older
    /// queued datagrams and return the newest. `None` on timeout.
    ///
    /// A datagram larger than `buf` is an error (`Error::Protocol`), not a silent
    /// truncation. Datagrams from a source other than the peer are counted in
    /// `foreign` and do not extend the overall wait past `read_timeout`, even if they
    /// keep arriving.
    pub fn recv_newest(&mut self, buf: &mut [u8]) -> Result<Option<Received>> {
        let deadline = Instant::now() + self.read_timeout;
        let mut shortened = false;

        let mut result = loop {
            match self.socket.recv_from(&mut self.scratch) {
                Ok((n, from)) => {
                    if self.accept(from) {
                        if n > buf.len() {
                            self.restore_read_timeout(shortened)?;
                            return Err(Error::Protocol(format!(
                                "datagram of {n} bytes exceeds the {} byte buffer",
                                buf.len()
                            )));
                        }
                        buf[..n].copy_from_slice(&self.scratch[..n]);
                        break Received { len: n, from, arrived: Instant::now(), skipped: 0 };
                    }
                    self.foreign += 1;
                    let now = Instant::now();
                    if now >= deadline {
                        self.restore_read_timeout(shortened)?;
                        return Ok(None);
                    }
                    self.socket.set_read_timeout(Some(deadline - now))?;
                    shortened = true;
                }
                Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {
                    self.restore_read_timeout(shortened)?;
                    return Ok(None);
                }
                Err(e) => {
                    self.restore_read_timeout(shortened)?;
                    return Err(e.into());
                }
            }
        };
        self.restore_read_timeout(shortened)?;

        self.socket.set_nonblocking(true)?;
        let drained = loop {
            match self.socket.recv_from(&mut self.scratch) {
                Ok((n, from)) => {
                    if self.accept(from) {
                        if n > buf.len() {
                            break Err(Error::Protocol(format!(
                                "datagram of {n} bytes exceeds the {} byte buffer",
                                buf.len()
                            )));
                        }
                        buf[..n].copy_from_slice(&self.scratch[..n]);
                        result.len = n;
                        result.from = from;
                        result.arrived = Instant::now();
                        result.skipped += 1;
                    } else {
                        self.foreign += 1;
                    }
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => break Ok(()),
                Err(e) => break Err(Error::from(e)),
            }
        };
        self.socket.set_nonblocking(false)?;
        drained?;
        Ok(Some(result))
    }

    /// Restore the socket's read timeout to `self.read_timeout` if it was shortened to
    /// respect an overall deadline while waiting out a run of foreign datagrams.
    fn restore_read_timeout(&self, shortened: bool) -> Result<()> {
        if shortened {
            self.socket.set_read_timeout(Some(self.read_timeout))?;
        }
        Ok(())
    }

    /// Send a reply to the locked peer.
    pub fn send(&self, data: &[u8]) -> Result<()> {
        let peer = self.peer.ok_or_else(|| Error::Protocol("no peer: nothing received from the robot yet".into()))?;
        self.socket.send_to(data, peer)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::UdpSocket;
    use std::time::Duration;

    fn link() -> UdpLink {
        UdpLink::bind("127.0.0.1:0".parse().unwrap(), Duration::from_millis(50)).unwrap()
    }

    #[test]
    fn first_datagram_locks_peer_and_reply_goes_back() {
        let mut l = link();
        let robot = UdpSocket::bind("127.0.0.1:0").unwrap();
        robot.set_read_timeout(Some(Duration::from_millis(500))).unwrap();
        assert!(l.send(b"x").is_err(), "no peer yet");
        robot.send_to(b"hello", l.local_addr().unwrap()).unwrap();
        let mut buf = [0u8; 64];
        let r = l.recv_newest(&mut buf).unwrap().unwrap();
        assert_eq!(&buf[..r.len], b"hello");
        assert_eq!(r.from, robot.local_addr().unwrap());
        assert_eq!(r.skipped, 0);
        assert_eq!(l.peer(), Some(robot.local_addr().unwrap()));
        l.send(b"reply").unwrap();
        let mut rb = [0u8; 64];
        let (n, _) = robot.recv_from(&mut rb).unwrap();
        assert_eq!(&rb[..n], b"reply");
    }

    #[test]
    fn drains_to_newest_datagram() {
        let mut l = link();
        let robot = UdpSocket::bind("127.0.0.1:0").unwrap();
        for m in [b"one", b"two", b"three".as_slice()] {
            robot.send_to(m, l.local_addr().unwrap()).unwrap();
        }
        std::thread::sleep(Duration::from_millis(20));
        let mut buf = [0u8; 64];
        let r = l.recv_newest(&mut buf).unwrap().unwrap();
        assert_eq!(&buf[..r.len], b"three");
        assert_eq!(r.skipped, 2);
        assert!(l.recv_newest(&mut buf).unwrap().is_none(), "queue is empty now");
    }

    #[test]
    fn timeout_returns_none() {
        let mut l = link();
        let mut buf = [0u8; 64];
        let t0 = Instant::now();
        assert!(l.recv_newest(&mut buf).unwrap().is_none());
        assert!(t0.elapsed() >= Duration::from_millis(40));
    }

    #[test]
    fn foreign_sources_are_ignored_until_reset() {
        let mut l = link();
        let robot = UdpSocket::bind("127.0.0.1:0").unwrap();
        let other = UdpSocket::bind("127.0.0.1:0").unwrap();
        robot.send_to(b"r1", l.local_addr().unwrap()).unwrap();
        let mut buf = [0u8; 64];
        l.recv_newest(&mut buf).unwrap().unwrap();
        other.send_to(b"o1", l.local_addr().unwrap()).unwrap();
        std::thread::sleep(Duration::from_millis(10));
        assert!(l.recv_newest(&mut buf).unwrap().is_none());
        assert_eq!(l.foreign(), 1);
        other.send_to(b"o2", l.local_addr().unwrap()).unwrap();
        robot.send_to(b"r2", l.local_addr().unwrap()).unwrap();
        std::thread::sleep(Duration::from_millis(10));
        let r = l.recv_newest(&mut buf).unwrap().unwrap();
        assert_eq!(&buf[..r.len], b"r2");
        assert_eq!(l.foreign(), 2);
        l.reset_peer();
        other.send_to(b"o3", l.local_addr().unwrap()).unwrap();
        let r = l.recv_newest(&mut buf).unwrap().unwrap();
        assert_eq!(&buf[..r.len], b"o3");
        assert_eq!(l.peer(), Some(other.local_addr().unwrap()));
    }

    #[test]
    fn bind_twice_fails() {
        let l = link();
        assert!(UdpLink::bind(l.local_addr().unwrap(), Duration::from_millis(10)).is_err());
    }

    #[test]
    fn oversized_datagram_is_an_error() {
        let mut l = link();
        let robot = UdpSocket::bind("127.0.0.1:0").unwrap();
        robot.send_to(b"0123456789abcdef", l.local_addr().unwrap()).unwrap();
        let mut buf = [0u8; 8];
        let result = l.recv_newest(&mut buf);
        assert!(matches!(result, Err(Error::Protocol(_))), "{result:?}");
        robot.send_to(b"WXYZ", l.local_addr().unwrap()).unwrap();
        let r = l.recv_newest(&mut buf).unwrap().unwrap();
        assert_eq!(&buf[..r.len], b"WXYZ", "socket must be back in blocking mode with its timeout");
    }

    #[test]
    fn foreign_flood_does_not_extend_the_wait() {
        let mut l = link();
        let addr = l.local_addr().unwrap();

        // Lock the peer first so the flood below is genuinely foreign, not the datagram
        // that locks the peer.
        let robot = UdpSocket::bind("127.0.0.1:0").unwrap();
        robot.send_to(b"lock", addr).unwrap();
        let mut buf = [0u8; 64];
        l.recv_newest(&mut buf).unwrap().unwrap();

        // Flood until the wait is over, without sleeping: on CI VMs a millisecond sleep can take
        // 10 ms or more, so a paced flood may not reach the link often enough to prove anything.
        let other = UdpSocket::bind("127.0.0.1:0").unwrap();
        let done = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let flooding = done.clone();
        let handle = std::thread::spawn(move || {
            let t0 = Instant::now();
            while !flooding.load(std::sync::atomic::Ordering::SeqCst) && t0.elapsed() < Duration::from_secs(2) {
                let _ = other.send_to(b"x", addr); // a full socket buffer may refuse some
                std::thread::yield_now();
            }
        });

        let t0 = Instant::now();
        let r = l.recv_newest(&mut buf).unwrap();
        let elapsed = t0.elapsed();
        done.store(true, std::sync::atomic::Ordering::SeqCst);
        handle.join().unwrap();
        assert!(r.is_none());
        assert!(elapsed < Duration::from_millis(200), "elapsed = {elapsed:?}");
        assert!(l.foreign() >= 5, "foreign = {}", l.foreign());
    }
}
