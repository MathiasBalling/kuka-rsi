//! Crate-wide error type.

use std::time::Duration;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("config: {0}")]
    Config(String),
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("timeout waiting for the robot")]
    RobotTimeout,
    #[error("timeout waiting for the queue to drain")]
    IdleTimeout,
    #[error("link lost: no telegram for {0:?}")]
    LinkLost(Duration),
    #[error("protocol: {0}")]
    Protocol(String),
    #[error("limit: {0}")]
    Limit(String),
    #[error("command queue is full")]
    QueueFull,
    #[error("driver faulted: motion and messages are refused until the KRL program restarts (a new session)")]
    Faulted,
    #[error("driver is closed")]
    Closed,
}

pub type Result<T> = std::result::Result<T, Error>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_display_mentions_cause() {
        let e = Error::Config("PORT out of range".into());
        assert_eq!(e.to_string(), "config: PORT out of range");
        let e: Error = std::io::Error::other("boom").into();
        assert!(e.to_string().starts_with("io: "));
        assert_eq!(
            Error::LinkLost(std::time::Duration::from_millis(500)).to_string(),
            "link lost: no telegram for 500ms"
        );
    }
}
