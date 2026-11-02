//! Python bindings for the kuka-rsi driver.

use std::net::SocketAddr;
use std::time::Duration;

use numpy::{PyArray1, PyReadonlyArray1, PyReadonlyArrayDyn};
use pyo3::create_exception;
use pyo3::exceptions::{PyException, PyOSError, PyTimeoutError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyDict;

use kuka_rsi::config::{DriverConfig, EthernetConfig, Limits};
use kuka_rsi::driver::Driver;
use kuka_rsi::sim::{ControllerSim, SimConfig, SimExit, SimHandle, SimStatus};
use kuka_rsi::state::{RobotState, SessionState};
use kuka_rsi::stats::Stats;

create_exception!(kuka_rsi, Error, PyException, "Base class for kuka_rsi errors.");
create_exception!(kuka_rsi, LinkLost, Error, "No telegram from the robot for longer than link_timeout.");
create_exception!(kuka_rsi, ProtocolError, Error, "The robot is not connected or sent something unexpected.");
create_exception!(kuka_rsi, LimitError, Error, "A target or parameter is outside its limits.");

pub(crate) fn to_py(e: kuka_rsi::Error) -> PyErr {
    use kuka_rsi::Error as E;
    match e {
        E::RobotTimeout | E::IdleTimeout => PyTimeoutError::new_err(e.to_string()),
        E::LinkLost(_) => LinkLost::new_err(e.to_string()),
        E::Protocol(_) => ProtocolError::new_err(e.to_string()),
        E::Limit(_) | E::QueueFull => LimitError::new_err(e.to_string()),
        E::Io(_) => PyOSError::new_err(e.to_string()),
        // A faulted driver is reported as a limit error: no dedicated exception exists for it.
        E::Faulted => LimitError::new_err(e.to_string()),
        E::Config(_) | E::Closed => Error::new_err(e.to_string()),
    }
}

/// Converts a numpy target array without the intermediate `Vec<f64>` that `vec6` requires
/// (used by `Driver::move_to`'s `q` argument).
pub(crate) fn arr6(v: PyReadonlyArray1<'_, f64>, what: &str) -> PyResult<[f64; 6]> {
    let s = v.as_slice()?;
    if s.len() != 6 {
        return Err(PyValueError::new_err(format!("{what} must have 6 values, got {}", s.len())));
    }
    let mut out = [0.0; 6];
    out.copy_from_slice(s);
    Ok(out)
}

pub(crate) fn vec6(v: Vec<f64>, what: &str) -> PyResult<[f64; 6]> {
    if v.len() != 6 {
        return Err(PyValueError::new_err(format!("{what} must have 6 values, got {}", v.len())));
    }
    let mut out = [0.0; 6];
    out.copy_from_slice(&v);
    Ok(out)
}

pub(crate) fn np6<'py>(py: Python<'py>, v: &[f64; 6]) -> Bound<'py, PyArray1<f64>> {
    PyArray1::from_slice(py, v)
}

fn parse_addr(s: &str, what: &str) -> PyResult<SocketAddr> {
    s.parse().map_err(|_| PyValueError::new_err(format!("{what} {s:?} is not host:port")))
}

// `skip_from_py_object` opts out of pyo3 0.29's now-deprecated automatic `FromPyObject`
// derive for `Clone` pyclasses; nothing here extracts a `PyLimits` by value from Python.
#[pyclass(name = "Limits", module = "kuka_rsi", skip_from_py_object)]
#[derive(Clone)]
pub struct PyLimits {
    pub inner: Limits,
}

#[pymethods]
impl PyLimits {
    /// KR 10 R1100-2 ranges with conservative speeds; scalars apply to all joints (rad/s, rad/s²).
    #[staticmethod]
    // `max_acceleration`'s default is `Some(300°/s²)` (`300f64.to_radians()`, matching
    // `Limits::kr10_defaults()`'s own default exactly), not `None`: pyo3 cannot tell
    // "omitted" from "explicitly `None`" on a plain `Option<f64>`, and the KR10 default
    // must keep the acceleration clamp on. Pass `max_acceleration=None` explicitly to
    // disable the clamp; pass `max_acceleration=<value>` to set all six joints to it.
    #[pyo3(signature = (max_velocity=None, max_acceleration=Some(300f64.to_radians())))]
    fn kr10_defaults(max_velocity: Option<f64>, max_acceleration: Option<f64>) -> PyLimits {
        let mut l = Limits::kr10_defaults();
        if let Some(v) = max_velocity {
            l = l.with_max_velocity_all(v);
        }
        l = l.with_max_acceleration_all(max_acceleration); // Some(v) sets all joints, None disables
        PyLimits { inner: l }
    }

    #[getter]
    fn position_min<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        np6(py, &self.inner.position_min)
    }
    #[setter]
    fn set_position_min(&mut self, v: Vec<f64>) -> PyResult<()> {
        self.inner.position_min = vec6(v, "position_min")?;
        Ok(())
    }
    #[getter]
    fn position_max<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        np6(py, &self.inner.position_max)
    }
    #[setter]
    fn set_position_max(&mut self, v: Vec<f64>) -> PyResult<()> {
        self.inner.position_max = vec6(v, "position_max")?;
        Ok(())
    }
    #[getter]
    fn max_velocity<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        np6(py, &self.inner.max_velocity)
    }
    #[setter]
    fn set_max_velocity(&mut self, v: Vec<f64>) -> PyResult<()> {
        self.inner.max_velocity = vec6(v, "max_velocity")?;
        Ok(())
    }
    #[getter]
    fn max_correction<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        np6(py, &self.inner.max_correction)
    }
    #[setter]
    fn set_max_correction(&mut self, v: Vec<f64>) -> PyResult<()> {
        self.inner.max_correction = vec6(v, "max_correction")?;
        Ok(())
    }
    #[getter]
    fn max_acceleration<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.inner.max_acceleration.as_ref().map(|a| np6(py, a))
    }
    #[setter]
    fn set_max_acceleration(&mut self, v: Option<Vec<f64>>) -> PyResult<()> {
        self.inner.max_acceleration = match v {
            Some(v) => Some(vec6(v, "max_acceleration")?),
            None => None,
        };
        Ok(())
    }
    #[getter]
    fn max_tracking_error(&self) -> f64 {
        self.inner.max_tracking_error
    }
    #[setter]
    fn set_max_tracking_error(&mut self, v: f64) {
        self.inner.max_tracking_error = v;
    }

    fn validate(&self) -> PyResult<()> {
        self.inner.validate().map_err(to_py)
    }

    fn __repr__(&self) -> String {
        format!("{:?}", self.inner)
    }
}

fn exit_name(e: SimExit) -> String {
    match e {
        SimExit::Stopped => "Stopped".into(),
        SimExit::RsiBad { .. } => "RsiBad".into(),
        SimExit::CorrMonExceeded { .. } => "CorrMonExceeded".into(),
        SimExit::EStrError => "EStrError".into(),
        SimExit::MaxCycles => "MaxCycles".into(),
        SimExit::Shutdown => "Shutdown".into(),
    }
}

fn status_dict<'py>(py: Python<'py>, s: &SimStatus) -> PyResult<Bound<'py, PyDict>> {
    let d = PyDict::new(py);
    d.set_item("cycles", s.cycles)?;
    d.set_item("replies", s.replies)?;
    d.set_item("late", s.late)?;
    d.set_item("ipoc", s.ipoc)?;
    d.set_item("q_setpoint_deg", np6(py, &s.q_setpoint_deg))?;
    d.set_item("q_actual_deg", np6(py, &s.q_actual_deg))?;
    d.set_item("last_ak_deg", np6(py, &s.last_ak_deg))?;
    d.set_item("clamped", s.clamped)?;
    d.set_item("overruns", s.overruns)?;
    d.set_item("exit", s.exit.map(exit_name))?;
    Ok(d)
}

/// Fake controller running in a background thread from construction on.
#[pyclass(name = "ControllerSim", module = "kuka_rsi")]
pub struct PyControllerSim {
    handle: Option<SimHandle>,
    last_exit: Option<SimExit>,
    /// Set when a `stop()` call takes the handle but `SimHandle::stop` itself errors (e.g.
    /// the simulator thread panicked); a later `stop()`/`__exit__` call has no handle left
    /// to retry against, so it re-raises this instead of silently reporting "Shutdown".
    stop_error: Option<String>,
}

#[pymethods]
impl PyControllerSim {
    #[new]
    #[pyo3(signature = (config, target, *, bind="0.0.0.0:0", cycle_ms=4.0, timeout=100, lag_cycles=0.0, jitter_us=0, drop_probability=0.0, start_deg=None, max_cycles=None, seed=None))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        config: &str,
        target: &str,
        bind: &str,
        cycle_ms: f64,
        timeout: u32,
        lag_cycles: f64,
        jitter_us: u64,
        drop_probability: f64,
        start_deg: Option<Vec<f64>>,
        max_cycles: Option<u64>,
        seed: Option<u64>,
    ) -> PyResult<Self> {
        let eth = EthernetConfig::from_path(config).map_err(to_py)?;
        let mut cfg = SimConfig::new(eth, parse_addr(target, "target")?);
        cfg.bind = parse_addr(bind, "bind")?;
        cfg.cycle = Duration::from_secs_f64(cycle_ms / 1000.0);
        cfg.timeout = timeout;
        cfg.lag_cycles = lag_cycles;
        cfg.jitter = Duration::from_micros(jitter_us);
        cfg.drop_probability = drop_probability;
        cfg.max_cycles = max_cycles;
        if let Some(s) = start_deg {
            cfg.q_start_deg = vec6(s, "start_deg")?;
        }
        if let Some(s) = seed {
            cfg.seed = s;
        }
        let handle = ControllerSim::spawn(cfg).map_err(to_py)?;
        Ok(PyControllerSim { handle: Some(handle), last_exit: None, stop_error: None })
    }

    fn local_addr(&self) -> PyResult<String> {
        self.handle.as_ref().map(|h| h.local_addr().to_string()).ok_or_else(|| Error::new_err("simulator stopped"))
    }

    fn status<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        match &self.handle {
            Some(h) => status_dict(py, &h.status()),
            None => {
                let mut s = SimStatus::default();
                s.exit = self.last_exit;
                status_dict(py, &s)
            }
        }
    }

    fn wait_exit(&self, py: Python<'_>, timeout: f64) -> Option<String> {
        let h = self.handle.as_ref()?;
        py.detach(|| h.wait_exit(Duration::from_secs_f64(timeout))).map(exit_name)
    }

    fn stop(&mut self, py: Python<'_>) -> PyResult<String> {
        if let Some(msg) = &self.stop_error {
            return Err(Error::new_err(msg.clone()));
        }
        if let Some(mut h) = self.handle.take() {
            match py.detach(|| h.stop()) {
                Ok(e) => self.last_exit = Some(e),
                Err(e) => {
                    self.stop_error = Some(e.to_string());
                    return Err(to_py(e));
                }
            }
        }
        Ok(self.last_exit.map(exit_name).unwrap_or_else(|| "Shutdown".into()))
    }

    fn __enter__(slf: PyRef<'_, Self>) -> PyRef<'_, Self> {
        slf
    }

    #[pyo3(signature = (_t=None, _v=None, _tb=None))]
    fn __exit__(&mut self, py: Python<'_>, _t: Option<Bound<'_, PyAny>>, _v: Option<Bound<'_, PyAny>>, _tb: Option<Bound<'_, PyAny>>) -> PyResult<bool> {
        self.stop(py)?;
        Ok(false)
    }
}

fn session_name(s: SessionState) -> &'static str {
    match s {
        SessionState::Listening => "Listening",
        SessionState::Active => "Active",
        SessionState::Lost => "Lost",
    }
}

#[pyclass(name = "RobotState", module = "kuka_rsi", frozen)]
pub struct PyRobotState {
    inner: RobotState,
}

#[pymethods]
impl PyRobotState {
    #[getter]
    fn t(&self) -> f64 {
        self.inner.t
    }
    #[getter]
    fn cycle(&self) -> u64 {
        self.inner.cycle
    }
    #[getter]
    fn ipoc(&self) -> u64 {
        self.inner.ipoc
    }
    #[getter]
    fn session(&self) -> &'static str {
        session_name(self.inner.session)
    }
    #[getter]
    fn q<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        np6(py, &self.inner.q)
    }
    #[getter]
    fn q_setpoint<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        np6(py, &self.inner.q_setpoint)
    }
    #[getter]
    fn tcp<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        np6(py, &self.inner.tcp)
    }
    #[getter]
    fn tcp_setpoint<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        np6(py, &self.inner.tcp_setpoint)
    }
    #[getter]
    fn torque<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        np6(py, &self.inner.torque)
    }
    #[getter]
    fn q_command<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        np6(py, &self.inner.q_command)
    }
    #[getter]
    fn ak<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        np6(py, &self.inner.ak)
    }
    #[getter]
    fn delay(&self) -> u32 {
        self.inner.delay
    }
    #[getter]
    fn speed_override(&self) -> f64 {
        self.inner.speed_override
    }
    #[getter]
    fn program_running(&self) -> bool {
        self.inner.program_running
    }

    fn as_dict<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let s = &self.inner;
        let d = PyDict::new(py);
        d.set_item("t", s.t)?;
        d.set_item("cycle", s.cycle)?;
        d.set_item("ipoc", s.ipoc)?;
        d.set_item("session", session_name(s.session))?;
        d.set_item("q", np6(py, &s.q))?;
        d.set_item("q_setpoint", np6(py, &s.q_setpoint))?;
        d.set_item("tcp", np6(py, &s.tcp))?;
        d.set_item("tcp_setpoint", np6(py, &s.tcp_setpoint))?;
        d.set_item("torque", np6(py, &s.torque))?;
        d.set_item("delay", s.delay)?;
        d.set_item("speed_override", s.speed_override)?;
        d.set_item("program_running", s.program_running)?;
        d.set_item("q_command", np6(py, &s.q_command))?;
        d.set_item("ak", np6(py, &s.ak))?;
        Ok(d)
    }

    fn __repr__(&self) -> String {
        format!("{:?}", self.inner)
    }
}

#[pyclass(name = "Stats", module = "kuka_rsi", frozen)]
pub struct PyStats {
    inner: Stats,
}

macro_rules! stats_getters {
    ($($name:ident : $ty:ty),* $(,)?) => {
        #[pymethods]
        impl PyStats {
            $(
                #[getter]
                fn $name(&self) -> $ty { self.inner.$name }
            )*

            fn as_dict<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
                let d = PyDict::new(py);
                $( d.set_item(stringify!($name), self.inner.$name)?; )*
                Ok(d)
            }

            fn __repr__(&self) -> String {
                format!("{:?}", self.inner)
            }
        }
    };
}

stats_getters!(
    sessions: u64, cycles: u64, replies: u64, late_replies: u64, reply_latency_max_us: u64, reply_latency_mean_us: f64,
    interarrival_max_us: u64, ipoc_gaps: u64, skipped_datagrams: u64, parse_errors: u64, io_errors: u64, sentype_mismatches: u64,
    position_clamps: u64, velocity_clamps: u64, acceleration_clamps: u64, correction_clamps: u64, starved_cycles: u64,
    log_drops: u64, queue_len: u32, robot_delay: u32,
);

/// Coerces any array-like (a plain list, a non-float64 array, ...) to a float64 numpy array
/// before extraction, so `stream`/`move_to` accept `ArrayLike` as their stubs promise instead of
/// raising a confusing `TypeError` on anything that isn't already a float64 `ndarray`.
fn as_f64_array<'py>(py: Python<'py>, obj: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyAny>> {
    let np = py.import("numpy")?;
    let float64 = np.getattr("float64")?;
    np.call_method1("asarray", (obj, float64))
}

fn points_from(points: PyReadonlyArrayDyn<'_, f64>) -> PyResult<Vec<[f64; 6]>> {
    let a = points.as_array();
    match a.ndim() {
        1 => {
            if a.len() != 6 {
                return Err(PyValueError::new_err(format!("a point must have 6 values, got {}", a.len())));
            }
            let mut p = [0.0; 6];
            for i in 0..6 {
                p[i] = a[[i]];
            }
            Ok(vec![p])
        }
        2 => {
            if a.shape()[1] != 6 {
                return Err(PyValueError::new_err(format!("points must be (n, 6), got {:?}", a.shape())));
            }
            Ok((0..a.shape()[0])
                .map(|r| {
                    let mut p = [0.0; 6];
                    for i in 0..6 {
                        p[i] = a[[r, i]];
                    }
                    p
                })
                .collect())
        }
        n => Err(PyValueError::new_err(format!("points must be 1-D or 2-D, got {n}-D"))),
    }
}

#[pyclass(name = "Driver", module = "kuka_rsi")]
pub struct PyDriver {
    inner: Driver,
    wait: f64,
}

#[pymethods]
impl PyDriver {
    #[new]
    #[pyo3(signature = (config, *, bind=None, limits=None, stop_channel=None, link_timeout=0.5, rt_priority=true, cycle_ms=4.0, wait=60.0))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        config: &str,
        bind: Option<&str>,
        limits: Option<&PyLimits>,
        stop_channel: Option<&str>,
        link_timeout: f64,
        rt_priority: bool,
        cycle_ms: f64,
        wait: f64,
    ) -> PyResult<Self> {
        let mut cfg = DriverConfig::from_ethernet_xml(config).map_err(to_py)?;
        if let Some(b) = bind {
            cfg = cfg.bind(parse_addr(b, "bind")?);
        }
        if let Some(l) = limits {
            cfg = cfg.limits(l.inner);
        }
        if let Some(s) = stop_channel {
            cfg = cfg.stop_channel(s);
        }
        cfg = cfg.link_timeout(Duration::from_secs_f64(link_timeout)).rt_priority(rt_priority).cycle(Duration::from_secs_f64(cycle_ms / 1000.0));
        Ok(PyDriver { inner: Driver::spawn(cfg).map_err(to_py)?, wait })
    }

    fn local_addr(&self) -> String {
        self.inner.local_addr().to_string()
    }

    fn session(&self) -> &'static str {
        session_name(self.inner.session())
    }

    #[pyo3(signature = (timeout=60.0))]
    fn wait_for_robot(&self, py: Python<'_>, timeout: f64) -> PyResult<()> {
        py.detach(|| self.inner.wait_for_robot(Duration::from_secs_f64(timeout))).map_err(to_py)
    }

    /// Waits for every queued command to be applied, the target queue to be empty and the
    /// command to have settled (reached the last target and the motion limiter to have caught
    /// up). Raises `TimeoutError` (`Error::IdleTimeout`) if a clamp keeps the command short of
    /// the target, or the program is paused, for longer than `timeout`.
    #[pyo3(signature = (timeout=10.0))]
    fn wait_idle(&self, py: Python<'_>, timeout: f64) -> PyResult<()> {
        py.detach(|| self.inner.wait_idle(Duration::from_secs_f64(timeout))).map_err(to_py)
    }

    fn state(&self) -> PyRobotState {
        PyRobotState { inner: self.inner.state() }
    }

    fn stats(&self) -> PyStats {
        PyStats { inner: self.inner.stats() }
    }

    fn events(&self) -> Vec<String> {
        self.inner.events().iter().map(|e| format!("{e:?}")).collect()
    }

    fn drain_log(&self) -> Vec<PyRobotState> {
        let mut v = Vec::new();
        self.inner.drain_log(&mut v);
        v.into_iter().map(|s| PyRobotState { inner: s }).collect()
    }

    #[pyo3(signature = (points, dt, replace=false))]
    fn stream(&self, py: Python<'_>, points: &Bound<'_, PyAny>, dt: f64, replace: bool) -> PyResult<()> {
        let arr = as_f64_array(py, points)?;
        let arr: PyReadonlyArrayDyn<'_, f64> = arr.extract()?;
        let pts = points_from(arr)?;
        self.inner.stream(&pts, Duration::from_secs_f64(dt.max(0.0)), replace).map_err(to_py)
    }

    /// `duration + 5.0` s of headroom past the requested duration when `block=True`, so a
    /// legitimate clamp-limited move that runs longer than `duration` still completes instead
    /// of raising `TimeoutError` early.
    #[pyo3(signature = (q, duration, block=false))]
    fn move_to(&self, py: Python<'_>, q: &Bound<'_, PyAny>, duration: f64, block: bool) -> PyResult<()> {
        let arr = as_f64_array(py, q)?;
        let arr: PyReadonlyArray1<'_, f64> = arr.extract()?;
        let q = arr6(arr, "q")?;
        self.inner.move_to(q, Duration::from_secs_f64(duration.max(0.0))).map_err(to_py)?;
        if block {
            py.detach(|| self.inner.wait_idle(Duration::from_secs_f64(duration + 5.0))).map_err(to_py)?;
        }
        Ok(())
    }

    fn hold(&self) -> PyResult<()> {
        self.inner.hold().map_err(to_py)
    }

    fn fault(&self, msg: &str) -> PyResult<()> {
        self.inner.fault(msg).map_err(to_py)
    }

    fn notify(&self, msg: &str) -> PyResult<()> {
        self.inner.notify(msg).map_err(to_py)
    }

    fn stop_program(&self) -> PyResult<()> {
        self.inner.stop_program().map_err(to_py)
    }

    fn set_speed_scale(&self, s: f64) -> PyResult<()> {
        self.inner.set_speed_scale(s).map_err(to_py)
    }

    fn set_limits(&self, limits: &PyLimits) -> PyResult<()> {
        self.inner.set_limits(limits.inner).map_err(to_py)
    }

    fn close(&self, py: Python<'_>) -> PyResult<()> {
        py.detach(|| self.inner.close()).map_err(to_py)
    }

    fn __enter__<'a>(slf: PyRef<'a, Self>, py: Python<'a>) -> PyResult<PyRef<'a, Self>> {
        let wait = slf.wait;
        let inner: &Driver = &slf.inner; // `&Driver` is Send; a `PyRef` is not
        py.detach(|| inner.wait_for_robot(Duration::from_secs_f64(wait))).map_err(to_py)?;
        Ok(slf)
    }

    #[pyo3(signature = (_t=None, _v=None, _tb=None))]
    fn __exit__(&self, py: Python<'_>, _t: Option<Bound<'_, PyAny>>, _v: Option<Bound<'_, PyAny>>, _tb: Option<Bound<'_, PyAny>>) -> PyResult<bool> {
        self.close(py)?;
        Ok(false)
    }
}

#[pymodule]
fn _core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    pyo3_log::init();
    m.add_class::<PyLimits>()?;
    m.add_class::<PyControllerSim>()?;
    m.add_class::<PyDriver>()?;
    m.add_class::<PyRobotState>()?;
    m.add_class::<PyStats>()?;
    m.add("Error", m.py().get_type::<Error>())?;
    m.add("LinkLost", m.py().get_type::<LinkLost>())?;
    m.add("ProtocolError", m.py().get_type::<ProtocolError>())?;
    m.add("LimitError", m.py().get_type::<LimitError>())?;
    Ok(())
}
