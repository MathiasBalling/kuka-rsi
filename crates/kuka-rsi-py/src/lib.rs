//! Python bindings for the kuka-rsi driver.

use std::net::SocketAddr;
use std::time::Duration;

use numpy::{PyArray1, PyReadonlyArray1};
use pyo3::create_exception;
use pyo3::exceptions::{PyException, PyOSError, PyTimeoutError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyDict;

use kuka_rsi::config::{EthernetConfig, Limits};
use kuka_rsi::sim::{ControllerSim, SimConfig, SimExit, SimHandle, SimStatus};

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
        // Task 19 may refine this mapping once the driver's fault handling grows a
        // dedicated exception; for now a faulted driver is reported as a limit error.
        E::Faulted => LimitError::new_err(e.to_string()),
        E::Config(_) | E::Closed => Error::new_err(e.to_string()),
    }
}

// Unused within Task 18; Task 19's streaming session bindings call this to convert a
// numpy target array without the intermediate `Vec<f64>` that `vec6` requires.
#[allow(dead_code)]
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

#[pymodule]
fn _core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    pyo3_log::init();
    m.add_class::<PyLimits>()?;
    m.add_class::<PyControllerSim>()?;
    m.add("Error", m.py().get_type::<Error>())?;
    m.add("LinkLost", m.py().get_type::<LinkLost>())?;
    m.add("ProtocolError", m.py().get_type::<ProtocolError>())?;
    m.add("LimitError", m.py().get_type::<LimitError>())?;
    Ok(())
}
