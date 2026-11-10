//! Multi-track WASM replay buffer matching `ballPredictionPipeline.ts` and `RocketSimWasmStateAndReplayBase.ts`.

use crate::layout::{CarControlsPod, GameStateBufferPod, PhysicsEvent, MAX_CARS};

pub const REPLAY_MAX_FRAMES: usize = 36000;
pub const REPLAY_MAX_INCIDENTS: usize = 4096;
pub const REPLAY_MAX_SIGNALS: usize = 4096;

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct ReplayHeader {
    pub magic: u32,
    pub version: u32,
    pub frame_count: u32,
    pub state_stride: u32,
    pub incident_count: u32,
    pub signal_count: u32,
    pub recording_active: u32,
    pub reserved: [u32; 9],
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct ReplayFrameEntry {
    pub state: GameStateBufferPod,
    pub packed_controls: [u8; 64],
}

pub struct ReplayStorage {
    pub header: ReplayHeader,
    pub state_track: Vec<ReplayFrameEntry>,
    pub incident_track: Vec<PhysicsEvent>,
    pub signal_track: Vec<PhysicsEvent>,
}

impl Default for ReplayStorage {
    fn default() -> Self {
        Self::new()
    }
}

impl ReplayStorage {
    pub fn new() -> Self {
        Self {
            header: ReplayHeader {
                magic: 0x524C5243,
                version: 2,
                frame_count: 0,
                state_stride: core::mem::size_of::<ReplayFrameEntry>() as u32,
                incident_count: 0,
                signal_count: 0,
                recording_active: 1,
                reserved: [0; 9],
            },
            state_track: Vec::new(),
            incident_track: Vec::new(),
            signal_track: Vec::new(),
        }
    }

    pub fn init(&mut self) {
        if self.state_track.capacity() == 0 {
            self.state_track.reserve(1024);
        }
        if self.incident_track.capacity() == 0 {
            self.incident_track.reserve(256);
        }
        if self.signal_track.capacity() == 0 {
            self.signal_track.reserve(256);
        }
        self.clear();
    }

    pub fn clear(&mut self) {
        self.state_track.clear();
        self.incident_track.clear();
        self.signal_track.clear();
        self.header.frame_count = 0;
        self.header.incident_count = 0;
        self.header.signal_count = 0;
    }

    pub fn record_frame(
        &mut self,
        state: &GameStateBufferPod,
        controls: &[CarControlsPod; MAX_CARS],
    ) {
        if self.header.recording_active == 0 || self.state_track.len() >= REPLAY_MAX_FRAMES {
            return;
        }
        let mut packed = [0u8; 64];
        for i in 0..MAX_CARS {
            let c = &controls[i];
            let base = i * 8;
            packed[base] = ((c.throttle.clamp(-1.0, 1.0) * 127.0) as i8) as u8;
            packed[base + 1] = ((c.steer.clamp(-1.0, 1.0) * 127.0) as i8) as u8;
            packed[base + 2] = ((c.pitch.clamp(-1.0, 1.0) * 127.0) as i8) as u8;
            packed[base + 3] = ((c.yaw.clamp(-1.0, 1.0) * 127.0) as i8) as u8;
            packed[base + 4] = ((c.roll.clamp(-1.0, 1.0) * 127.0) as i8) as u8;
            let mut flags = 0u8;
            if c.jump >= 0.5 {
                flags |= 1;
            }
            if c.boost >= 0.5 {
                flags |= 2;
            }
            if c.handbrake >= 0.5 {
                flags |= 4;
            }
            packed[base + 5] = flags;
        }
        self.state_track.push(ReplayFrameEntry {
            state: *state,
            packed_controls: packed,
        });
        self.header.frame_count = self.state_track.len() as u32;
    }

    pub fn push_incident(&mut self, ev: &PhysicsEvent) {
        if self.header.recording_active == 0 || self.incident_track.len() >= REPLAY_MAX_INCIDENTS {
            return;
        }
        self.incident_track.push(*ev);
        self.header.incident_count = self.incident_track.len() as u32;
    }

    pub fn push_signal(&mut self, ev: &PhysicsEvent) {
        if self.signal_track.len() >= REPLAY_MAX_SIGNALS {
            return;
        }
        self.signal_track.push(*ev);
        self.header.signal_count = self.signal_track.len() as u32;
    }
}
