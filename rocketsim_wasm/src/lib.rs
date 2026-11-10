//! Zero-Emscripten WebAssembly C-ABI exports (`_physics_*`, `_malloc`, `_free`) for RocketSim v3-rust.

pub mod layout;
pub mod replay;
pub mod sim_instance;

use std::{
    alloc::{alloc, dealloc, Layout},
    cell::UnsafeCell,
    collections::HashMap,
    sync::Mutex,
};

use glam::{EulerRot, Mat3A, Vec3A};
use rocketsim::{
    consts,
    math::PhysState,
    BallState, CarCore, CarState, GameMode, MutatorConfig,
};

use crate::{
    layout::{
        BallPredSlicePod, GameStateBufferPod, MutatorConfigC, PhysicsEvent, PhysicsEventBuffer,
        MAX_CARS,
    },
    replay::{ReplayFrameEntry, ReplayHeader},
    sim_instance::SimInstance,
};

struct GlobalSim(UnsafeCell<Option<SimInstance>>);
unsafe impl Sync for GlobalSim {}

static SIM: GlobalSim = GlobalSim(UnsafeCell::new(None));
static ALLOCATIONS: Mutex<Option<HashMap<usize, Layout>>> = Mutex::new(None);

fn with_sim<R>(f: impl FnOnce(&mut SimInstance) -> R) -> R {
    unsafe {
        let slot = &mut *SIM.0.get();
        let inst = slot.get_or_insert_with(SimInstance::new);
        f(inst)
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn _malloc(size: usize) -> usize {
    if size == 0 {
        return 0;
    }
    let Ok(layout) = Layout::from_size_align(size, 8) else {
        return 0;
    };
    let ptr = unsafe { alloc(layout) };
    if ptr.is_null() {
        return 0;
    }
    let addr = ptr as usize;
    let mut map_guard = ALLOCATIONS.lock().unwrap();
    map_guard.get_or_insert_with(HashMap::new).insert(addr, layout);
    addr
}

#[unsafe(no_mangle)]
pub extern "C" fn _free(ptr: usize) {
    if ptr == 0 {
        return;
    }
    let mut map_guard = ALLOCATIONS.lock().unwrap();
    if let Some(map) = map_guard.as_mut()
        && let Some(layout) = map.remove(&ptr)
    {
        unsafe { dealloc(ptr as *mut u8, layout) };
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_init(
    mesh_data_ptrs: *const *const u8,
    mesh_sizes: *const i32,
    chunk_count: i32,
) -> i32 {
    if mesh_data_ptrs.is_null() || mesh_sizes.is_null() || chunk_count <= 0 {
        return 0;
    }
    let count = chunk_count as usize;
    let mut slices = Vec::with_capacity(count);
    unsafe {
        let ptrs = core::slice::from_raw_parts(mesh_data_ptrs, count);
        let sizes = core::slice::from_raw_parts(mesh_sizes, count);
        for i in 0..count {
            if ptrs[i].is_null() || sizes[i] <= 0 {
                return 0;
            }
            slices.push(core::slice::from_raw_parts(ptrs[i], sizes[i] as usize));
        }
    }
    match rocketsim::init_or_reload_soccar_from_slices(&slices) {
        Ok(()) => 1,
        Err(_) => 0,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_createArena() -> i32 {
    with_sim(|sim| sim.create_arena())
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_step(ticks: i32) {
    with_sim(|sim| sim.step(ticks, false));
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_stepSilent(ticks: i32) {
    with_sim(|sim| sim.step(ticks, true));
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getStateSnapshotSize() -> i32 {
    core::mem::size_of::<GameStateBufferPod>() as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_saveState(out_buffer: *mut f32) -> i32 {
    if out_buffer.is_null() {
        return 0;
    }
    with_sim(|sim| {
        sim.sync_state_to_buffer();
        unsafe {
            let src = (&sim.state_buffer as *const GameStateBufferPod) as *const f32;
            core::ptr::copy_nonoverlapping(src, out_buffer, 448);
        }
        1
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_restoreState(in_buffer: *const f32) -> i32 {
    if in_buffer.is_null() {
        return 0;
    }
    let slice = unsafe { core::slice::from_raw_parts(in_buffer, 448) };
    with_sim(|sim| sim.restore_compact_state(slice))
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_restoreCompactState(in_buffer: *const f32, float_count: i32) -> i32 {
    if in_buffer.is_null() || float_count < 410 {
        return 0;
    }
    let slice = unsafe { core::slice::from_raw_parts(in_buffer, float_count as usize) };
    with_sim(|sim| sim.restore_compact_state(slice))
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_saveStateSlot(slot: i32) -> i32 {
    if !(0..8).contains(&slot) {
        return 0;
    }
    with_sim(|sim| {
        let Some(arena) = sim.arena.as_ref() else {
            return 0;
        };
        sim.saved_slots[slot as usize] = Some(crate::sim_instance::SavedSlotSnapshot {
            arena: arena.clone(),
            car_slots: sim.car_slots,
            car_custom: sim.car_custom,
            state_buffer: sim.state_buffer,
        });
        1
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_restoreStateSlot(slot: i32) -> i32 {
    if !(0..8).contains(&slot) {
        return 0;
    }
    with_sim(|sim| {
        let Some(saved) = sim.saved_slots[slot as usize].as_ref() else {
            return 0;
        };
        sim.arena = Some(saved.arena.clone());
        sim.car_slots = saved.car_slots;
        sim.car_custom = saved.car_custom;
        sim.state_buffer = saved.state_buffer;
        sim.sync_state_to_buffer();
        1
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_resetKickoff(seed: i32) {
    with_sim(|sim| {
        if let Some(arena) = sim.arena.as_mut() {
            arena.reset_to_random_kickoff(if seed >= 0 { Some(seed as u64) } else { None });
            sim.state_buffer.header.goal_scored_flag = 0.0;
        }
        sim.sync_state_to_buffer();
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_resetKickoffAtSpawn(spawn_index: i32, settle_ticks: i32) -> i32 {
    with_sim(|sim| {
        let Some(arena) = sim.arena.as_mut() else {
            return 0;
        };
        arena.set_ball(BallState::DEFAULT);
        let spawns = consts::car::spawn::SOCCAR_SPAWN_POSITIONS;
        let idx = (spawn_index.max(0) as usize) % spawns.len();
        for slot in 0..MAX_CARS {
            let Some(car_id) = sim.car_slots[slot] else {
                continue;
            };
            let Some(car_obj) = arena.get_cars().get(car_id) else {
                continue;
            };
            let team = car_obj.info.team;
            let sp = spawns[(idx + slot) % spawns.len()];
            let phys = PhysState {
                pos: Vec3A::new(sp.x, sp.y * -team.get_y_dir(), consts::car::spawn::SPAWN_Z),
                rot_mat: Mat3A::from_euler(
                    EulerRot::ZYX,
                    sp.yaw_ang + if team == rocketsim::Team::Blue { 0.0 } else { core::f32::consts::PI },
                    0.0,
                    0.0,
                ),
                vel: Vec3A::ZERO,
                ang_vel: Vec3A::ZERO,
            };
            let st = CarState::from(CarCore::grounded(
                phys,
                arena.get_mutator_config().car_spawn_boost_amount,
            ));
            let _ = arena.set_car_state_full(car_id, &st);
        }
        if settle_ticks > 0 {
            arena.step(settle_ticks as usize);
            let _ = arena.drain_events();
        }
        sim.state_buffer.header.goal_scored_flag = 0.0;
        sim.sync_state_to_buffer();
        1
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_resetGoalLatch() {
    with_sim(|sim| {
        sim.state_buffer.header.goal_scored_flag = 0.0;
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_clearGoalFlag() {
    with_sim(|sim| {
        sim.state_buffer.header.goal_scored_flag = 0.0;
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setImpactThresholds(
    _ball_ground: f32,
    _ball_wall: f32,
    _car_ball: f32,
    _cooldown_ticks: i32,
) {
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setSurfaceTaxonomyThresholds(_floor_nz: f32, _net_nz: f32) {}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setDirectionDotThreshold(min_dir_dot: f32) {
    with_sim(|sim| sim.direction_dot_threshold = min_dir_dot);
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getDirectionDotThreshold() -> f32 {
    with_sim(|sim| sim.direction_dot_threshold)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_initBallPrediction(max_seconds: f32) {
    with_sim(|sim| sim.update_ball_prediction(max_seconds));
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_updateBallPrediction(max_seconds: f32) {
    with_sim(|sim| sim.update_ball_prediction(max_seconds));
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_updateBallPredictionIncremental(max_seconds: f32) {
    with_sim(|sim| sim.update_ball_prediction(max_seconds));
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_updateBallPredictionManual(max_seconds: f32, _ticks_since_last: i32) {
    with_sim(|sim| sim.update_ball_prediction(max_seconds));
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getBallStateForTime(pred_time: f32, out_slice9: *mut f32) -> i32 {
    if out_slice9.is_null() {
        return 0;
    }
    with_sim(|sim| {
        if sim.pred_count == 0 {
            return 0;
        }
        let idx = ((pred_time * 120.0).round() as usize).min(sim.pred_count - 1);
        let s = &sim.pred_buffer[idx];
        unsafe {
            let out = core::slice::from_raw_parts_mut(out_slice9, 9);
            out[0..3].copy_from_slice(&s.pos);
            out[3..6].copy_from_slice(&s.vel);
            out[6..9].fill(0.0);
        }
        1
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getBallPredictionStride() -> i32 {
    6
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getBallPredictionPtr() -> *const BallPredSlicePod {
    with_sim(|sim| sim.pred_buffer.as_ptr())
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getBallPredictionCount() -> i32 {
    with_sim(|sim| sim.pred_count as i32)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_isBallSleeping() -> i32 {
    with_sim(|sim| {
        sim.arena
            .as_ref()
            .map(|a| i32::from(a.is_ball_sleeping()))
            .unwrap_or(0)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setSimulationFrozen(
    frozen: i32,
    target_resume_tick: i32,
    mask_inputs: i32,
) {
    with_sim(|sim| {
        sim.sim_frozen = frozen != 0;
        sim.target_resume_tick = target_resume_tick.max(0) as u32;
        sim.mask_inputs_while_frozen = mask_inputs != 0;
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setPossessionReportingEnabled(enabled: i32) {
    with_sim(|sim| sim.possession_reporting_enabled = enabled != 0);
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getPossessionStats(
    out_team: *mut i32,
    out_blue_points: *mut f32,
    out_orange_points: *mut f32,
    out_time: *mut f32,
) {
    with_sim(|sim| unsafe {
        if !out_team.is_null() {
            *out_team = sim.state_buffer.possession_team as i32;
        }
        if !out_blue_points.is_null() {
            *out_blue_points = sim.state_buffer.blue_possession_points;
        }
        if !out_orange_points.is_null() {
            *out_orange_points = sim.state_buffer.orange_possession_points;
        }
        if !out_time.is_null() {
            *out_time = sim.state_buffer.possession_time_accumulator;
        }
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_pushSignalEvent(
    event_type: i32,
    tick: i32,
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    norm_x: f32,
    norm_y: f32,
    norm_z: f32,
    primary_id: i32,
    secondary_id: i32,
    sub_type: i32,
    flags: i32,
    custom_float: f32,
    custom_int: i32,
) {
    with_sim(|sim| {
        let ev = PhysicsEvent {
            event_type: event_type as u32,
            tick: tick as u32,
            pos_x,
            pos_y,
            pos_z,
            norm_x,
            norm_y,
            norm_z,
            vel_x: 0.0,
            vel_y: 0.0,
            vel_z: 0.0,
            primary_id,
            secondary_id,
            sub_type: sub_type as u8,
            flags: flags as u8,
            reserved: 0,
            custom_float,
            custom_int,
        };
        sim.push_event(ev);
        sim.replay.push_signal(&ev);
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_initReplayBuffer() {
    with_sim(|sim| sim.replay.init());
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_clearReplayBuffer() {
    with_sim(|sim| sim.replay.clear());
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getReplayBufferPtr() -> *const ReplayHeader {
    with_sim(|sim| &sim.replay.header as *const ReplayHeader)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getReplayBufferSize() -> i32 {
    core::mem::size_of::<ReplayHeader>() as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_recordReplayFrame() {
    with_sim(|sim| {
        let st = sim.state_buffer;
        let ctrl = sim.controls_buffer;
        sim.replay.record_frame(&st, &ctrl);
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getReplayFrameCount() -> i32 {
    with_sim(|sim| sim.replay.state_track.len() as i32)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getReplayFramePtrByOffset(ticks_ago: i32) -> *const f32 {
    with_sim(|sim| {
        let len = sim.replay.state_track.len();
        if len == 0 || ticks_ago < 0 {
            return core::ptr::null();
        }
        let idx = len.saturating_sub(1 + ticks_ago as usize);
        (&sim.replay.state_track[idx].state as *const GameStateBufferPod) as *const f32
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getReplayFramePtrByIndex(frame_index: i32) -> *const f32 {
    with_sim(|sim| {
        let idx = frame_index as usize;
        if idx >= sim.replay.state_track.len() {
            return core::ptr::null();
        }
        (&sim.replay.state_track[idx].state as *const GameStateBufferPod) as *const f32
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getReplayControlsPtrByIndex(frame_index: i32) -> *const u8 {
    with_sim(|sim| {
        let idx = frame_index as usize;
        if idx >= sim.replay.state_track.len() {
            return core::ptr::null();
        }
        sim.replay.state_track[idx].packed_controls.as_ptr()
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getReplayStateTrackPtr() -> *const ReplayFrameEntry {
    with_sim(|sim| sim.replay.state_track.as_ptr())
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getReplayStateStride() -> i32 {
    core::mem::size_of::<ReplayFrameEntry>() as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getReplayIncidentTrackPtr() -> *const PhysicsEvent {
    with_sim(|sim| sim.replay.incident_track.as_ptr())
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getReplayIncidentCount() -> i32 {
    with_sim(|sim| sim.replay.incident_track.len() as i32)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getReplaySignalTrackPtr() -> *const PhysicsEvent {
    with_sim(|sim| sim.replay.signal_track.as_ptr())
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getReplaySignalCount() -> i32 {
    with_sim(|sim| sim.replay.signal_track.len() as i32)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setReplayRecordingActive(active: i32) {
    with_sim(|sim| sim.replay.header.recording_active = u32::from(active != 0));
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getReplayRecordingActive() -> i32 {
    with_sim(|sim| i32::from(sim.replay.header.recording_active != 0))
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getEventBufferPtr() -> *const PhysicsEventBuffer {
    with_sim(|sim| &sim.event_buffer as *const PhysicsEventBuffer)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getEventBufferSize() -> i32 {
    core::mem::size_of::<PhysicsEventBuffer>() as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_clearEvents() {
    with_sim(|sim| {
        sim.event_buffer.count = 0;
        sim.event_buffer.dropped_count = 0;
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getStatePtr() -> *const f32 {
    with_sim(|sim| (&sim.state_buffer as *const GameStateBufferPod) as *const f32)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getStateSize() -> i32 {
    448
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getControlsPtr() -> *mut f32 {
    with_sim(|sim| sim.controls_buffer.as_mut_ptr() as *mut f32)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getPadInfoPtr() -> *const f32 {
    with_sim(|sim| sim.pad_info_buffer.as_ptr() as *const f32)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_addCarAtSlot(car_index: i32, team: i32, hitbox_index: i32) -> i32 {
    if car_index < 0 {
        return -1;
    }
    with_sim(|sim| sim.add_car_at_slot(car_index as usize, team, hitbox_index))
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_addCar(team: i32, hitbox_index: i32) -> i32 {
    with_sim(|sim| sim.add_car(team, hitbox_index))
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_removeCar(car_index: i32) -> i32 {
    if car_index < 0 {
        return 0;
    }
    with_sim(|sim| sim.remove_car(car_index as usize))
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getActiveCarMask() -> i32 {
    with_sim(|sim| sim.active_car_mask())
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_isCarSlotActive(car_index: i32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| i32::from(sim.car_slots[car_index as usize].is_some()))
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getActiveCarCount() -> i32 {
    with_sim(|sim| sim.active_car_count())
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setCarHitbox(car_index: i32, hitbox_index: i32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| {
        let slot = car_index as usize;
        let Some(car_id) = sim.car_slots[slot] else {
            return 0;
        };
        let Some(arena) = sim.arena.as_mut() else {
            return 0;
        };
        let mut cfg = *SimInstance::hitbox_config_from_index(hitbox_index);
        if hitbox_index == 6 {
            cfg.three_wheels = true;
        }
        sim.car_custom[slot].hitbox_index = hitbox_index;
        sim.car_custom[slot].three_wheels = cfg.three_wheels;
        i32::from(arena.set_car_config(car_id, cfg))
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setCarTeam(car_index: i32, team: i32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| {
        let Some(car_id) = sim.car_slots[car_index as usize] else {
            return 0;
        };
        let Some(arena) = sim.arena.as_mut() else {
            return 0;
        };
        let rs_team = if team == 0 {
            rocketsim::Team::Blue
        } else {
            rocketsim::Team::Orange
        };
        i32::from(arena.set_car_team(car_id, rs_team))
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_demolishCar(car_index: i32, respawn_delay: f32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| {
        let Some(car_id) = sim.car_slots[car_index as usize] else {
            return 0;
        };
        let Some(arena) = sim.arena.as_mut() else {
            return 0;
        };
        arena.demolish_car_custom(car_id, respawn_delay);
        sim.sync_state_to_buffer();
        1
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_respawnCar(car_index: i32, seed: i32, boost_amount: f32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| {
        let Some(car_id) = sim.car_slots[car_index as usize] else {
            return 0;
        };
        let Some(arena) = sim.arena.as_mut() else {
            return 0;
        };
        let ok = arena
            .respawn_car(
                car_id,
                if seed >= 0 { Some(seed as u64) } else { None },
                Some(boost_amount),
            )
            .is_ok();
        sim.sync_state_to_buffer();
        i32::from(ok)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCarCount() -> i32 {
    with_sim(|sim| sim.highest_car_slot_len() as i32)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCarConfig(hitbox_index: i32) -> *const crate::layout::CarConfigC {
    with_sim(|sim| {
        let idx = hitbox_index.clamp(0, 6) as usize;
        &sim.car_config_cache[idx] as *const crate::layout::CarConfigC
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setCarState(car_index: i32, state_ptr: *const f32) {
    if !(0..8).contains(&car_index) || state_ptr.is_null() {
        return;
    }
    let slice = unsafe { &*(state_ptr as *const [f32; 40]) };
    with_sim(|sim| sim.set_car_state_from_slice(car_index as usize, slice));
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setBallState(state_ptr: *const f32) {
    if state_ptr.is_null() {
        return;
    }
    let slice = unsafe { &*(state_ptr as *const [f32; 18]) };
    with_sim(|sim| sim.set_ball_state_from_slice(slice));
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setUnlimitedBoost(unlimited: i32) {
    with_sim(|sim| sim.unlimited_boost_global = unlimited != 0);
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getUnlimitedBoost() -> i32 {
    with_sim(|sim| i32::from(sim.unlimited_boost_global))
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setCarUnlimitedBoost(car_index: i32, enabled: i32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| {
        sim.car_custom[car_index as usize].unlimited_boost = enabled != 0;
        1
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCarUnlimitedBoost(car_index: i32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| i32::from(sim.car_custom[car_index as usize].unlimited_boost))
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setCarBoostCollectable(car_index: i32, collectable: i32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| {
        sim.car_custom[car_index as usize].boost_collectable = collectable != 0;
        1
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCarBoostCollectable(car_index: i32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| i32::from(sim.car_custom[car_index as usize].boost_collectable))
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setCarBoostRecharge(
    car_index: i32,
    enabled: i32,
    recharge_per_second: f32,
    recharge_delay: f32,
) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| {
        let c = &mut sim.car_custom[car_index as usize];
        c.recharge_enabled = enabled != 0;
        c.recharge_per_second = recharge_per_second;
        c.recharge_delay = recharge_delay;
        1
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCarBoostRechargeEnabled(car_index: i32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| i32::from(sim.car_custom[car_index as usize].recharge_enabled))
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCarBoostRechargePerSecond(car_index: i32) -> f32 {
    if !(0..8).contains(&car_index) {
        return 0.0;
    }
    with_sim(|sim| sim.car_custom[car_index as usize].recharge_per_second)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCarBoostRechargeDelay(car_index: i32) -> f32 {
    if !(0..8).contains(&car_index) {
        return 0.0;
    }
    with_sim(|sim| sim.car_custom[car_index as usize].recharge_delay)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getBallOnGround() -> i32 {
    with_sim(|sim| {
        let Some(arena) = sim.arena.as_ref() else {
            return 0;
        };
        let b = arena.get_ball();
        let r = arena.get_mutator_config().ball_radius;
        i32::from((b.pos.z - r) <= 8.0 && b.vel.z.abs() <= 50.0)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getBallRadius() -> f32 {
    with_sim(|sim| {
        sim.arena
            .as_ref()
            .map(|a| a.get_mutator_config().ball_radius)
            .unwrap_or(91.25)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setSimControlFlags(flags: i32) {
    with_sim(|sim| sim.sim_control_flags = flags as u32);
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getSimControlFlags() -> i32 {
    with_sim(|sim| sim.sim_control_flags as i32)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getBallMotionState() -> i32 {
    with_sim(|sim| sim.state_buffer.ball_motion_state as i32)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setPossessionEventEnabled(enabled: i32) {
    with_sim(|sim| sim.possession_event_enabled = enabled != 0);
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getPossessionEventEnabled() -> i32 {
    with_sim(|sim| i32::from(sim.possession_event_enabled))
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCurrentPossessionCar() -> i32 {
    with_sim(|sim| sim.state_buffer.possession_car_index as i32)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCurrentPossessionTeam() -> i32 {
    with_sim(|sim| sim.state_buffer.possession_team as i32)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCarIsJumping(car_index: i32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| i32::from(sim.state_buffer.car_jump_air[car_index as usize].is_jumping > 0.5))
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCarAirTime(car_index: i32) -> f32 {
    if !(0..8).contains(&car_index) {
        return 0.0;
    }
    with_sim(|sim| sim.state_buffer.car_jump_air[car_index as usize].air_time)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCarJumpTime(car_index: i32) -> f32 {
    if !(0..8).contains(&car_index) {
        return 0.0;
    }
    with_sim(|sim| sim.state_buffer.car_jump_air[car_index as usize].jump_time)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCarAirTimeSinceJump(car_index: i32) -> f32 {
    if !(0..8).contains(&car_index) {
        return 0.0;
    }
    with_sim(|sim| sim.state_buffer.car_jump_air[car_index as usize].air_time_since_jump)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setCarLinearFactorAndGravity(
    car_index: i32,
    fx: f32,
    fy: f32,
    fz: f32,
    gravity_scale: f32,
) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| {
        let Some(car_id) = sim.car_slots[car_index as usize] else {
            return 0;
        };
        let Some(arena) = sim.arena.as_mut() else {
            return 0;
        };
        i32::from(arena.set_car_linear_factor_and_gravity(car_id, fx, fy, fz, gravity_scale))
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setCarCollisionMask(
    _car_index: i32,
    _interact_with_ball: i32,
    _interact_with_cars: i32,
) -> i32 {
    1
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setArenaMutators(
    gravity_z: f32,
    boost_accel_ground: f32,
    boost_accel_air: f32,
    ball_max_speed: f32,
    ball_bounciness: f32,
) -> i32 {
    with_sim(|sim| {
        let Some(arena) = sim.arena.as_mut() else {
            return 0;
        };
        let mut m = *arena.get_mutator_config();
        m.gravity.z = gravity_z;
        m.boost_accel_ground = boost_accel_ground;
        m.boost_accel_air = boost_accel_air;
        m.ball_max_speed = ball_max_speed;
        m.ball_world_restitution = ball_bounciness;
        arena.set_mutator_config(m);
        1
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getMutatorConfigSize() -> i32 {
    core::mem::size_of::<MutatorConfigC>() as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getMutatorConfig(out_buffer: *mut MutatorConfigC) -> i32 {
    if out_buffer.is_null() {
        return 0;
    }
    with_sim(|sim| {
        unsafe { *out_buffer = sim.get_mutator_config_c() };
        1
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setMutatorConfig(in_buffer: *const MutatorConfigC) -> i32 {
    if in_buffer.is_null() {
        return 0;
    }
    let cfg = unsafe { &*in_buffer };
    with_sim(|sim| sim.set_mutator_config_c(cfg))
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_resetMutatorConfig() -> i32 {
    with_sim(|sim| {
        let Some(arena) = sim.arena.as_mut() else {
            return 0;
        };
        arena.set_mutator_config(MutatorConfig::default(GameMode::Soccar));
        1
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setCarThreeWheels(car_index: i32, enabled: i32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| {
        let slot = car_index as usize;
        sim.car_custom[slot].three_wheels = enabled != 0;
        let Some(car_id) = sim.car_slots[slot] else {
            return 0;
        };
        let Some(arena) = sim.arena.as_mut() else {
            return 0;
        };
        i32::from(arena.set_car_three_wheels(car_id, enabled != 0))
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCarThreeWheels(car_index: i32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| i32::from(sim.car_custom[car_index as usize].three_wheels))
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setArenaPreset(preset_index: i32) {
    with_sim(|sim| sim.arena_preset = preset_index);
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getArenaPreset() -> i32 {
    with_sim(|sim| sim.arena_preset)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setGoalVolumeBox(
    enabled: i32,
    half_width_x: f32,
    min_y: f32,
    max_y: f32,
    max_z: f32,
) {
    with_sim(|sim| {
        sim.goal_box_enabled = enabled != 0;
        sim.goal_box_half_x = half_width_x;
        sim.goal_box_min_y = min_y;
        sim.goal_box_max_y = max_y;
        sim.goal_box_max_z = max_z;
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setCarMaxFlipCount(car_index: i32, max_flip_count: i32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| {
        sim.car_custom[car_index as usize].max_flip_count = max_flip_count;
        1
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCarMaxFlipCount(car_index: i32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 1;
    }
    with_sim(|sim| sim.car_custom[car_index as usize].max_flip_count)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setCarFlipTimerMode(car_index: i32, mode: i32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| {
        sim.car_custom[car_index as usize].flip_timer_mode = mode;
        1
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCarFlipTimerMode(car_index: i32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| sim.car_custom[car_index as usize].flip_timer_mode)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_setCarDemoSpawnSelection(car_index: i32, slot_index: i32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| {
        sim.car_custom[car_index as usize].demo_spawn_selection = slot_index;
        1
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCarDemoSpawnSelection(car_index: i32) -> i32 {
    if !(0..8).contains(&car_index) {
        return -1;
    }
    with_sim(|sim| sim.car_custom[car_index as usize].demo_spawn_selection)
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCarDemoRespawnTimer(car_index: i32) -> f32 {
    if !(0..8).contains(&car_index) {
        return 0.0;
    }
    with_sim(|sim| {
        let Some(car_id) = sim.car_slots[car_index as usize] else {
            return 0.0;
        };
        let Some(arena) = sim.arena.as_ref() else {
            return 0.0;
        };
        let st = arena.get_car(car_id);
        if st.is_demoed {
            st.demo_respawn_timer
        } else {
            0.0
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCarFlipsRemaining(car_index: i32) -> i32 {
    if !(0..8).contains(&car_index) {
        return 0;
    }
    with_sim(|sim| {
        let Some(car_id) = sim.car_slots[car_index as usize] else {
            return 0;
        };
        let Some(arena) = sim.arena.as_ref() else {
            return 0;
        };
        i32::from(arena.get_car(car_id).has_flip_or_jump())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn _physics_getCarFlipWindowTimer(car_index: i32) -> f32 {
    if !(0..8).contains(&car_index) {
        return 0.0;
    }
    with_sim(|sim| {
        let Some(car_id) = sim.car_slots[car_index as usize] else {
            return 0.0;
        };
        let Some(arena) = sim.arena.as_ref() else {
            return 0.0;
        };
        let st = arena.get_car(car_id);
        if st.has_flip_or_jump() && !st.is_on_ground() {
            (consts::car::JUMP_MAX_TIME + consts::car::DOUBLEJUMP_MAX_DELAY
                - st.air_time_since_jump)
                .max(0.0)
        } else {
            0.0
        }
    })
}
