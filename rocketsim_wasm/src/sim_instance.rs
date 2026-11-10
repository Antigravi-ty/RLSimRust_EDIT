//! `SimInstance` managing `Arena`, 8 car slots, state buffer sync, ball prediction, snapshots, and events.

use glam::{Mat3A, Vec3A};
use rocketsim::{
    consts,
    math::PhysState,
    Arena, ArenaConfig, ArenaEvent,ArenaMemWeightMode, BallState, CarConfig, CarControls,
    CarCore, CarState, DemoMode, GameMode, MutatorConfig, Team,
};

use crate::{
    layout::{
        BallPredSlicePod, BoostPadInfoPod, CarConfigC, CarControlsPod, GameStateBufferPod,
        MutatorConfigC, PhysicsEvent, PhysicsEventBuffer, WheelPairConfigC, MAX_CARS,
        MAX_EVENTS_PER_STEP, MAX_PADS, MAX_PREDICTION_STEPS,
    },
    replay::ReplayStorage,
};

#[derive(Clone, Copy)]
pub struct CarCustomSettings {
    pub unlimited_boost: bool,
    pub boost_collectable: bool,
    pub recharge_enabled: bool,
    pub recharge_per_second: f32,
    pub recharge_delay: f32,
    pub max_flip_count: i32,
    pub flip_timer_mode: i32,
    pub demo_spawn_selection: i32,
    pub three_wheels: bool,
    pub hitbox_index: i32,
}

impl Default for CarCustomSettings {
    fn default() -> Self {
        Self {
            unlimited_boost: false,
            boost_collectable: true,
            recharge_enabled: false,
            recharge_per_second: 10.0,
            recharge_delay: 0.5,
            max_flip_count: 1,
            flip_timer_mode: 0,
            demo_spawn_selection: -1,
            three_wheels: false,
            hitbox_index: 0,
        }
    }
}

#[derive(Clone)]
pub struct SavedSlotSnapshot {
    pub arena: Arena,
    pub car_slots: [Option<usize>; MAX_CARS],
    pub car_custom: [CarCustomSettings; MAX_CARS],
    pub state_buffer: GameStateBufferPod,
}

pub struct SimInstance {
    pub arena: Option<Arena>,
    pub car_slots: [Option<usize>; MAX_CARS],
    pub car_custom: [CarCustomSettings; MAX_CARS],
    pub state_buffer: GameStateBufferPod,
    pub controls_buffer: [CarControlsPod; MAX_CARS],
    pub pad_info_buffer: [BoostPadInfoPod; MAX_PADS],
    pub pred_buffer: [BallPredSlicePod; MAX_PREDICTION_STEPS],
    pub pred_count: usize,
    pub event_buffer: PhysicsEventBuffer,
    pub replay: ReplayStorage,
    pub saved_slots: [Option<SavedSlotSnapshot>; 8],
    pub car_config_cache: [CarConfigC; 7],
    pub unlimited_boost_global: bool,
    pub arena_preset: i32,
    pub goal_box_enabled: bool,
    pub goal_box_half_x: f32,
    pub goal_box_min_y: f32,
    pub goal_box_max_y: f32,
    pub goal_box_max_z: f32,
    pub sim_control_flags: u32,
    pub possession_event_enabled: bool,
    pub possession_reporting_enabled: bool,
    pub sim_frozen: bool,
    pub target_resume_tick: u32,
    pub mask_inputs_while_frozen: bool,
    pub direction_dot_threshold: f32,
}

impl Default for SimInstance {
    fn default() -> Self {
        Self::new()
    }
}

impl SimInstance {
    pub fn new() -> Self {
        let mut inst = Self {
            arena: None,
            car_slots: [None; MAX_CARS],
            car_custom: [CarCustomSettings::default(); MAX_CARS],
            state_buffer: GameStateBufferPod::default(),
            controls_buffer: [CarControlsPod::default(); MAX_CARS],
            pad_info_buffer: [BoostPadInfoPod::default(); MAX_PADS],
            pred_buffer: [BallPredSlicePod::default(); MAX_PREDICTION_STEPS],
            pred_count: 0,
            event_buffer: PhysicsEventBuffer::default(),
            replay: ReplayStorage::new(),
            saved_slots: [const { None }; 8],
            car_config_cache: [CarConfigC::default(); 7],
            unlimited_boost_global: false,
            arena_preset: 0,
            goal_box_enabled: false,
            goal_box_half_x: 892.755,
            goal_box_min_y: 5120.0,
            goal_box_max_y: 6000.0,
            goal_box_max_z: 642.775,
            sim_control_flags: 0x07,
            possession_event_enabled: true,
            possession_reporting_enabled: true,
            sim_frozen: false,
            target_resume_tick: 0,
            mask_inputs_while_frozen: false,
            direction_dot_threshold: 0.25,
        };
        for i in 0..7 {
            let cfg = Self::hitbox_config_from_index(i as i32);
            inst.car_config_cache[i] = Self::car_config_to_c(cfg);
        }
        inst
    }

    pub fn hitbox_config_from_index(idx: i32) -> &'static CarConfig {
        match idx {
            1 => CarConfig::dominus(),
            2 => CarConfig::plank(),
            3 => CarConfig::breakout(),
            4 => CarConfig::hybrid(),
            5 => CarConfig::merc(),
            6 => CarConfig::psyclops(),
            _ => CarConfig::octane(),
        }
    }

    pub fn car_config_to_c(cfg: &CarConfig) -> CarConfigC {
        CarConfigC {
            hitbox_size: cfg.hitbox_size.to_array(),
            hitbox_pos_offset: cfg.hitbox_pos_offset.to_array(),
            front_wheels: WheelPairConfigC {
                wheel_radius: cfg.front_wheels.wheel_radius,
                suspension_rest_length: cfg.front_wheels.suspension_rest_length,
                connection_point_offset: cfg.front_wheels.connection_point_offset.to_array(),
            },
            back_wheels: WheelPairConfigC {
                wheel_radius: cfg.back_wheels.wheel_radius,
                suspension_rest_length: cfg.back_wheels.suspension_rest_length,
                connection_point_offset: cfg.back_wheels.connection_point_offset.to_array(),
            },
            three_wheels: u8::from(cfg.three_wheels),
            _pad: [0; 3],
            dodge_deadzone: cfg.dodge_deadzone,
        }
    }

    pub fn create_arena(&mut self) -> i32 {
        let mut config = ArenaConfig {
            mem_weight_mode: ArenaMemWeightMode::Light,
            ..ArenaConfig::default()
        };
        if self.arena_preset == 1 {
            config.min_pos = Vec3A::new(-4500.0, -7500.0, -100.0);
            config.max_pos = Vec3A::new(4500.0, 7500.0, 2200.0);
        }
        let arena = Arena::new(GameMode::Soccar, config, 120);
        self.car_slots = [None; MAX_CARS];
        self.car_custom = [CarCustomSettings::default(); MAX_CARS];

        let pads = arena.get_boost_pads();
        let pad_count = pads.len().min(MAX_PADS);
        for (i, pad) in pads.iter().take(pad_count).enumerate() {
            self.pad_info_buffer[i] = BoostPadInfoPod {
                pos: pad.config.pos.to_array(),
                is_big: if pad.config.is_big { 1.0 } else { 0.0 },
            };
        }

        self.arena = Some(arena);
        self.sync_state_to_buffer();
        1
    }

    pub fn active_car_mask(&self) -> i32 {
        let mut mask = 0i32;
        for (slot, id) in self.car_slots.iter().enumerate() {
            if id.is_some() {
                mask |= 1 << slot;
            }
        }
        mask
    }

    pub fn active_car_count(&self) -> i32 {
        self.car_slots.iter().filter(|s| s.is_some()).count() as i32
    }

    pub fn highest_car_slot_len(&self) -> usize {
        for i in (0..MAX_CARS).rev() {
            if self.car_slots[i].is_some() {
                return i + 1;
            }
        }
        0
    }

    pub fn add_car_at_slot(&mut self, slot: usize, team: i32, hitbox_index: i32) -> i32 {
        if slot >= MAX_CARS {
            return -1;
        }
        let Some(arena) = self.arena.as_mut() else {
            return -1;
        };
        let rs_team = if team == 0 { Team::Blue } else { Team::Orange };
        let mut cfg = *Self::hitbox_config_from_index(hitbox_index);
        if hitbox_index == 6 {
            cfg.three_wheels = true;
        }

        if let Some(car_id) = self.car_slots[slot] {
            arena.set_car_team(car_id, rs_team);
            arena.set_car_config(car_id, cfg);
        } else {
            let car_id = arena.add_car(rs_team, cfg);
            self.car_slots[slot] = Some(car_id);
        }

        self.car_custom[slot] = CarCustomSettings {
            unlimited_boost: self.unlimited_boost_global,
            three_wheels: cfg.three_wheels,
            hitbox_index,
            ..CarCustomSettings::default()
        };
        self.sync_state_to_buffer();
        slot as i32
    }

    pub fn add_car(&mut self, team: i32, hitbox_index: i32) -> i32 {
        for slot in 0..MAX_CARS {
            if self.car_slots[slot].is_none() {
                return self.add_car_at_slot(slot, team, hitbox_index);
            }
        }
        -1
    }

    pub fn remove_car(&mut self, slot: usize) -> i32 {
        if slot >= MAX_CARS {
            return 0;
        }
        let Some(car_id) = self.car_slots[slot].take() else {
            return 0;
        };
        if let Some(arena) = self.arena.as_mut() {
            let _ = arena.remove_car(car_id);
        }
        self.state_buffer.cars[slot] = Default::default();
        self.state_buffer.car_jump_air[slot] = Default::default();
        self.sync_state_to_buffer();
        1
    }

    pub fn push_event(&mut self, ev: PhysicsEvent) {
        let idx = self.event_buffer.count as usize;
        if idx < MAX_EVENTS_PER_STEP {
            self.event_buffer.events[idx] = ev;
            self.event_buffer.count += 1;
        } else {
            self.event_buffer.dropped_count += 1;
        }
    }

    pub fn step(&mut self, ticks_to_simulate: i32, silent: bool) {
        if self.arena.is_none() || ticks_to_simulate <= 0 {
            return;
        }

        for _ in 0..ticks_to_simulate {
            if self.sim_frozen {
                let current_tick = self
                    .arena
                    .as_ref()
                    .map(|a| a.get_tick_count() as u32)
                    .unwrap_or(0);
                if self.target_resume_tick > 0 && current_tick >= self.target_resume_tick {
                    self.sim_frozen = false;
                } else {
                    self.sync_state_to_buffer();
                    return;
                }
            }

            if let Some(arena) = self.arena.as_mut() {
                for slot in 0..MAX_CARS {
                    let Some(car_id) = self.car_slots[slot] else {
                        continue;
                    };
                    let c = &self.controls_buffer[slot];
                    let controls = CarControls {
                        throttle: c.throttle,
                        steer: c.steer,
                        pitch: c.pitch,
                        yaw: c.yaw,
                        roll: c.roll,
                        jump: c.jump >= 0.5,
                        boost: c.boost >= 0.5,
                        handbrake: c.handbrake >= 0.5,
                    };
                    let _ = arena.set_car_controls(car_id, controls);
                }

                arena.step(1);

                for slot in 0..MAX_CARS {
                    let Some(car_id) = self.car_slots[slot] else {
                        continue;
                    };
                    let custom = &self.car_custom[slot];
                    if self.unlimited_boost_global || custom.unlimited_boost {
                        let mut st = *arena.get_car(car_id);
                        if st.boost < 100.0 {
                            st.boost = 100.0;
                            let _ = arena.set_car_state_full(car_id, &st);
                        }
                    } else if custom.recharge_enabled {
                        let mut st = *arena.get_car(car_id);
                        if !st.is_boosting && st.boost < 100.0 {
                            st.boost = (st.boost + custom.recharge_per_second / 120.0).min(100.0);
                            let _ = arena.set_car_state_full(car_id, &st);
                        }
                    }
                }
            }

            if !silent {
                self.drain_arena_events();
            } else if let Some(arena) = self.arena.as_mut() {
                let _ = arena.drain_events();
            }

            self.sync_state_to_buffer();

            if !silent && self.replay.header.recording_active != 0 {
                let st_copy = self.state_buffer;
                let ctrl_copy = self.controls_buffer;
                self.replay.record_frame(&st_copy, &ctrl_copy);
            }
        }
    }

    fn drain_arena_events(&mut self) {
        let Some(arena) = self.arena.as_mut() else {
            return;
        };
        let tick = arena.get_tick_count() as u32;
        let raw_events = arena.drain_events();
        for ev in raw_events {
            match ev {
                ArenaEvent::BallHitWorld(e) => {
                    let nz = e.contact_normal.z;
                    let sub_type = if nz > 0.7 {
                        1
                    } else if nz < -0.7 {
                        2
                    } else {
                        3
                    };
                    let pev = PhysicsEvent {
                        event_type: 1,
                        tick,
                        pos_x: e.contact_point.x,
                        pos_y: e.contact_point.y,
                        pos_z: e.contact_point.z,
                        norm_x: e.contact_normal.x,
                        norm_y: e.contact_normal.y,
                        norm_z: e.contact_normal.z,
                        primary_id: -1,
                        secondary_id: -1,
                        sub_type,
                        ..Default::default()
                    };
                    self.push_event(pev);
                }
                ArenaEvent::CarHitBall(e) => {
                    let slot = self
                        .car_slots
                        .iter()
                        .position(|&id| id == Some(e.car_idx))
                        .map(|s| s as i32)
                        .unwrap_or(e.car_idx as i32);
                    let pev = PhysicsEvent {
                        event_type: 2,
                        tick,
                        pos_x: e.contact_point.x,
                        pos_y: e.contact_point.y,
                        pos_z: e.contact_point.z,
                        vel_x: e.extra_hit_vel.x,
                        vel_y: e.extra_hit_vel.y,
                        vel_z: e.extra_hit_vel.z,
                        primary_id: slot,
                        secondary_id: -1,
                        ..Default::default()
                    };
                    if slot >= 0 && (slot as usize) < MAX_CARS {
                        self.state_buffer.possession_car_index = slot as f32;
                    }
                    self.push_event(pev);
                    self.replay.push_incident(&pev);
                }
                ArenaEvent::CarHitWorld(e) => {
                    let slot = self
                        .car_slots
                        .iter()
                        .position(|&id| id == Some(e.car_idx))
                        .map(|s| s as i32)
                        .unwrap_or(e.car_idx as i32);
                    let pev = PhysicsEvent {
                        event_type: 3,
                        tick,
                        pos_x: e.contact_point.x,
                        pos_y: e.contact_point.y,
                        pos_z: e.contact_point.z,
                        norm_x: e.contact_normal.x,
                        norm_y: e.contact_normal.y,
                        norm_z: e.contact_normal.z,
                        primary_id: slot,
                        secondary_id: -1,
                        ..Default::default()
                    };
                    self.push_event(pev);
                }
                ArenaEvent::CarBumpCar(e) => {
                    let s1 = self
                        .car_slots
                        .iter()
                        .position(|&id| id == Some(e.bumper_idx))
                        .map(|s| s as i32)
                        .unwrap_or(e.bumper_idx as i32);
                    let s2 = self
                        .car_slots
                        .iter()
                        .position(|&id| id == Some(e.victim_idx))
                        .map(|s| s as i32)
                        .unwrap_or(e.victim_idx as i32);
                    let pev = PhysicsEvent {
                        event_type: if e.is_demo { 5 } else { 4 },
                        tick,
                        pos_x: e.contact_point.x,
                        pos_y: e.contact_point.y,
                        pos_z: e.contact_point.z,
                        norm_x: e.contact_normal.x,
                        norm_y: e.contact_normal.y,
                        norm_z: e.contact_normal.z,
                        primary_id: s1,
                        secondary_id: s2,
                        flags: u8::from(e.is_demo),
                        ..Default::default()
                    };
                    self.push_event(pev);
                    self.replay.push_incident(&pev);
                }
                ArenaEvent::CarCollectedBoost(e) => {
                    let slot = self
                        .car_slots
                        .iter()
                        .position(|&id| id == Some(e.car_idx))
                        .map(|s| s as i32)
                        .unwrap_or(e.car_idx as i32);
                    let pev = PhysicsEvent {
                        event_type: 6,
                        tick,
                        primary_id: slot,
                        secondary_id: e.pad_idx as i32,
                        custom_float: e.boost_gained,
                        ..Default::default()
                    };
                    self.push_event(pev);
                }
            }
        }
    }

    pub fn sync_state_to_buffer(&mut self) {
        let Some(arena) = self.arena.as_ref() else {
            return;
        };

        let ball = arena.get_ball();
        let pads = arena.get_boost_pads();
        let pad_count = pads.len().min(MAX_PADS);

        self.state_buffer.header.tick_count = arena.get_tick_count() as f32;
        if arena.is_ball_scored() {
            self.state_buffer.header.goal_scored_flag = if ball.pos.y > 0.0 { 1.0 } else { 2.0 };
        }
        self.state_buffer.header.num_cars = self.highest_car_slot_len() as f32;
        self.state_buffer.header.num_pads = pad_count as f32;

        self.state_buffer.ball.pos = ball.pos.to_array();
        self.state_buffer.ball.forward = ball.rot_mat.x_axis.to_array();
        self.state_buffer.ball.right = ball.rot_mat.y_axis.to_array();
        self.state_buffer.ball.up = ball.rot_mat.z_axis.to_array();
        self.state_buffer.ball.vel = ball.vel.to_array();
        self.state_buffer.ball.ang_vel = ball.ang_vel.to_array();

        let speed_sq = ball.vel.length_squared();
        self.state_buffer.ball_motion_state = if speed_sq < 1.0 {
            0.0
        } else if (ball.pos.z - arena.get_mutator_config().ball_radius) <= 8.0 {
            1.0
        } else {
            2.0
        };

        for slot in 0..MAX_CARS {
            let Some(car_id) = self.car_slots[slot] else {
                continue;
            };
            let Some(car_obj) = arena.get_cars().get(car_id) else {
                continue;
            };
            let st = car_obj.get_state();
            let pod = &mut self.state_buffer.cars[slot];

            pod.pos = st.pos.to_array();
            pod.forward = st.rot_mat.x_axis.to_array();
            pod.right = st.rot_mat.y_axis.to_array();
            pod.up = st.rot_mat.z_axis.to_array();
            pod.vel = st.vel.to_array();
            pod.ang_vel = st.ang_vel.to_array();
            pod.boost = st.boost;
            pod.is_on_ground = if st.is_on_ground() { 1.0 } else { 0.0 };
            pod.is_supersonic = if st.is_supersonic { 1.0 } else { 0.0 };
            pod.is_demoed = if st.is_demoed {
                st.demo_respawn_timer.max(0.001)
            } else {
                0.0
            };
            pod.has_flip_or_jump = if st.has_flip_or_jump() { 1.0 } else { 0.0 };
            pod.is_boosting = if st.is_boosting { 1.0 } else { 0.0 };
            pod.is_flipping = if st.is_flipping() { 1.0 } else { 0.0 };

            for w in 0..4 {
                let (sus_len, steer, contact) = car_obj.wheel_telemetry(w);
                pod.wheels[w].suspension_length = sus_len;
                pod.wheels[w].steer_angle = steer;
                pod.wheels[w].has_contact = if contact { 1.0 } else { 0.0 };
            }

            let gn = st
                .world_contact_normal()
                .unwrap_or(if st.is_on_ground() {
                    st.rot_mat.z_axis
                } else {
                    Vec3A::ZERO
                });
            pod.ground_normal = gn.to_array();

            let ja = &mut self.state_buffer.car_jump_air[slot];
            ja.is_jumping = if st.is_jumping { 1.0 } else { 0.0 };
            ja.air_time = st.air_time;
            ja.jump_time = if st.is_demoed {
                self.car_custom[slot].demo_spawn_selection as f32
            } else {
                st.jump_time
            };
            ja.air_time_since_jump = st.air_time_since_jump;
        }

        for (i, pad) in pads.iter().take(pad_count).enumerate() {
            self.state_buffer.pads[i].is_active = if pad.state.is_active { 1.0 } else { 0.0 };
            self.state_buffer.pads[i].cooldown = pad.state.cooldown;
        }
    }

    pub fn set_ball_state_from_slice(&mut self, f: &[f32; 18]) {
        let Some(arena) = self.arena.as_mut() else {
            return;
        };
        let mut bs = *arena.get_ball();
        bs.pos = Vec3A::from_slice(&f[0..3]);
        let fwd = Vec3A::from_slice(&f[3..6]);
        let right = Vec3A::from_slice(&f[6..9]);
        let up = Vec3A::from_slice(&f[9..12]);
        if fwd.length_squared() > 0.2 && right.length_squared() > 0.2 && up.length_squared() > 0.2 {
            bs.rot_mat = Mat3A::from_cols(fwd, right, up);
        }
        bs.vel = Vec3A::from_slice(&f[12..15]);
        bs.ang_vel = Vec3A::from_slice(&f[15..18]);
        arena.set_ball(bs);
        self.sync_state_to_buffer();
    }

    pub fn set_car_state_from_slice(&mut self, slot: usize, f: &[f32; 40]) {
        if slot >= MAX_CARS {
            return;
        }
        let Some(car_id) = self.car_slots[slot] else {
            return;
        };
        let Some(arena) = self.arena.as_mut() else {
            return;
        };
        let mut cs = *arena.get_car(car_id);
        cs.pos = Vec3A::from_slice(&f[0..3]);
        let fwd = Vec3A::from_slice(&f[3..6]);
        let right = Vec3A::from_slice(&f[6..9]);
        let up = Vec3A::from_slice(&f[9..12]);
        if fwd.length_squared() > 0.2 && right.length_squared() > 0.2 && up.length_squared() > 0.2 {
            cs.rot_mat = Mat3A::from_cols(fwd, right, up);
        }
        cs.vel = Vec3A::from_slice(&f[12..15]);
        cs.ang_vel = Vec3A::from_slice(&f[15..18]);
        cs.boost = f[18].clamp(0.0, 100.0);
        cs.is_supersonic = f[20] >= 0.5;
        if f[21] > 0.0 {
            cs.is_demoed = true;
            cs.demo_respawn_timer = f[21];
        } else {
            cs.is_demoed = false;
            cs.demo_respawn_timer = 0.0;
        }
        cs.is_boosting = f[23] >= 0.5;
        let _ = arena.set_car_state_full(car_id, &cs);
        self.sync_state_to_buffer();
    }

    pub fn update_ball_prediction(&mut self, max_seconds: f32) {
        let Some(arena) = self.arena.as_ref() else {
            self.pred_count = 0;
            return;
        };
        let steps = ((max_seconds * 120.0).round() as usize).clamp(1, MAX_PREDICTION_STEPS);
        let mut pred_arena = arena.clone();
        for &car_id in self.car_slots.iter().flatten() {
            let _ = pred_arena.remove_car(car_id);
        }
        for i in 0..steps {
            pred_arena.step(1);
            let b = pred_arena.get_ball();
            self.pred_buffer[i] = BallPredSlicePod {
                pos: b.pos.to_array(),
                vel: b.vel.to_array(),
            };
        }
        self.pred_count = steps;
    }

    pub fn get_mutator_config_c(&self) -> MutatorConfigC {
        let m = self
            .arena
            .as_ref()
            .map(|a| *a.get_mutator_config())
            .unwrap_or_else(|| MutatorConfig::default(GameMode::Soccar));
        MutatorConfigC {
            gravity: m.gravity.to_array(),
            car_mass: m.car_mass,
            car_world_friction: m.car_world_friction,
            car_world_restitution: m.car_world_restitution,
            ball_mass: m.ball_mass,
            ball_max_speed: m.ball_max_speed,
            ball_drag: m.ball_drag,
            ball_world_friction: m.ball_world_friction,
            ball_world_restitution: m.ball_world_restitution,
            jump_accel: m.jump_accel,
            jump_immediate_force: m.jump_immediate_force,
            boost_accel_ground: m.boost_accel_ground,
            boost_accel_air: m.boost_accel_air,
            boost_used_per_second: m.boost_used_per_second,
            respawn_delay: m.respawn_delay,
            bump_cooldown_time: m.bump_cooldown_time,
            boost_pad_cooldown_big: m.boost_pad_cooldown_big,
            boost_pad_cooldown_small: m.boost_pad_cooldown_small,
            car_spawn_boost_amount: m.car_spawn_boost_amount,
            ball_hit_extra_force_scale: m.ball_hit_extra_force_scale,
            bump_force_scale: m.bump_force_scale,
            ball_radius: m.ball_radius,
            unlimited_flips: u8::from(m.unlimited_flips),
            unlimited_double_jumps: u8::from(m.unlimited_double_jumps),
            recharge_boost_enabled: u8::from(m.recharge_boost_enabled),
            _pad0: 0,
            recharge_boost_per_second: m.recharge_boost_per_second,
            recharge_boost_delay: m.recharge_boost_delay,
            demo_mode: m.demo_mode as i32,
            enable_team_demos: u8::from(m.enable_team_demos),
            _pad1: [0; 3],
            goal_base_threshold_y: m.goal_base_threshold_y,
        }
    }

    pub fn set_mutator_config_c(&mut self, c: &MutatorConfigC) -> i32 {
        let Some(arena) = self.arena.as_mut() else {
            return 0;
        };
        let mut m = *arena.get_mutator_config();
        m.gravity = Vec3A::from_array(c.gravity);
        m.car_mass = c.car_mass;
        m.car_world_friction = c.car_world_friction;
        m.car_world_restitution = c.car_world_restitution;
        m.ball_mass = c.ball_mass;
        m.ball_max_speed = c.ball_max_speed;
        m.ball_drag = c.ball_drag;
        m.ball_world_friction = c.ball_world_friction;
        m.ball_world_restitution = c.ball_world_restitution;
        m.jump_accel = c.jump_accel;
        m.jump_immediate_force = c.jump_immediate_force;
        m.boost_accel_ground = c.boost_accel_ground;
        m.boost_accel_air = c.boost_accel_air;
        m.boost_used_per_second = c.boost_used_per_second;
        m.respawn_delay = c.respawn_delay;
        m.bump_cooldown_time = c.bump_cooldown_time;
        m.boost_pad_cooldown_big = c.boost_pad_cooldown_big;
        m.boost_pad_cooldown_small = c.boost_pad_cooldown_small;
        m.car_spawn_boost_amount = c.car_spawn_boost_amount;
        m.ball_hit_extra_force_scale = c.ball_hit_extra_force_scale;
        m.bump_force_scale = c.bump_force_scale;
        m.ball_radius = c.ball_radius;
        m.unlimited_flips = c.unlimited_flips != 0;
        m.unlimited_double_jumps = c.unlimited_double_jumps != 0;
        m.recharge_boost_enabled = c.recharge_boost_enabled != 0;
        m.recharge_boost_per_second = c.recharge_boost_per_second;
        m.recharge_boost_delay = c.recharge_boost_delay;
        m.demo_mode = match c.demo_mode {
            1 => DemoMode::OnContact,
            2 => DemoMode::Disabled,
            _ => DemoMode::Normal,
        };
        m.enable_team_demos = c.enable_team_demos != 0;
        m.goal_base_threshold_y = c.goal_base_threshold_y;
        arena.set_mutator_config(m);
        1
    }

    pub fn restore_compact_state(&mut self, slice: &[f32]) -> i32 {
        if slice.len() < 410 {
            return 0;
        }
        let Some(arena) = self.arena.as_mut() else {
            return 0;
        };
        arena.set_tick_count(slice[0] as u64);
        self.state_buffer.header.goal_scored_flag = slice[1];

        let mut ball_arr = [0.0f32; 18];
        ball_arr.copy_from_slice(&slice[4..22]);
        self.set_ball_state_from_slice(&ball_arr);

        for slot in 0..MAX_CARS {
            let base = 22 + slot * 40;
            let mut car_arr = [0.0f32; 40];
            car_arr.copy_from_slice(&slice[base..base + 40]);
            self.set_car_state_from_slice(slot, &car_arr);
            if slice.len() >= 448 {
                let ja_base = 416 + slot * 4;
                if let Some(car_id) = self.car_slots[slot]
                    && let Some(arena) = self.arena.as_mut()
                {
                    let mut cs = *arena.get_car(car_id);
                    cs.is_jumping = slice[ja_base] > 0.5;
                    cs.air_time = slice[ja_base + 1];
                    if !cs.is_demoed {
                        cs.jump_time = slice[ja_base + 2];
                    }
                    cs.air_time_since_jump = slice[ja_base + 3];
                    let _ = arena.set_car_state_full(car_id, &cs);
                }
            }
        }
        self.sync_state_to_buffer();
        1
    }
}
