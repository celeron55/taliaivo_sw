extern crate arrayvec; // Use static arrays like the embedded code

use arrayvec::ArrayVec;
use libc_print::std_name::{println, eprintln, print, dbg};
use nalgebra::{Vector2, Point2, UnitComplex, Rotation2};

pub const MAP_T: f32 = 5.0; // Map tile width and height in cm
pub const MAP_W_REAL: f32 = 200.0; // Map width in cm
pub const MAP_H_REAL: f32 = MAP_W_REAL;
pub const MAP_W: u32 = (MAP_W_REAL / MAP_T) as u32; // Map width in tiles
pub const MAP_H: u32 = MAP_W;
pub const MAP_SIZE: usize = (MAP_W * MAP_H) as usize;

#[derive(Clone)]
pub struct Map {
    pub tile_wh: f32,
    pub width: u32,
    pub height: u32,
    pub data: ArrayVec<f32, MAP_SIZE>,
}

impl Map {
    pub fn new() -> Self {
        let mut data = ArrayVec::new();
        for _ in 0..MAP_SIZE {
            data.push(0.0);
        }
        Map {
            tile_wh: MAP_T,
            width: MAP_W,
            height: MAP_H,
            data: data,
        }
    }

    // something_seen: If nothing is found within sensor range, set this to
    // true, and set distance to the sensor maximum range
    pub fn paint_proximity_reading(&mut self, starting_position: Point2<f32>,
            angle_rad: f32, distance: f32, something_seen: bool) {
        // Calculate end point of the ray
        let direction: Vector2<f32> = Vector2::new(angle_rad.cos(), angle_rad.sin());
        let end_point = starting_position + direction * distance;

        // Convert to tile indices
        let mut x0 = (starting_position.x / self.tile_wh) as i32;
        let mut y0 = (starting_position.y / self.tile_wh) as i32;
        let x1 = (end_point.x / self.tile_wh) as i32;
        let y1 = (end_point.y / self.tile_wh) as i32;

        // Bresenham's line algorithm
        let dx = (x1 - x0).abs();
        let dy = -(y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;

        loop {
            // Paint the current tile
            if let Some(tile) = self.data.get_mut((y0 as u32 * self.width + x0 as u32) as usize) {
                *tile = -100.0;
            }

            if x0 == x1 && y0 == y1 { break; }
            let e2 = 2 * err;
            if e2 >= dy { err += dy; x0 += sx; }
            if e2 <= dx { err += dx; y0 += sy; }
        }

        if something_seen {
            // Paint the end tile
            if let Some(tile) = self.data.get_mut((y1 as u32 * self.width + x1 as u32) as usize) {
                *tile = 100.0;
            }
        }
    }

    pub fn global_forget(&mut self, factor: f32) {
        for y in 0..self.height {
            for x in 0..self.width {
                let idx = (y * self.width + x) as usize;
                let tile_value = &mut self.data[idx];
                *tile_value *= factor;
            }
        }
    }

    pub fn translate(&mut self, dx: i32, dy: i32) {
        let mut new_data = ArrayVec::<f32, MAP_SIZE>::new();
        for _ in 0..MAP_SIZE {
            new_data.push(0.0);
        }

        for y in 0..self.height {
            for x in 0..self.width {
                let new_x = x as i32 + dx;
                let new_y = y as i32 + dy;

                if new_x >= 0 && new_x < self.width as i32 && new_y >= 0 && new_y < self.height as i32 {
                    let old_idx = (y * self.width + x) as usize;
                    let new_idx = (new_y as u32 * self.width + new_x as u32) as usize;
                    new_data[new_idx] = self.data[old_idx];
                }
            }
        }

        self.data = new_data;
    }

    pub fn print(&self, robot_pos: Point2<f32>) {
        let robot_x = (robot_pos.x / self.tile_wh).round() as u32;
        let robot_y = (robot_pos.y / self.tile_wh).round() as u32;
        for y in 0..self.height {
            for x in 0..self.width {
                let idx = (y * self.width + x) as usize;
                let tile_value = self.data[idx];
                let symbol = if robot_x == x && robot_y == y {
                    "R"
                } else if tile_value < -50.0 {
                    " "
                } else if tile_value < -10.0 {
                    "."
                } else if tile_value < 10.0 {
                    "+"
                } else if tile_value < 50.0 {
                    "x"
                } else {
                    "X"
                };
                print!(" {}", symbol);
            }
            println!(); // New line at the end of each row
        }
    }

    pub fn find_pattern(&self, pattern: &[f32], pattern_width: u32, pattern_height: u32,
            ignore_map_below_significance: f32, score_for_ignore: f32)
            -> Option<(u32, u32, f32)> {
        let mut best_match = None;
        let mut best_score = f32::MAX;

        for y in 0..self.height.saturating_sub(pattern_height) {
            for x in 0..self.width.saturating_sub(pattern_width) {
                let score = self.calculate_match_score(x, y, pattern, pattern_width, pattern_height,
                        ignore_map_below_significance, score_for_ignore);
                if score < best_score {
                    best_score = score;
                    best_match = Some((x, y, score));
                }
            }
        }

        best_match
    }

    fn calculate_match_score(&self, x: u32, y: u32, pattern: &[f32], pattern_width: u32,
            pattern_height: u32, ignore_map_below_significance: f32, score_for_ignore: f32) -> f32 {
        let mut score = 0.0;

        for pattern_y in 0..pattern_height {
            for pattern_x in 0..pattern_width {
                let map_x = x + pattern_x;
                let map_y = y + pattern_y;
                let map_idx = (map_y * self.width + map_x) as usize;
                let pattern_idx = (pattern_y * pattern_width + pattern_x) as usize;

                if self.data[map_idx].abs() >= ignore_map_below_significance {
                    score += (self.data[map_idx] - pattern[pattern_idx]).abs();
                } else {
                    score += score_for_ignore;
                }
            }
        }

        score
    }

    pub fn find_binary_pattern(&self, pattern: &[bool], pattern_width: u32, pattern_height: u32,
            ignore_map_below_significance: f32, score_for_ignore: f32,
            weights: Option<&[f32]>)
            -> Option<(u32, u32, f32)> {
        let mut best_match = None;
        let mut best_score = f32::MAX;

        for y in 0..self.height.saturating_sub(pattern_height) {
            for x in 0..self.width.saturating_sub(pattern_width) {
                let score = self.calculate_binary_match_score(
                        x, y, pattern, pattern_width, pattern_height,
                        ignore_map_below_significance, score_for_ignore, weights);
                if score < best_score {
                    best_score = score;
                    best_match = Some((x, y, score));
                }
            }
        }

        best_match
    }

    fn calculate_binary_match_score(&self, x: u32, y: u32, pattern: &[bool], pattern_width: u32,
            pattern_height: u32, ignore_map_below_significance: f32, score_for_ignore: f32,
            weights: Option<&[f32]>) -> f32 {
        let mut score = 0.0;

        for pattern_y in 0..pattern_height {
            for pattern_x in 0..pattern_width {
                let map_x = x + pattern_x;
                let map_y = y + pattern_y;
                let map_idx = (map_y * self.width + map_x) as usize;
                let pattern_idx = (pattern_y * pattern_width + pattern_x) as usize;
                let mut weight = 1.0;
                if let Some(weights_) = weights {
                    weight = weights_[pattern_idx];
                }

                if self.data[map_idx].abs() >= ignore_map_below_significance {
                    if pattern[pattern_idx] && self.data[map_idx] < 0.0 {
                        score += weight;
                    } else if !pattern[pattern_idx] && self.data[map_idx] > 0.0 {
                        score += weight;
                    }
                } else {
                    score += score_for_ignore * weight;
                }
            }
        }

        score
    }
}
