extern crate arrayvec; // Use static arrays like the embedded code

use arrayvec::{ArrayVec, ArrayString};
use nalgebra::{Vector2, Point2};
#[allow(unused_imports)]
use micromath::F32Ext; // f32.sin and f32.cos
use core::cmp::Ordering;
#[allow(unused_imports)]
use log::{info, warn};

// MAP_T=3.0 causes too much load, MAP_T=4.0 works fine
pub const MAP_T: f32 = 4.0; // Map tile width and height in cm
pub const MAP_W_REAL: f32 = 170.0; // Map width in cm
pub const MAP_H_REAL: f32 = MAP_W_REAL;
pub const MAP_W: u32 = (MAP_W_REAL / MAP_T) as u32; // Map width in tiles
pub const MAP_H: u32 = MAP_W;
pub const MAP_SIZE: usize = (MAP_W * MAP_H) as usize;

// Hough parameters
const DISTANCE_STEP: usize = 2; // Distance resolution (tiles)
const ANGLE_STEP: usize = 10; // Angle resolution (degrees)
const HOUGH_THRESHOLD: usize = (40.0 / MAP_T) as usize;
const KEEP_NUM_TOP_LINES: usize = 10;
pub const MAX_NUM_LINE_CANDIDATES: usize = 20;
const EDGE_MIN_POS: f32 = 40.0;
const EDGE_MAX_NEG: f32 = -5.0;
const MAX_DISTANCE: i32 = MAP_W as i32;
const NUM_DISTANCES: usize = MAX_DISTANCE as usize / DISTANCE_STEP;
const NUM_ANGLES: usize = 360 / ANGLE_STEP;

//const ANGLE_SIMILARITY_THRESHOLD: f32 = 20.0;
//const DISTANCE_SIMILARITY_THRESHOLD: f32 = 25.0;

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
    // Return value: Some(point) if a previously unoccupied point was now marked
    // occupied. This is returned in tile coordinates.
    pub fn paint_proximity_reading(&mut self, starting_position: Point2<f32>,
            angle_rad: f32, distance: f32, something_seen: bool,
            occupation_event_max_starting_value: f32) ->
            Option<Point2<u32>> {
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
                *tile = (*tile - 40.0).clamp(-100.0, 300.0); // Mark as unoccupied
            }

            if x0 == x1 && y0 == y1 { break; }
            let e2 = 2 * err;
            if e2 >= dy { err += dy; x0 += sx; }
            if e2 <= dx { err += dx; y0 += sy; }
        }

        let mut newly_occupied = None;

        if something_seen {
            // Paint the end tile
            let x = x1 as u32;
            let y = y1 as u32;
            if let Some(tile) = self.data.get_mut((y * self.width + x) as usize) {
                if *tile < occupation_event_max_starting_value {
                    newly_occupied = Some(Point2::new(x, y));
                }
                *tile = (*tile + 60.0).clamp(-100.0, 300.0); // Mark as occupied
            }
        }

        newly_occupied
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
            let mut line = ArrayString::<100>::new();
            for x in 0..self.width {
                let idx = (y * self.width + x) as usize;
                let tile_value = self.data[idx];
                let symbol = if robot_x == x && robot_y == y {
                    'R'
                } else if tile_value < -50.0 {
                    ' '
                } else if tile_value < -10.0 {
                    '.'
                } else if tile_value < 10.0 {
                    '+'
                } else if tile_value < 50.0 {
                    'x'
                } else {
                    'X'
                };
                line.push(symbol);
            }
            info!("{}", line);
        }
    }

    pub fn find_pattern<F>(&self, pattern: &[f32], pattern_width: u32, pattern_height: u32,
            ignore_map_below_significance: f32, score_for_ignore: f32, filter: F)
            -> Option<(u32, u32, f32)>
    where F: Fn(u32, u32) -> bool {
        let mut best_match = None;
        let mut best_score = f32::MAX;

        for y in 0..self.height.saturating_sub(pattern_height) {
            for x in 0..self.width.saturating_sub(pattern_width) {
                if filter(x, y) {
                    let score = self.calculate_match_score(x, y, pattern, pattern_width, pattern_height,
                            ignore_map_below_significance, score_for_ignore);
                    if score < best_score {
                        best_score = score;
                        best_match = Some((x, y, score));
                    }
                }
            }
        }

        best_match
    }

    // Finds pattern in such a way that if there are multiple matches with the
    // same score, the one closest to the given point is chosen
    pub fn find_pattern_starting_at<F>(
            &self, pattern: &[f32], pattern_width: u32, pattern_height: u32,
            ignore_map_below_significance: f32, score_for_ignore: f32, filter: F,
            x0: i32, y0: i32)
            -> Option<(u32, u32, f32)>
    where F: Fn(u32, u32) -> bool {
        let mut best_match = None;
        let mut best_score = f32::MAX;

        // TODO: Iterate the right amount
        for i in 0..(self.height.saturating_sub(pattern_height) *
                self.width.saturating_sub(pattern_width)) {
            let (x1, y1) = inverse_spiral_index(i);
            let x = x0 - pattern_width as i32 / 2 + x1;
            let y = y0 - pattern_height as i32 / 2 + y1;
            //info!("({}, {}) ({}, {}) ({}, {}) ({}, {})", x1, y1, x, y,
            //        pattern_width, pattern_height, self.width, self.height);
            if x < 0 || x > self.width.saturating_sub(pattern_width) as i32 {
                continue;
            }
            if y < 0 || y > self.height.saturating_sub(pattern_height) as i32 {
                continue;
            }
            let x: u32 = x.try_into().unwrap();
            let y: u32 = y.try_into().unwrap();
            if filter(x, y) {
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
}

pub fn iterate_outwards_from_center(len: u32, i: u32) -> u32 {
    let center = len / 2;
    if (i % 2) == 0 {
        center + i / 2
    } else {
        center - (i + 1) / 2
    }
}

pub fn inverse_spiral_index(index: u32) -> (i32, i32) {
    // Special cases for the initial indices of the spiral
    match index {
        0 => return (0, 0),
        1 => return (0, -1),
        2 => return (1, -1),
        3 => return (1, 0),
        4 => return (1, 1),
        5 => return (0, 1),
        6 => return (-1, 1),
        7 => return (-1, 0),
        8 => return (-1, -1),
        _ => {}
    };

    let index = (index + 1) as i32;

    let layer = (((index as f32).sqrt() - 1.0) / 2.0).ceil() as i32;
    let prev_layer_end = (2 * (layer - 1) + 1).pow(2);
    let pos_in_layer = index - prev_layer_end - 1;
    let side_len = layer * 2;

    let side = pos_in_layer / side_len;
    let pos_in_side = pos_in_layer % side_len;
    let offset = pos_in_side - (layer - 1);

    match side {
        0 => (offset, -layer),
        1 => (layer, offset),
        2 => (-offset, layer),
        _ => (-layer, -offset),
    }
}

pub fn accept_any_xy(_x: u32, _y: u32) -> bool { true }

impl Map {
    pub fn find_binary_pattern<F>(&self, pattern: &[bool], pattern_width: u32, pattern_height: u32,
            ignore_map_below_significance: f32, score_for_ignore: f32,
            weights: Option<&[f32]>, filter: F)
            -> Option<(u32, u32, f32)>
    where F: Fn(u32, u32) -> bool {
        let mut best_match = None;
        let mut best_score = f32::MAX;

        for y in 0..self.height.saturating_sub(pattern_height) {
            for x in 0..self.width.saturating_sub(pattern_width) {
                if filter(x, y) {
                    let score = self.calculate_binary_match_score(
                            x, y, pattern, pattern_width, pattern_height,
                            ignore_map_below_significance, score_for_ignore, weights);
                    if score < best_score {
                        best_score = score;
                        best_match = Some((x, y, score));
                    }
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

type Accumulator = ArrayVec<ArrayVec<u16, NUM_DISTANCES>, NUM_ANGLES>;

#[derive(Clone)]
pub struct HoughLine {
    pub angle: f32, // Unit: degrees
    pub distance: f32, // Unit: map tiles
    pub votes: usize,
}

impl HoughLine {
    pub fn new(angle: f32, distance: f32, votes: usize) -> Self {
        HoughLine {
            angle: angle,
            distance: distance,
            votes: votes,
        }
    }
    pub fn default() -> Self {
        HoughLine {
            angle: 0.0,
            distance: 0.0,
            votes: 0,
        }
    }
    pub fn distance(&self, point: Vector2<f32>) -> f32 {
        let angle_rad = self.angle.to_radians();
        let cos_theta = angle_rad.cos();
        let sin_theta = angle_rad.sin();
        let rho = self.distance;

        ((point.x * cos_theta + point.y * sin_theta) - rho).abs()
    }
    pub fn vector_to_point(&self, point: Vector2<f32>) -> Vector2<f32> {
        let angle_rad = self.angle.to_radians();
        let line_normal = Vector2::new(angle_rad.cos(), angle_rad.sin());
        
        // Calculate the closest point on the line to the given point
        let perpendicular_offset = point.dot(&line_normal) - self.distance;
        let closest_point_on_line = point - line_normal * perpendicular_offset;

        // The vector from the point to the closest point on the line
        point - closest_point_on_line
    }
    pub fn move_towards_point(&self, target_point: Vector2<f32>, move_distance: f32) -> HoughLine {
        let angle_rad = self.angle.to_radians();
        let line_normal = Vector2::new(angle_rad.cos(), angle_rad.sin());

        // Calculate the current perpendicular distance from the target point to the line
        // TODO: Why isn't this used at all?
        //let current_distance = (target_point.dot(&line_normal) - self.distance).abs();

        // Calculate the new distance for the line after moving
        let new_distance = if self.distance < target_point.dot(&line_normal) {
            self.distance + move_distance
        } else {
            self.distance - move_distance
        };

        HoughLine::new(self.angle, new_distance, self.votes)
    }
}

pub fn angle_difference(angle1: f32, angle2: f32) -> f32 {
    let diff = (angle1 - angle2).abs() % 360.0;
    if diff > 180.0 { 360.0 - diff } else { diff }
}

fn quicksort<T, F>(arr: &mut [T], compare: &F)
where
    F: Fn(&T, &T) -> Ordering,
{
    if arr.len() > 1 {
        let p = partition(arr, compare);
        quicksort(&mut arr[0..p], compare);
        quicksort(&mut arr[p + 1..], compare);
    }
}

fn partition<T, F>(arr: &mut [T], compare: &F) -> usize
where
    F: Fn(&T, &T) -> Ordering,
{
    let pivot_index = arr.len() / 2;
    arr.swap(pivot_index, arr.len() - 1);
    let mut i = 0;

    for j in 0..arr.len() - 1 {
        if compare(&arr[j], &arr[arr.len() - 1]) != Ordering::Greater {
            arr.swap(i, j);
            i += 1;
        }
    }

    arr.swap(i, arr.len() - 1);
    i
}

// units: tiles
pub fn calculate_intersection(ray_origin: Vector2<f32>, ray_direction: Vector2<f32>,
        line: &HoughLine) -> Option<Vector2<f32>> {
    let angle_rad = line.angle.to_radians();
    let line_normal = Vector2::new(angle_rad.cos(), angle_rad.sin());

    /*{
        let normal_direction = Vector2::new(angle_rad.cos(), angle_rad.sin());
        let line_point = normal_direction * line.distance;
        let line_direction = Vector2::new((angle_rad+f32::pi()*0.5).cos(),
                (angle_rad+f32::pi()*0.5).sin());

        info!("Line at: p(tiles)={:?}, direction={:?}", line_point, line_direction);
        info!("Ray at: o(tiles)={:?}, direction={:?}", ray_origin, ray_direction);
    }*/
    
    // Line equation: x cos(theta) + y sin(theta) = rho
    let rho = line.distance;

    // Ray equation: ray_origin + s * ray_direction
    // Substitute the ray equation into the line equation and solve for s
    let denominator = ray_direction.dot(&line_normal);

    if denominator.abs() < 1e-10 {
        // Ray is parallel to the line, no intersection
        return None;
    }

    let s = (rho - ray_origin.dot(&line_normal)) / denominator;

    if s < 0.0 {
        // Intersection is behind the ray's origin
        return None;
    }

    Some(ray_origin + s * ray_direction)
}

impl Map {
    pub fn hough_transform(&self) -> ArrayVec<HoughLine, MAX_NUM_LINE_CANDIDATES> {
        let mut accumulator = Accumulator::new();
        // Initialize accumulator with zerosa
        for _ in 0..NUM_ANGLES {
            let mut distances = ArrayVec::<u16, NUM_DISTANCES>::new();
            for _ in 0..NUM_DISTANCES {
                distances.push(0); // Initialize each distance element to 0
            }
            accumulator.push(distances); // Add to accumulator
        }

        for y in 0..self.height {
            for x in 0..self.width {
                if self.is_edge(x, y) {
                    for angle_index in 0..NUM_ANGLES {
                        let angle = angle_index as f32 * ANGLE_STEP as f32;
                        let distance = (x as f32 * angle.to_radians().cos() + y as f32 * angle.to_radians().sin()) as f32;
                        let distance_index = distance as usize / DISTANCE_STEP;
                        if distance >= 0.0 && distance < MAX_DISTANCE as f32 &&
                                distance_index < NUM_DISTANCES {
                           /* info!("x: {:?}, y: {:?}, distance: {:?}, angle: {:?} -> angle_index: {:?}, distance_index: {:?}",
                                    x, y, distance, angle, angle_index, distance_index);*/
                            accumulator[angle_index][distance_index] += 1;
                        }
                    }
                }
            }
        }

        self.detect_lines(&accumulator)
    }

    pub fn is_edge(&self, x: u32, y: u32) -> bool {
        // The given x,y position is considered to be position 0.
        // For us to consider the position an edge, it must have a high value
        // (i.e. it must be the inside of a solid wall). If it is not, we
        // discard the position immediately.
        let i0 = (y * self.width + x) as usize;
        if self.data[i0] < EDGE_MIN_POS {
            return false;
        }
        // For us to consider the position an edge, it must have a neighbor with
        // a low value.
        let dirs = [
            (-1, 0),
            ( 1, 0),
            ( 0,-1),
            ( 0, 1),
        ];
        for dir in dirs {
            let x1 = x as i32 + dir.0;
            let y1 = y as i32 + dir.1;
            if x1 < 0 || x1 >= self.width as i32 || y1 < 0 || y1 >= self.height as i32 {
                continue;
            }
            let i1 = (y1 as u32 * self.width + x1 as u32) as usize;
            if self.data[i1] < -EDGE_MAX_NEG {
                return true;
            }
        }
        return false;
    }

    pub fn detect_lines(&self, accumulator: &Accumulator) -> ArrayVec<HoughLine, MAX_NUM_LINE_CANDIDATES> {
        let mut line_candidates: ArrayVec<HoughLine, MAX_NUM_LINE_CANDIDATES> =
                ArrayVec::new();

        // Find peaks in the accumulator
        for (angle_index, distances) in accumulator.iter().enumerate() {
            for (distance_index, &votes) in distances.iter().enumerate() {
                //info!("votes: {:?}", votes);
                if votes as usize >= HOUGH_THRESHOLD {
                    let angle = angle_index as f32 * ANGLE_STEP as f32;
                    let distance = distance_index as f32 * DISTANCE_STEP as f32 +
                            DISTANCE_STEP as f32 / 2.0;
                    // TODO: Maybe care about the result of the try_push
                    let _ = line_candidates.try_push(HoughLine::new(angle, distance, votes.into()));
                }
            }
        }

        // TODO: Make this work?
        //line_candidates = self.merge_similar_lines(line_candidates);

        // Sort by votes and select top lines
        // Sort in descending order of votes
        // NOTE: ArrayVec::sort_by is not available with no_std
        //line_candidates.sort_by(|a, b| b.votes.cmp(&a.votes));
        // Use our own quicksort instead
        quicksort(&mut line_candidates, &|a: &HoughLine, b: &HoughLine| b.votes.cmp(&a.votes));
        line_candidates.truncate(KEEP_NUM_TOP_LINES); // Keep only the top lines

        return line_candidates;
    }

    // TODO: Fix this?
    /*fn merge_similar_lines(&self, lines: ArrayVec<HoughLine, MAX_NUM_LINE_CANDIDATES>)
            -> ArrayVec<HoughLine, MAX_NUM_LINE_CANDIDATES> {
        let mut merged_lines: ArrayVec<HoughLine, MAX_NUM_LINE_CANDIDATES> = ArrayVec::new();
        let mut used: ArrayVec<bool, MAX_NUM_LINE_CANDIDATES> = ArrayVec::new();
        for _i in 0..lines.len() {
            used.push(false);
        }

        for (i, line) in lines.iter().enumerate() {
            if used[i] { continue; }

            let mut similar_lines: ArrayVec<HoughLine, MAX_NUM_LINE_CANDIDATES> = ArrayVec::new();
            similar_lines.push(line.clone());
            for (j, other_line) in lines.iter().enumerate().skip(i + 1) {
                if used[j] { continue; }

                if angle_difference(line.angle, other_line.angle) < ANGLE_SIMILARITY_THRESHOLD &&
                   (line.distance - other_line.distance).abs() < DISTANCE_SIMILARITY_THRESHOLD {
                    similar_lines.push(other_line.clone());
                    used[j] = true;
                }
            }

            let average_line = similar_lines.iter().fold(HoughLine::default(), |acc, line| {
                HoughLine {
                    angle: 0.0, // Cannot be averaged this way
                    //distance: acc.distance + line.distance * line.votes as f32, // Weighed average
                    distance: acc.distance + line.distance,
                    votes: acc.votes + line.votes,
                    // handle other fields if any
                }
            });

            let sum_vector = similar_lines.iter().fold(Vector2::new(0.0, 0.0), |acc, line| {
                let angle_rad = line.angle.to_radians();
                acc + Vector2::new(angle_rad.cos(), angle_rad.sin())
            });
            let average_vector = sum_vector / similar_lines.len() as f32;
            let average_angle = average_vector.y.atan2(average_vector.x).to_degrees();

            merged_lines.push(HoughLine {
                angle: average_angle, // TODO: Weighed average
                //distance: average_line.distance / average_line.votes as f32, // Weighed average
                distance: average_line.distance / similar_lines.len() as f32, // Weighed average
                votes: average_line.votes,
                // handle other fields if any
            });

            /*let average_line = similar_lines.iter().fold(HoughLine::default(), |acc, line| {
                HoughLine {
                    angle: acc.angle + line.angle,
                    distance: acc.distance + line.distance,
                    votes: acc.votes + line.votes,
                    // handle other fields if any
                }
            });
            
            merged_lines.push(HoughLine {
                angle: average_line.angle / similar_lines.len() as f32,
                distance: average_line.distance / similar_lines.len() as f32,
                votes: average_line.votes,
                // handle other fields if any
            });*/
        }

        merged_lines
    }*/

}

#[cfg(test)]
mod tests {
    use super::*;
    use fixedstr::str_format;

    // NOTE: Run using "cargo test -- --test-threads=1" or "cargo test <test
    //       name>" to see the console output properly

    #[test]
    fn test_hough_transform() {
        // Set up a Map with known lines
        let mut map = Map::new();
        // Populate `map` with data that forms clear, detectable lines
        for y in 0..map.height {
            for x in 0..map.width {
                let i = (y * map.width + x) as usize;
                // This should create a 100 long vertical line and a 150 long
                // horizontal line
                if y < (150.0 / map.tile_wh) as u32 &&
                        x < (100.0 / map.tile_wh) as u32{
                    map.data[i] = -100.0;
                } else {
                    map.data[i] = 100.0;
                }
            }
        }

        map.print(Point2::new(0.0, 0.0));

        // Perform the Hough Transform
        let lines = map.hough_transform();

        for line in &lines {
            info!("HoughLine: angle={:?} distance={:?} votes={:?}",
                    line.angle, line.distance, line.votes);
        }

        // The longest line is the 150 long horizontal line (= angle 0), at
        // Y=100 (= distance 100)
        assert_eq!(lines[0].angle, 0.0);
        assert!((lines[0].distance - 100.0 / map.tile_wh).abs() <= 1.5);

        // The second longest line is the 100 long vertical line (= angle 90),
        // at X=150 (= distance 150)
        assert_eq!(lines[1].angle, 90.0);
        assert!((lines[1].distance - 150.0 / map.tile_wh).abs() <= 1.5);
        assert!(lines[0].votes > lines[1].votes);
    }

    #[test]
    fn test_hough_transform_2() {
        // Set up a Map with known lines
        let mut map = Map::new();
        // Populate `map` with data that forms clear, detectable lines
        for y in 0..map.height {
            for x in 0..map.width {
                let i = (y * map.width + x) as usize;
                // This should create a diagonal line 200/sqrt(2) away from
                // origin
                if y + x < (200.0 / map.tile_wh) as u32 {
                    map.data[i] = -100.0;
                } else {
                    map.data[i] = 100.0;
                }
            }
        }

        map.print(Point2::new(0.0, 0.0));

        // Perform the Hough Transform
        let lines = map.hough_transform();

        for line in &lines {
            info!("HoughLine: angle={:?} distance={:?} votes={:?}",
                    line.angle, line.distance, line.votes);
        }

        // Ideally we would get a 45 degree line at 141.42 / map.tile_wh
        // distance, but instead we get 40 and 50 degree lines at distances
        // close to this one
        for i in 0..(4.min(KEEP_NUM_TOP_LINES - 1)) {
            assert!((lines[i].angle - 45.0).abs() <= 5.0);
            assert!((lines[i].distance - 141.42 / map.tile_wh).abs() <= 2.5);
            if i != 3 {
                assert_eq!(lines[i].votes, lines[i+1].votes);
            }
        }
    }

    #[test]
    fn test_calculate_intersection() {
        // HoughLines units are degrees and tiles
        // This line is parallel to the Y axis, crossing the X axis at 10.0
        let line = HoughLine::new(0.0, 10.0, 0);
        // Use a ray starting at (0, 5) going parallel with the +X axis
        let ray_origin = Vector2::new(0.0, 5.0);
        let ray_direction = Vector2::new(1.0, 0.0);
        let r = calculate_intersection(ray_origin, ray_direction, &line);
        info!("r: {:?}", r);
        assert!(!r.is_none());
        if let Some(intersection_point) = r {
            // The intersection should happen at (10, 5)
            assert_eq!(intersection_point.x, 10.0);
            assert_eq!(intersection_point.y, 5.0);
        }

        // HoughLines units are degrees and tiles
        // This line is at a 45 degree slope from -X-Y to +X+Y, crossing the X
        // axis at 5 * sqrt(2)
        let line = HoughLine::new(315.0, 5.0, 0);
        // Use a ray starting at (0, 5) going parallel with the +X axis
        let ray_origin = Vector2::new(0.0, 5.0);
        let ray_direction = Vector2::new(1.0, 0.0);
        let r = calculate_intersection(ray_origin, ray_direction, &line);
        info!("r: {:?}", r);
        assert!(!r.is_none());
        if let Some(intersection_point) = r {
            // Should intersect at Y=5 because our ray travels parallel to X
            assert_eq!(intersection_point.y, 5.0);
            // Was experimentally found out but apparently is correct
            assert!((intersection_point.x - 12.07).abs() <= 0.1);
        }
    }

    #[test]
    fn test_distance_from_line() {
        let line = HoughLine::new(0.0, 10.0, 0); // Line parallel to Y-axis, 10 units from X-axis
        let point = Vector2::new(15.0, 0.0); // Point to the right of the line

        let distance = line.distance(point);
        assert_eq!(distance, 5.0); // Distance should be 5 units
    }

    #[test]
    fn test_vector_to_point() {
        let line = HoughLine::new(90.0, 10.0, 0); // Line parallel to X-axis, 10 units from Y-axis
        let point = Vector2::new(0.0, 15.0); // Point above the line

        let vector = line.vector_to_point(point);
        // Vector pointing downwards, 5 units
        assert!((vector.x - 0.0).abs() < 1e-6);
        assert_eq!(vector.y, 5.0);
    }

    fn assert_vector_eq_with_tolerance(vec1: Vector2<f32>, vec2: Vector2<f32>, tolerance: f32) {
        if (vec1.x - vec2.x).abs() >= tolerance || (vec1.y - vec2.y).abs() >= tolerance {
            info!("Assertion failed: Vectors not equal within tolerance");
            info!("Actual vector:   [{}, {}]", vec1.x, vec1.y);
            info!("Expected vector: [{}, {}]", vec2.x, vec2.y);
            info!("Tolerance: {}", tolerance);
            panic!("Vectors differ beyond tolerance");
        }
    }


    const FLOAT_TOLERANCE: f32 = 1e-6;

    #[test]
    fn test_vector_to_point_horizontal_line() {
        let line = HoughLine::new(90.0, 10.0, 0);

        let point_above = Vector2::new(0.0, 15.0);
        let vector_above = line.vector_to_point(point_above);
        assert_vector_eq_with_tolerance(vector_above, Vector2::new(0.0, 5.0), FLOAT_TOLERANCE);

        let point_below = Vector2::new(0.0, 5.0);
        let vector_below = line.vector_to_point(point_below);
        assert_vector_eq_with_tolerance(vector_below, Vector2::new(0.0, -5.0), FLOAT_TOLERANCE);
    }

    #[test]
    fn test_vector_to_point_vertical_line() {
        let line = HoughLine::new(0.0, 10.0, 0);

        let point_left = Vector2::new(5.0, 0.0);
        let vector_left = line.vector_to_point(point_left);
        assert_vector_eq_with_tolerance(vector_left, Vector2::new(-5.0, 0.0), FLOAT_TOLERANCE);

        let point_right = Vector2::new(15.0, 0.0);
        let vector_right = line.vector_to_point(point_right);
        assert_vector_eq_with_tolerance(vector_right, Vector2::new(5.0, 0.0), FLOAT_TOLERANCE);
    }

    #[test]
    fn test_iterate_outwards_from_center() {
        assert_eq!(iterate_outwards_from_center(1, 0), 0);

        assert_eq!(iterate_outwards_from_center(2, 0), 1);
        assert_eq!(iterate_outwards_from_center(2, 1), 0);

        assert_eq!(iterate_outwards_from_center(3, 0), 1);
        assert_eq!(iterate_outwards_from_center(3, 1), 0);
        assert_eq!(iterate_outwards_from_center(3, 2), 2);

        assert_eq!(iterate_outwards_from_center(4, 0), 2);
        assert_eq!(iterate_outwards_from_center(4, 1), 1);
        assert_eq!(iterate_outwards_from_center(4, 2), 3);
        assert_eq!(iterate_outwards_from_center(4, 3), 0);

        assert_eq!(iterate_outwards_from_center(5, 0), 2);
        assert_eq!(iterate_outwards_from_center(5, 1), 1);
        assert_eq!(iterate_outwards_from_center(5, 2), 3);
        assert_eq!(iterate_outwards_from_center(5, 3), 0);
        assert_eq!(iterate_outwards_from_center(5, 4), 4);
    }

    #[test]
    fn test_inverse_spiral_index() {
        stderrlog::new()
            .verbosity(log::LevelFilter::Info)
            .init().unwrap();

        const D: usize = 15;
        let mut table: [u32; D*D] = [1337; D*D];
        for i in 0..(D*D) as u32 {
            let (x, y) = inverse_spiral_index(i);
            //info!("inverse_spiral_index({}) -> {:?}", i, (x, y));
            let table_i = ((y + D as i32 / 2) * D as i32 + (x + D as i32 / 2)) as usize;
            table[table_i] = i;
        }
        info!("inverse_spiral_index():");
        for y in 0..D {
            let mut line = ArrayString::<100>::new();
            for x in 0..D {
                let table_i = y * D + x;
                let i = table[table_i];
                let s = str_format!(fixedstr::str8, "{:>3}", i);
                line.push_str(&s);
                line.push_str(",");
            }
            info!("{}", line);
        }

        // Initial 9 special cases
        assert_eq!(inverse_spiral_index( 0), (0, 0));
        assert_eq!(inverse_spiral_index( 1), (0, -1));
        assert_eq!(inverse_spiral_index( 2), (1, -1));
        assert_eq!(inverse_spiral_index( 8), (-1, -1));
        // Actual spiral
        assert_eq!(inverse_spiral_index( 9), (-1, -2));
        assert_eq!(inverse_spiral_index(10), (0, -2));
        assert_eq!(inverse_spiral_index(11), (1, -2));
        assert_eq!(inverse_spiral_index(12), (2, -2));
        assert_eq!(inverse_spiral_index(13), (2, -1));
        assert_eq!(inverse_spiral_index(14), (2, 0));
        assert_eq!(inverse_spiral_index(15), (2, 1));
        assert_eq!(inverse_spiral_index(30), (3, -3));
    }
}

