---
title: Coverage planner
---
# {{ $frontmatter.title }}

## Overview

`coverage_server` plans mowing paths with `coverage_planner`, Daniel Wiegert's Boost.Geometry planner (it replaced
the slic3r-based planner OpenMower originally used). Per area:

1. Build the field: the outline minus exclusions; exclusions fully inside get rings.
2. Perimeter loops (`outline_count`) and obstacle rings from shared per-level contours. A loop wraps around an
   obstacle that sits in its band.
3. Fill: serpentine lanes (`lane_skip` groups for grass recovery, validated U-turns) or concentric rings.
4. Order the fill and obstacle runs by proximity after the perimeter.

## Service

`/area_coverage` (`open_mower_next/srv/AreaCoverage`): `area_id`, `with_exclusions`, `headland_loops`, `swath_angle`.
It returns `paths` (`CoveragePath[]`: kind, `is_outline`, `nav_msgs/Path`) in driving order, their concatenation
`path`, and the mowable geometry.

## Parameters

`tool_width` (0.22), `overlap` (0.35), `clearance` (0.04), `obstacle_clearance` (follows clearance), `outline_count`
(-1: request), `fill_mode` (request | serpentine | concentric), `lane_skip` (2), `optimize_sweep_angle` (true, uses
the area's principal axis; `swath_angle` applies when false), `edge_side` (right), `body_width`/`body_length`,
`min_turn_radius`, `blade_offset_x/y`, `path_spacing` (0.1).
