#!/usr/bin/env python3
"""
PROFESSIONAL AEB VISUALIZATION SYSTEM
ROS2 Humble | Pygame | Python 3.10+
Futuristic ADAS demo with full dashboard and scenario events.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import threading
import math
import json
import time
import random
import os
from typing import List, Tuple, Optional, Dict

from autonomous_braking.config import (
    VISUALIZATION as VIZ, ROAD, SAFETY, PHYSICS,
    VehicleStatus, LanePosition, Topics
)

# ═══════════════════════════════════════════════════════════════
#  DISPLAY CONSTANTS
# ═══════════════════════════════════════════════════════════════

WIDTH  = VIZ.width   # 1400
HEIGHT = VIZ.height  # 800
FPS    = VIZ.fps     # 60

HORIZON_Y    = 255
CENTER_X     = WIDTH // 2
DASH_HEIGHT  = 182                    # bottom dashboard panel height
ROAD_BOTTOM_Y = HEIGHT - DASH_HEIGHT  # where road perspective ends (618)

# ─── Colour palette ──────────────────────────────────────────
C = {
    # Sky / environment
    'sky_zenith':   (5,   8,  22),
    'sky_mid':      (12,  18,  48),
    'sky_horizon':  (25,  38,  80),
    'horizon_glow': (60,  90, 140),
    'mountain1':    (18,  28,  52),
    'mountain2':    (14,  22,  42),
    'grass_far':    (14,  30,  14),
    'grass_near':   (18,  38,  16),

    # Road
    'road':         (36,  38,  44),
    'road_alt':     (30,  32,  38),
    'road_edge':    (55,  58,  65),
    'shoulder':     (48,  50,  56),

    # Lane markings
    'lane_white':   (210, 215, 225),
    'lane_yellow':  (255, 195,  40),

    # Dashboard background / chrome
    'dash_bg':      (8,   11,  20),
    'dash_panel':   (13,  17,  30),
    'dash_border':  (30,  40,  65),
    'dash_divider': (25,  33,  55),
    'dash_header':  (18,  24,  42),

    # Text
    'text_bright':  (230, 240, 255),
    'text_mid':     (160, 175, 200),
    'text_dim':     (75,  88, 115),
    'text_label':   (95, 110, 145),

    # Status colours
    'safe':         (30,  210,  110),
    'caution':      (90,  175,  255),
    'warning':      (255, 200,   35),
    'danger':       (255, 110,   30),
    'emergency':    (255,  35,   35),
    'stopped':      (140, 145,  160),
    'braking':      (255,  75,   30),
    'lane_change':  (80,  170,  255),

    # LIDAR
    'lidar_bg':     (7,   10,  18),
    'lidar_grid':   (20,  30,  50),
    'lidar_sweep':  (0,  220,  100),
    'lidar_pt_safe':(0,  255,  120),
    'lidar_pt_warn':(255,195,    0),
    'lidar_pt_crit':(255,  45,   45),
    'lidar_ego':    (40, 140,  255),

    # Misc
    'target_box':   (255, 60,   60),
    'headlight':    (255,255,  180),
}

STATUS_COLORS = {
    'SAFE':        C['safe'],
    'CAUTION':     C['caution'],
    'WARNING':     C['warning'],
    'DANGER':      C['danger'],
    'EMERGENCY':   C['emergency'],
    'STOPPED':     C['stopped'],
    'BRAKING':     C['braking'],
    'LANE_CHANGE': C['lane_change'],
}

CAR_SPRITES = ['car_red', 'car_blue', 'car_white', 'car_green',
               'car_yellow', 'car_purple', 'car_gray', 'car_orange']


# ═══════════════════════════════════════════════════════════════
#  SPRITE LOADING
# ═══════════════════════════════════════════════════════════════

SPRITES: Dict[str, Optional[pygame.Surface]] = {}

def load_sprites():
    # Sprites live at  <package_root>/sprites/  which is one directory above
    # this Python file (.../autonomous_braking/autonomous_braking/visualizer.py).
    # When installed via colcon, also check the ROS2 share directory.
    base = None

    # Try 1: installed share directory (colcon build without --symlink-install)
    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = os.path.join(
            get_package_share_directory('autonomous_braking'), 'sprites')
        if os.path.isdir(candidate):
            base = candidate
    except Exception:
        pass

    # Try 2: source directory — one level up from this .py file
    if base is None:
        candidate = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'sprites')
        if os.path.isdir(candidate):
            base = candidate

    if base is None:
        print("  Warning: sprites directory not found — using polygon fallbacks")
        return

    names = CAR_SPRITES + ['ego', 'truck', 'motorcycle', 'bicycle',
                            'pedestrian', 'child', 'ball', 'animal', 'fallen_tree']
    for name in names:
        path = os.path.join(base, f'{name}.png')
        if os.path.exists(path):
            try:
                SPRITES[name] = pygame.image.load(path).convert_alpha()
            except Exception as e:
                print(f"  Warning: could not load sprite {name}: {e}")
                SPRITES[name] = None
        else:
            SPRITES[name] = None
    loaded = sum(1 for v in SPRITES.values() if v is not None)
    print(f"  Loaded {loaded}/{len(names)} sprites from {base}")


def pick_car_sprite(color: Tuple) -> str:
    r, g, b = color[0], color[1], color[2]
    if r > 180 and g < 100 and b < 100: return 'car_red'
    if b > 150 and r < 120:             return 'car_blue'
    if r > 190 and g > 190 and b > 190: return 'car_white'
    if g > 150 and r < 100:             return 'car_green'
    if r > 190 and g > 150 and b < 80:  return 'car_yellow'
    if r > 120 and b > 120 and g < 80:  return 'car_purple'
    if r > 180 and g > 100 and b < 60:  return 'car_orange'
    return 'car_gray'


def draw_sprite(screen, name: str, sx: int, sy: int, sw: int, sh: int) -> bool:
    surf = SPRITES.get(name)
    if surf is None or sw < 4 or sh < 4:
        return False
    try:
        scaled = pygame.transform.scale(surf, (sw, sh))
        rect = scaled.get_rect(midbottom=(sx, sy))
        screen.blit(scaled, rect)
        return True
    except Exception:
        return False


# ═══════════════════════════════════════════════════════════════
#  ROS2 NODE
# ═══════════════════════════════════════════════════════════════

class AEBVisualizer(Node):
    def __init__(self):
        super().__init__('aeb_visualizer')
        self.state = {}
        self.ego = {}
        self.aeb = {}
        self.scenario = {}
        self.decision_text = "Safe: road clear"
        self.obstacles = []
        self.lidar_points = []
        self.stats = {}
        self.create_subscription(String, Topics.FULL_STATE, self._state_cb, 10)
        self.get_logger().info('AEB Visualizer started')

    def _state_cb(self, msg):
        try:
            s = json.loads(msg.data)
            self.ego          = s.get('ego', {})
            self.aeb          = s.get('aeb', {})
            self.scenario     = s.get('scenario', {})
            self.decision_text = s.get('decision_text', 'Safe: road clear')
            self.obstacles    = s.get('obstacles', [])
            self.lidar_points = s.get('lidar', [])
            self.stats        = s.get('stats', {})
        except json.JSONDecodeError:
            pass


def ros_spin(node):
    rclpy.spin(node)


# ═══════════════════════════════════════════════════════════════
#  PERSPECTIVE HELPERS
# ═══════════════════════════════════════════════════════════════

def world_to_screen(rel_x: float, rel_y: float,
                    width: float = 1.8, height: float = 1.5):
    if rel_y < 0.5:
        rel_y = 0.5
    # Smaller offset (3 instead of 5) + larger multiplier (6 instead of 3.5)
    # places objects noticeably lower on screen, making them feel much closer.
    depth = 250.0 / (rel_y + 3.0)
    sy = int(HORIZON_Y + depth * 6.0)
    sy = min(sy, ROAD_BOTTOM_Y)   # clamp: very close objects stay on road surface

    road_world_half = ROAD.num_lanes * ROAD.lane_width / 2.0   # 5.55 m
    clamped_y = max(HORIZON_Y, min(sy, ROAD_BOTTOM_Y))
    road_pixel_half = get_road_w(clamped_y) / 2.0
    x_scale = road_pixel_half / road_world_half

    sx = int(CENTER_X + rel_x * x_scale)
    sw = max(4, int(width  * x_scale))
    sh = min(180, max(3, int(height * depth * 2.0)))
    return sx, sy, sw, sh


def get_road_w(screen_y: int) -> float:
    if screen_y <= HORIZON_Y:
        return 40
    t = max(0.0, min(1.0, (screen_y - HORIZON_Y) / (ROAD_BOTTOM_Y - HORIZON_Y)))
    return 40 + t * 590


# ═══════════════════════════════════════════════════════════════
#  ENVIRONMENT DRAWING
# ═══════════════════════════════════════════════════════════════

def draw_sky(screen, stars, clouds):
    """Gradient sky with stars, subtle clouds, and horizon glow."""
    sky_h = HORIZON_Y + 30

    # Sky gradient — zenith to horizon
    for y in range(sky_h):
        t = y / sky_h
        if t < 0.5:
            s = t / 0.5
            r = int(C['sky_zenith'][0] + (C['sky_mid'][0] - C['sky_zenith'][0]) * s)
            g = int(C['sky_zenith'][1] + (C['sky_mid'][1] - C['sky_zenith'][1]) * s)
            b = int(C['sky_zenith'][2] + (C['sky_mid'][2] - C['sky_zenith'][2]) * s)
        else:
            s = (t - 0.5) / 0.5
            r = int(C['sky_mid'][0] + (C['sky_horizon'][0] - C['sky_mid'][0]) * s)
            g = int(C['sky_mid'][1] + (C['sky_horizon'][1] - C['sky_mid'][1]) * s)
            b = int(C['sky_mid'][2] + (C['sky_horizon'][2] - C['sky_mid'][2]) * s)
        pygame.draw.line(screen, (r, g, b), (0, y), (WIDTH, y))

    # Horizon atmospheric glow (wide soft band)
    glow_surf = pygame.Surface((WIDTH, 40), pygame.SRCALPHA)
    for dy in range(40):
        alpha = int(55 * (1 - dy / 40))
        pygame.draw.line(glow_surf, (*C['horizon_glow'], alpha), (0, dy), (WIDTH, dy))
    screen.blit(glow_surf, (0, HORIZON_Y - 10))

    # Stars
    for sx, sy, bright in stars:
        if sy < sky_h - 10:
            c = int(bright * 255)
            size = 1 if bright < 0.7 else 1
            pygame.draw.circle(screen, (c, c, int(c * 0.85)), (sx, sy), size)

    # Subtle cloud shapes
    for cx, cy, cw, alpha in clouds:
        cs = pygame.Surface((cw, 18), pygame.SRCALPHA)
        for dx in range(0, cw, 4):
            a = int(alpha * math.exp(-((dx - cw/2)**2) / (cw*cw/8)))
            pygame.draw.ellipse(cs, (180, 190, 210, a), (dx-8, 0, 24, 18))
        screen.blit(cs, (cx - cw//2, cy))

    # Mountain silhouettes (two layers for depth)
    pts1 = [(0, HORIZON_Y + 12)]
    for x in range(0, WIDTH + 40, 35):
        h = (math.sin(x * 0.0075) * 28 + math.sin(x * 0.014) * 14
             + math.sin(x * 0.003) * 38)
        pts1.append((x, HORIZON_Y + 12 - int(h) - 22))
    pts1.append((WIDTH, HORIZON_Y + 12))
    if len(pts1) > 2:
        pygame.draw.polygon(screen, C['mountain1'], pts1)

    pts2 = [(0, HORIZON_Y + 12)]
    for x in range(0, WIDTH + 30, 25):
        h = math.sin(x * 0.011 + 2) * 16 + math.sin(x * 0.005 + 1) * 22
        pts2.append((x, HORIZON_Y + 12 - int(h)))
    pts2.append((WIDTH, HORIZON_Y + 12))
    if len(pts2) > 2:
        pygame.draw.polygon(screen, C['mountain2'], pts2)


def draw_road(screen, ego_y: float, ego_x: float = 0.0):
    """Perspective road with shoulder, asphalt strips, and animated lane markings.

    ego_x (world metres) shifts the entire road laterally so that the lane
    markings stay geometrically consistent with world_to_screen().  When ego
    changes lanes the road visually slides to show the new lane position.
    """
    road_world_half = ROAD.num_lanes * ROAD.lane_width / 2.0   # 5.55 m
    # Lane boundaries at ±(lane_width/2) from road centre = ±1.85 m
    lane_div_world  = ROAD.lane_width / 2.0                     # 1.85 m
    num_strips = 65

    # Grass base coat
    pygame.draw.rect(screen, C['grass_far'],
                     (0, HORIZON_Y, WIDTH, ROAD_BOTTOM_Y - HORIZON_Y))

    for i in range(num_strips):
        t_top = i / num_strips
        t_bot = (i + 1) / num_strips
        y_top = int(HORIZON_Y + t_top * (ROAD_BOTTOM_Y - HORIZON_Y))
        y_bot = int(HORIZON_Y + t_bot * (ROAD_BOTTOM_Y - HORIZON_Y))
        w_top = get_road_w(y_top)
        w_bot = get_road_w(y_bot)

        # Pixel-per-metre scale at each depth row
        xs_top = w_top * 0.5 / road_world_half
        xs_bot = w_bot * 0.5 / road_world_half

        # Road visual centre on screen (shifts with ego_x so ego stays centred)
        rc_top = CENTER_X - ego_x * xs_top
        rc_bot = CENTER_X - ego_x * xs_bot

        hw_top = w_top * 0.5
        hw_bot = w_bot * 0.5

        # Shoulder strip (slightly wider than asphalt)
        pygame.draw.polygon(screen, C['road_edge'],
                            [(rc_top - hw_top * 1.14, y_top), (rc_top + hw_top * 1.14, y_top),
                             (rc_bot + hw_bot * 1.14, y_bot), (rc_bot - hw_bot * 1.14, y_bot)])

        # Alternating asphalt strips give subtle depth texture
        road_col = C['road'] if i % 2 == 0 else C['road_alt']
        pygame.draw.polygon(screen, road_col,
                            [(rc_top - hw_top, y_top), (rc_top + hw_top, y_top),
                             (rc_bot + hw_bot, y_bot), (rc_bot - hw_bot, y_bot)])

    # Outer edge lines
    for i in range(num_strips):
        t_top = i / num_strips
        t_bot = (i + 1) / num_strips
        y_top = int(HORIZON_Y + t_top * (ROAD_BOTTOM_Y - HORIZON_Y))
        y_bot = int(HORIZON_Y + t_bot * (ROAD_BOTTOM_Y - HORIZON_Y))
        w_top = get_road_w(y_top)
        w_bot = get_road_w(y_bot)
        xs_top = w_top * 0.5 / road_world_half
        xs_bot = w_bot * 0.5 / road_world_half
        rc_top = CENTER_X - ego_x * xs_top
        rc_bot = CENTER_X - ego_x * xs_bot
        hw_top = w_top * 0.5
        hw_bot = w_bot * 0.5
        alpha = max(70, int(240 * (1 - t_top * 0.65)))
        lw = max(1, int(3 * (1 - t_top * 0.75)))
        # Left edge: yellow solid line (oncoming-traffic side)
        pygame.draw.line(screen, (alpha, int(alpha * 0.82), 18),
                         (int(rc_top - hw_top), y_top), (int(rc_bot - hw_bot), y_bot), lw)
        # Right edge: white solid line
        pygame.draw.line(screen, (alpha, alpha, alpha),
                         (int(rc_top + hw_top), y_top), (int(rc_bot + hw_bot), y_bot), lw)

    # Animated dashed lane dividers
    dash_len, gap_len = 3.0, 5.0
    for i in range(num_strips):
        t = i / num_strips
        y_pos  = int(HORIZON_Y + t * (ROAD_BOTTOM_Y - HORIZON_Y))
        next_t = (i + 1) / num_strips
        next_y = int(HORIZON_Y + next_t * (ROAD_BOTTOM_Y - HORIZON_Y))
        w      = get_road_w(y_pos)
        next_w = get_road_w(next_y)

        world_dist = 5.0 / (t + 0.05)
        shifted = (world_dist + ego_y) % (dash_len + gap_len)
        if shifted >= dash_len:
            continue

        alpha = max(55, int(235 * (1 - t * 0.72)))
        lw    = max(1, int(2 * (1 - t * 0.7) + 1))

        xs_cur  = w      * 0.5 / road_world_half
        xs_next = next_w * 0.5 / road_world_half
        rc_cur  = CENTER_X - ego_x * xs_cur
        rc_next = CENTER_X - ego_x * xs_next

        # Dividers at ±1.85 m from road world-centre, shifted with ego
        for side in (-1, 1):
            x1 = int(rc_cur  + side * lane_div_world * xs_cur)
            x2 = int(rc_next + side * lane_div_world * xs_next)
            pygame.draw.line(screen, (alpha, alpha, int(alpha * 0.9)),
                             (x1, y_pos), (x2, next_y), lw)


# ═══════════════════════════════════════════════════════════════
#  OBSTACLE DRAWING
# ═══════════════════════════════════════════════════════════════

def draw_obstacle(screen, obs, ego_x, ego_y, font_sm):
    """Draw obstacle sprite/polygon with depth-based brightness and warning effects."""
    rel_x = obs.get('x', 0) - ego_x
    rel_y = obs.get('y', 0) - ego_y

    if rel_y < -5 or rel_y > 200:
        return

    obs_type = obs.get('type', 'car')
    color    = tuple(obs.get('color', [200, 50, 50]))
    w = obs.get('width', 1.8)
    h = obs.get('height', 1.5)

    sx, sy, sw, sh = world_to_screen(rel_x, rel_y, w, h)

    if sy < HORIZON_Y - 20 or sy > ROAD_BOTTOM_Y or sw < 4:
        return

    # Depth-based brightness multiplier
    brightness = max(0.45, min(1.0, 1.0 - rel_y / 220.0))

    # Select sprite
    sprite_map = {
        'motorcycle': 'motorcycle', 'bicycle': 'bicycle',
        'pedestrian': 'pedestrian', 'child': 'child',
        'ball': 'ball', 'fallen_tree': 'fallen_tree', 'animal': 'animal',
    }
    sprite_name = sprite_map.get(obs_type, pick_car_sprite(color))
    drawn = draw_sprite(screen, sprite_name, sx, sy, sw, sh)

    if not drawn:
        # Polygon fallback with depth-tinted colour
        bc = tuple(int(c * brightness) for c in color)
        body = pygame.Rect(sx - sw//2, sy - sh, sw, sh)
        pygame.draw.rect(screen, bc, body, border_radius=max(1, sw//8))
        roof_c = tuple(max(0, int(c * brightness * 0.7)) for c in color)
        roof = pygame.Rect(sx - sw//3, sy - sh + sh//5, sw*2//3, sh//3)
        pygame.draw.rect(screen, roof_c, roof, border_radius=max(1, sw//10))

    is_danger_obs = not obs.get('is_traffic', True)

    # Label tag
    label_map = {
        'car': 'CAR', 'motorcycle': 'MOTO', 'bicycle': 'BIKE',
        'pedestrian': 'PED', 'child': 'CHILD!',
        'ball': 'BALL', 'fallen_tree': 'TREE', 'animal': 'ANIMAL'
    }
    label = label_map.get(obs_type, obs_type.upper())
    if sw > 16:
        tag_col = (255, 215, 40) if is_danger_obs else (190, 200, 220)
        lbl = font_sm.render(label, True, tag_col)
        bg  = pygame.Surface((lbl.get_width() + 8, lbl.get_height() + 4), pygame.SRCALPHA)
        pygame.draw.rect(bg, (0, 0, 0, 170), bg.get_rect(), border_radius=3)
        screen.blit(bg,  (sx - lbl.get_width()//2 - 4, sy - sh - 17))
        screen.blit(lbl, (sx - lbl.get_width()//2,     sy - sh - 15))

    # Target bracket around nearest danger obstacle
    if is_danger_obs and rel_y < 40 and sw > 14:
        bx, by = sx - sw//2 - 4, sy - sh - 4
        bw, bh = sw + 8, sh + 8
        arm = max(5, sw // 3)
        col = C['target_box']
        # Four corner L-brackets
        for (ox, oy, dx, dy) in [
            (bx,    by,     1,  1),
            (bx+bw, by,    -1,  1),
            (bx,    by+bh,  1, -1),
            (bx+bw, by+bh, -1, -1),
        ]:
            pygame.draw.line(screen, col, (ox, oy), (ox + dx*arm, oy), 2)
            pygame.draw.line(screen, col, (ox, oy), (ox, oy + dy*arm), 2)


def draw_ego_vehicle(screen, status_color):
    """Draw ego vehicle at road bottom with headlight beams."""
    sx, sy, sw, sh = CENTER_X, ROAD_BOTTOM_Y + 15, 110, 170

    drawn = draw_sprite(screen, 'ego', sx, sy, sw, sh)
    if not drawn:
        body = pygame.Rect(sx - 55, sy - sh, sw, sh)
        pygame.draw.rect(screen, (35, 110, 240), body, border_radius=10)
        roof = pygame.Rect(sx - 35, sy - sh + 28, 70, sh//3)
        pygame.draw.rect(screen, (20, 75, 190), roof, border_radius=6)
        win  = pygame.Rect(sx - 30, sy - sh + 33, 60, sh//4)
        pygame.draw.rect(screen, (130, 195, 230), win, border_radius=4)

    # Status indicator light bar on roof
    bar_surf = pygame.Surface((sw, 5), pygame.SRCALPHA)
    pygame.draw.rect(bar_surf, (*status_color, 200), (0, 0, sw, 5), border_radius=2)
    screen.blit(bar_surf, (sx - sw//2, sy - sh + 2))

    # Headlight cones (subtle transparency)
    beam = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
    beam_pts_l = [(sx - 28, sy - sh + 12),
                  (sx - 220, ROAD_BOTTOM_Y - 280), (sx - 70, ROAD_BOTTOM_Y - 280)]
    beam_pts_r = [(sx + 28, sy - sh + 12),
                  (sx + 70, ROAD_BOTTOM_Y - 280), (sx + 220, ROAD_BOTTOM_Y - 280)]
    pygame.draw.polygon(beam, (255, 255, 170, 10), beam_pts_l)
    pygame.draw.polygon(beam, (255, 255, 170, 10), beam_pts_r)
    screen.blit(beam, (0, 0))

    # "EGO" label
    try:
        f = pygame.font.SysFont('consolas', 13, bold=True)
        lbl = f.render("EGO", True, (220, 235, 255))
        screen.blit(lbl, (sx - lbl.get_width()//2, sy - sh - 18))
    except Exception:
        pass


# ═══════════════════════════════════════════════════════════════
#  LIDAR DISPLAY  (top-left, forward-facing fan)
# ═══════════════════════════════════════════════════════════════

def draw_lidar_panel(screen, lidar_points, status, font_sm):
    """Futuristic forward-facing LIDAR fan — top left corner."""
    panel_w  = 230
    panel_h  = 215
    px, py   = 12, 12
    cx       = px + panel_w // 2
    cy       = py + panel_h - 25     # sweep origin at bottom of fan
    fan_r    = panel_h - 45
    half_fov = 60

    # Panel background + border
    bg = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
    pygame.draw.rect(bg, (7, 10, 20, 220), (0, 0, panel_w, panel_h), border_radius=8)
    screen.blit(bg, (px, py))
    pygame.draw.rect(screen, C['lidar_grid'], (px, py, panel_w, panel_h), 1, border_radius=8)

    # Title
    title = font_sm.render("◈  LIDAR  SENSOR", True, C['text_bright'])
    screen.blit(title, (px + panel_w//2 - title.get_width()//2, py + 5))

    # Fan fill (very dark tint)
    fan_pts = [(cx, cy)]
    for a in range(-half_fov, half_fov + 1, 2):
        rad = math.radians(a)
        fan_pts.append((cx + int(fan_r * math.sin(rad)),
                        cy - int(fan_r * math.cos(rad))))
    fan_pts.append((cx, cy))
    if len(fan_pts) > 2:
        fan_surf = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
        local = [(x - px, y - py) for x, y in fan_pts]
        pygame.draw.polygon(fan_surf, (0, 50, 20, 55), local)
        screen.blit(fan_surf, (px, py))

    # Distance arcs
    for ring_pct, dist_m in [(0.33, 50), (0.66, 100), (1.0, 150)]:
        r = int(fan_r * ring_pct)
        arc_pts = []
        for a in range(-half_fov, half_fov + 1, 3):
            rad = math.radians(a)
            arc_pts.append((cx + int(r * math.sin(rad)),
                            cy - int(r * math.cos(rad))))
        if len(arc_pts) > 1:
            pygame.draw.lines(screen, C['lidar_grid'], False, arc_pts, 1)
        # Distance label at right edge
        label_rad = math.radians(half_fov)
        lx = cx + int(r * math.sin(label_rad)) + 2
        ly = cy - int(r * math.cos(label_rad)) - 7
        dl = font_sm.render(f"{dist_m}m", True, C['text_dim'])
        screen.blit(dl, (lx, ly))

    # Radial lines (angle guides every 20°)
    for a in range(-half_fov, half_fov + 1, 20):
        rad = math.radians(a)
        ex = cx + int(fan_r * math.sin(rad))
        ey = cy - int(fan_r * math.cos(rad))
        pygame.draw.line(screen, C['lidar_grid'], (cx, cy), (ex, ey), 1)

    # LIDAR return points
    pt_count = 0
    for pt in lidar_points:
        angle = pt.get('angle', 0)
        dist  = pt.get('distance', 0)
        if dist <= 0 or dist > 150 or abs(angle) > half_fov:
            continue
        pt_count += 1
        norm = min(1.0, dist / 150.0)
        r    = int(norm * fan_r)
        rad  = math.radians(angle)
        px2  = cx + int(r * math.sin(rad))
        py2  = cy - int(r * math.cos(rad))

        if dist < 15:
            col, size = C['lidar_pt_crit'], 4
        elif dist < 35:
            col, size = C['lidar_pt_warn'], 3
        else:
            col, size = C['lidar_pt_safe'], 2
        pygame.draw.circle(screen, col, (px2, py2), size)

        # Glow for critical returns
        if dist < 15:
            g = pygame.Surface((size*8, size*8), pygame.SRCALPHA)
            pygame.draw.circle(g, (*col, 55), (size*4, size*4), size*4)
            screen.blit(g, (px2 - size*4, py2 - size*4))

    # Animated sweep line with fade trail
    sweep_t  = (time.time() % 2.0) / 2.0
    sweep_a  = -half_fov + sweep_t * half_fov * 2
    sweep_r  = math.radians(sweep_a)
    sweep_ex = cx + int(fan_r * math.sin(sweep_r))
    sweep_ey = cy - int(fan_r * math.cos(sweep_r))
    pygame.draw.line(screen, C['lidar_sweep'], (cx, cy), (sweep_ex, sweep_ey), 1)

    # Trail behind sweep
    trail_surf = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
    for trail_steps in range(1, 5):
        ta   = sweep_a - trail_steps * 4
        ta_r = math.radians(ta)
        tex  = cx + int(fan_r * math.sin(ta_r))
        tey  = cy - int(fan_r * math.cos(ta_r))
        alpha = 80 - trail_steps * 16
        if alpha > 0:
            pygame.draw.line(trail_surf, (0, 200, 80, alpha),
                             (cx - px, cy - py), (tex - px, tey - py), 1)
    screen.blit(trail_surf, (px, py))

    # FOV boundary lines
    for a in (-half_fov, half_fov):
        rad = math.radians(a)
        bx  = cx + int(fan_r * math.sin(rad))
        by  = cy - int(fan_r * math.cos(rad))
        pygame.draw.line(screen, (40, 70, 55), (cx, cy), (bx, by), 1)

    # Ego dot
    pygame.draw.circle(screen, C['lidar_ego'], (cx, cy), 5)
    pygame.draw.circle(screen, (255, 255, 255), (cx, cy), 2)

    # Point count
    count_lbl = font_sm.render(f"Returns: {pt_count}", True, C['text_mid'])
    screen.blit(count_lbl, (px + panel_w//2 - count_lbl.get_width()//2, py + panel_h - 18))

    return pt_count   # returned for dashboard display


# ═══════════════════════════════════════════════════════════════
#  BOTTOM DASHBOARD  (professional 5-column layout)
# ═══════════════════════════════════════════════════════════════

def draw_dashboard(screen, ego, aeb, scenario, decision_text, stats,
                   lidar_count, font_lg, font_md, font_sm, font_xs):
    """
    Full-width dashboard at bottom of screen.
    Five columns: Speed | Brake | ADAS Status | Lane/Scenario | Decision/Stats
    """
    dy = HEIGHT - DASH_HEIGHT   # top of dashboard (618)
    dw = WIDTH                  # 1400

    # ── Background & header bar ──────────────────────────────
    dash_surf = pygame.Surface((dw, DASH_HEIGHT), pygame.SRCALPHA)
    pygame.draw.rect(dash_surf, (8, 11, 22, 245), (0, 0, dw, DASH_HEIGHT))
    screen.blit(dash_surf, (0, dy))

    status      = aeb.get('status', 'SAFE')
    status_col  = STATUS_COLORS.get(status, C['safe'])

    # Coloured top separator line
    pygame.draw.line(screen, status_col, (0, dy), (dw, dy), 2)

    # Header strip
    hdr_surf = pygame.Surface((dw, 20), pygame.SRCALPHA)
    pygame.draw.rect(hdr_surf, (14, 18, 35, 230), (0, 0, dw, 20))
    screen.blit(hdr_surf, (0, dy + 2))

    hdr_text = font_xs.render(
        "ADAS CONTROL DASHBOARD  ─  AUTONOMOUS EMERGENCY BRAKING SYSTEM  ─  ROS2 HUMBLE",
        True, C['text_dim'])
    screen.blit(hdr_text, (dw//2 - hdr_text.get_width()//2, dy + 5))

    content_y = dy + 24   # content starts here
    row_h = DASH_HEIGHT - 28

    # Column widths & x positions
    cols = [0, 195, 390, 680, 990, dw]
    cx = [cols[i] + (cols[i+1] - cols[i])//2 for i in range(5)]

    # Vertical dividers
    for x in cols[1:-1]:
        pygame.draw.line(screen, C['dash_divider'],
                         (x, dy + 22), (x, HEIGHT - 2), 1)

    def section_label(label, col_idx):
        lbl = font_xs.render(label, True, C['text_label'])
        screen.blit(lbl, (cx[col_idx] - lbl.get_width()//2, content_y))

    # ── Column 0: SPEED ──────────────────────────────────────
    section_label("SPEED", 0)
    speed = ego.get('speed_kmh', 0.0)
    spd_str = f"{int(speed)}"
    spd_surf = font_lg.render(spd_str, True, C['text_bright'])
    screen.blit(spd_surf, (cx[0] - spd_surf.get_width()//2, content_y + 14))
    unit = font_xs.render("km/h", True, C['text_dim'])
    screen.blit(unit, (cx[0] - unit.get_width()//2, content_y + 72))

    # Speed bar
    speed_pct = min(1.0, speed / PHYSICS.max_speed_kmh)
    bar_col = (C['safe'] if speed_pct < 0.65
               else C['warning'] if speed_pct < 0.88 else C['danger'])
    bx, bw2, bh2 = cols[0] + 18, cols[1] - cols[0] - 36, 7
    pygame.draw.rect(screen, C['dash_border'], (bx, content_y + 88, bw2, bh2), border_radius=3)
    if speed_pct > 0:
        pygame.draw.rect(screen, bar_col,
                         (bx, content_y + 88, int(bw2 * speed_pct), bh2), border_radius=3)

    max_lbl = font_xs.render(f"MAX {int(PHYSICS.max_speed_kmh)} km/h", True, C['text_dim'])
    screen.blit(max_lbl, (cx[0] - max_lbl.get_width()//2, content_y + 100))

    # ── Column 1: BRAKE ──────────────────────────────────────
    section_label("BRAKE", 1)
    brake = aeb.get('brake_intensity', 0.0)
    brake_pct = int(brake * 100)
    brake_col = (C['danger'] if brake > 0.55
                 else C['warning'] if brake > 0.15 else C['safe'])
    bp_surf = font_lg.render(f"{brake_pct}%", True, brake_col)
    screen.blit(bp_surf, (cx[1] - bp_surf.get_width()//2, content_y + 14))

    # Vertical brake bar
    vbx  = cx[1] - 10
    vbh  = 65
    vby  = content_y + 75
    pygame.draw.rect(screen, C['dash_border'], (vbx, vby, 20, vbh), border_radius=4)
    if brake > 0:
        fill_h = int(vbh * brake)
        pygame.draw.rect(screen, brake_col,
                         (vbx, vby + vbh - fill_h, 20, fill_h), border_radius=4)

    # Decel label
    decel_ms = PHYSICS.comfort_deceleration + (PHYSICS.max_deceleration - PHYSICS.comfort_deceleration) * brake
    dcl = font_xs.render(f"{decel_ms:.1f} m/s²", True, C['text_dim'])
    screen.blit(dcl, (cx[1] - dcl.get_width()//2, content_y + 145))

    # ── Column 2: ADAS STATUS ────────────────────────────────
    section_label("ADAS STATUS", 2)

    # Pulsing dot for active status
    pulse = 0.5 + 0.5 * math.sin(time.time() * 5)
    dot_alpha = int(180 + 75 * pulse)
    dot_surf = pygame.Surface((14, 14), pygame.SRCALPHA)
    pygame.draw.circle(dot_surf, (*status_col, dot_alpha), (7, 7), 7)
    screen.blit(dot_surf, (cx[2] - 70, content_y + 20))

    st_surf = font_md.render(status, True, status_col)
    screen.blit(st_surf, (cx[2] - 55, content_y + 14))

    dist = aeb.get('nearest_distance', 999.0)
    dist_str = f"{dist:.1f} m" if dist < 500 else "— — —"
    screen.blit(font_xs.render("DISTANCE", True, C['text_label']), (cx[2] - 88, content_y + 46))
    dist_surf = font_sm.render(dist_str, True, C['text_bright'])
    screen.blit(dist_surf, (cx[2] - 88, content_y + 60))

    ttc = aeb.get('ttc', -1)
    ttc_str = f"{ttc:.1f} s" if ttc > 0 else "— — —"
    ttc_col = (C['safe'] if ttc < 0 or ttc > 3
               else C['warning'] if ttc > 1.5 else C['danger'])
    screen.blit(font_xs.render("TTC", True, C['text_label']), (cx[2] + 10, content_y + 46))
    screen.blit(font_sm.render(ttc_str, True, ttc_col), (cx[2] + 10, content_y + 60))

    det = aeb.get('nearest_type', '')
    screen.blit(font_xs.render("OBJECT", True, C['text_label']), (cx[2] - 88, content_y + 90))
    obj_surf = font_sm.render(det.upper() if det else "NONE", True,
                               C['warning'] if det else C['text_dim'])
    screen.blit(obj_surf, (cx[2] - 88, content_y + 104))

    # LIDAR point count
    screen.blit(font_xs.render("LIDAR PTS", True, C['text_label']), (cx[2] + 10, content_y + 90))
    screen.blit(font_sm.render(str(lidar_count), True, C['lidar_sweep']),
                (cx[2] + 10, content_y + 104))

    # ── Column 3: LANE / SCENARIO ────────────────────────────
    section_label("LANE  &  SCENARIO", 3)

    lane_name = ego.get('lane_name', 'CENTER')
    screen.blit(font_xs.render("CURRENT LANE", True, C['text_label']), (cx[3] - 90, content_y + 14))
    lane_surf = font_md.render(lane_name, True, C['lidar_ego'])
    screen.blit(lane_surf, (cx[3] - 90, content_y + 28))

    is_lc = aeb.get('is_changing_lane', False)
    lc_to  = aeb.get('lane_change_to', '')
    lc_prog = aeb.get('lane_change_progress', 0.0)

    screen.blit(font_xs.render("TARGET", True, C['text_label']), (cx[3] + 15, content_y + 14))
    if is_lc:
        tgt_surf = font_md.render(lc_to, True, C['lane_change'])
        screen.blit(tgt_surf, (cx[3] + 15, content_y + 28))
        prog_lbl = font_xs.render(f"{int(lc_prog*100)}%", True, C['lane_change'])
        screen.blit(prog_lbl, (cx[3] + 15, content_y + 54))
        # Lane change progress bar
        pb_x, pb_w = cx[3] - 90, 250
        pygame.draw.rect(screen, C['dash_border'], (pb_x, content_y + 70, pb_w, 6), border_radius=3)
        pygame.draw.rect(screen, C['lane_change'],
                         (pb_x, content_y + 70, int(pb_w * lc_prog), 6), border_radius=3)
    else:
        screen.blit(font_md.render("—", True, C['text_dim']), (cx[3] + 15, content_y + 28))

    # Lane diagram (L / C / R boxes)
    lbox_y   = content_y + 82
    lbox_w   = 46
    lbox_gap = 4
    lane_val = ego.get('lane', 1)
    change_to_val = {'LEFT': 0, 'CENTER': 1, 'RIGHT': 2}.get(lc_to, -1)
    for i, lbl in enumerate(['L', 'C', 'R']):
        lbx = cx[3] - 78 + i * (lbox_w + lbox_gap)
        lby = lbox_y
        if i == lane_val:
            pygame.draw.rect(screen, C['lidar_ego'], (lbx, lby, lbox_w, 28), border_radius=4)
        elif is_lc and i == change_to_val and int(time.time() * 4) % 2:
            pygame.draw.rect(screen, C['lane_change'], (lbx, lby, lbox_w, 28), border_radius=4)
        else:
            pygame.draw.rect(screen, C['dash_border'], (lbx, lby, lbox_w, 28), border_radius=4)
        lt = font_sm.render(lbl, True,
                            C['text_bright'] if i == lane_val else C['text_dim'])
        screen.blit(lt, (lbx + lbox_w//2 - lt.get_width()//2, lby + 6))

    # Active scenario
    scen_name = scenario.get('name', '')
    screen.blit(font_xs.render("ACTIVE SCENARIO", True, C['text_label']),
                (cx[3] - 90, content_y + 120))
    if scen_name:
        sc_col = C['warning']
        # Blink scenario name while active
        if int(time.time() * 2) % 2 == 0:
            sc_col = (min(255, C['warning'][0] + 30), C['warning'][1], C['warning'][2])
        sc_surf = font_sm.render(scen_name, True, sc_col)
        screen.blit(sc_surf, (cx[3] - 90, content_y + 134))
    else:
        screen.blit(font_sm.render("None", True, C['text_dim']),
                    (cx[3] - 90, content_y + 134))

    # ── Column 4: DECISION / STATS ───────────────────────────
    section_label("DECISION  &  INFO", 4)

    # Decision text (may wrap into 2 lines)
    max_chars = 38
    line1 = decision_text[:max_chars]
    line2 = decision_text[max_chars:] if len(decision_text) > max_chars else ''
    dec_col = (status_col if status not in ('SAFE', 'CAUTION') else C['text_bright'])
    dc_surf1 = font_sm.render(line1, True, dec_col)
    screen.blit(dc_surf1, (cols[4] + 12, content_y + 14))
    if line2:
        dc_surf2 = font_sm.render(line2, True, dec_col)
        screen.blit(dc_surf2, (cols[4] + 12, content_y + 32))

    # Stats grid
    sy_start = content_y + 58
    stat_rows = [
        ("Distance",    f"{int(stats.get('total_distance', 0))} m"),
        ("Uptime",      f"{int(stats.get('uptime', 0))} s"),
        ("Brk Events",  f"{stats.get('braking_events', 0)}"),
        ("Lane Chg",    f"{stats.get('lane_changes', 0)}"),
    ]
    for row_idx, (label, value) in enumerate(stat_rows):
        rx = cols[4] + 12 + (row_idx % 2) * 185
        ry = sy_start + (row_idx // 2) * 36
        screen.blit(font_xs.render(label.upper(), True, C['text_label']), (rx, ry))
        screen.blit(font_sm.render(value, True, C['text_bright']), (rx, ry + 13))

    # ── Scenario timer progress bar ──────────────────────────
    if scen_name:
        phase_t = scenario.get('timer', 0.0)
        pb2_y   = HEIGHT - 8
        pb2_x   = cols[4] + 12
        pb2_w   = dw - cols[4] - 24
        pygame.draw.rect(screen, C['dash_border'], (pb2_x, pb2_y - 4, pb2_w, 5), border_radius=2)
        fill_w = int(pb2_w * min(1.0, phase_t / 10.0))
        if fill_w > 0:
            pygame.draw.rect(screen, status_col,
                             (pb2_x, pb2_y - 4, fill_w, 5), border_radius=2)


# ═══════════════════════════════════════════════════════════════
#  TOP BAR
# ═══════════════════════════════════════════════════════════════

def draw_top_bar(screen, font_xs):
    bar = pygame.Surface((WIDTH, 26), pygame.SRCALPHA)
    pygame.draw.rect(bar, (7, 9, 18, 210), (0, 0, WIDTH, 26))
    screen.blit(bar, (0, 0))
    left = font_xs.render(
        "AEB SYSTEM  —  Autonomous Emergency Braking  —  ROS2 Humble  |  Q to exit",
        True, C['text_dim'])
    right = font_xs.render("ADAS FINAL PROJECT", True, C['text_dim'])
    screen.blit(left,  (10, 6))
    screen.blit(right, (WIDTH - right.get_width() - 10, 6))


# ═══════════════════════════════════════════════════════════════
#  EMERGENCY OVERLAY
# ═══════════════════════════════════════════════════════════════

def draw_emergency_overlay(screen, status, distance, font_md):
    """Flashing border and distance alert for DANGER / EMERGENCY."""
    if status not in ('DANGER', 'EMERGENCY'):
        return

    if int(time.time() * 3) % 2:
        col   = C['emergency'] if status == 'EMERGENCY' else C['danger']
        thick = 4 if status == 'EMERGENCY' else 3
        pygame.draw.rect(screen, col, (0, 0, WIDTH, HEIGHT - DASH_HEIGHT), thick)

    if distance < 50:
        warn = font_md.render(f"⚠  {distance:.0f} m", True, C['emergency'])
        # Semi-transparent background
        bg = pygame.Surface((warn.get_width() + 24, warn.get_height() + 10), pygame.SRCALPHA)
        pygame.draw.rect(bg, (30, 0, 0, 180), bg.get_rect(), border_radius=6)
        bx = CENTER_X - (warn.get_width() + 24) // 2
        by = HORIZON_Y + 70
        screen.blit(bg, (bx, by))
        screen.blit(warn, (bx + 12, by + 5))


# ═══════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = AEBVisualizer()

    thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    thread.start()

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("AEB System — Autonomous Emergency Braking | ROS2")
    clock = pygame.time.Clock()

    load_sprites()

    try:
        font_lg = pygame.font.SysFont('consolas', 46, bold=True)
        font_md = pygame.font.SysFont('consolas', 20, bold=True)
        font_sm = pygame.font.SysFont('consolas', 14)
        font_xs = pygame.font.SysFont('consolas', 11)
    except Exception:
        font_lg = pygame.font.SysFont('arial', 46, bold=True)
        font_md = pygame.font.SysFont('arial', 20, bold=True)
        font_sm = pygame.font.SysFont('arial', 14)
        font_xs = pygame.font.SysFont('arial', 11)

    # Pre-generate static scene elements
    stars  = [(random.randint(0, WIDTH), random.randint(0, HORIZON_Y - 30),
               random.uniform(0.25, 1.0)) for _ in range(140)]
    clouds = [(random.randint(100, WIDTH - 100),
               random.randint(10, HORIZON_Y - 60),
               random.randint(80, 200),
               random.randint(12, 28)) for _ in range(6)]

    running = True
    lidar_count = 0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False

        ego           = node.ego
        aeb           = node.aeb
        scenario      = node.scenario
        decision_text = node.decision_text
        obstacles     = node.obstacles
        lidar_pts     = node.lidar_points
        stats         = node.stats

        ego_y    = ego.get('y', 0)
        ego_x    = ego.get('x', 0)
        status   = aeb.get('status', 'SAFE')
        status_col = STATUS_COLORS.get(status, C['safe'])

        # ─── RENDER ────────────────────────────────────────────

        screen.fill(C['sky_zenith'])

        draw_sky(screen, stars, clouds)
        draw_road(screen, ego_y, ego_x)

        # Draw obstacles sorted far→near for correct depth ordering
        sorted_obs = sorted(obstacles,
                            key=lambda o: o.get('y', 0) - ego_y,
                            reverse=True)
        for obs in sorted_obs:
            draw_obstacle(screen, obs, ego_x, ego_y, font_sm)

        draw_ego_vehicle(screen, status_col)

        draw_emergency_overlay(screen, status, aeb.get('nearest_distance', 999), font_md)

        # LIDAR panel (top-left) — returns point count for dashboard
        lidar_count = draw_lidar_panel(screen, lidar_pts, status, font_sm)

        draw_top_bar(screen, font_xs)

        draw_dashboard(screen, ego, aeb, scenario, decision_text, stats,
                       lidar_count, font_lg, font_md, font_sm, font_xs)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
