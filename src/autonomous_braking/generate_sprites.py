#!/usr/bin/env python3
"""
AEB System - Sprite Generator
Run once to generate all vehicle sprites for the visualizer.
Output: sprites/ folder with PNG files for each obstacle type.
"""

from PIL import Image, ImageDraw, ImageFilter, ImageEnhance
import os
import math

OUTPUT_DIR = "sprites"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Sprite canvas size (will be scaled by perspective in visualizer) ─────────
W, H = 96, 160   # width x height in pixels (top-down, car facing up)

def make_transparent(size=(W, H)):
    return Image.new("RGBA", size, (0, 0, 0, 0))

def add_shadow(img: Image.Image, offset=(4, 6), blur=8, opacity=120) -> Image.Image:
    """Add drop shadow beneath vehicle."""
    shadow = Image.new("RGBA", img.size, (0, 0, 0, 0))
    mask = img.split()[3]  # alpha channel
    shadow_layer = Image.new("RGBA", img.size, (0, 0, 0, opacity))
    shadow.paste(shadow_layer, mask=mask)
    shadow = shadow.filter(ImageFilter.GaussianBlur(blur))
    result = Image.new("RGBA", img.size, (0, 0, 0, 0))
    result.paste(shadow, (offset[0], offset[1]), shadow)
    result.paste(img, (0, 0), img)
    return result


# ═══════════════════════════════════════════════════════
#  CAR (SEDAN)
# ═══════════════════════════════════════════════════════

def draw_car(color=(200, 50, 50), roof_color=None, name="car"):
    img = make_transparent()
    d = ImageDraw.Draw(img)
    r = color
    if roof_color is None:
        roof_color = tuple(max(0, c - 40) for c in color)

    # ── Body (rounded rect) ──────────────────────────────
    body_rect = [8, 16, W - 8, H - 16]
    d.rounded_rectangle(body_rect, radius=12, fill=(*r, 255))

    # ── Hood (front, bottom of image = front facing viewer) ──
    hood = [12, H - 50, W - 12, H - 16]
    hood_color = tuple(min(255, c + 20) for c in color)
    d.rounded_rectangle(hood, radius=8, fill=(*hood_color, 255))

    # ── Roof / cabin ─────────────────────────────────────
    roof = [16, 42, W - 16, H - 52]
    d.rounded_rectangle(roof, radius=8, fill=(*roof_color, 255))

    # ── Windshield (front glass) ─────────────────────────
    ws_front = [19, H - 54, W - 19, H - 38]
    d.rounded_rectangle(ws_front, radius=4, fill=(120, 190, 220, 200))
    # reflection line
    d.line([(22, H - 52), (W - 22, H - 40)], fill=(200, 230, 255, 120), width=2)

    # ── Rear windshield ──────────────────────────────────
    ws_rear = [19, 38, W - 19, 54]
    d.rounded_rectangle(ws_rear, radius=4, fill=(100, 160, 200, 180))

    # ── Side windows ─────────────────────────────────────
    d.rectangle([9, 58, 20, 90], fill=(110, 175, 215, 180))
    d.rectangle([W - 20, 58, W - 9, 90], fill=(110, 175, 215, 180))

    # ── Wheels (4 wheels) ────────────────────────────────
    wheel_color = (30, 30, 30, 255)
    rim_color = (180, 180, 185, 255)
    # Front-left, Front-right, Rear-left, Rear-right
    wheels = [(5, H - 46, 20, H - 20),
              (W - 20, H - 46, W - 5, H - 20),
              (5, 20, 20, 46),
              (W - 20, 20, W - 5, 46)]
    for wx1, wy1, wx2, wy2 in wheels:
        d.ellipse([wx1, wy1, wx2, wy2], fill=wheel_color)
        cx = (wx1 + wx2) // 2
        cy = (wy1 + wy2) // 2
        r2 = (wx2 - wx1) // 4
        d.ellipse([cx - r2, cy - r2, cx + r2, cy + r2], fill=rim_color)

    # ── Headlights ───────────────────────────────────────
    d.ellipse([13, H - 30, 27, H - 18], fill=(255, 250, 200, 255))
    d.ellipse([W - 27, H - 30, W - 13, H - 18], fill=(255, 250, 200, 255))
    # inner glow
    d.ellipse([16, H - 28, 24, H - 20], fill=(255, 255, 240, 255))
    d.ellipse([W - 24, H - 28, W - 16, H - 20], fill=(255, 255, 240, 255))

    # ── Taillights ───────────────────────────────────────
    d.ellipse([13, 18, 27, 30], fill=(200, 30, 30, 255))
    d.ellipse([W - 27, 18, W - 13, 30], fill=(200, 30, 30, 255))

    # ── Body highlight ────────────────────────────────────
    d.line([(W // 2, 20), (W // 2, H - 20)], fill=(255, 255, 255, 40), width=3)

    img = add_shadow(img)
    img.save(f"{OUTPUT_DIR}/{name}.png")
    print(f"  ✓ {name}.png")


# ═══════════════════════════════════════════════════════
#  EGO VEHICLE (player's blue car, slightly larger)
# ═══════════════════════════════════════════════════════

def draw_ego():
    W2, H2 = 100, 168
    img = Image.new("RGBA", (W2, H2), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)

    # Body
    d.rounded_rectangle([6, 14, W2 - 6, H2 - 14], radius=14, fill=(30, 100, 220, 255))
    # Hood
    d.rounded_rectangle([10, H2 - 52, W2 - 10, H2 - 14], radius=10,
                         fill=(50, 120, 240, 255))
    # Roof
    d.rounded_rectangle([14, 40, W2 - 14, H2 - 54], radius=10,
                         fill=(20, 70, 180, 255))
    # Front windshield
    d.rounded_rectangle([17, H2 - 56, W2 - 17, H2 - 38], radius=5,
                         fill=(140, 200, 230, 200))
    d.line([(20, H2 - 54), (W2 - 20, H2 - 40)], fill=(200, 230, 255, 130), width=2)
    # Rear glass
    d.rounded_rectangle([17, 38, W2 - 17, 54], radius=5, fill=(110, 175, 215, 180))
    # Side windows
    d.rectangle([7, 56, 18, 92], fill=(120, 185, 220, 180))
    d.rectangle([W2 - 18, 56, W2 - 7, 92], fill=(120, 185, 220, 180))
    # Wheels
    wc = (25, 25, 25, 255)
    rc = (190, 190, 195, 255)
    for wx1, wy1, wx2, wy2 in [(3, H2 - 50, 18, H2 - 20),
                                 (W2 - 18, H2 - 50, W2 - 3, H2 - 20),
                                 (3, 20, 18, 50),
                                 (W2 - 18, 20, W2 - 3, 50)]:
        d.ellipse([wx1, wy1, wx2, wy2], fill=wc)
        cx, cy = (wx1 + wx2) // 2, (wy1 + wy2) // 2
        r2 = (wx2 - wx1) // 4
        d.ellipse([cx - r2, cy - r2, cx + r2, cy + r2], fill=rc)
    # Headlights (white/blue)
    d.ellipse([12, H2 - 32, 26, H2 - 18], fill=(220, 240, 255, 255))
    d.ellipse([W2 - 26, H2 - 32, W2 - 12, H2 - 18], fill=(220, 240, 255, 255))
    # Taillights
    d.ellipse([12, 18, 26, 30], fill=(200, 30, 30, 255))
    d.ellipse([W2 - 26, 18, W2 - 12, 30], fill=(200, 30, 30, 255))
    # Center stripe
    d.line([(W2 // 2, 18), (W2 // 2, H2 - 18)], fill=(255, 255, 255, 50), width=4)
    # Roof light bar
    d.rounded_rectangle([W2 // 2 - 14, 44, W2 // 2 + 14, 52], radius=3,
                          fill=(200, 50, 50, 220))

    img = add_shadow(img)
    img.save(f"{OUTPUT_DIR}/ego.png")
    print("  ✓ ego.png")


# ═══════════════════════════════════════════════════════
#  MOTORCYCLE
# ═══════════════════════════════════════════════════════

def draw_motorcycle():
    mw, mh = 40, 100
    img = Image.new("RGBA", (mw, mh), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    # Body
    d.rounded_rectangle([10, 15, mw - 10, mh - 15], radius=6, fill=(80, 80, 200, 255))
    # Front wheel
    d.ellipse([6, mh - 28, mw - 6, mh - 6], fill=(25, 25, 25, 255))
    d.ellipse([12, mh - 24, mw - 12, mh - 10], fill=(160, 160, 165, 255))
    # Rear wheel
    d.ellipse([6, 6, mw - 6, 28], fill=(25, 25, 25, 255))
    d.ellipse([12, 10, mw - 12, 24], fill=(160, 160, 165, 255))
    # Rider helmet
    d.ellipse([mw // 2 - 7, mh // 2 - 8, mw // 2 + 7, mh // 2 + 8],
              fill=(40, 40, 40, 255))
    d.ellipse([mw // 2 - 4, mh // 2 - 3, mw // 2 + 4, mh // 2 + 3],
              fill=(120, 180, 220, 180))
    # Headlight
    d.ellipse([mw // 2 - 5, mh - 20, mw // 2 + 5, mh - 12], fill=(255, 250, 200, 255))
    img = add_shadow(img, offset=(2, 3), blur=5)
    img.save(f"{OUTPUT_DIR}/motorcycle.png")
    print("  ✓ motorcycle.png")


# ═══════════════════════════════════════════════════════
#  BICYCLE
# ═══════════════════════════════════════════════════════

def draw_bicycle():
    bw, bh = 32, 80
    img = Image.new("RGBA", (bw, bh), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    # Wheels
    d.ellipse([2, bh - 22, bw - 2, bh - 2], fill=(30, 30, 30, 255))
    d.ellipse([2, 2, bw - 2, 22], fill=(30, 30, 30, 255))
    # Frame
    cx = bw // 2
    d.line([(cx, bh - 12), (cx - 4, bh // 2), (cx + 4, bh // 2), (cx, 12)],
           fill=(50, 180, 80, 255), width=3)
    d.line([(cx - 4, bh // 2), (cx, 12)], fill=(50, 180, 80, 255), width=3)
    # Rider
    d.ellipse([cx - 5, bh // 2 - 14, cx + 5, bh // 2 - 4],
              fill=(220, 160, 100, 255))
    img = add_shadow(img, offset=(2, 3), blur=4)
    img.save(f"{OUTPUT_DIR}/bicycle.png")
    print("  ✓ bicycle.png")


# ═══════════════════════════════════════════════════════
#  PEDESTRIAN
# ═══════════════════════════════════════════════════════

def draw_pedestrian(name="pedestrian", skin=(220, 170, 110), shirt=(80, 80, 200)):
    pw, ph = 32, 56
    img = Image.new("RGBA", (pw, ph), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    cx = pw // 2
    # Shadow on ground
    d.ellipse([cx - 10, ph - 10, cx + 10, ph - 2], fill=(0, 0, 0, 60))
    # Legs
    d.rectangle([cx - 7, ph // 2 + 2, cx - 2, ph - 8], fill=(40, 60, 120, 255))
    d.rectangle([cx + 2, ph // 2 + 2, cx + 7, ph - 8], fill=(40, 60, 120, 255))
    # Body
    d.rounded_rectangle([cx - 9, ph // 2 - 12, cx + 9, ph // 2 + 4],
                          radius=4, fill=(*shirt, 255))
    # Arms
    d.rectangle([cx - 14, ph // 2 - 10, cx - 9, ph // 2 + 2], fill=(*shirt, 220))
    d.rectangle([cx + 9, ph // 2 - 10, cx + 14, ph // 2 + 2], fill=(*shirt, 220))
    # Head
    d.ellipse([cx - 7, ph // 2 - 22, cx + 7, ph // 2 - 8], fill=(*skin, 255))
    img = add_shadow(img, offset=(2, 3), blur=4, opacity=80)
    img.save(f"{OUTPUT_DIR}/{name}.png")
    print(f"  ✓ {name}.png")


# ═══════════════════════════════════════════════════════
#  CHILD
# ═══════════════════════════════════════════════════════

def draw_child():
    pw, ph = 26, 44
    img = Image.new("RGBA", (pw, ph), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    cx = pw // 2
    d.ellipse([cx - 8, ph - 8, cx + 8, ph - 2], fill=(0, 0, 0, 50))
    # Legs
    d.rectangle([cx - 5, ph // 2 + 2, cx - 2, ph - 6], fill=(50, 80, 160, 255))
    d.rectangle([cx + 2, ph // 2 + 2, cx + 5, ph - 6], fill=(50, 80, 160, 255))
    # Body
    d.rounded_rectangle([cx - 7, ph // 2 - 9, cx + 7, ph // 2 + 4],
                          radius=3, fill=(200, 100, 200, 255))
    # Head
    d.ellipse([cx - 6, ph // 2 - 18, cx + 6, ph // 2 - 6], fill=(220, 170, 110, 255))
    img = add_shadow(img, offset=(1, 2), blur=3, opacity=70)
    img.save(f"{OUTPUT_DIR}/child.png")
    print("  ✓ child.png")


# ═══════════════════════════════════════════════════════
#  BALL
# ═══════════════════════════════════════════════════════

def draw_ball():
    bw = 28
    img = Image.new("RGBA", (bw, bw), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    # Shadow
    d.ellipse([4, 6, bw - 2, bw - 2], fill=(0, 0, 0, 60))
    # Ball
    d.ellipse([0, 0, bw - 4, bw - 4], fill=(240, 210, 30, 255))
    # Stripes
    d.arc([2, 2, bw - 6, bw - 6], start=30, end=150, fill=(200, 60, 30, 180), width=3)
    d.arc([2, 2, bw - 6, bw - 6], start=210, end=330, fill=(200, 60, 30, 180), width=3)
    # Highlight
    d.ellipse([4, 4, 12, 12], fill=(255, 255, 200, 160))
    img.save(f"{OUTPUT_DIR}/ball.png")
    print("  ✓ ball.png")


# ═══════════════════════════════════════════════════════
#  FALLEN TREE
# ═══════════════════════════════════════════════════════

def draw_fallen_tree():
    tw, th = 200, 60
    img = Image.new("RGBA", (tw, th), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    # Trunk
    trunk_pts = [(10, th // 2 - 6), (tw - 30, th // 2 - 8),
                 (tw - 30, th // 2 + 8), (10, th // 2 + 6)]
    d.polygon(trunk_pts, fill=(101, 67, 33, 255))
    # Bark lines
    for x in range(20, tw - 35, 14):
        d.line([(x, th // 2 - 5), (x + 3, th // 2 + 5)],
               fill=(80, 50, 25, 180), width=2)
    # Roots / branches at base
    d.ellipse([tw - 45, th // 2 - 18, tw, th // 2 + 18], fill=(85, 55, 20, 255))
    d.ellipse([tw - 40, th // 2 - 14, tw - 10, th // 2 + 14], fill=(101, 67, 33, 255))
    # Foliage puff at crown
    for offset, col in [((0, 0), (40, 140, 40, 220)),
                         ((-12, -4), (50, 160, 50, 200)),
                         ((14, -6), (35, 130, 35, 200)),
                         ((4, 8), (45, 150, 45, 190))]:
        fx = 2 + offset[0]
        fy = th // 2 - 10 + offset[1]
        d.ellipse([fx, fy, fx + 28, fy + 24], fill=col)
    img = add_shadow(img, offset=(3, 5), blur=6, opacity=90)
    img.save(f"{OUTPUT_DIR}/fallen_tree.png")
    print("  ✓ fallen_tree.png")


# ═══════════════════════════════════════════════════════
#  ANIMAL (dog/deer silhouette)
# ═══════════════════════════════════════════════════════

def draw_animal():
    aw, ah = 50, 38
    img = Image.new("RGBA", (aw, ah), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    col = (139, 90, 43, 255)
    # Body
    d.ellipse([8, 10, aw - 8, ah - 6], fill=col)
    # Head
    d.ellipse([aw - 20, 4, aw - 2, 18], fill=col)
    # Ears
    d.polygon([(aw - 18, 4), (aw - 22, -2), (aw - 13, 4)], fill=col)
    d.polygon([(aw - 8, 4), (aw - 5, -2), (aw - 2, 6)], fill=col)
    # Legs
    for lx in [10, 17, 28, 35]:
        d.rectangle([lx, ah - 10, lx + 5, ah], fill=col)
    # Tail
    d.line([(8, 14), (2, 6)], fill=col, width=4)
    img = add_shadow(img, offset=(2, 3), blur=4, opacity=80)
    img.save(f"{OUTPUT_DIR}/animal.png")
    print("  ✓ animal.png")


# ═══════════════════════════════════════════════════════
#  TRUCK / SUV
# ═══════════════════════════════════════════════════════

def draw_truck(color=(180, 90, 30), name="truck"):
    tw, th = 104, 180
    img = Image.new("RGBA", (tw, th), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    r = color
    # Body
    d.rounded_rectangle([6, 12, tw - 6, th - 12], radius=10, fill=(*r, 255))
    # Cab
    d.rounded_rectangle([10, th - 68, tw - 10, th - 16], radius=8,
                          fill=(tuple(min(255, c + 25) for c in r)), )
    # Windshield
    d.rounded_rectangle([16, th - 62, tw - 16, th - 44], radius=5,
                          fill=(120, 190, 220, 200))
    d.line([(18, th - 60), (tw - 18, th - 46)], fill=(200, 230, 255, 120), width=2)
    # Rear window
    d.rounded_rectangle([16, 38, tw - 16, 54], radius=4, fill=(100, 160, 200, 180))
    # Wheels (bigger)
    wc, rc2 = (25, 25, 25, 255), (175, 175, 180, 255)
    for wx1, wy1, wx2, wy2 in [(3, th - 56, 20, th - 22),
                                 (tw - 20, th - 56, tw - 3, th - 22),
                                 (3, 22, 20, 56),
                                 (tw - 20, 22, tw - 3, 56)]:
        d.ellipse([wx1, wy1, wx2, wy2], fill=wc)
        cx2, cy2 = (wx1 + wx2) // 2, (wy1 + wy2) // 2
        r3 = (wx2 - wx1) // 4
        d.ellipse([cx2 - r3, cy2 - r3, cx2 + r3, cy2 + r3], fill=rc2)
    # Headlights
    d.ellipse([12, th - 34, 28, th - 20], fill=(255, 248, 200, 255))
    d.ellipse([tw - 28, th - 34, tw - 12, th - 20], fill=(255, 248, 200, 255))
    # Taillights
    d.ellipse([12, 20, 28, 34], fill=(200, 30, 30, 255))
    d.ellipse([tw - 28, 20, tw - 12, 34], fill=(200, 30, 30, 255))
    img = add_shadow(img)
    img.save(f"{OUTPUT_DIR}/{name}.png")
    print(f"  ✓ {name}.png")


# ═══════════════════════════════════════════════════════
#  ROAD TEXTURE TILE
# ═══════════════════════════════════════════════════════

def draw_road_tile():
    tw, th = 256, 256
    img = Image.new("RGB", (tw, th), (55, 55, 55))
    d = ImageDraw.Draw(img)
    # Subtle asphalt texture - random speckles
    import random
    random.seed(42)
    for _ in range(800):
        x = random.randint(0, tw - 1)
        y = random.randint(0, th - 1)
        c = random.randint(48, 68)
        d.point([(x, y)], fill=(c, c, c))
    img.save(f"{OUTPUT_DIR}/road_tile.png")
    print("  ✓ road_tile.png")


# ═══════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════

if __name__ == "__main__":
    print(f"\nGenerating sprites → ./{OUTPUT_DIR}/\n")

    # Multiple car colors
    draw_car(color=(200, 50,  50),  name="car_red")
    draw_car(color=(50,  80,  200), name="car_blue")
    draw_car(color=(200, 200, 200), name="car_white")
    draw_car(color=(50,  160, 60),  name="car_green")
    draw_car(color=(220, 160, 40),  name="car_yellow")
    draw_car(color=(150, 50,  160), name="car_purple")
    draw_car(color=(100, 100, 100), name="car_gray")
    draw_car(color=(210, 100, 30),  name="car_orange")

    draw_ego()
    draw_truck(color=(180, 90, 30), name="truck")
    draw_motorcycle()
    draw_bicycle()
    draw_pedestrian(name="pedestrian")
    draw_pedestrian(name="child", skin=(220, 180, 120), shirt=(200, 100, 200))
    draw_child()
    draw_ball()
    draw_fallen_tree()
    draw_animal()
    draw_road_tile()

    print(f"\n✅  Done! All sprites saved to ./{OUTPUT_DIR}/")
    print("Copy the sprites/ folder to your ROS2 package:\n")
    print("  cp -r sprites/ ~/ros2_ws/src/autonomous_braking/autonomous_braking/\n")
