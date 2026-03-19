"""
Lunar Landing Site Analyzer  v5
================================
Five-layer composite hazard score — lower is safer:

  1. LOCAL ROUGHNESS (albedo-normalised)
  2. NEIGHBOURHOOD CONTEXT (albedo-normalised)
  3. DIRECTIONAL MOUNTAIN DETECTION
  4. ZONE SIZE PENALTY  (soft — replaces hard exclusion mask)
  5. BORDER EXCLUSION   (hard — only for box-filter edge artefacts)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
v5 CHANGES vs v4
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

─── 1. Soil-colour / albedo correction ────────────────────────────

  Problem: the Moon has patches of naturally dark and light regolith.
  A flat dark plain sits at absolute intensity ~40; a flat bright plain
  sits at ~120.  When the old LMS variance measured absolute pixel
  deviations, it would:
    • Under-score craters on dark soil (low absolute contrast)
    • Over-score flat albedo-boundary regions (a dark plain adjacent
      to a bright plain looks like high variance even though both are
      physically flat)
    • Set global shadow/highlight thresholds that fire on flat bright
      or dark soil, generating fake mountain detections at every
      albedo boundary.

  Fix — albedo-normalised image:
    albedo = GaussianBlur(gray, sigma = 2 × window_size)
             (large enough to average over craters and mountains,
              small enough to track genuine albedo changes)
    norm_gray = gray / max(albedo, epsilon)       epsilon = 8.0

  On the normalised image:
    • Flat bright soil  → norm_gray ≈ 1.0   everywhere
    • Flat dark soil    → norm_gray ≈ 1.0   everywhere
    • Crater on any soil → norm_gray dips below 1 on shadow wall,
      rises above 1 on sunlit rim, giving reliable variance signal
    • Shadow/highlight thresholds are now relative to local albedo,
      so mountain detection fires on genuine elevation, not soil colour

  The LMS variance, neighbourhood context, and mountain detection ALL
  operate on norm_gray instead of gray.  The albedo map itself is
  shown as an extra panel for interpretability.

─── 2. Soft zone-size penalty ─────────────────────────────────────

  Problem v4: pixels with insufficient flat area (< 0.5% of image)
  within a landing-zone disk were hard-excluded — they showed as a
  dimmed void covering ~97% of the map, making the heatmap almost
  entirely dark and uninformative.

  Fix — continuous penalty weight:
    flat_in_disk  = count of flat pixels within zone_radius of each px
    ratio         = clamp(flat_in_disk / min_area, 0, 1)
    zone_penalty  = 1 - ratio²     ∈ [0, 1]

  Interpretation:
    zone_penalty = 0   → full 0.5% zone available         → no cost
    zone_penalty = 0.5 → ~70% of required zone available  → moderate cost
    zone_penalty = 1   → almost no flat area nearby        → heavy cost

  zone_penalty enters the composite with weight w_zone (default 2.0).
  The heatmap now shows gradual degradation rather than a hard cliff.
  Only the border strip (box-filter artefact zone = window_size px)
  is still hard-excluded.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Composite hazard score:
    S = 1.0 × norm(var_local)
      + w_context  × norm(var_context)
      + w_mountain × norm(mountain_proximity)
      + w_zone     × zone_penalty

All variance terms operate on the albedo-normalised image.
All normalised to [0,1] individually before weighting.
w_mountain default = 1.5, w_zone default = 2.0, w_context = 0.5.

Output figure — 4 rows × 2 cols:
  [0,0] Original image
  [0,1] Albedo map  (what the algorithm sees as "baseline brightness")
  [1,0] Mountain detection  (directional shadow/highlight, albedo-corrected)
  [1,1] LMS local roughness  (albedo-normalised)
  [2,0] Zone size penalty map  (soft 0.5% constraint)
  [2,1] Composite hazard score
  [3,0] Neighbourhood context  (albedo-normalised)
  [3,1] Final recommendations

CLI:
    python lunar_landing_analyzer.py moon.png
    python lunar_landing_analyzer.py moon.png --w-mountain 2.0 --w-zone 3.0
    python lunar_landing_analyzer.py moon.png --shadow-length-factor 8
"""

import sys
import os
import argparse
import math

import numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from scipy.ndimage import uniform_filter, gaussian_filter, sobel


# =============================================================================
# UTILITIES
# =============================================================================

def box_variance(arr: np.ndarray, window: int) -> np.ndarray:
    """O(N) local variance via Var = E[x²] - (E[x])².  Returns float64 >= 0."""
    a   = arr.astype(np.float64)
    mu  = uniform_filter(a,      size=window, mode='reflect')
    mu2 = uniform_filter(a ** 2, size=window, mode='reflect')
    return np.clip(mu2 - mu ** 2, 0.0, None)


def normalise(arr: np.ndarray,
              lo_pct: float = 2.0,
              hi_pct: float = 98.0) -> np.ndarray:
    """Percentile-robust linear normalisation to [0, 1]."""
    lo = float(np.percentile(arr, lo_pct))
    hi = float(np.percentile(arr, hi_pct))
    if hi - lo < 1e-12:
        return np.zeros_like(arr, dtype=np.float64)
    return np.clip((arr.astype(np.float64) - lo) / (hi - lo), 0.0, 1.0)


def shifted(arr: np.ndarray, dy: int, dx: int) -> np.ndarray:
    """
    Integer-pixel shift with zero-fill (no wraparound).
    dy > 0 = move image content downward.
    dx > 0 = move image content rightward.
    """
    out = np.zeros_like(arr)
    sr0 = max(0,  dy);  sr1 = arr.shape[0] + min(0, dy)
    sc0 = max(0,  dx);  sc1 = arr.shape[1] + min(0, dx)
    dr0 = max(0, -dy);  dr1 = arr.shape[0] + min(0, -dy)
    dc0 = max(0, -dx);  dc1 = arr.shape[1] + min(0, -dx)
    out[dr0:dr1, dc0:dc1] = arr[sr0:sr1, sc0:sc1]
    return out


# =============================================================================
# ALBEDO NORMALISATION
# =============================================================================

def compute_albedo_normalised(gray: np.ndarray,
                               window_size: int,
                               epsilon: float = 8.0) -> tuple:
    """
    Estimate local albedo (baseline soil brightness) and divide it out.

    albedo_sigma = 2 × window_size — large enough to smooth over individual
    craters and mountains while still tracking genuine multi-scale albedo
    variation across the image (dark mare vs bright highland patches).

    Returns
    -------
    norm_gray    : albedo-normalised image.  Flat bright soil ≈ flat dark
                   soil ≈ 1.0.  Deviations from 1.0 = physical roughness.
    albedo_est   : the smoothed albedo map (for display).
    """
    albedo_sigma = window_size * 2.0
    albedo_est   = gaussian_filter(gray, sigma=albedo_sigma, mode='reflect')
    albedo_floor = np.maximum(albedo_est, epsilon)
    norm_gray    = gray.astype(np.float64) / albedo_floor
    return norm_gray, albedo_est


# =============================================================================
# LIGHT DIRECTION
# =============================================================================

def estimate_light_direction(norm_gray: np.ndarray) -> tuple:
    """
    Magnitude-weighted mean Sobel gradient on the albedo-normalised image
    gives the dominant illumination direction, independent of soil colour.

    Returns (light_angle_rad, lx, ly, sx, sy)
      lx, ly = unit vector toward light source
      sx, sy = unit vector in shadow direction  (= -lx, -ly)
    """
    blurred   = gaussian_filter(norm_gray, sigma=3.0)
    gx        = sobel(blurred, axis=1).astype(np.float64)
    gy        = sobel(blurred, axis=0).astype(np.float64)
    mag       = np.sqrt(gx ** 2 + gy ** 2)

    strong  = mag > np.percentile(mag, 88)
    weights = mag[strong]
    mgx = float(np.average(gx[strong], weights=weights))
    mgy = float(np.average(gy[strong], weights=weights))

    norm = math.hypot(mgx, mgy)
    if norm < 1e-9:
        mgx, mgy, norm = 1.0, -1.0, math.sqrt(2.0)  # fallback: upper-right

    lx = mgx / norm;  ly = mgy / norm
    return math.atan2(ly, lx), lx, ly, -lx, -ly


# =============================================================================
# DIRECTIONAL MOUNTAIN DETECTION  (albedo-corrected)
# =============================================================================

def compute_mountain_score(norm_gray: np.ndarray,
                            shadow_pct: float,
                            shadow_length: float,
                            proximity_sigma_px: float,
                            n_steps: int = 20) -> dict:
    """
    Directional shadow/highlight co-occurrence on the albedo-normalised image.

    On norm_gray:
      • Shadows    = pixels relatively dark vs local soil  (norm < shadow_thresh_n)
      • Highlights = pixels relatively bright vs local soil (norm > highlight_thresh_n)

    Because we operate on norm_gray (not raw gray), flat dark soil and flat
    bright soil both have norm ≈ 1.0 and do NOT trigger shadow/highlight masks.
    Only genuine elevation-driven brightness extremes fire.

    Returns dict with all intermediate maps plus blurred proximity score.
    """
    light_angle, lx, ly, sx, sy = estimate_light_direction(norm_gray)

    shadow_thresh    = float(np.percentile(norm_gray, shadow_pct))
    highlight_thresh = float(np.percentile(norm_gray, 100.0 - shadow_pct))

    shadow_mask    = (norm_gray <= shadow_thresh).astype(np.float64)
    highlight_mask = (norm_gray >= highlight_thresh).astype(np.float64)

    # Ray C: shadow → toward light → find highlights  (marks shadow root)
    shadow_root = np.zeros_like(norm_gray)
    for k in range(1, n_steps + 1):
        dist = shadow_length * k / n_steps
        shadow_root += shadow_mask * shifted(highlight_mask,
                                              int(round(ly * dist)),
                                              int(round(lx * dist)))

    # Ray D: highlight → away from light → find shadows  (marks mountain face)
    mountain_face = np.zeros_like(norm_gray)
    for k in range(1, n_steps + 1):
        dist = shadow_length * k / n_steps
        mountain_face += highlight_mask * shifted(shadow_mask,
                                                   int(round(sy * dist)),
                                                   int(round(sx * dist)))

    # Combine — normalise each to [0,1] before summing so neither dominates
    def safe01(a):
        mx = a.max()
        return a / mx if mx > 1e-12 else np.zeros_like(a)

    raw     = safe01(shadow_root) + safe01(mountain_face)
    sigma   = max(proximity_sigma_px, 1.0)
    blurred = gaussian_filter(raw, sigma=sigma, mode='reflect')

    return dict(
        shadow_root      = shadow_root,
        mountain_face    = mountain_face,
        raw              = raw,
        blurred          = blurred,
        shadow_mask      = shadow_mask.astype(bool),
        highlight_mask   = highlight_mask.astype(bool),
        shadow_thresh    = shadow_thresh,
        highlight_thresh = highlight_thresh,
        lx=lx, ly=ly, sx=sx, sy=sy,
        light_angle_deg  = math.degrees(light_angle),
    )


# =============================================================================
# ZONE SIZE — SOFT PENALTY
# =============================================================================

def compute_zone_penalty(var_local_norm: np.ndarray,
                          total_px: int,
                          min_area_frac: float,
                          zone_radius: int,
                          border: int,
                          power: float = 2.0) -> tuple:
    """
    Soft continuous penalty for insufficient flat area within landing zone.

    For each pixel p:
      flat_in_disk = approximate count of flat pixels within zone_radius
      ratio        = clamp(flat_in_disk / min_area, 0, 1)
      penalty      = 1 - ratio^power          ∈ [0, 1]

    penalty = 0  → pixel has a full 0.5% flat landing zone  → no cost
    penalty = 1  → pixel has almost no flat area nearby     → maximum cost

    Flat = local normalised variance <= p30 of interior values.

    The border strip (box-filter artefact zone) stays hard-masked since
    those values are genuinely unreliable.

    Returns (zone_penalty, border_mask, flat_thresh)
    """
    min_area = min_area_frac * total_px

    interior_vals = var_local_norm[border:-border, border:-border].ravel()
    flat_thresh   = float(np.percentile(interior_vals, 30))
    flat_mask     = (var_local_norm <= flat_thresh).astype(np.float64)

    side         = 2 * zone_radius + 1
    flat_in_disk = (uniform_filter(flat_mask, size=side, mode='reflect')
                    * float(side * side) * (math.pi / 4.0))

    ratio        = np.clip(flat_in_disk / max(min_area, 1.0), 0.0, 1.0)
    zone_penalty = 1.0 - ratio ** power

    # Hard border mask — box-filter artefact zone
    border_mask = np.ones(var_local_norm.shape, dtype=bool)
    border_mask[:border,  :]  = False
    border_mask[-border:, :]  = False
    border_mask[:,  :border]  = False
    border_mask[:, -border:]  = False

    return zone_penalty, border_mask, flat_thresh


# =============================================================================
# COMPOSITE SCORE
# =============================================================================

def compute_all_scores(gray: np.ndarray,
                        window_size: int,
                        context_factor: int,
                        shadow_pct: float,
                        mountain_radius: float,
                        shadow_length_factor: float,
                        proximity_sigma: float,
                        w_context: float,
                        w_mountain: float,
                        w_zone: float,
                        zone_radius: int,
                        min_area_frac: float,
                        albedo_epsilon: float = 8.0) -> dict:
    """Compute all score layers and weighted composite."""

    H, W     = gray.shape
    total_px = H * W
    border   = window_size

    # ── Albedo correction ────────────────────────────────────────────────────
    norm_gray, albedo_est = compute_albedo_normalised(
        gray, window_size, epsilon=albedo_epsilon)

    # ── Variance layers on albedo-normalised image ────────────────────────────
    var_local   = box_variance(norm_gray, window_size)
    var_context = box_variance(norm_gray, window_size * context_factor)

    # ── Mountain detection on albedo-normalised image ─────────────────────────
    shadow_length      = mountain_radius * shadow_length_factor
    proximity_sigma_px = proximity_sigma * zone_radius
    mtn = compute_mountain_score(
        norm_gray,
        shadow_pct         = shadow_pct,
        shadow_length      = shadow_length,
        proximity_sigma_px = proximity_sigma_px,
    )

    # ── Zone size soft penalty (uses albedo-normalised variance) ──────────────
    zone_penalty, border_mask, flat_thresh = compute_zone_penalty(
        var_local, total_px, min_area_frac, zone_radius, border)

    # ── Normalise each term ───────────────────────────────────────────────────
    norm_local    = normalise(var_local)
    norm_context  = normalise(var_context)
    norm_mountain = normalise(mtn['blurred'])
    # zone_penalty is already in [0,1] by construction

    # ── Weighted composite ────────────────────────────────────────────────────
    composite_raw = (      norm_local
                    + w_context  * norm_context
                    + w_mountain * norm_mountain
                    + w_zone     * zone_penalty)
    composite = normalise(composite_raw, lo_pct=0, hi_pct=100)

    # Hard-mask border in composite
    composite[~border_mask] = 1.0

    return dict(
        gray             = gray,
        norm_gray        = norm_gray,
        albedo_est       = albedo_est,
        var_local        = var_local,
        var_context      = var_context,
        norm_local       = norm_local,
        norm_context     = norm_context,
        norm_mountain    = norm_mountain,
        zone_penalty     = zone_penalty,
        border_mask      = border_mask,
        flat_thresh      = flat_thresh,
        mountain         = mtn,
        composite        = composite,
    )


# =============================================================================
# CANDIDATE SELECTION
# =============================================================================

def find_top_sites(composite: np.ndarray,
                    border_mask: np.ndarray,
                    n: int,
                    min_sep: int) -> list:
    """Greedy NMS within border-valid pixels.  Returns list of (row, col)."""
    work = composite.copy()
    work[~border_mask] = np.inf
    H, W  = work.shape
    sites = []
    for _ in range(n):
        if np.all(np.isinf(work)):
            break
        r, c = np.unravel_index(np.argmin(work), work.shape)
        sites.append((r, c))
        rr, cc = np.ogrid[:H, :W]
        work[(rr - r) ** 2 + (cc - c) ** 2 <= min_sep ** 2] = np.inf
    return sites


# =============================================================================
# VISUALISATION
# =============================================================================

SAFETY_CMAP = mcolors.LinearSegmentedColormap.from_list(
    'safety',
    [(0.0, '#00cc44'), (0.4, '#aaff00'), (0.6, '#ffdd00'),
     (0.8, '#ff7700'), (1.0, '#ff1100')], N=512)

MOUNTAIN_CMAP = mcolors.LinearSegmentedColormap.from_list(
    'mountain',
    [(0.00, '#03030a'), (0.25, '#1a0050'),
     (0.55, '#7a1a00'), (0.80, '#ff8800'), (1.00, '#ffffc0')], N=512)

ALBEDO_CMAP = mcolors.LinearSegmentedColormap.from_list(
    'albedo',
    [(0.0, '#1a0a00'), (0.4, '#5c3a1a'),
     (0.7, '#b07840'), (1.0, '#f0e0c0')], N=512)

ZONE_CMAP = mcolors.LinearSegmentedColormap.from_list(
    'zone',
    [(0.0, '#003322'), (0.3, '#005533'),
     (0.6, '#cc8800'), (1.0, '#cc0000')], N=512)


def to_rgba_u8(arr01, cmap):
    return (cmap(np.clip(arr01, 0.0, 1.0)) * 255).astype(np.uint8)


def blend_heatmap(rgb: np.ndarray, heatmap_rgba: np.ndarray,
                   alpha: float) -> np.ndarray:
    b = rgb.astype(np.float64) / 255.0
    h = heatmap_rgba[:, :, :3].astype(np.float64) / 255.0
    return (np.clip((1 - alpha) * b + alpha * h, 0, 1) * 255).astype(np.uint8)


def add_cbar(fig, ax, cmap, lo: str = '0', hi: str = '1'):
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, 1))
    sm.set_array([])
    cb = fig.colorbar(sm, ax=ax, fraction=0.025, pad=0.015)
    cb.set_ticks([0, 0.5, 1])
    cb.set_ticklabels([lo, '↑', hi], fontsize=7, color='#cccccc')
    cb.ax.yaxis.set_tick_params(color='#888888')


def style_ax(ax, title: str):
    ax.axis('off')
    ax.set_title(title, color='white', fontsize=9, fontweight='bold',
                  pad=5, wrap=True)


def build_mountain_panel(norm_gray, mtn, norm_mountain):
    """Mountain hazard background with shadow/highlight pixel overlays."""
    base = to_rgba_u8(norm_mountain, MOUNTAIN_CMAP).astype(np.float64)
    for mask, colour, a in [
        (mtn['shadow_mask'],    np.array([0,   70, 220]), 0.42),
        (mtn['highlight_mask'], np.array([255, 210,   0]), 0.42),
    ]:
        ov           = np.zeros((*norm_gray.shape, 3), dtype=np.float64)
        ov[mask]     = colour
        ach          = np.zeros(norm_gray.shape, dtype=np.float64)
        ach[mask]    = a
        base[:, :, :3] = (base[:, :, :3] * (1 - ach[:, :, None])
                          + ov * ach[:, :, None])
    return np.clip(base, 0, 255).astype(np.uint8)


# =============================================================================
# MAIN PIPELINE
# =============================================================================

def analyze_image(image_path: str,
                  window_size: int            = 64,
                  context_factor: int         = 6,
                  shadow_pct: float           = 8.0,
                  shadow_length_factor: float = 6.0,
                  proximity_sigma: float      = 1.5,
                  w_context: float            = 0.5,
                  w_mountain: float           = 1.5,
                  w_zone: float               = 2.0,
                  min_area_fraction: float    = 0.005,
                  n_sites: int                = 5,
                  alpha: float                = 0.55,
                  albedo_epsilon: float       = 8.0,
                  output_dir: str             = None) -> str:

    # ── 1. Load ───────────────────────────────────────────────────────────────
    print(f"[1/8] Loading: {image_path}")
    img      = Image.open(image_path).convert('RGB')
    img_arr  = np.array(img)
    gray     = np.array(img.convert('L'), dtype=np.float64)
    H, W     = gray.shape
    total_px = H * W
    min_area = min_area_fraction * total_px

    # ── 2. Derived geometry ───────────────────────────────────────────────────
    zone_radius     = int(math.ceil(math.sqrt(min_area / math.pi)))
    mountain_radius = math.sqrt(total_px / (400.0 * math.pi))
    shadow_length   = mountain_radius * shadow_length_factor
    border          = window_size

    print(f"[2/8] Image: {W}×{H} = {total_px:,} px")
    print(f"      Albedo blur sigma  : {window_size*2} px")
    print(f"      Zone radius        : {zone_radius} px  "
          f"(min area {min_area:.0f} px = {min_area_fraction*100:.2f}%)")
    print(f"      Mountain radius    : {mountain_radius:.1f} px  (1/400 of image)")
    print(f"      Shadow length      : {shadow_length:.1f} px")

    # ── 3. Scores ─────────────────────────────────────────────────────────────
    print(f"[3/8] Computing all score layers (albedo-normalised)...")
    scores = compute_all_scores(
        gray,
        window_size          = window_size,
        context_factor       = context_factor,
        shadow_pct           = shadow_pct,
        mountain_radius      = mountain_radius,
        shadow_length_factor = shadow_length_factor,
        proximity_sigma      = proximity_sigma,
        w_context            = w_context,
        w_mountain           = w_mountain,
        w_zone               = w_zone,
        zone_radius          = zone_radius,
        min_area_frac        = min_area_fraction,
        albedo_epsilon       = albedo_epsilon,
    )
    mtn = scores['mountain']
    print(f"      Light direction    : {mtn['light_angle_deg']:.1f}°  "
          f"({mtn['lx']:.3f}, {mtn['ly']:.3f})")
    print(f"      Shadow threshold   : norm_gray <= {mtn['shadow_thresh']:.3f}")
    print(f"      Highlight threshold: norm_gray >= {mtn['highlight_thresh']:.3f}")
    print(f"      Flat threshold     : norm_var <= {scores['flat_thresh']:.5f}")
    print(f"      Zone penalty=0 area: "
          f"{(scores['zone_penalty']==0).mean()*100:.1f}% of image")

    # ── 4. Candidates ─────────────────────────────────────────────────────────
    print(f"[4/8] Selecting top {n_sites} candidates...")
    min_sep = max(zone_radius * 2, min(H, W) // 8)
    sites   = find_top_sites(scores['composite'], scores['border_mask'],
                              n_sites, min_sep)
    if not sites:
        print("  WARNING: no candidates found — try --min-area 0.002")

    # ── 5. Build heatmap images ───────────────────────────────────────────────
    print("[5/8] Building display images...")

    # Albedo panel: normalise display to [0,1]
    albedo_display  = normalise(scores['albedo_est'])
    hm_albedo       = to_rgba_u8(albedo_display, ALBEDO_CMAP)

    mountain_panel  = build_mountain_panel(
        scores['norm_gray'], mtn, scores['norm_mountain'])
    hm_local        = to_rgba_u8(scores['norm_local'],   SAFETY_CMAP)
    hm_context      = to_rgba_u8(scores['norm_context'], SAFETY_CMAP)
    hm_zone         = to_rgba_u8(scores['zone_penalty'], ZONE_CMAP)
    hm_composite    = to_rgba_u8(scores['composite'],    SAFETY_CMAP)

    # Final panel: composite blended over original
    final_panel = blend_heatmap(img_arr, hm_composite, alpha)

    # ── 6. Figure  (4×2) ──────────────────────────────────────────────────────
    print("[6/8] Composing figure...")
    fig, axes = plt.subplots(4, 2, figsize=(22, 22))
    fig.patch.set_facecolor('#06060c')
    for ax in axes.ravel():
        ax.axis('off')

    # [0,0] Original
    axes[0, 0].imshow(img_arr)
    style_ax(axes[0, 0], 'Original Image')

    # [0,1] Albedo map
    axes[0, 1].imshow(hm_albedo)
    style_ax(axes[0, 1],
             f'Estimated Albedo  (soil colour baseline)\n'
             f'bright = high-reflectance soil · dark = low-reflectance soil\n'
             f'blur σ={window_size*2}px — all other layers divide by this')
    add_cbar(fig, axes[0, 1], ALBEDO_CMAP, lo='dark soil', hi='bright soil')

    # [1,0] Mountain detection
    cx, cy    = W // 2, H // 2
    arr_len   = min(H, W) * 0.10
    axes[1, 0].imshow(mountain_panel)
    axes[1, 0].annotate(
        '', xy=(cx + mtn['lx'] * arr_len, cy + mtn['ly'] * arr_len),
        xytext=(cx, cy),
        arrowprops=dict(arrowstyle='->', color='white', lw=2.0))
    axes[1, 0].text(cx + mtn['lx'] * arr_len * 1.18,
                     cy + mtn['ly'] * arr_len * 1.18,
                     'light', color='white', fontsize=8, ha='center')
    style_ax(axes[1, 0],
             f'Directional Mountain Detection  (albedo-corrected)\n'
             f'blue=shadow · yellow=highlight · glow=mountain proximity hazard\n'
             f'thresholds relative to local soil brightness  '
             f'light {mtn["light_angle_deg"]:.0f}°')
    add_cbar(fig, axes[1, 0], MOUNTAIN_CMAP, lo='flat', hi='mountain')

    # [1,1] Local roughness
    axes[1, 1].imshow(hm_local)
    style_ax(axes[1, 1],
             f'LMS Local Roughness  (albedo-normalised, window={window_size}px)\n'
             f'green=smooth · red=rough  — soil colour does NOT affect this score')
    add_cbar(fig, axes[1, 1], SAFETY_CMAP, lo='smooth', hi='rough')

    # [2,0] Zone size penalty
    axes[2, 0].imshow(hm_zone)
    style_ax(axes[2, 0],
             f'Landing Zone Size Penalty  (soft  {min_area_fraction*100:.1f}% constraint)\n'
             f'green=sufficient flat area within r={zone_radius}px · '
             f'red=insufficient flat area\n'
             f'weight={w_zone}  — continuous penalty, no hard exclusion')
    add_cbar(fig, axes[2, 0], ZONE_CMAP, lo='zone OK', hi='zone too small')

    # [2,1] Composite hazard
    axes[2, 1].imshow(hm_composite)
    style_ax(axes[2, 1],
             f'Composite Hazard Score\n'
             f'1×roughness + {w_context}×context + '
             f'{w_mountain}×mountain + {w_zone}×zone_penalty')
    add_cbar(fig, axes[2, 1], SAFETY_CMAP, lo='safe', hi='dangerous')

    # [3,0] Neighbourhood context
    axes[3, 0].imshow(hm_context)
    style_ax(axes[3, 0],
             f'Neighbourhood Context  '
             f'(albedo-normalised, window={window_size*context_factor}px)\n'
             f'green=flat surroundings · red=mountainous surroundings')
    add_cbar(fig, axes[3, 0], SAFETY_CMAP, lo='flat surround', hi='rough surround')

    # [3,1] Final recommendations
    axes[3, 1].imshow(final_panel)
    style_ax(axes[3, 1],
             f'Recommended Landing Sites\n'
             f'circle = landing zone r={zone_radius}px  '
             f'({min_area_fraction*100:.1f}% of image)')

    label_off = zone_radius + 8
    for rank, (r, c) in enumerate(sites, start=1):
        circle = plt.Circle((c, r), zone_radius, color='#00ffaa',
                              fill=False, linewidth=2.0, linestyle='--')
        axes[3, 1].add_patch(circle)
        axes[3, 1].plot(c, r, '+', color='white',
                         markersize=12, markeredgewidth=2.0)
        pct_comp  = scores['composite'][r, c] * 100
        pct_mtn   = scores['norm_mountain'][r, c] * 100
        pct_local = scores['norm_local'][r, c] * 100
        pct_zone  = scores['zone_penalty'][r, c] * 100
        axes[3, 1].text(
            c + label_off, r,
            (f'#{rank}\n'
             f'composite {pct_comp:.1f}%\n'
             f'roughness {pct_local:.1f}%\n'
             f'mountain  {pct_mtn:.1f}%\n'
             f'zone pen  {pct_zone:.1f}%'),
            color='white', fontsize=7.5, fontweight='bold',
            va='center', linespacing=1.4,
            bbox=dict(boxstyle='round,pad=0.3', fc='#000000cc', ec='none'))

    fig.suptitle(
        'Lunar Landing Site Analysis  v5  ·  '
        'Albedo-Normalised LMS  +  Context  +  '
        'Directional Mountain Detection  +  Soft Zone Constraint',
        color='white', fontsize=12, fontweight='bold', y=1.002)
    plt.tight_layout(rect=[0, 0, 1, 1])

    # ── 7. Save ───────────────────────────────────────────────────────────────
    print("[7/8] Saving...")
    base    = os.path.splitext(os.path.basename(image_path))[0]
    out_dir = output_dir or os.path.dirname(os.path.abspath(image_path))
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"{base}_landing_analysis.png")
    fig.savefig(out_path, dpi=150, bbox_inches='tight',
                facecolor=fig.get_facecolor())
    plt.close(fig)

    # ── 8. Console report ─────────────────────────────────────────────────────
    sep = '─' * 80
    print(f"\n  Saved -> {out_path}\n")
    print(sep)
    print(f"  {'Rk':>2}  {'Row':>5}  {'Col':>5}  "
          f"{'Roughness':>10}  {'Mountain':>10}  "
          f"{'Context':>9}  {'ZonePen':>8}  {'Composite':>10}")
    print(sep)
    for rank, (r, c) in enumerate(sites, start=1):
        print(f"  #{rank:<2}  {r:5d}  {c:5d}  "
              f"{scores['norm_local'][r,c]*100:9.1f}%  "
              f"{scores['norm_mountain'][r,c]*100:9.1f}%  "
              f"{scores['norm_context'][r,c]*100:8.1f}%  "
              f"{scores['zone_penalty'][r,c]*100:7.1f}%  "
              f"{scores['composite'][r,c]*100:9.1f}%")
    print(sep)
    print(f"  Albedo correction    : blur σ={window_size*2}px, floor={albedo_epsilon}")
    print(f"  Light direction      : {mtn['light_angle_deg']:.1f}°  "
          f"vector ({mtn['lx']:.3f}, {mtn['ly']:.3f})")
    print(f"  Shadow threshold     : norm_gray <= {mtn['shadow_thresh']:.3f}")
    print(f"  Highlight threshold  : norm_gray >= {mtn['highlight_thresh']:.3f}")
    print(f"  Shadow length        : {shadow_length:.0f} px  "
          f"({shadow_length_factor}× mountain r {mountain_radius:.1f}px)")
    print(f"  Zone radius          : {zone_radius} px → "
          f"{math.pi*zone_radius**2:.0f} px² = {min_area_fraction*100:.2f}% of image")
    print(f"  Zone penalty=0 area  : "
          f"{(scores['zone_penalty']==0).mean()*100:.1f}% of image")
    print(f"  Weights              : roughness=1.0  context={w_context}  "
          f"mountain={w_mountain}  zone={w_zone}")
    print(sep)

    return out_path


# =============================================================================
# CLI
# =============================================================================

def main():
    p = argparse.ArgumentParser(
        description='Lunar Landing Site Analyzer v5 — '
                    'albedo-normalised + soft zone penalty',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    p.add_argument('image',
                   help='Path to lunar surface image (PNG/JPG/TIFF/...)')
    p.add_argument('--window',               type=int,   default=64,
                   help='LMS local window (px) — landing footprint size')
    p.add_argument('--context-factor',       type=int,   default=6,
                   help='Context window = factor × window')
    p.add_argument('--shadow-pct',           type=float, default=8.0,
                   help='Bottom/top %% of normalised pixels = shadow/highlight')
    p.add_argument('--shadow-length-factor', type=float, default=6.0,
                   help='Shadow length = factor × mountain_radius')
    p.add_argument('--proximity-sigma',      type=float, default=1.5,
                   help='Mountain hazard blur: sigma = factor × zone_radius')
    p.add_argument('--w-context',            type=float, default=0.5,
                   help='Weight of neighbourhood context term')
    p.add_argument('--w-mountain',           type=float, default=1.5,
                   help='Weight of mountain hazard term')
    p.add_argument('--w-zone',               type=float, default=2.0,
                   help='Weight of zone-size penalty term')
    p.add_argument('--min-area',             type=float, default=0.005,
                   help='Target landing zone as fraction of image (0.005=0.5%%)')
    p.add_argument('--sites',                type=int,   default=5,
                   help='Number of candidate sites to mark')
    p.add_argument('--alpha',                type=float, default=0.55,
                   help='Heatmap overlay opacity on final panel (0-1)')
    p.add_argument('--albedo-epsilon',       type=float, default=8.0,
                   help='Floor value for albedo denominator (prevents div/0)')
    p.add_argument('--output-dir',           default=None,
                   help='Output directory (default: same folder as input)')

    args = p.parse_args()

    if not os.path.isfile(args.image):
        print(f"Error: file not found: {args.image}", file=sys.stderr)
        sys.exit(1)

    analyze_image(
        image_path           = args.image,
        window_size          = args.window,
        context_factor       = args.context_factor,
        shadow_pct           = args.shadow_pct,
        shadow_length_factor = args.shadow_length_factor,
        proximity_sigma      = args.proximity_sigma,
        w_context            = args.w_context,
        w_mountain           = args.w_mountain,
        w_zone               = args.w_zone,
        min_area_fraction    = args.min_area,
        n_sites              = args.sites,
        alpha                = args.alpha,
        albedo_epsilon       = args.albedo_epsilon,
        output_dir           = args.output_dir,
    )


if __name__ == '__main__':
    main()
"""
Lunar Landing Site Analyzer  v5
================================
Five-layer composite hazard score — lower is safer:

  1. LOCAL ROUGHNESS (albedo-normalised)
  2. NEIGHBOURHOOD CONTEXT (albedo-normalised)
  3. DIRECTIONAL MOUNTAIN DETECTION
  4. ZONE SIZE PENALTY  (soft — replaces hard exclusion mask)
  5. BORDER EXCLUSION   (hard — only for box-filter edge artefacts)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
v5 CHANGES vs v4
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

─── 1. Soil-colour / albedo correction ────────────────────────────

  Problem: the Moon has patches of naturally dark and light regolith.
  A flat dark plain sits at absolute intensity ~40; a flat bright plain
  sits at ~120.  When the old LMS variance measured absolute pixel
  deviations, it would:
    • Under-score craters on dark soil (low absolute contrast)
    • Over-score flat albedo-boundary regions (a dark plain adjacent
      to a bright plain looks like high variance even though both are
      physically flat)
    • Set global shadow/highlight thresholds that fire on flat bright
      or dark soil, generating fake mountain detections at every
      albedo boundary.

  Fix — albedo-normalised image:
    albedo = GaussianBlur(gray, sigma = 2 × window_size)
             (large enough to average over craters and mountains,
              small enough to track genuine albedo changes)
    norm_gray = gray / max(albedo, epsilon)       epsilon = 8.0

  On the normalised image:
    • Flat bright soil  → norm_gray ≈ 1.0   everywhere
    • Flat dark soil    → norm_gray ≈ 1.0   everywhere
    • Crater on any soil → norm_gray dips below 1 on shadow wall,
      rises above 1 on sunlit rim, giving reliable variance signal
    • Shadow/highlight thresholds are now relative to local albedo,
      so mountain detection fires on genuine elevation, not soil colour

  The LMS variance, neighbourhood context, and mountain detection ALL
  operate on norm_gray instead of gray.  The albedo map itself is
  shown as an extra panel for interpretability.

─── 2. Soft zone-size penalty ─────────────────────────────────────

  Problem v4: pixels with insufficient flat area (< 0.5% of image)
  within a landing-zone disk were hard-excluded — they showed as a
  dimmed void covering ~97% of the map, making the heatmap almost
  entirely dark and uninformative.

  Fix — continuous penalty weight:
    flat_in_disk  = count of flat pixels within zone_radius of each px
    ratio         = clamp(flat_in_disk / min_area, 0, 1)
    zone_penalty  = 1 - ratio²     ∈ [0, 1]

  Interpretation:
    zone_penalty = 0   → full 0.5% zone available         → no cost
    zone_penalty = 0.5 → ~70% of required zone available  → moderate cost
    zone_penalty = 1   → almost no flat area nearby        → heavy cost

  zone_penalty enters the composite with weight w_zone (default 2.0).
  The heatmap now shows gradual degradation rather than a hard cliff.
  Only the border strip (box-filter artefact zone = window_size px)
  is still hard-excluded.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Composite hazard score:
    S = 1.0 × norm(var_local)
      + w_context  × norm(var_context)
      + w_mountain × norm(mountain_proximity)
      + w_zone     × zone_penalty

All variance terms operate on the albedo-normalised image.
All normalised to [0,1] individually before weighting.
w_mountain default = 1.5, w_zone default = 2.0, w_context = 0.5.

Output figure — 4 rows × 2 cols:
  [0,0] Original image
  [0,1] Albedo map  (what the algorithm sees as "baseline brightness")
  [1,0] Mountain detection  (directional shadow/highlight, albedo-corrected)
  [1,1] LMS local roughness  (albedo-normalised)
  [2,0] Zone size penalty map  (soft 0.5% constraint)
  [2,1] Composite hazard score
  [3,0] Neighbourhood context  (albedo-normalised)
  [3,1] Final recommendations

CLI:
    python lunar_landing_analyzer.py moon.png
    python lunar_landing_analyzer.py moon.png --w-mountain 2.0 --w-zone 3.0
    python lunar_landing_analyzer.py moon.png --shadow-length-factor 8
"""

import sys
import os
import argparse
import math

import numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from scipy.ndimage import uniform_filter, gaussian_filter, sobel


# =============================================================================
# UTILITIES
# =============================================================================

def box_variance(arr: np.ndarray, window: int) -> np.ndarray:
    """O(N) local variance via Var = E[x²] - (E[x])².  Returns float64 >= 0."""
    a   = arr.astype(np.float64)
    mu  = uniform_filter(a,      size=window, mode='reflect')
    mu2 = uniform_filter(a ** 2, size=window, mode='reflect')
    return np.clip(mu2 - mu ** 2, 0.0, None)


def normalise(arr: np.ndarray,
              lo_pct: float = 2.0,
              hi_pct: float = 98.0) -> np.ndarray:
    """Percentile-robust linear normalisation to [0, 1]."""
    lo = float(np.percentile(arr, lo_pct))
    hi = float(np.percentile(arr, hi_pct))
    if hi - lo < 1e-12:
        return np.zeros_like(arr, dtype=np.float64)
    return np.clip((arr.astype(np.float64) - lo) / (hi - lo), 0.0, 1.0)


def shifted(arr: np.ndarray, dy: int, dx: int) -> np.ndarray:
    """
    Integer-pixel shift with zero-fill (no wraparound).
    dy > 0 = move image content downward.
    dx > 0 = move image content rightward.
    """
    out = np.zeros_like(arr)
    sr0 = max(0,  dy);  sr1 = arr.shape[0] + min(0, dy)
    sc0 = max(0,  dx);  sc1 = arr.shape[1] + min(0, dx)
    dr0 = max(0, -dy);  dr1 = arr.shape[0] + min(0, -dy)
    dc0 = max(0, -dx);  dc1 = arr.shape[1] + min(0, -dx)
    out[dr0:dr1, dc0:dc1] = arr[sr0:sr1, sc0:sc1]
    return out


# =============================================================================
# ALBEDO NORMALISATION
# =============================================================================

def compute_albedo_normalised(gray: np.ndarray,
                               window_size: int,
                               epsilon: float = 8.0) -> tuple:
    """
    Estimate local albedo (baseline soil brightness) and divide it out.

    albedo_sigma = 2 × window_size — large enough to smooth over individual
    craters and mountains while still tracking genuine multi-scale albedo
    variation across the image (dark mare vs bright highland patches).

    Returns
    -------
    norm_gray    : albedo-normalised image.  Flat bright soil ≈ flat dark
                   soil ≈ 1.0.  Deviations from 1.0 = physical roughness.
    albedo_est   : the smoothed albedo map (for display).
    """
    albedo_sigma = window_size * 2.0
    albedo_est   = gaussian_filter(gray, sigma=albedo_sigma, mode='reflect')
    albedo_floor = np.maximum(albedo_est, epsilon)
    norm_gray    = gray.astype(np.float64) / albedo_floor
    return norm_gray, albedo_est


# =============================================================================
# LIGHT DIRECTION
# =============================================================================

def estimate_light_direction(norm_gray: np.ndarray) -> tuple:
    """
    Magnitude-weighted mean Sobel gradient on the albedo-normalised image
    gives the dominant illumination direction, independent of soil colour.

    Returns (light_angle_rad, lx, ly, sx, sy)
      lx, ly = unit vector toward light source
      sx, sy = unit vector in shadow direction  (= -lx, -ly)
    """
    blurred   = gaussian_filter(norm_gray, sigma=3.0)
    gx        = sobel(blurred, axis=1).astype(np.float64)
    gy        = sobel(blurred, axis=0).astype(np.float64)
    mag       = np.sqrt(gx ** 2 + gy ** 2)

    strong  = mag > np.percentile(mag, 88)
    weights = mag[strong]
    mgx = float(np.average(gx[strong], weights=weights))
    mgy = float(np.average(gy[strong], weights=weights))

    norm = math.hypot(mgx, mgy)
    if norm < 1e-9:
        mgx, mgy, norm = 1.0, -1.0, math.sqrt(2.0)  # fallback: upper-right

    lx = mgx / norm;  ly = mgy / norm
    return math.atan2(ly, lx), lx, ly, -lx, -ly


# =============================================================================
# DIRECTIONAL MOUNTAIN DETECTION  (albedo-corrected)
# =============================================================================

def compute_mountain_score(norm_gray: np.ndarray,
                            shadow_pct: float,
                            shadow_length: float,
                            proximity_sigma_px: float,
                            n_steps: int = 20) -> dict:
    """
    Directional shadow/highlight co-occurrence on the albedo-normalised image.

    On norm_gray:
      • Shadows    = pixels relatively dark vs local soil  (norm < shadow_thresh_n)
      • Highlights = pixels relatively bright vs local soil (norm > highlight_thresh_n)

    Because we operate on norm_gray (not raw gray), flat dark soil and flat
    bright soil both have norm ≈ 1.0 and do NOT trigger shadow/highlight masks.
    Only genuine elevation-driven brightness extremes fire.

    Returns dict with all intermediate maps plus blurred proximity score.
    """
    light_angle, lx, ly, sx, sy = estimate_light_direction(norm_gray)

    shadow_thresh    = float(np.percentile(norm_gray, shadow_pct))
    highlight_thresh = float(np.percentile(norm_gray, 100.0 - shadow_pct))

    shadow_mask    = (norm_gray <= shadow_thresh).astype(np.float64)
    highlight_mask = (norm_gray >= highlight_thresh).astype(np.float64)

    # Ray C: shadow → toward light → find highlights  (marks shadow root)
    shadow_root = np.zeros_like(norm_gray)
    for k in range(1, n_steps + 1):
        dist = shadow_length * k / n_steps
        shadow_root += shadow_mask * shifted(highlight_mask,
                                              int(round(ly * dist)),
                                              int(round(lx * dist)))

    # Ray D: highlight → away from light → find shadows  (marks mountain face)
    mountain_face = np.zeros_like(norm_gray)
    for k in range(1, n_steps + 1):
        dist = shadow_length * k / n_steps
        mountain_face += highlight_mask * shifted(shadow_mask,
                                                   int(round(sy * dist)),
                                                   int(round(sx * dist)))

    # Combine — normalise each to [0,1] before summing so neither dominates
    def safe01(a):
        mx = a.max()
        return a / mx if mx > 1e-12 else np.zeros_like(a)

    raw     = safe01(shadow_root) + safe01(mountain_face)
    sigma   = max(proximity_sigma_px, 1.0)
    blurred = gaussian_filter(raw, sigma=sigma, mode='reflect')

    return dict(
        shadow_root      = shadow_root,
        mountain_face    = mountain_face,
        raw              = raw,
        blurred          = blurred,
        shadow_mask      = shadow_mask.astype(bool),
        highlight_mask   = highlight_mask.astype(bool),
        shadow_thresh    = shadow_thresh,
        highlight_thresh = highlight_thresh,
        lx=lx, ly=ly, sx=sx, sy=sy,
        light_angle_deg  = math.degrees(light_angle),
    )


# =============================================================================
# ZONE SIZE — SOFT PENALTY
# =============================================================================

def compute_zone_penalty(var_local_norm: np.ndarray,
                          total_px: int,
                          min_area_frac: float,
                          zone_radius: int,
                          border: int,
                          power: float = 2.0) -> tuple:
    """
    Soft continuous penalty for insufficient flat area within landing zone.

    For each pixel p:
      flat_in_disk = approximate count of flat pixels within zone_radius
      ratio        = clamp(flat_in_disk / min_area, 0, 1)
      penalty      = 1 - ratio^power          ∈ [0, 1]

    penalty = 0  → pixel has a full 0.5% flat landing zone  → no cost
    penalty = 1  → pixel has almost no flat area nearby     → maximum cost

    Flat = local normalised variance <= p30 of interior values.

    The border strip (box-filter artefact zone) stays hard-masked since
    those values are genuinely unreliable.

    Returns (zone_penalty, border_mask, flat_thresh)
    """
    min_area = min_area_frac * total_px

    interior_vals = var_local_norm[border:-border, border:-border].ravel()
    flat_thresh   = float(np.percentile(interior_vals, 30))
    flat_mask     = (var_local_norm <= flat_thresh).astype(np.float64)

    side         = 2 * zone_radius + 1
    flat_in_disk = (uniform_filter(flat_mask, size=side, mode='reflect')
                    * float(side * side) * (math.pi / 4.0))

    ratio        = np.clip(flat_in_disk / max(min_area, 1.0), 0.0, 1.0)
    zone_penalty = 1.0 - ratio ** power

    # Hard border mask — box-filter artefact zone
    border_mask = np.ones(var_local_norm.shape, dtype=bool)
    border_mask[:border,  :]  = False
    border_mask[-border:, :]  = False
    border_mask[:,  :border]  = False
    border_mask[:, -border:]  = False

    return zone_penalty, border_mask, flat_thresh


# =============================================================================
# COMPOSITE SCORE
# =============================================================================

def compute_all_scores(gray: np.ndarray,
                        window_size: int,
                        context_factor: int,
                        shadow_pct: float,
                        mountain_radius: float,
                        shadow_length_factor: float,
                        proximity_sigma: float,
                        w_context: float,
                        w_mountain: float,
                        w_zone: float,
                        zone_radius: int,
                        min_area_frac: float,
                        albedo_epsilon: float = 8.0) -> dict:
    """Compute all score layers and weighted composite."""

    H, W     = gray.shape
    total_px = H * W
    border   = window_size

    # ── Albedo correction ────────────────────────────────────────────────────
    norm_gray, albedo_est = compute_albedo_normalised(
        gray, window_size, epsilon=albedo_epsilon)

    # ── Variance layers on albedo-normalised image ────────────────────────────
    var_local   = box_variance(norm_gray, window_size)
    var_context = box_variance(norm_gray, window_size * context_factor)

    # ── Mountain detection on albedo-normalised image ─────────────────────────
    shadow_length      = mountain_radius * shadow_length_factor
    proximity_sigma_px = proximity_sigma * zone_radius
    mtn = compute_mountain_score(
        norm_gray,
        shadow_pct         = shadow_pct,
        shadow_length      = shadow_length,
        proximity_sigma_px = proximity_sigma_px,
    )

    # ── Zone size soft penalty (uses albedo-normalised variance) ──────────────
    zone_penalty, border_mask, flat_thresh = compute_zone_penalty(
        var_local, total_px, min_area_frac, zone_radius, border)

    # ── Normalise each term ───────────────────────────────────────────────────
    norm_local    = normalise(var_local)
    norm_context  = normalise(var_context)
    norm_mountain = normalise(mtn['blurred'])
    # zone_penalty is already in [0,1] by construction

    # ── Weighted composite ────────────────────────────────────────────────────
    composite_raw = (      norm_local
                    + w_context  * norm_context
                    + w_mountain * norm_mountain
                    + w_zone     * zone_penalty)
    composite = normalise(composite_raw, lo_pct=0, hi_pct=100)

    # Hard-mask border in composite
    composite[~border_mask] = 1.0

    return dict(
        gray             = gray,
        norm_gray        = norm_gray,
        albedo_est       = albedo_est,
        var_local        = var_local,
        var_context      = var_context,
        norm_local       = norm_local,
        norm_context     = norm_context,
        norm_mountain    = norm_mountain,
        zone_penalty     = zone_penalty,
        border_mask      = border_mask,
        flat_thresh      = flat_thresh,
        mountain         = mtn,
        composite        = composite,
    )


# =============================================================================
# CANDIDATE SELECTION
# =============================================================================

def find_top_sites(composite: np.ndarray,
                    border_mask: np.ndarray,
                    n: int,
                    min_sep: int) -> list:
    """Greedy NMS within border-valid pixels.  Returns list of (row, col)."""
    work = composite.copy()
    work[~border_mask] = np.inf
    H, W  = work.shape
    sites = []
    for _ in range(n):
        if np.all(np.isinf(work)):
            break
        r, c = np.unravel_index(np.argmin(work), work.shape)
        sites.append((r, c))
        rr, cc = np.ogrid[:H, :W]
        work[(rr - r) ** 2 + (cc - c) ** 2 <= min_sep ** 2] = np.inf
    return sites


# =============================================================================
# VISUALISATION
# =============================================================================

SAFETY_CMAP = mcolors.LinearSegmentedColormap.from_list(
    'safety',
    [(0.0, '#00cc44'), (0.4, '#aaff00'), (0.6, '#ffdd00'),
     (0.8, '#ff7700'), (1.0, '#ff1100')], N=512)

MOUNTAIN_CMAP = mcolors.LinearSegmentedColormap.from_list(
    'mountain',
    [(0.00, '#03030a'), (0.25, '#1a0050'),
     (0.55, '#7a1a00'), (0.80, '#ff8800'), (1.00, '#ffffc0')], N=512)

ALBEDO_CMAP = mcolors.LinearSegmentedColormap.from_list(
    'albedo',
    [(0.0, '#1a0a00'), (0.4, '#5c3a1a'),
     (0.7, '#b07840'), (1.0, '#f0e0c0')], N=512)

ZONE_CMAP = mcolors.LinearSegmentedColormap.from_list(
    'zone',
    [(0.0, '#003322'), (0.3, '#005533'),
     (0.6, '#cc8800'), (1.0, '#cc0000')], N=512)


def to_rgba_u8(arr01, cmap):
    return (cmap(np.clip(arr01, 0.0, 1.0)) * 255).astype(np.uint8)


def blend_heatmap(rgb: np.ndarray, heatmap_rgba: np.ndarray,
                   alpha: float) -> np.ndarray:
    b = rgb.astype(np.float64) / 255.0
    h = heatmap_rgba[:, :, :3].astype(np.float64) / 255.0
    return (np.clip((1 - alpha) * b + alpha * h, 0, 1) * 255).astype(np.uint8)


def add_cbar(fig, ax, cmap, lo: str = '0', hi: str = '1'):
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, 1))
    sm.set_array([])
    cb = fig.colorbar(sm, ax=ax, fraction=0.025, pad=0.015)
    cb.set_ticks([0, 0.5, 1])
    cb.set_ticklabels([lo, '↑', hi], fontsize=7, color='#cccccc')
    cb.ax.yaxis.set_tick_params(color='#888888')


def style_ax(ax, title: str):
    ax.axis('off')
    ax.set_title(title, color='white', fontsize=9, fontweight='bold',
                  pad=5, wrap=True)


def build_mountain_panel(norm_gray, mtn, norm_mountain):
    """Mountain hazard background with shadow/highlight pixel overlays."""
    base = to_rgba_u8(norm_mountain, MOUNTAIN_CMAP).astype(np.float64)
    for mask, colour, a in [
        (mtn['shadow_mask'],    np.array([0,   70, 220]), 0.42),
        (mtn['highlight_mask'], np.array([255, 210,   0]), 0.42),
    ]:
        ov           = np.zeros((*norm_gray.shape, 3), dtype=np.float64)
        ov[mask]     = colour
        ach          = np.zeros(norm_gray.shape, dtype=np.float64)
        ach[mask]    = a
        base[:, :, :3] = (base[:, :, :3] * (1 - ach[:, :, None])
                          + ov * ach[:, :, None])
    return np.clip(base, 0, 255).astype(np.uint8)


# =============================================================================
# MAIN PIPELINE
# =============================================================================

def analyze_image(image_path: str,
                  window_size: int            = 64,
                  context_factor: int         = 6,
                  shadow_pct: float           = 8.0,
                  shadow_length_factor: float = 6.0,
                  proximity_sigma: float      = 1.5,
                  w_context: float            = 0.5,
                  w_mountain: float           = 1.5,
                  w_zone: float               = 2.0,
                  min_area_fraction: float    = 0.005,
                  n_sites: int                = 5,
                  alpha: float                = 0.55,
                  albedo_epsilon: float       = 8.0,
                  output_dir: str             = None) -> str:

    # ── 1. Load ───────────────────────────────────────────────────────────────
    print(f"[1/8] Loading: {image_path}")
    img      = Image.open(image_path).convert('RGB')
    img_arr  = np.array(img)
    gray     = np.array(img.convert('L'), dtype=np.float64)
    H, W     = gray.shape
    total_px = H * W
    min_area = min_area_fraction * total_px

    # ── 2. Derived geometry ───────────────────────────────────────────────────
    zone_radius     = int(math.ceil(math.sqrt(min_area / math.pi)))
    mountain_radius = math.sqrt(total_px / (400.0 * math.pi))
    shadow_length   = mountain_radius * shadow_length_factor
    border          = window_size

    print(f"[2/8] Image: {W}×{H} = {total_px:,} px")
    print(f"      Albedo blur sigma  : {window_size*2} px")
    print(f"      Zone radius        : {zone_radius} px  "
          f"(min area {min_area:.0f} px = {min_area_fraction*100:.2f}%)")
    print(f"      Mountain radius    : {mountain_radius:.1f} px  (1/400 of image)")
    print(f"      Shadow length      : {shadow_length:.1f} px")

    # ── 3. Scores ─────────────────────────────────────────────────────────────
    print(f"[3/8] Computing all score layers (albedo-normalised)...")
    scores = compute_all_scores(
        gray,
        window_size          = window_size,
        context_factor       = context_factor,
        shadow_pct           = shadow_pct,
        mountain_radius      = mountain_radius,
        shadow_length_factor = shadow_length_factor,
        proximity_sigma      = proximity_sigma,
        w_context            = w_context,
        w_mountain           = w_mountain,
        w_zone               = w_zone,
        zone_radius          = zone_radius,
        min_area_frac        = min_area_fraction,
        albedo_epsilon       = albedo_epsilon,
    )
    mtn = scores['mountain']
    print(f"      Light direction    : {mtn['light_angle_deg']:.1f}°  "
          f"({mtn['lx']:.3f}, {mtn['ly']:.3f})")
    print(f"      Shadow threshold   : norm_gray <= {mtn['shadow_thresh']:.3f}")
    print(f"      Highlight threshold: norm_gray >= {mtn['highlight_thresh']:.3f}")
    print(f"      Flat threshold     : norm_var <= {scores['flat_thresh']:.5f}")
    print(f"      Zone penalty=0 area: "
          f"{(scores['zone_penalty']==0).mean()*100:.1f}% of image")

    # ── 4. Candidates ─────────────────────────────────────────────────────────
    print(f"[4/8] Selecting top {n_sites} candidates...")
    min_sep = max(zone_radius * 2, min(H, W) // 8)
    sites   = find_top_sites(scores['composite'], scores['border_mask'],
                              n_sites, min_sep)
    if not sites:
        print("  WARNING: no candidates found — try --min-area 0.002")

    # ── 5. Build heatmap images ───────────────────────────────────────────────
    print("[5/8] Building display images...")

    # Albedo panel: normalise display to [0,1]
    albedo_display  = normalise(scores['albedo_est'])
    hm_albedo       = to_rgba_u8(albedo_display, ALBEDO_CMAP)

    mountain_panel  = build_mountain_panel(
        scores['norm_gray'], mtn, scores['norm_mountain'])
    hm_local        = to_rgba_u8(scores['norm_local'],   SAFETY_CMAP)
    hm_context      = to_rgba_u8(scores['norm_context'], SAFETY_CMAP)
    hm_zone         = to_rgba_u8(scores['zone_penalty'], ZONE_CMAP)
    hm_composite    = to_rgba_u8(scores['composite'],    SAFETY_CMAP)

    # Final panel: composite blended over original
    final_panel = blend_heatmap(img_arr, hm_composite, alpha)

    # ── 6. Figure  (4×2) ──────────────────────────────────────────────────────
    print("[6/8] Composing figure...")
    fig, axes = plt.subplots(4, 2, figsize=(22, 22))
    fig.patch.set_facecolor('#06060c')
    for ax in axes.ravel():
        ax.axis('off')

    # [0,0] Original
    axes[0, 0].imshow(img_arr)
    style_ax(axes[0, 0], 'Original Image')

    # [0,1] Albedo map
    axes[0, 1].imshow(hm_albedo)
    style_ax(axes[0, 1],
             f'Estimated Albedo  (soil colour baseline)\n'
             f'bright = high-reflectance soil · dark = low-reflectance soil\n'
             f'blur σ={window_size*2}px — all other layers divide by this')
    add_cbar(fig, axes[0, 1], ALBEDO_CMAP, lo='dark soil', hi='bright soil')

    # [1,0] Mountain detection
    cx, cy    = W // 2, H // 2
    arr_len   = min(H, W) * 0.10
    axes[1, 0].imshow(mountain_panel)
    axes[1, 0].annotate(
        '', xy=(cx + mtn['lx'] * arr_len, cy + mtn['ly'] * arr_len),
        xytext=(cx, cy),
        arrowprops=dict(arrowstyle='->', color='white', lw=2.0))
    axes[1, 0].text(cx + mtn['lx'] * arr_len * 1.18,
                     cy + mtn['ly'] * arr_len * 1.18,
                     'light', color='white', fontsize=8, ha='center')
    style_ax(axes[1, 0],
             f'Directional Mountain Detection  (albedo-corrected)\n'
             f'blue=shadow · yellow=highlight · glow=mountain proximity hazard\n'
             f'thresholds relative to local soil brightness  '
             f'light {mtn["light_angle_deg"]:.0f}°')
    add_cbar(fig, axes[1, 0], MOUNTAIN_CMAP, lo='flat', hi='mountain')

    # [1,1] Local roughness
    axes[1, 1].imshow(hm_local)
    style_ax(axes[1, 1],
             f'LMS Local Roughness  (albedo-normalised, window={window_size}px)\n'
             f'green=smooth · red=rough  — soil colour does NOT affect this score')
    add_cbar(fig, axes[1, 1], SAFETY_CMAP, lo='smooth', hi='rough')

    # [2,0] Zone size penalty
    axes[2, 0].imshow(hm_zone)
    style_ax(axes[2, 0],
             f'Landing Zone Size Penalty  (soft  {min_area_fraction*100:.1f}% constraint)\n'
             f'green=sufficient flat area within r={zone_radius}px · '
             f'red=insufficient flat area\n'
             f'weight={w_zone}  — continuous penalty, no hard exclusion')
    add_cbar(fig, axes[2, 0], ZONE_CMAP, lo='zone OK', hi='zone too small')

    # [2,1] Composite hazard
    axes[2, 1].imshow(hm_composite)
    style_ax(axes[2, 1],
             f'Composite Hazard Score\n'
             f'1×roughness + {w_context}×context + '
             f'{w_mountain}×mountain + {w_zone}×zone_penalty')
    add_cbar(fig, axes[2, 1], SAFETY_CMAP, lo='safe', hi='dangerous')

    # [3,0] Neighbourhood context
    axes[3, 0].imshow(hm_context)
    style_ax(axes[3, 0],
             f'Neighbourhood Context  '
             f'(albedo-normalised, window={window_size*context_factor}px)\n'
             f'green=flat surroundings · red=mountainous surroundings')
    add_cbar(fig, axes[3, 0], SAFETY_CMAP, lo='flat surround', hi='rough surround')

    # [3,1] Final recommendations
    axes[3, 1].imshow(final_panel)
    style_ax(axes[3, 1],
             f'Recommended Landing Sites\n'
             f'circle = landing zone r={zone_radius}px  '
             f'({min_area_fraction*100:.1f}% of image)')

    label_off = zone_radius + 8
    for rank, (r, c) in enumerate(sites, start=1):
        circle = plt.Circle((c, r), zone_radius, color='#00ffaa',
                              fill=False, linewidth=2.0, linestyle='--')
        axes[3, 1].add_patch(circle)
        axes[3, 1].plot(c, r, '+', color='white',
                         markersize=12, markeredgewidth=2.0)
        pct_comp  = scores['composite'][r, c] * 100
        pct_mtn   = scores['norm_mountain'][r, c] * 100
        pct_local = scores['norm_local'][r, c] * 100
        pct_zone  = scores['zone_penalty'][r, c] * 100
        axes[3, 1].text(
            c + label_off, r,
            (f'#{rank}\n'
             f'composite {pct_comp:.1f}%\n'
             f'roughness {pct_local:.1f}%\n'
             f'mountain  {pct_mtn:.1f}%\n'
             f'zone pen  {pct_zone:.1f}%'),
            color='white', fontsize=7.5, fontweight='bold',
            va='center', linespacing=1.4,
            bbox=dict(boxstyle='round,pad=0.3', fc='#000000cc', ec='none'))

    fig.suptitle(
        'Lunar Landing Site Analysis  v5  ·  '
        'Albedo-Normalised LMS  +  Context  +  '
        'Directional Mountain Detection  +  Soft Zone Constraint',
        color='white', fontsize=12, fontweight='bold', y=1.002)
    plt.tight_layout(rect=[0, 0, 1, 1])

    # ── 7. Save ───────────────────────────────────────────────────────────────
    print("[7/8] Saving...")
    base    = os.path.splitext(os.path.basename(image_path))[0]
    out_dir = output_dir or os.path.dirname(os.path.abspath(image_path))
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"{base}_landing_analysis.png")
    fig.savefig(out_path, dpi=150, bbox_inches='tight',
                facecolor=fig.get_facecolor())
    plt.close(fig)

    # ── 8. Console report ─────────────────────────────────────────────────────
    sep = '─' * 80
    print(f"\n  Saved -> {out_path}\n")
    print(sep)
    print(f"  {'Rk':>2}  {'Row':>5}  {'Col':>5}  "
          f"{'Roughness':>10}  {'Mountain':>10}  "
          f"{'Context':>9}  {'ZonePen':>8}  {'Composite':>10}")
    print(sep)
    for rank, (r, c) in enumerate(sites, start=1):
        print(f"  #{rank:<2}  {r:5d}  {c:5d}  "
              f"{scores['norm_local'][r,c]*100:9.1f}%  "
              f"{scores['norm_mountain'][r,c]*100:9.1f}%  "
              f"{scores['norm_context'][r,c]*100:8.1f}%  "
              f"{scores['zone_penalty'][r,c]*100:7.1f}%  "
              f"{scores['composite'][r,c]*100:9.1f}%")
    print(sep)
    print(f"  Albedo correction    : blur σ={window_size*2}px, floor={albedo_epsilon}")
    print(f"  Light direction      : {mtn['light_angle_deg']:.1f}°  "
          f"vector ({mtn['lx']:.3f}, {mtn['ly']:.3f})")
    print(f"  Shadow threshold     : norm_gray <= {mtn['shadow_thresh']:.3f}")
    print(f"  Highlight threshold  : norm_gray >= {mtn['highlight_thresh']:.3f}")
    print(f"  Shadow length        : {shadow_length:.0f} px  "
          f"({shadow_length_factor}× mountain r {mountain_radius:.1f}px)")
    print(f"  Zone radius          : {zone_radius} px → "
          f"{math.pi*zone_radius**2:.0f} px² = {min_area_fraction*100:.2f}% of image")
    print(f"  Zone penalty=0 area  : "
          f"{(scores['zone_penalty']==0).mean()*100:.1f}% of image")
    print(f"  Weights              : roughness=1.0  context={w_context}  "
          f"mountain={w_mountain}  zone={w_zone}")
    print(sep)

    return out_path


