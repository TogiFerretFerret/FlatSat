"""
Lunar Landing Site Analyzer  v5 (Optimized)
================================
Five-layer composite hazard score — lower is safer.
"""

import sys
import os
import argparse
import math
import time

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
    a   = arr.astype(np.float64)
    mu  = uniform_filter(a,      size=window, mode='reflect')
    mu2 = uniform_filter(a ** 2, size=window, mode='reflect')
    return np.clip(mu2 - mu ** 2, 0.0, None)

def normalise(arr: np.ndarray, lo_pct: float = 2.0, hi_pct: float = 98.0) -> np.ndarray:
    lo = float(np.percentile(arr, lo_pct))
    hi = float(np.percentile(arr, hi_pct))
    if hi - lo < 1e-12: return np.zeros_like(arr, dtype=np.float64)
    return np.clip((arr.astype(np.float64) - lo) / (hi - lo), 0.0, 1.0)

def shifted(arr: np.ndarray, dy: int, dx: int) -> np.ndarray:
    out = np.zeros_like(arr)
    sr0 = max(0,  dy);  sr1 = arr.shape[0] + min(0, dy)
    sc0 = max(0,  dx);  sc1 = arr.shape[1] + min(0, dx)
    dr0 = max(0, -dy);  dr1 = arr.shape[0] + min(0, -dy)
    dc0 = max(0, -dx);  dc1 = arr.shape[1] + min(0, -dx)
    out[dr0:dr1, dc0:dc1] = arr[sr0:sr1, sc0:sc1]
    return out

# =============================================================================
# CORE LAYERS
# =============================================================================

def compute_albedo_normalised(gray: np.ndarray, window_size: int, epsilon: float = 8.0) -> tuple:
    albedo_sigma = window_size * 2.0
    albedo_est   = gaussian_filter(gray, sigma=albedo_sigma, mode='reflect')
    albedo_floor = np.maximum(albedo_est, epsilon)
    norm_gray    = gray.astype(np.float64) / albedo_floor
    return norm_gray, albedo_est

def estimate_light_direction(norm_gray: np.ndarray) -> tuple:
    blurred   = gaussian_filter(norm_gray, sigma=3.0)
    gx        = sobel(blurred, axis=1).astype(np.float64)
    gy        = sobel(blurred, axis=0).astype(np.float64)
    mag       = np.sqrt(gx ** 2 + gy ** 2)
    strong    = mag > np.percentile(mag, 88)
    weights   = mag[strong]
    mgx       = float(np.average(gx[strong], weights=weights))
    mgy       = float(np.average(gy[strong], weights=weights))
    norm      = math.hypot(mgx, mgy)
    if norm < 1e-9: mgx, mgy, norm = 1.0, -1.0, math.sqrt(2.0)
    lx = mgx / norm;  ly = mgy / norm
    return math.atan2(ly, lx), lx, ly, -lx, -ly

def compute_mountain_score(norm_gray: np.ndarray, shadow_pct: float, shadow_length: float, proximity_sigma_px: float, n_steps: int = 15) -> dict:
    light_angle, lx, ly, sx, sy = estimate_light_direction(norm_gray)
    shadow_thresh    = float(np.percentile(norm_gray, shadow_pct))
    highlight_thresh = float(np.percentile(norm_gray, 100.0 - shadow_pct))
    shadow_mask      = (norm_gray <= shadow_thresh).astype(np.float64)
    highlight_mask   = (norm_gray >= highlight_thresh).astype(np.float64)
    
    shadow_root = np.zeros_like(norm_gray)
    mountain_face = np.zeros_like(norm_gray)
    for k in range(1, n_steps + 1):
        dist = shadow_length * k / n_steps
        shadow_root += shadow_mask * shifted(highlight_mask, int(round(ly * dist)), int(round(lx * dist)))
        mountain_face += highlight_mask * shifted(shadow_mask, int(round(sy * dist)), int(round(sx * dist)))

    def safe01(a):
        mx = a.max()
        return a / mx if mx > 1e-12 else np.zeros_like(a)

    raw     = safe01(shadow_root) + safe01(mountain_face)
    sigma   = max(proximity_sigma_px, 1.0)
    blurred = gaussian_filter(raw, sigma=sigma, mode='reflect')

    return dict(shadow_root=shadow_root, mountain_face=mountain_face, raw=raw, blurred=blurred,
                shadow_mask=(norm_gray <= shadow_thresh), highlight_mask=(norm_gray >= highlight_thresh),
                shadow_thresh=shadow_thresh, highlight_thresh=highlight_thresh,
                lx=lx, ly=ly, sx=sx, sy=sy, light_angle_deg=math.degrees(light_angle))

def compute_zone_penalty(var_local_norm: np.ndarray, total_px: int, min_area_frac: float, zone_radius: int, border: int, power: float = 2.0) -> tuple:
    min_area = min_area_frac * total_px
    interior_vals = var_local_norm[border:-border, border:-border].ravel()
    flat_thresh   = float(np.percentile(interior_vals, 30))
    flat_mask     = (var_local_norm <= flat_thresh).astype(np.float64)
    side         = 2 * zone_radius + 1
    flat_in_disk = (uniform_filter(flat_mask, size=side, mode='reflect') * float(side * side) * (math.pi / 4.0))
    ratio        = np.clip(flat_in_disk / max(min_area, 1.0), 0.0, 1.0)
    zone_penalty = 1.0 - ratio ** power
    border_mask = np.ones(var_local_norm.shape, dtype=bool)
    border_mask[:border,:]=False; border_mask[-border:,:]=False; border_mask[:,:border]=False; border_mask[:,-border:]=False
    return zone_penalty, border_mask, flat_thresh

def compute_all_scores(gray, window_size, context_factor, shadow_pct, mountain_radius, shadow_length_factor, proximity_sigma, w_context, w_mountain, w_zone, zone_radius, min_area_frac, albedo_epsilon=8.0):
    H, W = gray.shape
    norm_gray, albedo_est = compute_albedo_normalised(gray, window_size, epsilon=albedo_epsilon)
    var_local   = box_variance(norm_gray, window_size)
    var_context = box_variance(norm_gray, window_size * context_factor)
    shadow_length = mountain_radius * shadow_length_factor
    mtn = compute_mountain_score(norm_gray, shadow_pct, shadow_length, proximity_sigma * zone_radius)
    zone_penalty, border_mask, flat_thresh = compute_zone_penalty(var_local, H*W, min_area_frac, zone_radius, window_size)
    norm_local = normalise(var_local); norm_context = normalise(var_context); norm_mountain = normalise(mtn['blurred'])
    composite_raw = (norm_local + w_context * norm_context + w_mountain * norm_mountain + w_zone * zone_penalty)
    composite = normalise(composite_raw, lo_pct=0, hi_pct=100)
    composite[~border_mask] = 1.0
    return dict(gray=gray, norm_gray=norm_gray, albedo_est=albedo_est, var_local=var_local, var_context=var_context,
                norm_local=norm_local, norm_context=norm_context, norm_mountain=norm_mountain,
                zone_penalty=zone_penalty, border_mask=border_mask, flat_thresh=flat_thresh, mountain=mtn, composite=composite)

def find_top_sites(composite, border_mask, n, min_sep):
    work = composite.copy()
    work[~border_mask] = np.inf
    H, W = work.shape
    sites = []
    for _ in range(n):
        if np.all(np.isinf(work)): break
        r, c = np.unravel_index(np.argmin(work), work.shape)
        sites.append((r, c))
        rr, cc = np.ogrid[:H, :W]
        work[(rr - r) ** 2 + (cc - c) ** 2 <= min_sep ** 2] = np.inf
    return sites

# =============================================================================
# VISUALISATION
# =============================================================================

SAFETY_CMAP = mcolors.LinearSegmentedColormap.from_list('safety', [(0.0, '#00cc44'), (0.4, '#aaff00'), (0.6, '#ffdd00'), (0.8, '#ff7700'), (1.0, '#ff1100')], N=512)
MOUNTAIN_CMAP = mcolors.LinearSegmentedColormap.from_list('mountain', [(0.00, '#03030a'), (0.25, '#1a0050'), (0.55, '#7a1a00'), (0.80, '#ff8800'), (1.00, '#ffffc0')], N=512)
ALBEDO_CMAP = mcolors.LinearSegmentedColormap.from_list('albedo', [(0.0, '#1a0a00'), (0.4, '#5c3a1a'), (0.7, '#b07840'), (1.0, '#f0e0c0')], N=512)
ZONE_CMAP = mcolors.LinearSegmentedColormap.from_list('zone', [(0.0, '#003322'), (0.3, '#005533'), (0.6, '#cc8800'), (1.0, '#cc0000')], N=512)

def to_rgba_u8(arr01, cmap): return (cmap(np.clip(arr01, 0.0, 1.0)) * 255).astype(np.uint8)
def blend_heatmap(rgb, heatmap_rgba, alpha):
    b = rgb.astype(np.float64) / 255.0
    h = heatmap_rgba[:, :, :3].astype(np.float64) / 255.0
    return (np.clip((1 - alpha) * b + alpha * h, 0, 1) * 255).astype(np.uint8)

def add_cbar(fig, ax, cmap, lo='0', hi='1'):
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, 1))
    cb = fig.colorbar(sm, ax=ax, fraction=0.025, pad=0.015)
    cb.set_ticks([0, 0.5, 1]); cb.set_ticklabels([lo, '↑', hi], fontsize=7, color='#ccc')

def style_ax(ax, title):
    ax.axis('off'); ax.set_title(title, color='white', fontsize=9, fontweight='bold', pad=5)

def build_mountain_panel(norm_gray, mtn, norm_mountain):
    base = to_rgba_u8(norm_mountain, MOUNTAIN_CMAP).astype(np.float64)
    for mask, col, a in [(mtn['shadow_mask'], [0,70,220], 0.4), (mtn['highlight_mask'], [255,210,0], 0.4)]:
        ach = np.zeros(norm_gray.shape); ach[mask] = a
        ov = np.zeros((*norm_gray.shape, 3)); ov[mask] = col
        base[:,:,:3] = base[:,:,:3]*(1-ach[:,:,None]) + ov*ach[:,:,None]
    return np.clip(base, 0, 255).astype(np.uint8)

# =============================================================================
# MAIN PIPELINE
# =============================================================================

def analyze_image(image_path, window_size=64, context_factor=6, shadow_pct=8.0, shadow_length_factor=6.0,
                  proximity_sigma=1.5, w_context=0.5, w_mountain=1.5, w_zone=2.0, min_area_fraction=0.005,
                  n_sites=5, alpha=0.55, albedo_epsilon=8.0, output_dir=None, progress_callback=None, fast_mode=False):
    def report(msg, p):
        if progress_callback: progress_callback(msg, p)
        else: print(f"[{p*100:2.0f}%] {msg}")

    report("Loading image", 0.05)
    img = Image.open(image_path).convert('RGB')
    img_arr = np.array(img); gray = np.array(img.convert('L'), dtype=np.float64)
    H, W = gray.shape; min_area = min_area_fraction * H * W
    zone_radius = int(math.ceil(math.sqrt(min_area / math.pi)))
    mountain_radius = math.sqrt(H * W / (400.0 * math.pi))

    report("Computing layers", 0.15)
    scores = compute_all_scores(gray, window_size, context_factor, shadow_pct, mountain_radius, shadow_length_factor, proximity_sigma, w_context, w_mountain, w_zone, zone_radius, min_area_fraction, albedo_epsilon)
    
    report("Selecting sites", 0.60)
    sites = find_top_sites(scores['composite'], scores['border_mask'], n_sites, max(zone_radius*2, min(H,W)//8))

    report("Rendering", 0.75)
    hm_composite = to_rgba_u8(scores['composite'], SAFETY_CMAP)
    final_panel = blend_heatmap(img_arr, hm_composite, alpha)

    report("Saving", 0.90)
    base = os.path.splitext(os.path.basename(image_path))[0]
    out_dir = output_dir or os.path.dirname(os.path.abspath(image_path))
    out_path = os.path.join(out_dir, f"{base}_landing_analysis.png")

    if fast_mode:
        Image.fromarray(final_panel).save(out_path, quality=85)
    else:
        fig, axes = plt.subplots(4, 2, figsize=(18, 18))
        fig.patch.set_facecolor('#06060c')
        for ax in axes.ravel(): ax.axis('off')
        axes[0,0].imshow(img_arr); style_ax(axes[0,0], 'Original')
        albedo_display = normalise(scores['albedo_est'])
        axes[0,1].imshow(to_rgba_u8(albedo_display, ALBEDO_CMAP)); style_ax(axes[0,1], 'Albedo')
        axes[1,0].imshow(build_mountain_panel(scores['norm_gray'], scores['mountain'], scores['norm_mountain'])); style_ax(axes[1,0], 'Mountain')
        axes[1,1].imshow(to_rgba_u8(scores['norm_local'], SAFETY_CMAP)); style_ax(axes[1,1], 'Roughness')
        axes[2,0].imshow(to_rgba_u8(scores['zone_penalty'], ZONE_CMAP)); style_ax(axes[2,0], 'Zone Penalty')
        axes[2,1].imshow(hm_composite); style_ax(axes[2,1], 'Composite Hazard')
        axes[3,0].imshow(to_rgba_u8(scores['norm_context'], SAFETY_CMAP)); style_ax(axes[3,0], 'Context')
        axes[3,1].imshow(final_panel); style_ax(axes[3,1], 'Recommendations')
        for rank, (r, c) in enumerate(sites, start=1):
            axes[3,1].add_patch(plt.Circle((c, r), zone_radius, color='#00ffaa', fill=False, linewidth=1.5))
            axes[3,1].text(c+zone_radius, r, f'#{rank}', color='white', fontsize=8, fontweight='bold')
        plt.tight_layout(); fig.savefig(out_path, dpi=100, facecolor=fig.get_facecolor()); plt.close(fig)

    report("Complete", 1.0)
    site_data = [{"rank": i+1, "row": int(r), "col": int(c), "hazard": float(scores['composite'][r,c])} for i, (r,c) in enumerate(sites)]
    return {"output_path": out_path, "sites": site_data, "zone_radius": zone_radius}

if __name__ == '__main__':
    # Minimal CLI for backward compatibility
    if len(sys.argv) > 1:
        res = analyze_image(sys.argv[1])
        print(f"Result saved to: {res['output_path']}")
