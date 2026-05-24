"""
sensor_fusion.py
Computer Vision and Lane Detection Module.

ROLE IN ARCHITECTURE:
This module is responsible for the "Perception" part of the LKA system. It takes raw 
camera frames (via TCP) and processes them to find the lane lines and determine the 
vehicle's lateral error (how far off-center it is).

KEY PIPELINE STEPS:
1. Bird's-Eye View (BEV) Warp: Removes perspective distortion so that parallel lanes 
   appear parallel, making geometry calculations accurate.
2. Sliding Window Search: Scans the BEV image from bottom to top to trace the lane lines.
3. Polynomial Fitting: Fits mathematical lines to the detected pixels to estimate the 
   lane center and lookahead points.
4. Error Calculation: Computes the lateral deviation in meters, passing it back to LKA_Main.
"""

import cv2
import numpy as np
import time
import config as cfg
import os
import datetime

# File lives at: src/LKA/sensor_fusion.py  →  results/LKA/ is two levels up
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_RESULTS_LKA_DIR = os.path.normpath(os.path.join(_SCRIPT_DIR, "..", "..", "results", "LKA"))


class SensorFusion:
    """
    Camera-based lane detection using Bird's-Eye-View (BEV) perspective
    transform + sliding-window search.

    Pipeline:
        1. Perspective warp to bird's-eye view
        2. Sliding-window search to find left & right lane pixel clusters
        3. Polynomial fit through each lane's pixels
        4. Lane center & lateral error computed in BEV pixel space
        5. Convert to meters using calibrated BEV pixel-to-meter ratio
    """

    def __init__(self):
        # ── State ──
        self.last_valid_error = 0.0
        self._debug_count = 0
        self._frames_since_detection = 0
        self._ever_detected = False
        self._lane_mode = "none"

        # ── Error smoothing ──
        self._error_history = []
        self._history_len = 3

        # ── Previous polynomial fits (for re-use when detection is sparse) ──
        self._left_fit = None
        self._right_fit = None
        self._left_fit_age = 999     # frames since last valid left fit
        self._right_fit_age = 999    # frames since last valid right fit
        self._max_fit_age = 15       # discard fit after N frames without update

        # ── Image geometry ──
        self._img_w = cfg.IMG_W      # 640
        self._img_h = cfg.IMG_H      # 480

        # ── BEV perspective transform ──
        # Source points: trapezoid in camera image covering the road ahead.
        # Picked from observed frames (TICK 0010/0030) where both lane lines
        # and center dashes are clearly visible.
        #   - Top edge at y=260: just below horizon, lane markings start
        #   - Bottom edge at y=450: near car hood, lane markings end
        #   - Left/right edges follow the lane line positions
        src = np.float32([
            [100, 300],    # top-left     (wide to catch lanes when car drifts)
            [540, 300],    # top-right    (must reach right lane at x≈560)
            [630, 450],    # bottom-right
            [ 10, 450],    # bottom-left
        ])
        # Destination: warped rectangle (same image size)
        dst = np.float32([
            [ 50,   0],    # top-left
            [590,   0],    # top-right
            [590, 480],    # bottom-right
            [ 50, 480],    # bottom-left
        ])
        self._M_bev = cv2.getPerspectiveTransform(src, dst)
        self._M_bev_inv = cv2.getPerspectiveTransform(dst, src)
        self._bev_src = src
        self._bev_dst = dst

        # ── BEV calibration ──
        # In BEV, the lane (left solid → right solid) maps to ~340px.
        # Real lane width = 3.5m → ~97 px/m
        self._bev_px_per_meter = 340.0 / cfg.LANE_WIDTH_M
        self._bev_center_x = self._img_w // 2   # 320 = car position in BEV

        # ── Sliding window parameters ──
        self._n_windows = 12          # vertical slices
        self._window_margin = 60      # half-width of search window [px]
        self._min_pix_recenter = 10   # min white pixels to shift window center

        # ── CLAHE (reused each frame) ──
        self._clahe = cv2.createCLAHE(clipLimit=6.0, tileGridSize=(8, 8))

        # ── Debug output ──
        run_tag = datetime.datetime.now().strftime("run_%Y%m%d_%H%M%S")
        self._frames_dir = os.path.join(_RESULTS_LKA_DIR, "debug_frames", run_tag)
        os.makedirs(self._frames_dir, exist_ok=True)
        print(f"[DEBUG] Saving frames to: {self._frames_dir}")

    # ── Public helpers ──────────────────────────────────────────────────

    def set_telemetry(self, steer_cmd=0.0, speed_kmh=0.0):
        """Store latest telemetry for debug overlay."""
        self._steer_cmd = steer_cmd
        self._speed_kmh = speed_kmh

    # ── White-pixel extraction ──────────────────────────────────────────

    def _create_white_mask(self, img):
        """
        Binary mask of lane markings using morphological top-hat transform.

        Why top-hat:
          - Brightness threshold: road texture ≈ lane paint → both pass
          - Adaptive threshold: near-black image → everything passes
          - Canny: road-grass edge drowns lane edges
          - Top-hat extracts NARROW bright features (lane lines = 5-15px wide)
            and suppresses WIDE uniform regions (road surface, grass).
            Specifically designed for this exact problem.
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Step 1: Aggressive digital brightness boost before CLAHE
        # Multiply by 5 to stretch tiny differences (15 vs 25 → 75 vs 125)
        boosted = cv2.multiply(gray, np.array([5.0]))

        # Step 2: CLAHE on boosted image
        enhanced = self._clahe.apply(boosted)

        # Step 3: Morphological top-hat — extracts bright features narrower
        # than the kernel.  Lane lines ≈ 5-15px wide; kernel = 31px wide.
        # Road surface and grass are wider → suppressed.
        tophat_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (31, 31))
        tophat = cv2.morphologyEx(enhanced, cv2.MORPH_TOPHAT, tophat_kernel)

        # Step 4: Threshold the top-hat result
        # Top-hat values: lane paint = 30-100, road/grass = 0-10
        _, binary = cv2.threshold(tophat, 15, 255, cv2.THRESH_BINARY)

        # Step 5: Mask to source trapezoid
        trap_mask = np.zeros_like(binary)
        cv2.fillPoly(trap_mask, [self._bev_src.astype(np.int32)], 255)
        binary = cv2.bitwise_and(binary, trap_mask)

        # Step 6: Light morphological close to bridge dashed-line gaps
        close_k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 7))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, close_k)

        return binary, enhanced

    # ── Perspective warp ────────────────────────────────────────────────

    def _warp_to_bev(self, binary):
        """Warp binary mask to bird's-eye view."""
        return cv2.warpPerspective(
            binary, self._M_bev, (self._img_w, self._img_h),
            flags=cv2.INTER_NEAREST
        )

    # ── Sliding window search ───────────────────────────────────────────

    def _sliding_window_search(self, bev_binary):
        """
        Find lane pixels via bottom-up sliding-window search.

        Returns:
            left_pixels:  (xs, ys) arrays
            right_pixels: (xs, ys) arrays
            window_vis:   BGR visualization image (or None)
        """
        h, w = bev_binary.shape

        # Histogram of bottom half → starting x for each lane
        hist = np.sum(bev_binary[h // 2:, :], axis=0)
        midpoint = w // 2

        left_base = int(np.argmax(hist[:midpoint]))
        right_base = int(np.argmax(hist[midpoint:])) + midpoint

        min_hist_peak = 100
        left_valid = hist[left_base] > min_hist_peak
        right_valid = hist[right_base] > min_hist_peak

        win_h = h // self._n_windows
        margin = self._window_margin

        nonzero = bev_binary.nonzero()
        nz_y = np.array(nonzero[0])
        nz_x = np.array(nonzero[1])

        left_cur = left_base if left_valid else None
        right_cur = right_base if right_valid else None

        left_inds = []
        right_inds = []

        is_debug = (self._debug_count + 1) % 1 == 0
        vis = cv2.cvtColor(bev_binary, cv2.COLOR_GRAY2BGR) if is_debug else None

        for win_idx in range(self._n_windows):
            y_lo = h - (win_idx + 1) * win_h
            y_hi = h - win_idx * win_h

            if left_cur is not None:
                xl_lo = max(0, left_cur - margin)
                xl_hi = min(w, left_cur + margin)
                good = ((nz_y >= y_lo) & (nz_y < y_hi) &
                        (nz_x >= xl_lo) & (nz_x < xl_hi)).nonzero()[0]
                left_inds.append(good)
                if vis is not None:
                    cv2.rectangle(vis, (xl_lo, y_lo), (xl_hi, y_hi), (255, 0, 0), 2)
                if len(good) >= self._min_pix_recenter:
                    left_cur = int(np.mean(nz_x[good]))

            if right_cur is not None:
                xr_lo = max(0, right_cur - margin)
                xr_hi = min(w, right_cur + margin)
                good = ((nz_y >= y_lo) & (nz_y < y_hi) &
                        (nz_x >= xr_lo) & (nz_x < xr_hi)).nonzero()[0]
                right_inds.append(good)
                if vis is not None:
                    cv2.rectangle(vis, (xr_lo, y_lo), (xr_hi, y_hi), (0, 0, 255), 2)
                if len(good) >= self._min_pix_recenter:
                    right_cur = int(np.mean(nz_x[good]))

        left_inds = np.concatenate(left_inds) if left_inds else np.array([], dtype=np.int64)
        right_inds = np.concatenate(right_inds) if right_inds else np.array([], dtype=np.int64)

        left_px = (nz_x[left_inds], nz_y[left_inds]) if len(left_inds) else (np.array([]), np.array([]))
        right_px = (nz_x[right_inds], nz_y[right_inds]) if len(right_inds) else (np.array([]), np.array([]))

        return left_px, right_px, vis

    # ── Polynomial fitting ──────────────────────────────────────────────

    def _fit_lane(self, xs, ys, min_points=15):
        """Fit x = a*y + b (linear). Returns [a,b] or None.
        
        Linear fit instead of quadratic:
        - At 30 km/h on mild curves, lanes appear nearly straight in BEV
        - Linear fit is far more robust with sparse/noisy data
        - Prevents the wild parabolic curves seen with degree 2
        """
        if len(xs) < min_points:
            return None
        try:
            return np.polyfit(ys, xs, 1)
        except Exception:
            return None

    def _eval_poly(self, fit, y_vals):
        """Evaluate polynomial at y values → x positions."""
        if len(fit) == 3:
            return fit[0] * y_vals ** 2 + fit[1] * y_vals + fit[2]
        else:
            return fit[0] * y_vals + fit[1]

    # ── Main update ─────────────────────────────────────────────────────

    def update(self, img, yaw_rate=0.0):
        """
        Process camera frame → lateral error.

        Returns:
            lateral_error:  signed [m], positive = car left of lane center
            lane_detected:  bool
            lookahead_pt:   (x, y) in camera image coordinates
        """
        if img is None:
            return self.last_valid_error, False, (self._img_w // 2, self._img_h // 2)

        h, w = self._img_h, self._img_w
        midpoint = w // 2

        try:
            # 1. White-pixel mask
            binary, enhanced = self._create_white_mask(img)

            # Pixel stats diagnostic (temporary)
            gray_diag = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            if self._debug_count % 30 == 0:
                print(f"[PIX STATS] gray min={gray_diag.min()} max={gray_diag.max()} "
                      f"mean={gray_diag.mean():.1f} | "
                      f"binary white%={100*np.count_nonzero(binary)/binary.size:.1f}%")

            # 2. Warp to BEV
            bev = self._warp_to_bev(binary)

            # 3. Sliding window search
            left_px, right_px, sw_vis = self._sliding_window_search(bev)

            # 4. Fit polynomials
            left_fit = self._fit_lane(left_px[0], left_px[1])
            right_fit = self._fit_lane(right_px[0], right_px[1])

            # Age management for stored fits
            if left_fit is not None:
                self._left_fit = left_fit
                self._left_fit_age = 0
            else:
                self._left_fit_age += 1
                if self._left_fit_age > self._max_fit_age:
                    self._left_fit = None

            if right_fit is not None:
                self._right_fit = right_fit
                self._right_fit_age = 0
            else:
                self._right_fit_age += 1
                if self._right_fit_age > self._max_fit_age:
                    self._right_fit = None

            lf = self._left_fit
            rf = self._right_fit

            # 5. Lateral error in BEV space
            lane_detected = False
            vision_error = 0.0
            lane_center_bev = float(self._bev_center_x)

            eval_y = np.array([h - 1, h * 3 // 4, h // 2])

            if lf is not None and rf is not None:
                left_xs = self._eval_poly(lf, eval_y)
                right_xs = self._eval_poly(rf, eval_y)
                lane_width = np.mean(right_xs - left_xs)

                if 150 < lane_width < 600:
                    lane_center_bev = (left_xs[0] + right_xs[0]) / 2.0
                    vision_error = (self._bev_center_x - lane_center_bev) / self._bev_px_per_meter
                    lane_detected = True
                    self._lane_mode = "both"

            if not lane_detected and rf is not None:
                right_x_bot = self._eval_poly(rf, eval_y)[0]
                half_lane_px = (cfg.LANE_WIDTH_M / 2.0) * self._bev_px_per_meter
                lane_center_bev = right_x_bot - half_lane_px
                vision_error = (self._bev_center_x - lane_center_bev) / self._bev_px_per_meter
                lane_detected = True
                self._lane_mode = "right"

            if not lane_detected and lf is not None:
                left_x_bot = self._eval_poly(lf, eval_y)[0]
                half_lane_px = (cfg.LANE_WIDTH_M / 2.0) * self._bev_px_per_meter
                lane_center_bev = left_x_bot + half_lane_px
                vision_error = (self._bev_center_x - lane_center_bev) / self._bev_px_per_meter
                lane_detected = True
                self._lane_mode = "left"

            # 6. Smoothing & hold
            if lane_detected:
                self._frames_since_detection = 0
                self._ever_detected = True
                self._error_history.append(vision_error)
                while len(self._error_history) > self._history_len:
                    self._error_history.pop(0)
                lateral_error = float(np.mean(self._error_history))
                self.last_valid_error = lateral_error
            else:
                self._frames_since_detection += 1
                self._lane_mode = "hold"
                if not self._ever_detected:
                    lateral_error = 0.0
                else:
                    lateral_error = self.last_valid_error

            # 7. Lookahead point (BEV → camera)
            if lane_detected and (lf is not None or rf is not None):
                la_y_bev = h // 3
                if lf is not None and rf is not None:
                    la_x_bev = (self._eval_poly(lf, np.array([la_y_bev]))[0] +
                                self._eval_poly(rf, np.array([la_y_bev]))[0]) / 2.0
                elif rf is not None:
                    half_px = (cfg.LANE_WIDTH_M / 2.0) * self._bev_px_per_meter
                    la_x_bev = self._eval_poly(rf, np.array([la_y_bev]))[0] - half_px
                else:
                    half_px = (cfg.LANE_WIDTH_M / 2.0) * self._bev_px_per_meter
                    la_x_bev = self._eval_poly(lf, np.array([la_y_bev]))[0] + half_px

                bev_pt = np.array([[[la_x_bev, la_y_bev]]], dtype=np.float32)
                cam_pt = cv2.perspectiveTransform(bev_pt, self._M_bev_inv)
                la_x = int(np.clip(cam_pt[0, 0, 0], 0, w - 1))
                la_y = int(np.clip(cam_pt[0, 0, 1], 0, h - 1))
                lookahead_pt = (la_x, la_y)
            else:
                lookahead_pt = (midpoint, h // 2)

            # 8. Debug visualization
            self._debug_count += 1

            if self._debug_count % 1 == 0:
                tick_num = self._debug_count
                n_left = len(left_px[0])
                n_right = len(right_px[0])

                if self._debug_count % 2 == 0:
                    print(f"[VISION DIAG] mode={self._lane_mode}, err={lateral_error:+.3f}, "
                          f"n_left={n_left}, n_right={n_right}, "
                          f"fit_L={'Y' if lf is not None else 'N'}, "
                          f"fit_R={'Y' if rf is not None else 'N'}, "
                          f"miss={self._frames_since_detection}")

                try:
                    cv2.imwrite(os.path.join(_RESULTS_LKA_DIR, "debug_raw_frame.png"), img)
                    cv2.imwrite(os.path.join(_RESULTS_LKA_DIR, "debug_edges.png"), binary)
                    cv2.imwrite(os.path.join(_RESULTS_LKA_DIR, "debug_bev.png"), bev)

                    # Sliding-window + fit visualization
                    if sw_vis is not None:
                        plot_y = np.linspace(0, h - 1, 50).astype(int)
                        if lf is not None:
                            plot_x = self._eval_poly(lf, plot_y).astype(int)
                            pts = np.column_stack((plot_x, plot_y))
                            pts = pts[(pts[:, 0] >= 0) & (pts[:, 0] < w)]
                            if len(pts) > 1:
                                cv2.polylines(sw_vis, [pts], False, (255, 255, 0), 2)
                        if rf is not None:
                            plot_x = self._eval_poly(rf, plot_y).astype(int)
                            pts = np.column_stack((plot_x, plot_y))
                            pts = pts[(pts[:, 0] >= 0) & (pts[:, 0] < w)]
                            if len(pts) > 1:
                                cv2.polylines(sw_vis, [pts], False, (0, 255, 255), 2)
                        lc = int(lane_center_bev)
                        cv2.line(sw_vis, (lc, 0), (lc, h), (0, 165, 255), 2)
                        cv2.line(sw_vis, (self._bev_center_x, 0),
                                 (self._bev_center_x, h), (255, 255, 255), 1)
                        cv2.imwrite(os.path.join(_RESULTS_LKA_DIR, "debug_sliding_window.png"), sw_vis)

                    # Camera-view visualization
                    vis = img.copy()
                    steer_val = getattr(self, '_steer_cmd', 0.0)
                    speed_val = getattr(self, '_speed_kmh', 0.0)

                    trap = self._bev_src.astype(int)
                    for i in range(4):
                        cv2.line(vis, tuple(trap[i]), tuple(trap[(i + 1) % 4]),
                                 (100, 100, 100), 1)

                    cv2.line(vis, (midpoint, 0), (midpoint, h), (255, 255, 255), 1)

                    la_x, la_y = lookahead_pt
                    cv2.drawMarker(vis, (la_x, la_y), (255, 0, 255),
                                   cv2.MARKER_CROSS, 20, 2)

                    cv2.putText(vis,
                        f"TICK {tick_num:04d}  err={lateral_error:+.3f}m  mode={self._lane_mode}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    cv2.putText(vis,
                        f"steer={steer_val:+.3f}rad  v={speed_val:.1f}km/h",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                    cv2.putText(vis,
                        f"pix: L={n_left} R={n_right}",
                        (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                    cv2.imwrite(os.path.join(_RESULTS_LKA_DIR, "debug_vision.png"), vis)
                    fname = f"frame_tick{tick_num:04d}.png"
                    cv2.imwrite(os.path.join(self._frames_dir, fname), vis)

                except Exception as dbg_e:
                    print(f"[DEBUG] Image save failed: {dbg_e}")

            return lateral_error, lane_detected, lookahead_pt

        except Exception as e:
            print(f"[SensorFusion] Error: {e}")
            import traceback
            traceback.print_exc()
            return self.last_valid_error, False, (midpoint, h - 50)
