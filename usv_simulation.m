%% usv_simulation_sapf_camera.m
% Simulasi USV 4-DOF: D* Lite + G2-CBS C² + ILOS + PID + SAPF + Camera
%
% Fitur:
%   - SAPF dengan cone-based repulsion untuk obstacle DINAMIS
%   - Deteksi kamera dengan FOV dan range
%   - Obstacle STATIS tambahan: muncul & memicu replan saat terdeteksi kamera
%   - Toggle enable_extra_obstacle (true/false)
%     * true  → obstacle statis baru ada, replan terjadi saat kamera mendeteksi
%     * false → tidak ada obstacle statis tambahan, tidak ada replan
%   - Obstacle DINAMIS tetap berjalan normal (tidak terpengaruh toggle)

clear; clc; close all;

%% ===================================================================
%% KONFIGURASI SCENARIO
%% ===================================================================

enable_extra_obstacle = false;
% true  → obstacle statis tambahan ada di environment; replan saat kamera mendeteksi
% false → tidak ada obstacle statis tambahan; simulasi normal tanpa replan
%         (obstacle dinamis tetap ada dan SAPF tetap aktif)

%% ===== SKALA GRID & PETA (METER) =====
cell_m = 2;
mapSize_m = [33 50];
nR = ceil(mapSize_m(1)/cell_m);
nC = ceil(mapSize_m(2)/cell_m);
mapSize = [nR nC];

m2g = @(p_m) [p_m(:,1)/cell_m + 0.5, p_m(:,2)/cell_m + 0.5];
g2m = @(pg)  [(pg(:,1)-0.5)*cell_m,   (pg(:,2)-0.5)*cell_m];

%% ===== OBSTACLES, START, GOAL (METER) =====
obstacles_static_m = [20.0 20.0 0.25;
                      40.0 20.0 0.25;
                      10.0 10.0 0.25;
                      30.0 10.0 0.25;
                      17.0 16.5 0.25;
                      41.0 16.0 0.25];

start_m    = [ 1.0  8.0];
waypoint_m = [25.0 20.0];
goal_m     = [48.0 13.0];

start    = m2g(start_m);
waypoint = m2g(waypoint_m);
goal_g   = m2g(goal_m);

%% ===== PARAMETER KAPAL LSS-01 =====
A1  =  1.5066;  A2  = -0.7405;  A3  =  0.4219;  A4  = -0.1397;
A5  = -0.1464;  A6  = -3.1952;  A7  =  4.1189;  A8  = 0.0;  A9  = 0.0;
A18 =  0.0178;  A19 =  0.02;
A10 = (A1 / A18) * A19;
A11 = (1.0 / A18) * A19;
A12 = -0.35;    A13 =  1.4038;  A14 = -2.0764;
A15 =  0.0010;  A16 =  0.9671;  A17 =  0.0021;
A20 =  0.0;     A21 =  0.0;     A22 =  0.0;
KpLin  =  0.0;     KpAbs  =  0.0;  KpCub  =  0.0;
Kphi   =  13.5523; Kfy    = -0.0175;
Kv_phi = -3.3096;  Kr_phi = -2.7576; Kbias  = -0.3631;
g_accel = 9.81;

%% ===== BATAS AKTUATOR =====
lim_TX = 200.0;
lim_TY =  60.0;
lim_TK =  60.0;

%% ===== GAINS FORCE CONTROLLER =====
Ku   = 80.0;
Kr   = 55.0;
Kd_r =  4.0;

%% ===== BATAS STATE =====
u_max_ms = 3.0;  v_max_ms = 3.0;
u_max_g  = u_max_ms / cell_m;
v_max_g  = v_max_ms / cell_m;
r_max    = 0.7;
p_max    = 2.0;

%% ===== BANKING CONTROL =====
phi_max  = deg2rad(10.0);
tau_phi  = 0.25;
Kphi_p   = 6.0;
Kphi_i   = 0.5;
Kphi_d   = 3.0;

%% ===== ILOS GUIDANCE =====
k_integral     = 0.3;
lookahead_m    = 1.5;
cte_thresh_m   = 1.5;
lookahead_k    = 1.5;
psi_filter_tau = 0.67;

%% ===== PID HEADING =====
kp_psi     = 2.0;
ki_psi     = 0.05;
kd_psi     = 0.5;
RUDDER_MAX = deg2rad(40.0);

%% ===== MISI =====
u_des       = 1.5;
arrive_dist = 0.5;
goal_tol    = 4.0;
fade_in_sec = 2.0;

%% ===== SAPF PARAMETERS =====
sapf = struct();
sapf.zeta         = 1.0;
sapf.eta          = 2.5;
sapf.R_infl       = 4.0;
sapf.d_safe       = 1.2;
sapf.d_vort       = 2.6;   % zona vortex: d_safe + 0.5*(R_infl - d_safe)
sapf.v_min_factor = 0.3;
sapf.v_max        = u_des;

%% ===== CAMERA PARAMETERS =====
% Kamera digunakan untuk:
%   (1) Deteksi obstacle DINAMIS → SAPF avoidance
%   (2) Deteksi obstacle STATIS tambahan → replan D* Lite
camera = struct();
camera.fov       = deg2rad(120);  % field of view [rad]
camera.minRange  = 0.5;           % minimum detection range [m]
camera.maxRange  = 8.0;           % maximum detection range [m]
camera.sigma_r   = 0.1;           % range noise std [m]
camera.sigma_b   = deg2rad(2);    % bearing noise std [rad]

% Kamera untuk deteksi obstacle statis (range lebih jauh, FOV sama)
camera_static = struct();
camera_static.range = 12.0;       % detection range [m]
camera_static.fov   = deg2rad(70.0);  % field of view [rad]

%% ===== OBSTACLE DINAMIS =====
obs_size = 0.45;

obs_dyn1 = struct();
obs_dyn1.pos    = [15, 7.5];
obs_dyn1.vel    = [0, 0.3];
obs_dyn1.rad    = obs_size/2 + 0.3;
obs_dyn1.active = true;

obs_dyn2 = struct();
obs_dyn2.pos    = [37, 32];
obs_dyn2.vel    = [0, -0.3];
obs_dyn2.rad    = obs_size/2 + 0.3;
obs_dyn2.active = true;

%% ===== OBSTACLE STATIS TAMBAHAN (GROUND TRUTH) =====
% Ada di environment jika enable_extra_obstacle = true
% Tidak diketahui planner sampai kamera mendeteksinya
if enable_extra_obstacle
    extraObs_gt_m = [35.2 19.3 0.25];  % [cx cy radius] dalam meter
else
    extraObs_gt_m = [NaN NaN NaN];     % dummy — tidak ada obstacle
end

extra_known     = ~enable_extra_obstacle;  % false jika aktif (belum diketahui)
replan_done     = ~enable_extra_obstacle;  % false jika aktif (belum replan)
obstacles_all_m = obstacles_static_m;      % obstacle yang diketahui planner

%% ===== SIMULASI =====
dt    = 0.02;
T_max = 300;
N_max = round(T_max / dt);

%% ===== PLANNING PARAMS =====
safe_dist_m = 1.5;
safe_plan_m = 0.3;
inflate_m   = safe_plan_m + safe_dist_m;
n_per_seg   = 25;
eps_rdp     = 1.0;

%% ===================================================================
%% 1. BANGUN OCCUPANCY GRID
%% ===================================================================
occ = false(nR, nC);
min_occ_m = 0.5 * sqrt(2) * cell_m;

for k_obs = 1:size(obstacles_static_m, 1)
    cx    = obstacles_static_m(k_obs, 1);
    cy    = obstacles_static_m(k_obs, 2);
    r_occ = max(obstacles_static_m(k_obs, 3) + inflate_m, min_occ_m);

    for row = 1:nR
        for col = 1:nC
            cx_cell = (col - 0.5) * cell_m;
            cy_cell = (row - 0.5) * cell_m;
            if (cx_cell - cx)^2 + (cy_cell - cy)^2 <= r_occ^2
                occ(row, col) = true;
            end
        end
    end
end

fprintf('Map: %d×%d grid (%d×%d m) | %d obstacle cells\n', ...
    nR, nC, mapSize_m(1), mapSize_m(2), sum(occ(:)));

%% ===================================================================
%% 2. D* LITE PATH PLANNING
%% ===================================================================
waypoints_list_m = {waypoint_m; goal_m};
path_global_m    = start_m;
seg_start        = start;
seg_start_m      = start_m;

for wi = 1:length(waypoints_list_m)
    seg_goal_m = waypoints_list_m{wi};
    seg_goal   = m2g(seg_goal_m);

    fprintf('Rencana WP%d: (%.1f,%.1f) → (%.1f,%.1f) m\n', ...
        wi, seg_start_m(1), seg_start_m(2), seg_goal_m(1), seg_goal_m(2));

    path_g = dstar_lite_plan(occ, nR, nC, seg_start, seg_goal);

    if isempty(path_g)
        warning('Path WP%d tidak ditemukan → fallback garis lurus', wi);
        path_g = [round(seg_start); round(seg_goal)];
    end

    seg_path_m        = g2m(path_g);
    seg_path_m(1,:)   = seg_start_m;
    seg_path_m(end,:) = seg_goal_m;

    if size(path_global_m, 1) == 1
        path_global_m = seg_path_m;
    else
        path_global_m = [path_global_m; seg_path_m(2:end,:)];
    end

    seg_start   = seg_goal;
    seg_start_m = seg_goal_m;
end

fprintf('D* Lite: %d titik raw\n', size(path_global_m, 1));

%% ===================================================================
%% 3. G2-CBS C² PATH SMOOTHING + CLEARANCE ENFORCEMENT
%% ===================================================================
[~, idx_wp_raw] = min(vecnorm(path_global_m - waypoint_m, 2, 2));

seg1_raw = path_global_m(1:idx_wp_raw, :);
seg2_raw = path_global_m(idx_wp_raw:end, :);

seg1_smooth = smooth_path_g2cbs(seg1_raw, n_per_seg, eps_rdp);
seg2_smooth = smooth_path_g2cbs(seg2_raw, n_per_seg, eps_rdp);

[seg1_smooth, clr1] = enforce_clearance(seg1_smooth, obstacles_static_m, safe_plan_m);
[seg2_smooth, clr2] = enforce_clearance(seg2_smooth, obstacles_static_m, safe_plan_m);

seg1_smooth(1,:)   = start_m;
seg1_smooth(end,:) = waypoint_m;
seg2_smooth(1,:)   = waypoint_m;
seg2_smooth(end,:) = goal_m;

path_smooth_m = [seg1_smooth; seg2_smooth(2:end,:)];

clr_info.min_clearance = min(clr1.min_clearance, clr2.min_clearance);
clr_info.iterations    = clr1.iterations + clr2.iterations;

fprintf('Smoothing: %d titik | min_clearance=%.3f m | iter=%d\n', ...
    size(path_smooth_m, 1), clr_info.min_clearance, clr_info.iterations);

%% ===================================================================
%% 4. KONDISI AWAL
%% ===================================================================
x_g = start(1);
y_g = start(2);
psi = 0.0;
phi = 0.0;
nu  = [0; 0; 0; 0];

wp1_passed   = false;
phi_des_prev = 0.0;
eInt_phi     = 0.0;

sigma      = 0.0;
psi_d_filt = 0.0;
ilos_init  = false;

pid_integral  = 0.0;
pid_prev_err  = 0.0;
pid_filt_rate = 0.0;

% Logging — 31 kolom
% Col 1-23 : sama dengan original
% Col 24-25: F_att [x y]   gaya atraktif SAPF
% Col 26-27: F_rep [x y]   gaya repulsif SAPF (total dari semua obs dinamis)
% Col 28-29: F_tot [x y]   gaya resultan SAPF
% Col 30   : sapf_active   flag (1 jika minimal 1 obs dinamis dalam R_infl)
% Col 31   : p_goal_local [x y] disimpan terpisah di bawah
LOG = zeros(N_max, 31);
LOG_replan  = false(N_max, 1);
obs_dyn_log = zeros(N_max, 4);
camera_log  = zeros(N_max, 6);  % [seen1, d1, b1, seen2, d2, b2]
k_end = N_max;

occ_init         = occ;
path_smooth_init = path_smooth_m;
path_smooth_replan = [];
did_replan = false;

% Buffer gaya SAPF untuk visualisasi animasi
F_att_log  = zeros(N_max, 2);  % gaya atraktif [Fx Fy]
F_rep_log  = zeros(N_max, 2);  % gaya repulsif total [Fx Fy]
F_tot_log  = zeros(N_max, 2);  % gaya resultan [Fx Fy]
pgoal_log  = zeros(N_max, 2);  % local lookahead point pada path

%% ===================================================================
%% 5. MAIN SIMULATION LOOP
%% ===================================================================
for k = 1:N_max
    t = (k - 1) * dt;

    x_m = (x_g - 0.5) * cell_m;
    y_m = (y_g - 0.5) * cell_m;
    pos = [x_m, y_m];

    d_goal = norm(pos - goal_m);

    %% ==== UPDATE OBSTACLE DINAMIS ====
    obs_dyn1.pos = obs_dyn1.pos + obs_dyn1.vel * dt;
    obs_dyn2.pos = obs_dyn2.pos + obs_dyn2.vel * dt;
    obs_dyn_log(k,:) = [obs_dyn1.pos obs_dyn2.pos];

    %% ==== CEK WAYPOINT 1 ====
    if ~wp1_passed && norm(pos - waypoint_m) < arrive_dist
        wp1_passed = true;
        fprintf('Waypoint 1 passed at t = %.2f s\n', t);
    end

    %% ==== CEK ARRIVAL GOAL ====
    if d_goal < arrive_dist
        fprintf('Goal reached at t=%.1f s | pos=(%.1f, %.1f) m\n', t, x_m, y_m);
        k_end = k - 1;
        break;
    end

    %% ================================================================
    %% DETEKSI OBSTACLE DINAMIS (SAPF)
    %% ================================================================
    [seen1, meas1] = camera_detect(pos, psi, obs_dyn1, camera);
    [seen2, meas2] = camera_detect(pos, psi, obs_dyn2, camera);

    if seen1
        camera_log(k, 1:3) = [1, meas1.dist, meas1.bearing];
    end
    if seen2
        camera_log(k, 4:6) = [1, meas2.dist, meas2.bearing];
    end

    %% ================================================================
    %% DETEKSI OBSTACLE STATIS TAMBAHAN (TRIGGER REPLAN)
    %% ================================================================
    % Obstacle statis tambahan ada di environment jika enable_extra_obstacle=true,
    % namun tidak diketahui planner sampai kamera mendeteksinya pertama kali.
    % Deteksi ini menggunakan camera_static (range/FOV berbeda dari SAPF camera).

    if enable_extra_obstacle && ~extra_known
        [seenExtra] = camera_detect_static(pos, psi, extraObs_gt_m, camera_static);

        if seenExtra
            fprintf('[Camera] Obstacle statis baru terdeteksi t=%.2f s | pos=(%.1f,%.1f) m\n', ...
                t, extraObs_gt_m(1), extraObs_gt_m(2));
            extra_known     = true;
            obstacles_all_m = [obstacles_all_m; extraObs_gt_m];

            %% ============================================================
            %% REPLAN D* LITE SETELAH DETEKSI KAMERA
            %% ============================================================
            fprintf('[Replan] D* Lite dijalankan ulang dari pos=(%.1f,%.1f) m\n', pos(1), pos(2));

            % Tambahkan obstacle baru ke occupancy grid
            r_occ_new = max(extraObs_gt_m(3) + inflate_m, min_occ_m);
            for row = 1:nR
                for col = 1:nC
                    cx_c = (col - 0.5) * cell_m;
                    cy_c = (row - 0.5) * cell_m;
                    if (cx_c - extraObs_gt_m(1))^2 + (cy_c - extraObs_gt_m(2))^2 <= r_occ_new^2
                        occ(row, col) = true;
                    end
                end
            end

            start_replan = m2g(pos);

            % Strategi replan berdasarkan status waypoint
            if ~wp1_passed
                % Belum lewat WP1: replan dua segmen
                path1_g = dstar_lite_plan(occ, nR, nC, start_replan, m2g(waypoint_m));
                path2_g = dstar_lite_plan(occ, nR, nC, m2g(waypoint_m), m2g(goal_m));

                if ~isempty(path1_g) && ~isempty(path2_g)
                    path_global_m_re = [g2m(path1_g); g2m(path2_g(2:end,:))];
                else
                    warning('[Replan] Gagal — tetap pakai path lama');
                    replan_done = true;
                    continue;
                end
            else
                % Sudah lewat WP1: replan langsung ke goal
                path_g_re = dstar_lite_plan(occ, nR, nC, start_replan, m2g(goal_m));
                if ~isempty(path_g_re)
                    path_global_m_re = g2m(path_g_re);
                else
                    warning('[Replan] Gagal — tetap pakai path lama');
                    replan_done = true;
                    continue;
                end
            end

            path_global_m_re(1,:)   = pos;
            path_global_m_re(end,:) = goal_m;

            path_smooth_re = smooth_path_g2cbs(path_global_m_re, n_per_seg, eps_rdp);
            [path_smooth_re, ~] = enforce_clearance(path_smooth_re, obstacles_all_m, safe_plan_m);
            path_smooth_re(1,:)   = pos;
            path_smooth_re(end,:) = goal_m;

            path_smooth_m      = path_smooth_re;
            path_smooth_replan = path_smooth_re;

            % Reset ILOS state
            sigma      = 0.0;
            psi_d_filt = psi;
            ilos_init  = true;

            replan_done   = true;
            did_replan    = true;
            LOG_replan(k) = true;

            fprintf('[Replan] Selesai: %d titik smooth baru\n', size(path_smooth_m, 1));
        end
    end

    %% ================================================================
    %% ILOS GUIDANCE
    %% ================================================================
    fade_in = min(1.0, t / fade_in_sec);

    if d_goal < lookahead_m
        psi_ilos = wrap_angle(atan2(goal_m(2) - pos(2), goal_m(1) - pos(1)));
        cte_val  = 0.0;
        if ~ilos_init
            psi_d_filt = psi_ilos;
            ilos_init  = true;
        end
    else
        if ~ilos_init
            psi_d_filt = psi;
            ilos_init  = true;
        end
        [psi_ilos, sigma, psi_d_filt, cte_val] = ilos_compute( ...
            pos, path_smooth_m, sigma, psi_d_filt, dt, ...
            k_integral, lookahead_m, cte_thresh_m, lookahead_k, psi_filter_tau);
    end

    %% ================================================================
    %% SAPF LOCAL OBSTACLE AVOIDANCE (OBSTACLE DINAMIS)
    %% ================================================================
    dist_to_path = vecnorm(path_smooth_m - pos, 2, 2);
    [~, idx_nearest] = min(dist_to_path);
    idx_lookahead = min(idx_nearest + 5, size(path_smooth_m, 1));
    p_goal_local  = path_smooth_m(idx_lookahead, :);

    psi_des    = psi_ilos;
    u_cmd_base = u_des;

    % Gaya SAPF: dihitung eksplisit untuk logging & visualisasi
    vec_g   = p_goal_local - pos;
    d_g     = norm(vec_g) + 1e-9;
    F_att_k = sapf.zeta * (vec_g / d_g);   % gaya atraktif
    F_rep_k = [0 0];                        % akumulasi gaya repulsif

    % Obstacle dinamis 1
    if seen1
        obs_for_sapf        = struct();
        obs_for_sapf.pos    = meas1.pos_est;
        obs_for_sapf.rad    = obs_dyn1.rad;
        obs_for_sapf.active = true;

        [psi_sapf1, v_sapf1, F_rep1] = sapf_local_planner(pos, psi, p_goal_local, ...
            obs_for_sapf, u_cmd_base, sapf);
        F_rep_k = F_rep_k + F_rep1;

        d_obs1 = meas1.dist - obs_dyn1.rad;
        if d_obs1 < sapf.R_infl
            w1  = (1 - (d_obs1 / sapf.R_infl))^2;
            w1  = max(0, min(1, w1));
            psi_des    = wrap_angle(psi_ilos + w1 * wrap_angle(psi_sapf1 - psi_ilos));
            u_cmd_base = v_sapf1;
        end
    end

    % Obstacle dinamis 2
    if seen2
        obs_for_sapf        = struct();
        obs_for_sapf.pos    = meas2.pos_est;
        obs_for_sapf.rad    = obs_dyn2.rad;
        obs_for_sapf.active = true;

        [psi_sapf2, v_sapf2, F_rep2] = sapf_local_planner(pos, psi, p_goal_local, ...
            obs_for_sapf, u_cmd_base, sapf);
        F_rep_k = F_rep_k + F_rep2;

        d_obs2 = meas2.dist - obs_dyn2.rad;
        if d_obs2 < sapf.R_infl
            w2  = (1 - (d_obs2 / sapf.R_infl))^2;
            w2  = max(0, min(1, w2));
            psi_des    = wrap_angle(psi_des + w2 * wrap_angle(psi_sapf2 - psi_des));
            u_cmd_base = min(u_cmd_base, v_sapf2);
        end
    end

    % Heading SAPF & titik lookahead sudah dicatat di LOG di bawah

    %% ================================================================
    %% PID HEADING
    %% ================================================================
    e_psi   = wrap_angle(psi_des - psi);
    e_psi_c = max(-deg2rad(20), min(deg2rad(20), e_psi));

    r_now = nu(3);
    [r_pid, pid_integral, pid_prev_err, pid_filt_rate] = pid_update_with_rate( ...
        e_psi_c, r_now, pid_integral, pid_prev_err, pid_filt_rate, ...
        kp_psi, ki_psi, kd_psi, dt);

    r_cmd = max(-1.0, min(1.0, r_pid)) * fade_in * RUDDER_MAX;

    %% ================================================================
    %% SPEED COMMAND
    %% ================================================================
    cos_factor = max(0.15, cos(e_psi));
    u_cmd      = u_cmd_base * min(1.0, d_goal / goal_tol) * fade_in * cos_factor;
    u_cmd_g    = u_cmd / cell_m;

    %% ================================================================
    %% FORCES
    %% ================================================================
    Fx = Ku * (u_cmd_g - nu(1));
    Fx = max(-lim_TX, min(lim_TX, Fx));

    Fy = Kr * (r_cmd - r_now) - Kd_r * r_now;
    Fy = max(-lim_TY, min(lim_TY, Fy));

    %% ================================================================
    %% BANKING CONTROL
    %% ================================================================
    U_phys  = max(0.3, norm(nu(1:2)) * cell_m);
    a_y_cmd = U_phys * r_cmd;
    phi_cmd = 5.0 * atan(a_y_cmd / g_accel) + 0.3 * phi;
    phi_cmd = max(-phi_max, min(phi_max, phi_cmd));

    alpha_phi    = dt / (tau_phi + dt);
    phi_des      = phi_des_prev + alpha_phi * (phi_cmd - phi_des_prev);
    phi_des_prev = phi_des;

    e_phi    = phi_des - phi;
    eInt_phi = eInt_phi + e_phi * dt;

    Tk = Kphi_p * e_phi + Kphi_i * eInt_phi - Kphi_d * nu(4);
    Tk = max(-lim_TK, min(lim_TK, Tk));

    %% ================================================================
    %% DYNAMICS
    %% ================================================================
    u = nu(1); v = nu(2); r = nu(3); p = nu(4);

    udot = A1*v*r + A2*u + A3*abs(u)*u + A4*(abs(u)^2)*u + A18*Fx;

    vdot = -(1/A1)*u*r + A5*v + A6*abs(v)*v + A7*(abs(v)^2)*v ...
           + A8*abs(r)*v + A9*abs(v)*r;

    rdot = -A10*v*u + A11*u*v + A12*r + A13*abs(r)*r + A14*(abs(r)^2)*r ...
           + A15*abs(r)*u + A16*abs(u)*r + A17*abs(u)*u ...
           + A20*abs(r)*u + A21*abs(u)*r + A22*abs(u)*u + A19*Fy;

    pdot = -KpLin*p - KpAbs*abs(p)*p - KpCub*(abs(p)^2)*p ...
           - Kphi*sin(phi) + Kfy*Fy + Kv_phi*v + Kr_phi*r + Kbias;

    xdot_g = u*cos(psi) - v*sin(psi);
    ydot_g = u*sin(psi) + v*cos(psi);

    %% ================================================================
    %% EULER INTEGRATION
    %% ================================================================
    nu  = nu + dt * [udot; vdot; rdot; pdot];
    x_g = x_g + dt * xdot_g;
    y_g = y_g + dt * ydot_g;
    psi = psi + dt * nu(3);
    phi = phi + dt * nu(4);

    nu(1) = max(-u_max_g, min(u_max_g, nu(1)));
    nu(2) = max(-v_max_g, min(v_max_g, nu(2)));
    nu(3) = max(-r_max,   min(r_max,   nu(3)));
    nu(4) = max(-p_max,   min(p_max,   nu(4)));
    psi   = wrap_angle(psi);

    if any(~isfinite([x_g, y_g, psi, phi, nu']))
        warning('State non-finite di t=%.2f s!', t);
        nu(:) = 0;
        k_end = k;
        break;
    end

    %% ---- Reference point on path ----
    dist_to_path2 = vecnorm([path_smooth_m(:,1) - x_m, path_smooth_m(:,2) - y_m], 2, 2);
    [~, idx_ref] = min(dist_to_path2);
    x_ref = path_smooth_m(idx_ref, 1);
    y_ref = path_smooth_m(idx_ref, 2);

    %% ==== COLLISION CHECK (obstacle dinamis) ====
    d1 = norm(pos - obs_dyn1.pos);
    d2 = norm(pos - obs_dyn2.pos);
    if d1 < obs_dyn1.rad
        fprintf('COLLISION dengan obstacle dinamis 1 pada t=%.2f s (d=%.3f m)\n', t, d1);
    end
    if d2 < obs_dyn2.rad
        fprintf('COLLISION dengan obstacle dinamis 2 pada t=%.2f s (d=%.3f m)\n', t, d2);
    end

    %% ---- Logging ----
    % Col  1    : t
    % Col  2- 3 : x_m, y_m
    % Col  4- 5 : psi, phi
    % Col  6- 8 : u, v, r  [m/s, m/s, rad/s]
    % Col  9-10 : cte_val, psi_des
    % Col 11-12 : Fx, Fy
    % Col 13-14 : u_cmd, r_cmd
    % Col 15    : phi_des
    % Col 16-17 : x_ref, y_ref
    % Col 18-19 : d1, d2  (jarak ke obs dinamis)
    % Col 20-21 : seen1, seen2
    % Col 22-23 : psi_ilos, u_cmd_base
    % Col 24-25 : F_att_k [x y]
    % Col 26-27 : F_rep_k [x y]
    % Col 28-29 : F_tot   [x y]
    % Col 30-31 : p_goal_local [x y]
    LOG(k,:) = [t, x_m, y_m, psi, phi, ...
                nu(1)*cell_m, nu(2)*cell_m, nu(3), ...
                cte_val, psi_des, Fx, Fy, u_cmd, r_cmd, ...
                phi_des, x_ref, y_ref, ...
                d1, d2, ...
                seen1, seen2, psi_ilos, u_cmd_base, ...
                F_att_k(1), F_att_k(2), ...
                F_rep_k(1), F_rep_k(2), ...
                F_att_k(1)+F_rep_k(1), F_att_k(2)+F_rep_k(2), ...
                p_goal_local(1), p_goal_local(2)];

    if mod(k, 500) == 0
        fprintf('t=%5.1f s | (%.1f, %.1f) m | u=%.2f m/s | ψ=%5.1f° | d_goal=%.1f m\n', ...
            t, x_m, y_m, nu(1)*cell_m, rad2deg(psi), d_goal);
    end
end

%% ===================================================================
%% 6. POTONG LOG & EKSTRAK DATA
%% ===================================================================
LOG        = LOG(1:k_end, :);
LOG_replan = LOG_replan(1:k_end);
obs_dyn_log = obs_dyn_log(1:k_end,:);
camera_log  = camera_log(1:k_end,:);
F_att_log  = F_att_log(1:k_end,:);
F_rep_log  = F_rep_log(1:k_end,:);
F_tot_log  = F_tot_log(1:k_end,:);
pgoal_log  = pgoal_log(1:k_end,:);

% Isi dari kolom LOG (sudah tersimpan di sana)
F_att_log(:,1) = LOG(:,24);  F_att_log(:,2) = LOG(:,25);
F_rep_log(:,1) = LOG(:,26);  F_rep_log(:,2) = LOG(:,27);
F_tot_log(:,1) = LOG(:,28);  F_tot_log(:,2) = LOG(:,29);
pgoal_log(:,1) = LOG(:,30);  pgoal_log(:,2) = LOG(:,31);

t_log    = LOG(:,1);
x_log    = LOG(:,2);  y_log    = LOG(:,3);
psi_log  = LOG(:,4);  phi_log  = LOG(:,5);
u_log    = LOG(:,6);  v_log    = LOG(:,7);   r_log    = LOG(:,8);
cte_log  = LOG(:,9);  psid_log = LOG(:,10);
Fx_log   = LOG(:,11); Fy_log   = LOG(:,12);
ucmd_log = LOG(:,13); rcmd_log = LOG(:,14);
phid_log = LOG(:,15); xref_log = LOG(:,16);  yref_log = LOG(:,17);
dist_dyn1 = LOG(:,18);
dist_dyn2 = LOG(:,19);
seen1_log = LOG(:,20);
seen2_log = LOG(:,21);

replan_idx = find(LOG_replan, 1);

dist_to_obs = zeros(length(t_log), size(obstacles_static_m,1));
for k_obs = 1:size(obstacles_static_m,1)
    dist_to_obs(:,k_obs) = sqrt((x_log - obstacles_static_m(k_obs,1)).^2 + ...
                                 (y_log - obstacles_static_m(k_obs,2)).^2);
end
safe_radius = obstacles_static_m(:,3)' + safe_dist_m;

%% ===================================================================
%% 7. VISUALISASI
%% ===================================================================
th_circ = linspace(0, 2*pi, 64)';
safeDist_show_m = 0.6;
th = linspace(0, 2*pi, 60);

%% --- Figure 1: Global Path ---
figure('Name','Fig 1: Global Path & Smoothing','Position',[50 50 900 550]);
clf; hold on; axis equal; grid on; box on;
xlabel('X [m]'); ylabel('Y [m]');
axis([0 mapSize_m(2) 0 mapSize_m(1)]);
title('Global Plan: D* Lite + G2-CBS C^2 Smoothing');

for k_obs = 1:size(obstacles_static_m,1)
    cx_o    = obstacles_static_m(k_obs,1);
    cy_o    = obstacles_static_m(k_obs,2);
    r_p     = obstacles_static_m(k_obs,3);
    r_guard = r_p + safeDist_show_m;
    fill(cx_o + r_p*cos(th), cy_o + r_p*sin(th), 'r','FaceAlpha',0.3,'EdgeColor','none');
    plot(cx_o + r_guard*cos(th), cy_o + r_guard*sin(th), 'r--','LineWidth',0.8);
end

hRaw  = plot(path_global_m(:,1), path_global_m(:,2), 'c--','LineWidth',1.2,'DisplayName','D* Lite (raw)');
hSmth = plot(path_smooth_init(:,1), path_smooth_init(:,2), 'k-','LineWidth',2.0,'DisplayName','Smoothed');
plot(start_m(1), start_m(2), 'yo','MarkerFaceColor','y','MarkerSize',9);
plot(waypoint_m(1), waypoint_m(2), 'mo','MarkerFaceColor','m','MarkerSize',9);
plot(goal_m(1), goal_m(2), 'ro','MarkerFaceColor','r','MarkerSize',9);
legend([hRaw hSmth],'Location','bestoutside');

%% --- Figure 2: Animated Trajectory ---
playback_speed = 5;
fps_target = 30;
frame_step = max(1, round(playback_speed / (fps_target * dt)));

figure('Name','Fig 2: Trajektori USV (Animated)','Position',[100 80 950 620]);
clf; hold on; axis equal; grid on; box on;
xlabel('X [m]'); ylabel('Y [m]');
axis([0 mapSize_m(2) 0 mapSize_m(1)]);
title(sprintf('Trajektori USV — SAPF (Dyn) + Camera-Replan (Statis) — %dx speed', playback_speed));

% Obstacle statis awal
for k_obs = 1:size(obstacles_static_m,1)
    cx_o    = obstacles_static_m(k_obs,1);
    cy_o    = obstacles_static_m(k_obs,2);
    r_p     = obstacles_static_m(k_obs,3);
    fill(cx_o + r_p*cos(th), cy_o + r_p*sin(th), 'r','FaceAlpha',0.3,'EdgeColor','none','HandleVisibility','off');
end

% Obstacle statis tambahan (awalnya tersembunyi, muncul saat terdeteksi)
if enable_extra_obstacle
    cx_e = extraObs_gt_m(1); cy_e = extraObs_gt_m(2); r_e = extraObs_gt_m(3);
    r_e_guard = r_e + safeDist_show_m;
    h_extra_fill  = fill(cx_e + r_e*cos(th), cy_e + r_e*sin(th), 'm','FaceAlpha',0.3,'EdgeColor','none','Visible','off','HandleVisibility','off');
    h_extra_guard = plot(cx_e + r_e_guard*cos(th), cy_e + r_e_guard*sin(th), 'm--','LineWidth',0.8,'Visible','off','HandleVisibility','off');
else
    h_extra_fill  = [];
    h_extra_guard = [];
end

plot(start_m(1), start_m(2), 'yo','MarkerFaceColor','y','MarkerSize',9);
plot(waypoint_m(1), waypoint_m(2), 'mo','MarkerFaceColor','m','MarkerSize',9);
plot(goal_m(1), goal_m(2), 'ro','MarkerFaceColor','r','MarkerSize',9);

h_trail_pre  = plot(NaN, NaN, 'b-', 'LineWidth',2,'DisplayName','Sebelum replan');
h_trail_post = plot(NaN, NaN, 'm-', 'LineWidth',2,'DisplayName','Setelah replan');

% Kapal
L_real    = 1.6; L_model = 1.8;
shipScale = L_real / L_model;
baseShip  = shipScale * [1.00 0.00; -0.80 0.45; -0.40 0.00; -0.80 -0.45];
tKapal    = hgtransform('Parent', gca);
patch('XData', baseShip(:,1), 'YData', baseShip(:,2), ...
      'FaceColor',[0 0.7 0],'EdgeColor','k','LineWidth',0.8,'Parent',tKapal,'HandleVisibility','off');
set(tKapal, 'Matrix', makehgtform('translate',[x_log(1) y_log(1) 0],'zrotate',psi_log(1)));

% FOV kamera (untuk SAPF — obstacle dinamis)
h_fov = patch('XData',NaN,'YData',NaN,'FaceColor',[0 0.5 1],'FaceAlpha',0.15,'EdgeColor','b','LineWidth',0.5,'HandleVisibility','off');

% Obstacle dinamis
h_dyn1 = rectangle('Position',[obs_dyn1.pos(1)-obs_size/2, obs_dyn1.pos(2)-obs_size/2, obs_size, obs_size],'FaceColor','r','EdgeColor','k');
h_dyn2 = rectangle('Position',[obs_dyn2.pos(1)-obs_size/2, obs_dyn2.pos(2)-obs_size/2, obs_size, obs_size],'FaceColor','b','EdgeColor','k');

% Marker deteksi obstacle dinamis
h_det1 = plot(NaN, NaN, 'ro','MarkerSize',12,'LineWidth',2,'HandleVisibility','off');
h_det2 = plot(NaN, NaN, 'bo','MarkerSize',12,'LineWidth',2,'HandleVisibility','off');

% Info box
hInfoBox = text(1, -1.5, '','FontSize',11,'FontWeight','bold', ...
    'BackgroundColor','w','EdgeColor','k','LineWidth',1.2,'VerticalAlignment','top');

%% ==== SETUP GAYA REPULSIF RADIAL DARI OBSTACLE ====
% Panah-panah kecil memancar radial keluar dari tiap obstacle dinamis.
% Panjang panah proporsional dengan magnitude gaya repulsif SAPF.
% Hanya muncul saat kapal dalam R_infl (terdeteksi kamera).

n_rep_arrows = 12;
rep_angles   = linspace(0, 2*pi*(1 - 1/n_rep_arrows), n_rep_arrows);
th_ri        = linspace(0, 2*pi, 60);

% Quiver handle untuk repulsif obstacle 1 (merah) dan obstacle 2 (biru)
h_rep1 = quiver(zeros(1,n_rep_arrows), zeros(1,n_rep_arrows), ...
                zeros(1,n_rep_arrows), zeros(1,n_rep_arrows), 0, ...
                'Color',[1.0 0.25 0.25],'LineWidth',1.8,'MaxHeadSize',0.6, ...
                'DisplayName','F_{rep} Obs1','Visible','off');
h_rep2 = quiver(zeros(1,n_rep_arrows), zeros(1,n_rep_arrows), ...
                zeros(1,n_rep_arrows), zeros(1,n_rep_arrows), 0, ...
                'Color',[0.25 0.45 1.0],'LineWidth',1.8,'MaxHeadSize',0.6, ...
                'DisplayName','F_{rep} Obs2','Visible','off');

% Lingkaran zona pengaruh SAPF (R_infl) — muncul bersamaan dengan panah
h_rinfl1 = plot(NaN, NaN, '--','Color',[1.0 0.6 0.6],'LineWidth',0.9, ...
    'HandleVisibility','off');
h_rinfl2 = plot(NaN, NaN, '--','Color',[0.6 0.7 1.0],'LineWidth',0.9, ...
    'HandleVisibility','off');

tic;
for k = 1:frame_step:length(t_log)
    set(tKapal, 'Matrix', makehgtform('translate',[x_log(k) y_log(k) 0],'zrotate',psi_log(k)));

    % FOV kamera
    fov_range  = camera.maxRange;
    fov_angles = linspace(psi_log(k) - camera.fov/2, psi_log(k) + camera.fov/2, 20);
    fov_x = [x_log(k), x_log(k) + fov_range*cos(fov_angles), x_log(k)];
    fov_y = [y_log(k), y_log(k) + fov_range*sin(fov_angles), y_log(k)];
    set(h_fov, 'XData', fov_x, 'YData', fov_y);

    % Posisi obstacle dinamis
    x1 = obs_dyn_log(k,1); y1 = obs_dyn_log(k,2);
    x2 = obs_dyn_log(k,3); y2 = obs_dyn_log(k,4);
    set(h_dyn1, 'Position', [x1-obs_size/2, y1-obs_size/2, obs_size, obs_size]);
    set(h_dyn2, 'Position', [x2-obs_size/2, y2-obs_size/2, obs_size, obs_size]);

    if seen1_log(k), set(h_det1,'XData',x1,'YData',y1,'Visible','on');
    else;            set(h_det1,'Visible','off'); end
    if seen2_log(k), set(h_det2,'XData',x2,'YData',y2,'Visible','on');
    else;            set(h_det2,'Visible','off'); end

    %% -- ANIMASI GAYA REPULSIF RADIAL --
    % Panah memancar dari permukaan obstacle ke luar.
    % Panjang panah = f(jarak kapal ke obstacle): makin dekat → makin panjang.

    % Obstacle 1
    if seen1_log(k)
        d_o1    = norm([x_log(k)-x1, y_log(k)-y1]);
        d_s1    = max(d_o1 - obs_dyn1.rad, 1e-3);

        if d_s1 < sapf.R_infl
            % Magnitude gaya repulsif SAPF di posisi kapal
            mag1     = sapf.eta * (1/d_s1 - 1/sapf.R_infl) / (d_s1^2);
            mag1     = max(0, mag1);
            arr_len1 = min(mag1 * 0.8, sapf.R_infl * 0.55);

            % Panah berasal dari permukaan obstacle
            ox1 = x1 + obs_dyn1.rad * cos(rep_angles);
            oy1 = y1 + obs_dyn1.rad * sin(rep_angles);

            set(h_rep1, 'XData', ox1, 'YData', oy1, ...
                        'UData', arr_len1 * cos(rep_angles), ...
                        'VData', arr_len1 * sin(rep_angles), 'Visible','on');
            set(h_rinfl1, 'XData', x1 + sapf.R_infl*cos(th_ri), ...
                          'YData', y1 + sapf.R_infl*sin(th_ri), 'Visible','on');
        else
            set(h_rep1,   'Visible','off');
            set(h_rinfl1, 'Visible','off');
        end
    else
        set(h_rep1,   'Visible','off');
        set(h_rinfl1, 'Visible','off');
    end

    % Obstacle 2
    if seen2_log(k)
        d_o2 = norm([x_log(k)-x2, y_log(k)-y2]);
        d_s2 = max(d_o2 - obs_dyn2.rad, 1e-3);

        if d_s2 < sapf.R_infl
            mag2     = sapf.eta * (1/d_s2 - 1/sapf.R_infl) / (d_s2^2);
            mag2     = max(0, mag2);
            arr_len2 = min(mag2 * 0.8, sapf.R_infl * 0.55);

            ox2 = x2 + obs_dyn2.rad * cos(rep_angles);
            oy2 = y2 + obs_dyn2.rad * sin(rep_angles);

            set(h_rep2, 'XData', ox2, 'YData', oy2, ...
                        'UData', arr_len2 * cos(rep_angles), ...
                        'VData', arr_len2 * sin(rep_angles), 'Visible','on');
            set(h_rinfl2, 'XData', x2 + sapf.R_infl*cos(th_ri), ...
                          'YData', y2 + sapf.R_infl*sin(th_ri), 'Visible','on');
        else
            set(h_rep2,   'Visible','off');
            set(h_rinfl2, 'Visible','off');
        end
    else
        set(h_rep2,   'Visible','off');
        set(h_rinfl2, 'Visible','off');
    end

    % Trail & obstacle statis tambahan
    if ~isempty(replan_idx) && k >= replan_idx
        set(h_trail_pre,  'XData', x_log(1:replan_idx),  'YData', y_log(1:replan_idx));
        set(h_trail_post, 'XData', x_log(replan_idx:k),  'YData', y_log(replan_idx:k));
        if enable_extra_obstacle
            set(h_extra_fill,  'Visible','on');
            set(h_extra_guard, 'Visible','on');
        end
    else
        set(h_trail_pre, 'XData', x_log(1:k), 'YData', y_log(1:k));
    end

    % Info box
    det_str = '';
    if seen1_log(k), det_str = [det_str ' Dyn1']; end
    if seen2_log(k), det_str = [det_str ' Dyn2']; end
    if isempty(det_str), det_str = 'None'; end
    set(hInfoBox, 'String', sprintf(' t=%.1fs | v=%.2f m/s | \\psi=%.1f\\circ | SAPF Det:%s', ...
        t_log(k), hypot(u_log(k), v_log(k)), rad2deg(psi_log(k)), det_str));

    target_t = t_log(k) / playback_speed;
    elapsed  = toc;
    if elapsed < target_t
        pause(target_t - elapsed);
    else
        drawnow limitrate;
    end
end

% Frame akhir
set(tKapal,'Matrix',makehgtform('translate',[x_log(end) y_log(end) 0],'zrotate',psi_log(end)));
if ~isempty(replan_idx)
    set(h_trail_pre,  'XData',x_log(1:replan_idx),  'YData',y_log(1:replan_idx));
    set(h_trail_post, 'XData',x_log(replan_idx:end), 'YData',y_log(replan_idx:end));
else
    set(h_trail_pre, 'XData',x_log, 'YData',y_log);
end
legend([h_trail_pre, h_trail_post, h_rep1, h_rep2], ...
    'Location','northwest','FontSize',8);

%% --- Figure 3: Jarak ke Obstacle Dinamis ---
figure('Name','Fig 3: Jarak ke Obstacle Dinamis','Position',[150 110 900 480]);
hold on; grid on; box on;

plot(t_log, dist_dyn1, 'r-','LineWidth',1.5,'DisplayName','Obstacle Dinamis 1');
plot(t_log, dist_dyn2, 'b-','LineWidth',1.5,'DisplayName','Obstacle Dinamis 2');

det1_idx = find(seen1_log);
det2_idx = find(seen2_log);
if ~isempty(det1_idx), scatter(t_log(det1_idx), dist_dyn1(det1_idx), 10,'r','filled','HandleVisibility','off'); end
if ~isempty(det2_idx), scatter(t_log(det2_idx), dist_dyn2(det2_idx), 10,'b','filled','HandleVisibility','off'); end

yline(obs_dyn1.rad, 'r--','LineWidth',1.2,'Label','Collision Radius');
yline(sapf.R_infl,  'k:', 'LineWidth',1.2,'Label','SAPF Influence');

if ~isempty(replan_idx)
    xline(t_log(replan_idx),'m--','LineWidth',1.2,'Label',sprintf('Replan t=%.1fs',t_log(replan_idx)),'LabelHorizontalAlignment','right');
end

xlabel('t [s]'); ylabel('Distance [m]');
title('Distance to Dynamic Obstacles (dots = camera detection active)');
legend('Location','northeast');

%% --- Figure 4: Speed & Heading ---
figure('Name','Fig 4: Speed & Heading','Position',[200 140 900 500]);

subplot(2,1,1);
hold on; grid on; box on;
plot(t_log, u_log,    'b-','LineWidth',1.5,'DisplayName','u actual');
plot(t_log, ucmd_log, 'k:','LineWidth',1.0,'DisplayName','u_{cmd}');
yline(u_des,'r--','LineWidth',1.2,'Label',sprintf('u_{des}=%.1f',u_des));
if ~isempty(replan_idx)
    xline(t_log(replan_idx),'m--','LineWidth',0.8,'HandleVisibility','off');
end
xlabel('t [s]'); ylabel('u [m/s]');
title('Surge Speed (dengan SAPF slowdown untuk obstacle dinamis)');
legend('Location','southeast');

subplot(2,1,2);
hold on; grid on; box on;
plot(t_log, rad2deg(psi_log),  'b-', 'LineWidth',1.5,'DisplayName','\psi actual');
plot(t_log, rad2deg(psid_log), 'r--','LineWidth',1.5,'DisplayName','\psi_d desired');
if ~isempty(replan_idx)
    xline(t_log(replan_idx),'m--','LineWidth',0.8,'HandleVisibility','off');
end
xlabel('t [s]'); ylabel('\psi [°]');
title('Heading (Yaw)');
legend('Location','northwest');

%% --- Figure 5: Path Comparison (hanya jika replan terjadi) ---
if enable_extra_obstacle && did_replan && ~isempty(path_smooth_replan)

    figure('Name','Fig 5: Perbandingan Path','Position',[300 200 950 580]);
    clf; hold on; axis equal; grid on; box on;
    xlabel('X [m]'); ylabel('Y [m]');
    axis([0 mapSize_m(2) 0 mapSize_m(1)]);
    title('Figure 5: Global Path Awal vs Setelah Replan (Camera-Triggered)');

    for k_obs = 1:size(obstacles_static_m,1)
        cx_o    = obstacles_static_m(k_obs,1); cy_o = obstacles_static_m(k_obs,2);
        r_p     = obstacles_static_m(k_obs,3);
        fill(cx_o + r_p*cos(th), cy_o + r_p*sin(th),'r','FaceAlpha',0.3,'EdgeColor','none','HandleVisibility','off');
        plot(cx_o + safe_dist_m*cos(th), cy_o + safe_dist_m*cos(th),'r--','LineWidth',0.8,'HandleVisibility','off');
    end

    % Obstacle statis tambahan
    fill(cx_e + r_e*cos(th), cy_e + r_e*sin(th),'m','FaceAlpha',0.4,'EdgeColor','none','HandleVisibility','off');
    plot(cx_e + (r_e+safeDist_show_m)*cos(th), cy_e + (r_e+safeDist_show_m)*sin(th),'m--','LineWidth',1.2,'HandleVisibility','off');
    text(cx_e, cy_e + r_e + 0.8,'Obs. Statis Baru','HorizontalAlignment','center','FontSize',8,'Color',[0.6 0 0.6],'FontWeight','bold');

    h_init_p   = plot(path_smooth_init(:,1),   path_smooth_init(:,2),   'k--','LineWidth',2.2,'DisplayName','Path awal');
    h_replan_p = plot(path_smooth_replan(:,1), path_smooth_replan(:,2), 'b-', 'LineWidth',2.2,'DisplayName','Path setelah replan');

    if ~isempty(replan_idx) && replan_idx > 1
        h_tpre  = plot(x_log(1:replan_idx),   y_log(1:replan_idx),  ':','Color',[0 0.45 0.74],'LineWidth',1.2,'DisplayName','Traj (sebelum replan)');
        h_tpost = plot(x_log(replan_idx:end),  y_log(replan_idx:end),':','Color',[0.85 0.33 0.1],'LineWidth',1.2,'DisplayName','Traj (setelah replan)');
    else
        h_tpre  = plot(x_log, y_log,':','Color',[0 0.45 0.74],'LineWidth',1.2,'DisplayName','Traj aktual');
        h_tpost = [];
    end

    if ~isempty(replan_idx)
        h_rp = plot(x_log(replan_idx), y_log(replan_idx),'kp','MarkerSize',14,'MarkerFaceColor','y', ...
            'DisplayName', sprintf('Titik replan (t=%.1f s)', t_log(replan_idx)));
    end

    plot(start_m(1),    start_m(2),    'yo','MarkerFaceColor','y','MarkerSize',10,'HandleVisibility','off');
    plot(waypoint_m(1), waypoint_m(2), 'mo','MarkerFaceColor','m','MarkerSize',10,'HandleVisibility','off');
    plot(goal_m(1),     goal_m(2),     'ro','MarkerFaceColor','r','MarkerSize',10,'HandleVisibility','off');
    text(start_m(1)+0.5,    start_m(2),    'Start',    'FontSize',8,'Color',[0.4 0.4 0]);
    text(waypoint_m(1)+0.5, waypoint_m(2), 'Waypoint', 'FontSize',8,'Color',[0.5 0 0.5]);
    text(goal_m(1)+0.5,     goal_m(2),     'Goal',     'FontSize',8,'Color',[0.7 0 0]);

    handles_leg = [h_init_p, h_replan_p, h_tpre];
    if ~isempty(h_tpost),    handles_leg = [handles_leg, h_tpost]; end
    if ~isempty(replan_idx), handles_leg = [handles_leg, h_rp];    end
    legend(handles_leg,'Location','bestoutside','FontSize',9);

else
    fprintf('Replan tidak terjadi (enable_extra_obstacle=false) — Fig 5 dilewati.\n');
end

%% ===================================================================
%% 8. RINGKASAN
%% ===================================================================
fprintf('\n========== Simulation Summary ==========\n');
fprintf('enable_extra_obstacle : %s\n', mat2str(enable_extra_obstacle));
fprintf('Replan terjadi        : %s\n', mat2str(did_replan));
if did_replan && ~isempty(replan_idx)
    fprintf('Waktu deteksi/replan  : %.2f s\n', t_log(replan_idx));
end
fprintf('Duration              : %.1f s\n', t_log(end));
fprintf('Mean surge            : %.3f m/s (target: %.1f m/s)\n', mean(u_log), u_des);
fprintf('Max surge             : %.3f m/s\n', max(u_log));
fprintf('Min dist to Dyn Obs1  : %.3f m (collision < %.2f m)\n', min(dist_dyn1), obs_dyn1.rad);
fprintf('Min dist to Dyn Obs2  : %.3f m (collision < %.2f m)\n', min(dist_dyn2), obs_dyn2.rad);
fprintf('Camera det rate Dyn1  : %.1f%%\n', 100*sum(seen1_log)/length(seen1_log));
fprintf('Camera det rate Dyn2  : %.1f%%\n', 100*sum(seen2_log)/length(seen2_log));
fprintf('Final distance        : %.3f m\n', norm([x_log(end) y_log(end)] - goal_m));
fprintf('=========================================\n');

%% ===================================================================
%%                      HELPER FUNCTIONS
%% ===================================================================

%% CAMERA DETECT STATIC OBSTACLE
% Digunakan khusus untuk mendeteksi obstacle STATIS tambahan.
% Tidak ada noise (model sederhana berbasis range dan FOV).
% Sebelum terdeteksi, planner tidak tahu obstacle ini ada.
function seen = camera_detect_static(pos, psi, obstacle, cam_static)
    seen = false;
    if any(isnan(obstacle)); return; end

    dx = obstacle(1) - pos(1);
    dy = obstacle(2) - pos(2);
    dist    = hypot(dx, dy);
    bearing = wrap_angle(atan2(dy, dx) - psi);

    seen = (dist <= cam_static.range) && (abs(bearing) <= cam_static.fov/2);
end

%% SAPF LOCAL PLANNER (untuk obstacle DINAMIS)
function [psi_ref, v_ref, F_rep_out] = sapf_local_planner(pos, psi, p_goal, obsDyn, v_ref_in, sapf) %#ok<INUSL>
    vec_g = p_goal - pos;
    d_g   = norm(vec_g) + 1e-9;
    F_att = sapf.zeta * (vec_g / d_g);

    F_rep = [0 0];

    if obsDyn.active
        r_vo    = obsDyn.pos - pos;
        d_o     = norm(r_vo) + 1e-9;
        d_s     = d_o - obsDyn.rad;
        d_s_eff = max(d_s, 1e-3);
        dir_rep = -r_vo / d_o;

        if d_s < sapf.R_infl
            mag_rep  = sapf.eta * (1/d_s_eff - 1/sapf.R_infl) / (d_s_eff^2);
            mag_rep  = max(mag_rep, 0);
            grad_rep = mag_rep * dir_rep;

            % Sudut rotasi vortex γ
            gamma = sapf_gamma(d_s, sapf.d_safe, sapf.d_vort);

            % Arah rotasi: obstacle di kanan jalur → belok kiri (+1, CCW)
            %              obstacle di kiri jalur  → belok kanan (-1, CW)
            to_goal = p_goal - pos;
            to_obs  = obsDyn.pos - pos;
            cross_z = to_goal(1)*to_obs(2) - to_goal(2)*to_obs(1);
            rot_sign = 1.0;
            if cross_z > 0; rot_sign = -1.0; end

            % R(γ) · grad_rep
            g = rot_sign * gamma;
            R = [cos(g), -sin(g); sin(g), cos(g)];
            F_rep = (R * grad_rep')';
        end
    end

    F_tot = F_att + F_rep;
    if norm(F_tot) < 1e-6; F_tot = F_att; end
    psi_ref   = atan2(F_tot(2), F_tot(1));
    F_rep_out = F_rep;

    v_ref = v_ref_in;
    if obsDyn.active
        r_vo    = obsDyn.pos - pos;
        d_o     = norm(r_vo) + 1e-9;
        d_s     = d_o - obsDyn.rad;
        d_s_eff = max(d_s, 1e-6);

        if d_s < sapf.R_infl
            s     = (d_s_eff - sapf.d_safe) / max(sapf.R_infl - sapf.d_safe, 1e-6);
            s     = max(0, min(1, s));
            scale = sapf.v_min_factor + (1 - sapf.v_min_factor)*s;
            v_ref = scale * v_ref_in;
        end
    end
    v_ref = max(sapf.v_min_factor*v_ref_in, min(sapf.v_max, v_ref));
end

%% SAPF GAMMA — sudut rotasi vortex berdasarkan jarak
function gamma = sapf_gamma(d_s, d_safe, d_vort)
    if d_s <= d_safe
        gamma = pi/2;
    elseif d_s >= d_vort
        gamma = 0.0;
    else
        d_rel = (d_s - d_safe) / (d_vort - d_safe);
        gamma = (pi/2) * (1 - d_rel);
    end
end

%% CAMERA DETECT (untuk obstacle DINAMIS — dengan noise)
function [seen, meas] = camera_detect(pos, psi, obsDyn, camera)
    meas = struct('dist',[], 'bearing',[], 'pos_est',[]);
    seen = false;

    if ~obsDyn.active; return; end

    r_vo = obsDyn.pos - pos;
    d    = norm(r_vo);
    if d < 1e-6; return; end
    if d < camera.minRange || d > camera.maxRange; return; end

    bearing_world = atan2(r_vo(2), r_vo(1));
    alpha = atan2(sin(bearing_world - psi), cos(bearing_world - psi));
    if abs(alpha) > camera.fov/2; return; end

    d_meas     = d + camera.sigma_r * randn();
    alpha_meas = alpha + camera.sigma_b * randn();

    bearing_est = psi + alpha_meas;
    pos_est = pos + d_meas * [cos(bearing_est), sin(bearing_est)];

    meas.dist    = d_meas;
    meas.bearing = alpha_meas;
    meas.pos_est = pos_est;
    seen = true;
end

%% D* LITE PATH PLANNING
function path_g = dstar_lite_plan(occ, nR, nC, start_g, goal_g, w)
    if nargin < 6, w = 1.52; end

    sx = max(1, min(nC, round(start_g(1))));
    sy = max(1, min(nR, round(start_g(2))));
    gx = max(1, min(nC, round(goal_g(1))));
    gy = max(1, min(nR, round(goal_g(2))));

    trav = ~occ;
    trav(sy, sx) = true;
    if ~trav(gy, gx)
        [gx, gy] = nearest_free_cell(occ, gx, gy, nC, nR);
    end

    sid = sub2ind([nR nC], sy, sx);
    gid = sub2ind([nR nC], gy, gx);

    DX = [ 1 -1  0  0  1 -1  1 -1];
    DY = [ 0  0  1 -1  1 -1 -1  1];
    DC = [ 1  1  1  1  sqrt(2) sqrt(2) sqrt(2) sqrt(2)];

    N   = nR * nC;
    g   = 1e12 * ones(N, 1);
    rhs = 1e12 * ones(N, 1);
    km  = 0;
    Uk  = zeros(0, 2);
    Ui  = zeros(0, 1);

    rhs(gid) = 0;
    [Uk, Ui] = dsl_push(Uk, Ui, gid, dsl_key(g, rhs, km, sid, gid, nR, nC, w));

    while ~isempty(Ui)
        k_top   = dsl_topkey(Uk);
        k_start = dsl_key(g, rhs, km, sid, sid, nR, nC, w);
        if ~dsl_less(k_top, k_start) && rhs(sid) == g(sid), break; end

        [k_old, u, Uk, Ui] = dsl_pop(Uk, Ui);
        k_now = dsl_key(g, rhs, km, sid, u, nR, nC, w);

        if dsl_less(k_old, k_now)
            [Uk, Ui] = dsl_push(Uk, Ui, u, k_now);
        elseif g(u) > rhs(u)
            g(u) = rhs(u);
            nb = dsl_succ(u, trav, nR, nC, DX, DY, DC);
            for ii = 1:size(nb, 1)
                [g, rhs, Uk, Ui] = dsl_upv(nb(ii,1), gid, sid, g, rhs, km, Uk, Ui, trav, nR, nC, DX, DY, DC, w);
            end
        else
            g(u) = 1e12;
            nb = dsl_succ(u, trav, nR, nC, DX, DY, DC);
            [g, rhs, Uk, Ui] = dsl_upv(u, gid, sid, g, rhs, km, Uk, Ui, trav, nR, nC, DX, DY, DC, w);
            for ii = 1:size(nb, 1)
                [g, rhs, Uk, Ui] = dsl_upv(nb(ii,1), gid, sid, g, rhs, km, Uk, Ui, trav, nR, nC, DX, DY, DC, w);
            end
        end
    end

    if isinf(g(sid)), path_g = []; return; end
    cur    = sid;
    path_g = [sx, sy];
    for step = 1:N
        if cur == gid, break; end
        nb = dsl_succ(cur, trav, nR, nC, DX, DY, DC);
        if isempty(nb), path_g = []; return; end
        [~, idx] = min(nb(:,2) + g(nb(:,1)));
        cur = nb(idx, 1);
        [yy, xx] = ind2sub([nR nC], cur);
        path_g(end+1,:) = [xx, yy]; %#ok<AGROW>
    end
    if cur ~= gid, path_g = []; end
end

function [g, rhs, Uk, Ui] = dsl_upv(u, gid, sid, g, rhs, km, Uk, Ui, trav, nR, nC, DX, DY, DC, w)
    if u ~= gid
        nb = dsl_succ(u, trav, nR, nC, DX, DY, DC);
        if isempty(nb); rhs(u) = 1e12;
        else;           rhs(u) = min(nb(:,2) + g(nb(:,1))); end
    end
    keep = (Ui ~= u);
    Uk = Uk(keep, :); Ui = Ui(keep);
    if g(u) ~= rhs(u)
        [Uk, Ui] = dsl_push(Uk, Ui, u, dsl_key(g, rhs, km, sid, u, nR, nC, w));
    end
end

function K = dsl_key(g, rhs, km, sid, s, nR, nC, w)
    m = min(g(s), rhs(s));
    [sy, sx] = ind2sub([nR nC], sid);
    [vy, vx] = ind2sub([nR nC], s);
    K = [m + w * hypot(double(sx-vx), double(sy-vy)) + km, m];
end

function res = dsl_less(k1, k2)
    res = k1(1) < k2(1) - 1e-10 || ...
          (abs(k1(1) - k2(1)) < 1e-10 && k1(2) < k2(2) - 1e-10);
end

function k = dsl_topkey(Uk)
    [~, ord] = sortrows(Uk, [1 2]);
    k = Uk(ord(1), :);
end

function [k, u, Uk, Ui] = dsl_pop(Uk, Ui)
    [~, ord] = sortrows(Uk, [1 2]);
    k = Uk(ord(1), :); u = Ui(ord(1));
    Uk(ord(1), :) = []; Ui(ord(1)) = [];
end

function [Uk, Ui] = dsl_push(Uk, Ui, s, k)
    Uk(end+1, :) = k; Ui(end+1) = s;
end

function nb = dsl_succ(s, trav, nR, nC, DX, DY, DC)
    [y, x] = ind2sub([nR nC], s);
    nb = zeros(0, 2);
    for i = 1:8
        nx = x + DX(i); ny = y + DY(i);
        if nx>=1 && nx<=nC && ny>=1 && ny<=nR && trav(ny, nx)
            nb(end+1,:) = [sub2ind([nR nC], ny, nx), DC(i)]; %#ok<AGROW>
        end
    end
end

function [gx, gy] = nearest_free_cell(occ, gx0, gy0, nC, nR)
    for rr = 1:max(nC, nR)
        for dx = -rr:rr
            for dy = -rr:rr
                nx = gx0+dx; ny = gy0+dy;
                if nx>=1 && nx<=nC && ny>=1 && ny<=nR && ~occ(ny,nx)
                    gx = nx; gy = ny; return;
                end
            end
        end
    end
    gx = gx0; gy = gy0;
end

%% G2-CBS C² PATH SMOOTHING
function Ps = smooth_path_g2cbs(path_xy, n_per_seg, eps_rdp)
    P = rm_dups(path_xy);
    if size(P,1) <= 2; Ps = P; return; end
    if eps_rdp > 0 && size(P,1) > 3
        P = rdp_simplify(P, eps_rdp);
        P = rm_dups(P);
        if size(P,1) <= 2; Ps = P; return; end
    end
    N     = size(P,1);
    t_arc = [0; cumsum(vecnorm(diff(P), 2, 2))];
    if t_arc(end) < 1e-9; Ps = P(1,:); return; end
    h  = diff(t_arc);
    Mx = spline_second_deriv(t_arc, P(:,1));
    My = spline_second_deriv(t_arc, P(:,2));
    sx = diff(P(:,1)) ./ h; sy = diff(P(:,2)) ./ h;
    mx = zeros(N,1); my = zeros(N,1);
    mx(1) = sx(1) - h(1)*(2*Mx(1)+Mx(2))/6;
    my(1) = sy(1) - h(1)*(2*My(1)+My(2))/6;
    for ii = 2:N-1
        mx(ii) = 0.5*((sx(ii-1)+h(ii-1)*(Mx(ii-1)+2*Mx(ii))/6) + (sx(ii)-h(ii)*(2*Mx(ii)+Mx(ii+1))/6));
        my(ii) = 0.5*((sy(ii-1)+h(ii-1)*(My(ii-1)+2*My(ii))/6) + (sy(ii)-h(ii)*(2*My(ii)+My(ii+1))/6));
    end
    mx(N) = sx(N-1) + h(N-1)*(Mx(N-1)+2*Mx(N))/6;
    my(N) = sy(N-1) + h(N-1)*(My(N-1)+2*My(N))/6;
    Ps = P(1,:);
    for ii = 1:N-1
        hi = h(ii); b0 = P(ii,:); b3 = P(ii+1,:);
        b1 = b0 + (hi/3)*[mx(ii),   my(ii)];
        b2 = b3 - (hi/3)*[mx(ii+1), my(ii+1)];
        tau = linspace(0,1,n_per_seg)';
        B   = (1-tau).^3.*b0 + 3*(1-tau).^2.*tau.*b1 + 3*(1-tau).*tau.^2.*b2 + tau.^3.*b3;
        Ps  = [Ps; B(2:end,:)]; %#ok<AGROW>
    end
    Ps = rm_dups(Ps);
end

function M = spline_second_deriv(t, y)
    N = length(y);
    if N <= 2; M = zeros(N,1); return; end
    h = diff(t);
    a = zeros(N,1); b = ones(N,1); c = zeros(N,1); d = zeros(N,1);
    for ii = 2:N-1
        a(ii) = h(ii-1); b(ii) = 2*(h(ii-1)+h(ii)); c(ii) = h(ii);
        d(ii) = 6*((y(ii+1)-y(ii))/h(ii) - (y(ii)-y(ii-1))/h(ii-1));
    end
    for ii = 2:N
        if abs(b(ii-1)) < 1e-14, continue; end
        mv = a(ii)/b(ii-1); b(ii) = b(ii)-mv*c(ii-1); d(ii) = d(ii)-mv*d(ii-1);
    end
    M = zeros(N,1);
    if abs(b(N)) > 1e-14; M(N) = d(N)/b(N); end
    for ii = N-1:-1:1
        if abs(b(ii)) > 1e-14; M(ii) = (d(ii)-c(ii)*M(ii+1))/b(ii); end
    end
end

%% CLEARANCE ENFORCEMENT
function [P, info] = enforce_clearance(path_xy, obstacles_m, safe_dist_m, max_iter, gain, max_step, lam, ds)
    if nargin < 4; max_iter = 80;   end
    if nargin < 5; gain     = 0.6;  end
    if nargin < 6; max_step = 0.2;  end
    if nargin < 7; lam      = 0.15; end
    if nargin < 8; ds       = 0.5;  end
    P   = resample_path(path_xy, ds);
    N   = size(P,1); obs = obstacles_m;
    if N <= 2 || isempty(obs); info.min_clearance = inf; info.iterations = 0; return; end
    it = 0;
    for it = 1:max_iter
        dP = zeros(N,2); vio = false;
        for ii = 2:N-1
            push = [0 0];
            for ko = 1:size(obs,1)
                v_ob = P(ii,:)-obs(ko,1:2); d_ob = norm(v_ob); R = obs(ko,3)+safe_dist_m;
                if d_ob < R && d_ob > 1e-9; push = push+gain*(R-d_ob)*(v_ob/d_ob); vio = true; end
            end
            sm = lam*((P(ii-1,:)+P(ii+1,:))/2-P(ii,:)); delta = push+sm;
            nm = norm(delta); if nm > max_step; delta = delta*(max_step/nm); end
            dP(ii,:) = delta;
        end
        P(2:end-1,:) = P(2:end-1,:) + dP(2:end-1,:);
        if ~vio; break; end
    end
    P = rm_dups(P); mc = inf;
    for ko = 1:size(obs,1)
        d = vecnorm(P-obs(ko,1:2),2,2)-(obs(ko,3)+safe_dist_m); mc = min(mc,min(d));
    end
    info.min_clearance = double(mc); info.iterations = it;
end

%% ILOS GUIDANCE
function [psi_d, sigma, psi_d_filt, e_y] = ilos_compute(pos, path, sigma, psi_d_filt, dt, k_int, lookahead, cte_thresh, la_k, psi_tau)
    e_y = 0;
    if size(path,1) < 2; psi_d = psi_d_filt; return; end
    seg_idx = ilos_find_segment(pos, path);
    p1 = path(seg_idx,:); p2 = path(min(seg_idx+1,size(path,1)),:);
    seg_vec = p2-p1; seg_len = norm(seg_vec);
    if seg_len < 1e-6; psi_d = psi_d_filt; return; end
    alpha_k  = atan2(seg_vec(2), seg_vec(1));
    seg_norm = [-seg_vec(2), seg_vec(1)] / seg_len;
    e_y      = dot(pos-p1, seg_norm);
    delta_eff = max(lookahead, abs(e_y)*la_k);
    if abs(e_y) > cte_thresh; sigma = 0.0; end
    nu_val = e_y + k_int*sigma;
    denom  = delta_eff^2 + nu_val^2;
    sigma  = sigma + dt*delta_eff*e_y/denom;
    nu_val    = e_y + k_int*sigma;
    psi_d_raw = alpha_k + atan2(-nu_val, delta_eff);
    alpha_f    = min(1.0, dt/psi_tau);
    psi_d_filt = psi_d_filt + alpha_f*wrap_angle(psi_d_raw - psi_d_filt);
    psi_d      = wrap_angle(psi_d_filt);
end

function idx = ilos_find_segment(pos, path)
    min_d = inf; idx = 1;
    for ii = 1:size(path,1)-1
        p1 = path(ii,:); p2 = path(ii+1,:); seg = p2-p1;
        L2 = max(1e-12, dot(seg,seg)); tt = max(0,min(1,dot(pos-p1,seg)/L2));
        d  = norm(pos-(p1+tt*seg));
        if d < min_d; min_d = d; idx = ii; end
    end
end

%% PID WITH RATE FEEDBACK
function [out, integral, prev_err, filt_rate] = pid_update_with_rate(error, measured_rate, integral, prev_err, filt_rate, kp, ki, kd, dt)
    p_term = kp * error;
    if error * prev_err < 0; integral = integral * 0.5; end
    integral  = integral + error * dt;
    integral  = max(-1.0, min(1.0, integral));
    i_term    = ki * integral;
    filt_rate = filt_rate + 0.8*(measured_rate - filt_rate);
    d_term    = -kd * filt_rate;
    prev_err  = error;
    out       = p_term + i_term + d_term;
end

%% UTILITIES
function P = rm_dups(P, tol)
    if nargin < 2; tol = 1e-8; end
    if isempty(P); return; end
    keep = [true; vecnorm(diff(P),2,2) > tol]; P = P(keep,:);
end

function P = resample_path(P, ds)
    if size(P,1) < 2 || ds <= 0; return; end
    segs = vecnorm(diff(P),2,2); s = [0; cumsum(segs)];
    ss = (0:ds:s(end))';
    if isempty(ss) || ss(end) < s(end); ss(end+1) = s(end); end
    P = [interp1(s,P(:,1),ss), interp1(s,P(:,2),ss)];
end

function P = rdp_simplify(P, eps)
    if size(P,1) <= 2; return; end
    A = P(1,:); B = P(end,:); AB = B-A; L2 = max(1e-12,dot(AB,AB));
    dists = zeros(size(P,1),1);
    for ii = 2:size(P,1)-1
        tt = max(0,min(1,dot(P(ii,:)-A,AB)/L2)); dists(ii) = norm(P(ii,:)-(A+tt*AB));
    end
    [dm, ix] = max(dists);
    if dm > eps
        L = rdp_simplify(P(1:ix,:),eps); R = rdp_simplify(P(ix:end,:),eps); P = [L(1:end-1,:); R];
    else
        P = [A; B];
    end
end

function a = wrap_angle(a)
    a = mod(a + pi, 2*pi) - pi;
end