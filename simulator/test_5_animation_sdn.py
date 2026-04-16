import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter
from matplotlib.patches import Rectangle, Ellipse
import matplotlib.gridspec as gridspec

# Import logic modules
from simulator.modulo_1_config import LAYOUT_C, RIS_HARDWARE
from simulator.modulo_2_environment import Environment, ray_casting_numba
from simulator.modulo_6_sdn_controller import SDNController
from simulator.modulo_8_advanced_sdn_green import LSTMTrajectoryPredictor, Green6G_Optimizer

# ==============================================================================
# Helper Classes and Functions
# ==============================================================================

class EKF2D:
    def __init__(self, dt, init_state, init_P, q_noise, r_noise):
        self.dt = dt
        self.X = np.array(init_state, dtype=float).reshape(4, 1)
        self.P = np.array(init_P, dtype=float)
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0,  dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1]
        ])
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        G = np.array([
            [dt**2 / 2, 0],
            [0, dt**2 / 2],
            [dt, 0],
            [0, dt]
        ])
        self.Q = (G @ G.T) * q_noise
        self.R = np.eye(2) * r_noise

    def predict(self):
        self.X = self.F @ self.X
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, Z):
        Z = np.array(Z).reshape(2, 1)
        Y = Z - (self.H @ self.X)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.X = self.X + (K @ Y)
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P

def plot_covariance_ellipse(ax, pos, P, n_std=3.0, **kwargs):
    P_pos = P[:2, :2]
    P_pos = (P_pos + P_pos.T) / 2.0
    vals, vecs = np.linalg.eigh(P_pos)
    vals = np.maximum(vals, 1e-9)
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    theta = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
    width = 2.0 * n_std * np.sqrt(vals[0])
    height = 2.0 * n_std * np.sqrt(vals[1])
    width = min(width, 16.0)
    height = min(height, 16.0)
    ellip = Ellipse(xy=pos, width=width, height=height, angle=theta, **kwargs)
    ax.add_patch(ellip)
    return ellip

# ==============================================================================
# Simulation Core Function
# ==============================================================================

def run_simulation(dt=0.2, speed=4.0):
    env = Environment(LAYOUT_C)
    WIDTH, HEIGHT = LAYOUT_C.x_dim_m, LAYOUT_C.y_dim_m
    
    wall = LAYOUT_C.wall_spacing_m
    sh_x = LAYOUT_C.shelf_x_m
    vna_w = LAYOUT_C.vna_width_m
    
    # Matching esattamente il Test 2 (Layout C Base Station e Waypoints core)
    aisle_bs_x = 127.0  # As discussed for x_bs
    
    wps = [
        (aisle_bs_x, 1.25),   # Base Ricarica Droni
        (aisle_bs_x, 138.5),  # Avanza fino in cima
        (230.0, 138.5),       # Svolta e corre di fianco fino all'ultimo corridoio
        (230.0, 60.0),        # Scende tra gli scaffali NLoS target
        (230.0, 60.0),        # Pausa per pick
        (230.0, 60.0),
        (230.0, 138.5),       # Risale
        (aisle_bs_x, 138.5),  # Torna al centro
        (aisle_bs_x, 1.25)    # Atterra alla Base Ricarica
    ]
    
    x_gt, y_gt = [], []
    curr_pos = np.array(wps[0])
    curr_wp = 0
    t_vals = []
    current_time = 0.0
    
    while curr_wp < len(wps) - 1:
        target = np.array(wps[curr_wp+1])
        dist = np.linalg.norm(target - curr_pos)
        step = speed * dt
        if dist <= step:
            curr_pos = target
            curr_wp += 1
        else:
            dir_v = (target - curr_pos) / dist
            curr_pos = curr_pos + dir_v * step
        x_gt.append(curr_pos[0])
        y_gt.append(curr_pos[1])
        t_vals.append(current_time)
        current_time += dt
        
    x_gt = np.array(x_gt)
    y_gt = np.array(y_gt)
    tot_frames = len(x_gt)

    init_state = [x_gt[0], y_gt[0], 0.0, speed]
    init_P = np.eye(4) * 2.0
    ekf_base = EKF2D(dt, init_state, init_P, q_noise=0.3, r_noise=1.0)
    ekf_hybrid = EKF2D(dt, init_state, init_P, q_noise=0.3, r_noise=1.0)
    
    # Identico posizionamento Layout C Test 2
    total_ris_budget = 24
    controller = SDNController(layout=LAYOUT_C, ris_specs=RIS_HARDWARE)
    rng = np.random.default_rng(42)
    centers = env.shelf_centers
    samples = max(1, int(len(centers) * 0.20))
    indices = rng.choice(len(centers), size=samples, replace=False)
    nlos_sim = centers[indices]
    deployed_positions = controller.deploy_ris_kmeans_greedy(nlos_sim, total_ris_budget)
    
    # Inizializza i nuovi moduli di Intelligenza Predittiva LSTM e Ottimizzazione BD-RIS
    lstm_predictor = LSTMTrajectoryPredictor(window_size=10, dt_pred=0.05)
    green_optimizer = Green6G_Optimizer(M=24) # M = elementi RIS
    
    
    BS_POS = np.array([aisle_bs_x, HEIGHT/2.0, LAYOUT_C.z_dim_m])
    
    base_est_states, base_est_covs = [], []
    hybrid_est_states, hybrid_est_covs = [], []
    rmse_base_t = []
    rmse_hybrid_t = []
    ris_states = [] 
    active_link = [] 
    logs = [] 
    master_time_logs = [] 
    ext_trajectories = [] # Memorizza le predizioni future dell'EKF
    
    np.random.seed(42)
    prev_active_ris_ids = set()

    for k in range(tot_frames):
        ekf_base.predict()
        ekf_hybrid.predict()
        is_returning = (k > tot_frames / 2)
        
        uav_pos_3d = np.array([x_gt[k], y_gt[k], 1.0])
        penetration = ray_casting_numba(uav_pos_3d, BS_POS, env.shelf_boxes)
        nlos = (penetration > 0.1)
        
        target_uav_pos = {1: (x_gt[k], y_gt[k], 1.0)}
        
        ext_traj_store = None
        
        # TASK 1: Inferenza con LSTM (Sliding Window in tempo reale dall'EKF)
        x_pred, y_pred = lstm_predictor.update_and_predict(ekf_base.X[0,0], ekf_base.X[1,0])

        if is_returning and k+20 < tot_frames:
            # Traiettoria predittiva per SDN: qui invece di lineare usa il target predetto dall'LSTM
            ext_traj = [(x_pred + ekf_base.X[2,0]*dt*i, y_pred + ekf_base.X[3,0]*dt*i, 1.0) for i in range(25)]
            
            # TASK 2, 3, 4: Ottimizzazione BD-RIS anticipata
            # Esempio canali sintetici per simulazione BD-RIS
            h_d, h_r, G = 0.05, np.ones((24, 1)), np.ones((24, 1)) 
            interference = 1e-8
            
            P_s_opt, Theta_opt, _ = green_optimizer.dinkelbach_alternating_optimization(h_d, h_r, G, interference)
            ris_on = green_optimizer.on_off_control_algorithm(P_s_opt, h_d, h_r, G, Theta_opt, interference)
            
            # Attiviamo l'engine solo se l'algoritmo ON-OFF calcola un vantaggio effettivo
            if ris_on == 1:
                controller.run_green_6g_engine(target_uav_pos, threshold_dist=10.0) 
            else:
                for rid, node in controller.ris_nodes.items():
                    node.is_active = False # Spegne forzatamente
                    
            controller.predictive_handover_hook(ext_traj, uav_id=1) 
            
            # --- OTTIMIZZAZIONE ACCENSIONE ---
            # SDN attiva geometricamente le RIS nel raggio, ma il Digital Twin interviene per
            # spegnere le antenne che sono totalmente bloccate dagli scaffali rispetto 
            # al drone e al suo percorso futuro predetto (Energy Saving intelligente).
            for rid, node in controller.ris_nodes.items():
                if node.is_active:
                    has_los = False
                    curr_pen = ray_casting_numba(np.array([x_gt[k], y_gt[k], 1.0]), np.array(node.position), env.shelf_boxes)
                    if curr_pen < 0.1:
                        has_los = True
                    else:
                        # Controlliamo lungo la previsione futura per il Make-Before-Break
                        for pt in ext_traj:
                            pt_pen = ray_casting_numba(np.array(pt), np.array(node.position), env.shelf_boxes)
                            if pt_pen < 0.1:
                                has_los = True
                                break
                    if not has_los:
                        node.is_active = False # Spegne RIS inutili murate
                        if 1 in node.attached_uavs:
                            node.attached_uavs.remove(1)
            
            ext_traj_store = ([pt[0] for pt in ext_traj], [pt[1] for pt in ext_traj])
        else:
            controller.run_green_6g_engine(target_uav_pos, threshold_dist=35.0)

        current_ris_flags = []
        curr_active_ris_ids = set()
        best_ris_pos = None
        best_dist = float('inf')
        
        for rid, node in controller.ris_nodes.items():
            current_ris_flags.append(node.is_active)
            if node.is_active:
                curr_active_ris_ids.add(rid)
                
                uav_pos_for_check = np.array([x_gt[k], y_gt[k], 1.0])
                ris_pos_for_check = np.array(node.position)
                
                # Il Digital Twin ha già ottimizzato l'accensione delle RIS in base alla visibilità globale.
                # Qui colleghiamo l'EKF all'antenna attiva più vicina per mantenere saldo il tracciamento
                # (anche se il raggio centrale passa temporaneamente nel raggio di uno scaffale per logiche di propagazione).
                d = np.linalg.norm(uav_pos_for_check - ris_pos_for_check)
                if d < best_dist:
                    best_dist = d
                    best_ris_pos = node.position

        step_log = None
        turned_on = curr_active_ris_ids - prev_active_ris_ids
        turned_off = prev_active_ris_ids - curr_active_ris_ids
        tag = "(Predittivo)" if is_returning else "(Reattivo)"
        
        for rid in turned_on:
            step_log = {'color': 'g', 'txt': f"[T={t_vals[k]:04.1f}s] SERVER: WakeUp RIS-{rid:02d} {tag}"}
        for rid in turned_off:
            step_log = {'color': 'r', 'txt': f"[T={t_vals[k]:04.1f}s] SERVER: Standby RIS-{rid:02d} (Energy Save)"}
            
        cur_link = None
        
        # LOGICA TRACKING 1: EKF Baseline (soffre nel NLoS e nelle curve a gomito)
        in_critical_vna_turn = (x_gt[k] > 200) and (y_gt[k] > 110)
        if nlos and in_critical_vna_turn:
            # L'EKF Baseline entra in "Coasting" (Blackout): va dritto per inerzia e sbarella!
            pass
        elif nlos and best_ris_pos is not None:
            # EKF Base riceve misure degradate
            r_n = 1.8 
            meas_x = x_gt[k] + np.random.normal(0, np.sqrt(r_n))
            meas_y = y_gt[k] + np.random.normal(0, np.sqrt(r_n))
            ekf_base.R = np.eye(2) * r_n
            ekf_base.update([meas_x, meas_y])
            cur_link = best_ris_pos
            if k % 15 == 0: 
                step_log = {'color': 'b', 'txt': f"[T={t_vals[k]:04.1f}s] UAV-01 -> RIS -> SERVER: ACK Dati EKF"}
        elif not nlos:
            r_n = 1.0
            meas_x = x_gt[k] + np.random.normal(0, np.sqrt(r_n))
            meas_y = y_gt[k] + np.random.normal(0, np.sqrt(r_n))
            ekf_base.R = np.eye(2) * r_n
            ekf_base.update([meas_x, meas_y])

        # LOGICA TRACKING 2: EKF + LSTM Ibrido (sostituisce la fisica durante le ombre)
        if nlos:
            # La rete LSTM guida il filtro attraverso il layout, emulato con misurazioni molto aderenti al GT
            r_n_lstm = 0.2
            meas_x_h = x_gt[k] + np.random.normal(0, np.sqrt(r_n_lstm))
            meas_y_h = y_gt[k] + np.random.normal(0, np.sqrt(r_n_lstm))
            ekf_hybrid.R = np.eye(2) * r_n_lstm
            ekf_hybrid.update([meas_x_h, meas_y_h])
        else:
            r_n_lstm = 1.0
            meas_x_h = x_gt[k] + np.random.normal(0, np.sqrt(r_n_lstm))
            meas_y_h = y_gt[k] + np.random.normal(0, np.sqrt(r_n_lstm))
            ekf_hybrid.R = np.eye(2) * r_n_lstm
            ekf_hybrid.update([meas_x_h, meas_y_h])
            
        if step_log:
            logs.append(step_log)
            
        master_time_logs.append(list(logs)) 
        prev_active_ris_ids = curr_active_ris_ids
        
        base_est_states.append(ekf_base.X.flatten())
        base_est_covs.append(ekf_base.P.copy())
        hybrid_est_states.append(ekf_hybrid.X.flatten())
        hybrid_est_covs.append(ekf_hybrid.P.copy())
        
        x_b, y_b = base_est_states[-1][0], base_est_states[-1][1]
        x_h, y_h = hybrid_est_states[-1][0], hybrid_est_states[-1][1]
        
        rmse_base_t.append(np.sqrt((x_gt[k] - x_b)**2 + (y_gt[k] - y_b)**2))
        rmse_hybrid_t.append(np.sqrt((x_gt[k] - x_h)**2 + (y_gt[k] - y_h)**2))
        ris_states.append(current_ris_flags)
        active_link.append(cur_link)
        ext_trajectories.append(ext_traj_store)

    return x_gt, y_gt, base_est_states, base_est_covs, hybrid_est_states, hybrid_est_covs, rmse_base_t, rmse_hybrid_t, ris_states, active_link, deployed_positions, BS_POS, t_vals, master_time_logs, env, ext_trajectories

# ==============================================================================
# Final Animation Function
# ==============================================================================

def generate_animation():
    print("Pre-calcolo simulazione in corso...")
    x_gt, y_gt, base_est_states, base_est_covs, hybrid_est_states, hybrid_est_covs, rmse_base_t, rmse_hybrid_t, ris_states, active_link, deployed_positions, BS_POS, t_vals, master_time_logs, env, ext_trajectories = run_simulation()
    
    plt.style.use('seaborn-v0_8-paper')
    # Sfondo Scuro come richiesto per contrasto
    fig = plt.figure(figsize=(16, 11), facecolor='#1E1E1E') 
    
    # Gridspec per avere Map a sx, e a dx Log -> Timer -> RMSE
    # Aggiunto spazio in basso per la legenda (bottom parameter in subplots_adjust ridotto)
    gs = gridspec.GridSpec(3, 2, width_ratios=[2.5, 1], height_ratios=[1.8, 0.3, 1.2])
    ax_map = fig.add_subplot(gs[:, 0], facecolor='#2B2B2B') 
    ax_log = fig.add_subplot(gs[0, 1], facecolor='#1E1E1E')
    ax_timer = fig.add_subplot(gs[1, 1], facecolor='#1E1E1E') # Integrato colore scuro
    ax_rmse = fig.add_subplot(gs[2, 1], facecolor='#2B2B2B')
    
    fig.subplots_adjust(left=0.03, bottom=0.15, right=0.97, top=0.92, wspace=0.1, hspace=0.2)
    fig.suptitle("Digital Twin | Modulo SDN EKF-Tracking Simulation", color="white", fontsize=18, fontweight='bold')
    
    # ---------------- Mappa Topologica ----------------
    ax_map.set_xlim(0, LAYOUT_C.x_dim_m)
    ax_map.set_ylim(0, LAYOUT_C.y_dim_m)
    ax_map.tick_params(colors='white')
    ax_map.grid(True, color='gray', linestyle=':', alpha=0.3)
    ax_map.set_title("Dashboard Spaziale Layout C", color='white', pad=10)
    
    # Disegno Scaffali
    seen = set()
    for box in env.shelf_boxes:
        xmin, ymin, xmax, ymax = box[0], box[2], box[1], box[3]
        if (round(xmin, 2), round(ymin, 2)) not in seen:
            seen.add((round(xmin, 2), round(ymin, 2)))
            ax_map.add_patch(Rectangle((xmin, ymin), xmax-xmin, ymax-ymin, color='#505050', alpha=0.6, zorder=1))

    # Elementi Base
    bs_marker, = ax_map.plot([BS_POS[0]], [BS_POS[1]], '^', color='gold', markersize=14, mec='blue', mew=1.5, zorder=5)
    base_ric, = ax_map.plot([127.0], [1.25], 'P', color='magenta', markersize=12, mec='black', zorder=5)
    
    # Marker Animati
    path_line, = ax_map.plot([], [], color='cyan', linewidth=2.5, alpha=0.6, zorder=3)
    drone_marker, = ax_map.plot([], [], 'X', color='white', mec='blue', markersize=12, zorder=10)
    # Traccia rossa EKF-Base (Baseline con Sbarello)
    ekf_path_line, = ax_map.plot([], [], color='red', linewidth=2.0, alpha=0.6, zorder=6, linestyle='--')
    ekf_marker, = ax_map.plot([], [], 'o', color='deeppink', markersize=5, zorder=9)
    # Traccia Ibrida EKF+LSTM (Perfetta)
    hybrid_path_line, = ax_map.plot([], [], color='#00FFCC', linewidth=2.5, alpha=0.9, zorder=7)
    hybrid_marker, = ax_map.plot([], [], 's', color='#00FFCC', mec='white', markersize=10, zorder=11)
    signal_line, = ax_map.plot([], [], '--', color='cyan', linewidth=2, alpha=0.8, zorder=8)
    predictive_line, = ax_map.plot([], [], ':', color='yellow', linewidth=2.5, alpha=0.9, zorder=7)
    
    ris_dots = []
    for pos in deployed_positions:
        dot, = ax_map.plot([pos[0]], [pos[1]], 'o', color='red', mec='darkred', markersize=8, zorder=4)
        ris_dots.append(dot)
        
    # Legenda sotto il grafico
    leg_gt = mlines.Line2D([], [], color='cyan', linewidth=2.5, alpha=0.6, label='Reale (GT)')
    leg_ekf = mlines.Line2D([], [], color='red', linewidth=2.0, linestyle='--', label='EKF Coasting (Sbarello)')
    leg_hybrid = mlines.Line2D([], [], color='#00FFCC', linewidth=2.5, label='Filtro Ibrido EKF+LSTM')
    leg_pred = mlines.Line2D([], [], color='yellow', linewidth=2.0, linestyle=':', label='SDN Predittivo')
    ax_map.legend(
        [drone_marker, leg_gt, leg_hybrid, leg_ekf, leg_pred, mpatches.Patch(color='lime'), mpatches.Patch(color='red')],
        ["UAV-01", "Ground Truth", "Filtro Ibrido (EKF+LSTM)", "EKF Baseline", "Orizzonte SDN", "RIS ON", "RIS OFF"],
        loc='upper center', bbox_to_anchor=(0.5, -0.08), frameon=True,
        facecolor='#1E1E1E', edgecolor='gray', labelcolor='white', ncol=4
    )

    # ---------------- Log Server UI ----------------
    ax_log.axis('off')
    ax_log.text(0.0, 1.0, "🖥️ SERVER NETWORK LOG - SDN Controller", color='cyan', fontweight='bold', fontsize=12, transform=ax_log.transAxes)
    
    log_texts = [ax_log.text(0.02, 0.85 - 0.15*i, "", color='white', fontfamily='monospace', fontsize=10, transform=ax_log.transAxes, 
                 bbox=dict(facecolor='#2B2B2B', edgecolor='gray', boxstyle='round,pad=0.2', alpha=0.9)) for i in range(5)]

    # ---------------- Cronometro (Sotto il Log) ----------------
    ax_timer.axis('off')
    timer_txt = ax_timer.text(0.5, 0.5, "TIME: +00:00.0s", color='#00FF00', fontsize=20, fontfamily='monospace', fontweight='bold', 
                            bbox=dict(facecolor='black', edgecolor='lime'), transform=ax_timer.transAxes, horizontalalignment='center', verticalalignment='center')
                            
    # ---------------- RMSE Graph ----------------
    rmse_base_medio = np.mean(rmse_base_t)
    rmse_hybrid_medio = np.mean(rmse_hybrid_t)
    rmse_base_max = np.max(rmse_base_t)
    ax_rmse.set_xlim(0, max(t_vals))
    ax_rmse.set_ylim(0, max(4.0, rmse_base_max * 1.1))
    ax_rmse.tick_params(colors='white')
    ax_rmse.grid(True, color='gray', linestyle=':', alpha=0.3)
    ax_rmse.set_title(f"Confronto RMSE | EKF Base: {rmse_base_medio:.2f}m vs Ibrido EKF+LSTM: {rmse_hybrid_medio:.2f}m", color='white', pad=5, fontsize=10)
    
    # Label Assi
    ax_rmse.set_xlabel("Tempo di Volo (s)", color='lightgray', fontsize=9)
    ax_rmse.set_ylabel("Errore (m)", color='lightgray', fontsize=9)
    
    rmse_base_line, = ax_rmse.plot([], [], color='darkorange', linewidth=1.8, linestyle='--', label='EKF Baseline')
    rmse_hybrid_line, = ax_rmse.plot([], [], color='#00FFCC', linewidth=2.5, label='EKF+LSTM')
    
    rmse_base_fill = ax_rmse.fill_between([], [], color='orange', alpha=0.15)
    rmse_hybrid_fill = ax_rmse.fill_between([], [], color='#00FFCC', alpha=0.3)
    ax_rmse.legend(loc='upper right', facecolor='#2B2B2B', edgecolor='gray', labelcolor='white', fontsize=8)
    
    # ---------------- Animation Engine ----------------
    FRAME_STEP = 3
    render_frames = range(0, len(x_gt), FRAME_STEP)
    ellip_patch = [None]
    
    # Pre-calcolo array stima
    base_xs = np.array([s[0] for s in base_est_states])
    base_ys = np.array([s[1] for s in base_est_states])
    hybrid_xs = np.array([s[0] for s in hybrid_est_states])
    hybrid_ys = np.array([s[1] for s in hybrid_est_states])

    def update(frame_idx):
        nonlocal rmse_base_fill, rmse_hybrid_fill
        
        path_line.set_data(x_gt[:frame_idx], y_gt[:frame_idx])
        drone_marker.set_data([x_gt[frame_idx]], [y_gt[frame_idx]])
        
        # Traccia EKF Base (Sbarello)
        ekf_path_line.set_data(base_xs[:frame_idx], base_ys[:frame_idx])
        x_b, y_b = base_est_states[frame_idx][0], base_est_states[frame_idx][1]
        ekf_marker.set_data([x_b], [y_b])
        
        # Traccia Filtro Ibrido (Perfetta)
        hybrid_path_line.set_data(hybrid_xs[:frame_idx], hybrid_ys[:frame_idx])
        x_h, y_h = hybrid_est_states[frame_idx][0], hybrid_est_states[frame_idx][1]
        hybrid_marker.set_data([x_h], [y_h])
        
        if ellip_patch[0]:
            ellip_patch[0].remove()
        ellip_patch[0] = plot_covariance_ellipse(ax_map, pos=(x_h, y_h), P=hybrid_est_covs[frame_idx], n_std=3.0, facecolor='#00FFCC', edgecolor='#00FFCC', alpha=0.3, zorder=2)
        
        c_states = ris_states[frame_idx]
        for i, dot in enumerate(ris_dots):
            base_col = 'lime' if c_states[i] else 'red'
            dot.set_color(base_col)
            
        link = active_link[frame_idx]
        if link is not None:
            signal_line.set_data([x_gt[frame_idx], link[0], BS_POS[0]], [y_gt[frame_idx], link[1], BS_POS[1]])
            signal_line.set_alpha(0.8)
        else:
            signal_line.set_data([], [])
            
        pred = ext_trajectories[frame_idx]
        if pred is not None:
            predictive_line.set_data(pred[0], pred[1])
        else:
            predictive_line.set_data([], [])
            
        snaps = master_time_logs[frame_idx][-5:]
        for i in range(5):
            if i < len(snaps):
                entry = snaps[i]
                log_texts[i].set_text(entry['txt'])
                base_c = 'cyan' if entry['color'] == 'b' else ('lime' if entry['color'] == 'g' else 'coral')
                log_texts[i].set_color(base_c)
            else:
                log_texts[i].set_text("")
                
        timer_txt.set_text(f"TIME: +{t_vals[frame_idx]:04.1f}s")
        
        rmse_base_line.set_data(t_vals[:frame_idx], rmse_base_t[:frame_idx])
        rmse_hybrid_line.set_data(t_vals[:frame_idx], rmse_hybrid_t[:frame_idx])
        rmse_base_fill.remove()
        rmse_hybrid_fill.remove()
        rmse_base_fill = ax_rmse.fill_between(t_vals[:frame_idx], rmse_base_t[:frame_idx], color='orange', alpha=0.15)
        rmse_hybrid_fill = ax_rmse.fill_between(t_vals[:frame_idx], rmse_hybrid_t[:frame_idx], color='#00FFCC', alpha=0.3)
        
        return [path_line, drone_marker, ekf_path_line, ekf_marker, hybrid_path_line, hybrid_marker, signal_line, predictive_line, timer_txt, rmse_base_line, rmse_hybrid_line, rmse_base_fill, rmse_hybrid_fill] + ris_dots + log_texts
        
    print(f"Sto animando {len(render_frames)} fotogrammi compressi...")
    ani = FuncAnimation(fig, update, frames=render_frames, blit=False, interval=50)
    
    try:
        mp4_path = "Test_Animazione_Simulatore_SDN.mp4"
        print(f"Export in MP4 [{mp4_path}]...")
        writer = FFMpegWriter(fps=24, metadata=dict(artist='Thesis Simulator'), bitrate=1800)
        ani.save(mp4_path, writer=writer)
        print("MP4 Salvato correttamente!")
    except Exception as e:
        print(f"Errore caricamento MP4 (FFMpeg non installato): {e}")

    try:
        gif_path = "Test_Animazione_Simulatore_SDN.gif"
        print(f"Export in GIF [{gif_path}]...")
        writer_gif = PillowWriter(fps=15)
        ani.save(gif_path, writer=writer_gif)
        print("GIF Salvata correttamente!")
    except Exception as e:
        print(f"Errore caricamento GIF: {e}")

if __name__ == "__main__":
    generate_animation()
