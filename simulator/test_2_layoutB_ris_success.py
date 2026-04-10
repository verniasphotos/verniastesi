import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Rectangle
import matplotlib.patches as mpatches
import matplotlib.lines as mlines

# Integrazione con i moduli del simulatore
from simulator.modulo_1_config import LAYOUT_B, RIS_HARDWARE
from simulator.modulo_2_environment import Environment, ray_casting_numba
from simulator.modulo_6_sdn_controller import SDNController

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
    max_diameter = 16.0
    width = min(width, max_diameter)
    height = min(height, max_diameter)
    ellip = Ellipse(xy=pos, width=width, height=height, angle=theta, **kwargs)
    ax.add_patch(ellip)

def generate_ris_success_map():
    # 1. Setup Ambiente Reale Identico al Test 1.1 (Layout B)
    env = Environment(LAYOUT_B)
    WIDTH, HEIGHT = LAYOUT_B.x_dim_m, LAYOUT_B.y_dim_m
    
    wall = LAYOUT_B.wall_spacing_m
    sh_x = LAYOUT_B.shelf_x_m
    vna_w = LAYOUT_B.vna_width_m
    
    aisle_5 = wall + sh_x + vna_w/2 + 4*(sh_x + vna_w)    # ~22.0
    aisle_10 = wall + sh_x + vna_w/2 + 9*(sh_x + vna_w)   # ~43.0
    aisle_15 = wall + sh_x + vna_w/2 + 14*(sh_x + vna_w)  # ~64.0
    
    BS_POS = np.array([aisle_10, HEIGHT / 2.0, LAYOUT_B.z_dim_m])
    
    y_service_low = 1.25   
    y_service_high = HEIGHT - 1.5 
    
    wp_charge = (aisle_5, y_service_low)          
    wp_a5_top = (aisle_5, y_service_high)         
    wp_a10_top = (aisle_10, y_service_high)       
    wp_a10_bot = (aisle_10, y_service_low)        
    wp_a15_bot = (aisle_15, y_service_low)        
    wp_shelf = (aisle_15, 60.0)                   
    
    # 2. Setup Grafico
    plt.style.use('seaborn-v0_8-paper')
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.set_title(f"Test 2.1: Mitigazione NLoS con RIS e Traiettoria Complessa ({LAYOUT_B.name})", fontsize=16, fontweight='bold', pad=20)
    ax.set_xlim(0, WIDTH)
    ax.set_ylim(0, HEIGHT)
    ax.set_facecolor('#F0F0F0')
    
    # Disegno Scaffali
    seen = set()
    for box in env.shelf_boxes:
        xmin, xmax, ymin, ymax = box[0], box[1], box[2], box[3]
        if (round(xmin, 2), round(ymin, 2)) not in seen:
            seen.add((round(xmin, 2), round(ymin, 2)))
            ax.add_patch(Rectangle((xmin, ymin), xmax-xmin, ymax-ymin, color='#404040', alpha=0.5, zorder=1))

    # Generazione Cinematica Fisicamente Realistica
    wps = [wp_charge, wp_a5_top, wp_a10_top, wp_a10_bot, wp_a15_bot, wp_shelf]
    dt = 0.2
    speed = 3.0
    
    x_gt, y_gt = [], []
    curr_pos = np.array(wps[0])
    curr_wp = 0
    # Aumentato il range di T per coprire una traiettoria più lunga
    t_vals = np.arange(0, 200, dt) 
    
    for _ in t_vals:
        if curr_wp < len(wps) - 1:
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
        
    x_gt = np.array(x_gt)
    y_gt = np.array(y_gt)

    # 3. ESECUZIONE REALE DEL FILTRO EKF (Tracking)
    init_state = [x_gt[0], y_gt[0], 0.0, speed]
    init_P = np.eye(4) * 2.0
    q_noise = 0.2   
    r_noise_los = 1.0   
    r_noise_ris = 1.8 # Segnale rimbalzato ottimizzato!
    
    ekf = EKF2D(dt, init_state, init_P, q_noise, r_noise_los)
    
    est_states = []
    est_covs = []
    nlos_flags = []
    
    np.random.seed(42) # Riproducibilità errore sensore
    
    # Precalcolo posizioni RIS per non ripetere (Farà testo la logica della mappa topologica - 12 RIS)
    total_ris_budget = 12 
    controller = SDNController(layout=LAYOUT_B, ris_specs=RIS_HARDWARE)
    rng = np.random.default_rng(42)
    centers = env.shelf_centers
    samples = max(1, int(len(centers) * 0.20))
    indices = rng.choice(len(centers), size=samples, replace=False)
    nlos_simulated = centers[indices]
    deployed_positions = controller.deploy_ris_kmeans_greedy(nlos_simulated, total_ris_budget)
    
    # Determiniamo accensione SDN Dinamica
    active_ris_list = []
    standby_ris_list = []
    for pos in deployed_positions:
        x, y, z = pos
        # Euristica: accese solo dove serve: Corsia 5 e Corsia 15 (Corsia 10 in LOS)
        if abs(x - aisle_5) < 30.0 or abs(x - aisle_15) < 30.0:
            active_ris_list.append((x, y, z))
        else:
            standby_ris_list.append((x, y, z))
            
    for k in range(len(t_vals)):
        ekf.predict()
        
        uav_pos_3d = np.array([x_gt[k], y_gt[k], 1.0])
        penetration_m = ray_casting_numba(uav_pos_3d, BS_POS, env.shelf_boxes)
        nlos = (penetration_m > 0.1) 
        nlos_flags.append(nlos)
        
        if nlos:
            # Aiutati dai RIS
            meas_x = x_gt[k] + np.random.normal(0, np.sqrt(r_noise_ris))
            meas_y = y_gt[k] + np.random.normal(0, np.sqrt(r_noise_ris))
            ekf.R = np.eye(2) * r_noise_ris
            ekf.update([meas_x, meas_y])
        else:
            # In LoS
            meas_x = x_gt[k] + np.random.normal(0, np.sqrt(r_noise_los))
            meas_y = y_gt[k] + np.random.normal(0, np.sqrt(r_noise_los))
            ekf.R = np.eye(2) * r_noise_los
            ekf.update([meas_x, meas_y])
            
        est_states.append(ekf.X.flatten())
        est_covs.append(ekf.P.copy())
        
    est_states = np.array(est_states)
    x_est = est_states[:, 0]
    y_est = est_states[:, 1]
    
    # Calcolo Metriche RMSE Fisico
    rmse_tot = np.sqrt(np.mean((x_gt - x_est)**2 + (y_gt - y_est)**2))

    # 4. PLOT DELLE RIS
    # Plot RIS Standby (Rosse)
    for pos in standby_ris_list:
        ax.plot(pos[0], pos[1], 'o', mfc='red', mec='darkred', markersize=8, markeredgewidth=1, zorder=2)
        
    # Plot RIS Attive (Lime/Yellow)
    for pos in active_ris_list:
        if pos[2] >= float(LAYOUT_B.z_dim_m * 0.95):
            ax.plot(pos[0], pos[1], 'o', mfc='lime', mec='yellow', markersize=12, markeredgewidth=2, zorder=6)
        else:
            ax.plot(pos[0], pos[1], 's', mfc='#00BFFF', mec='yellow', markersize=12, markeredgewidth=2, zorder=6)

    # 5. PLOT TRAIETTORIE LINEARI
    for i in range(len(x_gt)-1):
        color = 'blue' if nlos_flags[i] else '#2ca02c' 
        ax.plot(x_gt[i:i+2], y_gt[i:i+2], color=color, linewidth=4, zorder=3)
    
    ax.plot(x_est, y_est, 'r--', linewidth=2.5, zorder=4)
    
    for i in range(0, len(x_est), 4):
        # Disegnamo l'ellisse dalla covarianza matematica EKF
        plot_covariance_ellipse(ax, pos=(x_est[i], y_est[i]), P=est_covs[i], 
                                n_std=3.0, facecolor='deeppink', edgecolor='deeppink', 
                                alpha=0.3, zorder=2)

    # 6. MARKERS CHIAVE
    ax.plot(BS_POS[0], BS_POS[1], '^', color='gold', markersize=15, markeredgecolor='blue', markeredgewidth=2, zorder=5)
    ax.plot(wp_charge[0], wp_charge[1], 'bo', markersize=10, markeredgecolor='white', label='Base Ricarica', zorder=6)
    ax.plot(wp_shelf[0], wp_shelf[1], 'bs', markersize=10, markeredgecolor='white', label='Scaffale Target', zorder=6)
    
    # Setting assi
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, linestyle=':', alpha=0.5)
    
    # --- Elementi Informativi (Posizionati per non sovrapporsi) ---
    textstr = f"RISULTATI ANALITICI EKF\nRMSE Complessivo: {rmse_tot:.2f} m\nModello: R-Noise Dinamico\nStato SDN: Accensione Selettiva ({len(active_ris_list)} su {total_ris_budget})"
    props = dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='gray')
    # Posizioniamo il box dei risultati in alto a destra all'INTERNO del grafico
    ax.text(0.98, 0.98, textstr, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', horizontalalignment='right', bbox=props, zorder=10)

    # Legenda 
    shelf_patch = mpatches.Patch(color='#404040', alpha=0.5, label='Scaffali Metallici')
    bs_marker = mlines.Line2D([], [], color='none', marker='^', markerfacecolor='gold', markeredgecolor='blue', markersize=12, markeredgewidth=2, label='Base Station')
    ris_active_marker = mlines.Line2D([], [], color='none', marker='o', markerfacecolor='lime', markeredgecolor='yellow', markersize=10, markeredgewidth=2, label='RIS Attiva')
    ris_standby_marker = mlines.Line2D([], [], color='none', marker='o', markerfacecolor='red', markeredgecolor='darkred', markersize=8, markeredgewidth=1, label='RIS Standby')
    gt_los = mlines.Line2D([], [], color='#2ca02c', linewidth=4, label='GT LoS')
    gt_alos = mlines.Line2D([], [], color='blue', linewidth=4, label='GT RIS-Assisted LoS')
    ekf_line = mlines.Line2D([], [], color='red', linestyle='--', linewidth=2.5, label='Stima EKF (Corretta)')
    cov_patch = mpatches.Patch(color='deeppink', alpha=0.3, label='Incertezza EKF (Fisiologica)')
    
    ax.legend(handles=[shelf_patch, bs_marker, ris_active_marker, ris_standby_marker, gt_los, gt_alos, ekf_line, cov_patch], 
              loc='upper center', bbox_to_anchor=(0.5, -0.12), ncol=4, fontsize=10, frameon=True, shadow=True)

    output_path = "/Users/vernias/Desktop/verniastesi/Test_2.1_Successo_RIS.png"
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Grafico ottimizzato e salvato in: {output_path}")

if __name__ == "__main__":
    generate_ris_success_map()
