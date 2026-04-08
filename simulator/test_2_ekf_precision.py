import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Ellipse

# =====================================================================
# INTEGRAZIONE NATIVA col SIMULATORE 6G (Layout Veritieri)
# =====================================================================
from simulator.modulo_1_config import LAYOUT_A, LAYOUT_B, LAYOUT_C
from simulator.modulo_2_environment import Environment, ray_casting_numba

# =====================================================================
# MOTORE: EXTENDED KALMAN FILTER (EKF) 2D
# =====================================================================

class EKF2D:
    """
    Filtro di Kalman Esteso per il tracciamento LGV / Drone 2D.
    Modello a velocità costante (Constant Velocity Model).
    """
    def __init__(self, dt, init_state, init_P, q_noise, r_noise):
        self.dt = dt
        # Stato X: [posizione_x, posizione_y, velocità_x, velocità_y]^T
        self.X = np.array(init_state, dtype=float).reshape(4, 1)
        self.P = np.array(init_P, dtype=float)
        
        # Modello di transizione cinematica F
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0,  dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1]
        ])
        
        # Matrice di osservazione H (misuriamo solo la posizione GPS/UWB)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # Covarianza rumore di processo Q (process noise)
        G = np.array([
            [dt**2 / 2, 0],
            [0, dt**2 / 2],
            [dt, 0],
            [0, dt]
        ])
        self.Q = (G @ G.T) * q_noise
        
        # Matrice rumore di misura R
        self.R = np.eye(2) * r_noise

    def predict(self):
        """Fase Predittiva autonoma. Aumenta fisiologicamente l'incertezza P."""
        self.X = self.F @ self.X
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, Z):
        """Correzione matematica basata sull'acquisizione sensoriale in LoS."""
        Z = np.array(Z).reshape(2, 1)
        Y = Z - (self.H @ self.X)
        
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        self.X = self.X + (K @ Y)
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P

# =====================================================================
# MOTORE GENERATORE DI TRAIETTORIE E SCENARI REALI
# =====================================================================

def setup_lgv_trajectory(layout_config, total_time=150.0, dt=0.5):
    """
    Genera un oggetto Environment con l'esatta griglia di scaffali 3D, 
    ed estrae una traiettoria 'veritiera' per un robot o drone in volo rasoterra 
    (con z=1.0m) che si muove fisicamente all'interno dei corridoi VNA!
    """
    env = Environment(layout_config)
    
    wall = layout_config.wall_spacing_m
    sh_x = layout_config.shelf_x_m
    vna_w = layout_config.vna_width_m
    
    # 1. Rilevamento automatico di TUTTI i corridoi VNA sicuri
    aisles = []
    x = wall + sh_x/2
    aisles.append(wall / 2.0) # Primo corridoio (muro-scaffale)
    while x + sh_x/2 <= layout_config.x_dim_m - wall:
        aisles.append(x + sh_x/2 + vna_w/2)
        x += sh_x + vna_w
        
    # 2. Scelta di un percorso ad anello su due corsie VNA distinte (pattugliamento robotico)
    mid_idx = len(aisles) // 2
    idx1 = max(0, mid_idx - 2)              # Scende in una corsia
    idx2 = min(len(aisles)-1, mid_idx + 2)  # Sale nell'altra
    
    ax1 = aisles[idx1] 
    ax2 = aisles[idx2]
    
    y_bottom = wall + 1.2
    y_top = layout_config.y_dim_m - wall - 1.2
    
    wps = [
        (ax1, y_bottom),
        (ax1, y_top),
        (ax2, y_top),
        (ax2, y_bottom),
        (ax1, y_bottom)
    ]
    wps = wps * 40 # Array ciclico infinito per coprire tutta la durata T
    
    # 3. Risoluzione Cinematica (Fisica del Moviemento Costante m/s)
    t_vals = np.arange(0, total_time, dt)
    x_true = []
    y_true = []
    
    curr_wp = 0
    curr_pos = np.array(wps[0])
    speed = 4.5 # m/s velocità di simulazione drone veloce in corridoio
    
    for _ in t_vals:
        target = np.array(wps[curr_wp+1])
        dist = np.linalg.norm(target - curr_pos)
        step = speed * dt
        
        if dist <= step:
            curr_pos = target
            curr_wp += 1
        else:
            dir_v = (target - curr_pos) / dist
            curr_pos = curr_pos + dir_v * step
            
        x_true.append(curr_pos[0])
        y_true.append(curr_pos[1])
        
    # 4. Estrazione layer Bounding Boxes 2D per il Rendering Visivo senza duplicati Z
    shelf_rects_2d = []
    seen = set()
    for box in env.shelf_boxes:
        xmin, xmax, ymin, ymax = box[0], box[1], box[2], box[3]
        key = (round(xmin, 2), round(ymin, 2))
        if key not in seen:
            seen.add(key)
            shelf_rects_2d.append((xmin, ymin, xmax - xmin, ymax - ymin))
            
    return env, np.array(x_true), np.array(y_true), t_vals, shelf_rects_2d


def plot_covariance_ellipse(ax, pos, P, n_std=3.0, **kwargs):
    """ Plot Ellisse spettrale usando Matplotlib """
    P_pos = P[:2, :2]
    vals, vecs = np.linalg.eigh(P_pos)
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    theta = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
    width, height = 2 * n_std * np.sqrt(vals)
    ellip = Ellipse(xy=pos, width=width, height=height, angle=theta, **kwargs)
    ax.add_patch(ellip)


# =====================================================================
# ESECUZIONE TEST E RENDERING GRAFICO
# =====================================================================

def run_test_2():
    print("\n" + "="*70)
    print(" AVVIO TEST 2: SIMULATORE 6G REALE - TRAIETTORIE EKF IN VNA ")
    print("="*70)

    # I Layout del TEST 0 importati esatti da config.py
    layouts_confs = [LAYOUT_A, LAYOUT_B, LAYOUT_C]
    
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    dt = 0.5
    total_time = 150.0

    for idx, layout_cfg in enumerate(layouts_confs):
        ax = axes[idx]
        print(f"[*] Elaborazione Motore su {layout_cfg.name}...")
        
        # 1. SETUP AMBIENTE REALE DALLA LIBRERIA ENVIRONMENT
        env, x_true, y_true, t_vals, shelves_2d = setup_lgv_trajectory(layout_cfg, total_time, dt)
        
        # 2. DEFINIZIONE POSIZIONE BASE STATION (Esattamente al centro sul tetto, 3D come modulo 4)
        bs_pos_3d = np.array([layout_cfg.x_dim_m / 2.0, layout_cfg.y_dim_m / 2.0, layout_cfg.z_dim_m])
        
        init_state = [x_true[0], y_true[0], 0.0, 4.5]
        init_P = np.eye(4) * 2.0
        q_noise = 0.5   
        r_noise = 1.0   
        
        ekf = EKF2D(dt, init_state, init_P, q_noise, r_noise)
        est_states = []
        est_covs = []
        nlos_flags = []
        
        # --- CICLO DELLA VITA EKF ---
        for k in range(len(t_vals)):
            ekf.predict()
            
            uav_pos_3d = np.array([x_true[k], y_true[k], 1.0])
            
            # MAGIA NUMBA 3D! Reale Ray-Casting come usato nel vero Simulatore Test 1
            penetration_m = ray_casting_numba(uav_pos_3d, bs_pos_3d, env.shelf_boxes)
            nlos = (penetration_m > 0.1) # Se intercettiamo solidi scaffali, c'è blocco!
            nlos_flags.append(nlos)
            
            if not nlos:
                meas_x = x_true[k] + np.random.normal(0, np.sqrt(r_noise))
                meas_y = y_true[k] + np.random.normal(0, np.sqrt(r_noise))
                ekf.update([meas_x, meas_y])
                
            est_states.append(ekf.X.flatten())
            est_covs.append(ekf.P.copy())
            
        est_states = np.array(est_states)
        x_est, y_est = est_states[:, 0], est_states[:, 1]
        
        # METRICHE
        error = np.sqrt((x_true - x_est)**2 + (y_true - y_est)**2)
        print(f"  > [RISULTATO] Max. Error Covarianza:  {np.max(error):6.2f} m")
        print(f"  > [RISULTATO] Time-step in NLoS Puro: {sum(nlos_flags)}/{len(t_vals)}\n")
        
        # === VERNICIATURA ESTETICA (TEST COERENTE COL TEST 0) ===
        for i, (sx, sy, sw, sh) in enumerate(shelves_2d):
            rect = patches.Rectangle((sx, sy), sw, sh, color='gray', alpha=0.3, 
                                     label="Scaffali (True Layout)" if (i==0 and idx==0) else "")
            ax.add_patch(rect)
            
        # Base Station a terra nel 2D
        ax.plot(bs_pos_3d[0], bs_pos_3d[1], '^', color='gold', markersize=14, 
                markeredgecolor='black', label="Base Station 6G")
        
        ax.plot(x_true, y_true, '-', color='black', linewidth=2.5, alpha=0.9, label="Traiettoria VNA (LGV)")
        ax.plot(x_est, y_est, 'r--', linewidth=2, label="Stima Modello EKF")
        
        # Overlay arancione su percorsi NLoS
        nlos_x = [x_true[i] for i in range(len(t_vals)) if nlos_flags[i]]
        nlos_y = [y_true[i] for i in range(len(t_vals)) if nlos_flags[i]]
        if nlos_x:
            ax.scatter(nlos_x, nlos_y, c='orange', marker='x', s=25, alpha=0.8,
                       label="Coasting in NLoS")
            
        # Dispersione delle Ellissi (1 ogni TOT per leggibilità)
        step_ellipse = max(1, len(t_vals) // 25)
        for i in range(0, len(t_vals), step_ellipse):
            plot_covariance_ellipse(ax, pos=(x_est[i], y_est[i]), P=est_covs[i], 
                                    n_std=3.0, edgecolor='red', facecolor='red', alpha=0.25)
            pass

        # Estetica Standardizzata
        ax.set_title(layout_cfg.name, fontsize=13, fontweight='bold')
        ax.set_xlim(0, layout_cfg.x_dim_m); ax.set_ylim(0, layout_cfg.y_dim_m)
        ax.set_xlabel("X (metri)"); ax.set_ylabel("Y (metri)")
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.set_aspect('equal', adjustable='box')

    fig.legend(loc='lower center', ncol=5, bbox_to_anchor=(0.5, -0.05), fontsize=12)
    plt.tight_layout(rect=[0, 0.05, 1, 1])
    
    output_path = os.path.join(os.path.dirname(__file__), "test_2_risultati_ekf.png")
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    
    print("="*70)
    print(f"Layout Esatti e Traiettorie VNA Completati. Grafico in -> {output_path}")
    print("="*70 + "\n")

if __name__ == "__main__":
    run_test_2()
