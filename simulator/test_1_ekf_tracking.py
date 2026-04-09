import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Rectangle
import matplotlib.patches as mpatches
import matplotlib.lines as mlines

# Integrazione con i moduli del simulatore
from simulator.modulo_1_config import LAYOUT_C
from simulator.modulo_2_environment import Environment, ray_casting_numba

def generate_ekf_drift_map():
    # 1. Setup Ambiente Reale
    env = Environment(LAYOUT_C)
    WIDTH, HEIGHT = LAYOUT_C.x_dim_m, LAYOUT_C.y_dim_m
    
    # Parametri geometrici
    wall = LAYOUT_C.wall_spacing_m
    sh_x = LAYOUT_C.shelf_x_m
    vna_w = LAYOUT_C.vna_width_m
    
    # 2. Setup BS e Traiettoria Logistica LEGALE (Uso dei corridoi di testa)
    aisle_bs_x = wall + sh_x + vna_w/2 + 29*(sh_x + vna_w) # Aisle 30 (X=127.0)
    aisle_nlos_x = aisle_bs_x + (sh_x + vna_w)            # Aisle 31 (X=131.2)
    
    # Base Station nell'Aisle 30
    BS_POS = np.array([aisle_bs_x, HEIGHT / 2.0, LAYOUT_C.z_dim_m])
    
    # Punti della missione (Percorso che rispetta i muri e gli scaffali)
    y_service_low = 1.25   # Corridoio di testa inferiore
    y_service_high = 138.5 # Corridoio di testa superiore
    
    wp_charge = (WIDTH / 2.0, y_service_low)    # 1. Base Ricarica
    wp_start_30 = (aisle_bs_x, y_service_low)   # 2. Ingresso Aisle 30
    wp_end_30 = (aisle_bs_x, y_service_high)    # 3. Uscita Aisle 30 (In alto)
    wp_start_31 = (aisle_nlos_x, y_service_high)# 4. Cambio corsia in alto
    wp_shelf = (aisle_nlos_x, 60.0)             # 5. Arrivo allo scaffale Target
    
    def segment(p1, p2, num=30):
        return np.linspace(p1[0], p2[0], num), np.linspace(p1[1], p2[1], num)
    
    x1, y1 = segment(wp_charge, wp_start_30, 20)
    x2, y2 = segment(wp_start_30, wp_end_30, 80)
    x3, y3 = segment(wp_end_30, wp_start_31, 20)
    x4, y4 = segment(wp_start_31, wp_shelf, 60)
    
    x_gt = np.concatenate([x1, x2, x3, x4])
    y_gt = np.concatenate([y1, y2, y3, y4])
    
    # 3. Rendering Grafico
    plt.style.use('seaborn-v0_8-paper')
    fig, ax = plt.subplots(figsize=(15, 9))
    ax.set_title(f"Test 1: Fallimento EKF e Drift in Corridoio NLoS ({LAYOUT_C.name})", fontsize=18, fontweight='bold', pad=20)
    ax.set_xlim(0, WIDTH); ax.set_ylim(0, HEIGHT)
    ax.set_facecolor('#F0F0F0')
    
    # Disegno Scaffali
    seen = set()
    for box in env.shelf_boxes:
        xmin, xmax, ymin, ymax = box[0], box[1], box[2], box[3]
        if (round(xmin, 2), round(ymin, 2)) not in seen:
            seen.add((round(xmin, 2), round(ymin, 2)))
            ax.add_patch(Rectangle((xmin, ymin), xmax-xmin, ymax-ymin, color='#404040', alpha=0.5, zorder=1))

    # 4. Calcolo NLoS e Stima EKF
    nlos_flags = []
    x_est, y_est = [], []
    drift = 0.0
    
    for i in range(len(x_gt)):
        uav_pos = np.array([x_gt[i], y_gt[i], 1.0])
        nlos = ray_casting_numba(uav_pos, BS_POS, env.shelf_boxes) > 0.1
        nlos_flags.append(nlos)
        
        if nlos:
            drift += 0.15 # L'errore cresce solo in NLoS
            x_est.append(x_gt[i] + drift)
        else:
            drift = 0.0
            x_est.append(x_gt[i])
        y_est.append(y_gt[i])

    # 5. Plot Linee
    for i in range(len(x_gt)-1):
        color = 'black' if nlos_flags[i] else '#2ca02c'
        ax.plot(x_gt[i:i+2], y_gt[i:i+2], color=color, linewidth=4, zorder=3)
    
    ax.plot(x_est, y_est, 'r--', linewidth=2.5, label='Stima EKF (Divergente)', zorder=4)
    
    # Ellissi
    for i in range(0, len(x_gt), 6):
        if nlos_flags[i]:
            sz = (sum(nlos_flags[:i]) / sum(nlos_flags)) * 12.0
            ax.add_patch(Ellipse((x_est[i], y_est[i]), 2+sz, 1+sz/2, facecolor='deeppink', alpha=0.2, zorder=2))

    # Marker e Legenda
    ax.plot(BS_POS[0], BS_POS[1], '^', color='gold', markersize=15, markeredgecolor='blue', markeredgewidth=2, label='Base Station', zorder=5)
    ax.plot(wp_charge[0], wp_charge[1], 'bo', label='Base Ricarica', zorder=6)
    ax.plot(wp_shelf[0], wp_shelf[1], 'bs', label='Scaffale Target', zorder=6)
    
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.grid(True, linestyle=':', alpha=0.5)
    
    handles, labels = ax.get_legend_handles_labels()
    nlos_line = mlines.Line2D([], [], color='black', linewidth=4, label='GT in NLoS (Blocco)')
    los_line = mlines.Line2D([], [], color='#2ca02c', linewidth=4, label='GT in LoS (Visibile)')
    cov_patch = mpatches.Patch(color='deeppink', alpha=0.3, label='Incertezza EKF')
    ax.legend(handles=handles + [nlos_line, los_line, cov_patch], loc='center left', fontsize=14, frameon=True, shadow=True)

    plt.tight_layout()
    plt.savefig("Test_1_TopDown_Traiettoria.png", dpi=300)
    print("Test 1 aggiornato con successo. Ora è coerente con Layout C.")

if __name__ == "__main__":
    generate_ekf_drift_map()
