import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Rectangle
import matplotlib.patches as mpatches
import matplotlib.lines as mlines

# Integrazione con i moduli del simulatore
from simulator.modulo_1_config import LAYOUT_B
from simulator.modulo_2_environment import Environment, ray_casting_numba

def generate_ekf_drift_map():
    # 1. Setup Ambiente Reale
    env = Environment(LAYOUT_B)
    WIDTH, HEIGHT = LAYOUT_B.x_dim_m, LAYOUT_B.y_dim_m
    
    # Parametri geometrici
    wall = LAYOUT_B.wall_spacing_m
    sh_x = LAYOUT_B.shelf_x_m
    vna_w = LAYOUT_B.vna_width_m
    
    # 2. Setup BS e Traiettoria Logistica LEGALE (Uso dei corridoi di testa)
    # Calcolo coordinate X delle corsie (Aisle)
    aisle_5 = wall + sh_x + vna_w/2 + 4*(sh_x + vna_w)    # ~22.0
    aisle_10 = wall + sh_x + vna_w/2 + 9*(sh_x + vna_w)   # ~43.0
    aisle_15 = wall + sh_x + vna_w/2 + 14*(sh_x + vna_w)  # ~64.0
    
    # Base Station a metà dell'Aisle 10
    BS_POS = np.array([aisle_10, HEIGHT / 2.0, LAYOUT_B.z_dim_m])
    
    # Punti della missione (Percorso a serpente che rispetta i muri e gli scaffali)
    y_service_low = 1.25   # Corridoio di testa inferiore
    y_service_high = HEIGHT - 1.5 # Corridoio di testa superiore (98.5)
    
    wp_charge = (aisle_5, y_service_low)          # 1. Base Ricarica (Inizio Aisle 5)
    wp_a5_top = (aisle_5, y_service_high)         # 2. Fine Aisle 5
    wp_a10_top = (aisle_10, y_service_high)       # 3. Spostamento orizzontale a Aisle 10
    wp_a10_bot = (aisle_10, y_service_low)        # 4. Discesa per Aisle 10 (LoS totale)
    wp_a15_bot = (aisle_15, y_service_low)        # 5. Spostamento orizzontale a Aisle 15
    wp_shelf = (aisle_15, 60.0)                   # 6. Arrivo allo scaffale Target
    
    def segment(p1, p2, num=30):
        return np.linspace(p1[0], p2[0], num), np.linspace(p1[1], p2[1], num)
    
    x1, y1 = segment(wp_charge, wp_a5_top, 80)
    x2, y2 = segment(wp_a5_top, wp_a10_top, 30)
    x3, y3 = segment(wp_a10_top, wp_a10_bot, 80)
    x4, y4 = segment(wp_a10_bot, wp_a15_bot, 30)
    x5, y5 = segment(wp_a15_bot, wp_shelf, 60)
    
    x_gt = np.concatenate([x1, x2, x3, x4, x5])
    y_gt = np.concatenate([y1, y2, y3, y4, y5])
    
    # 3. Rendering Grafico
    plt.style.use('seaborn-v0_8-paper')
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.set_title(f"Test 1.1: Fallimento EKF e Drift con Traiettoria Complessa ({LAYOUT_B.name})", fontsize=16, fontweight='bold', pad=20)
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
    drift_x = 0.0
    drift_y = 0.0
    
    for i in range(len(x_gt)):
        uav_pos = np.array([x_gt[i], y_gt[i], 1.0])
        nlos = ray_casting_numba(uav_pos, BS_POS, env.shelf_boxes) > 0.1
        nlos_flags.append(nlos)
        
        # Direzione del movimento per un drift più realistico (ortogonale o direzionale)
        if i > 0:
            dx = x_gt[i] - x_gt[i-1]
            dy = y_gt[i] - y_gt[i-1]
        else:
            dx, dy = 0, 1  # Movimento iniziale verso l'alto
            
        norm = np.sqrt(dx**2 + dy**2) + 1e-6
        # Il drift cresce in NLoS
        if nlos:
            # Aggiungiamo drift laterale e un po' in avanti
            drift_x += 0.1 * (dy / norm) + 0.02 * (dx / norm)
            drift_y += -0.1 * (dx / norm) + 0.02 * (dy / norm)
            x_est.append(x_gt[i] + drift_x)
            y_est.append(y_gt[i] + drift_y)
        else:
            # Ricondizionamento EKF con la LoS
            # Il drift decade rapitamente se siamo in LoS (l'EKF corregge)
            drift_x *= 0.7
            drift_y *= 0.7
            x_est.append(x_gt[i] + drift_x)
            y_est.append(y_gt[i] + drift_y)

    # 5. Plot Linee
    for i in range(len(x_gt)-1):
        color = 'black' if nlos_flags[i] else '#2ca02c'
        ax.plot(x_gt[i:i+2], y_gt[i:i+2], color=color, linewidth=4, zorder=3)
    
    ax.plot(x_est, y_est, 'r--', linewidth=2.5, label='Stima EKF (Divergente / Corretta in LoS)', zorder=4)
    
    # Ellissi di incertezza EKF
    current_uncertainty = 0.0
    for i in range(0, len(x_gt), 6):
        if nlos_flags[i]:
            current_uncertainty += 0.5
            current_uncertainty = min(current_uncertainty, 15.0) # max uncertainty
        else:
            current_uncertainty *= 0.4 # Rapida riduzione incertezza (EKF update con segnale pulito)
            current_uncertainty = max(current_uncertainty, 1.0)
            
        # Disegnamo l'ellisse solo se sufficientemente grande
        if current_uncertainty > 1.2:
            ax.add_patch(Ellipse((x_est[i], y_est[i]), 2+current_uncertainty, 1+current_uncertainty/2, facecolor='deeppink', alpha=0.2, zorder=2))

    # Marker e Legenda
    ax.plot(BS_POS[0], BS_POS[1], '^', color='gold', markersize=15, markeredgecolor='blue', markeredgewidth=2, label='Base Station', zorder=5)
    ax.plot(wp_charge[0], wp_charge[1], 'bo', markersize=10, label='Base Ricarica', zorder=6)
    ax.plot(wp_shelf[0], wp_shelf[1], 'bs', markersize=10, label='Scaffale Target', zorder=6)
    
    ax.set_xlabel("X (m)", fontsize=12); ax.set_ylabel("Y (m)", fontsize=12)
    ax.grid(True, linestyle=':', alpha=0.5)
    
    handles, labels = ax.get_legend_handles_labels()
    nlos_line = mlines.Line2D([], [], color='black', linewidth=4, label='GT in NLoS (Blocco)')
    los_line = mlines.Line2D([], [], color='#2ca02c', linewidth=4, label='GT in LoS (Visibile)')
    cov_patch = mpatches.Patch(color='deeppink', alpha=0.3, label='Incertezza EKF')
    
    # Spostiamo la legenda sotto il grafico per non coprire la traiettoria
    ax.legend(handles=handles + [nlos_line, los_line, cov_patch], 
              loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=3,
              fontsize=10, frameon=True, shadow=True, facecolor='white')

    plt.tight_layout()
    plt.savefig("Test_1.1_TopDown_Traiettoria.png", dpi=300)
    print("Test 1.1 (Layout B) generato con successo e immagine salvata come Test_1.1_TopDown_Traiettoria.png.")

if __name__ == "__main__":
    generate_ekf_drift_map()
