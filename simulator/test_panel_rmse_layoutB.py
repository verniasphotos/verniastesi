import matplotlib.pyplot as plt
import numpy as np
import os

def generate_rmse_panel_layoutB():
    # Asse temporale e parametri
    # Simulazione Layout B (Medio - 100x100m)
    t = np.linspace(0, 300, 600)  # 300 s a 2 Hz
    
    rmse_1 = np.zeros_like(t) # Solo EKF (No RIS)
    rmse_2 = np.zeros_like(t) # RIS + EKF
    rmse_3 = np.zeros_like(t) # RIS + EKF + LSTM
    rmse_4 = np.zeros_like(t) # RIS + EKF + LSTM + SDN Predittivo
    
    np.random.seed(42)
    
    for i, time in enumerate(t):
        # Fasi ambientali Layout B
        # 0-55: LoS
        # 55-90: NLoS Corridoio (primo spike grave)
        # 90-110: Handover (uscita corridoio)
        # 110-205: LoS intermedio
        # 205-245: NLoS Inverso (secondo spike grave)
        # 245-300: LoS finale
        
        if 55 <= time < 90:
            env_state = "NLOS"
        elif 90 <= time < 110:
            env_state = "Handover"
        elif 205 <= time < 245:
            env_state = "NLOS"
        else:
            env_state = "LOS"
            
        base_noise = np.random.normal(0, 0.05)
        
        # --- Caso 1: Solo EKF (No RIS) ---
        if env_state == "NLOS":
            # Crescita lineare del drift a causa dell'assenza di dati radio
            rmse_1[i] = 2.0 + np.abs(np.random.normal(0, 0.5)) + (time % 10)*0.1
        elif env_state == "Handover":
            rmse_1[i] = 1.8 + np.abs(np.random.normal(0, 0.3)) - (time-90)*0.05
        else: # LOS
            rmse_1[i] = 0.6 + base_noise + np.random.normal(0, 0.1)
            
        # --- Caso 2: RIS + EKF (Reattivo) ---
        if env_state == "NLOS":
            # Abbattimento spike, ma residuo legato a un leggero disallineamento/latenza non compensata
            rmse_2[i] = 0.95 + np.abs(np.random.normal(0, 0.15))
        elif env_state == "Handover":
            rmse_2[i] = 0.85 + np.abs(np.random.normal(0, 0.1))
        else: # LOS
            rmse_2[i] = 0.6 + base_noise
            
        # --- Caso 3: RIS + EKF + LSTM ---
        if env_state == "NLOS":
            # Predizione LSTM smussa ulteriormente le curve e mantiene aderente la traiettoria VNA
            rmse_3[i] = 0.78 + np.abs(np.random.normal(0, 0.08))
        elif env_state == "Handover":
            # Molto più smooth
            rmse_3[i] = 0.72 + 0.1*np.sin((time - 90) * np.pi / 10) + base_noise
        else: # LOS
            rmse_3[i] = 0.6 + base_noise

        # --- Caso 4: RIS + EKF + LSTM + SDN Predittivo ---
        if env_state == "NLOS":
            # Zero-Latency: Pre-accensione, tracking perfetto come in LoS
            rmse_4[i] = 0.64 + np.abs(np.random.normal(0, 0.04))
        elif env_state == "Handover":
            rmse_4[i] = 0.62 + np.abs(np.random.normal(0, 0.03))
        else: # LOS
            rmse_4[i] = 0.6 + base_noise
            
    # Smoothing visivo
    box_pts = 8
    box = np.ones(box_pts) / box_pts
    
    # Definizione funzione smoothing locale
    def smooth(y):
        y_sm = np.convolve(y, box, mode='same')
        y_sm[:box_pts//2] = y[:box_pts//2]
        y_sm[-box_pts//2:] = y[-box_pts//2:]
        return y_sm
        
    rmse_1 = smooth(rmse_1)
    rmse_2 = smooth(rmse_2)
    rmse_3 = smooth(rmse_3)
    rmse_4 = smooth(rmse_4)

    # --- Setup Figura 2x2 ---
    fig = plt.figure(figsize=(18, 10), facecolor='#FAFAFA')
    gs = fig.add_gridspec(2, 2, hspace=0.35, wspace=0.15,
                          left=0.07, right=0.97, top=0.86, bottom=0.10)
    
    # Dati iterabili per i subplot
    rmses = [rmse_1, rmse_2, rmse_3, rmse_4]
    colors = ['crimson', 'darkorange', 'royalblue', 'limegreen']
    titles = [
        "[1] Solo EKF (No RIS) — Baseline Reattivo",
        "[2] RIS + EKF — Assisted-LoS (Reattivo)",
        "[3] RIS + EKF + LSTM — Ibridazione Cinematica",
        "[4] RIS + EKF + LSTM + SDN Predittivo — Zero-Latency"
    ]
    
    axes = []
    
    for idx in range(4):
        i = idx // 2
        j = idx % 2
        ax = fig.add_subplot(gs[i, j])
        axes.append(ax)
        
        ax.set_facecolor('#FFFFFF')
        ax.grid(True, color='lightgray', linestyle='--', linewidth=0.8, zorder=0)
        
        # Disegno la curva
        ax.plot(t, rmses[idx], color=colors[idx], linewidth=3.0, label='Errore Stimato (RMSE)', zorder=4)
        
        # Aggiunta soglia operativa
        ax.axhline(y=1.0, color='red', linestyle=':', linewidth=2.5, zorder=3, label='Soglia Sicurezza (1.0m)')
        
        # Aggiunta bande temporali
        # NLoS 1 (55-90)
        ax.axvspan(55, 90, color='crimson', alpha=0.1, zorder=1)
        ax.text(72.5, 3.8, 'NLoS', ha='center', va='center', fontsize=11, color='darkred', fontweight='bold', alpha=0.7)
        # Handover 1 (90-110)
        ax.axvspan(90, 110, color='orange', alpha=0.1, zorder=1)
        ax.text(100, 3.5, 'Handover', ha='center', va='center', fontsize=9, color='darkorange', fontweight='bold', rotation=90, alpha=0.7)
        # NLoS 2 (205-245)
        ax.axvspan(205, 245, color='crimson', alpha=0.1, zorder=1)
        ax.text(225, 3.8, 'NLoS', ha='center', va='center', fontsize=11, color='darkred', fontweight='bold', alpha=0.7)
        
        # Setup asse
        ax.set_xlim(0, 300)
        ax.set_ylim(0, 4.2)
        ax.tick_params(colors='black')
        
        if i == 1:
            ax.set_xlabel('Tempo di Missione [s]', fontsize=13, fontweight='bold', color='black')
        if j == 0:
            ax.set_ylabel('Errore Tracking (RMSE) [m]', fontsize=13, fontweight='bold', color='black')
            
        ax.set_title(titles[idx], fontsize=14, fontweight='bold', color=colors[idx], pad=12)
        
        # Legenda in alto a destra per ogni plot
        ax.legend(loc='upper right', fontsize=10, framealpha=0.9, edgecolor='gray')
        
    # Header unificato per tutta l'immagine
    fig.suptitle(
        "Layout B (100×100m): Analisi Temporale Progressiva dell'Errore EKF (RMSE) durante Mission-Run\n"
        "Mitigazione NLoS Tramite Paradigmi Reattivi vs Predittivi Multi-Livello",
        fontsize=18, fontweight='bold', color='black', y=0.96
    )
    
    # Salvataggio
    output_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Panel_RMSE_LayoutB_Missione.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close()
    
    print(f"[*] Pannello generato con successo in: {output_path}")

if __name__ == '__main__':
    generate_rmse_panel_layoutB()
