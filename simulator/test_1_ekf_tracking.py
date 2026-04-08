import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Rectangle
from scipy.interpolate import interp1d

# Importazione dei moduli di configurazione e ambiente del simulatore 6G
from simulator.modulo_1_config import LAYOUT_A, LAYOUT_B, LAYOUT_C
from simulator.modulo_2_environment import Environment

class EKF2D:
    """
    [MATEMATICA TESI] Implementazione dell'Extended Kalman Filter (EKF) per il Tracking 2D.
    Il filtro fonde il modello cinematico (predizione) con i dati dei sensori (aggiornamento).
    """
    def __init__(self, dt, std_meas=2.0, std_proc=0.1):
        self.dt = dt
        
        # Stato iniziale: [x, y, vx, vy]^T (Posizione e Velocità)
        self.state = np.zeros(4) 
        
        # Matrice di Covarianza dello Stato (P): Rappresenta l'incertezza iniziale sulla posizione
        self.P = np.eye(4) * 10.0 
        
        # Matrice di Transizione di Stato (F): Definisce come lo stato evolve nel tempo (Cinematica)
        # x_new = x + vx * dt
        # y_new = y + vy * dt
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Matrice di Osservazione (H): Mappa lo stato nelle misure (leggiamo solo x e y)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # Matrice di Rumore di Misura (R): Incertezza dei sensori radio (es. jitter SNR)
        self.R = np.array([
            [std_meas**2, 0],
            [0, std_meas**2]
        ])
        
        # Matrice di Rumore di Processo (Q): Incertezza sul modello fisico (es. raffiche di vento o attriti)
        q = std_proc**2
        self.Q = np.array([
            [(dt**4)/4, 0, (dt**3)/2, 0],
            [0, (dt**4)/4, 0, (dt**3)/2],
            [(dt**3)/2, 0, dt**2, 0],
            [0, (dt**3)/2, 0, dt**2]
        ]) * q

    def predict(self):
        """
        FASE 1: PREDIZIONE
        Propaga lo stato e l'incertezza in avanti nel tempo usando il modello fisico.
        """
        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        """
        FASE 2: AGGIORNAMENTO (CORREZIONE)
        Utilizza la misura reale 'z' per correggere la stima del filtro.
        """
        # Calcolo dell'innovazione (differenza tra misura reale e attesa)
        y = z - (self.H @ self.state)
        # Calcolo della covarianza dell'innovazione
        S = self.H @ self.P @ self.H.T + self.R
        # Guadagno di Kalman (K): determina quanto fidarsi della misura rispetto alla predizione
        K = self.P @ self.H.T @ np.linalg.inv(S)
        # Aggiornamento dello stato e della covarianza
        self.state = self.state + (K @ y)
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P


def generate_burst_nlos(N, target_prob, burst_len_avg=30):
    """
    Simula la presenza di ostacoli metallici densi (NLoS) in modo realistico.
    Invece di interruzioni casuali, genera "blocchi" di tempo (burst) senza segnale.
    """
    if target_prob == 0: return np.zeros(N, dtype=bool)
    p_10 = 1.0 / burst_len_avg
    p_01 = (target_prob * p_10) / (1.0 - target_prob)
    
    state = 0
    mask = np.zeros(N, dtype=bool)
    for i in range(N):
        if state == 0:
            if np.random.rand() < p_01: state = 1 # Passa a NLoS
        else:
            if np.random.rand() < p_10: state = 0 # Torna in LoS
        mask[i] = (state == 1)
    return mask

def calculate_trajectory(waypoints_x, waypoints_y, dt, velocity=1.5):
    """
    Genera una traiettoria "spigolosa" (lineare) per droni industriali.
    Garantisce una velocità costante lungo tutto il percorso ottimizzato.
    """
    dx = np.diff(waypoints_x)
    dy = np.diff(waypoints_y)
    dist = np.sqrt(dx**2 + dy**2)
    cum_dist = np.concatenate(([0], np.cumsum(dist)))
    
    total_dist = cum_dist[-1]
    duration = total_dist / velocity
    
    N = int(duration / dt)
    d_vals = np.linspace(0, total_dist, N)
    
    # Interpolazione lineare per mantenere gli angoli netti a 90° nei corridoi
    fx_interp = interp1d(cum_dist, waypoints_x, kind='linear')
    fy_interp = interp1d(cum_dist, waypoints_y, kind='linear')
    
    x_true = fx_interp(d_vals)
    y_true = fy_interp(d_vals)
    
    # Calcolo velocità vettoriale per ogni istante
    vx_true = np.gradient(x_true, dt)
    vy_true = np.gradient(y_true, dt)
    
    return x_true, y_true, vx_true, vy_true, N, duration

def run_simulation(layout_config, prob_nlos, velocity=3.0, dt=0.1, std_meas=1.5, std_proc=0.1, seed=42):
    """
    Esegue la simulazione completa di volo e tracking per un dato layout e probabilità di NLoS.
    """
    # Definizione waypoint specifici per ogni layout (percorsi che non attraversano scaffali)
    if layout_config.name == LAYOUT_A.name:
        waypoints_x = np.array([26.2, 26.2,  9.4,  9.4, 26.2, 26.2])
        waypoints_y = np.array([ 1.25, 38.75, 38.75,  1.25,  1.25, 38.75])
    elif layout_config.name == LAYOUT_B.name:
        waypoints_x = np.array([51.4, 51.4, 17.8, 17.8, 80.8, 80.8, 51.4, 51.4])
        waypoints_y = np.array([ 1.25, 98.75, 98.75, 1.25, 1.25, 98.75, 98.75, 1.25])
    else: # Layout C
        waypoints_x = np.array([127.0, 127.0, 47.2, 47.2, 215.2, 215.2, 127.0])
        waypoints_y = np.array([ 1.25, 138.75, 138.75, 1.25, 1.25, 138.75, 138.75])

    x_true, y_true, vx_true, vy_true, N, duration = calculate_trajectory(waypoints_x, waypoints_y, dt, velocity)

    np.random.seed(seed)
    meas_noise_x = np.random.normal(0, std_meas, N)
    meas_noise_y = np.random.normal(0, std_meas, N)
    
    # Generazione maschera NLoS (Blocchi di ostacoli)
    nlos_mask = generate_burst_nlos(N, prob_nlos, burst_len_avg=30)
    
    ekf = EKF2D(dt, std_meas=std_meas, std_proc=std_proc)
    ekf.state = np.array([x_true[0], y_true[0], vx_true[0], vy_true[0]])
    
    est_x, est_y = np.zeros(N), np.zeros(N)
    rmse = np.zeros(N)
    covs = np.zeros((N, 2, 2))
    
    # LOOP PRINCIPALE DI TRACKING
    for i in range(N):
        # 1. PREDICI la posizione futura
        ekf.predict()
        
        # 2. SE c'è segnale (LoS), AGGIORNA con la misura del sensore
        # SE c'è blocco (NLoS), salta l'update: il filtro va in COASTING (solo predizione)
        z = np.array([x_true[i] + meas_noise_x[i], y_true[i] + meas_noise_y[i]])
        if not nlos_mask[i]:
            ekf.update(z)
            
        # CONSTRAINT FISICO SDN: Il drone non può essere fuori dalle mura
        ekf.state[0] = np.clip(ekf.state[0], 0.0, layout_config.x_dim_m)
        ekf.state[1] = np.clip(ekf.state[1], 0.0, layout_config.y_dim_m)
        
        est_x[i] = ekf.state[0]
        est_y[i] = ekf.state[1]
        
        # Calcolo dell'errore RMSE puntuale
        rmse[i] = np.sqrt((x_true[i] - est_x[i])**2 + (y_true[i] - est_y[i])**2)
        covs[i] = ekf.P[0:2, 0:2] # Salvataggio covarianza per le ellissi nel plot
        
    return {
        "x_true": x_true, "y_true": y_true, "est_x": est_x, "est_y": est_y,
        "rmse": rmse, "covs": covs, "nlos": nlos_mask, "N": N, "duration": duration,
        "t_vals": np.linspace(0, duration, N)
    }

def main():
    print("=========================================================================")
    print("      TEST 1: Accuratezza del Posizionamento e Tracking EKF In-Doorr   ")
    print("=========================================================================")
    
    layouts = [LAYOUT_A, LAYOUT_B, LAYOUT_C]
    scenarios = [(0.05, "Scenario 1 (5% NLoS)"), (0.20, "Scenario 2 (20% NLoS)"), (0.45, "Scenario 3 (45% NLoS)")]
    
    results = {}
    
    # Esecuzione della batteria di test per ogni combinazione Layout/Scenario
    print("[*] Esecuzione Simulazioni Matematiche e Algoritmiche (EKF) in corso...")
    for layout in layouts:
        results[layout.name] = {}
        for prob, scen_name in scenarios:
            res = run_simulation(layout, prob)
            results[layout.name][scen_name] = res
            print(f"    -> {layout.name} | {scen_name} | RMSE Medio: {np.mean(res['rmse']):.2f}m")
            
    # --- GENERAZIONE ELABORATI GRAFICI PER LA TESI ---

    # IMMAGINE 1: Evoluzione temporale dell'errore (RMSE)
    print("\n[*] Generazione Immagine 1 (Test_1_RMSE_Temporale.png)...")
    plt.style.use('seaborn-v0_8-paper')
    fig1, axes1 = plt.subplots(3, 3, figsize=(18, 14))
    fig1.suptitle("Test 1: Evoluzione Temporale dell'Errore EKF (RMSE) multi-layout", fontsize=16, fontweight="bold")
    
    for row_idx, (prob, scen_name) in enumerate(scenarios):
        for col_idx, layout in enumerate(layouts):
            ax = axes1[row_idx, col_idx]
            res = results[layout.name][scen_name]
            ax.plot(res["t_vals"], res["rmse"], color="red", linewidth=1.5, label="Errore RMSE")
            # Shading grigio per i momenti di NLoS (coasting)
            ax.fill_between(res["t_vals"], 0, 1, where=res["nlos"], color='grey', alpha=0.3, 
                           transform=ax.get_xaxis_transform(), label="Blocco NLoS")
            ax.set_title(f"{layout.name}\n{scen_name}", fontweight='bold')
            ax.set_xlabel("Tempo [s]")
            ax.set_ylabel("Errore EKF RMSE [m]")
            ax.set_ylim(0, max(15, np.max(res["rmse"]) * 1.1))
            ax.grid(True, linestyle='--', alpha=0.7)
            if row_idx == 0 and col_idx == 0: ax.legend()
    plt.tight_layout()
    plt.savefig("Test_1_RMSE_Temporale.png", dpi=200)
    plt.close()

    # IMMAGINE 2: CDF (Cumulative Distribution Function) - Analisi Statistica
    print("\n[*] Generazione Immagine 2 (Test_1_CDF_RMSE.png)...")
    fig2, ax2 = plt.subplots(figsize=(10, 7))
    ax2.set_title("Test 1: Probabilità Cumulativa (CDF) Errore Posizionamento", fontweight="bold")
    
    # Nomi precisi per gli assi con notazione matematica
    ax2.set_xlabel("Errore EKF RMSE $\epsilon$ [m]")
    ax2.set_ylabel("Probabilità Cumulativa $P(X \leq \epsilon)$")
    
    probs_cdf = [(0.05, "Scen. 1 (5% NLoS)", "blue"), (0.20, "Scen. 2 (20% NLoS)", "darkorange"), (0.45, "Scen. 3 (45% NLoS)", "red")]
    
    global_max_rmse = 0
    for prob, name, color in probs_cdf:
        res_cdf = run_simulation(LAYOUT_A, prob)
        rmse_sorted = np.sort(res_cdf["rmse"])
        cdf = np.arange(1, len(rmse_sorted) + 1) / len(rmse_sorted)
        ax2.plot(rmse_sorted, cdf, label=name, color=color, linewidth=2.5)
        global_max_rmse = max(global_max_rmse, np.max(rmse_sorted))
    
    # Aggiornamento scala: Aggiungiamo un margine del 10% al massimo errore trovato
    ax2.set_xlim(0, global_max_rmse * 1.1)
    ax2.set_ylim(0, 1.05) # Per vedere bene quando tocca il tetto del 100%
    ax2.legend(loc="lower right", fontsize=12)
    ax2.grid(True, linestyle='--', alpha=0.7)
    plt.savefig("Test_1_CDF_RMSE.png", dpi=200)
    plt.close()

    # IMMAGINE 3: Mappa Top-Down (Traiettoria reale vs stimata nel magazzino)
    print("\n[*] Generazione Immagine 3 (Test_1_TopDown_Traiettoria.png)...")
    layout_trj = LAYOUT_B
    res_trj = results[layout_trj.name]["Scenario 2 (20% NLoS)"]
    env_trj = Environment(layout_trj)
    fig3, ax3 = plt.subplots(figsize=(10, 10))
    ax3.set_title(f"Test 1: Tracking EKF vs Ground Truth\n({layout_trj.name} - Viaggio Ottimizzato)", fontweight="bold")
    
    # Rendering Mura
    ax3.plot([0, layout_trj.x_dim_m, layout_trj.x_dim_m, 0, 0], [0, 0, layout_trj.y_dim_m, layout_trj.y_dim_m, 0], color="black", linewidth=2.5)
    
    # Rendering Scaffali dal Modulo 2
    shelf_xy = set()
    for cx, cy, cz in env_trj.shelf_centers: shelf_xy.add((cx, cy))
    hx, hy = layout_trj.shelf_x_m / 2, layout_trj.shelf_y_m / 2
    first = True
    for cx, cy in shelf_xy:
        ax3.add_patch(Rectangle((cx - hx, cy - hy), layout_trj.shelf_x_m, layout_trj.shelf_y_m, 
                                linewidth=1, edgecolor='lightgray', facecolor='#E0E0E0', 
                                label="Scaffali" if first else ""))
        first = False
        
    # Plot Traiettorie
    ax3.plot(res_trj["x_true"], res_trj["y_true"], 'b-', linewidth=2.5, label="Ground Truth (Reale)")
    ax3.plot(res_trj["x_true"][0], res_trj["y_true"][0], marker='P', color='green', markersize=14, label="Start")
    ax3.plot(res_trj["est_x"], res_trj["est_y"], 'r--', linewidth=2.0, label="Stima EKF")
    
    # Legenda Ellissi
    ax3.add_patch(Ellipse((0, 0), 1, 1, facecolor='red', alpha=0.3, label="Incertezza EKF (P)"))
    
    # Rendering Ellissi di Covarianza (ogni 5 secondi)
    for i in range(0, res_trj["N"], int(5.0 / 0.1)):
        val, vec = np.linalg.eigh(res_trj["covs"][i])
        width, height = 2 * 3 * np.sqrt(val)
        ax3.add_patch(Ellipse(xy=(res_trj["est_x"][i], res_trj["est_y"][i]), width=width, height=height, 
                              angle=np.degrees(np.arctan2(vec[1, 0], vec[0, 0])), edgecolor='red', facecolor='red', alpha=0.3))
        
    ax3.set_aspect('equal')
    ax3.legend(loc="upper center", bbox_to_anchor=(0.5, -0.05), ncol=3)
    plt.savefig("Test_1_TopDown_Traiettoria.png", dpi=300)
    plt.close()

    print("\n[✔] Elaborati architetturali Test 1 generati con successo.\n")

if __name__ == "__main__":
    main()
