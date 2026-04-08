import os # Modulo standard di Python per interagire con il sistema operativo (es. creare cartelle, gestire percorsi file)
import numpy as np # Libreria potentissima per il calcolo scientifico e statistico (veloce perché scritta in C sotto il cofano)
import matplotlib.pyplot as plt # Libreria fondamentale per creare grafici e visualizzazioni scientifiche
import matplotlib.patches as patches # Modulo specifico di Matplotlib per disegnare forme geometriche (rettangoli, cerchi, ecc.)
from matplotlib.patches import Ellipse # Importiamo specificamente la classe Ellipse per disegnare le ellissi di covarianza

# =====================================================================
# FUNZIONI GEOMETRICHE E RAY-CASTING (NLOS)
# =====================================================================

def ccw(A, B, C):
    """
    Verifica se i punti A, B, C sono in ordine antiorario.
    Utilizzato per il test di intersezione tra segmenti.
    """
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

def intersect(A, B, C, D):
    """
    Determina se il segmento AB interseca il segmento CD.
    Algoritmo basato sull'orientamento dei punti (CCW).
    """
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

def check_nlos(drone_pos, bs_pos, shelves):
    """
    Rilevamento Non-Line-of-Sight (NLoS).
    Controlla se il segmento tra drone e Base Station interseca uno degli scaffali (rettangoli).
    """
    A = drone_pos
    B = bs_pos
    for (sx, sy, sw, sh) in shelves:
        # Coordinate dei vertici dello scaffale
        xmin, xmax = sx, sx + sw
        ymin, ymax = sy, sy + sh
        
        # 1. Controllo se drone o BS sono dentro lo scaffale
        if (xmin <= A[0] <= xmax and ymin <= A[1] <= ymax) or \
           (xmin <= B[0] <= xmax and ymin <= B[1] <= ymax):
            return True
            
        # 2. Controllo intersezione del raggio visivo con i 4 lati dello scaffale
        R1, R2, R3, R4 = (xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)
        
        # Lati: Inferiore, Destro, Superiore, Sinistro
        if intersect(A, B, R1, R2) or intersect(A, B, R2, R3) or \
           intersect(A, B, R3, R4) or intersect(A, B, R4, R1):
            return True
            
    return False

# =====================================================================
# STARTUP DEGLI SCENARI E TRAIETTORIE
# =====================================================================

def generate_scenario(layout_type, dt=0.5, total_time=100.0):
    """
    Genera la geometria del magazzino e la traiettoria nominale (Ground Truth).
    Parametrizzato per i 3 layout logistici richiesti dalla tesi.
    """
    t_vals = np.arange(0, total_time, dt)
    
    if layout_type == 'A':
        # Scenario 1: Spazio piccolo, pochi scaffali, bassa occlusione
        width, height = 50.0, 40.0
        shelves = [
            (10, 10, 10, 5), (30, 10, 10, 5),
            (10, 25, 10, 5), (30, 25, 10, 5)
        ]
        # Traiettoria base ovale
        omega = 2 * np.pi / total_time
        x_true = width/2 + width * 0.35 * np.cos(omega * t_vals)
        y_true = height/2 + height * 0.35 * np.sin(omega * t_vals)
        description = "Layout A"

    elif layout_type == 'B':
        # Scenario 2: Spazio medio, densità standard di scaffali
        width, height = 100.0, 80.0
        shelves = []
        for sx in [15, 65]:
            for sy in [10, 35, 60]:
                shelves.append((sx, sy, 20, 10))
                
        # Traiettoria a Figura 8 (Lissajous)
        omega = 2 * np.pi / total_time
        x_true = width/2 + width * 0.35 * np.sin(omega * t_vals * 2)
        y_true = height/2 + height * 0.35 * np.sin(omega * t_vals)
        description = "Layout B"

    elif layout_type == 'C':
        # Scenario 3: Stress Test - Grande spazio, VNA (Very Narrow Aisles), blocco severo
        width, height = 200.0, 150.0
        shelves = []
        # Generazione automatica corridoi stretti
        for sx in np.arange(20, 180, 20):
            if sx != 100: # Spazio libero per la BS centrale
                shelves.append((sx, 15, 5, 50))
                shelves.append((sx, 85, 5, 50))
                
        # Traiettoria a sweep sinusoidale per attraversare molti corridoi (massimo NLoS)
        x_true = 10 + (width - 20) * (t_vals / total_time)
        y_true = height/2 + (height * 0.35) * np.sin(6 * np.pi * t_vals / total_time)
        description = "Layout C - Stress Test"
        
    else:
        raise ValueError("Layout non supportato")
        
    return width, height, shelves, x_true, y_true, t_vals, description

# =====================================================================
# MOTORE: EXTENDED KALMAN FILTER (EKF) 2D
# =====================================================================

class EKF2D:
    """
    Filtro di Kalman Esteso per il tracciamento drone.
    Modello a velocità costante (Constant Velocity Model).
    """
    def __init__(self, dt, init_state, init_P, q_noise, r_noise):
        self.dt = dt
        # Stato X: [posizione_x, posizione_y, velocità_x, velocità_y]^T
        self.X = np.array(init_state, dtype=float).reshape(4, 1)
        # Matrice di Covarianza dello stato P
        self.P = np.array(init_P, dtype=float)
        
        # Modello di transizione cinematica F
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0,  dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1]
        ])
        
        # Matrice di osservazione H (misuriamo solo la posizione x, y)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # Costruzione matrice di rumore di processo Q (process noise)
        G = np.array([
            [dt**2 / 2, 0],
            [0, dt**2 / 2],
            [dt, 0],
            [0, dt]
        ])
        self.Q = (G @ G.T) * q_noise
        
        # Matrice di rumore di misura R (measurement noise)
        self.R = np.eye(2) * r_noise

    def predict(self):
        """Fase di Predizione: stima dello stato futuro e aumento dell'incertezza."""
        self.X = self.F @ self.X
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, Z):
        """Fase di Update: correzione dello stato basata sulla misura reale."""
        Z = np.array(Z).reshape(2, 1)
        # Innovazione Y = Misura - Stima
        Y = Z - (self.H @ self.X)
        
        # Calcolo del guadagno di Kalman K
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Aggiornamento stato e covarianza
        self.X = self.X + (K @ Y)
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P

# =====================================================================
# UTILITIES PER MATPLOTLIB
# =====================================================================

def plot_covariance_ellipse(ax, pos, P, n_std=3.0, **kwargs):
    """
    Estrae autovalori e autovettori dalla sottomatrice 2x2 di P (posizione)
    per disegnare l'ellisse di incertezza al livello n_std (sigma).
    """
    P_pos = P[:2, :2]
    vals, vecs = np.linalg.eigh(P_pos)
    
    # Calcolo dei semiassi e rotazione
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    theta = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
    
    width, height = 2 * n_std * np.sqrt(vals)
    ellip = Ellipse(xy=pos, width=width, height=height, angle=theta, **kwargs)
    ax.add_patch(ellip)

# =====================================================================
# FUNZIONE MAIN (TEST 2)
# =====================================================================

def run_test_2():
    """Parametri principali ed esecuzione del test comparativo sui 3 layout."""
    layouts = ['A', 'B', 'C']
    
    # Inizializzazione figura accademica
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    
    dt = 0.5
    total_time = 100.0
    
    print("\n" + "="*70)
    print(" AVVIO TEST 2: Precisione Traiettoria 2D e Covarianza EKF ")
    print("="*70)

    for idx, layout in enumerate(layouts):
        ax = axes[idx]
        # Generazione scenario e traiettoria vera
        width, height, shelves, x_true, y_true, t_vals, desc = generate_scenario(layout, dt, total_time)
        
        # Base Station fissa al centro geometrico
        bs_pos = (width / 2.0, height / 2.0)
        
        # Inizializzazioni EKF
        init_state = [x_true[0], y_true[0], 0.0, 0.0]
        init_P = np.eye(4) * 5.0
        q_noise = 0.5   # Incertezza del modello drone
        r_noise = 2.0   # Rumore Gaussiano del sensore posizione
        
        ekf = EKF2D(dt, init_state, init_P, q_noise, r_noise)
        
        est_states = []
        est_covs = []
        nlos_flags = []
        
        # --- Ciclo di Simulazione ---
        for k in range(len(t_vals)):
            true_pos = (x_true[k], y_true[k])
            
            # 1. Fase EKF Predict (Sempre eseguita)
            ekf.predict()
            
            # 2. Controllo Ray-Casting per NLoS (Blocco visivo Base Station)
            nlos = check_nlos(true_pos, bs_pos, shelves)
            nlos_flags.append(nlos)
            
            # 3. Fase EKF Update (Solo se in Line-of-Sight)
            if not nlos:
                # Simulazione misura rumorosa (Sensore virtuale)
                meas_x = true_pos[0] + np.random.normal(0, np.sqrt(r_noise))
                meas_y = true_pos[1] + np.random.normal(0, np.sqrt(r_noise))
                ekf.update([meas_x, meas_y])
            else:
                # In NLoS l'EKF fa "coasting": la covarianza P aumenta visibilmente
                pass 
            
            est_states.append(ekf.X.flatten())
            est_covs.append(ekf.P.copy())
            
        est_states = np.array(est_states)
        x_est, y_est = est_states[:, 0], est_states[:, 1]
        
        # --- Calcolo Metriche Precisione ---
        error = np.sqrt((x_true - x_est)**2 + (y_true - y_est)**2)
        max_error = np.max(error)
        avg_error = np.mean(error)
        
        print(f"[{desc}]")
        print(f"  > Maximum Cross-Track Error: {max_error:6.2f} m")
        print(f"  > Frequenza Blocchi NLoS:    {sum(nlos_flags)} / {len(t_vals)} steps\n")
        
        # --- Sezione Plotting ---
        # Disegno Scaffali logistici
        for i, (sx, sy, sw, sh) in enumerate(shelves):
            rect = patches.Rectangle((sx, sy), sw, sh, color='gray', alpha=0.4, 
                                     label="Ostacolo (NLoS)" if (i==0 and idx==0) else "")
            ax.add_patch(rect)
            
        # Base Station (Triangolo Giallo)
        ax.plot(bs_pos[0], bs_pos[1], '^', color='gold', markersize=14, 
                markeredgecolor='black', label="Base Station 6G" if idx==0 else "")
        
        # Ground Truth (Nera continua)
        ax.plot(x_true, y_true, 'k-', linewidth=2, label="Traiettoria Reale" if idx==0 else "")
        
        # Stima EKF (Rossa tratteggiata)
        ax.plot(x_est, y_est, 'r--', linewidth=2, label="Stima EKF" if idx==0 else "")
        
        # Marker occlusione NLoS
        nlos_x = [x_true[i] for i in range(len(t_vals)) if nlos_flags[i]]
        nlos_y = [y_true[i] for i in range(len(t_vals)) if nlos_flags[i]]
        if nlos_x:
            ax.scatter(nlos_x, nlos_y, c='orange', marker='x', s=20, alpha=0.8,
                       label="Area NLoS" if idx==0 else "")
            
        # Ellissi di Covarianza (Visualizzazione incertezza)
        step_ellipse = max(1, len(t_vals) // 25)
        for i in range(0, len(t_vals), step_ellipse):
            plot_covariance_ellipse(ax, pos=(x_est[i], y_est[i]), P=est_covs[i], 
                                    n_std=3.0, edgecolor='red', facecolor='red', alpha=0.15)
            
        if idx == 0:
            ax.plot([], [], 'o', color='red', alpha=0.3, markersize=10, label="Incertezza EKF (3σ)")

        # Estetica Subplot
        ax.set_title(f"Scenario {idx+1}: {desc}", fontsize=13, fontweight='bold')
        ax.set_xlim(0, width); ax.set_ylim(0, height)
        ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")
        ax.grid(True, linestyle=':', alpha=0.7)
        ax.set_aspect('equal', adjustable='box')

    # Configurazione finale legenda globale e salvataggio
    fig.legend(loc='lower center', ncol=6, bbox_to_anchor=(0.5, -0.05), fontsize=11, frameon=True)
    plt.tight_layout(rect=[0, 0.05, 1, 1])
    
    output_path = os.path.join(os.path.dirname(__file__), "test_2_risultati_ekf.png")
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    
    print("="*70)
    print(f"Analisi completata. Immagine salvata in: {output_path}")
    print("="*70 + "\n")
    plt.show()

if __name__ == "__main__":
    run_test_2()
