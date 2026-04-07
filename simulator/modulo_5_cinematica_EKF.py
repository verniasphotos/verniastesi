# MODULO 5: UAV Dynamics & Tracking Engine
import numpy as np
import math
from filterpy.kalman import ExtendedKalmanFilter
from .modulo_2_environment import Environment, CollisionError
from .modulo_1_config import UAVSpecs

class UAVKinematics:
    """
    [5.2] UAV Physics Engine
    Si occupa di simulare il modello di stato cinetico reale del drone
    (posizione, velocità, inerzia) nello spazio 3D.
    """
    def __init__(self, start_pos, dt=0.1):
        self.dt = dt
        # Stato Reale: [x, y, z, vx, vy, vz]
        self.state = np.array([start_pos[0], start_pos[1], start_pos[2], 0.0, 0.0, 0.0], dtype=float)
        self.gravity = 9.81
    
    def update_physics(self, ax, ay, az):
        """
        Aggiorna la fisica del drone basandosi sulle accelerazioni fornite (ax, ay, az).
        Discretizzazione al passo dt=0.1s.
        """
        # Aggiungiamo l'effetto gravitazionale e l'inerzia
        # Aggiornamento Velocità: v(t) = v(t-1) + a * dt
        self.state[3] += ax * self.dt
        self.state[4] += ay * self.dt
        
        # Se l'acceleratore Z è inferiore a gravità, il drone cade
        self.state[5] += (az - self.gravity) * self.dt 
        
        # Aggiornamento Posizione: p(t) = p(t-1) + v(t) * dt
        self.state[0] += self.state[3] * self.dt
        self.state[1] += self.state[4] * self.dt
        self.state[2] += max(0.0, self.state[5] * self.dt) # Evitiamo che vada nel sottosuolo brutalmente
        
        return self.state[:3].copy() # Ritorna X, Y, Z

class UAVTrackerEKF:
    """
    [5.3] Extended Kalman Filter (EKF)
    Il filtro che cerca di "indovinare" la posizione vera del drone basandosi sui sensori
    rumorosi (GPS approssimato e RSSI/SNR che fluttua).
    """
    def __init__(self, initial_guess, dt=0.1):
        self.dt = dt
        # Creiamo un filtro EKF con 6 variabili di stato (x, y, z, vx, vy, vz)
        # e 3 misure in ingresso (x_gps, y_gps, z_gps) misurate
        self.ekf = ExtendedKalmanFilter(dim_x=6, dim_z=3)
        
        # Inizializziamo lo stato Stimato (la nostra scommessa iniziale)
        self.ekf.x = np.array([initial_guess[0], initial_guess[1], initial_guess[2], 0.0, 0.0, 0.0])
        
        # Matrice di Transizione di Stato (F) - Matrice della Cinematica
        self.ekf.F = np.eye(6)
        self.ekf.F[0, 3] = dt
        self.ekf.F[1, 4] = dt
        self.ekf.F[2, 5] = dt
        
        # Matrice di Misurazione (H) - Mappa quali variabili leggono i sensori
        self.ekf.H = np.zeros((3, 6))
        self.ekf.H[0, 0] = 1.0 # Sensore legge X
        self.ekf.H[1, 1] = 1.0 # Sensore legge Y
        self.ekf.H[2, 2] = 1.0 # Sensore legge Z
        
        # Matrice di Covarianza dell'Errore (P) - Incertezza iniziale
        self.ekf.P *= 10.0 
        
        # Matrice di Rumore di Processo (Q)
        # L'incertezza sul fatto che le nostre formule fisiche siano perfette (es. colpi di vento)
        self.ekf.Q = np.eye(6) * 0.1
        
        # Matrice di Rumore di Misurazione (R)
        # Quanto rumore/errore c'è nei sensori reali (RSSI molto fluttuante = R alto)
        self.ekf.R = np.eye(3) * 2.0 
        
    def H_jacobian(self, x):
        """Jacobiano della matrice H (Nel nostro caso è lineare per la posizione)"""
        return self.ekf.H
        
    def hx(self, x):
        """Funzione di misura non lineare (nel nostro estraiamo xyz)"""
        return np.array([x[0], x[1], x[2]])

    def predict_and_update(self, z):
        """
        Esegue la "Predict" (previsione basata sulla formula matematica)
        e l'"Update" (correzione basata sulla lettura del sensore ricalcolata in R)
        """
        # 1. Previsione: Muovi la posizione stimata in avanti in base alla velocità calcolata al ciclo prima
        self.ekf.predict()
        
        # 2. Correzione: Applica la misurazione z (lettura del GPS/RSSI) e valuta la Q e R
        self.ekf.update(z, HJacobian=self.H_jacobian, Hx=self.hx)
        
        return self.ekf.x[:3].copy() # Ritorna la stima X, Y, Z corretta

class TrackingManager:
    """
    Manager di alto livello per il volo. 
    Gestisce la simulazione unendo la Fisica del Drone reale con la Stima (EKF), 
    e sorveglia le collisioni con l'Ambiente (KDTree).
    """
    def __init__(self, start_pos, env: Environment):
        self.env = env
        self.physics = UAVKinematics(start_pos)
        self.tracker = UAVTrackerEKF(start_pos)
        
    def step_fly(self, control_accel, noisy_sensor_reading):
        """
        [5.4] Calcolo Metriche (RMSE)
        Ogni 0.1s il drone spinge i motori (control_accel) e il controller riceve dati (noisy_sensor).
        """
        # 1. Drone "Si muove" FISICAMENTE
        real_pos = self.physics.update_physics(control_accel[0], control_accel[1], control_accel[2])
        
        # 2. L'Antenna "Riceve" la posizione SPORCA (noisy_sensor_reading) e l'EKF la pulisce
        estimated_pos = self.tracker.predict_and_update(noisy_sensor_reading)
        
        # 3. [RMSE] Errore metrico quadratico. 
        # La distanza in metri (3D) tra dove è veramente il drone e dove il computer pensa che sia!
        rmse_error = np.linalg.norm(real_pos - estimated_pos)
        
        # 4. Sicurezza Critica (Collision Check dal Modulo 2)
        # Se l'errore o l'incertezza diventano critiche e potremmo schiantarci!
        if rmse_error > 1.5:
            try:
                # Verifica con l'ambiente se siamo a meno di 1.5 m da un ostacolo reale
                self.env.validate_clearance(real_pos, margin=1.5)
            except CollisionError as e:
                # Gestiamo e passiamo l'eccezione se c'è veramente la collisione
                raise e
            
        return real_pos, estimated_pos, rmse_error

# BLOCCO DI TEST / DEBUG DEL MODULO (Per test e spiegazione database):
if __name__ == "__main__":
    from .modulo_1_config import LayoutConfig
    import sqlite3
    import os
    
    print("Inizializzazione Modulo 5...")
    layout_A = LayoutConfig("Layout A", 50.0, 40.0, 10.0, 0, [])
    env = Environment(layout_A)
    
    start = np.array([20.0, 20.0, 2.0])
    manager = TrackingManager(start_pos=start, env=env)
    
    # Creiamo un Database SQLite locale per l'esercizio di telemetria e lo popoliamo
    db_path = "telemetry_test.db"
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE IF NOT EXISTS flight_tracker
                      (id INTEGER PRIMARY KEY AUTOINCREMENT, 
                       step INTEGER,
                       real_x REAL, real_y REAL, real_z REAL,
                       est_x REAL, est_y REAL, est_z REAL,
                       rmse REAL)''')
    
    print("\\nSimulazione Volo di Test e salvataggio nel DB...")
    
    # Facciamo fare al drone 50 "passi" di tempo in avanti
    for step in range(50):
        # Spinta motori (Volo in avanti in diagonale, mantiene quota sfidando gravità)
        accel = np.array([0.2, 0.1, 9.81]) 
        
        # Nel mondo reale, l'accelerata causa il movimento.
        temp_real = manager.physics.state[:3] + accel * 0.1
        
        # Il sensore a bordo è guasto o sporcato dal metallo e inserisce "Rumore" (Random)
        rumore = np.random.normal(0, 1.5, 3) # Fino a 1.5 metri di offset casuale
        sensor_sporco = temp_real + rumore
        
        # Chiamiamo il Tracker che usa il Kalman Filter (EKF)
        real_p, est_p, error = manager.step_fly(accel, sensor_sporco)
        
        # Inseriamo riga per riga nel database locale!
        cursor.execute('''INSERT INTO flight_tracker 
                          (step, real_x, real_y, real_z, est_x, est_y, est_z, rmse)
                          VALUES (?, ?, ?, ?, ?, ?, ?, ?)''',
                      (step, real_p[0], real_p[1], real_p[2], est_p[0], est_p[1], est_p[2], error))
        
    conn.commit()
    conn.close()
    print(f"Salvato database SQLite: {os.path.abspath(db_path)}")
    print("\\nEKF Test Terminato con Successo!")
