# ==========================================
# MODULO 1: COSTANTI E PARAMETRI (config.py)
# ==========================================
# Questo file contiene tutti i parametri fisici e le costanti della simulazione.
# Modificare questi valori cambierà il comportamento dei droni, dei RIS e della rete.

# --- Parametri di Rete 6G ---
FREQ = 3.5e9               # Frequenza del segnale in Hertz (3.5 GHz)
TX_POWER_DRONE = 20.0      # Potenza di trasmissione del Drone in dBm
R_BS = 50.0                # Raggio di copertura massimo della Base Station (metri)
R_RIS = 15.0               # Raggio di copertura effettivo di un pannello RIS (metri)

# --- Parametri Cinematici dei Droni ---
V_DRONE = 3.0              # Velocità di volo costante del Drone in metri al secondo (m/s)
DT = 0.1                   # Passo temporale della simulazione (1 step = 0.1 secondi)

# --- Parametri della Batteria ---
BATTERY_MAX = 100.0        # Capacità massima della batteria del drone (%)
BATTERY_RTH_THRESHOLD = 20.0 # Soglia di batteria sotto la quale il drone torna a ricaricarsi (%), RTH = Return To Home
CONSUMO_BATTERIA_DT = 0.05 # Percentuale di batteria consumata ad ogni passo temporale (DT)

# --- Consumi Energetici dei Pannelli RIS (Watt) ---
P_SLEEP = 0.5              # Consumo del pannello RIS quando è "spento" o a riposo (W)
P_PASSIVE = 5.0            # Consumo del pannello RIS quando devia i segnali in modo "passivo" (W)
P_ACTIVE = 50.0            # Consumo del pannello RIS quando amplifica attivamente il segnale (W)

# --- Dimensioni Scaffali Magazzino (metri) ---
L_SCAFFALE = 1.2           # Lunghezza media di un modulo scaffale (metri)
W_SCAFFALE = 1.0           # Larghezza/Profondità di un modulo scaffale (metri)
