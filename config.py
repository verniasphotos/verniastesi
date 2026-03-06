# ==========================================
# MODULO 1: COSTANTI E PARAMETRI (config.py)
# ==========================================
# Questo file contiene tutti i parametri fisici e le costanti della simulazione.
# Modificare questi valori cambierà il comportamento dei droni, dei RIS e della rete.
# NOTA: Le quote assolute (es. altezza BS, RIS) dipendono dall'altezza TOTALE del magazzino
# che viene calcolata e fornita dal Modulo 2 (environment.py). Qui definiamo solo OFFSET e COSTANTI.

# --- Parametri di Rete 6G e Propagazione ---
FREQ = 3.5e9               # Frequenza del segnale in Hertz (3.5 GHz)
TX_POWER_DRONE = 20.0      # Potenza di trasmissione del Drone in dBm
R_BS = 50.0                # Raggio di copertura massimo della Base Station (metri)
R_RIS = 15.0               # Raggio di copertura effettivo di un pannello RIS (metri)
ATTENUAZIONE_SCAFFALE = 15.0 # Attenuazione fissa del segnale per ogni scaffale attraversato (dB)
SOGLIA_RIS_ATTIVAZIONE = 5.0 # Soglia SNR (dB) sotto la quale il Controller accende un RIS
SOGLIA_RICEVITORE = -90.0  # Sensibilità minima del ricevitore (dBm): sotto questa soglia il drone è disconnesso
RUMORE_BIANCO = -100.0     # Potenza del rumore di fondo nel magazzino logistico (dBm)

# --- NOTE: La Base Station (BS) in questa simulazione funge ANCHE da RIS ---
# La BS non è solo un punto di ricezione, ma è in grado di riflettere e reindirizzare
# i segnali dei droni, agendo come un'Antenna Intelligente. Questo è coerente con
# l'architettura Centralizzata 6G descritta nella tesi.
BS_E_ANCHE_RIS = True      # Flag: True = la BS può operare anche come nodo RIS

# --- Parametri Cinematici dei Droni ---
V_DRONE = 3.0              # Velocità di volo costante del Drone in m/s (≈ 10.8 km/h)
DT = 0.1                   # Passo temporale della simulazione (1 step = 0.1 secondi)

# --- Parametri della Batteria ---
BATTERY_MAX = 100.0        # Capacità massima della batteria del drone (%)
BATTERY_RTH_THRESHOLD = 20.0 # Soglia RTH (Return To Home): il drone torna alla base se scende sotto questo valore (%)
CONSUMO_BATTERIA_DT = 0.05 # Percentuale batteria consumata ad ogni step temporale DT

# --- Consumi Energetici dei Pannelli RIS (Watt) ---
P_SLEEP = 0.5              # Consumo a riposo: pannello in ascolto ma inattivo (W)
P_PASSIVE = 5.0            # Consumo in modalità passiva: pannello riflette segnali senza amplificarli (W)
P_ACTIVE = 50.0            # Consumo in modalità attiva: pannello amplifica e ridirige il segnale (W)

# --- Dimensioni Scaffalature (metri) ---
L_SCAFFALE = 1.2           # Lunghezza (asse X) di un singolo modulo scaffalatura (metri)
W_SCAFFALE = 1.0           # Profondità (asse Y) di un singolo modulo scaffalatura (metri)
# NOTA: L'ALTEZZA TOTALE DEGLI SCAFFALI non è definita qui perché è una variabile del
# layout e viene calcolata dinamicamente in environment.py in base al numero di livelli/mensole.

# --- Parametri Mensole (Piano-Ripiano) ---
H_MENSOLA = 0.6            # Altezza standard tra una mensola e la successiva (metri)
                           # (Tipico standard industriale: 50-70cm di luce tra i ripiani)
MARGINE_SICUREZZA_DRONE = 0.15 # Margine di sicurezza aggiuntivo sopra la mensola (metri)
                           # Il drone vola a H_MENSOLA - MARGINE_SICUREZZA_DRONE per non rischiare collisioni
                           # Questo definisce la "SafeZone" di volo in ogni corridoio di livello.

# --- Modalità di Volo dei Droni ---
# Il drone può operare in due modalità distinte (scegliere UNA delle due):
# 'FISSO'       = Il drone vola sempre alla stessa altezza fissa nei corridoi.
#                 Si alza/abbassa SOLO quando arriva allo scaffale target per raccogliere il pacco.
# 'MULTILIVELLO'= Il drone vola su corridoi orizzontali a quote multiple (una per ogni livello mensola).
#                 Il sistema assegna il drone al livello corrispondente al pacco da recuperare.
MODALITA_VOLO_DRONE = 'FISSO'  # <-- MODIFICA QUI per cambiare il comportamento di navigazione
Z_DRONE_FISSO = 1.0           # Altezza di crociera fissa per la modalità 'FISSO' (metri)
                               # (Generalmente il corridoio al primo livello del pavimento)

# --- Offset di Installazione RIS e BS (relativi all'altezza del magazzino) ---
# Le quote assolute si calcolano in environment.py come: Z = altezza_magazzino - OFFSET
# Esempio: se il magazzino è alto 10m e Z_BS_OFFSET=0.5, la BS sarà installata a 9.5m
Z_BS_OFFSET_DAL_SOFFITTO = 0.3    # BS montata SUL SOFFITTO a questo offset verso il basso (metri)
Z_RIS_PARETE_OFFSET_SOFFITTO = 1.0 # RIS a PARETE: installata a questa distanza dal soffitto (metri)
Z_RIS_SOFFITTO_OFFSET = 0.1       # RIS a SOFFITTO: offset rispetto all'intradosso del soffitto (metri)

# --- Limiti Stress Test ---
MAX_RIS_CALLS_PER_DT = 10  # Massimo numero di attivazioni RIS gestibili dal server in un singolo DT (0.1s)
                           # Superato questo limite -> la rete va in Breakdown (condizione di stop test)
