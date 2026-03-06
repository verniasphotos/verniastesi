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

# --- Anti-Collisione Droni in Modalità FISSO ---
# In modalità FISSO tutti i droni volano alla stessa quota Z. 
# È quindi fondamentale che il Controller garantisca una distanza minima in LUNGHEZZA del corridoio
# tra un drone e l'altro per evitare sovrapposizioni fisiche.
# Questo vincolo è critico per magazzini PICCOLI e MEDI dove i corridoi sono brevi
# e il numero di droni per corsia può essere elevato rispetto alla lunghezza disponibile.
MIN_DISTANZA_ANTICOLLISIONE = 1.5  # Distanza minima (metri) tra due droni nello stesso corridoio (modo FISSO).
                                   # Il Controller in devices.py non assegnerà a un drone
                                   # una posizione target se un altro drone è già entro questa distanza.

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
W_SCAFFALE = 1.0           # Profondita' (asse Y) di un singolo modulo scaffalatura (metri)
# NOTA: L'ALTEZZA TOTALE DEGLI SCAFFALI non e' definita qui perche' e' una variabile del
# layout e viene calcolata dinamicamente in environment.py in base al numero di livelli/mensole.

# --- Identificatori e Struttura Scaffali ---
# Ogni scaffale generato in environment.py ricevera' un ID intero progressivo (0, 1, 2...).
# Viene anche costruita una stringa leggibile nel formato: C{corsia}-S{scaffale}-L{livello}
# Esempio: C01-S05-L02 = Corsia 1, Scaffale n.5, Livello di mensola 2
# Questo formato e' lo standard industriale dei magazzini logistici (usato da Amazon, DHL ecc.)
#
# NOTA: Il numero di mensole NON è fisso qui. Dipende dall'altezza totale del magazzino 
# (Caso A=basso, Caso C=alto). Verrà calcolato in environment.py dinamicamente.

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

# --- Strategia di Deployment RIS (soffitto e parete) ---
# Due tipi di RIS con ruoli distinti:
#
#   SOFFITTO: 1 RIS centrata sopra ogni corridoio. Ha visione diretta sul drone (no scaffali in mezzo).
#             Usata per il tracking POSIZIONALE (X, Y) del drone lungo la corsia.
#             Attiva in ENTRAMBE le modalità di volo (FISSO e MULTILIVELLO).
#
#   PARETE:   1 RIS per lato, a metà altezza, per discriminare la QUOTA (Z) del drone.
#             Utile principalmente in modalità MULTILIVELLO (volo su più livelli di mensola).
#             In modalità FISSO la quota è nota a priori → la parete è opzionale.

Z_BS_OFFSET_DAL_SOFFITTO = 0.3     # BS a soffitto: offset (in metri) sotto l'intradosso
Z_RIS_SOFFITTO_OFFSET = 0.1        # RIS a soffitto: offset (in metri) sotto l'intradosso
Z_RIS_PARETE_RAPPORTO_ALTEZZA = 0.5 # RIS a parete: frazione dell'altezza totale (0.5 = metà parete)

# --- Abilitazione RIS per modalità di volo ---
# Controlla quali tipi di RIS vengono deployate in base alla modalità drone scelta.
RIS_SOFFITTO_ABILITATA = True       # Sempre True: 1 RIS a soffitto per corridoio, per il tracking XY
RIS_PARETE_ABILITATA_FISSO = False  # In modalità FISSO la quota è nota → parete opzionale (risparmio)
RIS_PARETE_ABILITATA_MULTILIVELLO = True # In MULTILIVELLO serve la parete per discriminare la quota Z

# --- Ottimizzazione Copertura (minimizzare il numero di RIS/BS) ---
# Il sistema calcolerà il numero minimo di RIS/BS per coprire l'intera area del magazzino.
# La logica: piazzo la prima RIS, calcolo la sua area coperta, piazzo la prossima dove la copertura finisce.
COPERTURA_TARGET = 1.0             # Frazione dell'area da coprire (1.0 = 100%): obiettivo piena copertura
SOGLIA_OVERLAP_RIS = 0.10          # Overlap massimo accettato tra due RIS adiacenti (10%).
                                   # Troppo overlap = RIS sprecate; troppo poco = buchi di copertura.


# --- Limiti Stress Test ---
MAX_RIS_CALLS_PER_DT = 10  # Massimo numero di attivazioni RIS gestibili dal server in un singolo DT (0.1s)
                           # Superato questo limite -> la rete va in Breakdown (condizione di stop test)
