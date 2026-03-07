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

# ==========================================
# MODULO 2: GEOMETRIA E MAGAZZINO (Step 2)
# ==========================================
import math

class Magazzino:
    """
    Rappresenta l'ambiente fisico del magazzino 3D.
    Calcola il layout (scaffali e corridoi) partendo dalle dimensioni (L, W, H)
    e posiziona in base ad esse l'hardware di rete 6G (BS e RIS).
    """

    def __init__(self, lunghezza, larghezza, altezza):
        # 1. Dimensioni Fisiche del magazzino (metri)
        self.lunghezza = lunghezza  # Asse X
        self.larghezza = larghezza  # Asse Y
        self.altezza = altezza      # Asse Z

        # Calcolo del numero di livelli fisici basato sull'altezza del magazzino.
        # Lasciamo un margine del soffitto per le antenne/droni.
        margine_soffitto = 1.0
        spazio_utile_z = max(0, self.altezza - margine_soffitto)
        self.n_livelli_mensola = int(spazio_utile_z // H_MENSOLA)
        
        # L'altezza EFFETTIVA dello scaffale
        self.h_scaffale = self.n_livelli_mensola * H_MENSOLA

        # --- Generazione Layout (Corridoi e Scaffali) ---
        self.scaffali = []  # Conterrà dizionari con i dati di ciascuno scaffale
        self.corridoi = []  # Coordinate Y del centro di ogni corsia
        
        # Larghezza di un corridoio (spazio vuoto tra due scaffali).
        LARGHEZZA_CORRIDOIO = 1.5

        # Quanti moduli "scaffale + corridoio" stanno lungo l'asse Y (larghezza)?
        passo_y = W_SCAFFALE + LARGHEZZA_CORRIDOIO
        
        n_file_y = int(self.larghezza // passo_y)
        if n_file_y == 0: n_file_y = 1 # Minimo 1 fila
        
        # Quanti scaffali (L_SCAFFALE) stanno lungo l'asse X (lunghezza)?
        n_elementi_x = int(self.lunghezza // L_SCAFFALE)

        id_scaffale = 0
        
        for fila in range(n_file_y):
            y_min = fila * passo_y
            y_max = y_min + W_SCAFFALE
            
            # Il centro del corridoio si trova dopo lo scaffale
            y_centro_corridoio = y_max + (LARGHEZZA_CORRIDOIO / 2.0)
            self.corridoi.append(y_centro_corridoio)

            for colonna in range(n_elementi_x):
                x_min = colonna * L_SCAFFALE
                x_max = x_min + L_SCAFFALE
                
                # Bounding Box 3D
                self.scaffali.append({
                    'id': id_scaffale,
                    'file_name': f"C{fila+1:02d}-S{colonna+1:02d}", # Es. C01-S05
                    'x_min': x_min, 'x_max': x_max,
                    'y_min': y_min, 'y_max': y_max,
                    'z_min': 0.0,   'z_max': self.h_scaffale
                })
                id_scaffale += 1

        # --- Deploy Hardware di Rete 6G (Posizionamento Antenne) ---
        
        # 1. Base Station (BS): La posizioniamo al centro del soffitto del magazzino
        z_bs = self.altezza - Z_BS_OFFSET_DAL_SOFFITTO
        self.base_stations = [{
            'id': 0,
            'x': self.lunghezza / 2.0,
            'y': self.larghezza / 2.0,
            'z': z_bs
        }]

        # 2. Pannelli RIS (Reconfigurable Intelligent Surfaces)
        self.ris_soffitto = []
        self.ris_parete = []
        id_ris = 0

        # RIS a Soffitto (1 per ogni corridoio, centrata a metà corridoio)
        if RIS_SOFFITTO_ABILITATA:
            z_ris_soffitto = self.altezza - Z_RIS_SOFFITTO_OFFSET
            for corridoio_y in self.corridoi:
                self.ris_soffitto.append({
                    'id': id_ris,
                    'tipo': 'soffitto',
                    'x': self.lunghezza / 2.0, # Centrata sulla lunghezza
                    'y': corridoio_y,          # Centrata sul corridoio
                    'z': z_ris_soffitto
                })
                id_ris += 1

        # RIS a Parete (opzionali in base alla modalità di volo)
        if (MODALITA_VOLO_DRONE == 'FISSO' and RIS_PARETE_ABILITATA_FISSO) or \
           (MODALITA_VOLO_DRONE == 'MULTILIVELLO' and RIS_PARETE_ABILITATA_MULTILIVELLO):
            z_ris_parete = self.altezza * Z_RIS_PARETE_RAPPORTO_ALTEZZA
            for corridoio_y in self.corridoi:
                # 2 RIS per corridoio, una all'inizio (X=0) e una alla fine (X=lunghezza)
                self.ris_parete.append({
                    'id': id_ris, 'tipo': 'parete_start',
                    'x': 0.0, 'y': corridoio_y, 'z': z_ris_parete
                })
                id_ris += 1
                self.ris_parete.append({
                    'id': id_ris, 'tipo': 'parete_end',
                    'x': self.lunghezza, 'y': corridoio_y, 'z': z_ris_parete
                })
                id_ris += 1

    def get_bom(self):
        """
        Calcola e restituisce la Bill of Materials (distinta base),
        cioè l'inventario matematico dell'ambiente generato.
        """
        return {
            'n_scaffali': len(self.scaffali),
            'n_corridoi': len(self.corridoi),
            'n_livelli_mensola': self.n_livelli_mensola,
            'h_scaffale_totale': self.h_scaffale,
            'n_base_station': len(self.base_stations),
            'n_ris_soffitto': len(self.ris_soffitto),
            'n_ris_parete': len(self.ris_parete)
        }

    def check_LOS_and_shielding(self, p1, p2):
        """
        Simula la propagazione 6G "sparando" un raggio lineare da P1 a P2 
        e contando quanti ostacoli metallici (scaffali) incrocia.
        """
        x1, y1, z1 = p1
        x2, y2, z2 = p2
        
        ostacoli_attraversati = 0
        
        # Bounding Box globale del segmento per escludere scaffali lontani
        min_x, max_x = min(x1, x2), max(x1, x2)
        min_y, max_y = min(y1, y2), max(y1, y2)
        
        for scaffale in self.scaffali:
            # 1. CONTROLLO VELOCE ASSE Z:
            # Se raggio è interamente sopra lo scaffale -> no collisione
            if z1 > scaffale['z_max'] and z2 > scaffale['z_max']:
                continue 
            
            # 2. CONTROLLO VELOCE ASSE X, Y:
            if scaffale['x_max'] < min_x or scaffale['x_min'] > max_x:
                continue
            if scaffale['y_max'] < min_y or scaffale['y_min'] > max_y:
                continue
                
            # 3. INTERSEZIONE GEOMETRICA 2D:
            # Usiamo un test di intersezione tra segmenti sul piano X-Y.
            dx = x2 - x1
            dy = y2 - y1
            
            if dx == 0 and dy == 0:
                continue # Punti coincidenti
                
            # Tempi di intersezione con le linee infinite X e Y dello scaffale
            t_min_x = (scaffale['x_min'] - x1) / dx if dx != 0 else float('-inf')
            t_max_x = (scaffale['x_max'] - x1) / dx if dx != 0 else float('inf')
            
            if t_min_x > t_max_x: t_min_x, t_max_x = t_max_x, t_min_x
            
            t_min_y = (scaffale['y_min'] - y1) / dy if dy != 0 else float('-inf')
            t_max_y = (scaffale['y_max'] - y1) / dy if dy != 0 else float('inf')
            
            if t_min_y > t_max_y: t_min_y, t_max_y = t_max_y, t_min_y
            
            # Gestione casi verticali/orizzontali
            if dx == 0:
                if x1 < scaffale['x_min'] or x1 > scaffale['x_max']:
                    continue
                t_min_x = float('-inf')
                t_max_x = float('inf')
            
            if dy == 0:
                if y1 < scaffale['y_min'] or y1 > scaffale['y_max']:
                    continue
                t_min_y = float('-inf')
                t_max_y = float('inf')
                
            # Intersezione degli intervalli [tx_min, tx_max] e [ty_min, ty_max]
            t_in = max(t_min_x, t_min_y)
            t_out = min(t_max_x, t_max_y)
            
            # C'è intersezione se t_in <= t_out
            if t_in <= t_out:
                # E se l'intersezione avviene DENTRO il segmento [0, 1]
                if t_in <= 1.0 and t_out >= 0.0:
                    ostacoli_attraversati += 1
                    
        return ostacoli_attraversati

if __name__ == "__main__":
    # Test del Modulo (Magazzino Piccolo)
    print("--- Test Generazione Magazzino ---")
    ambiente = Magazzino(lunghezza=30.0, larghezza=20.0, altezza=8.0)
    
    bom = ambiente.get_bom()
    for k, v in bom.items():
         print(f" - {k}: {v}")
         
    print("\n--- Test Ranging (LOS/NLOS) ---")
    pa = (5.0, ambiente.corridoi[0], Z_DRONE_FISSO)
    pb = (20.0, ambiente.corridoi[0], pa[2])
    ostacoli = ambiente.check_LOS_and_shielding(pa, pb)
    print(f"Test 1 (Stesso corridoio): {ostacoli} ostacoli (Atteso: 0)")
    
    if len(ambiente.corridoi) > 1:
        pc = (5.0, ambiente.corridoi[1], pa[2])
        ostacoli_2 = ambiente.check_LOS_and_shielding(pa, pc)
        print(f"Test 2 (Attraversa corsia): {ostacoli_2} ostacoli (Atteso: >0)")
