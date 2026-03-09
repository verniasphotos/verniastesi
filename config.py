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
import math # Libreria per calcoli matematici

class Magazzino:
    """ Rappresenta l'ambiente fisico del magazzino 3D.
    Calcola il layout (scaffali e corridoi) partendo dalle dimensioni (L, W, H)
    e posiziona in base ad esse l'hardware di rete 6G (BS e RIS). """

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
        
        # Generazione griglia di scaffali
        for fila in range(n_file_y):
            y_min = fila * passo_y
            y_max = y_min + W_SCAFFALE
            
            # Il centro del corridoio si trova dopo lo scaffale
            y_centro_corridoio = y_max + (LARGHEZZA_CORRIDOIO / 2.0)
            self.corridoi.append(y_centro_corridoio)

            for colonna in range(n_elementi_x):
                x_min = colonna * L_SCAFFALE
                x_max = x_min + L_SCAFFALE
                
                # Scatola di Collisione 3D per gli ostacoli (scaffali)
                self.scaffali.append({
                    'id': id_scaffale,
                    'file_name': f"C{fila+1:02d}-S{colonna+1:02d}", # Es. C01-S05
                    'x_min': x_min, 'x_max': x_max,
                    'y_min': y_min, 'y_max': y_max,
                    'z_min': 0.0,   'z_max': self.h_scaffale
                })
                id_scaffale += 1 # Contatore

        # Deploy Hardware di Rete 6G (Posizionamento Antenne)
        
        # 1. Base Station (BS): Deploy a griglia per magazzini grandi (Ridondanza e Copertura)
        self.base_stations = []
        z_bs = self.altezza - Z_BS_OFFSET_DAL_SOFFITTO
        
        # Distanza tra due BS per garantire sovrapposizione (overlap)
        # Invece di 2*R_BS (nessun overlap), usiamo 1.5*R_BS (forte ridondanza e continuità di segnale)
        distanza_bs = R_BS * 1.5 
        
        # Calcoliamo quante BS servono per coprire asse X e Y
        n_bs_x = math.ceil(self.lunghezza / distanza_bs)
        n_bs_y = math.ceil(self.larghezza / distanza_bs)
        
        # Assicuriamo almeno 1 BS
        n_bs_x = max(1, n_bs_x)
        n_bs_y = max(1, n_bs_y)
        
        # Ricalcoliamo il passo esatto per distribuirle uniformemente (copertura)
        passo_x = self.lunghezza / n_bs_x if n_bs_x > 1 else self.lunghezza
        passo_y = self.larghezza / n_bs_y if n_bs_y > 1 else self.larghezza
        
        id_bs = 0 # ID base station inizializzato a 0
        for ix in range(n_bs_x):
            for iy in range(n_bs_y):
                # Se c'è solo 1 BS per asse, la mettiamo al centro (es. magazzini piccoli)
                # Altrimenti le distribuiamo uniformemente
                x_bs = (self.lunghezza / 2.0) if n_bs_x == 1 else (passo_x / 2.0) + (ix * passo_x)
                y_bs = (self.larghezza / 2.0) if n_bs_y == 1 else (passo_y / 2.0) + (iy * passo_y)
                
                self.base_stations.append({
                    'id': id_bs,
                    'x': x_bs,
                    'y': y_bs,
                    'z': z_bs
                })
                id_bs += 1

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

    # BOM : documento che dice quanti pezzi fisici servono per costruire un progetto
    def get_bom(self):
        """Calcola e restituisce la Bill of Materials (distinta base),
           cioè l'inventario matematico dell'ambiente generato """
        
        return {
            'n_scaffali': len(self.scaffali),
            'n_corridoi': len(self.corridoi),
            'n_livelli_mensola': self.n_livelli_mensola,
            'h_scaffale_totale': self.h_scaffale,
            'n_base_station': len(self.base_stations),
            'n_ris_soffitto': len(self.ris_soffitto),
            'n_ris_parete': len(self.ris_parete)
        }

    # LOS : Line of Sight (linea di vista) 
    def check_LOS_and_shielding(self, p1, p2):
        """ Simula la propagazione 6G "sparando" un raggio lineare da P1 a P2 
            e contando quanti ostacoli metallici (scaffali) incrocia """
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


# Esecuzione del programma di configurazione magazzino 6G
if __name__ == "__main__":
    print("=" * 60)
    print(" BENVENUTO NEL CONFIGURATORE MAGAZZINO 6G ".center(60, '='))
    print("=" * 60)
    print("Inserisci le dimensioni reali del magazzino per calcolare")
    print("l'hardware necessario (Base Stations e RIS) e ricevere")
    print("un consiglio sul dimensionamento della flotta droni.\n")

    try:
        # 1. Acquisizione Dati dall'Utente
        input_l = float(input(" -> Lunghezza del magazzino (in metri): "))
        input_w = float(input(" -> Larghezza del magazzino (in metri): "))
        input_h = float(input(" -> Altezza del magazzino (in metri): "))
        
        # 2. Generazione dell'Ambiente
        print("\n... Generazione ambiente 3D in corso ...")
        ambiente = Magazzino(lunghezza=input_l, larghezza=input_w, altezza=input_h)
        bom = ambiente.get_bom()
        
        # 3. Raccomandazione Flotta Droni
        # Costruiamo una logica empirica: 
        # Magazzini piccoli (< 5 corridoi): 1 drone per corsia
        # Magazzini grandi (>= 5 corridoi): 1 drone ogni 2 corsie per evitare congestione
        n_corr = bom['n_corridoi']
        if n_corr < 5:
            droni_consigliati = n_corr * 1
        else:
            droni_consigliati = max(5, int(n_corr / 2))
            
        print("\n" + "=" * 60)
        print(" REPORT INFRASTRUTTURA ".center(60, ' '))
        print("=" * 60)
        print(f" ► Dimensioni: {input_l}x{input_w}x{input_h} metri")
        print(f" ► Scaffali metallici generati: {bom['n_scaffali']}")
        print(f" ► Livelli di mensole: {bom['n_livelli_mensola']}")
        print(f" ► Corridoi di volo: {bom['n_corridoi']}")
        
        print("\n--- Hardware 6G Necessario ---")
        print(f" 📡 Base Stations (Raggio {R_BS}m): {bom['n_base_station']} (con griglia di sovrapposizione)")
        print(f" 🪞 Pannelli RIS a Soffitto: {bom['n_ris_soffitto']}")
        print(f" 🪞 Pannelli RIS a Parete: {bom['n_ris_parete']}")

        print("\n--- Flotta Droni ---")
        print(f" 🚁 Numero di Droni consigliato per non saturare la rete: {droni_consigliati}")
        
        print("\n--- Spiegazione Modalità di Volo Attuale ---")
        if MODALITA_VOLO_DRONE == 'FISSO':
            print("Modalità corrente: [FISSO]")
            print(" -> I droni voleranno tutti alla stessa quota di sicurezza (Z fissa).")
            print(" -> È la modalità più semplice, previene incidenti verticali ma gestisce")
            print("    meno traffico. I droni si alzeranno/abbasseranno solo arrivati")
            print("    davanti allo scaffale bersaglio per compiere l'operazione.")
        elif MODALITA_VOLO_DRONE == 'MULTILIVELLO':
            print("Modalità corrente: [MULTILIVELLO]")
            print(" -> I droni verranno assegnati a corridoi orizzontali su quote (Z) diverse.")
            print(" -> Modalità avanzata: permette a più droni di operare simultaneamente")
            print("    sopra lo stesso tratto di corridoio su piani sfalsati. Il traffico")
            print("    di rete sarà più denso e intenso.")

        print("=" * 60)
        
    except ValueError:
        print("\n[ERRORE] Inserimento non valido. Devi inserire un numero (usa i punti per i decimali, es: 10.5). Riprova.")

