# ==========================================
# MODULO 1: COSTANTI E PARAMETRI
# ==========================================

# Parametri di Rete 6G e Propagazione

FREQ = 3.5e9                 # Frequenza del segnale in Hertz (3.5 GHz)
TX_POWER_DRONE = 20.0        # Potenza di trasmissione del Drone in dBm
R_BS = 50.0                  # Raggio di copertura massimo della Base Station (metri)
R_RIS = 15.0                 # Raggio di copertura effettivo di un pannello RIS (metri)
ATTENUAZIONE_SCAFFALE = 15.0 # Attenuazione fissa del segnale per ogni scaffale attraversato (dB)
SOGLIA_RIS_ATTIVAZIONE = 5.0 # Soglia SNR (dB) sotto la quale il Controller accende un RIS
SOGLIA_RICEVITORE = -90.0    # Sensibilità minima del ricevitore (dBm): sotto questa soglia il drone è disconnesso
RUMORE_BIANCO = -100.0       # Potenza del rumore di fondo nel magazzino logistico (dBm)

# ------------------------------------------
# PARAMETRI DRONI 
# ------------------------------------------

#Parametri Cinematici dei Droni
V_DRONE = 3.0              # Velocità di volo costante del Drone in m/s (≈ 10.8 km/h)
DT = 0.1                   # Passo temporale della simulazione (1 step = 0.1 secondi)

#Parametri della Batteria drone
BATTERY_MAX = 100.0        # Capacità massima della batteria del drone (%)
BATTERY_RTH_THRESHOLD = 20.0 # Soglia RTH (Return To Home): il drone torna alla base se scende sotto questo valore (%)
CONSUMO_BATTERIA_DT = 0.05 # Percentuale batteria consumata ad ogni step temporale  

# Modalità di Volo dei Droni e Sicurezza
# Il drone può operare in due modalità distinte (ora scelte AUTOMATICAMENTE dal simulatore in base all'area):
# 'FISSO'       = Magazzini Piccoli/Medi. Il drone vola sempre alla stessa altezza fissa nei corridoi.
#                 Si alza/abbassa SOLO quando arriva allo scaffale target per raccogliere il pacco.
# 'MULTILIVELLO'= Magazzini Grandi. Il drone vola su corridoi orizzontali a quote multiple (una per ogni livello mensola).

Z_DRONE_FISSO = 1.0           # Altezza di crociera fissa per la modalità 'FISSO' (metri)
                               # (Generalmente il corridoio al primo livello del pavimento)

# Anti-Collisione Droni
# Regola UNIVERSALE (valida per FISSO e su ogni singolo livello del MULTILIVELLO). 
# È fondamentale che il Controller garantisca una distanza minima orizzontale
# tra un drone e l'altro per evitare sovrapposizioni fisiche nella stessa corsia.
# Questo vincolo è critico per calcolare la saturazione del magazzino.
MIN_DISTANZA_ANTICOLLISIONE = 1.5  # Distanza minima (metri) tra due droni nello stesso tubo di volo.
                                   # Il Controller in devices.py non assegnerà a un drone
                                   # una posizione target se un altro drone è già entro questa distanza.

# ------------------------------------------
# PARAMETRI BS-RIS
# ------------------------------------------

# La Base Station (BS) è in modalità ibrida: ricevitore + RIS
BS_E_ANCHE_RIS = True      # Flag: True = la BS può operare anche come nodo RIS

# Consumi Energetici dei Pannelli RIS (Watt)
P_SLEEP = 0.5              # Consumo a riposo: pannello in ascolto ma inattivo (W)
P_PASSIVE = 5.0            # Consumo in modalità passiva: pannello riflette segnali senza amplificarli (W)
P_ACTIVE = 50.0            # Consumo in modalità attiva: pannello amplifica e ridirige il segnale (W)

# Parametri di posizionamento delle RIS e BS (soffitto e parete)

Z_BS_OFFSET_DAL_SOFFITTO = 0.3     # BS a soffitto: offset (in metri) sotto l'intradosso
Z_RIS_SOFFITTO_OFFSET = 0.1        # RIS a soffitto: offset (in metri) sotto l'intradosso
Z_RIS_PARETE_RAPPORTO_ALTEZZA = 0.5 # RIS a parete: frazione dell'altezza totale (0.5 = metà parete)

#Abilitazione RIS per modalità di volo -> Controlla quali tipi di RIS vengono deployate in base alla modalità drone scelta.

RIS_SOFFITTO_ABILITATA = True       # Sempre True: 1 RIS a soffitto per corridoio, per il tracking XY
RIS_PARETE_ABILITATA_FISSO = False  # In modalità FISSO la quota è nota → parete opzionale (risparmio)
RIS_PARETE_ABILITATA_MULTILIVELLO = True # In MULTILIVELLO serve la parete per discriminare la quota Z

# Ottimizzazione Copertura (minimizzare il numero di RIS/BS) 
# Il sistema calcolerà il numero minimo di RIS/BS per coprire l'intera area del magazzino.
# La logica: piazzo la prima RIS, calcolo la sua area coperta, piazzo la prossima dove la copertura finisce.
COPERTURA_TARGET = 1.0             # Frazione dell'area da coprire (1.0 = 100%): obiettivo piena copertura
SOGLIA_OVERLAP_RIS = 0.10          # Overlap massimo accettato tra due RIS adiacenti (10%).
                                   # Troppo overlap = RIS sprecate; troppo poco = buchi di copertura.


# ------------------------------------------
# PARAMETRI EXTRA
# ------------------------------------------
# Dimensioni Scaffalature (metri)
L_SCAFFALE = 1.2           # Lunghezza (asse X) di un singolo modulo scaffalatura (metri)
W_SCAFFALE = 1.0           # Profondita' (asse Y) di un singolo modulo scaffalatura (metri)
# NOTA: L'ALTEZZA TOTALE DEGLI SCAFFALI non e' definita qui perche' e' una variabile del
# layout e viene calcolata dinamicamente in nel blocco 2 in base al numero di livelli/mensole

# Identificatori e Struttura Scaffali
H_MENSOLA = 0.6            # Altezza standard tra una mensola e la successiva (metri)
                           # (Tipico standard industriale: 50-70cm di luce tra i ripiani)
MARGINE_SICUREZZA_DRONE = 0.15 # Margine di sicurezza aggiuntivo sopra la mensola (metri)
                           # Il drone vola a H_MENSOLA - MARGINE_SICUREZZA_DRONE per non rischiare collisioni
                           # Questo definisce la "SafeZone" di volo in ogni corridoio di livello.

# Limiti Stress Test
MAX_RIS_CALLS_PER_DT = 10  # Massimo numero di attivazioni RIS gestibili dal server in un singolo DT (0.1s)
                           # Superato questo limite -> la rete va in Breakdown (condizione di stop test)



# ==========================================
# MODULO 2: GEOMETRIA E MAGAZZINO
# ==========================================
import math # Libreria per calcoli matematici

class Magazzino:
    """ Rappresenta l'ambiente fisico del magazzino 3D.
    Calcola il layout (scaffali e corridoi) partendo dalle dimensioni (L, W, H)
    e posiziona in base ad esse l'hardware di rete 6G (BS e RIS) """

    def __init__(self, lunghezza, larghezza, altezza):
        # 1. Dimensioni Fisiche del magazzino (metri)
        self.lunghezza = lunghezza  # Asse X
        self.larghezza = larghezza  # Asse Y
        self.altezza = altezza      # Asse Z
        self.area_mq = self.lunghezza * self.larghezza

        # Calcolo del numero di livelli fisici basato sull'altezza del magazzino
        # Lasciamo un margine del soffitto per le antenne/droni
        margine_soffitto = 1.0
        spazio_utile_z = max(0, self.altezza - margine_soffitto)
        self.n_livelli_mensola = int(spazio_utile_z // H_MENSOLA)
        
        # L'altezza EFFETTIVA dello scaffale
        self.h_scaffale = self.n_livelli_mensola * H_MENSOLA

        # Generazione Layout (Corridoi e Scaffali) 
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
        
        # Base Station (BS): Deploy a griglia per magazzini grandi (Ridondanza e Copertura)
        self.base_stations = []
        z_bs = self.altezza - Z_BS_OFFSET_DAL_SOFFITTO
        
        # Distanza tra due BS per garantire sovrapposizione (overlap)
        distanza_bs = R_BS * 1.5 #usiamo 1.5 per garantire una ridondanza e continuità di segnale
        
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

        # Pannelli RIS (Reconfigurable Intelligent Surfaces)
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

        # Decisione Intelligente: Modalità di Volo
        # Se il magazzino e' >= 10.000 mq, si attiva il MULTILIVELLO per gestire flotte grandi
        if self.area_mq >= 10000:
            self.modalita_volo = 'MULTILIVELLO'
        else:
            self.modalita_volo = 'FISSO'

        # RIS a Parete (dinamiche base alla modalità di volo calcolata)
        if (self.modalita_volo == 'FISSO' and RIS_PARETE_ABILITATA_FISSO) or \
           (self.modalita_volo == 'MULTILIVELLO' and RIS_PARETE_ABILITATA_MULTILIVELLO):
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
        
    # 2. BOM : documento che dice quanti pezzi fisici servono per costruire un progetto
    def get_bom(self):
        """Calcola e restituisce la Bill of Materials (distinta base),
           cioè l'inventario matematico dell'ambiente generato """
        
        return {
            'modalita_volo': self.modalita_volo,
            'n_scaffali': len(self.scaffali),
            'n_corridoi': len(self.corridoi),
            'n_livelli_mensola': self.n_livelli_mensola,
            'h_scaffale_totale': self.h_scaffale,
            'n_base_station': len(self.base_stations),
            'n_ris_soffitto': len(self.ris_soffitto),
            'n_ris_parete': len(self.ris_parete)
        }

    # 3. LOS : Line of Sight (linea di vista) 
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


# ==========================================
# MODULO 3: ENTITÀ DELLA SIMULAZIONE E HARDWARE
# ==========================================

import time

class Pacchetto_Rete: 
    """ Rappresenta il pacchetto dati inviato dal drone alla Base Station """
    def __init__(self, id_drone, tx_power, battery_level, package_id, target_x, target_y, target_z):
        # Header (Intestazione del messaggio radio)
        self.id_drone = id_drone
        self.ts = time.time()  # Timestamp corrente (quando viene creato il pacchetto)
        self.tx_power = tx_power
        self.battery_level = battery_level
        
        # Tracciamento Multi-Hop (Il percorso che fa il pacchetto)
        # Quando nasce, il primo nodo attraversato è ovviamente il Drone stesso.
        self.nodi_attraversati = [f"Drone-{self.id_drone}"]
        
        # Payload (Dati utili del messaggio logistico)
        self.package_id = package_id
        self.route_target = (target_x, target_y, target_z)

    def aggiungi_hop_ris(self, id_ris):
        """ Aggiunge la "firma" del pannello RIS che ha riflesso il segnale """
        self.nodi_attraversati.append(f"RIS-{id_ris}")
        
    def aggiungi_hop_bs(self, id_bs):
        """ Aggiunge la "firma" della Base Station che ha ricevuto il segnale """
        self.nodi_attraversati.append(f"BS-{id_bs}")

    def __repr__(self):
        """ Come viene stampato il pacchetto a schermo (utile per il debugging) """
        percorso_str = " -> ".join(self.nodi_attraversati)
        return (f"<Pacchetto_Rete [{percorso_str}] | Batt:{self.battery_level:.1f}% | "
                f"TxPwr:{self.tx_power}dBm>")


class Drone:
    """ Rappresenta l'entità Drone fisico e la sua logica di volo """
    def __init__(self, id_drone, x, y, z):
        self.id_drone = id_drone
        self.stato_missione = 'IN_MISSIONE' # Stati possibili: 'IN_MISSIONE', 'RTH_RICARICA'
        
        # Posizione spaziale 3D (metri)
        self.x = x
        self.y = y
        self.z = z
        
        # Stato Energetico
        self.batteria = BATTERY_MAX
        
    def aggiorna_batteria(self):
        """ Simula il consumo della batteria del drone ad ogni step (DT) """
        self.batteria -= CONSUMO_BATTERIA_DT
        
        # Se la batteria scende sotto la soglia di sicurezza, il drone deve tornare alla base
        if self.batteria <= BATTERY_RTH_THRESHOLD and self.stato_missione != 'RTH_RICARICA':
            self.stato_missione = 'RTH_RICARICA'
            # (In futuro, imposteremo come target la Base Station più vicina)

    def muovi_verso(self, target_x, target_y, target_z):
        """ Sposta il drone verso le coordinate bersaglio di un passettino pari a V_DRONE * DT """
        # 1. Calcolo la distanza totale verso il target (distanza euclidea 3D)
        dx = target_x - self.x
        dy = target_y - self.y
        dz = target_z - self.z
        distanza_totale = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # 2. Se è già arrivato (o quasi) evito di muoverlo e di dividere per zero
        if distanza_totale < 0.1:
            self.x, self.y, self.z = target_x, target_y, target_z
            return True # Restituisce True per dire "Sono arrivato!"
            
        # 3. Calcolo di quanto si muove in questo step temporale
        passo_lineare = V_DRONE * DT
        
        # Se il passo lineare supera la distanza totale, lo piazzo esattamente sul target
        if passo_lineare >= distanza_totale:
            self.x, self.y, self.z = target_x, target_y, target_z
            return True
            
        # 4. Movimento proporzionale: aggiungo alla posizione attuale la frazione di spostamento corretta
        versore_x = dx / distanza_totale
        versore_y = dy / distanza_totale
        versore_z = dz / distanza_totale
        
        self.x += versore_x * passo_lineare
        self.y += versore_y * passo_lineare
        self.z += versore_z * passo_lineare
        
        return False # Non è ancora arrivato al target
        
    def genera_pacchetto(self, package_id, target_x, target_y, target_z):
        """ Crea il pacchetto radio da spedire via 6G alla Base Station """
        return Pacchetto_Rete(
            id_drone=self.id_drone,
            tx_power=TX_POWER_DRONE,
            battery_level=self.batteria,
            package_id=package_id,
            target_x=target_x,
            target_y=target_y,
            target_z=target_z
        )


class RIS:
    """ Rappresenta il pannello Reconfigurable Intelligent Surface (lo specchio 6G) """
    def __init__(self, id_ris, x, y, z):
        self.id_ris = id_ris
        self.x = x
        self.y = y
        self.z = z
        self.stato = 'sleep' # Stati possibili: 'sleep', 'passive', 'active'
        
    def cambia_stato(self, nuovo_stato):
        """ Il Controller chiama questo metodo per accendere/spegnere il pannello """
        stati_validi = ['sleep', 'passive', 'active']
        if nuovo_stato in stati_validi:
            self.stato = nuovo_stato
            
    def get_consumo(self):
        """ Restituisce l'attuale consumo in Watt basato sullo stato del pannello """
        if self.stato == 'sleep':
            return P_SLEEP
        elif self.stato == 'passive':
            return P_PASSIVE
        elif self.stato == 'active':
            return P_ACTIVE
        return 0.0

    def inoltra_pacchetto(self, pacchetto):
        """ 
        [MULTI-HOP]
        Il pannello riceve il segnale, appone la sua firma per tracciamento 
        e idealmente lo riflette verso la Base Station. 
        """
        # Aggiungiamo il timbro del RIS per dire "è passato di qua"
        pacchetto.aggiungi_hop_ris(self.id_ris)
        
        # Se il RIS è attivo (amplifica), aumentiamo artificialmente 
        # la potenza del segnale contenuta nel pacchetto di un +10 dBm
        if self.stato == 'active':
            pacchetto.tx_power += 10.0
            
        return pacchetto

# ==========================================
# MODULO 4: FISICA DEL CANALE E PROPAGAZIONE
# ==========================================

def calcola_path_loss_dB(distanza):
    """
    Calcola l'attenuazione del segnale nello spazio libero (Free Space Path Loss).
    Usa la formula di Friis: PL = 20*log10(d) + 20*log10(f) + 20*log10(4*pi/c)
    """
    if distanza <= 0.1:  # Evita errori logaritmo per distanze minuscole
        distanza = 0.1
        
    c = 3e8  # Velocità della luce in m/s
    # Calcolo dell'attenuazione base dovuta alla distanza fisica
    path_loss = 20 * math.log10(distanza) + 20 * math.log10(FREQ) + 20 * math.log10((4 * math.pi) / c)
    return path_loss

def esegui_2way_ranging(drone, bs_dict, magazzino, ris_list=None):
    """
    Simula il protocollo 2-Way Ranging.
    Calcola l'attenuazione totale del segnale (Distanza + Ostacoli Metallici).
    Se il segnale diretto Drone <-> BS è troppo debole, prova a cercare una RIS.
    """
    # 1. Coordinate di partenza (Drone) e arrivo (Base Station)
    p_drone = (drone.x, drone.y, drone.z)
    p_bs = (bs_dict['x'], bs_dict['y'], bs_dict['z'])
    
    # 2. Calcolo Distanza Fisica (Euclidea)
    dx = p_drone[0] - p_bs[0]
    dy = p_drone[1] - p_bs[1]
    dz = p_drone[2] - p_bs[2]
    distanza = math.sqrt(dx**2 + dy**2 + dz**2)
    
    # 3. Path Loss (indebolimento naturale nello spazio)
    path_loss = calcola_path_loss_dB(distanza)
    
    # 4. Attenuazione da Ostacoli (Effetto Scaffali Metallici)
    ostacoli = magazzino.check_LOS_and_shielding(p_drone, p_bs)
    attenuazione_ostacoli = ostacoli * ATTENUAZIONE_SCAFFALE
    
    # 5. Attenuazione Totale e Calcolo SNR (Signal-To-Noise Ratio)
    # L'SNR è la "qualità" del segnale: Potenza Trasmessa - Perdite - Rumore di fondo
    attenuazione_totale = path_loss + attenuazione_ostacoli
    snr_diretto = TX_POWER_DRONE - attenuazione_totale - RUMORE_BIANCO
    
    risultato = {
        'distanza_m': distanza,
        'ostacoli_n': ostacoli,
        'attenuazione_totale_dB': attenuazione_totale,
        'snr_diretto_dB': snr_diretto,
        'connesso': snr_diretto >= SOGLIA_RICEVITORE,
        'usa_ris': False,
        'id_ris_scelta': None
    }
    
    # 6. Logica RIS (Specchi Intelligenti)
    # Se il segnale è sotto la soglia di allerta, e abbiamo RIS disponibili, cerchiamo aiuto
    if snr_diretto < SOGLIA_RIS_ATTIVAZIONE and ris_list and len(ris_list) > 0:
        miglior_snr = snr_diretto
        miglior_ris = None
        
        for ris in ris_list:
            p_ris = (ris['x'], ris['y'], ris['z'])
            
            # Quanti ostacoli ci sono tra Drone e RIS? E tra RIS e BS?
            ost_drone_ris = magazzino.check_LOS_and_shielding(p_drone, p_ris)
            ost_ris_bs = magazzino.check_LOS_and_shielding(p_ris, p_bs)
            
            # Se la RIS ha visuale libera sul Drone e sulla BS (0 ostacoli)
            if ost_drone_ris == 0 and ost_ris_bs == 0:
                # Calcola distanza totale (Drone->RIS + RIS->BS)
                d1 = math.sqrt((p_drone[0]-p_ris[0])**2 + (p_drone[1]-p_ris[1])**2 + (p_drone[2]-p_ris[2])**2)
                d2 = math.sqrt((p_bs[0]-p_ris[0])**2 + (p_bs[1]-p_ris[1])**2 + (p_bs[2]-p_ris[2])**2)
                
                # Calcola nuova attenuazione (sui due tratti e aggiungendo il guadagno teorico della RIS attiva)
                pl1 = calcola_path_loss_dB(d1)
                pl2 = calcola_path_loss_dB(d2)
                # Ipotizziamo un "guadagno" di 10 dB se usiamo una RIS attiva
                attenuazione_via_ris = (pl1 + pl2) - 10.0 
                
                snr_via_ris = TX_POWER_DRONE - attenuazione_via_ris - RUMORE_BIANCO
                
                # Troviamo la RIS che offre l'SNR migliore
                if snr_via_ris > miglior_snr:
                    miglior_snr = snr_via_ris
                    miglior_ris = ris['id']
                    
        # Se abbiamo trovato una RIS che migliora davvero la situazione
        if miglior_ris is not None and miglior_snr >= SOGLIA_RICEVITORE:
            risultato['usa_ris'] = True
            risultato['id_ris_scelta'] = miglior_ris
            risultato['snr_effettivo_dB'] = miglior_snr
            risultato['connesso'] = True
        else:
            risultato['snr_effettivo_dB'] = snr_diretto
    else:
        risultato['snr_effettivo_dB'] = snr_diretto
        
    return risultato







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
        print(f"  Base Stations (Raggio {R_BS}m): {bom['n_base_station']} (con griglia di sovrapposizione)")
        print(f"  Pannelli RIS a Soffitto: {bom['n_ris_soffitto']}")
        print(f"  Pannelli RIS a Parete: {bom['n_ris_parete']}")

        print("\n--- Flotta Droni ---")
        print(f" 🚁 Numero di Droni consigliato per non saturare la rete: {droni_consigliati}")
      
        print("\n--- Spiegazione Modalità di Volo Attuale ---")
        if ambiente.modalita_volo == 'FISSO':
            print("Modalità corrente: [FISSO]")
            print(" -> I droni voleranno tutti alla stessa quota di sicurezza (Z fissa).")
            print(" -> È la modalità più semplice, previene incidenti verticali ma gestisce")
            print("    meno traffico. I droni si alzeranno/abbasseranno solo arrivati")
            print("    davanti allo scaffale bersaglio per compiere l'operazione.")
        elif ambiente.modalita_volo == 'MULTILIVELLO':
            print("Modalità corrente: [MULTILIVELLO]")
            print(" -> I droni verranno assegnati a corridoi orizzontali su quote (Z) diverse.")
            print(" -> Modalità avanzata: permette a più droni di operare simultaneamente")
            print("    sopra lo stesso tratto di corridoio su piani sfalsati. Il traffico")
            print("    di rete sarà più denso e intenso.")

        print("\n--- Test Modulo 4: Fisica del Canale (2-Way Ranging) ---")
        if len(ambiente.base_stations) > 0:
            # Creiamo un drone fittizio per il test, posizionato in un angolo in basso
            drone_test = Drone(id_drone=99, x=1.0, y=1.0, z=1.0)
            bs_target = ambiente.base_stations[0]
            
            # Combiniamo tutte le RIS disponibili per il test
            tutte_le_ris = ambiente.ris_soffitto + ambiente.ris_parete
            
            risultati_ranging = esegui_2way_ranging(drone_test, bs_target, ambiente, tutte_le_ris)
            
            print(f" Posizione Drone: X={drone_test.x:.1f}, Y={drone_test.y:.1f}, Z={drone_test.z:.1f}")
            print(f" Posizione BS(0): X={bs_target['x']:.1f}, Y={bs_target['y']:.1f}, Z={bs_target['z']:.1f}")
            print(f" > Distanza Drone-BS: {risultati_ranging['distanza_m']:.2f} metri")
            print(f" > Scaffali attraversati: {risultati_ranging['ostacoli_n']}")
            print(f" > Attenuazione totale: {risultati_ranging['attenuazione_totale_dB']:.1f} dB")
            
            status_conn = "CONNESSO" if risultati_ranging['connesso'] else "DISCONNESSO"
            print(f" > SNR Diretto: {risultati_ranging['snr_diretto_dB']:.1f} dB [{status_conn}]")
            
            if risultati_ranging['usa_ris']:
                print(f" > [!] SNR Diretto debole, sotto soglia! RIS ID={risultati_ranging['id_ris_scelta']} attivata!")
                print(f" > Nuovo SNR con RIS: {risultati_ranging['snr_effettivo_dB']:.1f} dB")
            else:
                print(" > Nessun pannello RIS attivato (segnale sufficiente o nessuna RIS in vista).")

        print("=" * 60)
      
    except ValueError:
        print("\n[ERRORE] Inserimento non valido. Devi inserire un numero (usa i punti per i decimali, es: 10.5). Riprova.")
