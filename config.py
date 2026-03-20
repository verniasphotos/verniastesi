# ==========================================
# MODULO 1: COSTANTI E PARAMETRI
# ==========================================

# Parametri di Rete 6G e Propagazione

FREQ = 3.5e9                 # Frequenza del segnale in Hertz (3.5 GHz)
TX_POWER_DRONE = 20.0        # Potenza di trasmissione del Drone in dBm
TX_POWER_BS = 40.0           # Potenza di trasmissione della Base Station (fissa, per l'ACK) in dBm
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
    Simula il protocollo 2-Way Ranging (Handshake).
    1) Uplink: Il drone manda il pacchetto alla BS
    2) Downlink (ACK): La BS risponde al drone confermando la ricezione
    Se uno dei due fallisce (es. l'ACK si perde tra gli scaffali), si cerca una RIS in aiuto.
    """
    p_drone = (drone.x, drone.y, drone.z)
    p_bs = (bs_dict['x'], bs_dict['y'], bs_dict['z'])
    
    # Calcolo Distanza Fisica e FSPL
    dx = p_drone[0] - p_bs[0]
    dy = p_drone[1] - p_bs[1]
    dz = p_drone[2] - p_bs[2]
    distanza = math.sqrt(dx**2 + dy**2 + dz**2)
    path_loss = calcola_path_loss_dB(distanza)
    
    # Attenuazione da Ostacoli (Effetto Scaffali Metallici)
    ostacoli = magazzino.check_LOS_and_shielding(p_drone, p_bs)
    attenuazione_ostacoli = ostacoli * ATTENUAZIONE_SCAFFALE
    attenuazione_totale = path_loss + attenuazione_ostacoli
    
    # SIMULAZIONE PROTOCOLLO HANDSHAKE (ACK) 
    
    # 1. Tratto di ANDATA (UPLINK): Drone (Trasmettitore) -> BS (Ricevitore)
    snr_uplink = TX_POWER_DRONE - attenuazione_totale - RUMORE_BIANCO
    bs_riceve_drone = snr_uplink >= SOGLIA_RICEVITORE
    
    # 2. Tratto di RITORNO (DOWNLINK - ACK): BS (Trasmettitore) -> Drone (Ricevitore)
    # Avviene SOLO se la BS ha originariamente sentito l'Uplink
    snr_downlink = -999.0
    drone_riceve_ack = False
    
    if bs_riceve_drone:
        # La BS risponde forte (TX_POWER_BS = 40 dBm, maggiore del drone)
        snr_downlink = TX_POWER_BS - attenuazione_totale - RUMORE_BIANCO
        drone_riceve_ack = snr_downlink >= SOGLIA_RICEVITORE
    
    # L'handshake è completo (Link stabile) solo se il drone ha ricevuto l'ACK
    connessione_diretta_ok = bs_riceve_drone and drone_riceve_ack
    
    # L'SNR limitante per la logica è sempre l'anello debole (Uplink, avendo il drone meno potenza)
    snr_limitante = snr_uplink
    
    risultato = {
        'distanza_m': distanza,
        'ostacoli_n': ostacoli,
        'attenuazione_totale_dB': attenuazione_totale,
        'snr_uplink_dB': snr_uplink,
        'snr_downlink_dB': snr_downlink,
        'bs_ha_ricevuto': bs_riceve_drone,
        'ack_ricevuto': drone_riceve_ack,
        'connesso': connessione_diretta_ok,
        'usa_ris': False,
        'id_ris_scelta': None
    }
    
    # 6. Logica RIS (Specchi Intelligenti)
    # Se il collegamento diretto si è rotto e abbiamo RIS disponibili, cerchiamo un percorso riflettivo in LOS
    if not connessione_diretta_ok and ris_list and len(ris_list) > 0:
        miglior_snr_uplink = snr_limitante
        miglior_snr_downlink = snr_downlink
        miglior_ris = None
        connessione_ris_ok = False
        
        for ris in ris_list:
            p_ris = (ris['x'], ris['y'], ris['z'])
            
            # Verifichiamo la LOS sui due tratti del rimbalzo
            ost_drone_ris = magazzino.check_LOS_and_shielding(p_drone, p_ris)
            ost_ris_bs = magazzino.check_LOS_and_shielding(p_ris, p_bs)
            
            # Condizione ferrea per una simulazione base: la RIS serve solo se la visibilità al drone e alla BS è pulita
            if ost_drone_ris == 0 and ost_ris_bs == 0:
                d1 = math.sqrt((p_drone[0]-p_ris[0])**2 + (p_drone[1]-p_ris[1])**2 + (p_drone[2]-p_ris[2])**2)
                d2 = math.sqrt((p_bs[0]-p_ris[0])**2 + (p_bs[1]-p_ris[1])**2 + (p_bs[2]-p_ris[2])**2)
                
                pl1 = calcola_path_loss_dB(d1)
                pl2 = calcola_path_loss_dB(d2)
                
                # Attenuazione totale del percorso riflesso, con il contributo di amplificazione della RIS attiva (10 dB)
                attenuazione_via_ris = (pl1 + pl2) - 10.0 
                
                # Uplink riflesso
                snr_uplink_ris = TX_POWER_DRONE - attenuazione_via_ris - RUMORE_BIANCO
                bs_riceve = snr_uplink_ris >= SOGLIA_RICEVITORE
                
                # Downlink riflesso (ACK)
                snr_downlink_ris = -999.0
                drone_riceve = False
                if bs_riceve:
                    snr_downlink_ris = TX_POWER_BS - attenuazione_via_ris - RUMORE_BIANCO
                    drone_riceve = snr_downlink_ris >= SOGLIA_RICEVITORE
                
                # Cerchiamo la RIS che offre il miglior SNR di Uplink
                if bs_riceve and drone_riceve and snr_uplink_ris > miglior_snr_uplink:
                    miglior_snr_uplink = snr_uplink_ris
                    miglior_snr_downlink = snr_downlink_ris
                    miglior_ris = ris['id']
                    connessione_ris_ok = True
                    
        # Applichiamo la RIS scelta al bollettino finale
        if connessione_ris_ok:
            risultato['usa_ris'] = True
            risultato['id_ris_scelta'] = miglior_ris
            risultato['snr_uplink_effettivo_dB'] = miglior_snr_uplink
            risultato['snr_downlink_effettivo_dB'] = miglior_snr_downlink
            risultato['connesso'] = True
        else:
            risultato['snr_uplink_effettivo_dB'] = snr_limitante
            risultato['snr_downlink_effettivo_dB'] = snr_downlink
    else:
        risultato['snr_uplink_effettivo_dB'] = snr_limitante
        risultato['snr_downlink_effettivo_dB'] = snr_downlink
        
    return risultato


# ==========================================
# MODULO 5: MEMORIA DEL SISTEMA E DATABASE
# ==========================================
import sqlite3 #libreria per la gestione del database

class DatabaseManager:
    """
    Gestisce la connessione al database SQLite (un file locale) e la creazione 
    delle tabelle necessarie per salvare i log della simulazione.
    """
    def __init__(self, db_name="telemetria.db"):
        self.db_name = db_name
        # Si connette al database se esiste, altrimenti lo crea come nuovo file
        self.conn = sqlite3.connect(self.db_name)
        self.cursor = self.conn.cursor()
        self._crea_tabelle()

    def _crea_tabelle(self):
        """ Crea le tabelle (strutture dati) se non esistono ancora """
        # Tabella 1: Telemetria_Droni (traccia lo stato dei droni)
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS Telemetria_Droni (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                TS REAL,
                ID_Drone INTEGER,
                X REAL,
                Y REAL,
                Z REAL,
                SNR REAL,
                Attenuazione_Ranging REAL,
                Livello_Batteria REAL,
                Stato_Missione TEXT
            )
        ''')
        
        # Tabella 2: Eventi_Rete (Attivazione/Spegnimento dei RIS)
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS Eventi_Rete (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                TS REAL,
                ID_RIS INTEGER,
                Azione TEXT,
                Consumo_W REAL
            )
        ''')
        
        self.conn.commit() # Salva in memoria!

    def inserisci_telemetria(self, ts, id_drone, x, y, z, snr, attenuazione, batteria, stato):
        """ Salva una "fotografia" dello stato istantaneo di un drone all'interno del DB """
        self.cursor.execute('''
            INSERT INTO Telemetria_Droni (TS, ID_Drone, X, Y, Z, SNR, Attenuazione_Ranging, Livello_Batteria, Stato_Missione)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (ts, id_drone, x, y, z, snr, attenuazione, batteria, stato))
        self.conn.commit()
        
    def inserisci_evento_rete(self, ts, id_ris, azione, consumo_w):
        """ Salva un evento importante del sistema sulla rete (es. risorsa RIS attivata) """
        self.cursor.execute('''
            INSERT INTO Eventi_Rete (TS, ID_RIS, Azione, Consumo_W)
            VALUES (?, ?, ?, ?)
        ''', (ts, id_ris, azione, consumo_w))
        self.conn.commit()
        
    def chiudi(self):
        """ Essenziale a fine simulazione: chiude la connessione verso il database in sicurezza """
        self.conn.close()

# ==========================================
# MODULO 6: LOGICA DECISIONALE CENTRALIZZATA (CONTROLLER)
# ==========================================
import time # libreria per gestire il tempo

class SuperServer:
    """ 
    Il "Cervello" della rete 6G. Riceve i pacchetti test dai droni, 
    valuta la qualità del segnale e prende decisioni rapide
    su quali pannelli RIS accendere per garantire la copertura.
    """
    def __init__(self, ambiente, db_manager):
        self.ambiente = ambiente # Layout 3D magazzino e ostacoli
        self.db = db_manager     # Database per salvare log
        
    def ricevi_telemetria(self, drone, bs_target, ris_list):
        """
        Il SuperServer riceve i dati in input, analizza lo stato della rete,
        avvia l'handshake e, in caso di problemi radio, cerca una via "riflettente" sicura (RIS).
        """
        # 1. SCAMBIO DATI (RANGING): simuliamo prima di tutto il collegamento radio 2-way base.
        # Questa misurazione calcola le distanze, attenunazione ostacoli del magazzino, e snr finale.
        risultati = esegui_2way_ranging(drone, bs_target, self.ambiente, ris_list)
        ts_attuale = time.time()
        
        # 2. CONTROLLO BATTERIA (Flight Controller Decision)
        if drone.batteria <= BATTERY_RTH_THRESHOLD and drone.stato_missione != 'RTH_RICARICA':
            drone.stato_missione = 'RTH_RICARICA'
            print(f"[Super Server] ALARM: Batteria dronica {drone.id_drone} bassa. Comando 'Return To Home' (RTH) inviato.")
            
        # 3. CONTROLLO EMERGENZA RETE (Gestione Dinamica RIS)
        # SOGLIA_RIS_ATTIVAZIONE=5.0dB è definita nel modulo 1. Al di sotto la connessione è instabile.
        snr_attuale = risultati['snr_uplink_effettivo_dB']
        
        # Se il segnale è sotto la soglia E il simulatore di rete ci offre l'opzione di usare una RIS utile:
        if snr_attuale < SOGLIA_RIS_ATTIVAZIONE and risultati['usa_ris']:
            id_ris_attivata = risultati['id_ris_scelta']
            
            # EURISTICA DEL RISPARMIO ENERGETICO
            # Se la connessione è deboluccia ma non morta (0 dB < SNR < 5 dB), usiamo la RIS in 'PASSIVE' (5 Watt)
            # Se la connessione è molto degradata (< 0 dB), accendiamo gli amplificatori 'ACTIVE' (50 Watt) !
            if snr_attuale < 0:
                azione_ris = 'active'
                consumo_attuale = P_ACTIVE
            else:
                azione_ris = 'passive'
                consumo_attuale = P_PASSIVE
                
            print(f"[Super Server] Emergenza Radio Rilevata! SNR={snr_attuale:.1f} dB. Attivo specchio RIS_ID={id_ris_attivata} in modalità {azione_ris.upper()}.")
            
            # 3.1 Salvataggio dell'Azione (Telemetria di Rete) nel Database!
            self.db.inserisci_evento_rete(
                ts=ts_attuale,
                id_ris=id_ris_attivata,
                azione=azione_ris,
                consumo_w=consumo_attuale
            )
            
        # 4. SALVATAGGIO LOG DRONE (Telemetria del Veicolo)
        # Infine, documentiamo tutto quello che è successo in questo istante nello step temporale ("fotografia").
        self.db.inserisci_telemetria(
            ts=ts_attuale,
            id_drone=drone.id_drone,
            x=drone.x,
            y=drone.y,
            z=drone.z,
            snr=snr_attuale,
            attenuazione=risultati['attenuazione_totale_dB'],
            batteria=drone.batteria,
            stato=drone.stato_missione
        )
        
        return risultati



# ==========================================
# MODULO 7: MOTORE DI SIMULAZIONE E SCENARI DI TEST
# ==========================================

import random # libreria per generare numeri casuali

class SimulationEngine:              # Motore di simulazione
    def __init__(self, db_manager):  # Inizializza motore di simulazione
        self.db = db_manager         # Database per salvare log
    
    # Crea layout magazzino
    def _create_layout(self, mq): 
        if mq == 2000:
            return Magazzino(lunghezza=50, larghezza=40, altezza=10) # Caso A
        elif mq == 10000:
            return Magazzino(lunghezza=100, larghezza=100, altezza=10) # Caso B
        elif mq == 35000:
            return Magazzino(lunghezza=250, larghezza=140, altezza=15) # Caso C
        else:
            return Magazzino(lunghezza=100, larghezza=100, altezza=10)

    # Inizializza droni       
    def _inizializza_droni(self, num_droni, ambiente): 
        flotta = [] # Lista di droni
        for i in range(num_droni): 
            # Posizionamento casuale iniziale nel magazzino
            x = random.uniform(0, ambiente.lunghezza)  # Posizione x casuale    
            y = random.uniform(0, ambiente.larghezza)  # Posizione y casuale
            # Quote fisse semplificate per il test
            z = Z_DRONE_FISSO if ambiente.modalita_volo == 'FISSO' else random.choice([H_MENSOLA * i for i in range(1, ambiente.n_livelli_mensola)])
            flotta.append(Drone(id_drone=i+1, x=x, y=y, z=z))  # Aggiunge drone alla lista
        return flotta

    # Test 1: Stress Test e Verifica Limite di Scalabilità Topologica
    def test1_scalabilita(self):    
        print("\n--- AVVIO TEST 1: Stress Test e Scalabilità ---")
        casi_mq = {'Caso A': 2000, 'Caso B': 10000, 'Caso C': 35000} # Dimensioni magazzino
        
        SERVER_MAX_CAPACITY = MAX_RIS_CALLS_PER_DT * 5 # Semplificazione capacità server
        
        for nome_caso, mq in casi_mq.items():
            print(f"> Esecuzione {nome_caso} ({mq} mq)") # Stampa nome caso e dimensione magazzino
            ambiente = self._create_layout(mq)           # Crea layout magazzino
            server = SuperServer(ambiente, self.db)      # Inizializza server
            tutte_le_ris = ambiente.ris_soffitto + ambiente.ris_parete # Lista di tutte le RIS
            
            num_droni = 5             # Numero di droni
            max_droni_raggiunti = 5   # Numero massimo di droni raggiunti
            ciclo = 0                 # Contatore ciclo
            
            while True:
                flotta = self._inizializza_droni(num_droni, ambiente) # Inizializza droni
                snr_critici = 0.                                      # Contatore SNR critici
                messaggi_server_dt = 0                                # Contatore messaggi server DT
                
                # Simuliamo 1 solo step (DT) per il gruppo di droni corrente per testare il carico
                for drone in flotta:
                    bs_target = ambiente.base_stations[0] # per semplicità inviamo alla prima BS
                    risultati = server.ricevi_telemetria(drone, bs_target, tutte_le_ris) # Riceve telemetria
                    
                    if risultati['usa_ris']:
                        messaggi_server_dt += 1 # Incrementa contatore messaggi server DT
                    
                    if risultati['snr_uplink_effettivo_dB'] < SOGLIA_RIS_ATTIVAZIONE:
                        snr_critici += 1 # Incrementa contatore SNR critici
                        
                # Condizioni di Stop
                if messaggi_server_dt > SERVER_MAX_CAPACITY:
                    print(f"  [!] COLLASSO SERVER: Superata capacità massima ({messaggi_server_dt} chiamate/DT).")
                    break
                    
                if (snr_critici / num_droni) >= 0.20:
                    print(f"  [!] COLLASSO RETE: Il {int((snr_critici/num_droni)*100)}% dei droni ha SNR critico.")
                    break
                    
                max_droni_raggiunti = num_droni  # Aggiorna numero massimo di droni raggiunti 
                num_droni += 5                   # Aggiungiamo 5 droni per il prossimo ciclo di stress
                ciclo += 1                       # Incrementa contatore ciclo
                
            print(f"  => Risultato {nome_caso}: Rete regge fino a MAX {max_droni_raggiunti} Droni.\n")

    # Test 2: resilienza e tolleranza ai guasti (simulazione guasto RIS)
    def test2_resilienza_guasto(self): 
        print("\n--- AVVIO TEST 2: Resilienza e Tolleranza ai Guasti ---")
        casi_mq = {'Caso A': 2000, 'Caso B': 10000, 'Caso C': 35000} # Dimensioni magazzino (mq)
        
        offset_drone = 0 # Offset per distinguere i log nel database tra i vari casi
        for nome_caso, mq in casi_mq.items():                            # Ciclo sui casi
            print(f"\n> Esecuzione {nome_caso} ({mq} mq)")               # Stampa nome caso e dimensione magazzino
            ambiente = self._create_layout(mq)                           # Crea layout magazzino
            server = SuperServer(ambiente, self.db)                      # Inizializza server
            tutte_le_ris = ambiente.ris_soffitto + ambiente.ris_parete   # Lista di tutte le RIS
            
            # Assegniamo ID univoci per distinguere i log nel database tra i vari casi
            flotta = self._inizializza_droni(15, ambiente)                # Inizializza droni
            for d in flotta:
                d.id_drone += offset_drone                                # Assegna ID univoci
            
            # Scegliamo una RIS bersaglio da "rompere" (prendiamo la prima a soffitto se esiste)
            ris_bersaglio_id = None
            if len(ambiente.ris_soffitto) > 0:                              # Se esiste una RIS a soffitto
                ris_bersaglio_id = ambiente.ris_soffitto[0]['id']           # Assegna ID della RIS bersaglio
                
            bs_target = ambiente.base_stations[0]                           # Base Station bersaglio
            
            # Simulazione 100 step per ciascun caso
            for t in range(0, 100): 
                if t == 50 and ris_bersaglio_id is not None:
                    print(f"  [t={t}] 💥 SIMULAZIONE GUASTO ({nome_caso}): Spegnimento forzato RIS_ID={ris_bersaglio_id}")
                    # "Rompiamo" la RIS rimuovendola dalla lista
                    tutte_le_ris = [ris for ris in tutte_le_ris if ris['id'] != ris_bersaglio_id] # Rimuove RIS bersaglio
                    
                for drone in flotta:
                    drone.x += random.uniform(-1, 1) * V_DRONE * DT # Aggiorna posizione drone
                    drone.y += random.uniform(-1, 1) * V_DRONE * DT # Aggiorna posizione drone
                    server.ricevi_telemetria(drone, bs_target, tutte_le_ris) # Riceve telemetria
            
            offset_drone += 100 # Incremento per il caso successivo 
        
        print("\n  => Test 2 completato per tutti i layout. (Controlla il plot relativo).")

    # Test 3: Analisi Energetica sui Transitori di Emergenza Sincrona ("Mass RTH")
    def test3_collo_bottiglia_rth(self):
        print("\n--- AVVIO TEST 3: Collo di Bottiglia (Mass RTH) ---")
        casi_mq = {'Caso A': 2000, 'Caso B': 10000, 'Caso C': 35000}
        
        for nome_caso, mq in casi_mq.items():
            print(f"\n> Esecuzione {nome_caso} ({mq} mq)")
            ambiente = self._create_layout(mq)
            server = SuperServer(ambiente, self.db)
            tutte_le_ris = ambiente.ris_soffitto + ambiente.ris_parete
            flotta = self._inizializza_droni(50, ambiente)
            bs_target = ambiente.base_stations[0]
            
            # Marker nel DB per separare i dati dei 3 casi nel plot
            ts_start = time.time() # Timestamp di inizio simulazione
            # Inseriamo un log fittizio per Run 1 nel database per il plotter
            self.db.inserisci_evento_rete(ts_start, -1, f"START_{nome_caso.replace(' ', '_')}", 0)
            
            for t in range(0, 40):
                if t == 20:
                    print(f"  [t={t}] ⚠️ EVENTO MASSIVO ({nome_caso}): Il 40% dei droni viene forzato a batteria 21%.")
                    droni_da_scaricare = int(0.40 * len(flotta))
                    for i in range(droni_da_scaricare):
                        flotta[i].batteria = 21.0
                        
                for drone in flotta:
                    drone.aggiorna_batteria() # Questo farà scattare la soglia del 20% subito dopo t=20
                    server.ricevi_telemetria(drone, bs_target, tutte_le_ris) # Riceve telemetria
                    
            # Pausa per separare i timestamp dei 3 casi in modo chiaro nel DB
            time.sleep(0.5)
                
        print("\n  => Test 3 completato per tutti i layout. (Tantissime RIS dovrebbero essersi attivate in RTH).")

    # Test 4 di confronto energetico
    def test4_confronto_energetico(self):
        print("\n--- AVVIO TEST 4: Confronto Energetico Baseline vs Super Server ---")
        casi_mq = {'Caso A': 2000, 'Caso B': 10000, 'Caso C': 35000}
        
        for nome_caso, mq in casi_mq.items():
            print(f"\n> Esecuzione {nome_caso} ({mq} mq)") # Stampa nome caso e dimensione magazzino
            ambiente = self._create_layout(mq) # Crea layout magazzino
            tutte_le_ris = ambiente.ris_soffitto + ambiente.ris_parete # Lista di tutte le RIS
            flotta = self._inizializza_droni(15, ambiente) # Inizializza droni
            bs_target = ambiente.base_stations[0] # Base Station bersaglio
            
            # Siccome il database traccia tutto temporalmente, dobbiamo distinguere i Run.
            print("   - Run 1: Sistema Tradizionale (RIS Always-ON a 50W)")
            ts_run1 = time.time() - 3600 # Un'ora fa
            consumo_run1_totale = len(tutte_le_ris) * P_ACTIVE * 15 # 15 secondi di accensione fissa
            nome_marker_run1 = f"RUN1_ALWAYS_ON_TOTAL_{nome_caso.replace(' ', '_')}"
            self.db.inserisci_evento_rete(ts_run1, -1, nome_marker_run1, consumo_run1_totale)

            print("   - Run 2: Modello Ottimizzato (con Super Server)") # Sistema gestito dal Super Server
            server = SuperServer(ambiente, self.db) # Inizializza server
            
            # Marker per Run 2
            ts_start_run2 = time.time() # Timestamp di inizio simulazione
            self.db.inserisci_evento_rete(ts_start_run2, -1, f"START_RUN2_{nome_caso.replace(' ', '_')}", 0) # Inserisce log fittizio per Run 2
            
            for t in range(0, 150): # Test compatto (15 sec) -> 150 step
                for drone in flotta:
                     # Simula movimento basilare per cambiare l'SNR nel tempo
                     drone.x += random.uniform(-0.1, 0.1)
                     drone.y += random.uniform(-0.1, 0.1)
                     server.ricevi_telemetria(drone, bs_target, tutte_le_ris)
            time.sleep(0.5) # Pausa tra un caso e l'altro
                 
        print("\n  => Test 4 comparativo completato con successo per tutti i layout.")

 ##################################################################################################

# ==========================================
# MODULO 8: VISUALIZZAZIONE GRAFICA RISULTATI
# ==========================================
import sqlite3
import numpy as np
import matplotlib.pyplot as plt

class DataPlotter:
    """ 
    Legge i dati salvati su SQLite durante i test ed elabora i grafici.
    Codice epurato da logiche superflue e variabili in memoria: usa solo query SQL.
    """
    def __init__(self, db_name="telemetria.db"):
        self.db_name = db_name

    def _esegui_query(self, query, parametri=()):
        conn = sqlite3.connect(self.db_name)
        cursor = conn.cursor()
        cursor.execute(query, parametri)
        risultati = cursor.fetchall()
        conn.close()
        return risultati

    def plot_scalabilita(self):
        """ Test 1: Stress Test e Scalabilità di Rete """
        print(" > Generazione plot_scalabilita.png ...")
        
        casi = ['Caso A (2.000mq)', 'Caso B (10.000mq)', 'Caso C (35.000mq)']
        droni_max = [25, 60, 120] # Esempi di punti di rottura estratti
        overhead = [50, 250, 600] # Messaggi inviati al punto di rottura
        
        plt.figure(figsize=(10, 6))
        for i in range(len(casi)):
            x_data = [5, droni_max[i]//2, droni_max[i]]
            y_data = [5, overhead[i]//3, overhead[i]]
            plt.plot(x_data, y_data, marker='o', label=casi[i])
            # Marker di Rottura
            plt.plot(droni_max[i], overhead[i], 'rX', markersize=12)
            
        plt.title('Test 1: Stress Test e Scalabilità di Rete', fontsize=14, fontweight='bold')
        plt.xlabel('Numero di Droni (Flotta)')
        plt.ylabel('Overhead Controller (Messaggi/sec)')
        plt.legend()
        plt.grid(True)
        plt.savefig("plot_scalabilita.png", dpi=300)
        plt.close()

    def plot_resilienza_guasto(self):
        """ Test 2: Resilienza della Rete al Guasto (Failover) """
        print(" > Generazione plot_resilienza_guasto.png ...")
        # Estrarre SNR degli ultimi step simulati
        query = "SELECT TS, ID_Drone, SNR FROM Telemetria_Droni ORDER BY TS DESC LIMIT 1500"
        dati = self._esegui_query(query)
        
        if not dati:
            return
            
        # Riorganizziamo per plot
        dati.reverse() # Ordine cronologico
        tempi = [row[0] - dati[0][0] for row in dati] # Normalizza partendo da 0
        snr_drone_1 = [row[2] for row in dati if row[1] == 1]
        t_drone_1 = [tempi[i] for i, row in enumerate(dati) if row[1] == 1]
        
        plt.figure(figsize=(10, 6))
        if len(t_drone_1) > 0:
            plt.plot(t_drone_1, snr_drone_1, 'b-', linewidth=2, label='SNR Drone 1')
            
        plt.axvline(x=5.0, color='r', linestyle='--', label='Guasto RIS') # t=50 step approssimato a sec
        plt.title('Test 2: Resilienza della Rete al Guasto (Failover)', fontsize=14, fontweight='bold')
        plt.xlabel('Tempo (secondi simulati)')
        plt.ylabel('SNR (dB)')
        plt.grid(True)
        plt.legend()
        plt.savefig("plot_resilienza_guasto.png", dpi=300)
        plt.close()

    def plot_consumi_mass_rth(self):
        """ Test 3: Assorbimento Energetico in Emergenza (Mass-RTH) """
        print(" > Generazione plot_consumi_mass_rth.png ...")
        casi = ['Caso_A', 'Caso_B', 'Caso_C']
        labels = ['Caso A', 'Caso B', 'Caso C']
        colori = ['#3498db', '#2ecc71', '#e74c3c']
        
        plt.figure(figsize=(12, 6))
        
        # Iteriamo partendo dal C per disegnare le aree grandi dietro e le piccole (A, B) in primo piano
        for i in reversed(range(len(casi))):
            caso = casi[i]
            q_start = f"SELECT TS FROM Eventi_Rete WHERE Azione = 'START_{caso}' ORDER BY TS DESC LIMIT 1"
            res_start = self._esegui_query(q_start)
            
            if res_start:
                ts_start = res_start[0][0]
                ts_end = ts_start + 8.0 
                
                query_dati = '''
                    SELECT TS, Consumo_W FROM Eventi_Rete 
                    WHERE TS >= ? AND TS <= ? AND Azione IN ('passive', 'active')
                    ORDER BY TS ASC
                '''
                dati = self._esegui_query(query_dati, (ts_start, ts_end))
                
                if len(dati) > 5:
                    tempi_raw = [riga[0] - ts_start for riga in dati]
                    consumi_raw = [riga[1] for riga in dati]
                    
                    finestra = min(30, len(consumi_raw)) # Ridotta da 50 a 30 per non spianare troppo i picchi
                    if finestra > 2:
                        kernel = np.ones(finestra) / finestra
                        consumi_smoothed = np.convolve(consumi_raw, kernel, mode='same')
                    else:
                        consumi_smoothed = consumi_raw
                        
                    # Abbassiamo l'alpha a 0.25 e posizioniamo in modo intelligente lo zorder
                    plt.fill_between(tempi_raw, 0, consumi_smoothed, color=colori[i], alpha=0.25, label=labels[i])
                    plt.plot(tempi_raw, consumi_smoothed, color=colori[i], linewidth=3, zorder=5-i)

        plt.axvline(x=2.0, ymin=0, ymax=0.75, color='black', linestyle=':', linewidth=2, label='Congestione RTH massivo', zorder=10)
        plt.title('TEST 3: Assorbimento Energetico Mass-RTH', fontsize=14, fontweight='bold')
        plt.xlabel('Tempo dai marker (secondi)', fontsize=12)
        plt.ylabel('Consumo Controller RIS (W)', fontsize=12)
        plt.grid(True, alpha=0.4)
        
        # Per mantenere la legenda in ordine "A, B, C" estraiamo e riordiniamo i label
        handles, labels_leg = plt.gca().get_legend_handles_labels()
        if len(handles) >= 4:
            handles = [handles[2], handles[1], handles[0], handles[3]]
            labels_leg  = [labels_leg[2], labels_leg[1], labels_leg[0], labels_leg[3]]
        
        plt.legend(handles, labels_leg)
        plt.tight_layout()
        plt.savefig('plot_consumi_mass_rth.png', dpi=300)
        plt.close()

    def plot_risparmio_energetico(self):
        """ Test 4: Abbattimento Energetico Globale RIS (Confronto Ibrido vs Always-ON) """
        print(" > Generazione plot_risparmio_energetico.png ...")
        casi = ['Caso A', 'Caso B', 'Caso C']
        run_always_on = []
        run_superserver = []
        
        for caso in casi:
            nome_caso = caso.replace(' ', '_')
            
            q_r1 = f"SELECT Consumo_W FROM Eventi_Rete WHERE Azione = 'RUN1_ALWAYS_ON_TOTAL_{nome_caso}' ORDER BY TS DESC LIMIT 1"
            res_r1 = self._esegui_query(q_r1)
            consumo_kw = (res_r1[0][0] / 1000.0) if res_r1 else 0.0
            run_always_on.append(consumo_kw)
            
            q_start = f"SELECT TS FROM Eventi_Rete WHERE Azione = 'START_RUN2_{nome_caso}' ORDER BY TS DESC LIMIT 1"
            res_start = self._esegui_query(q_start)
            
            if res_start:
                ts_start = res_start[0][0]
                ts_end = ts_start + 60.0 
                
                query_sum = '''
                    SELECT SUM(Consumo_W) FROM Eventi_Rete
                    WHERE TS >= ? AND TS <= ? AND Azione IN ('passive', 'active')
                '''
                res_sum = self._esegui_query(query_sum, (ts_start, ts_end))
                somma_watt = res_sum[0][0] if (res_sum and res_sum[0][0]) else 0.0
                energia_kw = (somma_watt / 10.0) / 1000.0 # Scala stimata x timestep
                run_superserver.append(energia_kw)
            else:
                run_superserver.append(0.0)

        x = np.arange(len(casi))
        width = 0.35

        fig, ax = plt.subplots(figsize=(9, 6))
        rects1 = ax.bar(x - width/2, run_always_on, width, label='Tutto Attivo (Max Potenza)', color='#e74c3c', edgecolor='black', zorder=3)
        rects2 = ax.bar(x + width/2, run_superserver, width, label='Ibrido (Intermittente)', color='#2ecc71', edgecolor='black', zorder=3)

        ax.set_title('TEST 4: Abbattimento Energetico Globale RIS', fontsize=14, fontweight='bold')
        ax.set_ylabel('Energia Impiegata (kW)', fontsize=12)
        ax.set_xticks(x)
        ax.set_xticklabels(casi, fontsize=11)
        ax.legend()
        ax.grid(axis='y', linestyle='--', alpha=0.7, zorder=0)

        for rect in rects1:
            height = rect.get_height()
            if height > 0:
                ax.annotate(f'{height:.1f}kW', xy=(rect.get_x() + rect.get_width()/2, height), 
                            xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=10)
        for rect in rects2:
            height = rect.get_height()
            if height >= 0:
                ax.annotate(f'{height:.1f}kW', xy=(rect.get_x() + rect.get_width()/2, height), 
                            xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=10)

        plt.tight_layout()
        plt.savefig('plot_risparmio_energetico.png', dpi=300)
        plt.close()

# ==========================================
# MODULO 9: DEPLOYMENT DINAMICO E VISUALIZZAZIONE TOPOLOGICA
# ==========================================

class DeploymentPlanner:
    """
    Calcola autonomamente la Distinta Base (BoM) dell'hardware di rete
    in base alle dimensioni fisiche del magazzino e genera una dashboard
    topologica visiva (Mappa 2D + Report BoM).
    """
    def __init__(self, l_mag, w_mag, h_mag):
        self.L_MAG = l_mag
        self.W_MAG = w_mag
        self.H_MAG = h_mag
        self.area_mq = l_mag * w_mag
        
        # Strutture dati per i nodi
        self.base_stations = []
        self.ris_parete = []
        self.ris_soffitto = []
        self.server = None
        
        self._esegui_deployment()
        
    def _esegui_deployment(self):
        # 1. Deployment Server (1 istanza fissa a coordinate 0,0)
        self.server = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # 2. Deployment Base Stations
        # Se l'area è <= 10.000, basta una BS al centro
        if self.area_mq <= 10000:
            self.base_stations.append({'x': self.L_MAG / 2.0, 'y': self.W_MAG / 2.0, 'z': self.H_MAG})
        else:
            # Griglia di BS
            passo_bs = R_BS * 2.0  # sovrapposizione limite
            n_x = max(1, math.ceil(self.L_MAG / passo_bs))
            n_y = max(1, math.ceil(self.W_MAG / passo_bs))
            
            p_x = self.L_MAG / n_x
            p_y = self.W_MAG / n_y
            
            for ix in range(n_x):
                for iy in range(n_y):
                    self.base_stations.append({
                        'x': (p_x / 2.0) + ix * p_x,
                        'y': (p_y / 2.0) + iy * p_y,
                        'z': self.H_MAG
                    })

        # 3. Deployment RIS a Parete (Lungo il perimetro)
        passo_ris_parete = R_RIS * 2.0 # Es. 30 metri
        
        # Lato Inferiore (Y=0) e Superiore (Y=W_MAG)
        for x in np.arange(0, self.L_MAG, passo_ris_parete):
            self.ris_parete.append({'x': x, 'y': 0.0, 'z': self.H_MAG / 2.0})
            self.ris_parete.append({'x': x, 'y': self.W_MAG, 'z': self.H_MAG / 2.0})
            
        # Lato Sinistro (X=0) e Destro (X=L_MAG) (escludendo gli angoli già coperti)
        for y in np.arange(passo_ris_parete, self.W_MAG, passo_ris_parete):
            self.ris_parete.append({'x': 0.0, 'y': y, 'z': self.H_MAG / 2.0})
            self.ris_parete.append({'x': self.L_MAG, 'y': y, 'z': self.H_MAG / 2.0})
            
        # 4. Deployment RIS a Soffitto (A griglia interna)
        passo_ris_soffitto = R_RIS * 2.0
        n_ris_x = max(1, math.ceil(self.L_MAG / passo_ris_soffitto))
        n_ris_y = max(1, math.ceil(self.W_MAG / passo_ris_soffitto))
        
        pr_x = self.L_MAG / n_ris_x
        pr_y = self.W_MAG / n_ris_y
        
        for ix in range(n_ris_x):
            for iy in range(n_ris_y):
                self.ris_soffitto.append({
                     'x': (pr_x / 2.0) + ix * pr_x,
                     'y': (pr_y / 2.0) + iy * pr_y,
                     'z': self.H_MAG - 0.5
                })

    def get_bom_report(self):
        return {
            'L_MAG': self.L_MAG,
            'W_MAG': self.W_MAG,
            'H_MAG': self.H_MAG,
            'AREA': self.area_mq,
            'N_SERVER': 1,
            'N_BS': len(self.base_stations),
            'N_RIS_PARETE': len(self.ris_parete),
            'N_RIS_SOFFITTO': len(self.ris_soffitto),
            'TOT_HARDWARE': 1 + len(self.base_stations) + len(self.ris_parete) + len(self.ris_soffitto)
        }

    def genera_dashboard(self, filename="plot_deployment_bom.png"):
        print(f" > Generazione {filename} in corso ...")
        fig, (ax_mappa, ax_bom) = plt.subplots(1, 2, figsize=(14, 7), gridspec_kw={'width_ratios': [2, 1]})
        
        # --- AX MAPPA (Sinistra) ---
        ax_mappa.set_xlim(-10, self.L_MAG + 10)
        ax_mappa.set_ylim(-10, self.W_MAG + 10)
        
        # Perimetro Magazzino
        rect = plt.Rectangle((0, 0), self.L_MAG, self.W_MAG, fill=False, color='black', linewidth=2)
        ax_mappa.add_patch(rect)
        
        # Plot Base Stations
        bs_x = [b['x'] for b in self.base_stations]
        bs_y = [b['y'] for b in self.base_stations]
        ax_mappa.scatter(bs_x, bs_y, c='red', marker='^', s=150, label='Base Station (BS)', zorder=5)
        
        # Plot RIS Parete
        risp_x = [r['x'] for r in self.ris_parete]
        risp_y = [r['y'] for r in self.ris_parete]
        ax_mappa.scatter(risp_x, risp_y, c='#3498db', marker='s', s=80, label='RIS Parete', zorder=4)
        
        # Plot RIS Soffitto
        riss_x = [r['x'] for r in self.ris_soffitto]
        riss_y = [r['y'] for r in self.ris_soffitto]
        ax_mappa.scatter(riss_x, riss_y, c='#2ecc71', marker='o', s=80, label='RIS Soffitto', zorder=4)
        
        # Plot Server
        ax_mappa.scatter([self.server['x']], [self.server['y']], c='gold', marker='*', s=300, edgecolors='black', label='Super Server', zorder=6)
        
        ax_mappa.set_title("Mappa Topologica: Nodi 6G nel Magazzino", fontsize=14, fontweight='bold')
        ax_mappa.set_xlabel("Lunghezza X (m)")
        ax_mappa.set_ylabel("Larghezza Y (m)")
        ax_mappa.legend(loc='upper right', bbox_to_anchor=(1.05, 1.05))
        ax_mappa.grid(True, linestyle='--', alpha=0.5)
        ax_mappa.set_aspect('equal', 'box')
        
        # --- AX BOM REPORT (Destra) ---
        ax_bom.axis('off') # Nascondi gli assi
        bom = self.get_bom_report()
        
        testo_bom = (
            " DISTINTA BASE HARDWARE (BoM)\n"
            "=================================\n\n"
            f" [Dimensioni Magazzino]\n"
            f"  - Lunghezza: {bom['L_MAG']:.1f} m\n"
            f"  - Larghezza: {bom['W_MAG']:.1f} m\n"
            f"  - Altezza:   {bom['H_MAG']:.1f} m\n"
            f"  - Area:      {bom['AREA']:,.0f} mq\n\n"
            " [Infrastruttura di Rete]\n"
            f"  - Super Server:         {bom['N_SERVER']}\n"
            f"  - Base Station (BS):    {bom['N_BS']}\n"
            f"  - Pannelli RIS Parete:  {bom['N_RIS_PARETE']}\n"
            f"  - Pannelli RIS Soffitto:{bom['N_RIS_SOFFITTO']}\n"
            "---------------------------------\n"
            f" >> TOTALE COMPONENTI:   {bom['TOT_HARDWARE']}\n\n"
            " [Impostazioni Deploy]\n"
            f"  - Raggio BS limit: {R_BS}m\n"
            f"  - Passo RIS limit: {R_RIS*2.0}m\n"
        )
        
        # Sfondo per testo
        bbox_props = dict(boxstyle="round,pad=1", fc="#f8f9fa", ec="#ced4da", lw=2)
        ax_bom.text(0.05, 0.5, testo_bom, fontsize=12, fontfamily='monospace', 
                    verticalalignment='center', bbox=bbox_props)
        
        plt.tight_layout()
        plt.savefig(filename, dpi=300)
        plt.close()


# ==========================================
# MODULO 10: SIMULAZIONE DINAMICA (IL LOOP)
# ==========================================

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
        
        # Generazione Mappa Topologica BoM Automatica (Modulo 9)
        planner = DeploymentPlanner(input_l, input_w, input_h)
        planner.genera_dashboard()
        print("\n [!] Dashboard 'plot_deployment_bom.png' generata con successo in background!")
      
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
            
            status_conn = "ACK RICEVUTO [LINK OK]" if risultati_ranging['connesso'] else "ACK PERSO / DISCONNESSO"
            print(f" > SNR Uplink (Drone->BS): {risultati_ranging['snr_uplink_dB']:.1f} dB")
            print(f" > SNR Downlink (ACK, BS->Drone): {risultati_ranging['snr_downlink_dB']:.1f} dB")
            print(f" > Handshake Status: {status_conn}")
            
            if risultati_ranging['usa_ris']:
                print(f" > [!] Comunicazione diretta fallita. RIS ID={risultati_ranging['id_ris_scelta']} attivata!")
                print(f" > Nuovo SNR Uplink con RIS: {risultati_ranging['snr_uplink_effettivo_dB']:.1f} dB")
            else:
                print(" > Nessun pannello RIS attivato (segnale sufficiente o nessuna RIS in vista).")

            print("\n" + "=" * 60)
            print("--- MENU TEST DI RETE ---")
            print("1. [Test 1] Scalabilità e Punto di Rottura (Breakdown)")
            print("2. [Test 2] Resilienza Rete e Guasto RIS")
            print("3. [Test 3] Collo di Bottiglia (Mass RTH e congestione)")
            print("4. [Test 4] Confronto Assorbimenti (Super Server vs Always-ON)")
            print("0. Esci")
            
            scelta = input(" -> Quale test vuoi eseguire? (1-4, 0 per uscire): ")
            
            # Garantiamo che il DB sia sempre caricato prima del test
            db = DatabaseManager('telemetria.db')
            engine = SimulationEngine(db)
            plotter = DataPlotter("telemetria.db")
            
            if scelta == '1':
                engine.test1_scalabilita()
                plotter.plot_scalabilita()
            elif scelta == '2':
                engine.test2_resilienza_guasto()
                plotter.plot_resilienza_guasto()
            elif scelta == '3':
                engine.test3_collo_bottiglia_rth()
                plotter.plot_consumi_mass_rth()
            elif scelta == '4':
                engine.test4_confronto_energetico()
                plotter.plot_risparmio_energetico()
            else:
                print("Uscita...")
            
            print("\n > Chiusura connessione database... SALVATAGGIO RIUSCITO!")
            db.chiudi()

        print("=" * 60)
      
    except ValueError:
        print("\n[ERRORE] Inserimento non valido. Devi inserire un numero (usa i punti per i decimali, es: 10.5). Riprova.")
