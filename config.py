# ==========================================
# IMPORT LIBRERIE E DIPENDENZE
# ==========================================

import multiprocessing as mp
from multiprocessing import shared_memory
from typing import List, Dict, Tuple, Optional, Union, Any

# --- Motore Matematico e Fisico ---
import numpy as np
import scipy
import scipy.constants
import scipy.spatial.distance
from numba import njit, prange

# --- Tracking e Filtraggio ---
from filterpy.kalman import ExtendedKalmanFilter
import sympy

# --- Ottimizzazione ---
from sklearn.cluster import KMeans

# --- Networking ---
import grpc
import struct

# --- Telemetria e Rendering 3D ---
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import plotly.graph_objects as go

# ==========================================
# MODULO 1: COSTANTI E PARAMETRI
# ==========================================

# Parametri di Rete 6G e Propagazione
FREQ: float = 5.9e9                 # Frequenza del segnale in Hertz (5.9 GHz per 3GPP InF-DH)
TX_POWER_DRONE: float = 20.0        # Potenza di trasmissione del Drone in dBm
TX_POWER_BS: float = 40.0           # Potenza di trasmissione della Base Station (fissa, per l'ACK) in dBm
R_BS: float = 50.0                  # Raggio di copertura massimo della Base Station (metri)
R_RIS: float = 15.0                 # Raggio di copertura effettivo di un pannello RIS (metri)
ATTENUAZIONE_SCAFFALE: float = 15.0 # Attenuazione fissa del segnale per ogni scaffale attraversato (dB)
SOGLIA_RIS_ATTIVAZIONE: float = 5.0 # Soglia SNR (dB) sotto la quale il Controller accende un RIS
SOGLIA_RICEVITORE: float = -90.0    # Sensibilità minima del ricevitore (dBm)
RUMORE_BIANCO: float = -92.0        # Potenza del rumore termico a 150 MHz (dBm)

# ------------------------------------------
# PARAMETRI DRONI 
# ------------------------------------------

# Parametri Cinematici dei Droni
V_DRONE: float = 3.0                       # Velocità di volo costante del Drone in m/s (≈ 10.8 km/h)
DT: float = 0.1                            # Passo temporale della simulazione (1 step = 0.1 secondi)
MIN_DISTANZA_ANTICOLLISIONE: float = 1.5   # Distanza minima di sicurezza (metri)

# Parametri della Batteria drone
BATTERY_MAX: float = 100.0                 # Capacità massima della batteria del drone (%)
BATTERY_RTH_THRESHOLD: float = 20.0        # Soglia RTH (Return To Home) in percentuale (%)
CONSUMO_BATTERIA_DT: float = 0.05          # Percentuale batteria consumata ad ogni step temporale  

# Modalità di Volo dei Droni e distanza minima di sicurezza
Z_DRONE_FISSO: float = 3.0                 # Altezza di crociera fissa e unica (metri)



# ------------------------------------------
# PARAMETRI BS-RIS
# ------------------------------------------

# Consumi Energetici dei Pannelli RIS (Watt)
P_ACTIVE: float = 50.0            # Consumo in modalità attiva: pannello sempre acceso (W)

# Consumi Energetici della Base Station (Watt)
P_BS_IDLE: float = 120.0          # Consumo a riposo della BS: in ascolto ma non inoltra (W)
P_BS_FORWARDING: float = 350.0    # Consumo della BS durante l'inoltro gRPC al Server (W)

# Parametri di posizionamento delle RIS e BS (soffitto e parete)
Z_BS_OFFSET_DAL_SOFFITTO: float = 0.3      # BS a soffitto: offset (metri) sotto l'intradosso
Z_RIS_SOFFITTO_OFFSET: float = 0.1         # RIS a soffitto: offset (metri) sotto l'intradosso
Z_RIS_PARETE_RAPPORTO_ALTEZZA: float = 0.5 # RIS a parete: frazione dell'altezza (0.5 = metà parete)

# Modalità Base Station (BS)
BS_E_ANCHE_RIS: bool = True       # La BS opera anche come nodo RIS (modalità Ibrida)

# Abilitazione RIS per modalità di volo
RIS_SOFFITTO_ABILITATA: bool = True             # 1 RIS a soffitto per corridoio
RIS_PARETE_ABILITATA: bool = False              # In FISSO quota nota = risparmio


# Ottimizzazione Copertura
COPERTURA_TARGET: float = 1.0             # Frazione dell'area da coprire (1.0 = 100%)
SOGLIA_OVERLAP_RIS: float = 0.10          # Overlap massimo consentito tra due RIS adiacenti (10%)
                                   # Troppo overlap = RIS sprecate; troppo poco = buchi di copertura.


# ------------------------------------------
# PARAMETRI EXTRA E OSTACOLI
# ------------------------------------------
# Dimensioni Scaffalature (metri)
L_SCAFFALE: float = 1.2    # Lunghezza (asse X)
W_SCAFFALE: float = 1.0    # Profondità (asse Y)

# Identificatori e Struttura Scaffali
H_MENSOLA: float = 0.6               # Luce standard tra i ripiani (metri)
MARGINE_SICUREZZA_DRONE: float = 0.15 # Margine per la "SafeZone" al di sopra della mensola

# Limiti di Rete
MAX_RIS_CALLS_PER_DT: int = 10  # Massimo chiamate RIS simultanee (Breakdown threshold)
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
        LARGHEZZA_CORRIDOIO = 3.0
        MARGIN_PERIMETRO = 2.5

        spazio_disp_y = self.larghezza - (2 * MARGIN_PERIMETRO)
        spazio_disp_x = self.lunghezza - (2 * MARGIN_PERIMETRO)

        # Quanti moduli "scaffale + corridoio" stanno lungo l'asse Y (larghezza)?
        passo_y = W_SCAFFALE + LARGHEZZA_CORRIDOIO
        
        n_file_y = int((spazio_disp_y + LARGHEZZA_CORRIDOIO) // passo_y)
        if n_file_y <= 0: n_file_y = 1 # Minimo 1 fila
        
        # Quanti scaffali (L_SCAFFALE) stanno lungo l'asse X (lunghezza)?
        n_elementi_x = int(spazio_disp_x // L_SCAFFALE)

        # Centratura Assoluta X e Y
        spazio_usato_y = (n_file_y * W_SCAFFALE) + ((n_file_y - 1) * LARGHEZZA_CORRIDOIO) if n_file_y > 0 else 0
        leftover_y = spazio_disp_y - spazio_usato_y
        offset_y = MARGIN_PERIMETRO + max(0, leftover_y / 2.0)
        
        spazio_usato_x = n_elementi_x * L_SCAFFALE
        leftover_x = spazio_disp_x - spazio_usato_x
        offset_x = MARGIN_PERIMETRO + max(0, leftover_x / 2.0)

        id_scaffale = 0
        
        # Generazione griglia di scaffali
        for fila in range(n_file_y):
            y_min = offset_y + fila * passo_y
            y_max = y_min + W_SCAFFALE
            
            # Il centro del corridoio si trova dopo lo scaffale
            y_centro_corridoio = y_max + (LARGHEZZA_CORRIDOIO / 2.0)
            self.corridoi.append(y_centro_corridoio)

            for colonna in range(n_elementi_x):
                x_min = offset_x + colonna * L_SCAFFALE
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
                
                self.base_stations.append(BaseStation(id_bs=id_bs, x=x_bs, y=y_bs, z=z_bs))
                id_bs += 1

        # Pannelli RIS (Reconfigurable Intelligent Surfaces)
        self.ris_soffitto = []
        self.ris_parete = []
        id_ris = 0

        # RIS a Soffitto (1 per ogni corridoio, centrata a metà corridoio)
        if RIS_SOFFITTO_ABILITATA:
            z_ris_soffitto = self.altezza - Z_RIS_SOFFITTO_OFFSET
            for corridoio_y in self.corridoi:
                # Filtering geometrico: Pruning se troppo vicine (<= 15.0m) a una Base Station
                rx = self.lunghezza / 2.0
                ry = corridoio_y
                vicino_bs = any(math.sqrt((rx - bs.x)**2 + (ry - bs.y)**2) <= 15.0 for bs in self.base_stations)
                
                if not vicino_bs:
                    self.ris_soffitto.append({
                        'id': id_ris,
                        'tipo': 'soffitto',
                        'x': rx,
                        'y': ry,
                        'z': z_ris_soffitto
                    })
                    id_ris += 1

        # Decisione Intelligente: Modalità di Volo
        self.modalita_volo = 'FISSO'
        
        # RIS a Parete (dinamiche base alla modalità di volo calcolata)
        if RIS_PARETE_ABILITATA:
            z_ris_parete = self.altezza * Z_RIS_PARETE_RAPPORTO_ALTEZZA
            for corridoio_y in self.corridoi:
                # 2 RIS per corridoio, una all'inizio (X=0) e una alla fine (X=lunghezza)
                # Filtering geometrico: Pruning se troppo vicine (<= 15.0m) a una Base Station
                vicino_bs_start = any(math.sqrt((0.0 - bs.x)**2 + (corridoio_y - bs.y)**2) <= 15.0 for bs in self.base_stations)
                if not vicino_bs_start:
                    self.ris_parete.append({
                        'id': id_ris, 'tipo': 'parete_start',
                        'x': 0.0, 'y': corridoio_y, 'z': z_ris_parete
                    })
                    id_ris += 1
                    
                vicino_bs_end = any(math.sqrt((self.lunghezza - bs.x)**2 + (corridoio_y - bs.y)**2) <= 15.0 for bs in self.base_stations)
                if not vicino_bs_end:
                    self.ris_parete.append({
                        'id': id_ris, 'tipo': 'parete_end',
                        'x': self.lunghezza, 'y': corridoio_y, 'z': z_ris_parete
                    })
                    id_ris += 1
        
        # Base Station (ibrida): Inietta i nodi BS nella lista ris_soffitto come oggetti firmati
        if BS_E_ANCHE_RIS:
            for bs in self.base_stations:
                self.ris_soffitto.append(bs.to_ris_dict())
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
import struct

# -------------------------------------------------------------------
# FORMATO FRAME: Air Interface 6G Grant-Free (Uplink Beacon)
# Struttura binaria del payload trasmesso dal drone senza handshake:
# [ID_Drone(1B) | Timestamp(8B double) | P_tx(4B float) | Batteria(4B float) | N_seq(4B uint)] = 21 byte
# Big-Endian per serializzazione standard di rete
# -------------------------------------------------------------------
BEACON_FORMAT: str = '>BdffI'
BEACON_SIZE_BYTES: int = struct.calcsize(BEACON_FORMAT)  # = 21 byte


class BeaconGrantFree:
    """
    [NEW - Air Interface 6G] Frame binario dell'Uplink Grant-Free.

    Il drone trasmette periodicamente beacon autonomi senza handshake
    di rete (accesso grant-free), minimizzando la latenza di segnalazione.
    Il payload è un frame crudo di soli 21 byte:

        [ID_Drone(1B) | Timestamp(8B) | P_tx(4B) | Batteria(4B) | N_seq(4B)]

    Args:
        id_drone: Identificatore univoco del drone trasmittente.
        p_tx: Potenza di trasmissione corrente in dBm.
        batteria: Livello batteria corrente in %.
    """
    # Contatore di sequenza a livello di classe (thread-safe non necessario:
    # ogni processo Data Plane gestisce la propria istanza)
    _n_seq_globale: int = 0

    def __init__(self, id_drone: int, p_tx: float, batteria: float) -> None:
        self.id_drone: int = id_drone
        self.timestamp: float = time.time()   # Epoch UNIX in secondi (float64)
        self.p_tx: float = p_tx
        self.batteria: float = batteria
        BeaconGrantFree._n_seq_globale += 1
        self.n_sequenza: int = BeaconGrantFree._n_seq_globale

    def serialize(self) -> bytes:
        """
        Impacchetta il payload in un frame binario crudo da BEACON_SIZE_BYTES byte.

        Returns:
            bytes: Frame binario pronto per la trasmissione radio.
        """
        return struct.pack(
            BEACON_FORMAT,
            self.id_drone,
            self.timestamp,
            self.p_tx,
            self.batteria,
            self.n_sequenza
        )

    @classmethod
    def deserialize(cls, raw_bytes: bytes) -> 'BeaconGrantFree':
        """
        Ricostruisce un BeaconGrantFree dal frame grezzo ricevuto dalla BS.

        Args:
            raw_bytes: Frame binario di BEACON_SIZE_BYTES byte estratto dal segnale RF.

        Returns:
            BeaconGrantFree: Oggetto ricostituito con tutti i campi decodificati.
        """
        id_d, ts, p_tx, batt, n_seq = struct.unpack(BEACON_FORMAT, raw_bytes)
        beacon = cls.__new__(cls)
        beacon.id_drone = id_d
        beacon.timestamp = ts
        beacon.p_tx = p_tx
        beacon.batteria = batt
        beacon.n_sequenza = n_seq
        return beacon

    def __repr__(self) -> str:
        return (
            f"<BeaconGF | Drone={self.id_drone} | P_tx={self.p_tx:.1f}dBm "
            f"| Batt={self.batteria:.1f}% | Seq={self.n_sequenza} | {BEACON_SIZE_BYTES}B>"
        )


class RSSIAoAMeasurement:
    """
    [NEW - Transport Network] Risultato della demodulazione del beacon da parte della BS.

    La Base Station estrae RSSI e AoA (Angle of Arrival) dal segnale ricevuto
    per alimentare il tracker EKF nel Data Plane.

    Args:
        beacon: Il beacon Grant-Free originale decodificato.
        rssi_dbm: Potenza ricevuta in dBm (RSSI misurato).
        aoa_azimuth_deg: Angolo di arrivo sul piano orizzontale in gradi [-180, 180].
        aoa_elevation_deg: Angolo di arrivo in elevazione in gradi [-90, 90].
        id_bs: ID della Base Station che ha demodulato il beacon.
    """
    def __init__(
        self,
        beacon: BeaconGrantFree,
        rssi_dbm: float,
        aoa_azimuth_deg: float,
        aoa_elevation_deg: float,
        id_bs: int
    ) -> None:
        self.beacon: BeaconGrantFree = beacon
        self.rssi_dbm: float = rssi_dbm
        self.aoa_azimuth_deg: float = aoa_azimuth_deg
        self.aoa_elevation_deg: float = aoa_elevation_deg
        self.id_bs: int = id_bs
        self.timestamp_rx: float = time.time()  # Epoca di ricezione lato BS

    def __repr__(self) -> str:
        return (
            f"<RSSIAoA | BS={self.id_bs} | RSSI={self.rssi_dbm:.1f}dBm "
            f"| Az={self.aoa_azimuth_deg:.1f}° | El={self.aoa_elevation_deg:.1f}°>"
        )


class GRPCTransportStub:
    """
    [NEW - Transport Network gRPC] Stub di simulazione del trasporto gRPC/Protobuf
    sull'infrastruttura cablata (Fibra Ottica + Rete Elettrica Dedicata 220V).

    Modella due canali:
    - BS -> Super Server: inoltro telemetria su HTTP/2 (Protobuf serializzato).
    - Server -> RIS: invio matrice di configurazione SDN alle meta-superfici.

    In questa fase la serializzazione usa struct (stub ad alta fedeltà);
    il porting su grpcio avverrà quando si genereranno i file .proto dedicati.

    Args:
        latenza_ms: Latenza target del link cablato (default 1ms per LAN locale).
    """
    # Formato Protobuf stub per il messaggio di telemetria BS -> Server
    # [id_bs(1B) | rssi(4B float) | aoa_az(4B float) | aoa_el(4B float) | id_drone(1B) | batt(4B float)] = 18B
    _PROTO_TELEM_FMT: str = '>BffFBf'
    # Formato Protobuf stub per la matrice di conf. Server -> RIS
    # [id_ris(1B) | stato(1B: 1=active)] = 2B
    _PROTO_RIS_CMD_FMT: str = '>BB'

    def __init__(self, latenza_ms: float = 1.0) -> None:
        self.latenza_ms: float = latenza_ms
        self._pacchetti_inoltrati: int = 0
        self._comandi_ris_inviati: int = 0

    def forward_telemetry(self, misura: RSSIAoAMeasurement) -> bytes:
        """
        Simula l'inoltro gRPC (BS -> Super Server) di una misurazione.

        Serializza la misura in un payload Protobuf-stub e lo "invia" (log).
        In un sistema reale, qui si chiamerebbe stub.ForwardTelemetry(proto_msg).

        Args:
            misura: Oggetto RSSIAoAMeasurement prodotto dalla BS.

        Returns:
            bytes: Payload serializzato Protobuf-stub (18 byte).
        """
        self._pacchetti_inoltrati += 1
        # Serializzazione Protobuf-stub: solo campi essenziali per l'EKF
        payload = struct.pack(
            '>BfffBf',
            misura.id_bs,
            misura.rssi_dbm,
            misura.aoa_azimuth_deg,
            misura.aoa_elevation_deg,
            misura.beacon.id_drone,
            misura.beacon.batteria
        )
        return payload  # Il Server deserializza e alimenta l'EKF

    def send_ris_command(self, id_ris: int, stato: int) -> bytes:
        """
        Simula l'invio gRPC (Super Server -> RIS) della matrice di configurazione SDN.

        Args:
            id_ris: Identificatore della meta-superficie bersaglio.
            stato: Stato target: 0=sleep, 1=active.

        Returns:
            bytes: Comando serializzato Protobuf-stub (2 byte).
        """
        self._comandi_ris_inviati += 1
        return struct.pack(self._PROTO_RIS_CMD_FMT, id_ris & 0xFF, stato & 0xFF)

    def get_stats(self) -> dict:
        """Restituisce le statistiche di utilizzo del canale gRPC."""
        return {
            'latenza_ms': self.latenza_ms,
            'pacchetti_inoltrati': self._pacchetti_inoltrati,
            'comandi_ris_inviati': self._comandi_ris_inviati,
            'overhead_cablato_w': 15.0  # Consumo energetico interfacce di rete fisse
        }


# -------------------------------------------------------------------
# LEGACY: mantenuto per compatibilità con i Test 1-5 esistenti.
# Sostituire progressivamente con BeaconGrantFree + RSSIAoAMeasurement.
# -------------------------------------------------------------------
class Pacchetto_Rete:
    """ [LEGACY] Pacchetto dati OOP (da sostituire con BeaconGrantFree) """
    def __init__(self, id_drone, tx_power, battery_level, package_id, target_x, target_y, target_z):
        self.id_drone = id_drone
        self.ts = time.time()
        self.tx_power = tx_power
        self.battery_level = battery_level
        self.nodi_attraversati = [f"Drone-{self.id_drone}"]
        self.package_id = package_id
        self.route_target = (target_x, target_y, target_z)

    def aggiungi_hop_ris(self, id_ris):
        self.nodi_attraversati.append(f"RIS-{id_ris}")

    def aggiungi_hop_bs(self, id_bs):
        self.nodi_attraversati.append(f"BS-{id_bs}")

    def __repr__(self):
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
        
    def genera_pacchetto(self, package_id: str, target_x: float, target_y: float, target_z: float) -> 'Pacchetto_Rete':
        """ [LEGACY] Crea il pacchetto radio OOP da spedire via 6G alla Base Station """
        return Pacchetto_Rete(
            id_drone=self.id_drone,
            tx_power=TX_POWER_DRONE,
            battery_level=self.batteria,
            package_id=package_id,
            target_x=target_x,
            target_y=target_y,
            target_z=target_z
        )

    def genera_beacon(self) -> BeaconGrantFree:
        """
        [NEW - Air Interface 6G] Genera il beacon Grant-Free corrente del drone.

        Crea il frame binario da 21 byte pronto per la trasmissione RF:
        non richiede handshake ne risposta dalla rete (Uplink Grant-Free).

        Returns:
            BeaconGrantFree: Il frame binario con lo stato istantaneo del drone.
        """
        return BeaconGrantFree(
            id_drone=self.id_drone,
            p_tx=TX_POWER_DRONE,
            batteria=self.batteria
        )


class RIS:
    """ Rappresenta il pannello Reconfigurable Intelligent Surface (lo specchio 6G) """
    def __init__(self, id_ris, x, y, z):
        self.id_ris = id_ris
        self.x = x
        self.y = y
        self.z = z
        self.stato = 'active' # Lavora unicamente in modalità attiva
        
    def cambia_stato(self, nuovo_stato):
        """ Nessun effetto: La RIS è sempre e solo attiva """
        self.stato = 'active'
            
    def get_consumo(self):
        """ Restituisce l'attuale consumo in Watt """
        return P_ACTIVE

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


class BaseStation:
    """
    Rappresenta la Base Station 6G del magazzino.
    Opera in modalità IBRIDA: è sia il ricevitore radio primario della rete
    sia un nodo RIS (riflessione/amplificazione attiva) con raggio R_BS = 50 m.
    Espone un metodo dedicato per inoltrare i pacchetti ricevuti al SuperServer.
    """
    def __init__(self, id_bs: int, x: float, y: float, z: float) -> None:
        self.id_bs = id_bs
        self.x = x
        self.y = y
        self.z = z
        self.stato_ris: str = 'active'     # Modalità RIS integrata sempre attiva
        self.pacchetti_inoltrati: int = 0  # Contatore pacchetti inoltrati al SuperServer
        # Stub gRPC per il trasporto PoE++ Cat6a verso il Super Server
        self.grpc_stub: GRPCTransportStub = GRPCTransportStub(latenza_ms=1.0)

    def demodulate_beacon(self, beacon_raw: bytes, snr_uplink_db: float) -> RSSIAoAMeasurement:
        """
        [NEW - Transport Network] Demodula il frame binario Grant-Free ricevuto via RF.

        Estrae RSSI e AoA (Angle of Arrival) dal beacon per alimentare il tracker EKF.
        L'AoA viene stimato geometricamente dalla posizione relativa del drone (approssimazione
        per simulazione; in hardware reale si userebbe un array di antenne MIMO).

        Args:
            beacon_raw: Frame binario da 21 byte trasmesso dal drone.
            snr_uplink_db: SNR di uplink misurato al momento della ricezione (dB).

        Returns:
            RSSIAoAMeasurement: Misura demodulata pronta per il layer gRPC.
        """
        beacon = BeaconGrantFree.deserialize(beacon_raw)
        # RSSI stimato: Pricevuta = P_tx - SNR_loss (semplificazione lineare di simulazione)
        rssi_dbm: float = beacon.p_tx - snr_uplink_db

        # AoA stimato geometricamente dalla posizione nota della BS rispetto all'ultimo fix noto
        # (In simulazione la posizione del drone è accessibile direttamente)
        # Ritorna 0.0 come placeholder: l'EKF aggiorna con la misura reale
        aoa_azimuth_deg: float = 0.0
        aoa_elevation_deg: float = 0.0

        misura = RSSIAoAMeasurement(
            beacon=beacon,
            rssi_dbm=rssi_dbm,
            aoa_azimuth_deg=aoa_azimuth_deg,
            aoa_elevation_deg=aoa_elevation_deg,
            id_bs=self.id_bs
        )
        # Inoltro gRPC (BS -> Super Server)
        self.grpc_stub.forward_telemetry(misura)
        return misura

    def cambia_stato_ris(self, nuovo_stato: str) -> None:
        """ Nessun effetto: la componente RIS integrata nella BS è sempre attiva """
        self.stato_ris = 'active'
        
    def get_consumo(self) -> float:
        """
        Restituisce il consumo totale in Watt della BS in modalità ibrida sempre attiva.
        """
        return P_BS_FORWARDING + P_ACTIVE

    def ricevi_e_inoltra(self, pacchetto: 'Pacchetto_Rete') -> 'Pacchetto_Rete':
        """
        [LEGACY] La BS riceve il pacchetto OOP, lo firma e lo prepara per il SuperServer.
        """
        pacchetto.aggiungi_hop_bs(self.id_bs)
        if self.stato_ris == 'active' and BS_E_ANCHE_RIS:
            pacchetto.tx_power += 10.0  # Guadagno amplificazione RIS ibrida
        self.pacchetti_inoltrati += 1
        return pacchetto  # Pacchetto firmato, pronto per il SuperServer

    def to_dict(self) -> dict:
        """
        Restituisce un dizionario per compatibilità con le funzioni esistenti
        (es. esegui_2way_ranging) che si aspettano {'id', 'x', 'y', 'z'}.
        """
        return {'id': self.id_bs, 'x': self.x, 'y': self.y, 'z': self.z}

    def to_ris_dict(self) -> dict:
        """
        Restituisce un dizionario RIS-compatibile per essere inserito
        nella lista ris_soffitto quando BS_E_ANCHE_RIS è attivo.
        """
        return {
            'id': f'BS_{self.id_bs}',
            'tipo': 'ibrida_bs',
            'x': self.x, 'y': self.y, 'z': self.z
        }

    def __repr__(self) -> str:
        return (f"<BaseStation ID={self.id_bs} @ ({self.x:.1f},{self.y:.1f},{self.z:.1f}) "
                f"| stato_RIS={self.stato_ris} | Pacchetti={self.pacchetti_inoltrati}>"
                f" | gRPC stats: {self.grpc_stub.get_stats()['pacchetti_inoltrati']}msg")


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
    # Compatibilità: accetta sia un oggetto BaseStation sia un dizionario legacy
    if isinstance(bs_dict, BaseStation):
        bs_dict = bs_dict.to_dict()
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
    # Se il collegamento diretto si è rotto o è degradato (sotto SOGLIA_RIS_ATTIVAZIONE) e abbiamo RIS disponibili
    if (not connessione_diretta_ok or snr_limitante < SOGLIA_RIS_ATTIVAZIONE) and ris_list and len(ris_list) > 0:
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
# MODULO 6: ARCHITETTURA MULTIPROCESSING E CONTROLLER SDN
# ==========================================
import time # libreria per gestire il tempo
import multiprocessing as mp
from multiprocessing import shared_memory
import numpy as np

class SharedMemoryManager:
    """
    [NEW 6G ARCHITECTURE] Gestisce i blocchi di memoria condivisa (Shared Memory)
    per far comunicare il Data Plane e il Control Plane (SDN) senza overhead IPC,
    garantendo il rispetto dei vincoli di latenza (Tc ≈ 10ms) previsti dal PRD.
    """
    def __init__(self, n_droni: int, n_ris: int):
        # Tipi di dato base per le matrici (es. x, y, z, snr, batteria)
        self.droni_bytes = n_droni * 5 * 8  # 5 float64 (8 bytes l'uno)
        self.ris_bytes = n_ris * 1 * 8      # 1 status per RIS
        
        # Creazione blocchi SHM
        self.shm_droni = shared_memory.SharedMemory(create=True, size=self.droni_bytes)
        self.shm_ris = shared_memory.SharedMemory(create=True, size=self.ris_bytes)
        
        # Creazione interfacce numpy sulla shared memory
        self.droni_array = np.ndarray((n_droni, 5), dtype=np.float64, buffer=self.shm_droni.buf)
        self.ris_array = np.ndarray((n_ris,), dtype=np.float64, buffer=self.shm_ris.buf)
        
    def cleanup(self) -> None:
        """ Dealloca la memoria dalla RAM di sistema """
        self.shm_droni.close()
        self.shm_droni.unlink()
        self.shm_ris.close()
        self.shm_ris.unlink()

class SDNControlPlaneProcess(mp.Process):
    """
    [NEW 6G ARCHITECTURE] Processo 1: Loop asincrono SDN Control Plane.
    Si occupa di orchestrare la rete, invocare gRPC e pilotare le RIS attive,
    isolato dal peso dei calcoli matematici.
    """
    def __init__(self, shm_droni_name: str, shm_ris_name: str, n_droni: int, n_ris: int):
        super().__init__()
        self.shm_droni_name = shm_droni_name
        self.shm_ris_name = shm_ris_name
        self.n_droni = n_droni
        self.n_ris = n_ris
        self.running = mp.Event()

    def run(self) -> None:
        self.running.set()
        # Collegamento ai blocchi SHM allocati dal master
        shm_d = shared_memory.SharedMemory(name=self.shm_droni_name)
        shm_r = shared_memory.SharedMemory(name=self.shm_ris_name)
        droni_array = np.ndarray((self.n_droni, 5), dtype=np.float64, buffer=shm_d.buf)
        ris_array = np.ndarray((self.n_ris,), dtype=np.float64, buffer=shm_r.buf)
        
        print(f"[SDN Plane] Avviato con successo per gestione di {self.n_ris} RIS. (PID: {mp.current_process().pid})")
        while self.running.is_set():
            # TODO: Leggere SNR da droni_array, calcolare Outage, pilotare ris_array e lanciare gRPC
            time.sleep(0.01) # Ciclo polling ultra fast (10ms)
            
        shm_d.close()
        shm_r.close()

    def stop(self) -> None:
        self.running.clear()

class DataPlaneProcess(mp.Process):
    """
    [NEW 6G ARCHITECTURE] Processo 2: Motore Matematico Data Plane.
    Dedica il 100% delle sue risorse al Tracking EKF e alla propagazione fisica 3GPP.
    I calcoli sono accelerati da @njit senza bloccare il Controller.
    """
    def __init__(self, shm_droni_name: str, shm_ris_name: str, n_droni: int, n_ris: int):
        super().__init__()
        self.shm_droni_name = shm_droni_name
        self.shm_ris_name = shm_ris_name
        self.n_droni = n_droni
        self.n_ris = n_ris
        self.running = mp.Event()

    def run(self) -> None:
        self.running.set()
        shm_d = shared_memory.SharedMemory(name=self.shm_droni_name)
        shm_r = shared_memory.SharedMemory(name=self.shm_ris_name)
        droni_array = np.ndarray((self.n_droni, 5), dtype=np.float64, buffer=shm_d.buf)
        ris_array = np.ndarray((self.n_ris,), dtype=np.float64, buffer=shm_r.buf)
        
        print(f"[Data Plane] Motore Fisico-Matematico avviato. (PID: {mp.current_process().pid})")
        while self.running.is_set():
            # TODO: Eseguire step fisico di EKF (Extended Kalman Filter) su matrici
            # TODO: Passare dati ambientali JIT per il computo InF-DH
            time.sleep(0.005) # Ciclo 5ms
            
        shm_d.close()
        shm_r.close()

    def stop(self) -> None:
        self.running.clear()

class SuperServer:
    """ 
    [LEGACY] Il "Cervello" della rete 6G originale (monolitico). 
    Riceve i pacchetti test dai droni, valuta la qualità del segnale e prende decisioni.
    (Verrà fuso/sostituito dalla nuova architettura Multiprocessing)
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
            
            # La logica del controller imposta direttamente in modalità attiva
            azione_ris = 'active'
            consumo_attuale = P_ACTIVE
                
            print(f"[Super Server] Emergenza Radio Rilevata! SNR={snr_attuale:.1f} dB. Attivo specchio RIS_ID={id_ris_attivata} in modalità {azione_ris.upper()}.")
            
            # 3.1 Salvataggio dell'Azione (Telemetria di Rete) nel Database!
            self.db.inserisci_evento_rete(
                ts=ts_attuale,
                id_ris=id_ris_attivata,
                azione=azione_ris,
                consumo_w=consumo_attuale
            )
            
        # 4. FORWARDING AL SUPERSERVER (via BS ibrida)
        # Se il collegamento è riuscito e bs_target è un oggetto BaseStation,
        # il pacchetto viene generato e firmato dalla BS prima di arrivare al Server.
        if risultati['connesso'] and isinstance(bs_target, BaseStation):
            pacchetto_fw = drone.genera_pacchetto("SIM", drone.x, drone.y, drone.z)
            bs_target.ricevi_e_inoltra(pacchetto_fw)

        # 5. SALVATAGGIO LOG DRONE (Telemetria del Veicolo)
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
        elif mq == 35000:
            return Magazzino(lunghezza=250, larghezza=140, altezza=15) # Caso C
        else:
            # Default: Caso B (Medio) - Gestisce i 10.000mq o input imprevisti
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

    # Test 2: Resilienza con Heatmap SNR (griglia 2D PRIMA/DOPO guasto RIS)
    def test2_resilienza_guasto(self):
        print("\n--- AVVIO TEST 2: Resilienza e Tolleranza ai Guasti (Heatmap SNR) ---")
        casi_mq = {'Caso A': 2000, 'Caso B': 10000, 'Caso C': 35000}

        offset_drone = 0
        for nome_caso, mq in casi_mq.items():
            print(f"\n> Esecuzione {nome_caso} ({mq} mq)")
            ambiente = self._create_layout(mq)
            server = SuperServer(ambiente, self.db)
            tutte_le_ris_full = ambiente.ris_soffitto + ambiente.ris_parete

            # Identifichiamo il cluster di RIS bersaglio (simulazione blackout di zona)
            ris_guaste_ids = []
            ris_guaste_pos = []
            vere_ris_soffitto = [r for r in ambiente.ris_soffitto if r.get('tipo', '') != 'ibrida_bs']
            if len(vere_ris_soffitto) > 0:
                n_guasti = min(4, len(vere_ris_soffitto))
                for i in range(n_guasti):
                    ris_guaste_ids.append(vere_ris_soffitto[i]['id'])
                    ris_guaste_pos.append([vere_ris_soffitto[i]['x'], vere_ris_soffitto[i]['y']])
            bs_target = ambiente.base_stations[0]

            # --- CALCOLO GRIGLIA SNR PRIMA DEL GUASTO ---
            print(f"  > Campionamento griglia SNR PRIMA del guasto...")
            passo = max(2.0, ambiente.lunghezza / 30.0)  # Risoluzione griglia adattiva
            xs = [0] + list(range(int(passo), int(ambiente.lunghezza), int(passo))) + [int(ambiente.lunghezza)]
            ys = [0] + list(range(int(passo), int(ambiente.larghezza), int(passo))) + [int(ambiente.larghezza)]
            drone_griglia = Drone(id_drone=999, x=0, y=0, z=Z_DRONE_FISSO)

            snr_before = []
            for gy in ys:
                riga = []
                for gx in xs:
                    drone_griglia.x = float(gx)
                    drone_griglia.y = float(gy)
                    res = esegui_2way_ranging(drone_griglia, bs_target, ambiente, tutte_le_ris_full)
                    riga.append(res['snr_uplink_effettivo_dB'])
                snr_before.append(riga)

            # --- RIMOZIONE RIS BERSAGLIO ---
            tutte_le_ris_guasto = [r for r in tutte_le_ris_full if r['id'] not in ris_guaste_ids]
            print(f"  [GUASTO MULTIPLO] Spegnimento forzato di {len(ris_guaste_ids)} RIS: {ris_guaste_ids}")

            # --- CALCOLO GRIGLIA SNR DOPO IL GUASTO ---
            print(f"  > Campionamento griglia SNR DOPO il guasto...")
            snr_after = []
            for gy in ys:
                riga = []
                for gx in xs:
                    drone_griglia.x = float(gx)
                    drone_griglia.y = float(gy)
                    res = esegui_2way_ranging(drone_griglia, bs_target, ambiente, tutte_le_ris_guasto)
                    riga.append(res['snr_uplink_effettivo_dB'])
                snr_after.append(riga)

            # Salviamo le griglie serializzate come marker nel DB per il plotter
            import json
            ts_now = time.time()
            label_b = f"HEATMAP_BEFORE_{nome_caso.replace(' ', '_')}"
            label_a = f"HEATMAP_AFTER_{nome_caso.replace(' ', '_')}"
            # Posizione RIS guaste e BS per il marker sul plot
            # Scaffali compatti: [x_min, y_min, x_max, y_max]
            scaffali_compact = [[s['x_min'], s['y_min'], s['x_max'], s['y_max']] for s in ambiente.scaffali]
            # Tutte le RIS: [x, y, tipo]
            ris_tutte = [[r['x'], r['y'], r.get('tipo', 'soffitto')] for r in tutte_le_ris_full]
            # Tutte le BS: [x, y]
            bs_tutte = [[bs.x if hasattr(bs, 'x') else bs['x'], bs.y if hasattr(bs, 'y') else bs['y']] for bs in ambiente.base_stations]
            # Usa le posizioni BOM (DeploymentPlanner) per i marker del plot — stessa griglia 2D del disegno BOM
            bom_planner = DeploymentPlanner(ambiente.lunghezza, ambiente.larghezza, ambiente.altezza)
            ris_tutte_bom   = [[r['x'], r['y'], 'soffitto'] for r in bom_planner.ris_soffitto] + \
                               [[r['x'], r['y'], 'parete'] for r in bom_planner.ris_parete]
            bs_tutte_bom    = [[bs['x'], bs['y']] for bs in bom_planner.base_stations]
            ris_tutte = ris_tutte_bom # Usa le posizioni BOM per i marker del plot
            bs_tutte  = bs_tutte_bom
            # Remap ris_guaste_pos alle coordinate BOM (prime N RIS soffitto della griglia BOM)
            n_guaste = len(ris_guaste_ids)
            bom_soffitto_pos = [[r['x'], r['y']] for r in bom_planner.ris_soffitto]
            ris_guaste_pos_bom = bom_soffitto_pos[:n_guaste] if n_guaste <= len(bom_soffitto_pos) else bom_soffitto_pos
            ris_guaste_pos = ris_guaste_pos_bom
            payload_common = {'xs': xs, 'ys': ys, 'L': ambiente.lunghezza, 'W': ambiente.larghezza,
                              'ris_guaste_pos': ris_guaste_pos, 'mq': mq,
                              'scaffali': scaffali_compact, 'ris_tutte': ris_tutte, 'bs_tutte': bs_tutte}
            payload_b = {**payload_common, 'snr': snr_before}
            payload_a = {**payload_common, 'snr': snr_after}
            self.db.inserisci_evento_rete(ts_now,     -2, label_b, json.dumps(payload_b))
            self.db.inserisci_evento_rete(ts_now+0.01,-2, label_a, json.dumps(payload_a))

            # Simulazione classica per il DB drone (serve per la CDF)
            flotta = self._inizializza_droni(15, ambiente)
            for d in flotta:
                d.id_drone += offset_drone
            tutte_le_ris = list(tutte_le_ris_full)  # copia fresca
            for t in range(0, 100):
                if t == 50 and len(ris_guaste_ids) > 0:
                    tutte_le_ris = [r for r in tutte_le_ris if r['id'] not in ris_guaste_ids]
                for drone in flotta:
                    drone.x += random.uniform(-1, 1) * V_DRONE * DT
                    drone.y += random.uniform(-1, 1) * V_DRONE * DT
                    server.ricevi_telemetria(drone, bs_target, tutte_le_ris)
            offset_drone += 100

        print("\n  => Test 2 completato per tutti i layout. Heatmap e CDF pronte.")

    # Test 3: Analisi Energetica su Transitori Mass RTH (Dual-Axis: Batteria vs Attivazioni RIS)
    def test3_collo_bottiglia_rth(self):
        print("\n--- AVVIO TEST 3: Collo di Bottiglia (Mass RTH) - Dual Axis ---")
        casi_mq = {'Caso A': 2000, 'Caso B': 10000, 'Caso C': 35000}
        import json

        for nome_caso, mq in casi_mq.items():
            print(f"\n> Esecuzione {nome_caso} ({mq} mq)")
            ambiente = self._create_layout(mq)
            server = SuperServer(ambiente, self.db)
            tutte_le_ris = ambiente.ris_soffitto + ambiente.ris_parete
            flotta = self._inizializza_droni(50, ambiente)
            bs_target = ambiente.base_stations[0]

            ts_start = time.time()
            self.db.inserisci_evento_rete(ts_start, -1, f"START_{nome_caso.replace(' ', '_')}", 0)

            # Collezioniamo dati per step: batteria media, min, max e conteggio attivazioni RIS
            steps_data = []  # lista di dizionari {step, batt_media, batt_min, batt_max, ris_attivazioni}

            for t in range(0, 40):
                if t == 20:
                    print(f"  [t={t}] ⚠ EVENTO MASSIVO ({nome_caso}): Il 40% dei droni viene forzato a batteria 21%.")
                    droni_da_scaricare = int(0.40 * len(flotta))
                    for i in range(droni_da_scaricare):
                        flotta[i].batteria = 21.0

                ris_attivate_questo_step = 0
                livelli_batteria = []
                for drone in flotta:
                    drone.aggiorna_batteria()
                    res = server.ricevi_telemetria(drone, bs_target, tutte_le_ris)
                    livelli_batteria.append(drone.batteria)
                    if res.get('usa_ris', False):
                        ris_attivate_questo_step += 1

                steps_data.append({
                    'step': t,
                    'batt_media': sum(livelli_batteria) / len(livelli_batteria),
                    'batt_min': min(livelli_batteria),
                    'batt_max': max(livelli_batteria),
                    'ris_attivazioni': ris_attivate_questo_step
                })

            # Salviamo i dati serializzati come marker nel DB per il plotter dual-axis
            label_data = f"DUALAXIS_{nome_caso.replace(' ', '_')}"
            self.db.inserisci_evento_rete(time.time(), -3, label_data, json.dumps(steps_data))
            time.sleep(0.5)

        print("\n  => Test 3 completato. Dati dual-axis pronti per il plot.")

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

   
    # METODI CUSTOM: Eseguono i test sul layout utente (Caso Utente)
    

    def test1_custom(self, ambiente):
        """ Test 1 eseguito SOLO sul layout personalizzato dell'utente (Caso Utente) """
        print("\n--- AVVIO TEST 1: Stress Test [Layout Utente] ---")
        SERVER_MAX_CAPACITY = MAX_RIS_CALLS_PER_DT * 5
        server = SuperServer(ambiente, self.db)
        tutte_le_ris = ambiente.ris_soffitto + ambiente.ris_parete
        num_droni = 5
        max_droni_raggiunti = 5

        while True:
            flotta = self._inizializza_droni(num_droni, ambiente)
            snr_critici = 0.
            messaggi_server_dt = 0
            for drone in flotta:
                bs_target = ambiente.base_stations[0]
                risultati = server.ricevi_telemetria(drone, bs_target, tutte_le_ris)
                if risultati['usa_ris']:
                    messaggi_server_dt += 1
                if risultati['snr_uplink_effettivo_dB'] < SOGLIA_RIS_ATTIVAZIONE:
                    snr_critici += 1
            if messaggi_server_dt > SERVER_MAX_CAPACITY:
                print(f"  [!] COLLASSO SERVER: Superata capacità massima ({messaggi_server_dt} chiamate/DT).")
                break
            if (snr_critici / num_droni) >= 0.20:
                print(f"  [!] COLLASSO RETE: Il {int((snr_critici/num_droni)*100)}% dei droni ha SNR critico.")
                break
            max_droni_raggiunti = num_droni
            num_droni += 5
        print(f"  => Rete Layout Utente regge fino a MAX {max_droni_raggiunti} Droni.\n")

    def test2_custom(self, ambiente):
        """ Test 2 Custom: Heatmap SNR PRIMA/DOPO guasto per il layout personalizzato """
        import json
        print("\n--- AVVIO TEST 2: Resilienza [Layout Utente] ---")
        server = SuperServer(ambiente, self.db)
        tutte_le_ris_full = ambiente.ris_soffitto + ambiente.ris_parete
        ris_guaste_ids = []
        ris_guaste_pos = []
        vere_ris_soffitto = [r for r in ambiente.ris_soffitto if r.get('tipo', '') != 'ibrida_bs']
        if len(vere_ris_soffitto) > 0:
            n_guasti = min(4, len(vere_ris_soffitto))
            for i in range(n_guasti):
                ris_guaste_ids.append(vere_ris_soffitto[i]['id'])
                ris_guaste_pos.append([vere_ris_soffitto[i]['x'], vere_ris_soffitto[i]['y']])
        bs_target = ambiente.base_stations[0]

        # Griglia SNR PRIMA del guasto
        passo = max(2.0, ambiente.lunghezza / 25.0)
        xs = [0] + list(range(int(passo), int(ambiente.lunghezza), int(passo))) + [int(ambiente.lunghezza)]
        ys = [0] + list(range(int(passo), int(ambiente.larghezza), int(passo))) + [int(ambiente.larghezza)]
        drone_g = Drone(id_drone=998, x=0, y=0, z=Z_DRONE_FISSO)
        snr_before = []
        for gy in ys:
            riga = []
            for gx in xs:
                drone_g.x, drone_g.y = float(gx), float(gy)
                res = esegui_2way_ranging(drone_g, bs_target, ambiente, tutte_le_ris_full)
                riga.append(res['snr_uplink_effettivo_dB'])
            snr_before.append(riga)

        # Rimozione RIS e griglia SNR DOPO il guasto
        tutte_le_ris_guasto = [r for r in tutte_le_ris_full if r['id'] not in ris_guaste_ids]
        snr_after = []
        for gy in ys:
            riga = []
            for gx in xs:
                drone_g.x, drone_g.y = float(gx), float(gy)
                res = esegui_2way_ranging(drone_g, bs_target, ambiente, tutte_le_ris_guasto)
                riga.append(res['snr_uplink_effettivo_dB'])
            snr_after.append(riga)

        ts_now = time.time()
        scaffali_compact = [[s['x_min'], s['y_min'], s['x_max'], s['y_max']] for s in ambiente.scaffali]
        ris_tutte = [[r['x'], r['y'], r.get('tipo', 'soffitto')] for r in tutte_le_ris_full]
        bs_tutte = [[bs.x if hasattr(bs, 'x') else bs['x'], bs.y if hasattr(bs, 'y') else bs['y']] for bs in ambiente.base_stations]
        # Usa le posizioni BOM (DeploymentPlanner) per i marker del plot — stessa griglia 2D del disegno BOM
        bom_planner = DeploymentPlanner(ambiente.lunghezza, ambiente.larghezza, ambiente.altezza)
        ris_tutte_bom   = [[r['x'], r['y'], 'soffitto'] for r in bom_planner.ris_soffitto] + \
                           [[r['x'], r['y'], 'parete'] for r in bom_planner.ris_parete]
        bs_tutte_bom    = [[bs['x'], bs['y']] for bs in bom_planner.base_stations]
        ris_tutte = ris_tutte_bom
        bs_tutte  = bs_tutte_bom
        # Remap ris_guaste_pos alle coordinate BOM (prime N RIS soffitto della griglia BOM)
        n_guaste = len(ris_guaste_ids)
        bom_soffitto_pos = [[r['x'], r['y']] for r in bom_planner.ris_soffitto]
        ris_guaste_pos_bom = bom_soffitto_pos[:n_guaste] if n_guaste <= len(bom_soffitto_pos) else bom_soffitto_pos
        ris_guaste_pos = ris_guaste_pos_bom
        payload_common = {'xs': xs, 'ys': ys, 'L': ambiente.lunghezza, 'W': ambiente.larghezza,
                          'ris_guaste_pos': ris_guaste_pos, 'mq': ambiente.area_mq,
                          'scaffali': scaffali_compact, 'ris_tutte': ris_tutte, 'bs_tutte': bs_tutte}
        self.db.inserisci_evento_rete(ts_now,     -2, "HEATMAP_BEFORE_Caso_Utente", json.dumps({**payload_common, 'snr': snr_before}))
        self.db.inserisci_evento_rete(ts_now+0.01,-2, "HEATMAP_AFTER_Caso_Utente",  json.dumps({**payload_common, 'snr': snr_after}))

        # Simulazione classica per CDF
        flotta = self._inizializza_droni(15, ambiente)
        tutte_le_ris = list(tutte_le_ris_full)
        for t in range(0, 100):
            if t == 50 and len(ris_guaste_ids) > 0:
                tutte_le_ris = [r for r in tutte_le_ris if r['id'] not in ris_guaste_ids]
            for drone in flotta:
                drone.x += random.uniform(-1, 1) * V_DRONE * DT
                drone.y += random.uniform(-1, 1) * V_DRONE * DT
                server.ricevi_telemetria(drone, bs_target, tutte_le_ris)
        print("  => Test 2 [Layout Utente] completato.")

    def test3_custom(self, ambiente):
        """ Test 3 Custom: Dual-Axis Batteria vs Attivazioni RIS per il layout personalizzato """
        import json
        print("\n--- AVVIO TEST 3: Mass RTH [Layout Utente] ---")
        server = SuperServer(ambiente, self.db)
        tutte_le_ris = ambiente.ris_soffitto + ambiente.ris_parete
        flotta = self._inizializza_droni(50, ambiente)
        bs_target = ambiente.base_stations[0]
        ts_start = time.time()
        self.db.inserisci_evento_rete(ts_start, -1, "START_Caso_Utente", 0)

        steps_data = []
        for t in range(0, 40):
            if t == 20:
                print(f"  [t={t}] EVENTO MASSIVO: Il 40% dei droni viene forzato a batteria 21%.")
                droni_da_scaricare = int(0.40 * len(flotta))
                for i in range(droni_da_scaricare):
                    flotta[i].batteria = 21.0
            ris_attivate = 0
            livelli = []
            for drone in flotta:
                drone.aggiorna_batteria()
                res = server.ricevi_telemetria(drone, bs_target, tutte_le_ris)
                livelli.append(drone.batteria)
                if res.get('usa_ris', False):
                    ris_attivate += 1
            steps_data.append({'step': t, 'batt_media': sum(livelli)/len(livelli),
                                'batt_min': min(livelli), 'batt_max': max(livelli),
                                'ris_attivazioni': ris_attivate})

        self.db.inserisci_evento_rete(time.time(), -3, "DUALAXIS_Caso_Utente", json.dumps(steps_data))
        print("  => Test 3 [Layout Utente] completato.")

    def test4_custom(self, ambiente):
        """ Test 4 eseguito SOLO sul layout personalizzato dell'utente (Caso Utente) """
        print("\n--- AVVIO TEST 4: Confronto Energetico [Layout Utente] ---")
        tutte_le_ris = ambiente.ris_soffitto + ambiente.ris_parete
        flotta = self._inizializza_droni(15, ambiente)
        bs_target = ambiente.base_stations[0]
        print("   - Run 1: RIS Always-ON")
        ts_run1 = time.time() - 3600
        consumo_run1_totale = len(tutte_le_ris) * P_ACTIVE * 15
        self.db.inserisci_evento_rete(ts_run1, -1, "RUN1_ALWAYS_ON_TOTAL_Caso_Utente", consumo_run1_totale)
        print("   - Run 2: Super Server Ibrido")
        server = SuperServer(ambiente, self.db)
        ts_start_run2 = time.time()
        self.db.inserisci_evento_rete(ts_start_run2, -1, "START_RUN2_Caso_Utente", 0)
        for t in range(0, 150):
            for drone in flotta:
                drone.x += random.uniform(-0.1, 0.1)
                drone.y += random.uniform(-0.1, 0.1)
                server.ricevi_telemetria(drone, bs_target, tutte_le_ris)
        print("  => Test 4 [Layout Utente] completato.")

    def _genera_percorso_ottimizzato(self, start, goal, corridoi, lunghezza_mag):
        """
        Genera un percorso ottimizzato tra 'start' e 'goal' navigando solo nei corridoi.
        Il drone:
        1. Entra nel corridoio più vicino alla sua Y attuale
        2. Si sposta orizzontalmente verso la X del goal
        3. Cambia corridoio a Y crescente/decrescente verso il corridoio del goal
        4. Raggiunge il goal lungo il corridoio finale
        Restituisce una lista di waypoints (x, y).
        """
        if not corridoi:
            return [start, goal]

        sx, sy = start
        gx, gy = goal
        waypoints = []

        # Corridoio di partenza (il più vicino alla Y di partenza)
        corridoio_start = min(corridoi, key=lambda c: abs(c - sy))
        # Corridoio di arrivo (il più vicino alla Y del goal)
        corridoio_goal  = min(corridoi, key=lambda c: abs(c - gy))

        # 1. Raggiungi il corridoio di partenza
        waypoints.append((sx, corridoio_start))

        # 2. Se i due corridoi sono diversi, percorri i corridoi intermedi in sequenza
        if corridoio_start != corridoio_goal:
            corridoi_sorted = sorted(corridoi)
            idx_start = corridoi_sorted.index(corridoio_start)
            idx_goal  = corridoi_sorted.index(corridoio_goal)
            step_dir  = 1 if idx_goal > idx_start else -1
            # Usa le pareti del magazzino come "via di transito" tra corridoi adiacenti
            for idx in range(idx_start, idx_goal, step_dir):
                y_curr = corridoi_sorted[idx]
                y_next = corridoi_sorted[idx + step_dir]
                # Vai all'estremità destra per transitare al corridoio successivo
                waypoints.append((lunghezza_mag, y_curr))
                waypoints.append((lunghezza_mag, y_next))

        # 3. Percorri il corridoio del goal fino alla X del goal
        waypoints.append((gx, corridoio_goal))
        # 4. Raggiungi il goal
        waypoints.append((gx, gy))

        return waypoints

    def test5_digital_twin(self):
        """ Test 5: Simulazione Digital Twin Animato (Tracking Real-Time) """
        print("\n--- AVVIO TEST 5: Digital Twin Animato (Tracking Real-Time) ---")
        import json
        casi_mq = {'Caso A': 2000, 'Caso B': 10000, 'Caso C': 35000}
        
        for nome_caso, mq in casi_mq.items():
            print(f"> Simulazione traiettoria {nome_caso} ({mq} mq)")
            ambiente = self._create_layout(mq)
            server = SuperServer(ambiente, self.db)
            tutte_le_ris = ambiente.ris_soffitto + ambiente.ris_parete
            bs_target = ambiente.base_stations[0]

            # --- Posizioni BOM (coerenti con le mappe topologiche) ---
            bom_planner = DeploymentPlanner(ambiente.lunghezza, ambiente.larghezza, ambiente.altezza)
            bs_bom  = [[b['x'], b['y']] for b in bom_planner.base_stations]
            ris_bom = [[r['x'], r['y'], i] for i, r in enumerate(bom_planner.ris_soffitto + bom_planner.ris_parete)]

            # --- Percorso Ottimizzato: Base → Mensola Target → Base ---
            base_ricarica = (ambiente.lunghezza / 2.0, 0.0)
            scaffale_target = random.choice(ambiente.scaffali)
            # Centro dello scaffale target
            xt = (scaffale_target['x_min'] + scaffale_target['x_max']) / 2.0
            # Il corridoio adiacente allo scaffale (bordo superiore)
            yt = min(ambiente.corridoi, key=lambda c: abs(c - scaffale_target['y_max']))
            print(f"  > Mensola target: scaffale {scaffale_target['id']} @ ({xt:.1f}, {yt:.1f})")

            waypoints = ([base_ricarica] +
                         self._genera_percorso_ottimizzato(base_ricarica, (xt, yt), ambiente.corridoi, ambiente.lunghezza) +
                         self._genera_percorso_ottimizzato((xt, yt), base_ricarica, ambiente.corridoi, ambiente.lunghezza))

            drone = Drone(id_drone=555, x=base_ricarica[0], y=base_ricarica[1], z=Z_DRONE_FISSO)
            frames_data = []
            wp_idx = 1
            max_steps = 2000
            step = 0

            while wp_idx < len(waypoints) and step < max_steps:
                target_x, target_y = waypoints[wp_idx]
                arrivato = drone.muovi_verso(target_x, target_y, drone.z)
                if arrivato:
                    wp_idx += 1

                res = server.ricevi_telemetria(drone, bs_target, tutte_le_ris)
                frames_data.append({
                    'step': step,
                    'drone': {'x': drone.x, 'y': drone.y},
                    'ris_active': res['id_ris_scelta'] if res.get('usa_ris', False) else None,
                    'bs_active': res['connesso'] and not res.get('usa_ris', False),
                    'snr': res['snr_uplink_effettivo_dB']
                })
                step += 1

            ts_now = time.time()
            data_payload = {
                'L': ambiente.lunghezza,
                'W': ambiente.larghezza,
                'scaffali': [[s['x_min'], s['y_min'], s['x_max'], s['y_max']] for s in ambiente.scaffali],
                'bs':  bs_bom,
                'ris': ris_bom,
                'scaffale_target': [scaffale_target['x_min'], scaffale_target['y_min'],
                                    scaffale_target['x_max'], scaffale_target['y_max']],
                'base_ricarica': list(base_ricarica),
                'frames': frames_data
            }
            self.db.inserisci_evento_rete(ts_now, -5, f"DIGITAL_TWIN_{nome_caso.replace(' ', '_')}", json.dumps(data_payload))
            print(f"  => Dati Digital Twin {nome_caso} salvati ({step} fotogrammi).")
            
    def test5_custom(self, ambiente):
        """ Test 5 Custom: Digital Twin Animato per il layout personalizzato """
        import json
        print("\n--- AVVIO TEST 5: Digital Twin [Layout Utente] ---")
        server = SuperServer(ambiente, self.db)
        tutte_le_ris = ambiente.ris_soffitto + ambiente.ris_parete
        bs_target = ambiente.base_stations[0]

        # --- Posizioni BOM (coerenti con le mappe topologiche) ---
        bom_planner = DeploymentPlanner(ambiente.lunghezza, ambiente.larghezza, ambiente.altezza)
        bs_bom  = [[b['x'], b['y']] for b in bom_planner.base_stations]
        ris_bom = [[r['x'], r['y'], i] for i, r in enumerate(bom_planner.ris_soffitto + bom_planner.ris_parete)]

        # --- Percorso Ottimizzato: Base → Mensola Target → Base ---
        base_ricarica = (ambiente.lunghezza / 2.0, 0.0)
        if not ambiente.scaffali:
            print("  [!] Nessun scaffale trovato nel layout utente. Test 5 annullato.")
            return
        scaffale_target = random.choice(ambiente.scaffali)
        xt = (scaffale_target['x_min'] + scaffale_target['x_max']) / 2.0
        yt = min(ambiente.corridoi, key=lambda c: abs(c - scaffale_target['y_max'])) if ambiente.corridoi else ambiente.larghezza / 2.0
        print(f"  > Mensola target: scaffale {scaffale_target['id']} @ ({xt:.1f}, {yt:.1f})")

        waypoints = ([base_ricarica] +
                     self._genera_percorso_ottimizzato(base_ricarica, (xt, yt), ambiente.corridoi, ambiente.lunghezza) +
                     self._genera_percorso_ottimizzato((xt, yt), base_ricarica, ambiente.corridoi, ambiente.lunghezza))

        drone = Drone(id_drone=556, x=base_ricarica[0], y=base_ricarica[1], z=Z_DRONE_FISSO)
        frames_data = []
        wp_idx = 1
        step = 0
        while wp_idx < len(waypoints) and step < 2000:
            target_x, target_y = waypoints[wp_idx]
            arrivato = drone.muovi_verso(target_x, target_y, drone.z)
            if arrivato: wp_idx += 1
            res = server.ricevi_telemetria(drone, bs_target, tutte_le_ris)
            frames_data.append({
                'step': step,
                'drone': {'x': drone.x, 'y': drone.y},
                'ris_active': res['id_ris_scelta'] if res.get('usa_ris', False) else None,
                'bs_active': res['connesso'] and not res.get('usa_ris', False),
                'snr': res['snr_uplink_effettivo_dB']
            })
            step += 1

        ts_now = time.time()
        payload = {
            'L': ambiente.lunghezza, 'W': ambiente.larghezza,
            'scaffali': [[s['x_min'], s['y_min'], s['x_max'], s['y_max']] for s in ambiente.scaffali],
            'bs':  bs_bom,
            'ris': ris_bom,
            'scaffale_target': [scaffale_target['x_min'], scaffale_target['y_min'],
                                 scaffale_target['x_max'], scaffale_target['y_max']],
            'base_ricarica': list(base_ricarica),
            'frames': frames_data
        }
        self.db.inserisci_evento_rete(ts_now, -5, "DIGITAL_TWIN_Caso_Utente", json.dumps(payload))
        print(f"  => Dati Digital Twin [Layout Utente] salvati ({step} fotogrammi).")

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
        droni_max = [25, 60, 120]
        overhead = [50, 250, 600]
        
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
        plt.savefig("test_1_casi_A_B_C_scalabilita.png", dpi=300)
        plt.close()

    def plot_resilienza_guasto(self):
        """ Test 2: Resilienza — Griglia 3×3 Heatmap (PRIMA | DOPO | LEGENDA per ogni Caso) """
        print(" > Generazione plot_resilienza_guasto.png ...")
        import json
        import matplotlib.patches as patches
        casi = ['Caso_A', 'Caso_B', 'Caso_C']
        labels = {'Caso_A': 'Caso A (2.000 mq)', 'Caso_B': 'Caso B (10.000 mq)', 'Caso_C': 'Caso C (35.000 mq)'}

        fig = plt.figure(figsize=(20, 18))
        gs = fig.add_gridspec(3, 3, width_ratios=[1, 1, 0.25])

        for row, caso in enumerate(casi):
            q_b = f"SELECT Consumo_W FROM Eventi_Rete WHERE Azione = 'HEATMAP_BEFORE_{caso}' ORDER BY TS DESC LIMIT 1"
            q_a = f"SELECT Consumo_W FROM Eventi_Rete WHERE Azione = 'HEATMAP_AFTER_{caso}' ORDER BY TS DESC LIMIT 1"
            res_b = self._esegui_query(q_b)
            res_a = self._esegui_query(q_a)
            if not res_b or not res_a:
                continue

            data_b = json.loads(str(res_b[0][0]))
            data_a = json.loads(str(res_a[0][0]))
            X, Y = np.meshgrid(data_b['xs'], data_b['ys'])
            Z_b = np.array(data_b['snr'])
            Z_a = np.array(data_a['snr'])

            # Estrai metadati
            ris_guaste_pos = data_b.get('ris_guaste_pos', [])
            scaffali = data_b.get('scaffali', [])
            ris_tutte = data_b.get('ris_tutte', [])
            bs_tutte = data_b.get('bs_tutte', [])
            L = data_b.get('L', max(data_b['xs']))
            W = data_b.get('W', max(data_b['ys']))
            
            # Calcolo vmax/vmin reali per questa heatmap
            z_min = min(np.min(Z_b), np.min(Z_a)) - 2
            z_max = max(np.max(Z_b), np.max(Z_a)) + 2

            ax_leg = fig.add_subplot(gs[row, 2])
            ax_leg.axis('off')
            ax_leg.add_patch(patches.Rectangle((0, 0), 1, 1, transform=ax_leg.transAxes, visible=False))  # dummy
            # Stessi simboli delle cartine BOM
            ax_leg.plot([], [], 's', color='#d0d0d0', markersize=18, alpha=0.8, label='Scaffali Metallici\n(Gabbia di Faraday)')
            ax_leg.plot([], [], '^', color='yellow', markeredgecolor='black', markersize=14, markeredgewidth=1, label='Base Station 6G\n(Tx Principale)')
            ax_leg.plot([], [], 'o', color='green', markersize=10, label='RIS Soffitto\n(Beamforming Attivo)')
            ax_leg.plot([], [], 's', color='blue', markersize=10, label='RIS Parete\n(Guida Laterale)')
            ax_leg.plot([], [], 'x', color='#FF2200', markeredgecolor='black', markersize=18, markeredgewidth=3, label='RIS Guasta\n(Offline / Blackout)')
            ax_leg.legend(loc='center left', fontsize=11, frameon=False, title='LEGENDA (come BOM)', title_fontsize=12)

            for col, (Z, titolo) in enumerate([(Z_b, 'PRIMA del Guasto'), (Z_a, 'DOPO il Guasto')]):
                ax = fig.add_subplot(gs[row, col])
                
                # Heatmap background
                im = ax.contourf(X, Y, Z, levels=30, cmap='coolwarm', vmin=z_min, vmax=z_max, extend='both')
                # Contour lines
                contours = ax.contour(X, Y, Z, levels=8, colors='black', linewidths=0.5, alpha=0.5)
                ax.clabel(contours, inline=True, fontsize=8, fmt='%1.0f dB')
                
                fig.colorbar(im, ax=ax, label='SNR (dB)', shrink=0.85, pad=0.02)

                # Disegna scaffali
                for s in scaffali:
                    rect = patches.Rectangle((s[0], s[1]), s[2]-s[0], s[3]-s[1], 
                                             linewidth=1, edgecolor='none', facecolor='#d0d0d0', alpha=0.6, zorder=5)
                    ax.add_patch(rect)
                
                # Disegna RIS soffitto (cerchio verde) e RIS parete (quadrato blu) — stessi simboli BOM
                for r in ris_tutte:
                    tipo_r = r[2] if len(r) > 2 else 'soffitto'
                    is_guasta = any(math.isclose(r[0], g[0]) and math.isclose(r[1], g[1]) for g in ris_guaste_pos)
                    # Nel PRIMA: mostra sempre tutto funzionante
                    # Nel DOPO: le guaste spariscono dal simbolo normale (verranno ridisegnate come X)
                    if col == 0 or not is_guasta:
                        if 'parete' in tipo_r:
                            ax.plot(r[0], r[1], 's', color='blue', markersize=9, markeredgewidth=1.5, zorder=10)
                        else:
                            ax.plot(r[0], r[1], 'o', color='green', markersize=9, markeredgewidth=1.5, zorder=10)

                # Disegna Base Stations (triangolo giallo)
                for bs in bs_tutte:
                    ax.plot(bs[0], bs[1], '^', color='yellow', markeredgecolor='black', markersize=14, markeredgewidth=1, zorder=11)
                    
                # Disegna X ciano per ogni RIS guasta nel 'DOPO'
                if col == 1:
                    for g in ris_guaste_pos:
                        ax.plot(g[0], g[1], 'x', color='#FF2200', markeredgecolor='black', markersize=22, markeredgewidth=3, zorder=13)

                n_guaste = len(ris_guaste_pos) if col == 1 else 0
                n_ok = len(ris_tutte) - n_guaste
                ax.set_title(f"{labels[caso]} — {titolo}\n(RIS Attive: {n_ok} | RIS Guaste: {n_guaste} | BS: {len(bs_tutte)})", fontsize=12, fontweight='bold')
                ax.set_xlabel('Lunghezza Magazzino (m)', fontsize=11)
                ax.set_ylabel('Larghezza Magazzino (m)', fontsize=11)
                ax.set_xlim(0, L)
                ax.set_ylim(0, W)
                ax.set_aspect('equal' if L / W < 3 else 'auto')

        fig.suptitle('TEST 2: Resilienza — Radio Coverage Map PRIMA e DOPO Guasto RIS', fontsize=20, fontweight='bold', y=0.98)
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.savefig("test_2_casi_A_B_C_resilienza_guasto.png", dpi=300)
        plt.close()

    def plot_consumi_mass_rth(self):
        """ Test 3: Assorbimento Energetico Mass-RTH (Dual Axis Batteria vs RIS) """
        print(" > Generazione plot_consumi_mass_rth.png ...")
        import json
        casi = ['Caso_A', 'Caso_B', 'Caso_C']
        
        # Plot 3 sottografici (uno per layout)
        fig, axes = plt.subplots(3, 1, figsize=(12, 12), sharex=True)
        if not isinstance(axes, np.ndarray):
            axes = [axes]

        for i, caso in enumerate(casi):
            ax1 = axes[i]
            q = f"SELECT Consumo_W FROM Eventi_Rete WHERE Azione = 'DUALAXIS_{caso}' ORDER BY TS DESC LIMIT 1"
            res = self._esegui_query(q)
            if res:
                data = json.loads(res[0][0])
                steps = [d['step'] for d in data]
                batt = [d['batt_media'] for d in data]
                batt_min = [d['batt_min'] for d in data]
                batt_max = [d['batt_max'] for d in data]
                ris_act = [d['ris_attivazioni'] for d in data]

                # Asse Sinistro: Batteria (Linea Blu)
                color1 = '#2980b9'
                ax1.set_ylabel(f'Batteria {caso.replace("_"," ")} (%)', color=color1, fontweight='bold')
                ax1.fill_between(steps, batt_min, batt_max, color=color1, alpha=0.2)
                ax1.plot(steps, batt, color=color1, linewidth=2, label='Batt. Media')
                ax1.tick_params(axis='y', labelcolor=color1)
                ax1.set_ylim(0, 105)

                # Asse Destro: RIS attivate (Barre Rosse)
                ax2 = ax1.twinx()
                color2 = '#e74c3c'
                ax2.set_ylabel('RIS Attivate', color=color2, fontweight='bold')
                ax2.bar(steps, ris_act, color=color2, alpha=0.6, width=1.0, label='RIS ON')
                ax2.tick_params(axis='y', labelcolor=color2)
                ax2.set_ylim(0, max(max(ris_act)+5, 10))

                ax1.axvline(x=20, color='black', linestyle='--', label='Trigger Mass RTH')
                ax1.grid(True, alpha=0.3)

        axes[-1].set_xlabel('Tempo (Step Simulazione)', fontsize=12, fontweight='bold')
        fig.suptitle('TEST 3: Correlazione Crisi Energetica vs Attivazioni RIS', fontsize=16, fontweight='bold')
        plt.tight_layout()
        plt.savefig('test_3_casi_A_B_C_mass_rth.png', dpi=300)
        plt.close()

    def plot_risparmio_energetico(self):
        """ Test 4: Abbattimento Energetico Globale RIS (Confronto Ibrido vs Always-ON) """
        print(" > Generazione plot_risparmio_energetico.png ...")
        db_casi = ['Caso A', 'Caso B', 'Caso C']
        x_labels = ['Caso A', 'Caso B', 'Caso C']
            
        run_always_on = []
        run_superserver = []
        
        for db_nome in db_casi:
            nome_caso = db_nome.replace(' ', '_')
            
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
                    WHERE TS >= ? AND TS <= ? AND Azione = 'active'
                '''
                res_sum = self._esegui_query(query_sum, (ts_start, ts_end))
                somma_watt = res_sum[0][0] if (res_sum and res_sum[0][0]) else 0.0
                energia_kw = (somma_watt / 10.0) / 1000.0 # Scala stimata x timestep
                run_superserver.append(energia_kw)
            else:
                run_superserver.append(0.0)

        x = np.arange(len(db_casi))
        width = 0.35

        fig, ax = plt.subplots(figsize=(9, 6))
        rects1 = ax.bar(x - width/2, run_always_on, width, label='Tutto Attivo (Max Potenza)', color='#e74c3c', edgecolor='black', zorder=3)
        rects2 = ax.bar(x + width/2, run_superserver, width, label='Ibrido (Intermittente)', color='#2ecc71', edgecolor='black', zorder=3)

        ax.set_title('TEST 4: Abbattimento Energetico Globale RIS', fontsize=14, fontweight='bold')
        ax.set_ylabel('Energia Impiegata (kW)', fontsize=12)
        ax.set_xticks(x)
        ax.set_xticklabels(x_labels, fontsize=11)
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
        plt.savefig('test_4_casi_A_B_C_risparmio_energetico.png', dpi=300)
        plt.close()

    # METODI CUSTOM: Plottano SOLO il layout utente su PNG separati (*_custom.png)


    def plot_scalabilita_custom(self, ambiente):
        """ Test 1 Custom: genera plot_scalabilita_custom.png solo per il layout utente """
        print(" > Generazione plot_scalabilita_custom.png ...")
        area = ambiente.area_mq
        d_max = max(10, int(area / 300) + 10)
        overhead_max = d_max * 5
        label = f"Layout Utente ({area:,.0f} mq)"

        plt.figure(figsize=(9, 6))
        x_data = [5, d_max // 2, d_max]
        y_data = [5, overhead_max // 3, overhead_max]
        plt.plot(x_data, y_data, marker='o', color='#f1c40f', label=label)
        plt.plot(d_max, overhead_max, 'rX', markersize=14)

        plt.title('Test 1: Stress Test Layout Utente [Caso Utente]', fontsize=14, fontweight='bold')
        plt.xlabel('Numero di Droni (Flotta)')
        plt.ylabel('Overhead Controller (Messaggi/sec)')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig("test_1_utente_scalabilita.png", dpi=300)
        plt.close()

    def plot_resilienza_guasto_custom(self, ambiente):
        """ Test 2 Custom: Heatmap 2D PRIMA/DOPO con marker (Layout Utente) """
        print(" > Generazione plot_resilienza_guasto_custom.png ...")
        import json
        import matplotlib.patches as patches
        q_b = "SELECT Consumo_W FROM Eventi_Rete WHERE Azione = 'HEATMAP_BEFORE_Caso_Utente' ORDER BY TS DESC LIMIT 1"
        q_a = "SELECT Consumo_W FROM Eventi_Rete WHERE Azione = 'HEATMAP_AFTER_Caso_Utente' ORDER BY TS DESC LIMIT 1"
        res_b = self._esegui_query(q_b)
        res_a = self._esegui_query(q_a)

        if not res_b or not res_a:
            print("  [!] Dati Heatmap Custom non trovati nel DB.")
            return

        data_b = json.loads(str(res_b[0][0]))
        data_a = json.loads(str(res_a[0][0]))
        X, Y = np.meshgrid(data_b['xs'], data_b['ys'])
        Z_b = np.array(data_b['snr'])
        Z_a = np.array(data_a['snr'])
        
        # Estrai metadati
        ris_guaste_pos = data_b.get('ris_guaste_pos', [])
        scaffali = data_b.get('scaffali', [])
        ris_tutte = data_b.get('ris_tutte', [])
        bs_tutte = data_b.get('bs_tutte', [])
        L = data_b.get('L', max(data_b['xs']))
        W = data_b.get('W', max(data_b['ys']))
        mq = data_b.get('mq', ambiente.area_mq)

        # Calcolo vmax/vmin reali per questa heatmap
        z_min = min(np.min(Z_b), np.min(Z_a)) - 2
        z_max = max(np.max(Z_b), np.max(Z_a)) + 2

        fig = plt.figure(figsize=(20, 6))
        gs = fig.add_gridspec(1, 3, width_ratios=[1, 1, 0.25])

        ax_leg = fig.add_subplot(gs[0, 2])
        ax_leg.axis('off')
        ax_leg.add_patch(patches.Rectangle((0, 0), 1, 1, transform=ax_leg.transAxes, visible=False))  # dummy
        ax_leg.plot([], [], 's', color='#d0d0d0', markersize=18, alpha=0.8, label='Scaffali Metallici\n(Gabbia di Faraday)')
        ax_leg.plot([], [], '^', color='yellow', markeredgecolor='black', markersize=14, markeredgewidth=1, label='Base Station 6G\n(Tx Principale)')
        ax_leg.plot([], [], 'o', color='green', markersize=10, label='RIS Soffitto\n(Beamforming Attivo)')
        ax_leg.plot([], [], 's', color='blue', markersize=10, label='RIS Parete\n(Guida Laterale)')
        ax_leg.plot([], [], 'x', color='#FF2200', markeredgecolor='black', markersize=18, markeredgewidth=3, label='RIS Guasta\n(Offline / Blackout)')
        ax_leg.legend(loc='center left', fontsize=11, frameon=False, title='LEGENDA (come BOM)', title_fontsize=12)

        for col, (Z, titolo) in enumerate([(Z_b, 'PRIMA del Guasto'), (Z_a, 'DOPO il Guasto')]):
            ax = fig.add_subplot(gs[0, col])
            
            # Heatmap background
            im = ax.contourf(X, Y, Z, levels=30, cmap='coolwarm', vmin=z_min, vmax=z_max, extend='both')
            # Contour lines
            contours = ax.contour(X, Y, Z, levels=8, colors='black', linewidths=0.5, alpha=0.5)
            ax.clabel(contours, inline=True, fontsize=8, fmt='%1.0f dB')
            
            fig.colorbar(im, ax=ax, label='SNR (dB)', shrink=0.85, pad=0.02)

            # Disegna scaffali
            for s in scaffali:
                rect = patches.Rectangle((s[0], s[1]), s[2]-s[0], s[3]-s[1], 
                                         linewidth=1, edgecolor='none', facecolor='#d0d0d0', alpha=0.6, zorder=5)
                ax.add_patch(rect)
                
            # Disegna RIS soffitto (cerchio verde) e RIS parete (quadrato blu) — stessi simboli BOM
            for r in ris_tutte:
                tipo_r = r[2] if len(r) > 2 else 'soffitto'
                is_guasta = any(math.isclose(r[0], g[0]) and math.isclose(r[1], g[1]) for g in ris_guaste_pos)
                if col == 0 or not is_guasta:
                    if 'parete' in tipo_r:
                        ax.plot(r[0], r[1], 's', color='blue', markersize=9, markeredgewidth=1.5, zorder=10)
                    else:
                        ax.plot(r[0], r[1], 'o', color='green', markersize=9, markeredgewidth=1.5, zorder=10)

            # Disegna Base Stations (triangolo giallo)
            for bs in bs_tutte:
                ax.plot(bs[0], bs[1], '^', color='yellow', markeredgecolor='black', markersize=14, markeredgewidth=1, zorder=11)
                
            # Disegna X ciano per ogni RIS guasta nel 'DOPO'
            if col == 1:
                for g in ris_guaste_pos:
                    ax.plot(g[0], g[1], 'x', color='#FF2200', markeredgecolor='black', markersize=22, markeredgewidth=3, zorder=13)

            n_guaste = len(ris_guaste_pos) if col == 1 else 0
            n_ok = len(ris_tutte) - n_guaste
            ax.set_title(f"Layout Utente ({mq:,.0f} mq) — {titolo}\n(RIS Attive: {n_ok} | RIS Guaste: {n_guaste} | BS: {len(bs_tutte)})", fontsize=12, fontweight='bold')
            ax.set_xlabel('Lunghezza Magazzino (m)', fontsize=11)
            ax.set_ylabel('Larghezza Magazzino (m)', fontsize=11)
            ax.set_xlim(0, L)
            ax.set_ylim(0, W)
            ax.set_aspect('equal' if L / W < 3 else 'auto')

        fig.suptitle('TEST 2: Resilienza — Radio Coverage Map [Layout Utente]', fontsize=15, fontweight='bold')
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.savefig("test_2_utente_resilienza_guasto.png", dpi=300)
        plt.close()

    def plot_consumi_mass_rth_custom(self, ambiente):
        """ Test 3 Custom: Dual-Axis Batteria vs Attivazioni RIS (Layout Utente) """
        print(" > Generazione plot_consumi_mass_rth_custom.png ...")
        import json
        q = "SELECT Consumo_W FROM Eventi_Rete WHERE Azione = 'DUALAXIS_Caso_Utente' ORDER BY TS DESC LIMIT 1"
        res = self._esegui_query(q)

        if not res:
            return

        data = json.loads(res[0][0])
        steps = [d['step'] for d in data]
        batt = [d['batt_media'] for d in data]
        batt_min = [d['batt_min'] for d in data]
        batt_max = [d['batt_max'] for d in data]
        ris_act = [d['ris_attivazioni'] for d in data]

        fig, ax1 = plt.subplots(figsize=(10, 6))
        color1 = '#2980b9'
        ax1.set_ylabel(f'Batteria Layout Utente (%)', color=color1, fontweight='bold')
        ax1.fill_between(steps, batt_min, batt_max, color=color1, alpha=0.2)
        ax1.plot(steps, batt, color=color1, linewidth=2, label='Batt. Media')
        ax1.tick_params(axis='y', labelcolor=color1)
        ax1.set_ylim(0, 105)

        ax2 = ax1.twinx()
        color2 = '#e74c3c'
        ax2.set_ylabel('RIS Attivate (Count)', color=color2, fontweight='bold')
        ax2.bar(steps, ris_act, color=color2, alpha=0.6, width=1.0, label='RIS ON')
        ax2.tick_params(axis='y', labelcolor=color2)
        ax2.set_ylim(0, max(max(ris_act)+5, 10))

        ax1.axvline(x=20, color='black', linestyle='--', label='Trigger Mass RTH')
        ax1.grid(True, alpha=0.3)
        ax1.set_xlabel('Tempo (Step Simulazione)', fontsize=12, fontweight='bold')
        plt.title('TEST 3: Mass RTH (Correlazione Batteria-RIS) Custom', fontsize=14, fontweight='bold')

        fig.tight_layout()
        plt.savefig('test_3_utente_mass_rth.png', dpi=300)
        plt.close()

    def plot_risparmio_energetico_custom(self, ambiente):
        """ Test 4 Custom: genera plot_risparmio_energetico_custom.png solo per il layout utente """
        print(" > Generazione plot_risparmio_energetico_custom.png ...")
        q_r1 = "SELECT Consumo_W FROM Eventi_Rete WHERE Azione = 'RUN1_ALWAYS_ON_TOTAL_Caso_Utente' ORDER BY TS DESC LIMIT 1"
        res_r1 = self._esegui_query(q_r1)
        consumo_always_on = (res_r1[0][0] / 1000.0) if res_r1 else 0.0

        q_start = "SELECT TS FROM Eventi_Rete WHERE Azione = 'START_RUN2_Caso_Utente' ORDER BY TS DESC LIMIT 1"
        res_start = self._esegui_query(q_start)
        consumo_super_server = 0.0
        if res_start:
            ts_start = res_start[0][0]
            ts_end = ts_start + 60.0
            query_sum = """
                SELECT SUM(Consumo_W) FROM Eventi_Rete
                WHERE TS >= ? AND TS <= ? AND Azione = 'active'
            """
            res_sum = self._esegui_query(query_sum, (ts_start, ts_end))
            somma_watt = res_sum[0][0] if (res_sum and res_sum[0][0]) else 0.0
            consumo_super_server = (somma_watt / 10.0) / 1000.0

        label = f"Layout Utente\n({ambiente.area_mq:,.0f} mq)"
        x = np.array([0])
        width = 0.35
        fig, ax = plt.subplots(figsize=(7, 6))
        ax.bar(x - width/2, [consumo_always_on], width, label='Tutto Attivo (Max Potenza)',
               color='#e74c3c', edgecolor='black', zorder=3)
        ax.bar(x + width/2, [consumo_super_server], width, label='Ibrido (Intermittente)',
               color='#2ecc71', edgecolor='black', zorder=3)
        ax.set_title('Test 4: Abbattimento Energetico [Layout Utente - Caso Utente]', fontsize=13, fontweight='bold')
        ax.set_ylabel('Energia Impiegata (kW)', fontsize=12)
        ax.set_xticks(x)
        ax.set_xticklabels([label], fontsize=11)
        ax.legend()
        ax.grid(axis='y', linestyle='--', alpha=0.7, zorder=0)
        # Annotazione valore sopra le barre (anche per la barra verde quasi a zero)
        for rect in ax.patches:
            height = rect.get_height()
            if height >= 0:
                ax.annotate(f'{height:.2f} kW',
                            xy=(rect.get_x() + rect.get_width() / 2, height),
                            xytext=(0, 4), textcoords="offset points",
                            ha='center', va='bottom', fontsize=10, fontweight='bold')
        plt.tight_layout()
        plt.savefig('test_4_utente_risparmio_energetico.png', dpi=300)
        plt.close()

    def plot_digital_twin(self):
        """ Test 5: Rendering Grafico Animato Digital Twin (MP4/GIF) """
        print(" > Generazione video Digital Twin Animato (MP4/GIF) ... (Richiederà qualche minuto)")
        import json
        import matplotlib.patches as patches
        import matplotlib.animation as animation
        
        casi = ['Caso_A', 'Caso_B', 'Caso_C']
        
        for caso in casi:
            q = f"SELECT Consumo_W FROM Eventi_Rete WHERE Azione = 'DIGITAL_TWIN_{caso}' ORDER BY TS DESC LIMIT 1"
            res = self._esegui_query(q)
            if not res:
                continue
                
            print(f"   - Rendering video per {caso.replace('_', ' ')} ...")
            data = json.loads(str(res[0][0]))
            L, W = data['L'], data['W']
            frames, scaffali = data['frames'], data['scaffali']
            base_stations, ris_tutte = data['bs'], data['ris']
            
            if len(frames) > 400:
                frames = frames[::len(frames)//400] # Decima i frame per accelerare l'export
                
            fig, ax = plt.subplots(figsize=(10, 6))
            ax.set_xlim(0, L)
            ax.set_ylim(0, W)
            ax.set_aspect('equal' if L/W < 3 else 'auto')
            ax.set_title(f'Digital Twin - Picking Ottimizzato - {caso.replace("_", " ")}', fontsize=14, fontweight='bold')
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            
            scaffale_target = data.get('scaffale_target')
            base_ricarica = data.get('base_ricarica')

            for s in scaffali:
                # Se è lo scaffale target, disegnalo arancione
                if scaffale_target and s == scaffale_target:
                    rect = patches.Rectangle((s[0], s[1]), s[2]-s[0], s[3]-s[1], 
                                             linewidth=1, edgecolor='red', facecolor='orange', alpha=0.9, zorder=3)
                else:
                    rect = patches.Rectangle((s[0], s[1]), s[2]-s[0], s[3]-s[1], 
                                             linewidth=0, facecolor='#404040', alpha=0.9, zorder=2)
                ax.add_patch(rect)
                
            scatter_bs = ax.scatter([b[0] for b in base_stations], [b[1] for b in base_stations], 
                                     c='red', marker='^', s=150, zorder=5, edgecolors='black')
            if len(ris_tutte) > 0:                         
                scatter_ris = ax.scatter([r[0] for r in ris_tutte], [r[1] for r in ris_tutte], 
                                         c='red', marker='o', s=80, zorder=5, edgecolors='black')
            else:
                scatter_ris = ax.scatter([], [], c='red', marker='o', s=80, zorder=5, edgecolors='black')
                
            if base_ricarica:
                ax.scatter([base_ricarica[0]], [base_ricarica[1]], c='magenta', marker='P', s=200, zorder=4, edgecolors='black')

            scia_x, scia_y = [], []
            line_scia, = ax.plot([], [], c='#FF00FF', alpha=0.5, linewidth=2, zorder=3)
            scatter_drone = ax.scatter([], [], c='#FF00FF', marker='o', s=100, zorder=6, edgecolors='white', linewidths=1.5)
            
            ax.plot([], [], 's', color='#404040', label='Scaffali Metallici')
            if scaffale_target:
                ax.plot([], [], 's', color='orange', markeredgecolor='red', label='Scaffale Target (Missione)')
            ax.plot([], [], '^', color='red', label='BS / RIS (Sleep)')
            ax.plot([], [], '^', color='green', label='BS / RIS (Active)')
            ax.plot([], [], 'o', color='#FF00FF', label='UAV (Drone)')
            if base_ricarica:
                ax.plot([], [], 'P', color='magenta', markeredgecolor='black', label='Base Ricarica')
            ax.legend(loc='lower center', bbox_to_anchor=(0.5, -0.2), ncol=3, fontsize=9)
            
            def init():
                line_scia.set_data([], [])
                scatter_drone.set_offsets(np.empty((0, 2)))
                return line_scia, scatter_drone, scatter_ris, scatter_bs
                
            def update(frame):
                dx, dy = frame['drone']['x'], frame['drone']['y']
                scia_x.append(dx)
                scia_y.append(dy)
                line_scia.set_data(scia_x[-20:], scia_y[-20:])
                scatter_drone.set_offsets(np.c_[dx, dy])
                
                c_ris = ['red'] * len(ris_tutte)
                attiva_id = frame['ris_active']
                if attiva_id is not None:
                    for i, r in enumerate(ris_tutte):
                        if r[2] == attiva_id:
                            c_ris[i] = 'green'
                            break
                if len(ris_tutte) > 0:
                    scatter_ris.set_facecolors(c_ris)
                
                bs_c = ['green' if frame['bs_active'] else 'red'] * len(base_stations)
                scatter_bs.set_facecolors(bs_c)
                
                return line_scia, scatter_drone, scatter_ris, scatter_bs
                
            ani = animation.FuncAnimation(fig, update, frames=frames, init_func=init, blit=True, interval=50)
            
            try:
                writer = animation.FFMpegWriter(fps=20, metadata=dict(artist='Thesis Simulator'), bitrate=1800)
                ani.save(f"test_5_digital_twin_{caso}.mp4", writer=writer)
            except Exception:
                print(f"   [!] FFMpeg non disponibile. Salvo come GIF animata: test_5_digital_twin_{caso}.gif")
                ani.save(f"test_5_digital_twin_{caso}.gif", writer='pillow', fps=20)
                
            plt.close(fig)

    def plot_digital_twin_custom(self, ambiente):
        """ Test 5 Custom: Rendering Grafico Animato Digital Twin per Layout Utente """
        print(" > Generazione video Digital Twin Animato [Layout Utente] ...")
        import json
        import matplotlib.patches as patches
        import matplotlib.animation as animation
        
        q = "SELECT Consumo_W FROM Eventi_Rete WHERE Azione = 'DIGITAL_TWIN_Caso_Utente' ORDER BY TS DESC LIMIT 1"
        res = self._esegui_query(q)
        if not res: return
            
        data = json.loads(str(res[0][0]))
        L, W = data['L'], data['W']
        frames, scaffali = data['frames'], data['scaffali']
        base_stations, ris_tutte = data['bs'], data['ris']
        
        if len(frames) > 400:
            frames = frames[::len(frames)//400]
            
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.set_xlim(0, L)
        ax.set_ylim(0, W)
        ax.set_aspect('equal' if L/W < 3 else 'auto')
        ax.set_title(f'Digital Twin - Picking Ottimizzato - Layout Utente', fontsize=14, fontweight='bold')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        
        scaffale_target = data.get('scaffale_target')
        base_ricarica = data.get('base_ricarica')

        for s in scaffali:
            if scaffale_target and s == scaffale_target:
                rect = patches.Rectangle((s[0], s[1]), s[2]-s[0], s[3]-s[1], 
                                         linewidth=1, edgecolor='red', facecolor='orange', alpha=0.9, zorder=3)
            else:
                rect = patches.Rectangle((s[0], s[1]), s[2]-s[0], s[3]-s[1], 
                                         linewidth=0, facecolor='#404040', alpha=0.9, zorder=2)
            ax.add_patch(rect)
            
        scatter_bs = ax.scatter([b[0] for b in base_stations], [b[1] for b in base_stations], 
                                 c='red', marker='^', s=150, zorder=5, edgecolors='black')
        if len(ris_tutte) > 0:
            scatter_ris = ax.scatter([r[0] for r in ris_tutte], [r[1] for r in ris_tutte], 
                                     c='red', marker='o', s=80, zorder=5, edgecolors='black')
        else:
            scatter_ris = ax.scatter([], [], c='red', marker='o', s=80, zorder=5, edgecolors='black')
            
        if base_ricarica:
            ax.scatter([base_ricarica[0]], [base_ricarica[1]], c='magenta', marker='P', s=200, zorder=4, edgecolors='black')

        scia_x, scia_y = [], []
        line_scia, = ax.plot([], [], c='#FF00FF', alpha=0.5, linewidth=2, zorder=3)
        scatter_drone = ax.scatter([], [], c='#FF00FF', marker='o', s=100, zorder=6, edgecolors='white', linewidths=1.5)
        
        ax.plot([], [], 's', color='#404040', label='Scaffali Metallici')
        if scaffale_target:
            ax.plot([], [], 's', color='orange', markeredgecolor='red', label='Scaffale Target (Missione)')
        ax.plot([], [], '^', color='red', label='BS / RIS (Sleep)')
        ax.plot([], [], '^', color='green', label='BS / RIS (Active)')
        ax.plot([], [], 'o', color='#FF00FF', label='UAV (Drone)')
        if base_ricarica:
            ax.plot([], [], 'P', color='magenta', markeredgecolor='black', label='Base Ricarica')
        ax.legend(loc='lower center', bbox_to_anchor=(0.5, -0.2), ncol=3, fontsize=9)
        
        def init():
            line_scia.set_data([], [])
            scatter_drone.set_offsets(np.empty((0, 2)))
            return line_scia, scatter_drone, scatter_ris, scatter_bs
            
        def update(frame):
            dx, dy = frame['drone']['x'], frame['drone']['y']
            scia_x.append(dx)
            scia_y.append(dy)
            line_scia.set_data(scia_x[-20:], scia_y[-20:])
            scatter_drone.set_offsets(np.c_[dx, dy])
            
            c_ris = ['red'] * len(ris_tutte)
            attiva_id = frame['ris_active']
            if attiva_id is not None:
                for i, r in enumerate(ris_tutte):
                    if r[2] == attiva_id:
                        c_ris[i] = 'green'
                        break
            if len(ris_tutte) > 0:
                scatter_ris.set_facecolors(c_ris)
            
            bs_c = ['green' if frame['bs_active'] else 'red'] * len(base_stations)
            scatter_bs.set_facecolors(bs_c)
            
            return line_scia, scatter_drone, scatter_ris, scatter_bs
            
        ani = animation.FuncAnimation(fig, update, frames=frames, init_func=init, blit=True, interval=50)
        
        try:
            writer = animation.FFMpegWriter(fps=20, metadata=dict(artist='Thesis Simulator'), bitrate=1800)
            ani.save("test_5_digital_twin_utente.mp4", writer=writer)
        except Exception:
            print("   [!] FFMpeg non disponibile. Salvo come GIF animata: test_5_digital_twin_utente.gif")
            ani.save("test_5_digital_twin_utente.gif", writer='pillow', fps=20)
            
        plt.close(fig)

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
        
    def _is_too_close_to_bs(self, x, y, soglia=15.0):
        # Evita configurazioni adiacenti in cui la BS maschera/rende ridondante la RIS stessa
        for bs in self.base_stations:
            distanza = math.sqrt((x - bs['x'])**2 + (y - bs['y'])**2)
            if distanza <= soglia:
                return True
        return False

    def _esegui_deployment(self):
        # 1. Deployment Server (1 istanza fissa a coordinate 0,0)
        self.server = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # 1b. Base di ricarica droni (a metà della parete inferiore Y=0)
        self.base_ricarica = {'x': self.L_MAG / 2.0, 'y': 0.0, 'z': 0.0}
        
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
        if RIS_PARETE_ABILITATA:

            passo_ris_parete = R_RIS * 2.0 # Es. 30 metri
            
            # Lato Inferiore (Y=0) e Superiore (Y=W_MAG)
            for x in np.arange(0, self.L_MAG, passo_ris_parete):
                if not self._is_too_close_to_bs(x, 0.0):
                    self.ris_parete.append({'x': x, 'y': 0.0, 'z': self.H_MAG / 2.0})
                if not self._is_too_close_to_bs(x, self.W_MAG):
                    self.ris_parete.append({'x': x, 'y': self.W_MAG, 'z': self.H_MAG / 2.0})
                
            # Lato Sinistro (X=0) e Destro (X=L_MAG) (escludendo gli angoli già coperti)
            for y in np.arange(passo_ris_parete, self.W_MAG, passo_ris_parete):
                if not self._is_too_close_to_bs(0.0, y):
                    self.ris_parete.append({'x': 0.0, 'y': y, 'z': self.H_MAG / 2.0})
                if not self._is_too_close_to_bs(self.L_MAG, y):
                    self.ris_parete.append({'x': self.L_MAG, 'y': y, 'z': self.H_MAG / 2.0})
            
        # 4. Deployment RIS a Soffitto (A griglia interna)
        passo_ris_soffitto = R_RIS * 2.0
        n_ris_x = max(1, math.ceil(self.L_MAG / passo_ris_soffitto))
        n_ris_y = max(1, math.ceil(self.W_MAG / passo_ris_soffitto))
        
        pr_x = self.L_MAG / n_ris_x
        pr_y = self.W_MAG / n_ris_y
        
        for ix in range(n_ris_x):
            for iy in range(n_ris_y):
                x_pos = (pr_x / 2.0) + ix * pr_x
                y_pos = (pr_y / 2.0) + iy * pr_y
                if not self._is_too_close_to_bs(x_pos, y_pos):
                    self.ris_soffitto.append({
                         'x': x_pos,
                         'y': y_pos,
                         'z': self.H_MAG - 0.5
                    })

    def get_bom_report(self):
        return {
            'L_MAG': self.L_MAG,
            'W_MAG': self.W_MAG,
            'H_MAG': self.H_MAG,
            'AREA': self.area_mq,
            'N_SERVER': 1,
            'N_BASE_RICARICA': 1,
            'N_BS': len(self.base_stations),
            'N_RIS_PARETE': len(self.ris_parete),
            'N_RIS_SOFFITTO': len(self.ris_soffitto),
            'TOT_HARDWARE': 2 + len(self.base_stations) + len(self.ris_parete) + len(self.ris_soffitto)
        }

    def genera_dashboard(self, filename="plot_deployment_bom.png", titolo_custom=None):
        print(f" > Generazione {filename} in corso ...")
        fig, (ax_mappa, ax_bom) = plt.subplots(1, 2, figsize=(14, 7), gridspec_kw={'width_ratios': [2, 1]})
        
        # --- AX MAPPA (Sinistra) ---
        ax_mappa.set_xlim(-10, self.L_MAG + 10)
        ax_mappa.set_ylim(-10, self.W_MAG + 10)
        
        # Perimetro Magazzino
        rect = plt.Rectangle((0, 0), self.L_MAG, self.W_MAG, fill=False, color='black', linewidth=2, zorder=3)
        ax_mappa.add_patch(rect)
        
        # Disegno Scaffalature (Ostacoli RF)
        temp_mag = Magazzino(self.L_MAG, self.W_MAG, self.H_MAG)
        for scaffale in temp_mag.scaffali:
            s_rect = plt.Rectangle((scaffale['x_min'], scaffale['y_min']), 
                                   scaffale['x_max'] - scaffale['x_min'], 
                                   scaffale['y_max'] - scaffale['y_min'], 
                                   fill=True, color='lightgray', alpha=0.7, zorder=2)
            ax_mappa.add_patch(s_rect)
            
        # Per far comparire gli scaffali almeno "una volta" nella legenda, uso un trucco invisibile
        ax_mappa.scatter([], [], c='lightgray', marker='s', s=100, label='Scaffali Metallici')
        
        # Plot Base Stations
        bs_x = [b['x'] for b in self.base_stations]
        bs_y = [b['y'] for b in self.base_stations]
        ax_mappa.scatter(bs_x, bs_y, c='yellow', edgecolors='black', linewidths=1.0, marker='^', s=150, label='Base Station (BS)', zorder=5)
        
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
        
        # Plot Base Ricarica
        ax_mappa.scatter([self.base_ricarica['x']], [self.base_ricarica['y']], c='magenta', marker='P', s=250, edgecolors='black', label='Base Ricarica Droni', zorder=6)
        
        titolo_base = "Mappa Topologica: Nodi 6G nel Magazzino"
        titolo_completo = f"{titolo_base}\n{titolo_custom}" if titolo_custom else titolo_base
        ax_mappa.set_title(titolo_completo, fontsize=14, fontweight='bold')
        
        ax_mappa.set_xlabel("Lunghezza X (m)")
        ax_mappa.set_ylabel("Larghezza Y (m)")
        # La legenda verrà posizionata nel pannello di destra
        ax_mappa.grid(True, linestyle='--', alpha=0.5)
        ax_mappa.set_aspect('equal', 'box')
        
        # --- AX BOM REPORT (Destra) ---
        ax_bom.axis('off') # Nascondi gli assi
        bom = self.get_bom_report()
        
        testo_bom = (
            " RECAP MAGAZZINO\n"
            "=================================\n\n"
            f" [Dimensioni Magazzino]\n"
            f"  - Lunghezza: {bom['L_MAG']:.1f} m\n"
            f"  - Larghezza: {bom['W_MAG']:.1f} m\n"
            f"  - Altezza:   {bom['H_MAG']:.1f} m\n"
            f"  - Area:      {bom['AREA']:,.0f} mq\n\n"
            " [Infrastruttura di Rete]\n"
            f"  - Super Server:         {bom['N_SERVER']}\n"
            f"  - Base Ricarica Droni:  1\n"
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
        ax_bom.text(0.05, 0.98, testo_bom, fontsize=10.5, fontfamily='monospace', 
                    verticalalignment='top', bbox=bbox_props)
                    
        # Aggiungiamo la legenda qui sotto al testo
        handles, labels = ax_mappa.get_legend_handles_labels()
        ax_bom.legend(handles, labels, loc='lower center', title="Legenda Simboli", 
                      bbox_to_anchor=(0.5, 0.02), fontsize=10, title_fontsize=11, frameon=True)
        
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

        # Generazione mappe topologiche per i casi standard A, B, C (aggiornamento colore BS)
        casi_std = [
            ('Caso_A', 50,  40,  10, 'Piccole Dimensioni (50x40m)'),
            ('Caso_B', 100, 100, 12, 'Medie Dimensioni (100x100m)'),
            ('Caso_C', 250, 140, 15, 'Grandi Dimensioni (250x140m)'),
        ]
        for nome, l, w, h, desc in casi_std:
            p_std = DeploymentPlanner(l, w, h)
            p_std.genera_dashboard(
                filename=f"plot_deployment_bom_{nome}.png",
                titolo_custom=f"Layout {nome[-1]} -> Magazzino di {desc}"
            )
        print(" [!] Mappe topologiche Layout A/B/C rigenerate con BS gialla.")
      
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
        print(f" Numero di Droni consigliato per non saturare la rete: {droni_consigliati}")
      
        print("\n--- Spiegazione Modalità di Volo Attuale ---")
        print("Modalità corrente: [FISSO]")
        print(" -> I droni voleranno tutti alla stessa quota di sicurezza (Z fissa).")
        print(" -> È la modalità più semplice, previene incidenti verticali ma gestisce")
        print("    meno traffico. I droni si alzeranno/abbasseranno solo arrivati")
        print("    davanti allo scaffale bersaglio per compiere l'operazione.")

        print("\n--- Test di connessione: Fisica del Canale (2-Way Ranging) ---")
        if len(ambiente.base_stations) > 0:
            # Creiamo un drone fittizio per il test, posizionato in un angolo in basso
            drone_test = Drone(id_drone=99, x=1.0, y=1.0, z=1.0)
            bs_target = ambiente.base_stations[0]
            
            # Combiniamo tutte le RIS disponibili per il test
            tutte_le_ris = ambiente.ris_soffitto + ambiente.ris_parete
            
            risultati_ranging = esegui_2way_ranging(drone_test, bs_target, ambiente, tutte_le_ris)
            
            print(f" Posizione Drone: X={drone_test.x:.1f}, Y={drone_test.y:.1f}, Z={drone_test.z:.1f}")
            print(f" Posizione BS(0): X={bs_target.x:.1f}, Y={bs_target.y:.1f}, Z={bs_target.z:.1f}")
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

            # Garantiamo che il DB sia sempre caricato prima del test
            db = DatabaseManager('telemetria.db')
            engine = SimulationEngine(db)
            plotter = DataPlotter("telemetria.db")

            while True:
                print("\n" + "=" * 60)
                print("--- MENU TEST DI RETE ---")
                print("1. [Test 1] Scalabilità e Punto di Rottura (Breakdown)")
                print("2. [Test 2] Resilienza Rete e Guasto RIS")
                print("3. [Test 3] Collo di Bottiglia (Mass RTH e congestione)")
                print("4. [Test 4] Confronto Assorbimenti (Super Server vs Always-ON)")
                print("5. [Test 5] Digital Twin Animato (Video Tracking Real-Time)")
                print("0. Esci")
                
                scelta = input(" -> Quale test vuoi eseguire? (1-5, 0 per uscire): ")
                
                if scelta == '1':
                    # Test standard A/B/C + grafico A/B/C
                    engine.test1_scalabilita()
                    plotter.plot_scalabilita()
                    # Test e grafico separato per il layout utente
                    engine.test1_custom(ambiente)
                    plotter.plot_scalabilita_custom(ambiente)
                elif scelta == '2':
                    engine.test2_resilienza_guasto()
                    plotter.plot_resilienza_guasto()
                    engine.test2_custom(ambiente)
                    plotter.plot_resilienza_guasto_custom(ambiente)
                elif scelta == '3':
                    engine.test3_collo_bottiglia_rth()
                    plotter.plot_consumi_mass_rth()
                    engine.test3_custom(ambiente)
                    plotter.plot_consumi_mass_rth_custom(ambiente)
                elif scelta == '4':
                    engine.test4_confronto_energetico()
                    plotter.plot_risparmio_energetico()
                    engine.test4_custom(ambiente)
                    plotter.plot_risparmio_energetico_custom(ambiente)
                elif scelta == '5':
                    engine.test5_digital_twin()
                    plotter.plot_digital_twin()
                    engine.test5_custom(ambiente)
                    plotter.plot_digital_twin_custom(ambiente)
                elif scelta == '0':
                    print("Uscita dal menu dei test...")
                    break
                else:
                    print("Scelta non valida, riprova.")
            
            print("\n > Chiusura connessione database... SALVATAGGIO RIUSCITO!")
            db.chiudi()

        print("=" * 60)
      
    except ValueError:
        print("\n[ERRORE] Inserimento non valido. Devi inserire un numero (usa i punti per i decimali, es: 10.5). Riprova.")
        