"""
Modulo 6: SDN Controller - Optimization & Placement
Modulo responsabile per il posizionamento ottimo delle RIS e il Control Plane della rete 6G.

Questo modulo rappresenta l'intelligenza di rete (SDN - Software Defined Networking).
Separa rigorosamente il piano dati dal piano di controllo, implementando algoritmi di:
1. Machine Learning Multi-obiettivo (K-Means + logica Greedy) per il deployment delle antenne.
2. Risparmio energetico (Green 6G) modulando dinamicamente la potenza rf emessa.
3. Intelligenza predittiva (Handover Make-before-break) estraendo le coordinate future fornite dall'EKF.
"""

import numpy as np
from typing import List, Tuple, Dict, Any
from sklearn.cluster import KMeans
from dataclasses import dataclass

# Importiamo le specifiche hardware e di Layout dal Modulo 1 (config2).
# Questo ci assicura di avere una "Single Source of Truth", quindi se cambiamo il peso 
from simulator.config2 import LayoutConfig, RISSpecs, UAVSpecs

@dataclass
class RISState:
    """
    Oggetto Dati per rappresentare in RAM lo stato operativo centrale di una RIS (Reconfigurable Intelligent Surface).
    Usa il modulo 'dataclass' per automatizzare il costruttore __init__ della classe e le stampe.
    
    Attributi:
        id (int): Identificativo univoco della superficie intelligente nell'archivio del Controller.
        position (Tuple[float, float, float]): Coordinate 3D in metri (X, Y, Z) all'interno del Digital Twin.
        is_active (bool): Flag che indica se il pannello è ACCESO (50W) oppure in SLEEP (0.5W).
        attached_uavs (List[int]): Lista di vettori che contiene gli ID numerici dei droni (UAV) 
                                 attualmente legati al cono RF di questa singola antenna.
    """
    id: int
    position: Tuple[float, float, float]
    is_active: bool
    attached_uavs: List[int]


class SDNController:
    """
    Controller SDN (Software-Defined Networking) per la gestione centralizzata della Rete 6G.
    
    In un'architettura 6G SBA (Service Based Architecture), l'SDN è il "Cervello" che osserva
    dall'alto tutti i link RF e la posizione matematica di antenne e terminali. Calcola i colli di bottiglia
    grazie agli array ad alte prestazioni di Numpy, ed esegue algoritmi per posizionare le antenne (fase Test 0)
    e gestire gli sleep per evitare di sprecare energia.
    """
    def __init__(self, layout: LayoutConfig, ris_specs: RISSpecs):
        """
        Inizializzatore (Costruttore) del Controller SDN.
        
        Args:
            layout (Layout): Oggetto contenente le misure X, Y, Z del capannone logistico corrente.
            ris_specs (RISSpecs): Specifica tecnica delle RIS passata da config2 (guadagno, rumore, voltaggi).
        """
        # Salva un riferimento all'oggetto Layout per poterne consultare i confini (es muri: X_max, Y_max)
        self.layout = layout 
        # Salva le specifiche delle antenne (per logiche fisiche o consumi se servisse esporle qua)
        self.ris_specs = ris_specs 
        
        # Dizionario di tipo 'hash-map' che memorizzerà il record di sensori RIS piazzati fisicamente
        # Mappa [ID_ANTENNA : ISTANZA_RISSTATE_CON_POSIZIONE]
        self.ris_nodes: Dict[int, RISState] = {}
        
        # Un semplice contatore per generare ID validi univoci a ogni nuovo inserimento d'antenna
        self.ris_counter = 0

    def deploy_ris_kmeans_greedy(self, nlos_points: np.ndarray, num_ris_available: int) -> List[Tuple[float, float, float]]:
        """
        [STEP 6.2 - Utilizzato per il TEST 0]
        Algoritmo di Intelligenza Artificiale ibrido per piazzare al centimetro perfetto
        le antenne sui muri del capannone e ridurre i punti neri radio al 1%.
        
        Combina un approccio Non-Supervisionato (K-Means) per capire matematicamente "dove si addensa di più l'ombra radio"
        tra gli scaffali. Una volta trovati i 'centri gravitazionali' di questi problemi, usa un 
        algoritmo 'Greedy' (ingordo) che aggancia rudemente l'antenna al muro più vicino, altrimenti 
        il cluster galleggerebbe a mezz'aria.
        
        Args:
            nlos_points (np.ndarray): Matrice Numpy [N, 3] di coordinate (X, Y, Z) dove l'algoritmo di Ray-Casting 
                                      ha riscontrato "Ostacolo fatale" in Test 0.
            num_ris_available (int): Budget imposto dall'operatore. Quante RIS hai comprato?
            
        Returns:
            List[Tuple[float, float, float]]: Lista di coordinate da dare ai tecnici virtuali per l'ancoraggio (X,Y,Z).
        """
        # Controllo di sicurezza base: Se il capannone è già perfetto e in Linea-Di-Vista assoluta (improbabile)
        if len(nlos_points) == 0:
            return []

        # --- FASE 1: MACHINE LEARNING (CLUSTERING) ---
        # Creiamo un limite: non possiamo creare più cluster di quante RIS fisiche possediamo nel budget.
        n_clusters = min(num_ris_available, len(nlos_points))
        
        # Inizializziamo il modello ML K-Means nativo. 
        # Random_state=42 garantisce risultati scientifici Ripetibili (non stocastici random ad ogni run) necessari per la Tesi.
        kmeans = KMeans(n_clusters=n_clusters, random_state=42, n_init="auto")
        
        # 'Fit' scatena il calcolo dei gradienti matematici. KMeans analizzerà le migliaia di punti 3D ciechi e sposterà
        # i suoi pesi finché non creerà 'n_clusters' nuvole perfette che abbracciano l'intera criticità.
        kmeans.fit(nlos_points)
        
        # Recuperiamo il cuore di questo ML: I centroidi! Ovvero le coordinate precise [X,Y,Z] in mezzo agli scaffali.
        centroids = kmeans.cluster_centers_

        # --- FASE 2: GREEDY PROJECTION (ANCORAGGIO FISICO) ---
        deployed_positions = []
        for c in centroids:
            
            # c[0] è X_centroide, c[1] è Y_centroide
            # Calcoliamo matematicamente l'ortogonale fino al perimetro estremo analizzando i 4 muri
            # Muro di Sinistra (Asse X = 0)
            dist_left = c[0] - 0
            # Muro di Destra (Asse X = Limite massimo capannone recuperato dall'oggetto layout)
            dist_right = self.layout.x_dim_m - c[0]
            # Muro Frontale (Asse Y = 0)
            dist_front = c[1] - 0
            # Muro Posteriore (Asse Y = Limite Y capannone)
            dist_back = self.layout.y_dim_m - c[1]
            
            # Creiamo un piccolo vettore delle 4 distanze. "argmin" estrarrà l'INDICE (0,1,2,3) del muro che garantisce la minor distanza matematica.
            dists = [dist_left, dist_right, dist_front, dist_back]
            min_idx = np.argmin(dists)
            
            # Resettiamo nuove variabili prelevando prima le vecchie dai centroidi convertendo il tipo da numpy.float a built-in Python float (più leggeri e sicuri col gRPC)
            new_x, new_y = float(c[0]), float(c[1])
            
            # Fissiamo empiricamente (Good Practice 3GPP) l'altezza al muro in alto: 80% dell'altezza del volume logistico
            wall_z = self.layout.z_dim_m * 0.8 
            
            # Spostamento manuale verso il vettore più rapido. Si cambia un solo asse (il muro agganciato), l'altro riflette il centroide in profondità.
            if min_idx == 0:
                new_x = 0.0 # Proiettata e fissata a sinistra
            elif min_idx == 1:
                new_x = self.layout.x_dim_m # Fissata a destra
            elif min_idx == 2:
                new_y = 0.0 # Fissata sul frontale
            elif min_idx == 3:
                new_y = self.layout.y_dim_m # Fissata sul posteriore
                
            deployed_pos = (new_x, new_y, float(wall_z))
            deployed_positions.append(deployed_pos)
            
            # 3. Registrazione SDN: ora che la matematica dell'installazione è nota, generiamo la struttura logica nel database SDN
            # Ogni antenna appena piazzata nasce di base "SVEGLIA" (is_active=True).
            self.ris_nodes[self.ris_counter] = RISState(id=self.ris_counter, position=deployed_pos, is_active=True, attached_uavs=[])
            
            # Incrementiamo il puntatore base di 1 per garantire ID primari sequenziali alla prossima passata del loop.
            self.ris_counter += 1

        return deployed_positions

    def run_green_6g_engine(self, uav_positions: Dict[int, Tuple[float, float, float]], threshold_dist: float = 30.0):
        """
        [STEP 6.3 - Utilizzato nei TEST 3]
        Ottimizzatore Energetico (Green 6G).
        È una macro-routine richiamata periodicamente (es. ogni ciclo iterativo 0.5 sec) dai workers Multiprocessing.
        Itera la flotta di droni in volo e guarda in parallelo tutte le RIS. Se una RIS è "Abbandonata" 
        ovvero nessun drone vola nella sua area di copertura, viene impostata in 'is_active = False' e si abbassa a modalità consumo 0.5W.
        
        Args:
            uav_positions (Dict): Registro centrale che mappa {id_drone: tupla_coordinate(X, Y, Z)}.
            threshold_dist (float): Guard-Band circolare (Distanza Tolleranza). Sotto questa quota in metri (Es. 30 mt), la RIS considera il drone "sotto il proprio ombrello RF".
        """
        # Convertiamo la lista di posizioni volanti direttamente in una super-matrice matematica C-compiled tramite numpy
        # per una fluidità estrema sui calcoli distanziali spaziali.
        uav_coords = np.array(list(uav_positions.values()))
        
        # Guard-Clause: il cielo è vuoto! C'è il coprifuoco o errore di spawn.
        if len(uav_coords) == 0:
            for ris in self.ris_nodes.values():
                ris.is_active = False # Deep Sleep forzato per tutti i sensori. Non essendoci utenza, non irradiamo inutilmente il capannone.
            return

        # Itariamo sul nostro inventario di RIS mappate sui muri
        for ris_id, ris in self.ris_nodes.items():
            ris_arr = np.array(ris.position)
            
            # CALCOLO CINETICO PARALLELIZZATO: Sottraiamo la Posizione RIS dal vettore array dei UAV e calcoliamo la "Norma" lineare geometrica. Restituirà un array di dimensioni [1 x Numero Droni] con le distanze espresse in metri.
            distances = np.linalg.norm(uav_coords - ris_arr, axis=1)
            
            # np.any scansiona l'array generato al volo. Se anche un SOLO drone (any) matcha la diseguaglianza logica 'distanza rispetto all'antenna è strettamente inferiore della tolleranza', la condizione restituisce VERO.
            if np.any(distances <= threshold_dist):
                # Il pannello intelligente esce dalla stasi e irradia attivamente energia
                ris.is_active = True
                
                # Questa parte traccia chi sta usando il ponte radio. Associa l'ID del drone usando i Python list-comprehension abbinati a enumerate 
                # (Estrapolando solo chi sta realmente nei famosi < 30 metri).
                ris.attached_uavs = [list(uav_positions.keys())[i] for i, d in enumerate(distances) if d <= threshold_dist]
            else:
                # Modello di Risparmio Rigido: Nessuno nell'area? Spegniti immediatamente.
                ris.is_active = False
                ris.attached_uavs = [] # Pulisce la memoria per correttezza del GC (Garbage Collector)

    def predictive_handover_hook(self, extrapolated_trajectory: List[Tuple[float, float, float]], uav_id: int):
        """
        [STEP 6.4 - Utilizzato per il TEST 4 Hook]
        Logica 'Make-Before-Break' (Prevenzione Rotture Connettività).
        
        A differenza dell'Engine Green (Reattivo), l'Hook è PREDITTIVO. Quando le equazioni differenziali del Modulo 5 (EKF)
        proiettano una traiettoria d'estrapolazione di dove il drone si troverà nei prossimi X metri (anche oltre lo scaffale), 
        l'SDN controlla quest'array futuro.
        Se le palline proiettate della rotta futura entrano già da ora nell'orbita di servizio di una RIS momentaneamente 
        spenta, la accende PRIMA DELL'ARRIVO EFFETTIVO del drone, garantendo il cono d'ombra libero.
        
        Args:
            extrapolated_trajectory: Un array temporale di coordinate 3D che indicano i "passi probabili" calcolati dal Filtro di Kalman (EKF).
            uav_id: DGN Identifier (Digital Twin Global Network) dello specifico sciame fuchi.
        """
        # Array-vectorization immediata per abbattere l'overhead del Global Interpreter Lock (GIL) sui loop python-nativi.
        traj_pts = np.array(extrapolated_trajectory)
        if len(traj_pts) == 0:
            return # EKF sta singhiozzando o manovra ferma di "Hovering", esci a vuoto.
            
        # Il raggio d'attivazione deve essere leggermente superiore al threshold statico 
        # (es 40.0 metri) per garantire il Make-Before-Break ed anticipare al netto dei delay di calcolo.
        activation_radius = 40.0 
        
        # Scorriamo tutto l'inventario installato
        for ris_id, ris in self.ris_nodes.items():
            ris_arr = np.array(ris.position)
            
            # Calcolo di intersezione tra la Linea Teorica Tracciata dell'EKF e i sensori a muro
            dists = np.linalg.norm(traj_pts - ris_arr, axis=1)
            
            # IF Combinato Logicamente Spaccato (Lazy Evaluation): 
            # Controlla PRIMA se la RIS dorme. Se dorme, controlla la complessità matematica dei punti di itersezione sui raggi d'azione.
            if not ris.is_active and np.any(dists <= activation_radius):
                
                # SVEGLIATA IN ANTICIPO! L'avanguardia tecnologica SDN-6G. 
                ris.is_active = True  
                
                # --- AUDIT & DEBUG ---
                # Nelle prossime fasi inseriremo uno spoofer Bulk SQLite. Qui mockiamo il print strutturato.
                # Questa print "in console" non basterebbe in produzione per farci le query sopra come chiesto dal prof!
                # Al modulo 7 intercetteremo questa pretesa di stampa salvandola su disco e calcolando la sottrazione sui timestamp lì.
                print(f"[SDN TEST-4 M-B-B AUDIT] => EVENT: 'PRE_ACTIVATION' | RIS_ID: {ris_id} | UAV_TARGET: {uav_id} (Algoritmo EKF Tracking Preemption)")
