import numpy as np
from scipy.spatial import KDTree
from numba import njit
from .config2 import LayoutConfig

class CollisionError(Exception):
    """
    Eccezione personalizzata sollevata quando il drone si scontra con uno scaffale o coi muri.
    Avere un errore "su misura" è una best practice: se ci sono problemi, 
    sappiamo subito che è stata una collisione e non un bug del linguaggio.
    """
    pass

class Environment:
    """
    Rappresenta il "Mondo Fisico" (il magazzino logistico).
    Si occupa di calcolare gli ostacoli, costruire le strutture dati matematiche
    (usate poi per il segnale radio) e verificare le regole base di volo.
    """
    def __init__(self, layout: LayoutConfig):
        # Riceve i parametri hardware e di planimetria (definiti in config2.py)
        self.layout = layout
        
        # 1. Generiamo le coordinate dei centri (assi X,Y,Z) di ogni singolo ripiano di scaffale
        self.shelf_centers = self._generate_grid()
        
        # 2. Inseriamo questi centri in un oggetto KDTree.
        # Il KDTree è fondamentale: pre-struttura lo spazio in settori geometrici, ci permetterà di 
        # trovare lo scaffale più vicino al drone quasi all'istante, anziché misurare ogni volta TUTTI gli scaffali.
        self.kdtree = KDTree(self.shelf_centers)
        
        # 3. Costruiamo i "Bounding Box" volumetrici: vere e proprie 'casse limitanti'.
        # Serviranno al sistema RF (Radio Frequenza) Numba per capire se il segnale ci passa attraverso.
        self.shelf_boxes = self._generate_boxes()

    def _generate_grid(self):
        """
        [2.2. Grid Generator Matrix]
        Simuliamo la pavimentazione del magazzino:
        Genera ed elenca le coordinate (X, Y, Z) posizionando fisicamente i banchi.
        Rispetta lo spazio per i corridoi VNA (Very Narrow Aisle).
        """
        centers = [] # Lista vuota che raccoglierà la posizione di ogni singolo ripiano
        
        # Il ciclo inizia tenendo conto del 'margine di rispetto' dai muri sinistri,
        # e aggiungendo mezza "profondità" dello scaffale così da piazzare il punto esatto al suo 'Centro Geografico'.
        x = self.layout.wall_spacing_m + (self.layout.shelf_x_m / 2)
        
        # Finché abbiamo spazio sul pavimento (Larghezza Max meno un lato di rispetto sul bordo finale)...
        while x + (self.layout.shelf_x_m / 2) <= self.layout.x_dim_m - self.layout.wall_spacing_m:
            
            # Per ogni posizione X, partiamo a riempire l'asse Y (formiamo la riga, in profondità).
            y = self.layout.wall_spacing_m + (self.layout.shelf_y_m / 2)
            while y + (self.layout.shelf_y_m / 2) <= self.layout.y_dim_m - self.layout.wall_spacing_m:
                
                # Su pavimento c'è uno spazio occupato da scaffali, iniziamo a impilarli verso il soffitto (asse Z).
                z = self.layout.shelf_z_spacing_m
                while z <= self.layout.z_dim_m - 1.0: # Lasciamo per sicurezza un metro dal tetto
                    # Abbiamo tracciato un centro esatto! Aggiungiamo alla lista come sub-lista [x, y, z]
                    centers.append([x, y, z])
                    # Saliamo dello spazio di un ripiano.
                    z += self.layout.shelf_z_spacing_m
                
                # Terminata un 'torre', ci si muove in avanti per il prossimo segmento sempre nella stessa Fila (lungo asse Y).
                y += self.layout.shelf_y_m
                
            # Finita interamente LA FILA, la collaudiamo! Ci spostiamo a destra sull'orizzonte (asse X)
            # della larghezza dello scaffale APPOSITA PIU' larghezza del Corridoio (così da staccare due settori fisicamente).
            x += self.layout.shelf_x_m + self.layout.vna_width_m
            
        # Chiudiamo ridando una struttura Numpy, ottimizzata e scritta in backend in C.
        return np.array(centers, dtype=np.float64)

    def _generate_boxes(self):
        """
        Partendo dal centro di ogni scaffale appena generato, questa stringe un vero
        volume squadrato per ciascuno. Ritorna i confini [x_min, x_max, y_min, y_max, z_min, z_max].
        """
        boxes = []
        # Calcoliamo prima le 'mezze misure', come per un raggio per il diametro.
        hx = self.layout.shelf_x_m / 2
        hy = self.layout.shelf_y_m / 2
        hz = self.layout.shelf_z_spacing_m / 2
        
        # Scorri ogni centro calcolato nel passaggio precedente
        for center in self.shelf_centers:
            cx, cy, cz = center
            # Sottraiamo le mezze misure al centro e otteniamo il vertice minimo. Le sommiamo -> vertice massimo.
            boxes.append([cx - hx, cx + hx, cy - hy, cy + hy, cz - hz, cz + hz])
            
        return np.array(boxes, dtype=np.float64)

    def validate_clearance(self, drone_pos: np.ndarray, margin: float = 1.5):
        """
        [2.5. Collision Check]
        Una rete 6G controlla un robot, deve assicurarne l'incolumità in loop costantemente!
        Usa l'albero per non consumare processore inutilmente e trova il 'pericolo più imminente'.
        """
        # Porgi la posizione del drore a KDTree. Come una 'bolla', esso dirà 'l'ostacolo più vicino è a TOT distanza, ed è al numero indice _'
        dist, _ = self.kdtree.query(drone_pos)
        
        x, y, z = drone_pos
        wall = self.layout.wall_spacing_m
        
        # Multiplo controllo logico: il drone tocca il pavimento? o Esce dal perimetro? o La distanza rilevata dalla bolla è più piccola del Margine di Sicurezza?
        if (z < 0.0 or 
            x < wall or x > self.layout.x_dim_m - wall or 
            y < wall or y > self.layout.y_dim_m - wall or 
            dist < margin):
            # C'E' LO SCHIANTO ! Chiamiamo in causa fermando il volo dell'evento
            raise CollisionError(f"Collisione imminente! Distanza minima: {dist:.2f}m")
        return True


# IL DECORATORE MAGICO DI COMPILAZIONE DELLA FISICA AVANZATA
# Convertisce al boot questo blocco di equazione di linea ad Istruzioni Binarie del PC (Velocizzando le reti neurali simulativi o algoritmi)
@njit(nogil=True)
def ray_casting_numba(p1, p2, boxes):
    """
    [2.4. JIT Ray-Casting (Numba)]
    Immaginiamo che p1 sia un pannello e p2 l'antenna del drone. Si instaura un raggio, il "Ponte Radio".
    Questa funzione (ottimizzata Numba) calcola se e "di quanti metri cumulativi" l'onda elettro radio (il raggio laser ideale) percorre il metallo scaffali.
    """
    penetration = 0.0 # Contatore della distanza "non visuale" passata nel metallo
    
    # [VETTORI MATEMATICI]: Direzione D del nostro laser
    d = p2 - p1
    # Lunghezza per calcolo norma Euclidea Pura per capire quanta 'strada' compierà il raggio in totale.
    length = np.sqrt(d[0]**2 + d[1]**2 + d[2]**2)
    
    if length == 0:
        return 0.0 # Tx e Rx sono posizionati l'uno sull'altro. Nessun assorbimento bloccante.
    
    # Otteniamo Vettore Unitario Direzionale ('Versore')
    dir_vec = d / length
    
    # ALGORITMO AABB BOX INTERSECTION -> Efficentamento per evitare fallimenti divisione Zero se andiamo orizzontale in una sola direzione:
    inv_dir = np.empty(3, dtype=np.float64)
    for i in range(3):
        if dir_vec[i] == 0:
            inv_dir[i] = 1e15 # Assegna falso infinito, evitiamo arresto anomalo.
        else:
            inv_dir[i] = 1.0 / dir_vec[i]

    # Iterazione di precisione su ognuno degli ostacoli box 3D (X, Y, Z limiti misurati all'avvio precedentemente)
    for b in range(boxes.shape[0]):
        box = boxes[b]
        
        # --- TEST 1 LATERALE (ASSE X) per ingressi Raggio ---
        # Si controlla l'incidenza tempo / punto con cui il raggio attraversa il limite Min e il Max della coordinata X dell'ostacolo Scaffale.
        tmin = (box[0] - p1[0]) * inv_dir[0]
        tmax = (box[1] - p1[0]) * inv_dir[0]
        if dir_vec[0] < 0: # Scambio minimo se il raggio è diretto diagonalmente al contrario.. 
            temp = tmin; tmin = tmax; tmax = temp
            
        # --- TEST 2 FRONTALE (ASSE Y)  ---
        tymin = (box[2] - p1[1]) * inv_dir[1]
        tymax = (box[3] - p1[1]) * inv_dir[1]
        if dir_vec[1] < 0:
            temp = tymin; tymin = tymax; tymax = temp
            
        # VERO RISPARMIO COMPUTAZIONALE: CULLING DEL MISS (LO HA MANCATO!)
        # Se 'Y minimo' di incrocio del raggio avviene 'dopo' che è uscito in teoria dalla scatola  in X. È certo che il raggio passa di fianco esterno alla cassa e non c'è intersezione solida in assoluto
        if (tmin > tymax) or (tymin > tmax):
            continue
            
        # Aggiornamento parametri estremi intersecati
        if tymin > tmin:
            tmin = tymin
        if tymax < tmax:
            tmax = tymax
            
        # --- TEST 3 ALTEZZA (ASSE Z)  ---
        tzmin = (box[4] - p1[2]) * inv_dir[2]
        tzmax = (box[5] - p1[2]) * inv_dir[2]
        if dir_vec[2] < 0:
            temp = tzmin; tzmin = tzmax; tzmax = temp
            
        # Ultima conferma, il raggio transita sopra allo scaffale senza toccarlo? (Ignora!)
        if (tmin > tzmax) or (tzmin > tmax):
            continue
            
        # Definitivamente, sappiamo il punto millimetrico d'entrata (tmin globale) ed uscita (tmax) al centro scatola.
        if tzmin > tmin:
            tmin = tzmin
        if tzmax < tmax:
            tmax = tzmax
            
        # SE SIAMO ARRIVATI QUI in fondo, significa che IL RAGGIO PERFORA IL CUORE DELL'OSTACOLO!
        # Si fa un filtraggio tra il momento in cui parte l'antenna P1 e arriva P2 e non andiamo oltre e calcoliamo i metri esatti interni allo spessore.
        if tmax > 0.0 and tmin < length:
            # Punti sicuri delimitatori fisici del raggio!  
            actual_tmin = max(0.0, tmin)
            actual_tmax = min(length, tmax)
            # Qualcosa è entrato nel metallo ed aggiungiamo in memoria tale 'attraversamento NLoS' !
            if actual_tmax > actual_tmin:
                penetration += (actual_tmax - actual_tmin)
                
    return penetration
