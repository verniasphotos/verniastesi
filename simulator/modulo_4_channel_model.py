#MODULO 4 - Modello di Canale e Fisica delle Onde Radio

import math              # Modulo standard di Python per operazioni matematiche (es. logaritmi, radici quadrate)
import numpy as np       # Libreria potentissima per il calcolo scientifico e statistico (veloce perché scritta in C sotto il cofano)
from typing import Tuple # Modulo per indicare i "tipi" di dato che le funzioni restituiscono (aiuta a prevenire bug)

# Importiamo le costanti e le "Dataclasses" che abbiamo definito nel file config2.py.
# Questo ci evita di scrivere i valori "a mano" (hardcoding) sparsi per tutto il codice.
from .modulo_1_config import NETWORK_6G, RIS_HARDWARE, WarehouseBase

class ChannelModel:
    """
    Modello di Canale basato rigorosamente sullo standard internazionale 3GPP TR 38.901 
    (Nello specifico, lo scenario: Indoor Factory - Dense Heterogeneous o "InF-DH").
    
    Spiegazione Ingegneristica: 
    Un "Modello di Canale" è un insieme di equazioni matematiche che tenta di simulare 
    come un segnale radio degrada, rimbalza e viene assorbito dall'ambiente circostante.
    """
    
    def __init__(self, carrier_freq_ghz: float = None):
        """
        Il metodo __init__ è il costruttore della classe. Viene chiamato automaticamente 
        ogni volta che crei un nuovo oggetto "ChannelModel()".
        """
        # Se non specifichiamo una frequenza, usa quella predefinita del 6G (5.9 GHz) dal file config2
        if carrier_freq_ghz is None:
            carrier_freq_ghz = NETWORK_6G.carrier_freq_ghz
            
        # Salviamo la frequenza (fc = Frequenza di Carrier/Portante) dentro l'oggetto (self)
        self.fc = carrier_freq_ghz
        
        # Istanziamo i parametri del magazzino (ci serviranno per sapere quanta attenuazione dà il metallo)
        self.warehouse_params = WarehouseBase()
        
    def calculate_path_loss_inf_dh(self, distance_3d: float, metal_penetration_m: float) -> float:
        """
        [4.2. Algoritmo Path Loss InF-DH]
        "Path Loss" significa "Perdita di tragitto". Più un drone è lontano dall'antenna, 
        più il segnale arriva debole. Questa funzione calcola esattamente quanti Decibel (dB) si perdono.
        
        Parametri richiesti dalla funzione:
        - distance_3d: La distanza in metri tra l'antenna e il drone (in linea d'aria retta 3D)
        - metal_penetration_m: I metri di spessore di scaffali d'acciaio che l'onda deve trapassare (se ci sono ostacoli).
        """
        # 1. Imposta una distanza minima di 1 metro. 
        # Perché? Perché i logaritmi (log 0) tendono a meno infinito e spaccherebbero il programma in caso 
        # il drone si trovasse esattamente alle coordinate 0.0 dell'antenna! Questo è un trucchetto difensivo.
        d = max(1.0, distance_3d)
        
        # 2. Free Space Path Loss (FSPL) - "Perdita nello spazio vuoto".
        # Questa è la formula empirica (ricavata sperimentalmente dal vivo) dal documento TR 38.901 del 3GPP.
        # Usa il logaritmo in base 10 (math.log10) sulla distanza e sulla frequenza.
        # Significato: le onde radio alte (6G) perdono energia molto più in fretta nello spazio rispetto al 4G!
        pl_los = 31.84 + 21.50 * math.log10(d) + 19.0 * math.log10(self.fc)
        
        # 3. Shadowing (Ombreggiatura / Fading a lenta variazione).
        # Nella realtà un'onda radio non perde potenza in modo perfettamente lineare. Rimbalza e crea interferenze!
        # Simuliamo questa imprevedibilità estraendo un numero casuale da una "Curva a Campana" (Distribuzione Gaussiana).
        # 'media=0, deviazione_standard=3.5 dB'. A volte il segnale migliorerà un po', altre peggiorerà.
        shadowing = np.random.normal(0, 3.5)
        
        # 4. Penalizzazione NLoS (Non-Line of Sight / Senza linea di vista diretta).
        # Se il "raggio" (calcolato col Numba nel Modulo 2) incrocia i muri o scaffali, calcoliamo i metri attraversati.
        # Moltiplichiamo i metri attraversati per la "penetration_loss_db_m" (es. 15.0 dB per ogni singolo metro di metallo).
        penetration_loss = metal_penetration_m * self.warehouse_params.penetration_loss_db_m
        
        # 5. La perdita totale è la somma della distanza pura, delle fluttuazioni casuali e degli ostacoli metallici.
        total_pl = pl_los + shadowing + penetration_loss
        return total_pl
        
    def apply_ris_amplification(self, base_snr: float, is_ris_active: bool) -> float:
        """
        [4.3. Algoritmo RIS Amp]
        Le RIS (Reconfigurable Intelligent Surfaces) sono i "pannelli specchio" intelligenti del 6G.
        Questa funzione elabora quanto il segnale migliora se la smart surface viene accesa in quel momento.
        """
        # Se la RIS dorme (magari per risparmiare energia grazie all'Intelligenza Artificiale che la spegne),
        # l'SNR (Signal to Noise Ratio) non subisce miglioramenti. Viene restituito intatto com'era.
        if not is_ris_active:
            return base_snr
            
        # Se la RIS è ACCESA:
        # L'SNR finale diventa l'SNR Base PIU' il guadagno dell'antenna (es. +20 dB).
        # Ma attenzione! La fisica ci dice che ogni circuito elettronico acceso emette un "ronzio" termico (Noise Figure).
        # Quindi dobbiamo sottrarre quel disturbo (es. -3 dB) altrimenti avremmo una simulazione irreale e troppo perfetta.
        enhanced_snr = base_snr + RIS_HARDWARE.gain_db - RIS_HARDWARE.noise_figure_db
        return enhanced_snr
        
    def compute_beam_misalignment_loss(self, pitch_deg: float, roll_deg: float) -> float:
        """
        [4.4. Beam Misalignment]
        La tecnologia 6G alle alte frequenze richiede fasci d'onda (beams) super-direzionali, stretti come un laser.
        Se l'assetto aerodinamico del drone devia troppo (cioè se il drone si inclina sbandando),
        il ricevitore perde "di mira" il laser e inizia a perdere decibel.
        """
        # Usiamo il Teorema di Pitagora per calcolare l'inclinazione "sommata" totale (pitch = beccheggio, roll = rollio)
        total_misalignment = math.sqrt(pitch_deg**2 + roll_deg**2)
        
        # Se siamo sotto i 5 gradi di inclinazione, il drone è abbastanza orizzontale. Il beam è puntato al 100%. Nessuna perdita (0.0).
        if total_misalignment <= 5.0:
            return 0.0
            
        # Se superiamo i 5°, la formula applica una penalità di 0.5 dB per ogni grado di "sbandamento".
        loss = (total_misalignment - 5.0) * 0.5
        
        # La funzione 'min' serve da tappo di sicurezza ("Cap"). La perdita massima impostata è 15.0 dB massimi. 
        # Oltre questo, è come se il fascio fosse letteralmente andato "fuori bersaglio" completamente.
        return min(loss, 15.0)

# =========================================================================
# BLOCCO DEMO PER SPIEGAZIONE E ESERCITAZIONE CON IL DATABASE (SQL)
# Questo blocco viene eseguito SOLO se fai partire questo singolo file.
# Non interferisce col simulatore quando gli altri file lo chiamano.
# =========================================================================
if __name__ == "__main__":
    import sqlite3     # Libreria nativa in Python per manipolare database relazionali (SQL) senza installare grandi server
    import random      # Generazione di numeri casuali (comodi per inventare dati finti per i test)
    import os          # Libreria per comunicare col sistema operativo (es. per gestire file, cancellarli, ecc)
    
    print("Avvio generazione dati campione SNR per l'esercitazione pratica SQL...")
    
    db_path = "snr_demo.db" # Nome del file del database che faremo comparire sul tuo Desktop virtuale.
    
    # Se un file vecchio esiste (magari hai riprovato il test), cancelleremo il vecchio per evitare doppioni
    if os.path.exists(db_path):
        os.remove(db_path)
    
    # "Connessione" nativa: ordiniamo a Python di creare o aprire il database e assegniamo un "Cursore".
    # Il cursore (cursor) è letteralmente la penna invisibile con cui scriveremo le nostre istruzioni (Query) in linguaggio SQL
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # LINGUAGGIO SQL: "Se non esiste, crea una tabella chiamata 'snr_logs' con le colonne che ti detto".
    # 'REAL' indica un numero con la virgola (equivalente del "float" di Python, ma in termine Database).
    cursor.execute('''
    CREATE TABLE snr_logs (
        id INTEGER PRIMARY KEY AUTOINCREMENT, 
        x REAL,
        y REAL,
        z REAL,
        distance_m REAL,
        metal_dist_m REAL,
        base_path_loss REAL,
        ris_active BOOLEAN,
        final_snr REAL
    )
    ''')
    
    # Prepariamo un "motore di calcolo" usando la nostra classe qua sopra!
    cm = ChannelModel()
    
    # Avviamo un ciclo for: lo ripercorriamo per 150 volte, così da inventare 150 rilevamenti del drone sparsi.
    for _ in range(150):
        # Inventiamoci coordinate a caso su X, Y, Z. Usiamo 'round' per avere solo 2 numeri dopo la virgola.
        x = round(random.uniform(0, 50), 2)
        y = round(random.uniform(0, 40), 2)
        z = round(random.uniform(0, 10), 2)
        
        # Teorema di Pitagora a 3 Dimensioni per ricavare la retta della distanza dal centro (0,0,0) all'antenna drone.
        dist = round(math.sqrt(x**2 + y**2 + z**2), 2)
        
        # Inventiamo un ostacolo! Tiriamo un dado percentuale invisibile (da 0.0 a 1.0 tramite random.random())
        metal_penetration = 0.0
        if random.random() > 0.6: # Nel 40% dei casi inseriamo degli ostacoli di metallo ...
            metal_penetration = round(random.uniform(0.5, 2.0), 2) # da mezzo metro a 2 metri di spessore.
            
        # 1. Calcoliamo la perdita base (Richiamando la nostra funzione Python scritta in alto)
        pl = cm.calculate_path_loss_inf_dh(dist, metal_penetration)
        
        # 2. Conversione da "Perdita" al concetto importantissimo dell'SNR (Signal-to-Noise Ratio).
        tx_power_dbm = 23.0    # Potenza d'inizio del Modem in Decibel Milliwatt (Circa 200mW reali)
        noise_floor_dbm = -90.0 # Pavimento di rumore tipico RF (L'inquinamento elettromagnetico costante della natura)
        
        # Ricavo il segnale netto finale. SNR = Potenza in uscita - Perdita via aria - Rumore della natura.
        snr_base = tx_power_dbm - pl - noise_floor_dbm
        
        # 3. Intervento dell'Intelligenza Artificiale RIS: Tiriamo a sorte e il 30% delle volte accendiamo un modulo a muro
        ris_accesa = random.choice([True, False, False])
        snr_con_ris = cm.apply_ris_amplification(snr_base, ris_accesa)
        
        # 4. Dinamica di volo: Il drone ogni tanto è disturbato aerodinamicamente. Perdita del Beam (disallineamento)
        pitch = random.uniform(0, 15)
        roll = random.uniform(0, 15)
        snr_finale = round(snr_con_ris - cm.compute_beam_misalignment_loss(pitch, roll), 2)
        
        # ALLA FINE DI QUESTI CALCOLI SALVIAMO TUTTO!
        # Chiediamo al 'Cursore' di fare eseguire un comando INSERT su SQL (Inserimento Dati) sostituendo 
        # i punti interrogativi '?' in modo ordinato con tutte le nostre variabili faticosamente calcolate.
        cursor.execute('''
            INSERT INTO snr_logs (x, y, z, distance_m, metal_dist_m, base_path_loss, ris_active, final_snr)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        ''', (x, y, z, dist, metal_penetration, round(pl, 2), ris_accesa, snr_finale))
        
    # 'Commit' è la parola chiave magica e finale. Solo quando fai "commit", 
    # la libreria prende tutto quello che hai inserito temporaneamente e lo scrive FINALMENTE a livello fisico sul Disco.
    conn.commit()
    conn.close() # Rilascia il file, da buoni cittadini dell'Informatica.
    
    print("\n✅ Database 'snr_demo.db' creato con successo con 150 rilevamenti incisi sul disco!")
    print("📌 TASK PER IL TUTORATO: Apri 'snr_demo.db' su VS Code con SQLite Viewer ed analizza i dati.")
