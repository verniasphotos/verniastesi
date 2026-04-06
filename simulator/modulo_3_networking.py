#MODULO 3 
import time              # Utile per usare time.sleep() e fermare l'esecuzione per qualche frazione di secondo (per simulare lag di rete)
import numpy as np       # Libreria per manipolare vettori (array) multidimensionali ad alta velocità
from multiprocessing import shared_memory # Il "magico" modulo che bypassa il GIL e permette la comunicazione tra due file Python diversi via RAM
import sqlite3           # Il modulo per interrogare o creare, modificare tabelle e database (quello che apri poi su SQLite Viewer)
import os                # Utile per manipolare percorsi dei file e farti trovare il DB nella cartella del Desktop

class SharedMemoryAllocator:
    """
    Gestisce l'allocazione di blocchi di memoria condivisa a basso livello.
    Bypassa il GIL (Global Interpreter Lock) di Python permettendo
    la comunicazione iper-veloce tra processi paralleli.
    
    Per dirla in modo semplice: anziché far passare i numeri (es. coordinate {x: 1, y: 3}) 
    dal processore centrale, riserviamo un cassetto fisicamente scavato nella tua memoria RAM 
    che chiunque può leggere/scrivere istantaneamente.
    """
    def __init__(self, name: str, size: int):
        # type hints: name di tipo stringa e size (dimensione in byte) di tipo numero intero
        self.name = name  # Assegniamo il nome della variabile (es. "flight_path")
        self.size = size  # e la sua grandezza che allocheremo nella RAM
        
        # Uso del blocco try-except: un tentativo "in sicurezza" per evitare crash letali del programma.
        try:
            # Tenta di agganciarsi a una memoria esistente con lo stesso nome.
            # Se un altro processo Python ha GIA' creato questa cella di memoria RAM, ci colleghiamo a quella.
            self.shm = shared_memory.SharedMemory(name=self.name)
            self._is_creator = False # Diciamo al codice: "Io non ho creato questa RAM, l'ha creata l'altro programma e ci entro."
            
        except FileNotFoundError:
            # Se `shared_memory(name=self.name)` fallisce perché nessuno l'ha creata (Non trovata),
            # il programma non muore ma esegue l'except come "Piano B".
            
            # Crea una nuova porzione di memoria RAM dal nulla (perché create=True).
            self.shm = shared_memory.SharedMemory(name=self.name, create=True, size=self.size)
            self._is_creator = True  # Segnamo che SIAMO NOI I CREATORI, quindi toccherà a noi distruggerla alla fine.
            
    def write_array(self, arr: np.ndarray):
        """Copia in modo efficiente un array numpy (coordinate spaziali) nella shared memory appena allocata."""
        
        # 'np.ndarray' in questo caso crea una "Finestra logica" dello stesso formato dell'array in ingresso.
        # E gli dice esplicitamente: "Invece di crearti uno spazio tuo nuovo, vai a posizionarti nel buffer magico creato su (self.shm.buf)"
        shm_array = np.ndarray(arr.shape, dtype=arr.dtype, buffer=self.shm.buf)
        
        # Le parentesi [:] indicano "Tutti gli elementi validi".
        # Qui Sovrascriviamo tutti i valori vecchi con i nuovi dell'array in input. 
        # Da questo momento esatto, chiunque legga la Shared Memory leggerà le nuove coordinate aggiornate.
        shm_array[:] = arr[:]
        
    def read_array(self, shape: tuple, dtype) -> np.ndarray:
        """Legge l'array puntando direttamente al buffer di memoria RAM senza fare copie faticose per il computer."""
        # Ri-costruiamo la finestra verso il buffer (questa volta chi invoca il metodo vuole solo leggere, non scrivere)
        shm_array = np.ndarray(shape, dtype=dtype, buffer=self.shm.buf)
        
        # Ritorniamo una 'fotografia' immediata (copy) dei numeri presenti nel cassettino
        return shm_array.copy()

    def cleanup(self):
        """Libera la memoria RAM per evitare che si saturi (fenomeno noto come "memory leaks" o fughe di memoria).
        Se non facessimo questo rituale di chiusura, il tuo computer (Mac) accumulerebbe spazzatura 
        nella RAM fino a bloccarsi del tutto richiedendo un riavvio forzato."""
        
        self.shm.close() # Chiude gentilmente il "tubo" di collegamento verso la memoria (come riagganciare la cornetta del telefono)
        
        # Ma attenzione, la chiusura logica non basta: bisogna svuotarla del tutto se sei tu che l'avevi creata.
        if self._is_creator:
            self.shm.unlink() # Distruggi materialmente i dati salvati e sgombra la RAM in modo definitivo per l'intero Sistema Operativo.


class SimulationClock:
    """
    Gestore del tempo matematico e deterministico.
    Garantisce che ogni avanzamento del simulatore avvenga al ritmo costante di dt = 0.1 secondi
    a prescindere da se il computer ci metta fisicamente 3 millisecondi o 12 millisecondi per fare i calcoli complessi del mondo 3D.
    """
    def __init__(self, dt: float = 0.1): # Di default il tick, il passo falso, è ogni 10 decimi di secondo
        self.dt = dt                     # Conserviamo il passo (il delta tempo)
        self.current_time = 0.0          # L'asse del tempo parte all'inizio perfetto a T = 0.0 sec
        
    def tick(self) -> float:
        """Questa funzione avanza l'orologio interno del Digital Twin, aggiungendo un piccolo passo al tempo attuale."""
        self.current_time += self.dt 
               
        # Quando lavoriamo coi decimali e i float, il computer sbaglia: 0.1 + 0.1 + 0.1 diventa spesso 0.3000000000000004
        # Se salvassimo quella robaccia nel database avremmo problemi visivi infiniti nei grafici.
        # round(self.current_time, 2) arrotonda forzatamente a due decimali perfetti.
        return round(self.current_time, 2)


class GRPCInterfaceMock:
    """
    Impalcatura di base (chiamata Mock in gergo, insomma una simulazione verosimile) di un server "gRPC".
    Simula e crea fittiziamente il ritardo di rete (la latenza del backhaul) tra la Base Station delle antenne 
    e il cervello del controller SDN.
    Inoltre, salva ogni singola comunicazione testuale su database relazionale SQLite per permetterne la tua ispezione futura.
    """
    def __init__(self, db_path: str = "telemetria.db"):
        # Nella rete 6g o 5g vera i pacchetti impiegano tempo per l'Handshake.
        # Noi stabiliamo a tavolino che impiega 0.05 secondi (ovvero 50 ms).
        self.latency = 0.05 
        
        # Modulo "os" (Operating System) importato all'inizio scende fuori dalla cartella `simulator`
        # Cosi facendo il file `.db` non si auto-salverà "dentro" la cartella sbagliata,
        # ma finirà bellissimo sul Desktop direttamente vicino al tuo file README e al file .md.
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.db_path = os.path.join(base_dir, db_path) # Cuce insieme il divario e compone il "path assoluto" corretto.
        
        # Finito l'assetto iniziale, si chiama subitissimo (in automatico) la funzione '_init_db' scritta qua sotto.
        self._init_db()

    def _init_db(self):
        """Inizializza la colonna e la tabella 'NetworkLogs' dal nulla, se è la pima volta che facciamo partire il programma.
        Questa è la funzione "mecca-idraulica" che usa le vere Query in linguaggio SQL."""
        
        # Connessione al file sqlite3. Se il file 'telemetria.db' non esiste oggi, sqlite3 magia nera provvede
        # a crearlo silenziosamente come file vuoto nello stesso istante.
        conn = sqlite3.connect(self.db_path) 
        
        # Cos'è un cursore? Immagina che il cursore sia la testina scrivente su un file testuale, 
        # il "dito" con cui posizioni e attivi fisicamente i comandi testuali passati dall'esterno.
        cursor = conn.cursor() 
        
        # Eseguiamo formalmente il comando standard. 
        # CREATE TABLE IF NOT EXISTS ha un senso geniale: "Crea una tabella e chiamala NetworkLogs, 
        # MA fallo solo se non è mai stata costruita ieri altrimenti sovrascrivi tutto e fai un patatrac".
        # Definiamo inoltre qui la forma architettonica della tabella:
        # - id: colonna tipo chiave primaria, ovvero autonumerante (riga 1, poi 2, poi 3...)
        # - timestamp: numero con la virgola (chiamato REAL in sqlite).
        # - message: Testo generico
        # - latency_ms: Altro valore decimale per appuntare quanto ritardo c'era simulato.
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS NetworkLogs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL,
                message TEXT,
                latency_ms REAL
            )
        ''')
        # Salva "su Rullino" le modifiche. Se non fai .commit(), come su videogioco tutto si resetterà allo spegnimento.
        conn.commit()
        # Regola aurea del Senior Dev: apri la porta ma poi la chiudi. Chiudi la connessione, eviterai che il Database resti impallato a vita ("Locked error")
        conn.close() 

    def send_telemetry(self, timestamp: float, uav_id: int, position: tuple) -> bool:
        """
        Simula fittiziamente un "invio dati" da parte del drone in movimento. 
        Questa funzione farà letteralmente paralizzare per 50 ms l'intelligenza artificiale, simulando l'attesa del feedback dal 6G.
        Conclude poi "spingendo in modo relazionale" la notizia storica all'interno della famosissima tabella log (SQLite).
        """
        # La pausa temporale ferma fisicamente il processore del tuo Mac ('sleep' di sonno profondo)
        time.sleep(self.latency) 
        
        # Convertiamo quei 0.05 "Secondi" in "millisecondi" interi per bellezza diagnostica (0.05 * 1000 = 50ms puri)
        latency_ms = self.latency * 1000 
        
        # Cosa sono le "f-string"? 
        # Le virgolette della scritta iniziano per f" ". Questo comando molto Pythoniano, consente di "iniettare" a vivo
        # un'altra variabile python racchiudendola tra due parentesi graffe: es {position} scriverà direttamente le sue varibili testuali allineate senza impazzire concatenando segni plus (+)
        msg = f"UAV_{uav_id} telemetry sync - XYZ Pos: {position}"
        
        # Ci colleghiamo di nuovo alla Cassaforte
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # L'istruzione "INSERT" per mandare stringhe all'interno spinge l'"Insert" nei tre campi giusti della tabella SQL citate.
        # Perché scriverai ( ?, ?, ? ) in SQL? 
        # E' l'approccio detto "Query Parametrizzata". Un metodo difensivo (Security Ingegnerizzata) 
        # per non incorrere nel famigerato errore "SQL INJECTION" proteggendo i dati mettendoli sicuri in coda dentro VALUES
        cursor.execute('''
            INSERT INTO NetworkLogs (timestamp, message, latency_ms)
            VALUES (?, ?, ?)
        ''', (timestamp, msg, latency_ms))
        
        conn.commit() # Timbra per storicizzare
        conn.close()  # E chiudi sempre.
        
        return True   # Al termine, restituisci "Vero" al sistema parent per dirgli "Tutto OK fratello, nessun crash, io ho mandato il pacchetto finto."
