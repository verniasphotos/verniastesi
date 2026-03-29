# Piano di Implementazione: Simulatore 6G per Tracking Droni Indoor assistito da RIS

Benvenuto! Questo documento è la nostra "bussola". Contiene il piano dettagliato, diviso per "Step", per implementare il tuo simulatore partendo rigorosamente da zero. Essendo tu un principiante, procederemo a piccoli passi. 

Ogni volta che vuoi avanzare, dimmi semplicemente quale Step eseguire (es. "Inizia lo Step 1"). Io scriverò il codice e, come richiesto, **ti spiegherò ogni riga, come funziona e come testarlo**.

---

## Step 1: Costanti e Parametri di Modello (`config.py`)
**Obiettivo:** Creare le fondamenta del progetto definendo tutte le variabili fisiche e geometriche (frequenze, potenze, velocità).
**Cosa faremo:**
- Creazione del file `config.py`.
- Tradurre i requisiti del PRD (es. 3.5 GHz, 20 dBm, batteria) in variabili Python.
**Task Finale per l'IA:** Spiega riga per riga il significato delle variabili, l'unità di misura utilizzata e come il variare di questi numeri influenzerà la fisica della simulazione.

## Step 2: Geometria dell'Ambiente e Magazzino (`environment.py`)
**Obiettivo:** Modellare matematicamente lo spazio 3D del magazzino logistico e gli scaffali metallici.
**Cosa faremo:**
- Creazione del file `environment.py`.
- Sviluppo di una classe per calcolare matematicamente le intersezioni tra la traiettoria del segnale e gli scaffali per capire se siamo in LOS (Line of Sight) o NLOS (Non-Line of Sight).
**Task Finale per l'IA:** Spiega in modo semplicissimo la geometria analitica usata (intersezione di segmenti), come rappresentiamo lo spazio 3D e come puoi stampare a schermo l'ambiente per verificarlo.

## Step 3: Entità della Simulazione (`devices.py`)
**Obiettivo:** Programmare i "protagonisti" (Droni, Pannelli RIS e Pacchetti).
**Cosa faremo:**
- Creazione del file `devices.py`.
- Scrittura del codice per il comportamento del Drone (movimento, consumo batteria, ritorno alla base).
- Scrittura del codice per i pannelli RIS (stati di energia e consumo).
**Task Finale per l'IA:** Spiega il concetto di "Programmazione Orientata agli Oggetti" (Classi) applicato a questo caso pratico. Spiega come far muovere un drone di prova e vedere come si scarica la sua batteria.

## Step 4: Fisica del Canale e Attenuazione (`channel.py`)
**Obiettivo:** Simulare come il segnale radio (3.5 GHz) si degrada attraversando il metallo.
**Cosa faremo:**
- Creazione del file `channel.py`.
- Creazione della funzione di *2-Way Ranging* che unisce le distanze e i calcoli degli ostacoli fatti nello Step 2 per restituire un valore in dB di attenuazione.
**Task Finale per l'IA:** Spiega la logica matematica usata per calcolare le distanze e simulare realisticamente l'effetto gabbia di Faraday degli scaffali, senza usare complesse simulazioni elettromagnetiche.

## Step 5: Memoria del Sistema e Database (`database.py`) - **[MOLTO IMPORTANTE]**
**Obiettivo:** Salvare ogni millisecondo tutti i dati dei droni e dei RIS in un Database relazionale.
**Cosa faremo:**
- Creazione del file `database.py`.
- Scrittura in codice Python per generare un file `telemetria.db` (SQLite).
- Creazione delle tabelle `Telemetria_Droni` e `Eventi_Rete`.
**Task Finale per l'IA:** 
1. Spiega in modo elementare cos'è SQL e come gestiamo un database su file.
2. Ti spiegherò passo-passo come installare un'estensione "SQLite Viewer" (come usarla in VS Code o scaricare DB Browser for SQLite).
3. Ti mostrerò come aprire il file `telemetria.db` per vedere i dati fisicamente mentre la simulazione gira o dopo che è finita, riga per riga.

## Step 6: Logica Decisionale Centralizzata (`controller.py`)
**Obiettivo:** Creare il "Cervello", ovvero la Base Station che accende e spegne i RIS in modo euristico.
**Cosa faremo:**
- Creazione del file `controller.py`.
- Programmare la logica: se un drone segnala bassa connessione a causa di ostacoli, individua il RIS migliore, accendilo e salva l'evento nel database.
**Task Finale per l'IA:** Spiega in parole povere l'intelligenza dell'algoritmo (euristica), come fa a prendere decisioni "al volo" e come verificare nel database se il Cervello ha preso la decisione giusta.

## Step 7: Motore Centrale e Stress Test (`main.py`)
**Obiettivo:** Unire tutti i pezzi e far letteralmente girare l'esperimento.
**Cosa faremo:**
- Creazione del file `main.py`.
- Mettere insieme ambiente, droni, fisica, database e controller.
- Creare il loop temporale. Scalare il numero di droni fino ad arrivare al limite fisico della rete (Stress Test / Breakdown).
**Task Finale per l'IA:** Spiega come far partire il programma dal tuo terminale, cosa significano le scritte (print) che compariranno a schermo e come capire ad occhio quando e perché la rete 6G è collassata sotto lo sforzo computazionale o di segnale.

## Step 8: Visualizzazione Grafica Risultati (`plotting.py`)
**Obiettivo:** Generare i grafici accattivanti da mostrare nella tua Tesi Triennale.
**Cosa faremo:**
- Creazione del file `plotting.py`.
- Creare script Python che leggono i dati dal nostro database `telemetria.db` per tracciare il percorso dei droni in 3D, scarica batteria e grafici a barre sui consumi della rete.
**Task Finale per l'IA:** Spiega come Python, tramite `matplotlib`, crea le immagini. Spiega come lanciare lo script per vederli apparire a schermo e come esportarli e salvarli come foto in altissima definizione, pronti ad essere incollati in Microsoft Word o LaTeX per la tua Tesi.

## Step 9: Simulazione Visuale Animata — Digital Twin con Tracking Real-Time (`config.py`)
**Obiettivo:** Implementare un modulo di rendering grafico bidimensionale (vista top-down) che generi un **Digital Twin animato** dell'intero magazzino. L'animazione permette di validare visivamente l'efficienza dell'algoritmo euristico di power management osservando in tempo reale le transizioni di stato energetico (Sleep → Passive → Active) dei nodi BS e RIS in risposta al movimento di un drone UAV.

**Parametri di Setup e Generazione Ambientale:**
- Il modulo esegue **tre sessioni di simulazione separate**, basate sulle topologie generate dinamicamente dal Modulo 2:
  - **Caso A** (Piccolo, 2.000 mq)
  - **Caso B** (Medio, 10.000 mq)
  - **Caso C** (Grande, 35.000 mq)
- Per ogni scenario, il clutter (corsie e scaffali metallici) e il deployment delle antenne (BS e RIS) sono calcolati automaticamente in base alla volumetria.
- Viene inizializzato un **singolo UAV** che segue una **traiettoria a serpentina** tra i corridoi, al fine di forzare continue transizioni tra visibilità diretta (LOS) e zone d'ombra (NLOS).

**Logica di Ranging e RF Dinamica:**
- Ad ogni step temporale (DT), il Super Server calcola il link budget a 3.5 GHz considerando il rumore termico di fondo (−100 dBm).
- Se l'attenuazione causata dagli scaffali fa crollare l'SNR avvicinandolo alla sensibilità critica del ricevitore (−90 dBm) e superando la `SOGLIA_RIS_ATTIVAZIONE`, il controller interviene immediatamente "risvegliando" il dispositivo RIS più idoneo per garantire un LOS virtuale continuo.

**Codifica Cromatica Tassativa (Interfaccia Visiva):**
- **Ostacoli (Scaffalature):** Rettangoli grigio scuro (`#404040`).
- **Drone (UAV):** Marker circolare in movimento (**magenta**, `#FF00FF`) con scia cinematica (trail).
- **Stato SLEEP (0.5 W — Standby):** BS e nodi RIS inattivi → marker circolare **ROSSO** (`#FF0000`).
- **Stato PASSIVE/ACTIVE (5 W / 50 W — On-Demand):** Dispositivo attivato dal Super Server → marker istantaneamente **VERDE** (`#00FF00`).

**Output Atteso:**
- Esportazione di **tre file video** (MP4 o GIF), uno per ciascun layout (A, B, C): `test_5_digital_twin_Caso_A.mp4`, `test_5_digital_twin_Caso_B.mp4`, `test_5_digital_twin_Caso_C.mp4`.
- I video dimostrano empiricamente l'efficacia dell'architettura proposta: la mappa appare per la quasi totalità "spenta" (nodi rossi), evidenziando solo una **"bolla di copertura" locale** (nodi verdi) che accompagna dinamicamente il drone.
- Ciò prova visivamente l'abbattimento dei consumi energetici rispetto a una rete Always-On tradizionale.

**Cosa faremo:**
- Aggiunta del metodo `test5_digital_twin()` alla classe `SimulationEngine`.
- Utilizzo di `matplotlib.animation.FuncAnimation` per il rendering frame-by-frame.
- Integrazione nel menu interattivo come opzione "5. [Test 5] Digital Twin Animato".

**Task Finale per l'IA:** Spiega il funzionamento di `matplotlib.animation.FuncAnimation`, il significato di `init_func` e `update_func`, come si configura il frame rate (FPS) e come si esporta il video risultante tramite i writer `ffmpeg` o `pillow`. Spiega inoltre come il video dimostra in modo empirico il risparmio energetico del sistema proposto.

---

### Mettiamoci al lavoro!
Quando sei pronto, rispondimi semplicemente copiando e incollando questa frase:
**"Perfetto, partiamo dallo Step 1. Crea il file `config.py` e ricordati di spiegarmi tutto alla fine."**
