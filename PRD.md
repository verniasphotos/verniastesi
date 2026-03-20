# Istruzioni di Sistema per l'Assistente IA
Agisci come un Senior Python Software Engineer e Tutor Universitario. Il tuo compito è aiutare uno studente di Ingegneria (livello laurea Triennale) a sviluppare un simulatore di rete 6G. 
Il codice deve essere scritto in **Python puro** usando solo le librerie `numpy`, `matplotlib`, `math` e `sqlite3`. 
**Vincoli architetturali:** - Scrivi codice pulito, iper-commentato in italiano e facile da capire. 
- Evita complessità inutili (niente multithreading, niente machine learning, niente simulazioni elettromagnetiche complesse: usa formule geometriche e distanze euclidee semplificate).
- **Non generare tutto il codice in una volta sola.** Leggi questo PRD, conferma di averlo compreso e chiedimi quale modulo vuoi che io ti faccia generare per primo (es. "Vuoi iniziare con config.py?").

---

# PRD: Simulatore 6G per Magazzino Logistico con Droni e RIS (Tesi Triennale)

## 1. Descrizione del Progetto
Il software simula un "cervello centrale" (Controller) in un magazzino logistico dove operano droni 24/7. Poiché gli scaffali metallici bloccano il segnale (NLOS), il sistema usa protocolli simulati di 2-Way Ranging (2WAY) per stimare l'attenuazione del metallo e accende dinamicamente dei pannelli RIS ("specchi intelligenti") per far rimbalzare il segnale radio verso i droni, spegnendoli poi per risparmiare energia. Il simulatore testa il limite di rottura (stress test) della rete.

## 2. Modulo 1: Costanti e Parametri (`config.py`)
- Frequenza: `FREQ = 3.5e9` (3.5 GHz).
- Potenza: `TX_POWER_DRONE = 20` (dBm).
- Cinematica: `V_DRONE = 3.0` (m/s), `DT = 0.1` (s).
- Batteria: `BATTERY_MAX = 100.0`, `BATTERY_RTH_THRESHOLD = 20.0` (soglia per ritorno alla base). Consumo stimato per ciclo DT: `0.05`.
- Consumi RIS: `P_SLEEP = 0.5` (W), `P_PASSIVE = 5.0` (W), `P_ACTIVE = 50.0` (W).
- Dimensioni: Scaffale `1.2m x 1.0m`.
- Raggi operativi: `R_BS = 50.0` (m), `R_RIS = 15.0` (m).

## 3. Modulo 2: Geometria e Hardware BoM (`environment.py`)
- **Classe Magazzino**: Genera layout 3D dati `(L, W, H)`.
- Calcola l'hardware necessario: N° di Base Station (BS), numero di livelli di volo in altezza, deployment delle RIS a parete e a griglia sul soffitto.
- **Metodo `check_LOS_and_shielding(p1, p2)`**: Calcola geometricamente quante volte il segmento che unisce `p1` e `p2` interseca uno scaffale. Restituisce un intero (numero di ostacoli) per stimare lo spessore della schermatura.

## 4. Modulo 3: Entità e Pacchetti di Rete (`devices.py`)
- **Classe Pacchetto_Rete**:
  - *Header*: `ID_Drone`, `TS` (Timestamp), `TX_Power`, `Battery_Level`.
  - *Payload*: `Package_ID`, `Route_Target` (coordinate xyz).
- **Classe Drone**:
  - Stato interno (`IN_MISSIONE`, `RTH_RICARICA`).
  - Funzione di movimento verso il target. La batteria decresce; se `<= 20.0`, il target diventa la Base Station.
- **Classe RIS**:
  - Stati: `sleep`, `passive`, `active`. Metodo per restituire il consumo attuale (W).

## 5. Modulo 4: Propagazione e Ranging (`channel.py` / `config.py`)
- **Metodo `esegui_2way_ranging(drone, bs, ris_list)`**:
  - Simula lo scambio di pacchetti 2WAY (Handshake) tra Drone e BS.
  - Usa `check_LOS_and_shielding` per capire quanti scaffali bloccano il segnale.
  - Calcola l'attenuazione dinamica (la tecnica di ranging rileva la "schermatura" o NLOS severity basandosi sulla caduta di segnale attraverso gli scaffali).
  - Simula l'Asimmetria: calcola l'**Uplink** (Drone -> BS a 20 dBm) e il **Downlink o ACK** (BS -> Drone a 40 dBm).
  - Il valore di attenuazione diventa l'input vitale per decidere il livello di amplificazione: se l'Handshake fallisce (l'ACK non torna), il sistema cerca una RIS.

## 6. Modulo 5: Server Tracking (`database.py`)
- Gestione `sqlite3` in memoria o su file `telemetria.db`.
- **Tabella 1 `Telemetria_Droni`**: `(TS, ID_Drone, X, Y, Z, SNR, Attenuazione_Ranging, Livello_Batteria, Stato_Missione)`.
- **Tabella 2 `Eventi_Rete`**: `(TS, ID_RIS, Azione, Consumo_W)`.
- Metodi per inserire dati a ogni step della simulazione.

## 7. Modulo 6: Controller Euristico (`controller.py`)
- **Classe SuperServer**:
  - Riceve il `Pacchetto_Rete` dal drone.
  - Analizza l'attenuazione. Se il segnale è sotto una soglia di allerta (es. SNR < 5 dB):
    - Cerca la RIS più vicina in LOS con il drone.
    - Accende la RIS (modalità `passive` se l'attenuazione è media, `active` se l'attenuazione è severa).
    - Salva l'evento in `Eventi_Rete`.
  - Coordina i droni (invia comando RTH se batteria bassa) e spegne le RIS quando non servono.

## 8. Modulo 7: Motore di Simulazione e Scenari di Test (`main.py`)
- Crea funzioni per eseguire 4 test separati. Per ogni test, il loop temporale avanza a scatti di `DT`, muove i droni e fa intervenire il `SuperServer`.
- **Test 1: Stress Test e Scalabilità (Punto di Rottura)**
  - *Setup*: Inizializza Caso A (2000 mq), Caso B (10000 mq), Caso C (35000 mq).
  - *Azione*: Inizia con 5 droni. Ogni N secondi simulati, aggiungi +5 droni alla flotta.
  - *Condizione di Stop*: Ferma il test per un Caso quando i messaggi elaborati dal Server superano la capacità massima o quando il 20% dei droni ha un SNR sotto la soglia critica per più di 5 secondi (collasso). Registra il numero massimo di droni raggiunto.
- **Test 2: Resilienza e Tolleranza ai Guasti (Fault Tolerance)**
  - *Setup*: Caso B con 15 droni in un'area specifica.
  - *Azione*: Fai girare la simulazione. All'istante t=50, simula un guasto spegnendo forzatamente la RIS più utilizzata in quel momento (es. stato = 'broken').
  - *Attesa*: Il Controller deve registrare il calo di SNR, eseguire il failover e "svegliare" le RIS adiacenti.
- **Test 3: Collo di Bottiglia "Cambio Turno" (Mass Return-To-Home)**
  - *Setup*: Caso C con 50 droni.
  - *Azione*: All'istante t=20, sovrascrivi forzatamente il livello di batteria del 40% dei droni portandolo al 21%.
  - *Attesa*: Subito dopo scenderanno sotto il 20%. Il Server invierà a tutti l'ACK di RTH_RICARICA. Registra l'esplosione di traffico e le accensioni RIS per gestire questo sciame.
- **Test 4: Confronto Energetico (La Baseline)**
  - *Setup*: Caso B con 15 droni. Simulazione di 10 minuti.
  - *Azione*: Esegui il run due volte. Run 1: spegni il Controller Euristico e tieni tutte le RIS sempre accese al massimo (50W). Run 2: usa il Controller Euristico (RIS in sleep a 0.5W, accese solo su richiesta).

## 9. Modulo 8: Visualizzazione Grafica Risultati (Data Plotting)
Il sistema deve fornire un'astrazione visiva ai *raw-data* telemetrici estratti dal Database SQLite elaborandoli matematicamente per produrre quattro grafici in formato `.png`, iterando l'analisi spaziale sui tre volumi operativi (Caso A: 2.000mq, Caso B: 10.000mq, Caso C: 35.000mq). Regola ferrea: non leggere variabili in memoria, ma eseguire query `SELECT`.

- **`plot_scalabilita.png`** (Grafico a Linee con Marker):
  - *Scopo*: Visualizzare il punto critico di rottura all'aumentare vertiginoso della flotta.
  - *Output visivo*: Asse X = Numero droni dispiegati. Asse Y = Messaggi processati/sec (Overhead). Traccia 3 rette ascendenti distinte per i tre Casi, terminanti con un marker speciale "x" rosso di collasso algoritmico della Rete.
- **`plot_resilienza_guasto.png`** (Grafico a Serie Temporali Sovrapposte):
  - *Scopo*: Dimostrare il rapido adattamento a percorsi di routing secondari in seguito allo spegnimento della RIS a soffitto.
  - *Output visivo*: Asse X = Tempo normalizzato a `0`s. Asse Y = SNR (dB). Tre curve colorate mostrano il ping SNR ininterrotto di tre droni campione a seguito dell'intersezione col marcatore tratteggiato (*Guasto RIS*) posto a `t=5s`, attestando il failover.
- **`plot_consumi_mass_rth.png`** (Grafico ad Aree / Serie Temporali Smoothed):
  - *Scopo*: Evidenziare la stringente gestione del picco di erogazione in caso di massivo *Return-To-Home*.
  - *Output visivo*: Asse X = Tempo sui Marker estratti (`START_Caso...`). Asse Y = Consumo istantaneo (W). Plotta tre aree (A, B, C) applicando un filtro "finestra mobile" di *smoothing* temporale a 50 campioni interpolando i record raw di eventi asincroni.
- **`plot_risparmio_energetico.png`** (Grafico a Barre Raggruppate):
  - *Scopo*: Esporre il *Benchmark* quantitativo sui benefici ecologici e computazionali.
  - *Output visivo*: Istogramma aggregato in kW tramite `SUM(Consumo_W)`. Asse X raggruppa su magazzino A, B, C comparando doppiamente la colonna termica *Always-On* (Tradizionale - rosso) con la contrazione estrema dei kW in Run 2 (Schema Proposto/Euristico - verde).

## 10. Modulo 9: Deployment Dinamico e Visualizzazione Topologica (Interactive BoM)

### Descrizione dell'Obiettivo
Questo modulo evolve il simulatore da un approccio statico a uno scalabile e parametrico. Il sistema è progettato per accettare in input le dimensioni tridimensionali fisiche di un generico magazzino logistico e calcolare in maniera autonoma la **Distinta Base (BoM - Bill of Materials)** dell'hardware di rete necessario. L'output finale è una dashboard visiva che presenta la **mappa topologica** dell'infrastruttura a sinistra e il **report quantitativo** esatto sulla destra.

### Variabili di Input
L'algoritmo riceve dall'utente i seguenti parametri strutturali:
- **L_MAG**: Lunghezza del magazzino (in metri).
- **W_MAG**: Larghezza del magazzino (in metri).
- **H_MAG**: Altezza del magazzino (in metri).

### Logica di Calcolo e Regole Ingegneristiche (Deployment)
Basandosi sui limiti fisici di propagazione del segnale impostati nel Modulo 1, l'algoritmo posiziona l'hardware seguendo queste regole:

- **Base Station (BS):** Considerato un raggio efficace `R_BS = 50.0 m`, il sistema calcola se è sufficiente una singola BS (per aree fino a 10.000 mq) posizionata al centro geometrico, oppure se è necessaria una griglia di BS per superfici maggiori.
- **RIS a Parete (Wall-mounted):** Considerato un raggio efficace `R_RIS = 15.0 m`, l'algoritmo posiziona i pannelli lungo l'intero perimetro del magazzino, distanziandoli in modo ottimale (es. ogni 30 metri) per garantire la riflessione del segnale nei corridoi perimetrali.
- **RIS a Soffitto (Ceiling-mounted):** Il sistema calcola una maglia a griglia (es. 30×30 m) e posiziona le RIS in sospensione per garantire la copertura verticale (Line-Of-Sight) alla flotta di droni in volo sopra o tra gli scaffali.
- **Super Server (Controller):** Quantità bloccata a **1 istanza**, posizionata tipicamente alle coordinate di origine o adiacente alla BS principale per minimizzare la latenza di rete fissa.

### Interfaccia di Output (UI e Mappa)
Il modulo genera un'interfaccia divisa in due sezioni:

- **Area di Plottaggio (Sinistra):** Una mappa 2D (vista top-down) renderizzata tramite `matplotlib` che illustra i confini del magazzino e la distribuzione spaziale dei nodi. Ogni dispositivo è identificato da marker specifici (es. ★ verde = Server, ▲ rosso = BS, ■/● azzurri = RIS).
- **Pannello BoM (Destra):** Una sezione di testo/tabella ancorata alla destra del grafico che espone in chiaro i risultati del calcolo algoritmico:
  - Dimensioni totali e Area (mq).
  - Numero esatto di Base Station installate.
  - Numero esatto di RIS a parete e RIS a soffitto necessarie.
  - Totale dell'hardware e numero di Super Server (1).

---
**Azione per l'IA:** Conferma di aver letto il PRD, riassumi in 2 righe l'obiettivo e chiedimi quale file vuoi che sviluppiamo per primo.