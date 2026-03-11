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

## 8. Modulo 7: Motore e Stress Test (`main.py`)
- Loop principale. Crea 3 layout:
  - **Caso A**: Piccolo (es. 2000 mq).
  - **Caso B**: Medio (es. 10000 mq).
  - **Caso C**: Grande (es. 35000 mq).
- **Breakdown Test**:
  - Avvia il loop temporale aggiungendo +5 droni alla volta.
  - I droni si muovono, consumano batteria, tornano alla base.
  - *Condizione di stop*: Si ferma quando troppi droni saturano il server (es. SNR crolla per >20% della flotta o vengono fatte troppe richieste alle RIS in un solo `DT`). Salva il numero massimo di droni.

## 9. Modulo 8: Visualizzazione (`plotting.py`)
- Esegue query sul database SQLite per generare grafici tramite `matplotlib`:
  1. **Mappa (Scatter Plot 2D/3D)**: Layout del magazzino con ostacoli, BS, RIS e traiettoria di un drone.
  2. **Line Chart Flotta**: Livello di batteria di un drone nel tempo (dimostra il ciclo 24/7 di scarica e ricarica).
  3. **Bar Chart Stress Test**: Confronto "Consumo Energetico Totale vs Overhead" per i casi A, B, C al punto di rottura.

---
**Azione per l'IA:** Conferma di aver letto il PRD, riassumi in 2 righe l'obiettivo e chiedimi quale file vuoi che sviluppiamo per primo.