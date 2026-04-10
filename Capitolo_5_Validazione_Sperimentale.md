# Capitolo 5: Validazione Sperimentale e Analisi dei Risultati

Il presente capitolo è dedicato all'analisi quantitativa e qualitativa delle prestazioni dell'architettura di tracciamento assistita da Reconfigurable Intelligent Surfaces (RIS) in ambiente end-to-end. Al fine di convalidare le ipotesi teoriche discusse nei capitoli precedenti, è stato sviluppato un framework di simulazione ad alta fedeltà (6G Digital Twin Indoor Simulator) basato su Python. Tale framework integra la fisica del canale propagativo a 5.9 GHz, la cinematica del drone basata su Extended Kalman Filter (EKF), e la logica di controllo Software-Defined Networking (SDN) per lo pseudo-layout ottimo delle RIS. La validazione procede secondo un approccio incrementale: partendo dalla definizione puntuale dei collaudi e dei parametri del layer fisico (Sezione 5.1), si analizza il fallimento strutturale dei metodi di localizzazione standard in regime di Non-Line-of-Sight (NLoS) profondo (Sezione 5.2). Successivamente, nella Sezione 5.3, si dimostra il recupero della stima sub-metrica grazie all'attivazione mirata delle risorse RIS. Infine, i test di stress architetturale, che mappano la sensibilità del sistema alla latenza di rete e l'efficienza degli algoritmi predittivi (Sezioni 5.4 e 5.5), delimitano l'inviluppo operativo di questa tecnologia per magazzini intelligenti in ottica "Green 6G".

## 5.1 Setup di Simulazione: Definizione dei layout logistici, tuning dei parametri di filtering e di rete

La robustezza di una simulazione a gemello digitale dipende intrinsecamente dal realismo dei parametri vettoriali, elettromagnetici e cinematici implementati. A tale scopo, l'ambiente di validazione assume il ruolo di una "Single Source of Truth", le cui metriche sono ancorate a standard industriali e protocolli di ricerca accademici sulle reti emergenti.

### 5.1.1 Configurazione degli Assetti Logistici Topologici: Razionale e Sfide
La scelta di operare su tre scenari planimetrici distinti non è puramente incrementale, ma risponde alla necessità di validare la resilienza del sistema a diversi ordini di complessità geometrica e radioelettrica. La varietà dei layout permette di mappare il comportamento dell'architettura 6G in scenari che vanno dalla distribuzione capillare (Smistamento rapido) alla gestione di volumi industriali massivi (Stoccaggio intensivo).

*   **Layout A (Small-Scale)**: Configurazione di 50x40x10 metri, corrispondente a un ambiente logistico di smistamento rapido. Il motivo di questo setup è fornire una baseline di controllo in condizioni di LoS (Line-of-Sight) frequente, dove la densità di ostacoli è minima e la latenza di rete è l'unico vero collo di bottiglia.
*   **Layout B (Medium-Scale)**: Configurazione quadrata da 100x100x10 metri. Questo layout introduce scaffalature addensate e corridoi ortogonali. La razionale di questo scenario è testare la capacità dell'SDN nel gestire i passaggi "angolo-a-angolo", dove il drone subisce uno shadowing intermittente ma prevedibile, tipico dei centri logistici di medie dimensioni.
*   **Layout C (Large-Scale "Long VNA")**: Configurazione industriale estesa pari a 250x140x15 metri. Questo è il cuore pulsante della validazione sperimentale. **Il motivo tecnico dietro l'adozione di questo layout risiede nella simulazione dei corridoi "Very Narrow Aisle" (VNA)**, uno standard nell'automazione logistica moderna (es. magazzini Amazon o centri distributivi di Classe A). 
    Dal punto di vista elettromagnetico, i corridoi VNA fungono da "tunnel metallici" profondi oltre 100 metri che inducono il blocco totale del segnale diretto (regime NLoS puro) non appena il drone si allontana dalla Base Station. Questo scenario rappresenta il *limite fisico* dello stato dell'arte e giustifica tecnicamente l'impiego delle RIS come unica soluzione per garantire la continuità del servizio di localizzazione.

![Figura 5.1: Rappresentazione topologica del Layout C (Large-Scale Warehouse). Sono evidenziati i corridoi VNA, la posizione della Base Station e la distribuzione dei rack metallici che inducono condizioni di shadowing severo.](simulator/Test_0.1_Mappa_Topologica.png)

La modellazione fisica del magazzino prevede ostacoli la cui perdita di penetrazione (Penetration Loss) è stata settata criticamente a $15.0 \text{ dB/m}$ per riflettere le attenuazioni indotte dall'acciaio profilato degli scaffali e dalla densità della merce stoccata. È inoltre garantita una distanza di rispetto (wall spacing) di $2.5 \text{ m}$ in prossimità dei muri portanti.

### 5.1.2 Parametrizzazione Radio-Elettromagnetica (6G-Ready)
Nel contesto delle telecomunicazioni indoor per la robotica autonoma, il drone comunica con le Base Station posizionate strategicamente e con l'infrastruttura RIS distribuita.
*   **Frequenza Operativa**: La rete trasmette su una banda "6G-ready" U-NII-4 centrata su $f_c = 5.9 \text{ GHz}$. Tale frequenza offre un buon trade-off temporale e risoluzione spaziale, garantendo al tempo stesso uno shift tecnologico propedeutico al Sub-THz sensing.
*   **Gestione Outage**: Affinché un pacchetto per la localizzazione (es. misurazione ToF/AoA) possa essere processato correttamente, è stata imposta una soglia di accettazione sul rapporto segnale-rumore pari a $\text{SNR}_{outage} = 5.0 \text{ dB}$. Al di sotto di questa soglia, la Base Station non può validare la misura geometrica, traducendosi in una mancata "observation" (measurement denial) per l'EKF interno al drone o nel Multi-Access Edge Computing (MEC).
*   **Specifiche RIS**: Ogni elemento di superficie riconfigurabile introduce un consumo passivo di sleep rate pari a $0.5 \text{ W}$, che sale a $50.0 \text{ W}$ in stato di eccitazione (Active Mode) programmato dal controllore SDN. Il beamforming garantisce un guadagno teorico stimato in $\approx 20.0 \text{ dB}$ direzionali, compensando ampiamente il rumore di figura ($3.0 \text{ dB}$) introdotto dalla circuiteria dei varactor.

### 5.1.3 Dinamica dell'UAV e Tuning Filtraggio (EKF)
Il calcolo cinetico presuppone un drone quadricottero per trasporto logistico dal peso standard di $1.2 \text{ kg}$. Il bilancio di potenza si aggira sui $150 \text{ W}$ statici in hovering, salendo a $170 \text{ W}$ in fase di traslazione o pitching (roll max 15 gradi). Il budget energetico relativo al solo modem 6G è isolato a $2.0 \text{ W}$.

Il cuore del tracciamento è basato sul filtro di Kalman esteso (EKF). Nell'ottica di garantire le strict requirement in real-time imposte dagli scenari industriali 4.0, il tuning della frequenza di update e sample è di altissimo profilo: il ciclo di predizione-correzione opera a $10 \text{ Hz}$ ($\Delta t = 0.1 \text{ s}$). 
Le matrici di covarianza intrinseche del filtro sono state bilanciate accuratamente: il rumore di processo $\mathbf{Q}$ sconta la non linearità e le folate trasversali durante il volo in VNA, mentre la matrice delle misurazioni $\mathbf{R}$ varia dinamicamente. Quando il drone entra in degradamento SNR, le deviazioni standard di ToF e AoA aumentano parametricamente per riflettere lo scattering e la diffrazione parassita.


## 5.2 Test 1: Analisi Baseline NLoS e Fallimento del Tracciamento 

L'obiettivo cardine della prima validazione sperimentale (Test 1) consiste nella mappatura diagnostica e quantificabile delle lacune strutturali possedute dagli algoritmi predittivi mono-sensore standard in assenza dell'infrastruttura di mitigazione (Assenza di RIS). Il focus è posto sull'osservazione del fenomeno di divergenza della posizione stimata nel dominio del tempo, quando indotto dal blocco persistente della linea di vista (Line-of-Sight - LoS) tra l'Unmanned Aerial Vehicle (UAV) e la Base Station (BS).

### 5.2.1 Diagnosi Ambientale e Intercettazione NLoS
Il test si svolge sul *Layout C (Grande)*, la cui topologia a corridoi lunghi ed estremamente stretti (VNA) massimizza intenzionalmente la durata di esposizione del drone agli artefatti di propagazione. La Base Station è collocata in corsia (Aisle 30), consentendo l'isolamento della dipendenza del segnale sul corridoio adiacente.

La Ground Truth (GT) generata dal simulatore assume un volo misto con logica "Stop-and-Turn": il drone si distacca dalla baia di ricarica, esegue un volo di perimetro lungo l'asse X servendo i corridoi di svincolo per poi immettersi nell'Aisle 31 (X = 131.2 m), navigando parallelamente all'infrastruttura degli scaffali metallici. Già al termine del segmento di transizione, il motore di ray-casting ad alta fedeltà certifica un abbattimento pressoché subitaneo del segnale utile. La massa dei blocchi logistici tra l'antenna ricevente del drone e la Base Station determina la transizione nel regime NLoS puro, bloccando fisicamente e intercettando ogni path primario.

### 5.2.2 Decadimento in "Coasting" ed Esplosione dell'Incertezza (Deriva Spaziale)
La mancanza di Line-of-Sight implica che le query ToF (Time of Flight) arrivino al ricevitore sommerse dal rumore, con segnali pesantemente attenuati ($>-15 \text{ dB/m}$ per ogni blocco) e alterati da log-normal shadow fading. A causa della caduta dell'SNR logaritmico al di sotto della soglia critica di reiezione (outage a $5.0 \text{ dB}$), il drone cessa di fornire observation affidabili (vettori di misura non validi nell'aggiornamento dell'EKF).

A questo punto, l'architettura EKF del twin simulator perde la capacità di calcolare l'Implicit Gain (Il Guadagno di Kalman $\mathbf{K}$). Rimanendo impossibilitato ad accorpare i residui d'innovazione, il filtro degenera operando asincronicamente in pura fase di **Prediction** (tecnica colloquialmente nota nel settore come *coasting*). L'equazione di proiezione dello stato fa affidamento esclusivo sul modello cinematico interno. Sfortunatamente, minime integrazioni del rumore analogico del processo virtuale di spinta unito alle approssimazioni del tempo discreto (drift $\approx 0.15$ costanti e crescenti nel simulatore ad ogni step NLoS) defluiscono nell'equazione di stato.

Visivamente e numericamente, il risultato è un fallimento locale assoluto della localizzazione:
1.  **Divergenza Traiettoria (Spatial Drift)**: La stima dirotta gradualmente la posizione rispetto all'asse del corridoio reale. Il simulatore ha registrato uno spatial drift disarmante, spostando la convinzione del robot in spazi virtuali sfondando persino le topologie degli scaffali. L'errore radicale quadratico medio locale (RMSE) scala da pochi centimetri (stabilità LoS) a deviazioni radiali superiori ai **$8-12$ metri** al momento di raggiungimento teorico dello scaffale target.
2.  **Dilatazione della Matrice di Covarianza (Covariance Explosion)**: Il secondo e drammatico sintomo è la corruzione dell'affidabilità spaziale interna, misurata dalle ellissi di incertezza EKF ($P_k$). Ad ogni step temporale ($\Delta t = 0.1 \text{ s}$), in difetto dello smorzamento correttivo $\mathbf{K} \cdot \mathbf{S} \cdot \mathbf{K}^T$, l'equazione di propagazione dell'errore amplifica illimitatamente la dispersione probabilistica. Per impedire l'occlusione visiva nel render, l'ellisse è costretta asintoticamente al capping radiante a $8.0 \text{ m}$, ben delineando il crollo della convergenza euristica. 

![Figura 5.2: Risultati del Test 1 - Analisi della traiettoria in assenza di RIS. Si osservi la divergenza della stima EKF (linea tratteggiata rossa) rispetto alla Ground Truth e l'esplosione delle ellissi di covarianza (area rosa) all'interno del corridoio NLoS.](simulator/Test_1_TopDown_Traiettoria.png)

Il Test 1 certifica univocamente che, affidandosi esclusivamente alla telemetria radio standard e all'estrapolazione del modello cinematico, è impossibile governare in sicurezza gli UAV in contesti industriali ostili. Questo delinea la necessità intrinseca di un ausilio esterno.


## 5.3 Test 2: Mitigazione tramite Attivazione Dinamica RIS (Assisted-LoS)

Per arginare le violente lacune fisiche del layout industriale illustrate nel Test 1, il Test 2 valida l'integrità del modello di copertura passivo-attiva pilotato dall'SDN Controller, introducendo i pannelli RIS (Reconfigurable Intelligent Surfaces). Lo scopo principale non è semplicemente abbattere l'errore, ma fornire una dimostrazione di robustezza nel tracciamento (resilienza al drift EKF) combinata con i paradigmi di efficienza energetica delle reti di prossima generazione ("Green 6G").

### 5.3.1 Il concetto di Assisted-LoS tramite l'SDN
In questo scenario, il layer logico dell'SDN controller monitora la topologia del magazzino e la probabile direzionalità dei nodi mobili. Non appena l'UAV si posiziona sulla soglia critica che conduce in NLoS (l'attraversamento dello svincolo dell'Aisle 31), il controller orchestra l'irradiazione di rimbalzo:
Il pannello RIS pre-calcolato ed installato sulla parete perimetrale o sull'incrocio strategico più vantaggioso riceve in multicast i pesi di sfasamento (phase shifts) necessari per direzionare e collimare nativamente il fascio di propagazione incidentale proveniente dalla Base Station. Si ottiene così una **Linea di Vista Assistita (Assisted-LoS)**.

Da un punto di vista dell'analisi di radiopropagazione interna al simulatore, l'Assisted-LoS abbatte drammaticamente il Path Loss esponenziale rielaborando le riflessioni come "sorgenti lineari secondarie". Questo espediente rigenera artificialmente un canale geometrico in cui i campioni di Time of Flight e Angle of Arrival oltrepassano il gradiente algoritmico di accettazione imposto a $5.0 \text{ dB}$ (Outage limit).

### 5.3.2 Stabilizzazione dell'RMSE e Riflessi Covarianti
I risultati prestazionali evinti dalla simulazione validano la tecnologia come una Soluzione Strutturale:
1.  **Ripristino Sub-Metrico del Tracciamento**: Pur mantenendo la medesima missione logistica del Test 1 ("Long VNA"), il ricevitore ritorna ad accettare observation valide per la matrice $\mathbf{R}$. L'attivazione del gain teorico del RIS inserisce un'innovazione nel filtro EKF, il cui gradiente di correttivo forza al ribasso l'RMSE. Le metriche di simulazione attestano un *ripristino ai requisiti sub-metrici standard target*, stabilizzando l'errore radiale spaziale solidamente attorno alla soglia di tollerabilità ($\approx 1.0 \text{ m}$ di deviazione media attesa con velocità operative di 3 m/s del drone cargo).
2.  **Smorzamento dell'Incertezza (Abbattimento del transitorio)**: Osservando il rendering covariante in mappa topologica, l'esplosione dei raggi di covarianza vista in assenza di RIS sparisce. Nel momento esatto del re-aggancio visivo assistito, l'ellisse si "restringe" in pochi cicli di update (raffreddamento transitorio di riconvergenza), stringendosi attorno all'asset sino a dimensioni non intralcianti, certificando all'operatore un alto "grado di certezza diagnostica".

![Figura 5.3: Risultati del Test 2 - Mitigazione tramite RIS. Recupero della traiettoria sub-metrica e stabilizzazione delle ellissi di incertezza grazie alla creazione di un canale Assisted-LoS nel corridoio critico.](simulator/Test_2.1_Successo_RIS.png)

![Figura 5.4: Confronto dell'errore RMSE e analisi della Cumulative Distribution Function (CDF) tra lo scenario baseline e lo scenario assistito da RIS. Si noti l'abbattimento del 90° percentile dell'errore sotto la soglia di 1 metro.](simulator/Test_2.2_RMSE_Confronto.png)


## 5.4 Test 3: Stress Test Architetturale (Sensibilità alla Latenza)

Mentre il precedente Test 2 ha validato la robustezza del tracciamento assistito da RIS in condizioni statiche, la trasposizione del sistema in una rete 6G reale introduce una variabile critica: la latenza deterministica dei cicli di controllo. Nel Test 3, si è proceduto a uno stress test dell'architettura variando il parametro del ritardo di rete (Round Trip Time) tra l'SDN Controller e i nodi RIS, al fine di mappare la sensibilità del sistema ai ritardi di processing tipici degli ambienti O-RAN (Open Radio Access Network) e Edge Computing.

### 5.4.1 Beam Misalignment: Il varco del limite fisico e il Breakdown Point
L'analisi sperimentale, visualizzata nel grafico di Figura 5.5, evidenzia una correlazione non lineare tra la latenza di rete ($\Delta t$) e la precisione di posizionamento (RMSE). I risultati permettono di identificare un **Breakdown Point** strutturale fissato a $50 \text{ ms}$.

![Figura 5.5: Stress Test Architetturale - Impatto della latenza di rete sull'errore di posizionamento EKF. Si noti la transizione critica tra la Safe Zone (tracciamento sub-metrico) e la Outage Zone (divergenza causata dal disallineamento dei fasci).](simulator/Test_3_Stress_Latenza.png)

Dall'analisi del set di dati si distinguono due regimi operativi netti:

1.  **Safe Zone ($\Delta t \le 50 \text{ ms}$)**: In questo spettro, il sistema dimostra una resilienza eccezionale. Nonostante la traslazione del drone a $3 \text{ m/s}$, l'errore di posizionamento si mantiene granitico attorno a un valore di **$0.6 \text{ m}$**. Questo valore rappresenta la risoluzione nominale dell'EKF in condizioni di Assisted-LoS, dove il fascio RIS riesce a "inseguire" il ricevitore UAV con un ritardo trascurabile rispetto alla larghezza del lobo principale di radiazione.
2.  **Outage Zone e Beam Misalignment ($\Delta t > 50 \text{ ms}$)**: Valicata la soglia critica dei 50 ms, il sistema entra in una fase di degradamento accelerato. Il fenomeno dominante è il **Beam Misalignment** (disallineamento del fascio): a causa del ritardo nella propagazione del comando di phase-steering, la RIS riflette il segnale verso una coordinata spaziale che il drone ha già superato. 

Numericamente, l'effetto è devastante: per una latenza di $150 \text{ ms}$, l'RMSE raddoppia superando i $1.5 \text{ m}$, mentre allo stress rate massimo di **$250 \text{ ms}$**, l'errore di tracciamento esplode fino a toccare i **$4.8 \text{ metri}$**.

### 5.4.2 Esplosione dell'Errore e Innesco del Coasting
Come osservabile dalla curva rossa in Figura 5.5, l'incremento dell'errore non è lineare ma segue un andamento quasi esponenziale nella Outage Zone. Questo accade perché il disallineamento del fascio provoca un ritorno forzato al regime di **EKF Coasting**: non ricevendo più riflessioni valide a causa del puntamento errato, il filtro di Kalman perde le misurazioni correttive e riprende la divergenza osservata nel Test 1.

Questi risultati definiscono per la prima volta l'inviluppo operativo della soluzione proposta: per garantire un tracciamento sub-metrico di sicurezza in magazzini industriali automatizzati, l'infrastruttura 6G deve garantire un budget di latenza della control-plane rigorosamente inferiore ai **50 ms**. Oltre questa soglia, l'architettura reattiva collassa, rendendo necessaria una compensazione predittiva come discusso nel capitolo successivo.


## 5.5 Test 4: Ottimizzazione Avanzata (Controllo Predittivo vs. Reattivo)

Esaurita la comprensione dei vincoli dettati dalla latenza in Control-Loop classici, il Test 4 chiude la proposta architetturale con l'analisi teorica e prestazionale della "Compensazione Predittiva". Non è ragionevole aspettarsi che una rete complessa abbatta l'RTT a latenze millimetriche su tutta la linea, ma è ingegneristicamente solido "ingannare" la latenza sfruttando il filtro per alterare il pattern di azionamento (Dall'approccio Reattivo Inefficiente, a un approccio Predittivo Intelligente).

### 5.5.1 Previsione Algoritmica dell'Innesco (Zeroing della Latenza Apparente)
La logica standard di intercettazione innesca un loop di chiamata RPC (Remote Procedure Call) al driver del pannello RIS unicamente nel momento in cui la Base Station dichiara il drop dell'SNR sotto la soglia di out-of-order. Il tempo morto trascorso per instradare messaggi all'SDN costa il discusso "Beam Misalignment".
Il paradigma di **Controllo Predittivo** avvia una fusione bidirezionale non convenzionale tra i nodi dell'infrastruttura. L'SDN, processando il vettore di stato stimato dell'EKF $(\hat{x}, \hat{y}, \hat{v}_x, \hat{v}_y)$, possiede l'intrinseca conoscenza cinematica sull'imminenza temporale dell'entrata nel vicolo cieco del Layout. 

Il sistema può dunque calcolare il tempo speso per la comunicazione gRPC fino al backhaul e sottrarlo dalla finestra di invio logico, inviando il pacchetto di phase-steering SDN in anticipo. Per il ricevitore UAV, il salto è istantaneo: la variazione di fase e riflessione del metallo scatta "esattamente" all'arrivo alla soglia del blocco NLoS, mascherando di netto i ritardi insormontabili, e portando alla teorica "Latenza Apparente Nulla". L'influsso dell'Outage Zone si assottiglia sino all'infinitesimo.

### 5.5.2 "Green 6G": Gestione dello Sleep-Cycle e Analisi Energetica
Questa ottimizzazione apporta non solo garanzie dirette per le curve d'errore (che evitano permanentemente il coasting divergenza), ma un beneficio essenziale sul layer ecologico: The Green Shift.
Declinando la RIS sul setup iniziale (Sezione 5.1), asservendo $50 \text{ W}$ attivi ma soli $0.5 \text{ W}$ statici per pannello spento, la temporizzazione dell'edge compute si erge ad arbitro di risparmio. Sapendo precisamente il varco d'ingaggio, il controllore destina l'attivazione della RIS circoscritta all'esatto "Duty Cycle" tracciato per la prefezione del robot, permettendo al silicio di rimanere nel profondo rate-sleep per oltre il $95\%$ del tempo di missione idle e tagliare i consumi cumulativi di decine di Watt su ordini di grandezza aziendale. Questo consacra le potenzialità della tecnologia a pieno titolo nel vertice dell'ecosistema Green 6G.

![Figura 5.6: Analisi dell'efficienza energetica e del duty-cycle operativo. Confronto tra attivazione statica e attivazione predittiva pilotata dall'SDN in ottica Green 6G.](simulator/Test_4_Efficiency_Green.png)
