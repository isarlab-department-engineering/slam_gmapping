# slam_gmapping_didactic - note di sviluppo

Versione didattica del nodo ROS 2 di GMapping. La libreria `openslam_gmapping`
non e' modificata: tutto passa dalla classe derivata `DidacticGridSlamProcessor`
(in `include/slam_gmapping_didactic/slam_gmapping.h`), che ridefinisce i tre
metodi chiamati dalla libreria nel ciclo del filtro, e dal nodo `SlamGmapping`
(in `src/slam_gmapping.cpp`). Le aggiunte nel codice sono marcate con
`// --- Versione didattica ---`.

## Passi di sviluppo

1. **Particelle in RViz2** - `/gmapping/particles` (PoseArray, a ogni scansione).
   Conversione dalla posa del "laser centrato" usata da GMapping alla posa del
   robot (per il TurtleBot3 il laser centrato e' ruotato di pi).
2. **Parametri e preset** - parametri dell'algoritmo esposti come parametri ROS;
   preset in `config/` selezionati con `config:=<nome>`.
3. **Tre fasi e numeri** - istantanee `update/step1_predicted`, `step2_matched`,
   `step3_resampled`; `neff`, `neff_threshold`, `resampled`, `num_particles`,
   `unique_particles`; testo riassuntivo `info`; particelle sollevate
   (`particles_z`); configurazione RViz2 `rviz/didattico.rviz`; file degli
   esperimenti `config/exp_*.yaml`.
4. **Mappe delle particelle** - mappe, traiettorie e pesi della particella con
   peso attuale massimo e minimo, catturati in `onScanmatchUpdate()`:
   `map_best_particle`, `map_worst_particle` (OccupancyGrid),
   `trajectory_best_particle`, `trajectory_worst_particle` (Path),
   `particle_weights` (MarkerArray), `weight_max`, `weight_min`.
   Parametri `publish_particle_maps`, `particle_maps_full` e `map_best_green`
   (ostacoli della mappa migliore pubblicati con valore 120, che RViz2 disegna in
   verde; solo per visualizzazione).

5. **Scan matching, albero, incertezza, discordanze, tempi, chiusura del ciclo** -
   `scanmatch` (MarkerArray: correzioni previsto -> corretto per ogni particella,
   scansione proiettata dalla posa prevista e corretta della particella migliore),
   `trajectory_tree` (albero delle traiettorie, colore = particelle discendenti),
   `coalescence_depth`, `uncertainty` (ellisse), `std_xy`, `std_theta_deg`,
   `map_disagreement` e `map_disagreement_cells`, `update_time_ms`,
   `didactic_time_ms`. Parametri `publish_tree`, `ellipse_sigma`, `ogain`.
   Mondo `worlds/anello.world` e launch `gazebo_mondo.launch.py`
   (`./start_gazebo.sh anello`), esperimento `exp_chiusura_ciclo`.

6. **Odometria con deriva** - nodo Python `scripts/odom_drift.py`: legge la TF
   vera odom -> base_footprint, integra gli spostamenti con errori di scala,
   deriva dell'orientamento e rumore, e pubblica odom_noisy -> odom (cosi'
   odom_noisy -> base_footprint = posa disturbata). Con `odom_drift: true` il
   nodo GMapping usa odom_noisy come frame dell'odometria e il launch avvia
   odom_drift con i parametri `drift_*` letti dalla stessa sezione del file di
   configurazione. Topic `gmapping/map_odometry` (mappa costruita con la sola
   odometria), `odom_drift/path_true`, `odom_drift/path_noisy`. Esperimenti
   `exp_odometria_deriva` (E10) e `exp_chiusura_ciclo_deriva` (E11).

## Punti chiave del passo 4

- Ogni particella ha la propria mappa (`Particle::map`). Per efficienza GMapping
  vi registra solo i punti colpiti dal laser (`generateMap = false`): e' la mappa
  usata nello scan matching. La mappa `/map` invece viene ricostruita dal nodo
  lungo la traiettoria di una particella, con lo spazio libero.
- Con `particle_maps_full: true` le mappe delle particelle con peso max e min
  vengono ricostruite allo stesso modo di `/map` (scansioni memorizzate nei nodi
  dell'albero delle traiettorie); con `false` si pubblica la mappa interna.
- Le mappe sono catturate subito dopo lo scan matching e prima del
  ricampionamento: non contengono ancora la scansione corrente, cioe' sono
  esattamente le mappe su cui la particella e' stata giudicata. Vanno catturate
  li' perche' dopo il ricampionamento i nodi delle particelle eliminate vengono
  cancellati.
- Due nozioni di peso: il peso attuale (dall'ultimo ricampionamento, decide Neff
  e il ricampionamento) e il peso cumulativo `weightSum` (somma su tutta la
  traiettoria). "Peso max" e "peso min" usano il peso attuale; `/map` usa la
  particella con peso cumulativo massimo (`getBestParticleIndex()`). Il testo in
  RViz2 indica se coincidono.
- I pesi normalizzati usano la formula di `normalize()`: esp(gain*(w - w_max)),
  con gain = 1/(obsSigmaGain * N), che "addolcisce" i pesi.

## Punti chiave del passo 5

- Coalescenza: si risale l'albero a passi uguali da tutte le particelle finche'
  resta un solo nodo; i livelli sono allineati perche' ogni aggiornamento aggiunge
  un nodo per particella. Senza ricampionamento coincide con l'intera storia.
- Tempo del filtro = tempo di elaborazione della scansione meno il tempo speso
  nell'aggancio per le mappe didattiche (`didactic_ms`).
- L'ellisse usa le posizioni del robot secondo le particelle (non pesate) a ogni
  scansione; l'angolo e' trattato come grandezza circolare.
- `ogain` e' l'obsSigmaGain di GMapping: divide i pesi logaritmici.
- Discordanze: 120 (verde) occupato solo per la migliore, 100 (viola) solo per la
  peggiore, 0 altrimenti; RViz2 con schema costmap rende trasparente lo 0.

## Punti chiave del passo 6

- Albero TF con deriva attiva: map -> odom_noisy -> odom -> base_footprint.
  odom ha un solo genitore (odom_noisy), quindi l'albero resta valido.
- La mappa della sola odometria registra le scansioni nelle pose date
  dall'odometria agli istanti degli aggiornamenti; il frame map coincide
  inizialmente con quello dell'odometria, quindi e' sovrapponibile a /map.
- Prova con simulatore minimale sul mondo anello (giro completo, 9
  ricampionamenti): con deriva 0.03 rad/m e scala rotazioni +5% l'odometria
  sbaglia di circa 3 m e 34 gradi a fine giro, la stima di GMapping di circa
  2 cm e meno di 1 grado; con 0.015 rad/m e +3%, 1.4 m e 15 gradi contro 2 cm
  e 1 grado.

## Verifiche

La classe `DidacticGridSlamProcessor` e' stata provata fuori da ROS con la
libreria GMapping su una stanza simulata (con e senza ricampionamento, anche con
AddressSanitizer). Dal passo 5 il pacchetto e' stato anche compilato con colcon
in un ambiente ROS 2 Humble (RoboStack) e il nodo eseguito con un simulatore
minimale che percorre l'anello pubblicando /scan e /tf: tutti i topic vengono
pubblicati. Il launch di Gazebo per il mondo anello va verificato nel container.
