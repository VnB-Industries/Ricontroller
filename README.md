# Ricardo - Controller Firmware

Firmware Arduino pour piloter plusieurs pompes via commandes serie et lire les badges RFID.

- Carte cible: Arduino Uno (ATmega328P)
- Fichier principal: `Ricontroler/Ricontroler.ino`
- Liaison avec l'IHM: USB serie 9600 bauds
- Nombre de pompes supporte: 2 a 6 (`MAX_PUMPS=6`)

## Role du firmware

Le controleur:

- recoit des commandes de dosage depuis l'IHM (`DISPENSE` variable-length),
- recoit des commandes d'etalonnage pompe (`CAL:pompe:quantite_cL`),
- recoit les constantes de debit calculees par l'IHM (`SETFLOW` variable-length),
- pilote les sorties pompes declarees,
- mesure le volume de la pompe 1 via capteur de debit,
- estime le volume des pompes 2..N via debit moyen (temps),
- expose la lecture RFID sous forme de trames serie.

## Materiel et brochage

- Capteur de debit YF-S401 (interruption): pin D2
- Capteur presence verre (IR): pin D4
- RFID PN532 (Grove NFC shield, SPI):
  - CS/SS par defaut sur D10 (option D9 selon strap du shield)
  - bus SPI via connecteur ICSP (MISO/MOSI/SCK)
  - IRQ non utilise (mode polling)

Brochage pompes (`MAX_PUMPS=6`):

- Pompe 1: D7 (debitmetre)
- Pompe 2: D6
- Pompe 3: D5
- Pompe 4: D8
- Pompe 5: D9
- Pompe 6: D3

## Prerequis logiciels

- Arduino IDE (ou Arduino CLI)
- Bibliotheque Arduino: `Adafruit PN532`

## Flash manuel

```bash
arduino-cli compile --fqbn arduino:avr:uno controler/Ricontroler
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno controler/Ricontroler
```

## Protocole serie

Configuration port serie:

- Vitesse: `9600`
- Fin de ligne: `\n` (ou `\r\n`)

### Commandes entrantes

- `DISPENSE:<p1_cL>:<p2_cL>[:<p3_cL>...]`
  - Exemple a 4 pompes: `DISPENSE:3:12:0:5`
  - 1 a `MAX_PUMPS` valeurs autorisees.
  - Valeurs flottantes >= 0.
  - Au moins une valeur doit etre > 0.
- `CAL:<pump_index>:<quantity_cL>`
  - Exemple: `CAL:3:20`
  - `pump_index` dans `[1..MAX_PUMPS]`.
  - Lance un mode etalonnage: la pompe selectionnee tourne jusqu'au retrait du verre.
- `SETFLOW:<p1_pulses_per_cL>:<p2_cLps>[:<p3_cLps>...]`
  - Exemple a 4 pompes: `SETFLOW:540.420:2.800:2.950:2.700`
  - Premiere valeur: constante pompe 1 (pulses/cL).
  - Valeurs suivantes: constantes des pompes 2..N (cL/s).
  - Ces valeurs ne sont pas ecrites en EEPROM: elles doivent etre renvoyees par l'IHM au demarrage.
- `STOP`
  - Arret immediat d'un dosage ou d'une calibration en cours.

### Reponses sortantes

- `READY`
  - Firmware pret apres boot.
- Distribution (N valeurs):
  - `STARTED:<v1>:<v2>...`
  - `PROGRESS:<v1>:<v2>...`
  - `OK:<v1>:<v2>...`
  - `STOPPED:<v1>:<v2>...`
  - `ERROR:...:<v1>:<v2>...` selon le contexte
- Calibration:
  - `CAL:STARTED:<pump_index>:<quantity_cL>`
  - `PROGRESS:<value>`
  - `CAL:STOPPED_BY_SENSOR:<pump_index>:<constant>`
  - `CAL:STOPPED:<pump_index>:<constant>`
- Mise a jour flow:
  - `SETFLOW:OK:<p1_pulses_per_cL>:<p2_cLps>...`
- Erreurs communes:
  - `ERROR:BUSY`
  - `ERROR:INVALID_VOLUME`
  - `ERROR:INVALID_CALIBRATION`
  - `ERROR:INVALID_CONSTANTS`
  - `ERROR:NOT_DISPENSING`
  - `ERROR:UNKNOWN_COMMAND`
  - `ERROR:TIMEOUT` (timeout securite capteur debit pompe 1)
- RFID:
  - `RFID:<UID_HEX>`
  - `NFC:NO_SHIELD`

## RFID

- Le RFID est scrute uniquement hors dosage (`dispensing == false`).
- Anti-rebond: meme badge ignore pendant 3 secondes.
- Trame envoyee: `RFID:<UID_HEX>` (exemple `RFID:04A1B2C3D4`).

## Calibration / constantes utiles

Constantes runtime:

- `pulsesPerCl` pour pompe 1
- `timeFlowClPerSec[i]` pour pompes 2..N

Valeurs par defaut au boot:

- `pulsesPerCl = 540.42`
- `defaultPump2FlowClPerSec = 2.8` (utilisee pour initialiser pompes 2..N)
- `expectedMinFlow_Lmin = 0.2`

Ajustement pendant calibration:

- Pompe 1: `pulsesPerCl = pulseCount / quantite_cL`
- Pompes 2..N: `flowClPerSec = quantite_cL / duree_secondes`

Important: la source de verite est l'IHM. Le firmware applique les constantes recues via `SETFLOW` mais ne les persiste pas hors redemarrage.

## Test rapide depuis un PC Linux

Terminal 1 (monitor):

```bash
stty -F /dev/ttyACM0 9600 -echo -onlcr
cat /dev/ttyACM0
```

Terminal 2 (commande):

```bash
echo 'DISPENSE:2:5:0:0' > /dev/ttyACM0
```

Arret:

```bash
echo 'STOP' > /dev/ttyACM0
```

## Depannage

- Pas de `READY` au boot:
  - verifier carte/port selectionnes,
  - verifier cable USB data (pas charge-only).
- `NFC:NO_SHIELD`:
  - verifier alimentation du shield,
  - verifier cablage SPI et strap SS (D10 par defaut),
  - verifier antenne sur bornier ANT1,
  - verifier la bibliotheque `Adafruit PN532`.
- `ERROR:TIMEOUT` sur pompe 1:
  - verifier capteur de debit et cablage D2,
  - ajuster `expectedMinFlow_Lmin`.
- Volumes incorrects:
  - recalibrer la pompe cible depuis l'IHM,
  - verifier que `SETFLOW` est bien envoye par l'IHM au demarrage.
