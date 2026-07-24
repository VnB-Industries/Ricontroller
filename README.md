# Ricardo - Controller Firmware

Firmware Arduino pour piloter deux pompes via commandes serie et lire les badges RFID.

- Carte cible: Arduino Uno (ATmega328P)
- Fichier principal: `Ricontroler/Ricontroler.ino`
- Liaison avec l'IHM: USB serie 9600 bauds

## Role du firmware

Le controleur:

- recoit des commandes de dosage depuis l'IHM (`DISPENSE:x:y`),
- recoit des commandes d'etalonnage pompe (`CAL:pompe:quantite_cL`),
- recoit les constantes de debit calculees par l'IHM (`SETFLOW:p1:p2`),
- pilote deux sorties pompes,
- mesure le volume de la pompe 1 via capteur de debit,
- estime le volume de la pompe 2 par debit moyen,
- expose la lecture RFID sous forme de trames serie.

## Materiel et brochage

- Capteur de debit YF-S401 (interruption): pin D2
- Relais NO Pompe 1: pin D7
- Relais NO Pompe 2: pin D6
- RFID PN532 (Grove NFC shield, SPI):
  - CS/SS par defaut sur D10 (option D9 selon strap du shield)
  - bus SPI via connecteur ICSP (MISO/MOSI/SCK)
  - IRQ non utilise (mode polling)

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

- `DISPENSE:<pump1_cL>:<pump2_cL>`
  - Exemple: `DISPENSE:2:20`
  - Les volumes sont des flottants positifs ou nuls.
  - Les deux volumes ne peuvent pas etre nuls en meme temps.
- `CAL:<pump_index>:<quantity_cL>`
  - Exemple: `CAL:1:20`
  - `pump_index` vaut `1` (pompe 1) ou `2` (pompe 2).
  - Lance un mode etalonnage: la pompe selectionnee tourne jusqu'au retrait du verre.
  - Le capteur de presence (IR) stoppe la calibration quand le verre est retire.
- `SETFLOW:<p1_pulses_per_cL>:<p2_cL_per_s>`
  - Exemple: `SETFLOW:540.420:2.800`
  - Met a jour les constantes runtime appliquees par le firmware.
  - Ces valeurs ne sont pas ecrites en EEPROM: elles doivent etre renvoyees par l'IHM au demarrage.
- `STOP`
  - Arret immediat d'un dosage ou d'une calibration en cours.

### Reponses sortantes

- `READY`
  - Firmware pret apres boot.
- `STARTED:<p1_req>:<p2_req>`
  - Dosage lance.
- `PROGRESS:<p1_mesure>:<p2_estime>`
  - Emis periodiquement pendant le dosage.
- `OK:<p1_final>:<p2_final>`
  - Dosage termine normalement.
- `STOPPED:<p1_final>:<p2_final>`
  - Dosage interrompu par `STOP`.
- `CAL:STARTED:<pump_index>:<quantity_cL>`
  - Calibration lancee.
- `CAL:STOPPED_BY_SENSOR:<p1_pulses_per_cL>:<p2_cL_per_s>`
  - Calibration terminee par retrait du verre.
  - Le firmware calcule et applique les nouvelles constantes avant emission.
- `CAL:STOPPED:<p1_pulses_per_cL>:<p2_cL_per_s>`
  - Calibration interrompue par `STOP`.
- `SETFLOW:OK:<p1_pulses_per_cL>:<p2_cL_per_s>`
  - Confirmation de mise a jour des constantes runtime.
- `ERROR:BUSY`
- `ERROR:INVALID_VOLUME`
- `ERROR:INVALID_CALIBRATION`
- `ERROR:INVALID_CONSTANTS`
- `ERROR:NOT_DISPENSING`
- `ERROR:UNKNOWN_COMMAND`
- `ERROR:TIMEOUT`
  - Timeout de securite du capteur de debit (pompe 1).
- `RFID:<UID_HEX>`
  - UID RFID detecte en hex uppercase.
- `NFC:NO_SHIELD`
  - Shield PN532 non detecte (le mode pompes reste fonctionnel).

## RFID

- Le RFID est scrute uniquement hors dosage (`dispensing == false`).
- Anti-rebond: meme badge ignore pendant 3 secondes.
- Trame envoyee: `RFID:<UID_HEX>` (exemple `RFID:04A1B2C3D4`).

## Calibration / constantes utiles

Le firmware expose deux constantes runtime:

- `pulsesPerCl` (pompe 1)
- `pump2FlowClPerSec` (pompe 2)

Valeurs par defaut au boot:

Dans `Ricontroler.ino`:

- `pulsesPerCl = 540.42`
- `pump2FlowClPerSec = 2.8`
- `expectedMinFlow_Lmin = 0.2`

Ajustement pendant calibration:

- Pompe 1: `pulsesPerCl = pulseCount / quantite_cL`
- Pompe 2: `pump2FlowClPerSec = quantite_cL / duree_secondes`

Important: la source de verite est l'IHM. Le firmware applique les constantes recues via `SETFLOW` mais ne les persiste pas hors redemarrage.

## Test rapide depuis un PC Linux

Terminal 1 (monitor):

```bash
stty -F /dev/ttyACM0 9600 -echo -onlcr
cat /dev/ttyACM0
```

Terminal 2 (commande):

```bash
echo 'DISPENSE:2:5' > /dev/ttyACM0
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
  - recalibrer `pulsesPerCl` (pompe 1) et `pump2FlowClPerSec` (pompe 2).
