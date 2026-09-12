# ESP32_AGOPEN – AgOpenGPS Autosteer Firmware (ESP32)

Ez a projekt egy **ESP32-WROOM-32** alapú autosteer (automata kormányzás) firmware az [AgOpenGPS](https://github.com/AgOpenGPS-Official) rendszerhez. Feladata: a GPS vevő NMEA adatainak feldolgozása, opcionális IMU (BNO08x) tájolás-számítás, a kormánymotor PID-alapú szabályozása, és mindezek kommunikálása az AgIO (PC-s AgOpenGPS kliens) felé Soros vagy WiFi/UDP kapcsolaton keresztül.

A build rendszer **PlatformIO** (`platformio.ini`, `env:esp32dev`).

---

## WiFi használata röviden

Az ESP32 kétféleképpen csatlakozhat a WiFi hálózathoz:

- **AP mód (ajánlott első beállításhoz):** az ESP32 saját WiFi hálózatot hoz létre `AGOPEN_ESP32_AP` néven. Csatlakozz ehhez a hálózathoz a `12345678` jelszóval, majd nyisd meg a `http://192.168.4.1/` címet.
- **STA mód:** az ESP32 a meglévő helyi WiFi hálózathoz csatlakozik. Ehhez a WiFi nevét és jelszavát a webes konfigurációs oldalon kell megadni.

Ha STA módban az ESP32 **60 másodpercen belül nem tud csatlakozni**, automatikusan visszavált AP módba. Ilyenkor az AP hálózat az ESP32-ben beállított AP névvel és jelszóval jelenik meg, és a készülék ismét a `192.168.4.1` címen érhető el.

A webes konfigurációs oldal használatával beállítható az AP/STA mód, a WiFi név és jelszó, az AP csatornája, valamint az AgOpenGPS UDP portja. Az AgOpenGPS adatkommunikáció alapértelmezett portja `8888`, az RTCM korrekciós adatok fogadására szolgáló port pedig `2233`. A beállítások mentés után újraindításkor is megmaradnak.

---

## 1. Architektúra

- **Platform**: ESP32-WROOM-32, dual-core Xtensa LX6 @ 240 MHz, Arduino keretrendszer + FreeRTOS.
- **Core 0**: a soros csomagparser task (`autoSteerPacketPerser`, prioritás 25) és – ha engedélyezve van – a WiFi stack.
- **Core 1**: háttér ADC-olvasó task (`adcTaskFunction`, prioritás 1) és a soros/UDP küldő taskok (prioritás 10).
- **`loop()`** felépítése ([src/main.cpp](src/main.cpp)):
  - `imuTask()` – minden iterációban lefut, kiüríti a BNO08x belső puffert (nincs benne trigonometria, csak nyers kvaternion mentés).
  - `gpsStream()` – nem blokkoló, karakterenkénti NMEA beolvasás a `Serial2`-ről (max. 64 karakter/iteráció).
  - `updateADCCacheAsync()` – csak jelzi, a tényleges ADC-olvasás a háttér taskban történik.
  - `t_inputSwitches` (5 Hz) – kapcsolók beolvasása.
  - `t_autosteerLoop` (`AUTOSTEER_INTERVAL`, alapból 50 Hz) – PID szabályozási ciklus.

A cél a **soha nem blokkoló főciklus**: minden I2C/Serial/UDP művelet vagy külön taskban, vagy karakter-/csomag-korlátozott adagokban fut.

### Fájlstruktúra

| Fájl | Feladat |
|---|---|
| [src/main.cpp](src/main.cpp) | Setup, fő ciklus, task létrehozás, EEPROM inicializálás |
| [src/zHandlers.cpp](src/zHandlers.cpp) / [include/zHandlers.h](include/zHandlers.h) | NMEA GGA/VTG kezelők, BNO08x IMU mintavételezés és tájolás-számítás |
| [src/zPackets.cpp](src/zPackets.cpp) / [include/zPackets.h](include/zPackets.h) | AgOpenGPS bináris PGN protokoll fel-/lebontása |
| [src/zSerial.cpp](src/zSerial.cpp) / [src/zUDP.cpp](src/zUDP.cpp) | Soros és opcionális WiFi/UDP kommunikáció (queue-alapú, aszinkron küldés) |
| [src/zAutosteer.cpp](src/zAutosteer.cpp) / [src/AutosteerPID.cpp](src/AutosteerPID.cpp) | Kormányszög-számítás, PID szabályozó, motorvezérlés |
| [src/zInput.cpp](src/zInput.cpp) | ADS1115 ADC nem blokkoló olvasása (kormányszög-érzékelő, nyomás/áram szenzor) |
| [src/zSpeedImpulse.cpp](src/zSpeedImpulse.cpp) | Opcionális sebesség-impulzus kimenet (hardver timer alapú) |
| [src/CyclicTimer.cpp](src/CyclicTimer.cpp) | Egyszerű, nem blokkoló periodikus időzítő |
| [include/Configuration.h](include/Configuration.h) | Az összes hangolható konstans (pin-kiosztás, PID paraméterek, WiFi, stb.) |

---

## 2. Kommunikáció (AgIO protokoll)

Az AgOpenGPS bináris PGN csomagformátumát használja:

```
[0x80][0x81][Forrás][PGN][Hossz][Adat...][Checksum]
```

Fontosabb PGN-ek ([src/zPackets.cpp](src/zPackets.cpp)):

| PGN | Irány | Tartalom |
|---|---|---|
| 0xFE (254) | AgIO → board | Kormány célszög, GPS sebesség, guidance állapot |
| 0xFD (253) | board → AgIO | Tényleges kormányszög, kapcsoló állapotok, PWM kijelzés |
| 0xFA (250) | board → AgIO | Nyomás/áram szenzor adat (minden 3. ciklusban) |
| 0xFC (252) | AgIO → board | PID hangolás (Kp, highPWM, lowPWM, minPWM, Ackerman, WAS offset) |
| 0xFB (251) | AgIO → board | Konfigurációs bitek (InvertWAS, CytronDriver, Danfoss, stb.) |
| 0xC8 (200) | AgIO → board | „Hello” handshake |
| 0xCA (202) | board → AgIO | Scan válasz (helyi IP, ha WiFi aktív) |

**Kettős csatorna**: soros porton mindig fut a kommunikáció; opcionálisan (`ENABLE_UDP`) egy `WiFiUDP`-alapú WiFi kapcsolat is aktiválható AP vagy STA módban, saját küldő-sorral és háttér taskkal.

## 3.1. WiFi és UDP beállítások

A WiFi konfigurációt az ESP32 NVS memóriában tárolja a firmware. A beállítások a webes konfigurációs oldalon módosíthatók a készülék IP-címén, és újraindítás után is megmaradnak.

### Alap WiFi beállítások

Az alapértékek az [include/Configuration.h](include/Configuration.h) fájlban találhatók:

| Beállítás | Alapérték | Jelentés |
|---|---:|---|
| `ENABLE_WIFI_CONFIG` | `1` | WiFi és webes konfiguráció engedélyezése |
| `ENABLE_UDP` | `1` | AgOpenGPS UDP kommunikáció engedélyezése |
| `WIFI_MODE` | `1` | `1` = AP mód, `0` = STA mód |
| `WIFI_SSID` | `AGOPEN_ESP32_AP` | Alap AP SSID és kezdeti STA SSID |
| `WIFI_PASS` | `12345678` | Alap AP jelszó és kezdeti STA jelszó |
| `WIFI_CHANNEL` | `6` | AP mód WiFi csatornája, 1–13 |
| `UDP_PORT` | `8888` | AgOpenGPS adatkommunikáció UDP portja |
| `WEB_SERVER_PORT` | `80` | Webes konfigurációs oldal portja |

Az AP és STA SSID/jelszó, az üzemmód, az AP csatornája és az AgOpenGPS UDP portja a webes felületen külön is beállítható. Az AP mód alapértelmezett címe `192.168.4.1`, a konfigurációs oldal alapértelmezett URL-je: `http://192.168.4.1/`.

### AP és STA mód működése

- **AP mód**: az ESP32 saját WiFi hálózatot hoz létre. Ez általában a legalacsonyabb késleltetésű működés, és a készülék az `192.168.4.1` címen érhető el.
- **STA mód**: az ESP32 a beállított külső WiFi hálózathoz csatlakozik. A csatlakozási kísérlet legfeljebb **60 másodpercig** tart.
- **Automatikus fallback**: ha a STA kapcsolat 60 másodpercen belül nem jön létre, a firmware automatikusan AP módba vált. Ilyenkor az AP SSID-jét, jelszavát és csatornáját használja, és továbbra is a `192.168.4.1` címen érhető el.

A WiFi energiatakarékos módja ki van kapcsolva az alacsonyabb kommunikációs késleltetés érdekében. A TX teljesítmény és a WiFi RX/TX pufferek értékei szintén az [include/Configuration.h](include/Configuration.h) fájlban állíthatók.

### RTCM/NTRIP továbbítás

Az RTCM korrekciós adatok számára külön UDP fogadó port működik:

- **RTCM UDP port:** `2233`
- **Irány:** bejövő UDP `2233` → `Serial2`
- **Serial2:** `115200 baud`, RX `GPIO16`, TX `GPIO17`
- Az RTCM fogadás külön, alacsony prioritású FreeRTOS taskban fut, ezért az autosteer időkritikus feladatai elsőbbséget kapnak.

Ez a csatorna minden beérkező UDP csomagot közvetlenül a `Serial2` kimenetre ír. A `Serial2` egyben a GPS NMEA bemenete is, ezért a bekötött GPS vevőnek támogatnia kell az RTCM korrekciós adatok fogadását ugyanazon a soros kapcsolaton. Az AgOpenGPS/PGN 215 útvonalon érkező RTCM adatok továbbra is támogatottak.

---

## 4. GPS feldolgozás

- `NMEAParser<2>` (könnyűsúlyú, header-only parser) dolgozza fel a `G-GGA` és `G-VTG` mondatokat egyetlen soros GPS bemeneten (`Serial2`, 115200 baud).
- `GGA_Handler()` minden GGA mondat után meghívja a `calculateIMU(micros())`-t, majd összeállítja és elküldi a PGN adatcsomagot (`buildnmeaPGN()`).
- **NTRIP/RTCM átjátszás implementálva van**: a PGN 215 csomagban érkező RTCM korrekciós adatot a firmware közvetlenül a `Serial2`-re (GPS vevő RX bemenetére) írja ki ([src/zPackets.cpp](src/zPackets.cpp), `case 215`).
- Nincs dupla-GPS (pozíció + fejirány vevő) támogatás, nincs automatikus baud-rate detektálás.

## 5. IMU kezelés (BNO08x) – GPS-szinkronizált tájolás

Ez a modul kapta a legtöbb egyedi fejlesztést ebben a firmware-ben:

- `imuTask()` minden `loop()` iterációban lefut, és **egy `while` ciklusban teljesen kiüríti** a BNO08x jelentés-sorát (Adafruit_BNO08x könyvtár, `SH2_GAME_ROTATION_VECTOR`, 100 Hz mintavétel). Itt **nincs trigonometria** – csak a nyers kvaternion + időbélyeg (`sensorValue.timestamp`, µs, azonos időalap a `micros()`-szal) mentése történik `imuPrev`/`imuCurr`-ba.
- `GGA_Handler()` a GGA mondat feldolgozásának pontos pillanatában (`micros()`) hívja a `calculateIMU()`-t, amely:
  1. Kiugrás-szűrést végez (`2*acos(dot)` szögkülönbség vs. megengedett max. szögsebesség × eltelt idő) – ha a legutóbbi minta fizikailag valószínűtlen ugrást mutat, visszaesik az előző mintára.
  2. **SLERP interpolációval/extrapolációval** (gömbi lineáris interpoláció) kiszámítja, milyen orientáció lett volna *pontosan* a GPS-fix pillanatában, a két legutóbbi IMU-minta között.
  3. Az eredményt egyszer alakítja Euler-szögekké (`quaternionToEuler`) – yaw, roll, pitch egyszerre, ugyanabból a szinkronizált kvaternionból.
- Ez azt jelenti, hogy a 10 Hz-es GPS-fixhez mindig egy, a fix pillanatára időben pontosan illesztett IMU-orientáció tartozik – nem egy 0–100 ms-mal korábbi, esetlegesen elavult minta.
- Nincs kettős antenna GPS-fúzió (a hivatalos Teensy kódban ez is csak részben, kikommentezve van jelen), nincs kompasz (CMPS14) alternatíva.

## 6. Autosteer / PID szabályozás

- 50 Hz-es (`AUTOSTEER_INTERVAL`) ciklusban fut ([src/zAutosteer.cpp](src/zAutosteer.cpp)): csomag-időtúllépés figyelése (steerEnable kikapcsolása, ha 1 mp-ig nem jön 0xFE csomag), kormányszög-számítás, motor állapotgép, majd PID.
- Két mód létezik ([src/AutosteerPID.cpp](src/AutosteerPID.cpp)):
  - **P-only** (alap, megegyezik a hivatalos Teensy logikájával: holtsáv-interpoláció `lowPWM`→`highPWM` között, minimum PWM hozzáadás a súrlódás legyőzésére).
  - **Auto-Tune PID** (`USE_AUTOTUNE_PID`, opcionális kiegészítés): teljes P+I+D szabályzás, ahol a **D tag automatikusan tanul** zérus-átmenet (oszcilláció) detektálással 500 ms-os ablakokban, és 5 percenként (vagy motor-kikapcsoláskor) EEPROM-ba mentődik. Ez a funkció **nincs meg** a hivatalos Teensy firmware-ben.
- Motorvezérlés: Cytron MD30C (irány+PWM) vagy IBT-2 (kettős PWM) meghajtó, 20 kHz / 10 bit felbontású LEDC PWM.
- A `Setup` struktúrában van `IsDanfoss` és `ShaftEncoder` konfigurációs bit (EEPROM-kompatibilitás a Teensy-vel), de a hozzá tartozó **PWM-leképezés (Danfoss) és a kerékenkóder-megszakítás (ShaftEncoder) nincs implementálva** ebben a firmware-ben – csak a bit létezik, funkció nem.

## 7. Bemenetek (ADC, kapcsolók)

- Külső **ADS1115** 16 bites I2C ADC (kormányszög-érzékelő + nyomás/áram szenzor csatorna). Nincs beépített ESP32 `analogRead()` használat ezekhez (szemben a Teensy natív ADC-jével).
- Az ADC-olvasás **teljesen egy külön FreeRTOS taskban** történik (Core 1), mutex-szel védett gyorsítótárral, 3-elemű medián szűrővel a kiugró értékek ellen – a főciklus soha nem blokkolódik I2C-olvasásra várva.
- Kapcsolók: `STEERSW_PIN`, `WORKSW_PIN`. **Nincs külön „remote” bemenet** és **nincs kormánykerék-enkóder** (ISR) támogatás, szemben a Teensy verzióval.

## 8. Sebesség-impulzus kimenet (opcionális)

- `SPEED_IMPULSE_ENABLED` esetén egy dedikált ESP32 hardver-timer (1 MHz órajel, megszakítás-alapú tűlevél) állít elő négyszögjelet a GPS-sebességből, konfigurálható impulzus/méter (`PULSES_PER_METER`) értékkel.
- Ez pontosabb és processzor-terhelés szempontjából olcsóbb megoldás, mint a Teensy `tone()`-alapú, blokkoló-jellegű implementációja.

## 9. Konfiguráció és EEPROM

- Minden hangolható konstans egy helyen: [include/Configuration.h](include/Configuration.h).
- 96 bájtos EEPROM elrendezés: azonosító, `Storage` (PID/steer beállítások), `Setup` (funkció-jelzők), IP-cím, és opcionálisan az auto-tune Kd érték.
- WiFi/UDP mód (AP vagy STA), teljesítmény és sávszélesség finomhangolása is itt konfigurálható.

---

## 10. Összehasonlítás a hivatalos AgOpenGPS Teensy AIO v2.5 firmware-rel

Referencia: [`AgOpenGPS-Official/Boards` – `TeensyModules/AIO v2.5/Firmware/Autosteer_gps_teensy_v2_5`](https://github.com/AgOpenGPS-Official/Boards/tree/main/TeensyModules/AIO%20v2.5/Firmware/Autosteer_gps_teensy_v2_5)

### Áttekintő táblázat

| Szempont | ESP32_AGOPEN (ez a projekt) | Teensy AIO v2.5 (hivatalos) | Melyik jobb? |
|---|---|---|---|
| **MCU** | ESP32, dual-core @240 MHz, beépített WiFi | Teensy 4.1, single-core Cortex-M7 @600 MHz | Teensy: nyers számítási sebesség/determinisztikus időzítés. ESP32: dual-core + beépített vezeték nélküli kommunikáció, olcsóbb hardver |
| **GPS bemenet** | 1 db GPS vevő (GGA+VTG), fix baud, NTRIP/RTCM átjátszás (PGN 215 → Serial2) | **2 db** F9P vevő (pozíció + fejirány), auto-baud detektálás, NTRIP/RTCM átjátszás, port-csere ha nincs GGA | **Teensy jobb** – valódi RTK dual-antenna heading/roll (bázisvonalból), nem csak IMU-becslés |
| **Kettős antenna fúzió** | Nincs | Részleges (a `fuseIMU` kód nagy része ki van kommentezve, gyakorlatilag csak nyers dual heading/roll megy ki) | Egyik sem teljes; Teensy-nek megvan hozzá az infrastruktúrája (RELPOSNED dekódolás), csak a fúziós logika hiányos |
| **IMU – adatgyűjtés** | Adafruit_BNO08x hivatalos könyvtár, teljes FIFO-ürítés minden loop-ban, 100 Hz | Egyedi, régi (SparkFun-eredetű) BNO080 port, fix 20 ms-os `systick_millis_count` alapú lekérdezés | **ESP32 jobb** – karbantartott upstream könyvtár, nincs sorbanállási torlódás |
| **IMU – GPS szinkronizáció** | **SLERP-interpolált** orientáció, pontosan a GGA érkezési pillanatára időbélyegezve (µs pontosság) | Nincs időbeli interpoláció – az utolsó BNO-olvasás értékét használja, ami akár ~10–20 ms-mal is elcsúszhat a GGA-tól | **ESP32 egyértelműen jobb** – ez a projekt fő hozzáadott értéke |
| **IMU – kiugrás-szűrés** | Szögsebesség-alapú adaptív küszöb (rad/s × Δt) | Nincs kiugrás-szűrés | **ESP32 jobb** |
| **Kompasz alternatíva (CMPS14)** | Nincs | Van | Teensy jobb (rugalmasabb hardverválasztás) |
| **PID szabályozás** | P-only **VAGY** opcionális auto-tanuló P+I+D (Kd öntanulás oszcilláció-detektálással, EEPROM-perzisztencia) | Csak P (holtsáv-interpolációval) | **ESP32 jobb** – a hivatalos firmware-ben nincs I/D tag és nincs automatikus hangolás |
| **Motor meghajtás** | Cytron / IBT-2, 20 kHz / 10 bit PWM | Cytron / IBT-2 **+ Danfoss** (25/75% arányú leképezés) | Teensy jobb – Danfoss szelep tényleges PWM-logikája implementálva van; ESP32-n csak a konfigurációs bit létezik, funkció nincs |
| **Kerékenkóder (ShaftEncoder)** | Konfigurációs bit létezik, de **nincs megvalósítva** | Teljes ISR-alapú implementáció | Teensy jobb |
| **Remote bemenet** | Nincs | Van (`REMOTE_PIN`) | Teensy jobb |
| **Nyomás/áram szenzor bemenet** | ADS1115 (külső I2C ADC), nem blokkoló háttér task, medián szűrő | Beépített `analogRead()`, EMA szűrő, blokkoló a fő cikluson belül | **ESP32 jobb** architektúrálisan (nem blokkol), de plusz alkatrészt (ADS1115) igényel |
| **Kommunikáció** | Soros **és/vagy** opcionális WiFi/UDP (AP vagy STA, aszinkron küldő-sor) | Soros **és/vagy** vezetékes Ethernet (NativeEthernet, csak Teensy 4.1) | Attól függ: WiFi rugalmasabb (nincs kábel), de az Ethernet stabilabb/kisebb késleltetésű ipari környezetben. Vezeték nélküli integráció ESP32-n egyszerűbb, mert beépített rádió van |
| **NTRIP/RTCM támogatás** | Van (PGN 215 → `Serial2` átjátszás egyetlen GPS vevőhöz) | Van (RTCM átjátszás rádió/soros porton, 2 vevőnek) | Teensy jobb – ott 2 vevő (pozíció + heading) kapja meg a korrekciót, és rádiós bemenet is van rá |
| **Auto-baud GPS detektálás** | Nincs | Van (UBX parancsokkal automatikusan detektálja és beállítja a vevő baud rate-jét) | Teensy jobb |
| **Sebesség-impulzus kimenet** | Dedikált hardver-timer, megszakítás-alapú, konfigurálható impulzus/méter | `tone()` alapú, szoftveres, fix 130 impulzus/méter | ESP32 jobb (pontosabb, kevésbé terheli a CPU-t) |
| **ADC/szenzor-olvasás nem-blokkolása** | Külön FreeRTOS task, mutex-szel védett gyorsítótár | Blokkoló `analogRead()` a fő időzített cikluson belül | ESP32 jobb architektúrálisan |
| **Kódszervezés / dokumentáltság** | PlatformIO projekt, külön header/forrás fájlok modulonként, részletes `docs/` mappa (changelog, kódreview jelentések) | Arduino `.ino` fájlok gyűjteménye, kevesebb elkülönített dokumentáció | ESP32 jobb a karbantarthatóság szempontjából |
| **Érettség / éles üzemi múlt** | Egyedi, kevésbé elterjedt, kisebb közösségi teszt-lefedettség | Hivatalos, széles körben tesztelt és használt AgOpenGPS referencia-implementáció | Teensy előnyben – nagyobb közösségi validáció |

### Összefoglaló értékelés

**Ahol az ESP32_AGOPEN firmware jobb:**
1. **GPS–IMU időbeli szinkronizáció** – ez a legjelentősebb technikai különbség. A Teensy egyszerűen a legutóbb leolvasott BNO-mintát küldi ki minden GGA-hoz, ami akár 10-20 ms csúszást is jelenthet 100 Hz-es IMU mellett. Az ESP32-verzió SLERP-interpolációval a GGA pontos érkezési pillanatára számítja át az orientációt.
2. **Adaptív kiugrás-szűrés** az IMU adatokon (szögsebesség-alapú, nem fix dot-product küszöb).
3. **Opcionális auto-tanuló PID** (P+I+D, öntanuló D-tag) – a hivatalos firmware-ben egyáltalán nincs I/D szabályzás.
4. **Nem blokkoló architektúra mindenhol** (ADC külön taskban, soros/UDP küldés queue-n keresztül) – kevésbé érzékeny az I2C/kommunikációs késleltetésekre.
5. Pontosabb, kevésbé CPU-igényes sebesség-impulzus generálás.

**Ahol a hivatalos Teensy firmware jobb / teljesebb:**
1. **Valódi dual-antenna RTK heading/roll** (két F9P vevő, RELPOSNED bázisvonal-dekódolás) és automatikus GPS baud-rate detektálás – ezek hiányoznak az ESP32 verzióból (az NTRIP/RTCM átjátszás alapszinten megvan, csak egyetlen GPS vevőre, rádiós bemenet nélkül).
2. **Danfoss szelep és kerékenkóder** ténylegesen megvalósítva (ESP32-n csak a konfigurációs bitek léteznek, funkció nélkül).
3. **Remote bemenet** és CMPS14 kompasz-alternatíva.
4. Sokkal szélesebb körben tesztelt, hivatalos referencia-implementáció, aktív közösségi támogatással.

**Következtetés:** az ESP32_AGOPEN firmware egy **egyantennás, IMU-ra támaszkodó** konfigurációban technikailag kifinomultabb (jobb szinkronizáció, jobb PID, nem blokkoló I/O), miközben olcsóbb, WiFi-képes hardveren fut, és az alapvető NTRIP/RTCM átjátszást is támogatja. Ha viszont **dual-antenna RTK heading**, rádiós RTK bemenet, Danfoss szelep vagy kerékenkóder szükséges, a hivatalos Teensy AIO v2.5 firmware jelenleg funkcionálisan teljesebb.

---

## 11. Build

```powershell
cd Boards\ESP32_AGOPEN
pio run                       # fordítás
pio run --target upload       # feltöltés (upload_port a platformio.ini-ben)
pio device monitor -b 460800  # soros monitor
```

Külső könyvtárfüggőségek (`platformio.ini`): `Adafruit BNO08x`, `Adafruit ADS1X15`.

További részletek a [docs/](docs/) mappában (changelog, kódfelülvizsgálati jelentések, gyors indítási útmutató, WiFi/UDP README).
