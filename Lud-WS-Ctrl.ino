/* ============================================================================
 *  CONTROLLER SYNTH MULTI-CATEGORIA + TASTIERA 32 NOTE
 *  Target : Raspberry Pi Pico (RP2040) - core Arduino-Pico (Earle Philhower)
 * ============================================================================
 *
 *  ARCHITETTURA HARDWARE
 *  ---------------------
 *  - Tastiera musicale 8 OUT x 4 IN = 32 note (F3 .. C6)
 *  - 3x TCA9548A (0x70/0x71/0x72)
 *  - 14x ADS1115 (0x48) -> 54 potenziometri
 *  - 10x PCF8574 (0x20) -> bottoni + LED
 *
 *  CATEGORIE
 *  ---------
 *    'A' = SynthA_Ctrl   16 pot,  8 btn,  8 led
 *    'B' = SynthB_Ctrl   16 pot,  8 btn,  8 led
 *    'T' = Teensy_Ctrl    8 pot, 10 btn, 10 led
 *    'D' = Data_Ctrl      6 pot,  8 btn,  8 led
 *    'M' = Mixer_Ctrl     8 pot
 *
 *  PROTOCOLLO SERIALE (Serial1)
 *  ----------------------------
 *  TX:
 *    Nota tastiera : cat | status | pitch | velocity | '&' | '!'
 *    Potenziometro : cat | 'P' | ID | valore(0-255) | '&' | '!'
 *    Bottone       : cat | 'B' | ID | valore(0-1)   | '&' | '!'
 *
 *  RX:
 *    Ping          : "p&!"
 *    Tast_Mode     : 'T' | 'M' | valore(0-4) | '&' | '!'
 *    Split_Note    : 'T' | 'S' | valore(0-31) | '&' | '!'
 *
 *  DEBUG (Serial USB CDC)
 *  ----------------------
 *  Tutti i messaggi di debug escono su Serial (USB), non su Serial1.
 *  Per attivare/disattivare il debug, cambia la macro DEBUG_ENABLED
 *  nella Sezione 1.
 * ==========================================================================*/


/* ============================================================================
 *  MAPPA DEI PIN DEL RASPBERRY PI PICO (RP2040)
 * ============================================================================
 *
 *  Il Pico espone 40 pin fisici, di cui 26 sono GPIO utilizzabili (GP0-GP22,
 *  GP26-GP28). I restanti sono alimentazione, ground e debug.
 *
 *  ---------------------------------------------------------------------------
 *  PIN FISICI E FUNZIONI
 *  ---------------------------------------------------------------------------
 *
 *  Pin | GPIO | Uso nel progetto
 *  ----|------|--------------------------------------------------------------
 *   1  | GP0  | UART0 TX -> SERIAL1_TX_PIN (router)
 *   2  | GP1  | UART0 RX -> SERIAL1_RX_PIN (router)
 *   3  | GND  | Ground
 *   4  | GP2  | Tastiera OUT 0 (keyOutPins[0])
 *   5  | GP3  | Tastiera OUT 1 (keyOutPins[1])
 *   6  | GP4  | I2C0 SDA (Wire, default core Philhower)
 *   7  | GP5  | I2C0 SCL (Wire, default core Philhower)
 *   8  | GND  | Ground
 *   9  | GP6  | Tastiera OUT 2 (keyOutPins[2])
 *  10  | GP7  | Tastiera OUT 3 (keyOutPins[3])
 *  11  | GP8  | Tastiera OUT 4 (keyOutPins[4])
 *  12  | GP9  | Tastiera OUT 5 (keyOutPins[5])
 *  13  | GND  | Ground
 *  14  | GP10 | Tastiera OUT 6 (keyOutPins[6])
 *  15  | GP11 | Tastiera OUT 7 (keyOutPins[7])
 *  16  | GP12 | Tastiera IN 0  (keyInPins[0])
 *  17  | GP13 | Tastiera IN 1  (keyInPins[1])
 *  18  | GND  | Ground
 *  19  | GP14 | Tastiera IN 2  (keyInPins[2])
 *  20  | GP15 | Tastiera IN 3  (keyInPins[3])
 *  21  | GP16 | libero
 *  22  | GP17 | libero
 *  23  | GND  | Ground
 *  24  | GP18 | libero
 *  25  | GP19 | libero
 *  26  | GP20 | libero
 *  27  | GP21 | libero
 *  28  | GND  | Ground
 *  29  | GP22 | libero
 *  30  | RUN  | Reset (pulsante reset onboard)
 *  31  | GP26 | libero (ADC0)
 *  32  | GP27 | libero (ADC1)
 *  33  | AGND | Analog Ground
 *  34  | GP28 | libero (ADC2)
 *  35  | ADC_VREF | ADC reference
 *  36  | 3V3(OUT) | 3.3V out (alimentazione digitale)
 *  37  | 3V3_EN  | 3.3V enable
 *  38  | GND     | Ground
 *  39  | VSYS    | System input
 *  40  | VBUS    | USB 5V
 *
 *  PIN INTERNI NON ESPOSTI: GP23 (power-save), GP24 (VBUS sense),
 *  GP25 (LED onboard), GP29 (VSYS monitor).
 *
 *  ---------------------------------------------------------------------------
 *  RIEPILOGO DEI PIN UTILIZZATI
 *  ---------------------------------------------------------------------------
 *  UART router : GP0 (TX), GP1 (RX)
 *  I2C         : GP4 (SDA), GP5 (SCL)
 *  Tastiera OUT: GP2, GP3, GP6, GP7, GP8, GP9, GP10, GP11
 *  Tastiera IN : GP12, GP13, GP14, GP15
 *  Liberi      : GP16..GP22 (7 GPIO)
 *  ADC liberi  : GP26, GP27, GP28 (3 canali)
 * ==========================================================================*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>


/* ============================================================================
 *  SEZIONE 1 - CONFIGURAZIONE GENERALE
 * ==========================================================================*/

/* --- Flag di debug ----------------------------------------------------------
 *  0 = nessun messaggio di debug (compilazione "pulita")
 *  1 = debug attivo su Serial (USB CDC)
 * --------------------------------------------------------------------------*/
#define DEBUG_ENABLED 1

/* --- Macro di comodo per il debug ------------------------------------------*/
#if DEBUG_ENABLED
  #define DPRINT(x)      Serial.print(x)
  #define DPRINTLN(x)    Serial.println(x)
  #define DPRINTF(...)   Serial.printf(__VA_ARGS__)
#else
  #define DPRINT(x)      do {} while (0)
  #define DPRINTLN(x)    do {} while (0)
  #define DPRINTF(...)   do {} while (0)
#endif

/* --- UART verso il router --------------------------------------------------*/
static const int      SERIAL1_TX_PIN = 0;
static const int      SERIAL1_RX_PIN = 1;
static const uint32_t SERIAL_BAUD    = 115200;

/* --- Tastiera --------------------------------------------------------------*/
static const uint8_t KEY_NUM_OUT = 8;
static const uint8_t KEY_NUM_IN  = 4;
static const uint8_t KEY_NUM     = KEY_NUM_OUT * KEY_NUM_IN;

static const uint8_t keyOutPins[KEY_NUM_OUT] = {2, 3, 6, 7, 8, 9, 10, 11};
static const uint8_t keyInPins [KEY_NUM_IN ] = {12, 13, 14, 15};

static const uint8_t  KEY_BASE_NOTE    = 53;   // F3 = MIDI 53, C6 = 84
static const uint8_t  KEY_VELOCITY_ON  = 100;
static const uint8_t  KEY_VELOCITY_OFF = 0;
static const uint8_t  KEY_DEBOUNCE_MS  = 5;
static const uint16_t KEY_SETTLE_US    = 30;


/* ============================================================================
 *  SEZIONE 2 - INDIRIZZI I2C
 * ==========================================================================*/

static const uint8_t MUX0_ADDR = 0x70;
static const uint8_t MUX1_ADDR = 0x71;
static const uint8_t MUX2_ADDR = 0x72;
static const uint8_t ADS_ADDR  = 0x48;
static const uint8_t PCF_ADDR  = 0x20;


/* ============================================================================
 *  SEZIONE 3 - PARAMETRI DI SCANSIONE
 * ==========================================================================*/

/* --- Potenziometri ---------------------------------------------------------*/
static const uint16_t POT_SETTLE_US  = 100;   // assestamento dopo selectMux
static const uint8_t  POT_THRESHOLD  = 2;     // delta minimo per inviare
static const int16_t  ADS_FULL_SCALE = 26400; // fondo scala GAIN_ONE (~3.3V)

/* --- Filtro adattivo per i potenziometri -----------------------------------
 *  alpha espresso in scala 0-255:
 *    0   = nessun aggiornamento (filtro "congelato")
 *    255 = aggiornamento immediato (nessuna smoothing)
 * --------------------------------------------------------------------------*/
static const uint8_t FILT_ALPHA_JUMP  = 255;  // absDiff >= 20
static const uint8_t FILT_ALPHA_BIG   = 180;  // absDiff >= 10
static const uint8_t FILT_ALPHA_MED   = 100;  // absDiff >= 4
static const uint8_t FILT_ALPHA_SMALL = 40;   // absDiff >= 1

/* --- Bottoni ---------------------------------------------------------------*/
static const uint8_t  DEBOUNCE_MS = 5;

/* --- Categorie -------------------------------------------------------------*/
#define CAT_SYNTHA 'A'
#define CAT_SYNTHB 'B'
#define CAT_TEENSY 'T'
#define CAT_DATA   'D'
#define CAT_MIXER  'M'

/* --- Tipi pin PCF8574 ------------------------------------------------------*/
#define PCF_TYPE_UNUSED 0
#define PCF_TYPE_BUTTON 1
#define PCF_TYPE_LED    2


/* ============================================================================
 *  SEZIONE 4 - VARIABILI DI STATO DELLA TASTIERA
 * ==========================================================================*/

static uint8_t Tast_Mode  = 2;   // 0=solo A, 1=solo B, 2=split A/B,
                                 // 3=split B/A, 4=entrambe
static uint8_t Split_Note = 16;  // indice 0-31 (0=F3, 31=C6)


/* ============================================================================
 *  SEZIONE 5 - OGGETTO ADS1115
 * ==========================================================================*/

Adafruit_ADS1115 ads;


/* ============================================================================
 *  SEZIONE 6 - STRUTTURE DI MAPPATURA
 * ==========================================================================*/

struct AdsGroup {
  uint8_t muxAddr;
  uint8_t muxCh;
  uint8_t adcCh[4];
  char    cat[4];
  uint8_t id[4];
  int16_t lastMapped[4] = {-1, -1, -1, -1};  // ultimo valore INVIATO
  uint8_t filtered[4]   = {0, 0, 0, 0};      // valore filtrato corrente
};

struct PcfGroup {
  uint8_t  muxAddr;
  uint8_t  muxCh;
  uint8_t  type[8];
  char     cat[8];
  uint8_t  id[8];
  uint32_t lastChange[8] = {0};
  uint8_t  debounced[8]  = {0};
  uint8_t  outputState   = 0xFF;
};


/* ============================================================================
 *  SEZIONE 7 - MAPPA HARDWARE
 * ============================================================================
 *  TCA #0 (0x70):
 *    Ch0..Ch3  ADS -> SynthA pots 0-15
 *    Ch4       PCF -> SynthA buttons 0-7
 *    Ch5       PCF -> SynthA LEDs 0-7
 *    Ch6,Ch7   ADS -> SynthB pots 0-7
 *
 *  TCA #1 (0x71):
 *    Ch0,Ch1   ADS -> SynthB pots 8-15
 *    Ch2       PCF -> SynthB buttons 0-7
 *    Ch3       PCF -> SynthB LEDs 0-7
 *    Ch4,Ch5   ADS -> Teensy pots 0-7
 *    Ch6       PCF -> Teensy buttons 0-7
 *    Ch7       PCF -> Teensy LEDs 0-7
 *
 *  TCA #2 (0x72):
 *    Ch0       PCF -> Teensy buttons 8-9 (pin 0-1)
 *    Ch1       PCF -> Teensy LEDs 8-9    (pin 0-1)
 *    Ch2,Ch3   ADS -> Data pots 0-5
 *    Ch4       PCF -> Data buttons 0-7
 *    Ch5       PCF -> Data LEDs 0-7
 *    Ch6,Ch7   ADS -> Mixer pots 0-7
 * ==========================================================================*/

static AdsGroup adsGroups[] = {
  // ===== SynthA_Ctrl : 16 pot =====
  { MUX0_ADDR, 0, {0,1,2,3}, {CAT_SYNTHA,CAT_SYNTHA,CAT_SYNTHA,CAT_SYNTHA}, { 0, 1, 2, 3} },
  { MUX0_ADDR, 1, {0,1,2,3}, {CAT_SYNTHA,CAT_SYNTHA,CAT_SYNTHA,CAT_SYNTHA}, { 4, 5, 6, 7} },
  { MUX0_ADDR, 2, {0,1,2,3}, {CAT_SYNTHA,CAT_SYNTHA,CAT_SYNTHA,CAT_SYNTHA}, { 8, 9,10,11} },
  { MUX0_ADDR, 3, {0,1,2,3}, {CAT_SYNTHA,CAT_SYNTHA,CAT_SYNTHA,CAT_SYNTHA}, {12,13,14,15} },

  // ===== SynthB_Ctrl : 16 pot =====
  { MUX0_ADDR, 6, {0,1,2,3}, {CAT_SYNTHB,CAT_SYNTHB,CAT_SYNTHB,CAT_SYNTHB}, { 0, 1, 2, 3} },
  { MUX0_ADDR, 7, {0,1,2,3}, {CAT_SYNTHB,CAT_SYNTHB,CAT_SYNTHB,CAT_SYNTHB}, { 4, 5, 6, 7} },
  { MUX1_ADDR, 0, {0,1,2,3}, {CAT_SYNTHB,CAT_SYNTHB,CAT_SYNTHB,CAT_SYNTHB}, { 8, 9,10,11} },
  { MUX1_ADDR, 1, {0,1,2,3}, {CAT_SYNTHB,CAT_SYNTHB,CAT_SYNTHB,CAT_SYNTHB}, {12,13,14,15} },

  // ===== Teensy_Ctrl : 8 pot =====
  { MUX1_ADDR, 4, {0,1,2,3}, {CAT_TEENSY,CAT_TEENSY,CAT_TEENSY,CAT_TEENSY}, { 0, 1, 2, 3} },
  { MUX1_ADDR, 5, {0,1,2,3}, {CAT_TEENSY,CAT_TEENSY,CAT_TEENSY,CAT_TEENSY}, { 4, 5, 6, 7} },

  // ===== Data_Ctrl : 6 pot =====
  { MUX2_ADDR, 2, {0,1,2,3}, {CAT_DATA,CAT_DATA,CAT_DATA,CAT_DATA}, {0,1,2,3} },
  { MUX2_ADDR, 3, {0,1,0xFF,0xFF}, {CAT_DATA,CAT_DATA,0,0}, {4,5,0,0} },

  // ===== Mixer_Ctrl : 8 pot =====
  { MUX2_ADDR, 6, {0,1,2,3}, {CAT_MIXER,CAT_MIXER,CAT_MIXER,CAT_MIXER}, {0,1,2,3} },
  { MUX2_ADDR, 7, {0,1,2,3}, {CAT_MIXER,CAT_MIXER,CAT_MIXER,CAT_MIXER}, {4,5,6,7} },
};
#define NUM_ADS_GROUPS (sizeof(adsGroups) / sizeof(adsGroups[0]))

static PcfGroup pcfGroups[] = {
  // ===== SynthA_Ctrl : 8 buttons + 8 LED =====
  { MUX0_ADDR, 4,
    {PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,
     PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,PCF_TYPE_BUTTON},
    {CAT_SYNTHA,CAT_SYNTHA,CAT_SYNTHA,CAT_SYNTHA,
     CAT_SYNTHA,CAT_SYNTHA,CAT_SYNTHA,CAT_SYNTHA},
    {0,1,2,3,4,5,6,7} },
  { MUX0_ADDR, 5,
    {PCF_TYPE_LED,PCF_TYPE_LED,PCF_TYPE_LED,PCF_TYPE_LED,
     PCF_TYPE_LED,PCF_TYPE_LED,PCF_TYPE_LED,PCF_TYPE_LED},
    {CAT_SYNTHA,CAT_SYNTHA,CAT_SYNTHA,CAT_SYNTHA,
     CAT_SYNTHA,CAT_SYNTHA,CAT_SYNTHA,CAT_SYNTHA},
    {0,1,2,3,4,5,6,7} },

  // ===== SynthB_Ctrl : 8 buttons + 8 LED =====
  { MUX1_ADDR, 2,
    {PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,
     PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,PCF_TYPE_BUTTON},
    {CAT_SYNTHB,CAT_SYNTHB,CAT_SYNTHB,CAT_SYNTHB,
     CAT_SYNTHB,CAT_SYNTHB,CAT_SYNTHB,CAT_SYNTHB},
    {0,1,2,3,4,5,6,7} },
  { MUX1_ADDR, 3,
    {PCF_TYPE_LED,PCF_TYPE_LED,PCF_TYPE_LED,PCF_TYPE_LED,
     PCF_TYPE_LED,PCF_TYPE_LED,PCF_TYPE_LED,PCF_TYPE_LED},
    {CAT_SYNTHB,CAT_SYNTHB,CAT_SYNTHB,CAT_SYNTHB,
     CAT_SYNTHB,CAT_SYNTHB,CAT_SYNTHB,CAT_SYNTHB},
    {0,1,2,3,4,5,6,7} },

  // ===== Teensy_Ctrl : 10 buttons + 10 LED =====
  { MUX1_ADDR, 6,
    {PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,
     PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,PCF_TYPE_BUTTON},
    {CAT_TEENSY,CAT_TEENSY,CAT_TEENSY,CAT_TEENSY,
     CAT_TEENSY,CAT_TEENSY,CAT_TEENSY,CAT_TEENSY},
    {0,1,2,3,4,5,6,7} },
  { MUX1_ADDR, 7,
    {PCF_TYPE_LED,PCF_TYPE_LED,PCF_TYPE_LED,PCF_TYPE_LED,
     PCF_TYPE_LED,PCF_TYPE_LED,PCF_TYPE_LED,PCF_TYPE_LED},
    {CAT_TEENSY,CAT_TEENSY,CAT_TEENSY,CAT_TEENSY,
     CAT_TEENSY,CAT_TEENSY,CAT_TEENSY,CAT_TEENSY},
    {0,1,2,3,4,5,6,7} },
  { MUX2_ADDR, 0,
    {PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,PCF_TYPE_UNUSED,PCF_TYPE_UNUSED,
     PCF_TYPE_UNUSED,PCF_TYPE_UNUSED,PCF_TYPE_UNUSED,PCF_TYPE_UNUSED},
    {CAT_TEENSY,CAT_TEENSY,0,0,0,0,0,0},
    {8,9,0,0,0,0,0,0} },
  { MUX2_ADDR, 1,
    {PCF_TYPE_LED,PCF_TYPE_LED,PCF_TYPE_UNUSED,PCF_TYPE_UNUSED,
     PCF_TYPE_UNUSED,PCF_TYPE_UNUSED,PCF_TYPE_UNUSED,PCF_TYPE_UNUSED},
    {CAT_TEENSY,CAT_TEENSY,0,0,0,0,0,0},
    {8,9,0,0,0,0,0,0} },

  // ===== Data_Ctrl : 8 buttons + 8 LED =====
  { MUX2_ADDR, 4,
    {PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,
     PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,PCF_TYPE_BUTTON,PCF_TYPE_BUTTON},
    {CAT_DATA,CAT_DATA,CAT_DATA,CAT_DATA,CAT_DATA,CAT_DATA,CAT_DATA,CAT_DATA},
    {0,1,2,3,4,5,6,7} },
  { MUX2_ADDR, 5,
    {PCF_TYPE_LED,PCF_TYPE_LED,PCF_TYPE_LED,PCF_TYPE_LED,
     PCF_TYPE_LED,PCF_TYPE_LED,PCF_TYPE_LED,PCF_TYPE_LED},
    {CAT_DATA,CAT_DATA,CAT_DATA,CAT_DATA,CAT_DATA,CAT_DATA,CAT_DATA,CAT_DATA},
    {0,1,2,3,4,5,6,7} },

  // ===== Mixer_Ctrl : bottoni/LED da definire =====
};
#define NUM_PCF_GROUPS (sizeof(pcfGroups) / sizeof(pcfGroups[0]))


/* ============================================================================
 *  SEZIONE 8 - STATO DELLA TASTIERA
 * ==========================================================================*/

static uint8_t  keyNoteMap[KEY_NUM_OUT][KEY_NUM_IN];
static bool     keyRaw[KEY_NUM];
static bool     keyStable[KEY_NUM];
static uint32_t keyLastChange[KEY_NUM];


/* ============================================================================
 *  SEZIONE 9 - UTILITY I2C
 * ==========================================================================*/

static inline void selectMux(uint8_t muxAddr, uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(muxAddr);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

static inline uint8_t pcfReadByte(uint8_t addr) {
  Wire.requestFrom((int)addr, 1);
  return Wire.available() ? (uint8_t)Wire.read() : 0xFF;
}

static inline void pcfWriteByte(uint8_t addr, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(value);
  Wire.endTransmission();
}


/* ============================================================================
 *  SEZIONE 10 - INVIO MESSAGGI SERIALI
 * ==========================================================================*/

/* --- Nomi delle note (per debug) ------------------------------------------*/
static const char* midiNoteName(uint8_t pitch) {
  static char buf[8];
  static const char* names[12] = {
    "C","C#","D","D#","E","F","F#","G","G#","A","A#","B"
  };
  if (pitch < 12 || pitch > 127) {
    snprintf(buf, sizeof(buf), "??");
  } else {
    uint8_t n = pitch % 12;
    int8_t  o = (int8_t)(pitch / 12) - 1;
    snprintf(buf, sizeof(buf), "%s%d", names[n], o);
  }
  return buf;
}

static void sendNoteTo(char cat, uint8_t status, uint8_t pitch, uint8_t velocity) {
  Serial1.write((uint8_t)cat);
  Serial1.write(status);
  Serial1.write(pitch);
  Serial1.write(velocity);
  Serial1.write((uint8_t)'&');
  Serial1.write((uint8_t)'!');

  DPRINTF("[TX] Note -> %c : %s %s (%d) vel=%d\n",
          cat,
          (status ? "ON " : "OFF"),
          midiNoteName(pitch),
          pitch,
          velocity);
}

static void dispatchNote(uint8_t status, uint8_t pitch, uint8_t velocity) {

  const uint8_t splitPitch = KEY_BASE_NOTE + Split_Note;

  DPRINTF("[RT] pitch=%d (%s) mode=%d split=%d\n",
          pitch, midiNoteName(pitch), Tast_Mode, splitPitch);

  switch (Tast_Mode) {
    case 0:
      sendNoteTo(CAT_SYNTHA, status, pitch, velocity);
      break;
    case 1:
      sendNoteTo(CAT_SYNTHB, status, pitch, velocity);
      break;
    case 2:
      if (pitch < splitPitch) sendNoteTo(CAT_SYNTHA, status, pitch, velocity);
      else                    sendNoteTo(CAT_SYNTHB, status, pitch, velocity);
      break;
    case 3:
      if (pitch < splitPitch) sendNoteTo(CAT_SYNTHB, status, pitch, velocity);
      else                    sendNoteTo(CAT_SYNTHA, status, pitch, velocity);
      break;
    case 4:
      sendNoteTo(CAT_SYNTHA, status, pitch, velocity);
      sendNoteTo(CAT_SYNTHB, status, pitch, velocity);
      break;
    default:
      DPRINTLN("[RT] WARN: Tast_Mode non valido");
      break;
  }
}

static void sendPot(char cat, uint8_t id, uint8_t value) {
  Serial1.write((uint8_t)cat);
  Serial1.write((uint8_t)'P');
  Serial1.write(id);
  Serial1.write(value);
  Serial1.write((uint8_t)'&');
  Serial1.write((uint8_t)'!');

  DPRINTF("[TX] Pot  %c id=%d val=%d\n", cat, id, value);
}

static void sendButton(char cat, uint8_t id, uint8_t value) {
  Serial1.write((uint8_t)cat);
  Serial1.write((uint8_t)'B');
  Serial1.write(id);
  Serial1.write(value);
  Serial1.write((uint8_t)'&');
  Serial1.write((uint8_t)'!');

  DPRINTF("[TX] Btn  %c id=%d %s\n", cat, id, (value ? "PRESS" : "RELEASE"));
}

static void sendPingReply() {
  Serial1.write((uint8_t)'p');
  Serial1.write((uint8_t)'&');
  Serial1.write((uint8_t)'!');

  DPRINTLN("[TX] Ping reply -> p&!");
}


/* ============================================================================
 *  SEZIONE 11 - PARSER RX
 * ==========================================================================*/

static const uint8_t RX_BUF_SIZE = 8;
static uint8_t rxBuf[RX_BUF_SIZE];
static uint8_t rxLen = 0;

static void processRxCommand(const uint8_t* buf, uint8_t len) {

  DPRINTF("[RX] Comando ricevuto (%d byte):", len);
  for (uint8_t i = 0; i < len; i++) DPRINTF(" %02X", buf[i]);
  DPRINTLN();

  // --- Ping "p&!" ---------------------------------------------------------
  if (len == 3 && buf[0] == 'p' && buf[1] == '&' && buf[2] == '!') {
    DPRINTLN("[RX] -> Ping");
    sendPingReply();
    return;
  }

  // --- Comandi "T..." a 5 byte -------------------------------------------
  if (len == 5 && buf[0] == 'T' && buf[3] == '&' && buf[4] == '!') {

    if (buf[1] == 'M') {
      if (buf[2] <= 4) {
        Tast_Mode = buf[2];
        DPRINTF("[RX] -> Tast_Mode = %d\n", Tast_Mode);
      } else {
        DPRINTF("[RX] -> Tast_Mode IGNORATO (valore %d non valido)\n", buf[2]);
      }
      return;
    }

    if (buf[1] == 'S') {
      if (buf[2] <= 31) {
        Split_Note = buf[2];
        DPRINTF("[RX] -> Split_Note = %d (pitch = %d = %s)\n",
                Split_Note,
                KEY_BASE_NOTE + Split_Note,
                midiNoteName(KEY_BASE_NOTE + Split_Note));
      } else {
        DPRINTF("[RX] -> Split_Note IGNORATO (valore %d non valido)\n", buf[2]);
      }
      return;
    }
  }

  DPRINTLN("[RX] -> Comando NON RICONOSCIUTO");
}

static void handleSerialRx() {
  while (Serial1.available() > 0) {
    const uint8_t b = (uint8_t)Serial1.read();

    if (rxLen >= RX_BUF_SIZE) {
      DPRINTLN("[RX] WARN: buffer pieno, reset");
      rxLen = 0;
    }

    rxBuf[rxLen++] = b;

    if (rxLen >= 2 && rxBuf[rxLen - 2] == '&' && rxBuf[rxLen - 1] == '!') {
      processRxCommand(rxBuf, rxLen);
      rxLen = 0;
    }
  }
}


/* ============================================================================
 *  SEZIONE 12 - TASTIERA
 * ==========================================================================*/

static void initKeyboard() {
  for (uint8_t i = 0; i < KEY_NUM_OUT; i++) pinMode(keyOutPins[i], INPUT);
  for (uint8_t i = 0; i < KEY_NUM_IN;  i++) pinMode(keyInPins[i],  INPUT_PULLUP);

  for (uint8_t o = 0; o < KEY_NUM_OUT; o++)
    for (uint8_t i = 0; i < KEY_NUM_IN; i++)
      keyNoteMap[o][i] = KEY_BASE_NOTE + (o * KEY_NUM_IN + i);

  for (uint8_t k = 0; k < KEY_NUM; k++) {
    keyRaw[k]        = false;
    keyStable[k]     = false;
    keyLastChange[k] = 0;
  }
}

static void keyMatrixRead(bool out[KEY_NUM]) {
  for (uint8_t o = 0; o < KEY_NUM_OUT; o++) {
    for (uint8_t i = 0; i < KEY_NUM_OUT; i++) pinMode(keyOutPins[i], INPUT);

    pinMode(keyOutPins[o], OUTPUT);
    digitalWrite(keyOutPins[o], LOW);

    delayMicroseconds(KEY_SETTLE_US);

    for (uint8_t i = 0; i < KEY_NUM_IN; i++) {
      out[o * KEY_NUM_IN + i] = (digitalRead(keyInPins[i]) == LOW);
    }

    pinMode(keyOutPins[o], INPUT);
  }
}

static void scanKeyboard() {
  bool current[KEY_NUM];
  keyMatrixRead(current);

  const uint32_t now = millis();

  for (uint8_t k = 0; k < KEY_NUM; k++) {
    if (current[k] != keyRaw[k]) {
      keyRaw[k]        = current[k];
      keyLastChange[k] = now;
      continue;
    }

    if (keyRaw[k] != keyStable[k] &&
        (uint32_t)(now - keyLastChange[k]) >= KEY_DEBOUNCE_MS) {

      keyStable[k] = keyRaw[k];

      const uint8_t pitch = keyNoteMap[k / KEY_NUM_IN][k % KEY_NUM_IN];
      if (pitch != 0) {
        DPRINTF("[KB] Tasto k=%d -> %s %s\n",
                k,
                (keyStable[k] ? "PREMO" : "RILASCIO"),
                midiNoteName(pitch));

        dispatchNote(keyStable[k] ? 1 : 0,
                     pitch,
                     keyStable[k] ? KEY_VELOCITY_ON : KEY_VELOCITY_OFF);
      }
    }
  }
}


/* ============================================================================
 *  SEZIONE 13 - I2C: INIZIALIZZAZIONE
 * ==========================================================================*/

static void initAds() {
  DPRINTLN("[I2C] Init ADS1115...");
  for (uint8_t g = 0; g < NUM_ADS_GROUPS; g++) {
    selectMux(adsGroups[g].muxAddr, adsGroups[g].muxCh);
    ads.begin(ADS_ADDR);
    ads.setGain(GAIN_ONE);
    ads.setDataRate(RATE_ADS1115_860SPS);

    DPRINTF("[I2C]   ADS group %d -> mux=0x%02X ch=%d\n",
            g, adsGroups[g].muxAddr, adsGroups[g].muxCh);
  }
  DPRINTF("[I2C] ADS1115 inizializzati: %d gruppi\n", NUM_ADS_GROUPS);
}

static void initPcf() {
  DPRINTLN("[I2C] Init PCF8574...");
  for (uint8_t g = 0; g < NUM_PCF_GROUPS; g++) {
    selectMux(pcfGroups[g].muxAddr, pcfGroups[g].muxCh);
    pcfWriteByte(PCF_ADDR, 0xFF);
    pcfGroups[g].outputState = 0xFF;

    DPRINTF("[I2C]   PCF group %d -> mux=0x%02X ch=%d\n",
            g, pcfGroups[g].muxAddr, pcfGroups[g].muxCh);
  }
  DPRINTF("[I2C] PCF8574 inizializzati: %d gruppi\n", NUM_PCF_GROUPS);
}


/* ============================================================================
 *  SEZIONE 14 - FILTRO ADATTIVO + SCANSIONE POTENZIOMETRI
 * ==========================================================================*/

/* --- Filtro adattivo --------------------------------------------------------
 *  Media mobile esponenziale con alpha variabile:
 *    |new - prev| >= 20  -> alpha = 255  (nessuna smoothing, reagisce subito)
 *    |new - prev| >= 10  -> alpha = 180  (smoothing leggero)
 *    |new - prev| >=  4  -> alpha = 100  (smoothing medio)
 *    |new - prev| >=  1  -> alpha =  40  (smoothing forte)
 *    |new - prev| ==  0  -> alpha =   0  (nessun aggiornamento)
 *
 *  Formula IIR in fixed-point 0-255:
 *    filtered = (alpha * new + (255 - alpha) * prev) / 255
 * --------------------------------------------------------------------------*/
static inline uint8_t adaptiveFilter(uint8_t prev, uint8_t newVal) {
  int16_t diff = (int16_t)newVal - (int16_t)prev;
  int16_t absDiff = (diff < 0) ? -diff : diff;

  uint8_t alpha;
  if      (absDiff >= 20) alpha = FILT_ALPHA_JUMP;
  else if (absDiff >= 10) alpha = FILT_ALPHA_BIG;
  else if (absDiff >=  4) alpha = FILT_ALPHA_MED;
  else if (absDiff >=  1) alpha = FILT_ALPHA_SMALL;
  else                    alpha = 0;

  if (alpha == 0) return prev;

  uint32_t r = (uint32_t)alpha * newVal
             + (uint32_t)(255 - alpha) * prev;
  return (uint8_t)((r + 127) / 255);
}

static void scanPots() {
  for (uint8_t g = 0; g < NUM_ADS_GROUPS; g++) {
    AdsGroup &grp = adsGroups[g];
    selectMux(grp.muxAddr, grp.muxCh);

    // Assestamento dopo il cambio canale del TCA9548A
    delayMicroseconds(POT_SETTLE_US);

    for (uint8_t i = 0; i < 4; i++) {
      if (grp.adcCh[i] == 0xFF) continue;

      // --- Lettura ADC ---
      int16_t raw = ads.readADC_SingleEnded(grp.adcCh[i]);
      if (raw < 0) raw = 0;

      // Mapping lineare raw -> 0..255 con clamping difensivo
      int32_t scaled = ((int32_t)raw * 255L) / ADS_FULL_SCALE;
      if (scaled > 255) scaled = 255;
      uint8_t v = (uint8_t)scaled;

      // --- Prima lettura in assoluto: inizializza senza inviare ---
      if (grp.lastMapped[i] < 0) {
        grp.filtered[i]   = v;
        grp.lastMapped[i] = v;
        continue;
      }

      // --- Filtro adattivo: smooth se piccolo, reattivo se grande ---
      uint8_t smoothed = adaptiveFilter(grp.filtered[i], v);
      grp.filtered[i] = smoothed;

      // --- Invio solo se il valore filtrato è cambiato abbastanza ---
      int16_t sdiff = (int16_t)smoothed - grp.lastMapped[i];
      if (sdiff < 0) sdiff = -sdiff;
      if (sdiff >= POT_THRESHOLD) {
        grp.lastMapped[i] = smoothed;
        sendPot(grp.cat[i], grp.id[i], smoothed);
      }
    }
  }
}


/* ============================================================================
 *  SEZIONE 15 - I2C: SCANSIONE BOTTONI
 * ==========================================================================*/

static void scanButtons() {
  const uint32_t now = millis();

  for (uint8_t g = 0; g < NUM_PCF_GROUPS; g++) {
    PcfGroup &grp = pcfGroups[g];

    bool hasButton = false;
    for (uint8_t p = 0; p < 8; p++) {
      if (grp.type[p] == PCF_TYPE_BUTTON) { hasButton = true; break; }
    }
    if (!hasButton) continue;

    selectMux(grp.muxAddr, grp.muxCh);
    uint8_t byte = pcfReadByte(PCF_ADDR);

    for (uint8_t p = 0; p < 8; p++) {
      if (grp.type[p] != PCF_TYPE_BUTTON) continue;

      bool pressed = ((byte >> p) & 1) == 0;
      uint8_t newState = pressed ? 1 : 0;

      if (newState != grp.debounced[p]) {
        if (grp.lastChange[p] == 0) {
          grp.lastChange[p] = now;
        } else if ((uint32_t)(now - grp.lastChange[p]) >= DEBOUNCE_MS) {
          grp.debounced[p] = newState;
          grp.lastChange[p] = 0;
          sendButton(grp.cat[p], grp.id[p], newState);
        }
      } else {
        grp.lastChange[p] = 0;
      }
    }
  }
}


/* ============================================================================
 *  SEZIONE 16 - SETUP
 * ==========================================================================*/

void setup() {
#if DEBUG_ENABLED
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) delay(10);
  delay(200);
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" Lud-WS Controller - Debug attivo");
  Serial.println("==============================================");
#endif

  Serial1.setTX(SERIAL1_TX_PIN);
  Serial1.setRX(SERIAL1_RX_PIN);
  Serial1.begin(SERIAL_BAUD);
  DPRINTF("[INIT] Serial1 (router) pronto a %lu baud\n", (unsigned long)SERIAL_BAUD);

  Wire.begin();
  Wire.setClock(100000);
  DPRINTLN("[INIT] I2C pronto a 100 kHz");

  initKeyboard();
  DPRINTF("[INIT] Tastiera pronta (base=%d F3, %d tasti)\n",
          KEY_BASE_NOTE, KEY_NUM);

  initAds();
  initPcf();

  DPRINTF("[INIT] Tast_Mode  = %d\n", Tast_Mode);
  DPRINTF("[INIT] Split_Note = %d (pitch = %d = %s)\n",
          Split_Note,
          KEY_BASE_NOTE + Split_Note,
          midiNoteName(KEY_BASE_NOTE + Split_Note));

  DPRINTLN("[INIT] Setup completato, entro nel loop");
  DPRINTLN("----------------------------------------------");
}


/* ============================================================================
 *  SEZIONE 17 - LOOP PRINCIPALE
 * ==========================================================================*/

void loop() {
  handleSerialRx();   // 1) comandi in arrivo
  scanKeyboard();     // 2) tastiera
  scanPots();         // 3) potenziometri (con filtro adattivo)
  scanButtons();      // 4) bottoni
                      // 5) LED (da definire)
}