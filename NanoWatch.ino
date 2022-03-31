#include "Streaming.h"

///////////////////////////////////////////////////////////////
//                                                           //
//           Serial Based relay Arduino WatchDog             //
//                                                           //
//    By Tiziano Arbizzani <tiziano.arbizzani@gmail.com>     //
//                                                           //
///////////////////////////////////////////////////////////////

/// DEFINES ///

#define SEC (1000)       // Milliseconds in a second (Functional define only)

#define RELAYPIN (2)     // Relay Control pin
#define BUTTONPIN (3)    // Manual Reset pin
#define HB_TX_PIN (11)   // LED pin (HearthBeat Received)
#define WARNG_PIN (10)   // LED pin (Reset Countdown)
#define TX_LOCKER (4)    // NPN Transistor Serial Lock pin

#define LEDFLASH (10)    // Led Flash time

#define RED_LED_DC (50)  // Red Led On duty-cycle
#define BLU_LED_DC (80)  // Blue Led On duty-cycle
#define YEL_LED_DC (100) // Yellow Led On duty-cycle

#define WD_TIMEOUT_MS (30 * SEC)     // Wait for a strobe for ...
#define RESET_DURATION_MS (5 * SEC)  // Reset Time
#define WARNING_FLASHES (10)         // How many times "warning led" will flash before reset

/// GLOBAL VARIABLES ///

unsigned long ResetAt = 0;  // Time to next state change (if not triggered again). Zero means WD disabled
unsigned long Remains = 0;  // Serial Print remaining time
const char cStaboner = '*'; // Char Sent via Serial Connection

/// FUNCTION PROTOTYPES ///

void reset_procedure(); // Relay management function
void led_strobe(uint8_t pin, int dc, int dms); // PWM Led flash function

/// SETUP ///

void setup() 
{
  Serial.begin(115200);  // Serial Start
  Serial << "[INIT]" << endl;

  // TX Locker (BC237 Transistor)
  pinMode(TX_LOCKER, OUTPUT);
  digitalWrite(TX_LOCKER, HIGH);

  // 
  pinMode(HB_TX_PIN, OUTPUT);
  digitalWrite(HB_TX_PIN, LOW);

  pinMode(RELAYPIN, OUTPUT);
  digitalWrite(RELAYPIN, LOW);

  pinMode(BUTTONPIN, INPUT);
  Serial << "[PINS]" << endl;

  ResetAt = millis() + WD_TIMEOUT_MS;
  Serial << "[TIME]" << endl;
}

/// LOOP ///

void loop() {

  unsigned long CurrentRemains;

  if (Serial.available()) // We have bytes in serial buffer ...
  {
    if (Serial.read() == cStaboner) // We received watchdog char ('*')
    {
      Serial << "[RSTD]" << endl;
      ResetAt = millis() + WD_TIMEOUT_MS; // Set new reset time
      led_strobe(HB_TX_PIN, BLU_LED_DC, LEDFLASH); // Flash for received char
    }
  }
 
  if (!digitalRead(BUTTONPIN)) reset_procedure(); // Reset by button
      
  if (millis() >= ResetAt) reset_procedure(); // Reset by timeout

  // Check for time remaining before
  CurrentRemains = (ResetAt - millis()) / SECONDS;
  if (CurrentRemains != Remains) 
  {
    Remains = CurrentRemains;
    Serial << "[ " << _WIDTHZ(Remains, 2) << " ]" << endl; // Print remaining seconds

    // Manage "warning flash" led
    if (Remains < WARNING_FLASHES) 
    {
      led_strobe(WARNG_PIN, YEL_LED_DC, LEDFLASH); // Flash yellow led
    }
  }
}

/// RESET MANAGEMENT ///

void reset_procedure() 
{
  Serial << "[THRW]" << endl;
  digitalWrite(RELAYPIN, HIGH);  // Throw relay (via 4N35 OptoCoupler + BD239 Transistor)
  digitalWrite(TX_LOCKER, LOW);  // Cut-off serial connection (via BC237 transistor)

  delay(RESET_DURATION_MS); // Waiting 'off' time ...

  digitalWrite(RELAYPIN, LOW);    // Release relay (via 4N35 OptoCoupler + BD239 Transistor)
  digitalWrite(TX_LOCKER, HIGH);  // Re-enable serial connection (via BC237 transistor)
  ResetAt = millis() + WD_TIMEOUT_MS; // Set new reset time
  Serial << "[RLSE]" << endl;
}

/// LED FLASH FUNCTION ///

void led_strobe(uint8_t pin, int dc, int dms) 
{
  analogWrite(pin, dc); // Start PWM at set duty-cycle
  delay(dms); // Wait (syncronous, but it's very fast ...)
  analogWrite(pin, 0); // Stop PWM
}