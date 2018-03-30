#ifndef REARM_H__
#define REARM_H__


//Config for ReArm v1.0


//NOTES:
// Filament detector pin and Fan RPM pin must be on a spare pin on Port0 or Port2 only (UNTESTED)
// Azteeg X5 (and maybe others) probe endstop pin is not an ADC pin, so only Digital is supported, or select another spare ADC capable pin
// Note. ADC inputs are NOT 5V tolerant
// Currently No support For Thermocouple etc sensors, only thermistors!


//TODO:: implement firmware update
//#define IAP_FIRMWARE_FILE "RepRapFirmware-AzteegX5Mini1_1.bin"

// Default board type
#define DEFAULT_BOARD_TYPE BoardType::ReArm1_0
#define ELECTRONICS "ReArm v1.0"

#define REARM1_0



// The physical capabilities of the machine

// The number of drives in the machine, including X, Y, and Z plus extruder drives
const size_t DRIVES = 5;

// Initialization macro used in statements needing to initialize values in arrays of size DRIVES.  E.g.,
// max_feed_rates[DRIVES] = {DRIVES_(1, 1, 1, 1, 1, 1, 1, 1, 1, 1)}
#define DRIVES_(a,b,c,d,e,f,g,h,i,j,k,l) { a,b,c,d,e }

// The number of heaters in the machine
// 0 is the heated bed even if there isn't one.
const size_t Heaters = 2; //ReArm (Bed + Hotend1) // set to 3 if using 2nd heater and update macro below, and  Tempsense and heat pins below and remove(chance to another) cooling fan pin too

// Initialization macro used in statements needing to initialize values in arrays of size HEATERS.  E.g.,
#define HEATERS_(a,b,c,d,e,f,g,h) { a,b }

const size_t MinAxes = 3;						// The minimum and default number of axes
const size_t MaxAxes = 5;						// The maximum number of movement axes in the machine, usually just X, Y and Z, <= DRIVES
// Initialization macro used in statements needing to initialize values in arrays of size MAX_AXES
#define AXES_(a,b,c,d,e,f,g,h,i) { a,b,c,d,e }

const size_t MaxExtruders = DRIVES - MinAxes;	// The maximum number of extruders
const size_t MaxDriversPerAxis = 2;				// The maximum number of stepper drivers assigned to one axis


// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to NoPin to flag unavailability.
// DRIVES
//                                              X      Y      Z     E1     E2
const Pin ENABLE_PINS[DRIVES] =             { P0_10, P0_19, P0_21, P0_4,  P4_29};
const Pin STEP_PINS[DRIVES] =               { P2_1,  P2_2,  P2_3,  P2_0,  P2_8};
const uint8_t STEP_PIN_PORT2_POS[DRIVES] =  { 1,     2,     3,     0,     8}; //SD: Used for calculating bitmap for stepping drivers (this is position of the pins on the port)
const uint32_t STEP_DRIVER_MASK =           0x0000010F; //SD: mask of the step pins on Port 2 used for writing to step pins in parallel
const Pin DIRECTION_PINS[DRIVES] =          { P0_11, P0_20, P0_22, P0_5,  P2_13};



// Endstops
// Note: RepRapFirmware only as a single endstop per axis
//       gcode defines if it is a max ("high end") or min ("low end")
//       endstop.  gcode also sets if it is active HIGH or LOW
//

//RE-Arm has 6 endstops. We will assume MAX endstops headers are used, leaving P1_24, 1_26 free for other purposes. 1_29 (Z-min used for probe)

const Pin END_STOP_PINS[DRIVES] = { P1_25, P1_27, P1_28, NoPin, NoPin}; // E stop could be mapped to a spare pin if needed...


//RaArm has no current control for drivers.

// Indices for motor current digipots (X,Y,Z,E1, E2)
//const uint8_t POT_WIPES[5] = { 0, 1, 2, 3, 0};
//const float digipotFactor = 106.0; //factor for converting current to digipot value

// HEATERS - The bed is assumed to be the at index 0

// Analogue pin numbers
//                                            Bed    H1
const Pin TEMP_SENSE_PINS[Heaters] = HEATERS_(P0_24, P0_23, /*P0_25*/c, d, e, f, g, h);


// Heater outputs

// Note: P2_0 to P2_5 is hardware PWM capable, P2_7 is not

const Pin HEAT_ON_PINS[Heaters] = HEATERS_(P2_7, P2_5, /*P2.4*/c, d, e, f, g, h); // bed, h0

// Default thermistor betas
const float BED_R25 = 100000.0;
const float BED_BETA = 4066.0;
const float BED_SHC = 0.0;
const float EXT_R25 = 100000.0;
const float EXT_BETA = 4066.0;
const float EXT_SHC = 0.0;

// Thermistor series resistor value in Ohms
const float THERMISTOR_SERIES_RS = 4700.0;


//???
const size_t MaxSpiTempSensors = 1;
// Digital pins the 31855s have their select lines tied to
const Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { NoPin };


// Digital pin number that controls the ATX power on/off
const Pin ATX_POWER_PIN = NoPin;

// Z Probe pin
// Must be an ADC capable pin.  Can be any of the ARM's A/D capable
// pins even a non-Arduino pin.

//Note: Azteeg X5 uses pin P1_29 which is NOT an ADC pin. Use a spare if need Analog in, else use digital options for probe
const Pin Z_PROBE_PIN = P1_29;

// Digital pin number to turn the IR LED on (high) or off (low)
const Pin Z_PROBE_MOD_PIN = NoPin;

// Use a PWM capable pin
const size_t NUM_FANS = 1;
const Pin COOLING_FAN_PINS[NUM_FANS] = { P2_4 }; // Fan 0 is a Hardware PWM pin

// Firmware will attach a FALLING interrupt to this pin
// see FanInterrupt() in Platform.cpp
// SD:: Note: Only GPIO pins on Port0 and Port2 support this. If needed choose from spare pins
const Pin COOLING_FAN_RPM_PIN = NoPin;


//SD: Internal SDCard is on SSP1
//    MOSI, MISO, SCLK, CS
//    P0_9, P0_8, P0_7, P0_6

//SD:: 2nd SDCard can be connected to SSP0
//    MOSI, MISO, SCLK
//    P0_18 P0_17 P0_15

// SD cards
//sd:: Internal SD card is on SSP1
//NOTE::: Although this is 2nd in the List, SSP1 is Configured to be Slot0 in coreNG to be compatible with RRF
//default to supporting 2 card..... if need 1_23 then change CS no No pin

const size_t NumSdCards = 2;//Note: use 2 even if only using 1 (internal) card
const Pin SdCardDetectPins[NumSdCards] = { NoPin, P1_31 };
const Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
const Pin SdSpiCSPins[NumSdCards] = { P0_6, P1_23 };// Internal, external. If need 1_23 pin, and no ext sd card set to NoPin Note:: ("slot" 0 in CORE is configured to be LCP SSP1 to match default RRF behaviour)
// Definition of which pins we allow to be controlled using M42
// Spare pins on the Arduino Due are
//
//  D5 / TIOA6  / C.25
//  D6 / PWML7  / C.24
// ### Removed: now E0_AXIS endstop D39 / PWMH2  / C.7
// D58 / AD3    / A.6
// D59 / AD2    / A.4
// ### Removed: now E6_DIR ExtV3 D66 / DAC0   / B.15
// ### Removed: now E6_STP ExtV3 D67 / DAC1   / B.16
// ### Removed: now E6_EN ExtV3 D68 / CANRX0 / A.1
// ### Removed: now MSi(=3.3V) ExtV3 D69 / CANTX0 / A.0
// D70 / SDA1   / A.17
// D71 / SCL1   / A.18
// D72 / RX LED / C.30
// D73 / TX LED / A.21

// M42 and M208 commands now use logical pin numbers, not firmware pin numbers.
// This is the mapping from logical pins 60+ to firmware pin numbers


//TODO:: default to spare pins from EXP1 and 2 (no support for LCD etc yet)

const Pin SpecialPinMap[] =
{
	NoPin
};

//Azteeg X5 Mini 1.1 EXP1 Header (not compat with V2.0 etc etc)

// P1_30        P1_22
// P0_26        P0_25
// P0_27(SDA)   P4_29
// P0_28(SCL)   P2_8
// 3.3v         GND

//Azteeg X5 Mini 1.1 Exp2 Header

// P1_31    P3_26
// P2_11    P3_25
// P1_23    P0_17
// P0_16    P2_6
// P0_15    P0_18
// GND      +5V



// Flash locations (may be expanded in the future)
//const uint32_t IAP_FLASH_START = 0x000F0000;
//const uint32_t IAP_FLASH_END = 0x000FFBFF;		// don't touch the last 1KB, it's used for NvData


/*
#ifdef LCD_UI

// Hardware I2C support for LCD
#define TWI_ID           ID_TWI1

#define FEATURE_CONTROLLER              7
#define UI_PAGES_DURATION            4000
#define UI_ANIMATION                    0
#define UI_SPEEDDEPENDENT_POSITIONING   0
#define UI_DISABLE_AUTO_PAGESWITCH      1
#define UI_AUTORETURN_TO_MENU_AFTER 30000
#define UI_ENCODER_SPEED                1
#define UI_KEY_BOUNCETIME              10
#define UI_KEY_FIRST_REPEAT           500
#define UI_KEY_REDUCE_REPEAT           50
#define UI_KEY_MIN_REPEAT              50
#define FEATURE_BEEPER                  1
#define UI_START_SCREEN_DELAY        1000

#define CASE_LIGHTS_PIN                -1
#define SPI_PIN                        77
#define SPI_CHAN                        0
#define UI_HAS_KEYS                     1
#define UI_HAS_BACK_KEY                 1
#define UI_DISPLAY_TYPE                 1
#define UI_DISPLAY_CHARSET              1
#define BEEPER_TYPE                     1
#define UI_COLS                        20
#define UI_ROWS                         4
#define BEEPER_PIN                     41
#define UI_DISPLAY_RS_PIN              42
#define UI_DISPLAY_RW_PIN              -1
#define UI_DISPLAY_ENABLE_PIN          43
#define UI_DISPLAY_D0_PIN              44
#define UI_DISPLAY_D1_PIN              45
#define UI_DISPLAY_D2_PIN              46
#define UI_DISPLAY_D3_PIN              47
#define UI_DISPLAY_D4_PIN              44
#define UI_DISPLAY_D5_PIN              45
#define UI_DISPLAY_D6_PIN              46
#define UI_DISPLAY_D7_PIN              47
#define UI_ENCODER_A                   52
#define UI_ENCODER_B                   50
#define UI_ENCODER_CLICK               48
#define UI_RESET_PIN                   -1
#define UI_DELAYPERCHAR                40
#define UI_INVERT_MENU_DIRECTION        0
#define UI_BUTTON_BACK                 71

// Beeper sound definitions for short beeps during key actions and longer
// beeps for important actions.  Parameters are the delay in microseconds
// followed by the number of repetitions.  Values must be in range 1..255
#define BEEPER_SHORT_SEQUENCE         2,2
#define BEEPER_LONG_SEQUENCE          8,8

#endif // LCD_UI
*/
#endif
