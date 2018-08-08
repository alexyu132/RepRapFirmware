#ifndef AZSMZ_H__
#define AZSMZ_H__


//Config for AZSMZ v1.0


//NOTES:
// Filament detector pin and Fan RPM pin must be on a spare pin on Port0 or Port2 only (UNTESTED)
// Azteeg X5 (and maybe others) probe endstop pin is not an ADC pin, so only Digital is supported, or select another spare ADC capable pin
// Note. ADC inputs are NOT 5V tolerant
// Currently No support For Thermocouple etc sensors, only thermistors!


//TODO:: implement firmware update
//#define IAP_FIRMWARE_FILE "RepRapFirmware-AzteegX5Mini1_1.bin"

// Default board type
#define DEFAULT_BOARD_TYPE BoardType::Azsmz
#define ELECTRONICS "AZSMZ Mini"

#define AZSMZ



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
const Pin ENABLE_PINS[DRIVES] =             { P0_4, P0_10, P0_19, P0_21,  P4_29};
const Pin STEP_PINS[DRIVES] =               { P2_0,  P2_1,  P2_2,  P2_3,  P2_8};
const uint8_t STEP_PIN_PORT2_POS[DRIVES] =  { 0,     1,     2,     3,     8}; //SD: Used for calculating bitmap for stepping drivers (this is position of the pins on the port)
const uint32_t STEP_DRIVER_MASK =           0x0000010F; //SD: mask of the step pins on Port 2 used for writing to step pins in parallel
const Pin DIRECTION_PINS[DRIVES] =          { P0_5, P0_11, P0_20, P0_22,  P2_13};



// Endstops
// Note: RepRapFirmware only as a single endstop per axis
//       gcode defines if it is a max ("high end") or min ("low end")
//       endstop.  gcode also sets if it is active HIGH or LOW
//

//RE-Arm has 6 endstops. We will assume MAX endstops headers are used, leaving P1_24, 1_26 free for other purposes. 1_29 (Z-min used for probe)

const Pin END_STOP_PINS[DRIVES] = { P1_24, P1_26, P1_28, NoPin, NoPin}; // E stop could be mapped to a spare pin if needed...


//RaArm has no current control for drivers.

// Indices for motor current digipots (X,Y,Z,E1, E2)
//const uint8_t POT_WIPES[5] = { 0, 1, 2, 3, 0};
//const float digipotFactor = 106.0; //factor for converting current to digipot value

// HEATERS - The bed is assumed to be the at index 0

// Analogue pin numbers
//                                            Bed    H1
const Pin TEMP_SENSE_PINS[Heaters] = HEATERS_(P0_25, P0_23, /*P0_24*/c, d, e, f, g, h);


// Heater outputs

// Note: P2_0 to P2_5 is hardware PWM capable, P2_7 is not

const Pin HEAT_ON_PINS[Heaters] = HEATERS_(P2_5, P2_4, /*P2.7*/c, d, e, f, g, h); // bed, h0

// Default thermistor betas
const float BED_R25 = 100000.0;
const float BED_BETA = 3950.0;
const float BED_SHC = 0.0;
const float EXT_R25 = 100000.0;
const float EXT_BETA = 3950.0;
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
const size_t NUM_FANS = 2;
const Pin COOLING_FAN_PINS[NUM_FANS] = { P2_7, P0_26 }; // Fan 0 is a Hardware PWM pin

// Firmware will attach a FALLING interrupt to this pin
// see FanInterrupt() in Platform.cpp
// SD:: Note: Only GPIO pins on Port0 and Port2 support this. If needed choose from spare pins (UNTESTED)
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
const Pin SdCardDetectPins[NumSdCards] = { NoPin, /*P3_25*/NoPin };
const Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
const Pin SdSpiCSPins[NumSdCards] = { P0_6, /*P0_16*/NoPin };// Internal, external. If need 1_23 pin, and no ext sd card set to NoPin Note:: ("slot" 0 in CORE is configured to be LCP SSP1 to match default RRF behaviour)


// Definition of which pins we allow to be controlled using M42

const Pin SpecialPinMap[] =
{
 NoPin
};


#endif
