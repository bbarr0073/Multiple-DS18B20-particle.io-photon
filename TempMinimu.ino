/*
 * Project TempSensors
 * Description: A flexible program that has many options to configure any number of DS18B20 devices on a single OneWire bus
 * Author: John Carrieres
 * Date: 6/21/2019
 */

// If ONLY_ONE DS18B20 is being put on the bus AND you don't want to find and input is UNIQUE address, 
// search for "ONLY_ONE" to find the five or six places in code that need to change 
// I haven't actually tested that though...


SYSTEM_THREAD(ENABLED);


#define ONEWIRE_SEARCH 0  // OneWire option: ignore the search code for devices/device addresses
#define ONEWIRE_CRC 1     // OneWire option: enable the CRC code
#define ONEWIRE_CRC16 0   // OneWire option: ignore 16-bit CRC code (redundant since CRC is eliminated on prior line)

#include "OneWire.h"
#include <tgmath.h>       // Only needed for the fabs() function...thinking about getting rid of this


//ds18b20 resolution is determined by the byte written to it's configuration register
enum DS18B20_RESOLUTION   : uint8_t {
  DS18B20_9BIT  = 0x1f,         //   9 bit   93.75 ms conversion time
  DS18B20_10BIT = 0x3f,         //  10 bit  187.50 ms conversion time
  DS18B20_11BIT = 0x5F,         //  11 bit  375.00 ms conversion time
  DS18B20_12BIT = 0x7F,         //  12 bit  750.00 ms conversion time
};

//if ds18b20 resolution is less than full 12-bit, the low bits of the data should be masked...
enum DS18B20_RES_MASK   :   uint8_t {
  DS18B20_9BIT_MASK  = 0xf8,        
  DS18B20_10BIT_MASK = 0xfc,      
  DS18B20_11BIT_MASK = 0xfe,        
  DS18B20_12BIT_MASK = 0xff,       
};

//ds18b20 conversion time is ALSO determined by the byte written to it's configuration register
enum DS18B20_CONVERSION_TIME   : uint16_t {
  DS18B20_9BIT_TIME  = 94,          //   9 bit   93.75 ms conversion time w/pad
  DS18B20_10BIT_TIME = 188,         //  10 bit  187.50 ms conversion time w/pad
  DS18B20_11BIT_TIME = 375,         //  11 bit  375.00 ms conversion time w/pad
  DS18B20_12BIT_TIME = 750,         //  12 bit  750.00 ms conversion time w/pad
};

#define DS18B20_PIN_ONEWIRE D2                       //  my system implements OneWire on Photon pin D2
#define NUM_DS18B20_DEVICES 5                        //  my system has FIVE DS18B20 devices attached to OneWire (on pin D2)
#define DS18B20_CONVERSION_TIME DS18B20_12BIT_TIME   //  match desired enumerated conversion time above
#define DS18B20_RESOLUTION  DS18B20_12BIT            //  match desired enumerated resolution above
#define DS18B20_RES_MASK DS18B20_12BIT_MASK          //  match desired enumerated resolution mask above (low bits at lower resolutions mean nothing)
#define DS18B20_CRC_RETRIES 2                        //  define how many DS18B20 CRC failure retries are done before moving on
#define DS18B20_FAIL_CRC_VALUE 0x07ff                //  returned when a CRC Fail Condition occurs: =2047 decimal...177 degree celsius...way outside of spec 
#define DS18B20_TEMP_HI_REG 0x55                     //  set to a known value, checkerboard pattern (could be used to abort a "going to fail" crc check)
#define DS18B20_TEMP_LO_REG 0xAA                     //  set to a known value, checkerboard pattern (ditto)

#define DS18B20_SAMPLE_INTERVAL  1000       //  defines project specific DS18B20 Sampling Interval, which determines how often to sample the 
                                            //  DS18B20 devices...in this code, interval reschedules automatically, but could be changed or
                                            //          implemented as a one-shot. 
                                            //  .....for periodic sampling, should be set to: DS18B20_CONVERSION_TIME + pad....but it doesn't matter
                                            //  IF SET to 0, temperature conversions are started and re-started as quickly as possible

//Publishing definitions and variables
#define PUBLISH_MAX_INTERVAL       300000     // every 5 minutes
#define PUBLISH_MIN_INTERVAL       1000     // every 1 second, publishing can occur this fast if a function requests a publishNOW, minimum 1000

  // publishing temperature differential, publish the data immediately (subject to PUBLISH_MIN_INTERVAL) if its 
  // temperature differential from the previous PUBLISHED value is greater than this number...used in a floating point comparison
  // ...An easy way to get quick publishes during testing...grab a probe with your hand and raise its temperature
#define PUBLISH_TEMPERATURE_DIFF      1     


bool publishNOW;                            // a particular function may request an immediate publish by setting this true
unsigned long currentMillis;                // set at the beginning of each pass through loop()
long int conversion_count_DS18B20;              //TESTING CODE , keeps track of the total # of DS18B20 conversions
long int crc_error_count_DS18B20;               //TESTING CODE , keeps track of the total # of CRC errors in DS18B20 conversions
long int crc_fail_count_DS18B20;                //TESTING CODE , keeps track of the total # of CRC failures (all tries) in DS18B20 conversions


// OneWire DS18B20 8-byte unique addresses that must be obtained and entered into the table below
// Use the OneWire example code called Address_Scanner to gather these...
// ...then input them into the array below.  
// Trying to to discover the addresses on the fly has proven to be troublesome for many who use multiple DS18B20 devices on a single OneWire
// ...it's more efficient to determine them and save them once forever, unless sensors in your system are constantly being swapped 
// ...for new ones...NOTE: finding addresses might be easy now with the FIX to the OneWire code that has been floated out there
// ONLY_ONE: addresses are not needed if there is ONLY_ONE DS18B20 on a OneWire bus, it is faster to not use them, but you still can use them
// If you don't want to find this unique addess for ONLY_ONE device, see the top of this file regarding ONLY_ONE device.
//
// ********* REPLACE THESE ADDRESSES WITH THE UNIQUE ADDRESSES OF YOUR DS18B20 DEVICES **************
//
const uint8_t DS18B20_OneWire_ADDRESSES[NUM_DS18B20_DEVICES][8] = 
    {0x28, 0xAA, 0x8D, 0x68, 0x3F, 0x14, 0x01, 0x2E,    // address of first DS18B20 device
     0x28, 0xAA, 0x49, 0x88, 0x3F, 0x14, 0x01, 0x5A,    // address of 2nd DS18B20 device
     0x28, 0xAA, 0x49, 0x67, 0x3F, 0x14, 0x01, 0x89,    // ..
     0x28, 0xAA, 0xB3, 0x6E, 0x3F, 0x14, 0x01, 0x01,    // ..
     0x28, 0xAA, 0xF6, 0x6F, 0x3C, 0x14, 0x01, 0x51};   // address of last DS18B20 device


int16_t current_temps_raw[NUM_DS18B20_DEVICES];    // current raw readings from temperature sensors
float f_current_temps[NUM_DS18B20_DEVICES];        // current temperature readings from sensors
float f_current_temps_pub[NUM_DS18B20_DEVICES];    // last published temperatures readings from sensors


// Function declarations
void start_DS18B20_Conversions();   
int16_t read_DS18B20_Conversion(const uint8_t addr[8], uint8_t ptr);
void doTemperatureCalculations();
bool DS18B20_SamplingComplete();
bool publishAllStatus();
bool publishNonBlocking(const char sheet_name, const char message);
void publishData();
bool timeToPublish();


OneWire ds18b20_onewire(DS18B20_PIN_ONEWIRE);   // instantiate the OneWire bus


void setup() {
  set_DS18B20_Resolutions(DS18B20_RESOLUTION); 

}


void loop() 
{
  currentMillis = millis();

  // Publish the status if conditions are met
  if (timeToPublish()) publishData();

  // When ready, update the current DS18B20 temperature readings
  if (DS18B20_SamplingComplete()) doTemperatureCalculations();
}



// function that publishes selected data...this will be expanded
void publishData(){
  if (publishAllStatus()) {     // function attempts to publish the status
    publishNOW = false;         // ...if successful then get ready for next publish
    for (uint8_t i = 0; i < NUM_DS18B20_DEVICES; i++) {
      f_current_temps_pub[i] = f_current_temps[i];  // update the published temperaturre data
      //other stuff to be added here
    }
  }
}

// function to check if it is time to Publish: either forced (publishNOW) or a timeout of the PUBLISH_MAX_INTERVAL
// and then setup for the next publish event
bool timeToPublish() {
  static long prior_publish_time;      
  if (((currentMillis - prior_publish_time >= PUBLISH_MIN_INTERVAL) && publishNOW) ||
      (currentMillis - prior_publish_time >= PUBLISH_MAX_INTERVAL)) {
    prior_publish_time = currentMillis;                    // setup for the next publish time
    //publishNOW = false;

    return(true);
  }
  return(false);
}


// This code starts a conversion on all DS18B20s simultaneously, and then, later when the conversions are finished, reads the results
// There is only one sampled conversion for each DS18B20..if the sampled conversion fails the CRC checks, a previous sampled conversion is kept 
// Since there is no rush to get these conversions recorded, this function is designed so 
// ...that only one conversion read happens on any given pass through it.  This avoids cramming 
// ...a bunch of execution time into one particular pass of the user code.
bool DS18B20_SamplingComplete() {
  static long prior_DS18B20_interval_start = 10000; 
  static long prior_DS18B20_conversion_start = 10000;
  static long current_DS18B20_interval_start = 20000;
  static int16_t temperature_read_raw;
  static uint8_t DS18B20_ptr = 0;
  static bool DS18B20_conversion_reads_in_progress = false;


  // Enter the code body ONLY if within a valid DS18B20 sampling interval window AND prior DS18B20 temperature conversions have had time to complete
  if (((currentMillis - prior_DS18B20_conversion_start) >= DS18B20_CONVERSION_TIME)  && 
      ((currentMillis - prior_DS18B20_interval_start) >= DS18B20_SAMPLE_INTERVAL)) {

    if (!DS18B20_conversion_reads_in_progress && (DS18B20_ptr == 0)) {      
        // starts temperature conversions on all DS18B20 devices attached to the OneWire bus           
      start_DS18B20_Conversions(); 
      prior_DS18B20_conversion_start = millis();                         // capture conversion start so the "reads" can be scheduled
      current_DS18B20_interval_start = prior_DS18B20_conversion_start;   // capture the start time so next interval can be scheduled
      DS18B20_conversion_reads_in_progress = true;
      conversion_count_DS18B20 += NUM_DS18B20_DEVICES; //TESTING: keeps track of the # of temperature conversions since reset
    }
    else if (DS18B20_conversion_reads_in_progress) {
        // reads one of the DS18B20 temperature conversions
      temperature_read_raw = read_DS18B20_Conversion(DS18B20_OneWire_ADDRESSES[DS18B20_ptr], DS18B20_ptr); //if ONLY_ONE DS18B20, take out the address reference
      
      if (temperature_read_raw != DS18B20_FAIL_CRC_VALUE)
        current_temps_raw[DS18B20_ptr] = temperature_read_raw; 
      else crc_fail_count_DS18B20++;  //TESTING else keep the old value, there were CRC failures on the intial read AND retries

      if (++DS18B20_ptr >= NUM_DS18B20_DEVICES)   
        DS18B20_conversion_reads_in_progress = false;  // all DS18B20 conversions have been read
    }
    else {              // all sampled conversion have been recorded, so setup for the next DS18B20 sample interval 
      DS18B20_ptr = 0;                  //  setup to read the sensors again
      prior_DS18B20_interval_start =    // check if (for any reason) it took longer than DS18B20_SAMPLE_INTERVAL to get the conversions
        ((currentMillis - current_DS18B20_interval_start) > DS18B20_SAMPLE_INTERVAL) ? millis() : current_DS18B20_interval_start;
      return(true);
    }
  }
  return(false);
}


// This does the temperature calculations (from the RAW values) and stores them,
// the latest results are updated and always available within these global arrays: 
// RAW values: current_temps_raw[NUM_DS18B20_DEVICES], these are the integer values read from the sensors
// current temperatures:  f_current_temps[NUM_DS18B20_DEVICES]
void doTemperatureCalculations() {
  float temperature;
  for (uint8_t i = 0; i < NUM_DS18B20_DEVICES; i++) {
    //temperature = current_temps_raw[i] / 16.0;  // this is the Celsius calculation read from the ds18b20
    temperature = current_temps_raw[i] / 16.0 * 1.8 + 32;  // this is the Farenheit calculation read from the ds18b20
         // force a publish if temperature has changed by more than 1 degree since last published
    if (fabs(f_current_temps_pub[i] - temperature) > PUBLISH_TEMPERATURE_DIFF) publishNOW = true;  
    f_current_temps[i] = temperature; 
  }
}



// this function sets the resolution for ALL ds18b20s on an instantiated OneWire
void set_DS18B20_Resolutions(uint8_t resolution)  
{
  ds18b20_onewire.reset();        // onewire intialization sequence, to be followed by other commands
  ds18b20_onewire.write(0xcc);    // onewire "SKIP ROM" command, selects ALL ds18b20s on bus
  ds18b20_onewire.write(0x4e);    // onewire "WRITE SCRATCHPAD" command (requires write to 3 registers: 2 hi-lo regs, 1 config reg)
  ds18b20_onewire.write(DS18B20_TEMP_HI_REG);   // 1) write known value to temp hi register 
  ds18b20_onewire.write(DS18B20_TEMP_LO_REG);   // 2) write known value to temp lo register
  ds18b20_onewire.write(resolution);            // 3) write selected resolution to configuration registers of all ds18b20s
}


// this function intitalizes simultaneous temperature conversions for ALL ds18b20s on an instantiated OneWire
void start_DS18B20_Conversions()    
{
  ds18b20_onewire.reset();          // onewire intitialization sequence, to be followed by other commands
  ds18b20_onewire.write(0xcc);      // onewire "SKIP ROM" command, addresses ALL (or one if there is only one) ds18b20s on bus
  ds18b20_onewire.write(0x44);      // onewire wire "CONVERT T" command, starts temperature conversion on ALL ds18b20s
}

/* 
// A fast conversion read routine for a temperature from from a DS18B20 device, no CRC checking and
// avoids having to read all 9 bytes over that slooooooowwww OneWire
int16_t fast_read_DS18B20_Conversion(const uint8_t addr[8])  
{
  byte  data[2];

  ds18b20_onewire.reset();          // onewire intitialization sequence, to be followed by other commands
  ds18b20_onewire.select(addr);     // issues onewire "MATCH ROM" address which selects a SPECIFIC (only one) ds18b20 device
  ds18b20_onewire.write(0xBE);      // onewire "READ SCRATCHPAD" command, to access selected ds18b20's scratchpad
  data[0] = ds18b20_onewire.read(); // low byte of temperature conversion
  data[1] = ds18b20_onewire.read(); // high byte of temperature conversion
  ds18b20_onewire.reset();          // per spec...if not reading all 9 bytes of the scratchpad, a reset must be issued
  return ((int16_t) (data[1] << 8) | (data[0] & DS18B20_RES_MASK));
}
*/

// this function returns the RAW temperature conversion result of a SINGLE selected DS18B20 device (via it's address)
// If there is a CRC failure in the process, the previously converted result is just re-read...a new conversion is not started.
// It is reattempted up to DS18B20_CRC_RETRIES times
// The pointer to a particular DS18B20 was addeed as a parameter for testing purposes  to check if a particular DS18B20 device
// was having issues with the OneWire Protocol.   I'm leaving it for now
int16_t read_DS18B20_Conversion(const uint8_t addr[8], uint8_t ptr)  // if ONLY_ONE DS18B20, take out address reference: read_DS18B20_Conversion(uint8_t ptr)
{
  byte  data[9];
  bool crc_error;
  int crc_retries = 0;

  do {
    ds18b20_onewire.reset();          // onewire intitialization sequence, to be followed by other commands
    ds18b20_onewire.select(addr);     // issues onewire "MATCH ROM" address which selects a SPECIFIC (only one) ds18b20 device
      //if ONLY_ONE DS18B20, replace the line above  "ds18b20_onewire.select(addr);" with the one directly below
      //  ds18b20_onewire.write(0xcc);      // onewire "SKIP ROM" command, selects the ONLY_ONE ds18b20 on bus without needing address
      //
    ds18b20_onewire.write(0xBE);      // onewire "READ SCRATCHPAD" command, to access selected ds18b20's scratchpad
      // reading the bytes (9 available) of the selected ds18b20's scratchpad 
    for (int i = 0; i < 9; i++) data[i] = ds18b20_onewire.read();
      // check the crc
    crc_error = (data[8] != OneWire::crc8(data, 8));  


//TESTING Debug Code for CRC --------------------
// All of this code simply prints out CRC failures and their successful resolutions.   The failing CRC data can be compared
// to the passing CRC data...its a simple logic analyzer for OneWire CRC failures...
// this can be commented out if there is no interest in seeing the CRC errors if they occur
    float temperature;
    temperature = ((int16_t)((data[1] << 8) | (data[0] & DS18B20_RES_MASK)))/16*1.8+32;
    if (crc_error) crc_error_count_DS18B20++;  

    if (crc_error && crc_retries <= DS18B20_CRC_RETRIES) 
      Serial.printlnf("  CRC err #%02d:  %02x %02x %02x %02x %02x %02x %02x %02x %02x  device: %02d temp: %0.1f", 
          (crc_retries+1),data[8],data[7], data[6], data[5], data[4], data[3], data[2], data[1],data[0],ptr,temperature);
    else if (!crc_error && (crc_retries > 0)) {
      Serial.printlnf("  Actual Data:  %02x %02x %02x %02x %02x %02x %02x %02x %02x  device: %02d temp: %0.1f", 
          data[8],data[7], data[6], data[5], data[4], data[3], data[2], data[1],data[0],ptr, temperature);
      Serial.println();
      }
    else if (crc_error) Serial.println();
//TESTING ----------------------------


  } while ((crc_error && (crc_retries++ < DS18B20_CRC_RETRIES)));

    // if the temperature conversion was successfully read, pass it back...else return the CRC FAIL value 
  return (int16_t) (crc_error ?  DS18B20_FAIL_CRC_VALUE : ((data[1] << 8) | (data[0] & DS18B20_RES_MASK)));
}



//
// Publishes the status, in my case: specifically sends it to a Google spreadsheet and the PoolController Android app
// Formatting (using snprintf) changes as per Scruff recommendation
bool publishAllStatus() {
  const char gsSheet[] = "temp";  // google sheet page
  char stats[622];  // place holder for now

snprintf(stats, sizeof(stats),
      "{\"T0\":%.1f"
        ",\"T1\":%.1f"
        ",\"T2\":%.1f"
        ",\"T3\":%.1f"
        ",\"T4\":%.1f"
        ",\"CC\":%ld"
        ",\"ERR_CRC\":%ld"
        ",\"ERR_CRCF\":%ld"
      "}",
      f_current_temps[0],
      f_current_temps[1],
      f_current_temps[2],
      f_current_temps[3],
      f_current_temps[4],
      conversion_count_DS18B20,
      crc_error_count_DS18B20,
      crc_fail_count_DS18B20
    );

  return publishNonBlocking(gsSheet, stats);
}



// A wrapper around Partical.publish() to check connection first to prevent
// blocking. The prefix "pool-" is added to all names to make subscription easy.
// "name" is the "sheet" name within the google spreadsheet that this is being sent to
bool publishNonBlocking(const char* sheet_name, const char* message) {
    const char evtPrefix[] = "pool-";
    char evtName[sizeof(evtPrefix) + strlen(sheet_name)];

    snprintf(evtName, sizeof(evtName), "%s%s", evtPrefix, sheet_name);
    // TODO replace with a failure queue?
    if (Particle.connected()) {
        bool success = Particle.publish(evtName, message, PUBLIC); // TODO, need to understand ramifications of making this PRIVATE
//        Serial.printlnf("Published \"%s\" : \"%s\" with success=%d",
//                        name.c_str(), message.c_str(), success);
        return success;
    } else {
//        Serial.printlnf("Published \"%s\" : \"%s\" with success=0 no internet",
//                        name.c_str(), message.c_str());
    }
    return false;
}
