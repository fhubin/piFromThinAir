/*
 * This sample code shows...
 * 
 * Author: Frédéric HUBIN (frederic.hubin@gmail.com)
 */

/*
 * Includes:
 */
#include <xo3labs_logs.h>
#include <xo3labs_latch.h>

/*
 * Input/Output defines:
 */
#define LED           4
#define GEIGER_TUBE   2 

/*
 * Latches definition:
 */
#define LED_LATCH         0
#define BUFFER_SIZE      32

/*
 * Global variables:
 */
byte readerOffset = 0;
volatile byte writerOffset = 0;

byte randomBitIndex = 0;
byte randomBit[6] = {0, 0, 0, 0, 0, 0};

unsigned long lastDuration = 0;
unsigned long lastTimestamp = 0;
unsigned long timestamps[BUFFER_SIZE];

unsigned long innerSample = 0;
unsigned long totalSample = 0;

/*
 * Code:
 */
void tubeImpulse () {

  // Store the pulse time:
  timestamps[writerOffset++] = millis();
  
  // Reset the writer offset to the head if required:
  if(writerOffset >= BUFFER_SIZE) {
    writerOffset = 0;
  }
}

void setupPinModes () {
  
  // Log beginning of the led pin mode setup:
  log("Set pin mode...");
  
  // Set the pin modes for the leds:
  pinMode(LED, OUTPUT);
  pinMode(GEIGER_TUBE, INPUT);

  // Log end of the led pin mode setup:
  log("Pin mode setup done!");  
}

void setupInterrupts () {
  interrupts();                                                       
  attachInterrupt(digitalPinToInterrupt(GEIGER_TUBE), tubeImpulse, FALLING); 
}

void setup () {

  // Set serial speed:
  Serial.begin(115200);

  // Log beginning of the setup:
  log("Start setup 'Pi from thin air'...");

  // Set the pin modes:
  setupPinModes();

  // Setup the latches:
  setupLatches();

  // Setup the interrupts:
  setupInterrupts();

  // Log end of the setup:
  digitalWrite(LED, LOW);    // switch off the led

  // Log end of the setup:
  log("Setup done");  
}

//
void bufferLoop () {

  // Check if there is any data to process:
  if(readerOffset != writerOffset) {

    unsigned short arrayIndex = randomBitIndex / 8;
    unsigned short bitIndex = randomBitIndex % 8;
    
    // Add one more bit to my randomBit:
    if(lastDuration > timestamps[readerOffset] - lastTimestamp) {        
      // We add a '1':
      randomBit[arrayIndex] |= bit(bitIndex);       
    }
    else {                                 
      // We add a '0':           
      randomBit[arrayIndex] &= ~bit(bitIndex); 
    }

    // Increase the bit index:
    randomBitIndex++;

    // Do we have enough bit?
    if (randomBitIndex == 48) {

      // Reset the index:
      randomBitIndex = 0;

      // Extract b and c:
      float b = getFloat(randomBit);
      float c = getFloat(randomBit + 3);
      
      // Log b & c:
      logPrefix();

      Serial.println("");
      
      // Is the sample inside the pie? If yes, increase the inner sample counter:
      innerSample += (b * b) + (c * c) < 1.0 ? 1 : 0;
      
      // Increase the total sample counter:
      totalSample++;

      // Compute pi:
      float pi = 4 * (float)innerSample / totalSample;

      // Log Pi:
      logPrefix();
      Serial.print("b = ");
      Serial.print(b, 4);
      Serial.print(" ; c = ");
      Serial.print(c, 4);
      Serial.print(" ; pi = ");
      Serial.print(pi, 4);
      Serial.println("");
    }

    // Update the last duration and timestamp :
    lastDuration = timestamps[readerOffset] - lastTimestamp;
    lastTimestamp = timestamps[readerOffset];

    // Advance the reader offset:
    readerOffset++;

    // Reset the reader offset to the head if required:
    readerOffset = readerOffset == BUFFER_SIZE ? 0 : readerOffset;
         
    setLatchIn(LED_LATCH, 50);
    digitalWrite(LED, HIGH);    // switch on the led  
  }

  if(checkLatch(LED_LATCH)) {
    digitalWrite(LED, LOW);    // switch off the led
  }
}

float getFloat (byte * randomBitArray) {

  unsigned long base = 0;

  unsigned long a = randomBitArray[0];
  unsigned long b = randomBitArray[1];
  unsigned long c = randomBitArray[2];

  base += a;    
  base += b << 8;    
  base += c << 16;

  return base / ((float)(0x1000000));  
}

// Main loop:
void loop () {

  // Delegate to the buffer loop:
  bufferLoop();
  
}
