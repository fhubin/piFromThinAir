/*
 * Compute pi from thin air.
 * Author: Frédéric HUBIN (frederic.hubin@gmail.com)
 */
#define GEIGER_TUBE  2
#define BUFFER_SIZE 32

byte readerOffset = 0;
volatile byte writerOffset = 0;

byte randomBitIndex = 0;
byte randomBit[6] = {0, 0, 0, 0, 0, 0};

unsigned long lastDuration = 0;
unsigned long lastTimestamp = 0;
volatile unsigned long timestamps[BUFFER_SIZE];

unsigned long innerSample = 0;
unsigned long totalSample = 0;

void tubeImpulse () {

  // Store the pulse time:
  timestamps[writerOffset++] = millis();
  
  // Reset the writer offset to the head if required:
  if(writerOffset >= BUFFER_SIZE) {
    writerOffset = 0;
  }
}

void setup () {

  //
  Serial.begin(115200);

  //
  pinMode(GEIGER_TUBE, INPUT);

  // Setup the interrupts:
  interrupts();
  attachInterrupt(digitalPinToInterrupt(GEIGER_TUBE), tubeImpulse, FALLING);

}

//
void bufferLoop () {

    // Check if there is any data to process:
    if(readerOffset == writerOffset) {
        return;
    }

    unsigned short arrayIndex = randomBitIndex / 8;
    unsigned short bitIndex = randomBitIndex % 8;

    // Add one more bit to my randomBit:
    if(lastDuration > timestamps[readerOffset] - lastTimestamp) {
      // We add a '1':
      randomBit[arrayIndex] |= bit(bitIndex);
      randomBitIndex++;
    }
    else if(lastDuration < timestamps[readerOffset] - lastTimestamp) {
      // We add a '0':
      randomBit[arrayIndex] &= ~bit(bitIndex);
      randomBitIndex++;
    }
    else {
      // NOP
    }

    // Do we have enough bit?
    if (randomBitIndex == 48) {

      // Reset the index:
      randomBitIndex = 0;

      // Extract b and c:
      float b = getFloat(randomBit);
      float c = getFloat(randomBit + 3);

      // Is the sample inside the pie? If yes, increase the inner sample counter:
      innerSample += (b * b) + (c * c) <= 1.0 ? 1 : 0;

      // Increase the total sample counter:
      totalSample++;

      // Compute pi:
      float pi = 4 * (float) innerSample / totalSample;

      // Log Pi:
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

}

float getFloat (byte * randomBitArray) {

  unsigned long base = 0;

  unsigned long a = randomBitArray[0];
  unsigned long b = randomBitArray[1];
  unsigned long c = randomBitArray[2];

  base += a;    
  base += b << 8;    
  base += c << 16;

  return base / ((float)(0xFFFFFF));  
}

// Main loop:
void loop () {

  // Delegate to the buffer loop:
  bufferLoop();

}
