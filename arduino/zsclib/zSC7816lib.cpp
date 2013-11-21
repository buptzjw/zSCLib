
#include "zSC7816lib.h"

/******************************************************************************
 * Definitions
 ******************************************************************************/

// STATES
enum state_t { START_STATE, FOUND_FIRST_FALLING_EDGE, FOUND_FIRST_RAISING_EDGE, SYNC_FOUND, READING_BITS, SENDING_BITS, PARITY_BIT, PARITY_ERROR, PARITY_READ, PRE_FINISHED, FINISHED };
//enum apdu_t0_state_t { SEND_HEADER, WAIT_ANSWER, SEND_DATA, RECEIVE_DATA, SEND_RESPONSE_SIZE, ADPU_FINISHED };

/******************************************************************************
 * Constructors
 ******************************************************************************/
/**
 * Creates a basic smart card reader object, with mandatory
 * pins assigned to establish smart card communication.
 *
 * c7_io               - IO line, connected to Card (C7)
 * c2_rst              - RST for Card (C2)
 * c1_vcc              - Vcc for Card (C1)
 * card_present        - Card present - HIGH - Card available - LOW - Card removed (Can be changed by card_present_invert)
 * c3_clk              - clk signal to SC (C3) - (timer1 / pin9 used)
 * card_present_invert - Use inverted off signal (LOW Card available - HIGH Card removed)
 */
CPUCardReader::CPUCardReader(uint8_t c7_io, uint8_t c2_rst, uint8_t c1_vcc, uint8_t card_present, uint8_t c3_clk, boolean card_present_invert) {
  _io_in_pin  = c7_io;
  _rstin_pin  = c2_rst;
  _cmdvcc_pin = c1_vcc;
  _off_pin    = card_present;
  _clk_pin    = c3_clk;

  _guardTime    = 2 * DEFAULT_ETU;
  _ignoreParity = true;
  _timeOutCB    = NULL;
  _off_invert   = card_present_invert;

  _includeTSinATR = true;

}

/******************************************************************************
 * User API
 ******************************************************************************/

//
// CARD INDEPENDENT FUNCTIONS
//
void CPUCardReader::_init(frequency_t freq) {
  pinMode(_off_pin, INPUT);
  pinMode(_cmdvcc_pin, OUTPUT);
  pinMode(_rstin_pin, OUTPUT);

  // Start with reading from Card
  pinMode(_io_in_pin, INPUT);

  // Default value for CMDVCCn and RSTIN
  digitalWrite(_cmdvcc_pin, LOW);
  digitalWrite(_rstin_pin, LOW);

  // Enable Debug PIN Toggling
  #if defined(USE_DEBUG_PIN)
  pinMode(DEFAULT_DEBUG_PIN, OUTPUT);
  digitalWrite(DEFAULT_DEBUG_PIN, HIGH);
  #endif
  
  _synchronous = false;
  _clkFrequency = freq;
  
  switch(freq) {
    case CLK_2MHZ:
      _initial_etu  = 186;
      _guardTime    = 2 * _initial_etu;
      _ocra1        = 3;
      break;
    case CLK_2DOT5MHZ:
      _initial_etu  = 148;
      _guardTime    = 2 * _initial_etu;
      _ocra1        = 2;
      break;
    case CLK_4MHZ:
      _initial_etu  = 93;
      _guardTime    = 2 * _initial_etu;
      _ocra1        = 1;
      break;
    case CLK_1MHZ:
      _initial_etu  = DEFAULT_ETU;
      _guardTime    = 2 * _initial_etu;
      _ocra1        = 7;
      break;
    default:
      // NO CLK generation ...
      _initial_etu  = 50;
      _ocra1        = 0;
      break;
  }      
      
  _etu          = _initial_etu;
  
  _activated = false;
  _high_active = true;
  
  // Set Clock to Output
  pinMode(_clk_pin, OUTPUT);
  
  // Do we have to generate CLK, by our own?
  if (_ocra1 > 0) {
    TCNT1=0;
    // Toggle OC1A on Compare Match
    TCCR1A = 0x00;
    bitSet(TCCR1A, COM1A0);
    // Clear Timer on Compare Match
    TCCR1B = 0x00;
    bitSet(TCCR1B, WGM12);
    // Set frequency (1 - 4MHz, 2 - 2.5Mhz, 3 - 2MHz / 7 - 1MHz)
    OCR1A = _ocra1;
    // No prescaling
    bitSet(TCCR1B, CS10);
  } else {
    // We need to stop a previous active clk
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    OCR1A = _ocra1;
	// Set default LOW CLK
	digitalWrite(_clk_pin, LOW);
  }

}

void CPUCardReader::setGuardTime(unsigned long t) {
  if (t != 0) {
    _guardTime = t;
  }
}

void CPUCardReader::ignoreParityErrors(boolean in) {
  _ignoreParity = in;
}

void CPUCardReader::setIncludeTSinATR(boolean include) {
  _includeTSinATR = include;
}


uint8_t CPUCardReader::calcEDC(uint8_t startValue, uint8_t *buf, uint16_t size) {
  uint8_t xorvalue = startValue;
  for(size_t i=0; i<size; i++) {
    xorvalue ^= buf[i];
  }
  return xorvalue;
}





uint16_t CPUCardReader::sendAPDU(APDU_t* command, boolean send) {
  // Internal State Machine
  //apdu_t0_state_t state = SEND_HEADER;
  
  // Create ADPU Command header
  uint8_t buf[] = { 0, 0, 0, 0, 0, 0, 0 ,0};
  uint16_t bytes_count = 0;
  uint16_t count = 0;
  uint16_t result = 0;
  uint16_t bytes_received = 0;

  uint8_t procedureByte;
  
  if(command == NULL) { 
  	delayMicroseconds(_guardTime);
   	return result;
  }
  
  // Fill buf with provided data
  buf[0] = command->cla;
  buf[1] = command->ins;
  buf[2] = command->p1;
  buf[3] = command->p2;
  buf[4] = command->data_size& 0xFF;
  bytes_count = 5;
  
  // Send Header to card
  _sendASyncBytes(buf, bytes_count);
  
  while(1){

	// Wait for Smart Card response
	bytes_received = _receiveASyncBytes(buf, 1);
	if( bytes_received <= 0 )
	{
		delayMicroseconds(_guardTime);
   		return result;
	}

	procedureByte = buf[0];

	if(procedureByte == (byte)0x60)
            continue;
	
	if (procedureByte == command->ins) {
		count = command->data_size;

		if(send){
			// Send all remaining data at once
		  		  // Just give the card time to switch to receive mode
		  delayMicroseconds(_guardTime);
		  _sendASyncBytes(command->data_buf, count);
		}else{
			bytes_received = _receiveASyncBytes(command->data_buf, count);
		}

	}
  	
  	 if((procedureByte > 0x60 && procedureByte <= 0x6F) || (procedureByte >= 0x90 && procedureByte <= 0x9F) ) {
        // Card send SW1
        result = procedureByte;
        result = result << 8;

        // Wait for Smart Card response
        if (_receiveASyncBytes(buf, 1) > 0) {
          result |= buf[0];
		  return result;
        } else {
          result = 0;
		  return result;
        }
      }
	
  }

  // As we might be a little bit to "fast", just prevent the user from
  // accessing the card to early
  delayMicroseconds(_guardTime);
  
  return result;
}



void CPUCardReader::setTimeOutCallback(SCLibVoidFuncPtr cb) {
  _timeOutCB = cb;
}

boolean CPUCardReader::cardInserted() {
	if (_off_invert)
	  return (digitalRead(_off_pin) == LOW);
	return (digitalRead(_off_pin) == HIGH);
}

boolean CPUCardReader::timeout() {
  return _timeout;
}

uint16_t CPUCardReader::getCurrentETU() {
  return _etu;
}


uint16_t CPUCardReader::activate(uint8_t* buf, uint16_t buf_size) {
  uint16_t bytes_received = 0;

  // Just for consistency reasons
  if (_activated) {
    deactivate();
  }
  
  //
  // Activation tries to find out which type of card is found
  //
  if(buf != NULL && buf_size > 0) {

    bytes_received = _activateASynchronCard(buf, buf_size);

    _activated = (bytes_received > 0);


  }

  return (_activated)?bytes_received:0;
}


void CPUCardReader::deactivate() {
  // Turn off power
  digitalWrite(_cmdvcc_pin, LOW);
  // Debug
  #if defined(USE_DEBUG_PIN)
  digitalWrite(DEFAULT_DEBUG_PIN, HIGH);
  #endif

  // Turn of CLK generation
  _init(CLK_NO_CLK);

  // Remove any previous found values
  _etu = _initial_etu;
  _activated = false;
  _high_active = true;
  _timeout = false;
}

/******************************************************************************
 * Private section
 ******************************************************************************/

//
// ASYNCHRONOUS FUNCTIONS
//

uint16_t CPUCardReader::_activateASynchronCard(uint8_t* buf, uint16_t buf_size) {
  uint16_t bytes_received = 0;
  uint8_t ts = 0;

  if(buf != NULL && buf_size > 0) {
    _init(CLK_1MHZ);

    _activateHW();

    // Read TS
    ts = _receiveTSByte();
    // try to read more
    if(ts != 0) {
      if (_includeTSinATR) {
        buf[bytes_received++] = ts;
      }
      bytes_received += _receiveASyncBytes(&buf[bytes_received], buf_size-bytes_received);
    }
  }

  return bytes_received;
}

uint16_t CPUCardReader::_receiveASyncBytes(uint8_t* buf, uint16_t buf_size) {
  uint16_t bytes_received = 0;
  _timeout = false;

  if (buf != NULL && buf_size > 0) {
    while (cardInserted() && !_timeout && bytes_received < buf_size) {
      if (_receiveByte(buf+bytes_received, _max_wwt) != 0) {
        // An error occured, while receiving ATR
        break;
      }
      bytes_received++;
    }
  }

  return bytes_received;
}

boolean CPUCardReader::_sendASyncBytes(uint8_t* buf, uint16_t count) {
  boolean parityError = false;
  if (buf != NULL && count > 0) {
    for(uint16_t i=0; i < count && cardInserted(); i++) {
      // _sendByte returns false in case parity error occured 
      if (!_sendByte(buf[i]) && !_ignoreParity) {
        delayMicroseconds(_guardTime);
        _sendByte(buf[i]);
        parityError = true;
      }
      if (i < count -1) {
        // Only wait between bytes and not at the end
        delayMicroseconds(_guardTime);
      }
      // Debug
      #if defined(USE_DEBUG_PIN)
      _toggleDebugPin();
      #endif
    }
  }
  return parityError;
}  

boolean CPUCardReader::_sendByte(uint8_t out) {
  uint8_t state = START_STATE;
  unsigned long nextBit = 0;
  boolean currentBit = false;
  boolean parity = false;
  boolean parityErrorSignaled = false;
  
  // 8 Data bits + parity
  uint8_t bits_left = 9;
  
  while (cardInserted() && state != FINISHED) {
    switch(state) {
      case START_STATE:
        // Change _io_in_pin to OUTPUT
        pinMode(_io_in_pin, OUTPUT);

        digitalWrite(_io_in_pin, LOW);

        // Debug
        #if defined(USE_DEBUG_PIN)
        _toggleDebugPin();
        #endif

        nextBit = micros() + _etu;
        state=SENDING_BITS;
        break;
      case SENDING_BITS:
        if (micros() >= nextBit) {
          currentBit = bitRead(out, 9 - bits_left);
          
          if (currentBit) {
            parity = !parity;
          }  
        
          digitalWrite(_io_in_pin, ((currentBit)?HIGH:LOW));
          
          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          nextBit += _etu;
          bits_left--;
          
          if (1 >= bits_left) {
            state = PARITY_BIT;
          }
        }
        break;
      case PARITY_BIT:
        if (micros() >= nextBit) {
          // Write Parity Bit
          digitalWrite(_io_in_pin, ((parity)?HIGH:LOW));

          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif
          
          // Create Parity Bit          
          nextBit += _etu;
          state = PARITY_ERROR;
        }
        break;
      case PARITY_ERROR:
        if (micros() >= nextBit) {
          // Wait for parity signal from card
          digitalWrite(_io_in_pin, HIGH);
          pinMode(_io_in_pin, INPUT);
                 
          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          // Read 
          nextBit+= _etu/2;          
          state=PARITY_READ;
        }
        break;
      case PARITY_READ:
        if (micros() >= nextBit) {
          parityErrorSignaled = (digitalRead(_io_in_pin) != HIGH);

          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          // Stop Sending Bity
          state=FINISHED;
        }
        break;
      default:
        state = FINISHED;
        break;
    }
  }
  return !parityErrorSignaled;
}   


int CPUCardReader::_receiveByte(uint8_t* buf, unsigned long timeout) {
  boolean startbit_found = false;
  
  // Calculate maximum wait time 
  unsigned long     endTime = micros() + timeout;
  int                result = 0;
  
  unsigned long nextBitTime = 0;
  
  // Wait for start bit
  while (cardInserted() && !startbit_found) {
    // First check if timeout occured
    if (micros() >= endTime) {
      // Timeout
      _timeOutoccured();
      result = -1;
      break;
    }
    // We are waiting for the falling edge of the start bit
    if (digitalRead(_io_in_pin) != HIGH) {
      // found it
      startbit_found = true;

      // Debug
      #if defined(USE_DEBUG_PIN)
      _toggleDebugPin();
      #endif

      // Set time for first bit
      nextBitTime = micros() + _etu + _etu / 2;
    }
  }
  
  if (startbit_found)
    _receiveDataBits(buf, nextBitTime, 8);
    
  return result;
}

void CPUCardReader::_receiveDataBits(uint8_t* buf, unsigned long startTime, uint8_t count) {
  // We also need to read the parity bit ;-.)
  uint8_t bits_left = count + 1;
  unsigned long nextBitTime = startTime;
  boolean parity = true;
  uint8_t state = READING_BITS;
  
  // Init buffer
  *buf = 0;

  while (cardInserted() && state != FINISHED && bits_left > 0) {
    switch(state) {
      case READING_BITS:
        if(micros() >= nextBitTime) {
          *buf = *buf >> 1;
          if (digitalRead(_io_in_pin) != LOW) {
            *buf |= 0x80;
            parity = !parity;
          } else {
            // Be sure to delete highest bit
            *buf &= 0x7F;
          }
          bits_left--;
        
          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          // Next is the Parity Bit
          if (bits_left <= 1) {
            state = PARITY_BIT;
          }          
          // Set arrival time for next data bit
          nextBitTime += _etu;
        }
        break;
      case PARITY_BIT:
        // Wait until next bit arrives
        if(micros() >= nextBitTime) {
          // Signal Parity Error, if requested
          if (!_ignoreParity && \
                ((parity && digitalRead(_io_in_pin) != HIGH) || \
                 (!parity && digitalRead(_io_in_pin) != LOW)) ) {
            state = PARITY_ERROR;
            nextBitTime += _etu / 2;
          } else {
            nextBitTime += _etu + _etu / 2;
            state = PRE_FINISHED;
          }

          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif
        }          
        break;
      case PARITY_ERROR:
        // Wait until parity error window arrives
        if(micros() >= nextBitTime) {
          if (!_ignoreParity) {
            // Signal Parity Error
            pinMode(_io_in_pin, OUTPUT);
            digitalWrite(_io_in_pin, LOW);
            delayMicroseconds(_etu);
            digitalWrite(_io_in_pin, LOW);
            pinMode(_io_in_pin, INPUT);
          } else {          
            // For now we just use the debug pin
            // Debug
            #if defined(USE_DEBUG_PIN)
            _toggleDebugPin();
            delayMicroseconds(_etu);
            _toggleDebugPin();
            #endif
          }
          nextBitTime += _etu / 2;
          state = PRE_FINISHED;
        }
        break;
      case PRE_FINISHED:
        // Just wait time to fly by
        if (micros() >= nextBitTime) {
          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          state = FINISHED;
        }
        break;
      default:
        // Just be sure to bail out of while loop
        state = FINISHED;
        break;
    }
  }
}

uint8_t CPUCardReader::_receiveTSByte() {
  unsigned long start = 0;
  // Wait max 40.000 * etu
  unsigned long endTime = micros() + MAX_WAIT_TIME;
  // Init receiving state machine
  int state = START_STATE;
  
  // As we know TS, we preset bits_left and result, with predefined values
  int      bits_left = 6;
  uint8_t  result    = 0;
    
  // timeout did not occur and CARD still inserted
  while (cardInserted() && state != FINISHED && bits_left) {
    // Check for timeout
    if(micros() >= endTime) {
      result = 0;
      _timeOutoccured();
      break;
    }  
     switch (state) {
       case START_STATE:
         if (digitalRead(_io_in_pin) != HIGH) {
           start = micros();
           state = FOUND_FIRST_FALLING_EDGE;

           // Debug
           #if defined(USE_DEBUG_PIN)
           _toggleDebugPin();
           #endif
         }
         break;
       case FOUND_FIRST_FALLING_EDGE:
         if (digitalRead(_io_in_pin) != LOW) {
           state = FOUND_FIRST_RAISING_EDGE;

           // Debug
           #if defined(USE_DEBUG_PIN)
           _toggleDebugPin();
           #endif
         }
         break;
       case FOUND_FIRST_RAISING_EDGE:
         if (digitalRead(_io_in_pin) != HIGH) {
           // Calculate _etu
           _etu = ((micros() - start) / 3);
           // Calculate initial _guardtime (Wi * etu) (Wi = 2)
           _guardTime = 2 * _etu;
           // 960 * D * Wi ( D = 1, Wi = 2)
           _wwt       = 1920 * _etu; 
           // WWT + D * 480 etus ( D = 1 )
           _max_wwt   = _wwt + 480 * _etu; 
           
           state = SYNC_FOUND;

           // Debug
           #if defined(USE_DEBUG_PIN)
           _toggleDebugPin();
           #endif
         }
         break;
       case SYNC_FOUND:
         // Wait half etu, to read values in the middle of the etu
         _receiveDataBits(&result, micros() + _etu/2, bits_left);

         // Add predefined ATR BITS (First 2 are always high) 
         // Used for ETU calculation
         result |= 0x3;
         state = FINISHED;     
       default:
         break;
     }
  }

  return result;
}




//
// CARD INDIPENDENT FUNCTIONS
//

#if defined(USE_DEBUG_PIN)
void CPUCardReader::_toggleDebugPin() {
  // Debug
  //PINB = _BV(PINB7);
  // Use a more portable version
  *portInputRegister(digitalPinToPort(DEFAULT_DEBUG_PIN)) = digitalPinToBitMask(DEFAULT_DEBUG_PIN);
}
#endif

#if defined(SC_DEBUG)
void CPUCardReader::dumpHEX(uint8_t* values, uint16_t size) {
  if (values != NULL && size > 0) {
    char ascii[17];
    for(uint16_t row=0; row<(size + 15)/16; row++) {
      // Print Adress
      if (row==0)
        Serial.print("0");
      Serial.print(row * 16, HEX);
      Serial.print("|");

      // Prefill ascii
      for(int i=0; i<16; i++)
        ascii[i] = '.';
      ascii[16] = (char)0x00;
      // colums
      for(uint16_t pos=row*16; pos<(row + 1) * 16; pos++ ) {
        if(pos < size) {
          if(values[pos] < 0x10)
            Serial.print("0");
          Serial.print(values[pos], HEX);
          if(isPrintable(values[pos]))
            ascii[pos - row*16] = (char)values[pos];
        } else {
          Serial.print("  ");
        }
        Serial.print(" ");
      }
      Serial.print("'");
      Serial.print(ascii);
      Serial.println("'");
    }
  }
}
#endif

void CPUCardReader::_activateHW() {
  // Different procedures needed for synchronous and asynchronous


  // Start activate sequence (RSTIN high)
  digitalWrite( _rstin_pin, HIGH );

  // Wait some time
  delayMicroseconds(100);

  // Start activate sequence (RSTIN high)
  digitalWrite(_cmdvcc_pin, HIGH);
  // Wait at least t3
  delayMicroseconds(100);

  // Revert RST Signal
  digitalWrite( _rstin_pin, LOW );

  // Get over t5
  delayMicroseconds(150);

  // This should trigger a ATR ...
  digitalWrite( _rstin_pin, HIGH );

}

void CPUCardReader::_timeOutoccured() {
  _timeout = true;
  if (_timeOutCB != NULL) {
    _timeOutCB();
  }
}  



uint16_t CPUCardReader::sendcommand( byte cla,byte ins ,byte p1,byte p2,byte p3,byte* buffer,boolean sendflag)
{
  APDU_t   command;
   // We just use the T=0 byte transfer
    command.cla       = cla;
    command.ins       = ins;
    command.p1        = p1;
    command.p2        = p2;
    command.data_size = p3;
    command.data_buf  = buffer;  
    
   uint16_t result = sendAPDU(&command,sendflag);
   return result;
}

void CPUCardReader::dumpBytes( byte* buf , byte len)
{
   Serial.print( '|' );
   for(byte i = (byte) 0; i < len ; i++)
   {
     Serial.print(buf[i] , HEX);;
     Serial.print( ' ' );
   }
   Serial.print( '|' );
  Serial.println( );

}

