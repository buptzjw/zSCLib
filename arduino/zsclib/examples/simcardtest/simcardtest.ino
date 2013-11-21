#include <Arduino.h>

#include <zSC7816lib.h>

// If you are using a Arduino Mega compatible board you need to change the SC_C2_CLK to 11 as the TIMER1A
// is used for asynchronous clock generation (1MHz with just plain arduino code is no fun ;-) )
// and the SC_C1_VCC can be changed to any other "free" digital pin.
#define SC_C2_RST              7
#define SC_C1_VCC              11
#define SC_C7_IO               10
#define SC_C2_CLK              9

#define SWITCH	6

// Default behavior of the signal connected to SC_SWITCH_CARD_PRESENT is
// that the signal is HIGH, when card is present and LOW otherwise.
#define SC_SWITCH_CARD_PRESENT 8

// If the signal on PIN SC_SWITCH_CARD_PRESENT has an inverted
// characteristic (LOW card present, HIGH otherwise) this can be signaled
// via the following define. The present signal will be inverted by SL44x2 object.
#define SC_SWITCH_CARD_PRESENT_INVERT false


// Create SmartCardReader object for further use
CPUCardReader sc(SC_C7_IO, SC_C2_RST, SC_C1_VCC, SC_SWITCH_CARD_PRESENT, SC_C2_CLK, SC_SWITCH_CARD_PRESENT_INVERT);

void setup() {
  Serial.begin(9600);
  pinMode(SWITCH, OUTPUT);
  digitalWrite(SWITCH, HIGH);
}



//
// Just send basic select command to async. smart card (T=0)
//
void loop() {
  uint16_t bytes_received = 0;
  
  Serial.println("Waiting for Smartcard");

  // Wait for card to be inserted into smart card slot
  while (!sc.cardInserted())
  ;

  Serial.println("Smartcard found");

  uint8_t data[255];

  // Just try to activate card 
  //
  // Gives reset sequence and wait for ATR to be send by smart card
  bytes_received = sc.activate(data, MAX_ATR_BYTES);
  if ( bytes_received > 0) {
    Serial.println("Received ATR ...");

#if defined(SC_DEBUG)
    sc.dumpHEX(data, bytes_received);
#else
    Serial.print(bytes_received);
	Serial.println(" bytes ATR received from card.");
#endif
    
    data[0] = (byte)0x3F;
    data[1] = (byte)0x00;
    uint16_t result = sc.sendcommand( (byte)0xA0, (byte)0xA4 , (byte)0x00 , (byte) 0x00, (byte)0x02,data,true);
    Serial.print("Received ... ");
    Serial.println(result, HEX);
    
    data[0] = (byte)0x7F;
    data[1] = (byte)0x10;
     result = sc.sendcommand( (byte)0xA0, (byte)0xA4 , (byte)0x00 , (byte) 0x00, (byte)0x02,data,true);
    Serial.print("Received ... ");
    Serial.println(result, HEX);

    data[0] = (byte)0x6F;
    data[1] = (byte)0x3A;
     result = sc.sendcommand( (byte)0xA0, (byte)0xA4 , (byte)0x00 , (byte) 0x00, (byte)0x02,data,true);
    Serial.print("Received ... ");
    Serial.println(result, HEX);
    
    for(byte i = (byte)0; i < (byte)0x1C ; i++)
   {
     data[i] = i;
   }
    //command(0xA0,0xDC,blocknum,0x04,0x1C,0x21); //更新记录数据
     result = sc.sendcommand( (byte)0xA0, (byte)0xDC , (byte)0x01 , (byte) 0x04, (byte)0x1C,data,true);
    Serial.print("Received ... ");
    Serial.println(result, HEX);
 
    for(byte i = (byte)0; i < (byte)0x1C ; i++)
   {
     data[i] = (byte)0x00;
   }


  //A0,0xB2,blocknum,0x04,0x1C,0x10
  result = sc.sendcommand( (byte)0xA0, (byte)0xB2 , (byte)0x01 , (byte) 0x04, (byte)0x1C,data,false);
    Serial.print("Received ... ");
    Serial.println(result, HEX);
    
    sc.dumpHEX( data, (byte)0x1C);

        
  } else {
    Serial.println("Unable to identify card ... Please remove");
  }
  delay(2000);

  // Deactivate smart card slot (Turn of power etc)
  sc.deactivate();

  // Wait for card to be removed physicaly from slot
  while (sc.cardInserted())
  ;
}


