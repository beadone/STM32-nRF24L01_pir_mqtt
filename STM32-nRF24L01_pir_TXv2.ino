/*

 Sensor Receiver.
 Each sensor modue has a unique ID.
 
 TMRh20 2014 - Updates to the library allow sleeping both in TX and RX modes:
      TX Mode: The radio can be powered down (.9uA current) and the Arduino slept using the watchdog timer
      RX Mode: The radio can be left in standby mode (22uA current) and the Arduino slept using an interrupt pin
 */
 /* nrf24 pins
 * |v+  |gnd | 
 * |csn |ce  |
 * |Mosi|sck |
 * |irq |miso|
 * stm32 mapping
 * v+  -> 3.3v | grn -> grd
 * csn -> PB1  | ce  -> PB0
 * MOSI-> PA7  | sck -> PA5
 * IRQ -> PB10 | MISO-> PA6
 * 
 * 
 * 
 * 
 */

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#define BOARD_PIR_PIN PB12
#define BOARD_LED_PIN PC13

volatile int val;


// Set up nRF24L01 radio on SPI-1 bus (MOSI-PA7, MISO-PA6, SCLK-PA5) ... IRQ not used?
RF24 radio(PB0,PB1);

//const uint64_t pipes[2] = { 0xF0F0F0F0E4LL, 0xF0F0F0F0e2LL };   // Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E6LL, 0xF0F0F0F0e2LL };   // Radio pipe addresses for the 2 nodes to communicate.
// 0xF0F0F0F0e2LL is the receiver address
// 0xF0F0F0F0E4LL is the sender address. you vary the "4" to connect up to 5 trasmitters to the receiver

char  message[32] = "kitchen/sensor1";  //initialise

volatile byte led_state = 0;
bool somethingmoved = 0;
bool readytodebounce = 0;
bool messagesent = 0;


void interruptFunction() {
  led_state = !led_state;

  
      if(digitalRead(BOARD_PIR_PIN)==HIGH)  
    {
      //Serial.println("Movement detected.");
      digitalWrite(BOARD_LED_PIN, led_state);
      somethingmoved = 1; //true

    }
    else  
    {
      //Serial.println("Nothing.");
      somethingmoved = 0;  // pir resetting
     // messagesent = 0; // reset the de loopper
      digitalWrite(BOARD_LED_PIN, led_state);
    }
}





void setup(){
     Serial.begin(115200);
  //delay(1000);

  //------------- interupt code ------------------------------
  pinMode(BOARD_PIR_PIN, INPUT);
  pinMode(BOARD_LED_PIN, OUTPUT);
  // digitalWrite(BOARD_PIR_PIN,LOW);
  digitalWrite(BOARD_LED_PIN, LOW);
 attachInterrupt(BOARD_PIR_PIN, interruptFunction, CHANGE);

 //------------- radio code ------------------------------
  Serial.println("\n\rRF24 Sensor pir transmitter");
  
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);

  // Setup and configure rf radio

  radio.begin();


  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);
  radio.setChannel(0x4c);  //channel 76
  //radio.setPALevel(RF24_PA_LOW);   //RF_SETUP        = 0x03
  radio.setPALevel(RF24_PA_MAX);   //RF_SETUP        = 0x07
  radio.enableAckPayload();
  radio.enableDynamicPayloads();  // this has to match your receiver
  

  radio.openWritingPipe(pipes[0]); // transmitt
  radio.openReadingPipe(1,pipes[1]);

  // Start listening
  radio.startListening();

  // Dump the configuration of the rf unit for debugging
  radio.printDetails();

}


void loop(){

  int loop_counter = 0;
  
 byte gotByte; 
                                         // Initialize a variable for the incoming response
    //>>>>>>>>>>>>>>>>start of pir code >>>>>>>>>>>>>>>>>>




  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    radio.stopListening();   
  //radio.printDetails();

 // if ((somethingmoved) && (!readytodebounce)) {
     if (somethingmoved) {
          //Serial.println("something moved ");
         // Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ");
         // digitalWrite(BOARD_LED_PIN, LOW);
      if ( radio.write(&message,sizeof(message)) ){             // Send the message to the other radio

        
             somethingmoved = 0;                          // send the message only once         
        if(!radio.available()){                             // If nothing in the buffer, we got an ack but it is blank
          //  Serial.print(F("Got blank response. round-trip delay: "));
    
         //   Serial.println(F(" microseconds")); 
            //delay(30000);
           //radio.printDetails();    
        }else{      
            while(radio.available() ){                      // If an ack with payload was received
                radio.read( &gotByte, 1 );                  // Read it, and display the response time
            //    unsigned long timer = micros();
                
           //     Serial.print(F("Got response "));
          //      Serial.print(gotByte);
           //     Serial.print(F(" round-trip delay: "));
               
            //    Serial.println(F(" microseconds"));
               
                //delay(30000);
            }  //while
        } //!radio.available - else
      } // radio.write

    
  }else { // pir state - something moved
   // digitalWrite(BOARD_LED_PIN, HIGH); 
    
  }// pir state
    
}// main loop






