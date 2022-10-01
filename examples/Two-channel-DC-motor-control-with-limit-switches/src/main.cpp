/**
  ******************************************************************************
  * @author  JSC electronics
  * @version V1.0
  * @date    28-09-2022
  * @brief   Two-channel DC motor control with limit switches.
  *          Controlls two independent DC motors (H-bridge connection) with
  *          4 endstop switches and two control push buttons.
  */ 


#include <Arduino.h>
#include "defines.h"

void setup() {
  //init inputs
  pinMode(PIN_BUTTON_OPEN, INPUT_PULLUP);
  pinMode(PIN_BUTTON_CLOSE, INPUT_PULLUP);
  pinMode(PIN_BUTTON_STOP, INPUT_PULLUP);
  pinMode(PIN_ENDSW_S1_UP, INPUT_PULLUP);
  pinMode(PIN_ENDSW_S2_DOWN, INPUT_PULLUP);
  pinMode(PIN_ENDSW_S3_UP, INPUT_PULLUP);
  pinMode(PIN_ENDSW_S4_DOWN, INPUT_PULLUP);
  pinMode(JUMP_1, INPUT_PULLUP);
  pinMode(JUMP_2, INPUT_PULLUP);
  pinMode(JUMP_3, INPUT_PULLUP);
  pinMode(JUMP_4, INPUT_PULLUP);
  pinMode(PIN_SIGNAL_LIGHT_A, OUTPUT);
  pinMode(PIN_SIGNAL_LIGHT_B, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(PIN_SIGNAL_LIGHT_A, LOW);
  digitalWrite(PIN_SIGNAL_LIGHT_B, LOW);

  // set timer up
  setupFlickeringTimer();

  // set default gate state
  coverState[CHANNEL_A] = UNKNOWN;
  coverState[CHANNEL_B] = UNKNOWN;
  lastGateState[CHANNEL_A] = coverState[CHANNEL_A];
  lastGateState[CHANNEL_B] = coverState[CHANNEL_B];

  // set buttons
  coverButtons = new CoverButtons();
  coverButtons->init();

  // set relays instances
  relay1 = new SimpleRelay(PIN_RELAY_M1A, false);
  relay2 = new SimpleRelay(PIN_RELAY_M1B, false);
  relay3 = new SimpleRelay(PIN_RELAY_M2A, false);
  relay4 = new SimpleRelay(PIN_RELAY_M2B, false);
  
  relay1->off();
  relay2->off();
  relay3->off();
  relay4->off();
  
  // serial output init
  Serial.begin(9600);
  Serial.println("JSC electronics");
  Serial.println("DC motor control");
  Serial.println("Version: 1.0");
  Serial.println("");
} 

void loop() {
  readJumpers();
  coverButtons->update();

  readEndSwitches(CHANNEL_A); 
  stateMachine(CHANNEL_A);

  readEndSwitches(CHANNEL_B);
  stateMachine(CHANNEL_B);

  serialPrinting(CHANNEL_A);
  serialPrinting(CHANNEL_B);
}

/**
	@brief setupFlickeringTimer
         3 Hz LED control flickering timer
	@param none
	@retval none
*/
void setupFlickeringTimer() {
  noInterrupts();
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // 3.0000480007680124 Hz (16000000/((20832+1)*256))
  OCR1A = 20832;
  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 256
  TCCR1B |= (1 << CS12);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

ISR(TIMER1_COMPA_vect) {
  indicatorState[CHANNEL_A] = !indicatorState[CHANNEL_A];
  indicatorState[CHANNEL_B] = !indicatorState[CHANNEL_B];
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

/**
	@brief readJumpers
         Read state of the jumpers
	@param none
	@retval none
*/
void readJumpers(void){
  (digitalRead(JUMP_1)) ? useNCLimitSW[0] = true : useNCLimitSW[0]  = false;
  (digitalRead(JUMP_2)) ? useNCLimitSW[1] = true : useNCLimitSW[1]  = false;
  (digitalRead(JUMP_3)) ? holdButtonUp    = true : holdButtonUp     = false;
  (digitalRead(JUMP_4)) ? holdButtonDown  = true : holdButtonDown   = false;
}

/**
	@brief readEndSwitches
         Read state of the end switches
	@param MOTOR_CHANNEL channel - select motor M1 or M2
	@retval none
*/
void readEndSwitches(MOTOR_CHANNEL channel) {
  if((digitalRead(coverLimitSwitchPins[channel][DIRECTION_UP]) == useNCLimitSW[channel]) && 
      (digitalRead(coverLimitSwitchPins[channel][DIRECTION_DOWN]) == useNCLimitSW[channel])) {
    stopCover(channel);
    coverState[channel] = UNKNOWN;
    return;
  }

  if((digitalRead(coverLimitSwitchPins[channel][DIRECTION_UP]) == useNCLimitSW[channel]) && 
      coverState[channel] != CLOSING && coverState[channel] != OPEN) {
    coverState[channel] = OPEN;
  }

  if((digitalRead(coverLimitSwitchPins[channel][DIRECTION_DOWN]) == useNCLimitSW[channel]) && coverState[channel] != OPENING && 
      coverState[channel] != CLOSED) {
    coverState[channel] = CLOSED;
  } 

  if((digitalRead(coverLimitSwitchPins[channel][DIRECTION_UP]) != useNCLimitSW[channel]) && coverState[channel] == OPEN) {
    coverState[channel] = UNKNOWN;
  } 

  if((digitalRead(coverLimitSwitchPins[channel][DIRECTION_DOWN]) != useNCLimitSW[channel]) && coverState[channel] == CLOSED) {
    coverState[channel] = UNKNOWN;
  }
}

/**
	@brief stateMachine
         State machine switch
	@param none
	@retval none
*/
void stateMachine(MOTOR_CHANNEL channel) {
  switch(coverState[channel]) {
    case STOP:
      stopCover(channel);
      break;

    case OPENING:
      openCover(channel);
      ( indicatorState[channel] == false ) ? 
            digitalWrite(coverIndicatorPins[channel], LOW) : digitalWrite(coverIndicatorPins[channel], HIGH);
      break;

    case OPEN:
      stopCover(channel);
      digitalWrite(coverIndicatorPins[channel], LOW);
      break;

    case CLOSING:
      closeCover(channel);
      ( indicatorState[channel] == false ) ? 
            digitalWrite(coverIndicatorPins[channel], LOW) : digitalWrite(coverIndicatorPins[channel], HIGH);
      break;

    case CLOSED:
      stopCover(channel);
      digitalWrite(coverIndicatorPins[channel], LOW);
      break;

    case UNKNOWN:
      if(!indicatorState[channel]){
        digitalWrite(coverIndicatorPins[channel], LOW);
      }
      else{
        digitalWrite(coverIndicatorPins[channel], HIGH);
      }
      break;
  } 
}

/**
	@brief stopCover
         Immediately stops gate
	@param none
	@retval none
*/
void stopCover(MOTOR_CHANNEL channel) {
  if ( channel == CHANNEL_A ) {
    relay1->off();
    relay2->off();
    return;
  }
  relay3->off();
  relay4->off();

  // coverRelays[channel][0]->off();
  // coverRelays[channel][1]->off();
}

/**
	@brief openCover
        Opens gate
	@param none
	@retval none
*/
void openCover(MOTOR_CHANNEL channel) {
  if ( channel == CHANNEL_A ) {
    relay1->on();
    relay2->off();
    return;
  }
  relay3->on();
  relay4->off();

  // coverRelays[channel][0]->on();
  // coverRelays[channel][1]->off();

 
}

/**
	@brief closeCover
        Closes gate
	@param none
	@retval none
*/
void closeCover(MOTOR_CHANNEL channel) {
  if ( channel == CHANNEL_A ) {
    relay1->off();
    relay2->on();
    return;
  }
  relay3->off();
  relay4->on();


  // coverRelays[channel][0]->off();
  // coverRelays[channel][1]->on();
}

/**
	@brief serialPrinting
        Print actual state on serial
	@param none
	@retval none
*/
void serialPrinting(MOTOR_CHANNEL channel){
  if ( coverState[channel] != lastGateState[channel] ) {
    lastGateState[channel] = coverState[channel]; 
    Serial.print("Channel "); Serial.print(channel); Serial.print(": ");

    switch(coverState[channel]){
    case STOP:
      Serial.println("STOP");
      break;
    case OPENING:
      Serial.println("OPENING");
      break;
    case OPEN:
      Serial.println("OPEN");
      break;
    case CLOSING:
      Serial.println("CLOSING");
      break;
    case CLOSED:
      Serial.println("CLOSED");
      break;
    case UNKNOWN:
      Serial.println("UNKNOWN");
      break;
    }
  }
}