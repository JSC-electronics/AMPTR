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
#include <SimpleRelay.h>
#include <ObjectButton.h>
#include <interfaces/IOnPressListener.h>

#define NUMBER_OF_CHANNELS      2

#define PIN_RELAY_M1A           3
#define PIN_RELAY_M1B           5
#define PIN_RELAY_M2A           10
#define PIN_RELAY_M2B           11

#define PIN_BUTTON_STOP         A5
#define PIN_BUTTON_OPEN         A6
#define PIN_BUTTON_CLOSE        A7

#define PIN_SIGNAL_LIGHT_A      6
#define PIN_SIGNAL_LIGHT_B      9

#define JUMP_1                  12 // limit switch logic A, log. 0 = NO (jumper attached), log. 1 = NC (jumper detached)
#define JUMP_2                  4  // limit switch logic B, log. 0 = NO (jumper attached), log. 1 = NC (jumper detached)
#define JUMP_3                  7  // push button logic UP, log. 0 = push once to move (jumper attached), log. 1 = hold to move (jumper detached)
#define JUMP_4                  8  // push button logic DOWN, log. 0 = push once to move (jumper attached), log. 1 = hold to move (jumper detached)

#define PIN_ENDSW_S1_UP         A3   
#define PIN_ENDSW_S2_DOWN       A2  
#define PIN_ENDSW_S3_UP         A1   
#define PIN_ENDSW_S4_DOWN       A0

#define DEBOUNCE_TICKS          5   //ms
#define CLICK_TICKS             150 //ms

// state machine states
typedef enum {
  STOP,
  OPENING,
  CLOSING,
  OPEN,
  CLOSED,
  UNKNOWN
} CoverState;

// Motor channel
typedef enum {
  CHANNEL_A = 0, // for motor A (M1)
  CHANNEL_B = 1 // for motor B (M2)
} MOTOR_CHANNEL;

// Motor channel
typedef enum {
  DIRECTION_UP = 0, // for motor A (M1)
  DIRECTION_DOWN = 1 // for motor B (M2)
} MOTOR_DIRECTION;

//control LED flickering
bool indicatorState[NUMBER_OF_CHANNELS] = {false, false};

// jumpers logic variables
bool holdButtonUp = false;
bool holdButtonDown = false;

// global variables
SimpleRelay *relay1;
SimpleRelay *relay2;
SimpleRelay *relay3;
SimpleRelay *relay4;
CoverState coverState[NUMBER_OF_CHANNELS];
CoverState lastGateState[NUMBER_OF_CHANNELS];

static const uint8_t coverLimitSwitchPins[NUMBER_OF_CHANNELS][2] = {
                                        {PIN_ENDSW_S1_UP, PIN_ENDSW_S2_DOWN},
                                        {PIN_ENDSW_S3_UP, PIN_ENDSW_S4_DOWN}
                                      };

static const uint8_t coverIndicatorPins[NUMBER_OF_CHANNELS] = {
                    PIN_SIGNAL_LIGHT_A,
                    PIN_SIGNAL_LIGHT_B
                  };                 

SimpleRelay *coverRelays[NUMBER_OF_CHANNELS][2] = {{relay1, relay2},{relay3, relay4}};                   

// function decalrations
void setupFlickeringTimer();
void readJumpers(void);
void readEndSwitches(MOTOR_CHANNEL channel);
void stateMachine(MOTOR_CHANNEL channel);
void stopCover(MOTOR_CHANNEL channel);
void openCover(MOTOR_CHANNEL channel);
void closeCover(MOTOR_CHANNEL channel);
void serialPrinting(MOTOR_CHANNEL channel);

class CoverButtons : private virtual jsc::IOnPressListener {
public:
    CoverButtons() = default;

    void init();

    void update();

private:

    void onPress(jsc::Button &button) override;

    void onRelease(jsc::Button &button) override;

    void onLongPressStart(jsc::Button &button) override {};

    void onLongPressEnd(jsc::Button &button) override {};

    jsc::DigitalButton buttonS5Up = jsc::DigitalButton(PIN_BUTTON_OPEN, true);
    jsc::DigitalButton buttonS6Down = jsc::DigitalButton(PIN_BUTTON_CLOSE, true);
    jsc::DigitalButton buttonS7Stop = jsc::DigitalButton(PIN_BUTTON_STOP, true);
};

void CoverButtons::onPress(jsc::Button &button) {
  switch(button.getId()){
    case PIN_BUTTON_OPEN:
      coverState[CHANNEL_A] = OPENING;
      coverState[CHANNEL_B] = OPENING; 
      break;
    case PIN_BUTTON_CLOSE:
      coverState[CHANNEL_A] = CLOSING;
      coverState[CHANNEL_B] = CLOSING;
      break;
    case PIN_BUTTON_STOP:
      if(coverState[CHANNEL_A] != UNKNOWN){
        coverState[CHANNEL_A] = STOP;
      }
      if(coverState[CHANNEL_B] != UNKNOWN){
        coverState[CHANNEL_B] = STOP;
      }
      break;
  }
}

void CoverButtons::onRelease(jsc::Button &button) {
  switch(button.getId()){
    case PIN_BUTTON_OPEN:
      if(holdButtonUp){
        coverState[CHANNEL_A] = STOP;
        coverState[CHANNEL_B] = STOP;
      }
      break;
    case PIN_BUTTON_CLOSE:
      if(holdButtonDown){
        coverState[CHANNEL_A] = STOP;
        coverState[CHANNEL_B] = STOP;
      }
      break;
  }
}

void CoverButtons::init() {
  buttonS5Up.setDebounceTicks(DEBOUNCE_TICKS);
  buttonS5Up.setOnPressListener(this);
  buttonS6Down.setDebounceTicks(DEBOUNCE_TICKS);
  buttonS6Down.setOnPressListener(this);
  buttonS7Stop.setDebounceTicks(DEBOUNCE_TICKS);
  buttonS7Stop.setOnPressListener(this);
}

void CoverButtons::update() {
  buttonS5Up.tick();
  buttonS6Down.tick();
  buttonS7Stop.tick();
}

// buttons instance
CoverButtons *coverButtons;

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
  relay1 = new SimpleRelay(PIN_RELAY_M1A, true);
  relay2 = new SimpleRelay(PIN_RELAY_M1B, true);
  relay1 = new SimpleRelay(PIN_RELAY_M2A, true);
  relay2 = new SimpleRelay(PIN_RELAY_M2B, true);
  
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
}

/**
	@brief readJumpers
         Read state of the jumpers
	@param none
	@retval none
*/
void readJumpers(void){
  (!digitalRead(JUMP_3)) ? holdButtonUp = true : holdButtonUp = false;
  (!digitalRead(JUMP_4)) ? holdButtonDown = true : holdButtonDown = false;
}

/**
	@brief readEndSwitches
         Read state of the end switches
	@param MOTOR_CHANNEL channel - select motor M1 or M2
	@retval none
*/
void readEndSwitches(MOTOR_CHANNEL channel) {
  if(!digitalRead(coverLimitSwitchPins[channel][DIRECTION_UP]) && 
      !digitalRead(coverLimitSwitchPins[channel][DIRECTION_DOWN])) {
    stopCover(channel);
    coverState[channel] = UNKNOWN;
    return;
  }

  if(!digitalRead(coverLimitSwitchPins[channel][DIRECTION_UP]) && 
      coverState[channel] != CLOSING && coverState[channel] != OPEN) {
    coverState[channel] = OPEN;
  }

  if(!digitalRead(coverLimitSwitchPins[channel][DIRECTION_DOWN]) && coverState[channel] != OPENING && 
      coverState[channel] != CLOSED) {
    coverState[channel] = CLOSED;
  } 

  if(digitalRead(coverLimitSwitchPins[channel][DIRECTION_UP]) && coverState[channel] == OPEN) {
    coverState[channel] = UNKNOWN;
  } 

  if(digitalRead(coverLimitSwitchPins[channel][DIRECTION_DOWN]) && coverState[channel] == CLOSED) {
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
      if(!indicatorState[channel]){
        digitalWrite(coverIndicatorPins[channel], LOW);
      }
      else{
        digitalWrite(coverIndicatorPins[channel], HIGH);
      }
      break;

    case OPEN:
      stopCover(channel);
      digitalWrite(coverIndicatorPins[channel], LOW);
      break;

    case CLOSING:
      closeCover(channel);
      if(!indicatorState[channel]){
        digitalWrite(coverIndicatorPins[channel], LOW);
      }
      else{
        digitalWrite(coverIndicatorPins[channel], HIGH);
      }
      break;

    case CLOSED:
      stopCover(channel);
      if(!indicatorState[channel]){
        digitalWrite(coverIndicatorPins[channel], LOW);
      }
      else{
        digitalWrite(coverIndicatorPins[channel], HIGH);
      }
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
  coverRelays[channel][0]->off();
  coverRelays[channel][1]->off();
}

/**
	@brief openCover
        Opens gate
	@param none
	@retval none
*/
void openCover(MOTOR_CHANNEL channel) {
  coverRelays[channel][0]->on();
  coverRelays[channel][1]->off();
}

/**
	@brief closeCover
        Closes gate
	@param none
	@retval none
*/
void closeCover(MOTOR_CHANNEL channel) {
  coverRelays[channel][0]->off();
  coverRelays[channel][1]->on();
}

/**
	@brief serialPrinting
        Print actual state on serial
	@param none
	@retval none
*/
void serialPrinting(MOTOR_CHANNEL channel){
  if ( coverState[channel] != lastGateState[channel] ) {
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
  lastGateState[channel] = coverState[channel]; 
}