#include <SimpleRelay.h>
#include <ObjectButton.h>
#include <interfaces/IOnPressListener.h>

#ifndef _DEFINES_H
#define _DEFINES_H

#define NUMBER_OF_CHANNELS      2

#define PIN_RELAY_M1A           3
#define PIN_RELAY_M1B           5
#define PIN_RELAY_M2A           10
#define PIN_RELAY_M2B           11

#define PIN_BUTTON_STOP         0
#define PIN_BUTTON_OPEN         A4
#define PIN_BUTTON_CLOSE        A5

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
bool useNCLimitSW[NUMBER_OF_CHANNELS] = {false, false}; // if true, normally closed switches will be used. 

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

// SimpleRelay *coverRelays[NUMBER_OF_CHANNELS][2] = {{relay1, relay2},{relay3, relay4}};     

// function declarations
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

    jsc::DigitalButton buttonS5Up = jsc::DigitalButton(PIN_BUTTON_OPEN, true); // switch to 5 V DC
    jsc::DigitalButton buttonS6Down = jsc::DigitalButton(PIN_BUTTON_CLOSE, true); // switch to 5 V DC
    jsc::DigitalButton buttonS7Stop = jsc::DigitalButton(PIN_BUTTON_STOP, true); // switch to 5 V DC
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

CoverButtons *coverButtons;

#endif /* _DEFINES_H */