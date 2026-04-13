#include "packets.h"
#include "serial_driver.h"
#include <AFMotor.h>

#define PIN19 0b00000100 //Estop button 

#define S0_BIT (1 << PA0) // D22
#define S1_BIT (1 << PA1) // D23
#define S2_BIT (1 << PA2) // D24
#define S3_BIT (1 << PA3) // D25
#define OUT_BIT (1 << PD3) // A8

#define FRONT_LEFT   2 // M4 on the driver shield
#define FRONT_RIGHT  3 // M1 on the driver shield
#define BACK_LEFT    1 // M3 on the driver shield
#define BACK_RIGHT   4 // M2 on the driver shield

// Both trigger PCINT1_vect
#define ENCODER_LEFT (1 << PJ0) // digital 15 PCINT9
#define ENCODER_RIGHT (1 << PJ1) //digital 14 PCINT10

#define ARM_BASE (1 << PK1) // A9
#define ARM_SHOULDER (1 << PK2) // A10
#define ARM_ELBOW (1 << PK3) // A11
#define ARM_GRIPPER (1 << PK4) // A12

// Direction values
typedef enum dir
{
  STOP,
  GO,
  BACK,
  CCW,
  CW
} dir;

AF_DCMotor motorFL(FRONT_LEFT);
AF_DCMotor motorFR(FRONT_RIGHT);
AF_DCMotor motorBL(BACK_LEFT);
AF_DCMotor motorBR(BACK_RIGHT);

volatile uint32_t timerTicks = 0;
volatile uint32_t lastTime = 0;
volatile uint32_t currentTime = 0;
volatile long moveStartTicks = 0;
volatile long moveDistance = 0;
volatile char moving = 0;
int speed = 255;
int increment = 5;
volatile int moveStateL = STOP;
volatile int moveStateR = STOP;

// estopStage == 0 means that button is unpressed rn
// estopStage == 1 means that button is pressed rn
volatile char estopStage = 0;

// --- Robot Arm Definitions ---
const int BASE_MASK     = (1 << PK1); // Analog Pin A9
const int SHOULDER_MASK = (1 << PK2); // Analog Pin A10 
const int ELBOW_MASK    = (1 << PK3); // Analog Pin A11
const int GRIPPER_MASK  = (1 << PK4); // Analog Pin A12

// Calibration 
const int BASE_MIN = 1000, BASE_MAX = 5000, BASE_RANGE = BASE_MAX - BASE_MIN;
const int SHOULDER_MIN = 1000, SHOULDER_MAX = 2778, SHOULDER_RANGE = SHOULDER_MAX - SHOULDER_MIN;
const int ELBOW_MIN = 1000, ELBOW_MAX = 2778, ELBOW_RANGE = ELBOW_MAX - ELBOW_MIN;
const int GRIPPER_MIN = 1667, GRIPPER_MAX = 2778, GRIPPER_RANGE = GRIPPER_MAX - GRIPPER_MIN;

const int BASE_TPD     = BASE_RANGE / 180;
const int SHOULDER_TPD = SHOULDER_RANGE / 180;
const int ELBOW_TPD    = ELBOW_RANGE / 180;
const int GRIPPER_TPD  = GRIPPER_RANGE / 180;

volatile int totalTime = 0;
volatile int baseTime     = BASE_RANGE/2 + BASE_MIN;
volatile int shoulderTime = SHOULDER_RANGE/2 + SHOULDER_MIN;
volatile int elbowTime    = ELBOW_RANGE/2 + ELBOW_MIN;
volatile int gripperTime  = GRIPPER_RANGE/2 + GRIPPER_MIN;

volatile int baseTarget     = baseTime;
volatile int shoulderTarget = shoulderTime;
volatile int elbowTarget    = elbowTime;
volatile int gripperTarget  = gripperTime;
volatile int part = 0;

unsigned long justNow = 0;
unsigned long msPerDeg = 10;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

volatile uint8_t lastPortJ = 0;
int lastPIDTime = 0;
// =============================================================
// Packet helpers (pre-implemented for you)
// =============================================================

/*
 * Build a zero-initialised TPacket, set packetType = PACKET_TYPE_RESPONSE,
 * command = resp, and params[0] = param.  Then call sendFrame().
 */
static void sendResponse(TResponseType resp, uint32_t param) {
  TPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.packetType = PACKET_TYPE_RESPONSE;
  pkt.command = resp;
  pkt.params[0] = param;
  sendFrame(&pkt);
}

/*
 * Send a RESP_STATUS packet with the current state in params[0].
 */
static void sendStatus(TState state) {
  sendResponse(RESP_STATUS, (uint32_t)state);
}

// arm commands
int stepTowards(int current, int target, int stepSize) {
  if (current < target) {
    current += stepSize;
    if (current > target) current = target;
  } else if (current > target) {
    current -= stepSize;
    if (current < target) current = target; // so it dont bounce
  }
  //Serial.println(current);
  return current;
}

void smoothen() {
  uint32_t now = timerTicks; // Assumes your 100us timer wrapper
  if (now - justNow >= msPerDeg) { 
    cli();
    baseTime = stepTowards(baseTime, baseTarget, BASE_TPD);
    shoulderTime = stepTowards(shoulderTime, shoulderTarget, SHOULDER_TPD);
    elbowTime = stepTowards(elbowTime, elbowTarget, ELBOW_TPD);
    gripperTime = stepTowards(gripperTime, gripperTarget, GRIPPER_TPD);
    sei(); 
    justNow = now;
  }
}

void homeAll() {
  baseTarget = (BASE_RANGE)/2 + BASE_MIN;
  shoulderTarget = (SHOULDER_RANGE)/2 + SHOULDER_MIN;
  elbowTarget = (ELBOW_RANGE)/2 + ELBOW_MIN;
  gripperTarget = (GRIPPER_RANGE)/2 + GRIPPER_MIN;
}

void armInit() {
  // Set PK1-PK4 (A9-A12) as outputs
  DDRK |= BASE_MASK | SHOULDER_MASK | ELBOW_MASK | GRIPPER_MASK;
  PORTK &= ~(BASE_MASK | SHOULDER_MASK | ELBOW_MASK | GRIPPER_MASK);
  
  // Configure Timer5 for CTC mode, Top = OCR5A, Prescaler 8
  TCCR5A = 0;
  TCNT5 = 0;
  OCR5A = 1000;
  TIMSK5 |= (1 << OCIE5A);
  TCCR5B = (1 << WGM52) | (1 << CS51); 
}
// =============================================================
// E-Stop state machine
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile static bool stateChanged = false;
//estop

ISR(INT2_vect) {
  // YOUR CODE HERE
  currentTime = timerTicks;
  if (currentTime - lastTime > 100) {  //threshold 5ms since every cycle is 100us
    if (buttonState == STATE_RUNNING && (PIND & PIN19)) {
      if (estopStage == 0) {
        buttonState = STATE_STOPPED;
        stateChanged = true;
        estopStage = 1;
        //move(STOP);
      } else {
        estopStage = 0;
      }
    }
    else if (buttonState == STATE_STOPPED && !(PIND & PIN19)) {
      if (estopStage == 0) {
        buttonState = STATE_RUNNING;
        stateChanged = true;
      } else {
        estopStage = 0;
      }
    }
    lastTime = currentTime;
  }
}

ISR(TIMER2_COMPA_vect) {
  timerTicks++;
}


volatile uint32_t edgeCount = 0;

// INT5 ISR — counts rising edges from TCS3200 OUT pin (PE5 / Digital 3)
ISR(INT3_vect) {
  edgeCount++;
}

static void colorSensorInit() {
  // S0–S3 as outputs
  DDRA |= (S0_BIT | S1_BIT | S2_BIT | S3_BIT);

  // 20% frequency scaling: S0=HIGH, S1=LOW (datasheet Table 1)
  PORTA |= S0_BIT;
  PORTA &= ~S1_BIT;

  // Default S2/S3 to LOW (Red channel)
  PORTA &= ~(S2_BIT | S3_BIT);
  // PD3
  // Sensor OUT pin PE5 as input, no internal pull-up
  DDRD &= ~OUT_BIT;
  PORTD &= ~OUT_BIT;

  // Configure INT5 for rising edge detection
  // EICRB: ISC51=1, ISC50=1 -> rising edge on INT5
  EICRA |= (1 << ISC31) | (1 << ISC30);
  // Do NOT enable INT5 yet — only enabled during measurement
}


// Timer5 ISR using PORTK
ISR(TIMER5_COMPA_vect) { 
  switch (part) {
    case 0: // Base (A9)
      PORTK |= BASE_MASK;
      OCR5A = baseTime;
      totalTime += baseTime;
      part = 1;
      break;
    case 1: // Shoulder (A10)
      PORTK &= ~BASE_MASK;
      PORTK |= SHOULDER_MASK;
      OCR5A = shoulderTime;
      totalTime += shoulderTime;
      part = 2;
      break;
    case 2: // Elbow (A11)
      PORTK &= ~SHOULDER_MASK;
      PORTK |= ELBOW_MASK;
      OCR5A = elbowTime;
      totalTime += elbowTime;
      part = 3;
      break;
    case 3: // Gripper (A12)
      PORTK &= ~ELBOW_MASK;
      PORTK |= GRIPPER_MASK;
      OCR5A = gripperTime;
      totalTime += gripperTime;
      part = 4;
      break;
    case 4: // Wait state (Padding to 20ms)
      PORTK &= ~GRIPPER_MASK;
      OCR5A = 40000U - totalTime;
      totalTime = 0;
      part = 0;
      break;
  }
}


static uint32_t measureChannel(uint8_t s2High, uint8_t s3High) {
  // Set S2
  if (s2High)
    PORTA |= S2_BIT;
  else
    PORTA &= ~S2_BIT;

  // Set S3
  if (s3High)
    PORTA |= S3_BIT;
  else
    PORTA &= ~S3_BIT;

  // Wait for filter + frequency divider to settle (~10 ms)
  // Using the existing Timer2 tick counter (100 µs per tick)
  {
    uint32_t start = timerTicks;
    while ((timerTicks - start) < 100);  // 100 ticks × 100 µs = 10 ms
  }

  // Reset edge counter, then enable INT5
  cli();
  edgeCount = 0;
  EIMSK |= (1 << INT3); // Enable INT5
  sei();

  // Measurement window: 100 ms (1000 ticks × 100 µs)
  {
    uint32_t start = timerTicks;
    while ((timerTicks - start) < 1000);
  }

  // Disable INT5, capture count
  cli();
  EIMSK &= ~(1 << INT3);
  uint32_t count = edgeCount;
  sei();

  return count;
}

/*
 * Read all three color channels and convert edge counts to Hz.
 *
 * frequency_Hz = edge_count / measurement_window_s
 * For a 100 ms window: frequency_Hz = edge_count × 10
 *
 * Datasheet Table 1 channel selection:
 *   Red:   S2=L(0), S3=L(0)
 *   Green: S2=H(1), S3=H(1)
 *   Blue:  S2=L(0), S3=H(1)
 */
static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
  *r = measureChannel(0, 0) * 10;   // Red:   S2=L, S3=L  -> Hz
  *g = measureChannel(1, 1) * 10;   // Green: S2=H, S3=H  -> Hz
  *b = measureChannel(0, 1) * 10;   // Blue:  S2=L, S3=H  -> Hz
}

// Motor control


ISR(PCINT0_vect) {
    if ((moveStateL == FORWARD)) leftTicks++;
    else if ((moveStateL == BACKWARD)) leftTicks--;
}

ISR(PCINT1_vect) {
    if ((moveStateR == FORWARD)) rightTicks++;
    else if ((moveStateR == BACKWARD)) rightTicks--;

}

void encoderInit() {
  DDRJ &= ~(ENCODER_LEFT | ENCODER_RIGHT);
  DDRB &= ~((1 << PB3));
  PCICR |= (1 << PCIE1) | (1 << PCIE0);
  PCMSK1 |= (1 << PCINT9) | (1 << PCINT10);
  PCMSK0 |= (1 << PCINT3);
  lastPortJ = PINJ;
}

void move(int direction, long distance=0)
{
  if (direction == STOP) {
    motorFL.run(RELEASE);
    motorFR.run(RELEASE);
    motorBL.run(RELEASE);
    motorBR.run(RELEASE); 
    leftTicks = 0;
    rightTicks = 0;
    moveStartTicks = 0;
    moveDistance = 0;
    moving = 0;
   
  } else {
    motorFL.setSpeed(speed);
    motorFR.setSpeed(speed);
    motorBL.setSpeed(speed);
    motorBR.setSpeed(speed);
    leftTicks = 0;
    rightTicks = 0;
    moveStartTicks = abs(leftTicks);
    moveDistance = distance;
    moving = 1;
    switch(direction)
    {
      case BACK:
        moveStateL = BACKWARD;
        moveStateR = BACKWARD;

        motorFL.run(BACKWARD);
        motorFR.run(FORWARD);
        motorBL.run(BACKWARD);
        motorBR.run(BACKWARD); 
      break;
      case GO:
        moveStateL = FORWARD;
        moveStateR = FORWARD;

        motorFL.run(FORWARD);
        motorFR.run(BACKWARD);
        motorBL.run(FORWARD);
        motorBR.run(FORWARD); 
      break;
      case CCW:
        moveStateL = BACKWARD;
        moveStateR = FORWARD;

        motorFL.run(BACKWARD);
        motorFR.run(BACKWARD);
        motorBL.run(BACKWARD);
        motorBR.run(FORWARD); 
      break;
      case CW:
        moveStateL = FORWARD;
        moveStateR = BACKWARD;

        motorFL.run(FORWARD);
        motorFR.run(FORWARD);
        motorBL.run(FORWARD);
        motorBR.run(BACKWARD); 
      break;
    }
  }
  
}

void pidMotorSync(int leftSpeed, int rightSpeed) {
  double mError = 0;
  if (leftSpeed == 0 || rightSpeed == 0) {
    leftSpeed = 0;
    rightSpeed = 0;
  } else {
    mError = abs(rightTicks) - abs(leftTicks);
    mError = mError / (256);
    
    motorFL.setSpeed(constrain((abs(leftSpeed) + 5 * mError), 0, 255));
    motorBL.setSpeed(constrain((abs(leftSpeed) + 5 * mError), 0, 255));
    motorFR.setSpeed(constrain((abs(rightSpeed) - 5 * mError), 0, 255));
    motorBR.setSpeed(constrain((abs(rightSpeed) - 5 * mError), 0, 255));
  }
}
// =============================================================
// Command handler
// =============================================================

/*
 * Dispatch incoming commands from the Pi.
 *
 * COMMAND_ESTOP is pre-implemented: it sets the Arduino to STATE_STOPPED
 * and sends back RESP_OK followed by a RESP_STATUS update.
 *
 * TODO (Activity 2): add a case for your color sensor command.
 *   Call your color-reading function, then send a response packet with
 *   the channel frequencies in Hz.
 */
static void handleCommand(const TPacket *cmd) {
  if (cmd->packetType != PACKET_TYPE_COMMAND) return;

  switch (cmd->command) {
    case COMMAND_ESTOP:
      move(STOP);
      cli();
      if (buttonState == STATE_RUNNING) {
        estopStage = 1;
        buttonState = STATE_STOPPED;
      } else if (buttonState == STATE_STOPPED) {
        estopStage = 0;
        buttonState = STATE_RUNNING;
      } 
      stateChanged = false;
      sei();
      {
        // The data field of a TPacket can carry a short debug string (up to
        // 31 characters).  pi_sensor.py prints it automatically for any packet
        // where data is non-empty, so you can use it to send debug messages
        // from the Arduino to the Pi terminal -- similar to Serial.print().
        TPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.packetType = PACKET_TYPE_RESPONSE;
        pkt.command = RESP_OK;
        strncpy(pkt.data, "This is a debug message", sizeof(pkt.data) - 1);
        pkt.data[sizeof(pkt.data) - 1] = '\0';
        sendFrame(&pkt);
      }
      sendStatus(buttonState);
      break;

      // TODO (Activity 2): add COMMAND_COLOR case here.
      //   Call your color-reading function (which returns Hz), then send a
      //   response packet with the three channel frequencies in Hz.
    case COMMAND_COLOUR:
      {
        TPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.packetType = PACKET_TYPE_RESPONSE;
        pkt.command = RESP_COLOUR;
        strncpy(pkt.data, "This is a colour message", sizeof(pkt.data) - 1);
        pkt.data[sizeof(pkt.data) - 1] = '\0';
        readColorChannels(&(pkt.params[0]), &(pkt.params[1]), &(pkt.params[2]));
        sendFrame(&pkt);
      }
      break;

    case COMMAND_GO:
      if (moving) {
        {
        TPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.packetType = PACKET_TYPE_RESPONSE;
        pkt.command = RESP_OK;
        strncpy(pkt.data, "wait im still moving", sizeof(pkt.data) - 1);
        pkt.data[sizeof(pkt.data) - 1] = '\0';
        sendFrame(&pkt);
        }
      } else {
        {
          TPacket pkt;
          memset(&pkt, 0, sizeof(pkt));
          pkt.packetType = PACKET_TYPE_RESPONSE;
          pkt.command = RESP_OK;
          strncpy(pkt.data, "moving forward", sizeof(pkt.data) - 1);
          pkt.data[sizeof(pkt.data) - 1] = '\0';
          move(GO, cmd->params[0]);
          sendFrame(&pkt);
        }
      }
      break;
    
    case COMMAND_CW:
      if (moving) {
        {
        TPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.packetType = PACKET_TYPE_RESPONSE;
        pkt.command = RESP_OK;
        strncpy(pkt.data, "wait im still moving", sizeof(pkt.data) - 1);
        pkt.data[sizeof(pkt.data) - 1] = '\0';
        sendFrame(&pkt);
        }
      } else {
        {
          TPacket pkt;
          memset(&pkt, 0, sizeof(pkt));
          pkt.packetType = PACKET_TYPE_RESPONSE;
          pkt.command = RESP_OK;
          strncpy(pkt.data, "turning clockwise", sizeof(pkt.data) - 1);
          pkt.data[sizeof(pkt.data) - 1] = '\0';
          move(CW, cmd->params[0]);
          sendFrame(&pkt);
        }
      }
      break;

    case COMMAND_CCW:
      if (moving) {
        {
        TPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.packetType = PACKET_TYPE_RESPONSE;
        pkt.command = RESP_OK;
        strncpy(pkt.data, "wait im still moving", sizeof(pkt.data) - 1);
        pkt.data[sizeof(pkt.data) - 1] = '\0';
        sendFrame(&pkt);
        }
      } else {
        {
          TPacket pkt;
          memset(&pkt, 0, sizeof(pkt));
          pkt.packetType = PACKET_TYPE_RESPONSE;
          pkt.command = RESP_OK;
          strncpy(pkt.data, "turning counter clockwise", sizeof(pkt.data) - 1);
          pkt.data[sizeof(pkt.data) - 1] = '\0';
          move(CCW, cmd->params[0]);
          sendFrame(&pkt);
        }
      }
      break;

    case COMMAND_BACK:
      if (moving) {
        {
        TPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.packetType = PACKET_TYPE_RESPONSE;
        pkt.command = RESP_OK;
        strncpy(pkt.data, "wait im still moving", sizeof(pkt.data) - 1);
        pkt.data[sizeof(pkt.data) - 1] = '\0';
        sendFrame(&pkt);
        }
      } else {
        {
          TPacket pkt;
          memset(&pkt, 0, sizeof(pkt));
          pkt.packetType = PACKET_TYPE_RESPONSE;
          pkt.command = RESP_OK;
          strncpy(pkt.data, "moving backward", sizeof(pkt.data) - 1);
          pkt.data[sizeof(pkt.data) - 1] = '\0';
          move(BACK, cmd->params[0]);
          sendFrame(&pkt);
        }
      }
      break;

    case COMMAND_STOP:
        {
          TPacket pkt;
          memset(&pkt, 0, sizeof(pkt));
          pkt.packetType = PACKET_TYPE_RESPONSE;
          pkt.command = RESP_OK;
          strncpy(pkt.data, "move stopping", sizeof(pkt.data) - 1);
          pkt.data[sizeof(pkt.data) - 1] = '\0';
          move(STOP);
          sendFrame(&pkt);
        }
        break;

    case COMMAND_FASTER:
        speed += increment;
        speed = constrain(speed, 0, 255);
        {
          TPacket pkt;
          memset(&pkt, 0, sizeof(pkt));
          pkt.packetType = PACKET_TYPE_RESPONSE;
          pkt.command = RESP_OK;
          char speedText[8];
          itoa(speed, speedText, 10);
          char message[32] = "moving faster, speed: ";
          strcat(message, speedText);
          strncpy(pkt.data, message, sizeof(pkt.data) - 1);
          pkt.data[sizeof(pkt.data) - 1] = '\0';
          sendFrame(&pkt);
        }
        break;

    case COMMAND_SLOWER:
      speed -= increment;
      speed = constrain(speed, 0, 255);
      {
        TPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.packetType = PACKET_TYPE_RESPONSE;
        pkt.command = RESP_OK;
        char speedText[8];
        itoa(speed, speedText, 10);
        char message[32] = "moving slower, speed: ";
        strcat(message, speedText);
        strncpy(pkt.data, message, sizeof(pkt.data) - 1);
        pkt.data[sizeof(pkt.data) - 1] = '\0';
        sendFrame(&pkt);
      }
      break;

    case COMMAND_ARM_MOVE:
      {
        // Extract the variables from the packet
        uint32_t servoID = cmd->params[0];
        long angle = constrain(cmd->params[1], 0, 180);

        // Route the angle to the correct target variable
        switch (servoID) {
          case SERVO_BASE:
            baseTarget = (angle * BASE_RANGE) / 180 + BASE_MIN;
            break;
            
          case SERVO_SHOULDER:
            shoulderTarget = (angle * SHOULDER_RANGE) / 180 + SHOULDER_MIN;
            break;
            
          case SERVO_ELBOW:
            elbowTarget = (angle * ELBOW_RANGE) / 180 + ELBOW_MIN;
            break;
            
          case SERVO_GRIPPER:
            gripperTarget = (angle * GRIPPER_RANGE) / 180 + GRIPPER_MIN;
            break;

          case SERVO_SPEED:
            msPerDeg = angle;
            break;
        }

        sendResponse(RESP_OK, 0);
      }
      break;
    }
  }


// =============================================================
// Arduino setup() and loop()
// =============================================================

void setup() {
  // Initialise the serial link at 9600 baud.
  // Serial.begin() is used by default; usartInit() takes over once
  // USE_BAREMETAL_SERIAL is set to 1 in serial_driver.h.
#if USE_BAREMETAL_SERIAL
  usartInit(103);  // 9600 baud at 16 MHz
#else
  Serial.begin(9600);
#endif
  // TODO (Activity 1): configure the button pin and its external interrupt,
  // then call sei() to enable global interrupts.
  cli();
  EICRA |= 0b00010000;   //INT2, change in state
  DDRD &= ~(1 << PD2);    //pin 19 as input pin
  TCCR2A = 0b00000010;  //CTC mode
  TIMSK2 |= 0b00000010;  //OCIE2A
  TCNT2 = 0;
  OCR2A = 199;
  EIMSK |= 0b00000100;
  TCCR2B = 0b00000010;  //prescaler 8
  armInit();
  colorSensorInit();
  encoderInit();
  sei();
}

void loop() {
  smoothen();
  // --- 1. Report any E-Stop state change to the Pi ---
  if (stateChanged) {
    cli();
    TState state = buttonState;
    stateChanged = false;
    sei();
    sendStatus(state);
    move(STOP);
  }

  if (moving && (labs(leftTicks) >= moveDistance)) {
    {
      move(STOP);
      TPacket pkt;
      memset(&pkt, 0, sizeof(pkt));
      pkt.packetType = PACKET_TYPE_RESPONSE;
      pkt.command = RESP_OK;
      strncpy(pkt.data, "done moving", sizeof(pkt.data) - 1);
      pkt.data[sizeof(pkt.data) - 1] = '\0';
      sendFrame(&pkt);
    }
  } else if (moving) {
    pidMotorSync(speed, speed);
  }

   /*{
      TPacket pkt;
      memset(&pkt, 0, sizeof(pkt));
      pkt.packetType = PACKET_TYPE_RESPONSE;
      pkt.command = RESP_OK;
      char buf[32];
      snprintf(buf, sizeof(buf), "L:%ld R:%ld D:%ld Last:%d R:%d L:%d", leftTicks, rightTicks, moveDistance, lastPortJ, moveStateR == FORWARD, moveStateL == FORWARD);
      strncpy(pkt.data, buf, sizeof(pkt.data) - 1);
      pkt.data[sizeof(pkt.data) - 1] = '\0';
      sendFrame(&pkt);
    }*/

  // --- 2. Process incoming commands from the Pi ---
  TPacket incoming;
  if (receiveFrame(&incoming)) {
    handleCommand(&incoming);
  }
}

