#include "packets.h"
#include "serial_driver.h"
#include <AFMotor.h>

#define PIN19 0b00000100 //Estop button 

#define S0_BIT (1 << PA0) // D22
#define S1_BIT (1 << PA1) // D23
#define S2_BIT (1 << PA2) // D24
#define S3_BIT (1 << PA3) // D25
#define OUT_BIT (1 << PE5)

#define FRONT_LEFT   4 // M4 on the driver shield
#define FRONT_RIGHT  1 // M1 on the driver shield
#define BACK_LEFT    3 // M3 on the driver shield
#define BACK_RIGHT   2 // M2 on the driver shield

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
uint32_t moveStartTime = 0;
uint32_t moveDuration = 0;
char moving = 0;
int speed = 200;
int increment = 5;

// estopStage == 0 means that button is unpressed rn
// estopStage == 1 means that button is pressed rn
volatile char estopStage = 0;
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
ISR(INT5_vect) {
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

  // Sensor OUT pin PE5 as input, no internal pull-up
  DDRE  &= ~OUT_BIT;
  PORTE &= ~OUT_BIT;

  // Configure INT5 for rising edge detection
  // EICRB: ISC51=1, ISC50=1 -> rising edge on INT5
  EICRB |= (1 << ISC51) | (1 << ISC50);
  // Do NOT enable INT5 yet — only enabled during measurement
}

/*
 * Measure one color channel by counting rising edges over 100 ms.
 *
 * Datasheet Table 1 — Photodiode type selection:
 *   S2=L, S3=L  ->  Red
 *   S2=H, S3=H  ->  Green
 *   S2=L, S3=H  ->  Blue
 *   S2=H, S3=L  ->  Clear (not used)
 *
 * Datasheet p.7: After any transition of S2/S3, the output-scaling
 * counter registers are cleared and a new valid period begins on the
 * next pulse of the principal frequency.  Response time is one period
 * of the new frequency plus 1 µs.
 *
 * At 20% scaling with max full-scale of 120 kHz, the longest period
 * is ~8.3 µs, so a few milliseconds of settle time is more than
 * sufficient.  We use a generous 10 ms to guarantee clean data.
 *
 * Returns the raw edge count (caller multiplies by 10 for Hz).
 */
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
  EIMSK |= (1 << INT5);    // Enable INT5
  sei();

  // Measurement window: 100 ms (1000 ticks × 100 µs)
  {
    uint32_t start = timerTicks;
    while ((timerTicks - start) < 1000);
  }

  // Disable INT5, capture count
  cli();
  EIMSK &= ~(1 << INT5);
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

void move(int direction, int duration=0)
{
  if (direction == STOP) {
    motorFL.run(RELEASE);
    motorFR.run(RELEASE);
    motorBL.run(RELEASE);
    motorBR.run(RELEASE); 
    moving = 0;
  } else {
    moving = 1;
    moveStartTime = timerTicks;
    moveDuration = duration;
    motorFL.setSpeed(speed);
    motorFR.setSpeed(speed);
    motorBL.setSpeed(speed);
    motorBR.setSpeed(speed);

    switch(direction)
    {
      case BACK:
        motorFL.run(FORWARD);
        motorFR.run(FORWARD);
        motorBL.run(BACKWARD);
        motorBR.run(FORWARD); 
      break;
      case GO:
        motorFL.run(BACKWARD);
        motorFR.run(BACKWARD);
        motorBL.run(FORWARD);
        motorBR.run(BACKWARD); 
      break;
      case CCW:
        motorFL.run(FORWARD);
        motorFR.run(BACKWARD);
        motorBL.run(BACKWARD);
        motorBR.run(BACKWARD); 
      break;
      case CW:
        motorFL.run(BACKWARD);
        motorFR.run(FORWARD);
        motorBL.run(FORWARD);
        motorBR.run(FORWARD); 
      break;
      case STOP:
      default:
        motorFL.run(RELEASE);
        motorFR.run(RELEASE);
        motorBL.run(RELEASE);
        motorBR.run(RELEASE); 
    }
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

  colorSensorInit();
  sei();
}

void loop() {
  // --- 1. Report any E-Stop state change to the Pi ---
  if (stateChanged) {
    cli();
    TState state = buttonState;
    stateChanged = false;
    sei();
    sendStatus(state);
  }

  if (moving && timerTicks - moveStartTime >= moveDuration * 10) {
    {
      TPacket pkt;
      memset(&pkt, 0, sizeof(pkt));
      pkt.packetType = PACKET_TYPE_RESPONSE;
      pkt.command = RESP_OK;
      strncpy(pkt.data, "done moving", sizeof(pkt.data) - 1);
      pkt.data[sizeof(pkt.data) - 1] = '\0';
      move(STOP);
      sendFrame(&pkt);
    }
  }
  // --- 2. Process incoming commands from the Pi ---
  TPacket incoming;
  if (receiveFrame(&incoming)) {
    handleCommand(&incoming);
  }
}

