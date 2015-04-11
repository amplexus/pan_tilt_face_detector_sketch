/*
 * A program to remotely control a pan-tilt servo platform. 
 *
 * Requires:
 * - Sparkfun XBee shield (series 1)
 * - Sparkfun XBee USB explorer
 * - arduino uno
 * - arduino uno breadboard shield
 * - 2 x servos
 * - dfrobot DF05BB pan-tilt kit 
 *
 */

/* 
 * If you enable the DLINE switch on your sparkfun arduino servo shield, it will use lines D2/D3 for comms, whcih frees up the UART (lines D0/D1) for serial comms - great for debugging! 
 * But in doing so on a UNO, you will need to install the softserial version of the arduino XBee library over the top of the version that uses hardware UART.
 */

//#define DLINE_ENABLED 1 // Requires softserial version of XBee library to be installed in sketchbook/libraries


#ifdef DLINE_ENABLED
#include <SoftwareSerial.h>
#endif

#include <Servo.h>
#include <XBee.h>

// Commands I understand

#define CMD_PAN_LEFT		        0
#define CMD_PAN_RIGHT		        1
#define CMD_TILT_UP			2
#define CMD_TILT_DOWN		        3
#define CMD_PAN_TO		        4
#define CMD_TILT_TO			5
#define CMD_PING			6
#define CMD_SELFTEST			7
#define CMD_RESET			8

// Pin allocation

#define PAN_SERVO_PIN                   9  // The servo that moves horizontally
#define TILT_SERVO_PIN                  10 // The servo that moves vertically
#define LED_PIN                         13

#ifdef DLINE_ENABLED
#define XBEE_IN_PIN                     2
#define XBEE_OUT_PIN                    3
#endif

// Servo position limits

#define PAN_SERVO_MIN                  20 // Can't be less than 0
#define PAN_SERVO_MAX                  160 // Can't be more than 180
#define TILT_SERVO_MIN                 60 // Can't be less than 0
#define TILT_SERVO_MAX                 120 // Can't be more than 180

// Per Hitec HS-422 specs (http://www.servodatabase.com/servo/hitec/hs-422)
#define PULSE_CYCLE_MILLIS           20
#define PULSE_MIN_MICROS             900
#define PULSE_MAX_MICROS             2100

Servo panServo; 
Servo tiltServo;

int panServoPos = 0;      // Current pan servo position
int tiltServoPos = 0;     // Current tilt servo position 

XBee xbee = XBee() ;

#ifdef DLINE_ENABLED
SoftwareSerial softSerial(XBEE_IN_PIN, XBEE_OUT_PIN); // Sparkfun xbee shield is switched to DLINE mode - i.e. pins D2 and D3
#endif

void setup(void) { 
  pinMode(LED_PIN, OUTPUT) ;

  panServo.attach(PAN_SERVO_PIN, PULSE_MIN_MICROS, PULSE_MAX_MICROS);
  tiltServo.attach(TILT_SERVO_PIN, PULSE_MIN_MICROS, PULSE_MAX_MICROS);

#ifdef DLINE_ENABLED
  Serial.begin(9600); // Flick the switch on the xbee shield (http://littlebirdelectronics.com/products/xbee-shield) from uart (d0/d1) to 
                      // dline (d2/d3) so i can upload sketches without unplugging xbee
  Serial.println("Begins") ;

  softSerial.begin(9600);
  xbee.setSerial(softSerial); 
#else
  xbee.begin(9600);
#endif

  reset();
//  delay(PULSE_CYCLE_MILLIS);  
//  selfTest();
//  selfTest();
} 

void loop(void) {

  /*
   * Read command from remote XBee
   */
  XBeeResponse response = XBeeResponse();
  Rx16Response rx16 = Rx16Response(); // Response data will be stuffed in here
  xbee.readPacket(); // Read some data

  if (xbee.getResponse().isAvailable()) {
    //XXX Serial.println("Something is available");
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      xbee.getResponse().getRx16Response(rx16); // Put the response data in rx16
      int command = rx16.getData(0);
      int num = rx16.getData(1) ; 
      switch(command) {
      case CMD_PAN_LEFT:
        panLeft(num);
        break ;
      case CMD_PAN_RIGHT:
        panRight(num);
        break ;
      case CMD_TILT_UP:
        tiltUp(num);
        break ;
      case CMD_TILT_DOWN:
        tiltDown(num);
        break ;
      case CMD_PAN_TO:
        panTo(num);
        break ;
      case CMD_TILT_TO:
        tiltTo(num);
        break ;
      case CMD_PING:
        ping();
        break ;
      case CMD_SELFTEST:
        selfTest();
        break ;
      case CMD_RESET:
        reset();
        break ;
      default:
#ifdef DLINE_ENABLED
        //XXX Serial.println("loop(): Unexpected command");
#endif
        break;
      }
    } 
    else {
#ifdef DLINE_ENABLED
      Serial.println("loop(): Unexpected api-id");
#endif
    }
  } 
  else if(xbee.getResponse().isError()) {
#ifdef DLINE_ENABLED
    Serial.println("loop(): XBee reported an error");
#endif
  } 
  else {
//     Serial.print('.');
  }
}

void tiltUp(int num) {
#ifdef DLINE_ENABLED
  Serial.print("tiltUp(): "); Serial.println(num);
#endif
  tiltServoPos += num;
  if(tiltServoPos > TILT_SERVO_MAX)
    tiltServoPos = TILT_SERVO_MAX;
  tiltServo.write(tiltServoPos);
  delay(PULSE_CYCLE_MILLIS);
}

void tiltDown(int num) {
#ifdef DLINE_ENABLED
  Serial.print("tiltUp(): "); Serial.println(num);
#endif
  tiltServoPos -= num;
  if(tiltServoPos < TILT_SERVO_MIN)
    tiltServoPos = TILT_SERVO_MIN;
  tiltServo.write(tiltServoPos);
  delay(PULSE_CYCLE_MILLIS);
}

void panLeft(int num) {
#ifdef DLINE_ENABLED
  Serial.print("panLeft(): "); Serial.println(num);
#endif
  panServoPos += num;
  if(panServoPos > PAN_SERVO_MAX)
    panServoPos = PAN_SERVO_MAX;
  panServo.write(panServoPos);
  delay(PULSE_CYCLE_MILLIS);
}

void panRight(int num) {
#ifdef DLINE_ENABLED
  Serial.print("panRight(): "); Serial.println(num);
#endif
  panServoPos -= num;
  if(panServoPos < PAN_SERVO_MIN)
    panServoPos = PAN_SERVO_MIN;
  panServo.write(panServoPos);
  delay(PULSE_CYCLE_MILLIS);
}

void panTo(int num) {
#ifdef DLINE_ENABLED
  Serial.print("panTo(): "); Serial.println(num);
#endif
  panServoPos = num;
  if(panServoPos < PAN_SERVO_MIN)
    panServoPos = PAN_SERVO_MIN;
  if(panServoPos > PAN_SERVO_MAX)
    panServoPos = PAN_SERVO_MAX;
  panServo.write(panServoPos);
  delay(PULSE_CYCLE_MILLIS);
}

void tiltTo(int num) {

#ifdef DLINE_ENABLED
  Serial.print("tiltTo(): "); Serial.println(num);
#endif
  tiltServoPos = num;
  if(tiltServoPos < TILT_SERVO_MIN)
    tiltServoPos = TILT_SERVO_MIN;
  if(tiltServoPos > TILT_SERVO_MAX)
    tiltServoPos = TILT_SERVO_MAX;
  tiltServo.write(tiltServoPos);
  delay(PULSE_CYCLE_MILLIS);
}

void ping() {

#ifdef DLINE_ENABLED
  Serial.println("ping(): begins");
#endif

//  uint8_t payload[] = { 0, 0 };
//  Tx16Request tx = Tx16Request(0x1874, payload, sizeof(payload));
//  TxStatusResponse txStatus = TxStatusResponse();
//  if (xbee.readPacket(5000)) {
//    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
//       xbee.getResponse().getZBTxStatusResponse(txStatus);
//    }      
//  }
  flashLed(LED_PIN, 2, 125);
  
#ifdef DLINE_ENABLED
  Serial.println("ping(): ends");
#endif
}

void selfTest() {

  int num;
  
#ifdef DLINE_ENABLED
  Serial.println("selfTest(): begins");
#endif
  flashLed(LED_PIN, 4, 125);
  reset();
  for(num = PAN_SERVO_MIN; num < PAN_SERVO_MAX; num++)
    panTo(num);

  for(num = PAN_SERVO_MAX; num > PAN_SERVO_MIN; num--)
    panTo(num);

  for(num = TILT_SERVO_MIN; num < TILT_SERVO_MAX; num++)
    tiltTo(num);

  for(num = TILT_SERVO_MAX; num > TILT_SERVO_MIN; num--)
    tiltTo(num);
  reset();
  flashLed(LED_PIN, 4, 125);
#ifdef DLINE_ENABLED
  Serial.println("selfTest(): ends");
#endif
}

void reset() {
#ifdef DLINE_ENABLED
  Serial.println("reset(): begins");
#endif
  panTo(90);
  tiltTo(90);
#ifdef DLINE_ENABLED
  Serial.println("reset(): ends");
#endif
}

void flashLed(int pin, int times, int wait) {

#ifdef DLINE_ENABLED
  Serial.println("flashLed");
#endif  
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(wait);
    digitalWrite(pin, LOW);

    if (i + 1 < times) {
      delay(wait);
    }
  }
}


