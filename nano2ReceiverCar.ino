//######################Program Name: nano2receiverProto5.ino####################
// Mapped X-Axis (0-1023) → 0-180 for servo steering.
// Mapped Y-Axis (0-1023) → -255 to 255 for speed/direction.
// OLED now displays "STR" for steering & "SPD" for speed instead of raw values.
//Added wiring for L398N module to nano. Motor control code  added here
//Added a new 'state' var to struct DataPacket to record switch values
//set in xmitter module.
//###################################################################################
//###########################nRF24L01Wiring Diagram:#################################
// nRF24L01	                  Arduino Nano
// VCC (3.3V)	                3.3V (or external 3.3V regulator)
// GND	                               GND
// CE	               Any digital pin (e.g., D9)
// CSN	                      Any digital pin (e.g., D10)
// SCK	                      D13 (SCK)
// MOSI	                      D11 (MOSI)
// MISO	                      D12 (MISO)
// IRQ	                      Not needed (leave unconnected)
//##################################################################################
//##########################L298N to Arduino Nano Wiring#############################################
// L298N
// Pin	          Function	                                  Arduino Nano Pin
// 12V	      Power for motors (Use external 12V battery)	    12V Battery
// GND	      Ground	                                          GND all grounds must be common
// 5V OUT	    Regulated 5V output	                            Not needed (Nano already powered)
// IN1	      Motor A Direction                                 D4	
// IN2	      Motor A Direction                                 D8
// IN3	      Motor B Direction                                 D6	
// IN4	      Motor B Direction                                 D7	
// ENA	      Motor A Speed                                     D3 (PWM)	
// ENB	      Motor B Speed                                     D5	
//####################Battery Module###################################################################
//Need Three 3.7V Li-ion rechargeable batteries, providing total of 11V, connected in series.
//Need 3-Battery holder and charger.
//####################################################################################################
//Disclaimer:This code is open and free to use, modify, and even share.
// It is based on common programming principles and standard usage
// of the Arduino, nRF24L01 and L298N libraries, which are widely used in open-source projects.
//########################################################################################################
//##################################Updates:##############################################################
//Added a new 'state' var to struct DataPacket to record switch values
//set in xmitter module.
//4/15/2025 Removed OLED Hardware and SW from Receiver code.
// Removed Adafruit_GFX and Adafruit_SSD1306 libraries associated with OLED display operation
//4/17/2025 Added address lock using dip encode switches for rcvvr/xmitter match
//################################################################################
//5/13/2025 Added address lock using dip encode switches for rcvvr/xmitter match
//assigned A4 and A5
//################################################################################

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>

#define CE_PIN 9
#define CSN_PIN 10
#define DEAD_ZONE 15    // Dead zone to avoid fluctuations
#define JOY_CENTER 508  // Adjust for joystick calibration

// L298N Motor Driver all D Pins
#define IN1 4
#define IN2 8
#define IN3 6
#define IN4 7
#define ENA 3// Left Motor PWM
#define ENB 5// Right Motor PWM

//Speed Multiplier
float speedCntl=.5;
//Declare pins to be used by functions printChannelXState
int ledPin1 = A0; // Assign Analog pin A3 as a digital pin for the first LED
int ledPin2 = A1; // Assign Analog pin A4 as a digital pin for the second LED
int ledPin3 = A2; // Assign Analog pin A5 as a digital pin for the third LED

//##########################ADDRESS SELECT###################
const byte address1[] = { '0', '0', '0', '0', '1' };  // Car 1
const byte address2[] = { '0', '0', '0', '0', '2' };  // Car 2
const byte address3[] = { '0', '0', '0', '0', '3' };  // Car 3
const byte address4[] = { '0', '0', '0', '0', '4' };  // Car 4

byte addSel[] = { '0', '0', '0', '0', '0' };
byte address[] = { '0', '0', '0', '0', '0' };

const int switch1Pin = A4; // Pin for switch 1
const int switch2Pin = A5; // Pin for switch 2

RF24 radio(CE_PIN, CSN_PIN);
struct DataPacket {
    uint16_t joyX; // Joystick X-axis, using 2 bytes
    uint16_t joyY; // Joystick Y-axis, using 2 bytes
    uint8_t states; // uint8_t variable to hold switch states
};

const int ledPins[] = {ledPin1, ledPin2, ledPin3}; // Array of LED pins

void handleState(uint8_t state) {
  Serial.print("Handling State: ");
  Serial.println(state);

  // Add your task logic here
  switch (state) {
    case 0: 
      Serial.println("Executing Green Task...");
      // Perform action for Green state
      break;
    case 1: 
      Serial.println("Executing Yellow Task...");
      // Perform action for Yellow state
      break;
    case 2: 
      Serial.println("Executing Red Task...");
      // Perform action for Red state
      break;
  }
}

void printChannelStates(uint8_t states) {
    Serial.print("LED States: ");
    for (int i = 0; i < 3; i++) {
        uint8_t state = (states >> i) & 1; // Extract individual bit
        Serial.print(state ? "ON " : "OFF ");  // Debug print
        digitalWrite(ledPins[i], state ? HIGH : LOW);
        if (state == HIGH) {
          handleState(i);  // Call function based on detected LED state
        }
    }
    Serial.println(); // New line after debug output
}

// Function to control L298N motor driver 
// Your joystick's Y-axis determines the speed & direction.
// Forward → IN1 = HIGH, IN2 = LOW
// Reverse → IN1 = LOW, IN2 = HIGH
// Stopping → IN1 = LOW, IN2 = LOW
//The PWM signal on ENA/ENB determines the motor speed
//////////////////////////////////////////
void controlMotors(int speed, int steering) {
    int leftSpeed=0, rightSpeed=0;
    bool spinMode = false;

    // check for spin mode
    if (speed == 0 && (steering < 75 || steering > 105))
        spinMode = true;

    // set motor direction
    if (spinMode) {
        if(steering < 75){
            // spin left (right forward, left backward)
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        }
        else{
            // spin right (left forward, right backward)
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
        }
    }
    else if (speed > 0) { // all Forward
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else if (speed < 0) { // all Reverse
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    } else { // Stop
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
    }

    if(spinMode){
        leftSpeed = map(steering, 0, 180, -255, 255);
        rightSpeed = map(steering, 0, 180, -255, 255);
    }
    else{
        // Adjust speed based on steering
        //leftSpeed = speed - map(steering, 0, 180, -50, 50);
        //rightSpeed = speed + map(steering, 0, 180, -50, 50);
        leftSpeed = speed - map(steering, 0, 180, -125, 125);
        rightSpeed = speed + map(steering, 0, 180, -125, 125);
    
        leftSpeed = (constrain(leftSpeed, -255, 255)*speedCntl);
        rightSpeed = (constrain(rightSpeed, -255, 255)*speedCntl);
    }
    analogWrite(ENA, abs(leftSpeed));  // Set left motor speed
    analogWrite(ENB, abs(rightSpeed)); // Set right motor speed
}

void setup() {
    pinMode(switch1Pin, INPUT_PULLUP);
    pinMode(switch2Pin, INPUT_PULLUP);
    Serial.begin(9600);

    int switch1State = digitalRead(switch1Pin); // Read state of switch 1
    int switch2State = digitalRead(switch2Pin); // Read state of switch 2

    // Combine states into a single integer (0-3)
    int encodedValue = (!switch1State << 1) | (!switch2State);
    memset(addSel, '0', sizeof(addSel)); // Fill with default '0'

    Serial.print("Encoded Value: ");
    Serial.println(encodedValue); // Debugging check

    // Process the combined state with a switch structure
    switch (encodedValue) {
      case 0:
        memcpy(addSel, address1, sizeof(address1));
        Serial.println("Both switches are OFF (state 0)");
        break;
      case 1:
        memcpy(addSel, address2, sizeof(address2));
        Serial.println("Switch 1 is OFF, Switch 2 is ON (state 1)");
        break;
      case 2:
        memcpy(addSel, address3, sizeof(address3));
        Serial.println("Switch 1 is ON, Switch 2 is OFF (state 2)");
        break;
      case 3:
        memcpy(addSel, address4, sizeof(address4));
        Serial.println("Both switches are ON (state 3)");
        break;
    }
    memcpy(address, addSel, sizeof(addSel));  // Address for communication note: 6th element is a null char
    String formattedAddress = "";
    for (int i = 0; i < 5; i++) {
        formattedAddress += (char)address[i];  // Convert bytes to characters
    }
    Serial.print("Selected Address: ");
    Serial.println(formattedAddress);  // Prints as "00001", "00002", etc.
    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_LOW);
    radio.startListening();

    //Initialize functions led pins
    pinMode(ledPin1, OUTPUT); // Set A3 as an output pin
    pinMode(ledPin2, OUTPUT); // Set A4 as an output pin
    pinMode(ledPin3, OUTPUT); // Set A5 as an output pin

    // Setup L298N motor control pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
}

void loop() {
    if (radio.available()) {
        DataPacket data;
        radio.read(&data, sizeof(data));

        // Call functions to print the state of each channel
        printChannelStates(data.states);    

        // Map X-Axis (Steering) to Servo Angle (0-180)
        int steering = map(data.joyX, 0, 1023, 0, 180);
        int offset = 90 - map(JOY_CENTER, 0, 1023, 0, 180);
        steering += offset; 

        // Map Y-Axis (Speed) to Motor Speed (-255 to 255)
        int speed = map(data.joyY, 0, 1023, -255, 255);

        // Apply improved dead zone for joystick drift
        if (abs(speed) < DEAD_ZONE) {
            speed = 0;
        } else {
            if (speed > 0) {
                speed = map(speed, DEAD_ZONE, 255, 1, 255);
            } else if (speed < 0) {
                speed = map(speed, -255, -DEAD_ZONE, -255, -1);
            }
        }

        Serial.print("Steering: ");
        Serial.print(steering);
        Serial.print(" | Speed: ");
        Serial.println(speed);
        

        // Call the motor control function
        controlMotors(speed, steering);                

        // **Display Optimization**
        static int prevSteering = -1, prevSpeed = -1;  // Store previous values
        static unsigned long lastUpdateTime = 0;       // Timer for display refresh
        const unsigned long updateInterval = 100;      // Update every 100ms

        if ((steering != prevSteering || speed != prevSpeed) && millis() - lastUpdateTime >= updateInterval) {
            prevSteering = steering;
            prevSpeed = speed;
            lastUpdateTime = millis();  // Update timer
        }
    }
}
