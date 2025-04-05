//######################Program Name: nano2receiverOLEDProto4.ino####################
//nano2Rcvr: Using an OLED display for troubleshooting purposes.
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
// GND	                      GND
// CE	                        Any digital pin (e.g., D9)
// CSN	                      Any digital pin (e.g., D10)
// SCK	                      D13 (SCK)
// MOSI	                      D11 (MOSI)
// MISO	                      D12 (MISO)
// IRQ	                      Not needed (leave unconnected)
//##################################################################################
// #########################Joystick Module Pinout##################################
// Joystick        Pin	Function	Arduino Nano Pin
// VCC	           Power (5V)	            5V
// GND	           Ground	                GND
// VRX	           X-axis output	        A0
// VRY	           Y-axis output	        A1
// SW(Optional)    Push button switch	    D2 *(if used, needs pinMode(D2, INPUT_PULLUP))         
// ##################Joystick Wiring Diagram########################################################
// VRX → A0 (X-axis, controls left/right)
// VRY → A1 (Y-axis, controls forward/backward)
// VCC → 5V
// GND → GND
// SW (if used) → D2, set as INPUT_PULLUP
//NOTE: Once connected, your X-Axis (VRX) should control steering (0-180 degrees) and Y-Axis (VRY)...
// should control speed/direction (0-255 forward, -255 reverse).
//##########################L298N to Arduino Nano Wiring#############################################
// L298N
// Pin	          Function	                                  Arduino Nano Pin
// 12V	      Power for motors (Use external 12V battery)	    12V Battery
// GND	      Ground	                                          GND all grounds must be common
// 5V OUT	    Regulated 5V output	                            Not needed (Nano already powered)
// IN1	      Motor A Direction                                 D4	
// IN2	      Motor A Direction                                 D8//D5	
// IN3	      Motor B Direction                                 D6	
// IN4	      Motor B Direction                                 D7	
// ENA	      Motor A Speed                                     D3 (PWM)	
// ENB	      Motor B Speed                                     D5//D8 (PWM)	
//####################Battery Module###################################################################
//Need Three 3.7V Li-ion rechargeable batteries, providing total of 11V, connected in series.
//Need 3-Battery holder and charger.
//####################################################################################################
//Disclaimer:This code is open and free to use, modify, and even share.
// It’s based on common programming principles and standard usage
// of the Arduino, nRF24L01, L298N, and OLED libraries, which are all widely used in open-source projects.
//########################################################################################################
//##################################Updates:##############################################################
//Added a new 'state' var to struct DataPacket to record switch values
//set in xmitter module.

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define CE_PIN 9
#define CSN_PIN 10
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define DEAD_ZONE 15    // Dead zone to avoid fluctuations
#define JOY_CENTER 508  // Adjust for joystick calibration

// L298N Motor Driver Pins
#define IN1 4
#define IN2 8//5
#define IN3 6
#define IN4 7
#define ENA 3  // Left Motor PWM
#define ENB 5//8 // Right Motor PWM
//Speed Multiplier
float speedCntl=.5;
//Declare pins to be used by functions printChannelXState
int ledPin1 = A0; // Assign Analog pin A0 as a digital pin for the first LED
int ledPin2 = A1; // Assign Analog pin A1 as a digital pin for the second LED
int ledPin3 = A2; // Assign Analog pin A2 as a digital pin for the third LED

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Joystick data structure total capacity 32 bytes
struct DataPacket {
    uint16_t joyX; // Joystick X-axis, using 2 bytes
    uint16_t joyY; // Joystick Y-axis, using 2 bytes
    uint8_t states; // uint8_t variable to hold switch states
};

// Function to decode and print Channel 1 state GREEN
void printChannel1State(uint8_t states) {
    uint8_t channel1_state = (states >> 0) & 1; // Extract bit 0
    Serial.print("Channel 1: ");
    Serial.println(channel1_state ? "ON" : "OFF");
    if (channel1_state == 1){
      digitalWrite(ledPin1, HIGH);
    }else{
      digitalWrite(ledPin1, LOW);
    }
}

// Function to decode and print Channel 2 state YELLOW
void printChannel2State(uint8_t states) {
    uint8_t channel2_state = (states >> 1) & 1; // Extract bit 1
    Serial.print("Channel 2: ");
    Serial.println(channel2_state ? "ON" : "OFF");
    if (channel2_state == 1){
      digitalWrite(ledPin2, HIGH);
    }else{
      digitalWrite(ledPin2, LOW);
    }
}

// Function to decode and print Channel 3 state RED
void printChannel3State(uint8_t states) {
    uint8_t channel3_state = (states >> 2) & 1; // Extract bit 2
    Serial.print("Channel 3: ");
    Serial.println(channel3_state ? "ON" : "OFF");
    if (channel3_state == 1){
      digitalWrite(ledPin3, HIGH);
    }else{
      digitalWrite(ledPin3, LOW);
    }

}

// Function to control L298N motor driver 
// Your joystick's Y-axis determines the speed & direction.
// Forward → IN1 = HIGH, IN2 = LOW
// Reverse → IN1 = LOW, IN2 = HIGH
// Stopping → IN1 = LOW, IN2 = LOW
//The PWM signal on ENA/ENB determines the motor speed
//////////////////////////////////////////
void controlMotors(int speed, int steering) {
    int leftSpeed, rightSpeed;
    bool spinMode = false;

    // check for spin mode
    if (speed = 0 && steering < 75 || steering > 105)
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

    if(spinmode){
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
    Serial.begin(9600);
    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_LOW);
    radio.startListening();

    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }
    display.clearDisplay();
    display.display();

    //Initialize functions led pins
    pinMode(ledPin1, OUTPUT); // Set A0 as an output pin
    pinMode(ledPin2, OUTPUT); // Set A1 as an output pin
    pinMode(ledPin3, OUTPUT); // Set A2 as an output pin

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
        printChannel1State(data.states);
        printChannel2State(data.states);
        printChannel3State(data.states);        

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

            // Only clear specific text areas instead of full screen
            display.fillRect(70, 10, 50, 20, BLACK);  // Clear STR value area
            display.fillRect(70, 40, 50, 20, BLACK);  // Clear SPD value area

            display.setTextSize(2);
            display.setTextColor(WHITE);

            display.setCursor(10, 10);
            display.print("STR:");
            display.setCursor(70, 10);
            display.print(steering);

            display.setCursor(10, 40);
            display.print("SPD:");
            display.setCursor(70, 40);
            display.print(speed);

            display.display();  // Refresh OLED
        }
    }
}
