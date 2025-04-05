//nano1XmitterjoyStck3OLED
//Use of Joystick to read speed and steering values

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 9
#define CSN_PIN 10
#define POT1_PIN A0  // X:First potentiometer connected to A0
#define POT2_PIN A1  // Y:Second potentiometer connected to A1

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";  // Address for communication

struct DataPacket {
    uint16_t pot1Value;  // First potentiometer (0-1023)
    uint16_t pot2Value;  // Second potentiometer (0-1023)
    uint8_t states;      // variable to hold switch states
};

// Pin definitions for digital channels
const int channel1_pin = 5; // Assign digital pin D5
const int channel2_pin = 6; // Assign digital pin D6
const int channel3_pin = 7; // Assign digital pin D7


void setup() {
    // Configure digital channels as input
    pinMode(channel1_pin, INPUT);
    pinMode(channel2_pin, INPUT);
    pinMode(channel3_pin, INPUT);

    Serial.begin(9600);
    radio.begin();
    
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_LOW);
    radio.stopListening(); // Set as transmitter
}

void loop() {
    DataPacket data;
    // Read the state of each digital channel
    uint8_t channel1_state = digitalRead(channel1_pin);
    uint8_t channel2_state = digitalRead(channel2_pin);
    uint8_t channel3_state = digitalRead(channel3_pin);

    // Update the `states` field in the struct
    data.states = (channel1_state << 0) | (channel2_state << 1) | (channel3_state << 2);
  
    //Read joystick vals
    data.pot1Value = analogRead(POT1_PIN);  // Read first potentiometer
    data.pot2Value = analogRead(POT2_PIN);  // Read second potentiometer

    Serial.print("Sending Pot1(X): ");
    Serial.print(data.pot1Value);
    Serial.print(" | Pot2(Y): ");
    Serial.println(data.pot2Value);

    radio.write(&data, sizeof(DataPacket));  // Send both values
    delay(200);
}
