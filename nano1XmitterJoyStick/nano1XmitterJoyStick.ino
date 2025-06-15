//nano1XmitterjoyStck5
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
//Use of Joystick to read speed and steering values
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
//################################################################################
//Added 3 Pushbutton switches to inputs d5(Green), d6(Yellow), and d7(Red)... 
//used 10 kOhms resistos as pulldowns. Use 10KOhm Resistor to GND
// SWITCH SETUP if Pushbutton:
// [ Arduino Nano ]           [ Push-Button Switch ] example: using D5 thru D7 in Nano1 Xmitter 
//       Pin 5,6,7 ---------------------|   O O  |--------3.2VDC  
//                       |          |________|
//                       >
//                       <10KOhm
//                       |
//                      GND
//################################################################################
//5/13/2025 Added address lock using dip encode switches for rcvvr
//xmitter match
//assigned A3 and A4
//Address Polling Tested on all switch positions: 4 addresses
//6/13/2025 commented out, address locking and added timestamp monitor
//################################################################################
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 9
#define CSN_PIN 10
#define POT1_PIN A0  // X:First potentiometer connected to A0
#define POT2_PIN A1  // Y:Second potentiometer connected to A1

//##########################ADDRESS SELECT###################
const byte address1[] = { '0', '0', '0', '0', '1' };  // Car 1
const byte address2[] = { '0', '0', '0', '0', '2' };  // Car 2
const byte address3[] = { '0', '0', '0', '0', '3' };  // Car 3
const byte address4[] = { '0', '0', '0', '0', '4' };  // Car 4

byte addSel[] = { '0', '0', '0', '0', '0' };
byte address[] = { '0', '0', '0', '0', '0' };

const int switch1Pin = A3; // Pin for switch 1 
const int switch2Pin = A4; // Pin for switch 2

//#############################################################
RF24 radio(CE_PIN, CSN_PIN);

// Joystick data structure total capacity 32 bytes
struct DataPacket {
    uint16_t pot1Value;  // First potentiometer (0-1023)
    uint16_t pot2Value;  // Second potentiometer (0-1023)
    uint8_t states;      // variable to hold switch states
    unsigned long timestamp;  // Added timestamp field
};

// Pin definitions for digital channels(Push Buttons)
const int channel1_pin = 5; // Assign digital pin D5
const int channel2_pin = 6; // Assign digital pin D6
const int channel3_pin = 7; // Assign digital pin D7


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
    memcpy(address, addSel, 5);  // Address for communication note: 6th element is a null char
    String formattedAddress = "";
    for (int i = 0; i < 5; i++) {
        formattedAddress += (char)address[i];  // Convert bytes to characters
    }
    Serial.print("Selected Address: ");
    Serial.println(formattedAddress);  // Prints as "00001", "00002", etc.
    
    // Configure digital channels as input
    pinMode(channel1_pin, INPUT);
    pinMode(channel2_pin, INPUT);
    pinMode(channel3_pin, INPUT);

    radio.begin();
    radio.setPALevel(RF24_PA_HIGH);//Max Xmit Power
    radio.setDataRate(RF24_250KBPS); // Lower dat rate for optimized range
    radio.setChannel(76);
    radio.openWritingPipe(address);
    radio.setRetries(2, 10);
    radio.stopListening(); // Set as transmitter

}

//############Address Polling##################################
// void altSetup() {
//     int switch1State = digitalRead(switch1Pin);
//     int switch2State = digitalRead(switch2Pin);

//     int encodedValue = (!switch1State << 1) | (!switch2State);
//     byte newAddSel[5]; // Temporary variable for new address selection

//     switch (encodedValue) {
//       case 0: memcpy(newAddSel, address1, sizeof(address1)); break;
//       case 1: memcpy(newAddSel, address2, sizeof(address2)); break;
//       case 2: memcpy(newAddSel, address3, sizeof(address3)); break;
//       case 3: memcpy(newAddSel, address4, sizeof(address4)); break;
//     }

//     if (memcmp(newAddSel, address, sizeof(address)) != 0) {  // Only update if different
//         memcpy(address, newAddSel, sizeof(address));  // Apply new address
//         radio.openWritingPipe(address);  // Update address for communication
//         Serial.print("Updated Address: ");
//         for (int i = 0; i < 5; i++) Serial.print((char)address[i]);
//         Serial.println();
//     }
// }

int cycleCounter = 0;

//###############################################################
void loop() {
  //  cycleCounter++;

  //   if (cycleCounter % 10 == 0) {  // Run every other cycle
  //       altSetup();
  //   }
   
   unsigned long sentTime = millis();
   Serial.print("Sent at: ");
   Serial.println(sentTime);

    DataPacket data;
    // Include timestamp in the packet
    data.timestamp = sentTime;
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
    delay(30);//200
}
