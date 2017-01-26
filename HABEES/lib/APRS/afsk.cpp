#include "afsk.h"

//Frequency Constants
static const int MARK_FREQ = 1200;
static const int SPACE_FREQ = 2200;
static const float PREEMPHASIS_RATIO = 0.75;

//Time Constants
static const int BIT_RATE = 1200; //APRS standard.
static const uint32_t SAMPLE_RATE = 9600;
static const uint32_t SAMPLES_PER_BIT = SAMPLE_RATE / BIT_RATE;
static const int PTT_DELAY = 1000; //ms
static const int DEBUG_PRESCALER = 1;//Set to 1 for full speed, higher to slow down interrupts by that factor

//Phase Delta Constants
static const uint8_t SINE_WAVE_RESOLUTION = 12;
static const int SINE_TABLE_LENGTH = 512;
static const int SINE_WAVE_MAX = pow(2,SINE_WAVE_RESOLUTION + 1);

static const int ANGLE_RESOLUTION_PRESCALER = 1000;
static const uint32_t MARK_INCREMENT = ANGLE_RESOLUTION_PRESCALER * SINE_TABLE_LENGTH * MARK_FREQ / SAMPLE_RATE;
static const uint32_t SPACE_INCREMENT = ANGLE_RESOLUTION_PRESCALER * SINE_TABLE_LENGTH * SPACE_FREQ / SAMPLE_RATE;

// Interrupt-related constants and instance variables
IntervalTimer interruptTimer;
volatile bool txing = false;
volatile bool rxing = false;

volatile int freq = MARK_FREQ;

volatile uint16_t analogOut = 0;
volatile uint32_t currentPhase = 0; //32bit integer for higher res integer math
volatile uint32_t increment = MARK_INCREMENT;

//the index used to count how many samples we've sent for the current bit
volatile byte bitIndex = 0;
//the index denoting which bit within the current byte we are transmitting
volatile byte byteIndex = 0;
//the index denoting which bit we are transmitting
volatile int packetIndex = 0;
//the current byte being transmitted
volatile char currentByte = 0;
//data structure for the packet
volatile uint8_t* packet;
//the total number of bits in the packet
volatile int packet_size = 0;

// 1200 Baud settings
void radioISR(void);

uint16_t sineLookup(const int currentPhase) {
    uint16_t analogOut = 0;
    int index = currentPhase % (SINE_TABLE_LENGTH/2);
    analogOut = (index < SINE_TABLE_LENGTH/4) ? sineTable[index] : sineTable[SINE_TABLE_LENGTH/2 - index - 1];
    analogOut = (currentPhase >= ( SINE_TABLE_LENGTH/2 )) ? ((SINE_WAVE_MAX/2)- 1) - analogOut : analogOut;
    return analogOut;
}

void afsk_modulate_packet(volatile uint8_t *buffer, int size, int trailingBits) {
    packet = buffer;
    packet_size = size;
    afsk_timer_begin();
}

void afsk_timer_begin() {
    interruptTimer.begin(radioISR,(float)1E6/(SAMPLE_RATE));
    resetVolatiles();
    digitalWrite(DRA_PTT,LOW);
    delay(PTT_DELAY);//todo: change this to match DRA object's delay
    txing = true;
}

void resetVolatiles() {
    freq = MARK_FREQ;
    analogOut = 0;
    currentPhase = 0;
    increment = MARK_INCREMENT;
    bitIndex = 0;
    byteIndex = 0;
    packetIndex = 0;
    currentByte = 0;
}

void afsk_timer_stop() {
    interruptTimer.end();
}

void radioISR() {
    if(!txing) return;
    if(bitIndex == 0) {
        if(packetIndex >= packet_size) {
            txing = false;
            analogWrite(DRA_MIC,ANALOG_MAX/2);
            digitalWrite(DRA_PTT,HIGH);
            afsk_timer_stop();
            return;
        } else if (byteIndex== 0) {
            currentByte = packet[packetIndex/8]; //grab the next byte to transmit
            byteIndex = B10000000; //reset the byte bitmask to the first bit
        }
        //if we are supposed to send a 1 at this bit, maintain current TX frequency (NRZ encoding), unless we have sent 5 1s in a row and must stuff a 0
        if(currentByte & byteIndex) { //transmitting a 1
            //freq remains unchanged
        } else { //transmitting a 0
            freq = (freq == MARK_FREQ) ? SPACE_FREQ : MARK_FREQ; //if we are supposed to send a 0 at this bit, change the current TX frequency
        }
        increment = (freq == MARK_FREQ) ? MARK_INCREMENT : SPACE_INCREMENT; //adjust the phase delta we add each sample depending on whether we are transmitting 0 or 1
        bitIndex = SAMPLES_PER_BIT; //reset the bitIndex;
        packetIndex++;
        byteIndex>>=1; //shift to the next bit to be transmitted
    } //endif: bitIndex == 0
    currentPhase+=increment;
    if(currentPhase > SINE_TABLE_LENGTH*ANGLE_RESOLUTION_PRESCALER) {
        currentPhase-=SINE_TABLE_LENGTH*ANGLE_RESOLUTION_PRESCALER;
    }
//    Serial.println(currentPhase/ANGLE_RESOLUTION_PRESCALER);
    bitIndex--;
    analogOut = sineLookup((currentPhase/ANGLE_RESOLUTION_PRESCALER));
//    Serial.println(analogOut);
    if(freq == MARK_FREQ) {
        analogOut = analogOut*PREEMPHASIS_RATIO;
    }
    analogWrite(DRA_MIC, analogOut);
}
