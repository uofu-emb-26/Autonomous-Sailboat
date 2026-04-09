#include <Arduino.h>
#include <LoRa.h>

#define RFM95_CS   12
#define RFM95_RST   7
#define RFM95_INT   6
#define LORA_FREQ   915E6
                                                                                                                         
#define LORA_SF 7 // spread factor 
#define LORA_BW 125E3 //125 kHz bandwitdh
#define LORA_CR 5 // coding rate (redundant bits)
#define LORA_SYNC_WORD 0x12
#define LORA_PREAMBLE 8
#define LORA_TX_POWER 17 // 17 dBm, max of 20 dBm for the whip antennas
#define ADDR_SAMD21 0xAA
#define ADDR_STM32 0xBB

// How long to wait accumulating bytes before sending (ms)
#define FLUSH_TIMEOUT_MS 10
// Max bytes per packet (LoRa max is 255)
#define MAX_PKT_SIZE 128

static uint8_t txBuf[MAX_PKT_SIZE];
static uint8_t txLen = 0;
static uint32_t lastByteTime = 0;
static uint8_t seqNum = 0;

void sendPacket(const uint8_t* payload, uint8_t len) {
  if (!LoRa.beginPacket()) 
      return;

  LoRa.write(ADDR_STM32);
  LoRa.write(ADDR_SAMD21);
  LoRa.write(seqNum++);
  LoRa.write(len);        // payload length
  LoRa.write(payload, len);
  LoRa.endPacket();
}

void handleIncoming(int pktSize){
  if (pktSize < 4){              //not enough header
    return;
  }

  uint8_t dest = LoRa.read();
  uint8_t src = LoRa.read();
  uint8_t seq = LoRa.read();
  uint8_t len = LoRa.read();

  if (dest != ADDR_SAMD21){
    while (LoRa.available()) 
      LoRa.read();
    return;
  }

  uint8_t payload[MAX_PKT_SIZE];
  
  for (uint8_t i = 0; i < len && i < MAX_PKT_SIZE; i++) {
    payload[i] = (uint8_t)LoRa.read();
  }

  // Process payload
  while (LoRa.available()) LoRa.read();  // flush leftover bytes                                                                                         
    /*                                                                                                                                                       
    SerialUSB.print("[RX from 0x");
    SerialUSB.print(src, HEX);
    SerialUSB.print(" seq=");
    SerialUSB.print(seq);
    SerialUSB.print("rssi=");
    SerialUSB.print(LoRa.packetRssi());
    SerialUSB.print("dBm] ");
   */
  for (uint8_t i = 0; i < len && i < MAX_PKT_SIZE; i++){
    SerialUSB.write(payload[i]);
  }
  SerialUSB.println(); 
}

void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB);  // wait for USB serial to come up

  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);

  if (!LoRa.begin(LORA_FREQ)) {
    SerialUSB.println("LoRa init failed. Check wiring.");
    while (1);
  }

  // Sane defaults for short-range use
  LoRa.setSpreadingFactor(LORA_SF);      // SF7 = fastest, lowest range
  LoRa.setSignalBandwidth(LORA_BW);  // 125 kHz bandwidth
  LoRa.setCodingRate4(LORA_CR);          // 4/5 coding rate
  LoRa.setTxPower(LORA_TX_POWER);             // dBm, 2–20 valid
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setPreambleLength(LORA_PREAMBLE); 
  LoRa.enableCrc(); // for error checking

  SerialUSB.println("LoRa ready. Type a command and press Enter.");
}

static void flushTx() {
  if (txLen == 0) return;
  LoRa.beginPacket();
  LoRa.write(txBuf, txLen);
  LoRa.endPacket();
  txLen = 0;
}

void loop() {
  // USB → LoRa: buffer bytes, flush on timeout or full buffer
  /*
  while (SerialUSB.available())
    unit8_t b = (uint8_t)SerialUSB.read();
    SerialUSB.write(b);  // echo

    if ((b == '\n' || b == '\r')){
      if (txLen > 0) {
        flushTx();
        sendPacket(txBuf, txLen);
        txLen = 0;
      }
      else {
        if (txLen < MAX_PKT_SIZE) {
          txBuf[txLen++] = b;
          lastByteTime = millis();
        }
      }
    }
  }
  */
    
    txBuf[txLen++] = (uint8_t)SerialUSB.read();

    lastByteTime = millis();
    if (txLen >= MAX_PKT_SIZE) {
      flushTx();
    }

  // Flush after idle gap — makes it feel like a stream
  if (txLen > 0 && (millis() - lastByteTime) >= FLUSH_TIMEOUT_MS){
    flushTx();
  }

  // LoRa → USB
  int pktSize = LoRa.parsePacket();
  if (pktSize > 0)
    while (LoRa.available()) {
      SerialUSB.write((uint8_t)LoRa.read());
    }
  }