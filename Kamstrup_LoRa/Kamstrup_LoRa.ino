#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <ACROBOTIC_SSD1306.h>

#include "gcm.h"
#include "mbusparser.h"
#include "dasya_logo.h"
#include "secrets.h" // <-- create this file using "secrets.h.TEMPLATE"

#define DEBUG_BEGIN Serial.begin(115200);
#define DEBUG_PRINT(x) Serial.print(x); update_display(x);
#define DEBUG_PRINTLN(x) Serial.println(x); update_display(x);

/* Second serial port for meter reading */
#define RXD2 23 //16 is used for OLED_RST !
#define TXD2 17
#define BAUD_RATE 2400

const size_t headersize = 11;
const size_t footersize = 3;
uint8_t encryption_key[16];
uint8_t authentication_key[16];
uint8_t receiveBuffer[500];
uint8_t decryptedFrameBuffer[500];
VectorView decryptedFrame(decryptedFrameBuffer, 0);
MbusStreamParser streamParser(receiveBuffer, sizeof(receiveBuffer));
mbedtls_gcm_context m_ctx;

int8_t TX_INTERVAL = 110; // Time to wait between transmissions, not including TX windows
//int8_t TX_INTERVAL = 20; // Time to wait between transmissions, not including TX windows
int16_t payload[3];
/* Payload Format:
 * 0: Avg import in W
 * 1: Avg export in W
 * 2: Number of valid frames this is based on
 */


/* Heltec specifics */
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}
static osjob_t sendjob;
// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32},
};
static uint32_t mydata[64];

void setup() {
  DEBUG_BEGIN
  DEBUG_PRINTLN("Starting SPI")
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  SPI.begin(5, 19, 27);

  DEBUG_PRINTLN("Starting OLED display")
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 to high  
  Wire.begin(4,15);  
  oled.init();                      // Initialze SSD1306 OLED display
  oled.clearDisplay();

  // Show DASYA Logo. Not really needed for anything, but looks nice.
  draw_logo("Kamstrup <-> TTN");
  delay(3000);
  oled.clearDisplay();
  

  DEBUG_PRINTLN("Starting LMIC")
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  Serial.println("Initiating Serial2 on RX=23");
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RXD2, TXD2);
  hexStr2bArr(encryption_key, conf_key, sizeof(encryption_key));
  hexStr2bArr(authentication_key, conf_authkey, sizeof(authentication_key));
  Serial.println("Setup completed");
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}

void read_frame() {
  reset_payload();
  int32_t avg_im = 0;
  int32_t avg_ex = 0;
  int32_t read_cnt = 0;
  int break_time = os_getTime() + sec2osticks(TX_INTERVAL);
  int time_left;
  while(true) {
    if(os_getTime() > break_time) {
        DEBUG_PRINTLN("Timeout! Got "+String(read_cnt)+" frames.");
        payload[0] = avg_im;
        payload[1] = avg_ex;
        payload[2] = read_cnt;
        return;            
      }
    while (Serial2.available() > 0) {
      if (streamParser.pushData(Serial2.read())) {
        VectorView frame = streamParser.getFrame();
        if (streamParser.getContentType() == MbusStreamParser::COMPLETE_FRAME) {
          DEBUG_PRINTLN("Frame complete. Decrypting...");
          if (!decrypt(frame))
          {
            DEBUG_PRINTLN("Decrypt failed");
          }
          MeterData md = parseMbusFrame(decryptedFrame);
          if(md.activePowerPlusValid && md.activePowerMinusValid) {
            read_cnt++;
            DEBUG_PRINTLN("Decrypt ok, cnt:"+String(read_cnt));
            avg_im = do_average(avg_im, md.activePowerPlus, read_cnt);
            avg_ex = do_average(avg_ex, md.activePowerMinus, read_cnt);
            // Update display
            update_display(md.activePowerPlus, md.activePowerMinus, read_cnt, time_left);
          } else {
            // Update display
            update_display(read_cnt, time_left);
          }
          time_left = osticks2ms(break_time - os_getTime())/1000;
        }
      }
    }
  }
}

int32_t do_average(int32_t avg, int32_t val, int32_t cnt) {
  return (avg*(cnt-1)+val)/cnt;
}

void printHex(const unsigned char* data, const size_t length) {
  for (int i = 0; i < length; i++) {
    Serial.printf("%02X", data[i]);
  }
}

void printHex(const VectorView& frame) {
  for (int i = 0; i < frame.size(); i++) {
    Serial.printf("%02X", frame[i]);
  }
}

bool decrypt(const VectorView& frame) {

  if (frame.size() < headersize + footersize + 12 + 18) {
    Serial.println("Invalid frame size.");
  }

  memcpy(decryptedFrameBuffer, &frame.front(), frame.size());

  uint8_t system_title[8];
  memcpy(system_title, decryptedFrameBuffer + headersize + 2, 8);

  uint8_t initialization_vector[12];
  memcpy(initialization_vector, system_title, 8);
  memcpy(initialization_vector + 8, decryptedFrameBuffer + headersize + 14, 4);

  uint8_t additional_authenticated_data[17];
  memcpy(additional_authenticated_data, decryptedFrameBuffer + headersize + 13, 1);
  memcpy(additional_authenticated_data + 1, authentication_key, 16);

  uint8_t authentication_tag[12];
  memcpy(authentication_tag, decryptedFrameBuffer + headersize + frame.size() - headersize - footersize - 12, 12);

  uint8_t cipher_text[frame.size() - headersize - footersize - 18 - 12];
  memcpy(cipher_text, decryptedFrameBuffer + headersize + 18, frame.size() - headersize - footersize - 12 - 18);

  uint8_t plaintext[sizeof(cipher_text)];

  mbedtls_gcm_init(&m_ctx);
  int success = mbedtls_gcm_setkey(&m_ctx, MBEDTLS_CIPHER_ID_AES, encryption_key, sizeof(encryption_key) * 8);
  if (0 != success) {
    Serial.println("Setkey failed: " + String(success));
    return false;
  }
  success = mbedtls_gcm_auth_decrypt(&m_ctx, sizeof(cipher_text), initialization_vector, sizeof(initialization_vector),
                                     additional_authenticated_data, sizeof(additional_authenticated_data), authentication_tag, sizeof(authentication_tag),
                                     cipher_text, plaintext);
  if (0 != success) {
    Serial.println("authdecrypt failed: " + String(success));
    return false;
  }
  mbedtls_gcm_free(&m_ctx);

  //copy replace encrypted data with decrypted for mbusparser library. Checksum not updated. Hopefully not needed
  memcpy(decryptedFrameBuffer + headersize + 18, plaintext, sizeof(plaintext));
  decryptedFrame = VectorView(decryptedFrameBuffer, frame.size());

  return true;
}

void hexStr2bArr(uint8_t* dest, const char* source, int bytes_n)
{
  uint8_t* dst = dest;
  uint8_t* end = dest + sizeof(bytes_n);
  unsigned int u;

  while (dest < end && sscanf(source, "%2x", &u) == 1)
  {
    *dst++ = u;
    source += 2;
  }
}

/** METHODS FROM HELTEC OTAA SKETCH **/
void printHex2(unsigned v) {
  v &= 0xff;
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      DEBUG_PRINTLN(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      DEBUG_PRINTLN(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      DEBUG_PRINTLN(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      DEBUG_PRINTLN(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      DEBUG_PRINTLN(F("EV_JOINING"));
      break;
    case EV_JOINED:
      DEBUG_PRINTLN(F("EV_JOINED"));
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
        for (size_t i = 0; i < sizeof(artKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(nwkKey[i]);
        }
        Serial.println();
      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      break;
    case EV_JOIN_FAILED:
      DEBUG_PRINTLN(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      DEBUG_PRINTLN(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      DEBUG_PRINTLN(F("EV_TXCOMPLETE (inc. RX"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println("Received ack");
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime(), do_send);
      break;
    case EV_LOST_TSYNC:
      DEBUG_PRINTLN(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      DEBUG_PRINTLN(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      DEBUG_PRINTLN(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      DEBUG_PRINTLN(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      DEBUG_PRINTLN(F("EV_LINK_ALIVE"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_SCAN_FOUND:
      ||    Serial.println(F("EV_SCAN_FOUND"));
      ||    break;
    */
    case EV_TXSTART:
      DEBUG_PRINTLN(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      DEBUG_PRINTLN(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      DEBUG_PRINTLN(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;

    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    DEBUG_PRINTLN(F("OP_TXRXPEND, not sending"));
    os_setTimedCallback(&sendjob, os_getTime(), do_send);
  } else {
    DEBUG_PRINTLN("Listening for frame...");
    read_frame();
    DEBUG_PRINTLN("Back to do send");
    // Prepare upstream data transmission at the next possible time.
    if(payload[2] > 0) {
      DEBUG_PRINTLN("Starting send...");
      LMIC_setTxData2(1, (xref2u1_t)&payload, sizeof(payload), 0);
      DEBUG_PRINTLN("Packet queued");  
    } else {
      DEBUG_PRINTLN("No valid frames.");
      os_setTimedCallback(&sendjob, os_getTime(), do_send);
    }
  }
}

void update_display(int im, int ex, int cnt, int time_left) {
  oled.setTextXY(0,0);              // Set cursor position, start of line 0
  oled.putString("Import: "+String(im)+" W  ");
  oled.setTextXY(1,0);              // Set cursor position, start of line 1
  oled.putString("Export: "+String(ex)+" W  ");
  oled.setTextXY(2,0);              // Set cursor position, start of line 1
  oled.putString("Rds: "+String(cnt)+", T: "+String(time_left)+"s ");
}

void update_display(int cnt, int time_left) {
  oled.setTextXY(2,0);              // Set cursor position, start of line 1
  oled.putString("Rds: "+String(cnt)+", T: "+String(time_left)+"s ");
}

void update_display() {
  update_display(payload[0],payload[1],payload[2], -1);
}

void update_display(String msg) {
  oled.setTextXY(4,0);       
  oled.putString("Msg:");
  int line = 5;
  int curs = 0;
  while(line < 8) {
    oled.setTextXY(line,0);
    oled.putString("                "); // First clear the line
    oled.setTextXY(line,0); 
    oled.putString(msg.substring(curs,curs+16));
    msg = msg.substring(curs);
    curs += 16;
    line++;
  }
}

void reset_payload() {
  // Reset the payload values
  payload[2] = 0;
  update_display();
  payload[1] = 0;
  payload[0] = 0;
}
