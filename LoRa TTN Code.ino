#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>

#include <Wire.h>
#include <BH1750.h>
 
// Include your sensor library
#include "DHT.h"
#define DHTPIN 5     //DHT pin 
#define DHTTYPE DHT11   //DHT11
DHT dht(DHTPIN, DHTTYPE); 

BH1750 lightMeter;

// End your sensor library

// LoRaWAN NwkSKey, network session key replce with your KEY
static const PROGMEM u1_t NWKSKEY[16] = { 0xD3, 0x3F, 0x08, 0x44, 0x70, 0xAE, 0xE7, 0xBF, 0xD4, 0x84, 0x18, 0x10, 0xDD, 0x83, 0xFA, 0x9B };

// LoRaWAN AppSKey, application session key replce with your KEY
static const u1_t PROGMEM APPSKEY[16] = { 0x48, 0x70, 0xD0, 0x4B, 0xBB, 0xFD, 0x11, 0x6E, 0xF1, 0x66, 0xC8, 0x22, 0xB4, 0xDD, 0xF8, 0xA8 };

// LoRaWAN end-device address (DevAddr) replce with your DEVICE ADRESS
static const u4_t DEVADDR = 0x26011632 ;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

//Setup the interval between Payload updates in seconds
const unsigned TX_INTERVAL = 15;

// Pin mapping RFM95
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void payload() {
  CayenneLPP lpp(51);
  lpp.reset();   
//Fetch your sensor data
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  if (isnan(humidity) || isnan(temperature) ) {
  Serial.println("Failed to read from DHT sensor!");
    return; 
  } 
//End fetching your sensor data   

//Setup Cayenne LPP payload replace CHANNEL with the channel number and value with your sensor data
  //lpp.addDigitalInput(CHANNEL, uint8_t value);
  //lpp.addDigitalOutput(CHANNEL, uint8_t value);
  //lpp.addAbalogInput(CHANNEL, float value);
  //lpp.addAnalogOutput(CHANNEL, float value);
  //lpp.addLuminosity(CHANNEL, uint16_t lux);
  //lpp.addPresence(CHANNEL, uint8_tvalue);
  //lpp.addTemperature(CHANNEL, float celcius);
  //lpp.addRelativeHumidity(CHANNEL, float rh);
  //lpp.addAccelerometer(CHANNEL, float x, float y, float z);
  //lpp.addBarometricPressure(CHANNEL, float hpa);
  //lpp.addGyrometer(CHANNEL, floatx, float y, float z);
  //lpp.addGPS(CHANNEL, float latitude, float longitude, float meters);

//Sending your sensor data to the Cayenne dashboard    
uint16_t lux = lightMeter.readLightLevel(); 
  lpp.addTemperature(1, temperature);
  lpp.addRelativeHumidity(2, humidity); 
  lpp.addLuminosity(3, lux);  

     
  LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
//End sending your sensor data to the Cayenne dashboard  

// Serial print Payload data for debugging  
  Serial.print("Humidity: ");  
  Serial.print(humidity);  
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" *C ");    
  Serial.print("Light: ");
  Serial.print(lux); 
  Serial.println(" lx"); 
  Serial.println(F("Packet queued"));  

}
void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {  
    payload();
    }
    // Next TX is scheduled after TX_COMPLETE event.
}
void setup() {   
    Serial.begin(115200);
    Serial.println(F("Starting"));
//Start sensor setup
    dht.begin();
    lightMeter.begin();

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically   
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    //LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // Disable channels for testing on 1ch gateways uncomment the channels you dont use.
    // LMIC_disableChannel(0);    
    // LMIC_disableChannel(1);
    // LMIC_disableChannel(2);
    // LMIC_disableChannel(3);
    // LMIC_disableChannel(4);
    // LMIC_disableChannel(5);
    // LMIC_disableChannel(6);
    // LMIC_disableChannel(7);
    // LMIC_disableChannel(8);
     
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
