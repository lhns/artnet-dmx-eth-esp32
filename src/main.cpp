#include <Arduino.h>
#include <ETH.h>
#include <esp_dmx.h>
#include <ArtnetWifi.h>

/*
   * ETH_CLOCK_GPIO0_IN   - default: external clock from crystal oscillator
   * ETH_CLOCK_GPIO0_OUT  - 50MHz clock from internal APLL output on GPIO0 - possibly an inverter is needed for LAN8720
   * ETH_CLOCK_GPIO16_OUT - 50MHz clock from internal APLL output on GPIO16 - possibly an inverter is needed for LAN8720
   * ETH_CLOCK_GPIO17_OUT - 50MHz clock from internal APLL inverted output on GPIO17 - tested with LAN8720
*/
#ifdef ETH_CLK_MODE
#undef ETH_CLK_MODE
#endif
#define ETH_CLK_MODE    ETH_CLOCK_GPIO17_OUT

// Pin# of the enable signal for the external crystal oscillator (-1 to disable for internal APLL source)
#define ETH_POWER_PIN   -1

// Type of the Ethernet PHY (LAN8720 or TLK110)
#define ETH_TYPE        ETH_PHY_LAN8720

// I²C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)
#define ETH_ADDR        1

// Pin# of the I²C clock signal for the Ethernet PHY
#define ETH_MDC_PIN     23

// Pin# of the I²C IO signal for the Ethernet PHY
#define ETH_MDIO_PIN    18

const dmx_port_t dmx_num = DMX_NUM_2;

const int dmx_tx_pin = 32;
const int dmx_rx_pin = 33;
const int dmx_rts_pin = 14;

static bool eth_connected = false;

void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_ETH_START:
            Serial.println("ETH Started");
            //set eth hostname here
            ETH.setHostname("esp32-ethernet");
            break;
        case ARDUINO_EVENT_ETH_CONNECTED:
            Serial.println("ETH Connected");
            break;
        case ARDUINO_EVENT_ETH_GOT_IP:
            Serial.print("ETH MAC: ");
            Serial.print(ETH.macAddress());
            Serial.print(", IPv4: ");
            Serial.print(ETH.localIP());
            if (ETH.fullDuplex()) {
                Serial.print(", FULL_DUPLEX");
            }
            Serial.print(", ");
            Serial.print(ETH.linkSpeed());
            Serial.println("Mbps");
            eth_connected = true;
            break;
        case ARDUINO_EVENT_ETH_DISCONNECTED:
            Serial.println("ETH Disconnected");
            eth_connected = false;
            break;
        case ARDUINO_EVENT_ETH_STOP:
            Serial.println("ETH Stopped");
            eth_connected = false;
            break;
        default:
            break;
    }
}

bool isConnected() {
    return eth_connected;
}

uint8_t dmxData[DMX_MAX_PACKET_SIZE];

void onArtnetFrame(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t *data) {
    if (universe == 0) {
        dmxData[0] = 0;
        for (int i = 0; i < length && i < DMX_MAX_PACKET_SIZE - 1; i++) {
            dmxData[i + 1] = i < length ? data[i] : 0;
        }
        dmx_write(dmx_num, dmxData, DMX_MAX_PACKET_SIZE);
        dmx_send(dmx_num, DMX_MAX_PACKET_SIZE);
        dmx_wait_sent(dmx_num, DMX_TIMEOUT_TICK);
    }
}

ArtnetWifi artnet;

void setup() {
    Serial.begin(115200);
    WiFi.onEvent(WiFiEvent);
    ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
    artnet.setArtDmxCallback(onArtnetFrame);
    artnet.begin();
    dmx_set_pin(dmx_num, dmx_tx_pin, dmx_rx_pin, dmx_rts_pin);
    dmx_driver_install(dmx_num, DMX_DEFAULT_INTR_FLAGS);
    Serial.println("Initialized");
}


void loop() {
    if (isConnected()) {
        artnet.read();
    }
}
