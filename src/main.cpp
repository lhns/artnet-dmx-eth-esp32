#include <Arduino.h>
#include <ETH.h>
#include <esp_dmx.h>
#include <ArtnetWifi.h>

const dmx_port_t dmx_num = DMX_NUM_1;
const int dmx_tx_pin = 15;
const int dmx_rx_pin = 14;
const int dmx_rts_pin = 4;
const int dmx_universe_0_pin = 36;
const int dmx_universe_1_pin = 39;
const int dmx_universe_2_pin = 35;

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
    int dmx_universe =
            digitalRead(dmx_universe_0_pin) |
            digitalRead(dmx_universe_1_pin) << 1 |
            digitalRead(dmx_universe_2_pin) << 2;

    if (universe == dmx_universe) {
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
    Serial.println("Initializing...");

    WiFi.onEvent(WiFiEvent);
    ETH.begin();

    artnet.setArtDmxCallback(onArtnetFrame);
    artnet.begin();

    dmx_set_pin(dmx_num, dmx_tx_pin, dmx_rx_pin, dmx_rts_pin);
    dmx_driver_install(dmx_num, DMX_DEFAULT_INTR_FLAGS);

    pinMode(dmx_universe_0_pin, INPUT);
    pinMode(dmx_universe_1_pin, INPUT);
    pinMode(dmx_universe_2_pin, INPUT);

    Serial.println("Initialized");
}

void loop() {
    if (isConnected()) {
        artnet.read();
    }
}
