// written by Folkert van Heusden <mail@vanheusden.com>
// Released under AGPL v3.0
// with code from https://github.com/PaulStoffregen/Time
// and http://jtauber.github.io/mars-clock/
#include <TimeLib.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <PubSubClient.h>
char mqttServer[64] = "";
char mqttTopic[128] = "power_state";

WiFiClient client;
PubSubClient psclient(client);

volatile bool powerState = true;

void callback(char* topic, byte* payload, unsigned int length) {
	int s = (char)payload[0] - '0';

	static bool ps = false, first = true;
	powerState = s >= 1;

	if (ps != powerState || first) {
		ps = powerState;
		first = false;
		setAllColor(!powerState * 50, powerState * 50, 0);

		Serial.print(F("Power state change: "));
		Serial.println(powerState);

		delay(50);
		setAllColor(0, 0, 0);
	}
}

#include <Adafruit_NeoPixel.h>
#define NUMBER_OF_PIXELS 60
#define PIN_NEOPIXELS 2
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMBER_OF_PIXELS, PIN_NEOPIXELS, NEO_GRB + NEO_KHZ800);
const uint8_t brightness = 0x40;

constexpr char ssid[] = "REPLACE_THIS_BY_YOUR_WIFI_SSID";
constexpr char pass[] = "REPLACE_THIS_BY_YOUR_WIFI_PASSWORD";

// you might want to replace this
static const char ntpServerName[] = "193.67.79.202";

WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

void setAllColor(uint8_t r, uint8_t g, uint8_t b) {
	uint16_t total = r + g + b;

	while (total > brightness * 3) {
		r = (r * 2.0 / 3.0);
		g = (g * 2.0 / 3.0);
		b = (b * 2.0 / 3.0);
		total = r + g + b;
	}

	for (uint8_t i = 0; i < NUMBER_OF_PIXELS; i++)
		pixels.setPixelColor(i, r, g, b);

	pixels.show();
}

void handleMqtt() {
	if (!psclient.connected()) {
		Serial.println(F("(Re-)connecting MQTT"));
		setAllColor(0, 0, brightness);

		psclient.connect("marsklok");
		psclient.subscribe(mqttTopic);

		setAllColor(0, brightness / 2, 0);
	}

	psclient.loop();
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address) {
	setAllColor(0, brightness / 2, 0);
	// set all bytes in the buffer to 0
	memset(packetBuffer, 0, NTP_PACKET_SIZE);

	// Initialize values needed to form NTP request
	// (see URL above for details on the packets)
	packetBuffer[0] = 0b11100011;   // LI, Version, Mode
	packetBuffer[1] = 0;     // Stratum, or type of clock
	packetBuffer[2] = 6;     // Polling Interval
	packetBuffer[3] = 0xEC;  // Peer Clock Precision
	// 8 bytes of zero for Root Delay & Root Dispersion
	packetBuffer[12] = 49;
	packetBuffer[13] = 0x4E;
	packetBuffer[14] = 49;
	packetBuffer[15] = 52;

	// all NTP fields have been given values, now
	// you can send a packet requesting a timestamp:
	Udp.beginPacket(address, 123); //NTP requests are to port 123
	Udp.write(packetBuffer, NTP_PACKET_SIZE);
	Udp.endPacket();
}

time_t getNtpTime() {
	IPAddress ntpServerIP; // NTP server's ip address

	while (Udp.parsePacket() > 0) {
	} // discard any previously received packets

	Serial.println("Transmit NTP Request");

	// get a random server from the pool
	WiFi.hostByName(ntpServerName, ntpServerIP);
	Serial.print(ntpServerName);
	Serial.print(": ");
	Serial.println(ntpServerIP);

	sendNTPpacket(ntpServerIP);

	uint32_t beginWait = millis();
	while (millis() - beginWait < 1500) {
		int size = Udp.parsePacket();
		if (size >= NTP_PACKET_SIZE) {
			Serial.println("Receive NTP Response");
			Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
			unsigned long secsSince1900;
			// convert four bytes starting at location 40 to a long integer
			secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
			secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
			secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
			secsSince1900 |= (unsigned long)packetBuffer[43];
			return secsSince1900 - 2208988800UL;
		}
	}

	Serial.println("No NTP Response :-(");

	return 0; // return 0 if unable to get the time
}

void setup() {
	Serial.begin(115200);
	Serial.println(F("Init MarsKlok " __DATE__ " " __TIME__));
	delay(250);

	pixels.begin();
	setAllColor(brightness, brightness, 0);

	Serial.println(ssid);
	Serial.println(pass);
	WiFi.persistent(false);
	WiFi.mode(WIFI_OFF);
	WiFi.mode(WIFI_STA);

	setAllColor(brightness / 2, 0, 0);

	for (;;) {
		int rc = WiFi.status();
		if (rc == WL_CONNECTED)
			break;
		delay(100);
		Serial.print('[');
		Serial.print(rc);
		Serial.print(']');
		if (rc != WL_DISCONNECTED)
			WiFi.begin(ssid, pass);
	}

	Serial.print("IP address assigned by DHCP is ");
	Serial.println(WiFi.localIP());

	setAllColor(brightness / 2, 0, brightness / 2);

	Serial.println("Starting UDP");
	Udp.begin(localPort);
	Serial.print("Local port: ");
	Serial.println(Udp.localPort());
	Serial.println("waiting for sync");
	setSyncProvider(getNtpTime);
	setSyncInterval(300);

	if (mqttServer[0]) {
		psclient.setServer(mqttServer, 1883);
		psclient.setCallback(callback);
	}

	setAllColor(0, 0, brightness / 2);

	Serial.println(F("Go!"));
}
void loop() {
	if (mqttServer[0])
		handleMqtt();

	// Difference between TAI and UTC. This value should be
	// updated each time the IERS announces a leap second.
	double tai_offset = 37;

	double jd_ut = 2440587.5 + (now() / 8.64E4);
	double jd_tt = jd_ut + (tai_offset + 32.184) / 86400;
	double j2000 = jd_tt - 2451545.0;
	double m = fmod(19.3870 + 0.52402075 * j2000, 360.0);
	double alpha_fms = fmod(270.3863 + 0.52403840 * j2000, 360.0);
	double e = 0.09340 + 2.477E-9 * j2000;
	double pbs =
		0.0071 * cos((0.985626 * j2000 /  2.2353) +  49.409) +
		0.0057 * cos((0.985626 * j2000 /  2.7543) + 168.173) +
		0.0039 * cos((0.985626 * j2000 /  1.1177) + 191.837) +
		0.0037 * cos((0.985626 * j2000 / 15.7866) +  21.736) +
		0.0021 * cos((0.985626 * j2000 /  2.1354) +  15.704) +
		0.0020 * cos((0.985626 * j2000 /  2.4694) +  95.528) +
		0.0018 * cos((0.985626 * j2000 / 32.8493) +  49.095);
	double nu_m = (10.691 + 3.0E-7 * j2000) * sin(m) +
		0.623 * sin(2 * m) +
		0.050 * sin(3 * m) +
		0.005 * sin(4 * m) +
		0.0005 * sin(5 * m) +
		pbs;
	double nu = nu_m + m;
	double l_s = fmod(alpha_fms + nu_m, 360.0);
	double eot = 2.861 * sin(2 * l_s) - 0.071 * sin(4 * l_s) + 0.002 * sin(6 * l_s) - nu_m;
	double eot_h = eot * 24 / 360;
	double msd = (((j2000 - 4.5) / 1.027491252) + 44796.0 - 0.00096);
	Serial.println(msd);
	double mtc = fmod(24.0 * msd, 24.0);

	byte hh = floor(mtc);

	double x = mtc * 3600.0;
	double y = fmod(x, 3600.0);
	byte mm = floor(y / 60.0);

	byte ss = round(fmod(y, 60.0));

	Serial.print(hh);
	Serial.print(' ');
	Serial.print(mm);
	Serial.print(' ');
	Serial.println(ss);

	setAllColor(0, 0, 0);

	if (powerState) {
		pixels.setPixelColor(hh * 60.0 / 23, 255, 0, 0);
		pixels.setPixelColor(mm, 0, 255, 0);
		pixels.setPixelColor(ss, 0, 0, 255);
		pixels.show();
	}

	delay(1000);
}
