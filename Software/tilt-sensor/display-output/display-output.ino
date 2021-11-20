// Global Positioning System Interface with OLED output

// #include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NeoSWSerial.h>

NeoSWSerial nss( 8,9 );

volatile uint32_t newlines = 0UL;


#define OLED_RESET 5

Adafruit_SSD1306 display(OLED_RESET);

int rpy[3];

const byte numBytes = 32;
byte receivedBytes[numBytes];
byte numReceived = 0;

boolean newData = false;


double dataRoll, dataPitch, dataYaw;

void setup(){

	// Serial.begin(9600);

	nss.begin(9600);
	// nss.attachInterrupt( recvBytesWithStartEndMarkers );

	Wire.setClock(400000);
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize the OLED and set the I2C address to 0x3C (for the 128x64 OLED)

	display.clearDisplay();
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(0,0);
	display.print("Inertial Mease. Unit");

	nss.print("connected\n");

	display.display();
	display.setTextSize(1);
}

// MAIN : LOO

void loop() {
	recvBytesWithStartEndMarkers();
	showNewData();
}


void renderSensorData(){
	display.clearDisplay();

	display.setCursor(0,50);

	display.print("r:");
	display.print(round(dataRoll));

	display.print("p:");
	display.print(round(dataPitch));

	display.print("y:");
	display.print(round(dataYaw));
	display.display();
	// delay(20);

}


void recvBytesWithStartEndMarkers() {
	static boolean recvInProgress = false;
	static byte ndx = 0;
	byte startMarker = 0x3C;
	byte endMarker = 0x3E;
	byte rb;


	while (nss.available() > 0 && newData == false) {
		rb = nss.read();

		if (recvInProgress == true) {
			if (rb != endMarker) {
				receivedBytes[ndx] = rb;
				ndx++;
				if (ndx >= numBytes) {
					ndx = numBytes - 1;
				}
			}
			else {
				receivedBytes[ndx] = '\0'; // terminate the string
				recvInProgress = false;
				numReceived = ndx;  // save the number for use when printing
				ndx = 0;
				newData = true;
			}
		}

		else if (rb == startMarker) {
			recvInProgress = true;
		}
	}
}

void showNewData() {
	if (newData == true) {

		byte data[4];
		byte data1[4];
		byte data2[4];

		data[0] = receivedBytes[0];
		data[1] = receivedBytes[1];
		data[2] = receivedBytes[2];
		data[3] = receivedBytes[3];
		// data[4] = receivedBytes[4];
		// data[5] = receivedBytes[5];
		// data[6] = receivedBytes[6];
		// data[7] = receivedBytes[7];



		data1[0] = receivedBytes[4];
		data1[1] = receivedBytes[5];
		data1[2] = receivedBytes[6];
		data1[3] = receivedBytes[7];
		// data[4] = receivedBytes[12];
		// data[5] = receivedBytes[13];
		// data[6] = receivedBytes[14];
		// data[7] = receivedBytes[15];


		data2[0] = receivedBytes[8];
		data2[1] = receivedBytes[9];
		data2[2] = receivedBytes[10];
		data2[3] = receivedBytes[11];
		// data[4] = receivedBytes[20];
		// data[5] = receivedBytes[21];
		// data[6] = receivedBytes[22];
		// data[7] = receivedBytes[23];


		dataRoll= *((double*)(data));
		dataPitch = *((double*)(data1));
		dataYaw = *((double*)(data2));

		Serial.print(dataYaw);
		nss.print("  : ");

		Serial.print(dataPitch);
		Serial.print("  : ");

		Serial.print(dataRoll );

		Serial.print("\n");

		renderSensorData();

		newData = false;
	}
}
