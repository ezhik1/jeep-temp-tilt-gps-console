// TEMP Sensor Interface with OLED output and graphing

// Gotchas:
// - display has visual artifacts at ~40% dynamic memory
// - arxduino becomes unstable at ~50% dynamic memory

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <Fonts/URW_Gothic_L_Book_20.h>
#include <Fonts/URW_Gothic_L_Book_5.h>

#define OLED_RESET 4
#define DHTPIN 2     // [ INPUT ][ DIGITAL ] pin for Temp Sensor
#define DHTTYPE DHT22 //  Temp Sensor Type: DHT 22 (AM2302)

DHT dht(DHTPIN, DHTTYPE);
Adafruit_SSD1306 display(OLED_RESET);

bool sensorFailedToReport = false;
bool pollingTick = true;

long previousMillisSensor = 0;
long previousMillisGraph = 0;

// DHT sensor read is SLOW, needs ~2 seconds to read reliably
const long sensorReadInterval PROGMEM = 2000; // [ MILLIS ] between sensor reads

// Graph Size
const int graphHeight PROGMEM = 26;
const int graphWidth PROGMEM = 44;
const int xOffset PROGMEM = 14;
const int yOffset PROGMEM = graphHeight + 8;

// Historical update
const long graphTimeline PROGMEM = 3.6e+6; // [ MILLIS ] of overall historical reads to display ( 1 hour )
const long graphUpdateInterval PROGMEM = graphTimeline / (( graphWidth / 2 ) - 1); // [ MILLIS ] between graph updates

// Bounds
const int topBoundHumidity PROGMEM = 100;
const int lowBoundHumidity PROGMEM = 0;

const int topBoundTemp PROGMEM = 40;
const int lowBoundTemp PROGMEM = -10;

// Temp Sensor Values
float measuredHumidity = 0;
float measuredTemperature = 0;
// float measuredHeatIndex = 0;

byte count;
byte temperatureSensorArray[64]; // double the bar graph resolution
byte humiditySensorArray[64];

const char graphFill PROGMEM = 'F'; //decide either filled or dot display (D=dot, any else filled)
const char drawDirection PROGMEM = 'R'; //decide drawing direction, from right or from left (L=from left to right, any else from right to left)
const char slope PROGMEM = 'W'; //slope colour of filled mode white or black slope (W=white, any else black. Well, white is blue in this dispay but you get the point)

const unsigned char humidityBitmap [] PROGMEM = {
	0x00, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x04, 0x80, 0x00,
	0x00, 0x0c, 0xc0, 0x00, 0x00, 0x08, 0x40, 0x00, 0x00, 0x10, 0x20, 0x00, 0x00, 0x10, 0x20, 0x00,
	0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x0c, 0x00,
	0x00, 0x80, 0x04, 0x00, 0x01, 0x80, 0x06, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x3c, 0x43, 0x00,
	0x02, 0x64, 0xc1, 0x00, 0x02, 0x64, 0x81, 0x00, 0x06, 0x3d, 0x81, 0x80, 0x06, 0x11, 0x00, 0x80,
	0x04, 0x02, 0x00, 0x80, 0x06, 0x06, 0xf1, 0x80, 0x02, 0x04, 0x99, 0x00, 0x02, 0x0c, 0x99, 0x00,
	0x03, 0x08, 0xf3, 0x00, 0x01, 0x00, 0x02, 0x00, 0x01, 0x80, 0x04, 0x00, 0x00, 0xc0, 0x0c, 0x00,
	0x00, 0x38, 0x70, 0x00, 0x00, 0x0f, 0xc0, 0x00
};

const unsigned char temperatureBitmap [] PROGMEM = {
	0x00, 0x00, 0x80, 0x00, 0x00, 0xc1, 0xe0, 0x00, 0x01, 0xc2, 0x20, 0x00, 0x00, 0xc2, 0x20, 0x00,
	0x00, 0x7a, 0xb0, 0x00, 0x00, 0x6a, 0xb0, 0x00, 0x00, 0x42, 0xb0, 0x00, 0x00, 0x42, 0xb0, 0x00,
	0x00, 0x3a, 0xb8, 0x00, 0x00, 0x02, 0xb0, 0x00, 0x00, 0x02, 0xb0, 0x00, 0x00, 0x02, 0xa0, 0x00,
	0x00, 0x02, 0xb8, 0x00, 0x00, 0x02, 0xb0, 0x00, 0x00, 0x02, 0xb0, 0x00, 0x00, 0x02, 0xb0, 0x00,
	0x00, 0x02, 0xb0, 0x00, 0x00, 0x02, 0xb8, 0x00, 0x00, 0x02, 0xa0, 0x00, 0x00, 0x06, 0x98, 0x00,
	0x00, 0x08, 0x8c, 0x00, 0x00, 0x11, 0xc4, 0x00, 0x00, 0x12, 0x24, 0x00, 0x00, 0x12, 0x32, 0x00,
	0x00, 0x12, 0x26, 0x00, 0x00, 0x13, 0xe4, 0x00, 0x00, 0x18, 0x04, 0x00, 0x00, 0x0c, 0x08, 0x00,
	0x00, 0x07, 0xf0, 0x00, 0x00, 0x01, 0xc0, 0x00
};

void setup()
{
	// Serial.begin(9600);

	// Bring Temp Sensor Online
	dht.begin();
	// Serial.println("Temp Sensor Initializing...");

	for (count = 0; count <= 64; count++)
	{
		temperatureSensorArray[count] = 0;
		humiditySensorArray[count] = 0;
	}

	// Bring OLED Online
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C
	display.clearDisplay();

	display.setTextColor(WHITE);
	// Header
	display.setCursor(40,0);
	display.setTextSize(1);
	display.print("Temperature\n");

	display.setCursor(66,14);
	display.print("and\n");

	display.setCursor(50,28);
	display.print("Humidity");

	display.drawBitmap(0, 0,humidityBitmap,30,30, WHITE);
	display.drawBitmap(0, 35,temperatureBitmap,30,30, WHITE);

	display.display();
	display.setTextSize(1);
	display.setCursor(30,51);
	display.print("..Initializing..");

	display.display();
	delay(1500);
}

void loop()
{

	unsigned long currentMillis = millis();
	bool advanceGraph = false;

	if( previousMillisSensor == 0 || currentMillis - previousMillisSensor > sensorReadInterval) {
		previousMillisSensor = currentMillis;

		readFromSensor();
	}

	if(previousMillisGraph == 0 || currentMillis - previousMillisGraph > graphUpdateInterval) {
		previousMillisGraph = currentMillis;

		if( !sensorFailedToReport ){
			advanceGraph = true;
		}
	}


	// clear buffer
	display.fillScreen(BLACK);
	display.clearDisplay();

	renderNumericOutput( xOffset, 0, 'temp' );
	renderNumericOutput( xOffset, yOffset, 'humid' );

	// Temp Render
	renderGraph(advanceGraph, temperatureSensorArray, xOffset, 0);
	renderAxes( xOffset, 0,  topBoundTemp, lowBoundTemp, 'temp' );

	// Humidity Render
	renderGraph(advanceGraph, humiditySensorArray, xOffset, yOffset );
	renderAxes( xOffset, yOffset, topBoundHumidity, lowBoundHumidity, 'humid' );

	if( sensorFailedToReport && pollingTick ){
		display.fillRoundRect(20,25,94,20,3,WHITE);
		display.setCursor(22,32);

		display.setTextColor(BLACK,WHITE);
		display.print("Sensor Offline!");
		display.setTextColor(WHITE,BLACK);
		advanceGraph = false;
	}
	pollingTick = !pollingTick;
	display.display();
}

void readFromSensor(){
	float humidity = dht.readHumidity();
	float temperature = dht.readTemperature();

	// Bail  early if sensor fails to report proper reading
	if (isnan(temperature) || isnan(humidity)){
		sensorFailedToReport = true;
		return;
	}else{
		sensorFailedToReport = false;
	}

	// Humidity Reading
	measuredHumidity = humidity;
	humiditySensorArray[0] = map( humidity, lowBoundHumidity, topBoundHumidity, 0, graphHeight );

	// Temperature Reading
	measuredTemperature = temperature;
	int constrained = constrain(measuredTemperature, lowBoundTemp, 80);
	temperatureSensorArray[0] = map(constrained, lowBoundTemp, topBoundTemp, 0, graphHeight );

	// Compute heat index
	// measuredHeatIndex = dht.computeHeatIndex(measuredTemperature, measuredHumidity, false);
	// DEBUG
	// int pot = analogRead(A0);
	// measuredTemperature = (float) map(pot, 0, 1023, -40, 80 );
}

void renderGraph( bool advanceGraph, byte sensorArray[], int xOffset, int yOffset ){

	for (int step = 2; step < graphWidth; step+=2 ){
		int position = yOffset + graphHeight - sensorArray[step - 1];

		if( position != graphHeight + yOffset ){
			if (graphFill == 'D' || graphFill == 'd'){
				if (drawDirection == 'L' || drawDirection == 'l'){
					display.drawPixel(step + xOffset, position, WHITE);
				}else{
					// draw dots from right to left
					display.drawPixel(graphWidth - step + xOffset, position, WHITE);
				}
			}else{
				if (drawDirection == 'L' || drawDirection == 'l'){
					if (slope == 'W' || slope == 'w'){
						display.drawLine(step + xOffset, graphHeight + yOffset - 1, step + xOffset, position, WHITE);
					}else{
						display.drawLine(step + xOffset, 1, step + xOffset, position, WHITE);
					}
				}else{
					if (slope == 'W' || slope == 'w'){
						display.drawLine(graphWidth - step + xOffset, graphHeight  + yOffset - 1, graphWidth - step + xOffset, position, WHITE);
					}else{
						display.drawLine(graphWidth - step + xOffset, yOffset + 1, graphWidth - step + xOffset, position, WHITE);
					}
				}
			}
		}
	}
	if( advanceGraph ){
		// advanced historical values
		for (int step2 = graphWidth; step2 >= 2; step2--){
			sensorArray[step2 - 1] = sensorArray[step2 - 2];
		}
	}
}

void renderAxes( int xOffset, int yOffset, int topBound, int lowBound, char *type )
{

	//bounding vert line
	display.drawLine( xOffset, yOffset,  xOffset, graphHeight + yOffset, WHITE);
	display.drawLine(graphWidth + xOffset, yOffset, graphWidth + xOffset, graphHeight + yOffset, WHITE);

	// scale marks
	for (count = 0; count <= graphHeight; count += (graphHeight)){
		int position = graphHeight - count;
		int scaleMark = map( graphHeight - position, 0, graphHeight, lowBound, topBound);
		int scaleOffsetStart = 0;

		// format negative values differently
		if( scaleMark < 0 ){
			scaleMark = abs(scaleMark);
			scaleOffsetStart = 3;
			display.drawPixel(count , yOffset + graphHeight, WHITE);
			display.drawPixel(count+1 , yOffset + graphHeight, WHITE);
		}
		// tuck lower value of y-axis into graph
		if( count == 0 ){
			position -= 2;
		}

		if( type == 'temp' && count == graphHeight ){

			scaleOffsetStart += 3;
		}

		if( type == 'humid' && count == 0 ){

			scaleOffsetStart += 6;
		}

		display.setFont(&URW_Gothic_L_Book_5);
		display.setCursor(scaleOffsetStart, yOffset + position + 5);
		display.setTextSize(1);
		display.setTextColor(WHITE);
		display.print(scaleMark);
		display.setFont(NULL);

		display.drawLine(graphWidth + xOffset, count + yOffset, graphWidth - (graphWidth/16) + xOffset, count + yOffset, WHITE);
		display.drawLine( xOffset, count + yOffset, graphWidth/16 + xOffset, count + yOffset, WHITE);
	}

	for (count = 10 + xOffset; count < graphWidth + xOffset; count += 10){
		display.drawPixel(count, yOffset , WHITE);
		display.drawPixel(count, yOffset + graphHeight + 2 , WHITE);
	}
}

void renderNumericOutput( int xOffset, int yOffset, char *type ){

	// Text Spacing
	const int textPad PROGMEM = 4; // horizontal space between numerical output and graph
	const int newLinePad PROGMEM = 12; // vertical between each numerical output group
	const int numberPad PROGMEM = 8;  // additional vertical pad for font change offset
	int negativeMarkOffset =  (yOffset + newLinePad + numberPad - 4 ); // vertical position of the sign for numeric output
	int negativeMarkOffsetX = 0; // account for proper alignment when theres less than 4 digits shown
	const int unitOffset PROGMEM = 60; // units should be fixed position, right-aligned
	int sigFigPadTemp = 0; // account for proper alignment when theres less than 4 digits shown
	int sigFigPadHumidity = 0; // account for proper alignment when theres less than 4 digits shown

	//  Temperature
	if( type == 'temp'){

		int displayTemp = abs(measuredTemperature);

		// TEMP Headline
		display.setCursor(xOffset + graphWidth + textPad, 0);
		display.setTextSize(1);
		display.print("Temperature");

		// Numeric Output TEMP


		// empty character pad
		if( displayTemp < 10.0 ){
			sigFigPadTemp = 24;
			negativeMarkOffsetX = 6;
		}else{
			sigFigPadTemp = 12;
		}
		display.setCursor(xOffset + graphWidth + textPad + sigFigPadTemp -2, yOffset + newLinePad + numberPad);

		// actual negative sign takes up too much space
		if( measuredTemperature < 0 ){
			display.drawPixel(xOffset + graphWidth + ( textPad *3 ) + negativeMarkOffsetX, negativeMarkOffset, WHITE);
			display.drawPixel(xOffset + graphWidth + ( textPad *3 ) + negativeMarkOffsetX+1, negativeMarkOffset, WHITE);
		}

		display.setFont(&URW_Gothic_L_Book_20);
		display.setTextSize(1);
		display.print(abs(measuredTemperature), 1);

		// UNIT
		display.setFont(NULL);
		display.setTextSize(1);
		display.setCursor(xOffset + graphWidth + textPad + unitOffset - 6, yOffset + newLinePad + numberPad);
		display.write(0xf7);
		display.print("C");

	}else{

		if( measuredHumidity < 100.0 ){
			sigFigPadHumidity = 12;
		}

		// Humidity Headline
		display.setTextSize(1);
		display.setCursor(xOffset + graphWidth + textPad, yOffset);
		display.print("Humidity");

		// Numeric Output Humidity
		display.setFont(&URW_Gothic_L_Book_20);
		display.setTextSize(1);
		display.setCursor(xOffset + graphWidth + textPad + sigFigPadHumidity -2, yOffset + newLinePad + numberPad + 6);
		display.print(measuredHumidity, 1);

		// UNIT
		display.setFont(NULL);
		display.setCursor(xOffset + graphWidth + textPad + unitOffset , yOffset + newLinePad + numberPad);
		display.setTextSize(1);
		display.write(0x25);
	}
}
