// TEMP Sensor Interface with OLED output and graphing

// Gotchas:
// - IMPORTANT : Adafruit DHT Library 1.3.0 is the last stable library for Chinese DHT22 clones. Anything newer has, in practice, failed to report
// - display has visual artifacts at ~40% dynamic memory
// - arduino becomes unstable at ~50% dynamic memory

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include "URW_Gothic_L_Book_20.h"
#include "URW_Gothic_L_Book_5.h"

#define OLED_RESET 4
#define DHTPIN 2     // [ INPUT ][ DIGITAL ] pin for Temp Sensor
#define DHTTYPE DHT22 //  Temp Sensor Type: DHT 22 (AM2302)
#define DISPLAY_WIDTH 128 // [ PIXELS ] number of available horizontal pixels
#define DISPLAY_HEIGHT 64 // [ PIXELS ] number of available vertical pixels
#define LAST_LINE_INDENT 10

#define ALERT_BLINK_INTERVAL 500 // [ MILLIS ] between alert banners

DHT dht( DHTPIN, DHTTYPE );
Adafruit_SSD1306 display( DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, OLED_RESET );

byte lastLine = 0;
bool sensorFailedToReport = false;
bool pollingTick = true;
bool advanceGraph = false;

unsigned long previousMillisSensor = 0;
unsigned long previousMillisGraph = 0;
unsigned long previousMillisBlinkTick = 0;

// DHT sensor read is SLOW, needs ~2 seconds to read reliably
#define SENSOR_READ_INTERVAL 2000 // [ MILLIS ] between sensor reads

// Graph Size
#define GRAPH_HEIGHT 26
#define GRAPH_WIDTH 44
#define X_OFFSET 14
#define Y_OFFSET GRAPH_HEIGHT + 8

// Historical update
#define GRAPH_DATUM_COUNT 43
#define GRAPH_TIMELINE 3.6e+6 // [ MILLIS ] of overall historical reads to display ( 1 hour )
#define GRAPH_UPDATE_INTERVAL GRAPH_TIMELINE / (( GRAPH_WIDTH / 2 ) - 1) // [ MILLIS ] between graph updates

// Bounds
#define TOP_BOUND_HUMIDITY 100
#define LOW_BOUND_HUMIDITY 0

#define TOP_BOUND_TEMP 40
#define LOW_BOUND_TEMP -10

// Temp Sensor Values
float measuredHumidity = 0;
float measuredTemperature = 0;
// float measuredHeatIndex = 0;

byte temperatureSensorArray[ GRAPH_DATUM_COUNT ]; // double the bar graph resolution
byte humiditySensorArray[ GRAPH_DATUM_COUNT ];

void setup(){

	Wire.begin();
	Wire.setClock( 400000L ); // 400kHz I2C clock
	Wire.setWireTimeout( 4000, true );

	display.begin( SSD1306_SWITCHCAPVCC, 0x3C );
	wipeDisplay();

	display.setTextColor( WHITE );
	slowType( F("TEMP SENSE START >"), 10, true );

	lastLine += LAST_LINE_INDENT;

	slowType( F("SENSOR REPORTS: "), 10, true );
	dht.begin();
	delay( SENSOR_READ_INTERVAL );
	slowPrintSuccessOrFail(  dht.read()  );

	slowType( F("CONFIG DEVICE:  "), 10, true );
	clearHistoricalValues();
	slowPrintSuccessOrFail( true );

	display.setFont(&URW_Gothic_L_Book_20);
	display.setCursor( 22, 62 );
	display.print(F("ONLINE!"));
	display.display();

	delay( 3000 );
	wipeDisplay();
}

void loop(){

	advanceAnimationTicks();

	display.clearDisplay();

	renderNumericOutput( X_OFFSET, 0, 'temp' );
	renderNumericOutput( X_OFFSET, Y_OFFSET, 'humid' );

	// Temp Render
	renderGraph( advanceGraph, temperatureSensorArray, X_OFFSET, 0 );
	renderAxes( X_OFFSET, 0,  TOP_BOUND_TEMP, LOW_BOUND_TEMP, 'temp' );

	// Humidity Render
	renderGraph( advanceGraph, humiditySensorArray, X_OFFSET, Y_OFFSET );
	renderAxes( X_OFFSET, Y_OFFSET, TOP_BOUND_HUMIDITY, LOW_BOUND_HUMIDITY, 'humid' );

	if( sensorFailedToReport && pollingTick ){

		renderSensorErrorAlert();
		advanceGraph = false;
	}
	display.display();
}

void clearHistoricalValues(){

	for( byte count = 0; count < GRAPH_DATUM_COUNT; count++ ){

		temperatureSensorArray[ count ] = 0;
		humiditySensorArray[ count ] = 0;
	}
}

void renderSensorErrorAlert(){

	display.fillRoundRect( 20, 25, 94, 20, 3, WHITE );
	display.setCursor( 22, 32 );

	display.setTextColor( BLACK, WHITE );
	display.print( F("Sensor Offline!") );
	display.setTextColor( WHITE, BLACK );
}

void advanceAnimationTicks(){

	unsigned long currentMillis = millis();
	advanceGraph = false;

	bool alertBlinkTick = ( previousMillisBlinkTick == 0 || currentMillis - previousMillisBlinkTick > ALERT_BLINK_INTERVAL );

	if( previousMillisSensor == 0 || currentMillis - previousMillisSensor > SENSOR_READ_INTERVAL){

		previousMillisSensor = currentMillis;
		readFromSensor();
	}

	if( previousMillisGraph == 0 || currentMillis - previousMillisGraph > GRAPH_UPDATE_INTERVAL ){

		previousMillisGraph = currentMillis;

		if( !sensorFailedToReport ){

			advanceGraph = true;
		}
	}

	if( alertBlinkTick ){

		previousMillisBlinkTick = currentMillis;
		pollingTick = !pollingTick;
	}
}

void readFromSensor(){

	measuredHumidity = dht.readHumidity();
	measuredTemperature = dht.readTemperature();

	// Bail  early if sensor fails to report proper reading
	if( isnan( measuredTemperature ) || isnan( measuredHumidity )){

		sensorFailedToReport = true;
		measuredTemperature = isnan( measuredTemperature ) ? 0.0 : measuredTemperature;
		measuredHumidity = isnan( measuredHumidity ) ? 0.0 : measuredHumidity;
		return;
	}else{

		sensorFailedToReport = false;
	}

	// Humidity Reading
	humiditySensorArray[ 0 ] = map( measuredHumidity, LOW_BOUND_HUMIDITY, TOP_BOUND_HUMIDITY, 0, GRAPH_HEIGHT );

	// Temperature Reading
	int constrained = constrain( measuredTemperature, LOW_BOUND_TEMP, 80);
	temperatureSensorArray[ 0 ] = map( constrained, LOW_BOUND_TEMP, TOP_BOUND_TEMP, 0, GRAPH_HEIGHT );

	// Compute heat index
	// measuredHeatIndex = dht.computeHeatIndex(measuredTemperature, measuredHumidity, false);
	// DEBUG
	// int pot = analogRead(A0);
	// measuredTemperature = (float) map(pot, 0, 1023, -40, 80 );
}

void renderGraph( bool advanceGraph, byte sensorArray[], byte xOffset, byte yOffset ){

	for( byte step = 2; step < GRAPH_DATUM_COUNT; step+=2 ){

		byte position = yOffset + GRAPH_HEIGHT - sensorArray[ step - 1 ];

		if( position != GRAPH_HEIGHT + yOffset ){

			display.drawLine(GRAPH_WIDTH - step + xOffset, GRAPH_HEIGHT  + yOffset - 1, GRAPH_WIDTH - step + xOffset, position, WHITE);
		}
	}
	if( advanceGraph ){
		// advanced historical values
		for( byte step2 = GRAPH_DATUM_COUNT; step2 >= 2; step2--){

			sensorArray[ step2 - 1 ] = sensorArray[ step2 - 2 ];
		}
	}
}

void renderAxes( byte xOffset, byte yOffset, byte topBound, int lowBound, char *type ){

	//bounding vert line
	display.drawLine( xOffset, yOffset,  xOffset, GRAPH_HEIGHT + yOffset, WHITE);
	display.drawLine(GRAPH_WIDTH + xOffset, yOffset, GRAPH_WIDTH + xOffset, GRAPH_HEIGHT + yOffset, WHITE);

	// scale marks
	for( byte count = 0; count <= GRAPH_HEIGHT; count += GRAPH_HEIGHT ){

		byte position = GRAPH_HEIGHT - count;
		int scaleMark = map( GRAPH_HEIGHT - position, 0, GRAPH_HEIGHT, lowBound, topBound);
		int scaleOffsetStart = 0;

		// format negative values differently
		if( scaleMark < 0 ){
			scaleMark = abs(scaleMark);
			scaleOffsetStart = 3;
			display.drawPixel(count , yOffset + GRAPH_HEIGHT, WHITE);
			display.drawPixel(count+1 , yOffset + GRAPH_HEIGHT, WHITE);
		}
		// tuck lower value of y-axis into graph
		if( count == 0 ){

			position -= 2;
		}

		if( type == 'temp' && count == GRAPH_HEIGHT ){

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

		display.drawLine(GRAPH_WIDTH + xOffset, count + yOffset, GRAPH_WIDTH - (GRAPH_WIDTH/16) + xOffset, count + yOffset, WHITE);
		display.drawLine( xOffset, count + yOffset, GRAPH_WIDTH/16 + xOffset, count + yOffset, WHITE);
	}

	for( byte count = 10 + xOffset; count < GRAPH_WIDTH + xOffset; count += 10 ){

		display.drawPixel(count, yOffset , WHITE);
		display.drawPixel(count, yOffset + GRAPH_HEIGHT + 2 , WHITE);
	}
}

void renderNumericOutput( int xOffset, int yOffset, char *type ){

	// Text Spacing
	const int textPad PROGMEM = 4; // horizontal space between numerical output and graph
	const int newLinePad PROGMEM = 12; // vertical between each numerical output group
	const int numberPad PROGMEM = 10;  // additional vertical pad for font change offset
	int negativeMarkOffset =  (yOffset + newLinePad + numberPad - 4 ); // vertical position of the sign for numeric output
	int negativeMarkOffsetX = 0; // account for proper alignment when theres less than 4 digits shown
	const int unitOffset PROGMEM = 60; // units should be fixed position, right-aligned
	int sigFigPadTemp = 0; // account for proper alignment when theres less than 4 digits shown
	int sigFigPadHumidity = 0; // account for proper alignment when theres less than 4 digits shown

	//  Temperature
	if( type == 'temp'){

		int displayTemp = abs(measuredTemperature);

		// TEMP Headline
		display.setCursor(xOffset + GRAPH_WIDTH + textPad, 0);
		display.setTextSize(1);
		display.print(F("Temperature"));

		// Numeric Output TEMP

		// empty character pad
		if( displayTemp < 10.0 ){
			sigFigPadTemp = 24;
			negativeMarkOffsetX = 6;
		}else{
			sigFigPadTemp = 12;
		}
		display.setCursor(xOffset + GRAPH_WIDTH + textPad + sigFigPadTemp -2, yOffset + newLinePad + numberPad);

		// actual negative sign takes up too much space
		if( measuredTemperature < 0 ){
			display.drawPixel(xOffset + GRAPH_WIDTH + ( textPad *3 ) + negativeMarkOffsetX, negativeMarkOffset, WHITE);
			display.drawPixel(xOffset + GRAPH_WIDTH + ( textPad *3 ) + negativeMarkOffsetX+1, negativeMarkOffset, WHITE);
		}

		display.setFont(&URW_Gothic_L_Book_20);
		display.setTextSize(1);
		display.print(abs(measuredTemperature), 1);

		// UNIT
		display.setFont(NULL);
		display.setTextSize(1);
		display.setCursor(xOffset + GRAPH_WIDTH + textPad + unitOffset - 6, yOffset + newLinePad + numberPad);
		display.write(0xf7);
		display.print(F("C"));

	}else{

		if(measuredHumidity < 10.0 ){

			sigFigPadHumidity = 24;
		}else if( measuredHumidity < 100.0 ){
			
			sigFigPadHumidity = 12;
		}

		// Humidity Headline
		display.setTextSize(1);
		display.setCursor(xOffset + GRAPH_WIDTH + textPad, yOffset);
		display.print(F("Humidity"));

		// Numeric Output Humidity
		display.setFont(&URW_Gothic_L_Book_20);
		display.setTextSize(1);
		display.setCursor(xOffset + GRAPH_WIDTH + textPad + sigFigPadHumidity -2, yOffset + newLinePad + numberPad + 6);
		display.print(measuredHumidity, 1);

		// UNIT
		display.setFont(NULL);
		display.setCursor(xOffset + GRAPH_WIDTH + textPad + unitOffset , yOffset + newLinePad + numberPad);
		display.setTextSize(1);
		display.write(0x25);
	}
}

void slowPrintSuccessOrFail( bool condition ){

	condition ? slowType( F("OK"), 50, false ) : slowType( F("FAIL"), 50, false );
}

void slowType( String text, int delayTime, bool newLine ){

	if( newLine ){

		display.setCursor(0, lastLine );
		lastLine += LAST_LINE_INDENT;
	}

	for( int i = 0; i < text.length(); i++ ){

		display.print( text[ i ] );
		display.display();
		delay( delayTime );
	}
}

void wipeDisplay(){

	display.clearDisplay(); // remove library banner
	display.fillScreen(BLACK); // I see a red door...
	display.display(); // because fillScreen is misleading
}
