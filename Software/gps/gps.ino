// Global Positioning System Receivng Interface with OLED output
// - Immediate reporting of the following:
//		- Latitude, Longitude
// 		- Time (localized via constant)
//		- Heading, Speed, Altitude
//		- Miscellaneous Satellite details ( Signal, Elevation, Azimuth )
// - Historical Altitude Tracking

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FlashStorage_SAMD.h>
#include <Timezone.h> // http://github.com/JChristensen/Timezone
#include <ezButton.h>
#include <TinyGPSPlus.h>
#include "Lato_Thin_12.h"
#include "Lato_Thin_30.h"

// #define DEBUG 1

#define M_PI_LONG 3.141592653589793238462643383279502884L // [ LONG ] slice of pizza
#define EASE_COEFFICIENT 0.1 // [ FLOAT ] : coefficient used for lerp transforms

// Display Characteristics
#define DISPLAY_WIDTH 128 // [ PIXELS ] number of available horizontal pixels
#define DISPLAY_HEIGHT 64 // [ PIXELS ] number of available vertical pixels
#define SLOW_TYPE_NEXT_LINE_PADDING 10 // [ PIXELS ] of padding below any given text animated to UI via "slow typing" effect
#define OLED_RESET -1 // [ NUMBER ] digital read pin to reset the display | NOT IMPLEMENTED
#define ENTER_BUTTON_PIN 0 // [ NUMBER ] digital read pin for button inputs

// UI configs
#define DEFAULT_SCREEN 0 // [ NUMBER ] : index of the startup UI layout
#define SCREEN_COUNT 7 // [ NUMBER ] of UI layouts available
#define NUMBER_OF_CARDINAL_DIRECTIONS 16 // [ NUMBER ] of cardinal labels available for readable heading output
#define MAX_SATELLITES 6 // [ NUMBER ] of satellites tracked on the following metrics: ( Index, Elevation, Azimuth, Signal Strength )
#define NUMBER_OF_CONFIGURABLE_UNITS 2 // [ NUMBER ] of display units that can be configured for GPS data ie. distance, speed, etc.
#define LEAVE_EDIT_MODE_TIME 5000 // [ MILLISECONDS ] before disabling editMode due to inactivity
#define UNIT_EDIT_BLINKER_INTERVAL 2000 // [ MILLISECONDS ] between edit mode indicator blinks

#define GRAPH_WIDTH 100 // [ PIXELS ] of the historical chart's width
#define GRAPH_HEIGHT 44 // [ PIXELS ] of the historical chart's height
#define HISTORICAL_DATUM_COUNT 50 // [ NUMBER ] of data points tracked
#define ALTITUDE_HISTORY_INTERMEDIATE_SCALE 4 // [ NUMBER ] of intermediate scale magnitudes to show in the historical plot

// Altitude Tracking
#define MAX_READINGS_LOST_BEFORE_FAIL 3 // [ NUMBER ] : of GPS_READ_INTERVALs to wait before deeming a lost satellite fix; ie. no serial data from the device
#define ALTITUDE_OUT_OF_BOUND -9999.0F // [ FLOAT ] : control value for invalid altitude
#define MIN_ALTITUDE_TRACKED -100.0F // [ METERS ] minimum value plotted in history
#define MAX_ALTITUDE_TRACKED 4000.0F // [ METERS ] maximum value plotted in history

#define MIN_SPEED_TRACKED 0.0F // [ M/S ] minimum value plotted in history
#define MAX_SPEED_TRACKED 44.704F // [ M/S ] maximum value plotted in history

// Time
#define TIME_ZONE -8.0 // Pacific
TimeChangeRule PDT = { "PDT", Second, Sun, Mar, 2, -420 };    //Daylight time = UTC - 7 hours
TimeChangeRule PST = { "PST", First, Sun, Nov, 1, -480 };     //Standard time = UTC - 8 hours
Timezone Pacific(PDT, PST);

#define GRAPH_TIMELINE 3.6e+6 // [ MILLIS ] of overall historical values to display ( 1 hour )

// Update/Expire Intervals
#define GRAPH_UPDATE_INTERVAL GRAPH_TIMELINE / (( GRAPH_WIDTH / 2 ) - 1) // [ MILLIS ] between graph updates
#define GPS_READ_INTERVAL 1000 // [ MILLIS ] between serial data updates from the GPS module. This is a prediction, meaning we expect the device to report at this rate.
#define SAT_SEARCH_ANIMATION_INTERVAL 500 // [ MILLIS ]
#define SAT_DETAIL_INFO_MAX_AGE 5000 // [ MILLIS ] before tracked satellite details ( Index, Elevation, Azimuth, Signal Strength ) are cleared
#define SPEED_AVERAGE_POLLING_INTERVAL GPS_READ_INTERVAL // [ MILLIS ] between raw speed reads from GPS module

// Inferences
const byte maxNumberOfReadsBetweenHistoricalPlots = (byte) ceil( ( (float) GRAPH_TIMELINE / (float) HISTORICAL_DATUM_COUNT ) / (float) SPEED_AVERAGE_POLLING_INTERVAL );

Adafruit_SSD1306 display( DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, OLED_RESET );
ezButton enterButton( ENTER_BUTTON_PIN );
TinyGPSPlus gps;
TinyGPSCustom totalGPGSVMessages (gps, "GPGSV", 1 );
TinyGPSCustom messageNumber( gps, "GPGSV", 2 );
TinyGPSCustom satsInView( gps, "GPGSV", 3 );
TinyGPSCustom satNumber[ 4 ];
TinyGPSCustom elevation[ 4 ];
TinyGPSCustom azimuth[ 4 ];
TinyGPSCustom snr[ 4 ];

struct{
	bool active;
	int elevation;
	int azimuth;
	int snr;
} sats[ MAX_SATELLITES ];

struct bounds{
	bool isInteger;
	char unit[ 2 ];
	short denominator;
} graphBounds[ 2 ];

time_t utc, local;
int lastKnownYear;
byte lastKnownDay, lastKnownMonth, lastKnownHour, lastKnownMinute, lastKnownSecond;
static const char* directions[ NUMBER_OF_CARDINAL_DIRECTIONS ] = {
	"N ", "NNE", "NE", "ENE", "E ", "ESE", "SE", "SSE", "S ", "SSW", "SW", "WSW", "W ", "WNW", "NW", "NNW"
};

bool firstLoad = true;
bool updateTick = false;
bool advanceGraph = true;
bool sensorFailedToReport = false;
bool validSatelliteWindow = false;
bool isValidSatelliteDetailsWindow = false;
bool editMode = false;
bool editingUnit = false;
bool pollingTick = false;
bool speedTick = false;

byte unitToEdit = 0; // [number] from 0 - (configuredUnits.length - 1 ) which unit is being edited

// Button Characteristics
const int SHORT_PRESS_TIME = 1000; // 1000 milliseconds
const int LONG_PRESS_TIME = 1000; // 1000 milliseconds
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;
bool isPressing = false;
bool isLongDetected = false;
unsigned long editModeTime = 0;
unsigned long previousMillisBlinkTick = 0;

// Units
typedef struct Unit {
	char quantity[10]; // name of unit
	char types[10][10];
	char symbolTypes[10][10];
	char symbols[10][10]; // display symbol
	float coefficient[10]; // conversion coefficient from SI to desired unit
	byte numberOfSelectableTypes;
	byte memoryAddress;
	byte selectedUnit;
};

Unit distance = {
	"DISTANCE",
	{ "metric", "imperial" },
	{ "metric", "metric", "imperial", "imperial" },
	{ "M", "KM", "FT", "MI" },
	{ 1.0f, 0.001f, 3.28f, 0.000621371f  },
	2,
	1,
	0
};

Unit speed = {
	"SPEED",
	{ "metric", "imperial" },
	{ "metric", "metric", "imperial" },
	{ "M/S", "KM/H", "FT/S", "MPH" },
	{ 1.0f, 3.6f, 3.28084f, 2.23694f },
	2,
	2,
	0
};

Unit configuredUnits[] {
	distance,
	speed,
};

unsigned long testEpoch;
time_t lastTimeTicked = 0;
float currentLatitude = 0;
float currentLongitude = 0;
double currentHeading = 0;
float currentAltitude = ALTITUDE_OUT_OF_BOUND; // [ METERS ]
float currentSpeed = 0; // [ METERS/S ]
float averageSpeed = 0; // [ METERS/S ]
double smoothedHeadingAngle = 0;
double smoothedHeadingDigit = 0;
byte lastLine = 0;
byte numberOfFixedSatellites = 0;

float readings [ maxNumberOfReadsBetweenHistoricalPlots * 2 ];
int readIndex  = 0;
float total  = 0.0;
float average = 0.0;

short altitudeHistory[ HISTORICAL_DATUM_COUNT ];
short speedHistory[ HISTORICAL_DATUM_COUNT ];
unsigned long previousMillisGraph = 0;
unsigned long  previousMillisSatSearch = 0;
unsigned long  millisLastValidSignal = 0;
unsigned long  millisLastValidsatelliteInfo = 0;

const unsigned char bitmap [] PROGMEM = {
	0x00, 0x40, 0x00, 0x00, 0xa0, 0x00, 0x01, 0x16, 0x00, 0x02, 0x09, 0x00, 0x01, 0x10, 0x80, 0x00,
	0xa0, 0x40, 0x00, 0x40, 0x40, 0x00, 0x80, 0x80, 0x01, 0xc1, 0x40, 0x01, 0x22, 0x20, 0x00, 0x94,
	0x10, 0x00, 0xda, 0x20, 0x00, 0x31, 0x40, 0x00, 0x00, 0x80, 0x90, 0x00, 0x00, 0x90, 0x00, 0x00,
	0x8c, 0x00, 0x00, 0x40, 0x00, 0x00, 0x20, 0x00, 0x00, 0x1c, 0x00, 0x00
};

byte storedScreen = EEPROM.read( 0 );
byte screen = ( storedScreen != 255 ) ? storedScreen : DEFAULT_SCREEN;
byte lastScreen = screen;

byte storedDistanceUnit = EEPROM.read( configuredUnits[ 0 ].memoryAddress );
byte storedSpeedeUnit = EEPROM.read( configuredUnits[ 1 ].memoryAddress );

void setup(){

	#ifdef DEBUG
	Serial.begin(9600);
	while (!Serial);
	#endif

	Wire.begin();
	Wire.setClock( 400000 );
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize the OLED and set the I2C address to 0x3C (for the 128x64 OLED)
	wipeDisplay();

	display.setTextSize(1);
	display.setFont();
	display.setTextColor(WHITE);

	slowType( F("GPS >"), 10, true );
	slowType( F("Module Online: "), 10, true );
	Serial1.begin( 9600 );
	slowPrintSuccessOrFail( true );

	slowType( F("Setting Up Device: "), 10, true );

	enterButton.setDebounceTime( 50 );
	setTime(0, 0, 0, 0, 0, 0);
	clearAltitudeHistory();
	clearSpeedHistory();
	initCustomGPSObjects();

	slowPrintSuccessOrFail( true );

	configuredUnits[ 0 ].selectedUnit = ( storedDistanceUnit != 255 ) ? storedDistanceUnit : 0;
	configuredUnits[ 1 ].selectedUnit = ( storedSpeedeUnit != 255 ) ? storedSpeedeUnit : 0;

	lastLine += SLOW_TYPE_NEXT_LINE_PADDING;
	slowType( F("Online!"), 10, true );
	delay(2000);
	wipeDisplay();
	splashScreen();

	#ifdef DEBUG
	Serial.print("Coming Online\n");
	#endif
}

void loop() {

	listenToButtonPushes();
	#ifdef DEBUG
	bool hasData = true;
	#else
	bool hasData = isGPSDataAvailable() && gps.satellites.value() > 0;
	#endif

	if( hasData ){

		populateGPSData();
		updateTimeFromGPS();

		// update once for splash screen on number of satellites
		if( !validSatelliteWindow ){

			validSatelliteWindow = true;
			firstLoad = false;

			updateStatus( validSatelliteWindow );
			delay( 3000 );
		}

		millisLastValidSignal = millis();
	}else{


		bool isWithinGPSUpdateWindow = ( millis() - millisLastValidSignal < MAX_READINGS_LOST_BEFORE_FAIL * GPS_READ_INTERVAL );

		// Lost Connection : was valid, missed too many signals -> indicate connection lost once!
		if( validSatelliteWindow && !isWithinGPSUpdateWindow ){

			validSatelliteWindow = false;
			// sensorFailedToReport = true;
			updateStatus( validSatelliteWindow );

		// Lost Connection : display last valid data until connection is restored
		}else if( !firstLoad && !isWithinGPSUpdateWindow ){

			renderToDisplay( false );
		}else if( firstLoad ){

			drawSignalSearch();
		}
	}

	if( validSatelliteWindow ){

		renderToDisplay( true );
	}

	// this order matters!
	cacheHistoricalData(); // we always want to advance a historical reading when the device first comes online
	advanceAnimationTicks();

	// This checks every hour for the switch to and from DST
	testEpoch += 3600UL;
}

void initCustomGPSObjects(){

	for( int i=0; i<4; ++i ){

		satNumber[ i ].begin( gps, "GPGSV", 4 + 4 * i ); // offsets 4, 8, 12, 16
		elevation[ i ].begin( gps, "GPGSV", 5 + 4 * i ); // offsets 5, 9, 13, 17
		azimuth[ i ].begin( gps, "GPGSV", 6 + 4 * i ); // offsets 6, 10, 14, 18
		snr[ i ].begin( gps, "GPGSV", 7 + 4 * i ); // offsets 7, 11, 15, 19
	}
}

void cacheHistoricalData(){

	// update only valid readings and on preferred cadence
	if( advanceGraph && !sensorFailedToReport && currentAltitude != ALTITUDE_OUT_OF_BOUND ){

		// advanced historical values
		for (int i = HISTORICAL_DATUM_COUNT; i >= 2; i--){

			altitudeHistory[ i - 1 ] = altitudeHistory[ i - 2 ];
			speedHistory[ i - 1 ] = speedHistory[ i - 2 ];
		}

		altitudeHistory[ 0 ] = constrain( currentAltitude, MIN_ALTITUDE_TRACKED, MAX_ALTITUDE_TRACKED );
		speedHistory[ 0 ] = constrain( averageSpeed, MIN_SPEED_TRACKED, MAX_SPEED_TRACKED );

		advanceGraph = false;
	}
}

void clearAltitudeHistory(){

	for( byte i = 0; i < HISTORICAL_DATUM_COUNT; i++ ){

		altitudeHistory[ i ] = 0;
	}
}

void clearSpeedHistory(){

	for( byte i = 0; i < HISTORICAL_DATUM_COUNT; i++ ){

		speedHistory[ i ] = 0;
	}
}

void listenToButtonPushes(){

	unsigned long currentMillis = millis();

	enterButton.loop();

	if( editMode && ( currentMillis - editModeTime ) > LEAVE_EDIT_MODE_TIME ){

		exitEditMode();
	}

	if( enterButton.isPressed() ){

		pressedTime = millis();
		isPressing = true;
		isLongDetected = false;
	}

	if( enterButton.isReleased() ) {

		isPressing = false;
		releasedTime = millis();

		long pressDuration = releasedTime - pressedTime;

		if( pressDuration < SHORT_PRESS_TIME ){

			buttonIsPressed( false ); // on SHORT press
		}
	}

	if(isPressing == true && isLongDetected == false) {

		long pressDuration = millis() - pressedTime;

		if( pressDuration > LONG_PRESS_TIME ) {

			buttonIsPressed( true ); // on LONG press
			isLongDetected = true;
		}
	}
}

void exitEditMode(){

	editMode = false;
	editingUnit = false;
	unitToEdit = 0;
	screen = lastScreen;

	// save unit settings
	updateMemory( configuredUnits[ 0 ].memoryAddress, configuredUnits[ 0 ].selectedUnit );
	updateMemory( configuredUnits[ 1 ].memoryAddress, configuredUnits[ 1 ].selectedUnit );
}

void buttonIsPressed( bool isLongPress ){


	if( isLongPress ){
		#ifdef DEBUG
		Serial.print("Long Press! \n");
		#endif
		if( !editMode ){ // long press to enter edit mode

			lastScreen = screen;
			screen = 7;
			editMode = true;
			editModeTime = millis();
			#ifdef DEBUG
			Serial.print( "Entering Edit Mode..." );
			Serial.print("\n");
			#endif

		}else{ // long press in edit mode

			if( !editingUnit ){ // edit a specific unit

				editingUnit = true;
			}

			// long press to accept current selection
		}

		if( editingUnit ){
			#ifdef DEBUG
			Serial.print( "Saving currint unit: " );
			Serial.print( unitToEdit );
			Serial.print("\n");
			#endif

			// advance which unit
			if( unitToEdit == 1 ){
				unitToEdit = 0;
			}else{
				unitToEdit++;
			}
		}
	}else{
		#ifdef DEBUG
		Serial.print("Short Press! \n");
		#endif

		if( !editMode ){ // cycle screens on short press for everything outside of the config screen
			lastScreen = screen;
			screen++;

			if( screen == SCREEN_COUNT ){
				screen = 0;
			}

			updateMemory( 0, screen );

		}else{ // edit mode

			editModeTime = millis();

			byte numberOfUnitTypes = configuredUnits[ unitToEdit ].numberOfSelectableTypes;
			byte selectedUnit = configuredUnits[ unitToEdit ].selectedUnit;

			// cycle through possible units
			if( selectedUnit == ( numberOfUnitTypes - 1 )){

				configuredUnits[ unitToEdit ].selectedUnit = 0;
			}else{

				configuredUnits[ unitToEdit ].selectedUnit++;
			}
			#ifdef DEBUG
			Serial.print( "unitToEdit: " );
			Serial.print( unitToEdit );
			Serial.print("\n");

			Serial.print( "selectedUnit: " );
			Serial.print( configuredUnits[ unitToEdit ].selectedUnit );
			Serial.print("\n");
			#endif
		}
	}
}

void drawSignalSearch(){

	display.drawCircle( 60, 58, 5, WHITE );
	display.drawCircle( 10 + 60, 58 - 3, 2, updateTick );
	display.drawCircle( 10 + 60, 58 + 3, 2, !updateTick );
	display.display();
}

void splashScreen(){

	display.setTextSize( 1 );
	display.setFont( &Lato_Thin_12 );
	display.setTextColor( WHITE );

	display.setCursor( 60, 15 );
	display.print(F("GPS"));

	display.drawLine( 10, 25, 117, 25, WHITE );

	display.setCursor( 12, 40 );
	display.print(F("SEARCHING FOR"));

	display.setCursor( 5, 50 );
	display.print(F("NEARBY SPUTNIKS"));

	display.drawBitmap( 15, 0, bitmap, 20, 20, WHITE );
}

void updateStatus( bool isValidSignalWindow ){

	display.clearDisplay();
	display.setFont(&Lato_Thin_12);
	display.setTextSize(1);
	display.setTextColor(WHITE);

	if( isValidSignalWindow ){

		display.setCursor(12,23);
		display.print(F("DOWNLINK OK!\n"));

		display.drawRoundRect( 0, 3, 128, 30, 5, WHITE);

		display.setCursor(0,49);
		slowType(F("FIX ON: "), 10, false );

		display.print( numberOfFixedSatellites );
		slowType(F(" SPUTNIKS"), 10, false );
		display.display();
	}else{

		display.setCursor(45,0);
		display.print(F("GPS"));

		display.setCursor(22,32);
		display.print(F("CONNECTION"));

		display.setCursor(45,42);
		display.print(F("LOST!"));
		display.drawRoundRect( 10, 20, 110, 30, 5, WHITE);
		display.display();

		delay(2000);
		renderToDisplay( false );
	}
}

void populateGPSData(){

	#ifdef DEBUG

	currentLatitude = 32.0;
	currentLongitude = 120.0;
	currentHeading = pollingTick ? 360.00 : 180.00;
	currentSpeed = pollingTick ? 19.4 : 30;
	currentAltitude = 1000.0;
	numberOfFixedSatellites = 8;

	if ( firstLoad ){
		Serial.print("setting mock GPS DATA: ");
		Serial.print(currentLatitude);
		Serial.print(" : ");
		Serial.print(currentLongitude);
		Serial.print(" : ");
		Serial.print(currentHeading);
		Serial.print(" : ");
		Serial.print(currentAltitude);
		Serial.print(" : ");
		Serial.print(numberOfFixedSatellites);
		Serial.print("\n");
	}
	#else
	currentLatitude = gps.location.lat();
	currentLongitude = gps.location.lng();
	currentHeading = gps.course.deg();
	currentAltitude = gps.altitude.meters();
	currentSpeed = gps.speed.mps();
	numberOfFixedSatellites = gps.satellites.value();
	#endif

	float average;

	// subtract the last reading:
	total = total - readings[ readIndex ];

	readings[ readIndex ] = currentSpeed;

	// add value to total:
	total = total + readings[ readIndex ];
	// bump index
	readIndex = readIndex + 1;

	if( readIndex >= maxNumberOfReadsBetweenHistoricalPlots ) {

		readIndex = 0;
	}

	average = total / maxNumberOfReadsBetweenHistoricalPlots;
	averageSpeed = average;

	bool satelliteDetailsAreValid = totalGPGSVMessages.isUpdated();

	// refresh satellite info
	if( satelliteDetailsAreValid ){

		isValidSatelliteDetailsWindow = true;
		millisLastValidsatelliteInfo = millis();

		for( byte i = 0; i < 4; ++i ){

			byte sat = atoi( satNumber[ i ].value());

			if( sat >= 1 && sat <= MAX_SATELLITES ){
				sats[ sat - 1 ].elevation = atoi( elevation[ i ].value());
				sats[ sat - 1 ].azimuth = atoi( azimuth[ i ].value());
				sats[ sat - 1 ].snr = atoi( snr[ i ].value());
				sats[ sat - 1 ].active = true;
			}
		}
	}
}

void advanceAnimationTicks(){

	unsigned long currentMillis = millis();
	bool graphAnimationTick = ( previousMillisGraph == 0 || currentMillis - previousMillisGraph > GRAPH_UPDATE_INTERVAL );
	bool satelliteSearchAnimationTick = ( previousMillisSatSearch == 0 || currentMillis - previousMillisSatSearch > SAT_SEARCH_ANIMATION_INTERVAL );
	bool alertBlinkTick = ( previousMillisBlinkTick == 0 || currentMillis - previousMillisBlinkTick > UNIT_EDIT_BLINKER_INTERVAL );

	isValidSatelliteDetailsWindow = ( currentMillis - millisLastValidsatelliteInfo < SAT_DETAIL_INFO_MAX_AGE );

	if( graphAnimationTick ){

		previousMillisGraph = currentMillis;

		if( !sensorFailedToReport ){

			advanceGraph = true;
		}
	}

	if( satelliteSearchAnimationTick ){

		previousMillisSatSearch = currentMillis;
		updateTick = !updateTick;
	}

	if( alertBlinkTick ){

		previousMillisBlinkTick = currentMillis;
		pollingTick = !pollingTick;
	}

	// lerp relevant values
	smoothedHeadingAngle = lerpAngle( smoothedHeadingAngle, currentHeading, EASE_COEFFICIENT);
	smoothedHeadingDigit = lerp( smoothedHeadingDigit, currentHeading, EASE_COEFFICIENT);
}


bool isGPSDataAvailable(){

	sensorFailedToReport = true;

	while ( Serial1.available() ){
		// Each byte of NEMA data must be giving to TinyGPS by using encode(). True is returned when new data has been fully decoded and can be used
		if( gps.encode( Serial1.read() )){

			sensorFailedToReport = false;
		}
	}

	return sensorFailedToReport;
}


static void updateTimeFromGPS(){

	if( gps.time.age() < 500 ){
		setTime(
			gps.time.hour(),
			gps.time.minute(),
			gps.time.second(),
			gps.date.day(),
			gps.date.month(),
			gps.date.year()
		);
	}

	utc = now();

	// should time tick
	if( ( timeStatus()!= timeNotSet && utc != lastTimeTicked ) ){

		local = Pacific.toLocal( utc );

		lastTimeTicked = utc;
		lastKnownDay = day( local );
		lastKnownMonth = month( local );
		lastKnownYear = year( local );
		lastKnownHour = hour( local );
		lastKnownMinute = minute( local );
		lastKnownSecond = second( local );
	}
}

void renderToDisplay( bool hasConnection ){

	display.clearDisplay();

	if( screen == 0 ){

		drawComprehensiveView( hasConnection );
	}else if( screen == 1 ){

		drawCriticalViewLatLong( hasConnection );
	}else if( screen == 2 ){

		drawCriticalViewAltitudeSpeed( hasConnection );
	}else if( screen == 3 ){

		drawAltitudeHistory( hasConnection );
	}else if( screen == 4 ){

		drawSpeedHistory( hasConnection );
	}else if( screen == 5 ){

		drawSatelliteDetails( hasConnection );
	}else if( screen == 6 ){

		drawCurrentHeading( hasConnection );
	}else if( screen == 7 ){

		drawConfigurationSettings( hasConnection );
	}

	display.display();
}

void drawConfigurationSettings( bool hasConnection ){

	// bool isImperialSelected = configuredUnits[ unitToEdit ].selectedUnit  //unitToEdit < 1;
	bool distanceIsImperial = configuredUnits[ 0 ].selectedUnit == 0;
	bool speedIsImperial = configuredUnits[ 1 ].selectedUnit == 0;

	display.setTextSize( 1 );
	display.setFont( &Lato_Thin_12 );
	display.setTextColor( WHITE );

	display.setCursor( 20, 9 );
	display.print(F("DISTANCE >"));
	drawUnit( 5, 20, distanceIsImperial, 0 );

	display.setCursor( 20, 43 );
	display.setTextColor( WHITE );
	display.print(F("SPEED >"));
	drawUnit( 5, 54, speedIsImperial, 1 );

	byte xOffsetEditCircle = 10;
	byte yOffsetEditCircle = unitToEdit == 0 ? 4 : 38;

	if( pollingTick ){

		display.fillCircle( xOffsetEditCircle, yOffsetEditCircle, 4, WHITE );
	}
}

void drawUnit( byte xOffset, byte yOffset, bool isImperialSelected, byte unitType ){

	if( isImperialSelected ){

		display.fillRoundRect( xOffset - 4 , yOffset - 6, 66, 14, 5, WHITE);
	}else{
		display.setTextColor( WHITE );
		display.fillRoundRect( xOffset + 65 , yOffset - 6, 57, 14, 3, WHITE);
	}
	display.drawFastHLine( 0, yOffset - 9 , DISPLAY_WIDTH, WHITE );
	display.setTextColor( !isImperialSelected );
	display.setCursor( xOffset, yOffset + 5 );
	display.print("IMPERIAL");

	display.setTextColor( isImperialSelected );
	display.setCursor( xOffset + 69 , yOffset + 5 );
	display.print("METRIC");

	display.setTextColor( WHITE );
}

void drawCurrentHeading( bool hasConnection ){

	display.setTextSize( 1 );
	display.setFont( &Lato_Thin_12 );
	display.setTextColor( WHITE );

	display.setCursor( 0, 20 );
	display.print(F("HEADING"));

	display.setCursor( 5, 50 );
	display.setFont( &Lato_Thin_30 );
	display.print( smoothedHeadingDigit, 0 );

	byte radius = 20;
	display.setFont();
	renderGauge( smoothedHeadingAngle , ( DISPLAY_WIDTH / 2) - radius + 30, ( DISPLAY_HEIGHT / 2 ) - radius , radius );
}

void drawSatelliteDetails( bool hasConnection ){

	display.setTextSize(1);
	display.setFont();
	display.setTextColor( WHITE );

	bool satelliteDetailsAreValid = totalGPGSVMessages.isUpdated();

	if( isValidSatelliteDetailsWindow && satelliteDetailsAreValid ){

		for( int i=0; i < MAX_SATELLITES; ++i ){

			display.setCursor( 0, i * 10 );

			if( sats[ i ].active ){

				display.print( i + 1 ); // satellite number
				display.print(F("|E:"));
				display.print( sats[ i ].elevation );
				display.print(F("' A:"));
				display.print( sats[ i ].azimuth );
				display.print(F("' S:"));
				display.print( sats[ i ].snr );
				display.print(F("dB"));
			}else{
				display.print( i + 1 ); // satellite number
				display.print(F(" : No Details"));
			}
		}
	}else{

		display.setCursor( 12, 20 );
		display.setTextColor( WHITE );
		display.setFont( &Lato_Thin_12 );
		display.print(F("LISTENING FOR"));
		display.setCursor( 5, 40 );
		display.print(F("SPUTNIK DETAILS"));
		drawSignalSearch();
	}
}


void drawCriticalViewLatLong( bool hasConnection ){

	printDegreeMinuteSecond( currentLatitude, 0, 10, "LAT" );
	printDegreeMinuteSecond( currentLongitude, 0, 45, "LONG" );
}

void printDegreeMinuteSecond( float value, byte xOffset, byte yOffset, String label ){


	display.setTextSize( 1 );
	display.setFont( &Lato_Thin_12 );
	display.setTextColor( WHITE );
	display.setCursor( xOffset, yOffset );
	display.print( label );
	display.print(F(": "));
	display.print( value, 6 );

	display.setCursor( xOffset + 15, yOffset + 15 );
	bool isNegative = ( value < 0 );
	byte degree = (byte)abs( value );
	byte minute = (byte) ( (abs( value ) - (float)degree) * 60.f);
	float second = (float) ( (abs( value ) - (float)degree - (float)minute / 60.f) * 60.f * 60.f );
	if( isNegative ){
		display.print(F("-"));
	}
	display.print( degree );
	display.print( F("* " ));
	display.print( minute );
	display.print( F("' " ));
	display.print( second, 1 );
	display.print( F("''") );

	if( label == "LAT" ){
		if( degree > 0 ){
			display.print(F(" N"));
		}else{
			display.print(F(" S"));
		}
	}else{
		if( degree > 0 ){
			display.print(F(" W"));
		}else{
			display.print(F(" E"));
		}
	}

}
void drawCriticalViewAltitudeSpeed( bool hasConnection ){

	bool distanceIsImperial = configuredUnits[ 0 ].selectedUnit == 0;
	float altitude = distanceIsImperial ? currentAltitude * configuredUnits[ 0 ].coefficient[ 2 ] : currentAltitude;
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(10,8);
	display.setFont(&Lato_Thin_12);

	display.print( F("ALTITUDE: ") );
	display.setFont(&Lato_Thin_30);
	display.setCursor(10,31);
	display.print( altitude, 1 );
	display.setFont(&Lato_Thin_12);
	if( distanceIsImperial ){
		display.println( F(" FT") );
	}else{
		display.println( F(" M") );
	}

	display.setCursor(9,41);
	display.print( F("SPEED: ") );
	display.setFont(&Lato_Thin_30);

	bool speedIsImperial = configuredUnits[ 1 ].selectedUnit == 0;

	float speed = ( speedIsImperial ) ? gps.speed.mph(): gps.speed.kmph(); // convert with coefficients

	if( ( speed != -1 ? speed : 0) < 10 ){

		display.setCursor(47, 64); // single digit
	}else if( (  speed != -1 ? speed : 0 ) < 100 ){

		display.setCursor(28, 64); // double digit
	}else{

		display.setCursor(12, 64); // triple digit
	}

	display.print(  speed != -1 ? speed : 0, 1 );
	display.setFont( &Lato_Thin_12 );
	if( speedIsImperial ){

		display.println( "  MPH" );
	}else{

		display.println( "  KMH" );
	}
}

void drawAltitudeHistory( bool hasConnection ){

	bool distanceIsImperial = configuredUnits[ 0 ].selectedUnit == 0;
	float altitude = distanceIsImperial ? currentAltitude * configuredUnits[ 0 ].coefficient[ 2 ] : currentAltitude;

	float minAltitudeTracked = MIN_ALTITUDE_TRACKED; // [ METERS ] [ FEET ] : minimum value plotted in history : default unit METERS
	float maxAltitudeTracked = MAX_ALTITUDE_TRACKED; // [ METERS ] [ FEET ] : maximum value plotted in history : default unit METERS

	display.setTextSize(1);
	display.setFont();
	display.setTextColor( WHITE );
	display.setCursor( 0,0 );
	display.print(F("Altitude:"));
	display.print( altitude, 0 );

	if( distanceIsImperial ){

		display.println(F("FT"));
	}else{

		display.println(F("M"));
	}
	renderGraph( altitudeHistory, 0, 10, minAltitudeTracked, maxAltitudeTracked, MIN_ALTITUDE_TRACKED, MAX_ALTITUDE_TRACKED  );
}

void drawSpeedHistory( bool hasConnection ){

	bool speedIsImperial = configuredUnits[ 1 ].selectedUnit == 0;
	float speed = speedIsImperial ? currentSpeed * configuredUnits[ 1 ].coefficient[ 3 ] : currentSpeed * configuredUnits[ 1 ].coefficient[ 1 ];

	float minSpeedTracked = MIN_SPEED_TRACKED; // [ M/S ] : minimum value plotted in history
	float maxSpeedTracked = MAX_SPEED_TRACKED; // [ M/S ] : maximum value plotted in history

	display.setTextSize(1);
	display.setFont();
	display.setTextColor( WHITE );
	display.setCursor( 0,0 );
	display.print(F("Speed:"));
	display.print( speed, 0 );

	if( speedIsImperial ){

		display.println(F(" MPH"));
	}else{

		display.println(F(" KMH"));
	}
	renderSpeedGraph( speedHistory, 0, 10, minSpeedTracked, maxSpeedTracked, MIN_SPEED_TRACKED, MAX_SPEED_TRACKED  );
}

void drawComprehensiveView( bool hasConnection ){
	display.setFont();
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(0,0);

	if( hasConnection ){

		display.print(F("GPS | Satellites: "));
		display.println( numberOfFixedSatellites );//print number of satellites detected to display
	}else{

		display.setCursor(0,2);

		uint16_t color = updateTick ? WHITE : BLACK;
		if( updateTick ){
			display.print(F("Last Valid Coordinate"));
		}else{
			display.print(F(".Locating Satellites."));
		}

		display.drawLine( 0, 0, 127, 0, color);
		display.drawLine( 0, 10, 127, 10, color);
		display.setTextColor(WHITE);
	}

	printClock();


	display.setCursor(0,27);
	if( currentLatitude < 1 ){
		display.print(F("Lat:       "));
	}else{
		display.print(F("Lat:        "));
	}
	display.println(currentLatitude,6);

	display.setCursor(0,37);

	if( currentLongitude < 1 ){
		if( currentLongitude <= -100 ){
			display.print(F("Long:     ")); // extra pad
		}else{
			display.print(F("Long:     "));
		}
	}else{
		if( currentLongitude < 100 ){
			display.print(F("Long:       ")); // extra pad
		}else{
			display.print(F("Long:      "));  // extra pad
		}
	}
	display.println(currentLongitude,6);

	display.setCursor(0,47);
	display.print(F("Heading:  "));

	display.print( gps.cardinal( currentHeading ));
	display.print(F(": "));
	display.println( currentHeading, 1 );

	display.setCursor( 120,46 );
	display.write(0xf7);

	// Altitude
	display.setCursor( 0, 57 );

	bool distanceIsImperial = configuredUnits[ 0 ].selectedUnit == 0;
	float altitude = distanceIsImperial ? currentAltitude * configuredUnits[ 0 ].coefficient[ 2 ] : currentAltitude;

	char altitudeString[ 16 ];
	if( distanceIsImperial ){

		sprintf( altitudeString, "Altitude:  %d.%02d ft", (int)altitude, (int)(fabsf( altitude )*100)%100 );
	}else{

		sprintf( altitudeString, "Altitude:  %d.%02d m", (int)altitude, (int)(fabsf( altitude )*100)%100 );
	}
	display.println (altitudeString );
}

void printClock() {

	char dateString[ 32 ];
	sprintf( dateString,
		"%02d/%02d/%02d  %02d:%02d:%02d",
		lastKnownDay,
		lastKnownMonth,
		lastKnownYear,
		lastKnownHour,
		lastKnownMinute,
		lastKnownSecond
	);

	display.setCursor( 0, 15 );
	display.println( dateString );

	display.drawLine(0,13,127,13, WHITE);
	display.drawLine(0,23,127,23, WHITE);
}

void slowType( String text, int delayTime, bool newLine ){

	if( newLine ){

		display.setCursor(0, lastLine );
		lastLine += SLOW_TYPE_NEXT_LINE_PADDING;
	}

	for( byte i = 0; i < text.length(); i++ ){

		display.print( text[ i ] );
		display.display();
		delay( delayTime );
	}
}

void slowPrintSuccessOrFail( bool condition ){

	condition ? slowType( F("OK"), 50, false ) : slowType( F("FAIL"), 50, false );
}

void renderGraph( short sensorArray[], int xOffset, int yOffset, float minValue, float maxValue, float defaultMinValue, float defaultMaxValue ){

	int upperBound = 10;
	int lowerBound = minValue;
	short whichBoundHasMoreDatums = 0; // negative : low 1, positive: high
	bool distanceIsImperial = configuredUnits[ 0 ].selectedUnit == 0;
	bool speedIsImperial = configuredUnits[ 1 ].selectedUnit == 0;

	maxValue = distanceIsImperial ? defaultMaxValue * configuredUnits[ 0 ].coefficient[ 2 ] : defaultMaxValue;
	minValue = distanceIsImperial ? defaultMinValue * configuredUnits[ 0 ].coefficient[ 2 ] : defaultMinValue;

	for( byte i = 0; i < HISTORICAL_DATUM_COUNT; i++ ){

		int currentValue = distanceIsImperial ? sensorArray[ i ] * configuredUnits[ 0 ].coefficient[ 2 ] : sensorArray[ i ]; // feet, meters

		if( currentValue > upperBound ){

			upperBound = currentValue;
		}else if( currentValue < lowerBound ){

			lowerBound = currentValue;
		}

		whichBoundHasMoreDatums += ( currentValue > 0 ) ? 1 : ( currentValue < 0 ) ? -1 : 0;
	}

	upperBound = ( upperBound * 2 ) <= maxValue ? upperBound * 2 : maxValue;
	lowerBound = ( lowerBound * 2 ) >= minValue ? lowerBound * 2: minValue;

	byte topOfGraph = yOffset;
	byte bottomOfGraph = topOfGraph + GRAPH_HEIGHT;
	// moving scale; favors screen space where there are more datums
	float ratioOfHighestToLowest = ( whichBoundHasMoreDatums > 0 ) ? 4.0 : 2.0;
	byte splitPoint = bottomOfGraph - ceil( (float)GRAPH_HEIGHT / ratioOfHighestToLowest ); // sea level, 0 altitude
	byte graphLowerBound = bottomOfGraph - splitPoint;
	byte graphUpperBound = splitPoint - yOffset;
	byte xOffsetBoundsLabel = GRAPH_WIDTH + xOffset - 3;

	determineUnitAndScaleForBound( &graphBounds[ 0 ], lowerBound );
	determineUnitAndScaleForBound( &graphBounds[ 1 ], upperBound );

	// draw altitude scaled to upper and lower bounds
	for( byte step = 0; step < HISTORICAL_DATUM_COUNT; step++ ){

		short value = distanceIsImperial ? sensorArray[ step ] * configuredUnits[ 0 ].coefficient[ 2 ] : sensorArray[ step ]; // scale value to proper unit
		bool isNegative = (value < 0);
		double position;

		if( isNegative ){

			position = mapf( (double)abs( value ), 0.0, (double)abs(lowerBound), 0.0, 1.0 ); // normalized magnitude
			position = ceil( graphLowerBound * position ); // scaled to lower bound
		}else{

			position = mapf( (double)abs( value ), 0.0, (double)upperBound, 0.0, 1.0 ); // normalized magnitude
			position = ceil( graphUpperBound * position ); // scaled to upper bound
		}

		byte start = ( !isNegative ) ? ( splitPoint - position ) : splitPoint;
		byte length = ( !isNegative ) ? splitPoint - start : position;

		display.writeFastVLine( GRAPH_WIDTH + xOffset - (step * 2 ), start, length, WHITE ); // Line Datum
	}

	// graph height scale
	display.writeFastVLine(GRAPH_WIDTH + xOffset + 5, yOffset, GRAPH_HEIGHT, WHITE);

	// intermediate magnitudes (upperbound only)
	for( byte i = 0; i < ALTITUDE_HISTORY_INTERMEDIATE_SCALE - 1; i++){

		short step = ( splitPoint - topOfGraph ) / ALTITUDE_HISTORY_INTERMEDIATE_SCALE;
		byte start = topOfGraph + step;
		step = start + (step*i);

		for( byte j = xOffset; j < GRAPH_WIDTH; j+= 6 ){
			display.drawPixel( j , step, WHITE); // draw all the way across
		}

		display.drawPixel( GRAPH_WIDTH + xOffset + 7 , step, WHITE); // right of graph height scale

		float value = upperBound - (i + 1.0f) * ( upperBound / ALTITUDE_HISTORY_INTERMEDIATE_SCALE ); // 0 to upper bound
		value = value / (float)graphBounds[ 1 ].denominator;

		byte xPosition = GRAPH_WIDTH + xOffset + 9;
		byte yPosition = step - 4;
		byte decimal = 1;

		if( value == (int)value || value == 0 ){

			decimal = 0;// whole numbers displayed if there's no fractional component

			if( value < 10 ){
				xPosition += 3; // single digit slight bump
			}
		}

		display.setCursor( xPosition, yPosition );
		display.print( value , decimal );
	}

	// zero point
	display.writeFastHLine(GRAPH_WIDTH + xOffset + 3, splitPoint, 4, WHITE);
	display.setCursor( GRAPH_WIDTH + xOffset + 10, splitPoint );
	display.print( 0 );

	// upper bound
	display.writeFastHLine(GRAPH_WIDTH + xOffset + 3, yOffset, 4, WHITE);
	display.setCursor( xOffsetBoundsLabel , yOffset - 10 );

	// magnitude
	if( graphBounds[ 1 ].isInteger ){

		display.print( upperBound / graphBounds[ 1 ].denominator );
	}else{

		display.print((float)upperBound / graphBounds[ 1 ].denominator, 1 );
	}

	// unit
	display.print( graphBounds[ 1 ].unit );

	// lower bound
	display.writeFastHLine(GRAPH_WIDTH + xOffset + 3, bottomOfGraph, 4, WHITE);
	display.setCursor( xOffsetBoundsLabel , bottomOfGraph + 3 );

	// magnitude
	if( graphBounds[ 0 ].isInteger ){

		display.print( lowerBound / graphBounds[ 0 ].denominator );
	}else{

		display.print((float)lowerBound / graphBounds[ 0 ].denominator, 1 );
	}

	// unit
	display.print( graphBounds[ 0 ].unit );
}

void determineUnitAndScaleForBound( struct bounds *bound, int value ){

	bool distanceIsImperial = configuredUnits[ 0 ].selectedUnit == 0;
	bool speedIsImperial = configuredUnits[ 1 ].selectedUnit == 0;
	char unitOne[5];
	char unitTwo[5];
	int unitMagnitudeFactor;
	int floorMagnitude;

	unitMagnitudeFactor = distanceIsImperial ? 5280 : 1000; // feet : meters
	floorMagnitude = distanceIsImperial ? 328 : 100; // feet : meters

	strcpy( unitOne, distanceIsImperial ? "MI" : "KM" );
	strcpy( unitTwo, distanceIsImperial ? "FT" : "M" );

	if( value >= unitMagnitudeFactor || value >= floorMagnitude ){

		if( value >= unitMagnitudeFactor ){

			bound->denominator = unitMagnitudeFactor;
			bound->isInteger = true;
		}else{

			bound->denominator = unitMagnitudeFactor * 1.0f;
			bound->isInteger = false;
		}

		strcpy( bound->unit, unitOne );
	}else{

		bound->denominator = 1;
		bound->isInteger = true;
		strcpy( bound->unit, unitTwo );
	}
}

void renderSpeedGraph( short sensorArray[], int xOffset, int yOffset, float minValue, float maxValue, float defaultMinValue, float defaultMaxValue ){

	bool speedIsImperial = configuredUnits[ 1 ].selectedUnit == 0;

	int lowerBound = minValue;
	int upperBound = speedIsImperial ? defaultMaxValue * configuredUnits[ 1 ].coefficient[ 3 ] : defaultMaxValue * configuredUnits[ 1 ].coefficient[ 1 ];

	maxValue = speedIsImperial ? defaultMaxValue * configuredUnits[ 1 ].coefficient[ 3 ] : defaultMaxValue * configuredUnits[ 1 ].coefficient[ 1 ];

	byte topOfGraph = yOffset;
	byte bottomOfGraph = topOfGraph + GRAPH_HEIGHT;
	byte splitPoint = bottomOfGraph;

	byte graphWidth = GRAPH_WIDTH + 2;
	byte graphLowerBound = bottomOfGraph;
	byte graphUpperBound = splitPoint - yOffset;
	byte xOffsetBoundsLabel = graphWidth + xOffset + 3;

	// draw speed scaled to upper and lower bounds
	for( byte step = 0; step < HISTORICAL_DATUM_COUNT; step++ ){

		short value = speedIsImperial ? sensorArray[ step ] * configuredUnits[ 1 ].coefficient[ 3 ] : sensorArray[ step ] * configuredUnits[ 1 ].coefficient[ 1 ]; // scale value to proper display unit
		double position = mapf( (double)abs( value ), 0.0, (double)upperBound, 0.0, 1.0 ); // normalized magnitude

		position = ceil( graphUpperBound * position ); // scaled to upper bound

		byte start = splitPoint - position;
		byte length = splitPoint - start;

		display.writeFastVLine( graphWidth + xOffset - (step * 2 ), start, length, WHITE ); // Line Datum
	}

	// graph height scale
	display.writeFastVLine(graphWidth + xOffset + 5, yOffset, GRAPH_HEIGHT, WHITE);

	// intermediate magnitudes (upperbound only)
	for( byte i = 0; i < 5 - 1; i++){

		short step = ( splitPoint - topOfGraph ) / ALTITUDE_HISTORY_INTERMEDIATE_SCALE;
		byte start = topOfGraph + step;
		step = start + (step*i);

		for( byte j = xOffset; j < graphWidth; j+= 6 ){

			display.drawPixel( j , step, WHITE); // draw all the way across
		}

		display.drawPixel( graphWidth + xOffset + 7 , step, WHITE); // right of graph height scale

		float value = upperBound - (i + 1.0f) * ( upperBound / ALTITUDE_HISTORY_INTERMEDIATE_SCALE ); // 0 to upper bound
		value = value;

		byte xPosition = graphWidth + xOffset + 9;
		byte yPosition = step - 4;
		byte decimal = 1;

		if( value == (int)value || value == 0 ){

			decimal = 0;// whole numbers displayed if there's no fractional component

			if( value < 10 ){
				xPosition += 3; // single digit slight bump
			}
		}

		display.setCursor( xPosition, yPosition );
		display.print( value , decimal );
	}

	// zero point
	display.writeFastHLine(graphWidth + xOffset + 3, splitPoint, 4, WHITE);

	// upper bound
	display.writeFastHLine(graphWidth + xOffset + 3, yOffset, 4, WHITE);
	display.setCursor( xOffsetBoundsLabel , yOffset - 10 );

	// magnitude
	display.print( upperBound );
}

void renderAxes( int xOffset, int yOffset, int topBound, int lowBound ){

	//bounding vert line
	display.drawLine( xOffset, yOffset,  xOffset, GRAPH_HEIGHT + yOffset, WHITE);
	display.drawLine( GRAPH_WIDTH + xOffset, yOffset, GRAPH_WIDTH + xOffset, GRAPH_HEIGHT + yOffset, WHITE );

	// scale marks
	for( int count = 0; count <= GRAPH_HEIGHT; count += ( GRAPH_HEIGHT / 4 ) ){
		int position = GRAPH_HEIGHT - count;
		float scaleMark = mapf( (long)position, 0.0f, (long)GRAPH_HEIGHT, (long)topBound, (long)lowBound);
		int scaleOffsetStart = 0;

		display.setFont();
		display.setCursor( scaleOffsetStart, yOffset + position - 5 );
		display.setTextSize( 1 );
		display.setTextColor( WHITE );
		display.print( scaleMark, 1 );
		display.setFont();

		// scale marks horizontal
		display.drawLine( GRAPH_WIDTH + xOffset, count + yOffset, GRAPH_WIDTH - (GRAPH_WIDTH/32) + xOffset, count + yOffset, WHITE );
		display.drawLine( xOffset, count + yOffset, GRAPH_WIDTH/32 + xOffset, count + yOffset, WHITE );
	}

	for (int count = 10 + xOffset; count < GRAPH_WIDTH + xOffset; count += 10){

		display.drawPixel(count, yOffset , WHITE);
		display.drawPixel(count, yOffset + GRAPH_HEIGHT + 2 , WHITE);
	}
}

void renderGauge( double angle, byte xOffset, byte yOffset, byte radius ){

	double xCenter = radius;
	double yCenter = radius;
	double needleLength = ( radius * 1.3f );

	double x = -needleLength / 2;
	double y = 0;
	double x1 = needleLength / 2;
	double y1 = 0;
	double cx = ( x + x1 ) / 2;
	double cy = ( y + y1 ) / 2;

	double xTip = -needleLength / 12;
	double yTip = 0;
	double x1Tip = needleLength / 12;
	double y1Tip = 0;
	double cxTip = ( x + x1 ) / 2;
	double cyTip = ( y + y1 ) / 2;

	double headingTheta = -angle * ( M_PI_LONG / 180.0L ) - ( M_PI_LONG / 2.0L );

	// draw border of the gauge
	display.drawCircle( xCenter + xOffset, yCenter + yOffset, radius, WHITE );

	// gauge ticks 30 degree increments
	for( double angle = 0.0; angle < 360.0; angle += 360.0 / 16.0 ){

		display.drawPixel(
			xCenter + xOffset + (( 3 + radius ) * -cos( angle * (M_PI_LONG / 180.0L) )),
			yCenter + yOffset + (( 3 + radius ) * -sin( angle * (M_PI_LONG / 180.0L) )),
			WHITE
		);
	}

	for( int angle = 0; angle < 360; angle+= 360 / 8 ){

		int direction = (int)(( angle + 11.25f ) / 22.5f);
		double theta = ( (double)angle * ( M_PI_LONG / 180.0L ));

		int trimX = -2;
		int trimY = -2;

		if( direction > 8 && direction < 16 && direction != 12  ){ // left of circle

			trimX -= 5;
		}

		if( direction == 6 || direction == 10 ){ // SE, SW Label

			trimY -= 2;
		}else if(  direction == 4 ){ // E Label

			trimY -= 2;
		}else if( direction == 8 ){ // S Label

			trimY -= 3;
			trimX = -2;
		}else if( direction == 0 ){ // N Label

			trimY = -1;
			trimX = -2;
		}

		theta -= ( M_PI_LONG / 2.0L );

		byte x = xCenter + xOffset + trimX + (( 10 + radius ) * cos( theta ));
		byte y = yCenter + yOffset + trimY + (( 10 + radius ) * sin( theta ));

		display.setCursor( x, y );
		display.print( directions[ direction % 16 ] );
	}

	// start of needle
	int rotX = (  -(x - cx) * cos( headingTheta ) + (y - cy) * sin( headingTheta ) ) + cx;
	int rotY = ( (x - cx) * sin( headingTheta ) + (y - cy) * cos( headingTheta ) ) + cy;

	//end of needle
	int rotXEnd = ( -(x1 - cx) * cos( headingTheta ) + (y1 - cy) * sin( headingTheta ) ) + cx;
	int rotYEnd = ( (x1 - cx) * sin( headingTheta ) + (y1 - cy) * cos( headingTheta ) ) + cy;

	double rotXTip = ( -(x1Tip - cxTip) * cos( headingTheta ) + (y1Tip - cyTip) * sin( headingTheta ) ) + cxTip;
	double rotYTip = ( (x1Tip - cxTip) * sin( headingTheta ) + (y1Tip - cyTip) * cos( headingTheta ) ) + cyTip;

	display.drawCircle(rotX + xOffset + radius, rotY + yOffset + radius,2, WHITE);
	drawRotatedTriangle( -1, ( rotXTip + xOffset + radius ), (rotYTip + yOffset + radius ), headingTheta );

	display.drawLine(rotX + xOffset + radius, rotY + yOffset + radius, rotXEnd + xOffset + radius, rotYEnd + yOffset + radius, WHITE);
}

void drawRotatedTriangle( int sign, double xOffset, double yOffset, double theta ){

	double tX = 20 * sign;
	double tY = 0;

	double t1X = 0;
	double t1Y = 6 * sign;

	double t2X = 0;
	double t2Y = -6 * sign;

	display.drawTriangle(
		xOffset + (  tX * cos( theta ) + tY * sin( theta ) ),
		yOffset + ( -tX * sin( theta ) + tY * cos( theta ) ),

		xOffset + (  t1X * cos( theta ) + t1Y * sin( theta ) ),
		yOffset + ( -t1X * sin( theta ) + t1Y * cos( theta ) ),

		xOffset + (  t2X * cos( theta ) + t2Y * sin( theta ) ),
		yOffset + ( -t2X * sin( theta ) + t2Y * cos( theta ) ),
		WHITE
	);
}

void wipeDisplay(){

	display.clearDisplay(); // remove library banner
	display.fillScreen(BLACK); // I see a red door...
	display.display(); // because fillScreen is misleading
}

void updateMemory( byte address, byte value ){

	EEPROM.update( address, value );
	EEPROM.commit();
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max){
  return (double)(x - in_min) * (out_max - out_min) / (double)(in_max - in_min) + out_min;
}

double clampDouble(double value, double min, double max){

	if( value < min ){

		value = min;
	}else if( value > max ){

		value = max;
	}

	return value;
}

double clamp( double value ){
	if( value < 0.0L ){

		return 0.0L;
	}else if( value > 1.0L ){

		return 1.0L;
	}else{

		return value;
	}
}

double Repeat( double t, double length ){

	return clampDouble( t - floor( t / length ) * length, 0.0L, length );
}

double lerpAngle( double currentAngle, double targetAngle, double alpha ){

	double delta = Repeat( ( targetAngle - currentAngle ), 360.0L );

	if( delta > 180.0L ){

		delta -= 360.0L;
	}

	return currentAngle + delta * clamp( alpha );
}

float lerp( float a, float b, float num ){

	return a + num * (b - a);
}

// float movingAverage(float value) {
// 	const byte nvalues = 8;             // Moving average window size

// 	static byte current = 0;            // Index for current value
// 	static byte cvalues = 0;            // Count of values read (<= nvalues)
// 	static float sum = 0;               // Rolling sum
// 	static float values[nvalues];

// 	sum += value;

// 	// If the window is full, adjust the sum by deleting the oldest value
// 	if (cvalues == nvalues)
// 	sum -= values[current];

// 	values[current] = value;          // Replace the oldest with the latest

// 	if (++current >= nvalues)
// 	current = 0;

// 	if (cvalues < nvalues)
// 	cvalues += 1;

// 	return sum/cvalues;
// }
