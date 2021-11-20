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
#include <TimeLib.h>
#include <ezButton.h>
#include <TinyGPSPlus.h>
#include "Lato_Thin_12.h"
#include "Lato_Thin_30.h"

#define M_PI_LONG 3.141592653589793238462643383279502884L // [ LONG ] slice of pizza
#define EASE_COEFFICIENT 0.15 // [ FLOAT ] : coefficient used for lerp transforms

// Display Characteristics
#define DISPLAY_WIDTH 128 // [ PIXELS ] number of available horizontal pixels
#define DISPLAY_HEIGHT 64 // [ PIXELS ] number of available vertical pixels
#define SLOW_TYPE_NEXT_LINE_PADDINGE 10 // [ PIXELS ] of padding below any given text animated to UI via "slow typing" effect
#define OLED_RESET -1 // [ NUMBER ] digital read pin to reset the display | NOT IMPLEMENTED
#define ENTER_BUTTON_PIN 0 // [ NUMBER ] digital read pin for button inputs

// UI configs
#define DEFAULT_SCREEN 0 // [ NUMBER ] : index of the startup UI layout
#define SCREEN_COUNT 6 // [ NUMBER ] of UI layouts available
#define NUMBER_OF_CARDINAL_DIRECTIONS 16 // [ NUMBER ] of cardinal labels available for readable heading output
#define MAX_SATELLITES 6 // [ NUMBER ] of satellites tracked on the following metrics: ( Index, Elevation, Azimuth, Signal Strength )

#define GRAPH_WIDTH 99 // [ PIXELS ] of the historical chart's width
#define GRAPH_HEIGHT 44 // [ PIXELS ] of the historical chart's height
#define HISTORICAL_DATUM_COUNT 49 // [ NUMBER ] of data points tracked
#define ALTITUDE_HISTORY_INTERMEDIATE_SCALE 4 // [ NUMBER ] of intermediate scale magnitudes to show in the historical plot

// Altitude Tracking
#define MAX_READINGS_LOST_BEFORE_FAIL 3 // [ NUMBER ] : of GPS_READ_INTERVALs to wait before deeming a lost satellite fix; ie. no serial data from the device
#define MIN_ALTITUDE_TRACKED -100.0 // [ METERS ] : minimum value plotted in history
#define MAX_ALTITUDE_TRACKED 4000.0 // [ METERS ] : maximum value plotted in history

// Time
#define TIME_ZONE -8.0 // Pacific
#define GRAPH_TIMELINE 3.6e+6 // [ MILLIS ] of overall historical values to display ( 1 hour )

// Update/Expire Intervals
#define GRAPH_UPDATE_INTERVAL GRAPH_TIMELINE / (( GRAPH_WIDTH / 2 ) - 1) // [ MILLIS ] between graph updates
#define GPS_READ_INTERVAL 1000 // [ MILLIS ] between serial data updates from the GPS module. This is a prediction, meaning we expect the device to report at this rate.
#define SAT_SEARCH_ANIMATION_INTERVAL 500 // [ MILLIS ]
#define SAT_DETAIL_INFO_MAX_AGE 5000 // [ MILLIS ] before tracked satellite details ( Index, Elevation, Azimuth, Signal Strength ) are cleared

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

int lastKnownYear;
byte lastKnownDay, lastKnownMonth, lastKnownHour, lastKnownMinute, lastKnownSecond;
static const char* directions[ NUMBER_OF_CARDINAL_DIRECTIONS ] = {
	"N ", "NNE", "NE", "ENE", "E ", "ESE", "SE", "SSE", "S ", "SSW", "SW", "WSW", "W ", "WNW", "NW", "NNW"
};

bool firstLoad = true;
bool updateTick = false;
bool advanceGraph = false;
bool sensorFailedToReport = false;
bool validSatelliteWindow = false;
bool isValidSatelliteDetailsWindow = false;

unsigned long testEpoch;
time_t lastTimeTicked = 0;
float currentLatitude = 0;
float currentLongitude = 0;
float currentHeading = 0;
float currentAltitude = 0;
float smoothedHeading = 0;
byte lastLine = 0;
byte screen = DEFAULT_SCREEN;
byte numberOfFixedSatellites = 0;

short altitudeHistory[ HISTORICAL_DATUM_COUNT ];
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

void setup(){

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

	enterButton.setDebounceTime( 10 );
	setTime(0, 0, 0, 0, 0, 0);
	clearAltitudeHistory();
	initCustomGPSObjects();

	slowPrintSuccessOrFail( true );

	lastLine += SLOW_TYPE_NEXT_LINE_PADDINGE;
	slowType( F("Online!"), 10, true );
	delay(2000);
	wipeDisplay();
	splashScreen();
}

void loop() {

	listenToButtonPushes();

	if( isGPSDataAvailable() && gps.satellites.value() > 0 ){

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

			updateStatus( validSatelliteWindow );
			validSatelliteWindow = false;
		// Lost Connection : display last valid data until connection is restored
		}else if( !firstLoad && !isWithinGPSUpdateWindow ){

			displayGPSData( false );
		}else if( firstLoad ){

			drawSignalSearch();
		}
	}

	advanceAnimationTicks();
	cacheHistoricalData();

	if( validSatelliteWindow ){

		displayGPSData( true );
	}

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
	if( advanceGraph && !sensorFailedToReport ){

		// altitudeHistory[ 0 ] = map( constrain( currentAltitude, MIN_ALTITUDE_TRACKED, MAX_ALTITUDE_TRACKED ), MIN_ALTITUDE_TRACKED, MAX_ALTITUDE_TRACKED, 0, GRAPH_HEIGHT );
 		altitudeHistory[ 0 ] = constrain( currentAltitude, MIN_ALTITUDE_TRACKED, MAX_ALTITUDE_TRACKED );
		// advanced historical values
		for (int i = HISTORICAL_DATUM_COUNT; i >= 2; i--){

			altitudeHistory[ i - 1 ] = altitudeHistory[ i - 2 ];
		}
	}
}

void clearAltitudeHistory(){

	for( byte i = 0; i <= HISTORICAL_DATUM_COUNT; i++ ){

		altitudeHistory[ i ] = 0;
	}
}

void listenToButtonPushes(){

	enterButton.loop();

	if( enterButton.isPressed() ){

		screen = ( screen < SCREEN_COUNT - 1 ) ? screen+=1 : 0;
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

void updateStatus( bool validSatelliteWindow ){

	display.clearDisplay();
	display.setFont(&Lato_Thin_12);
	display.setTextSize(1);
	display.setTextColor(WHITE);

	if( validSatelliteWindow ){

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
		displayGPSData( false );
	}
}

void populateGPSData(){

	currentLatitude = gps.location.lat();
	currentLongitude = gps.location.lng();
	currentHeading = gps.course.deg();
	currentAltitude = gps.altitude.meters();
	numberOfFixedSatellites = gps.satellites.value();

	bool satelliteDetailsAreValid = totalGPGSVMessages.isUpdated();

	// refresh satellite info
	if( satelliteDetailsAreValid ){

		if( !isValidSatelliteDetailsWindow ){

			isValidSatelliteDetailsWindow = true;
		}
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

	isValidSatelliteDetailsWindow = ( currentMillis - millisLastValidsatelliteInfo < SAT_DETAIL_INFO_MAX_AGE );
	advanceGraph = false;

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

	// lerp relevant values
	smoothedHeading = lerpAngle( smoothedHeading, currentHeading,  EASE_COEFFICIENT );
}

bool isGPSDataAvailable(){

	while ( Serial1.available() ){
		// Each byte of NEMA data must be giving to TinyGPS by using encode(). True is returned when new data has been fully decoded and can be used
		if( gps.encode( Serial1.read() )){

			sensorFailedToReport = false;
			return true;
		}
	}

	sensorFailedToReport = true;
	return false;
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
		adjustTime(TIME_ZONE * SECS_PER_HOUR);
	}

	// should time tick
	if( ( timeStatus()!= timeNotSet && now() != lastTimeTicked ) ){

		lastTimeTicked = now();
		lastKnownDay = day();
		lastKnownMonth = month();
		lastKnownYear = year();
		lastKnownHour = hour();
		lastKnownMinute = minute();
		lastKnownSecond = second();
	}
}

void displayGPSData( bool hasConnection ){

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

		drawSatelliteDetails( hasConnection );
	}else if( screen == 5 ){

		drawCurrentHeading( hasConnection );
	}

	display.display();
}

void drawCurrentHeading( bool hasConnection ){

	display.setTextSize( 1 );
	display.setFont( &Lato_Thin_12 );
	display.setTextColor( WHITE );

	display.setCursor( 0, 20 );
	display.print(F("HEADING"));

	display.setCursor( 5, 50 );
	display.setFont( &Lato_Thin_30 );
	display.print( currentHeading, 0 );

	byte radius = 20;
	display.setFont();
	renderGauge( smoothedHeading , ( DISPLAY_WIDTH / 2) - radius + 30, ( DISPLAY_HEIGHT / 2 ) - radius , radius );
}

void drawSatelliteDetails( bool hasConnection ){

	display.setTextSize(1);
	display.setFont();
	display.setTextColor( WHITE );

	if( isValidSatelliteDetailsWindow ){

		for( int i=0; i < MAX_SATELLITES; ++i ){
			if( sats[ i ].active ){
				display.setCursor( 0, i * 10 );
				display.print( i + 1 ); // satellite number
				display.print(F("|E:"));
				display.print( sats[ i ].elevation );
				display.print(F("' A:"));
				display.print( sats[ i ].azimuth );
				display.print(F("' S:"));
				display.print( sats[ i ].snr );
				display.print(F("dB"));
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

	display.setCursor( xOffset, yOffset + 15 );
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

	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(10,8);
	display.setFont(&Lato_Thin_12);

	display.print( F("ALTITUDE: ") );
	display.setFont(&Lato_Thin_30);
	display.setCursor(10,31);
	display.print( currentAltitude, 1 );
	display.setFont(&Lato_Thin_12);
	display.println( F(" M") );

	display.setCursor(9,41);
	display.print( F("SPEED: ") );
	display.setFont(&Lato_Thin_30);

	float speed = gps.speed.mph();

	if( ( speed != -1 ? speed : 0) < 10 ){

		display.setCursor(47, 64); // single digit
	}else if( (  speed != -1 ? speed : 0 ) < 100 ){

		display.setCursor(28, 64); // double digit
	}else{

		display.setCursor(12, 64); // triple digit
	}

	display.print(  speed != -1 ? speed : 0, 1 );
	display.setFont( &Lato_Thin_12 );
	display.println( "  MPH" );
}

void drawAltitudeHistory( bool hasConnection ){

	display.setTextSize(1);
	display.setFont();
	display.setTextColor( WHITE );
	display.setCursor( 0,0 );
	display.print(F("Altitude:"));
	display.print( currentAltitude, 0 );
	display.println(F("M"));
	renderGraph( altitudeHistory, 0, 10 );
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

	int direction = (int)(( smoothedHeading + 11.25f ) / 22.5f);
	display.print( directions[ direction % 16 ] );
	display.print(F(": "));
	display.println( smoothedHeading, 1 );

	display.setCursor( 120,46 );
	display.write(0xf7);

	// Altitude
	display.setCursor( 0, 57 );

	char altitudeString[ 16 ];
	sprintf( altitudeString, "Altitude:  %d.%02d m", (int)currentAltitude, (int)(fabsf( currentAltitude )*100)%100 );
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
		lastLine += SLOW_TYPE_NEXT_LINE_PADDINGE;
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

void renderGraph( short sensorArray[], int xOffset, int yOffset ){

	int upperBound = 10;
	int lowerBound = MIN_ALTITUDE_TRACKED;
	short whichBoundHasMoreDatums = 0; // negative : low 1, positive: high

	for( byte i = 0; i < HISTORICAL_DATUM_COUNT; i++ ){
		int currentValue = sensorArray[ i ];

		if( currentValue > upperBound ){
			upperBound = currentValue;
		}else if( currentValue < lowerBound ){
			lowerBound = currentValue;
		}

		whichBoundHasMoreDatums += ( currentValue > 0 ) ? 1 : ( currentValue < 0 ) ? -1 : 0;
	}

	upperBound  = ( upperBound * 2 ) <= MAX_ALTITUDE_TRACKED ? upperBound * 2 : MAX_ALTITUDE_TRACKED;
	lowerBound  = ( lowerBound * 2 ) >= MIN_ALTITUDE_TRACKED ? lowerBound * 2: MIN_ALTITUDE_TRACKED;

	byte topOfGraph = yOffset;
	byte bottomOfGraph = topOfGraph + GRAPH_HEIGHT;
	float ratioOfHighestToLowest = ( whichBoundHasMoreDatums > 0 ) ? 4.0 : 2.0; // todo change ratio depending on where there's more plotted points and current altitude is at/below sealevel
	byte splitPoint = bottomOfGraph - ceil( (float)GRAPH_HEIGHT / ratioOfHighestToLowest ); // sea level, 0 altitude
	byte graphLowerBound = bottomOfGraph - splitPoint;
	byte graphUpperBound = splitPoint - yOffset;

	determineUnitAndScaleForBound( &graphBounds[ 0 ], lowerBound );
	determineUnitAndScaleForBound( &graphBounds[ 1 ], upperBound );

	// draw altitude scaled to upper and lower bounds
	for( byte step = 0; step < HISTORICAL_DATUM_COUNT; step++ ){

		short value = sensorArray[ step ];
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

	// advanced historical values

	if( advanceGraph ){

		for( byte step2 = HISTORICAL_DATUM_COUNT; step2 >= 2; step2-- ){

			sensorArray[ step2 - 1 ] = sensorArray[ step2 - 2 ];
		}
	}

	// graph height scale
	display.writeFastVLine(GRAPH_WIDTH + xOffset + 5, yOffset, GRAPH_HEIGHT, WHITE);

	// intermediate magnitudes (upperbound only)
	for( byte i = 0; i < ALTITUDE_HISTORY_INTERMEDIATE_SCALE - 1; i++){

		short step = ( splitPoint - topOfGraph ) / ALTITUDE_HISTORY_INTERMEDIATE_SCALE;
		byte start = topOfGraph + step;
		step = start + (step*i);

		for( byte j = 6; j < GRAPH_WIDTH; j+= 6 ){
			display.drawPixel( j , step, WHITE); // draw all the way across
		}

		display.drawPixel( GRAPH_WIDTH + xOffset + 7 , step, WHITE); // right of graph height scale

		float value =  upperBound - (i + 1.0f) * ( upperBound / ALTITUDE_HISTORY_INTERMEDIATE_SCALE ); // 0 to upper bound
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
	display.setCursor( GRAPH_WIDTH + xOffset - 3 , yOffset - 10 );

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
	display.setCursor( GRAPH_WIDTH + xOffset - 3 , bottomOfGraph + 3 );

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


	if( value >= 1000 || value >= 100 ){

		if( value >= 1000 ){
			bound->denominator = 1000;
			bound->isInteger = true;
		}else{
			bound->denominator = 1000.0f;
			bound->isInteger = false;
		}
		strcpy( bound->unit, "KM" );
	}else{

		bound->denominator = 1;
		bound->isInteger = true;
		strcpy( bound->unit, "M" );
	}
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

	byte xCenter = radius;
	byte yCenter = radius;
	byte needleLength = ( radius * 1.3f );

	int x = -needleLength / 2;
	byte y = 0;
	byte x1 = needleLength / 2;
	byte y1 = 0;
	byte cx = ( x + x1 ) / 2;
	byte cy = ( y + y1 ) / 2;

	double headingTheta = -angle * ( M_PI_LONG / 180.0L ) - ( M_PI_LONG / 2.0L );


	// draw border of the gauge
	display.drawCircle( xCenter + xOffset, yCenter + yOffset, radius, WHITE );

	// gauge ticks 30 degree increments
	for( double angle = 0.0; angle < 360.0; angle += 360.0 / 16.0 ){

		display.drawPixel( xCenter + xOffset + (( 3 + radius ) * -cos( angle * (M_PI_LONG / 180.0L) )) , yCenter + yOffset + (( 3 + radius ) * -sin( angle * (M_PI_LONG / 180.0L) )), WHITE );
	}

	for( int angle = 0; angle < 360; angle+= 360 / 8 ){

		int direction = (int)(( angle + 11.25f ) / 22.5f);
		float theta = ( angle * ( M_PI_LONG / 180.0f ));

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
			trimX = -1;
		}else if( direction == 0 ){ // N Label

			trimY = -1;
			trimX = -1;
		}

		theta -= ( M_PI_LONG / 2 );

		byte x = xCenter + xOffset + trimX + (( 10 + radius ) * cos( theta ));
		byte y = yCenter + yOffset + trimY + (( 10 + radius ) * sin( theta ));

		display.setCursor( x, y );
		display.print( directions[ direction % 16 ] );
	}

	// start of needle
	float rotX = (  -(x - cx) * cos( headingTheta ) + (y - cy) * sin( headingTheta ) ) + cx;
	float rotY = ( (x - cx) * sin( headingTheta ) + (y - cy) * cos( headingTheta ) ) + cy;

	//end of needle
	float rotXEnd = ( -(x1 - cx) * cos( headingTheta ) + (y1 - cy) * sin( headingTheta ) ) + cx;
	float rotYEnd = ( (x1 - cx) * sin( headingTheta ) + (y1 - cy) * cos( headingTheta ) ) + cy;

	display.drawCircle(rotX + xOffset + radius, rotY + yOffset + radius,2, WHITE);
	drawRotatedTriangle( -1, (rotXEnd + xOffset + radius), (rotYEnd + yOffset + radius), headingTheta );

	display.drawLine(rotX + xOffset + radius, rotY + yOffset + radius, rotXEnd + xOffset + radius, rotYEnd + yOffset + radius, WHITE);
}

void drawRotatedTriangle( int sign, int xOffset, int yOffset, float theta ){

	int tX = 5 * sign;
	byte tY = 0;

	byte t1X = 0;
	int t1Y = 2 * sign;

	byte t2X = 0;
	int t2Y = -2 * sign;

	display.drawTriangle(
		xOffset + (  tX * cos( theta ) + tY * sin( theta ) ),
		yOffset + ( -tX * sin( theta ) + tY * cos( theta ) ),

		xOffset + (  t1X * cos( theta ) + t1Y * sin( theta ) ),
		yOffset + ( -t1X * sin( theta ) + t1Y * cos( theta ) ),

		xOffset + (  t2X * cos( theta ) + t2Y * sin( theta ) ),
		yOffset + ( -t2X * sin( theta ) + t2Y * cos( theta ) ),
	WHITE);
}

void wipeDisplay(){

	display.clearDisplay(); // remove library banner
	display.fillScreen(BLACK); // I see a red door...
	display.display(); // because fillScreen is misleading
}


double mapf(double x, double in_min, double in_max, double out_min, double out_max){
  return (double)(x - in_min) * (out_max - out_min) / (double)(in_max - in_min) + out_min;
}

float clampFloat(float value, float min, float max){
	if( value < min ){

		value = min;
	}else if( value > max ){

		value = max;
	}

	return value;
}

float clamp( float value ){
	if( value < 0.0f ){

		return 0.0F;
	}else if( value > 1.0f ){

		return 1.0f;
	}else{

		return value;
	}
}

float Repeat( float t, float length ){

	return clampFloat( t - floor( t / length ) * length, 0.0f, length );
}

float lerpAngle(float a, float b, float t){

	float delta = Repeat( ( b - a ), 360.0f );

	if( delta > 180.0f ){
		delta -= 360.0f;
	}
	return a + delta * clamp(t);
}
