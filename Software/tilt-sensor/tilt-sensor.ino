// Graphical Tilt Data ( Pitch, Roll, Acceleration )
// - OLED display
// - Self-Calibrating
// - Yaw drifts, needs compass data to be meaningful
#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// #define DEBUG 1
#define M_PI_LONG 3.141592653589793238462643383279502884L
#define OLED_RESET -1 // [ NUMBER ] digital read pin to reset the display | NOT IMPLEMENTED
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define ENTER_BUTTON_PIN 5 // [ NUMBER ] digital read pin to interact with device

#define DISPLAY_WIDTH 128 // [ PIXELS ] number of available horizontal pixels
#define DISPLAY_HEIGHT 64 // [ PIXELS ] number of available vertical pixels
#define LAST_LINE_INDENT 10
#define SHORT_PRESS_TIME 1000 // [ MILLISECONDS ] less than for short
#define LONG_PRESS_TIME 1000 //[ MILLISECONDS ] greater than for long press

#define HEIGHT_OFFSET 46
#define READOUT_Y_OFFSET HEIGHT_OFFSET + 10
#define READOUT_X_OFFSET 10

Adafruit_SSD1306 display( DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, OLED_RESET );

MPU6050 mpu;

byte lastLine = 0;
// Acceleration lerp values
float displayAccelerationX = 0.0; // [ LSB ] current X component of acceleration vector of the displayed cursor
float displayAccelerationY = 0.0; // [ LSB ] current Y component of acceleration vector of the displayed cursor
float targetAccelerationX = 0.0; // [ LSB ] cached maximum X component of acceleration vector to which the displayed cursor will move
float targetAccelerationY = 0.0; // [ LSB ] cached maximum Y component of acceleration vector to which the displayed cursor will move

// whether or not the current displayAcceleration has lerped to the targetAcceleration
bool isApproachingX = false;
bool isApproachingY = false;

// MPU Status
bool dmpReady = false; // set true if DMP init was successful
uint8_t fifoBuffer[64]; // FIFO storage buffers

// Device Orientation Vectors
Quaternion q; // [w, x, y, z] quaternion container
VectorInt16 aa; // [x, y, z] accel sensor measurements
VectorInt16 aaReal; // [x, y, z] gravity-free accel sensor measurements
// VectorInt16 aaWorld; // [x, y, z] world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z] gravity vector
// float euler[ 3 ]; // [psi, theta, phi] Euler angle container
float ypr[ 3 ] = {0}; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

// This should be manually set to 0 when a brand new sensor is first used, to calibrate on first run
byte calibrationMode = 3; // [ 0 ] : uncalibrated, [ 1 ] : calibration edit mode, [ 2 ] : calibrating, [ 3 ] : calibrated

int lastState = LOW;  // the previous state from the input pin
bool buttonRawState;     // the current reading from the input pin
unsigned long pressedTime  = 0;
unsigned long releasedTime = 0;
bool isPressing = false;
bool isLongDetected = false;

struct SensorOffsets {
	int16_t xOffset;
	int16_t yOffset;
	int16_t zOffset;
	int16_t xGyroOffset;
	int16_t yGyroOffset;
	int16_t zGyroOffset;
};

SensorOffsets sensorOffsets = { 0, 0, 0, 0, 0, 0 };

void(* resetArduino ) (void) = 0;

void dmpDataReady() {
	mpuInterrupt = true;
}

void setup() {

	bool hasIMUReportedSuccessfully = false;
	byte hasIMUInitialized;

	Wire.begin();
	Wire.setClock( 400000L ); // 400kHz I2C clock. Comment this line if having compilation difficulties
	Wire.setWireTimeout( 4000, true );

	display.begin( SSD1306_SWITCHCAPVCC, 0x3C );  // initialize the OLED and set the I2C address to 0x3C (for the 128x64 OLED)
	wipeDisplay();

	display.setTextSize(1);
	display.setTextColor(WHITE);

	#ifdef DEBUG
	Serial.begin(9600);
	while (!Serial);
	#endif

	slowType( F("IMU STARTING >"), 10, true );
	mpu.initialize();
	pinMode( INTERRUPT_PIN, INPUT );
	pinMode(ENTER_BUTTON_PIN, INPUT_PULLUP);

	// verify connection
	hasIMUReportedSuccessfully = mpu.testConnection();
	slowType( F("IMU REPORTS:"), 10, true );
	slowPrintSuccessOrFail( hasIMUReportedSuccessfully );

	// load and configure the DMP
	slowType( F("DMP MOUNT:"), 10, true );
	hasIMUInitialized = mpu.dmpInitialize();

	if( hasIMUReportedSuccessfully && hasIMUInitialized == 0 ){

		slowPrintSuccessOrFail( true );

		// SETUP : calibrate on setup for debug/new hardware
		// mpu.CalibrateAccel( 6 );
		// mpu.CalibrateGyro( 6 );
		// mpu.PrintActiveOffsets();

		// Generate offsets and calibrate our MPU6050
		slowType( F("LOAD OFFSETS:"), 10, true );

		// Retrieve Offsets from memory and set on Device
		EEPROM.get( 0, sensorOffsets );
		mpu.setXAccelOffset( sensorOffsets.xOffset );
		mpu.setYAccelOffset( sensorOffsets.yOffset );
		mpu.setZAccelOffset( sensorOffsets.zOffset );
		mpu.setXGyroOffset( sensorOffsets.xGyroOffset );
		mpu.setYGyroOffset( sensorOffsets.yGyroOffset );
		mpu.setZGyroOffset( sensorOffsets.zGyroOffset );

		slowPrintSuccessOrFail( true );

		slowType( F("DMP ENABLE:"), 10, true );
		mpu.setDMPEnabled( true );
		slowPrintSuccessOrFail( true );

		lastLine -= LAST_LINE_INDENT * 2;
		dmpReady = true;

		delay( 1000 );
		display.clearDisplay();

		// slowType( F("Online !") , 10, true );
		attachInterrupt( digitalPinToInterrupt( INTERRUPT_PIN ), dmpDataReady, RISING );

		delay( 1000 );
	} else {
		// Fail mode
		slowPrintSuccessOrFail( false );
		delay( 3000 );

		// if MPU configuration fails, only prompt to reset the device
		wipeDisplay();
		slowType( F("FAILED TO COME ONLINE!"), 10, false );
		display.setCursor(0, LAST_LINE_INDENT * 2 );
		slowType( F("SHORT PRESS TO RESET!"), 10, false );
	}
}

void loop() {

	listenToButton();

	if( !dmpReady ){

		return;
	}

	// read a packet from FIFO
	if( mpu.dmpGetCurrentFIFOPacket( fifoBuffer )){

		// Compute Quaternion -> Euler Angles
		mpu.dmpGetQuaternion( &q, fifoBuffer );
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity( &gravity, &q );
		mpu.dmpGetYawPitchRoll( ypr, &q, &gravity );
		mpu.dmpGetLinearAccel( &aaReal, &aa, &gravity );
		// mpu.dmpGetLinearAccelInWorld( &aaWorld, &aaReal, &q );
		mpu.resetFIFO();
	}
	display.clearDisplay();

	render();

	display.display();
}

void listenToButton(){


	buttonRawState = digitalRead( ENTER_BUTTON_PIN );

	if( lastState == HIGH && buttonRawState == LOW ){ // pressed

		pressedTime = millis();
		isPressing = true;
		isLongDetected = false;
	}else if( lastState == LOW && buttonRawState == HIGH ){ // released

		isPressing = false;
		releasedTime = millis();

		long pressDuration = releasedTime - pressedTime;

		if( pressDuration < SHORT_PRESS_TIME ){

			buttonIsPressed( false ); // short
		}
	}

	if( isPressing == true && isLongDetected == false ){

		long pressDuration = millis() - pressedTime;

		if( pressDuration > LONG_PRESS_TIME ) {

			buttonIsPressed( true ); // long
			isLongDetected = true;
		}
	}
	// save the the last state
	lastState = buttonRawState;
}

void buttonIsPressed( bool isLongPress ){

	if( isLongPress ){ // already editing
		if( calibrationMode == 1 ){
			calibrationMode = 2;
		}
	}else{

		if( !dmpReady ){ // short press ( from failed state)

			resetArduino();
		} else if( calibrationMode == 3 ){ // short press from calibrated state

			calibrationMode = 1;
		}else if( calibrationMode == 1 ){ // short press, edit mode -> leave

			calibrationMode = 3;
		}
	}
}

void render(){

	if( calibrationMode == 1 ){

		display.setCursor( 10,20 );
		display.print(F("SENSOR CALIBRATION"));

		display.setCursor( 10,40 );
		display.print(F("LONG PRESS"));

		display.setCursor( 10,50 );
		display.print(F("TO CONTINUE"));
	}else if( calibrationMode == 2 ){

		lastLine = 20;
		display.clearDisplay();
		slowType( F("CALIBRATING:"), 10, true );

		// Calibrate Device
		mpu.CalibrateAccel( 6 );
		mpu.CalibrateGyro( 6 );

		// mpu.PrintActiveOffsets();

		// Save Offsets to Memory
		sensorOffsets = {
			mpu.getXAccelOffset(),
			mpu.getYAccelOffset(),
			mpu.getZAccelOffset(),
			mpu.getXGyroOffset(),
			mpu.getYGyroOffset(),
			mpu.getZGyroOffset()
		};

		EEPROM.put(0, sensorOffsets );

		slowPrintSuccessOrFail( true );
		calibrationMode = 3;
		delay( 2000 );
	}else{

		// int roll = round(( ypr[ 1 ] * 180 / M_PI_LONG ));
		// int pitch = round(( ypr[ 2 ] * 180 / M_PI_LONG ));
		// int yaw = round(( ypr[ 0 ] * 180 / M_PI_LONG ));

		drawHeader();

		// Gauges | type : roll [ 1 ] : acceleration : pitch [ 2 ]
		renderGauge( -round( ypr[ 1 ] * 180.0L / M_PI_LONG ), 6, 8, 0 );
		plotAcceleration( aaReal, 49, 7 );
		renderGauge( -round( ypr[ 2 ] * 180.0L / M_PI_LONG ), 93, 8, 1 );


		// renderGauge( -yaw + 90, 49, 8, 2 );

		// Numeric : Roll : Acceleration : Pitch
		printOffsetText( READOUT_X_OFFSET - 6,  READOUT_Y_OFFSET, ( ypr[ 1 ] * 180.0L / M_PI_LONG ));
		printOffsetText( READOUT_X_OFFSET + 39,  READOUT_Y_OFFSET, ( aaReal.y / 16384.0L ) * 9.80L ); // 2G scale ( +- 16384 LSB ) -> m/2^2
		printOffsetText( READOUT_X_OFFSET + 82,  READOUT_Y_OFFSET, ( ypr[ 2 ] * 180.0L / M_PI_LONG ));
	}
}

void slowPrintSuccessOrFail( bool condition ){

	condition ? slowType( F(" OK"), 50, false ) : slowType( F(" FAIL"), 50, false );
}

void drawHeader(){

	// Roll
	display.setCursor( READOUT_X_OFFSET, HEIGHT_OFFSET );
	display.print( F("Roll") );

	// Acceleration
	display.setCursor( READOUT_X_OFFSET + 45, HEIGHT_OFFSET );
	display.print( F("M/S^2") );

	// Pitch
	display.setCursor( READOUT_X_OFFSET + 85, HEIGHT_OFFSET );
	display.print( F("Pitch") );
}

void printOffsetText( byte xOffset, byte yOffset, float datum ){

	bool isNegative = datum <= -0.1;
	datum = abs( datum );

	if( isNegative ){

		display.setCursor( xOffset - 1, yOffset );
		display.print( F("-") );
	}

	if( datum < 10 ){

		xOffset += 10; // single digit
	}else if( datum < 100 ){

		xOffset += 4; // double digit
	}

	display.setCursor( xOffset, yOffset );
	display.print( datum, 1 );
}

void plotAcceleration( VectorInt16 acceleration, byte xOffset, byte yOffset ){

	byte width = 36;
	byte tickCount = 8;
	byte xCenter = xOffset + ( width / 2 );
	byte yCenter = yOffset + ( width / 2 );
	short accelerationScale = 500.0; // 1G (8192 LSB) | 0.5G (5461 LSB) | 0.25G (2730.5)

	display.drawRect( xOffset, yOffset, width, width, WHITE );

	for( byte i = 1; i <= tickCount; i++ ){

		display.drawPixel( xCenter, yOffset + (i * (width / tickCount)), WHITE );
		display.drawPixel( xOffset + (i * (width / tickCount)), yCenter, WHITE );
	}

	smoothAndApproach( targetAccelerationX, displayAccelerationX, isApproachingX, -acceleration.x );
	smoothAndApproach( targetAccelerationY, displayAccelerationY, isApproachingY, -acceleration.y );
	// displayAccelerationX = lerp( displayAccelerationX, acceleration.x, 0.1 );
	// displayAccelerationY = lerp( displayAccelerationY, acceleration.y, 0.1 );


	// indicator
	display.drawCircle(
		// normalized to 1G (8192 LSB) acceleration, scaled to half the width, subtract indicator radius to center
		xCenter - 9 + (( displayAccelerationX - -accelerationScale ) / ( accelerationScale - -accelerationScale ) * ( width / 2 )),
		yCenter - 9 + (( displayAccelerationY - -accelerationScale ) / ( accelerationScale - -accelerationScale ) * ( width / 2 )),
		3,
		WHITE
	);
}

void smoothAndApproach( float& target, float& current, bool& isApproachingTarget, float immediate ){

	// new immediate target
	if( abs( immediate ) > 0 && !isApproachingTarget ){

		isApproachingTarget = true;
		target = immediate;
	}

	// new immediate target is greater
	if( abs( immediate ) > abs( current ) ){

		target = immediate;
	}

	if( isEqual( current, target ) ){

		target = immediate;
		isApproachingTarget = false;
	}

	current = lerp( current, target, 0.15 );
}

void renderGauge( double angle, byte xOffset, byte yOffset, byte type ) {
	// type : roll [ 0 ] pitch [ 1 ] yaw [ 2 ]
	byte radius = 16;
	byte xCenter = radius;
	byte yCenter = radius;
	byte needleLength = ( type == 1 ) ? ( radius * 2 ) - 12 : ( radius * 2 ) + 8;

	short x = -needleLength / 2;
	byte y = 0;
	byte x1 = needleLength / 2;
	byte y1 = 0;
	byte cx = ( x + x1 ) / 2;
	byte cy = ( y + y1 ) / 2;

	// draw border of the gauge
	display.drawCircle( xCenter + xOffset, yCenter + yOffset, radius, WHITE );

	// gauge ticks 30 degree increments
	for( byte angle = 0; angle <= 180; angle += 30 ){

		display.drawPixel( xCenter + xOffset + (( 3 + radius ) * -cos( angle * (M_PI_LONG / 180.0L) )) , yCenter + yOffset + (( 3 + radius ) * -sin( angle * (M_PI_LONG / 180.0L) )), WHITE );
	}

	// start of needle
	float rotX = (  (x - cx) * cos( angle * ( M_PI_LONG / 180.0L ) ) + (y - cy) * sin( angle * ( M_PI_LONG / 180.0L ) ) ) + cx;
	float rotY = ( -(x - cx) * sin( angle * ( M_PI_LONG / 180.0L ) ) + (y - cy) * cos( angle * ( M_PI_LONG / 180.0L ) ) ) + cy;

	//end of needle
	float rotXEnd = (  (x1 - cx) * cos( angle * ( M_PI_LONG / 180.0L ) ) + (y1 - cy) * sin( angle * ( M_PI_LONG / 180.0L ) ) ) + cx;
	float rotYEnd = ( -(x1 - cx) * sin( angle * ( M_PI_LONG / 180.0L ) ) + (y1 - cy) * cos( angle * ( M_PI_LONG / 180.0L ) ) ) + cy;

	if( type == 1 ){

		// if( type == 1 ){

			display.drawCircle(rotX + xOffset + radius, rotY + yOffset + radius,2, WHITE);
			drawRotatedTriangle( 1, (rotXEnd + xOffset + radius), (rotYEnd + yOffset + radius), angle * ( M_PI_LONG / 180.0L ));
		// }
	}else if( type == 0 ){

		display.drawCircle(xOffset + radius , yOffset + radius,4, WHITE);
		display.drawCircle(xOffset + radius + ( -6 * sin( angle * ( M_PI_LONG / 180.0L ) )), yOffset + radius + ( -6 * cos( angle * ( M_PI_LONG / 180.0L ) )),2, WHITE);

	}else if( type == 1 ){

		display.drawCircle(rotXEnd + xOffset + radius, rotYEnd + yOffset + radius,2, WHITE);
	}

	display.drawLine(rotX + xOffset + radius, rotY + yOffset + radius, rotXEnd + xOffset + radius, rotYEnd + yOffset + radius, WHITE);
}

void drawRotatedTriangle( int sign, int xOffset, int yOffset, float theta ){

	int tX = 3 * sign;
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

float lerp( float a, float b, float num ){

	return a + num * (b - a);
}

void slowType( String text, int delayTime, bool newLine ){

	if( newLine ){
		display.setCursor(0, lastLine );
		lastLine += LAST_LINE_INDENT;
	}

	for( byte i = 0; i < text.length(); i++ ){

		display.print( text[ i ] );
		display.display();
		delay( delayTime );
	}
}

bool isEqual(float number, float secondNumber ){
	return abs(number - secondNumber ) <= 1e-2 * abs(number);
}
