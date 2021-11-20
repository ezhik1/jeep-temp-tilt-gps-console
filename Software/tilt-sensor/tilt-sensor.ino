// Graphical Tilt Data ( Pitch, Roll, Acceleration )
// - OLED display
// - Self-Calibrating
// - Yaw drifts, needs compass data to be meaningful
#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// #define DEBUG 1
#define M_PI_LONG 3.141592653589793238462643383279502884L
#define OLED_RESET 5
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

#define DISPLAY_WIDTH 128 // [ PIXELS ] number of available horizontal pixels
#define DISPLAY_HEIGHT 64 // [ PIXELS ] number of available vertical pixels
#define LAST_LINE_INDENT 10

Adafruit_SSD1306 display( DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, OLED_RESET );
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
// uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
// uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
// uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
// uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffers

byte lastLine = 0;
float displayAccelerationX = 0.0;
float displayAccelerationY = 0.0;
float targetAccelerationX = 0.0;
float targetAccelerationY = 0.0;
bool isApproachingX = false;
bool isApproachingY = false;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[ 3 ];         // [psi, theta, phi]    Euler angle container
float ypr[ 3 ];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}

void setup() {

	bool dmpReady = false;
	bool hasIMUReportedSuccessfully = false;
	byte hasIMUInitialized;

	// Fastwire::setup(400, true);
	Wire.begin();
	Wire.setClock( 400000 ); // 400kHz I2C clock. Comment this line if having compilation difficulties
	Wire.setWireTimeout( 4000, true );

	display.begin( SSD1306_SWITCHCAPVCC, 0x3C );  // initialize the OLED and set the I2C address to 0x3C (for the 128x64 OLED)
	wipeDisplay();

	display.setTextSize(1);
	display.setTextColor(WHITE);

	#ifdef DEBUG
	Serial.begin(9600);
	while (!Serial); // wait for Leonardo enumeration, others continue immediately
	#endif

	slowType( F("IMU Starting >"), 10, true );
	mpu.initialize();
	pinMode( INTERRUPT_PIN, INPUT );

	// verify connection
	hasIMUReportedSuccessfully = mpu.testConnection();
	slowType( F("IMU Reports: "), 10, true );
	slowPrintSuccessOrFail( hasIMUReportedSuccessfully );

	hasIMUInitialized = mpu.dmpInitialize();
	// load and configure the DMP
	slowType( F("DMP Online: "), 10, true );

	if( hasIMUReportedSuccessfully && hasIMUInitialized == 0 ){

		slowPrintSuccessOrFail( true );

		// Generate offsets and calibrate our MPU6050
		slowType( F("Calibrate Gyro: "), 10, true );
		mpu.CalibrateAccel( 6 );
		mpu.CalibrateGyro( 6 );
		// mpu.PrintActiveOffsets();
		slowPrintSuccessOrFail( true );

		slowType( F("Enable DMP: "), 10, true );
		mpu.setDMPEnabled( true );
		slowPrintSuccessOrFail( true );

		lastLine -= LAST_LINE_INDENT * 2;
		dmpReady = true;

		delay( 1000 );
		display.clearDisplay();

		slowType( "Online !", 10, true );
		attachInterrupt( digitalPinToInterrupt( INTERRUPT_PIN ), dmpDataReady, RISING );

		delay( 1000 );
	} else {
		// Fail mode
		slowPrintSuccessOrFail( false );
	}

	// if programming failed, don't try to do anything
	while( !dmpReady ){

	};

	wipeDisplay();
}

void loop() {


	// read a packet from FIFO
	if( mpu.dmpGetCurrentFIFOPacket( fifoBuffer )){

		// Compute Quaternion -> Euler Angles
		mpu.dmpGetQuaternion( &q, fifoBuffer );
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity( &gravity, &q );
		mpu.dmpGetYawPitchRoll( ypr, &q, &gravity );
		mpu.dmpGetLinearAccel( &aaReal, &aa, &gravity );
		mpu.dmpGetLinearAccelInWorld( &aaWorld, &aaReal, &q );
		mpu.resetFIFO();
	}
	render();
}

void render(){

	#define HEIGHT_OFFSET 47
	#define READOUT_Y_OFFSET HEIGHT_OFFSET + 10
	#define READOUT_X_OFFSET 10

	// int roll = round(( ypr[ 1 ] * 180 / M_PI_LONG ));
	// int pitch = round(( ypr[ 2 ] * 180 / M_PI_LONG ));
	// int yaw = round(( ypr[ 0 ] * 180 / M_PI_LONG ));

	display.clearDisplay();
	drawHeader();

	// Gauges | type : roll [ 0 ] pitch [ 1 ] yaw [ 2 ]
	renderGauge( -round( ypr[ 1 ] * 180.0L / M_PI_LONG ), 6, 8, 0 );
	renderGauge( -round( ypr[ 2 ] * 180.0L / M_PI_LONG ), 93, 8, 1 );
	plotAcceleration( aaWorld, 49, 7 );

	// renderGauge( -yaw + 90, 49, 8, 2 );

	// Numeric : Roll : Acceleration : Pitch
	printOffsetText( READOUT_X_OFFSET - 6,  READOUT_Y_OFFSET, ( ypr[ 1 ] * 180.0L / M_PI_LONG ));
	printOffsetText( READOUT_X_OFFSET + 82,  READOUT_Y_OFFSET, ( ypr[ 2 ] * 180.0L / M_PI_LONG ));
	printOffsetText( READOUT_X_OFFSET + 39,  READOUT_Y_OFFSET, ( aaWorld.y / 16384.0L ) * 9.80L ); // 2G scale ( +- 16384 LSB ) -> m/2^2

	display.display();
}

void slowPrintSuccessOrFail( bool condition ){

	condition ? slowType( F("OK"), 50, false ) : slowType( F("FAIL"), 50, false );
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

	#define width 36
	#define tickCount 8
	#define xCenter xOffset + ( width / 2 )
	#define yCenter yOffset + ( width / 2 )
	#define accelerationScale 5461 // 1G (8192 LSB) | 0.5G (5461 LSB) ? 0.25G (2730.5)

	display.drawRect( xOffset, yOffset, width, width, WHITE );

	for( byte i = 1; i <= tickCount; i++ ){

		display.drawPixel( xCenter, yOffset + (i * (width / tickCount)), WHITE );
		display.drawPixel( xOffset + (i * (width / tickCount)), yCenter, WHITE );
	}

	smoothAndApproach( targetAccelerationX, displayAccelerationX, isApproachingX, -acceleration.x );
	smoothAndApproach( targetAccelerationY, displayAccelerationY, isApproachingY, -acceleration.y );

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
	#define radius 16
	#define xCenter radius
	#define yCenter radius
	byte needleLength = ( type == 1 ) ? ( radius * 2 ) - 12 : ( radius * 2 ) + 8;

	#define x -needleLength / 2
	#define y  0
	#define x1 needleLength / 2
	#define y1 0
	#define cx ( x + x1 ) / 2
	#define cy ( y + y1 ) / 2

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

		if( type == 1 ){

			display.drawCircle(rotX + xOffset + radius, rotY + yOffset + radius,2, WHITE);
			drawRotatedTriangle( 1, (rotXEnd + xOffset + radius), (rotYEnd + yOffset + radius), angle * ( M_PI_LONG / 180.0L ));
		}
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
	#define tY  0

	#define t1X 0
	int t1Y = 2 * sign;

	#define t2X 0
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
