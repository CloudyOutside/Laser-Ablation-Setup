#include <AccelStepper.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Encoder.h>

// stepper 1 uses 1/8 microsteps (1600 full rev)
#define STEPPER_1_STEPS 3200
// stepper 2 uses 1/8 microsteps (1600 full rev)
#define STEPPER_2_STEPS 3200

#define STEPPER_2_STEPS_PER_MM 145.5

//#define STEPPER_2_SAMPLE_MM_DIAMETER 70

// encoder and stepper 1 pins
#define STEPPER1_STEP_PIN A0
#define STEPPER1_DIR_PIN A1
#define ENCODER1_CLOCK_PIN 2
#define ENCODER1_DT_PIN 7
#define ENCODER1_SW_PIN 6
Encoder encoder1(7,2);
long enc1Value = 0;

// encoder and stepper 2 pins
#define STEPPER2_STEP_PIN 8
#define STEPPER2_DIR_PIN A2
#define ENCODER2_CLOCK_PIN 3
#define ENCODER2_DT_PIN 5
#define ENCODER2_SW_PIN 4
Encoder encoder2(5,3);
long enc2Value = 0;

// LCD pins
#define DISPLAY_SCL_PIN A5
#define DISPLAY_SDA_PIN A4

// button switch pin
#define BTN_SWH_PIN A3

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET 0
Adafruit_SSD1306 lcd(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// create a variable to hold value of button switch
// btwn %spd and pos modes
int btnSwhVal = 0;

// create variable to set speed mode on/off
boolean isSpeedMode = true;

// create variables for ***STEPPER 1***, specifying
// the number of steps of the motor and the pins it is
// attached to
boolean stepper1Enabled = false;
double stepper1RPM = 1; // ROTATING MOTOR SPEED (starting RPM)
double stepper1StepsPerSec = 0;
int stepper1Position = 0;
int previousStepper1Position = 0;
AccelStepper stepper1(1, A0, A1);
// create variables for ***STEPPER 2***, specifying
// the number of steps of the motor and the pins it is
// attached to
boolean stepper2Enabled = false;
double stepper2RPM = 0.5; // BELT MOTOR SPEED
double stepper2MMPerSec = 0;
double stepper2StepsPerSec = 0;
int stepper2Position = 0;
int previousStepper2Position = 0;
AccelStepper stepper2(1, 8, A2);


// create long variables to hold the values of
// last button press for button 1, button 2,
// button switch, and the last LCD update
unsigned long lastButton1Press = 0;
unsigned long lastButton2Press = 0;
unsigned long lastButtonSwhPress = 0;
unsigned long lastLCD = 0;
unsigned long numStepsLastDir = 0;

// variable holds microseconds since last stepper2 step
unsigned long lastStepper2Step = 0;

// create an instance of the LCD I2C class
//LiquidCrystal_I2C lcd(0x27, 16, 2);

String modeLetter = "S";

void setup() {

  // set the pins connected encoders
  // clock, dt, and sw pins to inputs
  pinMode(ENCODER1_CLOCK_PIN, INPUT);
  pinMode(ENCODER1_DT_PIN, INPUT);
  pinMode(ENCODER1_SW_PIN, INPUT_PULLUP);
  pinMode(ENCODER2_CLOCK_PIN, INPUT);
  pinMode(ENCODER2_DT_PIN, INPUT);
  pinMode(ENCODER2_SW_PIN, INPUT_PULLUP);

  // set the state of step and dir pins to outputs
  pinMode(STEPPER1_STEP_PIN, OUTPUT);
  pinMode(STEPPER1_DIR_PIN, OUTPUT);
  digitalWrite(STEPPER1_DIR_PIN, HIGH);
  stepper1.setMaxSpeed(12800);
  stepper1.setSpeed(0);
  pinMode(STEPPER2_STEP_PIN, OUTPUT);
  pinMode(STEPPER2_DIR_PIN, OUTPUT);
  digitalWrite(STEPPER2_DIR_PIN, HIGH);
  stepper2.setMaxSpeed(16000);
  stepper2.setSpeed(0);

  // set state of button pin to input pullup
  pinMode(BTN_SWH_PIN, INPUT_PULLUP);
  
  // create an interrupt sequence for when encoders
  // clock pins are in the falling cycle
   attachInterrupt(digitalPinToInterrupt(ENCODER1_CLOCK_PIN), readEncoder1, FALLING);
   attachInterrupt(digitalPinToInterrupt(ENCODER2_CLOCK_PIN), readEncoder2, FALLING);

  // set starting speeds for steppers
  setStepsPerSec();
  mmPerSec();

  // set up LCD
  lcd.begin(SSD1306_SWITCHCAPVCC, 0x3c);
  lcd.clearDisplay();
  lcd.setTextSize(2);
  lcd.setTextColor(SSD1306_WHITE);
  lcd.setCursor(0, 0);
  lcd.println("\nRPM:" + (String)stepper1RPM);
  lcd.println("mm/s:" + (String)stepper2MMPerSec);
  lcd.setCursor(110,0);
  lcd.println(modeLetter);
  lcd.display();
}

void loop() {

  if(btnSwhVal % 2 == 0){
    // set to SPEED MODE
    if(stepper1Enabled){
      stepper1SpeedMode();
      stepper1.runSpeed();
    }
    if(stepper2Enabled){
      stepper2SpeedMode();
      stepper2.runSpeed();
    }
  }
  else {
    // set to POSITION MODE
    stepper1PositionMode();
    stepper2PositionMode();
    stepper1.runSpeed();
    stepper2.runSpeed();
  }

  // BUTTON SWITCH FOR POS AND SPD MODES:
  // read if button has been pressed
  checkBtnSwhState();
  
  // ENCODER1 BUTTON:
  // Read the state of encoder1 button
	checkBtn1State();
  
  // ENCODER2 BUTTON:
  // Read the state of encoder2 button
	checkBtn2State();
	
  if (!(stepper1Enabled || stepper2Enabled)){
	  // update LCD screen with speed and position
	  displayLCD();
    stepper1.setSpeed(0);
    stepper2.setSpeed(0);
  }
  if (!(stepper1Enabled || stepper2Enabled)){
	  // update LCD screen with speed and position
	  readEncoder1();
    readEncoder2();
  }

}

// ROTATING STEPPER ENCODER:
// Reads if encoder has been turned clockwise or counter-clockwise
// Updates relevant values based on which mode is set
void readEncoder1() {
  long newEnc1Value = encoder1.read();
  
  // encoder has been rotated clockwise
  if (newEnc1Value > enc1Value) {
     // updates position mode values
    if (!(isSpeedMode)){
      previousStepper1Position = stepper1Position;
      stepper1Position+=10;
    }
    //updates speed mode values
    else {
      // make sure only changes are made when not enabled
      if (!(stepper1Enabled || stepper2Enabled)){
        stepper1RPM += 0.1;
        updateStepper1StepsPerSec();
      }
    }
    enc1Value = newEnc1Value;
  }
  // encoder has been rotated counter-clockwise
  else if (newEnc1Value < enc1Value) {
    // updates position mode values
    if (!(isSpeedMode)) {
      previousStepper1Position = stepper1Position;
      stepper1Position-=10;
    }
    // updates speed mode values
    else {
      // make sure only changes are made when not enabled
      if (!(stepper1Enabled || stepper2Enabled)){
        stepper1RPM -= 0.1;
        updateStepper1StepsPerSec();
      }
    }
    enc1Value = newEnc1Value;
  }
}

// BELT STEPPER ENCODER:
// Reads if encoder has been turned clockwise or counter-clockwise
// Updates relevant values depending on which mode is set
void readEncoder2() {

  long newEnc2Value = encoder2.read();

  // encoder has been rotated clockwise
  if (newEnc2Value > enc2Value) {
    // updates position mode values
    if (isSpeedMode) {
      // make sure only changes are made when not enabled
      if (!(stepper2Enabled || stepper2Enabled)){
        stepper2MMPerSec += 0.1;
        updateStepper2StepsPerSec();
        //updateStepper2RPM();
      }
    }
    // updates speed mode values
    else {
      previousStepper2Position = stepper2Position;
      stepper2Position+=10;
    }
    enc2Value = newEnc2Value;
  }
  // encoder has been rotated counter-clockwise
  else if (newEnc2Value < enc2Value) {
    // updates position mode values
    if (isSpeedMode) {
      // make sure only changes are made when not enabled
      if (!(stepper2Enabled || stepper2Enabled)){
        stepper2MMPerSec -= 0.1;
        updateStepper2StepsPerSec();
        //updateStepper2RPM();
      }
    }
    // updates speed mode values
    else {
      previousStepper2Position = stepper2Position;
      stepper2Position-=10;
    }
    enc2Value = newEnc2Value;
  }
}

// ROTATING SPEED MODE:
// Takes one step each loop
void stepper1SpeedMode() {
  if (stepper1Enabled){
    stepper1.setSpeed(stepper1StepsPerSec);
}
}
// BELT SPEED MODE:
// Takes one step if correct delay time has passed
void stepper2SpeedMode() {
  if (stepper2Enabled){
    stepper2.setSpeed(stepper2StepsPerSec*40);
}
}

// ROTATING POSITION MODE:
// Calculates num of steps that must be taken and direction
// and moves calculated num of steps
void stepper1PositionMode() {
  int stepsToTake = stepper1Position - previousStepper1Position;
  // sets direction to move
  if (stepsToTake > 0) { 
    digitalWrite(STEPPER1_DIR_PIN, HIGH);
  }
  else {
    digitalWrite(STEPPER1_DIR_PIN, LOW);
  }
  // moves 1/20 of a revolution if stepper1 position has changed
  for(int i = 0; i<abs(stepsToTake)*30;i++){
     if(stepsToTake>0){
      stepper1.setSpeed(800);
    }
    if(stepsToTake<0){
      stepper1.setSpeed(-800);
  }
  }
  previousStepper1Position = stepper1Position;
}

// BELT POSITION MODE:
// Calculates num of steps that need to be taken and direction
// and moves calculated num of steps
void stepper2PositionMode() {
  int stepsToTake = stepper2Position - previousStepper2Position;
  // sets direction to move
  if (stepsToTake > 0) { 
    digitalWrite(STEPPER2_DIR_PIN, LOW);
  }
  else if (stepsToTake < 0){
    digitalWrite(STEPPER2_DIR_PIN, HIGH);
  }
  // moves 1/20 of a revolution if stepper2 position has changed
  for (int i = 0; i < abs(stepsToTake) * 1000; i++) {
    digitalWrite(STEPPER2_STEP_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEPPER2_STEP_PIN, LOW);
    delayMicroseconds(500);
  }
  previousStepper2Position = stepper2Position;
}

// BUTTON PRESS CHECK:
// Checks if the button to switch between spd and pos mode has
// been pressed
void checkBtnSwhState() {
  int btnSwhState = digitalRead(BTN_SWH_PIN);

	// if LOW signal detected, button switch is pressed
	if (btnSwhState == LOW) {
		// if 50ms have passed since last LOW pulse, it means that the
		// button has been pressed
		if (millis() - lastButtonSwhPress > 100) {
      btnSwhVal++;
      isSpeedMode = !isSpeedMode;
      stepper1Enabled = false;
      stepper2Enabled = false;
      if (isSpeedMode){
        modeLetter = "S";
      }
      else{
        modeLetter = "P";
      }
		}

		// Remember last button press event (last LOW reading)
		lastButtonSwhPress = millis();
	}
}

// ROTATING STEPPER ENCODER PRESS CHECK:
// Checks if the encoder has been pressed
void checkBtn1State() {
  int btn1State = digitalRead(ENCODER1_SW_PIN);

	// if LOW signal detected, encoder1 button is pressed
	if (btn1State == LOW) {
		// if 50ms have passed since last LOW pulse, it means that the
		// button has been pressed
		if (millis() - lastButton1Press > 100) {
      stepper1Enabled = !stepper1Enabled;
		}

		// Remember last button press event (last LOW reading)
		lastButton1Press = millis();
	}
}

// BELT STEPPER ENCODER PRESS CHECK:
// Checks if the encoder has been presed
void checkBtn2State() {
  int btn2State = digitalRead(ENCODER2_SW_PIN);

	//if LOW signal detected, encoder2 button is pressed
	if (btn2State == LOW) {
		// if 50ms have passed since last LOW pulse, it means that the
		// button has been pressed
		if (millis() - lastButton2Press > 100) {
      stepper2Enabled = !stepper2Enabled;
      }

		// Remember last button press event (last LOW reading)
		lastButton2Press = millis();
	}
}
// DISPLAYS LCD INFO:
// Prints the RPM and mm/s values
void displayLCD() {
	// if 500ms have passed since last LCD update, updates screen
  if (millis() - lastLCD > 500) {
    // update first line
    lcd.clearDisplay();
    lcd.setCursor(0,0);
    lcd.println("\nRPM:" + (String)(stepper1RPM));
    
    // update second line
    lcd.println("mm/s:");
    lcd.println((stepper2MMPerSec*4/5/10), 4);
    
    lcd.setCursor(110,0);
    lcd.print(modeLetter);
    lcd.display();
    
    // Remember last button press event (last LOW reading)
    lastLCD = millis();
	}	
}

// MM PER SEC:
// Sets initial mm/s value for both steppers
void mmPerSec(){
  stepper2MMPerSec = stepper2StepsPerSec / STEPPER_2_STEPS_PER_MM;
}

// STEPS PER SEC:
// Sets inital num of steps/sec for both steppers
void setStepsPerSec(){
  // Convert RPM to steps per second
  stepper1StepsPerSec = (double) stepper1RPM * STEPPER_1_STEPS / 60.0;
  stepper2StepsPerSec = (double) stepper2RPM * STEPPER_2_STEPS / 60.0;
}

// STEPPER DELAYS:
// Sets inital delay amount for both steppers


// UPDATE S1 STEPS/SEC:
// Updates step/sec val of stepper 1 after encoder changed
void updateStepper1StepsPerSec(){
  stepper1StepsPerSec = (double)((stepper1RPM * STEPPER_1_STEPS) / 60.0);
}


// UPDATE S2 STEPS/SEC:
// Updates step/sec val of stepper 2 after encoder changed
void updateStepper2StepsPerSec(){
  stepper2StepsPerSec = (stepper2MMPerSec * STEPPER_2_STEPS_PER_MM);
}

// UPDATE S2 RPM:
// Updates RPM value of stepper 2 using new step/sec value
//void updateStepper2RPM(){
  //stepper2RPM = 60.0 * stepper2StepsPerSec * (1.0/STEPPER_2_STEPS);
//}

//void stepsPerDiameter(){
  //stepper2StepsPerDiameter = STEPPER_2_SAMPLE_MM_DIAMETER * STEPPER_2_STEPS_PER_MM;
//}
