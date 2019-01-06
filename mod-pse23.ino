#include <ESP32_Servo.h>

#include <Bounce2.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <Wire.h>
#include <SerialDebug.h>

// Pin Definitions
#define PIN_LED 36
#define PIN_MOTOR_A 26
//#define PIN_MOTOR_B 34
//#define PIN_MOTOR_C 27
#define PIN_ESC_FET 27
#define PIN_TRIGGER_FULL 14
#define PIN_TRIGGER_HALF 12
#define PIN_TRIGGER_REV 13
#define PIN_ALT_MODE 33
#define PIN_E_STOP 38
#define PIN_PUSHER_FET 25
#define PIN_PROFILE_A 23
#define PIN_PROFILE_B 2
#define PIN_SELECT_FIRE_A 18
#define PIN_SELECT_FIRE_B 19
#define PIN_CONFIG_MODE 2
#define PIN_ENABLE_BT 21
#define PIN_ENABLE_DEBUG 17
#define PIN_MAG_SENSOR 35
#define PIN_DART_IN_MAG_SENSOR 32
#define PIN_PUSHER_RETURN_SENSOR 39
#define PIN_CHRONO_A 37
#define PIN_CHRONO_B 34
#define PIN_BATTERY_MONITOR 4

// Configuration Options
byte BurstSize = 3; // Configured
byte TargetDPS = 99; // 99 means full rate
bool EStopOnPusherJam = true; // Controls whether to stop on a pusher jam or not
bool FireOnEmptyMag = false; // Controls whether to fire on an empty mag
int MotorSpeedFull = 50; // For full-pull
int MotorSpeedHalf = 30; // For half-pull
int MagSize = 18; // Magazine size

// Profile Management
#define PROFILE_ALT_PROFILE_A 0
#define PROFILE_ALT_PROFILE_B 1
#define PROFILE_ALT_PROFILE_C 2
#define PROFILE_ALT_PROFILE_D 3
#define PROFILE_ALT_SWAP_DIM_FPS 4
#define PROFILE_ALT_NOTHING 5
struct ProfileDefinition
{
  byte FullPower = 100;
  byte HalfPower = 30;
  byte ROF = 99;
  byte BurstSize = 3;
  bool FireOnEmptyMag = false;
  byte AltAction = PROFILE_ALT_NOTHING;
};
ProfileDefinition Profiles[4];
byte CurrentProfile = 0;

// Config Screen Menu Paging
#define CONFIG_MENU_MAIN 0
#define CONFIG_MENU_PROFILE_A 1
#define CONFIG_MENU_PROFILE_A_ALT 2
#define CONFIG_MENU_PROFILE_B 3
#define CONFIG_MENU_PROFILE_B_ALT 4
#define CONFIG_MENU_PROFILE_C 5
#define CONFIG_MENU_PROFILE_C_ALT 6
#define CONFIG_MENU_PROFILE_D 7
#define CONFIG_MENU_PROFILE_D_ALT 8
#define CONFIG_MENU_SYSTEM 9
byte CurrentConfigMenuPage = CONFIG_MENU_MAIN;


// Pusher Controls
// Pusher 3S
#define PULSE_ON_TIME_3S 45
#define PULSE_RETRACT_TIME_3S 75
#define PULSE_DUTY_CYCLE_3S 255 
// Pusher 4S
#define PULSE_ON_TIME_4S 25
#define PULSE_RETRACT_TIME_4S 75
#define PULSE_DUTY_CYCLE_4S 192 
int PulseOnTime;
int PulseRetractTime;
int PulseDutyCycle;
// Firing Controls
#define FIRE_MODE_SINGLE 0
#define FIRE_MODE_BURST 1
#define FIRE_MODE_AUTO 2
#define FIRE_MODE_AUTO_LASTSHOT 3
#define FIRE_MODE_IDLE 4
int CurrentFireMode = FIRE_MODE_SINGLE; // This is the user request based on the button state
int ProcessingFireMode = FIRE_MODE_IDLE; // This is what will actually be fired.
bool ExecuteFiring = false; // Set to true when the Solenoid is supposed to move
int TimeBetweenShots = 0; // Calculated to lower ROF
int ShotsToFire = 0; // Number of shots in the queue
unsigned long LastShot = 0; // When the last shot took place.
#define SOLENOID_JAM_DURATION 1500
unsigned long SolenoidLastHome = 0;
#define PUSHER_SENSOR_ANALOG_THRESHOLD 3500

// Solenoid Controls
#define SOLENOID_CYCLE_IDLE 0
#define SOLENOID_CYCLE_PULSE 1
#define SOLENOID_CYCLE_RETRACT 2
#define SOLENOID_CYCLE_COOLDOWN 3
int CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
unsigned long LastSolenoidCycleStarted = 0;
bool PusherHome = true;

// Magazine status
#define DART_IN_MAG_SENSOR_ANALOG_THRESHOLD 2000
#define MAG_SENSOR_ANALOG_THRESHOLD 2000
bool DartLoaded = false;
bool MagLoaded = false;
int DartsInMag = 0;
long TotalDartsFired = 0;
long LifetimeDartsFired = 0; 

// Physical Switch Status
bool RevTriggerPressed = false; // Rev Trigger is Depressed
bool FireFullTriggerPressed = false; // Fire Trigger is Depressed
bool FireHalfTriggerPressed = false; // Fire Trigger is Depressed
bool ConfigModePressed = false; // Fire Trigger is Depressed

// OLED Stuff
SSD1306AsciiWire Display;
#define OLED_ADDR 0x3C

// Motor Controls
#define MotorKV 3000
int MaxMotorSpeed = 2000;
#define MOTOR_SPINUP_LAG 100
#define MOTOR_SPINDOWN_3S 4000
#define MOTOR_SPINDOWN_4S 6000
#define MOTOR_SPINUP_3S 0
#define MOTOR_SPINUP_4S 0
int DecelerateTime = 0;
int AccelerateTime = 0;
int MotorRampUpPerMS = 0;
int MotorRampDownPerMS = 0;
int MinMotorSpeed = 1000;
int CurrentMotorSpeed = MinMotorSpeed;
int TargetMotorSpeed = MinMotorSpeed;
bool MotorsEnabled = false;
byte SetMaxSpeed = 100; // in percent.
unsigned long TimeLastMotorSpeedChanged = 0;
Servo MotorA;
Servo MotorB;
Servo MotorC;

#define COMMAND_REV_NONE 0
#define COMMAND_REV_HALF 1
#define COMMAND_REV_FULL 2
int CommandRev = COMMAND_REV_NONE;
int PrevCommandRev = COMMAND_REV_NONE;
bool AutoRev = false; // True when the computer is managing the rev process.

// System Controls
#define MODE_NORMAL 0
#define MODE_CONFIG 1
#define MODE_MAG_OUT 2
#define MODE_DARTS_OUT 3
#define MODE_LOW_BATT 4
#define MODE_E_STOP 6
int SystemMode;
bool EStopActive = false;

// Inputs
#define DebounceWindow 5 // Debounce Window = 5ms
Bounce RevTriggerBounce = Bounce();
Bounce FireHalfTriggerBounce = Bounce();
Bounce FireFullTriggerBounce = Bounce();
Bounce PusherResetBounce = Bounce();
Bounce ModeSelectABounce = Bounce();
Bounce ModeSelectBBounce = Bounce();
Bounce EStopBounce = Bounce();
Bounce ConfigModeBounce = Bounce();

// Misc Stuff
bool DebugEnabled = true;

// Battery Controls
int BatteryS = 3;
#define BATTERY_3S_MIN 9.6
#define BATTERY_3S_MAX 12.6
#define BATTERY_4S_MIN 12.8
#define BATTERY_4S_MAX 16.8
#define BATTERY_CALFACTOR 0.0 // Adjustment for calibration.
float BatteryMaxVoltage;
float BatteryMinVoltage;
float BatteryCurrentVoltage = 99.0;
bool BatteryFlat = false;


void setup() {
  // put your setup code here, to run once:

 Serial.begin(115200); // Can change it to 230400, if you dont use debugIsr* macros

#ifdef __AVR_ATmega32U4__ // Arduino AVR Leonardo

    while (!Serial) {
        ; // wait for serial port to connect. Needed for Leonardo only
    }

#else

    delay(500); // Wait a time

#endif

        // Debug

        // Attention:
    // SerialDebug starts disabled and it only is enabled if have data avaliable in Serial
    // Good to reduce overheads.
        // if You want debug, just press any key and enter in monitor serial

    // Note: all debug in setup must be debugA (always), due it is disabled now.

    printlnA(F("**** Setup: initializing ..."));

    // Buildin led

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // WiFi connection, etc ....

    // ...

    // End

    printlnA(F("*** Setup end"));
 Serial.begin(57600); // Can change it to 230400, if you dont use debugIsr* macros

#ifdef __AVR_ATmega32U4__ // Arduino AVR Leonardo

    while (!Serial) {
        ; // wait for serial port to connect. Needed for Leonardo only
    }

#else

    delay(500); // Wait a time

#endif

        // Debug

        // Attention:
    // SerialDebug starts disabled and it only is enabled if have data avaliable in Serial
    // Good to reduce overheads.
        // if You want debug, just press any key and enter in monitor serial

    // Note: all debug in setup must be debugA (always), due it is disabled now.

    printlnA(F("**** Setup: initializing ..."));

    // Buildin led

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // WiFi connection, etc ....

    // ...

    // End

    printlnA(F("*** Setup end"));

  

  pinMode( PIN_ENABLE_DEBUG, INPUT_PULLUP );
  if( digitalRead( PIN_ENABLE_DEBUG ) == HIGH )
    DebugEnabled = true;
  else
    DebugEnabled = true;
  

  DebugInit; 
  DebugPrintln( F("Booting.. ") );  

  // Boot LCD
  DebugPrintln( F("Initialising Display") );
  Wire.begin();
  Wire.setClock(400000L);
  Display.begin(&Adafruit128x64, OLED_ADDR);
  DebugPrintln( F("Display Initialised") );
  

  // Set up debouncing
  DebugPrintln( F("Configuring Debouncing") );

  pinMode(PIN_TRIGGER_REV, INPUT_PULLUP);
  RevTriggerBounce.attach( PIN_TRIGGER_REV );
  RevTriggerBounce.interval( DebounceWindow );
  
  pinMode(PIN_TRIGGER_FULL, INPUT_PULLUP);
  FireFullTriggerBounce.attach( PIN_TRIGGER_FULL );
  FireFullTriggerBounce.interval( DebounceWindow );

  pinMode(PIN_TRIGGER_HALF, INPUT_PULLUP);
  FireHalfTriggerBounce.attach( PIN_TRIGGER_HALF );
  FireHalfTriggerBounce.interval( DebounceWindow );
  
  pinMode(PIN_PUSHER_RETURN_SENSOR, INPUT_PULLUP);
  PusherResetBounce.attach( PIN_PUSHER_RETURN_SENSOR );
  PusherResetBounce.interval( DebounceWindow );  

  pinMode(PIN_SELECT_FIRE_A, INPUT_PULLUP);
  ModeSelectABounce.attach( PIN_SELECT_FIRE_A );
  ModeSelectABounce.interval( DebounceWindow );
  
  pinMode(PIN_SELECT_FIRE_B, INPUT_PULLUP);
  ModeSelectBBounce.attach( PIN_SELECT_FIRE_B );
  ModeSelectBBounce.interval( DebounceWindow );  

  pinMode(PIN_E_STOP, INPUT_PULLUP);
  EStopBounce.attach( PIN_E_STOP );
  EStopBounce.interval( DebounceWindow );  

  pinMode(PIN_CONFIG_MODE, INPUT_PULLUP);
  ConfigModeBounce.attach( PIN_CONFIG_MODE );
  ConfigModeBounce.interval( DebounceWindow );    
  DebugPrintln( F("Debouncing Configured") );

  pinMode( PIN_DART_IN_MAG_SENSOR, INPUT_PULLUP );
  pinMode( PIN_MAG_SENSOR, INPUT_PULLUP );


  DebugPrintln( F("Initialising ESC") );
  // Turn on ESC's
  pinMode(PIN_ESC_FET, OUTPUT);
  digitalWrite( PIN_ESC_FET, HIGH );  
  // Set up motors
  MotorA.attach(PIN_MOTOR_A);
  //MotorB.attach(PIN_MOTOR_B);
  //MotorC.attach(PIN_MOTOR_C);
  // Arm ESC's
  MotorA.writeMicroseconds(MinMotorSpeed);
  //MotorB.writeMicroseconds(MinMotorSpeed);
  //MotorC.writeMicroseconds(MinMotorSpeed);
  delay(1000);   // Wait for ESC to initialise (9 seconds)
  DebugPrintln( F("ESC Initialised") );

  // Initialise the pusher
  pinMode(PIN_PUSHER_FET, OUTPUT);
  digitalWrite( PIN_PUSHER_FET, LOW );    

  // Battery Configuration
  //BatteryS = 3;

  SetupSelectBattery();
  Serial.println( F("Battery Selected") );
  
  if( BatteryS == 3 )
  {
    PulseOnTime = PULSE_ON_TIME_3S;
    PulseRetractTime = PULSE_RETRACT_TIME_3S;
    PulseDutyCycle = PULSE_DUTY_CYCLE_3S;

    BatteryMaxVoltage = BATTERY_3S_MAX;
    BatteryMinVoltage = BATTERY_3S_MIN;

    DecelerateTime = MOTOR_SPINDOWN_3S;
    AccelerateTime = MOTOR_SPINUP_3S;
  }
  else
  {
    PulseOnTime = PULSE_ON_TIME_4S;
    PulseRetractTime = PULSE_RETRACT_TIME_4S;
    PulseDutyCycle = PULSE_DUTY_CYCLE_4S;    

    BatteryMaxVoltage = BATTERY_4S_MAX;
    BatteryMinVoltage = BATTERY_4S_MIN;

    DecelerateTime = MOTOR_SPINDOWN_4S;
    AccelerateTime = MOTOR_SPINUP_4S;
  }
  
  SystemMode = MODE_NORMAL;
  CalculateRampRates();


  // Now wait until the trigger is high
  FireHalfTriggerBounce.update();
  while( FireHalfTriggerBounce.read() == HIGH )
  {
    delay(10);
    FireHalfTriggerBounce.update();
  }
  delay(10);
}

/*
 * Displays the startup screen to select the battery type
 */
void SetupSelectBattery()
{
  unsigned long StartTime = millis();
  int BatSel = 0;
  int LastBatSel = 99;
  bool Processed = false;

  while( !Processed )
  {
    RevTriggerBounce.update(); // Update the pin bounce state
    FireHalfTriggerBounce.update();

    if( RevTriggerBounce.fell() )
    {
      if( BatSel == 0 )
        BatSel = 1;
      else
        BatSel = 0;
    }
    if( FireHalfTriggerBounce.fell() )
    {
      Processed = true;
    }
  
    if( BatSel != LastBatSel )
    {
      Display.clear();
      Display.setCursor(0, 0);
      Display.setFont(ZevvPeep8x16);
      Display.print( F("Confirm Battery\n") );
      if( BatSel == 0 )
      {
        Display.print( F("> 3s\n") );
        Display.print( F("  4s") );
      }
      else
      {
        Display.print( F("  3s\n") );
        Display.print( F("> 4s") );      
      }
    }
    LastBatSel = BatSel;
  }

  if( BatSel == 0 )
    BatteryS = 3;
  else
    BatteryS = 4;

  Display.clear();
  Display.setCursor(0, 0);
  Display.print( F("Initialising") );
  
  while( millis() - StartTime < 3000 )
  {
    delay( 10 );
  }
}

/*
 * This is a boot time init sub to calcualte the Acceleration and 
 * deceleration ramp rates of the motors.
 * 
 */
void CalculateRampRates()
{
  int SpeedRange = (MaxMotorSpeed - MinMotorSpeed) * 1000; // Multiply by 1000 to give some resolution due to integer calculations
  if( AccelerateTime == 0 )
  {
    MotorRampUpPerMS = SpeedRange;  // For instant acceleration
  }
  else
  {
    MotorRampUpPerMS = SpeedRange / AccelerateTime;  // Use when Accelerating
  }

  if( DecelerateTime == 0 )
  {
    MotorRampDownPerMS = SpeedRange;  // For instant acceleration
  }
  else
  {
    MotorRampDownPerMS = SpeedRange / DecelerateTime;  // Use when Decelerating
  }  


  DebugPrint( "Ramp Up per MS = " );
  DebugPrintln( MotorRampUpPerMS );

  DebugPrint( "Ramp Dowbn per MS = " );
  DebugPrintln( MotorRampDownPerMS );
}

void loop() {
  // put your main code here, to run repeatedly:

  ProcessButtons(); // Get User and Sensor input
  ProcessMagRelease(); // Update any conditions based on magazine status
  ProcessBatteryMonitor(); // Check battery voltage
  ProcessSystemMode(); // Find out what the system should be doing
  ProcessEStop(); // Process the EStop Condition
  ProcessRevCommand(); // Handle motor intentions
  
  // Detected a change to the command. Reset the last speed change timer.
  if( PrevCommandRev != CommandRev )
  {
    TimeLastMotorSpeedChanged = millis();
    PrevCommandRev = CommandRev;
  }

  // Process speed control  
  ProcessSpeedControl();
  // Calcualte the new motor speed
  ProcessMotorSpeed();
  // Send the speed to the ESC
  ProcessMainMotors();

  // Process Firing Controls
  ProcessFiring();
  ProcessSolenoid();

  // Let's fix up the DartsInMag now... 
  // We might think that the mag is empty when there is really a dart still loaded
  if( (DartsInMag <= 0) && DartLoaded )
  {
    DartsInMag = 1; // We don't know how many, but at least one..
  }
  // Or we might think we have mire than we really do
  if( (DartsInMag > 0) && !DartLoaded )
  {
    DartsInMag = 0;
  }

  // Update the OLED
  ProcessDisplay();

  //DebugPrintln( "." );
}

void ProcessMagRelease()
{
  static bool LastMagLoaded = false;
  if( LastMagLoaded != MagLoaded )  // Detect a change in the status quo
  {
    if( MagLoaded )  // A mag has been inserted
    {
      DartsInMag = MagSize;
    }
    else // Mag has been dropped
    {
      DartsInMag = 0;
      LifetimeDartsFired += TotalDartsFired;
      // Save to EEPROM here...
    }
  }
  LastMagLoaded = MagLoaded;
}

void ProcessSolenoid()
{
  if( !PusherHome && (millis() - SolenoidLastHome > SOLENOID_JAM_DURATION) ) // The pusher is jammed
  {
    if( EStopOnPusherJam )
    {
      EStopActive = true;
      return;
    }
  }

  if( !ExecuteFiring ) // Just skip if there is no firing to execute
  {
    return;
  }

  // Calculate duty cycle whenever the target changes.
  static int PrevTargetDPS = 0;
  if( PrevTargetDPS != TargetDPS )
  {
    PrevTargetDPS = TargetDPS;
    if( TargetDPS == 99 ) // Full rate
    {
      TimeBetweenShots = 0;
    }
    else
    {
      int PulseOverhead = PulseOnTime + PulseRetractTime;
      int TotalPulseOverhead = PulseOverhead * TargetDPS;
      int FreeMS = 1000 - TotalPulseOverhead;
      if( FreeMS <= 0 )
      {
        TimeBetweenShots = 0; // Pusher won't achieve this rate
      }
      else
      {
        TimeBetweenShots = FreeMS / TargetDPS;
      }
    }
  }

  if( ProcessingFireMode == FIRE_MODE_IDLE )
  {
    return; // Solenoid is idling.
  }

  if( ShotsToFire == 0 && ProcessingFireMode != FIRE_MODE_IDLE )
  {
    ProcessingFireMode = FIRE_MODE_IDLE;
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
    digitalWrite( PIN_PUSHER_FET, LOW );
    digitalWrite( PIN_LED, HIGH );
    LastShot = millis();
    DebugPrintln( "Finished shooting" );
    ExecuteFiring = false;
    return;    
  }


  if( CommandRev == COMMAND_REV_NONE )
  {
    ProcessingFireMode = FIRE_MODE_IDLE;
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
    digitalWrite( PIN_PUSHER_FET, LOW );
    digitalWrite( PIN_LED, HIGH );
    LastShot = millis();
    DebugPrintln( "Shooting Aborted - Motors not running" );
    ExecuteFiring = false;
    return;        
  }

  if( (millis() - LastSolenoidCycleStarted) < PulseOnTime )
  {
    if( CurrentSolenoidCyclePosition != SOLENOID_CYCLE_PULSE )
    {
      //DebugPrintln( "Start Pulse" );
      //DebugPrint( "Shots to fire = " );
      //DebugPrintln( ShotsToFire );
      //DebugPrint( "ProcessingFireMode = " );
      //DebugPrintln( ProcessingFireMode );
      if( !PusherHome && EStopOnPusherJam ) // The pusher is not in the home position - so don't fire yet.
      {
        return;
      }
      if( (SystemMode == MODE_MAG_OUT) || ( SystemMode == MODE_CONFIG )) // Don't fire with the mag out or with config open
      {
        return;
      }
      if( !DartLoaded && !FireOnEmptyMag ) // Mag is empty, and we are configured not to fire.
      {
        ShotsToFire = 0;
        DebugPrintln( "Mag Empty!!" );
        return;
      } 
    }
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_PULSE;
    digitalWrite( PIN_PUSHER_FET, HIGH );
    digitalWrite( PIN_LED, LOW );
    return;
  }

  if( (millis() - LastSolenoidCycleStarted) < (PulseOnTime + PulseRetractTime) )
  {
    if( CurrentSolenoidCyclePosition != SOLENOID_CYCLE_RETRACT )
    {
      //DebugPrintln( "End Pulse" );
    }
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_RETRACT;
    digitalWrite( PIN_PUSHER_FET, LOW );
    digitalWrite( PIN_LED, HIGH );
    return;      
  }  

  if((millis() - LastSolenoidCycleStarted) < (PulseOnTime + PulseRetractTime + TimeBetweenShots))
  {
    if( CurrentSolenoidCyclePosition != SOLENOID_CYCLE_COOLDOWN )
    {
      //DebugPrintln( "Cooling Down" );
    }
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_COOLDOWN;
    digitalWrite( PIN_PUSHER_FET, LOW );
    digitalWrite( PIN_LED, HIGH );
    return;      
  }

  CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
  ShotsToFire -= 1;
  TotalDartsFired ++;
  if( DartsInMag > 0 )
  {
    DartsInMag --;
  }
  LastSolenoidCycleStarted = millis();
  DebugPrintln( "Bang!!" );  

  if( !DartLoaded && !FireOnEmptyMag ) // Mag is empty, and we are configured not to fire.
  {
    ShotsToFire = 0;
    DebugPrintln( "Mag Empty!!" );
  }  
}

// Process the firing request and queue up some darts to fire.
void ProcessFiring()
{
  if( !((SystemMode == MODE_NORMAL) || 
        (SystemMode == MODE_DARTS_OUT) || 
        (SystemMode == MODE_DARTS_OUT) ||
        (SystemMode == MODE_CONFIG)) ) // Finish off the stroke unless in low batt or e-stop.
  {
    ShotsToFire = 0;
    if( ProcessingFireMode == FIRE_MODE_AUTO_LASTSHOT )
      ProcessingFireMode = FIRE_MODE_AUTO;
    return;
  }

  if( CommandRev == COMMAND_REV_NONE ) // Don't try and push a dart into stationary flywheels..
  {
    if( ProcessingFireMode == FIRE_MODE_AUTO )
    {
      ProcessingFireMode = FIRE_MODE_AUTO_LASTSHOT;
      LastSolenoidCycleStarted = millis();
      ShotsToFire = 0;
      ExecuteFiring = true;
    }
    return;
  }
  
  if( FireFullTriggerBounce.fell() && ProcessingFireMode ==  FIRE_MODE_IDLE )
  {
    ProcessingFireMode = CurrentFireMode;
    switch( ProcessingFireMode )
    {
      case FIRE_MODE_SINGLE:
        ShotsToFire = 1; // Add another shot to the queue
        LastSolenoidCycleStarted = millis();
        ExecuteFiring = true;
        break;
      case FIRE_MODE_BURST:
        ShotsToFire = BurstSize; // Set the burst size
        LastSolenoidCycleStarted = millis();
        ExecuteFiring = true;
        break;        
      case FIRE_MODE_AUTO:
        ShotsToFire = 9999; // Set to something unreasonably high
        LastSolenoidCycleStarted = millis();
        ExecuteFiring = true;
        break;        
    }
  }
  else if( FireFullTriggerBounce.rose() && (ProcessingFireMode == FIRE_MODE_AUTO) ) 
  {
    ProcessingFireMode = FIRE_MODE_AUTO_LASTSHOT;
    LastSolenoidCycleStarted = millis();
    ExecuteFiring = true;

    if( CurrentSolenoidCyclePosition == SOLENOID_CYCLE_PULSE )
    {
      ShotsToFire = 1;
    }
    else
    {
      ShotsToFire = 0;
    }
  }
}


// We are toggline between different system states here..
void ProcessSystemMode()
{
  static int LastSystemMode = MODE_NORMAL;

  if( BatteryFlat ) // Battery low
  {
    SystemMode = MODE_LOW_BATT;
  }
  else
  {
    if( EStopActive )
    {
      SystemMode = MODE_E_STOP;
    }
    else
    {
      if( ConfigModePressed )
      {
        SystemMode = MODE_CONFIG;
        if( LastSystemMode != MODE_CONFIG ) // Reset back to the menu start on a when pressing the config button
          CurrentConfigMenuPage = CONFIG_MENU_MAIN;
        //return;       
      }
      else if( !MagLoaded )
      {
        SystemMode = MODE_MAG_OUT;
        //return;
      }      
      else if( !DartLoaded && !FireOnEmptyMag )
      {
        SystemMode = MODE_DARTS_OUT;
        //return;
      }
      else
        SystemMode = MODE_NORMAL;
    }
  }

  
  if( LastSystemMode != SystemMode )
  {
    DebugPrint( "New System Mode = " );
    DebugPrintln( SystemMode );
    LastSystemMode = SystemMode;
  }
}

// Handle the E-Stop condition
void ProcessEStop()
{
  static bool PrevEStopActive = false;
  if( PrevEStopActive != EStopActive ) // Only process when a change happens.
  {
    PrevEStopActive = EStopActive;
    if( EStopActive ) 
    {
      digitalWrite( PIN_ESC_FET, LOW ); // Turn off cages
      digitalWrite( PIN_PUSHER_FET, LOW ); // Turn off the pusher
    }
    else
    {
      digitalWrite( PIN_ESC_FET, HIGH ); // Turn on cages
    }
  }
}

// We need to set the Target Motor Speed here.
void ProcessSpeedControl()
{
  static byte LastSetMaxSpeed = 100;

  if( CommandRev == COMMAND_REV_HALF ) SetMaxSpeed = MotorSpeedHalf;
  if( CommandRev == COMMAND_REV_FULL ) SetMaxSpeed = MotorSpeedFull;
  if( CommandRev == COMMAND_REV_NONE ) SetMaxSpeed = 0;

  if( LastSetMaxSpeed == SetMaxSpeed ) return; // Speed hasn't changed

  if( CommandRev > COMMAND_REV_NONE ) 
  {
    SetMaxSpeed = constrain( SetMaxSpeed, 30, 100 ); // Constrain between 30% and 100%
  }
  
  TargetMotorSpeed = map( SetMaxSpeed, 0, 100, MinMotorSpeed, MaxMotorSpeed );  // Find out our new target speed.

  LastSetMaxSpeed = SetMaxSpeed;

  DebugPrint( F("New max speed % = ") );
  DebugPrintln( SetMaxSpeed );

  DebugPrint( F("New target speed = ") );
  DebugPrintln( TargetMotorSpeed );
  
}

/*
 * Process the manual commands leading to motor reving
 * 
 * Logic:
 * If AutoRev is being performed, disconnect it when the half trigger is pulled.
 * We are looking for the following events: 
 * If the Half Trigger is pressed, Rev to Speed A
 * If the Rev Trigger is pressed, and the Half Trigger is also pressed, Rev to Speed B
 * If the Rev Trigger is pressed, but the Half Trigger is not, then ignore the command.
 * 
 */
void ProcessRevCommand()
{
  
  static bool PreviousFireHalfTriggerPressed = false; // Keep track of the human input

  if( !((SystemMode == MODE_NORMAL) || (SystemMode == MODE_DARTS_OUT)) ) // Spin the motors down when something out of the ordinary happens.
  {
     CommandRev = COMMAND_REV_NONE;
     AutoRev = false;
     return;
  }

  if( PreviousFireHalfTriggerPressed != FireHalfTriggerPressed )
  {
    // Human has taken control - disengage autopilot
    PreviousFireHalfTriggerPressed = FireHalfTriggerPressed;
    AutoRev = false;
  }

  if( !AutoRev )
  {
    if( FireHalfTriggerPressed )
    {
      if( RevTriggerPressed )
      {
          CommandRev = COMMAND_REV_FULL;
      }
      else
      {
        CommandRev = COMMAND_REV_HALF;
      }
    }
    else
    {
      CommandRev = COMMAND_REV_NONE;
    }
  }
  // Else the computer is controlling, and the current rev trigger state is ignored. Autopilot will adjust CommandRev
  
}

void ProcessMainMotors()
{
  static long PreviousMotorSpeed = MinMotorSpeed;

  if( PreviousMotorSpeed != CurrentMotorSpeed ) 
  { 
    // Debugging output
    DebugPrintln(CurrentMotorSpeed);

    MotorA.writeMicroseconds( CurrentMotorSpeed );
    MotorB.writeMicroseconds( CurrentMotorSpeed );
    MotorC.writeMicroseconds( CurrentMotorSpeed );

    PreviousMotorSpeed = CurrentMotorSpeed;
  }
}

/*
 * Process input from Buttons and Sensors.
 */
void ProcessButtons()
{
  RevTriggerBounce.update(); // Update the pin bounce state
  RevTriggerPressed = !(RevTriggerBounce.read());

  ConfigModeBounce.update(); // Update the pin bounce state
  ConfigModePressed = !(ConfigModeBounce.read());

  FireFullTriggerBounce.update(); // Update the pin bounce state
  FireFullTriggerPressed = !(FireFullTriggerBounce.read()); // We don't really care about this, because firing is based on the fall to ground potential

  FireHalfTriggerBounce.update(); // Update the pin bounce state
  FireHalfTriggerPressed = (FireHalfTriggerBounce.read()); // n.b. This switch is NC.

  PusherResetBounce.update(); // Update the pin bounce state
  PusherHome = (PusherResetBounce.read() == LOW);
  if( PusherHome )
  {
    SolenoidLastHome = millis(); // Keep track of the last known time
  }
  if( !PusherHome )
  {
    int PusherSensorValue = analogRead( PIN_PUSHER_RETURN_SENSOR );
    if( PusherSensorValue <= PUSHER_SENSOR_ANALOG_THRESHOLD ) // Just in case something is dirty
      PusherHome = true; 
    //else
    //  DebugPrintln( PusherSensorValue );
  }

  DartLoaded = (digitalRead(PIN_DART_IN_MAG_SENSOR) == HIGH);
  if( !DartLoaded )
  {
    int DartInMagSensorValue = analogRead( PIN_DART_IN_MAG_SENSOR );
    if( DartInMagSensorValue >= DART_IN_MAG_SENSOR_ANALOG_THRESHOLD ) // Just in case something is dirty
      DartLoaded = true; 
    //else
      //DebugPrintln( DartInMagSensorValue );
  }

  MagLoaded = (digitalRead(PIN_MAG_SENSOR) == HIGH);
  if( !MagLoaded )
  {
    int MagSensorValue = analogRead( PIN_MAG_SENSOR );
    if( MagSensorValue >= MAG_SENSOR_ANALOG_THRESHOLD ) // Just in case something is dirty
      MagLoaded = true; 
    //else
      //DebugPrintln( DartInMagSensorValue );
  }  


  // Determine the current firing mode
  ModeSelectABounce.update();
  ModeSelectBBounce.update();

  if( ModeSelectABounce.read() == LOW && ModeSelectBBounce.read() == HIGH && CurrentFireMode != FIRE_MODE_AUTO_LASTSHOT )
    CurrentFireMode = FIRE_MODE_AUTO;
  else if( ModeSelectABounce.read() == HIGH && ModeSelectBBounce.read() == HIGH )
    CurrentFireMode = FIRE_MODE_BURST;
  else if( ModeSelectABounce.read() == HIGH && ModeSelectBBounce.read() == LOW )
    CurrentFireMode = FIRE_MODE_SINGLE;

  EStopBounce.update(); // Update the e-stop state.
  if( EStopBounce.fell() )
  {
    EStopActive = !EStopActive;
  }
}

/*
 * Run the main motors.
 */
void ProcessMotorSpeed()
{
  // Don't do anything if the motor is already running at the desired speed.
  if( CurrentMotorSpeed == TargetMotorSpeed )
  {
    return;
  }

  unsigned long CurrentTime = millis(); // Need a base time to calcualte from
  unsigned long MSElapsed = CurrentTime - TimeLastMotorSpeedChanged;
  if( MSElapsed == 0 ) // No meaningful time has elapsed, so speed will not change
  {
    return;
  }
  if( CurrentMotorSpeed < TargetMotorSpeed )
  {
    int SpeedDelta = (MSElapsed * MotorRampUpPerMS / 1000);
    if( SpeedDelta < 1 ) return; // Not enough cycles have passed to make an appreciable difference to speed.    
    int NewMotorSpeed = CurrentMotorSpeed + SpeedDelta; // Calclate the new motor speed..

    // If it's within 1% (which is 10) of target, then just set it
    if( NewMotorSpeed + 10 >= TargetMotorSpeed )
    {
      NewMotorSpeed = TargetMotorSpeed;
    }

    TimeLastMotorSpeedChanged = CurrentTime;
    CurrentMotorSpeed = NewMotorSpeed;
  }
  if( CurrentMotorSpeed > TargetMotorSpeed )
  {
    int SpeedDelta = (MSElapsed * MotorRampDownPerMS / 1000);
    if( SpeedDelta < 1 ) return; // Not enough cycles have passed to make an appreciable difference to speed.
    int NewMotorSpeed = CurrentMotorSpeed - SpeedDelta; // Calclate the new motor speed..

    // If it's within 1% (which is 10) of target, then just set it
    if( NewMotorSpeed - 10 <= TargetMotorSpeed )
    {
      NewMotorSpeed = TargetMotorSpeed;
    }

    TimeLastMotorSpeedChanged = CurrentTime;
    CurrentMotorSpeed = NewMotorSpeed;
  }
}
void ProcessBatteryMonitor()
{
  
  // Only count one in 10 run-through cycles
  static int RunNumber = 0;
  RunNumber++;
  if( RunNumber <= 300 ) // Only read once every 300 cycles.. For performance reasons.
    return;
  RunNumber = 0;
  
  #define NUM_SAMPLES 100
  static int CollectedSamples = 0;
  static float SampleAverage = 0;
  float SensorValue = analogRead( PIN_BATTERY_MONITOR );
  if( CollectedSamples < NUM_SAMPLES )
  {
    CollectedSamples ++;
    SampleAverage += SensorValue;
  }
  else
  {
    BatteryCurrentVoltage = (((float)SampleAverage / (float)CollectedSamples * 3.3)  / 4096.0 * (float)((47.0 + 10.0) / 10.0)) + BATTERY_CALFACTOR;  // Voltage dividor - 47k and 10k
    if( BatteryCurrentVoltage < BatteryMinVoltage )
    {
      if( BatteryCurrentVoltage > 1.5 ) // If the current voltage is 0, we are probably debugging
      {
        BatteryFlat = true;
      }
      else
      {
        BatteryFlat = false;
      }
    }
    else
    {
      BatteryFlat = false;
    }  
    //DebugPrint( "BatteryVoltage = " );
    //DebugPrintln( BatteryCurrentVoltage );
    CollectedSamples = 0;
    SampleAverage = 0;
  }
}




/*
 * OLED Display Stuff
 */

void ProcessDisplay()
{
/*
#define MODE_NORMAL 0
#define MODE_CONFIG 1
#define MODE_MAG_OUT 2
#define MODE_DARTS_OUT 3
#define MODE_LOW_BATT 4
#define MODE_E_STOP 6
*/
  
  static int LastSystemMode = 99;
  static int LastConfigMenuPage = 99;
  bool ClearScreen = false;
  
  if( LastSystemMode != SystemMode )
  {
    ClearScreen = true;
    LastSystemMode = SystemMode;
  }
  if( SystemMode == MODE_CONFIG )
  {
    if( LastConfigMenuPage != CurrentConfigMenuPage )
    {
      ClearScreen = true;
      LastConfigMenuPage = CurrentConfigMenuPage;
      DebugPrintln( "Config page change" );
    }
  }

  Display_ScreenHeader( ClearScreen );
  switch( SystemMode )
  {
    case MODE_MAG_OUT:
      Display_MagOut( ClearScreen );
      break;
    case MODE_CONFIG:
      Display_Config( ClearScreen );
      break;
    case MODE_LOW_BATT:
      Display_LowBatt( ClearScreen );
      break;
    case MODE_DARTS_OUT:
      Display_Darts_Out( ClearScreen );
      break;
    case MODE_E_STOP:
      Display_EStop( ClearScreen );
      break;
    default:
      Display_Normal( ClearScreen );
      break;
  }
} 

void Display_Config( bool ClearScreen )
{

  /*
#define CONFIG_MENU_MAIN 0
#define CONFIG_MENU_PROFILE_A 1
#define CONFIG_MENU_PROFILE_A_ALT 2
#define CONFIG_MENU_PROFILE_B 3
#define CONFIG_MENU_PROFILE_B_ALT 4
#define CONFIG_MENU_PROFILE_C 5
#define CONFIG_MENU_PROFILE_C_ALT 6
#define CONFIG_MENU_PROFILE_D 7
#define CONFIG_MENU_PROFILE_D_ALT 8
#define CONFIG_MENU_SYSTEM 9
   */
  switch( CurrentConfigMenuPage )
  {
    case CONFIG_MENU_PROFILE_A:

      break;
    case CONFIG_MENU_PROFILE_A_ALT:

      break;
    case CONFIG_MENU_PROFILE_B:

      break;    
    case CONFIG_MENU_PROFILE_B_ALT:

      break;    
    case CONFIG_MENU_PROFILE_C:

      break;    
    case CONFIG_MENU_PROFILE_C_ALT:

      break;    
    case CONFIG_MENU_PROFILE_D:

      break;    
    case CONFIG_MENU_PROFILE_D_ALT:

      break;   
    case CONFIG_MENU_SYSTEM:
      Display_ConfigSystem( ClearScreen );
      break; 
    default: // Main menu
      Display_ConfigMainMenu( ClearScreen );
      break;    
  }
}

void Display_ConfigMainMenu( bool ClearScreen )
{
  static byte MenuItem = 0;
  static byte LastMenuItem = 99;
  if( ClearScreen || (MenuItem != LastMenuItem) )
  {
    if( ClearScreen )
      MenuItem = 0;

    DebugPrintln( F("CONFIG SCREEN - MAIN MENU") );

    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 2);

    switch( MenuItem )
    {
      case 0:
        Display.print( F("> Profile A\n") );
        Display.print( F("  Profile B\n") );
        Display.print( F("  Profile C") );
        break;
     case 1:   
        Display.print( F("  Profile A\n") );
        Display.print( F("> Profile B\n") );
        Display.print( F("  Profile C") );
        break;
     case 2:   
        Display.print( F("  Profile B\n") );
        Display.print( F("> Profile C\n") );
        Display.print( F("  Profile D") );
        break;
     case 3:   
        Display.print( F("  Profile C\n") );
        Display.print( F("> Profile D\n") );
        Display.print( F("  System   ") );
        break;
     default:   
        Display.print( F("  Profile C\n") );
        Display.print( F("  Profile D\n") );
        Display.print( F("> System   ") );
        break;
    }

    LastMenuItem = MenuItem;
  }

  if( RevTriggerBounce.rose() )
  {
    MenuItem++;
    if( MenuItem > 4 )
      MenuItem = 0;
  }

  if( FireHalfTriggerBounce.rose() )
  {
    switch( MenuItem )
    {
      case 0:
        break;
      case 1:
        break;
      case 2:
        break;
      case 3:
        break;
      default:
        CurrentConfigMenuPage = CONFIG_MENU_SYSTEM;
        break;
    }
  }
}

void Display_ConfigSystem( bool ClearScreen )
{
  static byte MenuItem = 0;
  static byte LastMenuItem = 99;
  if( ClearScreen || (MenuItem != LastMenuItem) )
  {
    if( ClearScreen )
      MenuItem = 0;
    DebugPrintln( F("CONFIG SCREEN - SYSTEM") );

    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 2);

    switch( MenuItem )
    {
      case 0:
        Display.print( F("> Empty Mag F: ") );
        if( FireOnEmptyMag )
          Display.print( "Y" );
        else
          Display.print( "N" );
        Display.print( "\n" );
        Display.print( F("  P Jam Stop: ") );
         if( EStopOnPusherJam )
          Display.print( "Y" );
        else
          Display.print( "N" );
        Display.print( " \n" );       
        Display.print( F("  TDF: xxxxx      ") );
        break;
     case 1:   
        Display.print( F("  Empty Mag F: ") );
        if( FireOnEmptyMag )
          Display.print( "Y" );
        else
          Display.print( "N" );
        Display.print( "\n" );
        Display.print( F("> P Jam Stop: ") );
         if( EStopOnPusherJam )
          Display.print( "Y" );
        else
          Display.print( "N" );
        Display.print( " \n" ); 
        Display.print( F("  TDF: xxxxx      ") );
        break;
     case 2:   
        Display.print( F("  P Jam Stop: ") );
         if( EStopOnPusherJam )
          Display.print( "Y" );
        else
          Display.print( "N" );
        Display.print( " \n" );       
        Display.print( F("> TDF: xxxxx    \n") );
        Display.print( F("  LDF: xxxxx ") );
        break;
     case 3:   
        Display.print( F("  TDF: xxxxx    \n") );
        Display.print( F("> LDF: xxxxx  \n") );     
        Display.print( F("  Back         ") );
        break;
     default:   
        Display.print( F("  TDF: xxxxx  \n") );
        Display.print( F("  LDF: xxxxx  \n") );     
        Display.print( F("> Back         ") );
        break;
    }

    LastMenuItem = MenuItem;
  }

  if( RevTriggerBounce.rose() )
  {
    MenuItem++;
    if( MenuItem > 4 )
      MenuItem = 0;
  }

  if( FireHalfTriggerBounce.rose() )
  {
    switch( MenuItem )
    {
      case 0:
        FireOnEmptyMag = !FireOnEmptyMag;
        break;
      case 1:
        EStopOnPusherJam = !EStopOnPusherJam;
        break;
      case 2:
        break;
      case 3:
        break;
      default:
        CurrentConfigMenuPage = CONFIG_MENU_MAIN;
        break;
    }
    LastMenuItem = 99;
  }  
}

void Display_ScreenHeader( bool ClearScreen )
{
  static float LastBatteryVoltage = 99.0;
  static unsigned int LastTotalDartsFired = 9999;
  char Buffer[6];
  
  if( ClearScreen )
  {
    Display.clear();
  }
  // Do not clear screen, voltage has changed
  if( ClearScreen || ( (int)(LastBatteryVoltage*10) != (int)(BatteryCurrentVoltage*10) ) )
  {
    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 0);
    sprintf( Buffer, "%3d", (int)(BatteryCurrentVoltage * 10) );
    Buffer[4] = 0;
    Buffer[3] = Buffer[2];
    Buffer[2] = '.';
    //sprintf( Buffer, "%2.1f", BatteryCurrentVoltage );
    Display.print( Buffer );
    Display.print( F("v ") );
    Display.print( BatteryS );
    Display.print( F("s") );
    //DebugPrintln( BatteryCurrentVoltage );
    //DebugPrintln( Buffer );
  }

  
  if( ClearScreen || (LastTotalDartsFired != TotalDartsFired ) )
  {
    Display.setCursor( 70, 1 );
    sprintf(Buffer, "%5d", TotalDartsFired);
    Display.setFont(font5x7);
    //display.setRow( 1 );
    Display.print( "-> " );
    Display.print( Buffer );      
  }


  LastBatteryVoltage = BatteryCurrentVoltage;
  LastTotalDartsFired = TotalDartsFired;
}

void Display_Normal( bool ClearScreen )
{
  char Buffer[4];
  static int LastDartsInMag = 99;
  static byte LastCurrentFireMode = 99;
  static byte LastTargetDPS = 255;
  //static int LastSetMaxSpeed = 99;
  static byte LastBurstSize = 99;
  if( ClearScreen )
  {
    DebugPrintln( F("NORMAL MODE!!") );
  }
  if( ClearScreen || (CurrentFireMode != LastCurrentFireMode) || (BurstSize != LastBurstSize) )
  {  
    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 2);
    if( CurrentFireMode == FIRE_MODE_SINGLE )
    {
      Display.print( F("Single    ") );
    }
    else if( CurrentFireMode == FIRE_MODE_AUTO )
    {
      Display.print( F("Full Auto ") );
    }
    else
    {
      sprintf( Buffer, "%2d", BurstSize );
      Display.print( F("Burst: ") );
      Display.print( Buffer );
    }
  }
  
  if( ClearScreen || (TargetDPS != LastTargetDPS) )
  {
    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 4);
    Display.print( F("DPS: ") );
    if( TargetDPS == 99 )
      Display.print( F("MAX") );
    else 
    {
      sprintf( Buffer, "%2d", TargetDPS );
      Display.print( Buffer );     
    }
  }
  
  
  /*
  if( ClearScreen || (SetMaxSpeed != LastSetMaxSpeed) )
  {
    display.setFont(ZevvPeep8x16);
    display.setCursor(0, 6);
    sprintf( Buffer, "%3d", SetMaxSpeed );
    display.print( F("Pwr: ") );
    display.print( Buffer );
    display.print( "%" );
  }
  */
  if( ClearScreen || (DartsInMag != LastDartsInMag) )
  {
    Display.setFont(ZevvPeep8x16);
    Display.setCursor(90, 3);
    Display.set2X();
    sprintf( Buffer, "%2d", DartsInMag );
    Display.print( Buffer );
    Display.set1X();  
  }
  LastDartsInMag = DartsInMag;
  LastBurstSize = BurstSize;
  LastTargetDPS = TargetDPS;
  //LastSetMaxSpeed = SetMaxSpeed;
  LastCurrentFireMode = CurrentFireMode;
}

void Display_LowBatt( bool ClearScreen )
{
  if( ClearScreen )
  {
    DebugPrintln( F("LOW BATTERY!!") );
    DebugPrintln( BatteryCurrentVoltage );

    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 2);
    Display.print( F("################\n") );
    Display.print( F("# LOW BATTERY! #\n") );
    Display.print( F("################") ); 
  }
}

void Display_Darts_Out( bool ClearScreen )
{
  if( ClearScreen )
  {
    DebugPrintln( F("DARTS OUT!!") );

    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 2);
    Display.print( F("################\n") );
    Display.print( F("#  MAG EMPTY!  #\n") );
    Display.print( F("################") ); 
  }
}

void Display_MagOut( bool ClearScreen )
{
  if( ClearScreen )
  {
    DebugPrintln( F("MAG OUT!!") );

    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 2);
    Display.print( F("################\n") );
    Display.print( F("# MAG DROPPED! #\n") );
    Display.print( F("################") ); 
  }
}

void Display_EStop( bool ClearScreen )
{
  if( ClearScreen )
  {
    DebugPrintln( F("E STOP!!") );

    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 2);
    Display.print( F("################\n") );
    Display.print( F("#EMERGENCY STOP#\n") );
    Display.print( F("################") ); 
  }
}


/*
 * Debug Stuff
 * 
 */

void DebugInit()
{
  if( !DebugEnabled ) return;
  Serial.begin( 57600 );
  while( !Serial );
}

void DebugPrint( int Val )
{
  if( !DebugEnabled ) return;
  Serial.print( Val );  
}
void DebugPrint( float Val )
{
  if( !DebugEnabled ) return;
  Serial.print( Val );  
}
void DebugPrint( bool Val )
{
  if( !DebugEnabled ) return;
  Serial.print( Val );  
}
void DebugPrint( String Val )
{
  if( !DebugEnabled ) return;
  Serial.print( Val );  
}
void DebugPrint( char *Val )
{
  if( !DebugEnabled ) return;
  char CurrentChar;
  byte Index = 0;
  CurrentChar = *(Val + Index);
  while( (Index < 20) && (CurrentChar != 0) )
  {
    Serial.print( CurrentChar );
    Index ++;
    CurrentChar = *(Val + Index);
  }
}
void DebugPrint( const __FlashStringHelper* Val )
{
  if( !DebugEnabled ) return;
  Serial.print( Val );
  //Serial.print( (PGM_P)pgm_read_word(&(Val)) );  
}
void DebugPrintln( int Val )
{
  if( !DebugEnabled ) return;
  Serial.println( Val );  
}
void DebugPrintln( float Val )
{
  if( !DebugEnabled ) return;
  Serial.println( Val );  
}
void DebugPrintln( bool Val )
{
  if( !DebugEnabled ) return;
  Serial.println( Val );  
}
void DebugPrintln( String Val )
{
  if( !DebugEnabled ) return;
  Serial.println( Val );  
}
void DebugPrintln( char *Val )
{
  if( !DebugEnabled ) return;
  char CurrentChar;
  byte Index = 0;
  CurrentChar = *(Val + Index);
  while( (Index < 20) && (CurrentChar != 0) )
  {
    Serial.print( CurrentChar );
    Index ++;
    CurrentChar = *(Val + Index);
  }
  Serial.println( );
}
void DebugPrintln( const __FlashStringHelper* Val )
{
  if( !DebugEnabled ) return;
  //Serial.print( "WOTT: ");
  Serial.println( Val );
  //Serial.println( (PGM_P)pgm_read_word(&(Val)) );  
}
