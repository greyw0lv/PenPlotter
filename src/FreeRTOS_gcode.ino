#include <Arduino_FreeRTOS.h>


#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include <Keypad.h>
#include "gcode.h"

// Define hardware and settings
#define ENABLE_PIN 8
#define JOY_X A0
#define JOY_Y A1
//#define JOY_BTN 12
#define PEN_BTN 22
#define PEN_SERVO_PIN 23
#define COLOR_SERVO_PIN 24

// Pen positions
#define PEN_UP_POS 135
#define PEN_DOWN_POS 90

//Limit switch pins
#define LIMIT_X 50
#define LIMIT_Y 51

//test for real-time constraints
int startTime, endTime, timeDiff;

// Define color switch positions
const float COLOR_POSITIONS[5] = { 31, 65.5, 101, 137, 166.5 };  // Servo angles for 5 colors

// Keypad configuration
const byte ROWS = 4;  // Four rows
const byte COLS = 4;  // Four columns
char keys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};
byte rowPins[ROWS] = { 36, 38, 40, 42 };  // Connect keypad row pins to these pins
byte colPins[COLS] = { 37, 39, 41, 43 };  // Connect keypad column pins to these pins
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

AccelStepper stepper1(1, 2, 5);
AccelStepper stepper2(1, 3, 6);
AccelStepper stepper3(1, 4, 7);
MultiStepper steppersControl;

Servo penServo;
Servo colorServo;

// Task priorities
#define PRIORITY_KEYPAD 4
#define PRIORITY_PEN_UPDOWN 3
#define PRIORITY_MOTION 2
#define PRIORITY_IDLE 0

//#define PRIORITY_EMERGENCY_STOP 10
//#define PRIORITY_KEYPAD_GETKEY 1
//#define PRIORITY_MODE_SWITCH 8
//#define PRIORITY_HOMING 9
//#define PRIORITY_COLOR_SWITCH 8
// #define PRIORITY_GCODE 1



bool joystickControl;
const int deadzone = 20;  // Deadzone threshold (adjust as needed)
const int center = 512;   // Center position for analog joystick

// Shared resources
QueueHandle_t gcodeQueue;        // Queue for G-code commands
SemaphoreHandle_t stepperMutex;  // Mutex for stepper access

//long gotoposition[2];            // Target positions for each stepper motor
bool joystickMode = false;       // Start in joystick control mode
bool lastButtonState = HIGH;     // Track the last joystick mode button state
bool lastPenButtonState = HIGH;  // Track the last pen control button state
bool penDown = false;            // Pen state (true: down, false: up)

int gcodeIndex = 0;
int gcodeCommandCount = sizeof(gcode) / sizeof(gcode[0]);
long targetPosition[3] = { 0, 0, 0 };

void joystickControlTask(void *pvParameters) {
  int xValue, yValue, speed1, speed2;
  
  for (;;) {
    //debug("balls", "balls");
    if (joystickControl) {  // Check the state of joystickControl
      
      xValue = analogRead(JOY_X);
      yValue = analogRead(JOY_Y);
      //debug("x", JOY_X);
      //debug("y", JOY_Y);

      speed1 = 0;
      speed2 = 0;

      if (abs(xValue - center) > deadzone) {
        speed1 = map(xValue, 0, 1023, -1000, 1000);
      }

      if (abs(yValue - center) > deadzone) {
        speed2 = map(yValue, 0, 1023, -1000, 1000);
      }

      // Check pen control button
      bool currentPenButtonState = digitalRead(PEN_BTN);
      if (currentPenButtonState == LOW && lastPenButtonState == HIGH) {  // Button pressed
        penDown = !penDown;                                              // Toggle pen state
        penServo.write(penDown ? PEN_DOWN_POS : PEN_UP_POS);             // Move pen

        //endTime = micros();
        //timeDiff = endTime-startTime;

        //Serial.print("The time to raise and lower the pen is: ");
        //Serial.println(timeDiff);
        delay(200);  // Debounce delay
      }
      //startTime = micros();
      lastPenButtonState = currentPenButtonState;

      // Use mutex to safely access shared stepper resources
      if (xSemaphoreTake(stepperMutex, pdMS_TO_TICKS(10))) {
        //debug("hI", speed2);
        stepper1.setSpeed(-speed1);
        stepper2.setSpeed(speed2);
        stepper3.setSpeed(-speed2);
        stepper1.runSpeed();
        stepper2.runSpeed();
        stepper3.runSpeed();
        xSemaphoreGive(stepperMutex);
      }
      //Serial.print(speed1);
      //Serial.print("   ");
      //Serial.println(speed2);
    }

    vTaskDelay(pdMS_TO_TICKS(10));  // Run every 10 ms
  }
}


/*void gcodeProcessingTask(void *pvParameters) {
  char commandBuffer[50];

  for (;;) {
    if (xQueueReceive(gcodeQueue, commandBuffer, portMAX_DELAY)) {
      // Parse and execute G-code command
      if (strstr(commandBuffer, "G0") || strstr(commandBuffer, "G1")) {
        // Linear move
        long positions[2];
        sscanf(commandBuffer, "G0 X%ld Y%ld", &positions[0], &positions[1]);

        if (xSemaphoreTake(stepperMutex, portMAX_DELAY)) {
          stepper1.moveTo(positions[0]);
          stepper2.moveTo(positions[1]);
          stepper3.moveTo(-positions[1]);
          stepper1.runToPosition();
          stepper2.runToPosition();
          stepper3.runToPosition();
          xSemaphoreGive(stepperMutex);
        }
      }
    }
  }
}*/

void keypadTask(void *pvParameters) {
  Keypad keypad = *(Keypad *)pvParameters;

  for (;;) {
    char key = keypad.getKey();
    if (key) {
      if (key >= '1' && key <= '5') {
        int colorIndex = key - '1';
        penServo.write(PEN_UP_POS);
        delay(50);
        colorServo.write(COLOR_POSITIONS[colorIndex]);  // Move to color position
      }
      if (key == '9') {
        if (joystickControl == true)
          joystickControl = false;
        else { joystickControl = true; }
        Serial.println(joystickControl);
        delay(200);
      }
      if (key == '0') {
        //Run homing function
        while (digitalRead(LIMIT_Y) == 0) {
          //Serial.println(digitalRead(LIMIT_Y));
          stepper2.setSpeed(-500);
          stepper3.setSpeed(500);
          stepper2.runSpeed();
          stepper3.runSpeed();
        }
        //Serial.println("stop");
        stepper2.stop();
        stepper3.stop();

        while (digitalRead(LIMIT_X) == 0) {
          stepper1.setSpeed(500);
          stepper1.runSpeed();
        }
        stepper1.stop();
        stepper1.setCurrentPosition(0);
        stepper2.setCurrentPosition(0);
        stepper3.setCurrentPosition(0);
      }
      
      if (key == '7' && joystickControl == false) {
        //Run gcode
        for (; gcodeIndex < gcodeCommandCount;) {
          //Serial.println(gcode[gcodeIndex]);
          executeGcode(gcode[gcodeIndex]);
          gcodeIndex++;
        }
        penServo.write(PEN_UP_POS);
        //Serial.println("G-code Complete");
        gcodeIndex = 0;
      }
      
    }
  }

  vTaskDelay(pdMS_TO_TICKS(50));  // Check keypad every 50 ms
}


void idleTask(void *pvParameters) {
  for (;;) {
    // Perform background tasks like power management
    vTaskDelay(pdMS_TO_TICKS(500));  // Run every 500 ms
  }
}

void debug(String val, int valname) {
  Serial.println("**************");
  Serial.print(val);
  Serial.print(": ");
  Serial.println(valname);
  Serial.println("**************");
}

void executeGcode(String command) {

  command.trim();


  if (command.startsWith("G0") || command.startsWith("G1")) {
    parseLinearGcode(command);

    // Check for Z parameter in linear movement commands (G01 Z)
    int zIndex = command.indexOf('Z');
    if (zIndex >= 0) {
      float zValue = command.substring(zIndex + 1, command.indexOf(' ', zIndex)).toFloat();

      if (zValue < 0) {  // Pen down condition
        penServo.write(PEN_DOWN_POS);
        delay(250);
        penDown = true;
      } else if (zValue > 0) {  // Pen up condition
        penServo.write(PEN_UP_POS);
        delay(250);
        penDown = false;
      }
    }

    steppersControl.moveTo(targetPosition);
    steppersControl.runSpeedToPosition();

  } else if (command.startsWith("M3")) {
    penServo.write(PEN_DOWN_POS);
    penDown = true;
    delay(200);
  } else if (command.startsWith("M5")) {
    penServo.write(PEN_UP_POS);
    penDown = false;
    delay(100);
  } else if (command.startsWith("T")) {
    penServo.write(PEN_UP_POS);
    penDown = false;
    delay(300);
    colorServo.write(COLOR_POSITIONS[random(0, 5)]);
    delay(300);
    penServo.write(PEN_DOWN_POS);
    penDown = true;
    delay(300);
    
    //handleToolChange(command);
  } else {
    //Serial.print("Unsupported G-code command: ");
    //Serial.println(command);
  }
}

void parseLinearGcode(String command) {
  int xIndex = command.indexOf('X');
  int yIndex = command.indexOf('Y');

  if (xIndex >= 0) {
    String xValue = command.substring(xIndex + 1, command.indexOf(' ', xIndex));
    debug("X", xValue.toInt());
    targetPosition[1] = xValue.toInt()*10;  // Parse X position
    targetPosition[2] = -xValue.toInt()*10;
  }

  if (yIndex >= 0) {
    String yValue = command.substring(yIndex + 1, command.indexOf(' ', yIndex));
    targetPosition[0] = -yValue.toInt()*10;  // Parse Y position
  }
}

void setup() {
  // Initialize peripherals
  Serial.begin(9600);
  pinMode(LIMIT_X, INPUT_PULLUP);
  pinMode(LIMIT_Y, INPUT_PULLUP);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  pinMode(PEN_BTN, INPUT_PULLUP);

  joystickControl = true;
  stepper1.setMaxSpeed(1000);
  stepper2.setMaxSpeed(1000);
  stepper3.setMaxSpeed(1000);

  steppersControl.addStepper(stepper1);
  steppersControl.addStepper(stepper2);
  steppersControl.addStepper(stepper3);

  penServo.attach(PEN_SERVO_PIN);
  colorServo.attach(COLOR_SERVO_PIN);

  gcodeQueue = xQueueCreate(10, 50);       // Queue for 10 G-code commands
  stepperMutex = xSemaphoreCreateMutex();
  
  penServo.write(PEN_UP_POS);  

 
  xTaskCreate(joystickControlTask, "JoystickControl", 256, NULL, PRIORITY_MOTION, NULL);
  xTaskCreate(keypadTask, "Keypad", 256, (void *)&keypad, PRIORITY_KEYPAD, NULL);
  xTaskCreate(idleTask, "Idle", 128, NULL, PRIORITY_IDLE, NULL);



  vTaskStartScheduler();  // Start FreeRTOS scheduler
}

void loop() {
  // RTOS handles the loop; this will not be used
}
