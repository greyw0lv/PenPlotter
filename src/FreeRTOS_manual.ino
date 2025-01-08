#include <Arduino_FreeRTOS.h>


#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include <Keypad.h>

// Define hardware and settings
#define ENABLE_PIN 8
#define JOY_X A0
#define JOY_Y A1
#define JOY_BTN 12
#define PEN_BTN 22
#define PEN_SERVO_PIN 23
#define COLOR_SERVO_PIN 24

// Pen positions
#define PEN_UP_POS 135
#define PEN_DOWN_POS 90

// Define color switch positions
const float COLOR_POSITIONS[5] = {31, 65.5, 101, 137, 166.5}; // Servo angles for 5 colors

// Keypad configuration
const byte ROWS = 4; // Four rows
const byte COLS = 4; // Four columns
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {36, 38, 40, 42};   // Connect keypad row pins to these pins
byte colPins[COLS] = {37, 39, 41, 43};   // Connect keypad column pins to these pins
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

AccelStepper stepper1(1, 2, 5);
AccelStepper stepper2(1, 3, 6);
AccelStepper stepper3(1, 4, 7);
MultiStepper steppersControl;
Servo penServo;
Servo colorServo;

// Task priorities

#define PRIORITY_KEYPAD 4
#define PRIORITY_MOTION 3
#define PRIORITY_IDLE 0

//time measuring
//int timeJoyStart, timeJoyEnd, timeMotStart, timeMotEnd, timePenLiftStart, timePenLiftEnd, timePenLiftDiff;

bool joystickControl;
bool lastButtonState = HIGH;
bool lastPenButtonState = HIGH;
bool penDown = false;
const int deadzone = 50; // Deadzone threshold (adjust as needed)
const int center = 512;  // Center position for analog joystick

// Shared resources
QueueHandle_t gcodeQueue;   // Queue for G-code commands
SemaphoreHandle_t stepperMutex; // Mutex for stepper access

void joystickControlTask(void *pvParameters) {
    int xValue, yValue, speed1, speed2;

    for (;;) {
        if (joystickControl) { // Check the state of joystickControl
            xValue = analogRead(JOY_X);
            yValue = analogRead(JOY_Y);

          int speed1 = 0;
          int speed2 = 0;

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

      //  timePenLiftEnd = micros();
      //  timePenLiftDiff = timePenLiftStart -timePenLiftEnd;

        //Serial.print("The time to raise and lower the pen is (us)   : ");
        //Serial.println(timePenLiftDiff);
        delay(200);  // Debounce delay
      }
     // timePenLiftStart = micros();
      lastPenButtonState = currentPenButtonState;

            // Use mutex to safely access shared stepper resources
            if (xSemaphoreTake(stepperMutex, pdMS_TO_TICKS(10))) {
               // timeMotStart = micros();
                stepper1.setSpeed(-speed1);
                stepper2.setSpeed(speed2);
                stepper3.setSpeed(-speed2);
                stepper1.runSpeed();
                stepper2.runSpeed();
                stepper3.runSpeed();
               // timeMotEnd = micros();
               // timeJoyEnd = micros();
                //Serial.print("The time between motors responding to each other is (us): ");
                //Serial.println(timeMotEnd-timeMotStart);
                //Serial.print("The time between joystick input to motor input is (us): ");
                //Serial.println(timeJoyEnd-timeJoyStart);
                xSemaphoreGive(stepperMutex);
            }
           // timeJoyStart = micros();
        }

        vTaskDelay(pdMS_TO_TICKS(5)); // Run every 5 ms
    }
}

void gcodeProcessingTask(void *pvParameters) {
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
                    stepper1.runToPosition();
                    stepper2.runToPosition();
                    xSemaphoreGive(stepperMutex);
                }
            }
        }
    }
}

void keypadTask(void *pvParameters) {
    Keypad keypad = *(Keypad *)pvParameters;

    for (;;) {
        char key = keypad.getKey();
        if (key) {
            if (key >= '1' && key <= '5') {
                int colorIndex = key - '1';
                colorServo.write(COLOR_POSITIONS[colorIndex]); // Move to color position
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Check keypad every 50 ms
    }
}

void idleTask(void *pvParameters) {
    for (;;) {
        // Perform background tasks like power management
        vTaskDelay(pdMS_TO_TICKS(500)); // Run every 500 ms
    }
}

void setup() {
    // Initialize peripherals
    Serial.begin(9600);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);
    pinMode(JOY_BTN, INPUT_PULLUP);   // Joystick mode toggle button as input with pull-up
    pinMode(PEN_BTN, INPUT_PULLUP); 

    joystickControl =true;
    stepper1.setMaxSpeed(1000);
    stepper2.setMaxSpeed(1000);
    stepper3.setMaxSpeed(1000);

    penServo.attach(PEN_SERVO_PIN);
    colorServo.attach(COLOR_SERVO_PIN);
    penServo.write(PEN_UP_POS);

    gcodeQueue = xQueueCreate(10, 50);         // Queue for 10 G-code commands
    stepperMutex = xSemaphoreCreateMutex();   // Mutex for stepper access

  
    xTaskCreate(joystickControlTask, "JoystickControl", 256, NULL, PRIORITY_MOTION, NULL);
    xTaskCreate(keypadTask, "Keypad", 256, (void *)&keypad, PRIORITY_KEYPAD, NULL);
    xTaskCreate(idleTask, "Idle", 128, NULL, PRIORITY_IDLE, NULL);

    vTaskStartScheduler(); // Start FreeRTOS scheduler
}

void loop() {
    // RTOS handles the loop; this will not be used
}
