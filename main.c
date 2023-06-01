#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include "BluetoothSerial.h"  //Bluetooth library import

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define BUTTON_1_PIN 35 // Button 1 Pin 35
#define BUTTON_2_PIN 32 // Button 2 Pin 32

//Button contollers
struct Button {
  const uint8_t PIN;
  bool pressed;
};
int menu=1;
Button button1 = {32,false};
Button button2 = {35,false};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); //create OLED instance
Adafruit_BMP085 bme; //create BMP180 instance
BluetoothSerial SerialBT;  //create bluetooth instance

// Using for Thread
TaskHandle_t Task1; 
TaskHandle_t Task2;

//Using for temperature reading
String temperatureString = "";
unsigned long previousMillis = 0;  // Stores last time temperature was published
const long interval = 10000;       // interval at which to publish sensor readings

//intrupt handle function
void IRAM_ATTR isr() {
  button1.pressed = true;
}
void IRAM_ATTR isr2() {
  button2.pressed = true;
}





//Run at power On
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32");  //Bluetooth device name

  pinMode(button1.PIN, INPUT_PULLUP);
  pinMode(button2.PIN, INPUT_PULLUP);
  attachInterrupt(button1.PIN, isr, FALLING);
  attachInterrupt(button2.PIN, isr2, FALLING);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }

  //BMP180 startup
  bool status = bme.begin(0x77);
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }

  //OLED startup config
  delay(2000);
  display.clearDisplay();
  display.setTextColor(WHITE);


  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Task1code, /* Task function. */
    "Task1",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task1,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */
  delay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code, /* Task function. */
    "Task2",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task2,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */
  delay(500);
}

//Run as loop
//Task1code:Button & speedmeter(Underdevelopment)
void Task1code(void* pvParameters) {
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  if (button2.pressed) {
    Serial.println(F("Button 2 Pressed!"));
    button2.pressed = false;
    if(menu==0){
      menu=4;
    }
    else{
      menu--;
    }

  }
  if (button1.pressed) {
    Serial.println(F("Button 1 Pressed!"));
    button1.pressed = false;
    if(menu==4){
      menu=0;
    }
    else{
      menu++;
    }
  }
  Serial.println(menu);
}

//Run as loop
//Task2code:Display & bluetooth
void Task2code(void* pvParameters) {
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    display.clearDisplay();
    // display temperature
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Temperature: ");
    display.setTextSize(2);
    display.setCursor(0, 10);
    display.print(String(bme.readTemperature()));
    display.print(" ");
    display.setTextSize(1);
    display.cp437(true);
    display.write(167);
    display.setTextSize(2);
    display.print("C");
    display.display();
    unsigned long currentMillis = millis();
    // Send temperature readings
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      temperatureString =String(bme.readTemperature());
      SerialBT.println(temperatureString);
    }

    delay(1000);
  }
}

void loop() {
}
