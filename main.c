#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include "BluetoothSerial.h"  //Bluetooth library import

//MAX10302
#include "MAX30105.h"
#include "heartRate.h"



//----Current implemented-----//
//Hall effect
//OLED
//Bluetooth

//-----To Do
//Heart Rate
//Temperature
//Preasure altitude
//Humidity
//

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define BUTTON_1_PIN 35   // Button 1 Pin 35
#define BUTTON_2_PIN 32   // Button 2 Pin 35
#define HALL_EFFECT 33    //Hall effects

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);  //create OLED instance

Adafruit_BMP085 bme;  //create BMP180 instance

BluetoothSerial SerialBT;  //create bluetooth instance
//Max30102 start
MAX30105 particleSensor;   //create MAX10302 instance
const byte RATE_SIZE = 4;  //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];     //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;  //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
//Max30102 end


// Using for Thread
TaskHandle_t Task1;
TaskHandle_t Task2;

//Using for temperature & altitude reading
String altString = "";
String temperatureString = "";
unsigned long previousMillis = 0;  // Stores last time temperature was published
const long interval = 10000;       // interval at which to publish sensor readings

//Run at poweron
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32");  //Bluetooth device name
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(HALL_EFFECT, INPUT_PULLUP);  //Hall effects
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

  // Initialize MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))  //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1)
      ;
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup();                     //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A);  //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);   //Turn off Green LED



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

  for (;;) {
    long irValue = particleSensor.getIR();

    if (checkForBeat(irValue) == true) {

      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;  //Store this reading in the array
        rateSpot %= RATE_SIZE;                     //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }

    byte button_1_State = digitalRead(BUTTON_1_PIN);
    byte button_2_State = digitalRead(BUTTON_2_PIN);
    byte halleffect = digitalRead(HALL_EFFECT);  //Hall effect

    if (button_1_State == LOW) {
      Serial.println("Button 1 is pressed");
    }
    delay(100);

    if (button_2_State == LOW) {
      Serial.println("Button 2 is pressed");
    }

    if (halleffect == LOW) {  //Hall effect
      Serial.println("Hall Effect detected");
      SerialBT.println("Hall Effect detected");
    }
    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print("\nBPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);
    Serial.print("\n");
    delay(100);


    delay(100);
  }
}

//Run as loop
//Task2code:Display & bluetooth
void Task2code(void* pvParameters) {
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {


    display.clearDisplay();
    // display temperature on OLED
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

    // Send temperature readings via bluetooth communication
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      temperatureString = String(bme.readTemperature());
      SerialBT.println(temperatureString);
      //Altitude
      altString = String(bme.readAltitude());
      SerialBT.println(altString);
    }


    delay(1000);
  }
}

void loop() {
}