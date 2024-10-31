#include <DHT.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


#define DHT22_PIN  32 // ESP32 pin GPIO21 connected to DHT22 sensor

Adafruit_MPU6050 mpu;
const int TriggerPin = 5;
const int EchoPin = 18;
#define SPEED_OF_SOUND 0.034

const char* ssid = "Wokwi-GUEST";
const char* password = "";


// MQTT Broker
const char* mqtt_server = "broker.hivemq.com";

WiFiClient espClient;
PubSubClient client(espClient);
long durationMeasurement = 0;
int distanceinCm = 0;

const int mqtt_port = 1883;

DHT dht22(DHT22_PIN, DHT22);

/*Main Setup*/
void setup() {
/*Setup of serial output*/
Serial.begin(9600);

/*Setup MPU6050 sensor*/
InitializeMPU6050();

SetupGyroRange();
SetupAcceloRange();
SetupBandwidth();

/*Setup of untrasonic sensor*/
  pinMode(TriggerPin, OUTPUT);
  pinMode(EchoPin, INPUT);

 /*Setup of temp and humidity sensor*/
  dht22.begin(); // se initializeaza senzorul DHT22

/*Setup of WI fi connection*/
  setup_wifi();

/*Setup of MQTT server client */
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

}


void InitializeMPU6050()
{
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  else
  {
    Serial.println("MPU6050 Found!");
  }
}


void SetupGyroRange()
{
 mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }
}

void SetupAcceloRange()
{
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
}


void SetupBandwidth()
{
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) 
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}
void mpuloop()
{
  char accelerationX[8];
  char accelerationY[8];
  char accelerationZ[8];

  char gyroXchar[8];
  char gyroYchar[8];
  char gyroZchar[8];

  char tempMPU[8];
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  dtostrf(a.acceleration.x, 1, 2, accelerationX);
  dtostrf(a.acceleration.y, 1, 2, accelerationY);
  dtostrf(a.acceleration.z, 1, 2, accelerationZ);
  client.publish("esp32/accel","Accelerometer:");
  client.publish("esp32/accel",accelerationX);
  client.publish("esp32/accel",accelerationY);
  client.publish("esp32/accel",accelerationZ);


  dtostrf(g.gyro.x, 1, 2, gyroXchar);
  dtostrf(g.gyro.y, 1, 2, gyroYchar);
  dtostrf(g.gyro.z, 1, 2, gyroZchar);
  client.publish("esp32/gyro","Rotation:");
  client.publish("esp32/gyro",gyroXchar);
  client.publish("esp32/gyro",gyroYchar);
  client.publish("esp32/gyro",gyroZchar);

  dtostrf(temp.temperature, 1, 2, tempMPU);
  client.publish("esp32/tempMPU","TemperatureMPU:");
  client.publish("esp32/tempMPU",tempMPU);
}


void distancesensorloop()
{
  char distanceCmChar[8];
  digitalWrite(TriggerPin, LOW);
  delayMicroseconds(2);

  digitalWrite(TriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TriggerPin, LOW);

  durationMeasurement = pulseIn(EchoPin, HIGH);

  distanceinCm = durationMeasurement * SPEED_OF_SOUND/2;
  //dtostrf(distanceinCm, 1, 0, distanceCmstring);
  itoa(distanceinCm, distanceCmChar, 10);
  client.publish("esp32/dist", "Distanta in cm:");
  client.publish("esp32/dist",distanceCmChar);

}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to Wifi");

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print(topic);
  Serial.print(" ");
  String messageTempVar;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTempVar += (char)message[i];
  }
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempting  to connect with the ESP client
    if (client.connect("ESP32Client-AndreiMircea" )) {
      Serial.println("connected");
      // Subscribe on topics
      Serial.println("subscribe to temperature topic");
      client.subscribe("esp32/temp");
      Serial.println("subscribe to humidity topic");
      client.subscribe("esp32/hum");
      Serial.println("subscribe to distance topic");
      client.subscribe("esp32/dist");
      Serial.println("subscribe to gyroscope topic");
      client.subscribe("esp32/gyro");
      Serial.println("subscribe to accelometer topic");
      client.subscribe("esp32/accel");
      Serial.println("subscribe to tempMPU topic");
      client.subscribe("esp32/tempMPU");
    } else {
      Serial.print("failed, error state=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void MQTTloop()
{
   if (!client.connected()) {
    reconnect();
  }
  client.loop();
}


void loop() {

  char tempstring[8];
  char humstring[8];
   // read humidity
  float humi  = dht22.readHumidity();
  // read temperature in Celsius
  float tempC = dht22.readTemperature();

  //loop for MQTT connection
  MQTTloop();

  // check if the temperature and humidity can be read
  if ( isnan(tempC)  || isnan(humi)) {
    Serial.println("Failed to read from DHT22 sensor!");
  } 
  else {
    //Convert from float to char
    dtostrf(tempC, 1, 2, tempstring);
    //Convert from float to char   
    dtostrf(humi, 1, 2, humstring);
    //Publish on MQTT topic
    client.publish("esp32/temp", "Temperatura:");
    client.publish("esp32/temp",tempstring);
    //Publish on MQTT topic
    client.publish("esp32/hum", "Umiditate:");
    client.publish("esp32/hum",humstring);
  }
  distancesensorloop();
  mpuloop();

  // wait 500 ms 
  delay(500);
}

