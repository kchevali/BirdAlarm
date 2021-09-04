/* TODO
 * - Work on alarm condition -> LED
 * - Hook up buzzer (passive vs active??)
 * - Distance between sensors to detect birds instead of larger things
 */

constexpr int refreshRate = 10000;
int refreshIndex = 0; //callibration cycle - connects to maxDist in sensor
long long clck = 0;
int trigGap = 400; // num of frames before and after only bottom is triggered
int trigWait = 150;

constexpr int calPin = 6;
constexpr int ledLength = 5;
 
void swap(float& a, float& b){
  float t = a;
  a = b;
  b = t;
}

void sort(float* arr, int a, int b){
  if(a >= b) return;
  int i = a;
  for(int j = a; j <= b; j++){
    if(arr[j] <= arr[b])
      swap(arr[i++], arr[j]);
  }
 sort(arr, a, i-2);
 sort(arr, i, b);
  
}

struct USSensor{

  String sensorName;
  int echoPin, trigPin, detectPin;
  long duration;
  float distance;
  long long lastTrig;

  constexpr static int sampleSize = 20;
  float distSample[sampleSize];
  float median = 0;
  int slot = 0;

  float maxDist = 500, newMaxDist = 0; 
  int triggerIndex = 0, cooldownIndex = 0;
  bool firstTriggerRound = false;

  constexpr static float threshold = 0.5;
  constexpr static int cooldownRate = 50;
  constexpr static float maxDistanceAllowed = 1000;

  USSensor(String sensorName, int trigPin, int echoPin, int detectPin): sensorName(sensorName), echoPin(echoPin), trigPin(trigPin), detectPin(detectPin){
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(detectPin, OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  }

  void update(){
    // Clears the trigPin condition
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

    if(distance > maxDistanceAllowed)
      return;

    distSample[slot] = distance;
    slot = (slot + 1) % sampleSize;

    updateMedian();

    //update max distance if its callibration
    newMaxDist = distance > newMaxDist ? distance : newMaxDist;
    if(refreshIndex == 0){
      maxDist = newMaxDist;
      newMaxDist = 0;
    }

    if(isTriggered()){
      triggerIndex = (triggerIndex + 1) % ledLength;
      lastTrig = clck;
    }else if(triggerIndex > 0 && triggerIndex < (ledLength - 1)){
      triggerIndex = ledLength - 1;
    }else{
      triggerIndex = 0;
    }
  }

  void finalUpdate(){
    if(isStartTriggered()){
      cooldownIndex = 1;
    }else if (cooldownIndex > 0){
      cooldownIndex = (cooldownIndex + 1) % cooldownRate;
    }
  }

  bool isTriggered(){
    return median < (maxDist * threshold);
  }

  bool isStartTriggered(){
    return triggerIndex == 1 && cooldownIndex == 0;
  }

  bool isEndTriggered(){
    return triggerIndex == ledLength - 1;
  }

  void updateMedian(){
    float sortSample[sampleSize];
    memcpy(sortSample, distSample, sizeof(distSample));
    sort(sortSample, 0, sampleSize - 1);
    median = sortSample[sampleSize/2];
  }

  void print() const{
    Serial.print(median);
    Serial.print(" cm ");
  }
};

USSensor sensors[2] = {
  USSensor("Green", 2,3,7), //bottom
  USSensor("Blue", 4,5,8) //top
};

struct ActiveBuzzer{
  int buzzPin;
  long long lastTrigBottom;
  bool recentNotTop;
  ActiveBuzzer(int buzzPin): buzzPin(buzzPin), lastTrigBottom(0), recentNotTop(false){
    pinMode(buzzPin,OUTPUT);//initialize the buzzer pin as an output
  }

  void buzzTrigger(){
    digitalWrite(buzzPin,HIGH);
    delay(50);//wait for 1ms
    digitalWrite(buzzPin,LOW);
    delay(20);//wait for 1ms
  }

  void update(){
    if(sensors[0].lastTrig-sensors[1].lastTrig>= trigGap){ // top has not been triggered recently
      if(!recentNotTop){
        recentNotTop = true;
        lastTrigBottom = sensors[0].lastTrig;
      }
      
      if(clck - lastTrigBottom>=trigWait){ // 
        for(int i = 0; i < 5; i++) buzzTrigger();
        sensors[1].lastTrig = clck;
      }
    }else{
      recentNotTop = false;
    }
    Serial.println((String)"Sensor Gap: " + (int)(sensors[0].lastTrig - sensors[1].lastTrig) + " Past Gap: " +  (int)(clck - lastTrigBottom));
  }
  
};

ActiveBuzzer buzzer(9);

void setup() {  

  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");
  
}

void loop() {  
//  Serial.println((String)"Dist: " + sensors[0].median + " " + sensors[1].median + " MAX: " + sensors[0].maxDist + " " + sensors[1].maxDist+ " CAL: " + (100 * refreshIndex / refreshRate) + "%");
  
  //turn on LED when it recallibrates
  if(refreshIndex == 0){
    digitalWrite(calPin, HIGH); 
    Serial.println("CALLIBRATE");  
  }else if(refreshIndex == ledLength){
    digitalWrite(calPin, LOW);   
  }
  
  for(auto& sensor: sensors){
    sensor.update();
  }

//  Serial.println(sensors[0].sensorName + ": trigger = " + sensors[0].triggerIndex);
//  Serial.print("Distance: ");
  for(auto& sensor: sensors){
//    sensor.print();
    if(sensor.isStartTriggered()){
      sensor.firstTriggerRound = true;
      Serial.println(sensor.sensorName + ": TRIGGER - Dist: " + sensor.median + " Max: " + sensor.maxDist + " CAL: " + (100 * refreshIndex / refreshRate) + "%");
      digitalWrite(sensor.detectPin, HIGH); 
    }else if(sensor.isEndTriggered() && sensor.firstTriggerRound){
      sensor.firstTriggerRound = false;
//      Serial.println(sensor.sensorName + ": END");
      digitalWrite(sensor.detectPin, LOW); 
    }
  }

  buzzer.update();
//   Serial.println("");

   for(auto& sensor: sensors){
    sensor.finalUpdate();
  }
//  Serial.println((String)"Refresh: " + refreshIndex);
  refreshIndex = (refreshIndex + 1) % refreshRate;
  
  clck++;
}
