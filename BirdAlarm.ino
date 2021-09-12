/* TODO
 * - Work on alarm condition -> LED
 * - Hook up buzzer (passive vs active??)
 * - Distance between sensors to detect birds instead of larger things
 */

#define print(value) Serial.println(value)
#define printStr(str) Serial.println((String)str)

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
 
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

template<int stateCount>
struct Timer {
  unsigned long durations[stateCount];
  unsigned int stateIdx;
  unsigned long startTime;
  unsigned long nowTime;
  bool locked;

  template<typename... Ints>
  Timer(Ints... durations) : durations{durations...}, stateIdx(0), startTime(0), nowTime(0), locked(false) {}

  void start(){
    startTime = now();
  }

  bool stop(bool resetTimer=true){
    if(!locked)nowTime = now();
    if(startTime > nowTime){
      start();
    }else if(isRunning() && nowTime - startTime >= durations[stateIdx]){
      if(resetTimer){
        reset();
        stateIdx = (stateIdx + 1) % stateCount;
      }
      return true;
    }
    return false;
  }

  unsigned long now(){
    auto t = millis();
    return t <= 0 ? 1 : t; // makes sure to never return 0 (unlikely case of overflow followed by reading)
  }

  bool isRunning(){
    return startTime > 0;
  }

  void reset(){
    startTime = 0;
  }

  void lock(){
    nowTime = now();
    locked = true;
  }

  String status(){
    return (startTime == 0 ? (String)"Not running" : (String)"Rem: " + (durations[stateIdx] - now() + startTime) + " ms" ) + " [" + stateIdx + "]";
  }
};

struct BiTimer: public Timer<2>{
  template<typename... Args>
  BiTimer(Args... args): Timer(args...){}
  bool isLow(){ return stateIdx == 0; }
  bool isHigh() { return stateIdx == 1; }
};

struct TriTimer: public Timer<3>{
  template<typename... Args>
  TriTimer(Args... args): Timer(args...){}
  bool isLoad() { return stateIdx == 0; }
  bool isHigh() { return stateIdx == 1; }
  bool isLow() { return stateIdx == 2; }
  void setLoad() { stateIdx = 0, reset(); }
};

constexpr int calPin = 6;
BiTimer calTimer(
  5000UL /* calibration time */,
  100UL /* LED time */
);

struct USSensor{

  String sensorName;
  int echoPin, trigPin, detectPin; // arduino pins to the sensor
  float distance; // median distance read from sensor
  float maxDist = 500, newMaxDist = 0; 
  unsigned long lastTrig; // time in millis since last trig
  bool updateLastTrig; // can update lastTrig

  constexpr static int sampleSize = 5; // num sample to calc median distance
  float distSample[sampleSize]; //samples for median distance calc
  int slot = 0; // index for next raw distance value in median calc

  constexpr static float maxDistRatio = 0.5;
  constexpr static float maxDistanceAllowed = 1000;

  TriTimer triggerTimer;

  USSensor(String sensorName, int trigPin, int echoPin, int detectPin):
    sensorName(sensorName), echoPin(echoPin), trigPin(trigPin), detectPin(detectPin),
    triggerTimer(3000UL /* wait */, 100UL /* LED */, 5000UL /* cooldown */), updateLastTrig(true)
    {
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(detectPin, OUTPUT); // Sets the detectPin as an OUTPUT
    pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

    digitalWrite(trigPin, LOW);
  }

  void update(){
    // Clears the trigPin condition
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(trigPin, LOW);
    read();
  }

  void read(){
    // Calculating the distance
    float rawDistance = pulseIn(echoPin, HIGH) * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    if(rawDistance > maxDistanceAllowed)
      return;

    // get the median distance from previous measurements
    distSample[slot] = rawDistance;
    slot = (slot + 1) % sampleSize;
    distance = getMedianDistance();

    //update max distance if its callibration time
    newMaxDist = rawDistance > newMaxDist ? rawDistance : newMaxDist;
    // printStr(sensorName + ": " + (calTimer.stop(false) ? "STOP" : "--") + " " + (calTimer.isHigh() ? "HIGH" : "LOW"));
    if(calTimer.stop(false) && calTimer.isHigh()){
      printStr(sensorName + ": CALIBRATED!");
      maxDist = newMaxDist * maxDistRatio;
      newMaxDist = 0;
    }

    if(triggerTimer.stop()){ // timer ended: go to next stage
      if(!triggerTimer.isLoad()){ // on and cooldown
        triggerTimer.start();
      }
      trigger();
    }else if(distance < maxDist){
      if(triggerTimer.isLoad() && !triggerTimer.isRunning()){
        triggerTimer.start();
      }
    }else if(triggerTimer.isLoad()){
      triggerTimer.reset();
    }
  }

  void trigger(){
    if(triggerTimer.isHigh()){
      printStr("##" + sensorName + " triggered##");
      digitalWrite(detectPin, HIGH);
      if(updateLastTrig) lastTrig = triggerTimer.now();
    }else if(triggerTimer.isLow()){
      digitalWrite(detectPin, LOW);
    }
  }

  float getMedianDistance(){
    float sortSample[sampleSize];
    memcpy(sortSample, distSample, sizeof(distSample));
    sort(sortSample, 0, sampleSize - 1);
    return sortSample[sampleSize/2];
  }

  String status(){
    return sensorName + " | " + distance + " / " + maxDist + " " + newMaxDist + " cm | Timer: " + triggerTimer.status();
  }
};

USSensor sensors[2] = {
  USSensor("Green", 2,3,7), //bottom
  USSensor("Blue", 4,5,8) //top
};

struct ActiveBuzzer{
  int buzzPin;
  TriTimer buzzTimer;

  constexpr static unsigned long trigGap = 5000; //ms
  ActiveBuzzer(int buzzPin):
    buzzPin(buzzPin),
    buzzTimer(5000UL /* wait */, 20UL /* high */, 50UL /* low */){
    pinMode(buzzPin,OUTPUT);//initialize the buzzer pin as an output
  }

  void buzzTrigger(){
    digitalWrite(buzzPin,HIGH);
    sensors[0].lastTrig = sensors[1].lastTrig;
  }

  void update(){
    if(buzzTimer.stop()){ // timer ended: go to next stage
      if(!buzzTimer.isLoad()){ // on and cooldown
        buzzTimer.start();
      }
      buzzTrigger();
    }else if(sensors[0].lastTrig - sensors[1].lastTrig >= trigGap){
      if(buzzTimer.isLoad() && !buzzTimer.isRunning()){
        buzzTimer.start();
      }
    }else if(buzzTimer.isLoad()){
      buzzTimer.reset();
    }
  }

};

ActiveBuzzer buzzer(9);

int totalTick;
constexpr int tickSize = 10;
int ticks[tickSize];
int tickIdx;

void setup() {  

  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("##Start##");

  totalTick = 0, tickIdx = 0;
  memset(ticks, 0, sizeof(ticks));

  // printStr("CAL initializing: " + calTimer.status());
  calTimer.start();
  // printStr("CAL initialized: " + calTimer.status());
  
}

void loop() {  
  int loopStartTime = millis();

  // print("##------------------------------##");

  calTimer.lock();

  for(auto& sensor: sensors) sensor.update();
  sensors[0].updateLastTrig = sensors[1].distance >= sensors[0].distance;
  printStr("Sensors: " + sensors[0].status() + " || " + sensors[1].status());

  // buzzer.update();

  if(calTimer.stop()){
    
    if(calTimer.isHigh()) printStr("##CALIBRATE##");
    digitalWrite(calPin, calTimer.isLow() ? LOW : HIGH); 
    calTimer.start();
  }
  else{
    // printStr("CAL: " + calTimer.status());
  }

  calTimer.locked = false;

  totalTick -= ticks[tickIdx];
  ticks[tickIdx] = millis() - loopStartTime;
  totalTick += ticks[tickIdx];
  // printStr("Tick Time: " + (totalTick / tickSize) + " ms | raw: " + ticks[tickIdx]);
  tickIdx = (tickIdx + 1) % tickSize;
}
