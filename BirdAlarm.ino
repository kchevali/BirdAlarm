/* TODO
 * - Work on alarm condition -> LED
 * - Hook up buzzer (passive vs active??)
 * - Distance between sensors to detect birds instead of larger things
 */

#define print(value) Serial.println((value))
#define printStr(str, value) Serial.println((String)(str) + value)

#define SENSOR 1
#define BUZZER 1

unsigned long now;
bool calibrated;

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
  unsigned long offset;

  template<typename... Ints>
  Timer(Ints... durations) : durations{durations...}, stateIdx(0), startTime(0), offset(0) {
    for(int i = 3; i < stateCount; i++){
      this->durations[i] = this->durations[2 - (i % 2)];
    }
  }

  void start(){
    startTime = 1;
    setRemainingTime(durations[stateIdx]);
  }

  bool stop(bool modify=true){
    if(isRunning() && nowTime() >= durations[stateIdx] + startTime){
      if(modify){
        reset();
        stateIdx = (stateIdx + 1) % stateCount;
      }
      return true;
    }
    return false;
  }

  void setRemainingTime(unsigned long t){
    if(durations[stateIdx] + startTime < t + now) startTime = t;
    offset = durations[stateIdx] + startTime - t - now;
//    printStr("REM | Running: ", (isRunning() ? "YES" : "NO") + " " + status() + " Trigger: " + (nowTime()  >= durations[stateIdx] + startTime ? "YES" : "NO") + " Now: " + nowTime() + " Start: " + startTime + " Diff: " + (nowTime() - startTime));
  }

  bool isRunning(){
    return startTime > 0;
  }

  void reset(){
    startTime = 0;
  }

  unsigned long nowTime(){
    return now + offset;
  }

  String status(){
    return (startTime == 0 ? (String)"Not running" : (String)"Rem: " + (durations[stateIdx] - nowTime() + startTime) + " ms" ) + " [" + stateIdx + "]";
  }

  bool isLoad() { return stateIdx == 0; }
  virtual bool isHigh() { return (stateIdx % 2) == 1; }
  virtual bool isLow() { return stateIdx > 0 && (stateIdx % 2) == 0; }
};

struct BiTimer: public Timer<2>{
  template<typename... Args>
  BiTimer(Args... args): Timer(args...){}
  bool isLow(){ return stateIdx == 0; }
  bool isHigh() { return stateIdx == 1; }
};

constexpr int calPin = 6;
BiTimer calTimer(
  600000UL /* calibration time */,
  100UL /* LED time */
);

#ifdef SENSOR
struct USSensor{

  String sensorName;
  int echoPin, trigPin, detectPin; // arduino pins to the sensor
  float distance; // median distance read from sensor
  float maxDist = 500, newMaxDist = 0; 
  unsigned long lastTrig; // time in millis since last trig
  bool updateLastTrig; // can update lastTrig

  constexpr static int sampleSize = 10; // num sample to calc median distance
  float distSample[sampleSize]; //samples for median distance calc
  int slot = 0; // index for next raw distance value in median calc

  constexpr static float maxDistRatio = 0.5;
  constexpr static float maxDistanceAllowed = 1000;

  Timer<3> triggerTimer;

  USSensor(String sensorName, int trigPin, int echoPin, int detectPin, unsigned long waitTime):
    sensorName(sensorName), echoPin(echoPin), trigPin(trigPin), detectPin(detectPin),
    triggerTimer(waitTime /* wait */, 100UL /* LED */, 5000UL /* cooldown */), updateLastTrig(true)/* needs to callibrate first*/
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
    // printStr(sensorName, ": " + (calTimer.stop(false) ? "STOP" : "--") + " " + (calTimer.isHigh() ? "HIGH" : "LOW"));
    if(calTimer.stop(false) && calTimer.isHigh()){
      printStr(sensorName, ": CALIBRATED!");
      maxDist = newMaxDist * maxDistRatio;
      newMaxDist = 0;
    }
    if(!calibrated) return;

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
      printStr("##", sensorName + " triggered##");
      digitalWrite(detectPin, HIGH);
      if(updateLastTrig) lastTrig = now;
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
  USSensor("Green", 2,3,7, 3000UL /* wait time */), //top
  USSensor("Blue", 4,5,8, 1000UL /* wait time */) //bottom
};
#endif

#ifdef BUZZER
struct ActiveBuzzer{
  int buzzPin;
  Timer<9> buzzTimer;

  constexpr static unsigned long trigGap = 5000; //ms
  ActiveBuzzer(int buzzPin):
    buzzPin(buzzPin),
    buzzTimer(5000UL /* wait */, 20UL /* high */, 10UL /* low */){
    pinMode(buzzPin,OUTPUT);//initialize the buzzer pin as an output
    digitalWrite(buzzPin,LOW);
  }

  void buzzTrigger(){
    digitalWrite(buzzPin,HIGH);
    sensors[0].lastTrig = sensors[1].lastTrig;
  }

  void update(){
    if(buzzTimer.stop()){ // timer ended: go to next stage
      if(!buzzTimer.isLoad()){ // on and cooldown
        buzzTimer.start();
        if(buzzTimer.isHigh()){
          buzzTrigger();
        }else{
          digitalWrite(buzzPin,LOW);
        }
      }
    }else if(sensors[0].lastTrig  >= trigGap + sensors[1].lastTrig){
      if(buzzTimer.isLoad() && !buzzTimer.isRunning()){
        buzzTimer.start();
      }
    }else if(buzzTimer.isLoad()){
      buzzTimer.reset();
    }
  }

  String status(){
    return (String)"Buzzer | " + sensors[0].lastTrig + " / " + (trigGap + sensors[1].lastTrig) + " ms | Timer: " + buzzTimer.status() + " - " + (buzzTimer.isLoad() ? "LOAD" : (buzzTimer.isHigh() ? "HIGH" : "LOW"));
  }

};

ActiveBuzzer buzzer(9);
#endif

int totalTick;
constexpr int tickSize = 10;
int ticks[tickSize];
int tickIdx;

void setup() {  

  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  print("##Start##");

#ifdef SENSOR
  print("Sensor Enabled");
#endif

#ifdef BUZZER
  print("Buzzer Enabled");
#endif

  now = 1;
  calibrated = false;

  totalTick = 0, tickIdx = 0;
  memset(ticks, 0, sizeof(ticks));

  calTimer.start();
  calTimer.setRemainingTime(2000);
}

void loop() {
  // set clock
  now = millis();
  now += (now == 0);

  // print("##------------------------------##");
  // printStr("RAM: ", freeRam());

#ifdef SENSOR
  for(auto& sensor: sensors) sensor.update();
  sensors[0].updateLastTrig = calibrated && sensors[1].distance >= sensors[0].distance;
//  printStr("Sensors: ", sensors[0].status() + " || " + sensors[1].status());
#endif

#ifdef BUZZER
  buzzer.update();
  print(buzzer.status());
#endif

  if(calTimer.stop()){
    if(calTimer.isHigh()) print("##CALIBRATE##");
    digitalWrite(calPin, calTimer.isLow() ? LOW : HIGH); 
    calTimer.start();
    calibrated = true;
  }
  else{
//     printStr("CAL: ", calTimer.status());
  }

  totalTick -= ticks[tickIdx];
  ticks[tickIdx] = millis() - now;
  totalTick += ticks[tickIdx];
  // printStr("Tick Time: ", (totalTick / tickSize) + " ms | raw: " + ticks[tickIdx]);
  tickIdx = (tickIdx + 1) % tickSize;
}
