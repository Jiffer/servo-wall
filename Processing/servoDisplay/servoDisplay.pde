import java.util.ArrayList;
import java.util.Collections;
import oscP5.*;
import netP5.*;


OscP5 oscP5;
NetAddress myRemoteLocation;

int patLen = 8000;
int pulse = 130; // in milliseconds
int myFps = 300; //in frames per second

VoiceList vListA = new VoiceList();
VoiceList vListB = new VoiceList();
ArrayList <VoiceDynPair> vdpA = new ArrayList <VoiceDynPair>();
ArrayList <VoiceDynPair> vdpB = new ArrayList <VoiceDynPair>();
ArrayList <VoiceDynPair> templatePatA = new ArrayList <VoiceDynPair>();
ArrayList <VoiceDynPair> templatePatB = new ArrayList <VoiceDynPair>();

ArrayList <Integer> pList = new ArrayList<Integer>();
ArrayList <Integer> dList =  new ArrayList<Integer>();
ArrayList <Integer> lList =  new ArrayList<Integer>();

int rows = 3;
int columns = 5;
int numAgents = rows * columns;
Actuator[] actuators = new Actuator [numAgents];

Clocker myClock; 



void setup(){
  size(columns*100, rows*100);
  initObjects();
  oscP5 = new OscP5(this, 7400);
  myRemoteLocation = new NetAddress("127.0.0.1", 7400);
  frameRate(myFps);
}

void draw(){
  background(127);
  myClock.countFrame();
  if (myClock.pastConstThresh()){ // have 
    for (int i = 0; i < numAgents; i++){
      actuators[i].bang();
    }
  }
  for (int i = 0; i < numAgents; i++){
    actuators[i].update();
    actuators[i].display();
    actuators[i].sendMaxMessage();
  }
}
  
  
void initObjects(){
  myClock = new Clocker();
  templatePatA.add(new VoiceDynPair(new ArrayList<Integer>(Arrays.asList(1, 2, 3)), 1));
  templatePatA.add(new VoiceDynPair(new ArrayList<Integer>(Arrays.asList(0)), 0));
  templatePatA.add(new VoiceDynPair(new ArrayList<Integer>(Arrays.asList(7, 8, 9)), 1));
  templatePatA.add(new VoiceDynPair(new ArrayList<Integer>(Arrays.asList(0)), 0));
  templatePatA.add(new VoiceDynPair(new ArrayList<Integer>(Arrays.asList(13)), 1));
  templatePatB.add(new VoiceDynPair(new ArrayList<Integer>(Arrays.asList(0)), 0));
  templatePatB.add(new VoiceDynPair(new ArrayList<Integer>(Arrays.asList(5, 6)), 1));
  templatePatB.add(new VoiceDynPair(new ArrayList<Integer>(Arrays.asList(0)), 0));
  templatePatB.add(new VoiceDynPair(new ArrayList<Integer>(Arrays.asList(11, 12)), 1));
  templatePatB.add(new VoiceDynPair(new ArrayList<Integer>(Arrays.asList(0)), 0)); 
  
  int shufIndex = 0;
  int shufCount = 0;
  int shufTarget = 3;
  int blank = 0;
  VoiceDynPair v;
  for (int i = 0; i < patLen; i++){
    v = new VoiceDynPair(templatePatA.get(shufIndex).voices, templatePatA.get(shufIndex).dynamic);

    vdpA.add(v);
    if (shufIndex == 0){
      shufCount++;
      if (shufCount == shufTarget){
        Collections.shuffle(templatePatA);
        shufCount = 0;
      }
    }
    shufIndex = (shufIndex + 1) % templatePatA.size();
  }
  
  shufIndex = 0;
  shufCount = 0;
  for (int i = 0; i < patLen; i++){
    v = new VoiceDynPair(templatePatB.get(shufIndex).voices, templatePatB.get(shufIndex).dynamic);
    vdpB.add(v);
    if (shufIndex == 0){
      shufCount++;
      if (shufCount == shufTarget){
        Collections.shuffle(templatePatB);
        shufCount = 0;
      }
    }
    shufIndex = (shufIndex + 1) % templatePatB.size();
  }
 
  
   for (int i = 0; i < columns; i++){
    for (int j = 0; j < rows; j++){
     actuators[(i*rows) + j] = new Actuator((i*rows) + j + 1, i , j);
    }
  }
   myClock.setThreshold(pulse); //send number in milliseconds
  
}

 


class VoiceList{
  int index = 0;
  ArrayList<ArrayList<Integer>> vList = new ArrayList<ArrayList<Integer>>();
  
  VoiceList(){
  }
  
  void addItem(ArrayList <Integer> i){
    vList.add(i);
  }
  
  ArrayList <Integer> nextItem(){
    ArrayList <Integer> rList = new ArrayList<Integer>();
    rList = vList.get(index);
    index = (index + 1) % vList.size();
    return rList;
  }
  
  
  
}
  
 
class VoiceDynPair{
  int dynamic;
  ArrayList <Integer> voices = new ArrayList <Integer>();
  
  VoiceDynPair(){
  }

  VoiceDynPair(ArrayList <Integer> _voices, int d){
    dynamic = d;
    for (Integer v: _voices){
      voices.add(v);
    }
  }
}
  
class Actuator {
  int listIndex = 0;
  int eventCount = 0;
  float slewTime = 100; // in milliseconds
  boolean to = true; //to and fro
  int angMin = 45;
  int angMax = 135;
  float curAng = 90;
  int angDelta = 0;
  int eventIndex = 0;
  int nodeNum; // this starts at 1 to gibe with Max poly~
  int x;
  int y;
  OscMessage myMessage;
  ArrayList <Float> events = new ArrayList<Float>();
  int index = 0;
  int lenIndex = 0;
  int shufCur = 0;
  int shufMax = 5;
  int patMax = 4;  
  
  Actuator(int n, int _x, int _y){
      nodeNum = n;
      x = _x;
      y = _y;
    }
    
  void bang(){
    //this relies on glabally defined vdpA and vdpB
    VoiceDynPair tvpA = new VoiceDynPair();
    VoiceDynPair tvpB = new VoiceDynPair();
    tvpA = vdpA.get(listIndex); // these are the objects to pass to the actuators
    tvpB = vdpB.get(listIndex);
    ArrayList <Integer> talA = new ArrayList<Integer>();
    talA = tvpA.voices;
    ArrayList <Integer> talB = new ArrayList<Integer>();
    talB = tvpB.voices;
    for (Integer i: talA){
      if (i == nodeNum){
        newEvent(tvpA.dynamic);
//        println(nodeNum + " " + tvpA.dynamic);
   
      }
    }
    
    for (Integer i: talB){
      if (i == nodeNum){
        newEvent(tvpB.dynamic);
      }
    }
    listIndex = (listIndex + 1) % patLen;
   
      
  }
    

  void newEvent(int dyn){
     if (dyn > 0){
     if(to){
        to = false;
        switch (dyn) { // these deltas are the terminating point of the motion, which we then break up into smaller pieces in update
          case(1):
            angDelta = 10;
            break;
          case(2):
            angDelta = 20;
            break;
          case(3):
            angDelta = 30;
            break; 
        }
      }
     else { //if we are currently on fro
        to = true;
        switch (dyn) {
          case(1):
            angDelta = -10;
            break;
          case(2):
            angDelta = -20;
            break;
          case(3):
            angDelta = -30;
            break; 
        }
     }
     
     if ( (curAng + angDelta) > angMax || (curAng + angDelta) < angMin){
       angDelta *= -1;
       
     }
      
     float rScale = random(1.0, 1.0); // look at this!
     float res = angDelta * rScale;
     angDelta = round(res);
     makeEvents();
     }
   
  }

// calculate intermediate points to go to each frame  
  void makeEvents(){
    events = new ArrayList<Float>();
    eventIndex = 0;
    float fFrames = (slewTime / (1000. / myFps)); //slew times as num of frames, still float
    float frameDelta = angDelta / fFrames; // what to add each time to curAdd
//    println(angDelta + " " + frameDelta + " " + fFrames);
    int frames = int(fFrames);
    for (int i = 0; i < frames; i++){
      events.add(frameDelta);
//      println(frameDelta);
    }
}
  
  
  void update(){
//    println (eventIndex + " " + events.size());
    if ((eventIndex < (events.size() - 1)) && (events.size() > 0)){
      curAng += events.get(eventIndex); 
      eventIndex++;     
    }
//    println(curAng);
  }
  
  void sendMaxMessage(){
      myMessage = new OscMessage(str(nodeNum));
      myMessage.add(curAng); // add an int to the osc message
      oscP5.send(myMessage, myRemoteLocation);
  }
  
  void display(){
    int xMult = round(width / columns);
    int yMult = round(height / rows);
    int xOff = round(xMult * 0.5);
    int yOff = round(yMult * 0.5);
    pushMatrix();
    translate(x * xMult + xOff, y * yMult + yOff);
    ellipse(0, 0, 5, 5);
//    println("ang is " + ang);
    rotate(radians(curAng-90));
    line(0, 0, 0, 50);
    popMatrix();
  }
    
  
}

class Clocker {
  float thresh = 0;
  int elapsedFrames = 0;
  
  Clocker(){
  }
  
  float getThreshold(){
    return thresh;
  }
  
 
  
  void setThreshold(float t){
    thresh = milliToFrame(t);
  }
    
   void countFrame(){
     elapsedFrames++;
   }
  
  boolean pastConstThresh(){
    if (elapsedFrames >= thresh){
      elapsedFrames = 0;
      return true;
    }
    else {
      return false;
    }
  }
  
  
  boolean pastThresh(){
    if (elapsedFrames >= thresh){
      return true;
    }
    else {
      return false;
    }
  }
  
  int milliToFrame(float mills){
    float frameDur = 1000. / myFps; // each frame is this many milliseconds
    return round(mills / frameDur);
}
  
}



 
