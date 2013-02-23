import toxi.math.noise;
import java.util.ArrayList;
import java.util.Collections;
import oscP5.*;
import netP5.*;

OscP5 oscP5;
NetAddress myRemoteLocation;

 OscMessage myMessage;


//extent of neighbor average is modulated
int xDim = 1200;
int yDim = 551; //545

Cell[][] _cellArray;
int _numX = 11; //11
int _numY = 5;  //5
int cellX =  xDim / _numX;
int cellY =  yDim / _numY;
int cellOffX =  cellX / 2;
int cellOffY =  cellY / 2;
float noisePtr = 0.0;
float noiseInc = 0.002;
float noiseTimeInc = 0.02;
float sinPnt = 0;
float sinInc = TWO_PI / (3 * 30);

int myFps = 30;
float period = 3 * myFps;
float _LocInc = TWO_PI / period;
float periodMin = 2.0;
float periodMax = 20.0;

float gAGain = 5.0;
float nPtr = 0;
float nInc = 0.01;

float gPeriod = 15;
float gPhase = 0;
float phaseInc = 0;



void setup(){
  size(xDim, yDim);
  oscP5 = new OscP5(this, 7400);
  myRemoteLocation = new NetAddress("127.0.0.1", 7400);
  frameRate(myFps);
  smooth();
  phaseInc = TWO_PI / (gPeriod * myFps);
  restart();
}

void restart(){
  _cellArray = new Cell[_numX][_numY];
  for (int x = 0; x<_numX; x++){
      for (int y = 0; y<_numY; y++){
        Cell newCell = new Cell(x, y);
        //_cellArray[x][y] = newCell;
        _cellArray[x][y] = new Cell(x, y);
        _cellArray[x][y].setRotation(random(0.0, 1.0));
        _cellArray[x][y].setInc(TWO_PI / (random(periodMin, periodMax) * myFps));
//        _cellArray[x][y].setInc(TWO_PI / ( map(x * y, 0, _numX*_numY, periodMin, periodMax) * myFps));

        _cellArray[x][y].setPhase(random(0.0, TWO_PI));
        
      }
  }

for (int x = 0; x < _numX; x++){
  for (int y = 0; y < _numY; y++){
    
    int above = y-1;
    int below = y+1;
    int left = x-1;
    int right = x+1;
    
    if (above < 0) {above = _numY-1;}
    if (below == _numY) { below = 0;}
    if (left < 0) {left = _numX-1;}
    if (right == _numX) { right = 0;}
    
    _cellArray[x][y].addNeighbor(_cellArray[left][above]);
    _cellArray[x][y].addNeighbor(_cellArray[left][y]);
    _cellArray[x][y].addNeighbor(_cellArray[left][below]);
//    _cellArray[x][y].addNeighbor(_cellArray[x][below]);
    _cellArray[x][y].addNeighbor(_cellArray[right][below]);
    _cellArray[x][y].addNeighbor(_cellArray[right][y]);
    _cellArray[x][y].addNeighbor(_cellArray[right][above]);
//    _cellArray[x][y].addNeighbor(_cellArray[x][above]);
    
  }
}
}


void draw(){
  background(173, 184, 234);
  noisePtr += noiseTimeInc;
  drawConduit();
  gPhase += (phaseInc % TWO_PI);
  gAGain = map(sin(gPhase), -1, 1.0, 20, 20.0);


  for (int x = 0; x < _numX; x++){
    for (int y = 0; y < _numY; y++){
      _cellArray[x][y].calcNextState();
    }
  }
  
  translate((int) cellX / 2.0, (int) cellY / 2.0 );
  
  for (int x = 0; x < _numX; x++){
    for (int y = 0; y < _numY; y++){
      _cellArray[x][y].drawMe();
    }
  }
}

void mousePressed(){
  restart();
}


void drawConduit(){
  stroke(0, 0, 0, 120);
  int offset = cellOffY;
  for (int i = 0; i < _numY; i++){
    int yLoc = (height / _numY) * i;
    yLoc += offset;
    pushMatrix();
    

    translate(0, yLoc);
    line(0, 0, width, 0);
    popMatrix();
  }
  
  for (int i = 0; i < _numX; i++){
    for (int j = 0; j < _numY; j++){
      int xLoc = (width / _numX ) * i;
      int yLoc = (height / _numY) * j;
      pushMatrix();
      translate(xLoc, yLoc + ( cellY/2));
      noFill();
      arc(cellX/2, 0, cellX, cellY/1.2, 0, PI);
      popMatrix();
     noFill();
    }
  }
  
  
  for (int i = 0; i < _numX; i++){
    int xLoc = (width / _numX) * i;
//    xLoc += (cellOffX);
    pushMatrix();
    translate(xLoc, 0);
    fill(0);
    rect(0, 0, 2, height);
//    line(0, 0, 0, height);
//    line(5, 0, 5, height);
    noFill();
    popMatrix();
  }
  
  pushMatrix();
  translate(width-2, 0);
  fill(0);
  rect(0, 0, 2, height);
  noFill();
  popMatrix();
  
  
  stroke(0);
}
    


class Cell{
  float x, y;
  float pX, pY;
  boolean state;
  boolean nextState;
  Cell[] neighbors;
  float rotation = 0;
  float finRot = 0;
  float locInc;
  float phasePosition = 0.0;
  float nTimePtr;
  float nodeNum;

  
  Cell(float ex, float why){
    pX = ex;
    pY = why;
    x = ex * cellX;
    y = why * cellY;
    nodeNum = (pX * _numX) + pY;

 
    neighbors = new Cell[0];
    
  }
  
  
  void addNeighbor(Cell cell){
    neighbors = (Cell[])append(neighbors, cell);
  }
  
  void setRotation(float _rot){
    rotation = _rot;
  }

 void setInc(float _inc){
    locInc = _inc;
    //println(locInc);  
}
  
  void setPhase(float _phase){
    phasePosition = _phase;
    //println(phasePosition);
  }


void calcNextState(){
  finRot = map(noise(x*noiseInc, y*noiseInc, noisePtr), 0.1, 0.9, -90, 90);

}
 
 
 void drawMe(){
   state = nextState;
   fill(255);
  ellipse(x, y, 5, 5);
   pushMatrix();
   translate(x, y);
   //rotate(rotation);
   //rotate(PI * 1.75);
   float cRot = constrain(finRot, -90, 90);
   rotate(radians(cRot));
   line(0, -(cellY/2), 0, cellY/2);
//   line(0, 0, 0, 70);
   int nodeNum = round((pX * _numY) + pY + 1) ;
   float curAng = map(cRot, -90, 90, 40, 130);
   myMessage = new OscMessage(str(nodeNum));
   myMessage.add(curAng); // add an int to the osc message
   oscP5.send(myMessage, myRemoteLocation);
   popMatrix();
 }
 
}
