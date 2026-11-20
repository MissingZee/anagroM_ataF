#include <Arduino.h>
#include <rocciboard.h>
#include <Servo.h> //Servo motor
#include <U8g2lib.h> //OLED display
#include <Wire.h> //I2C
RocciBoard rb; //Control-board for about everything
Servo medikits; //rbe0
U8G2_SSD1306_128X64_NONAME_2_HW_I2C oled(U8G2_R0); //rbe1
RBLaser laser_front(0, TYPE_SHORTRANGE); //rb00
RBCompass compass(0); //rb01
RBLaser laser_front_bottom(1, TYPE_LONGRANGE); //rb10
#define CAMBOTTOM 1 //rb11
RBLaser laser_right(2, TYPE_SHORTRANGE); //rb20
#define CAMRIGHT 2 //rb21
RBLaser laser_left(3, TYPE_SHORTRANGE); //rb30
#define CAMLEFT 3 //rb31
RBLaser laser_back(4, TYPE_LONGRANGE); //rb40
RBLaser laser_right_back(5, TYPE_SHORTRANGE); //rb50
#define CAMRIGHTBACK 5 //rb51
RBLaser laser_left_back(6, TYPE_SHORTRANGE); //rb60
#define CAMLEFTBACK 6 //rb61
#define MOTORFR 1 //default: backwards
#define MOTORFL 2 //default: forward
#define MOTORBL 0 //default: forward
#define MOTORBR 3 //default: backwards
#define LED 1 //Port of the red LED
#define BUTTONMIDDLE 49
#define BUTTONRIGHT 51
#define BUTTONLEFT 47
#define BUTTONUP 45
#define BUTTONDOWN 43
#define DISTANCE 25 //Distance when the robot recognizes a bordering open tile
#define RUN 255 //Speed for normal driving
#define TURN 200 //Speed for turning
#define TILESIZE 30 //length of a single tile
#define NORTH 0 //|
#define EAST 1  //|Virtual orientation of the robot
#define SOUTH 2 //|NORTH is the orientation at the beginning of a run
#define WEST 3  //|
#define FRONT 0
#define RIGHT 1
#define BACK 2
#define LEFT 3
#define FRONTRIGHT 4
#define FRONTLEFT 5
#define RANDOM 6
#define DANGEROUSZONE 1 //defines whether or not the robot enters the dangerous zone(0=>avoid)
#define CAMIGNORE 0 //if 1, any information provided by the camera will be ignored
#define TESTMODE 0 //If >0, the robot only executes what is in the beginninng of void loop
#define FANCY 1
#define ACCESSVARIABLE demo
#define INDICATEFINISH 1 //If 0, the robot does not react to returning to its starting tile
#define DEBUGLED 1 // If 1, some processes use the red, Arduino or RoCCI-Board LED for indicating stuff
#define GETVARIABLENAME(Variable) (#Variable)
int tempVgd[5]; //order of surrounding tiles
int vgd = 0; //virtual geographic direction in North, South, West and East
int xc = 25; //x-coordinate on the map
int yc = 25; //y-coordinate on the map
int tileOrder = 1; //Order of the tiles, starting tile=1, adds one per visited tile
int reset90Counter = 0; //Countdown for the next reset90
int turnCal = 0; //Offset of the compass calclulated by aligning to the wall
int displayMode = 5; //decides, what the display shows
int didDriveBefore=0;
int demo = 0;
unsigned int lastRefresh = 0;
unsigned int lastPress = 0;
bool isPaused = true;

//contains information about a singular field in the map
class tile{
  public:
  int order = 0;
  char floorColour = 'w';
  char wallLetter[4];
  bool walls[4];

  int getOrder(){
    return order;
  }
  char getfloorColour(){
    return floorColour;
  }
  char getWallLetter(int direction){
           if(direction==0){return wallLetter[(vgd+3)%4];}       //left
    else if(direction==1){return wallLetter[vgd];}        //front
    else if(direction==2){return wallLetter[(vgd+1)%4];}  //right
    else if(direction==3){return wallLetter[(vgd+2)%4];}  //back
    else                 {return 'X';}
  }
  bool isWall(bool absolute, int direction){
    if(absolute==false){
           if(direction==0){return walls[(vgd+3)%4];}  //left
      else if(direction==1){return walls[vgd];}        //front
      else if(direction==2){return walls[(vgd+1)%4];}  //right
      else                 {return walls[(vgd+2)%4];}  //back
    }
    else{
      return walls[direction];
    }
  }
  void setOrder(int newOrder){
    order = newOrder;
  }
  void setFloorColour(char newFloorColour){
    floorColour = newFloorColour;
  }
  /**
  * @param newDirectionLetter side of the robot the letter is on (0->left;1->front;2->right)
  */
  void setWallLetter(char newWallLetter, int newDirectionLetter){
    int directionLetter = 0;
         if(newDirectionLetter==0){directionLetter=(vgd+3)%4;}
    else if(newDirectionLetter==1){directionLetter=vgd;}
    else if(newDirectionLetter==2){directionLetter=(vgd+1)%4;}
    wallLetter[directionLetter] = newWallLetter;
  }
  void setWall(bool absolute, int direction, bool makeWall = true){
    if(absolute==false){
           if(direction==0){walls[(vgd+3)%4] = makeWall;}  //left
      else if(direction==1){walls[vgd] = makeWall;}        //front
      else if(direction==2){walls[(vgd+1)%4] = makeWall;}  //right
      else                 {walls[(vgd+2)%4] = makeWall;}  //back
    }
    else{
      walls[direction] = makeWall;
    }
  }
};
tile maps[50][50]; //the map the robot creates of the maze

//first order spells:

//returns the letter currently recognized by the camera on port camPort
char getLetter(int camPort){
  //return 'W';
  rb.openSensorPort(camPort);
  Wire.requestFrom(0x55, 1);
  char c = Wire.read();
  rb.closeSensorPort(camPort);
  return c;
}
//swaps two digits of tempVgd(used only in lastFreeTile())
void swap(int one, int two){
  tempVgd[4]=tempVgd[one];
  tempVgd[one]=tempVgd[two];
  tempVgd[two]=tempVgd[4];
}
//calculates the visible tiles from the front ToF-sensor value
int visibleTiles(){
  int calc = (laser_front.getDistanceCentimeters()+15) / (TILESIZE+2);
  //if(calc > 2){
    //calc = (laser_front_bottom.getDistanceCentimeters()+15) / (TILESIZE+2);
  //}
  return calc;
}
//calculates the visible tiles from the back ToF-sensor value
int backVisibleTiles(){
  return (laser_back.getDistanceCentimeters()+10) / TILESIZE+2;
}
//says, if the value is about the tolerance around the goal
bool tolerance(int value, int goal, double tolerance){ 
  int calc = 0;
  if(value-goal<=0){
    calc=(value-goal)*(-1);
  }
  if(goal-value<0){
    calc=(goal-value)*(-1);
  }
  if(calc<=tolerance){
    return true;
  }
  else{
    return false;
  }
}
//prints out the generated map to the serial monitor
void giveOutMap(){
  for(int x=49;x>=0;x--){
    for(int y=0;y<=49;y++){
      if(maps[x][y].isWall(true, NORTH)){Serial.print(" --- ");}
      else{Serial.print(" ... ");}
    }
    Serial.println("");
    for(int y=0;y<=49;y++){
      if(maps[x][y].isWall(true, EAST)){Serial.print("| ");}
      else{Serial.print(": ");}
      Serial.print(maps[x][y].getOrder());
      if(maps[x][y].isWall(true, WEST)){Serial.print(" |");}
      else{Serial.print(" :");}
    }
    Serial.println("");
    for(int y=0;y<=49;y++){
      if(maps[x][y].isWall(true, SOUTH)){Serial.print(" ~~~ ");}
      else{Serial.print(" °°° ");}
    }
    Serial.println("");
  }
}
int comp(){
  int calculator=0;
  calculator=compass.getHeading()+45;
  if(calculator>360){
    calculator=calculator-360;
  }
  return calculator;
}
bool lasersLeftFree(double distance = DISTANCE){
  if(laser_left.getDistanceCentimeters()>=distance && laser_left_back.getDistanceCentimeters()>=distance){
    return 1;
  }
  else{
    return 0;
  }
}
bool lasersRightFree(double distance = DISTANCE){
  if(laser_right.getDistanceCentimeters()>=distance && laser_right_back.getDistanceCentimeters()>=distance){
    return 1;
  }
  else{
    return 0;
  }
}
float lasersFront(){
  float returnValue = laser_front.getDistanceCentimeters();
  if(laser_front.getDistanceCentimeters()<=64){
    returnValue=laser_front.getDistanceCentimeters();
  }
  else{
    returnValue=laser_front_bottom.getDistanceCentimeters();
  }
  return returnValue;
}
//checks, if the field in direction dir2 has already been visited(dir2=1->front,2->right,3->left)
bool known(int dir2){
  bool returnValue=0;
  int dir=0;
  if(dir2==2){
    dir=1;
  }
  else if(dir2==3){
    dir=-1;
  }
  if(vgd==(NORTH-dir)%4){ //North+West+East
    if(maps[xc+1][yc].getOrder()>=1){
      returnValue=1;
    }
    else{
      returnValue=0;
    }
  }
  else if(vgd==(SOUTH-dir)%4){ //South+East+West
    if(maps[xc-1][yc].getOrder()>=1){
      returnValue=1;
    }
    else{
      returnValue=0;
    }
  }
  else if(vgd==(EAST-dir)%4){ //East+North+South
    if(maps[xc][yc+1].getOrder()>=1){
      returnValue=1;
    }
    else{
      returnValue=0;
    }
  }
  else if(vgd==(WEST-dir)%4){ //West+South+North
    if(maps[xc][yc-1].getOrder()>=1){
      returnValue=1;
    }
    else{
      returnValue=0;
    }
  }
  return returnValue;
}
//returns if the tile next to the robot in direction dir is free(dir=0->North,1->East,2->South,3->West)
bool isFree(bool absolute, int dir){
  if(absolute==true){
    if(dir==vgd){ //N, E, S, W
      if(lasersFront()>=DISTANCE){return 1;}
      else{return 0;}
    }
    else if(dir==(vgd+1)%4){ //E, S, W, N
      if(lasersRightFree()){return 1;}
      else{return 0;}
    }
    else if(dir==(vgd+3)%4){ //W, N, E, S
      if(lasersLeftFree()){return 1;}
      else{return 0;}
    }
    else if(dir==(vgd+2)%4){ // S, W, N, E
      if(laser_back.getDistanceCentimeters()>=DISTANCE){return 1;}
      else{return 0;}
    }
    else{
      return 0; //shouldn't happen
    }
  }
  else{
    if(dir==FRONT){
      if(lasersFront()>=DISTANCE){return 1;}
      else{return 0;}
    }
    else if(dir==RIGHT){
      if(lasersRightFree()){return 1;}
      else{return 0;}
    }
    else if(dir==LEFT){
      if(lasersLeftFree()){return 1;}
      else{return 0;}
    }
    else if(dir==BACK){
      if(laser_back.getDistanceCentimeters()>=DISTANCE){return 1;}
      else{return 0;}
    }
    else{
      return 0; //shouldn't happen
    }
  }
}
//returns if the tile next to the robot in direction dir is black(dir=0->North,1->East,2->South,3->West)
bool isBlack(bool absolute, int dir){
  if(absolute==true){
      	 if(dir==NORTH && maps[xc+1][yc].getfloorColour()=='s'){return 1;}
    else if(dir==SOUTH && maps[xc-1][yc].getfloorColour()=='s'){return 1;}
    else if(dir==EAST  && maps[xc][yc+1].getfloorColour()=='s'){return 1;}
    else if(dir==WEST  && maps[xc][yc-1].getfloorColour()=='s'){return 1;}
    else                                                       {return 0;}
  }
  else{
          if(dir==(FRONT-vgd)%4 && maps[xc+1][yc].getfloorColour()=='s'){return 1;}
    else if(dir==(BACK-vgd)%4   && maps[xc-1][yc].getfloorColour()=='s'){return 1;}
    else if(dir==(RIGHT-vgd)%4  && maps[xc][yc+1].getfloorColour()=='s'){return 1;}
    else if(dir==(LEFT-vgd)%4   && maps[xc][yc-1].getfloorColour()=='s'){return 1;}
    else                                                                {return 0;}
  }
}//the robot drives forward (for speed>0)
void run(int speed) {
  rb.motor[MOTORFL].rotate(speed);
  rb.motor[MOTORFR].rotate(-speed);
  rb.motor[MOTORBL].rotate(speed);
  rb.motor[MOTORBR].rotate(-speed);
}
//the robot turns (speed>0 -> right)
void turn(int speed) {
  rb.motor[MOTORFL].rotate(speed);
  rb.motor[MOTORFR].rotate(speed);
  rb.motor[MOTORBL].rotate(speed);
  rb.motor[MOTORBR].rotate(speed);
}
//the robot stops its movement
void stop() {
  rb.motor[MOTORFL].stop(1);
  rb.motor[MOTORFR].stop(1);
  rb.motor[MOTORBL].stop(1);
  rb.motor[MOTORBR].stop(1);
}
//the robot writes its current position to the map
void mapsWrite(){
  if(vgd == NORTH){xc++;}
  if(vgd == SOUTH){xc--;}
  if(vgd == EAST){yc++;}
  if(vgd == WEST){yc--;}
  if(maps[xc][yc].getOrder()==0){
    tileOrder++;
    maps[xc][yc].setOrder(tileOrder);
  }
  if(vgd >= 4){vgd = 0;}
  if(vgd < 0){vgd = 3;}
}
//the robot turns the LED on and off for the duration of the given seconds
void blink(int interval, int blinkTimes){
  for(int i=0;i<=blinkTimes;i++){
    digitalWrite(LED, HIGH);
    digitalWrite(RB_DEBUG_LED, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(interval);
    if(isPaused){break;}
    digitalWrite(LED, LOW);
    digitalWrite(RB_DEBUG_LED, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    delay(interval);
    if(isPaused){break;}
  }
}

//second order spells:

//returns the recognized colour beneath the robot
char detectColour(){ //replace with bottom cam
  return getLetter(CAMBOTTOM);
}
//returns the direction of the tile with the smallest number that is free around the robot
int lastFreeTile(){
  int north=maps[xc+1][yc].getOrder();
  int south=maps[xc-1][yc].getOrder();
  int east=maps[xc][yc+1].getOrder();
  int west=maps[xc][yc-1].getOrder();
  int current=maps[xc][yc].getOrder();
  if(north==(current-1) && isFree(true, NORTH)){tempVgd[0]=north;}       //last Tiles,
  else if(south==(current-1) && isFree(true, SOUTH)){tempVgd[0]=south;}  //if there are any
  else if(east==(current-1) && isFree(true, EAST)){tempVgd[0]=east;}
  else if(west==(current-1) && isFree(true, WEST)){tempVgd[0]=west;}
  else{ //smallest numbered Tile
    tempVgd[0]=north;
    tempVgd[1]=east;
    tempVgd[2]=south;
    tempVgd[3]=west;
    if(tempVgd[0]<tempVgd[1]){swap(0,1);} //sort by size,
    if(tempVgd[2]<tempVgd[3]){swap(2,3);} //greatest first
    if(tempVgd[1]<tempVgd[2]){swap(1,2);}
    if(tempVgd[0]<tempVgd[1]){swap(0,1);}
    if(tempVgd[2]<tempVgd[3]){swap(2,3);}
    if(tempVgd[1]<tempVgd[2]){swap(1,2);}
  }
  for(int i=0;i<4;i++){ //Stelle im Array
    for(int j=0;j<4;j++){
      int k=0;
      if(j==NORTH){k=north;}
      else if(j==EAST){k=east;}
      else if(j==SOUTH){k=south;}
      else if(j==WEST){k=west;}
      if(tempVgd[i]==k && tempVgd[i]!=0 && tempVgd[i]<current){
        if(isFree(true, j) && isBlack(true, j)==0){
          if(j==vgd){return 1;}
          else if(j==(vgd+1)%4){return 2;}
          else if(j==(vgd+3)%4){return 3;}
          else if(j==(vgd+2)%4){return 4;}
        }
      }
    }
  }
  return 5; //When the robot has nowhere to go, for example when the starting tile is in a dead end
}
//checks all sensor values for obvious onnection errors
String sensorsFine(){
  String returnString = "Fine";
  turn(100); delay(300); stop();
  if(laser_left.getDistanceCentimeters()>6000){returnString = "Laser-Left";}
  else if(laser_left_back.getDistanceCentimeters()>6000){returnString = "Laser-Left-Back";}
  else if(laser_right.getDistanceCentimeters()>6000){returnString = "Laser-Right";}
  else if(laser_right_back.getDistanceCentimeters()>6000){returnString = "Laser-Right-Back";}
  else if(laser_front.getDistanceCentimeters()>6000){returnString = "Laser-Front";}
  //else if(laser_front_bottom.getDistanceCentimeters()>6000){returnString = "Laser-Front-Bottom";}
  else if(laser_back.getDistanceCentimeters()>6000){returnString = "Laser-Back";}
  else if(compass.getHeading()==0){returnString = "Compass";}
  turn(-100); delay(300); stop();
  return returnString;
}
//the robot drops one medikit
void deployMedikit(){
  medikits.write(360);
  delay(750);
  medikits.write(105);
  stop();
  delay(750);
  medikits.write(130);
}
void pause(){
  stop();
  if(isPaused==false){
    isPaused=true;
    #if DEBUGLED==1
      digitalWrite(RB_DEBUG_LED, HIGH);
    #endif
  }
  else{
    isPaused=false;
    #if DEBUGLED==1
      digitalWrite(RB_DEBUG_LED, LOW);
    #endif
  }
}
//asks getLetter() [steps] times and returns the letter sent most often
char averageLetter(int camPort, int steps){
  int countX=0;
  int countU=0;
  int countG=0;
  int countS=0;
  int countY=0;
  int countH=0;
  int countR=0;
  int greatest=0;
  for(int i=0; i<steps-1; i++){
    char letter = getLetter(camPort);
         if(letter=='U'){countU++;}
    else if(letter=='S'){countS++;}
    else if(letter=='H'){countH++;}
    else if(letter=='G'){countG++;}
    else if(letter=='Y'){countY++;}
    else if(letter=='R'){countR++;}
    else                {countX++;}
  }
  if(countU>greatest){greatest=countU;}
  if(countS>greatest){greatest=countS;}
  if(countH>greatest){greatest=countH;}
  if(countG>greatest){greatest=countG;}
  if(countY>greatest){greatest=countY;}
  if(countR>greatest){greatest=countR;}
  if(countX>greatest){greatest=countX;}
       if(greatest==countU){return 'U';}
  else if(greatest==countS){return 'S';}
  else if(greatest==countH){return 'H';}
  else if(greatest==countG){return 'G';}
  else if(greatest==countY){return 'Y';}
  else if(greatest==countR){return 'R';}
  else                     {return 'X';}
}
//calculates the tile orders in a fancier way
void fancyPlanning(){
  //setting 1s in the robots seeing range
  if(isFree(true, NORTH) && maps[xc+1][yc].getOrder()==0){
    maps[xc+1][yc].setOrder(1);
  }
  else if(isFree(true, EAST) && maps[xc][yc+1].getOrder()==0){
    maps[xc][yc+1].setOrder(1);
  }
  else if(isFree(true, SOUTH) && maps[xc-1][yc].getOrder()==0){
    maps[xc-1][yc].setOrder(1);
  }
  else if(isFree(true, WEST) && maps[xc][yc-1].getOrder()==0){
    maps[xc][yc-1].setOrder(1);
  }
  //maps[49/2][49/2].setOrder(1);
  //setting all already discovered fields (that aren't 1s) on a placeholder
  for(int y=0;y<=49;y++){
    for(int x=0;x<=49;x++){
      if(maps[x][y].getOrder()>1){
        maps[x][y].setOrder(-1);
      }
    }
  }
  bool planningFinished = false;
  //set all placeholders to their respective new values spreading from the previously set 1s
  for(int order=1;planningFinished==false/*order<=100*/;order++){
    //Serial.println("order: " + String(order));
    for(int y=0;y<=49;y++){
      //Serial.println("y: " + String(y));
      for(int x=0;x<=49;x++){
        //Serial.println("x: " + String(x));
        if(maps[x][y].getOrder()==order){
          if(maps[x+1][y].getOrder()==-1 && maps[x][y].isWall(true, NORTH)==false && x!=49){
            maps[x+1][y].setOrder(order+1);
          }
          if(maps[x-1][y].getOrder()==-1 && maps[x-1][y].isWall(true, SOUTH)==false && x!=0){
            maps[x-1][y].setOrder(order+1);
          }
          if(maps[x][y+1].getOrder()==-1 && maps[x][y+1].isWall(true, EAST)==false && y!=49){
            maps[x][y+1].setOrder(order+1);
          }
          if(maps[x][y-1].getOrder()==-1 && maps[x][y-1].isWall(true, WEST)==false && y!=0){
            maps[x][y-1].setOrder(order+1);
          }
        }
      }
    }
    //check if finished
    bool isPlaceholder = false;
    for(int y=0;y<=49;y++){
      for(int x=0;x<=49;x++){
        if(maps[x][y].getOrder()==-1){
          isPlaceholder=true;
          break;
        }
      }
      if(isPlaceholder==true){break;}
    }
    if(isPlaceholder==false){
      planningFinished=true;
    }
    else{
      planningFinished=false;
    }
  }
}
//basically lastFreeTile in a more sophisticated way and using the map created by fancyPlanning
int fancyDriving(int preferredDirection = LEFT){
  int smallest = 100;
  if(maps[xc+1][yc].getOrder()<smallest && !isBlack(true, NORTH) && isFree(true, NORTH)){
    smallest=maps[xc+1][yc].getOrder();
  }
  if(maps[xc][yc+1].getOrder()<smallest && !isBlack(true, EAST) && isFree(true, EAST)){
    smallest=maps[xc][yc+1].getOrder();
  }
  if(maps[xc-1][yc].getOrder()<smallest && !isBlack(true, SOUTH) && isFree(true, SOUTH)){
    smallest=maps[xc-1][yc].getOrder();
  }
  if(maps[xc][yc-1].getOrder()<smallest && !isBlack(true, WEST) && isFree(true, WEST)){
    smallest=maps[xc][yc-1].getOrder();
  }

	//universalise some relative map variables
	int frontTileX, frontTileY, rightTileX, rightTileY, leftTileX, leftTileY, backTileX, backTileY;
		   if(vgd==NORTH){frontTileX=xc+1; frontTileY=yc;}
	else if(vgd==EAST) {frontTileX=xc;   frontTileY=yc+1;}
	else if(vgd==SOUTH){frontTileX=xc-1; frontTileY=yc;}
	else					     {frontTileX=xc;   frontTileY=yc-1;}
		   if(vgd==NORTH){backTileX=xc-1;  backTileY=yc;}
	else if(vgd==EAST) {backTileX=xc;    backTileY=yc-1;}
	else if(vgd==SOUTH){backTileX=xc+1;  backTileY=yc;}
	else					     {backTileX=xc;    backTileY=yc+1;}
		   if(vgd==NORTH){leftTileX=xc;    leftTileY=yc-1;}
	else if(vgd==EAST) {leftTileX=xc+1;  leftTileY=yc;}
	else if(vgd==SOUTH){leftTileX=xc;    leftTileY=yc+1;}
	else					     {leftTileX=xc-1;  leftTileY=yc;}
		   if(vgd==NORTH){rightTileX=xc;   rightTileY=yc+1;}
	else if(vgd==EAST) {rightTileX=xc-1; rightTileY=yc;}
	else if(vgd==SOUTH){rightTileX=xc;   rightTileY=yc-1;}
	else					     {rightTileX=xc+1; rightTileY=yc;}
	
  if(preferredDirection==RANDOM){
    int coincidence = random(RIGHT,FRONTRIGHT);
    if(coincidence==BACK){coincidence=FRONTLEFT;}
    preferredDirection=coincidence;
	}

	if(preferredDirection==LEFT){
         if(smallest==maps[leftTileX][leftTileY].getOrder())  {return LEFT;}
    else if(smallest==maps[frontTileX][frontTileY].getOrder()){return FRONT;}
    else if(smallest==maps[rightTileX][rightTileY].getOrder()){return RIGHT;}
    else if(smallest==maps[backTileX][backTileY].getOrder())  {return BACK;}
	}
  else if(preferredDirection==RIGHT){
         if(smallest==maps[rightTileX][rightTileY].getOrder()){return RIGHT;}
    else if(smallest==maps[frontTileX][frontTileY].getOrder()){return FRONT;}
    else if(smallest==maps[leftTileX][leftTileY].getOrder())  {return LEFT;}
    else if(smallest==maps[backTileX][backTileY].getOrder())  {return BACK;}
	}
  else if(preferredDirection==FRONTLEFT){
         if(smallest==maps[frontTileX][frontTileY].getOrder()){return FRONT;}
    else if(smallest==maps[leftTileX][leftTileY].getOrder())  {return LEFT;}
    else if(smallest==maps[rightTileX][rightTileY].getOrder()){return RIGHT;}
    else if(smallest==maps[backTileX][backTileY].getOrder())  {return BACK;}
	}
  else if(preferredDirection==FRONTRIGHT){
         if(smallest==maps[frontTileX][frontTileY].getOrder()){return FRONT;}
    else if(smallest==maps[rightTileX][rightTileY].getOrder()){return RIGHT;}
    else if(smallest==maps[leftTileX][leftTileY].getOrder())  {return LEFT;}
    else if(smallest==maps[backTileX][backTileY].getOrder())  {return BACK;}
	}
  else{
    return 10; //shouldn't happen
  }
}

//third order spells:

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(52, OUTPUT);
  pinMode(53, OUTPUT);
  digitalWrite(52, LOW);
  digitalWrite(53, LOW);
  pinMode(BUTTONMIDDLE, INPUT_PULLUP);
  pinMode(BUTTONRIGHT, INPUT_PULLUP);
  pinMode(BUTTONLEFT, INPUT_PULLUP);
  pinMode(BUTTONUP, INPUT_PULLUP);
  pinMode(BUTTONDOWN, INPUT_PULLUP);
  Serial.begin(9600);
  Serial.write("Hello world!");
  medikits.attach(3);
  oled.begin();
  oled.setContrast(150);
  rb.init(false);
  rb.initRBSensor(laser_front);
  rb.initRBSensor(compass);
  rb.initRBSensor(laser_front_bottom);
  rb.initRBSensor(laser_right);
  rb.initRBSensor(laser_left);
  rb.initRBSensor(laser_back);
  rb.initRBSensor(laser_left_back);
  rb.initRBSensor(laser_right_back);
  randomSeed(analogRead(0));
  maps[25][25].setOrder(tileOrder);
  medikits.write(130);
  oled.clearBuffer();
  oled.setFont(u8g2_font_ncenB10_tr);
  oled.firstPage();
  do {
    oled.setCursor(0, 20);
    oled.print(("Hello World!"));
  } while ( oled.nextPage() );
  attachInterrupt(BUTTONMIDDLE, pause, FALLING);
  fancyPlanning();
}
//prints different values to the OLED display, also whilst driving
void oledValues(int modeAuto, String status){
  int DisplayMode=1;
  if(displayMode==0){
    DisplayMode=modeAuto;
  }
  else{
    DisplayMode=displayMode;
  }
  if(DisplayMode == 1){ //laser information + coordinates
    oled.nextPage();
    oled.clear();
    do{
      oled.setCursor(0,15);
      oled.print(status);
      oled.setCursor(115,15);
      oled.print(vgd);
      oled.setCursor(0,30);
      oled.print("F: " + String(laser_front.getDistanceCentimeters()) + " FB: " /*+ String(laser_front_bottom.getDistanceCentimeters())*/);
      oled.setCursor(0,45);
      oled.print("Left: " + String(laser_left.getDistanceCentimeters()));
      oled.setCursor(0,60);
      oled.print("Right: " + String(laser_right.getDistanceCentimeters()));
      oled.setCursor(95,60);
      oled.print("T:" + String(visibleTiles()));
      oled.setCursor(80,45);
      oled.print(String(xc) + "x" + String(yc));
    } while(oled.nextPage());
  }
  else if(DisplayMode == 2){ //colour information
    oled.nextPage();
    oled.clear();
    do{
      oled.setCursor(0,15);
      oled.print(status);
      oled.setCursor(0,30);
      oled.print("Colour: " + String(detectColour()));
    } while(oled.nextPage());
  }
  else if(DisplayMode==3){ //compass information
    oled.nextPage();
    oled.clear();
    do{
      oled.setCursor(0,15);
      oled.print(status);
      oled.setCursor(0,30);
      int accurateCompass = compass.getHeading()-turnCal;
      if(accurateCompass<0){accurateCompass+=360;}
      else if(accurateCompass>360){accurateCompass-=360;}
      oled.print("Heading: " + String(compass.getHeading()) + " / " + String(accurateCompass));
      oled.setCursor(0,45);
      oled.print("Roll: " + String(compass.getRoll()));
      oled.setCursor(0,60);
      oled.print("Pitch: " + String(compass.getPitch()));
    } while(oled.nextPage());
  }
  else if(DisplayMode==4){ //realign laser information
    oled.nextPage();
    oled.clear();
    do{
      oled.setCursor(0,15);
      oled.print(status);
      oled.setCursor(0,30);
      oled.print("Right: " + String(laser_right.getDistanceCentimeters()-1));
      oled.setCursor(0,45);
      oled.print("R.B: " + String(laser_right_back.getDistanceCentimeters()));
      oled.setCursor(0,60);
      oled.print("Left: " + String(laser_left.getDistanceCentimeters()-2));
      oled.setCursor(66,45);
      oled.print("L.B: " + String(laser_left_back.getDistanceCentimeters()));
    } while(oled.nextPage());
  }
  else if(DisplayMode==5){ //RoCCI Logo+Team Info
    oled.nextPage();
    oled.clear();
    do{
      oled.setCursor(0,15);
      oled.print(status);
      oled.setCursor(0,30);
      oled.print("anagroM_ataF");
      oled.setCursor(0,50);
      oled.print("RoCCI");
    } while(oled.nextPage());
  }
  else if(DisplayMode==6){ //known()+TileCounter+lastFreeTile()
    oled.nextPage();
    oled.clear();
    do{
      oled.setCursor(0,15);
      oled.print(status);
      oled.setCursor(0,30);
      oled.print("Front: " + String(known(1)));
      oled.setCursor(0,45);
      oled.print("Right: " + String(known(2)));
      oled.setCursor(0,60);
      oled.print("Left: " + String(known(3)));
      oled.setCursor(80,30);
      oled.print("TC: " + String(maps[xc][yc].getOrder()));
      oled.setCursor(80,45);
      oled.print("LFT: " + String(lastFreeTile()));
    } while(oled.nextPage());
  }
  else if(DisplayMode==7){ //tileOrder
    oled.nextPage();
    oled.clear();
    do{
      oled.setCursor(0,15);
      oled.print(status);
      oled.setCursor(0,30);
      oled.print("North: " + String(maps[xc+1][yc].getOrder()));
      oled.setCursor(0,45);
      oled.print("East: " + String(maps[xc][yc+1].getOrder()));
      oled.setCursor(0,60);
      oled.print("West: " + String(maps[xc][yc-1].getOrder()));
      oled.setCursor(80,30);
      oled.print("S: " + String(maps[xc-1][yc].getOrder()));
      oled.setCursor(80,45);
      oled.print("C: " + String(maps[xc][yc].getOrder()));
    } while(oled.nextPage());
  }
  else if(DisplayMode==8){ //is paused
    oled.nextPage();
    oled.clear();
    do{
      oled.setCursor(0,15);
      oled.print(status);
      oled.setCursor(0,30);
      oled.print(String(isPaused));
    } while(oled.nextPage());
  }
  else if(DisplayMode==9){ //button presses
  	oled.nextPage();
    oled.clear();
    do{
      oled.setCursor(0,15);
      oled.print(status);
      oled.setCursor(0,30);
      oled.print("L: " + String(digitalRead(BUTTONLEFT)));
      oled.setCursor(0,45);
      oled.print("U: " + String(digitalRead(BUTTONUP)));
      oled.setCursor(0,60);
      oled.print("D: " + String(digitalRead(BUTTONDOWN)));
      oled.setCursor(95,60);
      oled.print("M: " + String(digitalRead(BUTTONMIDDLE)));
      oled.setCursor(95,45);
      oled.print("R: " + String(digitalRead(BUTTONRIGHT)));
    } while(oled.nextPage());
  }
  else if(DisplayMode == 10){ //access variable
    oled.nextPage();
    oled.clear();
    do{
      oled.setCursor(0,15);
      oled.print("access:");
      oled.setCursor(0,30);
      oled.print(status);
      oled.setCursor(0,45);
      oled.print(ACCESSVARIABLE);
    } while(oled.nextPage());
  }
  else{ //ERROR!!!
    oled.nextPage();
    oled.clear();
    do{
      oled.setCursor(0,15);
      oled.print(status);
      oled.setCursor(0,40);
      oled.print("ERROR!!!");
    } while(oled.nextPage());
  }
}
void fancyMapsWrite(){
  if(vgd == NORTH){xc++;}
  if(vgd == SOUTH){xc--;}
  if(vgd == EAST){yc++;}
  if(vgd == WEST){yc--;}
  if(vgd >= 4){vgd = 0;}
  if(vgd < 0){vgd = 3;}
  fancyPlanning();
}

//fourth order spells:

//the robot realigns itself to the vgd grid using the realigned compass values
void compassAlign(){
  int speed = 0;
  int accurateCompass = compass.getHeading()-turnCal;
  if(accurateCompass<0){accurateCompass+=360;}
  else if(accurateCompass>360){accurateCompass-=360;}
  int shiftedAccurateCompass = accurateCompass;
  int shiftedVgd = vgd;
  if(vgd==NORTH){
    shiftedVgd = 2;
    shiftedAccurateCompass += 180;
    if(shiftedAccurateCompass>360){shiftedAccurateCompass-=360;}
  }
  if(shiftedAccurateCompass<(shiftedVgd*90)){speed = 100;}
  else{speed = -100;}
  while(!tolerance(shiftedAccurateCompass, (shiftedVgd*90), 2)){
    oledValues(3, "compassAlign");
    turn(speed);
    if(vgd==NORTH){shiftedAccurateCompass = compass.getHeading()-turnCal + 180;}
    else{shiftedAccurateCompass = compass.getHeading()-turnCal;}
    if(shiftedAccurateCompass<=0){shiftedAccurateCompass+=360;}
    else if(shiftedAccurateCompass>360){shiftedAccurateCompass-=360;}
    if(isPaused){break;}
  }
  stop();
}
void Menu(int refreshRate = 500, String variableName = "variable"){
  String status[10];
  status[0] = "laser";
  status[1] = "colour";
  status[2] = "compass";
  status[3] = "realign";
  status[4] = "Noa & Fabian";
  status[5] = "known";
  status[6] = "order";
  status[7] = "status";
  status[8] = "buttons";
  status[9] = variableName;
  status[10] = "ERROR!";
  while(isPaused){
    if(lastRefresh+refreshRate<=millis()){
      lastRefresh = millis();
      oledValues(displayMode, status[displayMode-1]);
    }
    if(displayMode==10){
      if(digitalRead(BUTTONUP)==LOW && lastPress+500<=millis()){
        lastPress = millis();
        ACCESSVARIABLE++;
      }
      else if(digitalRead(BUTTONDOWN)==LOW && lastPress+500<=millis()){
        lastPress = millis();
        ACCESSVARIABLE--;
      }
    }
    if(digitalRead(BUTTONRIGHT)==LOW && lastPress+500<=millis()){
      lastPress = millis();
      displayMode++;
    }
    else if(digitalRead(BUTTONLEFT)==LOW && lastPress+500<=millis()){
      lastPress = millis();
      displayMode--;
    }
    if(displayMode<=0){
      displayMode=10;
    }
    else if(displayMode>=11){
      displayMode=1;
    }
    #if DEBUGLED==1
      if(isPaused){digitalWrite(LED, HIGH);}
      else if(!isPaused){digitalWrite(LED, LOW);}
    #endif
  }
}
//the robot realigns itself to the left wall of the maze
void realign(){
  int speed = 100;
  if(laser_left.getDistanceCentimeters()-2<DISTANCE && laser_left_back.getDistanceCentimeters()<DISTANCE){
    if(laser_left.getDistanceCentimeters()-2<laser_left_back.getDistanceCentimeters()){
      while(!tolerance(laser_left.getDistanceCentimeters()-2, laser_left_back.getDistanceCentimeters(), 1)){
        turn(speed);
        oledValues(4, "Realign");
        if(isPaused){break;}
      }
    }
    else if(laser_left.getDistanceCentimeters()-2>laser_left_back.getDistanceCentimeters()){
      while(!tolerance(laser_left_back.getDistanceCentimeters(), laser_left.getDistanceCentimeters()-2, 1)){
        turn(-speed);
        oledValues(4, "Realign");
        if(isPaused){break;}
      }
    }
    stop();
    turnCal=compass.getHeading()%90;
    if(turnCal>45){
      turnCal=turnCal-90;
    }
  }
  else if(laser_right.getDistanceCentimeters()-1<DISTANCE && laser_right_back.getDistanceCentimeters()<DISTANCE){
    if(laser_right.getDistanceCentimeters()-1<laser_right_back.getDistanceCentimeters()){
      while(!tolerance(laser_right.getDistanceCentimeters()-1, laser_right_back.getDistanceCentimeters(), 1)){
        turn(-speed);
        oledValues(4, "Realign");
        if(isPaused){break;}
      }
    }
    else if(laser_right.getDistanceCentimeters()-1>laser_right_back.getDistanceCentimeters()){
      while(!tolerance(laser_right_back.getDistanceCentimeters(), laser_right.getDistanceCentimeters()-1, 1)){
        turn(speed);
        oledValues(4, "Realign");
        if(isPaused){break;}
      }
    }
    stop();
    turnCal=compass.getHeading()%90;
    if(turnCal>45){
      turnCal=turnCal-90;
    }
  }
}

//fifth order spells:

//the robot drives until it reaches the next tile
void runtile(int speed = RUN){
  float plate_distance = ((visibleTiles() - 1) * TILESIZE)+10;
  while(lasersFront()>plate_distance){
    oledValues(1, "Runtile");
    run(speed);
    if(laser_left.getDistanceCentimeters()<9){ //prevents the robot from bumping into the left wall
      turn(TURN);
      delay(100);
      stop();
    }
    if(laser_right.getDistanceCentimeters()<9){ //prevents the robot from bumping into the right wall
      turn(-TURN);
      delay(100);
      stop();
    }
    if(detectColour()=='S'){ //stops the motion when the robot enters a black tile
      stop();
      break;
    }
    if(laser_front.getDistanceCentimeters()<7){ //stops the robot before bumping into the front wall
      stop();
      oledValues(9, "WALL");
      break;
    }
    if(abs(compass.getRoll())>35){ // stops the robot when it is to steep, for example when the robot tries to drive up a wall
      stop();
      break;
    }
    if(isPaused){break;}
  }
  stop(); //stops any motor movement
	//writes its new position to the map
	#if FANCY==1
		fancyMapsWrite();
	#else
  	mapsWrite();
	#endif
  compassAlign();
  realign(); //realigns the robot to a side wall (if there is one)
  if(xc==25&&yc==25 && INDICATEFINISH==1){ //if the robot is on its starting tile, it indicates that
    oledValues(5, "Finished!");
    blink(1000, 5);
  }
  didDriveBefore=1;
}
//the robot turns itself about 90 degrees
void turn90(int speed = TURN){
  double tol = 0.5;
  int direction = compass.getHeading();
  int Ndirection = comp();
  if(speed>0){
    if(vgd==NORTH){
      while(!tolerance(direction, 90+turnCal, tol)){
        turn(speed);
        direction = compass.getHeading();
        if(isPaused){break;}
      }
      vgd++;
    }
    else if(vgd==EAST){
      while(!tolerance(direction, 180+turnCal, tol)){
        turn(speed);
        direction = compass.getHeading();
        if(isPaused){break;}
      }
      vgd++;
    }
    else if(vgd==SOUTH){
      while(!tolerance(direction, 270+turnCal, tol)){
        turn(speed);
        direction = compass.getHeading();
        if(isPaused){break;}
      }
      vgd++;
    }
    else if(vgd==WEST){
      while(!tolerance(Ndirection, 45+turnCal, tol)){
        turn(speed);
        Ndirection = comp();
        if(isPaused){break;}
      }
      vgd++;
    }
    if(vgd>WEST){
      vgd=NORTH;
    }
    stop();
  }
  if(speed<0){
    if(vgd==WEST){
      while(!tolerance(direction, 180+turnCal, tol)){
        turn(speed);
        direction = compass.getHeading();
        if(isPaused){break;}
      }
      vgd--;
    }
    else if(vgd==SOUTH){
      while(!tolerance(direction, 90+turnCal, tol)){
        turn(speed);
        direction = compass.getHeading();
        if(isPaused){break;}
      }
      vgd--;
    }
    else if(vgd==EAST){
      while(!tolerance(Ndirection, 45+turnCal, tol)){
        turn(speed);
        Ndirection = comp();
        if(isPaused){break;}
      }
      vgd--;
    }
    else if(vgd==NORTH){
      while(!tolerance(direction, 270+turnCal, tol)){
        turn(speed);
        direction = compass.getHeading();
        if(isPaused){break;}
      }
      vgd--;
    }
    if(vgd<NORTH){
      vgd=WEST;
    }
    stop();
  }
  realign();
  compassAlign();
  didDriveBefore=0;
}
//the robot turns and then drives one tile

//sixth order spells:

void turnrun(int speed90 = TURN, int speedRun = RUN){
  turn90(speed90);
  runtile(speedRun);
}
//executes actions following the recognition of a letter
void actLetter(int camPort){
  char currentLetter=averageLetter(camPort, 100);
  int direction = 0;
  if(camPort==CAMLEFT  || camPort==CAMLEFTBACK) {direction=0;}
  if(camPort==CAMRIGHT || camPort==CAMRIGHTBACK){direction=2;}
  if(currentLetter!='X' && ((laser_left.getDistanceCentimeters()<DISTANCE && direction==0) || (laser_right.getDistanceCentimeters()<DISTANCE && direction==2))){
    if((currentLetter=='U' && maps[xc][yc].getWallLetter(direction)!='U') || (currentLetter=='G' && maps[xc][yc].getWallLetter(direction)!='G')){
      oledValues(8, "Unharmed(" + String(currentLetter) + "), " + String(camPort));
      blink(500, 5);
      if(currentLetter=='U'){maps[xc][yc].setWallLetter('U',direction);}
      if(currentLetter=='G'){maps[xc][yc].setWallLetter('G',direction);}
    }
    else if((currentLetter=='S' && maps[xc][yc].getWallLetter(direction)!='S') || (currentLetter=='Y' && maps[xc][yc].getWallLetter(direction)!='Y')){
      oledValues(8, "Slightly(" + String(currentLetter) + "), " + String(camPort));
      blink(500, 5);
      turn90();
      deployMedikit();
      turn90(-TURN);
      if(currentLetter=='S'){maps[xc][yc].setWallLetter('S',direction);}
      if(currentLetter=='Y'){maps[xc][yc].setWallLetter('Y',direction);}
    }
    else if((currentLetter=='H' && maps[xc][yc].getWallLetter(direction)!='H') || (currentLetter=='R' && maps[xc][yc].getWallLetter(direction)!='R')){
      oledValues(8, "Harmed(" + String(currentLetter) + "), " + String(camPort));
      blink(500, 5);
      turn90();
      deployMedikit();
      deployMedikit();
      turn90(-TURN);
      if(currentLetter=='H'){maps[xc][yc].setWallLetter('H',direction);}
      if(currentLetter=='R'){maps[xc][yc].setWallLetter('R',direction);}
    }
  }
}

//seventh order spells:

//depending on the mode, the robot performs different driving algorithms
void Drive(int mode){
  int front, left, right=0;
  if(laser_front.getDistanceCentimeters()>DISTANCE){front=1;}
  if(laser_right.getDistanceCentimeters()>DISTANCE){right=1;}
  if(laser_left.getDistanceCentimeters()>DISTANCE){left=1;}
  if(mode==1){ //right wall
    if(right==1){turnrun();}
    else{
      if(front==1){runtile();}
      else{
        if(left==1){turnrun(-TURN);}
        else{turn90();}
      }
    }
  }
  if(mode==2){ //left wall
    if(left==1){turnrun(-TURN);}
    else{
      if(front==1){runtile();}
      else{
        if(right==1){turnrun();}
        else{turn90(-TURN);}
      }
    }
  }
  if(mode==3){ //completely random
    int dice = random(front+left+right);
    if(dice==0){
      if(front==1){runtile();}
      else if(left==1){turnrun(-TURN);}
      else if(right==1){turnrun();}
      else{turn90(); turn90();}
    }
    if(dice==1){
      if(right==1){turnrun();}
      else if(left==1){turnrun(-TURN);}
    }
    if(dice==2){
      if(left==1){turnrun(-TURN);}
    }
  }
  if(mode==4){ //with system(preferred right-front-left)
    if(lasersRightFree() && known(2)==0){
      oledValues(1,"right");
      turnrun();
    }
    else if(laser_front.getDistanceCentimeters()>DISTANCE && known(1)==0){
      oledValues(1,"front");
      runtile();
    }
    else if(lasersLeftFree() && known(3)==0){
      oledValues(1,"left");
      turnrun(-TURN);
    }
    else{
      int tempLastFreeTile=lastFreeTile();
      if(tempLastFreeTile==1){
        oledValues(1,"front2");
        runtile();
      }
      else if(tempLastFreeTile==2){
        oledValues(1,"right2");
        turnrun();
      }
      else if(tempLastFreeTile==3){
        oledValues(1,"left2");
        turnrun(-TURN);
      }
      else if(tempLastFreeTile==4 || (tileOrder==1 && vgd==NORTH && laser_front.getDistanceCentimeters()<DISTANCE && laser_right.getDistanceCentimeters()<DISTANCE && laser_left.getDistanceCentimeters()<DISTANCE)){
        oledValues(1,"back2");
        turn90();
      }
      else if(tempLastFreeTile==5){
        oledValues(1,"nowhere to go");
      }
    }
  }
  if(mode==5){ //with system(preferred left-front-right)
    if(lasersLeftFree(DISTANCE) && known(3)==0){
      oledValues(1,"left");
      turnrun(-TURN);
    }
    else if(laser_front.getDistanceCentimeters()>DISTANCE && known(1)==0){
      oledValues(1,"front");
      runtile();
    }
    else if(lasersRightFree(DISTANCE) && known(2)==0){
      oledValues(1,"right");
      turnrun();
    }
    else{
      int tempLastFreeTile=lastFreeTile();
      if(tempLastFreeTile==1){
        oledValues(1,"front2");
        runtile();
      }
      else if(tempLastFreeTile==2){
        oledValues(1,"right2");
        turnrun();
      }
      else if(tempLastFreeTile==3){
        oledValues(1,"left2");
        turnrun(-TURN);
      }
      else if(tempLastFreeTile==4 || (tileOrder==1 && vgd==NORTH && laser_front.getDistanceCentimeters()<DISTANCE && laser_right.getDistanceCentimeters()<DISTANCE && laser_left.getDistanceCentimeters()<DISTANCE)){
        oledValues(1,"back2");
        turn90();
      }
      else if(tempLastFreeTile==5){
        oledValues(1,"nowhere to go");
      }
    }
  }
  if(mode==6){ //with system(random preferred)
    int reTrack=0;
    int coincidence=random(2);
    if(coincidence==0){
      if(lasersLeftFree(DISTANCE) && known(3)==0){
        oledValues(1,"left");
        turnrun(-TURN);
      }
      else if(laser_front.getDistanceCentimeters()>DISTANCE && known(1)==0){
        oledValues(1,"front");
        runtile();
      }
      else if(lasersRightFree(DISTANCE) && known(2)==0){
        oledValues(1,"right");
        turnrun();
      }
      else{
        reTrack=1;
      }
    }
    else if(coincidence==1){
      if(lasersRightFree(DISTANCE) && known(2)==0){
        oledValues(1,"right");
        turnrun();
      }
      else if(laser_front.getDistanceCentimeters()>DISTANCE && known(1)==0){
        oledValues(1,"front");
        runtile();
      }
      else if(lasersLeftFree(DISTANCE) && known(3)==0){
        oledValues(1,"left");
        turnrun(-TURN);
      }
      else{
        reTrack=1;
      }
    }
    else if(coincidence==2){
      if(laser_front.getDistanceCentimeters()>DISTANCE && known(1)==0){
        oledValues(1,"front");
        runtile();
      }
      else if(lasersRightFree(DISTANCE) && known(2)==0){
        oledValues(1,"right");
        turnrun();
      }
      else if(lasersLeftFree(DISTANCE) && known(3)==0){
        oledValues(1,"left");
        turnrun(-TURN);
      }
      else{
        reTrack=1;
      }
    }
    if(reTrack==1){
      int tempLastFreeTile=lastFreeTile();
      if(tempLastFreeTile==1){
        oledValues(1,"front2");
        runtile();
      }
      else if(tempLastFreeTile==2){
        oledValues(1,"right2");
        turnrun();
      }
      else if(tempLastFreeTile==3){
        oledValues(1,"left2");
        turnrun(-TURN);
      }
      else if(tempLastFreeTile==4 || (tileOrder==1 && vgd==NORTH && laser_front.getDistanceCentimeters()<DISTANCE && laser_right.getDistanceCentimeters()<DISTANCE && laser_left.getDistanceCentimeters()<DISTANCE)){
        oledValues(1,"back2");
        turn90();
      }
      else if(tempLastFreeTile==5){
        oledValues(1,"nowhere to go");
      }
    }
  }
}
//performs the fanciest of all driving algorithms made by the one and only Fabian
void fancyDrive(int mode){
  int fancyResult = fancyDriving(mode);
  if(fancyResult==FRONT){
    runtile();
  }
  else if(fancyResult==LEFT){
    turnrun(-TURN);
  }
  else if(fancyResult==RIGHT){
    turnrun();
  }
  else if(fancyResult==BACK){
    turn90();
    turnrun();
  }
}

//eigth order spells:

void loop(){
//Testmode:
  #if TESTMODE==0
    Menu(500, "demo");
    #if DEBUGLED==1
      if(isPaused){digitalWrite(LED, HIGH);}
      else if(!isPaused){digitalWrite(LED, LOW);}
    #endif
  #endif
  #if TESTMODE==1
    Serial.println("laser_front: ");
    Serial.println(laser_front.getDistanceCentimeters());
    Serial.println("compass: ");
    Serial.println(compass.getHeading());
    Serial.println("laser_front_bottom: ");
    Serial.println(laser_front_bottom.getDistanceCentimeters());
    Serial.println("laser_right: ");
    Serial.println(laser_right.getDistanceCentimeters());
    Serial.println("laser_left: ");
    Serial.println(laser_left.getDistanceCentimeters());
    Serial.println("laser_back: ");
    Serial.println(laser_back.getDistanceCentimeters());
    Serial.println("laser_right_back: ");
    Serial.println(laser_right_back.getDistanceCentimeters());
    Serial.println("laser_left_back: ");
    Serial.println(laser_left_back.getDistanceCentimeters());
    delay(500);
    return;
  #endif
  #if TESTMODE==2
    Drive(4);
    return;
  #endif
  #if TESTMODE==3
    //Menu(500, "demo");
    return;
  #endif
  /*oledValues(8, "running");
  delay(250);
  return;*/
//Hinge:
  if(compass.getRoll()>35){
    oledValues(3,"Hinge");
    run(-RUN);
    delay(500);
    stop();
  }
//Letter recognition:
  actLetter(CAMLEFT);
  actLetter(CAMRIGHT);
  actLetter(CAMLEFTBACK);
  actLetter(CAMRIGHTBACK);
//Colour recognition:
  if(detectColour()=='S'){ //Black
    oledValues(2,"Black");
    maps[xc][yc].setFloorColour('s');
    int goal = ((visibleTiles()+1)*30);
    while(goal>lasersFront()){
      oledValues(1, String(goal));
      run(-RUN);
      if(isPaused){break;}
    }
    stop();
    if(vgd == NORTH){xc--;} //This part is for prohibiting the robot of thinking it is
    if(vgd == SOUTH){xc++;} //on the black tile rather than on the tile
    if(vgd == EAST){yc--;}  //before that, which it is really on,
    if(vgd == WEST){yc++;}  //because it didn't actually enter the black one.
  }
  else if(detectColour()=='R'){ //Red
    oledValues(2,"Red");
    maps[xc][yc].setFloorColour('r');
  }
  else if(detectColour()=='B'){ //Blue
    oledValues(2,"Blue");
    maps[xc][yc].setFloorColour('b');
    unsigned int start = millis();
    while(start+5000>=millis()){
      stop();
      if(isPaused){break;}
    }
  }
  else if(detectColour()=='C'){ //Silver
    oledValues(2,"Checkpoint");
    maps[xc][yc].setFloorColour('c');
  }
//Driving:
  fancyDrive(LEFT);
}