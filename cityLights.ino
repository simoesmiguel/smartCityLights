
#include <Wire.h> 
#include <avr/wdt.h>
#include <time.h>
#include <QList.h>

int previous_value;
int ledPin1=10;
int ledPin2=9;
int const yellowLed = A0;
int const photoresistorPin = A2;
int const led1FailurePinChecker = A3; //led1 faiure check
int const led2FailurePinChecker = A1; //led2 faiure check
int const pot1=8;
int const pot2=11;

int maxRead;
int darkTreshold=350;

//both cell coords
int a0=2;
int a1=3;
int a2=4;
int a3=5;
int b0=6;
int b1=7;
int b2=12;
int b3=13;

int ledFailureTreshold =900;

int cell1_x,cell1_y,cell2_x,cell2_y;
boolean b2State,b1State;
boolean led1inMedianState,led2inMedianState;
float t1=0;
float t2=0;
int potentiometer1Value;
int potentiometer2Value;

int ledMaxValue=255;
int ledMedianValue=40;
int ledSecurityLevel=5;
int ledOff=0;

int nbrOf254Messages =0;
int nbrOf255Messages =0;

int x_predict,y_predict;
int lastX_popped,lastY_popped;

unsigned long avgDifference;

QList<int> destiny_cells;
QList<int> neighbors_addresses;

QList<long> lastTimestampsReceived;
QList<int> lastSourcesReceived;

QList<long> timestampsDifferences;

String cell1_x_binary,cell1_y_binary,cell2_x_binary,cell2_y_binary;

boolean isMonitor=false;
boolean led1IsDown=false;
boolean led2IsDown=false;

byte message [7];

void setup() {
  Serial.begin(9600);

  pinMode (ledPin1, OUTPUT); 
  pinMode (ledPin2, OUTPUT); 
  pinMode (a0, OUTPUT); 
  pinMode (a1, OUTPUT); 
  pinMode (a2, OUTPUT); 
  pinMode (a3, OUTPUT); 
  pinMode (b0, OUTPUT); 
  pinMode (b1, OUTPUT); 
  pinMode (b2, OUTPUT); 
  pinMode (b3, OUTPUT); 

  pinMode(photoresistorPin, INPUT); //photoresistorSensor
  pinMode(led1FailurePinChecker, INPUT); //photoresistorSensor
  pinMode(led2FailurePinChecker, INPUT); //photoresistorSensor

  pinMode(pot1, INPUT); 
  pinMode(pot2, INPUT); 

  
  maxRead=readPhotoresistorBaseValue();

  //read both potentiometers default values
  potentiometer1Value= readPotentiometer(1);
  potentiometer2Value= readPotentiometer(2);

  // get cell1 and cell2 coordinates
  String coords=checkCoordinates();
  String x,y;
  for(int i=0;i<coords.length();i++){
    if(i<=3)
      x+=coords[i];
    else if(i>4)
      y+=coords[i];
  } 
  cell1_x=binaryToDecimal(x);
  cell1_y=binaryToDecimal(y);
  cell2_x=cell1_x+1;
  cell2_y= cell1_y; 
  
  cell1_x_binary=x.c_str();
  cell1_y_binary=y.c_str();
  cell2_x_binary= decToBinary(cell2_x,4);
  cell2_y_binary= decToBinary(cell2_y,4);

  int my_address = calculateAddress(cell1_x,cell1_y); 
  Serial.print("my_address :");
  Serial.println(my_address);

  Serial.println("cell 1: ");
  Serial.print("x: ");
  Serial.print(cell1_x);
  Serial.print(" | y: ");
  Serial.print(cell1_y);
  Serial.println();
  Serial.println("cell 2: ");
  Serial.print("x: ");
  Serial.print(cell2_x);
  Serial.print(" | y: ");
  Serial.println(cell2_y);  
  Serial.println("======================");

  Wire.begin(my_address);
  Wire.onReceive(receiveEvent);

  if((cell1_x==0 && cell1_y ==0) || (cell2_x==0 && cell2_y ==0)){
    isMonitor=true;
  }

/*
 clockSync(millis());
 
  avgDifference=calculateAvg();

  Serial.print("Old time: ");
  Serial.println(millis());
  Serial.print("New time: ");
  Serial.println(millis()-avgDifference);
*/

}

int calculateAvg(){
  int N=timestampsDifferences.size();
  unsigned long sum=0;
  for (int i=0;i<N;i++){
    sum+=timestampsDifferences.at(i);
  }
  Serial.print("difference: ");
  Serial.println(sum/N);
  return sum/N;
}


// clock synchronization
void clockSync(long a){
  QList<int> addressesNotRepeated;

  // we just want 4 nighbors of cell (x,y)- the north neighbor, the south, the west and east
  neighbors(cell1_x,cell1_y,"four");
  neighbors(cell2_x,cell2_y,"four"); 

  //delete the repeated values inside the neighbors_addresses array
  for(int i=0;i<neighbors_addresses.size();i++){
    bool repeated=false;
    for(int j=0;i<addressesNotRepeated.size();j++){
      if(neighbors_addresses.at(i)==addressesNotRepeated.at(j))
        repeated=true;    
    }
    if(repeated=false)
      addressesNotRepeated.push_back(neighbors_addresses.at(i));
  }



  QList<long> timestampsDifferences_Copy;

  while ((millis()-a)<5000){ //first 5 secs
    sendClock(cell1_x,cell1_y); //sends its clock to up to 4 neighbors always with an updated timestamp
    

    // while(timestampsDifferences_Copy.size() < neighbors_addresses.size()){
  //  while(timestampsDifferences_Copy.size()<1){ 
    //  Serial.println(timestampsDifferences.size());
      Serial.println("Receiving message ...");

      //source
      byte source = Wire.read();  // source == cell where the movement was detected
      //event
      byte event = Wire.read();

      //timestamp
      unsigned long data[4];
      data[0] = Wire.read();
      data[1] = Wire.read();
      data[2] = Wire.read();
      data[3] = Wire.read();
      unsigned long timestamp = (data[0] << 24) + (data[1] << 16) + (data[2] << 8) + data[3];
     
      //destination
      byte dest = Wire.read();

      Serial.print("timestamp: ");
      Serial.println(timestamp);
      Serial.print("Event: ");
      Serial.println(event);
      
      // gotta wait until receive all neighbors messages        
      Serial.println("w");
      timestampsDifferences.push_back(millis()-timestamp);
    //  timestampsDifferences_Copy.push_back(millis()-timestamp);

   // }
    avgDifference=calculateAvg(); //always calculate the avg difference in order to the clocks converge to the same value.
   // timestampsDifferences_Copy.clear();
  }
  //clear both neighbors and addresses arrays
  neighbors_addresses.clear();
  destiny_cells.clear();
}


unsigned long getTimeNow(){
  return millis()-avgDifference;
}

//function that is responsible for receiving all the messages 
void receiveEvent(int bytes) {
  
  Serial.println("Receiving message ...");

  //source
  byte source = Wire.read();  // source == cell where the movement was detected
  byte sourceX = source >> 4;
  byte aux = source << 4;
  byte sourceY = aux >> 4;

  //event
  byte event = Wire.read();

  //timestamp
  unsigned long data[4];
  data[0] = Wire.read();
  data[1] = Wire.read();
  data[2] = Wire.read();
  data[3] = Wire.read();
  unsigned long timestamp = (data[0] << 24) + (data[1] << 16) + (data[2] << 8) + data[3];
 
  //destination
  byte dest = Wire.read();
  byte destX = dest >> 4;
  byte aux2 = dest << 4;
  byte destY = aux2 >> 4;

  Serial.print("Src: ");
  Serial.println(String(sourceX)+String(sourceY));
  Serial.print("timestamp: ");
  Serial.println(timestamp);
  Serial.print("Dst: ");
  Serial.println(String(destX)+String(destY));
  Serial.print("Event: ");
  Serial.println(event);
  
  checkEvent(event,destX,destY,sourceX,sourceY,timestamp);

}

bool saveCoordsAndTimestamp(int sourceX, int sourceY,float timestamp){
  if(lastSourcesReceived.size()==0 && lastTimestampsReceived.size()==0){
       //save the timestamp and the source in order to detect potential correlated movements.
    lastSourcesReceived.push_back(sourceX);
    lastSourcesReceived.push_back(sourceY);
    lastTimestampsReceived.push_back(timestamp);
    return true;
  }
  return false;
}

void checkEvent(int event,int dstx,int dsty,int sourceX, int sourceY,long timestamp){
  if(event==0){
    //clock synchronization 

  }else if(event==1){
    //movement detected
    saveCoordsAndTimestamp(sourceX, sourceY, timestamp);

    if(dstx==cell1_x && dsty==cell1_y){
      adjustLed("cell_1","median");
    }
    else if(dstx==cell2_x && dsty==cell2_y){
      adjustLed("cell_2","median");
    }

  }else if(event==2){
    //correlated movement 
    if(dstx==cell1_x && dsty==cell1_y)
      adjustLed("cell_1","max");
    else if(dstx==cell2_x && dsty==cell2_y)
      adjustLed("cell_2","max");

  }else if(event==254){
    //failure of a lamp
    if(isMonitor)
      nbrOf254Messages+=1;

  }else if(event==255){
    //lamp is OK
    if(isMonitor)
      nbrOf255Messages+=1;
  }

}



void loop() {
  //check potentiometer
  
  boolean b1=checkPotentiometerValue(1);  
  boolean b2=checkPotentiometerValue(2);
  //Serial.println(b1);
  //Serial.println(b2);
  
  if(b1==true){ //foi detetado movimento
    bool predict=false;
    bool saved= saveCoordsAndTimestamp(cell1_x,cell1_y,getTimeNow());
    if(saved==false)
      bool predict = predictMovement(1);
    adjustLeds("cell_1"); //fazer o ajuste da luminosidade dos leds
    if(predict==false)  
      sendMessage(cell1_x,cell1_y,1);
  }
  if(b2==true){
    bool predict=false;
    bool saved=saveCoordsAndTimestamp(cell2_x,cell2_y,getTimeNow());
    if(saved==false)
      bool predict = predictMovement(2);
    adjustLeds("cell_2");
    if(predict==false)  
      sendMessage(cell2_x,cell2_y,1);
  }
  if(b1==false && b2==false){  //nao foi detetado movimento
    adjustLeds("noMotion");  
  }
  checkCounter(1);
  checkCounter(2);
  checkLedsFailure(); 
  checkNmbr254Messages();
  
}

void predictFutureCoords(int i){
  int x_diff,y_diff;

  lastX_popped = lastSourcesReceived.at(0);
  lastSourcesReceived.pop_front();
  lastY_popped = lastSourcesReceived.at(0);
  lastSourcesReceived.pop_front();
  Serial.println("last source X  || last source Y ");
  Serial.print(lastX_popped);
  Serial.println(lastY_popped);

  if(i ==1){   
    x_diff = cell1_x - lastX_popped;
    y_diff = cell1_y - lastY_popped;
    Serial.println("x_diff  || y_diff ");
    Serial.print(x_diff);
    Serial.println(y_diff);
    x_predict = cell1_x+ x_diff;
    y_predict = cell1_y+ y_diff;
    Serial.println("x_predict  || y_predict ");
    Serial.print(x_predict);
    Serial.println(y_predict);
  
  }else if(i==2){

    x_diff = cell2_x - lastX_popped;
    y_diff = cell2_y - lastY_popped;
    Serial.println("x_diff  || y_diff ");
    Serial.print(x_diff);
    Serial.println(y_diff);
    x_predict = cell2_x+ x_diff;
    y_predict = cell2_y+ y_diff;
    Serial.println("x_predict  || y_predict ");
    Serial.print(x_predict);
    Serial.println(y_predict);
  }
}

bool isDiagonal(int i){
  if(i==1){
    if( abs(cell1_x -x_predict)==1 && abs(cell1_y - y_predict)==1){
      return true;
    }
  }
  else if(i==2){
    if( abs(cell2_x -x_predict)==1 && abs(cell2_y - y_predict)==1){
      return true;
    }
  }
  return false;
}

bool predictMovement(int i){
  if(lastTimestampsReceived.size() != 0){
    predictFutureCoords(i);
    bool is_diagonal=isDiagonal(i);

    int delta_l;
    if(is_diagonal)
      delta_l=57;
    else
      delta_l=40;


    long lastTimeStamp = lastTimestampsReceived.at(0);
    long currentTimeStamp = getTimeNow();
    lastTimestampsReceived.pop_front();
    long delta_t = (currentTimeStamp-lastTimeStamp);
    delta_t=delta_t/1000;

    lastTimestampsReceived.clear();
    lastSourcesReceived.clear();

    
    Serial.println("Trying to predict movement: ");
    Serial.print("delta_l: ");
    Serial.println(delta_l);
    Serial.println("actual_time  ||  last_time_received");
    Serial.println(String(currentTimeStamp)+" || "+String(lastTimeStamp));
    Serial.print("delta_t: ");
    Serial.println(delta_t);
    Serial.print("delta_l/delta_t: ");
    Serial.println(delta_l/delta_t);

    Serial.println("x_predict  ||  y_predict");
    Serial.print(x_predict);
    Serial.println(y_predict);
    
    if((delta_l/delta_t)>0){
      if((delta_l/delta_t)>5.6){ //predict movement
        // verify if the predicted cell is one of the cells inside the same arduino
        if( ((String(x_predict)+String(y_predict))==String(cell1_x)+String(cell1_y)) || ((String(x_predict)+String(y_predict))==String(cell2_x)+String(cell2_y))){
          if((String(x_predict)+String(y_predict))==String(cell1_x)+String(cell1_y)){
            adjustLed("cell_1","max");
          }
          else if((String(x_predict)+String(y_predict))==String(cell2_x)+String(cell2_y)){
            adjustLed("cell_2","max");
          }
        }
        else{ // if the predicted cell is not inside the same arduino, send a message to that predicted cell
          if(i==1)
            sendOneMessage(cell1_x, cell1_y, x_predict, y_predict, 2);
          else if(i==2)
            sendOneMessage(cell2_x, cell2_y, x_predict, y_predict, 2);

        }
        return true;
      }else if((delta_l/delta_t)<5.6)
        return false;
    }
  }
  return false;
}


void checkNmbr254Messages(){
  if(isMonitor){
    if(nbrOf254Messages>nbrOf255Messages){
      Serial.println("LIGANDO LED AMARELO ");
      analogWrite(yellowLed,255);
    }
    else if(nbrOf255Messages==nbrOf254Messages){
      analogWrite(yellowLed,0);
      led2IsDown=false;
      led1IsDown=false;
    }
  }
}

int readPhotoresistor(){
  int value = constrain(analogRead(photoresistorPin),0,maxRead);  // constrain all values to he min and max value read by the photoresistor  
  //return value; 
  return 0; // SO PORQUE ESTAMOS COM UM PROBLEMA NUM DOS PHOTORESISTORS
}

int readPhotoresistorAndMap(){  
  int value = constrain(analogRead(photoresistorPin),0,maxRead);  // constrain all values to he min and max value read by the photoresistor  
  return map(value,0,maxRead,255,0); // map those values to the interval [0,255]
}



int readPhotoresistorBaseValue(){
   int maxi=analogRead(photoresistorPin);
  return maxi;
}


void adjustLeds(String cellId){
  //Serial.println(readPhotoresistor());
  if(readPhotoresistor()<darkTreshold){ //very dark or cloudy day
    if(cellId=="cell_1"){
      Serial.println("motion in cell 1");
      analogWrite(ledPin1,ledMaxValue); // onde foi detetado o movimento
      startCounter(1);
      if(t2==0){
        analogWrite(ledPin2,ledMedianValue); // a volta de onde foi detetado o movimento
        startCounter(2);
      }
    }
    else if(cellId=="cell_2"){
      Serial.println("motion in cell 2");      
      analogWrite(ledPin2,ledMaxValue); // onde foi detetado o movimento
      startCounter(2);
      if(t1==0){
        analogWrite(ledPin1,ledMedianValue); // a volta de onde foi detetado o movimento
        startCounter(1);
      }
    }
    else if(cellId=="noMotion"){
      if(t1==0)
        analogWrite(ledPin1,ledSecurityLevel); //luz de segurança
      if(t2==0)
        analogWrite(ledPin2,ledSecurityLevel); //luz de segurança  
    }
  }
  else{
      analogWrite(ledPin1,readPhotoresistorAndMap()); //luz de segurança  
      analogWrite(ledPin2,readPhotoresistorAndMap()); //luz de segurança  

  }
}

// adjust led when a message is received
void adjustLed(String cellId,String value){
  if(readPhotoresistor()<darkTreshold){ //very dark or cloudy day
    if(cellId=="cell_1"){
      if(t1==0){
        if(value=="median")
          analogWrite(ledPin1,ledMedianValue); // onde foi detetado o movimento
        else if(value=="max"){
          analogWrite(ledPin1,ledMaxValue);
          // send message another neighbor ??
        }
        startCounter(1);
      }
    }
    else {
      if(t2==0){
        if(value=="median")
          analogWrite(ledPin2,ledMedianValue); // onde foi detetado o movimento
        else if(value=="max"){
          analogWrite(ledPin2,ledMaxValue);
          // send message another neighbor ??
        }
        startCounter(2);
      }
    }
  }
}

String checkCoordinates(){
  int coords[] = {a3,a2,a1,a0,b3,b2,b1,b0};
  String x,y;
  for(int &coord : coords){
    if(coord==a0 || coord==a1 || coord==a2 || coord==a3){
      int a= digitalRead(coord);
      x+=a;
    }
    else{
      int b= digitalRead(coord);
      y+=b;
    }
  }
  return x+","+y;
}



int binaryToDecimal(String n) 
{ 
    String num = n; 
    int dec_value = 0; 
      
    // Initializing base value to 1, i.e 2^0 
    int base = 1; 
      
    int len = num.length(); 
    for (int i=len-1;i>=0;i--) 
    { 
        if (num[i] == '1')         
            dec_value += base; 
        base = base * 2; 
    } 
      
    return dec_value; 
} 

String decToBinary(int n,int mini) 
{ 
  String r="";
   while(n!=0){
    r=(n%2==0 ? "0":"1")+r;
    n=n/2;
   }
   if(r.length()!=mini){
    int dif = mini-r.length();
    for (int i=0;i<dif;i++){
      r="0"+r ;
    }
   }
   return r;
} 

void checkLedsFailure(){
  if(analogRead(led1FailurePinChecker)>ledFailureTreshold){ // so detecta quando o led esta no maximo 
    if(led1IsDown==false){
      if(isMonitor)
        nbrOf254Messages+=1;
      else 
        sendMessage(cell1_x, cell1_y, 254);       
      led1IsDown=true;
    }
  
  }
  else if(analogRead(led2FailurePinChecker)>ledFailureTreshold){
    if(led2IsDown==false){
      if(isMonitor)
        nbrOf254Messages+=1;
      else
        sendMessage(cell2_x, cell2_y, 254);

      led2IsDown=true;
    }
  }
  else if((analogRead(led1FailurePinChecker) <ledFailureTreshold && led1IsDown==true)){
    if(isMonitor)
      nbrOf255Messages+=1;
    else
      sendMessage(cell1_x,cell1_y,255);

    led1IsDown =false;
  }
  else if(analogRead(led2FailurePinChecker)< ledFailureTreshold && led2IsDown==true){
    if(isMonitor)
        nbrOf255Messages+=1;
      else
        sendMessage(cell2_x,cell2_y,255);

      led2IsDown =false;
  }

}


void startCounter(int ledId){
  if(ledId==1)
    t1=getTimeNow();
  else
    t2=getTimeNow();
}

void checkCounter(int ledId){
  if(ledId==1){
    if((getTimeNow()-t1)>10000){ //means that the led should return to the previous state
      t1=0;
      adjustLeds("noMotion"); 
    }
  }
  else{
    if((getTimeNow()-t2)>10000){ //means that the led should return to the previous state
      t2=0;
      adjustLeds("noMotion");   
    }
  }
}

/*
 * receives the potentiometer value and checks if it has changed since the last measure
 * returns true if potentiometer's value has changed
*/
boolean checkPotentiometerValue(int potId){
  if(potId==1){
    if(readPotentiometer(1) != potentiometer1Value){
      potentiometer1Value=readPotentiometer(1); //atualizar o valor do potenciometro 1
      return true;    
    }
  }
  else{
    if(readPotentiometer(2) != potentiometer2Value){
      potentiometer2Value=readPotentiometer(2); //atualizar o valor do potenciometro 2
      return true; 
    }
  }
  return false; 
}


int readPotentiometer(int potId){
  if(potId==1)
    return digitalRead(pot1); 
  else
    return digitalRead(pot2); 
  
}

void sendMessage(int srcX,int srcY,int event){
  Serial.println("Sending Message");
  int bit = (srcX << 4) + srcY;

  message[0] = bit;  // 8-bits
  message[1] = event;  //8-bits

  if(event!=255 && event!=254)
    neighbors(srcX,srcY,"all");
  else {
    neighbors_addresses.push_back(calculateAddress(0,0));
    destiny_cells.push_back(((0) << 4) + (0));

  }
  
  for (int i=0;i<neighbors_addresses.size();i++){

    unsigned long timestamp = getTimeNow();  
    message[2] = timestamp >> 24; // timestamp is splitted
    message[3] = timestamp >> 16; // into 4 parts
    message[4] = timestamp >> 8;  // of 8-bits each
    message[5] = timestamp;

  /*
   Serial.println("Sending message..");
    Serial.print("address : ");
    Serial.println(neighbors_addresses[i]);
    byte dest = destiny_cells[i];
    byte destX = dest >> 4;
    byte aux2 = dest << 4;
    byte destY = aux2 >> 4;
    Serial.print("destiny : ");
    Serial.println(String(destX)+String(destY));
*/
    message[6] = destiny_cells[i];

    Wire.beginTransmission(neighbors_addresses[i]); // transmit to device #8
    Wire.write(message,7);
    Wire.endTransmission();    // stop transmitting
  }
  //clear all arrays
  neighbors_addresses.clear();
  destiny_cells.clear();
}

void neighbors(int x, int y, String howMany){

  int N;

  if(howMany=="all"){
    N=15;    
  }
  else{
    N=7;
  }

  int neig_1 [N+1];
  if(howMany=="all"){
    neig_1[0] = -1;neig_1[1] = 1;neig_1[2] = 0;neig_1[3] = 1;neig_1[4] = 1;neig_1[5] = 1;neig_1[6] = -1;neig_1[7] = 0;
    neig_1[8] = -1;neig_1[9] = -1;neig_1[10] = 0;neig_1[11] = -1;neig_1[12] = 1;neig_1[13] = -1;neig_1[14] = 1;neig_1[15] = 0;
  }

  else{
    neig_1[0] = -1;neig_1[1] = 0;neig_1[2] = 1;neig_1[3] = 0;neig_1[4] = 0;neig_1[5] = 1;neig_1[6] = 0;neig_1[7] = -1;
  }

  QList<int> neig;
  for (int i=0;i<N;i+=2){

    if( (x+neig_1[i])>=0 && (y+neig_1[i+1])>=0){ 
       if( ((String(x+neig_1[i])+String(y+neig_1[i+1]))!= (String(cell1_x)+String(cell1_y))) && 
          ((String(x+neig_1[i])+String(y+neig_1[i+1]))!= (String(cell2_x)+String(cell2_y))) ) {
        neig.push_back(neig_1[i]);
        neig.push_back(neig_1[i+1]);
      }
    }
  }

  // add neighbor's addresses
  for (int i=0;i<neig.size()-1;i+=2){
    neighbors_addresses.push_back(calculateAddress(x+neig.at(i), y+neig.at(i+1)));
  }

  // add neighbor cells
  for (int i=0;i<neig.size()-1;i+=2){
    destiny_cells.push_back(((x+neig.at(i)) << 4) + (y+neig.at(i+1)));
  }
}

void sendOneMessage(int srcX, int srcY,int dstX,int dstY,int event){
  int bit = (srcX << 4) + srcY;

  message[0] = bit;  // 8-bits
  message[1] = event;  //8-bits

  unsigned long timestamp = getTimeNow();  
  message[2] = timestamp >> 24; 
  message[3] = timestamp >> 16; 
  message[4] = timestamp >> 8;  
  message[5] = timestamp;

/*
 Serial.println("Sending message..");
  Serial.print("address : ");
  Serial.println(neighbors_addresses[i]);
  byte dest = destiny_cells[i];
  byte destX = dest >> 4;
  byte aux2 = dest << 4;
  byte destY = aux2 >> 4;
  Serial.print("destiny : ");
  Serial.println(String(destX)+String(destY));
*/
  message[6] = (dstX << 4) + dstY;


  Wire.beginTransmission(calculateAddress(dstX,dstY)); // transmit to device #8
  Wire.write(message,7);
  Wire.endTransmission();    // stop transmitting
}


void sendClock(int srcX, int srcY){

  int bit = (srcX << 4) + srcY;
  message[0] = bit;  // 8-bits
  message[1] = 0;  //8-bits event

  for (int i=0;i<neighbors_addresses.size();i++){

    unsigned long timestamp = getTimeNow();  

    message[2] = timestamp >> 24; 
    message[3] = timestamp >> 16; 
    message[4] = timestamp >> 8;  
    message[5] = timestamp;
    message[6] = destiny_cells[i];

    Wire.beginTransmission(neighbors_addresses[i]); // transmit to device #8
    Wire.write(message,7);
    Wire.endTransmission();    // stop transmitting
  }
  
}


int calculateAddress(int x, int y){
    return ((int)(x/2))*16+y;
}
