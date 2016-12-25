/*

Laser Pointer tracking gun
by Anum Sheraz

 */

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
const int up =  7;
const int down =  6;
const int left =  4;
const int right =  5;
String x;
String y;

void setup() {
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  //inputString.reserve(200);
  pinMode(up, OUTPUT);
  pinMode(down, OUTPUT);
  pinMode(left, OUTPUT);
  pinMode(right, OUTPUT);
}

void loop() {
  serialEvent(); //call the function
  // print the string when a newline arrives:
  if (stringComplete) {
     //Serial.println(inputString);
     x=inputString.substring(0,1);
     y=inputString.substring(2,3);
     
      //Serial.print("X is =>");    
      //Serial.print(x);
      //Serial.print("Y is =>");    
      //Serial.print(y);

      move_x(x);     
      move_y(y); 
     
    inputString = "";
    stringComplete = false;
  }
}


void move_x(String x_direction){
  if (x_direction=="U"){
        //Serial.println("moving up");
        digitalWrite(up, LOW);
        digitalWrite(down, HIGH);
        delay(2);
        digitalWrite(up, LOW);
        digitalWrite(down, LOW);
        delay(2);            
  }else if (x_direction=="D"){
        //Serial.println("moving down");
        digitalWrite(up, HIGH);
        digitalWrite(down, LOW);
        delay(2);
        digitalWrite(up, LOW);
        digitalWrite(down, LOW);
        delay(2);    
  }else if (x_direction=="-"){
        //Serial.println("STOP");
        digitalWrite(up, LOW);
        digitalWrite(down, LOW);      
  }
}

void move_y(String y_direction){
  if (y_direction=="L"){
        //Serial.println("moving up");
        Serial.println("moving Left");
        digitalWrite(left, HIGH);
        digitalWrite(right, LOW);
        delay(20);
        digitalWrite(left, LOW);
        digitalWrite(right, LOW);
        delay(20);            
  }else if (y_direction=="R"){
        Serial.println("moving right");
        digitalWrite(left, LOW);
        digitalWrite(right, HIGH);
        delay(20);
        digitalWrite(left, LOW);
        digitalWrite(right, LOW);
        delay(20);    
  }else if (y_direction=="-"){
        //Serial.println("STOP");
        digitalWrite(left, LOW);
        digitalWrite(right, LOW);      
  }
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    //Serial.println(inChar);
    // add it to the inputString:
    //Serial.println(inChar);
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

