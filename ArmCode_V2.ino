

#include <Filters.h>
#include <Metro.h>

String inputString = ""; 
String inputString2 = ""; // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

double lim = 90.0;  //MAX REACH LIMIT TO PREVENT INJURY


//DISPLAY STUFF
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 3
Adafruit_SSD1306 display(OLED_RESET);
#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };
#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
//END OF DISPLAY STUFF

int motPin1 = 7;
int motPin2 = 5;
int motSpeedPin = 10;

boolean led = true; int ledPin = 14; int swPin = 15; double inertia = 0; int potPin = 7; int serP = 1;
int I = 1; double pos = 1; double posPrev = 0; double omega = 0; double omegaPrev = 0; double alpha = 0; 
double alphaPrev = 0; double pEst = 0; double omegaEst = 0; double error = 0; double errorW = 0;
int errCount = 0; double rea = 0; double motorSet = 0; double motorSet2 = 0; double del = 0;
double diff = 0; double phi = 0.8; double linpos = 0; double linspeed = 0; double linaccel = 0;
double linspeedPrev = 0; double linEst = 0; double linspeedEst = 0; int ticker = 0;

int interv = 50;

// filters out changes faster that 5 Hz.
float filterFrequency = 50.0 ;
FilterOnePole lowpassFilter( LOWPASS, filterFrequency );
FilterOnePole lowpassFilter2( LOWPASS, filterFrequency );
FilterOnePole lowpassFilter3( LOWPASS, filterFrequency );

Metro metro = Metro(interv);

void setup() {

    Serial.begin(9600);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    Serial.println("OPERATIONAL");
    display.print("DEVICE IS OPERATIONAL");
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.display();
    delay(2000);
    display.clearDisplay();

    analogReadResolution(12);
    analogReference(INTERNAL);
    analogReadAveraging(4);
    
    pinMode(ledPin,OUTPUT);
    pinMode(13, OUTPUT);
    pinMode(swPin, INPUT_PULLDOWN);
    inputString.reserve(200);

    pinMode(motPin1, OUTPUT);
    pinMode(motPin2, OUTPUT);
    pinMode(motSpeedPin, OUTPUT);

}

void loop() {
  if(metro.check() == 1){

    if(ticker%400 == 0){
      led = !led;
      digitalWrite(ledPin,led);
      digitalWrite(13,!led);
      ticker = 1;
    }
    ticker++;
    
    pos = lowpassFilter.input( analogRead(potPin));
    pos = phi * pos + (1-phi) * posPrev;
    //Calculate innertia 
    linpos = lin(pos);
    
    omega = (pos-posPrev)/(interv*pow(10,-3));
    omega = lowpassFilter2.input(omega);
    omega = phi * omega + (1-phi) * omegaPrev;
    
    linspeed = linDer(pos, omega);

    if(abs(omega) < 200.0){
      omega = 0;
    }

    alpha = (omega-omegaPrev)/(interv*pow(10,-3));
    alpha = lowpassFilter3.input(alpha);
    alpha = phi * alpha + (1-phi) * alphaPrev;

    linaccel = (linspeed-linspeedPrev)/(interv*pow(10,-3));
    
    if(alpha < 800.0){
      alpha = 0;
    }

    pEst = posPrev + omegaPrev*(interv*pow(10,-3));
    linEst = lin(pEst);
    omegaEst = omegaPrev + alphaPrev*(interv*pow(10,-3));
    linspeedEst = linDer(pos, omegaEst);
    error = (pos-pEst)/pos;
    errorW = (linspeed-linspeedEst)/linspeed;
    
    if (isnan(errorW) || isinf(errorW)) {
      errorW = 0;
    }

    motorSet2 += errorW; 
    //check switch to see if interupt is needed
    if(digitalRead(swPin) == 1)
    {
      hold2();
    }
  
    if(pos < 1000.0){
      motorStop();
    }
    else if(omega>0){
      direcSet(HIGH);
      analogWrite(motSpeedPin, abs(motorSet2));
    }
    else{
      direcSet(LOW);
      analogWrite(motSpeedPin, abs(motorSet2));
    }
    
    motorSet2 += errorW;
    
    if(ticker%3== 0) //DON'T PRINT SERIAL DATA CONSTANTLY TO PREVENT OVERLOADING THE BUFFER
    {
      //Serial.print();
     // Serial.print(", ");
      //Serial.print(millis());
      Serial.print(", ");
      Serial.println(motorSet2);
     // Serial.print(", ");
      //Serial.print(errorW);
     // Serial.println("");     
    }


    diff = 0;
    posPrev = pos;
    pos = 0;
    omegaPrev = omega; 
    alphaPrev = alpha;
    linspeedPrev = linspeed;
    delay(del);  
  }
}

void serialEvent() {
  inputString = "";
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      Serial.println("command detected");
      Serial.print("recieved message >> ");
      Serial.print(inputString);
      Serial.println();


      if(inputString.startsWith(String("hold")))
      {
        Serial.print("command recognized, holding");
        hold1();
      }
      
      if(inputString.startsWith(String("frequency")) > 0)
      {
        noiseAdj(inputString);
      }

      if(inputString.startsWith(String("sensativity")) > 0)
      {
        senAdj(inputString);
      }
      if(inputString.startsWith(String("zeroError")) > 0)
      {
        zeroError();
      }
      delay(700);
      stringComplete = true;

    }

  }

}







//USED FUNCTIONS...

void senAdj(String adj)
{
  adj.remove(adj.indexOf('-'));
  phi = double(adj.toInt())/100.0;
  Serial.print("smoothing adjusted to ");
  Serial.println(phi);
}

void noiseAdj(String noiseAdj)
{
  noiseAdj.remove(noiseAdj.indexOf('-'));
  filterFrequency = double(noiseAdj.toInt())/10.0;
  Serial.print("filtering adjusted to ");
  Serial.println(filterFrequency);
}

void zeroError()
{
  Serial.print("Zeroing error");
  motorSet = 0;
}




void hold1()
{
  Serial.print("command recognized, holding");
  while(true)
  {
    char inChar = (char)Serial.read();
    Serial.print(inputString2);
    inputString2 += inChar;
    if(inputString == "break")
    {
      Serial.println("breaking");
      break;
    }

  }
  inputString2 = "";
  
}

void hold2()
{
  while(true){
    digitalWrite(ledPin,HIGH);
    Serial.println("HOLDING");
    displayP("HOLDING",100);
    motorStop();

    
    if(digitalRead(swPin) == 0)
    {
      displayP("BREAKING",1000);
      Serial.println("break");
      break;
    }
    
    
  }

 
}

void displayP(String text, int dela){
//    fillScreen(1);
    display.setCursor(10,0);
    display.print(text);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.display();
    delay(dela);
    display.clearDisplay();
  
}


void displayPNC(String text){
    display.setCursor(10,0);
    display.print(text);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.display();
  
}

void direcSet(bool dir){
  dir = dir;

  if(dir){
    digitalWrite(motPin1,HIGH);
    digitalWrite(motPin2,LOW);
  }
  else{
    digitalWrite(motPin1,LOW);
    digitalWrite(motPin2,HIGH);
  }
}

void motorStop(){
    digitalWrite(motPin1,LOW);
    digitalWrite(motPin2,LOW);
  
}

double linDer(double theta,double dtheta){
  return (-10307.5*sin(theta))/(sqrt(20615*cos(theta)+28447.3));
}

double lin(double theta){
  return sqrt(pow((155+cos(theta)*66.5),2) + pow((66.5*sin(theta)),2));
}
