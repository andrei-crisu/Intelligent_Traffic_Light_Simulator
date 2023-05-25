

/* Author:  < Andrei Crisu >
* 
* This project represent the software part
* for the hardware implementation with Arduino Uno of a 
* traffic light
*
*
*/

//pins definition
#define GREEN_PIN (2)
#define YELLOW_PIN (3)
#define RED_PIN (4)

//define system states
#define OFF_STATE (0)
#define AUTO_STATE (1)
#define MANUAL_STATE (2)

//define color states
#define GREEN_COLOR_STATE (1)
#define YELLOW_COLOR_STATE (2)
#define RED_COLOR_STATE (3)

//other used constants
//definition for the second = 1000 ms
#define TIME_T_SEC (1000)

// current system state variable
unsigned char currentState=OFF_STATE;

// current color state variable
unsigned char currentColorState= GREEN_COLOR_STATE;

//curent time value 
//this variable is used in auto mode for timing 
unsigned char currentTimeValue=0;

//variables used to store the time ( in seconds)
//used for each color in auto mode
unsigned char redStateTime=8;
unsigned char yellowStateTime=2;
unsigned char greenStateTime=10;

void greenColorMode();
void yellowColorMode();
void redColorMode();
void offColorMode();

void trafficLightLogic(unsigned char colorState);
void autoModeFunctionality();


void setup() {
  // put your setup code here, to run once:
  //set the traffic light color pins as outputs
  digitalWrite(GREEN_PIN, HIGH);
  digitalWrite(YELLOW_PIN, HIGH);
  digitalWrite(RED_PIN, HIGH);

  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  switch(currentState)
  {
    case AUTO_STATE:
      autoModeFunctionality();
    break;

    case MANUAL_STATE:
      trafficLightLogic(currentColorState);
    break;

    case OFF_STATE:
    offState();
    break;

    default:
    offState();
    break;
  }

}



//function that implement 
//the color logic for LEDs
//based on  the current colorState
void trafficLightLogic(unsigned char colorState)
{
    switch(colorState)
    {
      case GREEN_COLOR_STATE:
      greenColorMode();
      break;

      case YELLOW_COLOR_STATE:
      yellowColorMode();
      break;

      case RED_COLOR_STATE:
      redColorMode();
      break;

      default:
      break;
    }
}

 //only the green LED is ON 
void greenColorMode()
{
    digitalWrite(GREEN_PIN, HIGH);
    digitalWrite(YELLOW_PIN, LOW);
    digitalWrite(RED_PIN, LOW);
}

 //only the yellow LED is ON 
void yellowColorMode()
{ 
    digitalWrite(GREEN_PIN, LOW);
    digitalWrite(YELLOW_PIN, HIGH);
    digitalWrite(RED_PIN, LOW);

}

 //only the red LED is ON 
void redColorMode()
{
    digitalWrite(GREEN_PIN, LOW);
    digitalWrite(YELLOW_PIN, LOW);
    digitalWrite(RED_PIN, HIGH);
}

 //all LEDs are OFF
void offState()
{
    digitalWrite(GREEN_PIN, LOW);
    digitalWrite(YELLOW_PIN, LOW);
    digitalWrite(RED_PIN, LOW);
}

void autoModeFunctionality()
{
    currentTimeValue=millis();
    
     switch(colorState)
    {
      case GREEN_COLOR_STATE:
      greenColorMode();
      break;

      case YELLOW_COLOR_STATE:
      yellowColorMode();
      break;

      case RED_COLOR_STATE:
      redColorMode();
      break;

      default:
      break;
    }

}


