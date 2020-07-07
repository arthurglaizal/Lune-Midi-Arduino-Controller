//----------------------------------- MIDI CONTROLLER WITH CLASSES -------------
//  Hello, to do your MIDI controller if you dont need more than 60 digitals(buttons,encoders and SR04) and 20 analogs(potentiometers) just look at the classes which are:
//  BUTTONplay ( int digiPin, int channel) ------------> button is ON when push and OFF when repush
//  BUTTONtemp ( int digiPin, int channel) ------------> button is ON when push and OFF after 15ms(for drums,...), keep pushing or just push does the same thing
//  BUTTONcue ( int digiPin, int channel)  ------------> button is ON when push and OFF when you stop pushing
//  POTENTIO (int analogPin, int channel )     --------> Potentiometer
//  SR ( int digiPinEcho, int digiPinTrig, int channel ) ------------> IF distance detector, you need to declare pinMode, digitalWrite,  in void setup(){...} as the example for each detector
// Now, in the MAIN, copy past add more potentiometers or buttons and change the value inside the parenthesis as it need. YOU DO NOT NEED TO READ THE ENTIRE CODE. 
// EXAMPLE:  BUTTONplay(8,4); button in mode play (cf classes),  button pin is digital #8, channel of the pc which receive midi is #4

//If you need more buttons or potentiometers read and modify the initialisation part, read the comments.
//-----------------------------------2017 Arthur Glaizal
//________________________________________________________________________________________________________________________________________________________________
 //____________________________INITIALISATION
// You can change the value maximum here if you have more button, potentiometer, etc...
#include <Keypad.h>//___________________PAD


#define _R 7
#define _G 8
#define _B 9

#define L1 10
#define L2 11
#define L3 12
#define L4 13

#define _R2 2
#define _G2 3
#define _B2 4

#define L5 5
#define L6 6
#define L7 45
#define L8 46


struct Color {
  Color(): red(0), green(0), blue(0) {}
  Color(byte r, byte g, byte b): red(r), green(g), blue(b) {}
  byte red;
  byte green;
  byte blue;
};

const uint8_t ROWS=1;
const uint8_t COLS=4;
char keys[ROWS][COLS] = {{1,2,3,4}};
byte rowPins[ROWS] = {26};
byte colPins[COLS] = {22, 23, 24, 25};


const uint8_t ROWS2=1;
const uint8_t COLS2=4;
char keys2[ROWS2][COLS2] = {{1,2,3,4}};
byte rowPins2[ROWS2] = {31};
byte colPins2[COLS2] = {27,28,29,30};


Color led[4];
Color led2[4];

uint8_t button_state[4] = {0,0,0,0};

uint8_t button_state2[4] = {0,0,0,0};

Keypad buttons = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

Keypad buttons2 = Keypad(makeKeymap(keys2), rowPins2, colPins2, ROWS2, COLS2);

Color color_states[] = {

  Color(255, 255  , 255 ),
   Color(0  , 255, 255),
 Color(0  , 0, 255  ),
  Color(0  , 255, 255),
  Color(255, 255  , 255 ),
   Color(255  , 255, 0),
};

uint8_t num_color_states = sizeof(color_states) / sizeof(color_states[0]);

//__________________________________

int currentState[60]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
              // Tab of 60 digitals (buttons,encoder and SR04) maximum, Here we initialise at 0, 
             // WARNING you have to remember, to access currentState of the button number X you have to take the currentState[X-1] same for others table
             // the ONLY change is in in the main you cant put for example BUTTONplay (60,...,..) because the max is BUTTONplay(59,...,...) and the min BUTTONplay(0,...,...)
             
int lastState[60]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  
int currentStateAn[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // Tab of 20 analogs (potentiometer) maximum, Here we initialise at 0, 
int lastStateAn[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
int counter[60]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // 60 counter for 60 play button maximum (if you want 60 play button)
long lecture_echo=0;  // for the distance detector
long cm[60]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // for the distance detector, 60 maximum
long lastcm[60]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
int minsr = 3; // For the SR
int maxsr = 15; // For the SR
int distancesecure = 10; // For the SR
int convdistmidi = 8; // For the SR
int nVal = 0; // For the encoder
 //_______________________________________________________________SETUP
void setup() 
{ 
//-------------------PAD
  pinMode(_R, OUTPUT);
  pinMode(_G, OUTPUT);
  pinMode(_B, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(L3, OUTPUT);
  pinMode(L4, OUTPUT);

  pinMode(_R2, OUTPUT);
  pinMode(_G2, OUTPUT);
  pinMode(_B2, OUTPUT);
  pinMode(L5, OUTPUT);
  pinMode(L6, OUTPUT);
  pinMode(L7, OUTPUT);
  pinMode(L8, OUTPUT);
writeLeds();

led[0] = Color (0, 255  , 255 );
led[1] = Color (255, 255  , 255 );
led[2] = Color (255, 255  , 255 );
led[3] = Color (255, 255   , 0);

writeLeds2();
led2[0] = Color (255 , 255  , 0);
led2[1] = Color (255, 255  , 255 );
led2[2] = Color (255, 255  , 255 );
led2[3] = Color (0, 255   , 255);
//----------------------------------------
  
  pinMode(37, OUTPUT); // TRIG of the distance detector is in digital pin 37
  digitalWrite(37, LOW); // TRIG of the distance detector is in digital pin 37
  pinMode(36, INPUT); // ECHO of the distance detector is in digital pin 36

    pinMode(39, OUTPUT); 
  digitalWrite(39, LOW); 
  pinMode(38, INPUT); 

 pinMode(18, INPUT); // For the ENCODER
  pinMode(19, INPUT);// For the encoder
  attachInterrupt(4, nAnalyse, CHANGE);// For the encoder, Not the same for arduino uno (look on google "attachInterrupt")


  
  Serial.begin(9600); //----- baud rate 9600
}
//___________________________________PROGRAM / MAIN __________________________READ HERE__________________________________________________________________________________

void loop() 
{ 
  POTENTIO(0, 1); //EQ 
  POTENTIO(1, 2);
  POTENTIO(2, 3);
  POTENTIO(3, 4);
  POTENTIO(4, 5);
  POTENTIO(5, 6); 
  

  
  BUTTONplay(32, 7);
  BUTTONcue(33, 8); 
  BUTTONplay(34, 9);
  BUTTONcue(35, 10); 
  
  SR (36, 37, 11); // Look setup()
  SR (38, 39, 12);

 POTENTIO(14, 13); // Linear 
 POTENTIO(13, 14); 
 POTENTIO(12, 15);
 POTENTIO(11, 16);  

  POTENTIO(6, 17); // Effects
  POTENTIO(7, 18);
  POTENTIO(8, 19); 


 writeLeds();
 readButtons();

  writeLeds2();
 readButtons2();
  
 // BUTTONplay(8, 4); // button in mode Play (cf classes),  button pin is digital #8, channel of the pc which receive midi is #4
 // BUTTONcue(9, 5);
  //POTENTIO(0, 6); // Potentiometer (cf classes),  analog pin is digital #0, channel of the pc which receive midi is #6
  //SR (3, 4, 7);  int digiPinEcho, int digiPinTrig, int channel 



  delay(10);
}

//_______________________________________________________________________________________________________________________________________________________________
//--------------------------------         CLASSES

void MIDImessage(byte command, byte data1, byte data2) // --------- NECESSARY, pass values out through standard Midi Command
{
   Serial.write(command);
   Serial.write(data1);
   Serial.write(data2);
}

void BUTTONplay ( int digiPin, int channel) { //-------------------button is ON when push and OFF when repush

  currentState[digiPin]= digitalRead(digiPin); // Interrogation du bouton-poussoir
  
  // La valeur précédente du bouton-poussoir est-elle différente de la valeur actuelle ?
  // if it is, the buttonState is HIGH:
  if (currentState[digiPin] != lastState[digiPin]) { 
    if(currentState[digiPin]  == HIGH){
      counter[digiPin]++; // Incrémentation du compteur(+1)
    }
    lastState[digiPin]  = currentState[digiPin] ; // Sauvegarde de la valeur actuelle du bouton-poussoir
   if(counter[digiPin]%2 == 0)                 // La variable du compteur est-elle un nombre pair ?
      MIDImessage(176,channel,1);
   else
      MIDImessage(176,channel,0);
   }
}


void BUTTONtemp (int digiPin, int channel) { //-----------------------button is ON when push and OFF after 15ms(for drums,...), keep pushing or just push does the same thing

  currentState[digiPin] = digitalRead(digiPin); // Interrogation du bouton-poussoir
  
  if (currentState[digiPin] == LOW && lastState[digiPin] == HIGH) { 
      
     MIDImessage(176,channel,0); 
  
  }
  else if (currentState[digiPin] == HIGH && lastState[digiPin] == LOW) { 
           MIDImessage(176,channel,1); 
            delay(15);
           MIDImessage(176,channel,0); 
   }
  lastState[digiPin]  = currentState[digiPin] ;      
}

void BUTTONcue ( int digiPin, int channel) { //------------------------button is ON when push and OFF when you stop pushing, you will maybe have to invert HIGH and LOW after test
  currentState[digiPin] = digitalRead(digiPin);  // read input value
  if (currentState[digiPin] != lastState[digiPin]) { 
    if(currentState[digiPin]  == HIGH){
      MIDImessage(176,channel,0); 
    }
    
   lastState[digiPin]  = currentState[digiPin] ; // Sauvegarde de la valeur actuelle du bouton-poussoir
   if(currentState[digiPin]  == LOW)  {
      MIDImessage(176,channel,1); 
    }               
    
  }
 }

void POTENTIO ( int analogPin, int channel ) { //------------------------------------------Potentio
    currentStateAn[analogPin] = analogRead(analogPin)/8;   // Divide by 8 to get range of 0-127 for midi
   if (currentStateAn[analogPin] != lastStateAn[analogPin]) // If the value does not = the last value the following command is made. This is because the pot has been turned. Otherwise the pot remains the same and no midi message is output.
  // if ((currentStateAn[analogPin]-lastStateAn[analogPin]>1) || (currentStateAn[analogPin]-lastStateAn[analogPin]<(-1)) ) 
   {
   MIDImessage(176,channel,currentStateAn[analogPin]);}         // 176 = CC command (channel 1 control change), 1 = Which Control, val = value read from Potentionmeter 1 NOTE THIS SAYS VAL not VA1 (lowercase of course)
   lastStateAn[analogPin] = currentStateAn[analogPin];
}



void SR ( int digiPinEcho, int digiPinTrig, int channel ) { //---------------------------------------------IF distance detector
  digitalWrite(digiPinTrig, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(digiPinTrig, LOW); 
  lecture_echo  = pulseIn(digiPinEcho, HIGH); 
  cm[digiPinEcho] = lecture_echo / 58; //  [digiPinEcho] just to give a pointer to a different case of the table for each detector
  if ((cm[digiPinEcho]<maxsr+distancesecure)||(lastcm[digiPinEcho]<maxsr+distancesecure)){  //if its not two times more than max+dist.sec (to avoid sending 0 all the time)
   if (cm[digiPinEcho]!=  lastcm[digiPinEcho])  {                              //if the value change
     if ((minsr<cm[digiPinEcho] ) &&(cm[digiPinEcho] <maxsr+distancesecure)) { // if the distance is between min and max+dist.sec
       if ((cm[digiPinEcho] - minsr)>maxsr)                                   // if the distance is between max and max+dist.sec -> 127
       MIDImessage(176,3,127);      
        else                                                                 // otherwise (the distance is between min and max) -> value
        MIDImessage(176,channel,(cm[digiPinEcho] -minsr)*convdistmidi);    
       
        }
     else {
          MIDImessage(176,channel,0);                                      // if the distance is not between min and max+dist.sec -> 0
    }
   }
    lastcm[digiPinEcho]=cm[digiPinEcho];
  }
}


void nAnalyse(){ //--------ENCODER, look setup
  if(digitalRead(18) == digitalRead(19)){
    nVal++;
  }else{
    nVal--;
  }
  if((1<nVal)&&(nVal<255))  MIDImessage(176,50,nVal); 

} 

//--------------------------------------- THE 2 PADS
void readButtons() {
  char key = buttons.getKey();
  if (key && key <= 4) {
    button_state[key-1] = (button_state[key-1] + 1) % num_color_states;
    led[key-1] = color_states[button_state[key-1]];
   
   MIDImessage(176,(25+key),1); //----MIDI MESSAGE KEYPAD
  }
}

void readButtons2() {
  char key = buttons2.getKey();
  if (key && key <= 4) {
    button_state2[key-1] = (button_state2[key-1] + 1) % num_color_states;
    led2[key-1] = color_states[button_state2[key-1]];
   
   MIDImessage(176,(30+key),1); //-----MIDI MESSAGE KEYPAD
  }
}

void writeLeds() {
  digitalWrite(_R, LOW);
  digitalWrite(_G, LOW);
  digitalWrite(_B, LOW);
  analogWrite(L1, 255-led[0].red);
  analogWrite(L2, 255-led[1].red);
  analogWrite(L3, 255-led[2].red);
  analogWrite(L4, 255-led[3].red);
  digitalWrite(_R, HIGH);
  // reds are on now, wait a bit
  delayMicroseconds(10);
  digitalWrite(_R, LOW);
  analogWrite(L1, 255-led[0].green);
  analogWrite(L2, 255-led[1].green);
  analogWrite(L3, 255-led[2].green);
  analogWrite(L4, 255-led[3].green);
  digitalWrite(_G, HIGH);
  // green are on now, wait a bit
  delayMicroseconds(10);
  digitalWrite(_G, LOW);
  analogWrite(L1, 255-led[0].blue);
  analogWrite(L2, 255-led[1].blue);
  analogWrite(L3, 255-led[2].blue);
  analogWrite(L4, 255-led[3].blue);
  digitalWrite(_B, HIGH);
  // blue are on now, wait a bit
  delayMicroseconds(10);
  digitalWrite(_B, LOW);
}


void writeLeds2() {
  digitalWrite(_R2, LOW);
  digitalWrite(_G2, LOW);
  digitalWrite(_B2, LOW);
  analogWrite(L5, 255-led2[0].red);
  analogWrite(L6, 255-led2[1].red);
  analogWrite(L7, 255-led2[2].red);
  analogWrite(L8, 255-led2[3].red);
  digitalWrite(_R2, HIGH);
  // reds are on now, wait a bit
  delayMicroseconds(10);
  digitalWrite(_R2, LOW);
  analogWrite(L5, 255-led2[0].green);
  analogWrite(L6, 255-led2[1].green);
  analogWrite(L7, 255-led2[2].green);
  analogWrite(L8, 255-led2[3].green);
  digitalWrite(_G2, HIGH);
  // green are on now, wait a bit
  delayMicroseconds(10);
  digitalWrite(_G2, LOW);
  analogWrite(L5, 255-led2[0].blue);
  analogWrite(L6, 255-led2[1].blue);
  analogWrite(L7, 255-led2[2].blue);
  analogWrite(L8, 255-led2[3].blue);
  digitalWrite(_B2, HIGH);
  // blue are on now, wait a bit
  delayMicroseconds(10);
  digitalWrite(_B2, LOW);
}



