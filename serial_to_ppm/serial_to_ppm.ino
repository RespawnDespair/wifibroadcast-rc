// from https://github.com/ckalpha/Generate-PPM-Signal/blob/master/Generate-PPM-Signal.ino

//this programm will put out a PPM signal

//////////////////////CONFIGURATION///////////////////////////////
#define chanel_number 12  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 0  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 10  //set PPM signal output pin on the arduino
//////////////////////////////////////////////////////////////////


/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[chanel_number];
bool ppm_running;

void setup(){ 
  ppm_running = false;
  Serial.begin(115200);
}


void startPPM(){
  if (!ppm_running){
    ppm_running = true;
    
    //initiallize default ppm values
    for(int i=0; i<chanel_number; i++){
      ppm[i]= default_servo_value;
    }
  
    pinMode(sigPin, OUTPUT);
    digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
    
    cli();
    TCCR1A = 0; // set entire TCCR1 register to 0
    TCCR1B = 0;
    
    OCR1A = 100;  // compare match register, change this
    TCCR1B |= (1 << WGM12);  // turn on CTC mode
    TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    sei();
  }
}

char cmd[16];
char param0[16];
char param1[16];

void appendToString(char* toString, char chr, int maxchars) {
  for (int i = 0; i < maxchars; i++)
  {
    if (toString[i] == 0){
      toString[i] = chr;
      break;
    }
  }
}

void readLineFromSerial() {
  memset(cmd, 0, sizeof(cmd));
  memset(param0, 0, sizeof(param0));
  memset(param1, 0, sizeof(param1));

  int iParamId = -1;
  
  while(true) {
    if (Serial.available() > 0) {
      auto bytRead = Serial.read();
      //Serial.write(bytRead);
      //String tmp;
      //tmp = "Character read: " + (int)bytRead;
      //Serial.println(tmp);

      if (bytRead == ':' && iParamId == -1 || bytRead == ',' && iParamId == 0)
        iParamId++;
      else if (bytRead == '\n' || bytRead == '\r')
        return;
      else {
        char* target = cmd;
        if (iParamId == 0)
          target = param0;
        else if (iParamId == 1)
          target = param1;

        appendToString(target, bytRead, 15);      
      }
    }
  }
}

void loop(){
  // TEST LOOPs
  /*
  if (Serial.available() > 0) {
    auto bytRead = Serial.read();
    Serial.write(bytRead);
  }

  return;
  */

  /*
  static int val = 1;
  
  ppm[0] = ppm[0] + val;
  if(ppm[0] >= 2000){ val = -1; }
  if(ppm[0] <= 1000){ val = 1; }
  delay(10);
  */
  
  // REAL LOOP
  while(true) {
    readLineFromSerial();
    if (0 == strcmp(cmd, "ppm")) {
      int iChannel = atoi(param0);
      int iValue = atoi(param1);

      if (iChannel >= 0 && iChannel < chanel_number && iValue >= 800 && iValue <= 2200) {
        startPPM();
        Serial.println("Valid PPM cmd received");
        ppm[iChannel] = iValue;
      }
    }
  }
}

ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= chanel_number){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
