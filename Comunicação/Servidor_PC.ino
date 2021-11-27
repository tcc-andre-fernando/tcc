#include "Motor.h"
//Variáveis referentes ao motor-----------------------------------------------------------------
  Motor motores(7,8, 11);
  String tensao;

//Variáveis referentes ao encoder---------------------------------------------------------------
  float counter = 1800;
  float last_counter;
  int state, lastState = 0;
  float counter_speed = 0;
  float counter2, last_counter2;
  int state2, lastState2 = 0;
  float counter_speed2 = 0;
  
  int changeMatrix [][4] = {{ 0,-1, 1, 0},
                            { 1, 0, 0,-1},
                            {-1, 0, 0, 1},
                            { 0, 1,-1, 0}};

  int changeMatrix2 [][4] = {{ 0,-1, 1, 0},
                            { 1, 0, 0,-1},
                            {-1, 0, 0, 1},
                            { 0, 1,-1, 0}};
                            
  #define A 3
  #define B 2
  #define C 5
  #define D 4 

//Variáveis referentes ao loop------------------------------------------------------------------
  int timer1_counter;
  const char terminator = '_';
  
//Interrupt (encoder)---------------------------------------------------------------------------
  void pciSetup(byte pin){
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
  }

  ISR (PCINT2_vect){ // handle pin change interrupt for D0 to D7 here

    if(digitalRead(6) == LOW){
      counter = 1800;
    }

    state = 0;
    state += digitalRead(A);
    state += 2*digitalRead(B);

    state2 = 0;
    state2 += digitalRead(C);
    state2 += 2*digitalRead(D);

    if(state != lastState){
      counter += changeMatrix[state][lastState];
      lastState = state;
    }

    if(counter>2400){
      counter -= 2400;
    }

    if (counter < 0){
      counter += 2400;
    }
    if(state2 != lastState2){
      counter2 += changeMatrix2[state2][lastState2]*0.4;
      lastState2 = state2;
    }
    if(counter>1793 and counter<1804){
      digitalWrite(A0, HIGH);     
    }else{
      digitalWrite(A0, LOW);     
    }
  }

//Interrupt(timer)------------------------------------------------------------------------------
  ISR(TIMER1_OVF_vect){
    TCNT1 = timer1_counter;

    counter_speed = counter - last_counter;
    last_counter  = counter;

    counter_speed2 = counter2 - last_counter2;
    last_counter2  = counter2;
  }

void setup(){
  Serial.begin(2000000);
  pinMode(A, INPUT_PULLUP); 
  pinMode(B, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(A0, OUTPUT); 

  pciSetup(A);                                //Inicialização encoder
  pciSetup(B);
  pciSetup(C);
  pciSetup(D);

   noInterrupts();           // disable all interrupts
   TCCR1A = 0;
   TCCR1B = 0;
   timer1_counter = 62411;   // preload timer 65536-16MHz/256/xHz (34286 for 0.5sec) (59286 for 0.1sec) (64286 for 0.02sec)

  
   TCNT1 = timer1_counter;   // preload timer
   TCCR1B |= (1 << CS12);    // 256 prescaler 
   TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
   
   interrupts();             // enable all interrupts
}


void loop(){
  //counter, counter_speed: Pêndulo
  //counter2, counter_speed2: Motor
  
  if (Serial.available() > 0){
      tensao = Serial.readStringUntil(terminator);
      motores.go(tensao.toInt());
      Serial.print(counter);
      Serial.print(';');
      Serial.print(counter_speed);
      Serial.print(';');
      Serial.print(counter2);
      Serial.print(';');
      Serial.println(counter_speed2);
  }
}
