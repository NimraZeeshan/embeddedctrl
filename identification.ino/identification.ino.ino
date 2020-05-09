// System Identification of DC Motor
//www.github.com/brilianputraa

//Parts:
//L298N H-Bridge Module
//25GA370 1380 RPM DC Motor With Rotary Encoder
//The Encoder raise 15 pulse per RPM

//Methods:
// To get the systems model, I use finite impulse response model (FIR) 
// By inputting PRBS signal into the DC Motor and record the input (PWM Period) and the velocity from encoder
// Finally, export the dataset to csv file

//Resource:
//https://web.stanford.edu/class/archive/ee/ee392m/ee392m.1034/Lecture8_ID.pdf

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     int en =  9; // For PWM signal
int ina = 4; // Logic of Left Side H-Bridge Transistor
int inb = 3; // Logic of Right Side H-Bridge Transistor
int encoder = 2;

void setup(){
  Serial.begin(9600)
  pinMode(en, OUTPUT);
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
}


void loop(){
  analogWrite(en,0);
  digitalWrite(ina, HIGH);
  digitalWrite(inb, HIGH);
}

void encode(){
  attachInterrupt(digitalPinToInterrupt(phaseA), addEncoder, RISING);
}

