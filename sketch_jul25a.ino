// Arduino Nano
// ATMega328P old bootloader

#include <Arduino.h>
#include <TM1638lite.h>
#include <EEPROM.h>  

TM1638lite tm(4, 7, 8); // STB-D4 CLK-D7 DIO-D8

const byte meterAddr = 0; // long = 4bytes

unsigned long time_probe_100ms_1;
unsigned long time_probe_100ms_2;

class Encoder
{
  public:
    volatile long last;
    volatile long counter = 0;
    static Encoder * instance_;
    //byte sectorAddr;

  public:

  static void ai0() 
  {
    Encoder::instance_->handleAi0();
  }
   
  static void ai1() 
  {
    Encoder::instance_->handleAi1();
  }

  void handleAi0(){
    // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
    // Check pin 3 to determine the direction
    if(digitalRead(3)==LOW) {
      counter++;
    }else{
      counter--;
    }
  }


  void handleAi1() 
  {
    // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
    // Check with pin 2 to determine the direction
    if(digitalRead(2)==LOW) {
      counter--;
    }else{
      counter++;
    }
  }
  
  
  Encoder()
  {
    Encoder::instance_ = this;
    pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 
    pinMode(3, INPUT_PULLUP); // internal pullup input pin 3
    //Setting up interrupt
    //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
    attachInterrupt(0, ai0, RISING);
    //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
    attachInterrupt(1, ai1, RISING);   
    //this->sectorAddr = sectorAddr;
  }

  bool isChange()
  {
    if( this->counter != this->last ){
      this->last = this->counter;
      return true;
    }    
    return false;
  }

  void reset()
  {
    this->counter=0;
  }


};
Encoder * Encoder::instance_;
Encoder encoder;


/*class Sensor
{
  public: 
    float sector; // длина сектора
    //float d = 50.4;
  private:
    int holes = 18;
    int pin = 2;
    //float c; // длина окружности
    bool cur=0;
    bool now=0;
    byte sectorAddr;

private:
  bool ask()
  {
    return digitalRead(this->pin);
  }

  bool askSeries(byte samples)
  {
    int sum=0;
    for( byte c=0; c<samples; c++){
      sum += this->ask();
    }
    if(sum==samples) {
        return 1;
    }
    return 0;
  }   
    

public:
 Sensor()
 {
  pinMode( this->pin, INPUT);
  //this->c = 2*PI*this->d;
  //this->sector = this->c/this->holes;
  this->sectorAddr = sectorAddr;
  
 }

  void init()
  {
    EEPROM.get(this->sectorAddr, this->sector);
  }

  void save()
  {
    EEPROM.put(this->sectorAddr, this->sector);
  }

  void recalc(int sectors)
  {
    this->sector = 1/(float)sectors;
    Serial.print("s:");
    Serial.println(this->sector);
  }

// сектор между 1->0 ... 1->0
// когда значение меняется с 1 на 0 - засчитываем проход сектора
  bool askSector()
  {
    
    now = this->askSeries(10);
    if( cur==1 && now==0 ) {
      cur=0;
      return 1;
    }
    else {
      cur = now;
      return 0;
    }
  }
  
};
Sensor sensor;*/

class Counter
{
  public:
  float lenght;
  long meterValue;
  byte meterAddr;

  Counter(byte meterAddr)
  {
    this->lenght=0;
    this->meterAddr=meterAddr;

  }

  void reset()
  {
    this->lenght=0;
  }

  void set(long val)
  {
    // оборот encoderPoints
    this->lenght = (float) val / this->meterValue ;
  }

  void setMeterValue(long counter)
  {
    this->meterValue = counter;
    Serial.print("s:");
    Serial.println(this->meterValue);
  }  

  void init()
  {
    EEPROM.get(this->meterAddr, this->meterValue);
  }

  void save()
  {
    EEPROM.put(this->meterAddr, this->meterValue);
  }   
};
Counter counter(meterAddr);

/*class Keys
{
  
}
Keys keys;
*/
class Display
{
  private:
    char outstr[8];

public:
  void writeFloat(float f_val)
  {
     dtostrf(f_val,8, 4, this->outstr);
     tm.displayText( this->outstr);
  }

  void writeText(String text)
  {
    tm.displayText(text);
  }

  void writeLong(long val)
  {
    itoa(val, this->outstr, 10);
    tm.displayText( this->outstr);
  }
  
};
Display display;

class Leds
{
  public:
    byte state=0;

  void move()
  {
    byte tmp;
    tmp = bitRead(state, 7);
    this->state = this->state << 1;
    this->state = this->state | tmp;
    this->set();
  }

  void set()
  {
    byte state;
    state = this->state;
    for (uint8_t position = 0; position < 8; position++) {
      tm.setLED(position, state & 1);
      state = state >> 1;
    }    
    
  }
  
};
Leds leds;

void setup() {
  Serial.begin(9600);
  counter.init();
  tm.reset();
  leds.state=1;
  leds.set();
  display.writeLong(counter.meterValue);
  delay(250);
  display.writeFloat(0);
  //tm.displayText("Eh");
  //tm.setLED(0, 1);
// delay(100);

}

void loop() {
  uint8_t buttons;

  //опрос кнопок 10 раз в секунду
  time_probe_100ms_1=millis();
  if(time_probe_100ms_1-time_probe_100ms_2>100){
      time_probe_100ms_2=time_probe_100ms_1;
      buttons = tm.readButtons();
      if( bitRead(buttons,0) ) {  // S1 - reset
        counter.reset();
        encoder.reset();
        leds.state=1;
        display.writeFloat(0);        
      }
      if( bitRead(buttons,6) && bitRead(buttons,7) ) settings(); // S7 + S8 - settings
      
  }
  if( encoder.isChange() ) {
      counter.set( encoder.counter );
      leds.move();
      Serial.print(encoder.counter);
      Serial.print(" - ");
      Serial.println(counter.lenght);
      /*boozer->on();
      delay(1);
      boozer->off();*/
      display.writeFloat(counter.lenght);
  }
}

void settings()
{
  byte buttons;
  display.writeLong(counter.meterValue);
  counter.reset();
  encoder.reset();
  leds.state=1;
  leds.set();
  delay(1000);
  display.writeText("SET1 RSE");
  while(true){
    //Buttons
    time_probe_100ms_1=millis();
    if(time_probe_100ms_1-time_probe_100ms_2>100){
      time_probe_100ms_2=time_probe_100ms_1;
      buttons = tm.readButtons();    
      if( bitRead(buttons,5) ) { // S6 - reset
        counter.reset();
        encoder.reset();
        leds.state=255;
        leds.set();  
        delay(100);
        leds.state=1;
        leds.set();  
      }
      if( bitRead(buttons,6) ) { // S7 - save
        counter.setMeterValue(encoder.counter);
        display.writeLong(counter.meterValue);
        counter.save();
        delay(500);
        display.writeText("ACCEPtEd");
        delay(500);
        leds.state=1;
        display.writeFloat(0);
        counter.reset();
        encoder.reset();
        return;        
      }
      if( bitRead(buttons,7) ) { // S8 - Exit
        leds.state=1;
        display.writeFloat(0);
        counter.reset();
        encoder.reset();
        return;
      }
    }
    //Sensor
    if( encoder.isChange() ) {
      leds.move();
      Serial.println(encoder.counter);
      display.writeLong(encoder.counter);
    }
    
  }

}
