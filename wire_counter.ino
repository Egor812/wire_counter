// Arduino Nano
// ATMega328P old bootloader

#include <Arduino.h>
#include <TM1638lite.h>
#include <EEPROM.h>  

TM1638lite tm(4, 7, 8);

const byte sectorAddr = 0; // float = 4bytes

unsigned long time_probe_100ms_1;
unsigned long time_probe_100ms_2;

class Sensor
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
Sensor sensor;

class Counter
{
  public:
  float lenght;

  Counter()
  {
    this->lenght=0;
  }

  void reset()
  {
    this->lenght=0;
  }

  void add(float val)
  {
    this->lenght += val;
  }
};
Counter counter;

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
  // put your setup code here, to run once:
  Serial.begin(9600);
  sensor.init();
  tm.reset();
  leds.state=1;
  leds.set();
  display.writeFloat(sensor.sector);
  delay(250);
  display.writeFloat(0);
  //tm.displayText("Eh");
  //tm.setLED(0, 1);
// delay(100);

}

void loop() {
  uint8_t buttons;
  
    time_probe_100ms_1=millis();
    if(time_probe_100ms_1-time_probe_100ms_2>100){
      time_probe_100ms_2=time_probe_100ms_1;
      buttons = tm.readButtons();
      if( bitRead(buttons,0) ) {
        counter.reset();
        leds.state=1;
        display.writeFloat(0);        
      }
      if( bitRead(buttons,6) && bitRead(buttons,7) ) settings();
      
    }
    if( sensor.askSector() ) {
      counter.add( sensor.sector );
      leds.move();
      /*boozer->on();
      delay(1);
      boozer->off();*/
      display.writeFloat(counter.lenght);
    }
}

void settings()
{
  int sectors=0;
  byte buttons;
  display.writeText("SET1 RSE");
  leds.state=1;
  leds.set();
  delay(500);
  while(true){
    //Buttons
    time_probe_100ms_1=millis();
    if(time_probe_100ms_1-time_probe_100ms_2>100){
      time_probe_100ms_2=time_probe_100ms_1;
      buttons = tm.readButtons();    
      if( bitRead(buttons,5) ) {
        sectors=0;
        leds.state=255;
        leds.set();  
        delay(100);
        leds.state=1;
        leds.set();  
      }
      if( bitRead(buttons,6) ) {
        sensor.recalc(sectors);
        display.writeFloat(sensor.sector);
        sensor.save();
        delay(500);
        display.writeText("ACCEPtEd");
        delay(500);
        leds.state=1;
        display.writeFloat(0);
          return;        
      }
      if( bitRead(buttons,7) ) {
        leds.state=1;
        display.writeFloat(0);
          return;
      }
    }
    //Sensor
    if( sensor.askSector() ) {
      sectors++;
      Serial.println(sectors);
      leds.move();
    }
    
  }

}
