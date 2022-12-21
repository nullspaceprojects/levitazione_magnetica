#include <NullSpaceLib.h>
#include <PID_v1.h>
#include <Adafruit_NeoPixel.h>

//decomment to use the PID Control
//comment to use the Bang-Bang Control
//#define USA_PID

TaskHandle_t TaskMagLev;
TaskHandle_t TaskLeds;


#define SAMPLING_TIME_PID 1 //1=ms //250 //microsec
#define ALPHA_LPF 0.3f

//LEDS
Adafruit_NeoPixel strip = Adafruit_NeoPixel(10, 14, NEO_GRB + NEO_KHZ800); //10 LEDS,
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(10, 12, NEO_GRB + NEO_KHZ800); //10 LEDS,
//colori
#define NUM_COLORI 3
uint32_t color1 = strip.Color(0,0,255);
uint32_t color2 = strip.Color(179,0,255);
uint32_t color3 = strip.Color(255,255,255);
uint32_t colors[NUM_COLORI] = {color1,color2,color3};
int  brightness = 0;
int8_t  brightness_direction = 1;
uint8_t id_colors=0;
int strip_state=0,strip_state_old=0;
int time_in_state=0;
int random_time_ms=5;
int count_rand_flickering=0;
TimerC timer_strip_led;

TimerC timer_print;


//PWM
#define PWM_PID_IN1_ATTRACTION 4
#define PWM_PID_IN2_REPULSION 16
const int PWMFreq = 4882; 
const int PWMChannelIn1Attraction = 0;
const int PWMChannelIn2Repulsion = 1;
const int PWMResolution = 14;
const int PWM_MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1); //=16383

//ULTRASONIC
#define TRIG_PIN 33
#define ECHO_PIN 32
int16_t air_temperature=19.0;

//PID
#define AI_KP_PIN 34
#define AI_KD_PIN 35
double Kp=500.0, Ki=0.0, Kd=0.0;
double pidSetpoint = 950;//1200;//raw
double pidInput, pidOutput;
PID PIDdiPosizione(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, REVERSE);//DIRECT);

//TimerC timer_loop_pid;
TimerMicro timer_loop_pid;
LPFClass lpf_analog_in(ALPHA_LPF);

#define HALL_PIN_AI 39 //GPIO 39 (Analog ADC1_CH6)

bool stato_uscita=false;

void TaskMagLevCode( void * pvParameters ){
  Serial.print("TaskMagLevCode running on core ");
  Serial.println(xPortGetCoreID());

#ifdef USA_PID 
//if (timer_loop_pid.getET()>=SAMPLING_TIME_PID)
  if (timer_loop_pid.getETMillis()>=SAMPLING_TIME_PID)
  {
    timer_loop_pid.reset();
    pidInput = analogRead(HALL_PIN_AI);
    pidInput = lpf_analog_in.compute(pidInput);
    //Kp
    uint16_t kp_raw = analogRead(AI_KP_PIN);
    Kp = mapf(kp_raw,0,4095,10,5000);
    //Kd
    uint16_t kd_raw = analogRead(AI_KD_PIN);
    Ki = mapf(kd_raw,0,4095,0,5000);
    PIDdiPosizione.SetTunings(Kp,Ki,Kd);
    double pos_error= pidSetpoint-pidInput;
    PIDdiPosizione.Compute(pos_error, false); //derivative on error = false --> derivative on input
    int32_t pidOutInt = static_cast<int32_t>(pidOutput);
    //ATTRACTION
    ledcWrite(PWMChannelIn1Attraction, pidOutInt);
    ledcWrite(PWMChannelIn2Repulsion, 0);
    if ( timer_print.getET()>=100)
    {
        timer_print.reset();
        char buffprint[50];
        //sprintf(buffprint, "U:%0.1f Kp:%0.1f Kd:%0.1f",lpf_analog_in.valfilt,Kp,Kd);
        sprintf(buffprint, "H:%0.1f Kp:%0.1f Kd:%0.1f Ki:%0.1f O:%d",lpf_analog_in.valfilt,Kp,Kd,Ki,pidOutInt);
        Serial.println(buffprint);
    }
  }
#else
  while(1)
  {
    pidInput = analogRead(HALL_PIN_AI);
    pidInput = lpf_analog_in.compute(pidInput);

    double pos_error= pidSetpoint-pidInput;

    if(pos_error>=0)
    {
      //switch off the coil
      digitalWrite(PWM_PID_IN1_ATTRACTION, false);
      stato_uscita=false;
    }
    else
    {
      //switch on the coil
      digitalWrite(PWM_PID_IN1_ATTRACTION, true);
      stato_uscita=true;
    }
    //Serial.println(lpf_analog_in.valfilt);
  }
#endif  

    //uint16_t setpoint_raw = analogRead(AI_KP_PIN);
    //pidSetpoint = map(setpoint_raw,0,4095,1150,1350);


   /*
    char buffprint[40];
    //sprintf(buffprint, "U:%0.1f Kp:%0.1f Kd:%0.1f",lpf_analog_in.valfilt,Kp,Kd);
    sprintf(buffprint, "U:%0.1f Kp:%0.1f Kd:%0.1f O:%d",lpf_analog_in.valfilt,Kp,Kd,pidOutInt);
    Serial.println(buffprint);
    */
    //Serial.println(lpf_analog_in.valfilt);

}//end of task maglev control

void setup() {
  Serial.begin(115200);

  pinMode(PWM_PID_IN1_ATTRACTION, OUTPUT);

  //SETUP LEDS NEOPIXELS
  strip.begin();
  strip2.begin();
  strip.setBrightness(brightness); //[0-255]
  strip2.setBrightness(brightness);
  strip.clear(); // metti ad off tutti i leds
  strip2.clear();
  strip.show(); // Initialize all pixels to 'off'
  strip2.show();

#ifdef USA_PID
  //PWM IN1 ATTRACTION
  ledcSetup(PWMChannelIn1Attraction, PWMFreq, PWMResolution);
  ledcAttachPin(PWM_PID_IN1_ATTRACTION, PWMChannelIn1Attraction);
  //PWM IN2 REPULSION
  ledcSetup(PWMChannelIn2Repulsion, PWMFreq, PWMResolution);
  ledcAttachPin(PWM_PID_IN2_REPULSION, PWMChannelIn2Repulsion);
#endif
  //PID
  //PIDdiPosizione.SetOutputLimits(-PWM_MAX_DUTY_CYCLE, PWM_MAX_DUTY_CYCLE);
  PIDdiPosizione.SetOutputLimits(0, PWM_MAX_DUTY_CYCLE);
  PIDdiPosizione.SetMode(AUTOMATIC);
  PIDdiPosizione.SetSampleTime(SAMPLING_TIME_PID);//ms

  randomSeed(analogRead(0));
  
  timer_loop_pid.start();
  timer_strip_led.start();
  timer_print.start();


  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  
  // xTaskCreatePinnedToCore(
  //                   TaskMagLevCode,   /* Task function. */
  //                   "TaskMagLev",     /* name of task. */
  //                   10000,       /* Stack size of task */
  //                   NULL,        /* parameter of the task */
  //                   0,           /* priority of the task */
  //                   &TaskMagLev,      /* Task handle to keep track of created task */
  //                   0);          /* pin task to core 0 */                  
  // delay(500); 

  xTaskCreatePinnedToCore(
                    TaskLedsCode,   /* Task function. */
                    "TaskLeds",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    20,           /* priority of the task */
                    &TaskLeds,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 
  
}

void TaskLedsCode( void * pvParameters ){
  Serial.print("TaskLeds running on core ");
  Serial.println(xPortGetCoreID());

  while(1)
  {
    //LEDS
    unsigned long et_strip = timer_strip_led.getET();
    if(et_strip>5)
    {
      timer_strip_led.reset();
      for(int i=0;i<strip.numPixels();++i)
      {
        strip.setPixelColor(i, colors[id_colors]);
      }
      for(int i=0;i<strip2.numPixels();++i)
      {
        strip2.setPixelColor(i, colors[id_colors]);
      }
      strip.setBrightness(brightness);
      strip.show();
      strip2.setBrightness(brightness);
      strip2.show();


      switch(strip_state)
      {
          case 0:
          {
            strip_state=100;
            break;
          }
          case 100:
          {
            //increase-decrease brightness lineraly
            brightness += brightness_direction;
            if(brightness>255)
            {
                
                brightness=255;
                brightness_direction *= -1;
                strip_state=200;
            }
            if(brightness<0)
            {
                //cambio colore
                ++id_colors;
                brightness=0;
                brightness_direction *= -1;
            }
            break;
          }
          case 200:
          {
            brightness=50;
            if(time_in_state>=300)
            {
              strip_state=300;
            }
            break;
          }
          case 300:
          {
            brightness=255;
            if(time_in_state>=200)
            {
              strip_state=400;
            }
            break;
          }
          case 400:
          {
            brightness=0;
            if(time_in_state>=50)
            {
              strip_state=500;
            }
            break;
          }
          case 500:
          {
            brightness=255;
            if(time_in_state>=200)
            {
              //strip_state=100;
              strip_state=600;
            }
            break;
          }
          case 600:
          {
            //generare il flickering
            brightness=0;
            if(time_in_state>=15)
            {
              strip_state=700;
            }
            break;
          }
          case 700:
          {
            //generare il flickering
            brightness=180;
            if(time_in_state>=30)
            {
              strip_state=800;
            }
            break;
          }
          case 800:
          {
            //generare il flickering
            brightness=0;
            if(time_in_state>=30)
            {
              strip_state=900;
            }
            break;
          }
          case 900:
          {
            //generare il flickering
            brightness=200;
            if(time_in_state>=50)
            {
              strip_state=1000;
            }
            break;
          }
          case 1000:
          {
            //generare il flickering
            brightness=100;
            if(time_in_state>=60)
            {
              strip_state=1100;
            }
            break;
          }
          case 1100:
          {
            //generare il flickering
            brightness=255;
            if(time_in_state>=40)
            {
              strip_state=1200;
            }
            break;
          }
          case 1200:
          {
            //generare il flickering
            brightness=20;
            if(time_in_state>=80)
            {
              strip_state=1300;
            }
            break;
          }
          case 1300:
          {
            //generare il flickering
            brightness=180;
            if(time_in_state>=30)
            {
              strip_state=1400;
            }
            break;
          }
          case 1400:
          {
            //generare il flickering
            brightness=0;
            if(time_in_state>=40)
            {
              strip_state=1450;
            }
            break;
          }
          case 1450:
          {
            random_time_ms = random(5,250);
            //generare il flickering random
            brightness=random(0,255);
            strip_state=1500;
            break;
          }
          case 1500:
          {
            
            if(time_in_state>=random_time_ms)
            {
              ++count_rand_flickering;
              if(count_rand_flickering>6)
              {
                count_rand_flickering=0;
                brightness=0;              
                strip_state=100;
              }
              else
              {
                strip_state=1450;
              }
              
            }
            break;
          }
          

          
      }

      //Serial.println(strip_state);
      //Serial.println(brightness);
      //Serial.println("---");


      if(id_colors>=NUM_COLORI)
      {
        id_colors=0;
      }
      time_in_state+=et_strip;
      if(strip_state != strip_state_old)
      {
        time_in_state=0;
      }
      strip_state_old = strip_state;
    }//fine if strip led timer
    }
}

void loop() {

#ifdef USA_PID 
//if (timer_loop_pid.getET()>=SAMPLING_TIME_PID)
  if (timer_loop_pid.getETMillis()>=SAMPLING_TIME_PID)
  {
    timer_loop_pid.reset();
    pidInput = analogRead(HALL_PIN_AI);
    pidInput = lpf_analog_in.compute(pidInput);
    //Kp
    uint16_t kp_raw = analogRead(AI_KP_PIN);
    Kp = mapf(kp_raw,0,4095,10,5000);
    //Kd
    uint16_t kd_raw = analogRead(AI_KD_PIN);
    Ki = mapf(kd_raw,0,4095,0,5000);
    PIDdiPosizione.SetTunings(Kp,Ki,Kd);
    double pos_error= pidSetpoint-pidInput;
    PIDdiPosizione.Compute(pos_error, false); //derivative on error = false --> derivative on input
    int32_t pidOutInt = static_cast<int32_t>(pidOutput);
    //ATTRACTION
    ledcWrite(PWMChannelIn1Attraction, pidOutInt);
    ledcWrite(PWMChannelIn2Repulsion, 0);
    if ( timer_print.getET()>=100)
    {
        timer_print.reset();
        char buffprint[50];
        //sprintf(buffprint, "U:%0.1f Kp:%0.1f Kd:%0.1f",lpf_analog_in.valfilt,Kp,Kd);
        sprintf(buffprint, "H:%0.1f Kp:%0.1f Kd:%0.1f Ki:%0.1f O:%d",lpf_analog_in.valfilt,Kp,Kd,Ki,pidOutInt);
        Serial.println(buffprint);
    }
  }
#else
  while(1)
  {
    pidInput = analogRead(HALL_PIN_AI);
    pidInput = lpf_analog_in.compute(pidInput);

    double pos_error= pidSetpoint-pidInput;

    if(pos_error>=0)
    {
      //switch off the coil
      digitalWrite(PWM_PID_IN1_ATTRACTION, false);
      stato_uscita=false;
    }
    else
    {
      //switch on the coil
      digitalWrite(PWM_PID_IN1_ATTRACTION, true);
      stato_uscita=true;
    }
    //Serial.println(lpf_analog_in.valfilt);
  }
#endif  


/*
  //LEDS
  unsigned long et_strip = timer_strip_led.getET();
  if(et_strip>5)
  {
    timer_strip_led.reset();
    for(int i=0;i<strip.numPixels();++i)
    {
      strip.setPixelColor(i, colors[id_colors]);
    }
    // for(int i=0;i<strip2.numPixels();++i)
    // {
    //   strip2.setPixelColor(i, colors[id_colors]);
    // }
    strip.setBrightness(brightness);
    strip.show();
    // strip2.setBrightness(brightness);
    // strip2.show();


    switch(strip_state)
    {
        case 0:
        {
          strip_state=100;
          break;
        }
        case 100:
        {
          //increase-decrease brightness lineraly
          brightness += brightness_direction;
          if(brightness>255)
          {
              
              brightness=255;
              brightness_direction *= -1;
              strip_state=200;
          }
          if(brightness<0)
          {
              //cambio colore
              ++id_colors;
              brightness=0;
              brightness_direction *= -1;
          }
          break;
        }
        case 200:
        {
          brightness=50;
          if(time_in_state>=300)
          {
            strip_state=300;
          }
          break;
        }
        case 300:
        {
          brightness=255;
          if(time_in_state>=200)
          {
            strip_state=400;
          }
          break;
        }
        case 400:
        {
          brightness=0;
          if(time_in_state>=50)
          {
            strip_state=500;
          }
          break;
        }
        case 500:
        {
          brightness=255;
          if(time_in_state>=200)
          {
            //strip_state=100;
            strip_state=600;
          }
          break;
        }
        case 600:
        {
          //generare il flickering
          brightness=0;
          if(time_in_state>=15)
          {
            strip_state=700;
          }
          break;
        }
        case 700:
        {
          //generare il flickering
          brightness=180;
          if(time_in_state>=30)
          {
            strip_state=800;
          }
          break;
        }
        case 800:
        {
          //generare il flickering
          brightness=0;
          if(time_in_state>=30)
          {
            strip_state=900;
          }
          break;
        }
        case 900:
        {
          //generare il flickering
          brightness=200;
          if(time_in_state>=50)
          {
            strip_state=1000;
          }
          break;
        }
        case 1000:
        {
          //generare il flickering
          brightness=100;
          if(time_in_state>=60)
          {
            strip_state=1100;
          }
          break;
        }
        case 1100:
        {
          //generare il flickering
          brightness=255;
          if(time_in_state>=40)
          {
            strip_state=1200;
          }
          break;
        }
        case 1200:
        {
          //generare il flickering
          brightness=20;
          if(time_in_state>=80)
          {
            strip_state=1300;
          }
          break;
        }
        case 1300:
        {
          //generare il flickering
          brightness=180;
          if(time_in_state>=30)
          {
            strip_state=1400;
          }
          break;
        }
        case 1400:
        {
          //generare il flickering
          brightness=0;
          if(time_in_state>=40)
          {
            strip_state=1450;
          }
          break;
        }
        case 1450:
        {
          random_time_ms = random(5,250);
          //generare il flickering random
          brightness=random(0,255);
          strip_state=1500;
          break;
        }
        case 1500:
        {
          
          if(time_in_state>=random_time_ms)
          {
            ++count_rand_flickering;
            if(count_rand_flickering>6)
            {
              count_rand_flickering=0;
              brightness=0;              
              strip_state=100;
            }
            else
            {
              strip_state=1450;
            }
            
          }
          break;
        }
        

        
    }

    //Serial.println(strip_state);
    //Serial.println(brightness);
    //Serial.println("---");


    if(id_colors>=NUM_COLORI)
    {
      id_colors=0;
    }
    time_in_state+=et_strip;
    if(strip_state != strip_state_old)
    {
      time_in_state=0;
    }
    strip_state_old = strip_state;
  }//fine if strip led timer
*/

}//fine loop

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    const float run = in_max - in_min;
    if(run == 0){       
        return 0;
    }
    const float rise = out_max - out_min;
    const float delta = x - in_min;
    return (delta * rise) / run + out_min;
}


float raw2mm49E_north_pole(float analog_raw, float offset_mm)
{
  if(analog_raw<1850)
  {
    return 40; // piu su di 80mm perdo informazione sulla posizione del magnete permanente
  }
  //facendo i test si nota un offset costante sul dominio di -10mm
  float x1=analog_raw;
  float x2=x1*x1;
  float x3=x2*x1;
  float x4=x3*x1;
  float x5=x4*x1;
  //polinomio 3th ordine
  //return ( -0.0000001298*x3 + 0.0009897689*x2 + -2.5091822775*x1 + 2130.0019277931) + offset_mm;
  //polinomio 5th ordine
  return (-7.649229525312638e-13*x5 + 9.429428771266764e-09*x4 + -4.629951478242045e-05*x3 + 0.113209292438759*x2 + -1.378929922289905e+02*x1 + 6.698064182520430e+04);
}
