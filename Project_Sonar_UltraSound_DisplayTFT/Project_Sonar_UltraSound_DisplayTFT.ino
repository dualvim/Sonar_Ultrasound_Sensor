/*
* Project_Sonar_UltraSound_DisplayTFT.ino     
*
*/


// --> Carregar as bibliotecas utilizadas no sketch
#include <CheapStepper.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>
#include <Adafruit_ST7735.h>




/********************************************
** Symbolic Constants and global variables **
********************************************/
// Stepper Motor pins
#define M1 22 //IN1 (Blue) 
#define M2 23 //IN2 (Purple) 
#define M3 24 //IN3 (White) 
#define M4 25 //IN4 (Orange) 

// Constants: Stepper motor parameters
int STEPS_PER_TURN = 2048; //Number of steps per complete turn in 28BYJ-48
#define MAX_SPEED 10 // Stepper motor speed in RPM: 10 rpm
#define MOV 5 // Movimento do motor (6 dgrs)

// SR-HC04 pins
#define TRIG 32
#define ECHO 34

// Maximum waiting time for a pulse
//unsigned long timeout = 29000; //Dmax = 500 cm, D=t/58, timeout=500*58 = 29000
#define TIMEOUT 29000 

//  Constant: LM35DZ pin connected to the Arduino
#define LM35DZ_PIN A0


// Constants: TFT display pins
#define TFT_CS 6 //Pino conectado a porta 'CS' do meu display
#define DCpin 9 //Pino conectado a porta 'RS' do meu display
#define RSTpin 7 //Pino conectado a porta 'RST' do meu display

//  Constants: hexadecimais values of the colors used here
#define BLACK 0x0000
#define YELLOW 0xFFE0
#define LITEYEL 0xFFF5
#define GREEN 0x07E0
#define WHITE 0xFFFF
#define CYAN 0x00FF
// Constants: Sonar parameters 
#define maxdist 50 // Maximum distance
#define radius 110.0 //Radius


// --> Global variables
bool clock_wise = true;
unsigned int distance_cm; //Distance in cm
unsigned int t_echo; //Echo time
float temperature; //temperature
float snd_speed; // Sound speed





/*******************************************************************
** Objetos para controlar os dispositivos conectados ao Arduino   **
*******************************************************************/  
CheapStepper my_stepper( M1, M2, M3, M4 ); //Controls the stepper motor 28BYJ-48
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, DCpin, RSTpin); //Controls the TFT display



/******************************
** Function prototypes       **
*****************************/ 
float read_temp_lm35dz( void );
float calc_sound_speed( float temp );
unsigned int ultrasound_t_echo( void );
unsigned int ultrasound_cm( unsigned int time_us, float snd_spd );
void radar1( void );
void radar2( int dgrs, unsigned int dist );
String string_text_temp_sound_speed( float temp, float snd_spd );
String string_text_distance_monitor_serial( int dgrs, unsigned int t_eco, unsigned int dist );




/******************
** setup()       ** 
******************/
void setup() {
      Serial.begin(9600); 

      // Temperature sensor LM35DZ
      pinMode(LM35DZ_PIN, INPUT);
      //analogReference(INTERNAL); //Arduino Uno/Nano
      analogReference(INTERNAL1V1); //Arduino Mega 2560
      
      // Ultrasonic distance sensor
      pinMode(TRIG, OUTPUT);
      pinMode(ECHO, INPUT);
      
      //Set maximum speed
      my_stepper.setRpm( MAX_SPEED );

      // Number of steps per full turn
      my_stepper.setTotalSteps( STEPS_PER_TURN );

      // --> Inicialize the TFT display
      tft.initR(INITR_BLACKTAB);
      tft.fillScreen(BLACK); 
      tft.setRotation(3);
      tft.setTextColor(WHITE, BLACK);
      tft.drawRect(0, 0, 128, 160, WHITE); 
      tft.drawRect(1, 1, 158, 126, WHITE); 
      tft.setTextSize(1);
      tft.setCursor(3,3);
      tft.print("distance");
      tft.setCursor(95,3);
      tft.print("Raio ");
      tft.setCursor(135,3);
      tft.print(maxdist); 
}




/****************
** loop()      **  
****************/
void loop() {
      // Draw the base of the output drawn in the TFT display
      radar1();
      
      // --> Move -90º
      clock_wise = false;
      my_stepper.moveDegrees(clock_wise, 90);
      delay(1000);

      
      // --> Read temp and calculate the sound speed
      temperature = read_temp_lm35dz();
      snd_speed = calc_sound_speed( temperature );
      //temperature = 20;
      //snd_speed = 343;
      Serial.println( string_text_temp_sound_speed(temperature, snd_speed) );
      
      
      //--> Make a 180º sweep
      Serial.println("Activating the sonar:");
      clock_wise = true;
      
      for(int dgrs = 0; dgrs <= 180; dgrs += MOV ){
            //Send a pulse and calculate the echo time:
            t_echo = ultrasound_t_echo();
            distance_cm = ultrasound_cm( t_echo, snd_speed );
            Serial.println( string_text_distance_monitor_serial( dgrs, t_echo, distance_cm) );

            // Write data in the TFT display:
            radar2( dgrs, distance_cm );

            //Next angle
            my_stepper.moveDegrees(clock_wise, MOV);
            
            //Wait 100ms:
            delay(100);
      }
      
      // --> Back to the 0º position
      Serial.println("Going back to the 0º position:");
      clock_wise = false;
      my_stepper.moveDegrees (clock_wise, 90);

      Serial.println("10 seconds pause");
      delay(10000);
}




/************************
** Auxiliary functions **  
************************/
// --> 'read_temp_lm35dz()'
float read_temp_lm35dz( void ){ return (analogRead(LM35DZ_PIN) * 110.0)/1023.0; }


/* --> 'calc_sound_speed()': Speed = 331.0 + 0.6* temperature
*/
float calc_sound_speed( float temp ){  return 331.0 + 0.6 * temp;  }


// -->'ultrasound_cm()'
unsigned int ultrasound_cm( unsigned int time_us, float snd_spd ){  return time_us * (snd_spd / 20000 );  }


// --> 'ultrasound_t_echo()'
unsigned int ultrasound_t_echo( void ){
      unsigned long previousMicros;
      
      // Desligar o pino 'TRIG' e liga-lo por 10 microsegundos
      digitalWrite(TRIG, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG, LOW);

      previousMicros = micros();
      while( !digitalRead(ECHO) && ((micros() - previousMicros)  <= TIMEOUT) ){    }
      
      previousMicros = micros();
      while( digitalRead(ECHO) && ((micros() - previousMicros)  <= TIMEOUT) ){       }

      // Time taken
      return micros() - previousMicros;
}


//--> 'radar1()'
void radar1( void ){
      tft.fillRect(2, 12, 156, 114, BLACK); // clear screen apart from frame
      tft.drawCircle(80, 128, radius/2, YELLOW); // draw arc to assist reading
      tft.drawCircle(80, 128,radius, YELLOW); //image and second arc
}


//--> 'radar2()'
void radar2( int dgrs, unsigned int dist ){     
      // converter 'dist' para array de caracteres:
      char printOut[5];
      String dist2 = String(dist); // convert distance to string
      if( dist < 10 ){ dist2 = "  " + dist2; }
      else if( dist < 100 ){ dist2 = " " + dist2; }
      dist2.toCharArray(printOut, 5); // convert string to characters

      // Mostrar a distance no display TFT
      tft.setCursor(60, 3); // move cursor
      tft.print(printOut); // display distance on screen

      // Posicao
      int x = radius * cos(dgrs * PI/180); // calculate scan line
      int y = radius * sin(dgrs * PI/180);
      tft.drawLine(80, 128, 80+x, 128-y, LITEYEL); // draw line from baseline to arc
      //x = x * dist/maxdist; // calculate position of object
      //y = y * dist/maxdist;

      if( dgrs <= 90 ){
            x = x * dist/maxdist; // calculate position of object
            y = y * dist/maxdist;
      
           tft.fillCircle(80+x, 128-y, 2, CYAN); // draw circle when object detected
           //Serial.print("80+x = "); Serial.print(80+x); Serial.print(";\t128-y = "); Serial.println(128-y); 
      }
      else{
            x = radius * cos((90-(dgrs-90) * PI/180)); // calculate scan line
            y = radius * sin(90-(dgrs-90) * PI/180);
            
            x = x * dist/maxdist; // calculate position of object
            y = y * dist/maxdist;
            tft.fillCircle(80-x, 128-y, 2, CYAN); // draw circle when object detected
            //Serial.print("80-x = "); Serial.print(80-x); Serial.print(";\t128-y = "); Serial.println(128-y); 
      }
      
}


// --> Funcao 'string_text_temp_sound_speed()': Gera a string com a temperature e velocidade do som
String string_text_temp_sound_speed( float temp, float snd_spd ){ return "temperature = " + String(temp, 1) + "ºC;\t snd_speed = " + String(snd_spd, 1) + " m/s"; }


// --> Funcao 'string_text_distance_monitor_serial()': Gera a string com a distance e o angulo
String string_text_distance_monitor_serial( int dgrs, unsigned int t_eco, unsigned int dist ){ return "dgrs = " + String(dgrs) + "º - t_eco = " + String(t_eco) + " us - distance = " + String(dist) + " cm";  }
