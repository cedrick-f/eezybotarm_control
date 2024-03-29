/*************************************************** 
  EEZYbotARM
  Programme pour piloter le bras de robot
  depuis une application Python

 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "RunningAverage.h"

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

/********************************************************************/
/* Version du robot */
/*  0 : BLEU  
/*  1 : ROUGE  
/*  2 : JAUNE  
/*  3 : VERT  
*/
#define ROBOT 0

/********************************************************************/
/* Angles limites des bras et avant-bras du robot */
const float  DEGREMIN[2] = {136, -65}; 
const float  DEGREMAX[2] = {40, 19}; 

/* Sens des moteurs */
const int16_t  SENS[2] = {-1, 1}; 

/* Pente des moteurs (rapport pulsation/angle) */
const float  PENTE[2] = {2, 2}; 

/* Pulsation à l'angle DEGREMIN (à déterminer expérimentalement) */
#if ROBOT == 0 // BLEU
const float PULSEMIN[2] = {210, 331}; // en pulse
#elif ROBOT == 1 // ROUGE
const float PULSEMIN[2] = {172, 310}; // en pulse
#elif ROBOT == 2 // JAUNE
const float PULSEMIN[2] = {242, 360}; // en pulse
#elif ROBOT == 3 // VERT
const float PULSEMIN[2] = {220, 250}; // en pulse
#endif

/* Décalage angulaire bras/servo (à déterminer expérimentalement ou bien DEGREMIN seulement) */
const float OFFSET[2] = {207, 188}; // en degrés


/* Longueurs de pulsation (pulse) et angles limites */
/* ATTENTION : différent pour chaque robot !!! */
//const uint16_t  PULSEMIN[2] = {SENS[0]*PENTE[0]*DEGREMIN[0],  SENS[1]*PENTE[1]*DEGREMIN[1]}//{220, 330}; // This is the 'minimum' pulse length count (out of 4096)
const uint16_t  PULSEMAX[2] = {SENS[0]*PENTE[0]*(DEGREMAX[0] - DEGREMIN[0]) + PULSEMIN[0], 
                               SENS[1]*PENTE[1]*(DEGREMAX[1] - DEGREMIN[1]) + PULSEMIN[1]}; //{400, 500}; // This is the 'maximum' pulse length count (out of 4096)



//const uint16_t PULSELIM[2] = {440, 480}; // positions limites sur servo 1 quand servo 0 au min et max
const float ANGLELIM[2] = {-10, -5}; // angles limites de l'avant-bras quand le bras est au min et max


/********************************************************************/
/* Ports utilisés */
const uint8_t SERVOPIN[2] = {15, 14};


const uint8_t ANGLEPIN[2] = {A0, A1};

/********************************************************************/
/* Paramètres caractéristiques du servomoteur pour mesure angle */
const float A = -3.557e-7;
const float B = 3.947e-4;
const float C = 0.3792;
const float D = -34.514;

/********************************************************************/
/* État initial */
//const uint16_t PULSECENTRE[2] = {330, 460};
const float ANGLECENTRE[2] = {90, 0};
float ANGLE[2] = {ANGLECENTRE[0], ANGLECENTRE[1]};
uint16_t LASTANGLE[2] = {360, 360};
uint16_t LASTPULSE[2] = {0, 0};

float ANGLE_MES[2] = {ANGLE[0], ANGLE[1]}; // Angles mesurés
const uint16_t PERIODE = 100; // période d'envoi des angles en ms
uint16_t lt = 0;

const uint16_t RA_L = 32;
RunningAverage RA_a(RA_L);
RunningAverage RA_b(RA_L);

//uint16_t PULSE[2] = {PULSECENTRE[0], PULSECENTRE[1]}; // position des servos (initialement "centrale") en pulse
                                // doit permettre d'atteindre les min et max des 2 axes
uint8_t current_servo = 0;
bool on = false;
bool force = true;

/********************************************************************/
/* Dimensions du robot */
const uint8_t a = 135; // mm
const uint8_t b = 147; // mm

#define PI 3.1415926535897932384626433832795
const uint8_t PAS = 1;  // pas de mouvement (mm)
//#define VIT 10 // vitesse de mouvement (mm/s)

/********************************************************************/
/* Pour réception données */
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
char cmd[2] = {0};
boolean newData = false;


/********************************************************************/
/********************************************************************/
void setup() {
  Serial.begin(460800);

  Serial.println("EezyBot Servo !");
  Serial.println("Commandes :");
  Serial.println("ad <angle degre> = consigne angle servo a");
  Serial.println("bd <angle degre> = consigne angle servo b");
  Serial.println("ar <angle rad> = consigne angle servo a");
  Serial.println("br <angle rad> = consigne angle servo b");
  Serial.println("cc = aller au centre");
  Serial.println("of = extinction moteurs");
  Serial.println("xy <x> <y> = consignes position poignet en mm");
  Serial.println("ap <pulse> = consigne servo a en pulse");
  Serial.println("bp <poulse> = consigne servo b en pulse");
  Serial.println("px = demande positions xy");
  Serial.println("pd = demande positions angulaire en degre");
  Serial.println("pp = demande consignes servo en pulse");
  Serial.println();
  Serial.print("Servo a :");
  Serial.println(PULSEMAX[0]);
  Serial.print("Servo b :");
  Serial.println(PULSEMAX[1]);
  Serial.setTimeout(500);

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(24460000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  RA_a.clear();
  RA_b.clear();
  RA_a.fillValue(ANGLE_MES[0], RA_L);
  RA_b.fillValue(ANGLE_MES[1], RA_L);


  delay(100);

  // placement à la position initiale
  centrer();
}


/********************************************************************/
/********************************************************************/
void loop() {
  send_angles_mes();
  recvWithStartEndMarkers();
  if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        
        char * strtokIndx; // this is used by strtok() as an index

        strtokIndx = strtok(tempChars," ");      // get the first part - the string
        strcpy(cmd, strtokIndx); // copy it to cmd
        Serial.println(cmd);

        if (strcmp(cmd,"ad")==0) {
          //int angle = Serial.parseInt();
          strtokIndx = strtok(NULL, " "); // this continues where the previous call left off
          int angle = atoi(strtokIndx);     // convert this part to an integer
          if (dans_zone(angle, ANGLE[1])) {
            move_to_degres(0, angle);
          } 
          send_angles();
          send_xy();
          
          

        } else if (strcmp(cmd,"bd")==0) {
          //int angle = Serial.parseInt();
          strtokIndx = strtok(NULL, " "); // this continues where the previous call left off
          int angle = atoi(strtokIndx);     // convert this part to an integer
          if (dans_zone(ANGLE[0], angle)) {
            move_to_degres(1, angle);
          }
          send_angles();
          send_xy();
          

        } else if (strcmp(cmd,"ar")==0) {
          // statements

        } else if (strcmp(cmd,"br")==0) {
          // statements

        } else if (strcmp(cmd,"ap")==0) {
          strtokIndx = strtok(NULL, " "); // this continues where the previous call left off
          long pulse = atoi(strtokIndx);     // convert this part to an integer
          move_to_pulse(0, pulse);
          //pwm.setPWM(SERVOPIN[0], 0, PULSE[0]);

        } else if (strcmp(cmd,"bp")==0) {
          strtokIndx = strtok(NULL, " "); // this continues where the previous call left off
          long pulse = atoi(strtokIndx);     // convert this part to an integer
          move_to_pulse(1, pulse);
          //pwm.setPWM(SERVOPIN[1], 0, PULSE[1]);

        } else if (strcmp(cmd,"cc")==0) {
          centrer();

        } else if (strcmp(cmd,"of")==0) {
          switch_off();

        } else if (strcmp(cmd,"xy")==0) {
          //int x = Serial.parseInt();
          //int y = Serial.parseInt();
          strtokIndx = strtok(NULL, " "); // this continues where the previous call left off
          int x = atoi(strtokIndx);     // convert this part to an integer
          strtokIndx = strtok(NULL, " "); // this continues where the previous call left off
          int y = atoi(strtokIndx);     // convert this part to an integer
          move_to_XY(x, y);

        } else if (strcmp(cmd,"px")==0) {
          send_xy();

        } else if (strcmp(cmd,"pd")==0) {
          send_angles();
        } else if (strcmp(cmd,"pp")==0) {
          send_pulses();

        } else if (strcmp(cmd,"pa")==0) {
          send_angle(0);
          
        } else if (strcmp(cmd,"pb")==0) {
          send_angle(1);
        }
        newData = false;
    }
}

/********************************************************************/
/* envoi des positions */
void send_pulses() {
  Serial.print("_pp ");
  Serial.print(to_pulse(0));
  Serial.print(",");
  Serial.println(to_pulse(1));
}

void send_angles() {
  Serial.print("_aa ");
  Serial.print(round(ANGLE[0]));
  Serial.print(",");
  Serial.println(round(ANGLE[1]));
  //Serial.flush();
}



void send_angle(uint8_t servo) {
  Serial.print("_a");
  Serial.print(servo);
  Serial.print(" ");
  Serial.println(round(ANGLE[servo]));
  //Serial.flush();
}

void send_xy() {
  float x0, y0;
  float alpha = to_rad(ANGLE[0]);
  float theta = to_rad(ANGLE[1]);
  angles_to_xy(x0, y0, alpha, theta);
  //Serial.flush();
  Serial.print("_xy ");
  Serial.print(round(x0));
  Serial.print(",");
  Serial.println(round(y0));
  //
}


/********************************************************************/
/* Placement au centre */
void centrer() {
  for (uint8_t servo = 0; servo < 2 ; servo++) {
    //pwm.setPWM(SERVOPIN[servo], 0, PULSECENTRE[servo]);
    move_to_degres(servo, ANGLECENTRE[servo]);
    ANGLE[servo] = ANGLECENTRE[servo];  // mise à jour position courante (consigne)
    send_angles();                      // envoi consignes angulaires
    //send_angles_mes();                  // envoi angles mesurés
    send_xy();
  }
}

void switch_on() {
  //centrer();
  on = true;
  for (uint8_t servo = 0; servo < 2 ; servo++) {
    move_to_degres(servo, ANGLE[servo]);
  }
  
}

/********************************************************************/
/* Extinction */
void switch_off() {
  for (uint8_t servo = 0; servo < 2 ; servo++) {
    pwm.setPWM(SERVOPIN[servo], 0, 4096);
  }
  on = false;
}


/********************************************************************/
/* Mouvements d'un axe (en pulse) */
void move_to_pulse(uint8_t servo, uint16_t pulselen) {
  if (!on) {
    switch_on();
  }
  pwm.setPWM(SERVOPIN[servo], 0, pulselen);
  


  //bool into = true;
  //Serial.print("Consigne:");
  //Serial.println(pulselen);

  //uint16_t minpulse = min_servo(servo);
  //uint16_t maxpulse = max_servo(servo);
  /*Serial.print("Limites :");
  Serial.print(minpulse);
  Serial.print("\t");
  Serial.println(maxpulse);*/

  /*if (pulselen < minpulse) {
    pulselen = minpulse;
    //Serial.print("---");
    //Serial.println(pulselen);
    into = false;
  }

  if (pulselen > maxpulse) {
    pulselen = maxpulse;
    //Serial.print("+++");
    //Serial.println(pulselen);
    into = false;
  }*/

  //PULSE[servo] = pulselen;
  /*float last = ANGLE[servo];
  //ANGLE[servo] = pulse_to_degres(servo, pulselen); //map(pulselen, PULSEMIN[servo], PULSEMAX[servo], DEGREMIN[servo], DEGREMAX[servo]);
  if (force || ANGLE[servo] != last) {
    send_angles();
    //send_angles_mes();                  // envoi angles mesurés
    send_xy();
    pwm.setPWM(SERVOPIN[servo], 0, pulselen);
    if (servo == 1) {
      force = false;
    }
  }
  return into;*/
}
bool dans_zone(float angle0, float angle1) {
  float mini0, maxi0, mini1, maxi1;
  mini0 = max(DEGREMAX[0], angle1 - (ANGLELIM[1] - DEGREMAX[0]));
  maxi0 = min(DEGREMIN[0], angle1 - (ANGLELIM[0] - DEGREMIN[0]));
  mini1 = max(DEGREMIN[1], angle0 + (ANGLELIM[0] - DEGREMIN[0]));
  maxi1 = min(DEGREMAX[1], angle0 + (ANGLELIM[1] - DEGREMAX[0]));
  return angle0 > mini0 && angle1 > mini1 && angle0 < maxi0 && angle1 < maxi1;
}


/********************************************************************/
/* Mouvements d'un axe (en degrés) */
void move_to_degres(uint8_t servo, float angle) {
  /*float mini, maxi;
  if (servo == 0) {
    mini = max(DEGREMAX[0], ANGLE[1] - (ANGLELIM[1] - DEGREMAX[0]));
    maxi = min(DEGREMIN[0], ANGLE[1] - (ANGLELIM[0] - DEGREMIN[0]));
  } else {
    mini = max(DEGREMIN[1], ANGLE[0] + (ANGLELIM[0] - DEGREMIN[0]));
    maxi = min(DEGREMAX[1], ANGLE[0] + (ANGLELIM[1] - DEGREMAX[0]));
  };
  /*Serial.print("mini-maxi ");
  Serial.print(servo);
  Serial.print(":");
  Serial.print(mini);
  Serial.print("_");
  Serial.println(maxi);
  bool into = true;
  if (angle > maxi) {
    angle = maxi;
    //Serial.print("+++");
    //Serial.println(angle);
    into = false;
  };

  if (angle < mini) {
    angle = mini;
    //Serial.print("---");
    //Serial.println(angle);
    into = false;
  };*/
  
  ANGLE[servo] = angle;
  if (uint16_t(round(ANGLE[servo])) != LASTANGLE[servo]) {
    send_angles();
    send_xy();
    LASTANGLE[servo] = round(ANGLE[servo]);
  }

  uint16_t pulselen = map(angle, DEGREMIN[servo], DEGREMAX[servo], PULSEMIN[servo], PULSEMAX[servo]);
  if (force || LASTPULSE[servo] != pulselen) {
    move_to_pulse(servo, pulselen);
    if (servo == 1) {
      force = false;
    }
    LASTPULSE[servo] = pulselen;
  };
}

/********************************************************************/
/* Mouvements du bras en ligne droite jusqu'aux coordonnées (x1,y1) */
void move_to_XY(float x1, float y1) {
  float x0, y0;
  float alpha = to_rad(ANGLE[0]);
  float theta = to_rad(ANGLE[1]);
  angles_to_xy(x0, y0, alpha, theta);

  float dist = sqrt(sq(x1-x0) + sq(y1-y0));
  //Serial.print("Dist:");
  //Serial.println(dist);
  uint16_t npas = dist/PAS;
  float pasx = (x1-x0)/npas;
  float pasy = (y1-y0)/npas;
  //Serial.print("Nbr pas:");
  //Serial.println(npas);

  float x = x0;
  float y = y0;

  for (uint16_t i=1; i<=npas ; i++) {
    //Serial.print("##### i=");
    //Serial.println(i);
    xy_to_angles(alpha, theta, x, y);
    //Serial.print(to_deg(alpha));
    //Serial.print("\t");
    //Serial.println(to_deg(theta));
    if (dans_zone(to_deg(alpha), to_deg(theta))) {
      move_to_degres(0, to_deg(alpha));
      move_to_degres(1, to_deg(theta));
      x = pasx*i + x0;
      y = pasy*i + y0;
    } else {
      send_angles();
      send_xy();
      Serial.println("OUT");
      break;
    };
    delay(10);
  }
}

/********************************************************************/
/* Calcul (x,y) en fonction des angles */
/* http://www.osrobotics.org/osr/kinematics/inverse_kinematics.html */
void xy_to_angles(float &alpha, float &theta, float &x, float &y) {
  /*Serial.print("xy_to_angles :");
  Serial.print(x);
  Serial.print("\t");
  Serial.print(y);*/

  float ad = float(a);
  float bd = float(b);

  float x_y = sq(x)+sq(y);
  float a_b = sq(ad)+sq(bd);
  float a_t = -acos((x_y-a_b)/(2*ad*bd)); // 0 -- PI

  float k1 = ad + bd*cos(a_t);
  float k2 = bd*sin(a_t);
  alpha = atan2(y, x) - atan2(k2, k1);  // -PI/2 -- PI/2
  theta = alpha + a_t; // -3PI/2 -- PI/2

  if (alpha < 0) {
    alpha += PI;
  }

  if (theta < -PI/2) {
    theta += PI;
  }
  /*Serial.print("\t>>>\t");
  Serial.print(to_deg(alpha));
  Serial.print("\t");
  Serial.println(to_deg(theta));*/
}

/********************************************************************/
/* Calcul (x,y) en fonction des angles */
void angles_to_xy(float &x, float &y, float &alpha, float &theta) {
  /*Serial.print("angles_to_xy :");
  Serial.print(to_deg(alpha));
  Serial.print("\t");
  Serial.print(to_deg(theta));*/
  x = float(a)*cos(alpha) + float(b)*cos(theta);
  y = float(a)*sin(alpha) + float(b)*sin(theta);
  /*Serial.print("\t>>>\t");
  Serial.print(x);
  Serial.print("\t");
  Serial.println(y);*/

}

/********************************************************************/
/* Conversions pulse<>degres<>radians */
float to_rad(float deg) {
  return PI*deg/180;
}

float to_deg(float rad) {
  return rad*180/PI;
}

float pulse_to_degres(uint8_t servo, uint16_t pulse) {
  return (float(pulse) - float(PULSEMIN[servo])) * (float(DEGREMAX[servo]) - float(DEGREMIN[servo])) / (float(PULSEMAX[servo]) - float(PULSEMIN[servo])) + float(DEGREMIN[servo]);
  //return map(pulse, PULSEMIN[servo], PULSEMAX[servo], DEGREMIN[servo], DEGREMAX[servo]);
}

float degres_to_pulse(uint8_t servo, int16_t degres) {
  return (float(degres) - float(DEGREMIN[servo])) * (float(PULSEMAX[servo]) - float(PULSEMIN[servo])) / (float(DEGREMAX[servo]) - float(DEGREMIN[servo])) + float(PULSEMIN[servo]);
  //return map(degres, DEGREMIN[servo], DEGREMAX[servo], PULSEMIN[servo], PULSEMAX[servo]);
}

float to_pulse(uint8_t servo) {
  return degres_to_pulse(servo, ANGLE[servo]);
}


/********************************************************************/
/* Calcul des limites (en pulse) */
/*uint16_t max_servo(uint8_t servo) {
  uint16_t pulse = to_pulse(servo);
  if (servo == 1) {
    pulse = min(PULSEMAX[1], degres_to_pulse(1, ANGLELIM[1])-to_pulse(0)+PULSEMAX[0]);
  } else if (servo == 0) {
    pulse = min(PULSEMAX[0], degres_to_pulse(0, ANGLELIM[1])-to_pulse(1)+PULSEMAX[0]);
  } else {
    //Serial.println("!!");
  }
  return pulse;
}

uint16_t min_servo(uint8_t servo) {
  uint16_t pulse = to_pulse(servo);
  if (servo == 1) {
    pulse = max(PULSEMIN[1], degres_to_pulse(1, ANGLELIM[0])-to_pulse(0)+PULSEMIN[0]);
  } else if (servo == 0) {
    pulse = max(PULSEMIN[0], degres_to_pulse(0, ANGLELIM[0])-to_pulse(1)+PULSEMIN[0]);
  } else {
    //Serial.println("!!");
  }
  return pulse;
}*/


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

/********************************************************************/
/* Mesure des angles des servomoteurs */

float degres(int val) {
  return A*val*val*val + B*val*val + C*val + D;
}

void send_angles_mes() {
  float a = ANGLE_MES[0];
  float b = ANGLE_MES[1];

  String aa = "N";
  String bb = "N";

  int16_t am = analogRead(ANGLEPIN[0]);
  int16_t bm = analogRead(ANGLEPIN[1]);

  if (am > 0) { // port connecté = mesure
    RA_a.addValue((degres(am)-OFFSET[0]) * SENS[0]);
    a = RA_a.getAverage();
    aa = String(a);
  }

  if (bm > 0) { // port connecté = mesure
    RA_b.addValue((degres(bm)-OFFSET[1]) * SENS[1]);
    b = RA_b.getAverage();
    bb = String(b);
  }
  
  uint16_t t = millis();
  if (t-lt > PERIODE && (a != ANGLE_MES[0] || b != ANGLE_MES[1]) && (am + bm > 0)) {
    ANGLE_MES[0] = a;
    ANGLE_MES[1] = b;
    Serial.print("_am ");
    Serial.print(aa);
    Serial.print(",");
    Serial.println(bb);
    //Serial.flush();
    lt = t;
  }
}