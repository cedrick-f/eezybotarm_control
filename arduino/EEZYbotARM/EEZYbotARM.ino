/*************************************************** 
  EEZYbotARM
  Programme pour piloter le bras de robot
  depuis une application Python

 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

/********************************************************************/
/* Angles limites des bras et avant-bras du robot */
const int16_t ANGLEMIN[2] = {136, -65}; 
const int16_t ANGLEMAX[2] = {40, 19};
const int16_t ANGLELIM[2] = {0, -30}; // angles limites de l'avant-bras quand le bras est au min et max

/* Sens des moteurs */
const int16_t  SENS[2] = {-1, 1}; 

/* Pente des moteurs (rapport pulsation/angle) */
const float  PENTE[2] = {2, 2}; 

/* Pulsation à l'angle ANGLEMIN (à déterminer expérimentalement) */
const int16_t PULSEMIN[2] = {210, 331};

/* Longueurs de pulsation (pulse) et angles limites */
/const uint16_t  PULSEMAX[2] = {SENS[0]*PENTE[0]*(ANGLEMAX[0] - ANGLEMIN[0]) + PULSEMIN[0], 
                               SENS[1]*PENTE[1]*(ANGLEMAX[1] - ANGLEMIN[1]) + PULSEMIN[1]};

/********************************************************************/
/* Ports utilisés */
const uint8_t SERVOPIN[2] = {15, 14};
const uint8_t ANGLEPIN[2] = {A0, A1};

/********************************************************************/
/* État initial */
const int16_t ANGLECENTRE[2] = {80, 0};
int16_t ANGLE[2] = {ANGLECENTRE[0], ANGLECENTRE[1]};// position des servos (initialement "centrale")
int16_t ANGLE_MES[2] = {ANGLE[0], ANGLE[1]}; // Angles mesurés


bool on = false;

/********************************************************************/
/* Dimensions du robot */
const uint8_t a = 135; // mm
const uint8_t b = 147; // mm

#define PI 3.1415926535897932384626433832795
#define PAS 2  // pas de mouvement (mm)
#define VIT 10 // vitesse de mouvement (mm/s)

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

  delay(100);

  // placement à la position initiale
  centrer();
}


/********************************************************************/
/********************************************************************/
void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        
        char * strtokIndx; // this is used by strtok() as an index

        strtokIndx = strtok(tempChars," ");      // get the first part - the string
        strcpy(cmd, strtokIndx); // copy it to cmd
        //Serial.println(cmd);

        if (strcmp(cmd,"ad")==0) {
          //int angle = Serial.parseInt();
          strtokIndx = strtok(NULL, " "); // this continues where the previous call left off
          int angle = atoi(strtokIndx);     // convert this part to an integer
          move_to_degres(0, angle);

        } else if (strcmp(cmd,"bd")==0) {
          //int angle = Serial.parseInt();
          strtokIndx = strtok(NULL, " "); // this continues where the previous call left off
          int angle = atoi(strtokIndx);     // convert this part to an integer
          move_to_degres(1, angle);

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
          send_angles_mes();                  // envoi angles mesurés
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
  Serial.print(ANGLE[0]);
  Serial.print(",");
  Serial.println(ANGLE[1]);
}

void send_angles_mes() {
  Serial.print("_am ");
  Serial.print(ANGLE_MES[0]);
  Serial.print(",");
  Serial.println(ANGLE_MES[1]);
}

void send_angle(uint8_t servo) {
  Serial.print("_a");
  Serial.print(servo);
  Serial.print(" ");
  Serial.println(ANGLE[servo]);
}

void send_xy() {
  double x0, y0;
  double alpha = to_rad(ANGLE[0]);
  double theta = to_rad(ANGLE[1]);
  angles_to_xy(x0, y0, alpha, theta);
  Serial.print("_xy ");
  Serial.print(x0);
  Serial.print(",");
  Serial.println(y0);
  //
}


/********************************************************************/
/* Placement au centre */
void centrer() {
  for (uint8_t servo = 0; servo < 2 ; servo++) {
    move_to_degres(servo, ANGLECENTRE[servo]);
    ANGLE[servo] = ANGLECENTRE[servo];  // mise à jour position courante (consigne)
    send_angles();                      // envoi consignes angulaires
    send_angles_mes();                  // envoi angles mesurés
    send_xy();
  }
}

void switch_on() {
  on = true;
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
bool move_to_pulse(uint8_t servo, uint16_t pulselen) {
  if (!on) {
    switch_on();
  }
  bool into = true;

  uint16_t minpulse = min_servo(servo);
  uint16_t maxpulse = max_servo(servo);

  if (pulselen < minpulse) {
    pulselen = minpulse;
    into = false;
  }

  if (pulselen > maxpulse) {
    pulselen = maxpulse;
    into = false;
  }

  ANGLE[servo] = pulse_to_degres(servo, pulselen);
  send_angles();
  send_angles_mes();                  // envoi angles mesurés
  send_xy();
  pwm.setPWM(SERVOPIN[servo], 0, pulselen);

  return into;
}

/********************************************************************/
/* Mouvements d'un axe (en degrés) */
bool move_to_degres(uint8_t servo, int16_t angle) {
  uint16_t pulselen = map(angle, ANGLEMIN[servo], ANGLEMAX[servo], PULSEMIN[servo], PULSEMAX[servo]);
  ANGLE[servo] = angle;
  return move_to_pulse(servo, pulselen);
}

/********************************************************************/
/* Mouvements du bras en ligne droite jusqu'aux coordonnées (x,y) */
void move_to_XY(int16_t x1, int16_t y1) {
  double x0, y0;
  double alpha = to_rad(ANGLE[0]);
  double theta = to_rad(ANGLE[1]);
  angles_to_xy(x0, y0, alpha, theta);

  double dist = sqrt(sq(double(x1)) + sq(double(y1)));
  uint16_t npas = dist/PAS;
  double pasx = (x1-x0)/npas;
  double pasy = (y1-y0)/npas;

  double x = x0;
  double y = y0;

  bool into = true;
  for (uint16_t i=0; i<npas ; i++) {
    xy_to_angles(alpha, theta, x, y);
    into &= move_to_degres(0, to_deg(alpha));
    into &= move_to_degres(1, to_deg(theta));
    x += pasx;
    y += pasy;
    delay(10);
  }
}

/********************************************************************/
/* Calcul (x,y) en fonction des angles */
/* http://www.osrobotics.org/osr/kinematics/inverse_kinematics.html */
void xy_to_angles(double &alpha, double &theta, double &x, double &y) {
  double ad = double(a);
  double bd = double(b);

  double x_y = sq(x)+sq(y);
  double a_b = sq(ad)+sq(bd);
  double a_t = -acos((x_y-a_b)/(2*ad*bd)); // 0 -- PI

  double k1 = ad + bd*cos(a_t);
  double k2 = bd*sin(a_t);
  alpha = atan2(y, x) - atan2(k2, k1);  // -PI/2 -- PI/2
  theta = alpha + a_t; // -3PI/2 -- PI/2

  if (alpha < 0) {
    alpha += PI;
  }

  if (theta < -PI/2) {
    theta += PI;
  }
}

/********************************************************************/
/* Calcul (x,y) en fonction des angles */
void angles_to_xy(double &x, double &y, double &alpha, double &theta) {
  x = double(a)*cos(alpha) + double(b)*cos(theta);
  y = double(a)*sin(alpha) + double(b)*sin(theta);
}

/********************************************************************/
/* Conversions pulse<>degres<>radians */
double to_rad(double deg) {
  return PI*deg/180;
}

double to_deg(double rad) {
  return rad*180/PI;
}

double pulse_to_degres(uint8_t servo, uint16_t pulse) {
  return map(pulse, PULSEMIN[servo], PULSEMAX[servo], ANGLEMIN[servo], ANGLEMAX[servo]);
}

double degres_to_pulse(uint8_t servo, int16_t degres) {
  return map(degres, ANGLEMIN[servo], ANGLEMAX[servo], PULSEMIN[servo], PULSEMAX[servo]);
}

double to_pulse(uint8_t servo) {
  return degres_to_pulse(servo, ANGLE[servo]);
}


/********************************************************************/
/* Calcul des limites (en pulse) */
uint16_t max_servo(uint8_t servo) {
  uint16_t pulse = to_pulse(servo);
  if (servo == 1) {
    pulse = min(PULSEMAX[1], degres_to_pulse(1, ANGLELIM[1])-to_pulse(0)+PULSEMAX[0]);
  } else if (servo == 0) {
    pulse = min(PULSEMAX[0], degres_to_pulse(0, ANGLELIM[1])-to_pulse(1)+PULSEMAX[0]);
  } else {
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
  }
  return pulse;
}


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

/**************************************************************************************************************************/
/* Mesure des angles pour fermeture boucle */
void get_angles() {
  ANGLE_MES[0] = degres(ANGLEPIN[0]);
  ANGLE_MES[1] = degres(ANGLEPIN[1]);
}

const float A = -3.557e-7;
const float B = 3.947e-4;
const float C = 0.3792;
const float D = -34.514;

float degres(int pin) {
  int deg_mes = analogRead(pin);
  return A*deg_mes*deg_mes*deg_mes + B*deg_mes*deg_mes + C*deg_mes + D;
}


