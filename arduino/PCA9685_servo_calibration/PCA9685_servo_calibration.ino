/*************************************************** 
  Programme pour calibrer les servomoteurs
  avec retour d'info
  pilotés par PCA9685

 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

/********************************************************************/
/* Angles limites des bras et avant-bras du robot */
const float  DEGREMIN = -65; 
const float  DEGREMAX = 19; 

/* Sens des moteurs */
const int16_t  SENS = 1; 

/* Pente des moteurs (rapport pulsation/angle) */
const float  PENTE = 2; 

/* Pulsation à l'angle DEGREMIN (à déterminer expérimentalement) */
const int16_t PULSEMIN = 331; // en pulse


/* Longueurs de pulsation (pulse) et angles limites */
/* ATTENTION : différent pour chaque robot !!! */
const uint16_t  PULSEMAX = SENS*PENTE*(DEGREMAX - DEGREMIN) + PULSEMIN; //{400, 500}; // This is the 'maximum' pulse length count (out of 4096)


/********************************************************************/
/* Ports utilisés */
const uint8_t SERVOPIN = 0;
const uint8_t ANGLEPIN = A3;

/********************************************************************/
/* Paramètres caractéristiques du servomoteur pour mesure angle */
const float A = -3.557e-7;
const float B = 3.947e-4;
const float C = 0.3792;
const float D = -34.514;

/********************************************************************/
/* État initial */
float ANGLE = 0;
float ANGLE_MES = ANGLE; // Angles mesurés

#define PI 3.1415926535897932384626433832795



/********************************************************************/
/********************************************************************/
void setup() {
  Serial.begin(460800);

  Serial.println("EezyBot Servo !");
  Serial.setTimeout(500);

  pwm.begin();
  pwm.setOscillatorFrequency(24460000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(100);
}


/********************************************************************/
/********************************************************************/
void loop() {
   for (uint16_t pulselen = PULSEMIN; pulselen < PULSEMAX; pulselen++) {
    pwm.setPWM(SERVOPIN, 0, pulselen);
    delay(200);
    Serial.print(pulselen);
    Serial.print("\t");
    Serial.println(analogRead(ANGLEPIN));
  }

  for (uint16_t pulselen = PULSEMAX; pulselen > PULSEMIN; pulselen--) {
    pwm.setPWM(SERVOPIN, 0, pulselen);
    delay(10);
  }
  delay(5000);
}

/********************************************************************/
/* envoi des positions */
void send_pulses() {
  Serial.print("_pp ");
  Serial.println(to_pulse());
}

void send_angles() {
  Serial.print("_aa ");
  Serial.println(ANGLE);
}

void send_angle(uint8_t servo) {
  Serial.print("_a");
  Serial.print(servo);
  Serial.print(" ");
  Serial.println(ANGLE);
  //Serial.flush();
}



/********************************************************************/
/* Mouvements d'un axe (en pulse) */
bool move_to_pulse(uint16_t pulselen) {
  ANGLE = pulse_to_degres(pulselen);
  pwm.setPWM(SERVOPIN, 0, pulselen);
}

/********************************************************************/
/* Mouvements d'un axe (en degrés) */
bool move_to_degres(int16_t angle) {
  uint16_t pulselen = map(angle, DEGREMIN, DEGREMAX, PULSEMIN, PULSEMAX);
  ANGLE = angle;
  return move_to_pulse(pulselen);
}


/********************************************************************/
/* Conversions pulse<>degres<>radians */
double to_rad(double deg) {
  return PI*deg/180;
}

double to_deg(double rad) {
  return rad*180/PI;
}

double pulse_to_degres(uint16_t pulse) {
  return map(pulse, PULSEMIN, PULSEMAX, DEGREMIN, DEGREMAX);
}

double degres_to_pulse(int16_t degres) {
  return map(degres, DEGREMIN, DEGREMAX, PULSEMIN, PULSEMAX);
}

double to_pulse() {
  return degres_to_pulse(ANGLE);
}



/********************************************************************/
/* Mesure des angles des servomoteurs */

float degres(int val) {
  return A*val*val*val + B*val*val + C*val + D;
}

void send_angles_mes() {
  float a = ANGLE_MES;
  String aa = "N";

  int16_t m = analogRead(ANGLEPIN);
  if (m > 0) { // port connecté = mesure
    a = degres(m) * SENS;
    aa = String(a);
  }

  uint16_t t = millis();
  if ((a != ANGLE_MES)) {
    ANGLE_MES = a;
    Serial.print("_am ");
    Serial.println(aa);
  }
}