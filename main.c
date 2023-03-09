#include <PID_v1.h>

// Pinovi za senzor HC-SR04
#define echoPin 2 // HC-SR04 echo pin
#define trigPin 3 // HC-SR04 trigger pin

// Motor A pinovi
#define enA 9
#define in1 8
#define in2 7

// Motor B pinovi
#define enB 5
#define in3 6
#define in4 4


// definisanje varijabli
double input;  // udaljenost izmerena senzorom
double output;  // PWM signal koji se salje motorima
double setpoint = 10; // postavljamo setpoint na 10cm
// Ako je objekat na manjoj udaljenosti od 10cm PID povecava brzinu motora
// Ako je objekat na vecoj udaljenost od 10cm PID smanjuje brzinu motora

// PID konstante
double Kp = 100;
double Kd = 1;
double Ki = 1;

// PID instanca
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
// Postavljamo PID u DIRECT mod - ukoliko se output povecava povecava se i input
// Drugim recima, sto je veca brzina motora veca je udaljenost od objekta


// definisanje funkcija
int izmeri_udaljenost() {
  
  // definisanje lokalnih varijabli
  long duration; // variable for the duration of sound wave travel
  int distance; // variable for the distance measurement

  // 'Cistimo' trigger pin na 0
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Postavljamo trigger pin na 1 u trajanju od 10us
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Slusamo echo pin
  duration = pulseIn(echoPin, HIGH);
  
  // Racunamo udaljenost
  distance = duration * 0.034 / 2; // Brzina zvuka podeljena sa 2 (dok signal ode, odbije se i vrati se - dupla putanja)

  return distance;
}

void vozi() {
  
  // output - PWM signal izmedju 0 i 255
  analogWrite(enA, output);
  analogWrite(enB, output);
  
  // Palimo motore A i B unapred
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}


void setup() {

  // Inicijalizacija pinova za HC-SR04 
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

  // Inicijalizacija pinova za Motor A i Motor B
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Inicijalno gasimo motore
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // Inicijalizacija serijske komunikacije
  Serial.begin(9600); 
  Serial.println("INIT");

  input = izmeri_udaljenost();
  setpoint = 10;
  
  // Palimo PID
  myPID.SetMode(AUTOMATIC);
    
  // Postavljamo PID parametre
  myPID.SetTunings(Kp, Ki, Kd);
}

void loop() {
  // Izmeri udaljenost pomocu senzora
  input = (double)izmeri_udaljenost();
  Serial.print("Udaljenost od senzora je ");
  Serial.print(input);
  Serial.print("    i output je");
  Serial.println(output);
  
  myPID.Compute();
  
  vozi();
  
  delay(20);
}