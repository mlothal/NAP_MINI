/* NVQ_PLC ROBOT PENETRÓMETRO version 1.0
*/
#include <ParserLib.h>
#include <Wire.h>
Parser parser;
// Sudelay_x#udelay_z#dist_frutas#num_frutas#udelay_y
// S100#100#1000#15#100\n
// ================ PINOUT =================== //
#define SONDA 2
#define PULX 3  // Pin del pulso del motor
#define DIRX 4  // Pin de la dirección del motor
//motores Y
#define PULY1 5  // Pin del pulso del motor
#define DIRY1 6  // Pin de la dirección del motor
#define PULY2 7  // Pin del pulso del motor
#define DIRY2 8  // Pin de la dirección del motor
//motor Z
#define PULZ 9  // Pin del pulso del motor
#define DIRZ 10  // Pin de la dirección del motor
//habilitaciones
#define ENAY 11 // Pin de habilitación del motor
#define ENAX 12 // Pin de habilitación del motor
//endstops
#define PARADA 13
#define ZMAX 14
#define ZMIN 15 
#define XMAX 16 // Pin del final de carrera XMAX
#define XMIN 17 // Pin del final de carrera XMIN
#define YMAX 18
#define YMIN 19
#define FRUTAS_POR_FILA 11
// ================ VARIABLES ================== //
int FLAG_STATE = 0;

long HOME_PULSES = 1000600;
long UP_PULSES = 35000;
long DOWN_PULSES = 5000;
long ABAJO_PULSES = 9000;
long ARRIBA_PULSES = 9000;
long udelaySensor = 1000;
long dist_frutas = 1000;
long FRONT_PULSES = 1000;
long FRONTAL = 200;
long BACK_PULSES = 30000;
long udelay_y = 100; // Retardo en microsegundos
long udelay_x = 100; 
long udelay_z = 70; 
int num_frutas = 5;
long ajuste = 3360;
int fino = 1200;

// "<=ERROR=>"
String command = " ";
String cini = "<=";
String cend = "=>";
String p_reader = "";
// =============== FUNCTIONS =================== //
// Sudelay_x#udelay_z#dist_frutas#num_frutas#udelay_y
// S100#100#1000#15#100\n

//funcion ARRIBA hasta final de carrera
void HOMING_UP(long UP_PULSES, long udelay_z){
    digitalWrite(DIRZ, HIGH); // Dirección ARRIBA
    digitalWrite(ENAY, HIGH); // Habilita el motor
    digitalWrite(ENAX, HIGH);
  for (long i = 0; i < UP_PULSES; i++) {
    digitalWrite(PULZ, HIGH);
    delayMicroseconds(udelay_z);
    digitalWrite(PULZ, LOW);
    delayMicroseconds(udelay_z);
    if (digitalRead(ZMIN) == HIGH) {
      break;}
    if (digitalRead(PARADA) == LOW) {
      break;}
}}
void RIGHT(long ajuste, long udelay_x) {      // FUNCIÓN DE DESPLAZAMIENTO A LA DERECHA de ajuste  
  digitalWrite(DIRX, HIGH); // Dirección derecha
  digitalWrite(ENAX, HIGH); // Habilita el motor
  digitalWrite(ENAY, HIGH);
  for (long i = 0; i < ajuste; i++) {
    digitalWrite(PULX, HIGH);
    delayMicroseconds(udelay_x);
    digitalWrite(PULX, LOW);
    delayMicroseconds(udelay_x);
    if (digitalRead(XMAX) == HIGH) {
      break;}
    if (digitalRead(PARADA) == LOW) {
      break;}
}}
// FUNCIÓN DOWN DE COMIENZO DE SECUENCIA
void DOWNSTART(int DOWN_PULSES, long udelay_z) {
    digitalWrite(DIRZ, LOW); // Dirección ABAJO
    digitalWrite(ENAY, HIGH); // Habilita el motor
    digitalWrite(ENAX, HIGH);
  for (long i = 0; i < DOWN_PULSES; i++) {
    digitalWrite(PULZ, HIGH);
    delayMicroseconds(udelay_z);
    digitalWrite(PULZ, LOW);
    delayMicroseconds(udelay_z);
    if (digitalRead(ZMAX) == HIGH) {
      break;}
    if (digitalRead(PARADA) == LOW) {
      break;}
    if (digitalRead(SONDA) == HIGH) {
    break;}      
}}
// FUNCIÓN SUBIDA
void UP(long ARRIBA_PULSES, long udelay_z) {
  digitalWrite(DIRZ, HIGH); // Dirección ARRIBA
  digitalWrite(ENAY, HIGH); // Habilita el motor
  digitalWrite(ENAX, HIGH);
  for (long i = 0; i < ARRIBA_PULSES; i++) {
    digitalWrite(PULZ, HIGH);
    delayMicroseconds(udelay_z);
    digitalWrite(PULZ, LOW);
    delayMicroseconds(udelay_z);
    if (digitalRead(ZMIN) == HIGH) { 
      break;}
    if (digitalRead(PARADA) == LOW) {
      break;}
}}
// FUNCIÓN DE BAJADA EN LOS CICLOS
void DOWN(int ABAJO_PULSES, long udelay_z) {
  digitalWrite(DIRZ, LOW); // Dirección ABAJO
  digitalWrite(ENAY, HIGH); // Habilita el motor
  digitalWrite(ENAX, HIGH);
  for (long i = 0; i < ABAJO_PULSES; i++) {
    digitalWrite(PULZ, HIGH);
    delayMicroseconds(udelay_z);
    digitalWrite(PULZ, LOW);
    delayMicroseconds(udelay_z);
    if (digitalRead(ZMAX) == HIGH) {
      break;}
    if (digitalRead(PARADA) == LOW) {
      break;}
    if (digitalRead(SONDA) == HIGH) {
      break;}
}}
void DER(long dist_frutas, long udelay_x) {
  digitalWrite(DIRX, HIGH); // Dirección derecha
  digitalWrite(ENAX, HIGH); // Habilita el motor
  digitalWrite(ENAY, HIGH);
  for (long i = 0; i < dist_frutas; i++) {
    digitalWrite(PULX, HIGH);
    delayMicroseconds(udelay_x);
    digitalWrite(PULX, LOW);
    delayMicroseconds(udelay_x);
    if (digitalRead(XMAX) == HIGH) {
      break;}
    if (digitalRead(PARADA) == LOW) {
      break;}
}}
void IZQ(long HOME_PULSES, long udelay_x) {
  digitalWrite(DIRX, LOW); // Dirección izquierda
  digitalWrite(ENAX, HIGH); // Habilita el motor
  digitalWrite(ENAY, HIGH);
  for (long i = 0; i < HOME_PULSES; i++) {
    digitalWrite(PULX, HIGH);
    delayMicroseconds(udelay_x);
    digitalWrite(PULX, LOW);
    delayMicroseconds(udelay_x);
    if (digitalRead(XMIN) == HIGH) { 
      break;}
    if (digitalRead(PARADA) == LOW) {
      break;}
}}
void ADELANTE(long FRONT_PULSES, long udelay_y) {
    digitalWrite(DIRY1, LOW); // Dirección ADELANTE
    digitalWrite(DIRY2, HIGH);
    digitalWrite(ENAX, HIGH); // Habilita el motor
    digitalWrite(ENAY, HIGH);
  for (long i = 0; i < FRONT_PULSES; i++) {
    digitalWrite(PULY1, HIGH);
    digitalWrite(PULY2, HIGH);
    delayMicroseconds(udelay_y);
    digitalWrite(PULY1, LOW);
    digitalWrite(PULY2, LOW);
    delayMicroseconds(udelay_y);
    if (digitalRead(YMAX) == HIGH) {
      break;}
    if (digitalRead(PARADA) == LOW) {
      break;}
}}
void ATRAS(long BACK_PULSES, long udelay_y) {
    digitalWrite(DIRY1, HIGH); // Dirección ATRAS
    digitalWrite(DIRY2, LOW);
    digitalWrite(ENAX, HIGH); // Habilita el motor
    digitalWrite(ENAY, HIGH);
  for (long i = 0; i < BACK_PULSES; i++) {
    digitalWrite(PULY1, HIGH);
    digitalWrite(PULY2, HIGH);
    delayMicroseconds(udelay_y);
    digitalWrite(PULY1, LOW);
    digitalWrite(PULY2, LOW);
    delayMicroseconds(udelay_y);
    if (digitalRead(YMIN) == HIGH) { 
      break;}
    if (digitalRead(PARADA) == LOW) {
      break;}
}}

void AJUSTE_FRONTAL(long FRONTAL, long udelay_y) {
    digitalWrite(DIRY1, LOW); // Dirección ADELANTE
    digitalWrite(DIRY2, HIGH);
    digitalWrite(ENAX, HIGH); // Habilita el motor
    digitalWrite(ENAY, HIGH);
  for (long i = 0; i < FRONTAL; i++) {
    digitalWrite(PULY1, HIGH);
    digitalWrite(PULY2, HIGH);
    delayMicroseconds(udelay_y);
    digitalWrite(PULY1, LOW);
    digitalWrite(PULY2, LOW);
    delayMicroseconds(udelay_y);
    if (digitalRead(YMAX) == HIGH) {
      break;}
    if (digitalRead(PARADA) == LOW) {
      break;}
}}
void sensor(long udelaySensor, int fino){ 
  for (int i=0; i<udelaySensor; i++){    //PENETROMETRO TOMA LECTURA
  digitalWrite(DIRZ,LOW);
  digitalWrite(ENAX,HIGH);
  digitalWrite(ENAY,HIGH);
  digitalWrite(PULZ,HIGH);
  delayMicroseconds(fino);
  digitalWrite(PULZ,LOW);
  delayMicroseconds(fino);
  if (digitalRead(ZMAX) == HIGH) {
    break;}
  if (digitalRead(PARADA) == LOW) {
      break;}
}}
// ================== SETUP ==================== //
void setup()
{
  Wire.begin();
  Serial.begin(115200);

  pinMode(PULX, OUTPUT);
  pinMode(DIRX, OUTPUT);
  pinMode(PULY1, OUTPUT);
  pinMode(DIRY1, OUTPUT);
  pinMode(PULY2, OUTPUT);
  pinMode(DIRY2, OUTPUT);
  pinMode(PULZ, OUTPUT);
  pinMode(DIRZ, OUTPUT);
  pinMode(ENAY, OUTPUT);
  pinMode(ENAX, OUTPUT);
  
  pinMode(XMIN, INPUT);
  pinMode(XMAX, INPUT);
  pinMode(YMIN, INPUT);
  pinMode(YMAX, INPUT);
  pinMode(ZMIN, INPUT);
  pinMode(ZMAX, INPUT);
  pinMode(PARADA, INPUT);
  pinMode(SONDA, INPUT);  
}
//====================================LOOP========================================//
void loop(){
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    switch (inByte)
    {
      // Sudelay_x#udelay_z#dist_frutas#num_frutas#udelay_y
      case 'S':
        p_reader = Serial.readStringUntil('\n');
        if (p_reader.length() > 0) {
          parser.Init(p_reader);
          udelay_x = parser.Read_Int16();
          parser.Skip(1);
          udelay_z = parser.Read_Int16();
          parser.Skip(1);
          dist_frutas = parser.Read_Int16();
          parser.Skip(1);
          num_frutas = parser.Read_Int16();
          parser.Skip(1);
          udelay_y = parser.Read_Int16();
          // Añadir código penetrómetro
          FLAG_STATE = 1; // START HOMMING
    }
    break;
    }}
  // ====== STATE MACHINE =========//
  if (FLAG_STATE == 1){
    command = cini+ FLAG_STATE + cend;
    Serial.println(command);
// secuencia de homing antes de los ciclos    
HOMING_UP(UP_PULSES, udelay_z);
ATRAS( BACK_PULSES, udelay_y);
IZQ(HOME_PULSES, udelay_x);
RIGHT(ajuste, udelay_x);
    delay(500);
AJUSTE_FRONTAL(FRONTAL, udelay_y);
    delay(500);
DOWNSTART(DOWN_PULSES, udelay_z);
//Serial.println("<=HOME=>");
FLAG_STATE = 2; // START EXPERIMENT 
} 
// comienza el ciclo
  else if (FLAG_STATE == 2) {
  for (int ciclo = 0; ciclo < num_frutas; ciclo++) {
    command = cini + FLAG_STATE + cend;
    Serial.println(command);
    DOWN(ABAJO_PULSES, udelay_z);
    sensor(udelaySensor, fino);
    if (digitalRead(PARADA) == LOW) {
      break; 
    }
    delay(1000);
    UP(ARRIBA_PULSES, udelay_z);
    if (digitalRead(PARADA) == LOW) {
      break; 
    }
    delay(1000);    
    // Avanza a la siguiente posición en la fila
    if ((ciclo + 1) % FRUTAS_POR_FILA != 0 || (ciclo + 1) == num_frutas) {
      DER(dist_frutas, udelay_x);
    }

    // Verifica si se ha alcanzado un múltiplo de 11 frutas
    if ((ciclo + 1) % FRUTAS_POR_FILA == 0 && ciclo + 1 < num_frutas) {
      // Si es así, realiza la secuencia de movimiento a la siguiente fila
      HOMING_UP(UP_PULSES, udelay_z);
      IZQ(HOME_PULSES, udelay_x);
      ADELANTE(FRONT_PULSES, udelay_y);
      RIGHT(ajuste, udelay_x);
      DOWN(DOWN_PULSES, udelay_z);
    }
  }
//Serial.println("<=HOMMING=>");
FLAG_STATE = 3; // HOME AT END
}
  else if (FLAG_STATE == 3){
HOMING_UP(UP_PULSES, udelay_z);
    delay(500);
ATRAS( BACK_PULSES, udelay_y);
    delay(500);
IZQ(HOME_PULSES, udelay_x);
    command = cini+ FLAG_STATE + cend;
    Serial.println(command);
    FLAG_STATE = 0; // END EXPERIMENT
  } 
  else{
    command = cini + FLAG_STATE + cend;
    Serial.println(command);
  }
  digitalWrite(ENAX, LOW);
  digitalWrite(ENAY, LOW);
  // Serial.print("Distancia Frutas: ");
  // Serial.println(dist_frutas);
  // Serial.print("DelayX: ");
  // Serial.println(udelay_X);
  // Serial.print("DelayY: ");
  // Serial.println(udelay_Z);
  // Serial.print("Numero de Frutas: ");
  // Serial.println(num_frutas);
}

 //           if (digitalRead(PARADA) == LOW) {
   //   break; 
 //}
