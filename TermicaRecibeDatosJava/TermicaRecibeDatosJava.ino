#include<SoftwareSerial.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

SoftwareSerial ModBluetooth(11, 10); // RX | TX

#define outputPeltier1 8
#define outputPeltier2 9
#define outputRES1 3

//////////////VARIABLES LECTURA DE DATOS GENERALES DE CONFIGURACION////////////
String incomingString, HoldString, TimeString, NocicleString;
float TNelem, TNelem1, Holdtemp, Holdtime, PrincipalData = 0;
int Nelem, Nocicle;
float Pasos[50]; 
unsigned long TiempoPasos[50];
int cond = 0, cond1 = 0, condt = 0, timeHold = 0, contHold = 0, contcicle;

//////////////VARIABLES-CONSTANTES LECTURA DE SENSOR DE TEMPERATURA////////////
const int Rc = 10000; //valor de la resistencia
const int Vcc = 5;
const int Vcc2 = 5;
const int SensorPIN = A0;
const int SensorPIN2 = A1;
float A = 1.294548729e-3;//1.143335990e-3;
float B = 2.344885576e-4;//2.317987715e-4;
float C = 1.008051067e-7;//0.9682360352e-7;
/*
  float A = 1.734524252e-3;
  float B = 1.772990644e-4;
  float C = -0.1848767565e-7;
*/
float K = 2.5; //factor de disipacion en mW/Ccv
float Val = 0;

////////////////////////////////////VARIABLES DE TIEMPOS////////////////////////////
unsigned long currentMillis = 0, previousMillis = 0;

//////////////////////////////PID coefficient, Values and config ///////////////////
int flag = 0, flagc = 1, flagd = 0, error = 0, error2 = 0, keycont = 0, keycontcicle = 0;
float control1, setpoint = 25.0, control12, setpointx = 60;
float e = 0.0, e_1 = 0.0, e_2 = 0.0, u = 0.0, u_1 = 0.0;
float kp, ti, td, q0, q1, q2, T = 0.1;
float k = 0.075, tao = 30.5, theta = 1;
float e2 = 0.0, e_12 = 0.0, e_22 = 0.0, u2 = 0.0, u_12 = 0.0;
float kp2, ti2, td2, q02, q12, q22, T2 = 0.1;
float k2 = 0.075, tao2 = 30.5, theta2 = 1;

///////////////////////VARIABLES LECTURA DE SENSORES DE TEMPERATURA////////////////
float raw = 0, V = 0, R = 0, logR = 0, R_th = 0, celsius = 0, kelvin = 0, raw2 = 0, V2 = 0, R2 = 0, logR2 = 0, R_th2 = 0, celsius2 = 0, kelvin2 = 0;

void setup()
{
  pinMode(outputPeltier1, OUTPUT);           ////////Salida de control de placa Peltier
  pinMode(outputPeltier2, OUTPUT);           ////////Salida de control de placa Peltier
  pinMode(outputRES1, OUTPUT);               ////////Salida de control de resistencia termica
  ModBluetooth.begin(9600);
  ModBluetooth.println("MODULO CONECTADO");
  ModBluetooth.print("#");
  Serial.begin(9600);                      ////////Inicio de comunicacion serial
  //Serial.setTimeout(50);                     ////////Tiempo de espera para lectura de puerto serie
  
  //////////////////////SINTONIA POR ZIEGLER y NICHOLS/////////////////////////
  kp = (1.2 * tao) / (k * theta);
  ti = 2 * theta;
  td = 0.5 * theta;
  /////////////////////Calculo de control PID digital//////////////////////////
  q0 = kp * (1 + T / (2 * ti) + td / T);
  q1 = -kp * (1 - T / (2 * ti) + (2 * td) / T);
  q2 = (kp * td) / T;
  
  //////////////////////SINTONIA POR ZIEGLER y NICHOLS/////////////////////////
  kp2 = (1.2 * tao2) / (k2 * theta2);
  ti2 = 2 * theta2;
  td2 = 0.5 * theta2;
  /////////////////////Calculo de control PID digital//////////////////////////
  q02 = kp2 * (1 + T2 / (2 * ti2) + td2 / T2);
  q12 = -kp2 * (1 - T2 / (2 * ti2) + (2 * td2) / T2);
  q22 = (kp2 * td2) / T2;
}

void loop()
{
  /////////////////CONVERSION DE VALOR ANALOGICO A TEMPERATURA/////////////////
  raw = analogRead(SensorPIN);
  V =  raw / 1024 * Vcc;
  R = (Rc * V ) / (Vcc - V);
  logR  = log(R);
  R_th = 1.0 / (A + B * logR + C * logR * logR * logR );
  kelvin = R_th - V * V / (K * R) * 1000;
  celsius = kelvin - 273.15;
  
  ////////////////////////////////////////////////////////////////////////////
  raw2 = analogRead(SensorPIN2);
  V2 =  raw2 / 1024 * Vcc2;
  R2 = (Rc * V2 ) / (Vcc2 - V2);
  logR2  = log(R2);
  R_th2 = 1.0 / (A + B * logR2 + C * logR2 * logR2 * logR2 );
  kelvin2 = R_th2 - V2 * V2 / (K * R2) * 1000;
  celsius2 = kelvin2 - 273.15;
  
  ////////////////IMPRIME LA TEMPERATURA ACTUAL Y EL SETPOINT CARGADO//////////
  if (PrincipalData == 1) 
  {
    Serial.println("\nTemp");
    ModBluetooth.print("#");
    Serial.print(celsius);
    ModBluetooth.print("#");
    Serial.println("C");
    ModBluetooth.print("#");
    Serial.print(setpoint);
    ModBluetooth.print("#");
    Serial.println("\n\nTemp 2 ");
    ModBluetooth.print("#");
    Serial.print(celsius2);
    ModBluetooth.print("#");
    Serial.println("C");
    ModBluetooth.print("#");
    Serial.print(setpointx);
    ModBluetooth.print("#");
  }
  
  ////////////////////////////////ENCONTRAR EL ERROR/////////////////////////////
  e = setpoint - celsius;
  ///////////////////////////////////Control PID/////////////////////////////////
  u = u_1 + q0 * e + q1 * e_1 + q2 * e_2;  ////////Ley del controlador PID discreto
  control1 = u * 1000.0 / raw;             ////////Escala de grados a bits para el PWM
  if (u >= 1000.0)                         ////////Saturo la accion de control 'uT' en un tope maximo y minimo
    u = 1000.0;
  if (u <= 0.0)
    u = 0.0;
  e_2 = e_1;
  e_1 = e;
  u_1 = u;
  
  ////////////////////////////////ENCONTRAR EL ERROR 2///////////////////////////
  e2 = setpointx - celsius2;
  ////////////////////////////////////Control PID////////////////////////////////
  u2 = u_12 + q02 * e2 + q12 * e_12 + q22 * e_22;   //Ley del controlador PID discreto
  control12 = u2 * 1000.0 / raw2;                   //Escala de grados a bits para el PWM
  if (u2 >= 1000.0)                                 //Saturo la accion de control 'uT' en un tope maximo y minimo
    u2 = 1000.0;
  if (u2 <= 0.0)
    u2 = 0.0;
  e_22 = e_12;
  e_12 = e2;
  u_12 = u2;
  
  ///////////////////////////////////////////////////////DATOS DE CONFIGURACION PRINCIPAL//////////////////////////////////////////////////////////
  //////////////
  //////////////     95,60000,35,55,45000,72,60000
  //////////////
  if ( PrincipalData == 0)                  /////////CONDICION EJECUTADA UNA VEZ PARA PEDIR DATOS DE CONFIGURACION
  {
    analogWrite(outputPeltier1, 0);             /////Se deshabilita la energizacion de la Peltier
    analogWrite(outputPeltier2, 0);
    digitalWrite(outputRES1, 0);
    if (ModBluetooth.available())                    /////Evita que se pida constantemente los valores requeridos
    {
      incomingString = ModBluetooth.readString();
      Holdtemp = incomingString.substring(0, 2).toFloat();
      Holdtime = incomingString.substring(3, 7).toInt();
      Holdtime = Holdtime * 1000;
      Nocicle = incomingString.substring(8, 10).toInt();
      Nelem = incomingString.substring(11, 13).toInt();

      for(int it = 0; it < Nelem; it++)
      {
        Pasos[it] = incomingString.substring(14+(it*8), 16+(it*8)).toFloat();
        Pasos[it] = Pasos[it] * 1000;
        TiempoPasos[it] = incomingString.substring(17+(it*8), 21+(it*8)).toInt();
      }
      
      PrincipalData = 1;                          /////////CONDICION EJECUTADA UNA VEZ PARA PEDIR DATOS DE CONFIGURACION
      ModBluetooth.println("\nHold Temperature");
      ModBluetooth.print("#");
      ModBluetooth.print(Holdtemp);
      ModBluetooth.print("#");
      ModBluetooth.println("\nHold Timer");
      ModBluetooth.print("#");
      ModBluetooth.print(Holdtime);
      ModBluetooth.print("#");
      ModBluetooth.println("\nCiclos");
      ModBluetooth.print("#");
      ModBluetooth.print(Nocicle);
      ModBluetooth.print("#");
      ModBluetooth.println("\nPasos");
      ModBluetooth.print("#");
      ModBluetooth.print(Nelem);
      ModBluetooth.print("#");
      for(int jt = 0; jt < Nelem; jt++)
      {
        String strUno = "\nTemperatura paso ";
        String strTres = ": ";
        String strDos = strUno+(jt+1)+strTres;
        ModBluetooth.println(strDos);
        ModBluetooth.print("#");
        ModBluetooth.println(Pasos[jt]);
        ModBluetooth.print("#");

        String str1Uno = "\nTiempo paso ";
        String str1Tres = ": ";
        String str1Dos = str1Uno+(jt+1)+str1Tres;
        ModBluetooth.println(str1Dos);
        ModBluetooth.print("#");
        ModBluetooth.println(TiempoPasos[jt]);
        ModBluetooth.print("#");
      }
      delay(5000);
    }
  }
  
  ////////////////////////MANTENER Holdtemperature EL TIEMPO INDICADO POR Holdtime/////////////////////////
  if ((flagc == 1) && (e2 <= 1 && e2 >= -1))
  {
    setpoint = Holdtemp;
    flag = 1;
    flagc = 2;
    flagd = 1;
    e = setpoint - celsius;
  }
  if (e <= 1 && e >= -1)                        /////Una vez que la temperatura leida es cercana al setpoint cargado
    error = 1;                                  /////cambia la bandera error para que pueda entrar al siguiente if
  else
    error = 0;
  if (flag == 1 && error == 1)                  /////Si el error es cercano a 0
  {
    currentMillis = millis();                   /////se carga el tiempo transcurrido al momento
    previousMillis = currentMillis + Holdtime;  /////y se le suma el tiempo introducido por el usuario
    flag = 2;                                   /////cambiando la bandera flag para que ya no la tome en cuenta
  }
  if (flag == 2)                                /////Se revisa la flag para mantener el setpoint el tiempo indicado
  { /////Valida que el tiempo corriente sea menor al tiempo total del paso
    currentMillis = millis();                   /////Manda un mensaje por el puerto serial para validar
    if (previousMillis >= currentMillis) {
      Serial.println("-----------TIEMPO CARGADO-----------");
      ModBluetooth.print("#");
      Serial.print(previousMillis);
      ModBluetooth.print("#");
      Serial.println("\n");
      ModBluetooth.print("#");
      Serial.println("-----------TIEMPO ACTUAL-----------");
      ModBluetooth.print("#");
      Serial.print(currentMillis);
      ModBluetooth.print("#");
      Serial.println("\n");
      ModBluetooth.print("#");
    }
    else                                        /////Si el tiempo corriente es mayor al tiempo total de paso
      flag = 3;                                 /////cambia la bandera y el setpoint por el valor leido en fdata
  }
  
  //////////////////////////////////////CONDICION PARA CARGAR SETPOINT Y TIMEPOINT///////////////////////////////////////////
  if (flag == 3)                                /////Revisa la bandera y asegura que se haya cumplido el control de un setpoint por un tiempo determinado
  { /////antes de cambiar el setpoint
    flag = 1;                                                 /////Cambia la bandera para que se realice la toma de tiempo y se controle el setpoint dicho tiempo
    if (keycont == Nelem)
    {
      if (keycontcicle >= (Nocicle - 1))                      /////Valida que se realicen solo los ciclos indicados por el usuario en Nocicle
      {
        flag = 4;                                             /////Una vez que se realizaron todos los ciclos cambia la bandera para deshabilitar la alimnetacion de la Peltier
      }
      keycont = 0;                                            /////Reinicia el valor del paso para que reinicie el ciclo con el paso i(keycont)
      keycontcicle++;                                         /////Incrementa el contador que indica el numero de ciclo en que se encuentra
    }
    setpoint = Pasos[keycont];                                /////Carga a setpoint el valor de la temperatura cargada previamente en la posicion i(keypoint)
    Holdtime = TiempoPasos[keycont];                          /////Carga a Holdtime el valor del tiempo correspondiente a la temperatura cargada en la posicion i(keypoint)
    keycont++;
  }
  if (flag == 4)                                /////Cuando la bandera es 4 es porque se han realizado los ciclos indicados
  {
    setpoint = 20.0;                              /////Carga un valor de seguridad al setpoint
    digitalWrite(outputPeltier1, 0);             /////Se deshabilita la energizacion de la Peltier
    digitalWrite(outputPeltier2, 0);
    digitalWrite(outputRES1, 0);
  }
  if (PrincipalData == 1)                       /////Revisa si se han cargado los valores de la configuracion inicial
  { /////si es asi, se activa el alentamiento de la resistencia de la placa caliente
    if (celsius < (setpointx - 10) && celsius > (setpoint + 10)) {
      if (celsius2 < setpointx)
        digitalWrite(outputRES1, HIGH);       /////Si es mas fria del setpoint se activa con un valor de control
      if (celsius2 >= setpointx)
        digitalWrite(outputRES1, 0);              /////Si es mas caliente se desactiva con un valor cero.
    }
    else {
      if (celsius2 < setpointx)
        digitalWrite(outputRES1, control12);       /////Si es mas fria del setpoint se activa con un valor de control
      if (celsius2 >= setpointx)
        digitalWrite(outputRES1, 0);
    }
  }
  ///////////*********** Señal PWM al valor de control en pin *********////////////
  if (flagd == 1) {
    if (celsius < (setpoint - 5) || celsius > (setpoint + 5))   /////Permite mandar el pulso completo hasta que se acerque al setpoint
    {
      if (celsius < setpoint)                       /////Activa la placa Peltier para que caliente el lado superior y enfrie el inferior
      {
        digitalWrite(outputPeltier1, HIGH);
        digitalWrite(outputPeltier2, 0);
      }
      if (celsius > setpoint) {                     /////Activa la placa Peltier para que caliente el lado inferior y enfrie el superior
        digitalWrite(outputPeltier1, 0);
        digitalWrite(outputPeltier2, HIGH);
      }
    }
    else {                                          /////Una vez que el valor se acerca al setpoint se activa el control PID
      if (celsius < setpoint)                       /////Activa la placa Peltier para que caliente el lado superior y enfrie el inferior
      {
        analogWrite(outputPeltier1, control1);
        analogWrite(outputPeltier2, 0);
      }
      if (celsius > setpoint) {                     /////Activa la placa Peltier para que caliente el lado inferior y enfrie el superior
        analogWrite(outputPeltier1, 0);
        analogWrite(outputPeltier2, control1);
      }
    }
  }
}
