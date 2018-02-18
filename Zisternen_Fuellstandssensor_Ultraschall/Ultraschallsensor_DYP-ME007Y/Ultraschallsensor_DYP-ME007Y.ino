#include <SoftwareSerial.h>

SoftwareSerial mySerial =  SoftwareSerial(10,11); 

#define echoPin 10
#define trigPin 11

unsigned int reading;
byte readByte;
byte read_buffer[4]; 
byte crcCalc;
word distance;
String outText;

void setup() {
  //
  // Настройка портов
  //
  mySerial.begin (9600);

  Serial.begin (9600);
  Serial.println("start");
  //
  // Очищаем буфер чтения
  //
  for (byte loopstep = 0; loopstep <= 3; loopstep++) {
    read_buffer[loopstep] = 0;
  }
}

void loop() {
  //
  // Проверка наличия данных в COM порту
  //
  if (mySerial.available() < 1) {
    return;  
  }
  //
  // Читаем в буфер
  //
  readByte = mySerial.read();
  
  for (byte loopstep = 0; loopstep <= 2; loopstep++) {
    read_buffer[loopstep] = read_buffer[loopstep + 01];
  }
  
  read_buffer[03] = readByte;   
  //
  // Анализ буфера
  //
  if (read_buffer[00] != 0xff) {
    return; // это не начало данных 
  };
  
  crcCalc = read_buffer[00] + read_buffer[01] + read_buffer[02];
  if (read_buffer[03] != crcCalc) {
    return; // контрольная сумма пакета данных не совпала
  };
  //
  // расчет расстояния
  //
  distance = (read_buffer[01] * 0xff) + read_buffer[02];
  //
  // вывод
  //
  outText = "bytes: ";
  outText = String(outText + read_buffer[00]);
  outText = String(outText + "+");
  outText = String(outText + read_buffer[01]);
  outText = String(outText + "+");
  outText = String(outText + read_buffer[02]);
  outText = String(outText + "+");
  outText = String(outText + read_buffer[03]);
  outText = String(outText + " = ");
  outText = String(outText + distance);
  outText = String(outText + " mm");
  
  Serial.println(outText);
  //
  // пауза - 1 секунда, можно любое значение
  //
  delay(1000);
  //
  // удаляем из буфера данные, которые во время паузы туда залезли
  //
  while (mySerial.available() > 0) {
     readByte = mySerial.read();
  }
}




