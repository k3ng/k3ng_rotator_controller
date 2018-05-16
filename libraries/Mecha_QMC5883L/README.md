# Mechasolution QMC5883L Library

[한글 설명 바로가기](https://github.com/keepworking/Mecha_QMC5883/blob/master/README_KO.md)


## Arduino Code

There are a few simple rules for using that library. Please read the following summary and apply it to your project

### Basic Elements

Required header files (#include ...) and Setup side code.

```cpp
#include <Wire.h>
#include <MechaQMC5883.h>

void setup(){
  Wire.begin();
}
```

### Object Declaration

The object declaration method. It is used outside the setup statement, and a name such as qmc can be changed to any other name you want.

```cpp
#include <Wire.h>
#include <MechaQMC5883.h>

MechaQMC5883 qmc;
```

### initialization

QMC5883 Sensor's setting function.

The init function allows you to take advantage of the features of the QMC5883 sensor by default.

```cpp
void setup(){
    Wire.begin();
    qmc.init();
}
```

If you want more detailed settings, you can use it as follows.

```cpp
void setup(){
  Wire.begin();
  qmc.init();
  qmc.setMode(Mode_Standby,ODR_200Hz,RNG_8G,OSR_512);
}
```

The values ​​used for setMode can take the following values:

```
Mode : Mode_Standby / Mode_Continuous

ODR : ODR_10Hz / ODR_50Hz / ODR_100Hz / ODR_200Hz
ouput data update rate

RNG : RNG_2G / RNG_8G
magneticfield measurement range

OSR : OSR_512 / OSR_256 / OSR_128 / OSR_64
over sampling rate
```

### Read values

How to read the measured sensor value is as follows.

```cpp
void loop(){
  int x,y,z;

  qmc.read(&x,&y,&z);
}
```

and we can get azimuth too.

```cpp
void loop(){
  int x,y,z;
  int a;
  //float a; //can get float value

  qmc.read(&x,&y,&z,&a);
}
```

also can claculate azimuth you want

```cpp
void loop(){
  int x,y,z;
  int a;

  qmc.read(&x,&y,&z);
  a = qmc.azimuth(&y,&x);
}
```

## Basic example

It can be seen as a collection of the contents mentioned above.

```cpp
#include <Wire.h>
#include <MechaQMC5883.h>

MechaQMC5883 qmc;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  qmc.init();
  //qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);
}

void loop() {
  int x,y,z;
  qmc.read(&x,&y,&z);

  Serial.print("x: ");
  Serial.print(x);
  Serial.print(" y: ");
  Serial.print(y);
  Serial.print(" z: ");
  Serial.print(z);
  Serial.println();
  delay(100);
}
```
