# Mechasolution QMC5883 Library

HMC5883 지자기 나침반 센서의 수명 만료(EOL)로인해 그동안의 HMC5883 센서는 생산이 중단 되었고, 대체 상품인 QMC5883으로 변경이 되었습니다.

## Arduino Code

해당 라이브러리를 이용하기 위한 몇가지 간단한 규칙이 있습니다. 아래 정리된 내용을 읽어주시고 사용하시는 프로젝트에 적용해 주세요

### 기본 요소

필수적으로 필요한 헤더파일(#include...)과 Setup 쪽 코드 입니다.

```cpp
#include <Wire.h>
#include <MechaQMC5883.h>

void setup(){
  Wire.begin();
}
```

### 객체 선언

객체 선언 방식입니다. setup문 밖에서 사용이 되며 qmc와 같은 이름은 사용자가 원하는 다른 이름으로 변경이 가능합니다.

```cpp
#include <Wire.h>
#include <MechaQMC5883.h>

MechaQMC5883 qmc;
```

### 사용 설정

QMC5883 센서의 설정 함수 입니다.

init 함수를 사용하면 기본 설정으로 QMC5883센서의 기능을 이용할 수 있습니다.

```cpp
void setup(){
    Wire.begin();
    qmc.init();
}
```

좀더 세세한 설정을 원한다면 다음과 같이 사용이 가능합니다.

```cpp
void setup(){
  Wire.begin();
  qmc.init();
  qmc.setMode(Mode_Standby,ODR_200Hz,RNG_8G,OSR_512);
}
```

setMode에 사용되는 값들은 다음 값들을 이용할 수 있습니다.

```
Mode : Mode_Standby / Mode_Continuous

ODR : ODR_10Hz / ODR_50Hz / ODR_100Hz / ODR_200Hz
ouput data update rate

RNG : RNG_2G / RNG_8G
magneticfield measurement range

OSR : OSR_512 / OSR_256 / OSR_128 / OSR_64
over sampling rate
```

### 값 읽기

측정한 센서의 값을 읽는 법은 다음과 같습니다.

```cpp
void loop(){
  int x,y,z;

  qmc.read(&x,&y,&z);
}
```

방위각에 대한 값입니다.

```cpp
void loop(){
  int x,y,z;
  int a;
  //float a; //float 형도 지원됩니다.

  qmc.read(&x,&y,&z,&a);
}
```

별도로 원하는 방위각도 구할 수 있습니다.

```cpp
void loop(){
  int x,y,z;
  int a;

  qmc.read(&x,&y,&z);
  a = qmc.azimuth(&y,&x);
}
```
### 기본 예제

다음은 라이브러리 기본 예제인 raw입니다.

위에 소개된 내용의 총집합으로 볼 수 있습니다.

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
