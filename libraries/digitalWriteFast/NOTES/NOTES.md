# Notes

Notes down the observations, experiences and occurrences during the development of the repo.

Useful for explaining certain decisions.


###### speed of digitalWriteFast:

On Arduino Uno clone, 16MHz

digitalWriteFast is so fast that 

```C++
void loop() {
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
}
```

The above did not result in 50% duty cycle. 
![Graph 1](graphs/1.png)

Rather, the pin stays low for quite some time until the program loops again. 

Took around 600ns for the program to repeat


So, the below was done instead:

```C++
void loop() {
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
//	...  and it goes on, multiple times
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
}
```

It only took 250ns to toggle, meaning it took only 250ns/2= **125ns** to set or clear the pin/port.
![Graph 2](graphs/2.png)

###### speed of digitalWrite:

On Arduino Uno clone, 16MHz

It took 12.56us to toggle, meaning it took 12.56us/2= **6280ns** to set or clear the pin/port.

**50 times slower than direct port manipulation!**

![Graph 3](graphs/3.png)

