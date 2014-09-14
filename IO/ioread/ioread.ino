
bool start=1;

#define pin_PWM 13
#define pin_read 2

void setup() {
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_read, INPUT);
  analogWrite(pin_PWM,10);
  Serial.begin(9600);
  Serial.println("Started");
}
extern volatile unsigned long timer0_millis;
void loop() {
  
  unsigned long i = 0; // test value
  unsigned long stop_time; // in milliseconds
  // calculate stop time (current time + 1 second)
  stop_time = millis() + 1000;
  int last=0;
  unsigned long pcount=0;
  unsigned long ncount=0;
  while(timer0_millis < stop_time) {
    int d=digitalRead(pin_read);
    if (d==0)
        ncount++;
      else
        pcount++;
     
    if(d!=last) {
      i++;
      last=d;
    }
  }
  
  
  // report performance results:
  // number of loop iterations in one second
  Serial.print(i);
  Serial.println(" loops in one second.");
  if(i>0) {
    int p=pcount/i;
    int n=ncount/i;
    Serial.println(p);
    Serial.println(n);
    Serial.println(256*p/(p+n));
  }
  else 
    Serial.println("No signal found");
  
  while(1); // and stop here
}
