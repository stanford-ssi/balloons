#define gps Serial3

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  gps.begin(9600); 
}

void loop() {
  // put your main code here, to run repeatedly:
  while(gps.available()){
    Serial.print(gps.read()); 
  }
}
