#define gps Serial1

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
  gps.begin(9600); 
  pinMode(6, OUTPUT); 
}


void loop() {
  // put your main code here, to run repeatedly:
  while(gps.available()){
    Serial.print((char)gps.read()); 
  }
  digitalWrite(6, LOW); 
}
