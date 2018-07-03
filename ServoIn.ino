
float microsecondsToDegrees( long microseconds );
void setOutVal( float val );
void setOutValFromInVal( float val );


void ICACHE_RAM_ATTR handleServoInChange() {
  boolean state = digitalRead(SERVO_IN_PIN);
  if( state )
    servoInOnUS = micros();
  else
    servoInOffUS = micros();
}

void ignoreCurrentPulse()
{
  servoInOnUS = -1l;
}
void setupServoIn()
{
  pinMode(SERVO_IN_PIN, INPUT); 
  attachInterrupt(SERVO_IN_PIN, handleServoInChange, CHANGE); 
}

boolean gotServoIn()
{
  return servoInOnUS > -1l && servoInOffUS > -1L;
}

void loopServoIn()
{
  long now = micros();

  //Serial.print( servoInOnUS );
  //Serial.print("  ");
  //Serial.println( servoInOffUS );
  
  if( servoInOnUS >0L && now - servoInOnUS > servoTimeoutUS )
  {
    // timeout, forget the old values
    servoInOnUS = -1L;
    servoInOffUS = -1L;
  }
  
  if( servoInOnUS > -1l && servoInOffUS > -1L )
  {
    if( servoInOffUS > servoInOnUS ) // between pulses
    {
      servoInUS = servoInOffUS - servoInOnUS;
      servoInDegrees = microsecondsToDegrees( servoInUS );

      setOutValFromInVal(servoInDegrees);
      //Serial.print( servoInUS );
      //Serial.print("  ");
      //Serial.println( servoInDegrees );
    }
   // else we are the middle of a pulse, don't update the time
  }
  else
  {
    servoInDegrees = -359.0;  
    Serial.println( servoInDegrees );
  }
}

