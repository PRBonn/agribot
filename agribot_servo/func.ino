bool Toggle_led(int times,int _delay){
  if(times == -1){
    digitalWrite(led_pin, HIGH-digitalRead(led_pin));   // blink the led
  }else{
    for(int i =0; i<times; i++){
      digitalWrite(led_pin, HIGH-digitalRead(led_pin));   // blink the led
      delay(_delay);
      digitalWrite(led_pin, HIGH-digitalRead(led_pin));   // blink the led
      delay(_delay);
    }
  }
}

bool trun_right(int degree){
    digitalWrite(DirOut_right, HIGH);
    analogWrite(PulOut_right, PWM_value);
    delay((int)(degree*Coef));
    analogWrite(PulOut_right, 0);
}

bool trun_left(int degree){
    digitalWrite(DirOut_left, HIGH);
    analogWrite(PulOut_left, PWM_value);
    delay((int)(degree*Coef));
    analogWrite(PulOut_left, 0);
}

void read_sensors(){
  sensorValue_left = analogRead(analogIn_left);
  //sensorValue_right = map(sensorValue_left, 0, 1023, 0, 255);
  sensorValue_right = analogRead(analogIn_right);
  //sensorValue_right = map(sensorValue_right, 0, 1023, 0, 255);
  //sensorValue_left = sensorValue_left*0.1+ old_sensorValue_left * 0.9;
  //sensorValue_right = sensorValue_right*0.1+ old_sensorValue_right * 0.9;
}

void print_val(){
    Serial.print("sensor_left = ");
    Serial.print(sensorValue_left);
    Serial.print("\t sensor_right = ");
    Serial.println(sensorValue_right);
}

bool set_position_right(int goal_pose_right){
  read_sensors();
  if(sensorValue_right>goal_pose_right+max_step+telorance || sensorValue_right<goal_pose_right-max_step-telorance){
    analogWrite(PulOut_right, 0);
    b_right = false;
  }else if(sensorValue_right<goal_pose_right+telorance && sensorValue_right>goal_pose_right-telorance){
    analogWrite(PulOut_right, 0);
    b_right =  true;
  }else if(sensorValue_right > goal_pose_right){
    digitalWrite(DirOut_right, LOW);
    analogWrite(PulOut_right, PWM_value);
    b_right = false;
  }else if(sensorValue_right < goal_pose_right){
    digitalWrite(DirOut_right, HIGH);
    analogWrite(PulOut_right, PWM_value);
    b_right =  false;
  }
  return b_right;
}

bool set_position_left(int goal_pose_left){
  read_sensors();
  if(sensorValue_left>goal_pose_left+max_step+telorance || sensorValue_left<goal_pose_left-max_step-telorance){
    analogWrite(PulOut_left, 0);
    b_left = false;
  }else if(sensorValue_left<goal_pose_left+telorance && sensorValue_left>goal_pose_left-telorance){
    analogWrite(PulOut_left, 0);
    b_left =  true;
  }else if(sensorValue_left > goal_pose_left){
    digitalWrite(DirOut_left, LOW);
    analogWrite(PulOut_left, PWM_value);
    b_left =  false;
  }else if(sensorValue_left < goal_pose_left){
    digitalWrite(DirOut_left, HIGH);
    analogWrite(PulOut_left, PWM_value);
    b_left =  false;
  }
  return b_left;
}
bool intial_position_both(int goal_pose_right, int goal_pose_left){
  if(Servo_stop){
    analogWrite(PulOut_right, 0);
    analogWrite(PulOut_left, 0);
    Servo_stop = false;
    return false;
  }else{
    read_sensors();
    if(sensorValue_right<goal_pose_right+telorance && sensorValue_right>goal_pose_right-telorance){
      analogWrite(PulOut_right, 0);
      b_right =  true;
    }else if(sensorValue_right > goal_pose_right){
      digitalWrite(DirOut_right, LOW);
      analogWrite(PulOut_right, PWM_value);
      b_right = false;
    }else if(sensorValue_right < goal_pose_right){
      digitalWrite(DirOut_right, HIGH);
      analogWrite(PulOut_right, PWM_value);
      b_right =  false;
    }
    read_sensors();
    if(sensorValue_left<goal_pose_left+telorance && sensorValue_left>goal_pose_left-telorance){
      analogWrite(PulOut_left, 0);
      b_left =  true;
    }else if(sensorValue_left > goal_pose_left){
      digitalWrite(DirOut_left, LOW);
      analogWrite(PulOut_left, PWM_value);
      b_left =  false;
    }else if(sensorValue_left < goal_pose_left){
      digitalWrite(DirOut_left, HIGH);
      analogWrite(PulOut_left, PWM_value);
      b_left =  false;
    }
    
    if(b_right && b_left){
      return true;
    }
  }
}

bool set_position_both(int goal_pose_right, int goal_pose_left){
  if(Servo_stop){
    analogWrite(PulOut_right, 0);
    analogWrite(PulOut_left, 0);
    Servo_stop = false;
    return false;
  }else{
    read_sensors();
    if(sensorValue_right>goal_pose_right+max_step+telorance || sensorValue_right<goal_pose_right-max_step-telorance){
      analogWrite(PulOut_right, 0);
      b_right = false;
    }else if(sensorValue_right<goal_pose_right+telorance && sensorValue_right>goal_pose_right-telorance){
      analogWrite(PulOut_right, 0);
      b_right =  true;
    }else if(sensorValue_right > goal_pose_right){
      digitalWrite(DirOut_right, LOW);
      analogWrite(PulOut_right, PWM_value);
      b_right = false;
    }else if(sensorValue_right < goal_pose_right){
      digitalWrite(DirOut_right, HIGH);
      analogWrite(PulOut_right, PWM_value);
      b_right =  false;
    }
    read_sensors();
    if(sensorValue_left>goal_pose_left+max_step+telorance || sensorValue_left<goal_pose_left-max_step-telorance){
      analogWrite(PulOut_left, 0);
      b_left = false;
    }else if(sensorValue_left<goal_pose_left+telorance && sensorValue_left>goal_pose_left-telorance){
      analogWrite(PulOut_left, 0);
      b_left =  true;
    }else if(sensorValue_left > goal_pose_left){
      digitalWrite(DirOut_left, LOW);
      analogWrite(PulOut_left, PWM_value);
      b_left =  false;
    }else if(sensorValue_left < goal_pose_left){
      digitalWrite(DirOut_left, HIGH);
      analogWrite(PulOut_left, PWM_value);
      b_left =  false;
    }
    
    if(b_right && b_left){
      return true;
    }
  }
}

bool Stop_both(int goal_pose_right, int goal_pose_left){
  read_sensors();
  if(Servo_stop){
    analogWrite(PulOut_right, 0);
    analogWrite(PulOut_left, 0);
    Servo_stop = false;
    return false;
  }else{
    if(sensorValue_right<goal_pose_right+telorance && sensorValue_right>goal_pose_right-telorance){
      analogWrite(PulOut_right, 0);
      b_right =  true;
    }
    if(sensorValue_left<goal_pose_left+telorance && sensorValue_left>goal_pose_left-telorance){
      analogWrite(PulOut_left, 0);
      b_left =  true;
    }
  }
  if(sensorValue_right>goal_pose_right+max_step+telorance || sensorValue_right<goal_pose_right-max_step-telorance){
     analogWrite(PulOut_right, 0);
     b_right =  false;
  }
  if(sensorValue_left>goal_pose_left+max_step+telorance || sensorValue_left<goal_pose_left-max_step-telorance){
     analogWrite(PulOut_left, 0);
     b_left =  false;
  }
  return b_left & b_right;
}


// Arduino Uno
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
