volatile float millis = 0;
unsigned long long prevMillis = 0;
volatile const int gaps = 13; // gaps on the encoder wheel
volatile int change = 0;
volatile bool newChange = 0;
volatile unsigned long int time = 0;

// occurs whenever octocoupler's signal is broken
void ISR_0(int TIMER1_CAPT_vect, int ICR1, int TCNT1)
{
  time += ICR1;
  change++;
  newChange = 1;
  TCNT1 = 0;
}

void ISR_1(int TIMER2_COMPA_vect)
{ // change the 0 to 1 for timer1 and 2 for timer2
  // interrupt commands here
  millis += 0.5;
}

// timer overflow event
void ISR_2(int TIMER1_OVF_vect, int ICR1, int TCNT1)
{
  time += ICR1;
  TCNT1 = 0;
}

int main() {
    //global
    #define slopeWidth 90

    float slopeTab[100];
    int slopeIndex = 0;

    // main
    float input_distance = 4.0;
    float input_time = 80.0;

    float current_distance = 0;
    float last_distance = 0;
    int change_time = 0;
    float motorPower = 0;
    float desired_velocity = 0;
    float last_velocity = 0;
    float filtered_velocity = 0;
    float kp = 1500;
    float ki = 5000;

    float acceleration = 0;

    float slope_integral = 0;

    float integral_part = 0;
    float proportional_part = 0;

    int OCR0A = 140;

    for (int i = slopeWidth; i >= 0; i--)
    {
        slopeTab[i] = 0;
    }
    slopeIndex = 0;
    change = 0;
    millis = 0;
    prevMillis = 0;

    while (current_distance < input_distance)
    {
        float current_velocity = 0;
        if (millis - prevMillis > 100)
        {
        newChange = 0;
        current_distance =0.932 * change * ((0.06565 * 3.14 * 13) / (gaps * 49));

        if (millis - prevMillis > 0)
        {
            change_time = millis - prevMillis;
        }
        float change_distance;
        
        change_distance = current_distance - last_distance;
        current_velocity = (change_distance / change_time) * 1000;
        filtered_velocity = 0.854 * filtered_velocity + 0.0728 * current_velocity + 0.0728 * last_velocity;
        last_velocity = current_velocity;
        if ((input_distance - current_distance) / (input_time - ((float)millis) / 1000) > 0)
        {
            desired_velocity = (input_distance - current_distance) / (input_time - ((float)millis) / 1000);
        }

        for (int i = slopeWidth; i > 0; i--)
        {
            slopeTab[i - 1] = slopeTab[i];
        }
        slopeTab[slopeWidth] = desired_velocity;
        acceleration = slopeTab[slopeWidth] - slopeTab[0];

        //printf("dv1:%f,fv:%f\n", desired_velocity * 1000, filtered_velocity * 1000);
        slopeIndex++;
        if(slopeIndex > slopeWidth && input_distance - current_distance > 0.3){
            slope_integral += 20 * acceleration;
            //desired_velocity += slope_integral;
        }
        // printf("%f,%f\n", desired_velocity*1000, filtered_velocity*1000);
        float diff = desired_velocity - filtered_velocity;
        integral_part += diff * ki * change_time / 1000;
        proportional_part = diff * kp;
        motorPower = integral_part + proportional_part;
        
        // saturation guard
        if (motorPower < 0)
        {
            motorPower = 0;
        }
        if (motorPower > 140)
        {
            motorPower = 140;
        }
        OCR0A = 255 - ((int)motorPower);
        
        //printf("%f,%f\n", current_distance, millis/1000);

        last_distance = current_distance;
        prevMillis = millis;

        //printf("page1.x0.val=%d%c%c%c",(int)(current_distance * 100), 255, 255, 255); 
        //kprintf("page1.x1.val=%d%c%c%c",(int)(millis/10), 255, 255, 255); 
        }
    }
    OCR0A = 255;
    float totalTime = millis;
    current_distance = 0.932 * change * ((0.06565 * 3.14 * 13) / (gaps * 49)); // calculating the arc
}