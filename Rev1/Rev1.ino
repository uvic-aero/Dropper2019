#include <Servo.h>

/* ------------ BUILD MODES --------------
 * MODE_FUTABA: for use with a futaba receiver, powered from the board
 *              Uses 70 HZ PWM, 1000us - 2000us duty cycle
 * MODE_PIXHAWK: for use with signals from pixhawk, not powered by board
 *              Uses 400Hz PWM, 1000us - 2000us duty cycle
 *              
 * Only one of these two can be 1 at the same time
 */
#define MODE_FUTABA     (1)
#define MODE_PIXHAWK    (0)

#if MODE_FUTABA == MODE_PIXHAWK
  #error Must choose one of: MODE_FUTABA or MODE_PIXHAWK
#endif

/* -------------- DEFINITIONS ------------------- */

#define SERVO           (5)     // pin: out servo signal
#define TRIGGER         (3)     // pin: in  PWM signal
#define SAFETY          (4)     // pin: in  PWM signal

#define START_POS       (90)    // degrees: initial pos
#define ANGLE1          (110)   // degrees: first position
#define ANGLE2          (55)    // degrees: second position
#define ANGLE3          (0)     // degrees: third position
#define NUM_FIRES       (3)     // count
#define RETRACT_DELAY   (500)   // ms
#define RETRACT_OFFSET  (30)    // ms

#if MODE_FUTABA
  #define PWM_FREQ          (70)    // hertz
  #define PWM_DUTY_THRESH   (1500)  // microseconds
  #define PWM_HIGH_CNT_MAX  (10)    // count
  #define PWM_LOW_CNT_MAX   (10)    // count
#elif MODE_PIXHAWK
  #define PWM_FREQ          (400)   // hertz
  #define PWM_DUTY_THRESH   (1500)  // microseconds
  #define PWM_HIGH_CNT_MAX  (55)    // count
  #define PWM_LOW_CNT_MAX   (55)    // count
#endif

/* -------------- GLOBAL VARIABLES --------------- */
Servo servo;
int angle[NUM_FIRES] = {ANGLE1, ANGLE2, ANGLE3};
int fireCount = 0;
int triggerHighCount = 0;
int triggerLowCount = 0;
int safetyHighCount = 0;
int safetyLowCount = 0;

/* --------------- FUNCTION DECLARATIONS ---------*/
bool detectTrigger();
bool detectTriggerRelease();
bool detectSafety();
bool detectSafetyRelease();
void fire();

/* -------------- MAIN -------------------------*/

void setup() {
  // put your setup code here, to run once:
  servo.attach(SERVO);
  servo.write(START_POS);
  pinMode(TRIGGER, INPUT);
  pinMode(SAFETY, INPUT);
  Serial.begin(9600);
  Serial.println("Up and Running!");
}

void loop()
{
#if MODE_FUTABA
  // With with futaba, there are two signals
  
  // wait for both the safety and the trigger
  // to be in the fire position simultaneously
  // the order doesn't matter
  while ( !detectSafety() )
    while ( !detectTrigger() );
    
  fire();

  // wait for both the trigger and safety
  // to return to no event condition
  // the order doesnt matter
  while (!detectTriggerRelease())
    while (!detectSafetyRelease());
    
#elif MODE_PIXHAWK
  // with the pixhawk, there is only one signal

  // wait for trigger
  while( !detectTrigger() );
  fire();
  // wait for trigger release
  while( !detectTriggerRelease() );
#endif
}

/* --------------- FUNCTION DEFINITIONS ---------------- */

/* @brief: detect the fire condition on the trigger signal
 *         Waits for and measures a pulse on the signal line
 *         After PWM_HIGH_CNT_MAX pulses above the pwm threshold,
 *         returns true.
 *         If a pulse is low, resets the high count
 */
bool detectTrigger()
{
  if(pulseIn(TRIGGER, HIGH)>= PWM_DUTY_THRESH)
  {
    triggerHighCount++;
    triggerLowCount = 0;
  }
  else
  {
    triggerLowCount++;
    triggerHighCount = 0;
  }
  
  if(triggerHighCount >= PWM_HIGH_CNT_MAX)
    return true;
  else
    return false;
}

/* @brief: detects release of the trigger signal
 *         Waits for a pulse and measures it
 *         After PWM_LOW_CNT_MAX pulses below threshold
 *         returns true.
 *         If a high pulse is seen, resets the count
 */
bool detectTriggerRelease()
{
  if(pulseIn(TRIGGER, HIGH)< PWM_DUTY_THRESH)
  {
    triggerLowCount++;
    triggerHighCount = 0;
  }
  else
  {
    triggerHighCount++;
    triggerLowCount = 0;
  }
  
  if(triggerLowCount >= PWM_LOW_CNT_MAX)
    return true;
  else
    return false;
}

/* @brief: detect the fire condition on the safety signal
 *         Waits for and measures a pulse on the signal line
 *         After PWM_HIGH_CNT_MAX pulses above the pwm threshold,
 *         returns true.
 *         If a pulse is low, resets the high count
 */
bool detectSafety()
{
  if(pulseIn(SAFETY, HIGH)>= PWM_DUTY_THRESH)
  {
    safetyHighCount++;
    safetyLowCount = 0;
  }
  else
  {
    safetyLowCount++;
    safetyHighCount = 0;
  }
  
  if(safetyHighCount >= PWM_HIGH_CNT_MAX)
    return true;
  else
    return false;
}

/* @brief: detects release of the safety signal
 *         Waits for a pulse and measures it
 *         After PWM_LOW_CNT_MAX pulses below threshold
 *         returns true.
 *         If a high pulse is seen, resets the count
 */
bool detectSafetyRelease()
{
  if(pulseIn(SAFETY, HIGH)< PWM_DUTY_THRESH)
  {
    safetyLowCount++;
    safetyHighCount = 0;
  }
  else
  {
    safetyHighCount++;
    safetyLowCount = 0;
  }
  
  if(safetyLowCount >= PWM_LOW_CNT_MAX)
    return true;
  else
    return false;
}

/* @brief: manages firing logic
 *         Moves the servo to the correct position to fire the next dropper
 *         Only fires NUM_FIRES times, after that will do nothing
 */
void fire(){
  if (fireCount < NUM_FIRES)
  { 
    servo.write(angle[fireCount]);
    fireCount++;

    if (fireCount == NUM_FIRES)
    {
      delay(RETRACT_DELAY);
      servo.write(angle[NUM_FIRES - 1] + 30);
    }
  }
}
