#ifndef __POSITION_H_
#define __POSITION_H_
typedef struct {
  byte pin;
  unsigned int MIN;
  unsigned int MAX;
  unsigned int MIDDLE;    
}SERVO;


typedef struct {
  unsigned int A; // genoux
  unsigned int B; // cheville 
  unsigned int C; // hanche
}LEG_POSITION;
extern const LEG_POSITION positions[17][4];
extern SERVO servos[12];
#endif
