#ifndef __POSITION_H_
#define __POSITION_H_
typedef struct {
  unsigned int A; // genoux
  unsigned int B; // cheville 
  unsigned int C; // hanche
}LEG_POSITION;
extern const LEG_POSITION positions[17][4];
#endif
