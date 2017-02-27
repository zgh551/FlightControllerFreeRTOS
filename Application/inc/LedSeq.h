#ifndef _LEDSEQ_H_
#define _LEDSEQ_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
  
/****LED Define ****/
typedef enum _Led_TypeDef Led_TypeDef;  

#define LEDSEQ_CHARGE_CYCLE_TIME  1000
//Led sequence action
#define LEDSEQ_WAITMS(X)  X
#define LEDSEQ_STOP      -1
#define LEDSEQ_LOOP      -2
  
typedef struct {
 bool value;
 int action;
} ledseq_t;


//Public API
void LedseqInit(void);
bool LedseqTest(void);
void LedseqRun ( Led_TypeDef led, ledseq_t * sequence);
void LedseqStop( Led_TypeDef led, ledseq_t * sequence);
void LedseqSetTimes(ledseq_t *sequence, int32_t onTime, int32_t offTime);

//Existing led sequences
extern ledseq_t seq_armed[];
extern ledseq_t seq_calibrated[];
extern ledseq_t seq_alive[];
extern ledseq_t seq_lowbat[];
extern ledseq_t seq_linkup[];
extern ledseq_t seq_altHold[];
extern ledseq_t seq_charged[];
extern ledseq_t seq_charging[];
extern ledseq_t seq_bootloader[];
extern ledseq_t seq_testPassed[];	

#ifdef __cplusplus
}
#endif
#endif
