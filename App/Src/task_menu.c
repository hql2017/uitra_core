
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

extern  void StartDefaultTask(void *argument);
extern  void auxTask02(void *argument);
extern  void keyScanTask03(void *argument);
extern  void laserWorkTask04(void *argument);
extern  void fastAuxTask05(void *argument);
extern  void hmiAppTask06(void *argument);
extern  void canReceiveTask07(void *argument);
extern  void powerOffTask08(void *argument);
extern  void laserProhotTask09(void *argument);
extern  void ge2117ManageTask10(void *argument);
extern  void musicTask11(void *argument);

extern  void LaserWorkTimerCallback01(void *argument);
extern  void cleanWaterCallback02(void *argument);
extern  void tmcMaxRunTimesCallback03(void *argument);
extern  void MX_FREERTOS_Init(void);