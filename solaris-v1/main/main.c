#include "macros_esp.h"
#include "spi.h"
#include "core/macros.h"
#include "icm20948.h"
#include "task.h"
#include "macros.h"
// #include "eventgroups.h"

static icm_data_t icm_data;

void app_main()
{
   /** Initialization of the sensors */
   retval_t SPP_HAL_SPI_BusInit(void);
   retval_t ret = SPP_ERROR;
   ret = IcmInit((void*)&icm_data);
   ret = IcmConfig((void*)&icm_data);
   
   /** Initialization of the task for reading data from sensors */
   void *p_icm_init;
   void * p_task_storage;
   void * p_buffer_eg;
   void *p_event_group = NULL;
   p_task_storage = SPP_OSAL_GetTaskStorage();
   p_icm_init = SPP_OSAL_TaskCreate(IcmGetSensorsData, "ICM Read Sensors Task", STACK_SIZE, (void*)&icm_data, ICM_READ_SENSORS_PRIORITY, p_task_storage);
   if (p_icm_init == NULL){
      return;
   }
   
   // p_buffer_eg = SPP_OSAL_GetEventGroupsBuffer();
   
   // ret = OSAL_EventGroupCreate(p_event_group, p_buffer_eg);
   // if (ret != SPP_OK){
   //    //Event group failed to be created
   // }
}  
