/**
 * @file indev.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "tft.h"
#include "lvgl.h"


#include "stm32h7xx.h"
#include "stm32h747i_discovery.h"
#include "stm32h747i_discovery_ts.h"


/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void touchpad_read_cb(lv_indev_drv_t * drv, lv_indev_data_t *data);

/**********************
 *  STATIC VARIABLES
 **********************/
static TS_State_t  TS_State;
TS_Init_t* hTS;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Initialize your input devices here
 */
void touchpad_init(void)
{

  hTS->Width = 800;
  hTS->Height = 480;
  hTS->Orientation = TS_SWAP_XY | TS_SWAP_Y;
  hTS->Accuracy = 0;

  BSP_TS_Init(0, hTS);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.read_cb = touchpad_read_cb;
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  lv_indev_drv_register(&indev_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void touchpad_read_cb(lv_indev_drv_t * drv, lv_indev_data_t *data)
{
	static int16_t last_x = 0;
	static int16_t last_y = 0;
	BSP_TS_GetState(0, &TS_State);
	if(TS_State.TouchDetected != 0) {
		data->point.x = TS_State.TouchX;
		data->point.y = TS_State.TouchY;
		last_x = data->point.x;
		last_y = data->point.y;
		data->state = LV_INDEV_STATE_PR;
	} else {
		data->point.x = last_x;
		data->point.y = last_y;
		data->state = LV_INDEV_STATE_REL;
	}
}
