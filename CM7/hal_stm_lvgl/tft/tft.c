/**
 * @file disp.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "../lv_conf.h"
#include "lvgl.h"
#include <string.h>
#include "main.h"

#include "tft.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx.h"
#include "../Components/otm8009a/otm8009a.h"

#include "stm32h747i_discovery.h"
#include "stm32h747i_discovery_lcd.h"
#include "stm32h747i_discovery_sdram.h"


/*********************
 *      DEFINES
 *********************/

#define LCD_FB_START_ADDRESS 0xD0000000


#if TFT_NO_TEARING
#define ZONES               2       /*Divide the screen into zones to handle tearing effect*/
#else
#define ZONES               1
#endif

#define HACT                (OTM8009A_800X480_WIDTH / ZONES)

#define LAYER0_ADDRESS      (LCD_FB_START_ADDRESS)

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/*For LittlevGL*/
static void tft_flush_cb(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p);

/*LCD*/
static void LCD_Init(void);


/*DMA to flush to frame buffer*/
static void DMA_TransferComplete(DMA_HandleTypeDef *han);
static void DMA_TransferError(DMA_HandleTypeDef *han);

/**********************
 *  STATIC VARIABLES
 **********************/

extern LTDC_HandleTypeDef hltdc;
extern DSI_HandleTypeDef hdsi;
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream0;


#if LV_COLOR_DEPTH == 16
static uint16_t * my_fb = (uint16_t *)LAYER0_ADDRESS;
#else
static uint32_t * my_fb = (uint32_t *)LAYER0_ADDRESS;
#endif

static lv_disp_drv_t disp_drv;


static lv_disp_drv_t disp_drv;
static volatile int32_t x1_flush;
static volatile int32_t y1_flush;
static volatile int32_t x2_flush;
static volatile int32_t y2_flush;
static volatile int32_t y_flush_act;
static volatile int32_t num_of_pixels;
static volatile const lv_color_t * buf_to_flush;

static volatile bool refr_qry;
static volatile uint32_t t_last = 0;

#if TFT_NO_TEARING
uint8_t pPage[]       = {0x00, 0x00, 0x01, 0xDF}; /*   0 -> 479 */
#endif

/* When changing these parameters, you should also change the settings of the LTDC & DSIHOST in the .ioc project
   (set the "horizontal pixles" value 800, 400 or 200 in LTDC section and "Maximum command size" in DSIHOST section) */
uint8_t pCols[ZONES][4] =
{
#if (ZONES == 4 )
  {0x00, 0x00, 0x00, 0xC7}, /*   0 -> 199 */
  {0x00, 0xC8, 0x01, 0x8F}, /* 200 -> 399 */
  {0x01, 0x90, 0x02, 0x57}, /* 400 -> 599 */
  {0x02, 0x58, 0x03, 0x1F}, /* 600 -> 799 */
#elif (ZONES == 2 )
  {0x00, 0x00, 0x01, 0x8F}, /*   0 -> 399 */
  {0x01, 0x90, 0x03, 0x1F}
#elif (ZONES == 1 )
  {0x00, 0x00, 0x03, 0x1F}, /*   0 -> 799 */
#endif
};


/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Monitor refresh time
 * */
void monitor_cb(lv_disp_drv_t * d, uint32_t t, uint32_t p)
{
    t_last = t;
}

/**
 * Initialize your display here
 */
void tft_init(void)
{
	LCD_Init();
	LCD_SetUpdateRegion(0);
	HAL_DSI_ShortWrite(&hdsi,
			0,
			DSI_DCS_SHORT_PKT_WRITE_P1,
			OTM8009A_CMD_DISPON,
			0x00);

	/*##-5- Select Callbacks functions called after Transfer complete and Transfer error */
	HAL_DMA_RegisterCallback(&hdma_memtomem_dma2_stream0, HAL_DMA_XFER_CPLT_CB_ID, DMA_TransferComplete);
	HAL_DMA_RegisterCallback(&hdma_memtomem_dma2_stream0, HAL_DMA_XFER_ERROR_CB_ID, DMA_TransferError);

	static lv_color_t disp_buf1[TFT_HOR_RES * 48];
	static lv_color_t disp_buf2[TFT_HOR_RES * 48];
	static lv_disp_draw_buf_t buf;
	lv_disp_draw_buf_init(&buf, disp_buf1, disp_buf2, TFT_HOR_RES * 48);

	lv_disp_drv_init(&disp_drv);
	disp_drv.draw_buf = &buf;
	disp_drv.flush_cb = tft_flush_cb;
	disp_drv.monitor_cb = monitor_cb;
	disp_drv.hor_res = 800;
	disp_drv.ver_res = 480;
	lv_disp_drv_register(&disp_drv);
}


void LCD_Init(void)
{
#if TFT_NO_TEARING
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, OTM8009A_CMD_CASET, pCols[0]);
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, OTM8009A_CMD_PASET, pPage);

	static uint8_t ScanLineParams[2];
	#if ZONES == 2
	  uint16_t scanline = 200;
	#elif ZONES == 4
		uint16_t scanline = 283;
	#endif
	ScanLineParams[0] = scanline >> 8;
	ScanLineParams[1] = scanline & 0x00FF;

	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 2, 0x44, ScanLineParams);
	/* set_tear_on */
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, OTM8009A_CMD_TEEON, 0x00);
#endif

}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void tft_flush_cb(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
//    lv_disp_flush_ready(drv);
//    return;
	SCB_CleanInvalidateDCache();

	/*Truncate the area to the screen*/
	int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
	int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
	int32_t act_x2 = area->x2 > TFT_HOR_RES - 1 ? TFT_HOR_RES - 1 : area->x2;
	int32_t act_y2 = area->y2 > TFT_VER_RES - 1 ? TFT_VER_RES - 1 : area->y2;

	x1_flush = act_x1;
	y1_flush = act_y1;
	x2_flush = act_x2;
	y2_flush = act_y2;
	y_flush_act = act_y1;
	buf_to_flush = color_p;


	/*Use DMA instead of DMA2D to leave it free for GPU*/
	HAL_StatusTypeDef err;
  	err = HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream0,(uint32_t)buf_to_flush, (uint32_t)&my_fb[y_flush_act * TFT_HOR_RES + x1_flush],
			  	  	  	   (x2_flush - x1_flush + 1));
	if(err != HAL_OK)
	{
		while(1);	/*Halt on error*/
	}
}





static volatile uint32_t LCD_ActiveRegion;






void LCD_SetUpdateRegion(int idx)
{
  HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, OTM8009A_CMD_CASET, pCols[idx]);
}


/**
  * @brief  Tearing Effect DSI callback.
  * @param  hdsi: pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval None
  */
void HAL_DSI_TearingEffectCallback(DSI_HandleTypeDef *hdsi)
{
    if(refr_qry)
    {
        LCD_ActiveRegion = 1;
        HAL_DSI_Refresh(hdsi);
        refr_qry = false;
    }
}

void HAL_DSI_EndOfRefreshCallback(DSI_HandleTypeDef *hdsi)
{

    if(LCD_ActiveRegion < ZONES )
    {
        /* Disable DSI Wrapper */
        __HAL_DSI_WRAPPER_DISABLE(hdsi);
        /* Update LTDC configuaration */
        LTDC_LAYER(&hltdc, 0)->CFBAR  = LAYER0_ADDRESS + LCD_ActiveRegion  * HACT * sizeof(lv_color_t);
        __HAL_LTDC_RELOAD_CONFIG(&hltdc);
        __HAL_DSI_WRAPPER_ENABLE(hdsi);

        LCD_SetUpdateRegion(LCD_ActiveRegion++);
        /* Refresh the right part of the display */
        HAL_DSI_Refresh(hdsi);

    }
    else
    {
        __HAL_DSI_WRAPPER_DISABLE(hdsi);
        LTDC_LAYER(&hltdc, 0)->CFBAR  = LAYER0_ADDRESS;

        __HAL_LTDC_RELOAD_CONFIG(&hltdc);
        __HAL_DSI_WRAPPER_ENABLE(hdsi);

        LCD_SetUpdateRegion(0);
       // if(disp_drv.draw_buf)  lv_disp_flush_ready(&disp_drv);

    }
}



/**
  * @brief  DMA conversion complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
static void DMA_TransferComplete(DMA_HandleTypeDef *han)
{

	y_flush_act++;

	if(y_flush_act > y2_flush)
	{
#if TFT_NO_TEARING
		if(lv_disp_flush_is_last(&disp_drv))
			refr_qry = true;
		else
			lv_disp_flush_ready(&disp_drv);
#else
		if(lv_disp_flush_is_last(&disp_drv)) HAL_DSI_Refresh(&hdsi);
		lv_disp_flush_ready(&disp_drv);
#endif
	}
	else
	{
	  buf_to_flush += (x2_flush - x1_flush + 1);
	  //##-7- Start the DMA transfer using the interrupt mode ####################
	  // Configure the source, destination and buffer size DMA fields and Start DMA Stream transfer
	  // Enable All the DMA interrupts
	  if(HAL_DMA_Start_IT(han,(uint32_t)buf_to_flush, (uint32_t)&my_fb[y_flush_act * TFT_HOR_RES + x1_flush],
						  (x2_flush - x1_flush + 1)) != HAL_OK)
	  {
	    while(1);	//Halt on error
	  }
	}
}

/**
  * @brief  DMA conversion error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
static void DMA_TransferError(DMA_HandleTypeDef *han)
{
    while(1);
}




