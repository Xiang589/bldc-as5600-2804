#include "motor_ui.h"
#include <stdio.h>
#include "as5600.h"
#include "i2c.h"
#include "lcd_ili9341.h"
#include "motor_control.h"
#include "touch_cal_storage.h"
#include "touch_xpt2046.h"

#define ENABLE_TOUCH_CALIBRATION 0U
#define C_BG 0x0000U
#define C_FG 0xFFFFU
#define C_BTN 0x07E0U
#define C_STOP 0xF800U
#define C_CAL 0x001FU
#define DUTY_STEP 0.05f

typedef struct { uint16_t x,y,w,h; const char *label; uint16_t color; } UiButton;
typedef struct { const char *name; uint16_t sx; uint16_t sy; uint16_t rx; uint16_t ry; } CalPoint;
typedef enum { UI_MODE_MAIN=0, UI_MODE_CAL_CONFIRM=1, UI_MODE_CALIBRATION=2, UI_MODE_CAL_DONE=3 } UiMode_t;

static UiButton g_btn_start={20,180,90,44,"START",C_BTN}, g_btn_stop={130,180,90,44,"STOP",C_STOP}, g_btn_plus={20,236,90,44,"DUTY+",C_BTN}, g_btn_minus={130,236,90,44,"DUTY-",C_BTN}, g_btn_cal={170,8,60,20,"CAL",C_CAL}, g_btn_yes={30,180,80,44,"YES",C_BTN}, g_btn_no={130,180,80,44,"NO",C_STOP};
static UiMode_t g_ui_mode=UI_MODE_MAIN;
static uint32_t g_last_draw=0U,g_mode_tick=0U;
static uint8_t g_touch_latch=0U,g_touch_pen=0U,g_touch_has_xy=0U,g_cal_wait_release=0U,g_wait_release_before_cal=0U,g_has_flash_cal=0U;
static uint16_t g_touch_x=0U,g_touch_y=0U;
static TouchCalibration_t g_prev_cal;
static CalPoint g_cal_points[5]={{"LT",20,20,0,0},{"RT",220,20,0,0},{"LB",20,300,0,0},{"RB",220,300,0,0},{"CT",120,160,0,0}};
static uint8_t g_cal_index=0U;

static uint8_t Ui_Hit(const UiButton*b,uint16_t x,uint16_t y){return (x>=b->x)&&(x<(b->x+b->w))&&(y>=b->y)&&(y<(b->y+b->h));}
static void Ui_DrawButton(const UiButton*b){LCD_FillRect(b->x,b->y,b->w,b->h,b->color);LCD_DrawRect(b->x,b->y,b->w,b->h,C_FG);LCD_DrawText((uint16_t)(b->x+10U),(uint16_t)(b->y+6U),b->label,C_FG,b->color);} 
static void Cal_DrawCross(uint16_t x,uint16_t y){LCD_FillRect((uint16_t)(x-10U),y,21U,1U,C_FG);LCD_FillRect(x,(uint16_t)(y-10U),1U,21U,C_FG);} 
static void Cal_DrawPointScreen(uint8_t i){char l[24];LCD_FillScreen(C_BG);snprintf(l,sizeof(l),"CAL %u/5",(unsigned)(i+1U));LCD_DrawText(10U,8U,l,C_FG,C_BG);LCD_DrawText(10U,24U,"PRESS CROSS",C_FG,C_BG);Cal_DrawCross(g_cal_points[i].sx,g_cal_points[i].sy);} 
static void Ui_DrawMainScreen(void){LCD_FillScreen(C_BG);LCD_DrawText(30U,8U,"AS5600 MOTOR",C_FG,C_BG);LCD_DrawRect(4U,4U,232U,24U,C_FG);Ui_DrawButton(&g_btn_cal);Ui_DrawButton(&g_btn_start);Ui_DrawButton(&g_btn_stop);Ui_DrawButton(&g_btn_plus);Ui_DrawButton(&g_btn_minus);} 
static void Ui_DrawConfirmScreen(void){LCD_FillScreen(C_BG);LCD_DrawText(20U,40U,"RECALIBRATE?",C_FG,C_BG);Ui_DrawButton(&g_btn_yes);Ui_DrawButton(&g_btn_no);} 
static void Ui_DrawStatus(void){char l[32],t[32];uint16_t raw=0;LCD_FillRect(10U,34U,220U,132U,C_BG);LCD_DrawText(10U,36U,"Motor:",C_FG,C_BG);LCD_DrawText(82U,36U,MotorControl_IsRunning()?"RUN":"STOP",MotorControl_IsRunning()?C_BTN:C_STOP,C_BG);snprintf(l,sizeof(l),"Duty: %u%%",(unsigned)(MotorControl_GetDuty()*100.0f));LCD_DrawText(10U,60U,l,C_FG,C_BG);if(AS5600_ReadRawAngle(&hi2c1,&raw)==HAL_OK)snprintf(l,sizeof(l),"AS5600: %u",raw);else snprintf(l,sizeof(l),"AS5600: ERR");LCD_DrawText(10U,84U,l,C_FG,C_BG);if(!g_touch_pen)snprintf(t,sizeof(t),"Touch: ---");else if(!g_touch_has_xy)snprintf(t,sizeof(t),"Touch: PEN");else snprintf(t,sizeof(t),"Touch: %u,%u",g_touch_x,g_touch_y);LCD_DrawText(10U,108U,t,C_FG,C_BG);} 
static TouchCalibration_t Cal_Build(void)
{
  TouchCalibration_t cal = {0};
  const CalPoint *lt = &g_cal_points[0];
  const CalPoint *rt = &g_cal_points[1];
  const CalPoint *lb = &g_cal_points[2];

  const float x0 = (float)lt->rx, y0 = (float)lt->ry;
  const float x1 = (float)rt->rx, y1 = (float)rt->ry;
  const float x2 = (float)lb->rx, y2 = (float)lb->ry;

  const float sx0 = 20.0f, sy0 = 20.0f;
  const float sx1 = 220.0f, sy1 = 20.0f;
  const float sx2 = 20.0f, sy2 = 300.0f;

  const float denom = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
  if ((denom > -1e-6f) && (denom < 1e-6f))
  {
    return cal;
  }

  const float a = ((sx1 - sx0) * (y2 - y0) - (sx2 - sx0) * (y1 - y0)) / denom;
  const float b = ((x1 - x0) * (sx2 - sx0) - (x2 - x0) * (sx1 - sx0)) / denom;
  const float c = sx0 - a * x0 - b * y0;

  const float d = ((sy1 - sy0) * (y2 - y0) - (sy2 - sy0) * (y1 - y0)) / denom;
  const float e = ((x1 - x0) * (sy2 - sy0) - (x2 - x0) * (sy1 - sy0)) / denom;
  const float f = sy0 - d * x0 - e * y0;

  cal.ax = (int32_t)(a * 65536.0f);
  cal.bx = (int32_t)(b * 65536.0f);
  cal.cx = (int32_t)(c * 65536.0f);
  cal.ay = (int32_t)(d * 65536.0f);
  cal.by = (int32_t)(e * 65536.0f);
  cal.cy = (int32_t)(f * 65536.0f);

  return cal;
}
static uint8_t Cal_Validate(const TouchCalibration_t*cal)
{
  uint16_t rbx=0U, rby=0U, cx=0U, cy=0U;
  int32_t erbx, erby, ecx, ecy;
  if (!cal) return 0U;
  if ((cal->ax == 0) && (cal->bx == 0) && (cal->ay == 0) && (cal->by == 0)) return 0U;

  if (Touch_MapRawToPoint(g_cal_points[3].rx, g_cal_points[3].ry, cal, &rbx, &rby) == 0U) return 0U;
  if (Touch_MapRawToPoint(g_cal_points[4].rx, g_cal_points[4].ry, cal, &cx, &cy) == 0U) return 0U;

  erbx = (int32_t)rbx - 220;
  erby = (int32_t)rby - 300;
  ecx = (int32_t)cx - 120;
  ecy = (int32_t)cy - 160;
  if (erbx < 0) erbx = -erbx;
  if (erby < 0) erby = -erby;
  if (ecx < 0) ecx = -ecx;
  if (ecy < 0) ecy = -ecy;

  if ((erbx > 40) || (erby > 40) || (ecx > 40) || (ecy > 40)) return 0U;
  return 1U;
}

void MotorUi_Init(void){TouchCalibration_t cal;LCD_Init();LCD_SetBacklight(1U);Touch_Init();if(TouchCalStorage_Load(&cal)){Touch_SetCalibration(&cal);g_prev_cal=cal;g_has_flash_cal=1U;printf("[CAL] loaded from flash\r\n");g_ui_mode=UI_MODE_MAIN;Ui_DrawMainScreen();Ui_DrawStatus();}else{Touch_LoadDefaultCalibration();printf("[CAL] use defaults\r\n");g_ui_mode=UI_MODE_CALIBRATION;g_cal_index=0U;g_cal_wait_release=0U;g_wait_release_before_cal=1U;LCD_FillScreen(C_BG);LCD_DrawText(10U,8U,"NO CAL IN FLASH",C_FG,C_BG);LCD_DrawText(10U,24U,"RELEASE THEN CAL",C_FG,C_BG);} }

void MotorUi_Update(uint32_t now){uint16_t x,y;
if(g_ui_mode==UI_MODE_CAL_DONE){if((now-g_mode_tick)>=1200U){g_ui_mode=UI_MODE_MAIN;Ui_DrawMainScreen();Ui_DrawStatus();}return;}
if(g_ui_mode==UI_MODE_CAL_CONFIRM){if(Touch_IsPressed()){if(!g_touch_latch&&Touch_ReadPoint(&x,&y)){g_touch_latch=1U;if(Ui_Hit(&g_btn_yes,x,y)){MotorControl_Stop();g_cal_index=0U;g_cal_wait_release=0U;g_wait_release_before_cal=1U;g_ui_mode=UI_MODE_CALIBRATION;LCD_FillScreen(C_BG);LCD_DrawText(10U,8U,"RELEASE TOUCH",C_FG,C_BG);}else if(Ui_Hit(&g_btn_no,x,y)){g_ui_mode=UI_MODE_MAIN;Ui_DrawMainScreen();Ui_DrawStatus();}}}else g_touch_latch=0U;return;}
if(g_ui_mode==UI_MODE_CALIBRATION){if(g_wait_release_before_cal){if(Touch_IsPressed()==0U){g_wait_release_before_cal=0U;Cal_DrawPointScreen(g_cal_index);}return;} if(g_cal_wait_release){if(Touch_IsPressed()==0U){g_cal_wait_release=0U;g_cal_index++;if(g_cal_index>=5U){TouchCalibration_t cal=Cal_Build();if(Cal_Validate(&cal)){TouchCalibration_t old;Touch_GetCalibration(&old);Touch_SetCalibration(&cal);MotorControl_Stop();if(TouchCalStorage_Save(&cal)){g_prev_cal=cal;g_has_flash_cal=1U;LCD_FillScreen(C_BG);LCD_DrawText(10U,8U,"CAL SAVED",C_FG,C_BG);}else{Touch_SetCalibration(&old);LCD_FillScreen(C_BG);LCD_DrawText(10U,8U,"CAL SAVE ERR",C_FG,C_BG);}}else{if(g_has_flash_cal)Touch_SetCalibration(&g_prev_cal);else Touch_LoadDefaultCalibration();LCD_FillScreen(C_BG);LCD_DrawText(10U,8U,"CAL INVALID",C_FG,C_BG);}g_mode_tick=now;g_ui_mode=UI_MODE_CAL_DONE;}else Cal_DrawPointScreen(g_cal_index);}return;} if(Touch_IsPressed()){uint32_t sx=0,sy=0;uint16_t rx=0,ry=0;uint8_t n=0;for(uint8_t i=0;i<8U;i++){if(Touch_ReadRaw(&rx,&ry)){sx+=rx;sy+=ry;n++;}HAL_Delay(5U);}if(n){g_cal_points[g_cal_index].rx=(uint16_t)(sx/n);g_cal_points[g_cal_index].ry=(uint16_t)(sy/n);printf("[CAL] %s screen_x=%u screen_y=%u raw_x=%u raw_y=%u\r\n",g_cal_points[g_cal_index].name,g_cal_points[g_cal_index].sx,g_cal_points[g_cal_index].sy,g_cal_points[g_cal_index].rx,g_cal_points[g_cal_index].ry);g_cal_wait_release=1U;}}return;}
if(Touch_IsPressed()){g_touch_pen=1U;if(!g_touch_latch){printf("[TOUCH] PEN low\r\n");if(Touch_ReadPoint(&x,&y)){g_touch_has_xy=1U;g_touch_x=x;g_touch_y=y;printf("[TOUCH] x=%u y=%u\r\n",x,y);if(Ui_Hit(&g_btn_cal,x,y)){printf("[TOUCH] CAL\r\n");g_ui_mode=UI_MODE_CAL_CONFIRM;Ui_DrawConfirmScreen();}else if(Ui_Hit(&g_btn_start,x,y)){printf("[TOUCH] START\r\n");MotorControl_Start();}else if(Ui_Hit(&g_btn_stop,x,y)){printf("[TOUCH] STOP\r\n");MotorControl_Stop();}else if(Ui_Hit(&g_btn_plus,x,y)){printf("[TOUCH] DUTY+\r\n");MotorControl_SetDuty(MotorControl_GetDuty()+DUTY_STEP);}else if(Ui_Hit(&g_btn_minus,x,y)){printf("[TOUCH] DUTY-\r\n");MotorControl_SetDuty(MotorControl_GetDuty()-DUTY_STEP);}}else g_touch_has_xy=0U;g_touch_latch=1U;}}else{g_touch_latch=0U;g_touch_pen=0U;g_touch_has_xy=0U;} if((now-g_last_draw)>=250U){Ui_DrawStatus();g_last_draw=now;}}
