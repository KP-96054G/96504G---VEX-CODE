
#pragma once

#include <string>

//selector configuration
#define HUE 360
#define DEFAULT 1
#define AUTONS "Front", "Back", "Do Nothing"

namespace selector{

extern int auton;
const char *b[] = {AUTONS, ""};
void init(int hue = HUE, int default_auton = DEFAULT, const char **autons = b);

}


/* int autonSelection = 10;

		  int autonPark = 0;
		  
		  static void event_handler(lv_event_t * e)
		  {
			  lv_event_code_t code = lv_event_get_code(e);
			  lv_obj_t * obj = lv_event_get_target(e);
			  if(code == LV_EVENT_VALUE_CHANGED) {
				  uint32_t id = lv_btnmatrix_get_selected_btn(obj);
				  const char * txt = lv_btnmatrix_get_btn_text(obj, id);
		  
				  LV_LOG_USER("%s was pressed\n", txt);
			  }
		  }
		  
		  static const char *btnmMap[] = {"Stack", "1 Cube", "Disable", ""};
		  
		  
		  
		  void btnmatrix_1(void)
		  {
			  lv_obj_t * btnm1 = lv_btnmatrix_create(lv_scr_act());
			  lv_btnmatrix_set_map(btnm1, btnmMap);
			  lv_btnmatrix_set_btn_width(btnm1, 10, 2);        
			  lv_btnmatrix_set_btn_ctrl(btnm1, 10, LV_BTNMATRIX_CTRL_CHECKABLE);
			  lv_btnmatrix_set_btn_ctrl(btnm1, 11, LV_BTNMATRIX_CTRL_CHECKED);
			  lv_obj_align(btnm1, LV_ALIGN_CENTER, 0, 0);
			  lv_obj_add_event_cb(btnm1, event_handler, LV_EVENT_ALL, NULL);
		  }

		  //45static lv_res_t redobjAction(lv_obj_t *obj, const char *txt)
		  static lv_res_t redBtnmAction(lv_obj_t *btnm, const char *txt)

		  {
		  
			  printf("red button: %s released\n", txt);
		  
			  //if (txt == "Stack")
		  
			if (strcmp(txt, "Stack") == 0)
		  
			  {
		  
				  autonSelection = -1; // or whatever red close is
		  
			  }
		  
			  if (strcmp(txt, "1 Cube") == 0)
		  
			  {
		  
				  autonSelection = -2;
		  
			  }
		  
			if (strcmp(txt, "Disable") == 0)
		  
			{
		  
			  autonSelection = -3;
		  
			}
		  
			  return LV_RES_OK; // return OK because the button matrix is not deleted
		  
		  }
		  
		  static lv_res_t blueBtnmAction(lv_obj_t *btnm, const char *txt)

		  
		  {
		  
			  printf("blue button: %s released\n", txt);
		  
			  if (strcmp(txt, "Stack") == 0)
		  
			  {
		  
				  autonSelection = 1;
		  
			  }
		  
			  if (strcmp(txt, "1 Cube") == 0)
		  
			  {
		  
				  autonSelection = 2;
		  
			  }
		  
			if (strcmp(txt, "Disable") == 0)
		  
			{
		  
			  autonSelection = 3;
		  
			}
		  
			  return LV_RES_OK;
		  
		  }
		  
		  static lv_res_t skillsBtnAction(lv_obj_t *btn, const char *txt)
		  
		  {
		  
			  printf("blue button: %s released\n", txt);
		  
			  if (strcmp(txt, "Skill 5pt") == 0)
		  
			  {
		  
				  autonSelection = 0;
		  
			  }
		  
			  if (strcmp(txt, "Skill 10pt") == 0)
		  
			  {
		  
				  autonSelection = 10;
		  
			  }
		  
			  if (strcmp(txt, "Skill 16pt") == 0)
		  
			  {
		  
				  autonSelection = 20;
		  
			  }
		  
			  return LV_RES_OK;
		  
		  }


		
		  */