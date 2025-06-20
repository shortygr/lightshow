#include <stdio.h>
#include <stdint.h>

#include "lvgl.h"

class LightSymbol {
public:    
    LightSymbol(lv_obj_t* screen);

    ~LightSymbol();

    virtual void touch_event_handler_hazard(lv_event_t *e); 
    virtual void create_touch_area(lv_coord_t x_ofs, lv_coord_t y_ofs, lv_coord_t w, lv_coord_t h);

}
 