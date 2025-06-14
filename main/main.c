#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_lcd_sh8601.h"
#include "touch_bsp.h"
#include "i2c_bsp.h"
#include "icons.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"

#include "lvgl.h"


static const char *TAG = "example";
static SemaphoreHandle_t lvgl_mux = NULL;

#define LCD_HOST    SPI2_HOST


#if CONFIG_LV_COLOR_DEPTH == 32
#define LCD_BIT_PER_PIXEL       (24)
#elif CONFIG_LV_COLOR_DEPTH == 16
#define LCD_BIT_PER_PIXEL       (16)
#endif
void esp_draw_bitmap(uint16_t clorck,esp_lcd_panel_handle_t panel_handle);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_LCD_CS            (GPIO_NUM_6)
#define EXAMPLE_PIN_NUM_LCD_PCLK          (GPIO_NUM_47) 
#define EXAMPLE_PIN_NUM_LCD_DATA0         (GPIO_NUM_18)
#define EXAMPLE_PIN_NUM_LCD_DATA1         (GPIO_NUM_7)
#define EXAMPLE_PIN_NUM_LCD_DATA2         (GPIO_NUM_48)
#define EXAMPLE_PIN_NUM_LCD_DATA3         (GPIO_NUM_5)
#define EXAMPLE_PIN_NUM_LCD_RST           (GPIO_NUM_17)
#define EXAMPLE_PIN_NUM_BK_LIGHT          (-1)

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              536
#define EXAMPLE_LCD_V_RES              240

#define EXAMPLE_USE_TOUCH               1

#define EXAMPLE_LVGL_BUF_HEIGHT        (EXAMPLE_LCD_V_RES / 4)
#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2

static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {

    {0x11, (uint8_t []){0x00}, 0, 120},   
   // {0x44, (uint8_t []){0x01, 0xD1}, 2, 0},
   // {0x35, (uint8_t []){0x00}, 1, 0},
    {0x36, (uint8_t []){0xF0}, 1, 0},   
    {0x3A, (uint8_t []){0x55}, 1, 0},  //16bits-RGB565
    {0x2A, (uint8_t []){0x00,0x00,0x02,0x17}, 4, 0}, 
    {0x2B, (uint8_t []){0x00,0x00,0x00,0xEF}, 4, 0},
    {0x51, (uint8_t []){0x00}, 1, 10},
    {0x29, (uint8_t []){0x00}, 0, 10},
    {0x51, (uint8_t []){0xFF}, 1, 0},
};

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    const int offsetx1 = area->x1 ;
    const int offsetx2 = area->x2 ;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;

#if LCD_BIT_PER_PIXEL == 24
    uint8_t *to = (uint8_t *)color_map;
    uint8_t temp = 0;
    uint16_t pixel_num = (offsetx2 - offsetx1 + 1) * (offsety2 - offsety1 + 1);

    // Special dealing for first pixel
    temp = color_map[0].ch.blue;
    *to++ = color_map[0].ch.red;
    *to++ = color_map[0].ch.green;
    *to++ = temp;
    // Normal dealing for other pixels
    for (int i = 1; i < pixel_num; i++) {
        *to++ = color_map[i].ch.red;
        *to++ = color_map[i].ch.green;
        *to++ = color_map[i].ch.blue;
    }
#endif

    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}






lv_obj_t* arrow_left_off = NULL;
lv_obj_t* arrow_right_off = NULL;
lv_obj_t* hazard_off = NULL;
lv_obj_t* brake_off = NULL; // Pointer to the arrow object
lv_obj_t* high_beam_off = NULL; // Pointer to the arrow object
lv_obj_t* low_beam_off = NULL; // Pointer to the arrow object
lv_obj_t* brake_on = NULL; // Pointer to the arrow object
lv_obj_t* high_beam_on = NULL; // Pointer to the arrow object
lv_obj_t* low_beam_on = NULL; // Pointer to the arrow object
lv_anim_timeline_t *hazard_timeline = NULL; // Pointer to the hazard timeline


lv_obj_t* bmwLogo = NULL;
lv_obj_t* bmwText = NULL;
lv_obj_t* mainScreen = NULL;

bool high_beam_on_flag = false;
bool arrow_left_blinking = false;
bool arrow_right_blinking = false;
bool hazard_blinking = false;



/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
static void example_lvgl_update_cb(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

    switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    }
}

void example_lvgl_rounder_cb(struct _lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    uint16_t x1 = area->x1;
    uint16_t x2 = area->x2;

    uint16_t y1 = area->y1;
    uint16_t y2 = area->y2;

    // round the start of coordinate down to the nearest 2M number
    area->x1 = (x1 >> 1) << 1;
    area->y1 = (y1 >> 1) << 1;
    // round the end of coordinate up to the nearest 2N+1 number
    area->x2 = ((x2 >> 1) << 1) + 1;
    area->y2 = ((y2 >> 1) << 1) + 1;
}

#if EXAMPLE_USE_TOUCH
static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t tp_x,tp_y;
    uint8_t win;
    win = getTouch(&tp_x,&tp_y); 
    if (win) 
    {
      data->point.x = tp_x;
      data->point.y = tp_y;
      data->state = LV_INDEV_STATE_PRESSED;
    } 
    else 
    {
      data->state = LV_INDEV_STATE_RELEASED;
    }
}
#endif

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static bool example_lvgl_lock(int timeout_ms)
{
    assert(lvgl_mux && "bsp_display_start must be called first");

    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

static void example_lvgl_unlock(void)
{
    assert(lvgl_mux && "bsp_display_start must be called first");
    xSemaphoreGive(lvgl_mux);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (example_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

void blink_anim_cb(void * obj, int32_t v) {
    lv_obj_set_style_opa(obj, v, 0); // v goes from 0 to 255
}


void remove_arrow_left_off()
{
    lv_obj_del(arrow_left_off);
    arrow_left_off = NULL; // Clear the pointer after deletion
}

void remove_arrow_right_off()
{
    lv_obj_del(arrow_right_off);
    arrow_right_off = NULL; // Clear the pointer after deletion
}

void remove_hazard_off()
{
    lv_obj_del(hazard_off);
    hazard_off = NULL; // Clear the pointer after deletion
}

void stop_animation_arrow_left()
{
    lv_anim_del(arrow_left_off, blink_anim_cb); // Delete the animated object
    arrow_left_blinking = false; // Clear the flag
}

void stop_animation_arrow_right()
{
    lv_anim_del(arrow_right_off, blink_anim_cb); // Delete the animated object
    arrow_right_blinking = false; // Clear the flag
}

void stop_animation_hazard()
{
    if(hazard_timeline != NULL) {
        lv_anim_timeline_stop(hazard_timeline);
    }
    hazard_blinking = false; // Clear the flag
}

void create_blinking_arrow_left(lv_anim_timeline_t *timeline)
{
    if(arrow_left_off == NULL) {
        LV_IMG_DECLARE(arrow_left);  // Declare the image (auto-defined by converter)
        arrow_left_off = lv_img_create(mainScreen);     // Create image object
        lv_img_set_src(arrow_left_off, &arrow_left);                 // Set image source
        lv_obj_align(arrow_left_off, LV_ALIGN_CENTER, -90, -70);
    }
    // Create an animation to blink the arrow
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, arrow_left_off);
    lv_anim_set_values(&a, 0, 255); // Fade out to transparent
    lv_anim_set_time(&a, 500);      // 500 ms for one phase
    lv_anim_set_playback_time(&a, 500); // Fade back in
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE); // Repeat forever
    lv_anim_set_exec_cb(&a, blink_anim_cb);
    if(timeline == NULL) {
        lv_anim_start(&a);
    }else{
        lv_anim_timeline_add(timeline,0,&a); // Add the animation to the timeline
    }
    arrow_left_blinking = true; // Set the flag to indicate the arrow is blinking
}

void create_blinking_arrow_right(lv_anim_timeline_t *timeline)
{
    if(arrow_right_off == NULL) {
        LV_IMG_DECLARE(arrow_right);  // Declare the image (auto-defined by converter)
        arrow_right_off = lv_img_create(mainScreen);     // Create image object
        lv_img_set_src(arrow_right_off, &arrow_right);                 // Set image source
        lv_obj_align(arrow_right_off, LV_ALIGN_CENTER, 90, -70);
    }
    // Create an animation to blink the arrow
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, arrow_right_off);
    lv_anim_set_values(&a, 0, 255); // Fade out to transparent
    lv_anim_set_time(&a, 500);      // 500 ms for one phase
    lv_anim_set_playback_time(&a, 500); // Fade back in
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE); // Repeat forever
    lv_anim_set_exec_cb(&a, blink_anim_cb);
    if(timeline == NULL) {
        lv_anim_start(&a);
    }else{
        lv_anim_timeline_add(timeline, 0,&a); // Add the animation to the timeline
    }
    arrow_right_blinking = true; // Set the flag to indicate the arrow is blinking
}

void create_blinking_hazard(lv_anim_timeline_t *timeline)
{
    if(hazard_off == NULL) {
        LV_IMG_DECLARE(hazard);  // Declare the image (auto-defined by converter)
        hazard_off = lv_img_create(mainScreen);     // Create image object
        lv_img_set_src(hazard_off, &hazard);                 // Set image source
        lv_obj_align(hazard_off, LV_ALIGN_CENTER, 0, -70);
    }        

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, hazard_off);
    lv_anim_set_values(&a, 0, 255); // Fade out to transparent
    lv_anim_set_time(&a, 500);      // 500 ms for one phase
    lv_anim_set_playback_time(&a, 500); // Fade back in
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE); // Repeat forever
    lv_anim_set_exec_cb(&a, blink_anim_cb);
    if(timeline == NULL) {
        lv_anim_start(&a);
    }else{
        lv_anim_timeline_add(timeline, 0, &a); // Add the animation to the timeline
    }
    hazard_blinking = true; // Set the flag to indicate the hazard is blinking
}

void crete_blinking_hazard_group(void)
{
    lv_anim_timeline_t *timeline = lv_anim_timeline_create();
    create_blinking_arrow_left(timeline);
    create_blinking_arrow_right(timeline);
    create_blinking_hazard(timeline);
    lv_anim_timeline_start(timeline);
    hazard_timeline = timeline; // Store the timeline for later use
}


void create_blinking_dot(void)
{
    lv_obj_t * dot = lv_label_create(lv_scr_act());

    lv_obj_align(dot, LV_ALIGN_OUT_TOP_LEFT, 0, 0);
    lv_label_set_text(dot, ".");  // You can use other directions too

    // Set the arrow color to green
    lv_obj_set_style_text_color(dot, lv_color_hex(0xFFFFFF), 0);
 
    // Create an animation to blink the arrow
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, dot);
    lv_anim_set_values(&a, 0, 255); // Fade out to transparent
    lv_anim_set_time(&a, 500);      // 500 ms for one phase
    lv_anim_set_playback_time(&a, 500); // Fade back in
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE); // Repeat forever
    lv_anim_set_exec_cb(&a, blink_anim_cb);
    lv_anim_start(&a);
}


void show_arrow_left(int32_t v)
{
    if(arrow_left_off == NULL) {
        LV_IMG_DECLARE(arrow_left);  // Declare the image (auto-defined by converter)
        arrow_left_off = lv_img_create(mainScreen);     // Create image object
        lv_img_set_src(arrow_left_off, &arrow_left);                 // Set image source
        lv_obj_align(arrow_left_off, LV_ALIGN_CENTER, -90, -70);
    }
    lv_obj_set_style_opa(arrow_left_off, v, 0);
}

void show_arrow_right(int32_t v)
{
    LV_IMG_DECLARE(arrow_right);  // Declare the image (auto-defined by converter)

    if(arrow_right_off == NULL) {
        LV_IMG_DECLARE(arrow_right);  // Declare the image (auto-defined by converter)
        arrow_right_off = lv_img_create(mainScreen);     // Create image object
        lv_img_set_src(arrow_right_off, &arrow_right);                 // Set image source
        lv_obj_align(arrow_right_off, LV_ALIGN_CENTER, 90, -70);
    }
    lv_obj_set_style_opa(arrow_right_off, v, 0);
}

void show_hazard(int32_t v)
{
    if(hazard_off == NULL) {
        LV_IMG_DECLARE(hazard);  // Declare the image (auto-defined by converter)
        hazard_off = lv_img_create(mainScreen);     // Create image object
        lv_img_set_src(hazard_off, &hazard);                 // Set image source
        lv_obj_align(hazard_off, LV_ALIGN_CENTER, 0, -70);
    }
    lv_obj_set_style_opa(hazard_off, v, 0);
}

void show_power(int32_t v)
{
    LV_IMG_DECLARE(power);  // Declare the image (auto-defined by converter)

    lv_obj_t *img_power = lv_img_create(mainScreen);     // Create image object
    lv_img_set_src(img_power, &power);                 // Set image source
    lv_obj_align(img_power, LV_ALIGN_CENTER, -170, 30);
    lv_obj_set_style_opa(img_power, LV_OPA_COVER, 0);
}

void show_high_beam(int32_t v)
{
    if(high_beam_off == NULL) {
        LV_IMG_DECLARE(highBeam);  // Declare the image (auto-defined by converter)
        high_beam_off = lv_img_create(mainScreen);     // Create image object
        lv_img_set_src(high_beam_off, &highBeam);                 // Set image source
        lv_obj_align(high_beam_off, LV_ALIGN_CENTER, 170, -70);
    }
    lv_obj_set_style_opa(high_beam_off, v, 0);
}

void show_low_beam(int32_t v)
{
    if(low_beam_off == NULL) {
        LV_IMG_DECLARE(lowBeam);  // Declare the image (auto-defined by converter)
        low_beam_off = lv_img_create(mainScreen);     // Create image object
        lv_img_set_src(low_beam_off, &lowBeam);                 // Set image source
        lv_obj_align(low_beam_off, LV_ALIGN_CENTER, 170, 30);
    }
    lv_obj_set_style_opa(low_beam_off, v, 0);
}

void show_brake(int32_t v)
{
    if(brake_off == NULL) {
        LV_IMG_DECLARE(brake);  // Declare the image (auto-defined by converter)
        brake_off = lv_img_create(mainScreen);     // Create image object
        lv_img_set_src(brake_off, &brake);                 // Set image source
        lv_obj_align(brake_off, LV_ALIGN_CENTER, -170, -70);
    }
    lv_obj_set_style_opa(brake_off, v, 0);
}

void show_all_icons(void)
{
    show_arrow_left(50); // Show the left arrow
    show_arrow_right(50); // Show the right arrow
    show_hazard(50); // Show the hazard icon
    show_power(50); // Show the power icon
    show_high_beam(100); // Show the high beam icon
    show_low_beam(255); // Show the low beam icon
    show_brake(50); // Show the brake icon
}


void create_splash_screen(void)
{
    LV_IMG_DECLARE(bmw);  // Declare the image (auto-defined by converter)

    lv_obj_t *img = lv_img_create(lv_scr_act());     // Create image object
    lv_img_set_src(img, &bmw);                        // Set image source
//    lv_obj_align(img, LV_ALIGN_OUT_TOP_MID , 0, 70);  
    lv_obj_align(img, LV_ALIGN_CENTER, 0, -30);  


    lv_obj_t * label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "BMW MOTORRAD");
    lv_obj_align_to(label, img, LV_ALIGN_CENTER, -230, 70);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);

    // Start fully transparent
    lv_obj_set_style_opa(img, LV_OPA_TRANSP, 0);
    lv_obj_set_style_opa(label, LV_OPA_TRANSP, 0);

    bmwLogo = img;  // Store the image object for later use
    bmwText = label; // Store the label object for later use
}

/*
void fade_in(lv_obj_t * obj) {
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_exec_cb(&a, blink_anim_cb);
    lv_anim_set_values(&a, LV_OPA_TRANSP, LV_OPA_COVER);  // 0 to 255
    lv_anim_set_time(&a, 500);  // 0.5s fade in
    lv_anim_start(&a);
}

void fade_out(lv_obj_t * obj) {
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_exec_cb(&a, blink_anim_cb);
    lv_anim_set_values(&a, LV_OPA_COVER, LV_OPA_TRANSP);
    lv_anim_set_time(&a, 500);  // 0.5s fade out
    lv_anim_start(&a);
}

void fade_sequence() {
    create_splash_screen();
    fade_in(bmwLogo);
    fade_in(bmwText);

    // After 5 seconds (5000 ms), fade out
    vTaskDelay(pdMS_TO_TICKS(5000));
    fade_out(bmwLogo);
    fade_out(bmwText);

}
*/

void show_main_screen(void)
{
    lv_scr_load(mainScreen);
}

void splash_timer_cb(lv_timer_t * timer) {
    show_main_screen();
    lv_timer_del(timer);
}

void show_splash_screen(void)
{
    LV_IMG_DECLARE(bmw); 
    // Create a new screen object
    lv_obj_t *splash = lv_obj_create(NULL);
    lv_obj_clear_flag(splash, LV_OBJ_FLAG_SCROLLABLE); // Make it non-scrollable
    lv_obj_set_style_bg_color(splash, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(splash, LV_OPA_COVER, 0);

    // Create and add the logo image
    lv_obj_t *img_logo = lv_img_create(splash);
    lv_img_set_src(img_logo, &bmw); // Use the variable from your image file
    lv_obj_align(img_logo, LV_ALIGN_CENTER, 0, -40);  // Centered, slightly up

    // Create and add the text label
    lv_obj_t *label = lv_label_create(splash);
    lv_label_set_text(label, "BMW MOTORRAD");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0); // White color
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 70); // Centered, slightly down

    // Load the splash screen
    lv_scr_load(splash);

    lv_timer_create(splash_timer_cb, 5000, NULL);

}

/*void my_touch_function()
{
    // Your custom logic here
    ESP_LOGI(TAG, "Touch area clicked!");
    if (arrow_off != NULL)
    {
        remove_arrow_off(); // Remove the arrow if it exists
        create_blinking_arrow();
    }
    else
    {
        stop_animation();
        show_arrow(); // Show the arrow if it doesn't exist
    }
}

static void touch_event_handler(lv_event_t *e) 
{
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_CLICKED) {
        my_touch_function();  // Call your function
    }
}*/

static void touch_event_handler_arrow_left(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED) {
        if(!arrow_left_blinking) {
            if(arrow_right_blinking){
                stop_animation_arrow_right();
                show_arrow_right(50); // Show the arrow if it doesn't exist
            }
            remove_arrow_left_off(); // Remove the arrow if it exists
            create_blinking_arrow_left(NULL);
        }else{
            stop_animation_arrow_left();
            show_arrow_left(50); // Show the arrow if it doesn't exist
        }
    }
}

static void touch_event_handler_arrow_right(lv_event_t *e) 
{
    lv_event_code_t code = lv_event_get_code(e);
    

    if (code == LV_EVENT_CLICKED) {
        if(!arrow_right_blinking) {
            if(arrow_left_blinking){
                stop_animation_arrow_left();
                show_arrow_left(50); // Show the arrow if it doesn't exist
            }
            remove_arrow_right_off(); // Remove the arrow if it exists
            create_blinking_arrow_right(NULL);
        }else{
            stop_animation_arrow_right();
            show_arrow_right(50); // Show the arrow if it doesn't exist
        }
    }
}

static void touch_event_handler_hazard(lv_event_t *e) 
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED) {
          if(!hazard_blinking) {
                remove_hazard_off(); // Remove the arrow if it exists
                remove_arrow_left_off();
                remove_arrow_right_off();
                crete_blinking_hazard_group(); // Create the hazard group with blinking arrows
            }else{
                stop_animation_hazard(); // Stop the hazard animation
                remove_hazard_off(); // Remove the arrow if it exists
                remove_arrow_left_off();
                remove_arrow_right_off();
                show_hazard(50); // Show the arrow if it doesn't exist
                show_arrow_right(50); // Show the arrow if it doesn't exist
                show_arrow_left(50); // Show the arrow if it doesn't exist
            }
        }
}

static void touch_event_handler_power(lv_event_t *e) 
{
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_CLICKED) {
    }

}

static void touch_event_handler_high_beam(lv_event_t *e) 
{
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_CLICKED) {
        if(high_beam_on_flag)
        {
            high_beam_on_flag = false;
            show_low_beam(255); // Show the low beam icon
            show_high_beam(100); // Hide the high beam icon
        }else{
            high_beam_on_flag = true;
            show_low_beam(50); // Hide the low beam icon
            show_high_beam(255); // Show the high beam icon
        }
    }
}

static void touch_event_handler_brake(lv_event_t *e) 
{
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_PRESSED) {
        // Handle brake touch event
        show_brake(255); // Show the brake icon
    }else{
        if(code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) 
            show_brake(50); // Hide the brake icon
    }
}

void create_touch_area_arrow_left(void) 
{
    lv_obj_t *touch_area_arrow_left = lv_obj_create(mainScreen);
    lv_obj_set_size(touch_area_arrow_left, 88, 88);        // Set the size of the touch area
    lv_obj_align(touch_area_arrow_left, LV_ALIGN_CENTER, -90, -70);  // Align to center or custom position

    lv_obj_set_style_bg_opa(touch_area_arrow_left, LV_OPA_TRANSP, 0);  // No background
    lv_obj_set_style_border_opa(touch_area_arrow_left, LV_OPA_TRANSP, 0);  // No border
    // Make it react to clicks
    lv_obj_add_event_cb(touch_area_arrow_left, touch_event_handler_arrow_left, LV_EVENT_ALL, NULL);
}

void create_touch_area_arrow_right(void) 
{
    lv_obj_t *touch_area_arrow_right = lv_obj_create(mainScreen);
    lv_obj_set_size(touch_area_arrow_right, 88, 88);        // Set the size of the touch area
    lv_obj_align(touch_area_arrow_right, LV_ALIGN_CENTER, 90, -70);  // Align to center or custom position

    lv_obj_set_style_bg_opa(touch_area_arrow_right, LV_OPA_TRANSP, 0);  // No background
    lv_obj_set_style_border_opa(touch_area_arrow_right, LV_OPA_TRANSP, 0);  // No border
    // Make it react to clicks
    lv_obj_add_event_cb(touch_area_arrow_right, touch_event_handler_arrow_right, LV_EVENT_ALL, NULL);
}

void create_touch_area_hazard(void) 
{
    lv_obj_t *touch_area_hazard = lv_obj_create(mainScreen);
    lv_obj_set_size(touch_area_hazard, 88, 88);        // Set the size of the touch area
    lv_obj_align(touch_area_hazard, LV_ALIGN_CENTER, 0, -70);  // Align to center or custom position

    lv_obj_set_style_bg_opa(touch_area_hazard, LV_OPA_TRANSP, 0);  // No background
    lv_obj_set_style_border_opa(touch_area_hazard, LV_OPA_TRANSP, 0);  // No border
    // Make it react to clicks
    lv_obj_add_event_cb(touch_area_hazard, touch_event_handler_hazard, LV_EVENT_ALL, NULL);
}

void create_touch_area_power(void) 
{
    lv_obj_t *touch_area_power = lv_obj_create(mainScreen);
    lv_obj_set_size(touch_area_power, 88, 88);        // Set the size of the touch area
    lv_obj_align(touch_area_power, LV_ALIGN_CENTER, -170, 30);  // Align to center or custom position

    lv_obj_set_style_bg_opa(touch_area_power, LV_OPA_TRANSP, 0);  // No background
    lv_obj_set_style_border_opa(touch_area_power, LV_OPA_TRANSP, 0);  // No border
    // Make it react to clicks
    lv_obj_add_event_cb(touch_area_power, touch_event_handler_power, LV_EVENT_ALL, NULL);
}

void create_touch_area_high_beam(void) 
{
    lv_obj_t *touch_area_high_beam = lv_obj_create(mainScreen);
    lv_obj_set_size(touch_area_high_beam, 88, 88);        // Set the size of the touch area
    lv_obj_align(touch_area_high_beam, LV_ALIGN_CENTER, 170, -70);  // Align to center or custom position

    lv_obj_set_style_bg_opa(touch_area_high_beam, LV_OPA_TRANSP, 0);  // No background
    lv_obj_set_style_border_opa(touch_area_high_beam, LV_OPA_TRANSP, 0);  // No border
    // Make it react to clicks
    lv_obj_add_event_cb(touch_area_high_beam, touch_event_handler_high_beam, LV_EVENT_ALL, NULL);
}

void create_touch_area_brake(void) 
{
    lv_obj_t *touch_area_brake = lv_obj_create(mainScreen );
    lv_obj_set_size(touch_area_brake, 88, 88);        // Set the size of the touch area
    lv_obj_align(touch_area_brake, LV_ALIGN_CENTER, -170, -70);  // Align to center or custom position

    lv_obj_set_style_bg_opa(touch_area_brake, LV_OPA_TRANSP, 0);  // No background
    lv_obj_set_style_border_opa(touch_area_brake, LV_OPA_TRANSP, 0);  // No border
    // Make it react to clicks
    lv_obj_add_event_cb(touch_area_brake, touch_event_handler_brake, LV_EVENT_ALL, NULL);
}

void create_touch_areas(void)
{
    create_touch_area_arrow_left();
    create_touch_area_arrow_right();
    create_touch_area_hazard();
    create_touch_area_power();
    create_touch_area_high_beam();
    create_touch_area_brake();
}

void app_main(void)
{
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif

    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_PCLK,
                                                                 EXAMPLE_PIN_NUM_LCD_DATA0,
                                                                 EXAMPLE_PIN_NUM_LCD_DATA1,
                                                                 EXAMPLE_PIN_NUM_LCD_DATA2,
                                                                 EXAMPLE_PIN_NUM_LCD_DATA3,
                                                                 EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * LCD_BIT_PER_PIXEL / 8);
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS,
                                                                                example_notify_lvgl_flush_ready,
                                                                                &disp_drv);
    sh8601_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_LOGI(TAG, "Install SH8601 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    I2C_master_Init(); //I2C Init
#if EXAMPLE_USE_TOUCH
    touch_Init();
#endif

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
#endif
    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.rounder_cb = example_lvgl_rounder_cb;
    disp_drv.drv_update_cb = example_lvgl_update_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    lv_obj_t * scr = lv_scr_act();  // Get the active screen
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);  // Set background color to black
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);       // Set background opacity to cover

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

#if EXAMPLE_USE_TOUCH
    static lv_indev_drv_t indev_drv;    // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    lv_indev_drv_register(&indev_drv);
#endif

    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Display LVGL demos");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (example_lvgl_lock(-1)) {
        
        //setup_ui(&guider_ui);
   	    //events_init(&guider_ui);
        //user_bsp_Init(&guider_ui);
        //lv_demo_widgets();      /* A widgets example */
        //lv_demo_music();        /* A modern, smartphone-like music player demo. */
        //lv_demo_stress();       /* A stress test for LVGL. */
        //lv_demo_benchmark();    /* A demo to measure the performance of LVGL or to compare different settings. */
        //; // Create a blinking arrow on the active screen
        ESP_LOGI(TAG, "Show animation");
        mainScreen= lv_scr_act(); // Get the current screen

 
        show_splash_screen();
        
        create_touch_areas();
        show_all_icons();
        create_blinking_dot();
        //create_blinking_arrow();


        example_lvgl_unlock();
    }
}