/**
 * @file        lv_obj_functions.cpp
 * @brief       object functions for lvgl
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @author      Ing. Kurt Baumann (OE1KBC)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-03-24
 */

#include "lv_obj_functions.h"
#include <configuration.h>
#include <aprs_structures.h>
#include <debugconf.h>
#include <loop_functions.h>
#include "tdeck_main.h"
#include "tdeck_extern.h"
#include "lv_obj_functions_extern.h"
#include "tdeck_helpers.h"
#include <loop_functions_extern.h>
#include <math.h>
#include <cstring>
#include <vector>

#include "event_functions.h"
#include <lora_setchip.h>

#include <esp32/esp32_flash.h>

#if defined(ENABLE_AUDIO)
#include <esp32/esp32_audio.h>
#endif

int         iKeyBoardType=1;

lv_obj_t    *btnlabelup;

lv_indev_t  *kb_indev = NULL;
lv_indev_t  *mouse_indev = NULL;
lv_indev_t  *touch_indev = NULL;

static lv_style_t ta_input_cursor;
static lv_obj_t *tab_menu_header = NULL;
static lv_obj_t *tab_menu_button = NULL;
static lv_obj_t *tab_menu_icon_label = NULL;
static lv_obj_t *header_time_label = NULL;
static lv_obj_t *header_sat_label = NULL;
static lv_obj_t *header_sat_icon = NULL;
static lv_obj_t *header_batt_label = NULL;
static lv_obj_t *header_batt_icon = NULL;
static lv_obj_t *header_locator_label = NULL;
static bool tab_menu_visible = false;

enum class MsgBubbleType
{
    Incoming = 0,
    Outgoing = 1,
    System = 2
};

struct MsgBubble
{
    MsgBubbleType type = MsgBubbleType::Incoming;
    String header;
    String timestamp;
    String body;
};

struct MsgTabEntry
{
    String group;
    lv_obj_t *button = NULL;
    std::vector<MsgBubble> bubbles;
};

static std::vector<MsgTabEntry> msg_tab_entries;
static int msg_active_tab_index = -1;
static lv_obj_t *msg_tab_bar = NULL;
static lv_obj_t *msg_tab_hint_label = NULL;

static void update_header_sat_indicator(void);
static void update_header_batt_indicator(float batt, int proz);
static void apply_tab_bar_styles(void);
static void msg_list_show_hint(const char *text);
static void init_msg_tab_bar(lv_obj_t *parent);
static void msg_tabs_update_hint(void);
static void ensure_msg_styles(void);
static void msg_list_clear(void);
static void msg_render_active_tab(void);
static void msg_list_append_bubble(const MsgBubble &bubble);
static int clamp_int(int value, int min_val, int max_val);

#ifndef DEFAULT_LOCATOR_TEXT
#define DEFAULT_LOCATOR_TEXT "JJ00AAAA"
#endif

#ifndef MSG_TAB_MAX_MESSAGES
#define MSG_TAB_MAX_MESSAGES 128
#endif

static bool msg_styles_ready = false;
static lv_style_t msg_style_incoming;
static lv_style_t msg_style_outgoing;
static lv_style_t msg_style_system;

lv_obj_t    *setup_callsign;
lv_obj_t    *setup_lat;
lv_obj_t    *setup_lon;
lv_obj_t    *setup_lat_c;
lv_obj_t    *setup_lon_c;
lv_obj_t    *setup_alt;
lv_obj_t    *setup_aprsgroup;
lv_obj_t    *setup_aprssymbol;
lv_obj_t    *setup_stone;
lv_obj_t    *setup_mtone;
lv_obj_t    *setup_name;
lv_obj_t    *setup_grc0;
lv_obj_t    *setup_grc1;
lv_obj_t    *setup_grc2;
lv_obj_t    *setup_grc3;
lv_obj_t    *setup_grc4;
lv_obj_t    *setup_grc5;
lv_obj_t    *setup_utc;

lv_obj_t    *btn_msg_id_label;
lv_obj_t    *btn_ack_id_label;

lv_obj_t    *msg_list = NULL;
lv_obj_t    *track_ta;

static lv_obj_t *msg_list_hint_label = NULL;

lv_obj_t    *btn_time_label = NULL;
lv_obj_t    *btn_time_label1 = NULL;
lv_obj_t    *btn_time_label2 = NULL;
lv_obj_t    *btn_time_label4 = NULL;
lv_obj_t    *btn_batt_label = NULL;
lv_obj_t    *btn_batt_label1 = NULL;
lv_obj_t    *btn_batt_label2 = NULL;
lv_obj_t    *btn_batt_label4 = NULL;
lv_obj_t    *text_input = NULL;
lv_obj_t    *position_ta = NULL;
lv_obj_t    *map_ta = NULL;
lv_obj_t    *mheard_ta = NULL;
lv_obj_t    *path_ta = NULL;
lv_obj_t    *tv = NULL;
lv_obj_t    *dm_callsign = NULL;
lv_obj_t    *dropdown_aprs = NULL;
lv_obj_t    *dropdown_country = NULL;
lv_obj_t    *dropdown_mapselect = NULL;
lv_obj_t    *dropdown_modusselect = NULL;
lv_obj_t    *web_sw = NULL;
lv_obj_t    *mesh_sw = NULL;
lv_obj_t    *noallmsg_sw = NULL;
lv_obj_t    *gpson_sw = NULL;
lv_obj_t    *track_sw = NULL;
lv_obj_t    *wifiap_sw = NULL;
static bool is_numeric_string(const String &value);
static void msg_focus_and_alert(bool bWithAudio);
static void update_header_locator_label(void);
static bool compute_locator_from_settings(char *buffer, size_t len);
static bool compute_maidenhead_locator(double lat, double lon, char *buffer, size_t len);

static lv_obj_t *get_tab_bar()
{
    if(tv == NULL)
        return NULL;

    return lv_tabview_get_tab_btns(tv);
}

static void dm_callsign_focus_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * ta = lv_event_get_target(e);

    if(code == LV_EVENT_FOCUSED)
    {
        /* show our cursor style when focused */
        lv_obj_add_style(ta, &ta_input_cursor, LV_PART_CURSOR);
    }
    else if(code == LV_EVENT_DEFOCUSED)
    {
        /* hide the cursor style when not focused */
        lv_obj_remove_style(ta, &ta_input_cursor, LV_PART_CURSOR);
    }
}

static void text_input_focus_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * ta = lv_event_get_target(e);

    if(code == LV_EVENT_FOCUSED)
    {
        /* show our cursor style when focused and place cursor at end of text */
        lv_obj_add_style(ta, &ta_input_cursor, LV_PART_CURSOR);
        const char *txt = lv_textarea_get_text(ta);
        if(txt != NULL)
            lv_textarea_set_cursor_pos(ta, (int)strlen(txt));
        else
            lv_textarea_set_cursor_pos(ta, 0);
    }
    else if(code == LV_EVENT_DEFOCUSED)
    {
        /* hide our cursor style when not focused */
        lv_obj_remove_style(ta, &ta_input_cursor, LV_PART_CURSOR);
    }
}

static void update_tab_button_state(bool show)
{
    if(tab_menu_button != NULL)
    {
        if(show)
            lv_obj_add_state(tab_menu_button, LV_STATE_CHECKED);
        else
            lv_obj_clear_state(tab_menu_button, LV_STATE_CHECKED);
    }

    if(tab_menu_icon_label != NULL)
    {
        lv_color_t color = show ? lv_palette_main(LV_PALETTE_RED)
                                : lv_palette_main(LV_PALETTE_LIGHT_GREEN);
        lv_obj_set_style_text_color(tab_menu_icon_label, color, LV_PART_MAIN);
    }
}

static void tdeck_set_tab_menu_visible(bool show)
{
    lv_obj_t *tab_bar = get_tab_bar();

    if(tab_bar == NULL)
        return;

    if(show)
    {
        lv_obj_clear_flag(tab_bar, LV_OBJ_FLAG_HIDDEN);
        tab_menu_visible = true;
    }
    else
    {
        lv_obj_add_flag(tab_bar, LV_OBJ_FLAG_HIDDEN);
        tab_menu_visible = false;
    }

    update_tab_button_state(show);
}

void tdeck_hide_tab_menu(void)
{
    tdeck_set_tab_menu_visible(false);
}

void tdeck_show_tab_menu(void)
{
    tdeck_set_tab_menu_visible(true);
}

void tdeck_toggle_tab_menu(void)
{
    tdeck_set_tab_menu_visible(!tab_menu_visible);
}

bool tdeck_tab_menu_is_visible(void)
{
    return tab_menu_visible;
}

static void tab_menu_button_event_cb(lv_event_t * e)
{
    if(lv_event_get_code(e) == LV_EVENT_CLICKED)
    {
        tdeck_toggle_tab_menu();
    }
}

//////////////////////////////////////////////
// MAP variables
LV_IMG_DECLARE(map_europe);
LV_IMG_DECLARE(map_deutschland);
LV_IMG_DECLARE(map_oesterreich);
LV_IMG_DECLARE(map_wien_umgebung);
LV_IMG_DECLARE(map_wien);

double map_lat_min[MAX_MAP]={0};
double map_lat_max[MAX_MAP]={0};
double map_lon_min[MAX_MAP]={0};
double map_lon_max[MAX_MAP]={0};

int map_x[MAX_MAP] = {0};
int map_y[MAX_MAP] = {0};

//////////////////////////////////////////////
// MAP points
lv_obj_t * map_point[MAX_POINTS];

String map_point_call[MAX_POINTS];
double map_point_lat[MAX_POINTS];
double map_point_lon[MAX_POINTS];
int map_point_count = 0;

//
//////////////////////////////////////////////

String map_pos_call[MAX_POINTS];
double map_pos_lat[MAX_POINTS];
double map_pos_lon[MAX_POINTS];
int map_pos_count = 0;

String getCountryDropbox();

/**
 * defines GUI layout
 */
void setDisplayLayout(lv_obj_t *parent)
{
    static lv_style_t lable_style;
    lv_style_init(&lable_style);
    lv_style_set_text_color(&lable_style, lv_color_white());

    static lv_style_t bg_style;
    lv_style_init(&bg_style);
    lv_style_set_text_color(&bg_style, lv_color_white());
    //lv_style_set_bg_img_src(&bg_style, &image);
    const lv_coord_t screen_w = lv_disp_get_hor_res(NULL);
    const lv_coord_t screen_h = lv_disp_get_ver_res(NULL);
    const lv_coord_t header_height = 32;

    tab_menu_header = lv_obj_create(parent);
    lv_obj_set_size(tab_menu_header, screen_w, header_height);
    lv_obj_set_style_bg_color(tab_menu_header, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(tab_menu_header, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(tab_menu_header, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(tab_menu_header, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(tab_menu_header, 4, LV_PART_MAIN);
    lv_obj_clear_flag(tab_menu_header, LV_OBJ_FLAG_SCROLLABLE);

    tab_menu_button = lv_btn_create(tab_menu_header);
    lv_obj_set_size(tab_menu_button, 40, header_height - 8);
    lv_obj_align(tab_menu_button, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_add_event_cb(tab_menu_button, tab_menu_button_event_cb, LV_EVENT_CLICKED, NULL);
    lv_color_t header_blue = lv_palette_main(LV_PALETTE_BLUE);
    lv_obj_set_style_bg_color(tab_menu_button, header_blue, LV_PART_MAIN);
    lv_obj_set_style_bg_color(tab_menu_button, header_blue, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_border_width(tab_menu_button, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(tab_menu_button, 4, LV_PART_MAIN);

    tab_menu_icon_label = lv_label_create(tab_menu_button);
    lv_label_set_text(tab_menu_icon_label, LV_SYMBOL_LIST);
    lv_obj_set_style_text_color(tab_menu_icon_label, lv_palette_main(LV_PALETTE_LIGHT_GREEN), LV_PART_MAIN);
    lv_obj_center(tab_menu_icon_label);

    header_time_label = lv_label_create(tab_menu_header);
    lv_label_set_text(header_time_label, "--:--");
    lv_label_set_long_mode(header_time_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_color(header_time_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(header_time_label, LV_ALIGN_LEFT_MID, 48, 0);

    header_sat_label = lv_label_create(tab_menu_header);
    lv_label_set_text(header_sat_label, "0");
    lv_label_set_long_mode(header_sat_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_color(header_sat_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(header_sat_label, LV_ALIGN_RIGHT_MID, 0, 0);

    header_sat_icon = lv_label_create(tab_menu_header);
    lv_label_set_text(header_sat_icon, LV_SYMBOL_GPS); // Symbol stammt von WpZoom (CC BY-SA 3.0, siehe README)
    lv_obj_align_to(header_sat_icon, header_sat_label, LV_ALIGN_OUT_LEFT_MID, -6, 0);

    header_batt_label = lv_label_create(tab_menu_header);
    lv_label_set_text(header_batt_label, "0%");
    lv_label_set_long_mode(header_batt_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_color(header_batt_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_align_to(header_batt_label, header_sat_icon, LV_ALIGN_OUT_LEFT_MID, -20, 0);

    header_batt_icon = lv_label_create(tab_menu_header);
    lv_label_set_text(header_batt_icon, LV_SYMBOL_BATTERY_EMPTY);
    lv_obj_align_to(header_batt_icon, header_batt_label, LV_ALIGN_OUT_LEFT_MID, -6, 0);

    header_locator_label = lv_label_create(tab_menu_header);
    lv_label_set_text(header_locator_label, "JJ00AAAA");
    lv_label_set_long_mode(header_locator_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_color(header_locator_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(header_locator_label, LV_ALIGN_CENTER, 0, 0);

    update_header_batt_indicator(global_batt > 0.0f ? global_batt / 1000.0f : 0.0f, global_proz);
    update_header_sat_indicator();
    update_header_locator_label();

    tv = lv_tabview_create(parent, LV_DIR_TOP, 42);
    lv_obj_set_size(tv, screen_w, LV_MAX(0, screen_h - header_height));
    lv_obj_set_pos(tv, 0, header_height);
    lv_obj_add_style(tv, &bg_style, LV_PART_MAIN);
    lv_obj_add_event_cb(tv, tv_event_cb, LV_EVENT_ALL, NULL);

    lv_obj_t *tab_content = lv_tabview_get_content(tv);
    if(tab_content != NULL)
    {
        lv_obj_set_scroll_dir(tab_content, LV_DIR_NONE);
    }

        /* Prefer a 'chat' / message icon if available, fall back to envelope or "MSG" */
        const char *tab_icon_msg =
    #ifdef LV_SYMBOL_MESSAGE
        LV_SYMBOL_MESSAGE
    #elif defined(LV_SYMBOL_ENVELOPE)
        LV_SYMBOL_ENVELOPE
    #else
        "MSG"
    #endif
        ;

        /* Prefer a keyboard/edit icon for the send tab if available */
        const char *tab_icon_snd =
    #ifdef LV_SYMBOL_KEYBOARD
        LV_SYMBOL_KEYBOARD
    #elif defined(LV_SYMBOL_EDIT)
        LV_SYMBOL_EDIT
    #else
        "SND"
    #endif
        ;

        lv_obj_t *t2 = lv_tabview_add_tab(tv, tab_icon_msg);
        lv_obj_t *t5 = lv_tabview_add_tab(tv, tab_icon_snd);
            lv_obj_t *t3 = lv_tabview_add_tab(tv, "POS");
            /* Prefer a globe/map icon for the Map tab if available */
            const char *tab_icon_map =
        #ifdef LV_SYMBOL_GLOBE
            LV_SYMBOL_GLOBE
        #elif defined(LV_SYMBOL_MAP)
            LV_SYMBOL_MAP
        #elif defined(LV_SYMBOL_LOCATION)
            LV_SYMBOL_LOCATION
        #else
            "MAP"
        #endif
            ;
            /* Prefer a GPS symbol for the GPS tab if available */
            const char *tab_icon_gps =
        #ifdef LV_SYMBOL_GPS
            LV_SYMBOL_GPS
        #else
            "GPS"
        #endif
            ;
            lv_obj_t *t7 = lv_tabview_add_tab(tv, tab_icon_map);
            lv_obj_t *t6 = lv_tabview_add_tab(tv, tab_icon_gps);
    lv_obj_t *t4 = lv_tabview_add_tab(tv, "MHD");
    lv_obj_t *t8 = lv_tabview_add_tab(tv, "PATH");
    lv_obj_t *t1 = lv_tabview_add_tab(tv, LV_SYMBOL_SETTINGS);

    lv_obj_set_scroll_dir(t2, LV_DIR_VER);
    /* Disable vertical scrolling for the SND tab (no up/down scrolling needed) */
    lv_obj_set_scroll_dir(t5, LV_DIR_NONE);
    lv_obj_set_scrollbar_mode(t5, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(t3, LV_DIR_VER);
    lv_obj_set_scroll_dir(t7, LV_DIR_VER);
    lv_obj_set_scroll_dir(t6, LV_DIR_VER);
    lv_obj_set_scroll_dir(t4, LV_DIR_VER);
    lv_obj_set_scroll_dir(t8, LV_DIR_VER);
    lv_obj_set_scroll_dir(t1, LV_DIR_VER);

    lv_obj_add_event_cb(tv, tabview_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    tdeck_hide_tab_menu();
    apply_tab_bar_styles();

    static lv_style_t ta_style;
    lv_style_init(&ta_style);
    lv_style_set_text_color(&ta_style, lv_color_black());
    lv_style_set_bg_opa(&ta_style, LV_OPA_100);
    lv_style_set_bg_color(&ta_style, lv_color_white());
    lv_style_set_border_color(&ta_style, lv_color_black());
    lv_style_set_line_width(&ta_style, 4);
    /* Make single-line small textarea text left-aligned and vertically centered */
    lv_style_set_text_align(&ta_style, LV_TEXT_ALIGN_LEFT);
    lv_style_set_pad_top(&ta_style, 4);
    lv_style_set_pad_bottom(&ta_style, 4);
    lv_style_set_pad_left(&ta_style, 4);

    static lv_style_t tr_style;
    lv_style_set_text_font(&tr_style, &lv_font_unscii_16);
    lv_style_set_text_color(&tr_style, lv_color_black());
    lv_style_set_bg_opa(&tr_style, LV_OPA_100);
    lv_style_set_bg_color(&tr_style, lv_color_white());
    lv_style_set_border_color(&tr_style, lv_color_black());
    lv_style_set_line_width(&tr_style, 4);


    static lv_style_t ta_input_style;
    lv_style_init(&ta_input_style);
    lv_style_set_text_color(&ta_input_style, lv_color_black()); // BLAU lv_color_make(0x1C, 0x73, 0xFF));
    lv_style_set_bg_opa(&ta_input_style, LV_OPA_100);
    lv_style_set_bg_color(&ta_input_style, lv_color_white());

    lv_style_init(&ta_input_cursor);
    lv_style_set_bg_opa(&ta_input_cursor, LV_OPA_COVER);
    lv_style_set_bg_color(&ta_input_cursor, lv_color_black());
    /* Reduce cursor border to shrink overall cursor size by ~2px */
    lv_style_set_border_width(&ta_input_cursor, 0);

    static lv_style_t cell_style;
    static lv_style_t cell_style1;
    static lv_style_t cell_style2;

    lv_style_init(&cell_style);
    lv_style_set_pad_top(&cell_style, 0);
    lv_style_set_pad_bottom(&cell_style, 0);
    lv_style_set_pad_left(&cell_style, 0);
    lv_style_set_pad_right(&cell_style, 0);
    lv_style_set_bg_color(&cell_style, lv_color_white());

    lv_style_init(&cell_style1);
    lv_style_set_pad_top(&cell_style1, 1);
    lv_style_set_pad_bottom(&cell_style1, 1);
    lv_style_set_pad_left(&cell_style1, 2);
    lv_style_set_pad_right(&cell_style1, 2);
    lv_style_set_bg_color(&cell_style1, lv_color_white());
    lv_style_set_text_color(&cell_style1, lv_color_black());
    lv_style_set_border_width(&cell_style1, 1);
    lv_style_set_border_color(&cell_style1, lv_color_black());
    lv_style_set_border_side(&cell_style1, LV_BORDER_SIDE_FULL);

    lv_style_init(&cell_style2);
    lv_style_set_pad_top(&cell_style2, 1);
    lv_style_set_pad_bottom(&cell_style2, 1);
    lv_style_set_pad_left(&cell_style2, 2);
    lv_style_set_pad_right(&cell_style2, 2);
    lv_style_set_bg_color(&cell_style2, lv_color_white());
    lv_style_set_text_color(&cell_style2, lv_color_black());
    lv_style_set_border_width(&cell_style2, 1);
    lv_style_set_border_color(&cell_style2, lv_color_black());
    lv_style_set_border_side(&cell_style2, LV_BORDER_SIDE_FULL);


    ////////////////////////////////////////////////////////////////////////////
    // LAYOUT START HERE
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    // SETUP

    // CALL
    lv_obj_t * btnsetup_callsign = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_callsign, 0, 0);
    lv_obj_set_size(btnsetup_callsign, 50, 25);

    lv_obj_t * label_btnsetup_callsign = lv_label_create(btnsetup_callsign);
    lv_label_set_text(label_btnsetup_callsign, "CALL");
    lv_obj_center(label_btnsetup_callsign);

    setup_callsign = lv_textarea_create(t1);
    lv_textarea_set_one_line(setup_callsign, true);
    lv_textarea_set_text_selection(setup_callsign, false);
    lv_obj_align(setup_callsign, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_callsign, 55, 0);
    lv_obj_set_size(setup_callsign, 100, 30);
    lv_textarea_set_text(setup_callsign, "");
    lv_textarea_set_max_length(setup_callsign, 9);
    lv_obj_add_style(setup_callsign, &ta_style, LV_PART_MAIN);
    
    lv_textarea_set_accepted_chars(setup_callsign, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-");

    // LAT
    lv_obj_t * btnsetup_lat = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_lat, 0, 32);
    lv_obj_set_size(btnsetup_lat, 50, 25);

    lv_obj_t * label_btnsetup_lat = lv_label_create(btnsetup_lat);
    lv_label_set_text(label_btnsetup_lat, "LAT");
    lv_obj_center(label_btnsetup_lat);

    setup_lat = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_lat, false);
    lv_obj_align(setup_lat, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_lat, 55, 30);
    lv_obj_set_size(setup_lat, 100, 30);
    lv_textarea_set_text(setup_lat, "");
    lv_textarea_set_max_length(setup_lat, 8);
    lv_obj_add_style(setup_lat, &ta_style, LV_PART_MAIN);
    
    lv_textarea_set_accepted_chars(setup_lat, "0123456789.");

    setup_lat_c = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_lat_c, false);
    lv_obj_align(setup_lat_c, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_lat_c, 155, 30);
    lv_obj_set_size(setup_lat_c, 40, 30);
    lv_textarea_set_text(setup_lat_c, "");
    lv_textarea_set_max_length(setup_lat_c, 1);
    lv_obj_add_style(setup_lat_c, &ta_style, LV_PART_MAIN);
    
    lv_textarea_set_accepted_chars(setup_lat_c, "NS");

    // LON
    lv_obj_t * btnsetup_lon = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_lon, 0, 62);
    lv_obj_set_size(btnsetup_lon, 50, 25);

    lv_obj_t * label_btnsetup_lon = lv_label_create(btnsetup_lon);
    lv_label_set_text(label_btnsetup_lon, "LON");
    lv_obj_center(label_btnsetup_lon);

    setup_lon = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_lon, false);
    lv_obj_align(setup_lon, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_lon, 55, 60);
    lv_obj_set_size(setup_lon, 100, 30);
    lv_textarea_set_text(setup_lon, "");
    lv_textarea_set_max_length(setup_lon, 9);
    lv_obj_add_style(setup_lon, &ta_style, LV_PART_MAIN);
    
    lv_textarea_set_accepted_chars(setup_lon, "0123456789.");

    setup_lon_c = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_lon_c, false);
    lv_obj_align(setup_lon_c, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_lon_c, 155, 60);
    lv_obj_set_size(setup_lon_c, 40, 30);
    lv_textarea_set_text(setup_lon_c, "");
    lv_textarea_set_max_length(setup_lon_c, 1);
    lv_obj_add_style(setup_lon_c, &ta_style, LV_PART_MAIN);
    
    lv_textarea_set_accepted_chars(setup_lon_c, "EW");

    // ALT
    lv_obj_t * btnsetup_alt = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_alt, 0, 92);
    lv_obj_set_size(btnsetup_alt, 50, 25);

    lv_obj_t * label_btnsetup_alt = lv_label_create(btnsetup_alt);
    lv_label_set_text(label_btnsetup_alt, "ALT");
    lv_obj_center(label_btnsetup_alt);

    setup_alt = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_alt, false);
    lv_obj_align(setup_alt, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_alt, 55, 90);
    lv_obj_set_size(setup_alt, 100, 30);
    lv_textarea_set_text(setup_alt, "");
    lv_textarea_set_max_length(setup_alt, 4);
    lv_obj_add_style(setup_alt, &ta_style, LV_PART_MAIN);
    
    lv_textarea_set_accepted_chars(setup_alt, "01234567890");

    // MODUS
    dropdown_modusselect = lv_dropdown_create(t1);
    lv_dropdown_set_text(dropdown_modusselect, (char*)"MODUS");
    lv_obj_set_pos(dropdown_modusselect, 195, 0);
    lv_obj_set_size(dropdown_modusselect, 110, 25);
    lv_dropdown_set_options(dropdown_modusselect, (char*)"OFF\nKB LOCK\nLIGHT ON\nKBL&LIGHT");
    lv_obj_add_event_cb(dropdown_modusselect, btn_event_handler_dropdown_modusselect, LV_EVENT_ALL, NULL);

    // MAP-SELECT TAB
    dropdown_mapselect = lv_dropdown_create(t1);
    lv_dropdown_set_text(dropdown_mapselect, (char*)"MAPS");
    lv_obj_set_pos(dropdown_mapselect, 195, 30);
    lv_obj_set_size(dropdown_mapselect, 110, 25);
    lv_dropdown_set_options(dropdown_mapselect, getMapDropbox().c_str());
    lv_obj_add_event_cb(dropdown_mapselect, btn_event_handler_dropdown_mapselect, LV_EVENT_ALL, NULL);

    // COUNTRY TAB
    dropdown_country = lv_dropdown_create(t1);
    lv_dropdown_set_text(dropdown_country, (char*)"CTRY");
    lv_obj_set_pos(dropdown_country, 195, 60);
    lv_obj_set_size(dropdown_country, 110, 25);
    lv_dropdown_set_options(dropdown_country, getCountryDropbox().c_str());
    lv_obj_add_event_cb(dropdown_country, btn_event_handler_dropdown_country, LV_EVENT_ALL, NULL);

    // APRS TAB
    dropdown_aprs = lv_dropdown_create(t1);
    lv_dropdown_set_text(dropdown_aprs, (char*)"APRS");
    lv_obj_set_pos(dropdown_aprs, 195, 92);
    lv_obj_set_size(dropdown_aprs, 110, 25);
    lv_dropdown_set_options(dropdown_aprs, (char*)"Runner\nCar\nCycle\nBike\nWX\nPhone\nBulli\nHouse\nNode");
    lv_obj_add_event_cb(dropdown_aprs, btn_event_handler_aprs, LV_EVENT_ALL, NULL);
    
    // MODUSSEL
    lv_obj_t * btnsetup_modusselect = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_modusselect, 195, 0);
    lv_obj_set_size(btnsetup_modusselect, 80, 25);
    lv_obj_add_event_cb(btnsetup_modusselect, btn_event_handler_dropdown_modusselect, LV_EVENT_ALL, NULL);

    lv_obj_t * label_btnsetup_modusselect = lv_label_create(btnsetup_modusselect);
    lv_label_set_text(label_btnsetup_modusselect, "MODUS");
    lv_obj_center(label_btnsetup_modusselect);

    // MAPSEL
    lv_obj_t * btnsetup_mapselect = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_mapselect, 195, 30);
    lv_obj_set_size(btnsetup_mapselect, 80, 25);
    lv_obj_add_event_cb(btnsetup_mapselect, btn_event_handler_dropdown_mapselect, LV_EVENT_ALL, NULL);

    lv_obj_t * label_btnsetup_mapselect = lv_label_create(btnsetup_mapselect);
    lv_label_set_text(label_btnsetup_mapselect, "MAPS");
    lv_obj_center(label_btnsetup_mapselect);

    // COUNTRY
    lv_obj_t * btnsetup_country = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_country, 195, 60);
    lv_obj_set_size(btnsetup_country, 80, 25);
    lv_obj_add_event_cb(btnsetup_country, btn_event_handler_dropdown_country, LV_EVENT_ALL, NULL);

    lv_obj_t * label_btnsetup_country = lv_label_create(btnsetup_country);
    lv_label_set_text(label_btnsetup_country, "COUNTRY");
    lv_obj_center(label_btnsetup_country);

    // APRS
    lv_obj_t * btnsetup_aprs = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_aprs, 195, 92);
    lv_obj_set_size(btnsetup_aprs, 80, 25);
    lv_obj_add_event_cb(btnsetup_aprs, btn_event_handler_aprs, LV_EVENT_ALL, NULL);

    lv_obj_t * label_btnsetup_aprs = lv_label_create(btnsetup_aprs);
    lv_label_set_text(label_btnsetup_aprs, "APRS");
    lv_obj_center(label_btnsetup_aprs);

    // START TONE
    lv_obj_t * btnsetup_stone = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_stone, 0, 122);
    lv_obj_set_size(btnsetup_stone, 50, 25);

    lv_obj_t * label_btnsetup_stone = lv_label_create(btnsetup_stone);
    lv_label_set_text(label_btnsetup_stone, "START");
    lv_obj_center(label_btnsetup_stone);

    setup_stone = lv_textarea_create(t1);
    lv_textarea_set_one_line(setup_stone, true);
    lv_textarea_set_text_selection(setup_stone, false);
    lv_obj_align(setup_stone, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_stone, 55, 120);
    lv_obj_set_size(setup_stone, 220, 30);
    lv_textarea_set_text(setup_stone, "");
    lv_textarea_set_max_length(setup_stone, 100);
    lv_obj_add_style(setup_stone, &ta_style, LV_PART_MAIN);
    

    // MESSAGE TONE
    lv_obj_t * btnsetup_mtone = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_mtone, 0, 152);
    lv_obj_set_size(btnsetup_mtone, 50, 25);

    lv_obj_t * label_btnsetup_mtone = lv_label_create(btnsetup_mtone);
    lv_label_set_text(label_btnsetup_mtone, "MESS");
    lv_obj_center(label_btnsetup_mtone);

    setup_mtone = lv_textarea_create(t1);
    lv_textarea_set_one_line(setup_mtone, true);
    lv_textarea_set_text_selection(setup_mtone, false);
    lv_obj_align(setup_mtone, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_mtone, 55, 150);
    lv_obj_set_size(setup_mtone, 220, 30);
    lv_textarea_set_text(setup_mtone, "");
    lv_textarea_set_max_length(setup_mtone, 100);
    lv_obj_add_style(setup_mtone, &ta_style, LV_PART_MAIN);
    

    // NAME
    lv_obj_t * btnsetup_name = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_name, 0, 182);
    lv_obj_set_size(btnsetup_name, 50, 25);

    lv_obj_t * label_btnsetup_name = lv_label_create(btnsetup_name);
    lv_label_set_text(label_btnsetup_name, "NAME");
    lv_obj_center(label_btnsetup_name);

    setup_name = lv_textarea_create(t1);
    lv_textarea_set_one_line(setup_name, true);
    lv_textarea_set_text_selection(setup_name, false);
    lv_obj_align(setup_name, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_name, 55, 180);
    lv_obj_set_size(setup_name, 220, 30);
    lv_textarea_set_text(setup_name, "");
    lv_textarea_set_max_length(setup_name, 20);
    lv_obj_add_style(setup_name, &ta_style, LV_PART_MAIN);
    


    // GRUPPEN
    lv_obj_t * btnsetup_grc = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_grc, 0, 214);
    lv_obj_set_size(btnsetup_grc, 50, 25);

    lv_obj_t * label_btnsetup_grc = lv_label_create(btnsetup_grc);
    lv_label_set_text(label_btnsetup_grc, "GRC");
    lv_obj_center(label_btnsetup_grc);

    setup_grc0 = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_grc0, false);
    lv_obj_align(setup_grc0, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_grc0, 55, 212);
    lv_obj_set_size(setup_grc0, 80, 30);
    lv_textarea_set_text(setup_grc0, "");
    lv_textarea_set_max_length(setup_grc0, 5);
    lv_obj_add_style(setup_grc0, &ta_style, LV_PART_MAIN);
    
    lv_textarea_set_accepted_chars(setup_grc0, "0123456789");

    setup_grc1 = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_grc1, false);
    lv_obj_align(setup_grc1, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_grc1, 135, 212);
    lv_obj_set_size(setup_grc1, 80, 30);
    lv_textarea_set_text(setup_grc1, "");
    lv_textarea_set_max_length(setup_grc1, 5);
    lv_obj_add_style(setup_grc1, &ta_style, LV_PART_MAIN);
    
    lv_textarea_set_accepted_chars(setup_grc1, "0123456789");

    setup_grc2 = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_grc2, false);
    lv_obj_align(setup_grc2, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_grc2, 215, 212);
    lv_obj_set_size(setup_grc2, 80, 30);
    lv_textarea_set_text(setup_grc2, "");
    lv_textarea_set_max_length(setup_grc2, 5);
    lv_obj_add_style(setup_grc2, &ta_style, LV_PART_MAIN);
    
    lv_textarea_set_accepted_chars(setup_grc2, "0123456789");

    setup_grc3 = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_grc3, false);
    lv_obj_align(setup_grc3, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_grc3, 55, 242);
    lv_obj_set_size(setup_grc3, 80, 30);
    lv_textarea_set_text(setup_grc3, "");
    lv_textarea_set_max_length(setup_grc3, 5);
    lv_obj_add_style(setup_grc3, &ta_style, LV_PART_MAIN);
    
    lv_textarea_set_accepted_chars(setup_grc3, "0123456789");

    setup_grc4 = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_grc4, false);
    lv_obj_align(setup_grc4, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_grc4, 135, 242);
    lv_obj_set_size(setup_grc4, 80, 30);
    lv_textarea_set_text(setup_grc4, "");
    lv_textarea_set_max_length(setup_grc4, 5);
    lv_obj_add_style(setup_grc4, &ta_style, LV_PART_MAIN);
    
    lv_textarea_set_accepted_chars(setup_grc4, "0123456789");
    
    setup_grc5 = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_grc5, false);
    lv_obj_align(setup_grc5, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_grc5, 215, 242);
    lv_obj_set_size(setup_grc5, 80, 30);
    lv_textarea_set_text(setup_grc5, "");
    lv_textarea_set_max_length(setup_grc5, 5);
    lv_obj_add_style(setup_grc5, &ta_style, LV_PART_MAIN);
    
    lv_textarea_set_accepted_chars(setup_grc5, "0123456789");
    
    // INFO
    lv_obj_t * btn_msg_id_info = lv_btn_create(t1);
    lv_obj_set_pos(btn_msg_id_info, 0, 276);
    lv_obj_set_size(btn_msg_id_info, 50, 25);

    lv_obj_t * btn_msg_id_info_label = lv_label_create(btn_msg_id_info);
    lv_label_set_text(btn_msg_id_info_label, "ID");
    lv_obj_center(btn_msg_id_info_label);

    lv_obj_t * btn_msg_id = lv_btn_create(t1);
    lv_obj_set_pos(btn_msg_id, 55, 276);
    lv_obj_set_size(btn_msg_id, 80, 25);

    btn_msg_id_label = lv_label_create(btn_msg_id);
    lv_label_set_text(btn_msg_id_label, "");
    lv_obj_center(btn_msg_id_label);

    lv_obj_t * btn_ack_id = lv_btn_create(t1);
    lv_obj_set_pos(btn_ack_id, 140, 276);
    lv_obj_set_size(btn_ack_id, 40, 25);

    btn_ack_id_label = lv_label_create(btn_ack_id);
    lv_label_set_text(btn_ack_id_label, "");
    lv_obj_center(btn_ack_id_label);

    // WEBSERVER ON/OFF
    lv_obj_t * btn_web = lv_btn_create(t1);
    lv_obj_set_pos(btn_web, 200, 276);
    lv_obj_set_size(btn_web, 35, 25);

    lv_obj_t * btn_web_label = lv_label_create(btn_web);
    lv_label_set_text(btn_web_label, "WEB");
    lv_obj_center(btn_web_label);

    web_sw = lv_switch_create(t1);
    lv_obj_set_pos(web_sw, 245, 276);
    lv_obj_set_size(web_sw, 45, 25);

    lv_obj_add_event_cb(web_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    // MESH ON/OFF
    lv_obj_t * btn_mesh = lv_btn_create(t1);
    lv_obj_set_pos(btn_mesh, 0, 307);
    lv_obj_set_size(btn_mesh, 50, 25);

    lv_obj_t * btn_mesh_label = lv_label_create(btn_mesh);
    lv_label_set_text(btn_mesh_label, "MESH");
    lv_obj_center(btn_mesh_label);

    mesh_sw = lv_switch_create(t1);
    lv_obj_set_pos(mesh_sw, 55, 307);
    lv_obj_set_size(mesh_sw, 45, 25);

    lv_obj_add_event_cb(mesh_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    // NOALLMSG ON/OFF
    lv_obj_t * btn_noallmsg = lv_btn_create(t1);
    lv_obj_set_pos(btn_noallmsg, 100, 307);
    lv_obj_set_size(btn_noallmsg, 50, 25);

    lv_obj_t * btn_noallmsg_label = lv_label_create(btn_noallmsg);
    lv_label_set_text(btn_noallmsg_label, "NO ALL");
    lv_obj_center(btn_noallmsg_label);

    noallmsg_sw = lv_switch_create(t1);
    lv_obj_set_pos(noallmsg_sw, 155, 307);
    lv_obj_set_size(noallmsg_sw, 45, 25);

    lv_obj_add_event_cb(noallmsg_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    // GPS ON/OFF
    lv_obj_t * btn_gps = lv_btn_create(t1);
    lv_obj_set_pos(btn_gps, 200, 307);
    lv_obj_set_size(btn_gps, 35, 25);

    lv_obj_t * btn_gps_label = lv_label_create(btn_gps);
    lv_label_set_text(btn_gps_label, "GPS");
    lv_obj_center(btn_gps_label);

    gpson_sw = lv_switch_create(t1);
    lv_obj_set_pos(gpson_sw, 245, 307);
    lv_obj_set_size(gpson_sw, 45, 25);

    lv_obj_add_event_cb(gpson_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    // UTC
    lv_obj_t * btnsetup_utc = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_utc, 0, 342);
    lv_obj_set_size(btnsetup_utc, 50, 25);

    lv_obj_t * label_btnsetup_utc = lv_label_create(btnsetup_utc);
    lv_label_set_text(label_btnsetup_utc, "UTC");
    lv_obj_center(label_btnsetup_utc);

    setup_utc = lv_textarea_create(t1);
    lv_textarea_set_text_selection(setup_utc, false);
    lv_obj_align(setup_utc, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_pos(setup_utc, 55, 340);
    lv_obj_set_size(setup_utc, 60, 30);
    lv_textarea_set_text(setup_utc, "");
    lv_textarea_set_max_length(setup_utc, 8);
    lv_obj_add_style(setup_utc, &ta_style, LV_PART_MAIN);
    
    lv_textarea_set_accepted_chars(setup_utc, "01234567890.-");


    // TRACK ON/OFF
    lv_obj_t * btn_track = lv_btn_create(t1);
    lv_obj_set_pos(btn_track, 185, 340);
    lv_obj_set_size(btn_track, 50, 25);

    lv_obj_t * btn_track_label = lv_label_create(btn_track);
    lv_label_set_text(btn_track_label, "TRACK");
    lv_obj_center(btn_track_label);

    track_sw = lv_switch_create(t1);
    lv_obj_set_pos(track_sw, 245, 340);
    lv_obj_set_size(track_sw, 45, 25);

    lv_obj_add_event_cb(track_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    // MUTE
    lv_obj_t * btnsetup_mute = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_mute, 0, 375);
    lv_obj_set_size(btnsetup_mute, 50, 25);

    lv_obj_t * btnsetup_mute_label = lv_label_create(btnsetup_mute);
    lv_label_set_text(btnsetup_mute_label, "MUTE");
    lv_obj_center(btnsetup_mute_label);

    mute_sw = lv_switch_create(t1);
    lv_obj_set_pos(mute_sw, 55, 375);
    lv_obj_set_size(mute_sw, 45, 25);

    lv_obj_add_event_cb(mute_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    // WIFIAP ON/OFF
    lv_obj_t * btn_wifiap = lv_btn_create(t1);
    lv_obj_set_pos(btn_wifiap, 185, 375);
    lv_obj_set_size(btn_wifiap, 50, 25);

    lv_obj_t * btn_wifiap_label = lv_label_create(btn_wifiap);
    lv_label_set_text(btn_wifiap_label, "WIFAP");
    lv_obj_center(btn_wifiap_label);

    wifiap_sw = lv_switch_create(t1);
    lv_obj_set_pos(wifiap_sw, 245, 375);
    lv_obj_set_size(wifiap_sw, 45, 25);

    lv_obj_add_event_cb(wifiap_sw, btn_event_handler_switch, LV_EVENT_ALL, NULL);

    // BTN SETUP
    lv_obj_t * btnsetup = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup, 0, 410);
    lv_obj_set_size(btnsetup, 100, 30);
    lv_obj_add_event_cb(btnsetup, btn_event_handler_setup, LV_EVENT_ALL, NULL);

    lv_obj_t * btnlabel_setup = lv_label_create(btnsetup);
    lv_label_set_text(btnlabel_setup, "save setting");
    lv_obj_center(btnlabel_setup);

    // VERSION
    lv_obj_t * btnsetup_version = lv_btn_create(t1);
    lv_obj_set_pos(btnsetup_version, 185, 410);
    lv_obj_set_size(btnsetup_version, 105, 30);

    lv_obj_t * label_btnsetup_version = lv_label_create(btnsetup_version);
    char sv[50];
    sprintf(sv, "MeshCom %s%s", SOURCE_VERSION, SOURCE_VERSION_SUB);
    lv_label_set_text(label_btnsetup_version, sv);
    lv_obj_center(label_btnsetup_version);

    ////////////////////////////////////////////////////////////////////////////
    // TEXT OUTPUT
    msg_list = lv_obj_create(t2);
    lv_obj_set_size(msg_list, 300, LV_VER_RES * 0.6);
    lv_obj_align(msg_list, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(msg_list, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(msg_list, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(msg_list, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(msg_list, 0, LV_PART_MAIN);
    lv_obj_set_scroll_dir(msg_list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(msg_list, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_flex_flow(msg_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(msg_list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(msg_list, LV_OBJ_FLAG_SCROLLABLE);
    msg_list_show_hint("No messages yet");

    init_msg_tab_bar(t2);

    ////////////////////////////////////////////////////////////////////////////
    // POSITION
    position_ta = lv_table_create(t3);
    lv_obj_add_style(position_ta, &cell_style, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_style(position_ta, &cell_style1, LV_PART_ITEMS|LV_STATE_DEFAULT);
    lv_obj_set_pos(position_ta, 0, 0);
    lv_obj_set_style_radius(position_ta, 10, 0);
    lv_obj_set_style_clip_corner(position_ta, true, 0);
    lv_obj_set_size(position_ta, 302, LV_VER_RES * 0.6);

    lv_table_set_row_cnt(position_ta, 1);
    lv_table_set_col_cnt(position_ta, 3);

    lv_table_set_col_width(position_ta, 0, 84); // call
    lv_table_set_col_width(position_ta, 1, 40); // time
    lv_table_set_col_width(position_ta, 2, 174); // postxt

    lv_obj_add_event_cb(position_ta, position_ta_draw_event, LV_EVENT_DRAW_PART_BEGIN, NULL);

    lv_obj_set_height(position_ta, LV_VER_RES * 0.6);

    // TIME
    lv_obj_t * btn_time1 = lv_btn_create(t3);    /*Add a button the current screen*/
    lv_obj_set_pos(btn_time1, 0, 145);           /*Set its position*/
    lv_obj_set_size(btn_time1, 145, 20);         /*Set its size*/

    btn_time_label1 = lv_label_create(btn_time1); /*Add a label to the button*/
    lv_label_set_text(btn_time_label1, "time");  /*Set the labels text*/
    lv_obj_center(btn_time_label1);

    // BATT
    lv_obj_t * btn_batt1 = lv_btn_create(t3);    /*Add a button the current screen*/
    lv_obj_set_pos(btn_batt1, 146, 145);           /*Set its position*/
    lv_obj_set_size(btn_batt1, 145, 20);         /*Set its size*/

    btn_batt_label1 = lv_label_create(btn_batt1); /*Add a label to the button*/
    lv_label_set_text(btn_batt_label1, "Batt --");  /*Set the labels text*/
    lv_obj_center(btn_batt_label1);

    ////////////////////////////////////////////////////////////////////////////
    // MAP
    map_ta = lv_img_create(t7);
    lv_img_set_src(map_ta, &map_europe);
    lv_obj_align(map_ta, LV_ALIGN_CENTER, 0, 0);
    lv_obj_align(map_ta, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_size(map_ta, 300, LV_VER_RES * 0.74);
    
    lv_obj_align(map_ta, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t * btzoomout = lv_btn_create(t7);
    lv_obj_set_pos(btzoomout, 240, 135);
    lv_obj_set_size(btzoomout, 20, 20);
    lv_obj_add_event_cb(btzoomout, btn_event_handler_zoomout, LV_EVENT_ALL, NULL);

    lv_obj_t * btnlabelzoomout = lv_label_create(btzoomout);
    lv_label_set_text(btnlabelzoomout, "-");
    lv_obj_center(btnlabelzoomout);

    lv_obj_t * btzoomin = lv_btn_create(t7);
    lv_obj_set_pos(btzoomin, 270, 135);
    lv_obj_set_size(btzoomin, 20, 20);
    lv_obj_add_event_cb(btzoomin, btn_event_handler_zoomin, LV_EVENT_ALL, NULL);

    lv_obj_t * btnlabelzoomin = lv_label_create(btzoomin);
    lv_label_set_text(btnlabelzoomin, "+");
    lv_obj_center(btnlabelzoomin);
    
    ////////////////////////////////////////////////////////////////////////////
    // TRACK POSITION
    track_ta = lv_textarea_create(t6);
    lv_textarea_set_cursor_click_pos(track_ta, false);
    lv_textarea_set_text_selection(track_ta, false);
    lv_textarea_set_cursor_pos(track_ta, 0);
    lv_obj_set_size(track_ta, 300, LV_VER_RES * 0.72);
    lv_textarea_set_text(track_ta, "");
    lv_textarea_set_max_length(track_ta, 1000);
    lv_obj_align(track_ta, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_style(track_ta, &tr_style, LV_PART_MAIN);

    lv_obj_t * btsendpos = lv_btn_create(t6);
    lv_obj_set_pos(btsendpos, 200, 140);
    lv_obj_set_size(btsendpos, 80, 20);
    lv_obj_add_event_cb(btsendpos, btn_event_handler_sendpos, LV_EVENT_ALL, NULL);

    lv_obj_t * btnlabelsendpos = lv_label_create(btsendpos);
    lv_label_set_text(btnlabelsendpos, "SEND POS");
    lv_obj_center(btnlabelsendpos);

    ////////////////////////////////////////////////////////////////////////////
    // TEXT MHEARD
    mheard_ta = lv_table_create(t4);
    lv_obj_add_style(mheard_ta, &cell_style, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_style(mheard_ta, &cell_style1, LV_PART_ITEMS|LV_STATE_DEFAULT);
    lv_obj_set_pos(mheard_ta, 0, 0);
    lv_obj_set_size(mheard_ta, 302, LV_VER_RES * 0.6);
    lv_obj_set_style_radius(mheard_ta, 10, 0);
    lv_obj_set_style_clip_corner(mheard_ta, true, 0);

    lv_table_set_row_cnt(mheard_ta, 1);
    lv_table_set_col_cnt(mheard_ta, 6);

    lv_table_set_col_width(mheard_ta, 0, 76);
    lv_table_set_col_width(mheard_ta, 1, 40);
    lv_table_set_col_width(mheard_ta, 2, 38);
    lv_table_set_col_width(mheard_ta, 3, 68);
    lv_table_set_col_width(mheard_ta, 4, 38);
    lv_table_set_col_width(mheard_ta, 5, 38);

    lv_obj_set_height(mheard_ta, LV_VER_RES * 0.6);

    lv_obj_add_event_cb(mheard_ta, mheard_ta_draw_event, LV_EVENT_DRAW_PART_BEGIN, NULL);

    // TIME
    lv_obj_t * btn_time2 = lv_btn_create(t4);    /*Add a button the current screen*/
    lv_obj_set_pos(btn_time2, 0, 145);           /*Set its position*/
    lv_obj_set_size(btn_time2, 145, 20);         /*Set its size*/

    btn_time_label2 = lv_label_create(btn_time2); /*Add a label to the button*/
    lv_label_set_text(btn_time_label2, "time");  /*Set the labels text*/
    lv_obj_center(btn_time_label2);

    // BATT
    lv_obj_t * btn_batt2 = lv_btn_create(t4);    /*Add a button the current screen*/
    lv_obj_set_pos(btn_batt2, 146, 145);           /*Set its position*/
    lv_obj_set_size(btn_batt2, 145, 20);         /*Set its size*/

    btn_batt_label2 = lv_label_create(btn_batt2); /*Add a label to the button*/
    lv_label_set_text(btn_batt_label2, "Batt --");  /*Set the labels text*/
    lv_obj_center(btn_batt_label2);

    ////////////////////////////////////////////////////////////////////////////
    // TEXT PATH
    path_ta = lv_table_create(t8);
    lv_obj_add_style(path_ta, &cell_style, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_style(path_ta, &cell_style1, LV_PART_ITEMS|LV_STATE_DEFAULT);
    lv_obj_set_pos(path_ta, 0, 0);
    lv_obj_set_size(path_ta, 302, LV_VER_RES * 0.6);
    lv_obj_set_style_radius(path_ta, 10, 0);
    lv_obj_set_style_clip_corner(path_ta, true, 0);

    lv_table_set_row_cnt(path_ta, 1);
    lv_table_set_col_cnt(path_ta, 3);

    lv_table_set_col_width(path_ta, 0, 76);
    lv_table_set_col_width(path_ta, 1, 40);
    lv_table_set_col_width(path_ta, 2, 182);

    lv_obj_set_height(path_ta, LV_VER_RES * 0.6);

    lv_obj_add_event_cb(path_ta, path_ta_draw_event, LV_EVENT_DRAW_PART_BEGIN, NULL);

    // TIME
    lv_obj_t * btn_time8 = lv_btn_create(t8);    /*Add a button the current screen*/
    lv_obj_set_pos(btn_time8, 0, 145);           /*Set its position*/
    lv_obj_set_size(btn_time8, 145, 20);         /*Set its size*/

    btn_time_label4 = lv_label_create(btn_time8); /*Add a label to the button*/
    lv_label_set_text(btn_time_label4, "time");  /*Set the labels text*/
    lv_obj_center(btn_time_label4);

    // BATT
    lv_obj_t * btn_batt4 = lv_btn_create(t8);    /*Add a button the current screen*/
    lv_obj_set_pos(btn_batt4, 146, 145);           /*Set its position*/
    lv_obj_set_size(btn_batt4, 145, 20);         /*Set its size*/

    btn_batt_label4 = lv_label_create(btn_batt4); /*Add a label to the button*/
    lv_label_set_text(btn_batt_label4, "Batt --");  /*Set the labels text*/
    lv_obj_center(btn_batt_label4);

    ////////////////////////////////////////////////////////////////////////////
    // TEXT INPUT
    text_input = lv_textarea_create(t5);
    /* Allow cursor to be placed by click/touch */
    lv_textarea_set_cursor_click_pos(text_input, true);
    lv_textarea_set_cursor_pos(text_input, 0);
    lv_textarea_set_text_selection(text_input, false);
    /* Place message field below tab bar and extend towards control row
     * NOTE: `tv` (tabview) is already positioned below the header, so
     * coordinates inside a tab are relative to the tab content. Do not
     * add `header_height` again here (that produced the large vertical
     * shift seen in the photo). Calculate available tab height and
     * reserve space for the bottom control row. */
    const lv_coord_t msg_x = 10;
    const lv_coord_t msg_y = 6; /* small top margin inside tab content */
    /* Limit the message box width so its right edge lines up with the
     * rightmost control (the clear/trash button) on the bottom control row.
     * Keep a tiny gap (-2) so the box doesn't touch the button. */
    const lv_coord_t rightmost_ctrl_x = 264;
    const lv_coord_t rightmost_ctrl_w = 35;
    const lv_coord_t rightmost_ctrl_right = rightmost_ctrl_x + rightmost_ctrl_w;
    const lv_coord_t msg_w = LV_MIN(screen_w - 20, rightmost_ctrl_right - msg_x - 2);
    const lv_coord_t available_tab_h = screen_h - header_height; /* height available for tabs */
    const lv_coord_t tab_btn_h = 42; /* height of the tab buttons created by lv_tabview */
    const lv_coord_t content_h = LV_MAX(0, available_tab_h - tab_btn_h); /* usable content height inside a tab */
    const lv_coord_t msg_h = content_h - msg_y - 48; /* leave room for bottom control row */
    lv_obj_set_size(text_input, msg_w, msg_h);
    lv_obj_set_pos(text_input, msg_x, msg_y);
    lv_textarea_set_text(text_input, "");
    lv_textarea_set_max_length(text_input, 200);
    lv_textarea_set_placeholder_text(text_input, "Type Message");
    lv_obj_add_style(text_input, &ta_input_style, LV_PART_MAIN);
    /* Do not add cursor style globally; show/hide via focus events */
    lv_obj_add_event_cb(text_input, text_input_focus_cb, LV_EVENT_FOCUSED, NULL);
    lv_obj_add_event_cb(text_input, text_input_focus_cb, LV_EVENT_DEFOCUSED, NULL);
    /* Cursor color will be provided by the focused cursor style */

    dm_callsign = lv_textarea_create(t5);
    lv_textarea_set_one_line(dm_callsign, true);
    lv_textarea_set_cursor_click_pos(dm_callsign, false);
    lv_textarea_set_text_selection(dm_callsign, false);
    /* Move input to the left edge (small left margin) and align to bottom of tab
     * Adjusted: move 1px up (y -9) and increase height by 1px (27 -> 28) so
     * the visual label sits slightly higher while the field extends 1px lower. */
    /* Move input: 2 px down relative to previous (-9 -> -7) and increase
     * height by 1 px (28 -> 29) as requested. */
    lv_obj_align(dm_callsign, LV_ALIGN_BOTTOM_LEFT, 10, -7);
    lv_obj_set_size(dm_callsign, 120, 33);
    lv_obj_set_scrollbar_mode(dm_callsign, LV_SCROLLBAR_MODE_OFF);
    lv_textarea_set_text(dm_callsign, "");
    lv_textarea_set_max_length(dm_callsign, 10);
    lv_textarea_set_placeholder_text(dm_callsign, "TO Call or Group");
    lv_obj_add_style(dm_callsign, &ta_style, LV_PART_MAIN);
    /* Center the placeholder/text vertically in the field by making the
     * main part paddings symmetric. This keeps text centered while the
     * cursor padding can be tuned independently. */
    lv_obj_set_style_pad_top(dm_callsign, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(dm_callsign, 6, LV_PART_MAIN);
    /* Reduce the visible cursor height by ~2px by decreasing the sum of
     * cursor top+bottom paddings. */
    lv_obj_set_style_pad_top(dm_callsign, 4, LV_PART_CURSOR);
    lv_obj_set_style_pad_bottom(dm_callsign, 0, LV_PART_CURSOR);
    /* Left-align text and cursor horizontally (restore default left alignment) */
    lv_obj_set_style_text_align(dm_callsign, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_style_pad_left(dm_callsign, 4, LV_PART_CURSOR);
    lv_obj_set_style_pad_right(dm_callsign, 0, LV_PART_CURSOR);
    /* Do not add the cursor style globally; show/hide via focus events */
    lv_obj_add_event_cb(dm_callsign, dm_callsign_focus_cb, LV_EVENT_FOCUSED, NULL);
    lv_obj_add_event_cb(dm_callsign, dm_callsign_focus_cb, LV_EVENT_DEFOCUSED, NULL);
    /* Cursor color will be provided by the focused cursor style */
    lv_textarea_set_accepted_chars(dm_callsign, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-");

    lv_obj_t * btn = lv_btn_create(t5);           /* Send button */
    lv_obj_align(btn, LV_ALIGN_BOTTOM_LEFT, 132, -8);                /* directly right of dm_callsign (10+120=130) */
    lv_obj_set_size(btn, 50, 31);
    lv_obj_add_event_cb(btn, btn_event_handler_send, LV_EVENT_ALL, NULL);

    lv_obj_t * btnlabel = lv_label_create(btn);
#ifdef LV_SYMBOL_OK
    lv_label_set_text(btnlabel, LV_SYMBOL_OK);
#else
    lv_label_set_text(btnlabel, "send");
#endif
    lv_obj_center(btnlabel);
    
    lv_obj_t * btn_vk = lv_btn_create(t5);
    lv_obj_align(btn_vk, LV_ALIGN_BOTTOM_LEFT, 186, -8);
    lv_obj_set_size(btn_vk, 35, 31);
    lv_obj_t * btn_vk_label = lv_label_create(btn_vk);
#if defined(LV_SYMBOL_KEYBOARD)
    lv_label_set_text(btn_vk_label, LV_SYMBOL_KEYBOARD);
#elif defined(LV_SYMBOL_EDIT)
    lv_label_set_text(btn_vk_label, LV_SYMBOL_EDIT);
#else
    lv_label_set_text(btn_vk_label, "vk");
#endif
    lv_obj_center(btn_vk_label);

    lv_obj_t * btnup = lv_btn_create(t5);
    lv_obj_align(btnup, LV_ALIGN_BOTTOM_LEFT, 225, -8);
    lv_obj_set_size(btnup, 35, 31);
    lv_obj_add_event_cb(btnup, btn_event_handler_up, LV_EVENT_ALL, NULL);

    btnlabelup = lv_label_create(btnup);
    lv_label_set_text(btnlabelup, "abc");
    lv_obj_center(btnlabelup);

    lv_obj_t * btnc = lv_btn_create(t5);
    lv_obj_align(btnc, LV_ALIGN_BOTTOM_LEFT, 264, -8);
    lv_obj_set_size(btnc, 35, 31);
    lv_obj_add_event_cb(btnc, btn_event_handler_clear, LV_EVENT_ALL, NULL);

    lv_obj_t * btnlabelc = lv_label_create(btnc);
#ifdef LV_SYMBOL_TRASH
    lv_label_set_text(btnlabelc, LV_SYMBOL_TRASH);
#elif defined(LV_SYMBOL_CLOSE)
    lv_label_set_text(btnlabelc, LV_SYMBOL_CLOSE);
#else
    lv_label_set_text(btnlabelc, "clear");
#endif
    lv_obj_center(btnlabelc);
}

/**
 * TODO
 */
String getMap(int iMap)
{
    if(iMap < 0 || iMap >= MAX_MAP)
    {
        return "none";
    }

    return strMaps[iMap];
}

/**
 * TODO
 */
int getMapID(String strMap)
{
    for(int ic = 0; ic < MAX_MAP; ic++)
    {
        if(strMaps[ic].compareTo(strMap) == 0)
            return ic;
    }

    return -1;
}

/**
 * TODO
 */
String getMapDropbox()
{
    String strRet = "";

    for(int ic = 0; ic < MAX_MAP; ic++)
    {
        int icc = strMaps[ic].compareTo("none");

        if(icc != 0)
        {
            if(!strRet.isEmpty())
                strRet.concat("\n");

            strRet.concat(strMaps[ic]);
        }
    }

    return strRet;
}

/**
 * TODO
 */
String getCountryDropbox()
{
    String strRet = "";

    for(int ic = 0; ic < max_country; ic++)
    {
        int icc = strCountry[ic].compareTo("none");

        if(icc != 0)
        {
            if(!strRet.isEmpty())
                strRet.concat("\n");

            strRet.concat(strCountry[ic]);
        }
    }

    return strRet;
}


/**
 * TODO
 */
int getMapDropboxID(String strMap)
{
    for(int ic = 0;ic <= MAX_MAP; ic++)
    {
        if(strMaps[ic].compareTo(strMap) == 0)
            return ic;
    }

    return -1;
}

/*
 *
 * map functions
 * 
 */

/**
 * add a point to the map
 */
void add_map_point(String callsign, double dlat, double dlon, bool bHome)
{
    if(bDEBUG)
        Serial.printf("[MAP]...check add call: %s\n", callsign.c_str());
 
    int ipoint = 0;
 
    bool bFound = false;

    for(int ip = 0; ip < MAX_POINTS; ip++)
    {
        if(map_point_call[ip] == callsign)
        {
            if(map_point[ip] != NULL)
            {
                lv_obj_del(map_point[ip]);

                delay(19);
            }

            ipoint = ip;

            bFound = true;

            break;
        }
    }

    lv_coord_t x = 0;
    lv_coord_t y = 0;

    // check on map
    if(dlat > map_lat_min[meshcom_settings.node_map] || dlat < map_lat_max[meshcom_settings.node_map])
    {
        Serial.printf("[MAP]...LAT: %.4lf not on map: %i\n", dlat, meshcom_settings.node_map);
    }

    if(dlon < map_lon_min[meshcom_settings.node_map] || dlon > map_lon_max[meshcom_settings.node_map])
    {
        Serial.printf("[MAP]...LON: %.4lf not on map: %i\n", dlon, meshcom_settings.node_map);
    }

    double latdiff = map_lat_min[meshcom_settings.node_map] - map_lat_max[meshcom_settings.node_map];
    double londiff = map_lon_max[meshcom_settings.node_map] - map_lon_min[meshcom_settings.node_map];

    double ye = (double)map_y[meshcom_settings.node_map] / latdiff;
    double xe = (double)map_x[meshcom_settings.node_map] / londiff;

    y = (lv_coord_t)((map_lat_min[meshcom_settings.node_map] - dlat) * ye);
    x = (lv_coord_t)((dlon - map_lon_min[meshcom_settings.node_map]) * xe);

    if(x > map_x[meshcom_settings.node_map])
        x = map_x[meshcom_settings.node_map];
    if(y > map_y[meshcom_settings.node_map])
        y = map_y[meshcom_settings.node_map];

    if(!bFound)
        ipoint = map_point_count;
    
    map_point_call[ipoint] = callsign;
    map_point_lat[ipoint] = dlat;
    map_point_lon[ipoint] = dlon;

    if(!bFound)
    {
        map_point_count++;
        if(map_point_count >= MAX_POINTS)
            map_point_count = 0;

        if(map_point[map_point_count] != NULL)
        {
            lv_obj_del(map_point[map_point_count]);

            delay(10);
        }

        map_point_call[map_point_count] = ""; // wieder frei machen;
        map_point[map_point_count] = NULL;
        map_point_lat[map_point_count] = 0.0;
        map_point_lon[map_point_count] = 0.0;
    }

    Serial.printf("\n[MAP]...%-10.10s point:%2i node_lat:%.4lf node_lon:%.4lf latd:%.4lf lonf:%.4lf xe:%.4lf, ye:%.4lf <%3i/%3i)\n", callsign.c_str(), ipoint, dlat, dlon, latdiff, londiff, xe, ye, x, y);

    map_point[ipoint] = lv_obj_create(map_ta);
    lv_obj_set_size(map_point[ipoint],10, 10);
    lv_obj_set_pos(map_point[ipoint], x, y);
    if(bHome)
        lv_obj_set_style_bg_color(map_point[ipoint] , (lv_color_t)LV_COLOR_MAKE(0, 0, 255), 0);
    else
        lv_obj_set_style_bg_color(map_point[ipoint] , (lv_color_t)LV_COLOR_MAKE(255, 0, 0), 0);

    lv_obj_set_style_radius(map_point[ipoint] , LV_RADIUS_CIRCLE, 0);

    // MAP screen
    /*
    lv_tabview_set_act(tv, 3, LV_ANIM_OFF);

    lv_obj_align(map_ta, LV_ALIGN_CENTER, 1, 0);
    lv_obj_align(map_ta, LV_ALIGN_CENTER, 0, 0);
    */
}

/**
 * initializes the default maps
 */
void init_map()
{
    map_lat_min[0] = 62.18341;
    map_lat_max[0] = 38.90292;
    map_lon_min[0] = -12.68952;
    map_lon_max[0] = 47.28335;
    map_x[0] = 320;
    map_y[0] = 201;
    
    map_lat_min[1] = 54.29605;
    map_lat_max[1] = 47.24435;
    map_lon_min[1] = 02.76120;
    map_lon_max[1] = 20.60340;
    map_x[1] = 320;
    map_y[1] = 201;
    
    map_lat_min[2] = 49.89170;
    map_lat_max[2] = 45.44086;
    map_lon_min[2] = 07.54073;
    map_lon_max[2] = 18.12056;
    map_x[2] = 320;
    map_y[2] = 200;
    
    map_lat_min[3] = 48.38202;
    map_lat_max[3] = 48.07556;
    map_lon_min[3] = 15.79216;
    map_lon_max[3] = 16.65630;
    map_x[3] = 320;
    map_y[3] = 163;

    map_lat_min[4] = 48.31630;
    map_lat_max[4] = 48.11084;
    map_lon_min[4] = 16.09416;
    map_lon_max[4] = 16.69725;
    map_x[4] = 320;
    map_y[4] = 164;

    for(int im=0; im<MAX_POINTS; im++)
    {
        map_point[im] = NULL;
        map_point_call[im] = "";
        
        map_pos_call[im] = "";
    }

    map_point_count = 0;

}

/**
 * redraws the map
 */
void refresh_map(int iMap)
{
    if(bDEBUG)
        Serial.printf("[MAP]...set to %i - %s\n", iMap, getMap(iMap).c_str());

    // pos update
    for(int im = 0; im < MAX_POINTS; im++)
    {
        if(map_pos_call[im].length() > 0)
        {
            bool bHome=false;
            if(map_pos_call[im].compareTo(meshcom_settings.node_call) == 0)
                bHome=true;

            add_map_point(map_pos_call[im], map_pos_lat[im], map_pos_lon[im], bHome);
        }
    }
}

/**
 * sets a map as active
 */
void set_map(int iMap)
{
    // strMaps[max_map] = {"Europe", "Germany", "Austria", "OE3"};

    if(bDEBUG)
        Serial.printf("[MAP]...set to %i - %s\n", iMap, getMap(iMap).c_str());

    switch (iMap)
    {
        case 0:  // Europe 
            lv_img_set_src(map_ta, &map_europe);
            break;

        case 1:
            lv_img_set_src(map_ta, &map_deutschland);
            break;

        case 2:
            lv_img_set_src(map_ta, &map_oesterreich);
            break;
        case 3:
            lv_img_set_src(map_ta, &map_wien_umgebung);
            break;
        case 4:
            lv_img_set_src(map_ta, &map_wien);
            break;

        default:
            lv_img_set_src(map_ta, &map_europe);
            break;

    }

    lv_obj_align(map_ta, LV_ALIGN_CENTER, 0, 0);
    lv_obj_align(map_ta, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_size(map_ta, map_x[iMap], map_y[iMap]);
    lv_obj_align(map_ta, LV_ALIGN_CENTER, 0, 0);

    for(int im = 0; im < MAX_POINTS; im++)
    {
        if(map_point[im] != NULL && lv_obj_is_valid(map_point[im]))
        {
            lv_obj_del(map_point[im]);

            delay(10);
        }

        map_point[im]=NULL;
    }

    refresh_map(iMap);
}

/**
 * turn tft backlight on
 */
void tft_on()
{
    resetBrightness();
    tdeck_tft_timer = millis();
}

/**
 * turn tft backlight off
 */
void tft_off()
{
    if (!meshcom_settings.node_backlightlock)
        setBrightness(0);
}


static void update_header_sat_indicator(void)
{
    if(header_sat_label == NULL || header_sat_icon == NULL)
        return;

    char sat_text[8];
    snprintf(sat_text, sizeof(sat_text), "%u", (unsigned int)posinfo_satcount);
    lv_label_set_text(header_sat_label, sat_text);

    lv_color_t icon_color = posinfo_fix ? lv_palette_main(LV_PALETTE_GREEN)
                                        : lv_palette_main(LV_PALETTE_RED);
    lv_obj_set_style_text_color(header_sat_icon, icon_color, LV_PART_MAIN);
}

static void update_header_batt_indicator(float batt, int proz)
{
    if(header_batt_label == NULL || header_batt_icon == NULL)
        return;

    const float usb_voltage_threshold = 4.2f;
    const bool usb_powered = (batt > usb_voltage_threshold);

    if(usb_powered)
    {
        lv_label_set_text(header_batt_label, "USB");
        lv_label_set_text(header_batt_icon, LV_SYMBOL_USB);
        lv_obj_set_style_text_color(header_batt_icon, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_MAIN);
        return;
    }

    int clamped_proz = clamp_int(proz, 0, 100);
    char percent_text[8];
    snprintf(percent_text, sizeof(percent_text), "%d%%", clamped_proz);
    lv_label_set_text(header_batt_label, percent_text);

    const char *icon = LV_SYMBOL_BATTERY_EMPTY;
    lv_color_t icon_color = lv_palette_main(LV_PALETTE_LIGHT_GREEN);

    if(clamped_proz >= 80)
    {
        icon = LV_SYMBOL_BATTERY_FULL;
    }
    else if(clamped_proz >= 60)
    {
        icon = LV_SYMBOL_BATTERY_3;
    }
    else if(clamped_proz >= 40)
    {
        icon = LV_SYMBOL_BATTERY_2;
    }
    else if(clamped_proz >= 20)
    {
        icon = LV_SYMBOL_BATTERY_1;
    }

    lv_label_set_text(header_batt_icon, icon);
    lv_obj_set_style_text_color(header_batt_icon, icon_color, LV_PART_MAIN);
}

static void apply_tab_bar_styles(void)
{
    lv_obj_t *tab_bar = get_tab_bar();
    if(tab_bar == NULL)
        return;

    lv_obj_set_style_text_color(tab_bar, lv_palette_main(LV_PALETTE_LIGHT_GREEN), LV_PART_ITEMS);
    lv_obj_set_style_text_color(tab_bar, lv_palette_main(LV_PALETTE_RED), LV_PART_ITEMS | LV_STATE_CHECKED);

    lv_obj_set_style_bg_color(tab_bar, lv_color_black(), LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(tab_bar, LV_OPA_80, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(tab_bar, lv_palette_darken(LV_PALETTE_BLUE, 2), LV_PART_ITEMS);
    lv_obj_set_style_bg_opa(tab_bar, LV_OPA_60, LV_PART_ITEMS);

    lv_obj_set_style_border_width(tab_bar, 0, LV_PART_ITEMS);
    lv_obj_set_style_pad_all(tab_bar, 4, LV_PART_ITEMS);
}

static void update_header_locator_label(void)
{
    if(header_locator_label == NULL)
        return;

    char locator[9];
    if(compute_locator_from_settings(locator, sizeof(locator)))
    {
        lv_label_set_text(header_locator_label, locator);
    }
    else
    {
        lv_label_set_text(header_locator_label, DEFAULT_LOCATOR_TEXT);
    }
}

static bool compute_locator_from_settings(char *buffer, size_t len)
{
    if(buffer == NULL || len < 9)
        return false;

    bool lat_valid = (meshcom_settings.node_lat_c == 'N' || meshcom_settings.node_lat_c == 'S');
    bool lon_valid = (meshcom_settings.node_lon_c == 'E' || meshcom_settings.node_lon_c == 'W');

    if(!lat_valid || !lon_valid)
        return false;

    double lat = meshcom_settings.node_lat;
    double lon = meshcom_settings.node_lon;

    if(meshcom_settings.node_lat_c == 'S')
        lat *= -1.0;

    if(meshcom_settings.node_lon_c == 'W')
        lon *= -1.0;

    if(lat == 0.0 && lon == 0.0)
        return false;

    return compute_maidenhead_locator(lat, lon, buffer, len);
}

static int clamp_int(int value, int min_val, int max_val)
{
    if(value < min_val)
        return min_val;
    if(value > max_val)
        return max_val;
    return value;
}

static void msg_tabs_update_hint(void)
{
    bool has_entries = !msg_tab_entries.empty();

    if(msg_tab_bar != NULL)
    {
        if(has_entries)
            lv_obj_clear_flag(msg_tab_bar, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(msg_tab_bar, LV_OBJ_FLAG_HIDDEN);
    }

    if(msg_tab_hint_label == NULL)
        return;

    if(has_entries)
        lv_obj_add_flag(msg_tab_hint_label, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_clear_flag(msg_tab_hint_label, LV_OBJ_FLAG_HIDDEN);
}

static void msg_list_clear(void)
{
    if(msg_list == NULL)
        return;

    lv_obj_clean(msg_list);
    msg_list_hint_label = NULL;
}

static void msg_list_show_hint(const char *text)
{
    if(msg_list == NULL)
        return;

    msg_list_clear();
    msg_list_hint_label = lv_label_create(msg_list);
    lv_label_set_text(msg_list_hint_label, text != NULL ? text : "");
    lv_obj_set_style_text_color(msg_list_hint_label, lv_palette_darken(LV_PALETTE_GREY, 1), LV_PART_MAIN);
    lv_obj_align(msg_list_hint_label, LV_ALIGN_CENTER, 0, 0);
}

static void ensure_msg_styles(void)
{
    if(msg_styles_ready)
        return;

    msg_styles_ready = true;

    lv_style_init(&msg_style_incoming);
    lv_style_set_bg_opa(&msg_style_incoming, LV_OPA_COVER);
    lv_style_set_bg_color(&msg_style_incoming, lv_color_hex(0xD7F5D0));
    lv_style_set_radius(&msg_style_incoming, 14);

    lv_style_init(&msg_style_outgoing);
    lv_style_set_bg_opa(&msg_style_outgoing, LV_OPA_COVER);
    lv_style_set_bg_color(&msg_style_outgoing, lv_color_hex(0xD4E8FF));
    lv_style_set_radius(&msg_style_outgoing, 14);

    lv_style_init(&msg_style_system);
    lv_style_set_bg_opa(&msg_style_system, LV_OPA_COVER);
    lv_style_set_bg_color(&msg_style_system, lv_palette_lighten(LV_PALETTE_GREY, 2));
    lv_style_set_radius(&msg_style_system, 14);
}

static String build_timestamp_string(void)
{
    char buf[32];
    int year_two_digits = meshcom_settings.node_date_year % 100;
    snprintf(buf, sizeof(buf), "%02i.%02i.%02i %02i:%02i",
        meshcom_settings.node_date_day,
        meshcom_settings.node_date_month,
        year_two_digits,
        meshcom_settings.node_date_hour,
        meshcom_settings.node_date_minute);
    return String(buf);
}

static bool is_numeric_string(const String &value)
{
    if(value.length() == 0)
        return false;

    for(size_t i = 0; i < value.length(); ++i)
    {
        if(!isDigit(value[i]))
            return false;
    }

    return true;
}

static void init_msg_tab_bar(lv_obj_t *parent)
{
    msg_tab_bar = lv_obj_create(parent);
    lv_obj_set_width(msg_tab_bar, lv_pct(100));
    lv_obj_set_height(msg_tab_bar, 28);
    lv_obj_align(msg_tab_bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_opa(msg_tab_bar, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_color(msg_tab_bar, lv_palette_lighten(LV_PALETTE_GREY, 4), LV_PART_MAIN);
    lv_obj_set_style_border_width(msg_tab_bar, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(msg_tab_bar, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_all(msg_tab_bar, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_column(msg_tab_bar, 6, LV_PART_MAIN);
    lv_obj_set_flex_flow(msg_tab_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(msg_tab_bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scroll_dir(msg_tab_bar, LV_DIR_HOR);
    lv_obj_set_scrollbar_mode(msg_tab_bar, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_flag(msg_tab_bar, LV_OBJ_FLAG_SCROLLABLE);

    msg_tab_hint_label = lv_label_create(msg_tab_bar);
    lv_label_set_text(msg_tab_hint_label, "No MSG groups");
    lv_obj_set_style_text_color(msg_tab_hint_label, lv_palette_darken(LV_PALETTE_GREY, 1), LV_PART_MAIN);

    msg_tab_entries.clear();
    msg_active_tab_index = -1;
    msg_tabs_update_hint();
}

static void msg_tabs_select_index(int index)
{
    if(msg_tab_entries.empty())
    {
        msg_active_tab_index = -1;
        msg_list_show_hint("No messages yet");
        return;
    }

    if(index < 0 || index >= (int)msg_tab_entries.size())
        index = 0;

    msg_active_tab_index = index;

    for(size_t i = 0; i < msg_tab_entries.size(); ++i)
    {
        if(msg_tab_entries[i].button == NULL)
            continue;

        if((int)i == msg_active_tab_index)
            lv_obj_add_state(msg_tab_entries[i].button, LV_STATE_CHECKED);
        else
            lv_obj_clear_state(msg_tab_entries[i].button, LV_STATE_CHECKED);
    }

    msg_render_active_tab();
}

static void msg_tabs_trim_history(std::vector<MsgBubble> &bubbles)
{
    if(bubbles.size() <= MSG_TAB_MAX_MESSAGES)
        return;

    size_t overflow = bubbles.size() - MSG_TAB_MAX_MESSAGES;
    bubbles.erase(bubbles.begin(), bubbles.begin() + overflow);
}

static MsgTabEntry *msg_tabs_find_entry(const String &group, int *index_out)
{
    for(size_t i = 0; i < msg_tab_entries.size(); ++i)
    {
        if(msg_tab_entries[i].group.equalsIgnoreCase(group))
        {
            if(index_out != NULL)
                *index_out = static_cast<int>(i);
            return &msg_tab_entries[i];
        }
    }

    return NULL;
}

static void msg_tab_button_event_cb(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    lv_obj_t *btn = lv_event_get_target(e);
    for(size_t i = 0; i < msg_tab_entries.size(); ++i)
    {
        if(msg_tab_entries[i].button == btn)
        {
            msg_tabs_select_index(static_cast<int>(i));
            break;
        }
    }
}

static void msg_header_click_cb(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;

    lv_obj_t *hdr = lv_event_get_target(e);
    if(hdr == NULL)
        return;

    const char *text = lv_label_get_text(hdr);
    if(text == NULL)
        return;

    String s = String(text);
    s.trim();

    /* Token selection rules:
     * - If the clicked text contains '->', evaluate left vs right as before.
     * - Otherwise the clicked label is already either the sender or dest
     *   portion; take the first token before any comma. This covers cases
     *   like multiple senders ("A,B,C -> D") where clicking the sender
     *   label should pick the first sender, and clicking the dest label
     *   should pick the first dest token (e.g. numeric channel or '*').
     */
    int arrow = s.indexOf("->");
    String token;
    if(arrow >= 0)
    {
        String right = s.substring(arrow + 2);
        right.trim();
        /* If the displayed text uses "all" for '*' we must map it back
         * to '*' for the selection logic. Use a temporary string for checks. */
        String right_check = right;
        if(right_check.equalsIgnoreCase("Public") || right_check.equalsIgnoreCase("all"))
            right_check = String("*");

        if(right_check.length() > 0 && (right_check.charAt(0) == '*' || is_numeric_string(String(right_check.charAt(0)))))
        {
            /* use right-hand token */
            if(right_check.charAt(0) == '*')
            {
                token = String("*");
            }
            else
            {
                int comma = right.indexOf(',');
                token = (comma >= 0) ? right.substring(0, comma) : right;
                token.trim();
            }
        }
        else
        {
            /* use left-hand token (sender) */
            String left = s.substring(0, arrow);
            left.trim();
            int comma = left.indexOf(',');
            token = (comma >= 0) ? left.substring(0, comma) : left;
            token.trim();
        }
    }
    else
    {
        /* clicked label is either left or right already; take first token */
        int comma = s.indexOf(',');
        token = (comma >= 0) ? s.substring(0, comma) : s;
        token.trim();
    }

    if(token.length() == 0)
        return;

    if(dm_callsign != NULL)
    {
        lv_textarea_set_text(dm_callsign, token.c_str());
        const char *txt = lv_textarea_get_text(dm_callsign);
        if(txt != NULL)
            lv_textarea_set_cursor_pos(dm_callsign, (int)strlen(txt));
    }

    if(tv != NULL)
        lv_tabview_set_act(tv, 1, LV_ANIM_OFF);
}

static void msg_bubble_click_cb(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;

    /* When a message bubble is clicked, copy the header destination into
     * the DM callsign field (quick-reply) and switch to the SND tab. We try
     * to locate the header label inside the wrapper/bubble object hierarchy
     * and reuse the same parsing logic as `msg_header_click_cb`.
     */
    lv_obj_t *start = lv_event_get_target(e);
    if(start == NULL)
    {
        if(tv != NULL)
            lv_tabview_set_act(tv, 1, LV_ANIM_OFF);
        return;
    }

    const char *hdr_text = NULL;
    /* Try several strategies to find the header label:
     *  - look for child->child->child label (wrapper->bubble->header_row->header)
     *  - look for child label directly
     *  - climb up the parent chain and repeat (defensive)
     */
    lv_obj_t *cur = start;
    for(int depth = 0; depth < 4 && cur != NULL && hdr_text == NULL; ++depth)
    {
        lv_obj_t *c = lv_obj_get_child(cur, 0);
        if(c != NULL)
        {
            lv_obj_t *g = lv_obj_get_child(c, 0);
            if(g != NULL)
            {
                const char *t = lv_label_get_text(g);
                if(t != NULL && t[0] != '\0')
                {
                    hdr_text = t;
                    break;
                }
            }

            const char *t2 = lv_label_get_text(c);
            if(t2 != NULL && t2[0] != '\0')
            {
                hdr_text = t2;
                break;
            }
        }

        cur = lv_obj_get_parent(cur);
    }

    if(hdr_text != NULL && hdr_text[0] != '\0')
    {
        String s = String(hdr_text);
        s.trim();

        int arrow = s.indexOf("->");
        String target;
        /* Use the first (left) call sign before '->' for bubble-click quick-reply */
        if(arrow >= 0)
        {
            target = s.substring(0, arrow);
            target.trim();
        }
        else
        {
            target = s;
        }

        if(target.length() > 0)
        {
            if(target.indexOf("*") >= 0)
            {
                if(dm_callsign != NULL)
                {
                    lv_textarea_set_text(dm_callsign, "*");
                    lv_textarea_set_cursor_pos(dm_callsign, 1);
                }
                if(tv != NULL)
                    lv_tabview_set_act(tv, 1, LV_ANIM_OFF);
                return;
            }

            if(dm_callsign != NULL)
            {
                int comma = target.indexOf(',');
                String final_target = (comma >= 0) ? target.substring(0, comma) : target;
                final_target.trim();
                lv_textarea_set_text(dm_callsign, final_target.c_str());
                const char *txt = lv_textarea_get_text(dm_callsign);
                if(txt != NULL)
                    lv_textarea_set_cursor_pos(dm_callsign, (int)strlen(txt));
            }
        }
    }

    if(tv != NULL)
        lv_tabview_set_act(tv, 1, LV_ANIM_OFF);
}

static MsgTabEntry *msg_tabs_get_or_create_entry(const String &group, int *index_out)
{
    MsgTabEntry *entry = msg_tabs_find_entry(group, index_out);
    if(entry != NULL)
        return entry;

    if(msg_tab_bar == NULL)
        return NULL;

    MsgTabEntry new_entry;
    new_entry.group = group;
    new_entry.button = lv_btn_create(msg_tab_bar);

    lv_obj_set_style_bg_color(new_entry.button, lv_palette_lighten(LV_PALETTE_BLUE, 3), LV_PART_MAIN);
    lv_obj_set_style_bg_color(new_entry.button, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(new_entry.button, LV_OPA_70, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(new_entry.button, LV_OPA_100, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_border_width(new_entry.button, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(new_entry.button, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_all(new_entry.button, 6, LV_PART_MAIN);
    lv_obj_add_flag(new_entry.button, LV_OBJ_FLAG_SCROLL_ON_FOCUS);
    lv_obj_add_event_cb(new_entry.button, msg_tab_button_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *label = lv_label_create(new_entry.button);
    lv_label_set_text(label, group.c_str());
    lv_obj_center(label);

    msg_tab_entries.push_back(new_entry);
    if(index_out != NULL)
        *index_out = static_cast<int>(msg_tab_entries.size() - 1);

    msg_tabs_update_hint();

    if(msg_active_tab_index < 0)
        msg_tabs_select_index(static_cast<int>(msg_tab_entries.size() - 1));
    else
        msg_tabs_select_index(msg_active_tab_index);

    return &msg_tab_entries.back();
}

static void msg_tabs_add_message(const String &group, const MsgBubble &bubble)
{
    String normalized = group;
    normalized.trim();
    if(normalized.length() == 0)
        normalized = "MSG";

    int index = -1;
    MsgTabEntry *entry = msg_tabs_get_or_create_entry(normalized, &index);

    if(entry == NULL)
        return;

    entry->bubbles.push_back(bubble);
    msg_tabs_trim_history(entry->bubbles);

    msg_tabs_select_index(index);
}

static void msg_render_active_tab(void)
{
    if(msg_list == NULL)
        return;

    if(msg_active_tab_index < 0 || msg_active_tab_index >= (int)msg_tab_entries.size())
    {
        msg_list_show_hint("No messages yet");
        return;
    }

    const MsgTabEntry &entry = msg_tab_entries[msg_active_tab_index];

    if(entry.bubbles.empty())
    {
        msg_list_show_hint("No messages in this conversation");
        return;
    }

    msg_list_clear();

    for(const MsgBubble &bubble : entry.bubbles)
    {
        msg_list_append_bubble(bubble);
    }

    lv_obj_t *last = lv_obj_get_child(msg_list, -1);
    if(last != NULL)
        lv_obj_scroll_to_view(last, LV_ANIM_OFF);
}

static void msg_list_append_bubble(const MsgBubble &bubble)
{
    if(msg_list == NULL)
        return;

    ensure_msg_styles();

    lv_obj_t *wrapper = lv_obj_create(msg_list);
    lv_obj_set_width(wrapper, lv_pct(100));
    lv_obj_set_height(wrapper, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(wrapper, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(wrapper, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(wrapper, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_left(wrapper, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_right(wrapper, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_top(wrapper, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(wrapper, 1, LV_PART_MAIN);
    lv_obj_clear_flag(wrapper, LV_OBJ_FLAG_SCROLLABLE);
    /* allow tapping the bubble wrapper to trigger quick-reply behavior */
    /* Do not attach a global click handler to the wrapper — only the header
     * label (sender) should be clickable for incoming (green) bubbles. */

    lv_coord_t screen_w = lv_disp_get_hor_res(NULL);
    if(screen_w <= 0)
        screen_w = 320; // default safeguard for sizing

    lv_coord_t max_bubble_width = (screen_w * 85) / 100;
    if(max_bubble_width < 80)
        max_bubble_width = 80;

    lv_coord_t content_max_width = max_bubble_width - 12; // allow for bubble padding
    if(content_max_width < 40)
        content_max_width = max_bubble_width;

    lv_obj_t *bubble_obj = lv_obj_create(wrapper);
    lv_obj_set_width(bubble_obj, LV_SIZE_CONTENT);
    lv_obj_set_height(bubble_obj, LV_SIZE_CONTENT);
    lv_obj_set_style_max_width(bubble_obj, max_bubble_width, LV_PART_MAIN);
    lv_obj_set_style_border_width(bubble_obj, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(bubble_obj, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_row(bubble_obj, 4, LV_PART_MAIN);
    lv_obj_clear_flag(bubble_obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(bubble_obj, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(bubble_obj, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    /* Make the bubble itself clickable so clicks on the inner area are handled */
    /* bubble_obj is not click-target; clicks are handled on header only */

    const lv_style_t *style = &msg_style_incoming;
    if(bubble.type == MsgBubbleType::Outgoing)
        style = &msg_style_outgoing;
    else if(bubble.type == MsgBubbleType::System)
        style = &msg_style_system;
    lv_obj_add_style(bubble_obj, const_cast<lv_style_t *>(style), LV_PART_MAIN);

    if(bubble.type == MsgBubbleType::Outgoing)
        lv_obj_align(bubble_obj, LV_ALIGN_TOP_RIGHT, 0, 0);
    else
        lv_obj_align(bubble_obj, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_obj_t *header_row = lv_obj_create(bubble_obj);
    lv_obj_set_width(header_row, content_max_width);
    lv_obj_set_height(header_row, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(header_row, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(header_row, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(header_row, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(header_row, 4, LV_PART_MAIN);
    lv_obj_clear_flag(header_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(header_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    /* Split header into sender and destination labels so clicking the
     * sender uses the left-hand token even when a destination/group exists.
     */
    String hdr = bubble.header;
    int arrow = hdr.indexOf("->");
    String left = hdr;
    String right = String("");
    if(arrow >= 0)
    {
        left = hdr.substring(0, arrow);
        right = hdr.substring(arrow + 2);
    }
    left.trim();
    right.trim();

    lv_obj_t *sender_label = lv_label_create(header_row);
    lv_label_set_text(sender_label, left.c_str());
    lv_obj_set_style_text_color(sender_label, lv_palette_darken(LV_PALETTE_BLUE_GREY, 1), LV_PART_MAIN);
    lv_label_set_long_mode(sender_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(sender_label, LV_SIZE_CONTENT);
    lv_obj_set_style_max_width(sender_label, content_max_width, LV_PART_MAIN);
    lv_obj_set_flex_grow(sender_label, 1);
    if(bubble.type == MsgBubbleType::Incoming)
    {
        lv_obj_add_flag(sender_label, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(sender_label, msg_header_click_cb, LV_EVENT_CLICKED, NULL);
    }

    if(right.length() > 0)
    {
        /* Keep the arrow and spacing so the visual header remains identical
         * to the original single-line display: "left -> right". If the
         * destination token is '*' we show "all" for a larger click area,
         * but preserve semantics in the click handler. */
        String display_right = right;
        if(display_right.equals("*"))
            display_right = String("Public");

        String dest_txt = String(" -> ") + display_right;
        lv_obj_t *dest_label = lv_label_create(header_row);
        lv_label_set_text(dest_label, dest_txt.c_str());
        lv_obj_set_style_text_color(dest_label, lv_palette_darken(LV_PALETTE_BLUE_GREY, 1), LV_PART_MAIN);
        lv_label_set_long_mode(dest_label, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(dest_label, LV_SIZE_CONTENT);
        lv_obj_set_style_text_align(dest_label, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
        /* add a small right padding so the time/date has separation (~2 chars) */
        lv_obj_set_style_pad_right(dest_label, 12, LV_PART_MAIN);
        if(bubble.type == MsgBubbleType::Incoming)
        {
            lv_obj_add_flag(dest_label, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(dest_label, msg_header_click_cb, LV_EVENT_CLICKED, NULL);
        }
    }

    if(bubble.timestamp.length() > 0)
    {
        lv_obj_t *time_label = lv_label_create(header_row);
        lv_label_set_text(time_label, bubble.timestamp.c_str());
        lv_obj_set_style_text_color(time_label, lv_palette_darken(LV_PALETTE_GREY, 1), LV_PART_MAIN);
        lv_label_set_long_mode(time_label, LV_LABEL_LONG_CLIP);
        lv_obj_set_width(time_label, LV_SIZE_CONTENT);
        lv_obj_set_style_text_align(time_label, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    }

    lv_obj_t *body = lv_label_create(bubble_obj);
    lv_label_set_text(body, bubble.body.c_str());
    lv_label_set_long_mode(body, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(body, content_max_width);
    lv_obj_set_style_text_color(body, lv_color_black(), LV_PART_MAIN);
    /* body is not clickable; only header label should trigger quick-reply */

    lv_obj_scroll_to_view(wrapper, LV_ANIM_OFF);
}

static void msg_tabs_clear_all(void)
{
    for(auto &entry : msg_tab_entries)
    {
        if(entry.button != NULL)
        {
            lv_obj_del(entry.button);
            entry.button = NULL;
        }
    }

    msg_tab_entries.clear();
    msg_active_tab_index = -1;
    msg_tabs_update_hint();
    msg_list_show_hint("No messages yet");
}

static bool compute_maidenhead_locator(double lat, double lon, char *buffer, size_t len)
{
    if(buffer == NULL || len < 9)
        return false;

    if(lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0)
        return false;

    double adj_lon = lon + 180.0;
    double adj_lat = lat + 90.0;

    if(adj_lon < 0.0 || adj_lon >= 360.0 || adj_lat < 0.0 || adj_lat >= 180.0)
        return false;

    int field_lon = (int)floor(adj_lon / 20.0);
    int field_lat = (int)floor(adj_lat / 10.0);

    double remainder_lon = adj_lon - (field_lon * 20.0);
    double remainder_lat = adj_lat - (field_lat * 10.0);

    int square_lon = (int)floor(remainder_lon / 2.0);
    int square_lat = (int)floor(remainder_lat / 1.0);

    remainder_lon -= square_lon * 2.0;
    remainder_lat -= square_lat * 1.0;

    const double subsquare_lon_span = 2.0 / 24.0;
    const double subsquare_lat_span = 1.0 / 24.0;

    int subsquare_lon = (int)floor(remainder_lon / subsquare_lon_span);
    int subsquare_lat = (int)floor(remainder_lat / subsquare_lat_span);

    remainder_lon -= subsquare_lon * subsquare_lon_span;
    remainder_lat -= subsquare_lat * subsquare_lat_span;

    const double extended_lon_span = subsquare_lon_span / 24.0;
    const double extended_lat_span = subsquare_lat_span / 24.0;

    int extended_lon = (int)floor(remainder_lon / extended_lon_span);
    int extended_lat = (int)floor(remainder_lat / extended_lat_span);

    buffer[0] = 'A' + clamp_int(field_lon, 0, 17);
    buffer[1] = 'A' + clamp_int(field_lat, 0, 17);
    buffer[2] = '0' + clamp_int(square_lon, 0, 9);
    buffer[3] = '0' + clamp_int(square_lat, 0, 9);
    buffer[4] = 'A' + clamp_int(subsquare_lon, 0, 23);
    buffer[5] = 'A' + clamp_int(subsquare_lat, 0, 23);
    buffer[6] = 'A' + clamp_int(extended_lon, 0, 23);
    buffer[7] = 'A' + clamp_int(extended_lat, 0, 23);
    buffer[8] = '\0';

    return true;
}


/**
 * update the battery label
 */
void tdeck_update_batt_label(float batt, int proz)
{
    char vChar[35];
    if (posinfo_fix)
    {
        snprintf(vChar, sizeof(vChar), "Batt: %.2fV (%i%%) SAT:%i", batt, proz, posinfo_satcount);
    }
    else
    {
        snprintf(vChar, sizeof(vChar), "Batt: %.2fV (%i%%)", batt, proz);
    }

    if(batt > 4.2)
    {
        if(posinfo_fix > 0)
        {
            snprintf(vChar, sizeof(vChar), "Batt: USB SAT:%i", posinfo_satcount);
        }
        else
        {
            snprintf(vChar, sizeof(vChar), "Batt: USB");
        }
    }

    if(btn_batt_label != NULL)
        lv_label_set_text(btn_batt_label, vChar);
    if(btn_batt_label1 != NULL)
        lv_label_set_text(btn_batt_label1, vChar);
    if(btn_batt_label2 != NULL)
        lv_label_set_text(btn_batt_label2, vChar);
    if(btn_batt_label4 != NULL)
        lv_label_set_text(btn_batt_label4, vChar);

    update_header_batt_indicator(batt, proz);
    update_header_sat_indicator();
}

/**
 * update the time label
 */
void tdeck_update_time_label()
{
    char cTime[50];
    sprintf(cTime, "%i-%02i-%02i %02i:%02i:%02i", 
        meshcom_settings.node_date_year,
        meshcom_settings.node_date_month,
        meshcom_settings.node_date_day,
        meshcom_settings.node_date_hour,
        meshcom_settings.node_date_minute,
        meshcom_settings.node_date_second);

    if(btn_time_label != NULL)
        lv_label_set_text(btn_time_label, cTime);
    if(btn_time_label1 != NULL)
        lv_label_set_text(btn_time_label1, cTime);
    if(btn_time_label2 != NULL)
        lv_label_set_text(btn_time_label2, cTime);
    if(btn_time_label4 != NULL)
        lv_label_set_text(btn_time_label4, cTime);

    if(header_time_label != NULL)
    {
        char header_time[8];
        snprintf(header_time, sizeof(header_time), "%02i:%02i",
            meshcom_settings.node_date_hour,
            meshcom_settings.node_date_minute);
        lv_label_set_text(header_time_label, header_time);
    }

    update_header_locator_label();
    update_header_batt_indicator(global_batt > 0.0f ? global_batt / 1000.0f : 0.0f, global_proz);
    update_header_sat_indicator();
}

/**
 * add a point to map position
 */
void tdeck_add_pos_point(String callsign, double u_dlat, char lat_c, double u_dlon, char lon_c)
{
    if (bDEBUG)
    {
        Serial.printf("[MAP]...add position point call:%s\n", callsign.c_str());
    }

    double dlat = u_dlat;
    if(lat_c == 'W')
        dlat = u_dlat * -1.0;

    double dlon = u_dlon;
    if(lon_c == 'S')
        dlon = u_dlon * -1.0;

    for(int ip = 0; ip < MAX_POINTS; ip++)
    {
        if(map_pos_call[ip] == callsign)
        {
            if(map_pos_lat[ip] == dlat && map_pos_lon[ip] == dlon)
                return;

            map_pos_lat[ip] = dlat;
            map_pos_lon[ip] = dlon;

            bool bHome=false;
            if (map_pos_call[map_pos_count].compareTo(meshcom_settings.node_call) == 0)
                bHome=true;

            add_map_point(callsign, dlat, dlon, bHome);

            return;
        }
    }

    map_pos_call[map_pos_count] = callsign;
    map_pos_lat[map_pos_count] = dlat;
    map_pos_lon[map_pos_count] = dlon;

    bool bHome=false;
    if(map_pos_call[map_pos_count].compareTo(meshcom_settings.node_call) == 0)
        bHome=true;
    
    add_map_point(callsign, dlat, dlon, bHome);
    
    map_pos_count++;
    if (map_pos_count >= MAX_POINTS)
        map_pos_count = 1;
}

/**
 * adds a position to the POS view
 */
void tdeck_add_to_pos_view(String callsign, double u_dlat, char lat_c, double u_dlon, char lon_c, int alt)
{
    char buf[2000];

    if (bDEBUG)
    {
        Serial.printf("[POSVIEW]...add %s\n", callsign.c_str());

    }

    double dlat = u_dlat;
    if(lat_c == 'W')
        dlat = u_dlat * -1.0;

    double dlon = u_dlon;
    if(lon_c == 'S')
        dlon = u_dlon * -1.0;

    // Tabelle push down
    if(posrow < MAX_POSROW)
    {
        posrow++;
        // 2025-04-23, OE3GJC: not required, autoextends on write to row
        // lv_table_set_row_cnt(position_ta, posrow);
    }

    if(posrow > 2)
    {
        for(int pos_push = posrow - 2; pos_push >= 1; pos_push--)
        {
            if (bDEBUG)
            {
                Serial.printf("[POSVIEW]...moving row %i to %i (%s)\n", pos_push, pos_push + 1, lv_table_get_cell_value(position_ta, pos_push, 0));
            }
            lv_table_set_cell_value(position_ta, pos_push + 1, 0, lv_table_get_cell_value(position_ta, pos_push, 0));
            lv_table_set_cell_value(position_ta, pos_push + 1, 1, lv_table_get_cell_value(position_ta, pos_push, 1));
            lv_table_set_cell_value(position_ta, pos_push + 1, 2, lv_table_get_cell_value(position_ta, pos_push, 2));
        }
    }

    snprintf(buf, 10, "%s", callsign.c_str());
    lv_table_set_cell_value(position_ta, 1, 0, buf);

    snprintf(buf, 6, "%02i:%02i", meshcom_settings.node_date_hour, meshcom_settings.node_date_minute);
    lv_table_set_cell_value(position_ta, 1, 1, buf);

    snprintf(buf, 24, "%.2lf%c/%.2lf%c/%i", dlat, lat_c, dlon, lon_c, alt);
    lv_table_set_cell_value(position_ta, 1, 2, buf);
}

/**
 * refresh SET view with current values
 */
void tdeck_refresh_SET_view()
{
    lv_textarea_set_text(setup_callsign, meshcom_settings.node_call);
    char vChar[10];
    sprintf(vChar, "%.4lf", meshcom_settings.node_lat);
    lv_textarea_set_text(setup_lat, vChar);
    sprintf(vChar, "%c", meshcom_settings.node_lat_c);
    lv_textarea_set_text(setup_lat_c, vChar);
    
    sprintf(vChar, "%.4lf", meshcom_settings.node_lon);
    lv_textarea_set_text(setup_lon, vChar);
    sprintf(vChar, "%c", meshcom_settings.node_lon_c);
    lv_textarea_set_text(setup_lon_c, vChar);

    sprintf(vChar, "%i", meshcom_settings.node_alt);
    lv_textarea_set_text(setup_alt, vChar);

    lv_textarea_set_text(setup_stone, meshcom_settings.node_audio_start.c_str());
    lv_textarea_set_text(setup_mtone, meshcom_settings.node_audio_msg.c_str());
    lv_textarea_set_text(setup_name, meshcom_settings.node_name);

    sprintf(vChar, "%i", meshcom_settings.node_gcb[0]);
    lv_textarea_set_text(setup_grc0, vChar);
    sprintf(vChar, "%i", meshcom_settings.node_gcb[1]);
    lv_textarea_set_text(setup_grc1, vChar);
    sprintf(vChar, "%i", meshcom_settings.node_gcb[2]);
    lv_textarea_set_text(setup_grc2, vChar);
    sprintf(vChar, "%i", meshcom_settings.node_gcb[3]);
    lv_textarea_set_text(setup_grc3, vChar);
    sprintf(vChar, "%i", meshcom_settings.node_gcb[4]);
    lv_textarea_set_text(setup_grc4, vChar);
    sprintf(vChar, "%i", meshcom_settings.node_gcb[5]);
    lv_textarea_set_text(setup_grc5, vChar);

    sprintf(vChar, "%08X", _GW_ID);
    lv_label_set_text(btn_msg_id_label, vChar);
    sprintf(vChar, "%d", meshcom_settings.node_ackid);
    lv_label_set_text(btn_ack_id_label, vChar);

    // WEB
    if (bWEBSERVER)
        lv_obj_add_state(web_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(web_sw, LV_STATE_CHECKED);
    // MESH
    if (bMESH)
        lv_obj_add_state(mesh_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(mesh_sw, LV_STATE_CHECKED);
    // NOALL
    if (bNoMSGtoALL)
        lv_obj_add_state(noallmsg_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(noallmsg_sw, LV_STATE_CHECKED);
    // GPS
    if (bGPSON)
        lv_obj_add_state(gpson_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(gpson_sw, LV_STATE_CHECKED);
    // UTC offset        
    sprintf(vChar, "%.1f", meshcom_settings.node_utcoff);
    lv_textarea_set_text(setup_utc, vChar);
    // TRACK
    if (bDisplayTrack)
        lv_obj_add_state(track_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(track_sw, LV_STATE_CHECKED);
    // MUTE
    if (meshcom_settings.node_mute)
        lv_obj_add_state(mute_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(mute_sw, LV_STATE_CHECKED);
    // WIFIAP
    if (bWIFIAP)
        lv_obj_add_state(wifiap_sw, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(wifiap_sw, LV_STATE_CHECKED);
}

char ctrack[300];

static void msg_focus_and_alert(bool bWithAudio)
{
    if(!meshcom_settings.node_keyboardlock)
        tft_on();

    if(tv != NULL)
    {
        int active = lv_tabview_get_tab_act(tv);
        if(active != 1 && active != 7)
        {
            lv_tabview_set_act(tv, 0, LV_ANIM_OFF);
        }
    }

    if(bWithAudio)
    {
        if (!play_file_from_sd_blocking(meshcom_settings.node_audio_msg.c_str(), 12))
        {
            play_cw_start();
        }
    }
}

/**
 * show GPS Posítion sent
 */
void tdeck_send_track_view()
{
    if(bDisplayTrack)
        snprintf(ctrack, sizeof(ctrack), "\n\n\n\n       TRACK\n   POSITION SENT\n");
    else
        snprintf(ctrack, sizeof(ctrack), "\n\n\n\n        GPS\n   POSITION SENT\n");

    lv_textarea_set_text(track_ta, ctrack);
}

void tdeck_add_system_message(const char *text)
{
    if(text == NULL)
        return;

    MsgBubble bubble;
    bubble.type = MsgBubbleType::System;
    bubble.header = "System";
    bubble.timestamp = build_timestamp_string();
    bubble.body = String(text);

    msg_tabs_add_message("SYSTEM", bubble);
    msg_focus_and_alert(false);
}

/**
 * refresh GPS view
 */
void tdeck_refresh_track_view()
{
    if (lv_tabview_get_tab_act(tv) != 4) // GPS screen not active
        return;

    int pos_seconds = (int)(((posinfo_timer + (posinfo_interval * 1000)) - millis()) / 1000);

    char cDatum[20];
    sprintf(cDatum, "%04i-%02i-%02i",
        meshcom_settings.node_date_year,
        meshcom_settings.node_date_month,
        meshcom_settings.node_date_day);
    char cZeit[20];
    sprintf(cZeit, "%02i:%02i:%02i",
        meshcom_settings.node_date_hour,
        meshcom_settings.node_date_minute,
        meshcom_settings.node_date_second);

    if(bGPSON)
    {
        if(posinfo_fix)
        {
            if(bDisplayTrack)
            {
                snprintf(ctrack, sizeof(ctrack), "TRACK:on %s %i\nDATE :%s\nTIME :%s\nLAT  :%08.4lf %c\nLON  :%08.4lf %c\nDIST :%.0lf m\nRATE :%4li %4isec\nDIR  :old %.0lf\nDIR  :new %.0lf",
                (posinfo_fix ? "fix" : "nofix"), 
                posinfo_hdop, 
                cDatum, 
                cZeit, 
                meshcom_settings.node_lat, 
                meshcom_settings.node_lat_c, 
                meshcom_settings.node_lon, 
                meshcom_settings.node_lon_c, 
                posinfo_distance, 
                posinfo_interval,
                pos_seconds,
                posinfo_last_direction, 
                posinfo_direction);
            }
            else
            {
                snprintf(ctrack, sizeof(ctrack), "GPS  :on %s %i\nDATE :%s\nTIME :%s\nLAT  :%08.4lf %c\nLON  :%08.4lf %c\nALT  :%i\nRATE :%4li %isec\nSAT  :%u\nDIR  :%.0lf",
                (posinfo_fix ? "fix" : "nofix"), 
                posinfo_hdop, 
                cDatum, 
                cZeit, 
                meshcom_settings.node_lat, 
                meshcom_settings.node_lat_c, 
                meshcom_settings.node_lon, 
                meshcom_settings.node_lon_c, 
                meshcom_settings.node_alt,
                posinfo_interval,
                pos_seconds,
                posinfo_satcount,
                posinfo_direction);
            }

            lv_textarea_set_text(track_ta, ctrack);
        }
    
    }

    if(!bGPSON || (bGPSON && !posinfo_fix))
    {
        // normal TRACK screen
        char ctypegps[10];
        char ctypetrack[10];

        if(bGPSON)
            snprintf(ctypegps, sizeof(ctypegps), "GPS  :on");
        else
            snprintf(ctypegps, sizeof(ctypegps), "GPS  :off");
        
        if(bDisplayTrack)
            snprintf(ctypetrack, sizeof(ctypetrack), "TRACK:on");
        else
            snprintf(ctypetrack, sizeof(ctypetrack), "TRACK:off");

        snprintf(ctrack, sizeof(ctrack), "%s %s %i\n%s\nDATE :%s\nTIME :%s\nLAT  :%08.4lf %c\nLON  :%08.4lf %c\nALT  :%i m\nAGE  :%u\nSAT  :%u",
            ctypegps,
            (posinfo_fix ? "fix" : "nofix"),
            posinfo_hdop,
            ctypetrack,
            cDatum,
            cZeit,
            meshcom_settings.node_lat,
            meshcom_settings.node_lat_c,
            meshcom_settings.node_lon,
            meshcom_settings.node_lon_c,
            meshcom_settings.node_alt,
            posinfo_age,
            posinfo_satcount);

        lv_textarea_set_text(track_ta, ctrack);
    }
}

/**
 * adds an message to the MSG view
 */
void tdeck_add_MSG(aprsMessage aprsmsg, bool bWithAudio)
{
    String payload = aprsmsg.msg_payload;
    int ack_pos = payload.indexOf('{');
    if(ack_pos > 0)
        payload = payload.substring(0, ack_pos);

    payload = utf8ascii(payload);

    String local_call = String(meshcom_settings.node_call);
    bool is_outgoing = aprsmsg.msg_source_path.equalsIgnoreCase(local_call)
        || aprsmsg.msg_source_call.equalsIgnoreCase(local_call);

    String conversation = is_outgoing ? aprsmsg.msg_destination_call : aprsmsg.msg_source_call;
    if(conversation.length() == 0)
        conversation = is_outgoing ? aprsmsg.msg_destination_path : aprsmsg.msg_source_path;
    conversation.trim();
    if(conversation.length() == 0)
        conversation = "MSG";

    MsgBubble bubble;
    bubble.type = is_outgoing ? MsgBubbleType::Outgoing : MsgBubbleType::Incoming;
    bubble.timestamp = build_timestamp_string();

    String source_descriptor = aprsmsg.msg_source_path.length() > 0 ? aprsmsg.msg_source_path : aprsmsg.msg_source_call;
    String dest_descriptor = aprsmsg.msg_destination_path.length() > 0 ? aprsmsg.msg_destination_path : aprsmsg.msg_destination_call;

    if(source_descriptor.length() == 0)
        source_descriptor = local_call.length() > 0 ? local_call : String("You");
    if(dest_descriptor.length() == 0)
        dest_descriptor = conversation;

    String tab_override = dest_descriptor;
    tab_override.trim();
    if(tab_override.equals("*") || is_numeric_string(tab_override))
        conversation = tab_override;

    bubble.header = source_descriptor + " -> " + dest_descriptor;
    bubble.body = payload;

    msg_tabs_add_message(conversation, bubble);
    msg_focus_and_alert(bWithAudio);
}                  

/**
 * adds an message to the MSG view
 */
void tdeck_add_MSG(String callsign, String path, String message, bool bWithAudio)
{
    String local_call = String(meshcom_settings.node_call);
    String conversation = callsign;
    conversation.trim();
    if(conversation.length() == 0)
    {
        conversation = path;
        conversation.trim();
    }
    if(conversation.length() == 0)
        conversation = "MSG";

    MsgBubble bubble;
    bool is_outgoing = path.equalsIgnoreCase(local_call);
    bubble.type = is_outgoing ? MsgBubbleType::Outgoing : MsgBubbleType::Incoming;
    bubble.timestamp = build_timestamp_string();

    String header_source = path.length() > 0 ? path : (is_outgoing ? local_call : conversation);
    String header_dest = callsign.length() > 0 ? callsign : conversation;
    bubble.header = header_source;
    if(header_dest.length() > 0)
        bubble.header += " -> " + header_dest;
    bubble.body = utf8ascii(message);

    String tab_override = header_dest;
    tab_override.trim();
    if(tab_override.equals("*") || is_numeric_string(tab_override))
        conversation = tab_override;

    msg_tabs_add_message(conversation, bubble);
    msg_focus_and_alert(bWithAudio);
}

void tdeck_reset_msg_tabs(void)
{
    msg_tabs_clear_all();
}
