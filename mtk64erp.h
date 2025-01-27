/*
 * mtk64erp.h
 * Copylight 2024 mentako_ya
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * このファイルは、mtk64erpキーボードのファームウェア設定および
 * 機能を定義しています。
 */

#pragma once

#include "quantum.h"
#include <stdint.h> // 基本的な型定義用


//////////////////////////////////////////////////////////////////////////////
// 定数定義

// スクロールスナップモードの定義
#define MTK_SCROLLSNAP_MODE_VERTICAL   0
#define MTK_SCROLLSNAP_MODE_HORIZONTAL 1
#define MTK_SCROLLSNAP_MODE_FREE       2

#ifndef EECONFIG_ADDITIONAL_H
    #define EECONFIG_ADDITIONAL_H

    void eeconfig_update_kb_64(uint64_t val);
    uint64_t eeconfig_read_kb_64(void);
#endif // EECONFIG_ADDITIONAL_H


//////////////////////////////////////////////////////////////////////////////
// キーボードレイアウト定義
// clang-format off
#define LAYOUT( \
    L01, L02, L03, L04, L05, L06,              R06, R05, R04, R03, R02, R01, \
    L10, L12, L13, L14, L15, L16,              R16, R15, R14, R13, R12, R10, \
    L20, L21, L23, L24, L25, L26,              R26, R25, R24, R23, R21, R20, \
    L30, L31, L32, L34, L35, L36, L50,    R50, R36, R35, R34, R32, R31, R30, \
    L40, L41,                L51, L52,    R52, R51,                R41, R40, \
              L42,      L43,      L53,    R53,      R43,      R42,           \
			                      L54,    R54,                               \
         L60, L61, L62,                                  R60, R61, R62       \
	) \
    { \
        {   KC_NO, L01,   L02,   L03,   L04,   L05,   L06  }, \
        {   L10,   KC_NO, L12,   L13,   L14,   L15,   L16  }, \
        {   L20,   L21,   KC_NO, L23,   L24,   L25,   L26  }, \
        {   L30,   L31,   L32,   KC_NO, L34,   L35,   L36  }, \
        {   L40,   L41,   L42,   L43, KC_NO, KC_NO, KC_NO  }, \
        {   L50,   L51,   L52,   L53,   L54, KC_NO, KC_NO  }, \
        {   L60,   L61,   L62, KC_NO, KC_NO, KC_NO, KC_NO  }, \
        {   KC_NO, R01,   R02,   R03,   R04,   R05,   R06  }, \
        {   R10,   KC_NO, R12,   R13,   R14,   R15,   R16  }, \
        {   R20,   R21, KC_NO,   R23,   R24,   R25,   R26  }, \
        {   R30,   R31,   R32, KC_NO,   R34,   R35,   R36  }, \
        {   R40,   R41,   R42,   R43, KC_NO, KC_NO, KC_NO  }, \
        {   R50,   R51,   R52,   R53,   R54, KC_NO, KC_NO  }, \
        {   R60,   R61,   R62, KC_NO, KC_NO, KC_NO, KC_NO  } \
    }
// clang-format on


//////////////////////////////////////////////////////////////////////////////
// 型定義
// トラックボールの動きに関するデータ構造
typedef struct {
    int16_t x;           // X方向の位置
    int16_t y;           // Y方向の位置
    int16_t active_time; // アクティブ時間
    int16_t inertia_x;   // 慣性スクロールの速度X
    int16_t inertia_y;   // 慣性スクロールの速度Y
} mtk_motion_t;

// EEPROM設定データ
typedef union __attribute__((__packed__)) {
    uint64_t raw; // 64ビットの生データ
    struct {
        uint8_t speed_adjust_value;   // 速度調整値
        uint8_t cpi;                  // CPI
        uint8_t sdiv;                 // スクロール速度の分割値
        uint8_t auto_mouse_time_out;  // 自動マウスのタイムアウト
        uint8_t auto_mouse;           // 自動マウス有効フラグ
        uint8_t scroll_snap_mode;     // スクロールスナップモード
        uint8_t sdir;                 // スクロール方向
        uint8_t oled_orient;          // OLED表示方向
    };
} ee_config_t;


typedef struct {
    uint16_t cpi_value;             // トラックボールの感度値
    bool     cpi_changed;           // 感度が変更されたかどうか
    bool     scroll_mode;           // スクロールモードの有効/無効
    bool     scroll_direction;      // スクロール方向 (true: 正方向, false: 逆方向)
    uint8_t  scroll_div;            // スクロール速度の分割値
    uint8_t  scroll_snap_mode;      // スクロールスナップモード (例: 0: 無効, 1: 水平, 2: 垂直)
    uint16_t auto_mouse_time_out;   // 自動マウスモードのタイムアウト時間 (ミリ秒)
    uint32_t scroll_snap_last;      // スクロールスナップの最後のタイムスタンプ
    int8_t   scroll_snap_tension_h; // 水平方向のスクロールスナップのテンション
    mtk_motion_t motion;            // トラックボールの動き
    bool     auto_mouse_mode;       // 自動マウスモードの有効/無効
    bool     key_pressed;           // 現在キーが押されているかどうか
    char     current_key[6];        // 押下されたキー名 (最大5文字 + NULL終端)
    uint8_t  active_layer;          // 現在のアクティブレイヤ
    bool     speed_adjust_enabled;  // トラックボール移動速度調整の有効/無効
    uint8_t  speed_adjust_value;    // トラックボール移動速度調整の倍率 (0-255)
    uint8_t  oled_orient;           // OLED表示方向
} mtk_config_t;

extern mtk_motion_t mtk_motion;
extern ee_config_t ee_config;
extern mtk_config_t mtk_config;

// カスタムキーコードの定義
enum custom_keycodes {
    KBC_LOAD = QK_KB_0,     // 設定をデフォルトにリセット
    KBC_SAVE,               // 設定をEEPROMに保存

    CPI_I10,                // CPIを+10増加
    CPI_D10,                // CPIを-10減少
    CPI_I100,               // CPIを+100増加
    CPI_D100,               // CPIを-100減少
    CPI_I1K,                // CPIを+1000増加
    CPI_D1K,                // CPIを-1000減少

    SCRL_TO,                // スクロールモードの切替
    SCRL_MO,                // 一時的なスクロールモード
    SCRL_INV,               // スクロール方向の反転
    SCRL_DVI,               // スクロール速度分割値を増加
    SCRL_DVD,               // スクロール速度分割値を減少

    AM_TG,                  // 自動マウスモードの切替
    AM_TOUT_INC,            // 自動マウスタイムアウトを増加
    AM_TOUT_DEC,            // 自動マウスタイムアウトを減少

    SSNP_VRT,               // スクロールスナップモードを垂直に設定
    SSNP_HOR,               // スクロールスナップモードを水平に設定
    SSNP_FRE,               // スクロールスナップモードを無効化

    ADJMS_TG,               // トラックボール速度調整の切替
    ADJMS_SPD_INC,          // トラックボール速度調整値を増加
    ADJMS_SPD_DEC,          // トラックボール速度調整値を減少

    OLED_ORI_TG,            // OLED表示方向の切替 (0: 横 / 1: 縦)
};


//////////////////////////////////////////////////////////////////////////////
// 公開関数 (Public Functions)

// スクロールモードの取得と設定
bool mtk_get_scroll_mode(void);
void mtk_set_scroll_mode(bool mode);

// スクロール方向の取得と設定
bool mtk_get_scroll_direction(void);
void mtk_set_scroll_direction(bool direction);

// スクロール速度分割値の取得と設定
uint8_t mtk_get_scroll_div(void);
void mtk_set_scroll_div(uint8_t div);

// CPI値の取得と設定
uint16_t mtk_get_cpi(void);
void mtk_set_cpi(uint16_t cpi);

// キーログの記録
void set_keylog(uint16_t keycode, keyrecord_t *record);


//////////////////////////////////////////////////////////////////////////////
// 設定関連関数 (Configuration Functions)

// 自動マウスモードの取得と設定
bool mtk_get_auto_mouse_mode(void);
void mtk_set_auto_mouse_mode(bool mode);

// 自動マウスタイムアウトの取得と設定
uint16_t mtk_get_auto_mouse_time_out(void);
void mtk_set_auto_mouse_time_out(uint16_t timeout);

// スクロールスナップモードの取得と設定
void mtk_set_scrollsnap_mode(uint8_t mode);
uint8_t mtk_get_scrollsnap_mode(void);

// トラックボール速度調整値の取得と設定
void mtk_set_speed_adjust_value(uint8_t value);
uint8_t mtk_get_speed_adjust_value(void);

// トラックボール速度調整の有効化状態を取得と設定
bool mtk_get_speed_adjust_enabled(void);
void mtk_set_speed_adjust_enabled(bool enabled);

// OLED表示方向の取得と設定
uint8_t mtk_get_oled_orient_value(void);
void mtk_set_oled_orient_value(uint8_t val);

// 値ロード
void load_mtk_config(void);

//////////////////////////////////////////////////////////////////////////////
// 描画関連関数 (Rendering Functions)

// 固定文字列を描画
void render_fixed_string(uint8_t col, uint32_t row, const char *text);

// 各種インジケーターを描画
void render_indicator(uint8_t col, uint32_t row, uint8_t layer);
void render_indicator2(uint8_t col, uint32_t row, uint8_t layer);
void render_indicator3(uint8_t col, uint32_t row, uint8_t layer);

// キーの値やスクロールモードを描画
void render_key_value(uint8_t col, uint32_t row, const char *key, const char *format, int value, bool invert);
void render_snap_mode(uint8_t col, uint32_t row, uint8_t snap_mode);

// 数値を小数形式で描画
void render_decimal_value(uint8_t col, uint32_t row, const char *key, int integer_part, int fractional_part, bool invert);


//////////////////////////////////////////////////////////////////////////////
// 補助関数 (Helper Functions)

// 稼働時間の描画
void oled_write_uptime(uint8_t col, uint32_t row);

// アニメーションの制御
void render_animation(uint8_t col, uint32_t row, uint8_t fill_count);
void start_oled_animation(void);
bool oled_task_user(void);
