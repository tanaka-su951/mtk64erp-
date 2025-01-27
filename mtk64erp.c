/*
Copylight 2024 mentako_ya
SPDX-License-Identifier: GPL-2.0-or-later
*/

#include "transactions.h"       // トランザクション管理用ヘッダ
#include "split_util.h"         // 分割キーボードユーティリティ
#include "eeprom.h"             // EEPROM操作用ヘッダ
#include "timer.h"              // タイマー管理ヘッダ
#include "quantum.h"            // QMK量子レイヤーヘッダ
#include <math.h>               // 数学関数ライブラリ
#include <print.h>              // デバッグ用プリントライブラリ
#include "../../drivers/sensors/pmw3389.h" // トラックボールセンサー用ライブラリ

#include QMK_KEYBOARD_H         // キーボード設定ヘッダ
#ifdef CONSOLE_ENABLE
#endif


//////////////////////////////////////////////////////////////////////////////

// clang-format off
const matrix_row_t matrix_mask[MATRIX_ROWS] = {

    0b01111110,
    0b01111101,
    0b01111011,
    0b01110111,
    0b01101111,
    0b01011111,
    0b00111111,

    0b01111110,
    0b01111101,
    0b01111011,
    0b01110111,
    0b01101111,
    0b01011111,
    0b00111111,

};
// clang-format on

#ifdef POINTING_DEVICE_ENABLE
#    if defined(POINTING_DEVICE_LEFT)
#        define POINTING_DEVICE_THIS_SIDE is_keyboard_left()
#    elif defined(POINTING_DEVICE_RIGHT)
#        define POINTING_DEVICE_THIS_SIDE !is_keyboard_left()
#    elif defined(POINTING_DEVICE_COMBINED)
#        define POINTING_DEVICE_THIS_SIDE true
#    endif
#endif

//////////////////////////////////////////////////////////////////////////////
// Configurations

// 0:横表示、1：縦表示
#ifndef MTK_OLED_ORIENT
#    define MTK_OLED_ORIENT               1   // OLED表示の方向設定
#endif

#define OLED_WIDTH                        10  // インジケータの横幅を定義（縦表示の場合、MAX10文字）
#define OLED_WIDTH_slave                  15  // インジケータの横幅を定義（縦表示の場合、MAX15文字）

#ifndef MTK_CPI_DEFAULT
#    define MTK_CPI_DEFAULT              1000 // トラックボールのデフォルトCPI（感度）
#endif

#ifndef MTK_SCROLL_DIV_MIN
#    define MTK_SCROLL_DIV_MIN           1    // スクロール分割値の最小値
#endif

#ifndef MTK_SCROLL_DIV_DEFAULT
#    define MTK_SCROLL_DIV_DEFAULT       10   // スクロール分割値のデフォルト値
#endif

#ifndef MTK_SCROLL_DIV_MAX
#    define MTK_SCROLL_DIV_MAX           32   // スクロール分割値の最大値
#endif

#ifndef MTK_SCROLLBALL_INHIVITOR
#    define MTK_SCROLLBALL_INHIVITOR     5    // スクロールボールの動き抑制値
#endif

#ifndef MTK_SCROLLSNAP_ENABLE
#    define MTK_SCROLLSNAP_ENABLE        1    // スクロールスナップ機能の有効化
#endif

#ifndef MTK_SCROLLSNAP_RESET_TIMER
#    define MTK_SCROLLSNAP_RESET_TIMER   100  // スクロールスナップのリセットタイマー値（ミリ秒）
#endif

#ifndef MTK_SCROLLSNAP_TENSION_THRESHOLD
#    define MTK_SCROLLSNAP_TENSION_THRESHOLD 12   // スクロールスナップの張力閾値
#endif

#ifndef MTK_SPEED_ADJUST_DEFAULT
#    define MTK_SPEED_ADJUST_DEFAULT     15   // 速度調整のデフォルト値
#    define MTK_SPEED_ADJUST_MAX         20  // 速度調整の最大値
#    define MTK_SPEED_ADJUST_MIN         8    // 速度調整の最小値
#    define MTK_SPEED_ADJUST_STEP        1    // 速度調整のステップ
#endif

#define XSCALE_FACTOR                    1.0    // X方向のスケールファクター
#define YSCALE_FACTOR                    1.0    // Y方向のスケールファクター

// メモリ変数初期化
ee_config_t ee_config = {0};

// OLEDに表示するレイヤ名
const char layer0_name[] = "Deflt";  // レイヤ0
const char layer1_name[] = "Shift";  // レイヤ1
const char layer2_name[] = "Mails";  // レイヤ2
const char layer3_name[] = "Numbr";  // レイヤ3
const char layer4_name[] = "Excel";  // レイヤ4
const char layer5_name[] = "Teams";  // レイヤ5
const char layer6_name[] = "Setup";  // レイヤ6
const char layer7_name[] = "Mouse";  // レイヤ7

const char *const layer_names[] = {
    layer0_name,
    layer1_name,
    layer2_name,
    layer3_name,
    layer4_name,
    layer5_name,
    layer6_name,
    layer7_name,
};

// 打鍵数カウント
static uint32_t type_count = 0;


//////////////////////////////////////////////////////////////////////////////
// Constants
/****************************************************************************
 * mtk_config
 *
 * トラックボールおよびスクロール関連の設定を保持する構造体。
 * CPI、スクロール設定、自動マウスモード、速度調整値、
 * OLED表示設定などの動作パラメータを管理。
 * ****************************************************************************/
mtk_config_t mtk_config = {
    .cpi_value            = MTK_CPI_DEFAULT,              // デフォルトCPI設定
    .cpi_changed          = false,                        // CPI変更フラグ
    .scroll_mode          = false,                        // スクロールモード無効
    .scroll_direction     = false,                        // スクロール方向（通常）
    .scroll_div           = MTK_SCROLL_DIV_DEFAULT,       // デフォルトのスクロール分割値
    .auto_mouse_mode      = true,                         // 自動マウスモード有効
    .auto_mouse_time_out  = AUTO_MOUSE_TIME,              // 自動マウスモードのタイムアウト値
    .scroll_snap_mode     = MTK_SCROLLSNAP_MODE_VERTICAL, // 垂直スナップモード
    .speed_adjust_enabled = true,                         // 速度調整機能有効
    .speed_adjust_value   = MTK_SPEED_ADJUST_DEFAULT,     // デフォルトの速度調整値
    .oled_orient          = MTK_OLED_ORIENT               // OLED表示の方向
};

/****************************************************************************
 * remote_motion
 *
 * トラックボールのリモート側（スレーブデバイス）から送信される動作情報を保持。
 * 動作の同期やデバイス間の調整処理に利用する。
 * ****************************************************************************/
mtk_motion_t remote_motion;


/****************************************************************************
 * add_cpi
 *
 * トラックボールの感度（CPI: Counts Per Inch）を調整する。
 * 現在のCPI値にdeltaを加算または減算し、結果を設定する。
 * CPI値が1未満にならないように制限する。
 * ****************************************************************************/
static void add_cpi(int16_t delta) {
    int16_t v = mtk_get_cpi() + delta;
    mtk_set_cpi(v < 1 ? 1 : v);
}


/****************************************************************************
 * add_scroll_div
 *
 * スクロール速度の分割値を調整する。
 * 現在の分割値にdeltaを加算または減算し、結果を設定する。
 * 分割値が最小値（MTK_SCROLL_DIV_MIN）を下回らないように制限する。
 * ****************************************************************************/
static void add_scroll_div(int16_t delta) {
    int8_t v = mtk_get_scroll_div() + delta;
    mtk_set_scroll_div(v < MTK_SCROLL_DIV_MIN ? MTK_SCROLL_DIV_MIN : v);
}


#ifdef POINTING_DEVICE_AUTO_MOUSE_ENABLE
/****************************************************************************
 * add_auto_mouse_time_out
 *
 * 自動マウスモードのタイムアウト値を調整する。
 * 現在のタイムアウト値にdeltaを加算または減算し、結果を設定する。
 * タイムアウト値が100未満にならないように制限する。
 * ****************************************************************************/
static void add_auto_mouse_time_out(int16_t delta) {
    int16_t v = mtk_get_auto_mouse_time_out() + delta;
    mtk_set_auto_mouse_time_out(v < 100 ? 100 : v);
}
#endif


/****************************************************************************
 * constrain_hid
 *
 * HIDの範囲内（-127から127）に値を制限する。
 * 入力値が範囲を超える場合は、それぞれの境界値に切り詰める。
 * ****************************************************************************/
static int8_t constrain_hid(int16_t value) {
    if (value > 127) {
        return 127;
    } else if (value < -127) {
        return -127;
    }
    return (int8_t)value;
}


/******************************************************************************
 * Function: motion_to_mouse
 * --------------------------------------------------------------------------
 * トラックボールの移動量をマウスレポートに変換
 * --------------------------------------------------------------------------
 * トラックボールから取得した移動量（delta_x, delta_y）をCPIや速度調整設定を考慮して計算し、
 * マウスレポート（x, y）に反映する。
 *
 * ■パラメータ詳細
 * 1. XSCALE_FACTOR および YSCALE_FACTOR
 *    - 説明: XSCALE_FACTOR と YSCALE_FACTOR は、カーソルの最終的な動きをスケーリング
 *            （倍率を掛ける）ためのパラメータです。これにより、トラックボールの動きが
 *            カーソルの動きにどれだけ反映されるかを調整します。
 *    - 設定方法: これらの値を「大きく」するとカーソルの移動が「遅く」なり、値を「小さく」
 *                するとカーソルの移動が「速く」なります。
 *    - 推奨設定: 非線形的な速度調整を目指す場合、1.0〜2.0 の範囲で微調整することで
 *                直感的に扱いやすい速度にします。
 *                非線形な速度を望むならば 1 前後に設定するのがよいです。過度に大きくすると
 *                全体的な動きが抑制されすぎてしまう可能性があります。
 *
 * 2. cpi（カウント・パー・インチ）
 *    - 説明: cpi は、トラックボールが動いた際に 1 インチあたりの検出される「移動単位」
 *            を表します。cpi が高い場合、カーソルは敏感に反応し、小さな動きでも速く動きます。
 *            cpi が低いと、カーソルの動きはゆっくりになります。
 *    - 設定方法: 高い cpi を設定すると、トラックボールの小さな動きでもカーソルが敏感に
 *                動きます。低い cpi を設定すると、トラックボールの動きがゆっくりと反映されます。
 *    - 推奨設定: 非線形な動きを実現するため、cpi は 200～800 程度に設定します。この範囲で
 *                トラックボールの精細な動きを反映しつつ、カーソルの過度な移動を防ぎます。
 *
 * 3. speed_adjust_value
 *    - 説明: speed_adjust_value は、トラックボールの動きに対する加速度合いを設定します。
 *            pow() 関数を使用して非線形な加速を実現するための重要なパラメータです。
 *    - 設定方法: 値が 1.0 より小さいと全体的に「緩やかな移動」となり、1.0 より大きいと
 *                「加速度的に速く」なります。
 *    - 推奨設定: speed_adjust_value を 1.5～2.0 の範囲で調整し、ゆっくりとした動きと
 *                加速的な動きのバランスを取ることが推奨されます。例えば 1.5 に設定すると
 *                非線形のスムーズな動きを得られます。
 *
 * ■推奨設定
 * - XSCALE_FACTOR および YSCALE_FACTOR: 1.0〜1.2（動きのスムーズさと制御性のバランス）
 * - cpi: 400（小さな動きにも反応しやすく、適度な移動速度を実現）
 * - speed_adjust_value: 1.5～2.0（小さな動きには細かく、大きな動きには加速度的に速くなる）
 *
 * ■戻り値
 * - なし
 *
 * ■注意点
 * トラックボールの速度調整は、パラメータの微調整に依存します。使用感に基づき、各値を試行錯誤しながら
 * 最適な設定を見つけてください。
 * ****************************************************************************/
static void motion_to_mouse(report_mouse_t *mouse_report, float delta_x, float delta_y, uint16_t speed_adjust, uint16_t cpi) {
    float x = delta_x;
    float y = delta_y;

    // 符号を取得
    int sign_x = (x > 0) - (x < 0);
    int sign_y = (y > 0) - (y < 0);


    // スピード調整のロジックk
    float speed_adjust_value =  (float)speed_adjust / 10.0;
    x = pow(fabs(x), speed_adjust_value) / (pow(cpi / 20, speed_adjust_value)) * cpi / 20 * sign_x;
    y = pow(fabs(y), speed_adjust_value) / (pow(cpi / 20, speed_adjust_value)) * cpi / 20 * sign_y;

    // スケールファクター（必要なら定義）
    x = x / XSCALE_FACTOR;
    y = y / YSCALE_FACTOR;

    // 最終値をマウスレポートに反映
    mouse_report->x = constrain_hid(mouse_report->x + (int8_t)roundf(x));
    mouse_report->y = constrain_hid(mouse_report->y + (int8_t)roundf(y));
}

/****************************************************************************
 * eeconfig_init_kb
 *
 * EEPROMに保存されるキーボード設定を初期化する。
 * 必要に応じてデフォルト値を設定し、ユーザー定義の初期化処理を実行する。
 * ****************************************************************************/
void eeconfig_init_kb(void) {
    if (eeconfig_read_kb() == 0) {
        // mtk_config構造体の初期化: キーボード設定のデフォルト値を設定
        mtk_config.cpi_value = MTK_CPI_DEFAULT;                            // CPI（感度）の初期値
        mtk_config.cpi_changed = false;                                    // CPI変更フラグ
        mtk_config.scroll_mode = false;                                    // スクロールモード無効
        mtk_config.scroll_direction = false;                               // スクロール方向（正方向）
        mtk_config.scroll_div = MTK_SCROLL_DIV_DEFAULT;                    // スクロール分割値のデフォルト
        mtk_config.auto_mouse_mode = true;                                 // 自動マウスモード有効
        mtk_config.auto_mouse_time_out = AUTO_MOUSE_TIME;                  // 自動マウスのタイムアウト値
        mtk_config.scroll_snap_mode = MTK_SCROLLSNAP_MODE_VERTICAL;        // スクロールスナップモードのディフォルト
        mtk_config.speed_adjust_value = MTK_SPEED_ADJUST_DEFAULT;          // トラックボールの速度調整倍率
        mtk_config.oled_orient = MTK_OLED_ORIENT;                          // OLED表示方向

        // ee_config_t構造体の作成と初期化: デフォルト値をEEPROM形式で準備
        ee_config_t c = {
            .cpi  = mtk_config.cpi_value / PMW33XX_CPI_STEP,               // CPIをEEPROM形式に変換
            .sdir = mtk_config.scroll_direction,                           // スクロール方向
            .sdiv = mtk_config.scroll_div,                                 // スクロール分割値
            #ifdef POINTING_DEVICE_AUTO_MOUSE_ENABLE
              .auto_mouse = mtk_config.auto_mouse_mode,                    // 自動マウスモード
              .auto_mouse_time_out = mtk_config.auto_mouse_time_out / 100, // タイムアウト
            #endif
            .scroll_snap_mode = mtk_get_scrollsnap_mode(),                 // スクロールスナップモード
            .speed_adjust_value = mtk_get_speed_adjust_value(),            // トラックボール速度調整値
            .oled_orient = mtk_get_oled_orient_value()                     //  OLED表示方向
        };
        eeconfig_update_kb_64(c.raw);                                      // EEPROMにデフォルト値を書き込む
    }
    eeconfig_init_user();                                                  // ユーザー定義の初期化処理を呼び出す
    load_mtk_config();                                                     // 初期化後に設定値を適用する
}


/****************************************************************************
 * load_mtk_config
 *
 * EEPROMからキーボードの設定値を読み込み、各設定値を復元する。
 * OLEDの初期化やスクロール関連設定の適用も行う。
 * ****************************************************************************/
void load_mtk_config(void) {
    //ee_config.raw = eeconfig_read_kb();                       // EEPROMから生データを読み込み
    ee_config.raw = eeconfig_read_kb_64();                      // 64ビットのデータを読み込み

    mtk_set_cpi(ee_config.cpi * PMW33XX_CPI_STEP);              // CPI設定値を復元
    mtk_set_scroll_direction(ee_config.sdir);                   // スクロール方向を復元
    mtk_set_scroll_div(ee_config.sdiv);                         // スクロール分割値を復元（スクロールスピード関連）

    #ifdef POINTING_DEVICE_AUTO_MOUSE_ENABLE
      mtk_set_auto_mouse_mode(ee_config.auto_mouse);            // 自動マウスモードとタイムアウト時間を復元
      mtk_set_auto_mouse_time_out(ee_config.auto_mouse_time_out * 100);
    #endif

    mtk_set_scrollsnap_mode(ee_config.scroll_snap_mode);        // スクロールスナップモード復元
    mtk_set_speed_adjust_value(ee_config.speed_adjust_value);   // トラックボール速度調整値を復元
    mtk_set_oled_orient_value(ee_config.oled_orient);           // OLED表示方向を復元

    // OLED初期化
    oled_clear();
    oled_init(mtk_get_oled_orient_value() == 0 ? OLED_ROTATION_0 : OLED_ROTATION_270);
}


/****************************************************************************
 * save_mtk_config
 *
 * 現在のキーボード設定値をEEPROMに保存する。
 * 各設定をee_config構造体に適切に格納し、書き込みを行う。
 * ****************************************************************************/
void save_mtk_config(void) {
    ee_config.cpi = mtk_config.cpi_value / PMW33XX_CPI_STEP;        // トラックボール速度調整値を復元
    ee_config.sdir = mtk_config.scroll_direction;                   // スクロール方向と分割値を保存
    ee_config.sdiv = mtk_config.scroll_div;

    #ifdef POINTING_DEVICE_AUTO_MOUSE_ENABLE
      ee_config.auto_mouse = mtk_config.auto_mouse_mode;            // 自動マウス機能のモードとタイムアウト時間を保存
      ee_config.auto_mouse_time_out = mtk_config.auto_mouse_time_out / 100;
    #endif

    ee_config.scroll_snap_mode = mtk_get_scrollsnap_mode();         // スクロールスナップモード
    ee_config.speed_adjust_value = mtk_get_speed_adjust_value();    // トラックボール速度調整値を保存
    ee_config.oled_orient = mtk_get_oled_orient_value();            // OLED表示方向を保存

    //eeconfig_update_kb(ee_config.raw);                              // EEPROMにデータを書き込む
    eeconfig_update_kb_64(ee_config.raw);                           // 64ビットのデータを保存
}


/****************************************************************************
 * matrix_init_kb
 *
 * キーボードのマトリックス初期化処理を実行する。
 * 設定値のロードと初期化を行い、必要に応じてデフォルト設定を適用する。
 * ****************************************************************************/
void matrix_init_kb(void) {
    load_mtk_config();

    if (mtk_config.cpi_value > PMW33XX_CPI_MAX) {
        eeconfig_init_kb();
    }
}


/****************************************************************************
 * pointing_device_init_kb
 *
 * ポインティングデバイス（トラックボールなど）の初期化を実行する。
 * センサー設定や自動マウスモードの適用を行う。
 * ****************************************************************************/
void pointing_device_init_kb(void) {
    pmw33xx_init(0);                                // index 1 is the second device.
    pmw33xx_set_cpi(0, mtk_config.cpi_value);       // applies to first sensor
#ifdef POINTING_DEVICE_AUTO_MOUSE_ENABLE
    set_auto_mouse_enable(mtk_config.auto_mouse_mode);
    set_auto_mouse_timeout(mtk_config.auto_mouse_time_out);
#endif
}


/****************************************************************************
 * pointing_device_task_kb
 *
 * ポインティングデバイスの動作タスクを実行する。
 * マウスレポートに基づいてスクロールや移動処理を適用し、
 * 状況に応じて速度調整やモーション蓄積を管理する。
 * ****************************************************************************/
static int16_t scroll_h;
static int16_t scroll_v;

report_mouse_t pointing_device_task_kb(report_mouse_t mouse_report) {
    // スクロールモードの有効状態をキャッシュ
    bool is_scroll_mode = mtk_get_scroll_mode();
    int16_t scroll_div = mtk_get_scroll_div() * 8;      // スクロール分解能
    uint8_t snap_mode = mtk_config.scroll_snap_mode;
    bool scroll_direction = mtk_get_scroll_direction();

    //X軸とY軸を反転させます（X軸→Y軸、Y軸→X軸）
    int16_t x_rev =  mouse_report.y * -1;
    int16_t y_rev =  mouse_report.x * -1;

    // スクロールモードが有効な場合
    if (is_scroll_mode) {
        // X軸とY軸を反転してスクロール量に加算
        scroll_h += x_rev;
        scroll_v += y_rev;

        // スケーリングされたスクロール量を計算
        int8_t scaled_scroll_h = scroll_h / scroll_div;
        int8_t scaled_scroll_v = scroll_v / scroll_div;

        // 水平方向のスクロール処理
        if (scaled_scroll_h != 0) {
            switch (snap_mode) {
                case MTK_SCROLLSNAP_MODE_VERTICAL:
                    mouse_report.h = 0;
                    break;
                case MTK_SCROLLSNAP_MODE_HORIZONTAL:
                case MTK_SCROLLSNAP_MODE_FREE:
                    mouse_report.h = scaled_scroll_h;
                    break;
                default:
                    break;
            }
            scroll_h = 0; // 水平方向のスクロール量をリセット
        }

        // 垂直方向のスクロール処理
        if (scaled_scroll_v != 0) {
            int8_t adjusted_v = (scroll_direction) ? scaled_scroll_v : -scaled_scroll_v;
            switch (snap_mode) {
                case MTK_SCROLLSNAP_MODE_VERTICAL:
                    mouse_report.v = adjusted_v;
                    break;
                case MTK_SCROLLSNAP_MODE_HORIZONTAL:
                    mouse_report.v = 0;
                    break;
                case MTK_SCROLLSNAP_MODE_FREE:
                    mouse_report.v = adjusted_v;
                    break;
                default:
                    break;
            }
            scroll_v = 0; // 垂直方向のスクロール量をリセット
        }

        // 通常の移動量を無効化
        mouse_report.x = 0;
        mouse_report.y = 0;
    } else {
        // スクロールモードが無効な場合
        if (mtk_get_speed_adjust_enabled()) {
            uint16_t cpi = mtk_get_cpi();
            motion_to_mouse(&mouse_report, mouse_report.x, mouse_report.y, mtk_get_speed_adjust_value(), cpi);

            // motion_to_mouse後のmouse_reportの向きを修正
            int16_t temp_x = mouse_report.x;
            mouse_report.x = mouse_report.y;
            mouse_report.y = temp_x;

            // 向きを反転させる場合
            mouse_report.x *= -1;
            mouse_report.y *= -1;

        } else {
            // 速度調整が無効な場合のみCPI反転値を適用
            mouse_report.x = x_rev;
            mouse_report.y = y_rev;
        }
        mouse_report.h = 0;
        mouse_report.v = 0;
    }

    // 一定時間動作がない場合、累積されたモーションをリセット
    uint16_t elapsed_time = timer_elapsed(mtk_config.motion.active_time);
    if (elapsed_time > 300) {
        mtk_config.motion.x = 0;
        mtk_config.motion.y = 0;
        mtk_config.motion.active_time = timer_read(); // タイマーをリセット
    }


    // マウスが動作している場合、アクティブなタイマーを更新
    if (mouse_report.x || mouse_report.y || mouse_report.h || mouse_report.v) {
        mtk_config.motion.active_time = timer_read();
    }

    // モーションを累積し、後続の処理で利用可能にする
    mtk_config.motion.x += mouse_report.x;
    mtk_config.motion.y += mouse_report.y;

    // 最終的なマウスレポートを返す
    return pointing_device_task_user(mouse_report);
}

/****************************************************************************
 * layer_state_set_kb
 *
 * レイヤの状態を管理する。レイヤの変更に伴う設定変更やモード切替を行う。
 * ユーザ設定に応じたマウスレイヤを考慮して動作。
 * ****************************************************************************/

layer_state_t layer_state_set_kb(layer_state_t state) {
    // キャッシュ: 現在の最上位レイヤ
    uint8_t highest_layer = get_highest_layer(state);

    #ifdef POINTING_DEVICE_AUTO_MOUSE_ENABLE
    // オートマウスモードの有効/無効を切り替え
    if (highest_layer >= 1 && highest_layer <= 6) {
        state = remove_auto_mouse_layer(state, false); // オートマウスレイヤを無効化
        set_auto_mouse_enable(false);
    } else {
        bool auto_mouse_mode = mtk_get_auto_mouse_mode();
        if (!auto_mouse_mode) {
            state = remove_auto_mouse_layer(state, false); // レイヤをリセット
        }
        set_auto_mouse_enable(auto_mouse_mode); // 自動マウスモードの状態に応じて有効化
    }

    // マウスレイヤ時のキー押下中の遷移を抑制
    if (mtk_config.key_pressed && mtk_get_scroll_mode()) {
        state = (1UL << AUTO_MOUSE_DEFAULT_LAYER); // ユーザ設定のマウスレイヤに固定
    } else if (highest_layer == AUTO_MOUSE_DEFAULT_LAYER) {
        // タイムアウトとキー押下状態をチェック
        uint16_t elapsed_time = timer_elapsed(mtk_config.motion.active_time);
        if (elapsed_time > mtk_config.auto_mouse_time_out && !mtk_config.key_pressed) {
            state = (1UL << 0); // デフォルトレイヤ（レイヤ0）に戻す
        }
    }
    #endif

    return layer_state_set_user(state); // ユーザー定義のレイヤ状態設定を呼び出し
}


/****************************************************************************
 * start_oled_animation
 *
 * OLEDディスプレイでアニメーションを開始する。
 * 初期化処理を行い、アニメーション実行フラグを立てる。
 * ****************************************************************************/
static uint8_t anim_step = 0;      // 現在のステップ数
static uint32_t last_frame_time;   // 最後にフレームを更新した時刻
static bool anim_running = false;  // アニメーション実行フラグ

void start_oled_animation(void) {
    anim_step = 0;
    last_frame_time = timer_read();
    anim_running = true;           // アニメーション開始
}


/****************************************************************************
 * oled_task_user
 *
 * OLEDディスプレイの描画タスクを実行する。
 * アニメーションが進行中の場合、一定時間ごとに次のフレームを描画する。
 * ****************************************************************************/
bool oled_task_user(void) {
    if (!anim_running) {
        return false; // アニメーションが実行中でなければ終了
    }

    // 定数値をキャッシュ
    const uint8_t total_steps = OLED_WIDTH;            // アニメーションのステップ数
    const uint8_t row_count = 15;                     // OLEDの行数
    const uint16_t total_animation_time_ms = 3000;    // アニメーション全体の時間（ミリ秒）
    const uint16_t total_frames = total_steps + row_count; // 全体のフレーム数
    const uint16_t frame_delay_ms = total_animation_time_ms / total_frames; // フレーム遅延

    // フレーム更新が必要かチェック
    uint32_t elapsed_time = timer_elapsed(last_frame_time);
    if (elapsed_time >= frame_delay_ms) {
        last_frame_time = timer_read(); // 時刻を更新

        // アニメーションが終了している場合はリセット
        if (anim_step >= total_frames) {
            anim_running = false;
            oled_clear();
            return false;
        }

        // 各行を描画
        for (uint8_t row = 0; row < row_count; row++) {
            oled_set_cursor(0, row);
            uint8_t filled_columns = (anim_step >= row) ? (anim_step - row + 1) : 0;
            for (uint8_t col = 0; col < OLED_WIDTH; col++) {
                oled_write_char(col < filled_columns ? '>' : '-', col < filled_columns);
            }
        }

        // 次のステップへ進める
        anim_step++;
    }

    return false;
}


 /****************************************************************************
 * housekeeping_task_kb
 *
 * 主にOLEDアニメーションのタイミング管理を行う定期タスク。
 * スプリットキーボードのレイヤ状態が有効な場合にのみ動作。
 * ****************************************************************************/
#ifdef SPLIT_LAYER_STATE_ENABLE
static int oled_anim_elapsed = 0;
void housekeeping_task_kb(void) {
    housekeeping_task_user();

    // OLEDアニメーションタイマーの更新
    if (get_highest_layer(layer_state) != 0) {
        oled_anim_elapsed = timer_read(); // タイマーを更新
    }
}
#endif


/****************************************************************************
 * process_record_kb
 *
 * キーボード固有のキー押下イベントを処理する。
 * 特定のキーコードに基づいて動作を実行する。
 * ****************************************************************************/
bool process_record_kb(uint16_t keycode, keyrecord_t *record) {

    if (!process_record_user(keycode, record)) {
        return false;
    }

    switch (keycode) {
        case SCRL_MO:
            if (record->event.pressed) {
                mtk_config.key_pressed = true;       // キーが押された
                mtk_set_scroll_mode(true);           // スクロールモードを有効化
                layer_on(AUTO_MOUSE_DEFAULT_LAYER);  // マウスレイヤに切り替え
            } else {
                mtk_config.key_pressed = false;      // キーが離された
                mtk_set_scroll_mode(false);          // スクロールモードを無効化
                layer_off(AUTO_MOUSE_DEFAULT_LAYER); // マウスレイヤを解除
            }
            return false; // キーイベントを他の処理に渡さない
    }

    // strip QK_MODS part.
    if (keycode >= QK_MODS && keycode <= QK_MODS_MAX) {
        keycode &= 0xff;
    }

    if (record->event.pressed) {
#ifdef OLED_ENABLE
        set_keylog(keycode, record);
#endif
        switch (keycode) {
        // process events which works on pressed only.
            case KBC_LOAD:
                load_mtk_config();
                break;
            case KBC_SAVE:
                save_mtk_config();
                break;
            case CPI_I10:
                add_cpi(10);
                break;
            case CPI_D10:
                add_cpi(-10);
                break;
            case CPI_I100:
                add_cpi(100);
                break;
            case CPI_D100:
                add_cpi(-100);
                break;
            case CPI_I1K:
                add_cpi(1000);
                break;
            case CPI_D1K:
                add_cpi(-1000);
                break;
            case SCRL_TO:
                mtk_set_scroll_mode(!mtk_config.scroll_mode);
                break;
            case SCRL_INV:
                mtk_set_scroll_direction(!mtk_config.scroll_direction);
                break;
            case SCRL_DVI:
                add_scroll_div(1);
                break;
            case SCRL_DVD:
                add_scroll_div(-1);
                break;
#ifdef POINTING_DEVICE_AUTO_MOUSE_ENABLE
            case AM_TG:
                mtk_set_auto_mouse_mode(!mtk_config.auto_mouse_mode);
                break;
            case AM_TOUT_INC:
                add_auto_mouse_time_out(50);
                break;
            case AM_TOUT_DEC:
                add_auto_mouse_time_out(-50);
                break;
#endif
            case SSNP_VRT:
                mtk_set_scrollsnap_mode(MTK_SCROLLSNAP_MODE_VERTICAL);
                break;
            case SSNP_HOR:
                mtk_set_scrollsnap_mode(MTK_SCROLLSNAP_MODE_HORIZONTAL);
                break;
            case SSNP_FRE:
                mtk_set_scrollsnap_mode(MTK_SCROLLSNAP_MODE_FREE);
                break;
            case ADJMS_TG:
                mtk_set_speed_adjust_enabled(!mtk_config.speed_adjust_enabled);
                break;
            case ADJMS_SPD_INC:  // 調整値を増やすキー
                mtk_config.speed_adjust_value += MTK_SPEED_ADJUST_STEP;  // 増加量（例：1）
                if (mtk_config.speed_adjust_value > MTK_SPEED_ADJUST_MAX) {
                    mtk_config.speed_adjust_value = MTK_SPEED_ADJUST_MAX;  // 上限値
                }
                break;
            case ADJMS_SPD_DEC:  // 調整値を減らすキー
                mtk_config.speed_adjust_value -= MTK_SPEED_ADJUST_STEP;  // 減少量（例：1）
                if (mtk_config.speed_adjust_value <= MTK_SPEED_ADJUST_MIN) {
                    mtk_config.speed_adjust_value = MTK_SPEED_ADJUST_MIN;  // 下限値
                }
                break;
            case OLED_ORI_TG:
                start_oled_animation(); // トグルボタンでアニメーションを開始

                // メモリに格納
                mtk_set_oled_orient_value(1 - mtk_get_oled_orient_value());

                // 新しい方向に応じた初期化を実行
                oled_clear();
                oled_init(mtk_get_oled_orient_value() == 0 ? OLED_ROTATION_0 : OLED_ROTATION_270);
                break;
            default:
                return true;
        }
        return false;
    }
    return true;
}


//////////////////////////////////////////////////////////////////////////////
// configration function
/****************************************************************************
 * mtk_get_scroll_mode
 *
 * 現在のスクロールモードの有効状態を取得する。
 * ****************************************************************************/
bool mtk_get_scroll_mode(void) {
    return mtk_config.scroll_mode;
}


/****************************************************************************
 * mtk_set_scroll_mode
 *
 * スクロールモードを設定する。
 * ****************************************************************************/
void mtk_set_scroll_mode(bool mode) {
    mtk_config.scroll_mode = mode;
}


/****************************************************************************
 * mtk_get_scroll_direction
 *
 * 現在のスクロール方向を取得する。
 * ****************************************************************************/
bool mtk_get_scroll_direction(void) {
    return mtk_config.scroll_direction;
}


/****************************************************************************
 * mtk_set_scroll_direction
 *
 * スクロール方向を設定する。
 * ****************************************************************************/
void mtk_set_scroll_direction(bool direction) {
    mtk_config.scroll_direction = direction;
}


/****************************************************************************
 * mtk_get_scroll_div
 *
 * 現在のスクロール分割値を取得する。
 * 設定が無効な場合はデフォルト値（MTK_SCROLL_DIV_DEFAULT）を返す。
 * ****************************************************************************/
uint8_t mtk_get_scroll_div(void) {
    return mtk_config.scroll_div == 0 ? MTK_SCROLL_DIV_DEFAULT : mtk_config.scroll_div;
}


/****************************************************************************
 * mtk_set_scroll_div
 *
 * スクロール分割値を設定する。
 * 分割値が最大値（MTK_SCROLL_DIV_MAX）を超えた場合は、
 * 最大値に丸めて設定する。
 * ****************************************************************************/
void mtk_set_scroll_div(uint8_t div) {
    mtk_config.scroll_div = div > MTK_SCROLL_DIV_MAX ? MTK_SCROLL_DIV_MAX : div;
}


/****************************************************************************
 * mtk_get_cpi
 *
 * 現在のCPI（カウントパーインチ）を取得する。
 * 設定が無効な場合はデフォルト値（MTK_CPI_DEFAULT）を返す。
 * ****************************************************************************/
uint16_t mtk_get_cpi(void) {
    return mtk_config.cpi_value == 0 ? MTK_CPI_DEFAULT : mtk_config.cpi_value;
}


/****************************************************************************
 * mtk_set_cpi
 *
 * CPI（カウントパーインチ）の値を設定する。
 * 値が範囲外の場合、最小値または最大値に丸めて設定する。
 * 設定後、CPI変更フラグを立て、ポインティングデバイスに適用する。
 * ****************************************************************************/
void mtk_set_cpi(uint16_t cpi) {
    if (cpi > PMW33XX_CPI_MAX) {
       cpi = PMW33XX_CPI_MAX;
    } else if (cpi < PMW33XX_CPI_MIN * 2) {
       cpi = PMW33XX_CPI_MIN * 2;
    }
    mtk_config.cpi_value   = cpi;
    mtk_config.cpi_changed = true;
    pointing_device_set_cpi(cpi == 0 ? MTK_CPI_DEFAULT - 1 : cpi - 1);
}


#ifdef POINTING_DEVICE_AUTO_MOUSE_ENABLE
/****************************************************************************
 * mtk_get_auto_mouse_mode
 *
 * 自動マウスモードの現在の状態を取得する。
 * ****************************************************************************/
bool mtk_get_auto_mouse_mode(void) {
    return mtk_config.auto_mouse_mode;
}


/****************************************************************************
 * mtk_set_auto_mouse_mode
 *
 * 自動マウスモードの状態を設定する。
 * ****************************************************************************/
void mtk_set_auto_mouse_mode(bool mode) {
    mtk_config.auto_mouse_mode = mode;
}


/****************************************************************************
 * mtk_get_auto_mouse_time_out
 *
 * 自動マウスのタイムアウト値を取得する。
 * ****************************************************************************/
uint16_t mtk_get_auto_mouse_time_out(void) {
    return mtk_config.auto_mouse_time_out;
}


/****************************************************************************
 * mtk_set_auto_mouse_time_out
 *
 * 自動マウスのタイムアウト値を設定する。
 * 設定後、ポインティングデバイスに適用する。
 * ****************************************************************************/
void mtk_set_auto_mouse_time_out(uint16_t timeout) {
    mtk_config.auto_mouse_time_out = timeout;
    set_auto_mouse_timeout(mtk_config.auto_mouse_time_out);
}
#endif

/****************************************************************************
 * mtk_set_scrollsnap_mode
 *
 * スクロールスナップモードを設定する。
 * 不正なモードが指定された場合はデフォルトのモード
 * （MTK_SCROLLSNAP_MODE_VERTICAL）を設定する。
 * ****************************************************************************/
void mtk_set_scrollsnap_mode(uint8_t mode) {
    if (mode > MTK_SCROLLSNAP_MODE_FREE) {
        mode = MTK_SCROLLSNAP_MODE_VERTICAL;
    }
    mtk_config.scroll_snap_mode = mode;
}


/****************************************************************************
 * mtk_get_scrollsnap_mode
 *
 * スクロールスナップモードの現在の状態を取得する。
 * ****************************************************************************/
uint8_t mtk_get_scrollsnap_mode(void) {
    return mtk_config.scroll_snap_mode;
}


/****************************************************************************
 * mtk_get_speed_adjust_enabled
 *
 * 速度調整モードが有効かどうかを取得する。
 * ****************************************************************************/
bool mtk_get_speed_adjust_enabled(void) {
    return mtk_config.speed_adjust_enabled;
}


/****************************************************************************
 * mtk_set_speed_adjust_enabled
 *
 * 速度調整モードを有効または無効に設定する。
 * ****************************************************************************/
void mtk_set_speed_adjust_enabled(bool enabled) {
    mtk_config.speed_adjust_enabled = enabled;
}


/****************************************************************************
 * mtk_set_speed_adjust_value
 *
 * 速度調整値を設定する。設定値が最小値未満または最大値を超える場合、
 * それぞれの境界値に丸める。
 * ****************************************************************************/
void mtk_set_speed_adjust_value(uint8_t value) {
    if (value < MTK_SPEED_ADJUST_MIN) value = MTK_SPEED_ADJUST_MIN; // 最小値
    if (value > MTK_SPEED_ADJUST_MAX) value = MTK_SPEED_ADJUST_MAX; // 最大値
    mtk_config.speed_adjust_value = value;
}


/****************************************************************************
 * mtk_get_speed_adjust_value
 *
 * 現在の速度調整値を取得する。
 * ****************************************************************************/
uint8_t mtk_get_speed_adjust_value(void) {
    return mtk_config.speed_adjust_value;
}


/****************************************************************************
 * mtk_set_oled_orient_value
 *
 * OLEDの表示方向値を設定する。1または0に正規化する。
 * ****************************************************************************/
void mtk_set_oled_orient_value(uint8_t val) {
    mtk_config.oled_orient = val ? 1 : 0;
}


/****************************************************************************
 * mtk_get_oled_orient_value
 *
 * 現在のOLED表示方向値を取得する。
 * ****************************************************************************/
uint8_t mtk_get_oled_orient_value(void) {
    return mtk_config.oled_orient;
}


/****************************************************************************
 * eeconfig_update_kb_64
 *
 * EEPROMのキーボード設定領域に64ビットの値を書き込む。
 * ****************************************************************************/
void eeconfig_update_kb_64(uint64_t val) {
    for (uint8_t i = 0; i < sizeof(uint64_t); i++) {
        eeprom_update_byte((void *)(EECONFIG_KEYBOARD + i), (uint8_t)(val >> (i * 8)));
    }
}


/****************************************************************************
 * eeconfig_read_kb_64
 *
 * EEPROMのキーボード設定領域から64ビットの値を読み込む。
 * ****************************************************************************/
uint64_t eeconfig_read_kb_64(void) {
    uint64_t result = 0;
    for (uint8_t i = 0; i < sizeof(uint64_t); i++) {
        result |= ((uint64_t)eeprom_read_byte((void *)(EECONFIG_KEYBOARD + i))) << (i * 8);
    }
    return result;
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////
// OLED関連処理
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef OLED_ENABLE
/****************************************************************************
 * oled_init_kb
 *
 * OLEDの初期化を行う。
 * キーボードのマスター状態やOLEDの向きの設定に応じて回転方向を決定する。
 * ****************************************************************************/
oled_rotation_t oled_init_kb(oled_rotation_t rotation) {
    if (is_keyboard_master()) {
        if (!is_keyboard_left()) {
            if (mtk_get_oled_orient_value() == 0) {                                   // 横向きの場合(oled.orient=0)
                return OLED_ROTATION_0;
            } else {                                                                  // 縦向きの場合(oled.orient=1)
                return OLED_ROTATION_270;
            }
        } else {
            if (mtk_get_oled_orient_value() == 0) {                                   // 横向きの場合(oled.orient=0)
                return OLED_ROTATION_0;
            } else {                                                                  // 縦向きの場合(oled.orient=1)
                return OLED_ROTATION_90;
            }
        }
    } else if (is_keyboard_left()) {
        return OLED_ROTATION_180;
    }
    return rotation;
}
#endif


/****************************************************************************
 * render_logo
 *
 * OLEDにアニメーションロゴを描画する。
 * 定義されたフレームごとに変更される文字列を描画する。
 * ****************************************************************************/
static int anim_frame = 0;
static int anim_frame_interval = 0;

static void render_logo(void) {
  static char indctr[3][6][22]=
  {
    {
      {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
      {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
      {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
      {0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F, 0x90, 0x91, 0x92, 0x93, 0x94},
      {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB0, 0xB1, 0xB2, 0xB3, 0xB4},
      {0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF, 0xD0, 0xD1, 0xD2, 0xD3, 0xD4},
    },
    {
      {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
      {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
      {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
      {0x80, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F, 0x90, 0x91, 0x92, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x93, 0x94},
      {0xA0, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB0, 0xB1, 0xB2, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xB3, 0xB4},
      {0xC0, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF, 0xD0, 0xD1, 0xD2, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xD3, 0xD4},
    },
    {
      {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
      {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
      {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
      {0x80, 0x8D, 0x8E, 0x8F, 0x90, 0x91, 0x92, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x93, 0x94},
      {0xA0, 0xAD, 0xAE, 0xAF, 0xB0, 0xB1, 0xB2, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xB3, 0xB4},
      {0xC0, 0xCD, 0xCE, 0xCF, 0xD0, 0xD1, 0xD2, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xD3, 0xD4},
    }
  };

  // フレームを進める。フレームが最後まで達した場合はリセット。
  anim_frame += 1;
  if(anim_frame > 2){
      anim_frame = 0;
  }

  // 各フレームに応じてOLEDに文字列を描画
  for (int i = 0; i < 6; i++) {
      oled_write(indctr[anim_frame][i], false); // 各行のデータを描画
  }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////
// OLED表示 画像パーツ
//
// 縦表示、横表示で共通的に使用。
//////////////////////////////////////////////////////////////////////////////////////////////////////////
static const char PROGMEM bitmap_Row[] = {
    0xff, 0x81, 0xbd, 0x95, 0xad, 0x81, 0x91, 0xa9, 0x91, 0x81, 0xff
};

static const char PROGMEM bitmap_Col[] = {
    0xff, 0x81, 0x99, 0xa5, 0xa5, 0x81, 0x91, 0xa9, 0x91, 0x81, 0xff
};

static const char PROGMEM bitmap_hex[] = {
    0xff, 0x81, 0xbd, 0x91, 0xbd, 0x81, 0xa9, 0x91, 0xa9, 0x81, 0xff
};

static const char PROGMEM bitmap_cip[] = {
    0xff, 0x99, 0xa5, 0xa5, 0x81, 0xbd, 0x81, 0xbd, 0x95, 0x89, 0xff
};

static const char PROGMEM bitmap_thr[] = {
    0xff, 0x85, 0xbd, 0x85, 0xbd, 0x89, 0xbd, 0xbd, 0x95, 0xa9, 0xff
};

static const char PROGMEM bitmap_mtn[] = {
    0xff, 0x85, 0xbd, 0x85, 0x91, 0xa9, 0x91, 0x85, 0xbd, 0x85, 0xff
};

static const char PROGMEM bitmap_runtime[] = {
    0xff, 0x81, 0xbd, 0x95, 0xad, 0x81, 0x85, 0xbd, 0x85, 0x81, 0xff
};

static const char PROGMEM bitmap_rgb[] = {
    0xff, 0x81, 0xbf, 0xa1, 0xfd, 0xd5, 0x81, 0xf9, 0xc9, 0xb1, 0xff
};

static const char PROGMEM bitmap_hub[] = {
    0xff, 0x81, 0xbd, 0x91, 0xbd, 0x81, 0xbd, 0xa1, 0xbd, 0x81, 0xff
};

static const char PROGMEM bitmap_sat[] = {
    0xff, 0xdd, 0xf5, 0x81, 0xf1, 0xa9, 0xf1, 0x85, 0xbd, 0x85, 0xff
};

static const char PROGMEM bitmap_val[] = {
    0xff, 0x9d, 0xa1, 0x9d, 0xb9, 0x95, 0xb9, 0x81, 0xbd, 0xa1, 0xff
};

static const char PROGMEM bitmap_keycnt[] = {
    0xff, 0x81, 0xbd, 0x91, 0xad, 0x81, 0x85, 0xb9, 0x85, 0x99, 0xa5, 0xa5, 0x81, 0x85, 0xbd, 0x85,
    0x81, 0xff
};

static const char PROGMEM bitmap_scr_on[] = {
    0xff, 0xff, 0x99, 0xb5, 0xad, 0x99, 0xff, 0xc3, 0xbd, 0xbd, 0xdb, 0xff, 0x81, 0xed, 0xcd, 0xb3,
    0xff, 0xdb, 0xff, 0xe7, 0xdb, 0xdb, 0xe7, 0xff, 0xc3, 0xf7, 0xef, 0xc3, 0xff, 0xff
};

static const char PROGMEM bitmap_scr_off[] = {
    0xff, 0x81, 0xe7, 0xcb, 0xd3, 0xe7, 0x81, 0xbd, 0xc3, 0xc3, 0xa5, 0x81, 0xff, 0x93, 0xb3, 0xcd,
    0x81, 0xa5, 0x81, 0x99, 0xa5, 0xa5, 0x99, 0xbd, 0x95, 0x85, 0xbd, 0x95, 0x85, 0xff
};

static const char PROGMEM bitmap_aml_on[] = {
    0xff, 0x9f, 0xe3, 0xed, 0xe3, 0x9f, 0xff, 0x81, 0xfb, 0xe7, 0xfb, 0x81, 0xff, 0x81, 0xbf, 0xbf,
    0xff, 0xdb, 0xff, 0xe7, 0xdb, 0xdb, 0xe7, 0xff, 0xc3, 0xf7, 0xef, 0xc3, 0xff, 0xff
};

static const char PROGMEM bitmap_aml_off[] = {
    0xff, 0xe1, 0x9d, 0x93, 0x9d, 0xe1, 0x81, 0xff, 0x85, 0x99, 0x85, 0xff, 0x81, 0xff, 0xc1, 0xc1,
    0x81, 0xa5, 0x81, 0x99, 0xa5, 0xa5, 0x99, 0xbd, 0x95, 0x85, 0xbd, 0x95, 0x85, 0xff
};

static const char PROGMEM bitmap_adj_on[] = {
    0xff, 0x9f, 0xe3, 0xed, 0xe3, 0x9f, 0xff, 0x81, 0xbd, 0xbd, 0xc3, 0xff, 0xcf, 0xbf, 0xbd, 0xc1,
    0xfd, 0xdb, 0xff, 0xe7, 0xdb, 0xdb, 0xe7, 0xff, 0xc3, 0xf7, 0xef, 0xc3, 0xff, 0xff
};

static const char PROGMEM bitmap_adj_off[] = {
    0xff, 0xe1, 0x9d, 0x93, 0x9d, 0xe1, 0x81, 0xff, 0xc3, 0xc3, 0xbd, 0x81, 0xb1, 0xc1, 0xc3, 0xbf,
    0x83, 0xa5, 0x81, 0x99, 0xa5, 0xa5, 0x99, 0xbd, 0x95, 0x85, 0xbd, 0x95, 0x85, 0xff
};

static const char PROGMEM bitmap_ssm_on[] = {
    0xff, 0xff, 0x99, 0xb5, 0xad, 0x99, 0xff, 0x99, 0xb5, 0xad, 0x99, 0xff, 0x81, 0xfb, 0xe7, 0xfb,
    0x81, 0xdb, 0xff, 0xe7, 0xdb, 0xdb, 0xe7, 0xff, 0xc3, 0xf7, 0xef, 0xc3, 0xff, 0xff
};

static const char PROGMEM bitmap_ssm_off[] = {
    0xff, 0x81, 0xe7, 0xcb, 0xd3, 0xe7, 0x81, 0xe7, 0xcb, 0xd3, 0xe7, 0x81, 0xff, 0x85, 0x99, 0x85,
    0xff, 0xa5, 0x81, 0x99, 0xa5, 0xa5, 0x99, 0xbd, 0x95, 0x85, 0xbd, 0x95, 0x85, 0xff
};

static const char PROGMEM bitmap_mtk[] = {
    0x7e, 0x81, 0x4a, 0x82, 0x4b, 0x82, 0x7c
};

static const char PROGMEM bitmap_right[] = {
    0x42, 0x66, 0x7e, 0x7e, 0x3c, 0x18
};

static const char PROGMEM bitmap_left[] = {
    0x18, 0x3c, 0x7e, 0x7e, 0x66, 0x42
};

static const char PROGMEM bitmap_under[] = {
    0x3f, 0x7e, 0xfc, 0xfc, 0x7e, 0x3f
};

static const char PROGMEM bitmap_layer0[] = {
      0x00, 0x78, 0x88, 0x26, 0x02, 0xca, 0x62, 0x34, 0x18, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xfc, 0xfe,
      0x1f, 0x0f, 0x07, 0x07, 0x87, 0xc7, 0x67, 0x3f, 0x1f, 0xfe, 0xfc, 0xf8, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x3f, 0x7f, 0xf8, 0xfc,
      0xe6, 0xe3, 0xe1, 0xe0, 0xe0, 0xf0, 0xf8, 0x7f, 0x3f, 0x1f, 0x00, 0x00
};

static const char PROGMEM bitmap_layer1[] = {
      0x00, 0x78, 0x88, 0x26, 0x02, 0xca, 0x62, 0x34, 0x18, 0x00, 0x04, 0x02, 0x00, 0x70, 0x70, 0x78,
      0x3c, 0x1e, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x00, 0x00
};

static const char PROGMEM bitmap_layer2[] = {
      0x00, 0x78, 0x88, 0x26, 0x02, 0xca, 0x62, 0x34, 0x18, 0x02, 0x01, 0x00, 0x00, 0xf0, 0xf8, 0x7c,
      0x3e, 0x1f, 0x0f, 0x07, 0x07, 0x87, 0xc7, 0xef, 0xff, 0xfe, 0x7c, 0x38, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xf0, 0xf8, 0xfc,
      0xfe, 0xff, 0xff, 0xef, 0xe7, 0xe3, 0xe1, 0xf0, 0xf8, 0xfc, 0x00, 0x00
};

static const char PROGMEM bitmap_layer3[] = {
      0x00, 0x78, 0x88, 0x26, 0x02, 0xca, 0x62, 0x34, 0x18, 0x00, 0x04, 0x02, 0x00, 0x00, 0x78, 0x7c,
      0x3e, 0x1f, 0x0f, 0x07, 0x87, 0x87, 0xc7, 0xff, 0xff, 0xfe, 0x7c, 0x38, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x7e, 0xfe, 0xfc, 0xf8,
      0xf0, 0xe0, 0xe1, 0xe3, 0xe7, 0xff, 0xff, 0x7f, 0x3e, 0x1c, 0x00, 0x00
};

static const char PROGMEM bitmap_layer4[] = {
      0x00, 0x78, 0x88, 0x26, 0x02, 0xca, 0x62, 0x34, 0x18, 0x02, 0x01, 0x00, 0x00, 0x80, 0xc0, 0xe0,
      0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x3f, 0x3f, 0x3d, 0x3c, 0x3c,
      0x3c, 0x3c, 0x3c, 0xff, 0xff, 0xff, 0xff, 0x3c, 0x3c, 0x3c, 0x00, 0x00
};

static const char PROGMEM bitmap_layer5[] = {
      0x00, 0x78, 0x88, 0x26, 0x02, 0xca, 0x62, 0x34, 0x18, 0x00, 0x04, 0x02, 0x00, 0x00, 0xff, 0xff,
      0xff, 0xff, 0xcf, 0xcf, 0xcf, 0xcf, 0xcf, 0xcf, 0xcf, 0x8e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0xf9, 0xf9, 0xf9, 0xf1,
      0xf1, 0xf1, 0xf1, 0xf1, 0xfb, 0xff, 0xff, 0x7f, 0x3f, 0x00, 0x00, 0x00
};

static const char PROGMEM bitmap_layer6[] = {
      0x00, 0x78, 0x88, 0x26, 0x02, 0xca, 0x62, 0x34, 0x18, 0x02, 0x01, 0x00, 0x00, 0x00, 0xfe, 0xff,
      0xff, 0xff, 0xcf, 0xcf, 0xcf, 0xcf, 0xcf, 0xcf, 0xcf, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff,
      0xf1, 0xf1, 0xf1, 0xf1, 0xfb, 0xff, 0xff, 0x7f, 0x3f, 0x00, 0x00, 0x00
};

static const char PROGMEM bitmap_layer7[] = {
      0x00, 0x78, 0x88, 0x26, 0x02, 0xca, 0x62, 0x34, 0x18, 0x00, 0x04, 0x02, 0x00, 0x00, 0x3f, 0x3f,
      0x3f, 0x3f, 0x0f, 0x8f, 0xcf, 0xef, 0xff, 0xff, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xc0, 0xfe,
      0xff, 0xff, 0xff, 0xff, 0xc3, 0xc1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};



//////////////////////////////////////////////////////////////////////////////////////////////////////////
// OLED表示 共通部品
//
// 縦表示、横表示で共通的に使用。
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// レイヤごとの画像データを配列で管理
const char *bitmap_layers[] = {
    bitmap_layer0,  // レイヤ0の画像
    bitmap_layer1,  // レイヤ1の画像
    bitmap_layer2,  // レイヤ2の画像
    bitmap_layer3,  // レイヤ3の画像
    bitmap_layer4,  // レイヤ4の画像
    bitmap_layer5,  // レイヤ5の画像
    bitmap_layer6,  // レイヤ6の画像
    bitmap_layer7   // レイヤ7の画像
};

const uint16_t bitmap_layer_sizes[] = {
    sizeof(bitmap_layer0),
    sizeof(bitmap_layer1),
    sizeof(bitmap_layer2),
    sizeof(bitmap_layer3),
    sizeof(bitmap_layer4),
    sizeof(bitmap_layer5),
    sizeof(bitmap_layer6),
    sizeof(bitmap_layer7)
};


/****************************************************************************
 * render_image
 *
 * 画像を指定された位置に描画する。
 * @param col         カーソルの列位置
 * @param row         カーソルの行位置
 * @param bitmap      描画する画像データ
 * @param bitmap_size 画像データのサイズ
 * ****************************************************************************/
void render_image(uint8_t col, uint8_t row, const char *bitmap, uint16_t bitmap_size) {
    oled_set_cursor(col, row);                // カーソル位置を設定
    oled_write_raw_P(bitmap, bitmap_size);    // 画像データを描画
}


/****************************************************************************
 * render_image2
 *
 * 画像を上下2行に分割して指定された位置に描画する。
 * @param col         カーソルの列位置
 * @param row         カーソルの行位置
 * @param bitmap      描画する画像データ
 * @param bitmap_size 画像データのサイズ
 * ****************************************************************************/
void render_image2(uint8_t col, uint8_t row, const char *bitmap, uint16_t bitmap_size) {
    // 1行分のデータサイズを計算 (横幅が40pxの場合 = 40 ÷ 8 = 5バイト)
    const uint8_t bytes_per_row = bitmap_size / 2;

    // 上下2行分に分割して描画
    oled_set_cursor(col, row);       // 1行目のカーソル位置
    oled_write_raw_P(bitmap, bytes_per_row); // 1行目を描画

    oled_set_cursor(col, row + 1);   // 2行目のカーソル位置
    oled_write_raw_P(bitmap + bytes_per_row, bytes_per_row); // 2行目を描画
}


/****************************************************************************
 * render_fixed_string
 *
 * 指定された位置に固定文字列を描画する。
 * @param col  カーソルの列位置
 * @param row  カーソルの行位置
 * @param text 描画する文字列
 * ****************************************************************************/
void render_fixed_string(uint8_t col, uint32_t row, const char *text) {
    oled_set_cursor(col, row);
    oled_write_ln_P(text, false);
}


/****************************************************************************
 * render_indicator
 *
 * インジケータ (#) を指定された位置に描画する。
 * レイヤ状態に応じてアクティブな表示を切り替える。
 * @param col   カーソルの列位置
 * @param row   カーソルの行位置
 * @param layer 現在のレイヤ
 * ****************************************************************************/
void render_indicator(uint8_t col, uint32_t row, uint8_t layer) {
    bool active = (layer < 8 && row <= layer);
    oled_set_cursor(col, row);
    if (active) {
        render_image(col, row, bitmap_under, sizeof(bitmap_under));
    } else {
        oled_write_ln_P("|", active);
    }
}


/****************************************************************************
 * render_indicator2
 *
 * レイヤごとのインジケータを描画する（左から増加版）。
 * @param col   カーソルの列位置
 * @param row   カーソルの行位置
 * @param layer 現在のレイヤ
 * ****************************************************************************/
void render_indicator2(uint8_t col, uint32_t row, uint8_t layer) {
    // OLED描画バッファを明示的に初期化
    oled_set_cursor(OLED_WIDTH, row);  // カーソルを移動
    oled_write_char(' ', false);    // 描画領域を空白で初期化

    // カウントに基づく表示上限を計算
    uint8_t active_indicators = type_count / 3000;
    if (active_indicators > OLED_WIDTH) {
        active_indicators = OLED_WIDTH; // 表示幅を制限
    }

    for (uint8_t i = 0; i < OLED_WIDTH; i++) {
        oled_set_cursor(col + i, row); // カーソルを現在の列に設定

        if (layer == 0) { // レイヤ0の場合
            if (i < active_indicators) {
                render_image(col + i, row, bitmap_mtk, sizeof(bitmap_mtk)); // 打鍵カウントに基づいてメンタコさんを表示
            } else {
                oled_write_char('-', false); // デフォルトで「-」を表示
            }
        } else if (i < layer) {
            // アクティブレイヤは画像を表示
            render_image(col + i, row, bitmap_right, sizeof(bitmap_right));
        } else if (layer == (DYNAMIC_KEYMAP_LAYER_COUNT - 1)) {
            // 現在のレイヤが最後のレイヤなら、非アクティブ部分をメンタコさんを表示
            render_image(col + i, row, bitmap_mtk, sizeof(bitmap_mtk));
        } else {
            // 非アクティブレイヤは通常の「-」
            oled_write_char('-', false);
        }
    }
}



/****************************************************************************
 * render_indicator3
 *
 * レイヤごとのインジケータを描画する（右から増加版）。
 * @param col   カーソルの列位置
 * @param row   カーソルの行位置
 * @param layer 現在のレイヤ
 ****************************************************************************/
void render_indicator_slave3(uint8_t col, uint32_t row, uint8_t layer) {
    // OLED描画バッファを明示的に初期化
    //oled_set_cursor(OLED_WIDTH - 1, row);  // カーソルを右端に移動
    //oled_write_char(' ', false);                // 描画領域を空白で初期化

    // カウントに基づく表示上限を計算
    uint8_t active_indicators = type_count / 3000;
    if (active_indicators > OLED_WIDTH) {
        active_indicators = OLED_WIDTH; // 表示幅を制限
    }

    // 右から左に描画
    for (int8_t i = OLED_WIDTH - 1; i >= 0; i--) { // カーソルを右端から左へ移動
        oled_set_cursor(col + i, row); // カーソルを現在の列に設定

        if (layer == 0) { // レイヤ0の場合
            if (OLED_WIDTH - 1 - i < active_indicators) {
                render_image(col + i, row, bitmap_mtk, sizeof(bitmap_mtk)); // 打鍵カウントに基づいてメンタコさんを表示
            } else {
                oled_write_char('-', false); // デフォルトで「-」を表示
            }
        } else if (OLED_WIDTH - 1 - i < layer) {
            // アクティブレイヤは画像を表示
            render_image(col + i, row, bitmap_left, sizeof(bitmap_left));
        } else if (layer == (DYNAMIC_KEYMAP_LAYER_COUNT - 1)) {
            // 現在のレイヤが最後のレイヤなら、非アクティブ部分をメンタコさんを表示
            render_image(col + i, row, bitmap_mtk, sizeof(bitmap_mtk));
        } else {
            // 非アクティブレイヤは通常の「-」
            oled_write_char('-', false);
        }
    }
}

/****************************************************************************
 * render_indicator_slave
 *
 * レイヤごとのインジケータを描画する（左から増加版）。
 * @param col   カーソルの列位置
 * @param row   カーソルの行位置
 * @param layer 現在のレイヤ
 * ****************************************************************************/
void render_indicator_slave(uint8_t col, uint32_t row, uint8_t layer) {
    // OLED描画バッファを明示的に初期化
    oled_set_cursor(OLED_WIDTH_slave, row);  // カーソルを移動
    oled_write_char(' ', false);    // 描画領域を空白で初期化

    // カウントに基づく表示上限を計算
    uint8_t active_indicators = type_count / 3000;
    if (active_indicators > OLED_WIDTH_slave) {
        active_indicators = OLED_WIDTH_slave; // 表示幅を制限
    }

    for (uint8_t i = 0; i < OLED_WIDTH_slave; i++) {
        oled_set_cursor(col + i, row); // カーソルを現在の列に設定

        if (layer == 0) { // レイヤ0の場合
            if (i < active_indicators) {
                render_image(col + i, row, bitmap_mtk, sizeof(bitmap_mtk)); // 打鍵カウントに基づいてメンタコさんを表示
            } else {
                oled_write_char('-', false); // デフォルトで「-」を表示
            }
        } else if (i < layer) {
            // アクティブレイヤは画像を表示
            render_image(col + i, row, bitmap_right, sizeof(bitmap_right));
        } else if (layer == (DYNAMIC_KEYMAP_LAYER_COUNT - 1)) {
            // 現在のレイヤが最後のレイヤなら、非アクティブ部分をメンタコさんを表示
            render_image(col + i, row, bitmap_mtk, sizeof(bitmap_mtk));
        } else {
            // 非アクティブレイヤは通常の「-」
            oled_write_char('-', false);
        }
    }
}

/****************************************************************************
 * render_indicator_slave2
 *
 * レイヤごとのインジケータを描画する（右から増加版）。
 * @param col   カーソルの列位置
 * @param row   カーソルの行位置
 * @param layer 現在のレイヤ
 * ****************************************************************************/
void render_indicator_slave2(uint8_t col, uint32_t row, uint8_t layer) {
    // OLED描画バッファを明示的に初期化
    //oled_set_cursor(OLED_WIDTH_slave - 1, row);  // カーソルを右端に移動
    //oled_write_char(' ', false);                // 描画領域を空白で初期化

    // カウントに基づく表示上限を計算
    uint8_t active_indicators = type_count / 3000;
    if (active_indicators > OLED_WIDTH_slave) {
        active_indicators = OLED_WIDTH_slave; // 表示幅を制限
    }

    // 右から左に描画
    for (int8_t i = OLED_WIDTH_slave - 1; i >= 0; i--) { // カーソルを右端から左へ移動
        oled_set_cursor(col + i, row); // カーソルを現在の列に設定

        if (layer == 0) { // レイヤ0の場合
            if (OLED_WIDTH_slave - 1 - i < active_indicators) {
                render_image(col + i, row, bitmap_mtk, sizeof(bitmap_mtk)); // 打鍵カウントに基づいてメンタコさんを表示
            } else {
                oled_write_char('-', false); // デフォルトで「-」を表示
            }
        } else if (OLED_WIDTH_slave - 1 - i < layer) {
            // アクティブレイヤは画像を表示
            render_image(col + i, row, bitmap_left, sizeof(bitmap_left));
        } else if (layer == (DYNAMIC_KEYMAP_LAYER_COUNT - 1)) {
            // 現在のレイヤが最後のレイヤなら、非アクティブ部分をメンタコさんを表示
            render_image(col + i, row, bitmap_mtk, sizeof(bitmap_mtk));
        } else {
            // 非アクティブレイヤは通常の「-」
            oled_write_char('-', false);
        }
    }
}


/****************************************************************************
 * render_key_value
 *
 * キー名と値を指定された位置に描画する。
 * @param col    カーソルの列位置
 * @param row    カーソルの行位置
 * @param key    描画するキー名
 * @param format 値のフォーマット文字列
 * @param value  描画する値
 * @param invert 色反転を適用するか
 * ****************************************************************************/
void render_key_value(uint8_t col, uint32_t row, const char *key, const char *format, int value, bool invert) {
    oled_set_cursor(col, row);
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%s%s", key, format);
    char final_buffer[16];
    snprintf(final_buffer, sizeof(final_buffer), buffer, value);
    oled_write_ln_P(final_buffer, invert);
}


/****************************************************************************
 * render_snap_mode
 *
 * スクロールスナップモードを指定された位置に描画する。
 * @param col      カーソルの列位置
 * @param row      カーソルの行位置
 * @param snap_mode 現在のスナップモード
 * ****************************************************************************/
void render_snap_mode(uint8_t col, uint32_t row, uint8_t snap_mode) {
    static const char *snap_modes[] = {"VER", "HOR", "FRE", "UNK"};
    uint8_t index = (snap_mode > 2) ? 3 : snap_mode;
    oled_set_cursor(col, row);
    oled_write_ln_P(snap_modes[index], false);
}


/****************************************************************************
 * render_decimal_value
 *
 * 小数値を指定された位置に描画する。
 * @param col             カーソルの列位置
 * @param row             カーソルの行位置
 * @param key             描画するキー名
 * @param integer_part    整数部分
 * @param fractional_part 小数部分
 * @param invert          色反転を適用するか
 * ****************************************************************************/
void render_decimal_value(uint8_t col, uint32_t row, const char *key, int integer_part, int fractional_part, bool invert) {
    char buffer[16]; // 十分なサイズを確保
    snprintf(buffer, sizeof(buffer), "%s%d.%d", key, integer_part, fractional_part); // 整数部と小数部を結合
    oled_set_cursor(col, row);
    oled_write_ln_P(buffer, invert);
}


/****************************************************************************
 * set_keylog
 *
 * キーコードとそのイベント情報を記録し、キーの位置やコード、名前を格納する。
 * @param keycode   イベント対象のキーコード
 * @param record    キーイベントの記録データ
 * ****************************************************************************/
#define MAX_KEYLOG_STR_LEN 11 // 必要な長さに応じて変更
static char keylog_str_h[MAX_KEYLOG_STR_LEN] = {'\0'};
static char keylog_str_n[MAX_KEYLOG_STR_LEN] = {'\0'};
static char keylog_str_R[MAX_KEYLOG_STR_LEN] = {'\0'};
static char keylog_str_C[MAX_KEYLOG_STR_LEN] = {'\0'};
const char code_to_name[60] = {
    ' ', ' ', ' ', ' ', 'a', 'b', 'c', 'd', 'e', 'f',
    'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p',
    'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
    '1', '2', '3', '4', '5', '6', '7', '8', '9', '0',
    'R', 'E', 'B', 'T', '_', '-', '=', '[', ']', '\\',
    '#', ';', '\'', '`', ',', '.', '/', ' ', ' ', ' '};

void set_keylog(uint16_t keycode, keyrecord_t *record) {
    char name = ' ';
    if ((keycode >= QK_MOD_TAP && keycode <= QK_MOD_TAP_MAX) ||
        (keycode >= QK_LAYER_TAP && keycode <= QK_LAYER_TAP_MAX)) {
            keycode = keycode & 0xFF;
    }

    if (keycode < 60) {
        name = code_to_name[keycode];
    }

  // update keylog
  //snprintf(keylog_str, sizeof(keylog_str), "c:%-3dr:%-3dk:%04x %-c",
  //         record->event.key.row, record->event.key.col,
  //         keycode, name);
  snprintf(keylog_str_R, sizeof(keylog_str_R), "%-3d"  ,record->event.key.row);
  snprintf(keylog_str_C, sizeof(keylog_str_C), "%-3d"  ,record->event.key.col);
  snprintf(keylog_str_h, sizeof(keylog_str_h), "%04x"  ,keycode);
  snprintf(keylog_str_n, sizeof(keylog_str_n), "%-c"   ,name);
}


/****************************************************************************
 * count_type
 *
 * 打鍵数をカウントする関数。
 * グローバル変数 `type_count` をインクリメントする。
 * ****************************************************************************/
void count_type(void) {
    type_count++;
}


/****************************************************************************
 * oled_write_type_count
 *
 * 打鍵数を OLED に表示する関数。
 * 数値を7桁右寄せで描画する。
 * @param col カーソルの列位置
 * @param row カーソルの行位置
 * ****************************************************************************/
void oled_write_type_count(uint8_t col, uint32_t row) {
    char type_count_str[7]; // 6桁 + 終端文字
    snprintf(type_count_str, sizeof(type_count_str), "%5lu", type_count); // 右寄せ形式
    oled_set_cursor(col, row);
    oled_write_ln(type_count_str, false); // 打鍵数を右寄せで描画
}


/****************************************************************************
 * process_record_user
 *
 * キーイベント処理。
 * 打鍵数をカウントする関数。
 * @param keycode   イベント対象のキーコード
 * @param record    キーイベントの記録データ
 * @return          他のキー処理を継続する場合はtrue
 * ****************************************************************************/
bool process_record_user(uint16_t keycode, keyrecord_t *record) {
#ifdef OLED_ENABLE
    if (record->event.pressed) {
        count_type(); // 打鍵数をカウント
    }
#endif
    return true;
}


/****************************************************************************
 * oled_write_uptime
 *
 * システムの稼働時間を OLED に分単位で表示する。
 * 最大3桁で右詰め表示を行う。
 * @param col カーソルの列位置
 * @param row カーソルの行位置
 * ****************************************************************************/
void oled_write_uptime(uint8_t col, uint32_t row) {
    static uint32_t uptime_minutes;
    char uptime_str[5]; // 最大3桁 + スペース埋め用 + 終端文字

    // 経過時間を分単位で計算
    uptime_minutes = (timer_read32() / 1000) / 60;

    // 最大3桁で右詰め表示
    snprintf(uptime_str, sizeof(uptime_str), "%3lu", uptime_minutes);
    oled_set_cursor(col, row);
    oled_write_ln(uptime_str, false);            // 経過時間
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
// OLED表示 横表示部品
//
// 行単位の画面表示メソッド。横表示のため、7行分作成。
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void oled_render_hor0(uint32_t row) {
    uint8_t layer = get_highest_layer(layer_state);
    render_fixed_string(0, row, "Layer");
    render_indicator(5, row, layer);
    render_fixed_string(7, row, keylog_str_h);
    render_fixed_string(11, row, "(");
    render_fixed_string(12, row, keylog_str_n);
    render_fixed_string(13, row, ")");
    render_indicator(15, row, layer);
    render_key_value(16, row, " AML ", "", 0, mtk_get_auto_mouse_mode());
}

void oled_render_hor1(uint32_t row) {
    uint8_t layer = get_highest_layer(layer_state);

    // 現在のレイヤに対応する画像を取得
    if (layer < sizeof(bitmap_layers) / sizeof(bitmap_layers[0])) {
        render_image2(0, row, bitmap_layers[layer], bitmap_layer_sizes[layer]);
    } else {
        // 対応する画像がない場合のフォールバック処理（必要なら追加）
        render_key_value(1, row , "", "%-d", layer, false);
        render_fixed_string(2, row, "th");
    }

    render_indicator(5, row, layer);
    render_key_value(7, row, "CPI:", "%-4d", mtk_config.cpi_value, false);
    render_indicator(15, row, layer);
    render_key_value(17, row, "", "%-3d", mtk_get_auto_mouse_time_out(), false);
}

void oled_render_hor2(uint32_t row) {
    uint8_t layer = get_highest_layer(layer_state);
    //render_key_value(2, row, "", "%-d", layer, false);
    render_indicator(5, row, layer);
    render_key_value(7, row, "THR:", "%-3d", AUTO_MOUSE_THRESHOLD, false);
    render_indicator(15, row, layer);
    render_key_value(16, row, " ADJ ", "", 0, mtk_get_speed_adjust_enabled());
}

void oled_render_hor3(uint32_t row) {
    uint8_t layer = get_highest_layer(layer_state);
    render_indicator(5, row, layer);
    render_key_value(7, row, "MTN:", "%-3d", abs(mtk_config.motion.x) + abs(mtk_config.motion.y), false);
    render_indicator(15, row, layer);
    render_decimal_value(17, row, "", mtk_config.speed_adjust_value / 10, mtk_config.speed_adjust_value % 10, false);
}

void oled_render_hor4(uint32_t row) {
    uint8_t layer = get_highest_layer(layer_state);
    const uint8_t num_layers = sizeof(layer_names) / sizeof(layer_names[0]);     // 配列の長さを取得
    const char *layer_name = (layer < num_layers) ? layer_names[layer] : "UNK";  // 配列範囲内であれば描画、それ以外は "UNK" を表示
    render_fixed_string(0, row, layer_name);

    render_indicator(5, row, layer);
    render_key_value(7, row, "RGB:", "%-2d", rgblight_get_mode(), false);
    render_indicator(15, row, layer);
    render_key_value(16, row, " SCR ", "", 0, mtk_get_scroll_mode());
}

void oled_render_hor5(uint32_t row) {
    uint8_t layer = get_highest_layer(layer_state);
    render_indicator(5, row, layer);
    render_key_value(7, row, "HUB:", "%-3d", rgblight_get_hue(), false);
    render_indicator(15, row, layer);
    render_key_value(17, row, "", "%-3d", mtk_config.scroll_div, false);
}

void oled_render_hor6(uint32_t row) {
    uint8_t layer = get_highest_layer(layer_state);
    render_fixed_string(0, row, "Mtk64");
    render_indicator(5, row, layer);
    render_key_value(7, row, "SAT:", "%-3d", rgblight_get_sat(), false);
    render_indicator(15, row, layer);
    render_key_value(16, row, " SSM ", "", 0, mtk_config.scroll_snap_mode == 0 || mtk_config.scroll_snap_mode == 1);
}

void oled_render_hor7(uint32_t row) {
    uint8_t layer = get_highest_layer(layer_state);
    render_fixed_string(1, row, "erp");
    render_indicator(5, row, layer);
    render_key_value(7, row, "VAL:", "%-3d", rgblight_get_val(), false);
    render_indicator(15, row, layer);
    render_snap_mode(17, row, mtk_config.scroll_snap_mode);
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////
//  OLED表示 縦表示部品
//
// 行単位の画面表示メソッド。縦表示のため、15行分作成。
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void oled_render_ver0(uint32_t row) {
    //render_key_value(0, row, " AML ", "", 0, mtk_get_auto_mouse_mode());
    if (mtk_get_auto_mouse_mode()) {
        render_image(0, row, bitmap_aml_on, sizeof(bitmap_aml_on));
    } else {
        render_image(0, row, bitmap_aml_off, sizeof(bitmap_aml_off));
    }

    //render_key_value(5, row, " SCR ", "", 0, mtk_get_scroll_mode());
    if (mtk_get_scroll_mode()) {
        render_image(5, row, bitmap_scr_on, sizeof(bitmap_scr_on));
    } else {
        render_image(5, row, bitmap_scr_off, sizeof(bitmap_scr_off));
    }
}

void oled_render_ver1(uint32_t row) {
    render_key_value(1, row, "", "%-3d", mtk_get_auto_mouse_time_out(), false);
    render_key_value(6, row, "", "%3d", mtk_config.scroll_div, false);
}

void oled_render_ver2(uint32_t row) {
    //render_key_value(0, row, " ADJ ", "", 0, mtk_get_speed_adjust_enabled());
    if (mtk_get_speed_adjust_enabled()) {
        render_image(0, row, bitmap_adj_on, sizeof(bitmap_adj_on));
    } else {
        render_image(0, row, bitmap_adj_off, sizeof(bitmap_adj_off));
    }

    //render_key_value(5, row, " SSM ", "", 0, mtk_config.scroll_snap_mode == 0 || mtk_config.scroll_snap_mode == 1);
    if (mtk_config.scroll_snap_mode == MTK_SCROLLSNAP_MODE_VERTICAL || mtk_config.scroll_snap_mode == MTK_SCROLLSNAP_MODE_HORIZONTAL) {
        render_image(5, row, bitmap_ssm_on, sizeof(bitmap_ssm_on));
    } else {
        render_image(5, row, bitmap_ssm_off, sizeof(bitmap_ssm_off));
    }
}

void oled_render_ver3(uint32_t row) {
    render_decimal_value(1, row, "", mtk_config.speed_adjust_value / 10, mtk_config.speed_adjust_value % 10, false);
    render_snap_mode(6, row, mtk_config.scroll_snap_mode);
}

void oled_render_ver4(uint32_t row) {
    uint8_t layer = get_highest_layer(layer_state);
    render_indicator2(0, row, layer);
}

void oled_render_ver5(uint32_t row) {
    uint8_t layer = get_highest_layer(layer_state);
    render_fixed_string(0, row, "Layer");
    //render_key_value(7, row, "", "%-d", layer, false);
    //render_fixed_string(8, row, "th");

    // 現在のレイヤに対応する画像を取得
    if (layer < sizeof(bitmap_layers) / sizeof(bitmap_layers[0])) {
        render_image2(5, row, bitmap_layers[layer], bitmap_layer_sizes[layer]);
    } else {
        // 対応する画像がない場合のフォールバック処理（必要なら追加）
        render_key_value(7, row, "", "%-d", layer, false);
        render_fixed_string(8, row, "th");
    }
}

void oled_render_ver6(uint32_t row) {
    uint8_t layer = get_highest_layer(layer_state);
    const uint8_t num_layers = sizeof(layer_names) / sizeof(layer_names[0]);     // 配列の長さを取得
    const char *layer_name = (layer < num_layers) ? layer_names[layer] : "UNK";  // 配列範囲内であれば描画、それ以外は "UNK" を表示
    render_fixed_string(0, row, layer_name);

    // 現在のレイヤに対応する画像を取得
    if (layer < sizeof(bitmap_layers) / sizeof(bitmap_layers[0])) {
        render_image2(5, row - 1, bitmap_layers[layer], bitmap_layer_sizes[layer]);
    } else {
        // 対応する画像がない場合のフォールバック処理（必要なら追加）
        render_key_value(7, row - 1, "", "%-d", layer, false);
        render_fixed_string(8, row - 1, "th");
    }

}

void oled_render_ver7(uint32_t row) {
    uint8_t layer = get_highest_layer(layer_state);
    render_indicator2(0, row, layer);
}

void oled_render_ver8(uint32_t row) {
    render_image(0, row, bitmap_Row, sizeof(bitmap_Row));
    render_fixed_string(2, row, keylog_str_R);
    render_image(5, row, bitmap_Col, sizeof(bitmap_Col));
    render_fixed_string(7, row, keylog_str_C);
}

void oled_render_ver9(uint32_t row) {
    render_image(0, row, bitmap_hex, sizeof(bitmap_hex));
    render_fixed_string(2, row, keylog_str_h);
    render_key_value(7, row, "(", "", 0, false);
    render_fixed_string(8, row, keylog_str_n);
    render_key_value(9, row, ")", "", 0, false);
}

void oled_render_ver10(uint32_t row) {
    uint8_t layer = get_highest_layer(layer_state);
    render_indicator2(0, row, layer);
}

void oled_render_ver11(uint32_t row) {
    //render_key_value(0, row, "Ci", "", 0, true);
    render_image(0, row, bitmap_cip, sizeof(bitmap_cip));
    render_key_value(2, row, "", "%3d", mtk_config.cpi_value, false);
    //render_key_value(5, row, "Rg", "", 0, true);
    render_image(5, row, bitmap_rgb, sizeof(bitmap_rgb));
    render_key_value(7, row, "", "%3d", rgblight_get_mode(), false);
}

void oled_render_ver12(uint32_t row) {
    //render_key_value(0, row, "Th", "", 0, true);
    render_image(0, row, bitmap_thr, sizeof(bitmap_thr));
    render_key_value(2, row, "", "%3d", AUTO_MOUSE_THRESHOLD, false);
    //render_key_value(5, row, "Hu", "", 0, true);
    render_image(5, row, bitmap_hub, sizeof(bitmap_hub));
    render_key_value(7, row, "", "%3d", rgblight_get_hue(), false);
}

void oled_render_ver13(uint32_t row) {
    //render_key_value(0, row, "Mn", "", 0, true);
    render_image(0, row, bitmap_mtn, sizeof(bitmap_mtn));
    render_key_value(2, row, "", "%3d", abs(mtk_config.motion.x) + abs(mtk_config.motion.y), false);
    //render_key_value(5, row, "Sa", "", 0, true);
    render_image(5, row, bitmap_sat, sizeof(bitmap_sat));
    render_key_value(7, row, "", "%3d", rgblight_get_sat(), false);

}

void oled_render_ver14(uint32_t row) {
    //render_key_value(0, row, "Ti", "", 0, true);
    render_image(0, row, bitmap_runtime, sizeof(bitmap_runtime));
    oled_write_uptime(2, row);
    //render_key_value(5, row, "Va", "", 0, true);
    render_image(5, row, bitmap_val, sizeof(bitmap_val));
    render_key_value(7, row, "", "%3d", rgblight_get_val(), false);
}

void oled_render_ver15(uint32_t row) {
    //render_key_value(0, row, "Cnt", "", 0, true);
    render_image(0, row, bitmap_keycnt, sizeof(bitmap_keycnt));
    oled_write_type_count(4, row);
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////
//  OLED表示 スレーブ側
//
// 行単位の画面表示メソッド。スレーブ側の表示部品。
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void oled_render_slave1(uint32_t row) {
    uint8_t layer = get_highest_layer(layer_state);

    // 現在のレイヤに対応する画像を取得
    if (layer < sizeof(bitmap_layers) / sizeof(bitmap_layers[0])) {
        render_image2(0, row, bitmap_layers[layer], bitmap_layer_sizes[layer]);
    } else {
        // 対応する画像がない場合のフォールバック処理（必要なら追加）
        render_key_value(0, row , "", "%-d", layer, false);
        render_fixed_string(2, row, "th");
    }

    render_indicator_slave(5, row, layer);
}

void oled_render_slave2(uint32_t row) {
    uint8_t layer = get_highest_layer(layer_state);

      // 現在のレイヤに対応する画像を取得
    if (layer < sizeof(bitmap_layers) / sizeof(bitmap_layers[0])) {
        render_image2(16, row, bitmap_layers[layer], bitmap_layer_sizes[layer]);
    } else {
        // 対応する画像がない場合のフォールバック処理（必要なら追加）
        render_key_value(16, row , "", "%-d", layer, false);
        render_fixed_string(2, row, "th");
    }
}

void oled_render_slave3(uint32_t row) {
    uint8_t layer = get_highest_layer(layer_state);
    render_indicator_slave2(1, row, layer);
}

void oled_clear_line(uint8_t row) {
    oled_set_cursor(0, row);  // 行の先頭にカーソルを設定
    for (uint8_t col = 0; col < 21; col++) {
        oled_write_char(' ', false);  // 空白で行をクリア
    }
}

/****************************************************************************
// * oled_partial_update_hor
// *
// * 指定された行範囲内でOLEDの部分更新を行う（横表示用）。
// * 各行に対応する描画関数を動的に呼び出す。
// * @param start_row 更新を開始する行番号
// * @param end_row   更新を終了する行番号
// * ****************************************************************************/
void oled_partial_update_hor(uint8_t start_row, uint8_t end_row) {
    // 関数ポインタ配列を作成
    void (*render_functions[])(uint8_t) = {
        (void (*)(uint8_t))oled_render_hor0,
        (void (*)(uint8_t))oled_render_hor1,
        (void (*)(uint8_t))oled_render_hor2,
        (void (*)(uint8_t))oled_render_hor3,
        (void (*)(uint8_t))oled_render_hor4,
        (void (*)(uint8_t))oled_render_hor5,
        (void (*)(uint8_t))oled_render_hor6,
        (void (*)(uint8_t))oled_render_hor7
    };

    // 指定範囲の行を描画
    for (uint8_t row = start_row; row <= end_row && row < sizeof(render_functions) / sizeof(render_functions[0]); row++) {
        render_functions[row](row);
    }
}


/****************************************************************************
 * oled_partial_update_ver
 *
 * 指定された行範囲内でOLEDの部分更新を行う（縦表示用）。
 * 各行に対応する描画関数を動的に呼び出す。
 * @param start_row 更新を開始する行番号
 * @param end_row   更新を終了する行番号
 * ****************************************************************************/
void oled_partial_update_ver(uint8_t start_row, uint8_t end_row) {
    // 関数ポインタ配列を作成
    void (*render_functions[])(uint8_t) = {
        (void (*)(uint8_t))oled_render_ver0,
        (void (*)(uint8_t))oled_render_ver1,
        (void (*)(uint8_t))oled_render_ver2,
        (void (*)(uint8_t))oled_render_ver3,
        (void (*)(uint8_t))oled_render_ver4,
        (void (*)(uint8_t))oled_render_ver5,
        (void (*)(uint8_t))oled_render_ver6,
        (void (*)(uint8_t))oled_render_ver7,
        (void (*)(uint8_t))oled_render_ver8,
        (void (*)(uint8_t))oled_render_ver9,
        (void (*)(uint8_t))oled_render_ver10,
        (void (*)(uint8_t))oled_render_ver11,
        (void (*)(uint8_t))oled_render_ver12,
        (void (*)(uint8_t))oled_render_ver13,
        (void (*)(uint8_t))oled_render_ver14,
        (void (*)(uint8_t))oled_render_ver15
    };

    // 指定範囲の行を描画
    for (uint8_t row = start_row; row <= end_row && row < sizeof(render_functions) / sizeof(render_functions[0]); row++) {
        render_functions[row](row);
    }
}


/****************************************************************************
 * oled_task_kb
 *
 * OLEDディスプレイのタスクを処理する関数。
 * - マスター/スレーブ状態に応じて異なる情報を表示。
 * - マスターの場合、OLEDの向きに応じて表示内容を更新。
 * - スレーブの場合、レイヤ情報やキー押下情報、アニメーションを表示。
 *
 * @return 常に false を返す（QMKの規約に準拠）。
 * ****************************************************************************/
bool oled_task_kb(void) {                                                                 // OLEDのタスクを処理する関数。
    static uint32_t last_update = 0;                                                      // 前回の更新タイムスタンプを保持する静的変数。

    if (timer_elapsed(last_update) > 100) {                                               // 前回の更新から100ms経過していたら更新を実行。
        last_update = timer_read();                                                       // 現在の時間を last_update に設定。

        if (is_keyboard_master()) {                                                       // マスターデバイスであるかどうかを判定。
            if (mtk_get_oled_orient_value() == 0) {                                       // 横向きの場合(oled.orient=1)
                oled_partial_update_hor(0, 7);                                            // 0行目から7行目まで部分的に更新。
            } else if (mtk_get_oled_orient_value() == 1) {                                // 縦向きの場合(oled.orient=0)
                oled_partial_update_ver(0, 15);                                           // 0行目から15行目まで部分的に更新。
            }
        } else {                                                                          // スレーブデバイスである場合。
#ifdef SPLIT_LAYER_STATE_ENABLE
           if (get_highest_layer(layer_state) != 0) {                                           // アクティブなレイヤが0でない場合、特定のレイヤ名を表示。
                oled_render_slave1(0);
                oled_render_slave2(6);
                oled_render_slave3(7);
            } else {                                                                      // それ以外の場合、アニメーションの描画。
                oled_clear_line(6);  // 行6をクリア
                oled_clear_line(7);  // 行7をクリア
                if (timer_elapsed(anim_frame_interval) > 300) {                           // 300msごとにアニメーションを更新。
                    anim_frame_interval = timer_read();                                   // アニメーションフレームのタイムスタンプを更新。
                    render_logo();                                                        // ロゴを描画。
                }
            }
#endif
        }
    }

    return false;                                                                         // QMKのOLEDタスクの規約によりfalseを返す。
}
