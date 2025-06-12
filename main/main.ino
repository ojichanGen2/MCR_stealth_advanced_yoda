#include "personal_setting.h"
/*
 * ログ関係
 * @note
 *	10ms間隔でログ記録
 *	100：1秒分 / 1000：5秒分
 */
#define LOG_BUFF_SIZE 400

/*
 *	データ保存関連
 */
volatile bool saveFlag = false; // ログ記録フラグ
volatile int16_t saveDataA[16][LOG_BUFF_SIZE];
// volatile int16_t saveDataB[16][LOG_BUFF_SIZE];
char filename[16];
volatile uint16_t logCt = 0;
volatile uint16_t logRd = 0;
volatile bool whichsave = false;
volatile bool isWriting = false;
volatile uint16_t log_pattern = 0;
volatile uint8_t motor_buff_Fl, motor_buff_Fr, motor_buff_Rl, motor_buff_Rr, motor_buff_stare;

/**********************************************************************/
/*
 * 型定義
 */
/**********************************************************************/
/*
 * プロトタイプ宣言
 */
void timerCallback(timer_callback_args_t __attribute((unused)) * p_args);

void initSens(void);
void initMotor(void);

unsigned char dipsw_get(void);
unsigned char pushsw_get(void);
void motor_r(int accele_l, int accele_r);
void motor2_r(int accele_l, int accele_r);
void motor_f(int accele_l, int accele_r);
void motor2_f(int accele_l, int accele_r);
// void motor_mode_r(int mode_l, int mode_r);  // 未使用
// void motor_mode_f(int mode_l, int mode_r);  // 未使用
void servoPwmOut(int pwm);
int check_crossline(void);
int check_rightline(void);
int check_leftline(void);

// int diff_fi(int pwm); // front_in内輪差を求める
// int diff_ri(int pwm); // diff_ri
// int diff_ro(int pwm); // diff_ri
// int diff(int pwm);

int getServoAngle(void);
int getAnalogSensor(void);
void servoControl(void);
void servoControl2(void);
void readDataFlashParameter(void);
void writeDataFlashParameter(void);
int lcdProcess(void);
int slopeCheck(void);

void courceOut(void);                        // コースハズレ特定関数
void mtTest(void);                           // モータテスト
int angleStreatCheck(int i, int jide_angle); // ブレーキ時のノイズ対策

// ブレーキ用
short Dig_M(short angle);
short Ang(void);
// void PDtrace_Control(short Dig, char boost_trig, short SP);
void PDtrace_Control(short Dig, short SP, char boost_trig = false);

// void StartTime(void);

// ログ関係
void SD_file_open(void);
void SD_file_close(void);
void writeLog(void);
void LOG_rec(void);

/**********************************************************************/
/*
 * 変数定義
 */

/*
 *	割り込み関係
 *	@note
 *		ログをSDカードに書きたいためここをメインとする！！
 */
static FspTimer interruptTimer;

/*
 *	ログ関係
 */
ArduinoSPI SPI(MISO1, MOSI1, SCK1, FORCE_SPI1_MODE); // RMC-RA4M1のmicroSD用SPIを選択(SPIの各端子はpins_arduino.hで定義)
char str[256];                                       // SDカードに書き込む用文字列とりあえず256Byte用意　　
File microSD;                                        // microSDのファイルアクセス用

/*
 *	analogセンサ用
 */
static mcr_ad ad;

// LED ON時のセンサ値
volatile uint16_t anaSensUR_on;
volatile uint16_t anaSensRR_on;
volatile uint16_t anaSensCR_on;
volatile uint16_t anaSensCC_on;
volatile uint16_t anaSensCL_on;
volatile uint16_t anaSensLL_on;
volatile uint16_t anaSensUL_on;
// LED OFF時のセンサ値
volatile uint16_t anaSensUR_off;
volatile uint16_t anaSensRR_off;
volatile uint16_t anaSensCR_off;
volatile uint16_t anaSensCC_off;
volatile uint16_t anaSensCL_off;
volatile uint16_t anaSensLL_off;
volatile uint16_t anaSensUL_off;
// LED ON/OFFの差分
volatile uint16_t anaSensUR_diff;
volatile uint16_t anaSensRR_diff;
volatile uint16_t anaSensCR_diff;
volatile uint16_t anaSensCC_diff;
volatile uint16_t anaSensCL_diff;
volatile uint16_t anaSensLL_diff;
volatile uint16_t anaSensUL_diff;

// 2値化用閾値
volatile uint16_t thrSensUR;
volatile uint16_t thrSensRR;
volatile uint16_t thrSensCR;
volatile uint16_t thrSensCC;
volatile uint16_t thrSensCL;
volatile uint16_t thrSensLL;
volatile uint16_t thrSensUL;
volatile uint16_t thrSensBK; // 黒閾値

// 2値化
volatile uint8_t digiSensUR;
volatile uint8_t digiSensRR;
volatile uint8_t digiSensCR;
volatile uint8_t digiSensCC;
volatile uint8_t digiSensCL;
volatile uint8_t digiSensLL;
volatile uint8_t digiSensUL;

/*
 *	R8Cプログラムから引用
 */
// const char *C_DATE = __DATE__; /* コンパイルした日付           */
// const char *C_TIME = __TIME__; /* コンパイルした時間           */

volatile int pattern = 0; // マイコンカー動作パターン

/*
 *	タイマカウント
 *	@note
 *		割り込み内でインクリメント
 */
volatile uint64_t cnt;
volatile uint64_t cnt1;          // タイマ用
volatile uint64_t cnt2;          // タイマ用
volatile uint64_t check_sen_cnt; // タイマ用
volatile uint64_t check_enc_cnt; // タイマ用
volatile uint64_t check_ana_cnt; // タイマ用
volatile uint64_t cnt_lcd;       // LCD処理で使用

/*
 *	走行モード・時間処理等
 */
volatile int8_t isBrakeOn;            // ブレーキ判定フラグ
volatile int8_t crankMode = 0;        // クランク判定   1:クランクモード 0:通常
volatile int8_t crankDirection = 'N'; // クランクの方向 R:右 L:左
volatile int8_t laneMode = 0;         // レーン判定
volatile int8_t laneDirection = 'N';  // レーンの方向 R:右 L:左
volatile int8_t slopeTotalCount = 0;  // 坂通過数（２度通過防止）

volatile long slopeFinTime = 0;  // 登坂後の安定待ち
volatile int laneClearTime = 0;  // レーン後のブレーキ防止
volatile int crankClearTime = 0; // クランク後のブレーキ防止
volatile int mtPower = 0;        // コーナ立ち上がり徐々に
volatile int temp;

volatile int8_t crank_count = 0;
volatile int8_t linellcount = 0;
volatile int8_t linerrcount = 0;

volatile int8_t sensLLon = OFF;
volatile int8_t sensRRon = OFF;

volatile bool START_flag = false;
volatile bool Run_end = false;

/*
 *	エンコーダ関連
 */
volatile int iTimer10;              // 10msカウント用
volatile long lEncoderTotal;        // 積算値保存用
volatile int iEncoder;              // 10ms毎の最新値
volatile unsigned int uEncoderBuff; // 計算用　割り込み内で使用
volatile long lEncoderBuff;         // エンコーダの値取得(距離制御用)

/*
 *	サーボ関連
 */
volatile int16_t iSensorBefore; // 前回のセンサ値保存
volatile int16_t iServoPwm;     // サーボＰＷＭ値
volatile int16_t iAngle0;       // 中心時のA/D値保存
volatile int16_t iAngle2;       // ステアリング角速度
volatile int16_t iAngleBuff;    // 計算用 割り込み内で使用

/*
 *	サーボ関連2
 */
volatile int16_t iSetAngle;
volatile int16_t iSetAngle3;
volatile int16_t iAngleBefore2;
volatile int16_t iAngleBefore3;
volatile int16_t iServoPwm2;
volatile int16_t iServoPwm3;
volatile int16_t cource = 0; // コースハズレ値

/*
 *	DataFlash関係
 */
uint8_t data_buff[16];

/*
 *	autoBreak関係ん関係
 */

// トレース用
volatile short Angle_D;    //
volatile short Angle_D_GF; //
// volatile char boost_flag = 0;
//  PD用変数
volatile char PD_trig = 0;
// ストップフラグ
// volatile char stop_D = 0;

/*
 *	LCD関連
 */
volatile uint8_t lcd_pattern = 1;

const int16_t speed_pulse[90] = {
    0, 4, 8, 12, 16, 21, 25, 29, 33, 37, 41,
    45, 49, 53, 57, 62, 66, 70, 74, 78, 82,
    86, 90, 94, 98, 103, 107, 111, 115, 119,
    123, 127, 131, 135, 139, 144, 148, 152,
    156, 160, 164, 168, 172, 176, 180, 185,
    189, 193, 197, 201, 205, 209, 213, 217,
    221, 226, 230, 234, 238, 242, 246, 250,
    254, 258, 262, 267, 271, 275, 279, 283,
    287, 291, 295, 299, 303, 308, 312, 316,
    320, 324, 328, 332, 336, 340, 344, 349,
    353, 357, 361, 365};

/************************************************************************/
/**
 * セットアップ(初期化).
 */
void setup()
{
    /* タイマ割り込み初期化 */
    // AGT 1msごとの割り込み処理の設定 PCLKB=24MHz ∴TIMER_SOURCE_DIV_1(1分周)なら、1/(24e6*1) * 24000 = 1ms  設定は１小さい値である23999を設定する
    // interruptTimer.begin(TIMER_MODE_PERIODIC, AGT_TIMER, 1, 23999, 1, (timer_source_div_t)TIMER_SOURCE_DIV_1, timerCallback);

    // AGTタイマーの設定: PCLKB = 24MHz, 分周 = 1, カウント値 = 5999（0.25ms）
    interruptTimer.begin(
        TIMER_MODE_PERIODIC,                    // 周期モード
        AGT_TIMER,                              // 使用するタイマー（例: AGT0, AGT1など）
        1,                                      // 優先度
        5999,                                   // カウント値（0.25msに対応）
        1,                                      // 割り込み番号
        (timer_source_div_t)TIMER_SOURCE_DIV_1, // 1分周
        timerCallback                           // 割り込み時に呼び出されるコールバック関数
    );

    IRQManager::getInstance().addPeripheral(IRQ_AGT, (void *)interruptTimer.get_cfg());
    interruptTimer.open();

    /* シリアル初期化 */
    Serial2.begin(115200);
    // Serial2.begin(38400);

    //  while(!Serial);

    /* プッシュスイッチ初期化 */
    pinMode(RUN_SWITCH, INPUT);

    /* dip-sw初期化 */
    // ディップスイッチ1
    pinMode(25, INPUT);
    // ディップスイッチ2
    pinMode(26, INPUT);

    /* CPUボードLED */
    // LED D2
    pinMode(23, OUTPUT);
    // LED D3
    pinMode(13, OUTPUT);
    // LED R
    pinMode(57, OUTPUT);
    // LED L
    pinMode(58, OUTPUT);
    // 赤外線LED
    pinMode(55, OUTPUT);

    /* モーター初期化 */
    initMotor();

    /* センサ初期化 */
    initSens();

    /* エンコーダ初期化 */
    // 1相エンコーダ GPT6 P401(D32端子)のGTETRGAを使用
    startGPT6_1SouEncoder(GTETRGA, 4, 1);

    /* スイッチ初期化 */
    SwitchInit();
    /* LCD初期化 */
    LcdInit();
    /* I2C初期化 */
    //	initI2CEeprom();
    // 走行パターン初期化
    pattern = 0;
    /* シリアル初期化 */
    Serial2.println("readyOK");
}

/************************************************************************/
/**
 * ループ(メイン処理).
 */

void loop()
{
    signed int i, ret;

    // DataFlashパラメータ読み込み
    readDataFlashParameter();

    // マイコンカーの状態初期化
    Serial2.print("start\n");
    motor_f(0, 0);
    motor_r(0, 0);
    servoPwmOut(0);

    // リセット動作確認
    for (i = 0; i < 5; i++)
    {
        CPU_LED_2 = ON;
        // L_LED = ON;
        // R_LED=ON;
        delay(100);

        CPU_LED_2 = OFF;
        // L_LED = OFF;
        // R_LED=OFF;
        delay(100);
    }

    while (1)
    {

        /* 途中で停止処理 */
        // プッシュSW操作判定
        // Serial2.print("lEncoderTotal=");
        // Serial2.print(lEncoderTotal);
        // Serial2.print("  pattern=");
        // Serial2.println(pattern);

        //			LcdPosition(0, 0);
        //			LcdPrintf("main_loop");
        //			LcdPosition(0, 1);
        //			LcdPrintf("pattern = %d ", pattern);
        //		I2CEepromProcess(); /* I2C EEP-ROM保存処理          */

        //		while(1) {
        //			printf("lane_count=%d  linerrcount=%d\n",lane_count,linerrcount);
        //			printf("check_leftline()=%d  check_rightline()=%d\n",check_leftline(),check_rightline());

        if (digiSensLL == ON)
        {
            linellcount++;
            if (linellcount > 40) // 30
            {
                sensLLon = ON;
                // CPU_LED_2 = ON;
            }
            // sensLLon = ON;
        }
        else
        {
            linellcount = 0;
            sensLLon = OFF;
        }

        if (digiSensRR == ON)
        {
            linerrcount++;
            if (linerrcount > 40)
            {
                sensRRon = ON;
                // CPU_LED_2 = ON;
            }
            // sensRRon = ON;
        }
        else
        {
            linerrcount = 0;
            sensRRon = OFF;
        }

        if (pattern >= 11 && pattern <= 50)
        {
            // クロスラインチェック
            if (check_crossline())
            {
                cnt1 = 0;
                crankMode = 1;
                pattern = 101;
                lEncoderBuff = lEncoderTotal;
            }

            // 左ハーフラインチェック
            if (check_leftline() && abs(getServoAngle()) < 25) // 15
            {
                cnt1 = 0;
                laneMode = 1;
                pattern = 151;
                laneDirection = 'L';
                lEncoderBuff = lEncoderTotal;
            }
            // 右ハーフラインチェック
            if (check_rightline() && abs(getServoAngle()) < 25) // 15
            {
                cnt1 = 0;
                laneMode = 1;
                pattern = 151;
                laneDirection = 'R';
                lEncoderBuff = lEncoderTotal;
            }
            // 登坂検出
            // if (slopeCheck() == 1 && slopeTotalCount == 0)
            // {
            //   // 坂走行処理へ	のぼるくん
            //   pattern = 191;
            // }
        }

        if (logCt != logRd && !Run_end)
        {
            writeLog(); // ログデータの書き込み
        }

        switch (pattern)
        {
        case 0:
            // pattern = 2;
            pattern = 1;
            SD_file_open();
            cnt1 = 0;
            break;

            /*
             * プッシュスイッチ押下待ち
             */
        case 1:
            servoPwmOut(0);
            // LCD表示、パラメータ設定処理
            lcdProcess();

            if (pushsw_get())
            {
                // clearI2CEeprom();

                //   パラメータ保存
                writeDataFlashParameter();
                Serial.print("writeDataFlashParameter");
                cnt1 = 0;
                iAngle0 = VR_CENTER; // センター値固定
                // iAngle0 = getServoAngle(); // 0度の位置記憶
                pattern = 2; // ゲートセンサ無し （手押し）
                // pattern = 3; // ゲートセンサ有り
                break;
            }
            break;

        // キャリブレーション
        case 2:
            static int sensorMax = anaSensCC_diff; // センサの最大値
            static int sensorMin = anaSensCC_diff; // センサの最小値

            static int sensorMaxRR = anaSensRR_diff; // RRの最大値
            static int sensorMinRR = anaSensRR_diff; // RRの最小値
            static int sensorMaxLL = anaSensLL_diff; // LLの最大値
            static int sensorMinLL = anaSensLL_diff; // LLの最小値

            static int sensorMaxUR = anaSensUR_diff; // URの最大値
            static int sensorMinUR = anaSensUR_diff; // URの最小値
            static int sensorMaxUL = anaSensUL_diff; // ULの最小値
            static int sensorMinUL = anaSensUL_diff; // ULの最小値

            static int angle = 0;      // 現在のサーボ角度
            static int Max_angle = 65; // 最大のサーボ角度
            static int step = 1;       // サーボの移動
            static int cycleCount = 0; // 首を振った回数

            static float Judg_percent = 0.4;    // 閾値パーセント  0.4
            static float Judg_BK_percent = 0.2; // 閾値パーセント

            // iAngle0 = getServoAngle(); /* 0度の位置記憶 */
            // iAngle0 = VR_CENTER; // センター値固定
            iSetAngle = angle;
            // iSetAngle = 40;
            servoPwmOut(iServoPwm2);

            if (cycleCount < 4)
            {
                if (cnt1 >= 5)
                {
                    cnt1 = 0;

                    // int sensorValue = (anaSensCR_diff + anaSensCL_diff) / 2; // 値取得
                    int sensorValue = anaSensCC_diff; // 値取得
                    if (sensorValue > sensorMax)
                    {
                        sensorMax = sensorValue;
                    }
                    if (sensorValue < sensorMin)
                    {
                        sensorMin = sensorValue;
                    }

                    if (angle > 0)
                    {
                        // RR値取得
                        int sensorValueRR = anaSensRR_diff;
                        if (sensorValueRR > sensorMaxRR)
                        {
                            sensorMaxRR = sensorValueRR;
                        }
                        else if (sensorValueRR < sensorMinRR)
                        {
                            sensorMinRR = sensorValueRR;
                        }

                        int sensorValueUR = anaSensUR_diff;
                        if (sensorValueUR > sensorMaxUR)
                        {
                            sensorMaxUR = sensorValueUR;
                        }
                        else if (sensorValueUR < sensorMinUR)
                        {
                            sensorMinUR = sensorValueUR;
                        }
                    }
                    else
                    {
                        // LL値取得
                        int sensorValueLL = anaSensLL_diff;
                        if (sensorValueLL > sensorMaxLL)
                        {
                            sensorMaxLL = sensorValueLL;
                        }
                        else if (sensorValueLL < sensorMinLL)
                        {
                            sensorMinLL = sensorValueLL;
                        }

                        int sensorValueUL = anaSensUL_diff;
                        if (sensorValueUL > sensorMaxUL)
                        {
                            sensorMaxUL = sensorValueUL;
                        }
                        else if (sensorValueUL < sensorMinUL)
                        {
                            sensorMinUL = sensorValueUL;
                        }
                    }
                    // サーボ角度更新
                    angle += step;

                    if (angle > Max_angle || angle < -Max_angle)
                    {
                        step = -step; // 方向転換
                        cycleCount++;
                    }
                }
            }
            else
            { // 片方向2回ずつ振る計4回
                // 閾値計算
                int threshold = (sensorMax - sensorMin) * Judg_percent;                                             // CC閾値
                int thresholdRL = (((sensorMaxRR - sensorMinRR) + (sensorMaxLL - sensorMinLL)) / 2) * Judg_percent; // RR,LL閾値
                // int thresholdRR = (sensorMaxRR - sensorMinRR) * Judg_percent;                                      // RR閾値
                // int thresholdLL = (sensorMaxLL - sensorMinLL) * Judg_percent;                                      // LL閾値
                int thresholdU = (((sensorMaxUR - sensorMinUR) + (sensorMaxUL - sensorMinUL)) / 2) * Judg_percent; // UR,UL閾値
                // int thresholdUL = (sensorMaxUL - sensorMinUL) * Judg_percent; // UL閾値
                int threshold_BK = (sensorMax - sensorMin) * Judg_BK_percent; // UR,UL閾値

                thrSensUR = thresholdU;
                thrSensRR = thresholdRL;
                thrSensCR = threshold;
                thrSensCC = threshold;
                thrSensCL = threshold;
                thrSensLL = thresholdRL;
                thrSensUL = thresholdU;
                thrSensBK = threshold_BK;

                angle = 0;

                if ((getServoAngle() == 0))
                {
                    pattern = 4; // 手押し 4
                    // pattern = 3; // STARTbar
                    // pattern = 1;
                    START_flag = false;
                    cnt1 = 0;
                    cnt2 = 0;
                    // 初期化
                    // sensorMax = INT_MIN;
                    // sensorMin = INT_MAX;
                    step = 0;
                    // cycleCount = 0;
                }
            }

            break;

            /*
             * スタートバー開待ち
             */
        case 3:
            servoPwmOut(iServoPwm / 2);
            if (pushsw_get() && cnt2 > 300)
            {
                START_flag = true;
            }

            // if ((anaSensRR_diff > 10000) && START_flag)
            if (digiSensRR == ON && START_flag)
            {
                // iAngle0 = getServoAngle(); /* 0度の位置記憶 */

                // iAngle0 = VR_CENTER; // センター値固定
                CPU_LED_2 = OFF;
                cnt1 = 0;
                // saveIndex = 0;
                // saveFlag = true; //pattern :5 で行う　/* データ保存開始               */
                check_sen_cnt = 0;
                check_enc_cnt = 0;
                check_ana_cnt = 0;
                // cnt1 = 0;
                pattern = 5;
                break;
            }
            else
            {
                if (cnt1 < 300)
                {
                    CPU_LED_2 = ON;
                    R_LED = ON;
                    L_LED = OFF;
                }
                else
                {
                    CPU_LED_2 = OFF;
                    R_LED = OFF;
                    L_LED = ON;
                    if (cnt1 > 600)
                    {
                        cnt1 = 0;
                    }
                }
            }
            break;

        /*
         * スタートSW待ち
         */
        case 4:
            servoPwmOut(iServoPwm / 2);
            // lcdProcess();
            if (pushsw_get() == ON && cnt2 > 300)
            {
                // iAngle0 = getServoAngle(); /* 0度の位置記憶 */
                // iAngle0 = VR_CENTER; // センター値固定
                CPU_LED_2 = OFF;
                cnt1 = 0;
                // saveIndex = 0;
                // saveFlag = true; //pattern :5 で行う　/* データ保存開始               */
                check_sen_cnt = 0;
                check_enc_cnt = 0;
                check_ana_cnt = 0;
                pattern = 5;
                delay(100);
                while (pushsw_get() == ON)
                {
                    if (cnt1 < 100)
                    {
                        CPU_LED_2 = ON;
                        R_LED = ON;
                        L_LED = OFF;
                    }
                    else
                    {
                        CPU_LED_2 = OFF;
                        R_LED = OFF;
                        L_LED = ON;
                        if (cnt1 > 200)
                        {
                            cnt1 = 0;
                        }
                    }
                }
            }
            if (cnt1 < 300)
            {
                CPU_LED_2 = ON;
                R_LED = ON;
                L_LED = OFF;
            }
            else
            {
                CPU_LED_2 = OFF;
                R_LED = OFF;
                L_LED = ON;
                if (cnt1 > 600)
                {
                    cnt1 = 0;
                }
            }
            break;

        case 5:
            if (cnt2 < 100)
            {
                CPU_LED_2 = ON;
                R_LED = ON;
                L_LED = OFF;
            }
            else
            {
                CPU_LED_2 = OFF;
                R_LED = OFF;
                L_LED = ON;
                if (cnt2 > 200)
                {
                    cnt2 = 0;
                }
            }

            if (cnt1 > data_buff[START_TIME_ADDR] * 1000)
            {
                LcdPosition(0, 0);
                LcdPrintf("case 11");
                // iAngle0 = getServoAngle(); /* 0度の位置記憶 */
                pattern = 11;
                cnt1 = 0;
                // saveIndex = 0;
                saveFlag = true; /* データ保存開始               */
                check_sen_cnt = 0;
                check_enc_cnt = 0;
            }
            servoPwmOut(iServoPwm / 2); // ライントレース制御
            break;

            /*
             * 通常走行処理
             */
        case 11:

            /* 通常トレース */
            servoPwmOut(iServoPwm); // ライントレース制御
            i = getServoAngle();    // ステアリング角度取得
                                    // 走行終了処理(角度)
                                    // if (abs(i) > 55) {
                                    //   if (cnt1 > 3000) {
                                    //     stop_D = 1;
                                    //   }
                                    // } else {
                                    //   cnt1 = 0;
                                    // }

            if (Angle_D > 0)
            {
                Angle_D_GF = 0;
            }
            else
            {
                Angle_D_GF = Angle_D;
            }

            // モーター制御
            if ((abs(i) > 110))
            {
                PDtrace_Control(i, (data_buff[CORNER_SPEED_ADDR] * 50 / 100) + Angle_D_GF);
            }
            else if ((abs(i) > 80))
            {
                PDtrace_Control(i, (data_buff[CORNER_SPEED_ADDR] * 65 / 100) + Angle_D_GF);
            }
            else if ((abs(i) > 68))
            {
                PDtrace_Control(i, (data_buff[CORNER_SPEED_ADDR] * 80 / 100) + Angle_D_GF);
            }
            else if ((abs(i) > 47))
            {
                PDtrace_Control(i, (data_buff[CORNER_SPEED_ADDR] * 88 / 100) + Angle_D_GF);
            }
            else if ((abs(i) > 13))
            {
                PDtrace_Control(i, (data_buff[CORNER_SPEED_ADDR] * 95 / 100) + Angle_D_GF);
            }
            else if ((abs(i) > 8))
            {
                PDtrace_Control(i, (data_buff[CORNER_SPEED_ADDR]) + Angle_D_GF);
            }
            else
            {
                PDtrace_Control(i, data_buff[TRG_SPEED_ADDR]);
            }
            break;

            /*
             * クランク走行処理
             */
        case 101:
            /* クロスライン通過処理 */
            servoPwmOut(iServoPwm);
            R_LED = ON;
            L_LED = ON;
            // motor_f(-85, -85);
            // motor_r(-80, -80);
            PDtrace_Control(i, data_buff[CRANK_SPEED_ADDR]);
            // if (cnt1 >= 50)
            if (lEncoderTotal - lEncoderBuff >= 350) // 350
            {                                        // 誤読み防止(225mm)
                cnt1 = 0;
                pattern = 102;
                laneMode = 0; // レーンモードフラグクリア
                break;
            }
            break;

        case 102:
            lEncoderBuff = lEncoderTotal;
            pattern = 106; // 106でもいい？
            break;

        case 104: // クロスライン後の処理(1段目の減速処理)
            servoPwmOut(iServoPwm);
            PDtrace_Control(i, data_buff[CRANK_SPEED_ADDR]);
            /*
            if (iEncoder >= data_buff[CRANK_SPEED_ADDR] + 2)
            {                    // エンコーダによりスピード制御
              motor_f(-45, -45); //-25
              motor_r(-65, -65); //-50
            }
            else if (iEncoder >= data_buff[CRANK_SPEED_ADDR])
            {
              motor_f(25, 25);
              motor_r(25, 25);
            }
            else
            {
              motor_f(60, 60);
              motor_r(60, 60);
            }
              */

            if (lEncoderTotal - lEncoderBuff >= 200) // 200
            {
                // 200m
                pattern = 106;
                break;
            }
            break;

        case 106: // クランク処理 (2段目の減速処理)　ハーフライン検出
            servoPwmOut(iServoPwm);
            PDtrace_Control(i, data_buff[CRANK_SPEED_ADDR]);
            /*
            if (iEncoder >= data_buff[CRANK_SPEED_ADDR] + 2)
            { // エンコーダによりスピード制御
              motor_f(-20, -20);
              motor_r(-20, -20);
            }
            else if (iEncoder >= data_buff[CRANK_SPEED_ADDR])
            {
              motor_f(10, 10);
              motor_r(10, 10);
            }
            else
            {
              motor_f(60, 60);
              motor_r(60, 60);
            }
            */
            if (sensLLon == ON)
            // if (digiSensLL == ON)
            {                              // クランク方向　左
                crankDirection = 'L';      // クランク方向記憶変数＝左クランク
                iSetAngle = CRANK_ANGLE_L; /* +で左 -で右に曲がります      */
                servoPwmOut(iServoPwm2);
                pattern = 108;
                break;
            }
            else if (sensRRon == ON)
            // else if (digiSensRR == ON)
            {                               // クランク方向　右
                crankDirection = 'R';       // クランク方向記憶変数＝左クランク
                iSetAngle = -CRANK_ANGLE_R; /* +で左 -で右に曲がります      */
                servoPwmOut(iServoPwm2);
                lEncoderBuff = lEncoderTotal;
                pattern = 108;
                break;
            }
            break;

        case 108: // クランク処理	 　ハーフライン検出後
            if (crankDirection == 'L')
            {                              // クランク方向　左
                iSetAngle = CRANK_ANGLE_L; /* +で左 -で右に曲がります      */
                servoPwmOut(iServoPwm2);

                motor_f(-70, -1); // 前 （左,右）
                motor_r(-40, -1); // 後モータ（左,右）
            }

            else if (crankDirection == 'R')
            {                               // クランク方向　右
                iSetAngle = -CRANK_ANGLE_R; /* +で左 -で右に曲がります      */
                servoPwmOut(iServoPwm2);

                motor_f(-1, -70); // 前 （左,右）
                motor_r(-1, -40); // 後モータ（左,右）
            }

            if ((digiSensCC == OFF && digiSensLL == OFF && digiSensRR == OFF) || (lEncoderTotal - lEncoderBuff) >= 75) // 75
            // if ((digiSensCC == OFF && digiSensCL == OFF && digiSensCR == OFF) || (lEncoderTotal - lEncoderBuff) >= 75) // 75
            // {
            {
                pattern = 110; // ﾌﾛﾝﾄのﾃﾞｼﾞﾀﾙｾﾝｻ全て黒の時次の処理へ
                lEncoderBuff = lEncoderTotal;
                cnt1 = 0;
                CPU_LED_2 = ON;
                break;
            }
            break;

        case 110: // クランク処理	 　ハーフライン検出後
            if (crankDirection == 'L')
            {                              // クランク方向　左
                iSetAngle = CRANK_ANGLE_L; /* +で左 -で右に曲がります      */
                servoPwmOut(iServoPwm2);

                motor_f(-70, -1); // 前 （左,右）
                motor_r(-40, -1); // 後モータ（左,右）

                if (sensRRon == ON)
                {
                    pattern = 112;    //(通常クランクの処理へ)　ﾌﾛﾝﾄ右ｾﾝｻ(out)反応時
                    cnt1 = 0;         // 116:20ms待ち
                    motor_f(-20, 1);  // 前 （左,右）1,1
                    motor_r(-50, 20); // 後モータ（左,右）
                    break;
                }

                if (sensLLon == ON && anaSensLL_diff < thrSensBK && (lEncoderTotal - lEncoderBuff) >= 100) // 75
                {
                    pattern = 131; //(低速進入時処理)　ﾌﾛﾝﾄ左ｾﾝｻ(in)反応時
                    cnt1 = 0;
                    break;
                    // （左,右） motor_f(1,1);
                    // //前 （左,右）
                    // motor_r(-40,1);
                }
            }
            else if (crankDirection == 'R')
            {                               // クランク方向　右
                iSetAngle = -CRANK_ANGLE_R; /* +で左 -で右に曲がります      */
                servoPwmOut(iServoPwm2);

                motor_f(-1, -70); // 前 （左,右）
                motor_r(-1, -40); // 後モータ（左,右）

                if (sensLLon == ON)
                {
                    pattern = 112;    //(通常クランクの処理へ) ﾌﾛﾝﾄ左ｾﾝｻ(out)反応時
                    motor_f(1, -20);  // 前 （左,右）1,1
                    motor_r(20, -50); // 後モータ（左,右）
                    cnt1 = 0;         // 116:20ms待ち
                    break;
                }

                if (sensRRon == ON && /*anaSensCR_diff < thrSensBK &&*/ (lEncoderTotal - lEncoderBuff) >= 100) // 要件等 75
                {
                    pattern = 131; //(低速進入時処理)　ﾌﾛﾝﾄ右ｾﾝｻ(in)反応時
                    cnt1 = 0;
                    break;
                }
            }
            break;

        case 112:
            if (crankDirection == 'L')
            { // クランク方向　左

                iSetAngle = CRANK_ANGLE_L; /* +で左 -で右に曲がります      */
                servoPwmOut(iServoPwm2);

                //					motor_f(1,1);
                ////前
                // （左,右）
                // motor_r(-40,1);
                // //後モータ（左,右） 			motor_f(-50,1);
                // //前 （左,右）
                // motor_r(-60,30);
                // //後モータ（左,右）

                motor_f(1, 1);    // 前 （左,右）
                motor_r(-50, 10); // 後モータ（左,右）

                // 通常クランク処理
                // ﾌﾛﾝﾄのﾃﾞｼﾞﾀﾙｾﾝｻ全て黒（右センサＯＦＦ）の時次の処理へ　　（センサー拡張のため　左センサー反応時を除く）
                if (digiSensCC == OFF && /*anaSensCR_diff < thrSensCR &&*/ digiSensRR == OFF && /*anaSensCL_diff < thrSensCL && */ cnt1 > 20)
                {
                    pattern = 116; // 114
                }
                // コースアウト対応クランク処理(←対応しない)
            }
            else if (crankDirection == 'R')
            { // クランク方向　右

                iSetAngle = -CRANK_ANGLE_R; /* +で左 -で右に曲がります      */
                servoPwmOut(iServoPwm2);

                //					motor_f(1,a1);
                ////前
                // （左,右）
                // motor_r(1,-40);
                // //後モータ（左,右）
                motor_f(1, 1);    // 前 （左,右）
                motor_r(10, -50); // 後モータ（左,右）

                // 通常クランク処理
                // ﾌﾛﾝﾄのﾃﾞｼﾞﾀﾙｾﾝｻ全て黒（左センサＯＦＦ）の時次の処理へ　（センサー拡張のため　左センサー反応時を除く）
                if (digiSensCC == OFF && /*anaSensCR_diff < thrSensCR &&*/ digiSensLL == OFF /*&& anaSensCL_diff < thrSensCL*/ && cnt1 > 20)
                {
                    pattern = 116; // 114
                }
                // コースアウト対応クランク処理(←対応しない)
            }
            break;

        case 114: // コースアウト時のクランク処理
            if (crankDirection == 'L')
            {                              // クランク方向　左
                iSetAngle = CRANK_ANGLE_L; /* +で左 -で右に曲がります      */
                servoPwmOut(iServoPwm2);
                //					motor_f(-70,1);
                ////前 （左,右）
                motor_f(1, 20); // 前 （左,右）1,20
                motor_r(1, 60); // 後モータ（左,右）motor_r(-40, 1);
                // ﾌﾛﾝﾄのﾃﾞｼﾞﾀﾙｾﾝｻ全て黒（右センサＯＦＦ）の時次の処理へ　　（センサー拡張のため　左センサー反応時を除く）
                if (digiSensCC == OFF && /*anaSensCR_diff < thrSensCR &&*/ digiSensRR == OFF /*&& anaSensCL_diff < thrSensCL*/)
                {
                    pattern = 116;
                    break;
                }
            }

            else if (crankDirection == 'R')
            {                               // クランク方向　右
                iSetAngle = -CRANK_ANGLE_R; /* +で左 -で右に曲がります      */
                servoPwmOut(iServoPwm2);
                //					motor_f(1,-70);
                ////前 （左,右）
                motor_f(20, 1); // 前 （左,右）20,1
                motor_r(60, 1); // 後モータ（左,右）
                // ﾌﾛﾝﾄのﾃﾞｼﾞﾀﾙｾﾝｻ全て黒（左センサＯＦＦ）の時次の処理へ　　（センサー拡張のため　右センサー反応時を除く）
                if (digiSensCC == OFF && /* anaSensCR_diff < thrSensCR &&*/ digiSensLL == OFF /*&& anaSensCL_diff < thrSensCL*/)
                {
                    pattern = 116;
                    break;
                }
            }
            break;

        case 116: // クランク処理
            if (crankDirection == 'L')
            {                              // クランク方向　左
                iSetAngle = CRANK_ANGLE_L; /* +で左 -で右に曲がります      */
                servoPwmOut(iServoPwm2);
                motor_f(100, 70); // 前 （左,右)(100,70) (30, 0)
                motor_r(50, 60);  // 後モータ(左,右)

                if (sensLLon == ON /*&& anaSensCL_diff < thrSensCL*/)
                {
                    pattern = 118; // ﾌﾛﾝﾄ左ｾﾝｻ（ﾃﾞｼﾞﾀﾙまたはｱﾅﾛｸﾞ）反応時次の処理へ
                    cnt1 = 0;
                    break;
                }
                /* ステアリング角 0:中央 +:左 -:右 1.6=1度 */
                //					ST_A = 0;
                //					ST_B = 1;
                //					ST_PWM=10;
            }
            else if (crankDirection == 'R')
            { // クランク方向　右
                /* ステアリング角 0:中央 +:左 -:右 1.6=1度 */
                //					ST_A = 1;
                //					ST_B = 0;
                //					ST_PWM=10;
                // ST_PWM=0;

                iSetAngle = -CRANK_ANGLE_R; /* +で左 -で右に曲がります      */
                servoPwmOut(iServoPwm2);
                motor_f(70, 100); // 前 （左,右）(70,100) (0, 30)
                motor_r(60, 50);  // 後モータ（左,右)

                if (sensRRon == ON /*&& anaSensCR_diff < thrSensCR*/)
                {
                    pattern =
                        118; // ﾌﾛﾝﾄ左ｾﾝｻ（ﾃﾞｼﾞﾀﾙまたはｱﾅﾛｸﾞ）反応時次の処理へ
                    cnt1 = 0;
                }
            }
            break;

        case 118:
            if (crankDirection == 'L')
            { // クランク方向　左
                //					ST_A = 0;
                //					ST_B = 1;
                //					ST_PWM=0;
                iSetAngle = CRANK_ANGLE_L / 2 + 20; /* +で左 -で右に曲がります      */
                servoPwmOut(iServoPwm2);
                motor_f(100, 70); // 前 （左,右）(100,70)
                motor_r(60, 50);  // 後モータ（左,右）　
            }
            else if (crankDirection == 'R')
            { // クランク方向　右
                // カウンターステア
                //					ST_A = 1;
                //					ST_B = 0;
                //					ST_PWM=0;
                // ST_PWM=0;

                iSetAngle = -(CRANK_ANGLE_R / 2 + 20); /* +で左 -で右に曲がります      */
                servoPwmOut(iServoPwm2);
                motor_f(70, 100); // 前 （左,右）(70,100)
                motor_r(50, 60);  // 後モータ（左,右）
            }
            if (cnt1 > 10 && digiSensCC == ON)
            { // 10ms後
                pattern = 120;
                crankMode = 0; // クランクモードクリア
                cource = 0;    // コース外れ値0クリア
            }
            break;

        case 120:
            /* 少し時間が経つまで待つ */
            i = getServoAngle(); // ステアリング角度取得
            servoPwmOut(iServoPwm);
            motor_r(100, 100);
            motor_f(100, 100);
            if (abs(i) < 5)
            {
                cnt1 = 0;
                pattern = 11;
                crankDirection = 0; // クランクモード（クランク方向）クリア
                laneMode = 0;       // レーンモードクリア
                laneDirection = 0;  // レーンモード（レーン方向）クリア
                crankClearTime = 50;
                break;
            }

            /* if(check_leftline()){
            crankDirection = 0;  // クランクモード（クランク方向）クリア
            laneMode = 0;  // レーンモードクリア
            laneDirection = 0;  // レーンモード（レーン方向）クリア
            crankClearTime=50;
            cnt1 = 0;
            laneMode = 1;
            pattern = 151;
            laneDirection = 'L';
            break;
          }
          if(check_rightline()){
            crankDirection = 0;  // クランクモード（クランク方向）クリア
            laneMode = 0;  // レーンモードクリア
            laneDirection = 0;  // レーンモード（レーン方向）クリア
            crankClearTime=50;
            cnt1 = 0;
            laneMode = 1;
            pattern = 151;
            laneDirection = 'R';
            break;
          }
          */
            break;

        // 低速進入時のクランクの処理
        case 131:
            if (crankDirection == 'L')
            {                              // クランク方向　左
                iSetAngle = CRANK_ANGLE_L; /* +で左 -で右に曲がります      */
                servoPwmOut(iServoPwm2);
                motor_f(-10, 60); // 前 （左,右)(100,70)
                motor_r(-10, 50); // 後モータ(左,右)
            }
            else if (crankDirection == 'R')
            {                               // クランク方向　右
                iSetAngle = -CRANK_ANGLE_R; /* +で左 -で右に曲がります      */
                servoPwmOut(iServoPwm2);
                motor_f(60, -10); // 前 （左,右）(70,100)
                motor_r(50, -10); // 後モータ（左,右)
            }
            //        if (cnt1 > 10 && digiSensCC == ON && abs(ANA_SENS_L-ANA_SENS_R)<100)  {
            if (cnt1 > 10 && digiSensCC == ON)
            {

                crankMode = 0; // クランクモードクリア(ライントレースモード復活のため)
                pattern = 132;
                cource = 0; // コース外れ値0クリア
            }
            break;

        case 132:
            /* 少し時間が経つまで待つ */
            i = getServoAngle(); // ステアリング角度取得
            servoPwmOut(iServoPwm);
            motor_r(100, 100);
            motor_f(100, 100);
            if (abs(i) < 5)
            {
                cnt1 = 0;
                pattern = 11;
                crankDirection = 0; // クランクモード（クランク方向）クリア
                laneMode = 0;       // レーンモードクリア
                laneDirection = 0;  // レーンモード（レーン方向）クリア
                crankClearTime = 50;
                break;
            }
            /*if(check_leftline()){
            crankDirection = 0;  // クランクモード（クランク方向）クリア
            laneMode = 0;  // レーンモードクリア
            laneDirection = 0;  // レーンモード（レーン方向）クリア
            crankClearTime=50;
            cnt1 = 0;
            laneMode = 1;
            pattern = 151;
            laneDirection = 'L';
            break;
          }
          if(check_rightline()){
            crankDirection = 0;  // クランクモード（クランク方向）クリア
            laneMode = 0;  // レーンモードクリア
            laneDirection = 0;  // レーンモード（レーン方向）クリア
            crankClearTime=50;
            cnt1 = 0;
            laneMode = 1;
            pattern = 151;
            laneDirection = 'R';
            break;
          }
          */
            break;

            /************************************************************************/
            /* レーンチェンジの処理*/
            /************************************************************************/
        case 151:                   // ハーフライン後の処理１（速度制御）
            servoPwmOut(iServoPwm); // ライントレース制御 motor_r(80, 80);
            // motor_f(60, 60);
            // motor_r(60, 60);
            PDtrace_Control(i, data_buff[LANE_SPEED_ADDR]);
            crankMode = 1; // ステアリング制御補正なし

            if (check_crossline())
            { /* クロスラインチェック         */
                cnt1 = 0;
                crankMode = 1;
                lEncoderBuff = lEncoderTotal;
                pattern = 101;
                break;
            }

            // if (cnt1 > 10)
            if (lEncoderTotal - lEncoderBuff > 60)
            { // 50mm
                lEncoderBuff = lEncoderTotal;
                pattern = 152;
                break;
            }
            break;

        case 152:                // クロスライン後の処理(白線トレース時)
            i = getServoAngle(); // ステアリング角度取得
            servoPwmOut(iServoPwm);
            PDtrace_Control(i, data_buff[LANE_SPEED_ADDR]);
            /*
                  if (iEncoder >= data_buff[LANE_SPEED_ADDR] + 3)
                  { // エンコーダによりスピード制御
                    motor_f(-20, -20);
                    motor_r(-30, -30);
                  }
                  else if (iEncoder >= data_buff[LANE_SPEED_ADDR])
                  {
                    motor_f(50, 50);
                    motor_r(50, 50);
                  }
                  else
                  {
                    motor_f(80, 80);
                    motor_r(80, 80);
                  }
            */
            // if (digiSensRR == OFF && digiSensLL == OFF && digiSensCC == OFF && anaSensCL_diff < thrSensCL && anaSensCR_diff < thrSensCR)
            if (digiSensCL == OFF && digiSensCC == OFF && digiSensCR == OFF)
            {
                pattern = 154; // 全てのセンサ　黒検出時次の処理へ
            }

            // レーン誤検知用の通常復帰
            if (lEncoderTotal - lEncoderBuff >= 3000)
            {                 // 2000m
                pattern = 11; // 通常に戻す
                break;
            }

            /* クロスラインチェック         */
            if (check_crossline())
            {
                cnt1 = 0;
                crankMode = 1;
                pattern = 101;
                break;
            }
            break;

        case 154:                // 白線トレース終了後処理	最外センサ　白反応待ち
            i = getServoAngle(); // ステアリング角度取得

            if (laneDirection == 'L')
            { // レーン方向　左

                iSetAngle = LANE_ANGLE_L; /* +で左 -で右に曲がります */
                servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                motor_f(-70, 25);         // 前 （左,右-70）
                motor_r(-30, 10);         // 後（左,右-30)

                if ((LANE_ANGLE_L)-5 < abs(i))
                {
                    pattern = 155; // ステアリング角度目標値付近
                    break;
                }

                if (digiSensCL == ON)
                {
                    cnt1 = 0;
                    pattern = 156;
                }
            }
            else if (laneDirection == 'R')
            { // レーン方向　右
                iSetAngle = -LANE_ANGLE_R;

                /* +で左 -で右に曲がります */
                servoPwmOut(iServoPwm2); // 2角度制御 3:割込制御無
                motor_f(25, -70);        // 前 （左,右-70）
                motor_r(10, -30);        // 後（左,右-30)

                if ((LANE_ANGLE_R)-5 < abs(i))
                {
                    pattern = 155; // ステアリング角度目標値付近
                    break;
                }

                if (digiSensCR == ON)
                {
                    cnt1 = 0;
                    pattern = 156;
                }
            }
            break;

        case 155:                // 白線トレース終了後処理	最外センサ　白反応待ち
            i = getServoAngle(); // ステアリング角度取得

            if (laneDirection == 'L')
            { // レーン方向　左

                iSetAngle = LANE_ANGLE_L; /* +で左 -で右に曲がります */
                servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                motor_f(70, 70);          // 前 （左,右-70）
                motor_r(1, 1);            // 後（左,右-30)

                if (digiSensCL == ON)
                {
                    cnt1 = 0;
                    pattern = 156;
                }
            }
            else if (laneDirection == 'R')
            {                              // レーン方向　右
                iSetAngle = -LANE_ANGLE_R; /* +で左 -で右に曲がります */
                servoPwmOut(iServoPwm2);   // 2角度制御 3:割込制御無
                motor_f(70, 70);           // 前 （左,右-70）
                motor_r(1, 1);             // 後（左,右-30)

                if (digiSensCR == ON)
                {
                    cnt1 = 0;
                    pattern = 156;
                }
            }
            break;

        case 156: // 白線トレース終了後処理	最外センサ　白反応待ち
            if (laneDirection == 'L')
            {                             // レーン方向　左
                iSetAngle = LANE_ANGLE_L; /* +で左 -で右に曲がります */
                servoPwmOut(iServoPwm2);  // 角度制御
                motor_f(85, 0);           // 前 （左,右）
                motor_r(85, 0);           // 後（左,右)
            }

            else if (laneDirection == 'R')
            {                              // レーン方向　右
                iSetAngle = -LANE_ANGLE_L; /* +で左 -で右に曲がります */
                servoPwmOut(iServoPwm2);   // 角度制御
                motor_f(0, 85);            // 前 （左,右）
                motor_r(0, 85);            // 後（左,右)
            }

            if (cnt1 >= 5)
            {
                pattern = 160; // 5ms後次の処理へ
                cnt1 = 0;
            }
            break;

        case 160: // 10m秒待ち後の処理　最内センサ　白反応時待ち
            if (laneDirection == 'L')
            { // レーン方向　左

                iSetAngle = LANE_ANGLE_L; /* +で左 -で右に曲がります */
                servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                motor_f(85, 0);           // 前 （左,右）
                motor_r(85, 0);           // 後（左,右)

                if (sensRRon == ON)
                {
                    cnt1 = 0;
                    pattern = 162; /*左デジタルセンサ反応時次の処理へ */
                }
            }
            else if (laneDirection == 'R')
            {                              // レーン方向　右
                iSetAngle = -LANE_ANGLE_L; /* +で左 -で右に曲がります */
                servoPwmOut(iServoPwm2);   // 2角度制御 3:割込制御無
                motor_f(0, 85);            // 前 （左,右）
                motor_r(0, 85);            // 後（左,右)

                if (sensLLon == ON)
                {
                    cnt1 = 0;
                    pattern = 162; /*左デジタルセンサ反応時次の処理へ */
                }
            }
            break;

        case 162: // 最内センサ反応後　10ms待ち
            if (laneDirection == 'L')
            {                             // レーン方向　左
                iSetAngle = LANE_ANGLE_L; /* +で左 -で右に曲がります */
                servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                motor_f(85, 0);           // 前 （左,右）
                motor_r(85, 0);           // 後（左,右)
            }
            else if (laneDirection == 'R')
            {                              // レーン方向　右
                iSetAngle = -LANE_ANGLE_L; /* +で左 -で右に曲がります */
                servoPwmOut(iServoPwm2);   // 2角度制御 3:割込制御無
                motor_f(0, 85);            // 前 （左,右）
                motor_r(0, 85);            // 後（左,右)
            }

            if (cnt1 >= 10)
            {
                cnt1 = 0;
                pattern = 164; // 10ms後次の処理へ
            }
            break;

        case 164: // 10ms待ち後の処理　最内センサ　黒反応時待ち
            if (laneDirection == 'L')
            {                             // レーン方向　左
                iSetAngle = LANE_ANGLE_L; /* +で左 -で右に曲がります */
                servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                motor_f(85, 0);           // 前 （左,右）
                motor_r(85, 0);           // 後（左,右)

                if (digiSensRR == OFF)
                {
                    pattern = 166; /*左デジタルセンサOFF反応時次の処理へ */
                    cnt1 = 0;
                }
            }

            else if (laneDirection == 'R')
            {                              // レーン方向　右
                iSetAngle = -LANE_ANGLE_L; /* +で左 -で右に曲がります */
                servoPwmOut(iServoPwm2);   // 2角度制御 3:割込制御無
                motor_f(0, 85);            // 前 （左,右）
                motor_r(0, 85);            // 後（左,右)

                if (digiSensLL == OFF)
                {
                    pattern = 166; /*左デジタルセンサOFF反応時次の処理へ */
                    cnt1 = 0;
                }
            }
            break;

        case 166: // 最内センサ　黒反応後の処理（大カウンター）　最内センサ　白反応時待ち
            if (laneDirection == 'L')
            { // レーン方向　左
                // iSetAngle = -LANE_ANGLE_L; /* +で左 -で右に曲がります */ // カウンターなので逆に振る　
                servoPwmOut(iServoPwm2); // 2角度制御 3:割込制御無
                motor_f(90, 80);         // 前 （左,右）
                motor_r(90, 0);          // 後（左,右)
                if (sensRRon == ON && cnt1 >= 10)
                {
                    pattern = 168;
                }
                if (digiSensCC == ON && cnt1 >= 10)
                {
                    pattern = 168;
                }
            }
            else if (laneDirection == 'R')
            { // レーン方向　右　カウンター処理
                // iSetAngle = LANE_ANGLE_R; /* +で左 -で右に曲がります */ // カウンターなので逆に振る　
                servoPwmOut(iServoPwm2); // 2角度制御 3:割込制御無
                motor_f(80, 90);         // 前 （左,右）
                motor_r(0, 90);          // 後（左,右)
                if (sensLLon == ON && cnt1 >= 10)
                {
                    pattern = 168;
                }
                if (digiSensCC == ON && cnt1 >= 10)
                {
                    pattern = 168;
                }
                break;
            }
            break;

        case 168: // センターセンサ　白反応時待ち
            if (laneDirection == 'L')
            { // レーン方向　左
                // iSetAngle = -LANE_ANGLE_L; /* +で左 -で右に曲がります */
                servoPwmOut(iServoPwm2); // 2角度制御 3:割込制御無
                motor_f(90, 80);         // 前 （左,右）
                motor_r(90, 0);          // 後（左,右)
            }

            else if (laneDirection == 'R')
            { // レーン方向　右　カウンター処理
                // iSetAngle = LANE_ANGLE_R; /* +で左 -で右に曲がります */
                servoPwmOut(iServoPwm2); // 2角度制御 3:割込制御無
                motor_f(80, 90);         // 前 （左,右）
                motor_r(0, 90);          // 後（左,右)
            }

            if (digiSensCC == ON && abs(anaSensCL_diff - anaSensCR_diff) < 100)
            {
                pattern = 170; /*中央デジタルセンサ反応時次の処理へ*/
                cource = 0;    // コース外れ値0クリア
                cnt1 = 0;
            }
            break;

        case 170:
            /* 少し時間が経つまで待つ */
            i = getServoAngle(); // ステアリング角度取得
            servoPwmOut(iServoPwm);
            motor_r(90, 90);
            motor_f(90, 90);
            if (abs(i) < 10 && cnt1 > 100)
            {
                cnt1 = 0;
                pattern = 11;
                crankMode = 0;      // クランクモードクリア
                crankDirection = 0; // クランクモード（クランク方向）クリア
                laneMode = 0;       // レーンモードクリア
                laneDirection = 0;  // レーンモード（レーン方向）クリア
                laneClearTime = 200;
                break;
            }
            break;

            /************************************************************************/
            /* 	坂モード処理	case 191                                        */
            /************************************************************************/
            //====================上り==========================//
        case 191: // 再チェック
            servoPwmOut(iServoPwm);
            motor_f(100, 100); // 前（左,右）
            motor_r(100, 100); // 後（左,右）

            if (cnt1 >= 15)
            { // 15ms
                if (SLOPE_ANGLE > SLOPE_UP_START - 5)
                { // 上るくん
                    pattern = 192;
                    temp = data_buff[PROP_GAIN_ADDR]; // 比例ゲイン
                    data_buff[PROP_GAIN_ADDR] = 3;    // 比例ゲインダウン
                    lEncoderBuff = lEncoderTotal;     // 1500ct程度
                    cnt1 = 0;
                }
                else
                { // 誤検出時
                    pattern = 11;
                }
            }
            break;

        case 192: // 車体を少し安定させる
            servoPwmOut(iServoPwm);
            motor_f(100, 100); // 前（左,右）
            motor_r(1, 1);     // 後（左,右）

            if (cnt1 >= 40)
            { // 40ms
                pattern = 194;
            }
            break;

        case 194: // 上り坂の4分の3くらいまではSLOPE_SPEEDで走行
            servoPwmOut(iServoPwm);
            if (iEncoder >= SLOPE_UP_SPEED + 3)
            {
                motor_f(-10, -10); // 前 (左,右)
                motor_r(-10, -10); // 後 (左,右)
            }
            else if (iEncoder >= SLOPE_UP_SPEED)
            {
                motor_f(1, 1);
                motor_r(1, 1);
            }
            else
            {
                motor_f(92, 92);
                motor_r(92, 92);
            }

            if (cnt2 < 300)
            {
                CPU_LED_2 = ON;
            }
            else
            {
                CPU_LED_2 = OFF;
                if (cnt2 > 600)
                {
                    cnt2 = 0;
                }
            }

            if (lEncoderTotal - lEncoderBuff >= 1500)
            { // 3/4(1000mm程度)
                pattern = 196;
                lEncoderBuff = lEncoderTotal; // 1500ct程度
                cnt1 = 0;
            }
            break;

        case 196:
            servoPwmOut(iServoPwm);
            motor_f(1, 1);     // 前（左,右）
            motor_r(-30, -30); // 後（左,右）
            if (lEncoderTotal - lEncoderBuff >= 300)
            { // 200mm程度
                pattern = 198;
                lEncoderBuff = lEncoderTotal; // 1500ct程度
                cnt1 = 0;
            }
            if (cnt2 < 200)
            {
                CPU_LED_2 = ON;
            }
            else
            {
                CPU_LED_2 = OFF;
                if (cnt2 > 400)
                {
                    cnt2 = 0;
                }
            }
            break;

        case 198:
            CPU_LED_2 = ON;
            servoPwmOut(iServoPwm);
            if (iEncoder >= SLOPE_UP_SPEED - 10)
            {                      // 3.0m/s程度
                motor_f(1, 1);     // 前 (左,右)
                motor_r(-30, -30); // 後 (左,右)
            }
            else
            {
                motor_f(1, 1);   // 前 (左,右)
                motor_r(30, 30); // 後 (左,右)
            }
            if (SLOPE_ANGLE <= SLOPE_UP_FIN + 20)
            { // 坂終わり判定登り
                pattern = 200;
            }
            break;

        case 200:
            servoPwmOut(iServoPwm);
            motor_f(1, 1);     // 前（左,右）
            motor_r(-30, -30); // 後（左,右）
            if (lEncoderTotal - lEncoderBuff >= 300)
            { // 200mm程度
                pattern = 202;
                cnt1 = 0;
            }
            break;

        case 202:
            servoPwmOut(iServoPwm);
            slopeTotalCount = 1;
            data_buff[PROP_GAIN_ADDR] = temp;
            slopeFinTime = 0; // 坂下りご検出防止タイマー
            pattern = 11;

            /*while(1){//確認用
            motor_f(0,0);	//前 (左,右)
            motor_r(0,0);	//後 (左,右)
          }
          */
            break;

            //====================上り終了======================//
            //====================下り==========================//
        case 211: // 検出再確認(不要？とりあえず)
            servoPwmOut(iServoPwm);
            motor_f(70, 70); // 前（左,右）
            motor_r(70, 70); // 後（左,右）

            if (cnt1 >= 15)
            { // 15ms
                if (SLOPE_ANGLE < SLOPE_DOWN_START + 5)
                { // 下るくん
                    pattern = 212;
                    temp = data_buff[PROP_GAIN_ADDR]; // 比例ゲイン
                    data_buff[PROP_GAIN_ADDR] = 3;    // 比例ゲインダウン
                    lEncoderBuff = lEncoderTotal;
                    cnt1 = 0;
                }
                else
                { // 誤検出時
                    pattern = 11;
                }
            }
            break;

        case 212: // センサーバーを落とすためのブレーキ
            servoPwmOut(iServoPwm);
            motor_f(0, 0);     // 前（左,右）
            motor_r(-60, -60); // 後（左,右）
            if (lEncoderTotal - lEncoderBuff >= 300)
            { // 200mm程度
                lEncoderBuff = lEncoderTotal;
                pattern = 214;
            }
            break;

        case 214: // 安定走行を目指し前輪で走行
            servoPwmOut(iServoPwm);
            motor_f(70, 70); // 前（左,右）
            motor_r(30, 30); // 後（左,右）
            if (lEncoderTotal - lEncoderBuff >= 450)
            { // 300mm程度
                lEncoderBuff = lEncoderTotal;
                pattern = 216;
            }
            break;

        case 216:
            servoPwmOut(iServoPwm);
            if (iEncoder >= SLOPE_DOWN_SPEED + 3)
            {
                motor_f(-10, -10); // 前 (左,右)
                motor_r(-10, -10); // 後 (左,右)
            }
            else if (iEncoder >= SLOPE_DOWN_SPEED)
            {
                motor_f(1, 1);
                motor_r(1, 1);
            }
            else
            {
                motor_f(85, 85);
                motor_r(85, 85);
            }

            if (SLOPE_ANGLE > SLOPE_DOWN_FIN - 5)
            { // 下り終わり検出
                lEncoderBuff = lEncoderTotal;
                pattern = 218;
            }
            break;

        case 218: // 車体安定のためのブレーキ（センサーバーを安定させる）
            servoPwmOut(iServoPwm);
            motor_f(-30, -30); // 前（左,右）
            motor_r(-30, -30); // 後（左,右）
            if (lEncoderTotal - lEncoderBuff >= 151)
            { // 50mm
                lEncoderBuff = lEncoderTotal;
                pattern = 220;
            }
            break;

        case 220: // 車体安定のためのフリー
            servoPwmOut(iServoPwm);
            motor_f(1, 1); // 前（左,右）
            motor_r(1, 1); // 後（左,右）
            if (lEncoderTotal - lEncoderBuff >= 151)
            { // 50mm
                pattern = 222;
            }
            break;

        case 222:
            servoPwmOut(iServoPwm);
            slopeTotalCount = 2;
            data_buff[PROP_GAIN_ADDR] = temp;
            pattern = 11;
            /*
          while(1){
            motor_f(0,0);	//前 (左,右)
            motor_r(0,0);	//後 (左,右)
          }
          */
            break;

            //====================下り終了======================//
        case 231:
            /* 停止処理 */
            servoPwmOut(iServoPwm);
            motor_f(0, 0);
            motor_r(0, 0);
            crankMode = 1;
            pattern = 232;

            if (!pushsw_get() && cnt1 > 500)
                ; // SW押され待ち
            break;

        case 232:
            servoPwmOut(iServoPwm);
            if (iEncoder <= 1)
            {
                // servoPwmOut(0);
                pattern = 233;
                cnt1 = 0;
                break;
            }
            break;

        case 233:
            servoPwmOut(0);
            saveFlag = false; // ログデータ保存停止
            Run_end = true;
            if (cnt2 < 100)
            {
                CPU_LED_2 = ON;
                CPU_LED_3 = OFF;
            }
            else
            {
                CPU_LED_2 = OFF;
                CPU_LED_3 = ON;
                if (cnt2 > 200)
                {
                    cnt2 = 0;
                }
            }
            if (pushsw_get() && cnt1 > 500)
            {
                pattern = 234;
            }
            break;

        case 234:
            servoPwmOut(0);
            // ログ出力
            Serial.print("\n");
            Serial.print("Run Data Out\n");
            SD_file_close();
            CPU_LED_2 = OFF;
            CPU_LED_3 = OFF;
            pattern = 235;
            // writeLog();//SDカードにログの書き込み
            // SD_file_close();

            // CPU_LED_2 = ON;
            // CPU_LED_3 = ON;

            // /* 最後のデータが書き込まれるまで待つ */
            // if (microSDProcessEnd() == 0) {
            //   pattern = 235;
            //   CPU_LED_2 = OFF;
            //   CPU_LED_3 = OFF;
            // }
            // pattern = 235;

            break;

        case 235:
            /* 何もしない */
            break;

        case 241:
            /* 停止 */
            servoPwmOut(0);
            motor_f(0, 0);
            motor_r(0, 0);
            // setBeepPatternS(0xc000);
            saveFlag = 0;
            // saveSendIndex = 0;
            pattern = 243;
            cnt1 = 0;
            break;

        case 242:
            /* プッシュスイッチが離されたかチェック */
            if (pushsw_get() == OFF)
            {
                pattern = 243;
                cnt1 = 0;
            }
            break;

        case 243:
            /* 0.5s待ち */
            if (cnt1 >= 500)
            {
                pattern = 245;
                cnt1 = 0;
            }
            break;

        case 245:
            // Serial2.begin(115200);
            while (!Serial2)
                ;
            /* タイトル転送、転送準備 */
            Serial2.print("\n");
            Serial2.print("Run Data Out\n");
            pattern = 246;
            break;

        case 246:
            /* データ転送 */
            /* 終わりのチェック */
            break;

        case 247:
            /* 転送終了 */
            while (1)
                ;
            break;

        default:
            break;
        }
    }
}

/**********************************************************************/
/*
 * 0.25msタイマ割り込み.
 */
void timerCallback(timer_callback_args_t __attribute((unused)) * p_args)
{
    static unsigned int timer_counter = 0; // 0.25msごとのカウンタ
    signed long i;

    // 1ms周期
    switch (++timer_counter)
    {
    case 1:
        cnt1++;
        cnt2++;         // LED制御用
        slopeFinTime++; // 坂誤検出防止タイマー

        mtPower++; // コーナリング時PWMを徐々にアップ用

        if (laneClearTime > 0)
        {
            laneClearTime--;
        }

        if (crankClearTime > 0)
        {
            crankClearTime--;
        }

        // センサ値取得
        anaSensUR_on = ANA_SENS_UR;
        anaSensRR_on = ANA_SENS_RR;
        anaSensCR_on = ANA_SENS_CR;
        anaSensCC_on = ANA_SENS_CC;
        anaSensCL_on = ANA_SENS_CL;
        anaSensLL_on = ANA_SENS_LL;
        anaSensUL_on = ANA_SENS_UL;

        INFRARED_LED = OFF;

        break;
    case 2:
        // センサ値取得
        anaSensUR_off = ANA_SENS_UR;
        anaSensRR_off = ANA_SENS_RR;
        anaSensCR_off = ANA_SENS_CR;
        anaSensCC_off = ANA_SENS_CC;
        anaSensCL_off = ANA_SENS_CL;
        anaSensLL_off = ANA_SENS_LL;
        anaSensUL_off = ANA_SENS_UL;

        // @TODO LED ON
        INFRARED_LED = ON;

        // 差分計算(外乱除去/トレース等はこの値を使用)
        anaSensUR_diff = anaSensUR_on - anaSensUR_off;
        anaSensRR_diff = anaSensRR_on - anaSensRR_off;
        anaSensCR_diff = anaSensCR_on - anaSensCR_off;
        anaSensCC_diff = anaSensCC_on - anaSensCC_off;
        anaSensCL_diff = anaSensCL_on - anaSensCL_off;
        anaSensLL_diff = anaSensLL_on - anaSensLL_off;
        anaSensUL_diff = anaSensUL_on - anaSensUL_off;

        // 2値化
        digiSensUR = ((anaSensUR_diff > thrSensUR) ? ON : OFF);
        digiSensRR = ((anaSensRR_diff > thrSensRR) ? ON : OFF);
        digiSensCR = ((anaSensCR_diff > thrSensCR) ? ON : OFF);
        digiSensCC = ((anaSensCC_diff > thrSensCC) ? ON : OFF);
        digiSensCL = ((anaSensCL_diff > thrSensCL) ? ON : OFF);
        digiSensLL = ((anaSensLL_diff > thrSensLL) ? ON : OFF);
        digiSensUL = ((anaSensUL_diff > thrSensUL) ? ON : OFF);

        /* サーボモータ制御(PD計算) */
        servoControl();
        servoControl2();
        break;
    case 3:
        // @TODO センサ値取得
        anaSensUR_on = ANA_SENS_UR;
        anaSensRR_on = ANA_SENS_RR;
        anaSensCR_on = ANA_SENS_CR;
        anaSensCC_on = ANA_SENS_CC;
        anaSensCL_on = ANA_SENS_CL;
        anaSensLL_on = ANA_SENS_LL;
        anaSensUL_on = ANA_SENS_UL;

        // @TODO LED OFF
        INFRARED_LED = OFF;

        break;
    case 4:
        // @TODO センサ値取得
        anaSensUR_off = ANA_SENS_UR;
        anaSensRR_off = ANA_SENS_RR;
        anaSensCR_off = ANA_SENS_CR;
        anaSensCC_off = ANA_SENS_CC;
        anaSensCL_off = ANA_SENS_CL;
        anaSensLL_off = ANA_SENS_LL;
        anaSensUL_off = ANA_SENS_UL;

        // @TODO LED ON
        INFRARED_LED = ON;

        // 差分計算(外乱除去/トレース等はこの値を使用)
        anaSensUR_diff = anaSensUR_on - anaSensUR_off;
        anaSensRR_diff = anaSensRR_on - anaSensRR_off;
        anaSensCR_diff = anaSensCR_on - anaSensCR_off;
        anaSensCC_diff = anaSensCC_on - anaSensCC_off;
        anaSensCL_diff = anaSensCL_on - anaSensCL_off;
        anaSensLL_diff = anaSensLL_on - anaSensLL_off;
        anaSensUL_diff = anaSensUL_on - anaSensUL_off;

        // 2値化
        digiSensUR = ((anaSensUR_diff > thrSensUR) ? ON : OFF);
        digiSensRR = ((anaSensRR_diff > thrSensRR) ? ON : OFF);
        digiSensCR = ((anaSensCR_diff > thrSensCR) ? ON : OFF);
        digiSensCC = ((anaSensCC_diff > thrSensCC) ? ON : OFF);
        digiSensCL = ((anaSensCL_diff > thrSensCL) ? ON : OFF);
        digiSensLL = ((anaSensLL_diff > thrSensLL) ? ON : OFF);
        digiSensUL = ((anaSensUL_diff > thrSensUL) ? ON : OFF);

        /* サーボモータ制御(PD計算) */
        servoControl();
        servoControl2();

        // 10ms周期処理(エンコーダ/ログ/舵角加速度)
        /* 10回中1回実行する処理 */
        switch (++iTimer10)
        {
        case 1:
            i = R_GPT6->GTCNT;
            iEncoder = i - uEncoderBuff;
            lEncoderTotal += iEncoder;
            uEncoderBuff = i;
            PD_trig = 1;
            break;
        case 2:
            i = BAR_ANGLE;
            iAngle2 = i - iAngleBuff;
            iAngleBuff = i;
            Angle_D = Ang();
            break;
        case 8:
            if (saveFlag)
            {
                LOG_rec(); // ログをRAMに保存
            }
            break;

        case 9:
            // writeLog();//SDにログの書き込み
            break;
        case 10:
            iTimer10 = 0;
            break;
        }

        // 停止処理(エンコーダの値を見ているため、エンコーダ処理の後に記述！)
        if (pattern >= 11 && pattern <= 230)
        {
            /* 距離による停止処理 */
            if (lEncoderTotal >= METER * data_buff[TOTAL_DIST_ADDR] && !(dipsw_get() & 0x01))
            {
                pattern = 231;
            }

            /* 脱輪時の停止処理（デジタルセンサ） */
            if ((digiSensLL == OFF && digiSensCC == OFF && digiSensRR == OFF) || (digiSensLL == ON && digiSensCC == ON && digiSensRR == ON))
            // if ((digiSensCL == OFF && digiSensCC == OFF && digiSensCR == OFF) || (digiSensCL == ON && digiSensCC == ON && digiSensCR == ON))
            {
                check_sen_cnt++;
                if (check_sen_cnt >= 1000) // 400
                {
                    pattern = 231;
                }
            }
            else
            {
                check_sen_cnt = 0;
            }

            /* 脱輪時の停止処理（ロータリエンコーダ） */
            if (iEncoder <= 1 && !(dipsw_get() & 0x01))
            {
                check_enc_cnt++;
                if (check_enc_cnt >= 1000) // 2000
                {
                    pattern = 231;
                }
            }
            else
            {
                check_enc_cnt = 0;
            }
            /* 途中で停止処理 */
            if (pushsw_get() == ON && lEncoderTotal > 1000)
            {
                pattern = 231;
                cnt1 = 0;
            }
        }
        timer_counter = 0; // カウンタリセット
        break;
    default:
        timer_counter = 0; // カウンタリセット
        break;
    }
}

/************************************************************************/
/* マイコンボード上のディップスイッチ値読み込み                         */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0～15                                              */
/************************************************************************/
unsigned char dipsw_get(void)
{
    //	return (!digitalRead(26) << 1 | !digitalRead(25) << 0);
    return ((!R_PORT3->PIDR_b.PIDR6) << 1) | ((!R_PORT3->PIDR_b.PIDR7) << 0);
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3上のプッシュスイッチ値読み込み          */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0:OFF 1:ON                                         */
/************************************************************************/
unsigned char pushsw_get(void)
{
    //	return !digitalRead(RUN_SWITCH);
    return !(R_PORT0->PIDR_b.PIDR3);
}

/************************************************************************/
/* 後輪の速度制御                                                       */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_r(int accele_l, int accele_r)
{
    // モータ停止モード(dipsw_get() & 0x01)
    if (!(dipsw_get() & 0x01))
    {
        motor2_r(accele_l * -1, accele_r);
    }
}

/************************************************************************/
/* 後輪の速度制御2 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_r(int accele_l, int accele_r)
{

    if (accele_l >= 100)
    {
        accele_l = 100;
    }
    if (accele_l <= -100)
    {
        accele_l = -100;
    }
    if (accele_r >= 100)
    {
        accele_r = 100;
    }
    if (accele_l <= -100)
    {
        accele_l = -100;
    }

    motor_buff_Rl = accele_l * -1;
    motor_buff_Rr = accele_r;
    // saveData[13][logCt] = accele_l;
    // saveData[12][logCt] = accele_r;

    // 左モータ制御
    if (accele_l > 0)
    {
        //	digitalWrite(MOTOR_RL_A, HIGH);
        RL_A = HIGH;
        //	digitalWrite(MOTOR_RL_B, LOW);
        RL_B = LOW;
        MOTOR_RL_PWM = (long)(MOTOR_RL_PWM_CYCLE + 1) * accele_l / 100;
    }
    else if (accele_l < 0)
    {
        //	digitalWrite(MOTOR_RL_A, LOW);
        RL_A = LOW;
        //	digitalWrite(MOTOR_RL_B, HIGH);
        RL_B = HIGH;
        MOTOR_RL_PWM = (long)(MOTOR_RL_PWM_CYCLE + 1) * (-accele_l) / 100;
    }

    else
    {
        //	digitalWrite(MOTOR_RL_A, LOW);
        RL_A = LOW;
        //	digitalWrite(MOTOR_RL_B, HIGH);
        RL_B = LOW;
        MOTOR_RL_PWM = (long)(MOTOR_RL_PWM_CYCLE + 1) * (-accele_l) / 100;
    }
    // MOTOR_RL_PWM = accele_l;

    // 右モータ制御
    if (accele_r > 0)
    {
        //	digitalWrite(MOTOR_RR_A, HIGH);
        RR_A = HIGH;
        //	digitalWrite(MOTOR_RR_B, LOW);
        RR_B = LOW;
        MOTOR_RR_PWM = (long)(MOTOR_RR_PWM_CYCLE + 1) * accele_r / 100;
    }
    else if (accele_r < 0)
    {
        //	digitalWrite(MOTOR_RR_A, LOW);
        RR_A = LOW;
        //	digitalWrite(MOTOR_RR_B, HIGH);
        RR_B = HIGH;
        MOTOR_RR_PWM = (long)(MOTOR_RR_PWM_CYCLE + 1) * (-accele_r) / 100;
    }

    else
    {
        //	digitalWrite(MOTOR_RR_A, LOW);
        RR_A = LOW;
        //	digitalWrite(MOTOR_RR_B, HIGH);
        RR_B = LOW;
        MOTOR_RR_PWM = (long)(MOTOR_RR_PWM_CYCLE + 1) * (-accele_r) / 100;
    }
    // MOTOR_RR_PWM = accele_r;
}

/************************************************************************/
/* 前輪の速度制御                                                       */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_f(int accele_l, int accele_r)
{
    // モータ停止モード
    if (!(dipsw_get() & 0x01))
    {
        motor2_f(accele_l * -1, accele_r);
    }
}

/************************************************************************/
/* 前輪の速度制御2 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_f(int accele_l, int accele_r)
{
    if (accele_l >= 100)
    {
        accele_l = 100;
    }

    if (accele_l <= -100)
    {
        accele_l = -100;
    }

    if (accele_r >= 100)
    {
        accele_r = 100;
    }

    if (accele_r <= -100)
    {
        accele_r = -100;
    }

    motor_buff_Fl = accele_l * -1;
    motor_buff_Fr = accele_r;

    // saveData[11][logCt] = accele_l;
    // saveData[10][logCt] = accele_r;

    // 左モータ制御
    if (accele_l > 0)
    {
        //	digitalWrite(MOTOR_FL_A, HIGH);
        FL_A = HIGH;
        //	digitalWrite(MOTOR_FL_B, LOW);
        FL_B = LOW;
        MOTOR_FL_PWM = (long)(MOTOR_FL_PWM_CYCLE + 1) * accele_l / 100;
    }
    else if (accele_l < 0)
    {
        //	digitalWrite(MOTOR_FL_A, LOW);
        FL_A = LOW;
        //	digitalWrite(MOTOR_FL_B, HIGH);
        FL_B = HIGH;
        MOTOR_FL_PWM = (long)(MOTOR_FL_PWM_CYCLE + 1) * (-accele_l) / 100;
    }
    else
    {
        //	digitalWrite(MOTOR_FL_A, LOW);
        FL_A = LOW;
        //	digitalWrite(MOTOR_FL_B, HIGH);
        FL_B = LOW;
        MOTOR_FL_PWM = (long)(MOTOR_FL_PWM_CYCLE + 1) * (-accele_l) / 100;
    }

    // 右モータ制御
    if (accele_r > 0)
    {
        //	digitalWrite(MOTOR_FR_A, HIGH);
        FR_A = HIGH;
        //	digitalWrite(MOTOR_FR_B, LOW);
        FR_B = LOW;
        MOTOR_FR_PWM = (long)(MOTOR_FR_PWM_CYCLE + 1) * accele_r / 100;
    }
    else if (accele_r < 0)
    {
        //	digitalWrite(MOTOR_FR_A, LOW);
        FR_A = LOW;
        //	digitalWrite(MOTOR_FR_B, HIGH);
        FR_B = HIGH;
        MOTOR_FR_PWM = (long)(MOTOR_FR_PWM_CYCLE + 1) * (-accele_r) / 100;
    }
    else
    {
        //	digitalWrite(MOTOR_FR_A, LOW);
        FR_A = LOW;
        //	digitalWrite(MOTOR_FR_B, HIGH);
        FR_B = LOW;
        MOTOR_FR_PWM = (long)(MOTOR_FR_PWM_CYCLE + 1) * (-accele_r) / 100;
    }
}

/************************************************************************/
/* サーボモータ制御                                                     */
/* 引数　 サーボモータPWM：-100～100                                    */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void servoPwmOut(int pwm)
{
    motor_buff_stare = pwm;

    // saveData[9][logCt] = pwm;
    // モータ制御
    if (pwm >= 0)
    {
        //	digitalWrite(MOTOR_ST_A, HIGH);
        ST_A = HIGH;
        //	digitalWrite(MOTOR_ST_B, LOW);
        ST_B = LOW;
        MOTOR_ST_PWM = (long)(MOTOR_ST_PWM_CYCLE + 1) * pwm / 100;
    }
    else
    {
        //	digitalWrite(MOTOR_ST_A, LOW);
        ST_A = LOW;
        //	digitalWrite(MOTOR_ST_B, HIGH);
        ST_B = HIGH;
        MOTOR_ST_PWM = (long)(MOTOR_ST_PWM_CYCLE + 1) * (-pwm) / 100;
    }
}

/************************************************************************/
/* 差分の正規化                                                          */
/* 引数　 なし                                                          */
/* 戻り値 なし                                                          */
/************************************************************************/
void Diff_Nomalization(void)
{
    // int digiSensUR_buf = (())
}

/************************************************************************/
/* クロスライン検出処理                                                 */
/* 引数　 なし                                                          */
/* 戻り値 0:クロスラインなし 1:あり                                     */
/************************************************************************/
int check_crossline(void)
{
    unsigned char b;
    int ret = 0;

    if (sensLLon == ON && digiSensCC == ON && sensRRon == ON)
    // if (digiSensLL == ON && digiSensCC == ON && digiSensRR == ON)
    {
        ret = 1;
    }
    else
    {
        ret = 0;
    }

    return ret;
}

/***********************9**************************************************/
/* 右ハーフライン検出処理                                               */
/* 引数　 なし                                                          */
/* 戻り値 0:右ハーフラインなし 1:あり                                   */
/************************************************************************/
int check_rightline(void)
{
    unsigned char b;
    int ret = 0;
    static int lane_count = 0;

    if (digiSensCC == ON && sensRRon == ON)
    // if (digiSensCC == ON && digiSensRR == ON)
    {
        // crank_count++;
        // lane_count++;
        // if (lane_count > 10)
        // {
        //   ret = 1;
        // }
        ret = 1;
    }
    else
    {
        lane_count = 0;
        ret = 0;
    }

    return ret;
}

/************************************************************************/
/* 左ハーフライン検出処理                                               */
/* 引数　 なし                                                          */
/* 戻り値 0:左ハーフラインなし 1:あり                                   */
/************************************************************************/
int check_leftline(void)
{
    unsigned char b;
    int ret = 0;
    static int lane_count = 0;

    if (digiSensCC == ON && sensLLon == ON)
    // if (digiSensCC == ON && digiSensLL == ON)
    {
        // crank_count++;
        // lane_count++;
        // if (lane_count > 10)
        // {
        //   ret = 1;
        // }
        ret = 1;
    }
    else
    {
        lane_count = 0;
        ret = 0;
    }

    return ret;
}

/************************************************************************/
/* アナログセンサ値取得 (岡谷工業  デジタルセンサ3つ用使用)                */
/* 引数　 なし                                                          */
/* 戻り値 センサ値                                                      */
/************************************************************************/
int getAnalogSensor(void)
{
    int ret;

    //   ret = ad1 - ad0; /* アナログセンサ情報取得       */
    // ret = (anaSensCL_diff >> 2) - (anaSensCR_diff >> 2); /* アナログセンサ情報取得    左大：＋ 　右大：-　  */
    ret = (anaSensCL_diff) - (anaSensCR_diff); /* アナログセンサ情報取得    左大：＋ 　右大：-　  */

    // 100程度が最大
    // if (ret < 0)
    // {
    //   ret = -curves18[abs(ret)];
    // }
    // else
    // {
    //   ret = curves18[abs(ret)];
    // }

    // if (((sensLLon == ON && digiSensCC == ON) || (sensRRon == ON && digiSensCC == ON)) && pattern != 3)
    // if (((digiSensCR == ON && digiSensCC == ON) || (digiSensCL == ON && digiSensCC == ON)) && pattern != 3)
    // if ((digiSensLL == OFF && digiSensCC == OFF && digiSensRR == OFF) || (digiSensLL == ON && digiSensCC == ON && digiSensRR == ON))
    // {
    //   ret = 0;
    // }

    // if (!crankMode) {
    /* クランクモードでなければ補正処理 */
    // courceOut():0のときは、アナログセンサによるトレース
    // courceOut();
    // switch (cource) // ここ要る？
    // {

    // case 1:
    // case 2:
    // case 3:
    //   ret = -200;
    //   break;

    // case -1:
    // case -2:
    // case -3:
    //   ret = 200;
    //   break;
    // }
    return ret;
}

/************************************************************************/
/* コース外れ値取得関数 */
/* 引数　 無し */
/* 戻り値 無し */
/*　注意：　変数cource＝グローバル変数
/************************************************************************/
void courceOut(void)
{
    if (digiSensCC == ON)
    { /* Cセンサ白線 */
        switch (cource)
        {
        case 1:
        case 2:
        case -1:
        case -2:
            cource = 0; /* コース上 */
            break;
        }
    }
    else if (anaSensCL_diff > thrSensCL)
    { /* Lセンサ白線 */
        switch (cource)
        {
        case 0:
        case -2:
            cource = -1; /* コース外れR1 */
            break;
        }
    }
    else if (anaSensCR_diff > thrSensCR)
    { /* Rセンサ白線 */
        switch (cource)
        {
        case 0:
        case 2:
            cource = 1; /* コース外れL1 */
            break;
        }
    }
    else if (digiSensLL == ON)
    { /* LLセンサ白線 */
        switch (cource)
        {
        case -1:
        case -3:
            cource = -2; /* コース外れR2 */
            break;
        }
    }
    else if (digiSensRR == ON)
    { /* RRセンサ白線 */
        switch (cource)
        {
        case 1:
        case 3:
            cource = 2; /* コース外れL2 */
            break;
        }
    }
    // else if (digiSensRR == OFF && digiSensCC == OFF && digiSensLL == OFF && anaSensCR_diff < thrSensCR && anaSensCL_diff < thrSensCL)
    // { /* 全てのセンサ黒 */
    //   switch (cource)
    //   {
    //   case 2:
    //     cource = 3; /* コース外れL3 */
    //     break;
    //   case -2:
    //     cource = -3; /* コース外れR3 */
    //     break;
    //   }
    // }
    //  return cource;
}

/************************************************************************/
/* サーボモータ制御  トレース用                                            */
/* 引数　 なし                                                          */
/* 戻り値 グローバル変数 iServoPwm に代入                                 */
/************************************************************************/
void servoControl(void)
{
    int i, iRet, iP, iD;
    int kp, kd;

    i = getAnalogSensor(); /* センサ値取得                 */
    // if(pattern ==  21 || pattern ==  31){
    //   kp = data_buff[PROP_GAIN_ADDR]*2;
    //   kd = data_buff[DIFF_GAIN_ADDR];
    // }
    //  else{
    kp = data_buff[PROP_GAIN_ADDR];
    kd = data_buff[DIFF_GAIN_ADDR];
    //  }

    /* サーボモータ用PWM値計算 */
    iP = kp * i;                   /* 比例                         */
    iD = kd * (iSensorBefore - i); /* 微分(目安はPの5～10倍)       */
    iRet = iP - iD;

    iRet /= 64; //  <<1 :/2  <<2 :/4  <<3 :/8  <<4 :/16   <<5 :/32  <<6 :/64
    // iRet /= 16; //  <<1 :/2  <<2 :/4  <<3 :/8  <<4 :/16   <<5 :/32  <<6 :/64

    /* PWMの上限の設定 */
    //    if (iRet > 70) iRet = 70;   /* マイコンカーが安定したら     */
    //    if (iRet < -70) iRet = -70; /* 上限を70くらいにしてください */

    if (iRet > 100)
        iRet = 100; /* マイコンカーが安定したら     */
    if (iRet < -100)
        iRet = -100; /* 上限を70くらいにしてください */

    iServoPwm = -iRet;
    iSensorBefore = i; /* 次回はこの値が1ms前の値となる*/
}

/************************************************************************/
/* サーボモータ2制御  角度制御用                                           */
/* 引数　 なし                                                          */
/* 戻り値 グローバル変数 iServoPwm に代入                               */
/************************************************************************/
void servoControl2(void)
{

    signed int i, j, iRet, iP, iD;
    signed int kp, kd;

    i = iSetAngle;
    j = getServoAngle();

    /* サーボモータ用PWM値計算 */
    iP = 10 * (j - i);             /* 比例                         */
    iD = 60 * (iAngleBefore2 - j); /* 微分(目安はPの5～10倍)       */
    iRet = iP - iD;
    iRet /= 2;

    /* PWMの上限の設定 */
    if (iRet > 100)
        iRet = 100; /* マイコンカーが安定したら     */
    if (iRet < -100)
        iRet = -100; /* 上限を70くらいにしてください */
    iServoPwm2 = iRet;

    iAngleBefore2 = j; /* 次回はこの値が1ms前の値となる*/
}

/************************************************************************/
/* サーボモータ2制御  角度制御用(キャリブレーション時)                        */
/* 引数　 なし                                                          */
/* 戻り値 グローバル変数 iServoPwm に代入                               */
/************************************************************************/
void servoControl3(void)
{

    signed int i, j, iRet, iP, iD;
    signed int kp, kd;

    i = iSetAngle;
    j = getServoAngle();

    /* サーボモータ用PWM値計算 */
    iP = 2 * (j - i);              /* 比例                         */
    iD = 10 * (iAngleBefore3 - j); /* 微分(目安はPの5～10倍)       */
    iRet = iP - iD;
    iRet /= 2;

    /* PWMの上限の設定 */
    if (iRet > 100)
        iRet = 100; /* マイコンカーが安定したら     */
    if (iRet < -100)
        iRet = -100; /* 上限を70くらいにしてください */
    iServoPwm3 = iRet;

    iAngleBefore3 = j; /* 次回はこの値が1ms前の値となる*/
}

/************************************************************************/
/* DataFlashのパラメータ読み込み                                        */
/* 引数         なし                                                    */
/* 戻り値       なし                                                    */
/************************************************************************/
void readDataFlashParameter(void)
{
    // データ読み取り
    for (uint16_t i = 0; i < MAX_NUM_ADDR; i++)
    {
        data_buff[i] = EEPROM.read(i);
    }

    // ヘッダー確認
    if (data_buff[HEADER_ADDR] != FLASH_HEADER)
    {
        // デフォルト値
        data_buff[TOTAL_DIST_ADDR] = 15;
        data_buff[START_TIME_ADDR] = 5;
        data_buff[PROP_GAIN_ADDR] = 4;
        data_buff[DIFF_GAIN_ADDR] = 14;
        data_buff[TRG_SPEED_ADDR] = 60;
        data_buff[CORNER_SPEED_ADDR] = 40;
        // data_buff[CRANK_SPEED_ADDR]		= 35;
        // data_buff[LANE_SPEED_ADDR]		= 42;
        // data_buff[SLOPE_UP_ADDR]		= 20;
        // data_buff[LANE_ANGLE_L_ADDR]	= 35;
        // data_buff[LANE_ANGLE_R_ADDR]	= 35;
        // data_buff[SENS_L_THOLD1_ADDR]	= (450 >> 8) & 0xFF;
        // data_buff[SENS_L_THOLD2_ADDR]	= (450     ) & 0xFF;
        // data_buff[SENS_R_THOLD1_ADDR]	= (450 >> 8) & 0xFF;
        // data_buff[SENS_R_THOLD2_ADDR]	= (450     ) & 0xFF;

        // パラメーター設定
        writeDataFlashParameter();
    }
}

/************************************************************************/
/* DataFlashへパラメータ書き込み                                        */
/* 引数         なし                                                    */
/* 戻り値       なし                                                    */
/************************************************************************/
void writeDataFlashParameter(void)
{
    // 各値書き込み
    EEPROM.put(HEADER_ADDR, FLASH_HEADER);
    EEPROM.put(TOTAL_DIST_ADDR, data_buff[TOTAL_DIST_ADDR]);
    EEPROM.put(START_TIME_ADDR, data_buff[START_TIME_ADDR]);
    EEPROM.put(PROP_GAIN_ADDR, data_buff[PROP_GAIN_ADDR]);
    EEPROM.put(DIFF_GAIN_ADDR, data_buff[DIFF_GAIN_ADDR]);
    EEPROM.put(TRG_SPEED_ADDR, data_buff[TRG_SPEED_ADDR]);
    EEPROM.put(CORNER_SPEED_ADDR, data_buff[CORNER_SPEED_ADDR]);
    EEPROM.put(CRANK_SPEED_ADDR, data_buff[CRANK_SPEED_ADDR]);
    EEPROM.put(LANE_SPEED_ADDR, data_buff[LANE_SPEED_ADDR]);
}

/************************************************************************/
/* LCDとスイッチを使ったパラメータセット処理                            */
/* 引数         なし                                                    */
/* 戻り値       なし                                                    */
/************************************************************************/
int lcdProcess(void)
{
    int i;
    char sw = 0; // LCDスイッチ情報用

    // printf("lcd_pattern=%d",lcd_pattern );
    printf(" pattern=%d\n", pattern);

    // スイッチ情報取得
    sw = SwitchGetState();
    //    printf("    sw=%d", sw);

    // メニュー＋１
    if (sw == MENU_UP)
    {
        lcd_pattern++;
        delay(200);

        if (lcd_pattern == 11)
            lcd_pattern = 1;
    }

    // メニュー－１
    if (sw == MENU_DOWN)
    {
        lcd_pattern--;
        delay(200);

        if (lcd_pattern == 0)
            lcd_pattern = 10;
    }

    /* LCD、スイッチ処理 */
    switch (lcd_pattern)
    {
    case 1:
        /* 走行停止距離調整 */
        servoPwmOut(0);
        i = data_buff[TOTAL_DIST_ADDR];
        if (sw == DATA_UP)
        {
            i++;
            if (i > 100)
                i = 100;
        }
        if (sw == DATA_DOWN)
        {
            i--;
            if (i < 0)
                i = 0;
        }
        data_buff[TOTAL_DIST_ADDR] = i;

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("01 Stop L[m]=%03d", i);
        LcdPosition(0, 1);
        LcdPrintf("Encoder = %03d   ", lEncoderTotal);
        break;

    case 2:
        /* スタート待ち時間調整 */
        servoPwmOut(0);

        i = data_buff[START_TIME_ADDR];
        if (sw == DATA_UP)
        {
            i++;
            if (i > 100)
                i = 100;
        }
        if (sw == DATA_DOWN)
        {
            i--;
            if (i < 0)
                i = 0;
        }
        data_buff[START_TIME_ADDR] = i;

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("02 St time = %03d", i);
        LcdPosition(0, 1);
        // LcdPrintf("L=%1d C=%1d R=%1d %1d   ",
        //           digiSensLL, digiSensCC, digiSensRR
        LcdPrintf("LL=%4d  RR=%4d   ",
                  thrSensLL, thrSensRR
                  /*,
                  SENS_ALL*/
        );
        cnt1 = 0;
        break;

    case 3:
        /* トレース比例制御調整 */
        // servoPwmOut(iServoPwm);
        servoPwmOut(0);

        i = data_buff[PROP_GAIN_ADDR];
        if (sw == DATA_UP)
        {
            i++;
            if (i > 100)
                i = 100;
        }
        if (sw == DATA_DOWN)
        {
            i--;
            if (i < 0)
                i = 0;
        }
        data_buff[PROP_GAIN_ADDR] = i;

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("03 Trace kp =%03d", i);
        LcdPosition(0, 1);
        LcdPrintf("L=%4d  R=%4d   ",
                  anaSensCL_diff, anaSensCR_diff);
        // LcdPrintf("L=%4d  R=%4d   ",
        //           ANA_SENS_CR, ANA_SENS_CL);
        // lcdPrintf("cnt1=%4d" , cnt1);

        if (pushsw_get())
        {
            //  パラメータ保存
            writeDataFlashParameter();
            LcdPosition(0, 0);
            LcdPrintf("para set           ");
            LcdPosition(0, 1);
            LcdPrintf("OK!                      ");
            while (1)
            {
                servoPwmOut(iServoPwm);
            }
        }
        break;

    case 4:
        /* トレース微分制御調整 */
        // servoPwmOut(-iServoPwm);
        servoPwmOut(0);
        i = data_buff[DIFF_GAIN_ADDR];
        if (sw == DATA_UP)
        {
            i++;
            if (i > 100)
                i = 100;
        }
        if (sw == DATA_DOWN)
        {
            i--;
            if (i < 0)
                i = 0;
        }
        data_buff[DIFF_GAIN_ADDR] = i;

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("04 Trace kd =%03d", i);
        LcdPosition(0, 1);
        courceOut();
        LcdPrintf("courceOut=%2d       ", cource);
        break;

    case 5:
        /* 直線走行目標速度設定 */
        servoPwmOut(0);

        i = data_buff[TRG_SPEED_ADDR];
        if (sw == DATA_UP)
        {
            i++;
            if (i > 127)
                i = 127;
        }
        if (sw == DATA_DOWN)
        {
            i--;
            if (i < 0)
                i = 0;
        }
        data_buff[TRG_SPEED_ADDR] = i;

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("05 Speed_S = %3d", i);
        LcdPosition(0, 1);
        LcdPrintf("Bar Angle = %4d", BAR_ANGLE);
        break;

    case 6:
        /* カーブ走行目標速度設定 */
        servoPwmOut(0);

        i = data_buff[CORNER_SPEED_ADDR];
        if (sw == DATA_UP)
        {
            i++;
            if (i > 100)
                i = 100;
        }
        if (sw == DATA_DOWN)
        {
            i--;
            if (i < 0)
                i = 0;
        }
        data_buff[CORNER_SPEED_ADDR] = i;

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("06 Speed_C = %3d", i);
        LcdPrintf("06 Speed_C = %3d", iEncoder);

        LcdPrintf("06                    ");

        // LcdPosition(0, 1);
        // LcdPrintf("Slope Angle=%4d", SLOPE_ANGLE);
        break;

    case 7:
        /* クランク進入目標速度設定 */
        servoPwmOut(0);

        i = data_buff[CRANK_SPEED_ADDR];
        if (sw == DATA_UP)
        {
            i++;
            if (i > 100)
                i = 100;
        }
        if (sw == DATA_DOWN)
        {
            i--;
            if (i < 0)
                i = 0;
        }
        data_buff[CRANK_SPEED_ADDR] = i;

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("07 Speed_CL =%3d", i);
        LcdPosition(0, 1);
        LcdPrintf("");
        break;

    case 8:
        /* レーンチェンジ進入目標速度設定 */
        servoPwmOut(0);

        i = data_buff[LANE_SPEED_ADDR];
        if (sw == DATA_UP)
        {
            i++;
            if (i > 100)
                i = 100;
        }
        if (sw == DATA_DOWN)
        {
            i--;
            if (i < 0)
                i = 0;
        }
        data_buff[LANE_SPEED_ADDR] = i;

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("08 Speed_RC =%3d", i);
        LcdPosition(0, 1);
        LcdPrintf("0x%X", dipsw_get());
        break;

    case 9:
        /* 設定パラメーター保存 */
        servoPwmOut(0);

        LcdPosition(0, 0);
        LcdPrintf("09 Parameter Set");

        // 設定値保存
        if (pushsw_get())
        {
            cnt1 = 0;
            do
            {
                // スイッチ情報取得
                sw = SwitchGetState();
                delay(500);
                if (cnt1 > 2000)
                {
                    // パラメータ保存
                    writeDataFlashParameter();
                    LcdPosition(0, 1);
                    LcdPrintf("Set Complete    ");
                }
                else
                {
                    LcdPosition(0, 1);
                    LcdPrintf("Setting Now     ");
                }
            } while (pushsw_get());
        }
        else
        {
            LcdPosition(0, 1);
            LcdPrintf("                ");
        }
        break;

    case 10:
        /* モーターテストドライバ基板確認 */
        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("10 Motor_Test   ");
        LcdPosition(0, 1);
        LcdPrintf("SW_1/2 ON!      ");

        if (pushsw_get() == 1 && sw == DATA_UP || pushsw_get() == 1 && DATA_DOWN)
        {
            do
            {
                // スイッチ情報取得
                sw = SwitchGetState();
                delay(200);
                LcdPosition(0, 1);
                LcdPrintf("SW_1/2 OFF!     ");
            } while (pushsw_get() == 1 && sw == DATA_UP || pushsw_get() == 1 && sw == DATA_DOWN);
            delay(15);
            cnt1 = 0;
            while (sw == 0)
            {
                // スイッチ情報取得
                sw = SwitchGetState();
                delay(200);

                servoPwmOut(0);

                // 基板テスト
                motor2_f(100, 100); // 前 （左,右）
                motor2_r(100, 100); // 後（左,右）

                servoPwmOut(100);

                LcdPosition(0, 0);
                LcdPrintf("Motor Test CW   ");
                LcdPosition(0, 1);
                LcdPrintf("Power = %4d%%   ", 100);
                delay(2000);

                motor2_f(0, 0); // 前 （左,右）
                motor2_r(0, 0); // 後（左,右）
                servoPwmOut(0);

                LcdPosition(0, 0);
                LcdPrintf("Motor Test STOP ");
                LcdPosition(0, 1);
                LcdPrintf("Power = %4d%%   ", 0);
                delay(1000);

                motor2_f(-100, -100); // 前 （左,右）
                motor2_r(-100, -100); // 後（左,右）
                servoPwmOut(-100);

                LcdPosition(0, 0);
                LcdPrintf("Motor Test CCW   ");
                LcdPosition(0, 1);
                LcdPrintf("Power = %4d%%   ", -100);
                delay(2000);

                motor2_f(0, 0); // 前 （左,右）
                motor2_r(0, 0); // 後（左,右）
                servoPwmOut(0);

                LcdPosition(0, 0);
                LcdPrintf("Motor Test STOP ");
                LcdPosition(0, 1);
                LcdPrintf("Power = %4d%%   ", 0);
                delay(1000);
                if (cnt1 > 3000)
                    break;
            }
            motor_f(0, 0); // 前 （左,右）
            motor_r(0, 0); // 後（左,右）
            do
            {
                // スイッチ情報取得
                sw = SwitchGetState();
                delay(200);
            } while (sw == 0x01 || sw == 0x02);
        }
        break;

    case 11:
        /* モーターテストドライバ基板確認 */

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("10 Motor_Test   ");
        LcdPosition(0, 1);
        LcdPrintf("SW_1 ON!        ");

        if (sw == 0x01 || sw == 0x02)
        {
            do
            {
                // スイッチ情報取得
                sw = SwitchGetState();
                delay(600);
                LcdPosition(0, 1);
                LcdPrintf("SW_1 OFF!       ");
            } while (sw == 0x01 || sw == 0x02);
            delay(15);
            i = 4;
            while (sw == 0 || i < 8)
            {
                // スイッチ情報取得
                sw = SwitchGetState();
                delay(1000);

                servoPwmOut(0);

                // 基板テスト
                if (i < 10)
                    i++;
                else
                    i = 4;
                motor2_f(i * 10, i * 10); // 前 （左,右）
                motor2_r(i * 10, i * 10); // 後（左,右）
                servoPwmOut(i * 10);

                LcdPosition(0, 0);
                LcdPrintf("Motor Test CW   ");
                LcdPosition(0, 1);
                LcdPrintf("Power = %4d%%   ", i * 10);
                delay(2000);

                motor2_f(0, 0); // 前 （左,右）
                motor2_r(0, 0); // 後 （左,右）
                servoPwmOut(0);

                LcdPosition(0, 0);
                LcdPrintf("Motor Test STOP ");
                LcdPosition(0, 1);
                LcdPrintf("Power = %4d%%   ", 0);
                delay(1000);

                motor2_f(-i * 10, -i * 10); // 前 （左,右）
                motor2_r(-i * 10, -i * 10); // 後（左,右）
                servoPwmOut(-i * 10);

                LcdPosition(0, 0);
                LcdPrintf("Motor Test CCW   ");
                LcdPosition(0, 1);
                LcdPrintf("Power = %4d%%   ", -i * 10);
                delay(2000);

                motor2_f(0, 0); // 前 （左,右）
                motor2_r(0, 0); // 後 （左,右）
                servoPwmOut(0);

                LcdPosition(0, 0);
                LcdPrintf("Motor Test STOP ");
                LcdPosition(0, 1);
                LcdPrintf("Power = %4d%%   ", 0);
                delay(1000);
            }
            motor_f(0, 0); // 前 （左,右）
            motor_r(0, 0); // 後 （左,右）
            do
            {
                // スイッチ情報取得
                sw = SwitchGetState();
                delay(200);
            } while (sw == 0x01 || sw == 0x02);
        }
        break;
    }
}

void mtTest()
{
    // 100% CW
    motor2_f(100, 100);
    motor2_r(100, 100);
    servoPwmOut(100);

    LcdPosition(0, 0);
    LcdPrintf("Motor Test CW   ");
    LcdPosition(0, 1);
    LcdPrintf("Power = %4d%%   ", 100);
    delay(2000);

    // 100% STOP
    motor2_f(0, 0);
    motor2_r(0, 0);
    servoPwmOut(0);

    LcdPosition(0, 0);
    LcdPrintf("Motor Test STOP  ");
    LcdPosition(0, 1);
    LcdPrintf("Power = %4d%%   ", -100);
    delay(2000);

    // 100% CCW
    motor2_f(-100, -100);
    motor2_r(-100, -100);
    servoPwmOut(-100);

    LcdPosition(0, 0);
    LcdPrintf("Motor Test CCW   ");
    LcdPosition(0, 1);
    LcdPrintf("Power = %4d%%   ", -100);
    delay(2000);

    // 100% STOP
    motor2_f(0, 0);
    motor2_r(0, 0);
    servoPwmOut(0);

    LcdPosition(0, 0);
    LcdPrintf("Motor Test STOP   ");
    LcdPosition(0, 1);
    LcdPrintf("Power = %4d%%   ", -100);
    delay(2000);
}

int angleStreatCheck(int i, int jide_angle)
{
    static int pattern11count = 0; /*11カウント用			 		 */
    if (abs(i) < jide_angle)
    {
        pattern11count++;
        if (pattern11count > 55)
        {
            pattern11count = 0;
            return 1;
        }
    }
    else
    {
        pattern11count = 0;
    }
    return 0;
}

int slopeCheck()
{
    static int count = 0; /*カウント用			 		 */
    // if (SLOPE_ANGLE > SLOPE_UP_START - 5) {
    //   count++;
    //   if (count > 15) {
    //     count = 0;
    //     return 1;
    //   }
    // } else {
    count = 0;
    // }
    return 0;
}

/**********************************************************************/
/**
 *	入力センサの初期化処理.
 */
void initSens(void)
{
    /* アナログセンサ */
    ad.useCh(SENS_A_UL); // CN8  9 D68
    ad.useCh(SENS_A_LL); // CN8  8 D67
    ad.useCh(SENS_A_CL); // CN8  6 D65
    ad.useCh(SENS_A_CC); // CN8  5 D64
    ad.useCh(SENS_A_CR); // CN8  4 D63
    ad.useCh(SENS_A_RR); // CN8  3 D62
    ad.useCh(SENS_A_UR); // CN8  2 D61
    ad.useCh(SENS_A_VR); // CN8  7 D66
    ad.start();
}

/**********************************************************************/
/**
 *	モータ関係 初期化.
 */
void initMotor(void)
{
    /* RRモータ */
    // defineで定義
    MOTOR_INIT_RR;

    /* RLモータ */
    // defineで定義
    MOTOR_INIT_RL;

    /* FRモータ */
    // defineで定義
    MOTOR_INIT_FR;

    /* FLモータ */
    // defineで定義
    MOTOR_INIT_FL;

    /* STモータ */
    // defineで定義
    MOTOR_INIT_ST;
}

/************************************************************************/
/**
 * SDカードファイルオープン.
 */
void SD_file_open(void)
{
    // 通信速度Hz(e6=10の6乗),microSDのCS1(pins_arduino.hで定義)
    if (!SD.begin(2.4e6, CS1))
    {
        // Serial.println("SD初期化に失敗しました");
        return;
    }

    int i = 0;
    microSD = SD.open("renban.txt", FILE_READ);
    if (microSD)
    {
        int length = microSD.available();
        if (length > 8)
            length = 8;
        microSD.read(filename, length);
        sscanf(filename, "%d", &i);
        if (i < 0 || i >= 99999)
            i = 0;
        microSD.close();
    }

    if (SD.exists("renban.txt"))
        SD.remove("renban.txt");

    microSD = SD.open("renban.txt", FILE_WRITE);
    if (microSD)
    {
        sprintf(filename, "%d", i + 1);
        microSD.println(filename);
        microSD.close();
    }

    sprintf(filename, "log%05d.csv", i + 1);
    microSD = SD.open(filename, FILE_WRITE);

    //  //通信速度Hz(e6=10の6乗),microSDのCS1(pins_arduino.hで定義)
    // 	if (SD.begin(2.4e6, CS1))
    // 	{
    //     //microSDへファイルオープン
    // 		microSD = SD.open("log.csv", FILE_WRITE);
    // 	}
}

/************************************************************************/
/**
 * ログをRAMに保存.
 */
void LOG_rec(void)
{
    // if (useBufferA)
    // {
    saveDataA[0][logCt] = digiSensLL << 2 | digiSensCC << 1 | digiSensRR;
    saveDataA[1][logCt] = iEncoder;
    saveDataA[2][logCt] = pattern;
    saveDataA[3][logCt] = cource;
    saveDataA[4][logCt] = getServoAngle();
    saveDataA[5][logCt] = iSetAngle;
    saveDataA[6][logCt] = anaSensCR_diff;
    saveDataA[7][logCt] = anaSensCL_diff;
    saveDataA[8][logCt] = anaSensCC_diff;
    saveDataA[9][logCt] = motor_buff_stare; //: PWMステアリング;
    saveDataA[10][logCt] = motor_buff_Fr;   //: PWM前右;
    saveDataA[11][logCt] = motor_buff_Fl;   //: PWM前左;
    saveDataA[12][logCt] = motor_buff_Rr;   //: PWM後右;
    saveDataA[13][logCt] = motor_buff_Rl;   //: PWM後左;
    saveDataA[14][logCt] = logCt;

    logCt++;
    // }
    // else
    // {
    //   saveDataB[0][logCt] = digiSensLL << 2 | digiSensCC << 1 | digiSensRR;
    //   saveDataB[1][logCt] = iEncoder;
    //   saveDataB[2][logCt] = pattern;
    //   saveDataB[3][logCt] = cource;
    //   saveDataB[4][logCt] = getServoAngle();
    //   saveDataB[5][logCt] = iAngle2;
    //   saveDataB[6][logCt] = anaSensCR_diff;
    //   saveDataB[7][logCt] = anaSensCL_diff;
    //   saveDataB[8][logCt] = anaSensCC_diff;
    //   saveDataB[9][logCt] = motor_buff_stare;  //: PWMステアリング;
    //   saveDataB[10][logCt] = motor_buff_Fr;    //: PWM前右;
    //   saveDataB[11][logCt] = motor_buff_Fl;    //: PWM前左;
    //   saveDataB[12][logCt] = motor_buff_Rr;    //: PWM後右;
    //   saveDataB[13][logCt] = motor_buff_Rl;    //: PWM後左;
    //   saveDataB[14][logCt] = logCt;
    // }

    if (logCt >= LOG_BUFF_SIZE)
    {
        logCt = 0;
        // isWriting = true;
    }

    // 一定数溜まったら書き込みトリガー
    // if (logCt >= LOG_BUFF_SIZE && !isWriting) {
    //   isWriting = true;
    //   logCt = 0;
    //   useBufferA = !useBufferA; // バッファ切り替え
    // }
}

/**********************************************************************/
/**
 *	ログ書き出し.
 */
void writeLog(void)
{
    sprintf(str, "d_0=%03d d_1=%03d d_2=%03d d_3=%03d d_4=%03d d_5=%03d "
                 "d_6=%03d d_7=%03d d_8=%03d d_9=%03d d_10=%03d d_11=%03d "
                 "d_12=%03d d_13=%03d d_14=%03d d_15=%03d fin \n\r",
            saveDataA[0][logRd],
            saveDataA[1][logRd],
            saveDataA[2][logRd],
            saveDataA[3][logRd],
            saveDataA[4][logRd],
            saveDataA[5][logRd],
            saveDataA[6][logRd],
            saveDataA[7][logRd],
            saveDataA[8][logRd],
            saveDataA[9][logRd],
            saveDataA[10][logRd],
            saveDataA[11][logRd],
            saveDataA[12][logRd],
            saveDataA[13][logRd],
            saveDataA[14][logRd],
            saveDataA[15][logRd]);
    microSD.print(str);
    // microSD.flush();
    logRd++;
    if (logRd >= LOG_BUFF_SIZE)
    {
        logRd = 0;
    }
}

/************************************************************************/
/**
 * SDカードファイルクローズ.
 */
void SD_file_close(void)
{
    microSD.close();
    servoPwmOut(0);
    CPU_LED_2 = OFF;
    CPU_LED_3 = OFF;
}

/************************************************************************/
/* サーボ角度取得                                                       */
/* 引数　 なし                                                          */
/* 戻り値 サーボ角度                                                */
/************************************************************************/
int getServoAngle(void)
{
    //	/* 検出移動平均算出用変数*/
    //	static signed int angleSum=0;			/* 移動平均値演算用変数 */
    //	static signed int angleBuf[8];			/* 移動平均値演算用変数 */
    //	static signed int angleCount=0;			/* 移動平均値演算用変数 */
    //	static signed int retAngle;			/* 移動平均値演算用変数 */

    /* 8点の速度の移動平均計算 angleBuf[8]*/
    //	angleSum =(angleSum + iAngle0) -angleBuf[angleCount];//合計に最も古いデータの値減算　最新データの値加算
    //	angleBuf[angleCount]=iAngle0; //最新のデータを代入
    //	angleCount++;//インデックスのインクリメント
    //	angleCount = angleCount & 0x07;//0→1　・・・　7→0→1・・
    //	retAngle = angleSum >> 3 ;// /8

    return Dig_M(BAR_ANGLE - iAngle0);
}

/************************************************************************/
/**
 * アングルフィルタ.
 */
short Dig_M(short angle)
{
    static short ang_buf[3];
    unsigned short MIN, MID, MAX, tmp;

    ang_buf[2] = ang_buf[1];
    ang_buf[1] = ang_buf[0];
    ang_buf[0] = angle;

    MIN = ang_buf[0];
    MID = ang_buf[1];
    MAX = ang_buf[2];

    if (MAX < MID)
    {
        tmp = MAX;
        MAX = MID;
        MID = tmp;
    }
    if (MAX < MIN)
    {
        tmp = MAX;
        MAX = MIN;
        MIN = tmp;
    }
    if (MID < MIN)
    {
        tmp = MIN;
        MIN = MID;
        MID = tmp;
    }

    return MID;
}

/************************************************************************/
/**
 * アングル速度.
 */
short Ang(void)
{
    static short Ang_B;
    short i;
    short ret;

    i = getServoAngle();

    if (i < 0)
        i * -1;
    ret = i - Ang_B;
    Ang_B = i;

    return ret;
}

/************************************************************************/
/**
 * トレース時のモーター制御.
 */
// void PDtrace_Control(short Dig, char boost_trig, short SP) {
void PDtrace_Control(short Dig, short SP, char boost_trig)
{

    static int LEnc_b; // D制御で使用する前回の値

    long i, iP, iD;
    int PWM;
    int DEF_PWM;

    // char BrakeS = 0;//未使用
    int ANG_Dr; // 未使用

    const char r1 = 70; // 外輪側後輪倍率 //80
    const char r2 = 90; // 内輪側前輪倍率 //90
    const char r3 = 60; // 内輪側後輪倍率 //70

    const char Dr1 = 60; // 外輪側後輪減衰倍率 //60
    const char Dr2 = 10; // 内輪側前輪減衰倍率 //30
    const char Dr3 = 20; // 内輪側後輪減衰倍率 //70
    const char DrN = 5;  // 外輪側前輪減衰倍率 //10

    int RR, RF, LR, LF;

    const int P_gain = 14;
    const int D_gain = 13;

    /*	const int P_gain = 10;
    const int D_gain = 15;*/

    const int Ofset = 20;
    const int F_Ofset = 40;      // 30
    const int F_BrakeRatio = 80; // 90
    const int R_BrakeRatio = 70; // 70
    const int Gain = 30;

    // ブースト
    //	if (boost_trig) {
    //		iP = (speed_pulse[80] - speed_pulse[iE_value[iEncoder]]) * P_gain;
    //		speed_target = 80;
    //	}
    //	else {
    // 速度制御　P制御 speed_pulse[SP]:目標値   speed_pulse[iE_value[iEncoder]]:現在速度
    iP = (speed_pulse[SP] - speed_pulse[iE_value[iEncoder]]) * P_gain;
    //		speed_target = SP;
    //	}
    // 速度制御　D制御　LEnc_b:前回取得の現在速度
    iD = (speed_pulse[iE_value[iEncoder]] - LEnc_b) * D_gain; // 未使用

    PWM = (iP - iD) * Gain / 100;
    PWM += Ofset;

    if (Angle_D > 0)
        ANG_Dr = 0;
    else
        ANG_Dr = Angle_D;

    if (PWM > 100)
        PWM = 100;
    if (PWM < -100)
        PWM = -100;

    // if ((ANG_Dr < -15) && (pattern == 11)) {
    //   //if(PWM > 0)PWM = PWM * 15 / 100;
    //   if (PWM > 0) PWM = 1;
    // }

    // 停止処理
    if (pattern >= 201)
    {
        if (PWM < -30)
            PWM = -30;
        if (PWM > 20)
            PWM = 20;
        // BrakeS = 1;
    }

    // Dig:ボリューム値
    i = abs((Dig * 3) / 10); // 角度に変換
    if (i < 0)
        i *= -1;

    DEF_PWM = PWM * i / 40;
    if (DEF_PWM > 100)
        DEF_PWM = 100;
    if (DEF_PWM < -100)
        DEF_PWM = -100;

// あえて内輪差逆
#if 0
        // 浅い右カーブ
        if (Dig < -7) {
          //アクセル
          if (PWM > 0) {
            //	motor_f(0, 0);			// 前(左, 右)
            //	motor_r(1, 0);			// 後(左, 右)
            LF = PWM - (DEF_PWM * DrN / 100);
            //	LR = (PWM*r1/100) - (DEF_PWM*Dr1/100);
            LR = 10;
            RF = (PWM * r2 / 100) - (DEF_PWM * Dr2 / 100);
            RR = (PWM * r3 / 100) - (DEF_PWM * Dr3 / 100);
          }  //ブレーキ
          else {
            // 深い右カーブ
            if (Dig < -33) {
              //RR = PWM;
              //RF = PWM*r1/100;
              //LR = PWM*r2/100;
              //LF = PWM*r3/100;
              //	if (iEncoder > speed_target + 5) {
              //		RR = -20;
              //		RF = -20;
              //		LR = -20;
              //		LF = -20;
              //	}
              //	else {
              RR = 10;
              RF = 10;
              LR = 10;
              LF = 10;
              //	}
            } else {
              RR = -R_Brake * Inside_ofset / 100;
              RF = -F_Brake * Inside_ofset / 100;
              LR = -R_Brake;
              LF = -F_Brake;
            }
    
            if (PWM < -50) {
              //	motor_f(0, 0);			// 前(左, 右)
              //	motor_r(0, 0);			// 後(左, 右)
            } else if (PWM < 0) {
              //	motor_f(0, 0);			// 前(左, 右)
              //	motor_r(1, 1);			// 後(左, 右)
              RR = 10;
              LR = 10;
            }
          }
        }
        // 浅い左カーブ
        else if (Dig > 7) {
          if (PWM > 0) {
            //	motor_f(0, 0);			// 前(左, 右)
            //	motor_r(0, 1);			// 後(左, 右)
            RF = PWM - (DEF_PWM * DrN / 100);
            //	RR = (PWM*r1/100) - (DEF_PWM*Dr1/100);
            RR = 10;
            LF = (PWM * r2 / 100) - (DEF_PWM * Dr2 / 100);
            LR = (PWM * r3 / 100) - (DEF_PWM * Dr3 / 100);
          } else {
            // 深い左カーブ
            if (Dig > 33) {
              //LR = PWM;
              //LF = PWM*r1/100;
              //RR = PWM*r2/100;
              //RF = PWM*r3/100;
              //	if (iEncoder > speed_target + 5) {
              //		RR = -20;
              //		RF = -20;
              //		LR = -20;
              //		LF = -20;
              //	}
              //	else {
              LR = 10;
              LF = 10;
              RR = 10;
              RF = 10;
              //	}
            } else {
              RR = -R_Brake;
              RF = -F_Brake;
              LR = -R_Brake * Inside_ofset / 100;
              LF = -F_Brake * Inside_ofset / 100;
            }
            if (PWM < -50) {
              //		motor_f(0, 0);			// 前(左, 右)
              //		motor_r(0, 0);			// 後(左, 右)
            } else if (PWM < 0) {
              //		motor_f(0, 0);			// 前(左, 右)
              //		motor_r(1, 1);			// 後(左, 右)
              LR = 10;
              RR = 10;
            }
          }
        } else {
          //	motor_f(0, 0);			// 前(左, 右)
          //	motor_r(0, 0);			// 後(左, 右)
          if (PWM > 0) {
            RF = PWM + F_Ofset;
            RR = PWM;
            LF = PWM + F_Ofset;
            LR = PWM;
          } else {
            LR = PWM * R_BrakeRatio / 100;
            LF = PWM * F_BrakeRatio / 100;
            RR = PWM * R_BrakeRatio / 100;
            RF = PWM * F_BrakeRatio / 100;
          }
        }

#else

    // 浅い右カーブ
    if (Dig < -7)
    {
        if (PWM > 0)
        {
            //	motor_f(0, 0);			// 前(左, 右)
            //	motor_r(0, 1);			// 後(左, 右)
            RF = PWM - (DEF_PWM * DrN / 100);
            //	RR = (PWM*r1/100) - (DEF_PWM*Dr1/100);
            RR = 10;
            LF = (PWM * r2 / 100) - (DEF_PWM * Dr2 / 100);
            LR = (PWM * r3 / 100) - (DEF_PWM * Dr3 / 100);
        }
        else
        {
            // 深い右カーブ
            if (Dig < -35)
            {
                // RR = PWM;
                // RF = PWM*r1/100;
                // LR = PWM*r2/100;
                // LF = PWM*r3/100;
                //	if (iEncoder > speed_target + 5) {
                //		RR = -20;
                //		RF = -20;
                //		LR = -20;
                //		LF = -20;
                //	}
                //	else {
                RR = 10;
                RF = 10;
                LR = 10;
                LF = 10;
                //	}
            }
            else
            {
                RR = -R_Brake;
                RF = -F_Brake;
                LR = -R_Brake * Inside_ofset / 100;
                LF = -F_Brake * Inside_ofset / 100;
            }
            if (PWM < -50)
            {
                //	motor_f(0, 0);			// 前(左, 右)
                //	motor_r(0, 0);			// 後(左, 右)
            }
            else if (PWM < 0)
            {
                //	motor_f(0, 0);			// 前(左, 右)
                //	motor_r(1, 1);			// 後(左, 右)
                RR = 10;
                LR = 10;
            }
        }
    }
    // 浅い左カーブ
    else if (Dig > 7)
    {
        if (PWM > 0)
        {
            //	motor_f(0, 0);			// 前(左, 右)
            //	motor_r(1, 0);			// 後(左, 右)
            LF = PWM - (DEF_PWM * DrN / 100);
            //	LR = (PWM*r1/100) - (DEF_PWM*Dr1/100);
            LR = 10;
            RF = (PWM * r2 / 100) - (DEF_PWM * Dr2 / 100);
            RR = (PWM * r3 / 100) - (DEF_PWM * Dr3 / 100);
        }
        else
        {
            // 深い左カーブ
            if (Dig > 35)
            {
                // LR = PWM;
                // LF = PWM*r1/100;
                // RR = PWM*r2/100;
                // RF = PWM*r3/100;
                //	if (iEncoder > speed_target + 5) {
                //		RR = -20;
                //		RF = -20;
                //		LR = -20;
                //		LF = -20;
                //	}
                //	else {
                LR = 10;
                LF = 10;
                RR = 10;
                RF = 10;
                //	}
            }
            else
            {
                RR = -R_Brake * Inside_ofset / 100;
                RF = -F_Brake * Inside_ofset / 100;
                LR = -R_Brake;
                LF = -F_Brake;
            }
            if (PWM < -50)
            {
                //		motor_f(0, 0);			// 前(左, 右)
                //		motor_r(0, 0);			// 後(左, 右)
            }
            else if (PWM < 0)
            {
                //		motor_f(0, 0);			// 前(左, 右)
                //		motor_r(1, 1);			// 後(左, 右)
                LR = 10;
                RR = 10;
            }
        }
    }
    else
    {
        //	motor_f(0, 0);			// 前(左, 右)
        //	motor_r(0, 0);			// 後(左, 右)
        if (PWM > 0)
        {
            RF = PWM + F_Ofset;
            RR = PWM;
            LF = PWM + F_Ofset;
            LR = PWM;
        }
        else
        {
            LR = PWM * R_BrakeRatio / 100;
            LF = PWM * F_BrakeRatio / 100;
            RR = PWM * R_BrakeRatio / 100;
            RF = PWM * F_BrakeRatio / 100;
        }
    }
#endif

    motor_f(LF, RF);
    motor_r(LR, RR);
    if (PD_trig)
    {
        LEnc_b = speed_pulse[iE_value[iEncoder]];
        PD_trig = 0;
    }
}
