// ===================== モータードライバ設定 =====================
// DRV8833 No.1 (左側モーター用)
const int Motor_L1_F_PIN = 12; // 左前輪モーター Forward
const int Motor_L1_B_PIN = 13; // 左前輪モーター Backward
const int Motor_L2_F_PIN = 14; // 左後輪モーター Forward
const int Motor_L2_B_PIN = 27; // 左後輪モーター Backward
const int CONTROL_PIN_15 = 15; // 左側DRV8833のnSLEEP/STBY

// DRV8833 No.2 (右側モーター用)
const int Motor_R1_F_PIN = 32; // 右前輪モーター Forward
const int Motor_R1_B_PIN = 33; // 右前輪モーター Backward
const int Motor_R2_F_PIN = 26; // 右後輪モーター Forward
const int Motor_R2_B_PIN = 25; // 右後輪モーター Backward
const int CONTROL_PIN_2  = 2;  // 右側DRV8833のnSLEEP/STBY

// ===================== 動作状態定義 =====================
enum RobotState {
  STOP,
  MANUAL_FORWARD,
  MANUAL_BACKWARD,
  LINE_TRACE
};

RobotState currentState = LINE_TRACE; // 電源ON直後はライントレースモードにしておく
int Speed = 30;                      // 手動前進・後退時の基本速度（0〜255）

// ===================== PWM 設定 =====================
const int PWM_FREQ       = 20000; // 20 kHz
const int PWM_RESOLUTION = 8;     // 8-bit -> 0..255
const int MIN_DUTY       = 180;   // モータが動き出す下限（0..255）
const int MAX_DUTY       = 252;   // 通常走行時のDuty上限（0..255）

// ===================== フォトリフレクタ設定 =====================
// 必要に応じてピン番号は変更
#define PR_1_PIN 23
#define PR_2_PIN 22
#define PR_3_PIN 21
#define PR_4_PIN 19
#define PR_5_PIN 18
#define PR_6_PIN 5
#define PR_7_PIN 17
#define PR_8_PIN 16

const int sensor_pins[8] = {
  PR_1_PIN, PR_2_PIN, PR_3_PIN, PR_4_PIN,
  PR_5_PIN, PR_6_PIN, PR_7_PIN, PR_8_PIN
};

// センサの重み（左がマイナス、右がプラス）
// センサの並びが [一番左 ... 一番右] になるように配線すると扱いやすい
const int sensor_weights[8] = {-30, -2, -1, 1, 2, 30, 0, 0};

// ★ 黒線ライントレース用の論理設定 ★
// センサ出力が「黒で0, 白で1」の場合 → 1 にする（反転して黒を1として扱う）
// センサ出力が「黒で1, 白で0」の場合 → 0 のままでOK
const int INVERT_SENSOR_LOGIC = 0;  // ← モジュールに合わせて 0 / 1 を切り替え

// ===================== ライントレース用パラメータ =====================
const int   BASE_SPEED = 100;   // ライントレース時の基本速度（0〜255）
// より精密なPIDゲイン設定
const float KP         = 900;  // 比例ゲイン（応答性向上）
const float KI         = 0;   // 積分ゲイン（定常偏差除去）
const float KD         = 0;  // 微分ゲイン（オーバーシュート抑制）

// ラインを見失った時の捜索用
const int LOST_LINE_TURN_SPEED = 100;  // 探すときの旋回速度（やや控えめ）

// 内部状態（PID用）
float lastError      = 0.0;
float integralError  = 0.0;   // 積分項
float prevError      = 0.0;   // 前回誤差（微分計算用）
unsigned long lastMs = 0;      // 前回処理時刻
const float I_TERM_LIMIT = 150.0f; // I項の出力上限（過積分防止）
const float I_DECAY = 0.95f;       // 積分項の減衰係数（安定性向上）
const float ERROR_THRESHOLD = 0.1f; // 微小誤差の無視閾値

// ===================== モーター制御関数 =====================
// pinF: 正転用ピン / pinB: 逆転用ピン
// speedValue: -255〜+255（+で正転、-で逆転、0で停止）
void setMotorSpeed(int pinF, int pinB, int speedValue) {
  if (speedValue == 0) {
    ledcWrite(pinF, 0);
    ledcWrite(pinB, 0);
    return;
  }

  int absSpeed = constrain(abs(speedValue), 0, 255);

  int duty = MIN_DUTY + (absSpeed * (MAX_DUTY - MIN_DUTY)) / 255;
  duty = constrain(duty, MIN_DUTY, MAX_DUTY);

  if (speedValue > 0) {
    // 正転
    ledcWrite(pinF, duty);
    ledcWrite(pinB, 0);
  } else {
    // 逆転
    ledcWrite(pinF, 0);
    ledcWrite(pinB, duty);
  }
}

// ===================== ライン誤差読み取り関数 =====================
bool readLineError(float &errorOut) {
  int sensor_value[8];
  int sumActive = 0;
  int weightedSum = 0;

  Serial.print("RAW: [");
  for (int i = 0; i < 8; i++) {
    int raw = digitalRead(sensor_pins[i]);
    Serial.print(raw);
    if (i < 7) Serial.print(",");
  }
  Serial.print("] PROC: [");
  
  for (int i = 0; i < 8; i++) {
    int v = digitalRead(sensor_pins[i]);
    if (INVERT_SENSOR_LOGIC) {
      v = 1 - v;  // 論理反転（黒線を1として扱う）
    }
    sensor_value[i] = v;

    Serial.print(v);
    if (i < 7) Serial.print(",");

    if (v == 1) {
      sumActive += 1;
      weightedSum += sensor_weights[i];
    }
  }
  Serial.print("]  ");

  if (sumActive == 0) {
    // 1個もラインを検出していない → ラインロスト扱い
    Serial.println("No line detected");
    return false;
  }

  // センサ重みの平均値 → error
  // 左に寄ると負の値、右に寄ると正の値になる
  errorOut = (float)weightedSum / (float)sumActive;
  Serial.print("error = ");
  Serial.println(errorOut);
  return true;
}

// ===================== ライントレース1ステップ処理 =====================
void lineTraceStep() {
  float error;
  bool hasLine = readLineError(error);

  int leftSpeed  = 0;
  int rightSpeed = 0;

  // 経過時間 dt（秒）を計算
  unsigned long now = millis();
  float dt = (lastMs == 0) ? 0.02f : (float)(now - lastMs) / 1000.0f; // 初回は目安の20ms
  if (dt <= 0.0f) dt = 0.02f;

  if (hasLine) {
    // より精密なPID制御
    
    // 微小誤差の無視（ノイズ除去）
    if (abs(error) < ERROR_THRESHOLD) {
      error = 0.0f;
    }
    
    // 積分項：ライン見えている時のみ蓄積、減衰付き
    integralError = integralError * I_DECAY + error * dt;
    
    // I項の出力制限（ワインドアップ防止）
    if (KI > 0.0f) {
      float iOut = KI * integralError;
      if (iOut > I_TERM_LIMIT)       integralError = I_TERM_LIMIT / KI;
      else if (iOut < -I_TERM_LIMIT) integralError = -I_TERM_LIMIT / KI;
    }

    // PID項の計算
    float P = KP * error;
    float I = KI * integralError;
    float D = (dt > 0.001f) ? KD * (error - prevError) / dt : 0.0f;
    
    // ローパスフィルタで微分項のノイズを除去
    static float filteredD = 0.0f;
    filteredD = 0.8f * filteredD + 0.2f * D;
    
    float steer = P + I + filteredD;

    // ステアリング量の制限（急激な変化を抑制）
    steer = constrain(steer*1.5, -BASE_SPEED * 2.5f, BASE_SPEED * 2.5f);

    // 左右のモーター速度に反映
    leftSpeed  = BASE_SPEED - (int)steer;
    rightSpeed = BASE_SPEED + (int)steer;

    // 速度を安全範囲に制限
    leftSpeed  = constrain(leftSpeed,  -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    prevError = error;
    lastError = error;
  } else {
    // ラインを見失ったとき：最後に見えていた方向に少し旋回して探す
    // 過積分を避けるため、ラインロスト時は積分をゆっくり減衰/リセット
    integralError = 0.0f;
    if (lastError >= 0) {
      // ラインは右側にあった → 右回りでぐるっと探す
      leftSpeed  =  LOST_LINE_TURN_SPEED;
      rightSpeed = -LOST_LINE_TURN_SPEED;
    } else {
      // ラインは左側にあった → 左回りで探す
      leftSpeed  = -LOST_LINE_TURN_SPEED;
      rightSpeed =  LOST_LINE_TURN_SPEED;
    }
  }

  lastMs = now;
  Serial.print("leftSpeed = ");
  Serial.print(leftSpeed);
  Serial.print(", rightSpeed = ");
  Serial.println(rightSpeed);

  // 計算した速度で4輪を駆動（左右は同じ速度を使う）
  setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, -leftSpeed);
  setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, leftSpeed);
  setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, rightSpeed);
  setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, rightSpeed);
}

// ===================== setup =====================
void setup() {
  // モーター制御ピンを出力設定
  pinMode(Motor_L1_F_PIN, OUTPUT);
  pinMode(Motor_L1_B_PIN, OUTPUT);
  pinMode(Motor_L2_F_PIN, OUTPUT);
  pinMode(Motor_L2_B_PIN, OUTPUT);
  pinMode(Motor_R1_F_PIN, OUTPUT);
  pinMode(Motor_R1_B_PIN, OUTPUT);
  pinMode(Motor_R2_F_PIN, OUTPUT);
  pinMode(Motor_R2_B_PIN, OUTPUT);

  // ESP32 PWM 初期化（ピン直指定の書き方）
  int motorPins[] = {
    Motor_L1_F_PIN, Motor_L1_B_PIN,
    Motor_L2_F_PIN, Motor_L2_B_PIN,
    Motor_R1_F_PIN, Motor_R1_B_PIN,
    Motor_R2_F_PIN, Motor_R2_B_PIN
  };
  for (int i = 0; i < 8; ++i) {
    int p = motorPins[i];
    ledcAttach(p, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(p, 0); // 初期は停止
  }

  // フォトリフレクタのピンを入力設定
  for (int i = 0; i < 8; i++) {
    // オープンコレクタ型なら INPUT_PULLUP の方が良い場合もある
    pinMode(sensor_pins[i], INPUT_PULLUP);
  }

  // DRV8833 の nSLEEP/STBY を有効化
  pinMode(CONTROL_PIN_2, OUTPUT);
  pinMode(CONTROL_PIN_15, OUTPUT);
  digitalWrite(CONTROL_PIN_2, HIGH);
  digitalWrite(CONTROL_PIN_15, HIGH);

  Serial.begin(115200);
  Serial.println("4WD Line Tracer with DRV8833 + 8 Sensors");
  Serial.println("Commands:");
  Serial.println("  0 / s : STOP");
  Serial.println("  1 / f : MANUAL FORWARD");
  Serial.println("  2 / b : MANUAL BACKWARD");
  Serial.println("  3 / l : LINE TRACE (black line)");

  // PID初期化
  lastMs = millis();
  lastError = 0.0f;
  prevError = 0.0f;
  integralError = 0.0f;
}

// ===================== loop（メイン制御） =====================
void loop() {
  // シリアルポートからコマンドを受信して状態を変更する
  if (Serial.available() > 0) {
    char command = Serial.read();
    // 改行文字は無視
    if (command != '\r' && command != '\n') {
      switch (command) {
        case '0':
        case 's':
          currentState = STOP;
          Serial.println("STATE: STOP");
          break;
        case '1':
        case 'f':
          currentState = MANUAL_FORWARD;
          Serial.println("STATE: MANUAL_FORWARD");
          break;
        case '2':
        case 'b':
          currentState = MANUAL_BACKWARD;
          Serial.println("STATE: MANUAL_BACKWARD");
          break;
        case '3':
        case 'l':
          currentState = LINE_TRACE;
          Serial.println("STATE: LINE_TRACE");
          break;
        default:
          Serial.println("Invalid command");
          break;
      }
    }
  }

  // 現在の状態に基づいてモーターを制御
  switch (currentState) {
    case STOP:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, 0);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, 0);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, 0);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, 0);
      break;

    case MANUAL_FORWARD:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, -Speed);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, Speed);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, Speed);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, Speed);
      break;

    case MANUAL_BACKWARD:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, Speed);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, -Speed);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, -Speed);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, -Speed);
      break;

    case LINE_TRACE:
      lineTraceStep();
      break;
  }

  // ループ周期（10〜20msくらいが目安）
  delay(20);
}
