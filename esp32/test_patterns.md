# テストパターン表示機能仕様書

## 概要
ROS2通信が利用できない場合やデバッグ時に使用するテストパターン表示機能の詳細仕様

## 1. XYZ軸表示パターン

### 仕様
- **目的**: 3D座標系の軸方向を視覚化
- **表示内容**:
  - X軸: 赤色の点列（東西方向）
  - Y軸: 緑色の点列（南北方向）
  - Z軸: 青色の点列（上下方向）
- **実装**:
```c
void draw_axis_pattern(uint8_t led_buffer[800][3]) {
    // 各LEDの球面座標から軸との距離を計算
    for (int i = 0; i < 800; i++) {
        float dist_to_x = calculate_distance_to_x_axis(led_coords[i]);
        float dist_to_y = calculate_distance_to_y_axis(led_coords[i]);
        float dist_to_z = calculate_distance_to_z_axis(led_coords[i]);
        
        // 軸に近いLEDを着色
        if (dist_to_x < AXIS_THRESHOLD) {
            led_buffer[i][0] = 255; // Red
            led_buffer[i][1] = 0;
            led_buffer[i][2] = 0;
        } else if (dist_to_y < AXIS_THRESHOLD) {
            led_buffer[i][0] = 0;
            led_buffer[i][1] = 255; // Green
            led_buffer[i][2] = 0;
        } else if (dist_to_z < AXIS_THRESHOLD) {
            led_buffer[i][0] = 0;
            led_buffer[i][1] = 0;
            led_buffer[i][2] = 255; // Blue
        }
    }
}
```

## 2. 文字表示パターン

### 仕様
- **目的**: デバッグ情報やステータス表示
- **表示可能文字**: 
  - "ISOLATION SPHERE"
  - "CALIBRATING..."
  - "NO SIGNAL"
  - IPアドレス
  - フレームレート
- **実装方式**:
  - 球面上に文字をマッピング
  - スクロール表示対応
  - フォント: 5x7ドットマトリックス

```c
typedef struct {
    char text[32];
    float scroll_position;
    uint8_t color[3];
} text_display_t;

void draw_text_pattern(uint8_t led_buffer[800][3], text_display_t* text) {
    // 球面の赤道付近に文字を配置
    for (int i = 0; i < strlen(text->text); i++) {
        draw_character(led_buffer, text->text[i], 
                      text->scroll_position + i * CHAR_WIDTH,
                      text->color);
    }
    text->scroll_position += SCROLL_SPEED;
}
```

## 3. グラデーションパターン

### 仕様
- **目的**: 色の連続性確認、視覚的効果
- **パターン種類**:
  1. **虹色回転**: HSV色空間で連続変化
  2. **単色グラデーション**: 明度変化
  3. **波パターン**: サイン波による明滅

```c
void draw_gradient_pattern(uint8_t led_buffer[800][3], float time) {
    for (int i = 0; i < 800; i++) {
        // 球面座標から色相を計算
        float hue = (led_coords[i].theta + time * ROTATION_SPEED) / (2 * M_PI);
        float saturation = 1.0;
        float value = 0.5 + 0.5 * sin(led_coords[i].phi);
        
        hsv_to_rgb(hue, saturation, value, 
                  &led_buffer[i][0], &led_buffer[i][1], &led_buffer[i][2]);
    }
}
```

## 4. 緯度経度グリッドパターン

### 仕様
- **目的**: 球面座標系の可視化、LED配置確認
- **表示内容**:
  - 緯線: 10度間隔（白色）
  - 経線: 15度間隔（白色）
  - 赤道: 明るい白色
  - 子午線: 明るい白色

```c
void draw_grid_pattern(uint8_t led_buffer[800][3]) {
    for (int i = 0; i < 800; i++) {
        float theta = led_coords[i].theta;
        float phi = led_coords[i].phi;
        
        // 緯線チェック（10度ごと）
        if (fmod(phi * 180 / M_PI, 10.0) < GRID_WIDTH) {
            led_buffer[i][0] = led_buffer[i][1] = led_buffer[i][2] = 128;
        }
        
        // 経線チェック（15度ごと）
        if (fmod(theta * 180 / M_PI, 15.0) < GRID_WIDTH) {
            led_buffer[i][0] = led_buffer[i][1] = led_buffer[i][2] = 128;
        }
        
        // 赤道と子午線は明るく
        if (fabs(phi - M_PI/2) < GRID_WIDTH || fabs(theta) < GRID_WIDTH) {
            led_buffer[i][0] = led_buffer[i][1] = led_buffer[i][2] = 255;
        }
    }
}
```

## 5. パターン切り替え機能

### 制御方法
- **自動切り替え**: 10秒ごとに次のパターンへ
- **UIコマンド**: `/ui/command`トピックで指定
- **ボタン操作**: GPIO入力で切り替え（オプション）

```c
typedef enum {
    PATTERN_AXIS = 0,
    PATTERN_TEXT,
    PATTERN_GRADIENT,
    PATTERN_GRID,
    PATTERN_LAST_IMAGE,
    PATTERN_COUNT
} test_pattern_t;

typedef struct {
    test_pattern_t current_pattern;
    float animation_time;
    bool auto_switch;
    uint32_t switch_interval_ms;
} pattern_controller_t;

void update_test_pattern(pattern_controller_t* ctrl, uint8_t led_buffer[800][3]) {
    // IMU姿勢補正を適用
    quaternion_t orientation = get_current_orientation();
    
    switch(ctrl->current_pattern) {
        case PATTERN_AXIS:
            draw_axis_pattern(led_buffer);
            break;
        case PATTERN_TEXT:
            draw_text_pattern(led_buffer, &text_config);
            break;
        case PATTERN_GRADIENT:
            draw_gradient_pattern(led_buffer, ctrl->animation_time);
            break;
        case PATTERN_GRID:
            draw_grid_pattern(led_buffer);
            break;
        case PATTERN_LAST_IMAGE:
            // 最後の有効画像を表示
            memcpy(led_buffer, last_valid_frame, sizeof(last_valid_frame));
            break;
    }
    
    // 姿勢補正適用
    apply_orientation_correction(led_buffer, orientation);
    
    // 自動切り替えチェック
    if (ctrl->auto_switch && 
        (esp_timer_get_time() - ctrl->last_switch) > ctrl->switch_interval_ms * 1000) {
        ctrl->current_pattern = (ctrl->current_pattern + 1) % PATTERN_COUNT;
        ctrl->last_switch = esp_timer_get_time();
    }
}
```

## 6. パフォーマンス要件

| パターン | 生成時間目標 | メモリ使用量 |
|---------|------------|-----------|
| XYZ軸 | 1ms以内 | 2.4KB |
| 文字表示 | 2ms以内 | 3KB + フォントデータ |
| グラデーション | 3ms以内 | 2.4KB |
| グリッド | 1ms以内 | 2.4KB |

## 7. テストケース

### 基本動作確認
- [ ] 各パターンが正常に表示される
- [ ] パターン切り替えが動作する
- [ ] IMU姿勢補正が全パターンに適用される

### エラー処理
- [ ] LED座標データがない場合のフォールバック
- [ ] メモリ不足時の動作
- [ ] 無効なパターン番号の処理

### 性能測定
- [ ] 各パターンの生成時間測定
- [ ] 30fps維持確認
- [ ] CPU使用率測定

## 8. 拡張案

### 将来の追加パターン
1. **3Dモデル表示**: 簡単な立体形状（立方体、球など）
2. **アニメーション**: 回転、脈動、波紋効果
3. **センサー値可視化**: IMU値をリアルタイム表示
4. **QRコード**: WiFi設定情報等の表示

### インタラクティブ機能
1. **加速度センサー連動**: 振ると色が変わる
2. **音声反応**: マイク入力に応じた表示（要追加HW）
3. **温度表示**: 色で温度を表現

---
作成日: 2024年
バージョン: 1.0.0