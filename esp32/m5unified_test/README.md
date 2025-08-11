# ESP32S3R Hello World Test

ESP-IDF環境の基本動作確認用テストプロジェクト

## ビルド手順

```bash
# 1. ESP-IDF環境の準備
source ~/esp/esp-idf/export.sh

# 2. プロジェクトディレクトリへ移動
cd /home/yakatano/work/isolation-sphere/esp32/test_hello_world

# 3. ターゲット設定（初回のみ）
idf.py set-target esp32s3

# 4. ビルド
idf.py build

# 5. フラッシュ書き込み
idf.py -p /dev/ttyUSB0 flash

# 6. シリアルモニター
idf.py -p /dev/ttyUSB0 monitor
```

## 確認項目

このテストプログラムは以下を確認します：

1. **チップ情報**
   - ESP32S3の認識
   - CPU コア数
   - 機能（WiFi, BT, BLE）
   - フラッシュサイズ

2. **メモリ情報**
   - 内部RAM容量と空き容量
   - PSRAM認識と容量（8MB期待）
   - メモリアロケーションテスト

3. **マルチコア動作**
   - Core 0でのタスク実行
   - Core 1でのタスク実行
   - タスク間の並行動作

4. **システム安定性**
   - 10回のメインループ実行
   - メモリリーク確認
   - ヒープ使用量監視

## 期待される出力

```
I (xxx) HELLO_TEST: ========================================
I (xxx) HELLO_TEST: ESP32S3R Isolation Sphere Test
I (xxx) HELLO_TEST: ========================================
I (xxx) HELLO_TEST: Chip: esp32s3
I (xxx) HELLO_TEST: Cores: 2
I (xxx) HELLO_TEST: Features: WiFi BT BLE
I (xxx) HELLO_TEST: PSRAM:
I (xxx) HELLO_TEST:   Status: Initialized ✓
I (xxx) HELLO_TEST:   Total: 8388608 bytes (8.00 MB)
I (xxx) HELLO_TEST:   Test: 1MB allocation successful ✓
```

## トラブルシューティング

### PSRAMが認識されない場合
```bash
idf.py menuconfig
# Component config → ESP PSRAM → Support for external, SPI-connected RAM を有効化
```

### USB デバイスが見つからない場合
```bash
# 権限の確認
ls -l /dev/ttyUSB*
sudo usermod -a -G dialout $USER
# ログアウト・ログイン後に再試行
```

### ビルドエラーの場合
```bash
# クリーンビルド
idf.py fullclean
idf.py build
```

## 成功基準

- [ ] ビルドが成功する
- [ ] フラッシュ書き込みが成功する
- [ ] ESP32S3として認識される
- [ ] PSRAMが8MB認識される
- [ ] 両コアでタスクが動作する
- [ ] 10回のループが完了する
- [ ] メモリリークがない

## 次のステップ

このテストが成功したら、以下のテストに進む：
1. I2C通信テスト（BNO055用）
2. SPI通信テスト（ディスプレイ用）
3. WiFi接続テスト
4. micro-ROS通信テスト