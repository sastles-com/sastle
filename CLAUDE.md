# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the "isolation-sphere" project, currently in initial setup phase. The repository contains only a minimal package-lock.json file.

isolatio-sphereは，球体ディスプレイ（ガジェット）で以下の特徴を持つ
- 直径110mmの球体ディスプレイ
- 球体表面に800個のLED（WS2812C-2020）が配置されている
- LEDは４つのLEDストリップに分割され，描画信号を送信する（800をシリアルに送信すると送信コストがかかり更新レートが下がるため）
- コントロールユニット: raspi
  - raspberry pi（OSはubuntu 22.04）で実行
  - 動画を読み込んで再生する
    - 動画の解像度は320x160で固定（全ての動画ファイルはraspi内で変換）
    - 動画のfpsは10Hz
  - UI機能
    - FASTAPI経由でwebアプリを作成し，各種のUIを実現
      - 動画制御機能：upload，データベース化，再生・停止など
      - LED制御機能：明るさ調整，姿勢オフセットなど
      - config機能：全体の制御方法など
    - ゲームパッドと接続することでUIをパッド経由でも入力可能
  - 通信機能
    - lan0: 無線LANルータで接続し，スマホなどと通信
    - lan1: Wifi経由でESP32と通信．主にROS2通信
    - bluetooth: ゲームパッド

- ガジェット本体: ESP32, RP2350
  - 内部にLiPo電池を保有し，電源とする．
  - ESP32, RP2350を内蔵
  - IMU（BNO055）を内蔵して，ガジェット本体の姿勢をquaternionで取得
  - リモート電源ON/OFF回路
  - LiPo充放電機能
  - IMU情報による自己の姿勢を元に動画を球体ディスプレイに表示する
    - 動画の更新レートより早く姿勢によって画像変換された映像を表示（画像が更新されるまでは同じ画像を画像変換する）．目標更新レートは30Hz
    - IMU情報から表示する映像はガジェットが回転しても上下は変化しないように画像変換する

コントローラーで動画を再生（320x160, 10fps）すると10fpsで動画が球体ディスプレイ上に表示されるが，この画像はガジェット本体が回転しても映像は回転しないようなディスプレイを作成する．

そのための制御プログラムを作成するプロジェクトである．



### hardware components

このプロジェクトは，
以下の三つのマイコンの連携によって実行される．

1. raspi: raspberry PIによるコントローラー機能
  - raspberry pi にはubuntu 22.04がインストールされている．
  - /home/yakatano/work/raspi　に動作しているプログラムを配置してあるので参照すること
  - UI機能：FastAPIでwebアプリを作成し，スマートフォンなどからコマンドを受け取る
  - ゲームパッド管理機能：ゲームパッドでUI操作を行う機能
  - 動画再生機能：UI操作などで，uploadされた動画ファイルに対してプレイリス尾を作成して再生する機能
  - ROS２通信機能：ROS2（Humble）を使って以下の情報の通信を行う
    - 画像: 再生されている動画の各フレームをjpeg圧縮して送信
    - UI: 各種UIで設定された操作コマンドを送信
    - IMU: マイコン側から送信されたIMU情報（Quaternion）を受信

2. ESP32: ESP32S3Rによるガジェット制御機能
  - ESP32S3R（https://tamanegi-digick.com/it/xiaos3/#toc16）
  - I2Cで接続したBNO055からquaternion情報を取得する機能
  - microROS2通信：ROS2通信によって各種の情報を送受信
    - IMU送信：quaternion情報を送信
    - 画像受信：ROS2経由でjpeg画像を受信，展開する機能
    - UI情報受信：raspiから各種のUI情報を受信する機能
  - 画像変換処理
    - 展開した画像を展開する
    - IMU情報から自己（ガジェット）の姿勢を計算
    - 自己姿勢情報を元に画像を変換する
    - 各LEDのRGB情報を計算する
  - SPI送信機能
    - RP2350にSPI経由で各LEDのRGB情報をDMA送信

3. RP2350:LED制御機能
  - SPI受信機能
    - ダブルバッファで現在描画中の画像情報を維持しながら裏のバッファにSPI経由で新しい画像を読み込む
    - 複数のLEDストリップがあるため，各ストリップに情報を割り振る
  - LED描画機能
    - pioを使って複数のLEDストリップにRGB情報を送信
  - 同期機能
    - 各LEDストリップが同時に更新されるように同期
    - 動画のフレーム更新作業よりも高い更新レートでLEDを描画（目標30Hz）する機能

今回のプロジェクトはesp32のプロジェクトのみとなる．
/home/yakatano/work/isolation-sphere/esp32
ここにテストコードも本コードも展開すること．



## Conversation Guidelines
- 常に日本語で会話すること

## Development Setup

### development poricy
以下のような手順で作業をすることにより，プロジェクトの精度を高める．
Claude.mdに書かれた手順は遵守し，ディスカッションによって適時新規のｍｄを作成したり，このmdを修正・更新すること

githubと連携し，適時コミットできるように環境整備する．
https://github.com/sastles-com/sastle
以下にisolation-sphere以下を展開する．

#### 用件定義
- 私との議論を重ねて先に用件定義を行い，requirement.mdを作成．
- requirement.mdを私と３回推敲し，requirement.mdをブラッシュアップ
- 私が最後に手動で修正したrequirement.mdを作成して，お互いに「用件定義完了」を確認してから次のステップに進む．

#### テストケース作成
- 用件定義に従い，テスト項目を洗い出す．私と常に相談してテスト項目を検討する．
- テストコードの作成：上記テスト項目が確定したのち，必要なテスト項目を達成するユニット単位でのテストコードを作成

#### ユニットテスト実装
- 上記のテストコードを元に各テスト項目に対してユニットテストを行う．
- *** プロジェクト実装時に組み込めるようにクラス


### ESP-IDF
ESP-iDFでコンパイル，フラッシュする．
- Double Buffer, DMA転送, PSRAM制御など高機能を要求するため

### Environment Setup
python を使用する場合，uv　を使用する．
```bash
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```



### Initial Setup Required
This project needs basic initialization:
1. Create a package.json file with appropriate dependencies
2. Set up the project structure based on requirements
3. Initialize any necessary configuration files

## Notes

- The project appears to be a Node.js application based on the presence of package-lock.json
- No existing source code or dependencies are currently defined
- The project will need to be properly initialized before development can begin