# 3axis-detecter

M5Stack ATOM-S3 で取得した IMU データと RSSI を UDP で Python サーバーへ送信し、ブラウザで可視化するプロジェクトです。

現在の構成は次の 2 つです。

- ファームウェア: `src/main.cpp`
- 受信サーバーとダッシュボード: `decoder/server.py`

## 構成

- `src/main.cpp`
  ATOM-S3 側のファームウェアです。IMU を一定周期で読み取り、固定長バイナリの UDP パケットとして送信します。

- `decoder/server.py`
  Python 側の受信サーバーです。UDP で受けたバイナリを復元し、HTTP API とダッシュボード画面を提供します。

- `decoder/dashboard.html`
  ブラウザ表示用のダッシュボードです。加速度・ジャイロのグラフと、姿勢の 3D 表示を行います。

- `docs/udp_binary_protocol.md`
  UDP バイナリ通信仕様です。

- `include/local_config.example.h`
  ローカル設定ファイルのテンプレートです。

## セットアップ

### 1. ローカル設定ファイルを作成する

`include/local_config.example.h` をコピーして `include/local_config.h` を作成し、Wi-Fi とサーバーIPを設定します。

```cpp
#pragma once

#include <WiFi.h>

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const IPAddress SERVER_IP(192, 168, 0, 10);
const uint16_t SERVER_PORT = 8001;
const uint16_t UDP_LOCAL_PORT = 4210;
```

`include/local_config.h` は `.gitignore` に入っているため、Git には含まれません。

### 2. ファームウェアを書き込む

PlatformIO で ATOM-S3 に書き込みます。

- ボード: `m5stack-atoms3`
- フレームワーク: Arduino

`platformio.ini` の環境は `env:m5stack-atoms3` です。

### 3. Python サーバーを起動する

必要な Python パッケージを入れてから `decoder/server.py` を起動します。

主に使うもの:

- `fastapi`
- `uvicorn`

例:

```bash
uvicorn decoder.server:app --host 0.0.0.0 --port 8000 --reload
```

### 4. ブラウザで確認する

サーバー起動後に次を開きます。

```text
http://<server-ip>:8000/
```

## 通信仕様

UDP は JSON ではなく固定長バイナリで送っています。詳細は次を参照してください。

- [UDP バイナリ通信仕様](docs/udp_binary_protocol.md)

## 補足

- UDP 送信先ポートは `8001`
- ダッシュボード用 HTTP サーバーは `8000`
- IMU の姿勢表示は加速度から算出しています
- パケット欠落は `seq` で検出できます
