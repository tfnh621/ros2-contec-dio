# Contec デジタル入出力ドライバーのビルド方法

公式ドキュメント：
- Windows: https://help.contec.com/pc-helper/api-tool-wdm/jp/APITOOL.htm
- Linux: https://help.contec.com/pc-helper/api-tool-lnx/jp/APITOOL.htm

# Linux 用ドライバー（API-DIO(LNX)）の手順

## すべてをやってくれるコマンド

```bash
sudo apt install -y build-essential linux-headers-$(uname -r) gcc-12
wget --content-disposition 'https://www.contec.com/-/media/contec/download/software/api-dio(lnx)/cdio_930f.tgz/'
tar xf cdio_930F.tgz && rm cdio_930F.tgz
cd contec/cdio
make && sudo make install
cd config && sudo ./config <<< $'s\ny\ny\nq' && sudo ./contec_dio_start.sh
```

<details>
<summary>詳細な説明（クリックで展開）</summary>

## デバイスドライバの入手

コンテックのホームページから `Linux版デジタル入出力ドライバ API-DIO(LNX) 開発環境(フルセット)` と書かれたファイルをダウンロードする  
デバイスドライバの検索ページから製品を検索し、「サポート・ダウンロード」→「デバイスドライバ (2)」と進むと見つけられる

### 🔗 リンク
- 製品検索ページ: https://www.contec.com/jp/download/search/?type=858a9226-3053-4418-80b9-a61e0b6ff8ac
- [Linux版デジタル入出力ドライバ API-DIO(LNX) 開発環境(フルセット) Ver. 9.30](https://www.contec.com/jp/download/contract/contract1?itemid=527cac72-4150-4ff8-b653-e3d552fa9bc0&downloaditemid=4132766f-bd75-420e-b437-d1fa52b21ec8)

## ビルド・インストール

### 必要なパッケージのインストール

ビルドに必要なパッケージ（`build-essential`, `gcc-12`, Linux カーネルのヘッダーファイル）をインストールする
```bash
sudo apt install build-essential linux-headers-$(uname -r) gcc-12
```

### 入手したデバイスドライバの展開

ダウンロードしたファイルを展開し、ドライバーがあるディレクトリまで移動する
```bash
tar xvf cdio_930F.tgz
cd contec/cdio
```

### ドライバーをビルド、インストールする

```bash
make
sudo make install
```

## ドライバーの設定

デジタル入出力機器をコンピューターに接続する

### 設定ツールを起動する

```bash
cd config
sudo ./config
```

プログラムを起動してしばらく待つと、接続されているデジタル入出力機器が一覧に表示される  
ここで、設定したいデジタル入出力機器が一覧に表示されていることを確認する

### 設定ファイルを保存する

次に、「s」を入力して設定の保存を行う  
途中の選択肢は「y」を2回入力し、保存されたら「q」で終了する

### ドライバーの設定スクリプトの実行

設定の保存後に生成されるスクリプトを実行することで、デジタル入出力機器と通信ができるようになる
```bash
sudo ./contec_dio_start.sh
```

</details>
