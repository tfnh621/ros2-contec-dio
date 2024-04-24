# Contec デジタル入出力ドライバーのビルド方法

公式ドキュメント：
- Windows: https://help.contec.com/pc-helper/api-tool-wdm/jp/APITOOL.htm
- Linux: https://help.contec.com/pc-helper/api-tool-lnx/jp/APITOOL.htm

# Linux 用ドライバー（API-DIO(LNX)）の手順

## すべてをやってくれるコマンド

```bash
sudo apt install -y build-essential linux-headers-$(uname -r) gcc-12
wget --cipher 'DEFAULT:!DH' --content-disposition 'https://www.contec.com/-/media/contec/download/software/api-dio(lnx)/cdio_880f.tgz/'
tar xf cdio_880F.tgz && rm cdio_880F.tgz
cd contec/cdio
sed -i -e 's/#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0))/#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6,5,0))\n\tpSgList->nr_pages\t= get_user_pages(v_address, pgcount, rw == READ, maplist);\n#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0))/' module/BusMaster.c module/BusMasterMemory.c
make && sudo make install
cd config && sudo ./config <<< $'s\ny\ny\nq' && sudo ./contec_dio_start.sh
```

<details>
<summary>以下は詳細な説明（クリックで展開）</summary>

## デバイスドライバの入手

コンテックのホームページから `Linux版デジタル入出力ドライバ API-DIO(LNX) 開発環境(フルセット)` と書かれたファイルをダウンロードする  
デバイスドライバの検索ページから製品を検索し、「サポート・ダウンロード」→「デバイスドライバ (2)」と進むと見つけられる

### 🔗 リンク
- 製品検索ページ: https://www.contec.com/jp/download/search/?type=858a9226-3053-4418-80b9-a61e0b6ff8ac
- [Linux版デジタル入出力ドライバ API-DIO(LNX) 開発環境(フルセット) Ver. 8.80](https://www.contec.com/jp/download/contract/contract1?itemid=527cac72-4150-4ff8-b653-e3d552fa9bc0&downloaditemid=4132766f-bd75-420e-b437-d1fa52b21ec8)

## ビルド・インストール

### 必要なパッケージのインストール

ビルドに必要なパッケージ（`build-essential`, `gcc-12`, Linux カーネルのヘッダーファイル）をインストールする
```bash
sudo apt install build-essential linux-headers-$(uname -r) gcc-12
```

### 入手したデバイスドライバの展開

ダウンロードしたファイルを展開し、ドライバーがあるディレクトリまで移動する
```bash
tar xvf cdio_880F.tgz
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

## 新しいカーネルバージョンだとビルドできない問題について

Linux カーネル 6.5 から `get_user_pages` の引数が変わったことが原因

### 影響を受けるバージョン

- API-DIO(LNX) デジタル入出力用ドライバ Ver.8.80 以下
- Linux カーネル 6.5 以上（ほかにも影響を受けるバージョンがあるかもしれないが未調査）

### 修正方法

`contec/cdio/module/BusMaster.c`, `contec/cdio/module/BusMasterMemory.c` の2ファイルを次のように編集する

```diff
diff --git a/contec/cdio/module/BusMaster.c b/contec/cdio/module/BusMaster.c
index a234a4c..63fe8cb 100644
--- a/contec/cdio/module/BusMaster.c
+++ b/contec/cdio/module/BusMaster.c
@@ -679,7 +679,9 @@ static long BmMakeSGList(PSGLIST pSgList, unsigned long dwDir)
 #else
        down_read(&current->mm->mmap_sem);
 #endif
-#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0))
+#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6,5,0))
+       pSgList->nr_pages       = get_user_pages(v_address, pgcount, rw == READ, maplist);
+#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0))
        pSgList->nr_pages       = get_user_pages(v_address, pgcount, rw == READ, maplist, NULL);
 #else
        pSgList->nr_pages       = get_user_pages(current, current->mm, v_address, pgcount, rw == READ, 0, maplist, NULL);
```
```diff
diff --git a/contec/cdio/module/BusMasterMemory.c b/contec/cdio/module/BusMasterMemory.c
index 61d49a8..267ebfc 100644
--- a/contec/cdio/module/BusMasterMemory.c
+++ b/contec/cdio/module/BusMasterMemory.c
@@ -434,7 +434,9 @@ long MemBmMakeSGList(PSGLIST pSgList, unsigned long dwDir)
 #else
        down_read(&current->mm->mmap_sem);
 #endif
-#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0))
+#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6,5,0))
+       pSgList->nr_pages       = get_user_pages(v_address, pgcount, rw == READ, maplist);
+#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0))
        pSgList->nr_pages       = get_user_pages(v_address, pgcount, rw == READ, maplist, NULL);
 #else
        pSgList->nr_pages       = get_user_pages(current, current->mm, v_address, pgcount, rw == READ, 0, maplist, NULL);
```

</details>
