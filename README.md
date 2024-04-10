# ros2-contec-dio

CONTEC Digital I/O for ROS 2

## Linux 用ドライバー（API-DIO(LNX)）のビルドとインストール

1. コンテックのホームページから最新のデバイスドライバをダウンロードする（ダウンロードには会員登録が必要）
   - 製品検索ページ: https://www.contec.com/jp/download/search/?type=858a9226-3053-4418-80b9-a61e0b6ff8ac
   - [Linux版デジタル入出力ドライバ API-DIO(LNX) 開発環境(フルセット) Ver. 8.80](https://www.contec.com/jp/download/contract/contract1?itemid=527cac72-4150-4ff8-b653-e3d552fa9bc0&downloaditemid=4132766f-bd75-420e-b437-d1fa52b21ec8)
   <!-- - [Windows版高機能デジタル入出力ドライバ API-DIO(WDM) 開発環境(フルセット) Ver. 9.90](https://www.contec.com/jp/download/contract/contract2?itemid=527cac72-4150-4ff8-b653-e3d552fa9bc0&downloaditemid=14506a61-ede1-4dc4-a335-2b5fc031ebcb) -->
2. ダウンロードしたファイルを解凍する: `tar zxvf cdio_880F.tgz` (880 はバージョン)
3. ドライバのソースコードがあるディレクトリに移動する: `cd contec/cdio`
4. `make` でビルドする
5. `sudo make install` でビルドしたドライバーをインストールする

## API-DIO(LNX) デジタル入出力用ドライバ がビルドできない問題について

Linux カーネル 6.5 から `get_user_pages` の引数が変わったことが原因

### 影響を受けるバージョン

- API-DIO(LNX) デジタル入出力用ドライバ Ver.8.80 以下
- Linux カーネル 6.5 以上（ほかにも影響を受けるバージョンがあるかもしれないが未調査）

### 修正方法

`contec/cdio/module/BusMaster.c`, `contec/cdio/module/BusMaster.c` の2ファイルを次のように編集する。

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

## リンク集

### 開発者向けオンラインヘルプ

- Windows: https://help.contec.com/pc-helper/api-tool-wdm/jp/APITOOL.htm
- Linux: https://help.contec.com/pc-helper/api-tool-lnx/jp/APITOOL.htm
