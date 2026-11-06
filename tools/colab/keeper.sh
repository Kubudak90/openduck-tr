#!/usr/bin/env bash
# Kullanim: tools/colab/keeper.sh SESSION CELL_FILE EXEC_LOG   (nohup caffeinate -i ... &)
# Colab VM'i ancak bagli bir kernel istemcisi varken tutar (~18-20 dk istemcisiz kalinca geri alir).
# 1) Gozetmen hucresine bagli kalir; exec cikarsa 30 sn sonra yeniden baglanir (hucre tekrar calistirilabilir).
# 2) Her 4 dk kisa bir exec ile yeni baglanti acar (ana baglanti takilirsa diye). Kuyruga giren "1"
#    hucreleri egitim bitince aninda calisip biter, zararsiz.
# "CIKIS" gorulunce ya da oturum sunucuda yoksa cikar.
S=$1; F=$2; LOG=$3
ts() { date '+%d %H:%M'; }
( while true; do
    sleep 240
    grep -a -q " CIKIS" "$LOG" && exit 0
    echo 1 | colab exec -s "$S" --timeout 15 >/dev/null 2>&1
  done ) &
PING=$!
trap 'kill $PING 2>/dev/null' EXIT
while true; do
  echo "$(ts) [keeper] baglaniyor" >> "$LOG"
  colab exec -s "$S" --timeout 172800 -f "$F" >> "$LOG" 2>&1
  echo "$(ts) [keeper] exec cikti rc=$?" >> "$LOG"
  grep -a -q " CIKIS" "$LOG" && { echo "$(ts) [keeper] egitim bitti" >> "$LOG"; exit 0; }
  if colab sessions 2>&1 | grep -q "No active sessions"; then echo "$(ts) [keeper] OTURUM YOK" >> "$LOG"; exit 2; fi
  sleep 30
done
