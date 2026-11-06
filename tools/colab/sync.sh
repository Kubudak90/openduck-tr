#!/usr/bin/env bash
# Kullanim: tools/colab/sync.sh SESSION RUN_NAME LOCAL_RUNS_DIR [ARALIK_SN]   (nohup caffeinate -i ... &)
# Kernel'e dokunmaz: gozetmenin yazdigi /content/sync_<ad>.tgz'yi indirir, runs/<ad>/ altina acar.
# Ag hatalarinda bekleyip tekrar dener; oturum sunucuda yoksa ya da egitim bitmisse cikar.
S=$1; N=$2; L=$3; INT=${4:-900}
mkdir -p "$L"
while true; do
  tmp=$(mktemp -d)
  if out=$(colab download -s "$S" "/content/sync_$N.tgz" "$tmp/s.tgz" 2>&1) && tar xzf "$tmp/s.tgz" -C "$L"; then
    last=$(grep -a -E "^adim +[0-9]+ \||bitti|Traceback" "$L/$N.log" | tail -1)
    echo "$(date '+%d %H:%M') indirildi | $last"
    if grep -a -q -E "^bitti:|Traceback" "$L/$N.log"; then echo "$(date '+%d %H:%M') EGITIM BITTI"; rm -rf "$tmp"; exit 0; fi
  else
    echo "$(date '+%d %H:%M') indirme hatasi: $(echo "$out" | tr '\n' ' ' | cut -c1-200)"
    if colab sessions 2>&1 | grep -q "No active sessions"; then echo "$(date '+%d %H:%M') OTURUM YOK"; rm -rf "$tmp"; exit 2; fi
  fi
  rm -rf "$tmp"; sleep "$INT"
done
