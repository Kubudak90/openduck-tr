# Google Colab'da GPU eğitimi (`colab` CLI)

`rl.train_mjx` eğitimlerini Google Colab VM'inde koşturur. Gerekenler: `colab` CLI kurulu ve hesaba bağlı
(`colab whoami`), compute unit (CU) bakiyesi (`colab usage`). Sonuçlar ve ölçümler:
`docs/workflow/openduck-tr/p7-colab.md`.

| Dosya | Nerede çalışır | Ne yapar |
|---|---|---|
| `setup.py` | VM (kernel) | repoyu klonlar, `/content/venv`'e RunPod'daki sabit sürümleri kurar (~25 sn) |
| `supervise.py` | VM (kernel, bloklar) | eğitimi başlatır ya da koşanı bekler; 10 dk'da bir run klasörünü `/content/sync_<ad>.tgz`'ye paketler |
| `keeper.sh` | Mac | gözetmen hücresine bağlı kalır, koparsa yeniden bağlanır, 4 dk'da bir kısa bağlantı açar |
| `sync.sh` | Mac | 15 dk'da bir `sync_<ad>.tgz`'yi indirip `runs/<ad>/` altına açar |

## Önemli: VM'i ayakta tutan şey bağlı istemci

Colab, VM'e bağlı bir kernel istemcisi (`colab exec` bağlantısı) kalmayınca **~18–20 dk sonra VM'i geri alır**.
Kernel'in meşgul olması da, hücrenin dakikada bir çıktı basması da yetmez; `colab download` / `colab console`
sayılmaz (üçü de denendi, üç koşu kayboldu). Bu yüzden `keeper.sh` eğitim boyunca Mac'te çalışmalı. Mac uyursa
(kapak kapanırsa) ya da ağ ~18 dk'dan uzun koparsa VM gider; checkpoint'ler `sync.sh` ile yerelde kalır.

Diğer tuzaklar:
- `colab exec` varsayılan `--timeout` 30 sn; uzun işlerde büyütün. CLI zaman aşımında ya da kopmada kernel'i
  durdurmaz, hücre VM'de sürer.
- Kernel meşgulken yeni `colab exec` kuyruğa girer; izleme için `sync.sh`'in indirdiği log'a bakın.
- Eğitim bitince VM kendiliğinden kapanmaz: `colab stop -s <oturum>`.

## Kullanım

```bash
S=duck-g4; N=colab-getup                       # oturum ve koşu adı
colab new -s $S --gpu G4                       # RTX PRO 6000 Blackwell 96 GB, ~10 CU/sa
colab exec -s $S --timeout 900 -f tools/colab/setup.py

# gözetmen hücresi = ayar başlığı + supervise.py (RUN_ENV, OPENDUCK_ROBOT/KAPPA/TUNE varsayılanlarını ezebilir)
mkdir -p runs
{ echo "RUN_NAME='$N'"; echo 'RUN_ENV={}'
  echo 'RUN_ARGS=["--task","getup","--num-envs","8192","--num-timesteps","1e9"]'
  cat tools/colab/supervise.py; } > runs/colab_$N.py

nohup caffeinate -i tools/colab/keeper.sh $S "$PWD/runs/colab_$N.py" "$PWD/runs/$N.exec.log" >/dev/null 2>&1 &
nohup caffeinate -i tools/colab/sync.sh $S $N "$PWD/runs" 900 >> runs/$N.sync.log 2>&1 &
```

İzleme: `runs/$N.sync.log` (her indirmede son `adim` satırı) ve `runs/$N.log` (tam eğitim çıktısı).
Bitince: `pkill -f tools/colab/keeper.sh; pkill -f tools/colab/sync.sh; colab stop -s $S`.

Yarıda kalan koşu: yeni VM'e `params_latest.pkl`'i `colab upload` ile koyup `RUN_ARGS`'a
`"--restore","runs/<ad>/params_latest.pkl"` ekleyin (getup'ta `--restore` müfredat sayacını sıfırlar).
