# -*- coding: utf-8 -*-
"""
Egitilmis politikayi tek ortamda oynatir, istenirse video/GIF kaydeder ve olcum yazar.

  python3 -m rl.play --policy runs/NAME/policy.onnx [--seconds 12] [--command vx vy wz [h sit] | --demo [D]]
                     [--getup-policy runs/G/policy.onnx] [--tip front|back|left|right]
                     [--video out.mp4] [--gif out.gif] [--seed S] [--json metrics.json] [--qpos-out traj.npy]
                     [--scene SCENE.xml] [--getup-head free|fixed]

--policy: .onnx dosyasi (yaninda <ad>_meta.json), .pt checkpoint ya da "zero" (hep sifir aksiyon = referans
poz). Gorev (walk / main) politikanin meta'sindan (meta["task"]; meta yoksa ONNX boyutlarindan); "zero" ve
gorevsiz .pt icin --task. Gozlem/komut duzeni ortamdan ve TaskSpec'ten gelir (play gozleme dokunmaz).
--demo / --demo walk: dur, ileri yuru, sola don, geri yuru, yana adim (toplam 19 s < bolum suresi 20 s).
--demo sit  (main politikasi): dur, yuru, alcak yuru, comel, otur, kalk, yuru (24 s).
--demo getup (main/walk politikasi + --getup-policy): dur, yuru, devirme (--tip yonunde acisal hiz itmesi),
   supervizor getup politikasina devreder, basari kosulu (yukseklik haric) 0.5 s surunce main'e doner
   (last_action = 0, alpha = 0, h_ref = H_HOME, 1 s sifir komut), sonra yuru.
Ortam: VecEnv(1, num_workers=0, randomize=False, noise=False, head_random=False, push=False).
Goruntu, gorsel meshli modelle (build_model(visual=True), ayni qpos duzeni) 30 fps'de ve govdeyi
izleyen kamerayla alinir (MUJOCO_GL=egl, ekransiz; macOS'ta MUJOCO_GL=cgl).

Olcumler (segment basina): komut ve gerceklesen ortalama hiz. Gerceklesen hiz govdenin (serbest
eklem) konum farkindan, yalniz yaw ile dondurulmus govde cercevesinde; wz yaw farkindan.
disp: net yer degistirme (dusmede parca parca toplanir), path: govde orijininin adim adim yolu
(salinim dahil), falls: zaman asimi olmayan bolum sonlari.
main gorevinde ek olarak: height_rmse (|trunk_z - z_ref| RMS, z_ref durus filtresinden), trunk_z ortalama/en
kucuk, peak_tau (tepe |tau| / 0.96), sit segmentlerinde t_to_seated (oturus kosulu: sit, alpha = 1, govde
yerde, gz < -0.9), sit'ten sonraki segmentte t_to_stand (z > 0.105, iki ayak yerde, gz < -0.9).
getup demosunda: supervizor olaylari, t_to_upright (devrilme -> main'e donus), donusten sonra 2 s icinde
tekrar dusme sayisi.

tr35_b2 sureci (OPENDUCK_ROBOT; xl330 programlari ve sabitleri aynen kalir): demo komutlari C.COMMAND_RANGES'in
kesirleri (0.5 vx_max, 0.5 wz_max, 0.5 vx_min, 0.5 vy_max), sit demosu yukseklikleri C.H_*'tan (tr35_programs),
TIP_W = 10 / sqrt(1.282); peak_tau eklem basina |tau_net| / NOMINAL tau_s (peak_tau_joints). --scene sahne MJCF'si
(ortamlar ve goruntu modeli); getup ortaminin kafa modu getup meta'sindaki getup_head'den (--getup-head ezer).
"""
import argparse
import json
import os

import numpy as np

from . import config as C
from . import task_math as T
from . import tasks as TK

FPS = 30
DEMO = [
    ("stand", (0.0, 0.0, 0.0), 2.0),
    ("walk forward", (0.15, 0.0, 0.0), 5.0),
    ("turn left", (0.0, 0.0, 0.5), 4.0),
    ("walk backward", (-0.08, 0.0, 0.0), 4.0),
    ("sidestep left", (0.0, 0.08, 0.0), 4.0),
]
SIT_DEMO = [
    ("stand", (0.0, 0.0, 0.0, C.H_HOME, 0.0), 2.0),
    ("walk forward", (0.15, 0.0, 0.0, C.H_HOME, 0.0), 3.0),
    ("crouch walk", (0.10, 0.0, 0.0, 0.095, 0.0), 4.0),
    ("crouch", (0.0, 0.0, 0.0, 0.075, 0.0), 3.0),
    ("sit", (0.0, 0.0, 0.0, 0.075, 1.0), 5.0),
    ("stand up", (0.0, 0.0, 0.0, C.H_HOME, 0.0), 4.0),
    ("walk", (0.15, 0.0, 0.0, C.H_HOME, 0.0), 3.0),
]
TIP_SEGMENT = "tip over"
GETUP_DEMO = [
    ("stand", (0.0, 0.0, 0.0), 2.0),
    ("walk forward", (0.15, 0.0, 0.0), 3.0),
    (TIP_SEGMENT, (0.0, 0.0, 0.0), 9.0),
    ("walk", (0.10, 0.0, 0.0), 4.0),
]
# devirme: serbest eklem acisal hizi (govde cercevesi) eksenleri; + y = burun asagi (one dusme)
TIP_AXES = {"front": (0.0, 1.0, 0.0), "back": (0.0, -1.0, 0.0), "left": (-1.0, 0.0, 0.0), "right": (1.0, 0.0, 0.0)}
TIP_W = 10.0                 # rad/s (walk-mjx-v2 politikasi 6 rad/s yan itmeyi dengeliyor; 10'da 4 yonde de duser)
TIP_RETRY_S, TIP_MAX = 1.0, 3
AFTER_HANDOFF_S = 2.0        # devirden sonra bu sure icinde tekrar dusme sayilir
TORQUE_LIMIT = C.GETUP["torque_limit"]
TR35 = "tr35_b2"
FROUDE_S = 1.282             # OpenDuck-TR olcek (spec; config.FROUDE_S cekirdekte)


def tr35_programs():
    """tr35_b2 demo programlari: komutlar C.COMMAND_RANGES'in kesirleri, yukseklikler C.H_*'tan (27 cm sabitleri
    yok). walk: 0.5 vx_max, 0.5 wz_max, 0.5 vx_min (geri), 0.5 vy_max; sit: comelerek yurume h = (H_HOME +
    H_MIN_WALK) / 2, comel h = (H_MIN_STAND + H_MIN_WALK) / 2; getup: 0.5 vx_max. Yavas yurume = vx_max / 3."""
    R = C.COMMAND_RANGES
    vf, vs = 0.5 * R["vx"][1], R["vx"][1] / 3.0
    h, hw = C.H_HOME, 0.5 * (C.H_HOME + C.H_MIN_WALK)
    hc = 0.5 * (C.H_MIN_STAND + C.H_MIN_WALK)
    demo = [("stand", (0.0, 0.0, 0.0), 2.0), ("walk forward", (vf, 0.0, 0.0), 5.0),
            ("turn left", (0.0, 0.0, 0.5 * R["wz"][1]), 4.0), ("walk backward", (0.5 * R["vx"][0], 0.0, 0.0), 4.0),
            ("sidestep left", (0.0, 0.5 * R["vy"][1], 0.0), 4.0)]
    sit = [("stand", (0.0, 0.0, 0.0, h, 0.0), 2.0), ("walk forward", (vf, 0.0, 0.0, h, 0.0), 3.0),
           ("crouch walk", (vs, 0.0, 0.0, hw, 0.0), 4.0), ("crouch", (0.0, 0.0, 0.0, hc, 0.0), 3.0),
           ("sit", (0.0, 0.0, 0.0, hc, 1.0), 5.0), ("stand up", (0.0, 0.0, 0.0, h, 0.0), 4.0),
           ("walk", (vf, 0.0, 0.0, h, 0.0), 3.0)]
    getup = [("stand", (0.0, 0.0, 0.0), 2.0), ("walk forward", (vf, 0.0, 0.0), 3.0),
             (TIP_SEGMENT, (0.0, 0.0, 0.0), 9.0), ("walk", (vs, 0.0, 0.0), 4.0)]
    return demo, sit, getup, 10.0 / np.sqrt(getattr(C, "FROUDE_S", None) or FROUDE_S)


if C.ROBOT_PROFILE == TR35:  # xl330 programlari yukarida aynen
    DEMO, SIT_DEMO, GETUP_DEMO, TIP_W = tr35_programs()
DEMOS = {"walk": DEMO, "sit": SIT_DEMO, "getup": GETUP_DEMO}


def infer_task(obs_dim, action_dim):
    """(obs, aksiyon) boyutlarindan gorev adi."""
    for t in TK.TASKS.values():
        if (t.num_obs, t.num_actions) == (obs_dim, action_dim):
            return t.name
    raise ValueError("%s/%s boyutlu politika hicbir goreve uymuyor" % (obs_dim, action_dim))


class ZeroPolicy(object):
    def __init__(self, task="walk"):
        self.task = TK.get_task(task).name
        self.meta = None

    def __call__(self, obs):
        return np.zeros((obs.shape[0], TK.get_task(self.task).num_actions), np.float32)


class OnnxPolicy(object):
    def __init__(self, path):
        import onnxruntime as ort
        self.sess = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
        meta_path = os.path.splitext(path)[0] + "_meta.json"
        obs_dim = self.sess.get_inputs()[0].shape[-1]
        act_dim = self.sess.get_outputs()[0].shape[-1]
        self.meta = None
        if os.path.exists(meta_path):
            with open(meta_path) as f:
                self.meta = json.load(f)
            spec = TK.get_task(self.meta.get("task", "walk"))
            if self.meta.get("obs_dim") != spec.num_obs or self.meta.get("action_dim") != spec.num_actions:
                raise ValueError("%s: obs/action boyutu %s gorevi ile uyusmuyor" % (meta_path, spec.name))
            lay = self.meta.get("obs_layout")
            if lay is not None and [tuple(x) for x in lay] != [tuple(x) for x in spec.obs_layout]:
                raise ValueError("%s: obs_layout %s gorevinin duzeninden farkli" % (meta_path, spec.name))
            self.task = spec.name
        else:
            self.task = infer_task(obs_dim, act_dim)
        spec = TK.get_task(self.task)
        if (isinstance(obs_dim, int) and obs_dim != spec.num_obs) or (isinstance(act_dim, int) and
                                                                       act_dim != spec.num_actions):
            raise ValueError("%s: ONNX %s/%s boyutlu, meta %s gorevi" % (path, obs_dim, act_dim, spec.name))

    def __call__(self, obs):
        return self.sess.run(["action"], {"obs": np.asarray(obs, np.float32)})[0]


class TorchPolicy(object):
    def __init__(self, path):
        import torch
        from .export_onnx import checkpoint_task, load_actor_critic
        self.torch = torch
        self.ac, ckpt = load_actor_critic(path)
        self.meta = {"steps": ckpt.get("steps")}
        a = self.ac.arch
        spec = TK.get_task(checkpoint_task(ckpt))
        self.task = spec.name if (a["num_obs"], a["num_actions"]) == (spec.num_obs, spec.num_actions) \
            else infer_task(a["num_obs"], a["num_actions"])

    def __call__(self, obs):
        with self.torch.no_grad():
            return self.ac.act_inference(self.torch.as_tensor(obs, dtype=self.torch.float32)).numpy()


def load_policy(spec, task=None):
    """spec: "zero" | .pt | .onnx. task: "zero" politikasinin gorevi; verilirse yuklenen politikanin gorevi
    bununla ayni olmali (ValueError)."""
    if spec == "zero":
        return ZeroPolicy(task or "walk")
    p = TorchPolicy(spec) if spec.endswith(".pt") else OnnxPolicy(spec)
    if task is not None and p.task != TK.get_task(task).name:
        raise ValueError("%s bir %s politikasi, %s bekleniyor" % (spec, p.task, TK.get_task(task).name))
    return p


def check_role(policy, roles, what):
    if policy.task not in roles:
        raise ValueError("%s: %s politikasi verildi, beklenen %s" % (what, policy.task, " ya da ".join(roles)))


def wrap_angle(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def yaw_of(quat):
    w, x, y, z = quat
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def fmt_cmd(cmd):
    return " ".join("%+.2f" % v for v in cmd)


def tip_over(data, direction, w=None):
    """Devirme itmesi: serbest eklemin acisal hizina (govde cercevesi) w rad/s (varsayilan TIP_W) ekler."""
    data.qvel[3:6] += (TIP_W if w is None else w) * np.asarray(TIP_AXES[direction])


class Supervisor(object):
    """main <-> getup devri (spec 3.7, C.SUPERVISOR). update(s) her kontrol adiminda AKTIF politikanin
    ortam durumuyla (VecEnv.task_state(), tek ortam) cagrilir; devir olursa yeni modu ("getup" | "main")
    dondurur, yoksa None.
      main -> getup: gz > fall_gz art arda fall_steps adim
      getup -> main: task_math.getup_success_now(use_height=False) ready_hold_s boyunca (robotta z olculmez)
    Devirde cagiran: last_action = 0, alpha = 0, h_ref = H_HOME (VecEnv.set_state) ve handoff_zero_cmd_s sifir komut."""

    def __init__(self, cfg=None, dt=C.CTRL_DT, mode="main"):
        self.cfg = dict(C.SUPERVISOR if cfg is None else cfg)
        self.dt = dt
        self.mode = mode
        self.n_fall = 0
        self.t_ready = np.zeros(1)

    def update(self, s):
        if self.mode == "main":
            gz = float(np.max(T.projected_gravity(np, s["quat"])[..., 2]))
            self.n_fall = self.n_fall + 1 if gz > self.cfg["fall_gz"] else 0
            if self.n_fall >= self.cfg["fall_steps"]:
                self.mode, self.n_fall, self.t_ready = "getup", 0, np.zeros(1)
                return "getup"
            return None
        ok = T.getup_success_now(np, s, use_height=False)
        self.t_ready, held = T.hold_timer(np, self.t_ready, ok.reshape(-1)[:1], self.cfg["ready_hold_s"], self.dt)
        if held[0] > 0.5:
            self.mode, self.t_ready = "main", np.zeros(1)
            return "main"
        return None


class Recorder(object):
    """Gorsel modeli egitim modelinin qpos'uyla gunceller, govdeyi izleyen kamerayla cizer.
    Kareler bellekte tutulmaz: mp4'e akitilir, GIF icin kucultulmus paletli kopyasi saklanir."""

    GIF_FPS, GIF_WIDTH = 15, 400

    def __init__(self, width, height, trunk_body, shadows=True, video=None, gif=None, scene_path=None):
        import mujoco
        from .robot_model import build_model
        self.mj = mujoco
        self.model = build_model(scene_path, visual=True)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = bool(shadows)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.distance = 2.0 * self.model.stat.extent
        self.cam.azimuth = 135.0
        self.cam.elevation = -15.0
        self.trunk = trunk_body
        self.lookat = None
        self.font = None
        self.n = 0
        self.video_path, self.gif_path = video, gif
        self.writer = None
        if video:
            import imageio
            # libx264/yuv420p cift boyut ister; tek boyutlar imageio tarafindan cifte yuvarlanir
            self.writer = imageio.get_writer(video, fps=FPS, codec="libx264", quality=8, macro_block_size=2)
        self.gif_step = max(1, int(round(FPS / self.GIF_FPS)))
        self.gif_frames = []

    def snap(self):
        self.lookat = None

    def capture(self, qpos, text=None):
        from PIL import Image, ImageDraw, ImageFont
        mj = self.mj
        self.data.qpos[:] = qpos
        mj.mj_forward(self.model, self.data)
        target = self.data.xpos[self.trunk].copy()
        target[2] = 0.22
        if self.lookat is None:
            self.lookat = target
        else:
            self.lookat = self.lookat + 0.15 * (target - self.lookat)
        self.cam.lookat[:] = self.lookat
        self.renderer.update_scene(self.data, camera=self.cam)
        im = Image.fromarray(self.renderer.render())
        if text:
            if self.font is None:
                try:
                    self.font = ImageFont.load_default(size=max(12, self.renderer.height // 30))
                except TypeError:
                    self.font = ImageFont.load_default()
            ImageDraw.Draw(im).text((10, 8), text, fill=(20, 20, 20), font=self.font)
        if self.writer is not None:
            self.writer.append_data(np.asarray(im))
        if self.gif_path and self.n % self.gif_step == 0:
            if im.width > self.GIF_WIDTH:
                im = im.resize((self.GIF_WIDTH, int(round(im.height * self.GIF_WIDTH / im.width))), Image.LANCZOS)
            self.gif_frames.append(im.convert("P", palette=Image.ADAPTIVE, colors=128))
        self.n += 1

    def close(self):
        """Ciktilari yazar (yarida kalan kosuda da)."""
        self.renderer.close()
        if self.writer is not None:
            self.writer.close()
            self.writer = None
            print("video: %s (%d kare, %d fps)" % (self.video_path, self.n, FPS))
        if self.gif_path and self.gif_frames:
            # GIF sureleri 10 ms birimli; birikimli yuvarlama ortalama hizi korur
            ms = [int(round(100.0 * i * self.gif_step / FPS)) * 10 for i in range(len(self.gif_frames) + 1)]
            self.gif_frames[0].save(self.gif_path, save_all=True, append_images=self.gif_frames[1:],
                                    duration=[b - a for a, b in zip(ms[:-1], ms[1:])], loop=0)
            print("gif: %s (%d kare, %d fps)" % (self.gif_path, len(self.gif_frames), self.GIF_FPS))
            self.gif_frames = []


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="OpenDuck politikasini oynat / kaydet / olc")
    p.add_argument("--policy", required=True, help="policy.onnx, checkpoint .pt ya da 'zero' (walk/main politikasi)")
    p.add_argument("--getup-policy", default=None, help="--demo getup icin getup politikasi (.onnx/.pt/'zero')")
    p.add_argument("--task", choices=("walk", "main"), default=None,
                   help="'zero' politikasinin gorevi (varsayilan: walk; --demo sit/getup: main)")
    p.add_argument("--seconds", type=float, default=None, help="sure (varsayilan 12; --demo'da program suresi)")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--command", type=float, nargs="+", metavar="V", default=None,
                   help="sabit komut: vx vy wz (m/s, m/s, rad/s) ya da main icin vx vy wz h sit; varsayilan 0.15 0 0")
    g.add_argument("--demo", nargs="?", const="walk", default=None, choices=sorted(DEMOS),
                   help="program: walk (varsayilan; dur / ileri / sola don / geri / yana adim), sit, getup")
    p.add_argument("--tip", choices=sorted(TIP_AXES), default="front", help="getup demosunda devirme yonu")
    p.add_argument("--video", default=None, help="mp4 cikti")
    p.add_argument("--gif", default=None, help="gif cikti (15 fps, 400 px)")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--no-shadows", action="store_true", help="golgesiz (yazilim GL'de ~2.5x hizli)")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--json", default=None, help="olcumleri JSON olarak da yaz")
    p.add_argument("--qpos-out", default=None, help="qpos yorungesini .npy olarak yaz (sim/collision_check.py icin)")
    p.add_argument("--scene", default=None, help="sahne MJCF'si (varsayilan config.SCENE_PATH; or. tr35 yer tutucu)")
    p.add_argument("--getup-head", choices=("free", "fixed"), default=None,
                   help="getup politikasinin kafa modu (varsayilan: getup meta'sindaki getup_head)")
    a = p.parse_args(argv)
    if a.command is not None and len(a.command) not in (3, 5):
        p.error("--command 3 (vx vy wz) ya da 5 (vx vy wz h sit) deger alir")
    return a


def build_schedule(args, task="walk"):
    """[(ad, komut, sure)]; main gorevinde 3 boyutlu komutlar [vx, vy, wz, H_HOME, 0] ile doldurulur."""
    pad = (lambda c: np.array(c, dtype=np.float64)) if task == "walk" else \
        (lambda c: TK.pad_command(task, np.array(c, dtype=np.float64)))
    if args.demo:
        sched, t = [], 0.0
        for name, cmd, dur in DEMOS[args.demo]:
            if args.seconds is not None and t >= args.seconds:
                break
            if args.seconds is not None:
                dur = min(dur, args.seconds - t)
            sched.append((name, pad(cmd), dur))
            t += dur
        return sched
    cmd = args.command if args.command is not None else (0.15, 0.0, 0.0)
    if task == "walk" and len(cmd) != 3:
        raise ValueError("walk politikasi 3 boyutlu komut alir (vx vy wz)")
    return [("command", pad(cmd), 12.0 if args.seconds is None else args.seconds)]


class _Seg(object):
    """Segment olcumleri (walk alanlari + main ek alanlari). tau_nom (14,) verilirse (tr35): doygunluk eklem
    basina |tau_net| / NOMINAL tau_s (peak_tau = en buyugu, peak_tau_joints gozlem eklemleri sirasiyla); yoksa
    max|tau| / TORQUE_LIMIT (xl330)."""

    def __init__(self, name, cmd, seconds, extra, tau_nom=None):
        self.d = dict(name=name, cmd=cmd.tolist(), seconds=seconds, v_sum=np.zeros(3), n=0, falls=0, timeouts=0,
                      path=0.0, disp=0.0)
        self.extra = extra
        self.e2, self.ne, self.zsum, self.zmin, self.tau = 0.0, 0, 0.0, np.inf, 0.0
        self.t_seated = self.t_stand = None
        self.tau_nom = None if tau_nom is None else np.asarray(tau_nom, dtype=np.float64)
        self.tau_j = None if tau_nom is None else np.zeros(len(self.tau_nom))

    def __getitem__(self, k):
        return self.d[k]

    def __setitem__(self, k, v):
        self.d[k] = v

    def add_main(self, s, t_rel, after_sit):
        """main gorev durumu (task_state, tek ortam) ile ek olcumler."""
        z = float(s["trunk_z"][0])
        zr = float(T.z_ref(np, s["h_ref"], s["alpha"])[0])
        self.e2 += (z - zr) ** 2
        self.ne += 1
        self.zsum += z
        self.zmin = min(self.zmin, z)
        if self.tau_nom is None:
            self.tau = max(self.tau, float(np.max(np.abs(s["torque"]))) / TORQUE_LIMIT)
            stand_gz = C.MAIN["seat_gz"]
        else:
            self.tau_j = np.maximum(self.tau_j, np.abs(np.asarray(s["tau_net"])[0]) / self.tau_nom)
            self.tau = max(self.tau, float(np.max(self.tau_j)))
            stand_gz = C.MAIN.get("stand_gz", C.MAIN["seat_gz"])      # tr35: ayakta dik testi oturustan ayri
        gz = float(T.projected_gravity(np, s["quat"])[0, 2])
        up = gz < C.MAIN["seat_gz"]
        _, _, sit = T.main_command(np, s["command"])
        if sit[0] > 0.5 and self.t_seated is None and up and s["trunk_floor"][0] > 0.5 \
                and T.is_seated(np, s["alpha"])[0] > 0.5:
            self.t_seated = t_rel
        if after_sit and self.t_stand is None and gz < stand_gz and z > C.MAIN["standup_z"] \
                and np.all(s["contact"][0] > 0.5):
            self.t_stand = t_rel

    def main_metrics(self):
        if not self.extra:
            return {}
        out = {"height_rmse": float(np.sqrt(self.e2 / self.ne)) if self.ne else None,
               "trunk_z_mean": self.zsum / self.ne if self.ne else None,
               "trunk_z_min": float(self.zmin) if self.ne else None, "peak_tau": self.tau}
        if self.tau_j is not None:
            out["peak_tau_joints"] = [float(v) for v in self.tau_j]
            out["peak_tau_basis"] = "|tau_net| / nominal tau_s"
        if self.d["cmd"][4] > 0.5:
            out["t_to_seated"] = self.t_seated
        if self.t_stand is not None or self.extra == "after_sit":
            out["t_to_stand"] = self.t_stand
        return out


def run(args):
    render = bool(args.video or args.gif)
    if render:
        os.environ.setdefault("MUJOCO_GL", "egl")
    from .env_cpu import VecEnv

    demo = args.demo
    zero_task = args.task or ("main" if demo in ("sit", "getup") else "walk")
    policy = load_policy(args.policy, task=zero_task if args.policy == "zero" else args.task)
    check_role(policy, ("walk", "main"), "--policy")
    task = policy.task
    if demo == "sit" and task != "main":
        raise ValueError("--demo sit bir main politikasi ister (verilen: %s)" % task)
    gpolicy = None
    if demo == "getup":
        if not args.getup_policy:
            raise ValueError("--demo getup icin --getup-policy gerekli")
        gpolicy = load_policy(args.getup_policy, task="getup")
    sched = build_schedule(args, task)
    total = sum(d for _, _, d in sched)
    n_steps = int(round(total / C.CTRL_DT))
    extra = {}
    if task != "walk":
        extra["reset_probs"] = {"home": 1.0}
    if demo in ("sit", "getup"):
        extra["max_episode_steps"] = n_steps + 1
    elif total > TK.get_task(task).episode_length_s:
        print("not: %.1f s > bolum suresi %.1f s; zaman asiminda robot sifirlanir" % (
            total, TK.get_task(task).episode_length_s))
    if demo == "getup":
        extra.update(terminate=False, model_variant="getup")     # dusunce sifirlama yok; devir icin ayni geomlar
    scene = getattr(args, "scene", None)
    sc = {"scene_path": scene} if scene else {}
    env = VecEnv(1, num_workers=0, seed=args.seed, randomize=False, noise=False, head_random=False, push=False,
                 fixed_command=sched[0][1], task=task, **dict(extra, **sc))
    genv = None
    ghead = None
    if gpolicy is not None:
        ghead = getattr(args, "getup_head", None) or (gpolicy.meta or {}).get("getup_head")
        gk = dict(sc, **({"getup_head": ghead} if ghead not in (None, "free") else {}))
        genv = VecEnv(1, num_workers=0, seed=args.seed + 1, randomize=False, noise=False, head_random=False,
                      push=False, task="getup", reset_probs={"stand": 1.0}, progress=1.0,
                      max_episode_steps=n_steps + 1, **gk)
    ridx = env.robot_index
    tau_nom = ridx.act["tau_s"][ridx.obs_act_id] if getattr(ridx, "act", None) is not None else None
    rec = None
    sup = Supervisor() if genv is not None else None
    events, getups = [], []
    traj = []
    try:
        obs, _ = env.reset()
        gobs = genv.reset()[0] if genv is not None else None
        active = env
        data = env.mj_data(0)
        from .robot_model import RobotIndex
        trunk = RobotIndex(env.mj_model(0)).trunk_body
        if render:
            rec = Recorder(args.width, args.height, trunk, shadows=not args.no_shadows, video=args.video,
                           gif=args.gif, scene_path=scene)

        bounds = np.cumsum([0.0] + [d for _, _, d in sched])
        after_sit = [task == "main" and i > 0 and sched[i - 1][1][4] > 0.5 and c[4] < 0.5
                     for i, (_, c, _) in enumerate(sched)]
        segs = [_Seg(n, c, d, None if task == "walk" else ("after_sit" if after_sit[i] else "main"), tau_nom)
                for i, (n, c, d) in enumerate(sched)]
        anchor = data.qpos[0:2].copy()      # net yer degistirme: segment basi ya da son sifirlamadan beri
        cur, next_frame, seg_t0 = 0, 0.0, 0.0
        hold_t, n_tip, t_tip = 0.0, 0, None
        if rec:
            rec.capture(data.qpos, "%s  cmd %s" % (sched[0][0], fmt_cmd(sched[0][1])))
            next_frame = 1.0 / FPS
        if args.qpos_out:
            traj.append(data.qpos.copy())
        for k in range(n_steps):
            t = k * C.CTRL_DT
            while cur + 1 < len(sched) and t >= bounds[cur + 1] - 1e-9:
                segs[cur]["disp"] += float(np.linalg.norm(data.qpos[0:2] - anchor))
                anchor = data.qpos[0:2].copy()
                cur += 1
                seg_t0 = bounds[cur]
                if active is env and hold_t <= 0.0:
                    obs, _ = env.set_commands(sched[cur][1][None], refresh_obs=True)
                if sched[cur][0] == TIP_SEGMENT and active is env:
                    tip_over(data, args.tip)
                    n_tip, t_tip = 1, t
            if genv is not None and active is env and n_tip and n_tip < TIP_MAX and sched[cur][0] == TIP_SEGMENT \
                    and t - t_tip >= TIP_RETRY_S - 1e-9 and not getups:
                tip_over(data, args.tip)            # politika dengeyi korudu: yeniden it
                n_tip, t_tip = n_tip + 1, t
            seg = segs[cur]
            p0 = data.qpos[0:3].copy()
            yaw0 = yaw_of(data.qpos[3:7])
            if active is env:
                obs, _, _, done, info = env.step(policy(obs))
            else:
                gobs, _, _, done, info = genv.step(gpolicy(gobs))
            if done[0]:
                seg["disp"] += float(np.linalg.norm(p0[:2] - anchor))
                anchor = data.qpos[0:2].copy()
                if info["time_outs"][0]:
                    seg["timeouts"] += 1
                else:
                    seg["falls"] += 1
                if rec:
                    rec.snap()
            else:
                dp = data.qpos[0:3] - p0
                yaw1 = yaw_of(data.qpos[3:7])
                c, s = np.cos(yaw0), np.sin(yaw0)
                vx = (c * dp[0] + s * dp[1]) / C.CTRL_DT
                vy = (-s * dp[0] + c * dp[1]) / C.CTRL_DT
                seg["v_sum"] += np.array([vx, vy, wrap_angle(yaw1 - yaw0) / C.CTRL_DT])
                seg["n"] += 1
                seg["path"] += float(np.hypot(dp[0], dp[1]))
            t1 = (k + 1) * C.CTRL_DT
            if task == "main" and active is env and not done[0]:
                seg.add_main(env.task_state(), t1 - seg_t0, seg.extra == "after_sit")
            if sup is not None:
                if active is env and hold_t > 0.0:
                    hold_t -= C.CTRL_DT
                    if hold_t <= 1e-9:
                        obs, _ = env.set_commands(sched[cur][1][None], refresh_obs=True)
                new = sup.update(active.task_state())
                if new == "getup":
                    gobs, _ = genv.set_state(0, data.qpos.copy(), data.qvel.copy(), warmstart=data.qacc_warmstart.copy())
                    active, data = genv, genv.mj_data(0)
                    events.append({"t": t1, "to": "getup"})
                    getups.append({"t_fall": t1, "t_upright": None, "fell_within_2s": False})
                    if any(g["t_upright"] is not None and t1 - g["t_upright"] <= AFTER_HANDOFF_S + 1e-9
                           for g in getups[:-1]):
                        getups[-2]["fell_within_2s"] = True
                elif new == "main":
                    env.set_state(0, data.qpos.copy(), data.qvel.copy(), warmstart=data.qacc_warmstart.copy())
                    zero = np.zeros(TK.get_task(task).num_commands)
                    if task == "main":
                        zero[3] = C.H_HOME
                    obs, _ = env.set_commands(zero[None], refresh_obs=True)
                    hold_t = C.SUPERVISOR["handoff_zero_cmd_s"]
                    active, data = env, env.mj_data(0)
                    events.append({"t": t1, "to": "main"})
                    getups[-1]["t_upright"] = t1
            if args.qpos_out:
                traj.append(data.qpos.copy())
            while rec and next_frame <= t1 + 1e-9:
                cmd = sched[cur][1]
                mode = "" if sup is None else ("  [%s]" % sup.mode)
                rec.capture(data.qpos, "%s  cmd %s  t %.1fs%s" % (sched[cur][0], fmt_cmd(cmd), t1, mode))
                next_frame += 1.0 / FPS
        segs[cur]["disp"] += float(np.linalg.norm(data.qpos[0:2] - anchor))
    finally:
        env.close()
        if genv is not None:
            genv.close()
        if rec is not None:
            rec.close()

    print("%-14s %5s | %22s | %22s | %5s %7s %7s" % ("segment", "s", "cmd vx vy wz", "achieved vx vy wz",
                                                      "falls", "disp m", "path m"))
    out = {"policy": args.policy, "seed": args.seed, "segments": []}
    for s in segs:
        v = (s["v_sum"] / s["n"]).tolist() if s["n"] else [float("nan")] * 3
        print("%-14s %5.1f | %+6.3f %+6.3f %+6.3f | %+6.3f %+6.3f %+6.3f | %5d %7.3f %7.3f" % (
            s["name"], s["seconds"], *s["cmd"][:3], *v, s["falls"], s["disp"], s["path"]))
        rec_ = {"name": s["name"], "seconds": s["seconds"], "command": s["cmd"], "achieved": v,
                "falls": s["falls"], "timeouts": s["timeouts"], "displacement": s["disp"], "path_length": s["path"]}
        mm = s.main_metrics()
        if mm:
            print("    h %.3f sit %d | height rmse %s  z mean %s min %s  peak tau %.2f%s%s" % (
                s["cmd"][3], int(s["cmd"][4]), _f3(mm["height_rmse"]), _f3(mm["trunk_z_mean"]), _f3(mm["trunk_z_min"]),
                mm["peak_tau"], ("  t_to_seated %s" % _f3(mm["t_to_seated"])) if "t_to_seated" in mm else "",
                ("  t_to_stand %s" % _f3(mm["t_to_stand"])) if "t_to_stand" in mm else ""))
            rec_.update(mm)
        out["segments"].append(rec_)
    out["falls"] = sum(s["falls"] for s in segs)
    out["displacement"] = sum(s["disp"] for s in segs)
    out["path_length"] = sum(s["path"] for s in segs)
    print("toplam: %d dusme, net yer degistirme %.3f m (segment toplami), govde yolu %.3f m, %.1f s" % (
        out["falls"], out["displacement"], out["path_length"], total))
    if task != "walk" or demo in ("sit", "getup"):
        out["task"] = task
        out["demo"] = demo
    if scene:
        out["scene"] = scene
    if tau_nom is not None:
        out["robot"] = ridx.robot
    if sup is not None:
        ups = [g["t_upright"] - g["t_fall"] for g in getups if g["t_upright"] is not None]
        out["getup_policy"] = args.getup_policy
        if ghead is not None:
            out["getup_head"] = ghead
        out["supervisor"] = {
            "events": events, "getups": getups, "tips": n_tip,
            "t_to_upright": ups[0] if ups else None,
            "falls_after_handoff": sum(1 for g in getups if g["fell_within_2s"]),
            "success": bool(ups), "final_mode": sup.mode,
        }
        print("supervizor: %d devrilme, kalkis %s, donusten sonra 2 s icinde dusme %d, son mod %s" % (
            len(getups), ", ".join("%.2f s" % u for u in ups) or "yok", out["supervisor"]["falls_after_handoff"],
            sup.mode))
    if args.qpos_out:
        np.save(args.qpos_out, np.array(traj))
        print("qpos: %s (%d kare)" % (args.qpos_out, len(traj)))
    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=1)
    return out


def _f3(v):
    return "-" if v is None else "%.3f" % v


def main(argv=None):
    run(parse_args(argv))


if __name__ == "__main__":
    main()
