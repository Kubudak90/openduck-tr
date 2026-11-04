# -*- coding: utf-8 -*-
"""
PPO / egitim dongusu / ONNX testleri. Gercek robot ortami gerekmez.

  python3 -m rl.tests.test_ppo          (pytest olmadan)
  python3 -m pytest rl/tests/test_ppo.py
"""
import copy
import json
import math
import os
import sys
import tempfile
import time

os.environ.setdefault("MUJOCO_GL", "egl")      # mujoco import edilmeden once (play goruntuleme testi)

import numpy as np
import torch
from torch.distributions import Normal, kl_divergence

from rl import config as C
from rl import export_onnx
from rl.export_onnx import check_onnx, export_policy, load_actor_critic, meta_path_for, policy_meta
from rl.ppo import PPO, ActorCritic, EmpiricalNormalization, RolloutStorage, gaussian_log_prob, set_torch_threads
from rl.train import Runner, effective_cpus, parse_args

SMALL = {"actor_hidden": [64, 64], "critic_hidden": [64, 64]}


class PointMassVecEnv(object):
    """2B nokta kutle hedef takibi, rl.env_cpu.VecEnv ile ayni arayuz.
    Aksiyon: hiz komutu (2,), [-1,1]'e kirpilir; konum += 0.1 * a. Odul: -0.1 * |hedef - konum|.
    Aktor gozlemi [hedef - konum, onceki aksiyon] (4); kritik gozlemi + mutlak konum (6).
    Bolum MAX adimda zaman asimi ile, |konum| > 3 olursa dusme (-1 ceza) ile biter."""

    num_actions = 2
    MAX = 40

    def __init__(self, num_envs, seed=0):
        self.N = num_envs
        self.rng = np.random.default_rng(seed)
        self.pos = np.zeros((num_envs, 2))
        self.target = np.zeros((num_envs, 2))
        self.last_a = np.zeros((num_envs, 2))
        self.t = np.zeros(num_envs, np.int64)
        self.ep_rew = np.zeros(num_envs)

    def _reset(self, ids):
        n = len(ids)
        self.pos[ids] = self.rng.uniform(-1, 1, (n, 2))
        self.target[ids] = self.rng.uniform(-1, 1, (n, 2))
        self.last_a[ids] = 0.0
        self.t[ids] = 0
        self.ep_rew[ids] = 0.0

    def _obs(self):
        obs = np.concatenate([self.target - self.pos, self.last_a], 1).astype(np.float32)
        return obs, np.concatenate([obs, self.pos], 1).astype(np.float32)

    def reset(self):
        self._reset(np.arange(self.N))
        return self._obs()

    def step(self, actions):
        a = np.clip(np.asarray(actions, np.float64), -1.0, 1.0)
        self.pos += 0.1 * a
        self.last_a = a
        self.t += 1
        dist = np.linalg.norm(self.target - self.pos, axis=1)
        term = np.abs(self.pos).max(1) > 3.0
        rew = -0.1 * dist - 1.0 * term
        self.ep_rew += rew
        timeout = (self.t >= self.MAX) & ~term
        done = term | timeout
        info = {"time_outs": timeout}
        ids = np.flatnonzero(done)
        if len(ids):
            info["episode"] = {"ep_len": float(self.t[ids].mean()), "rew_dist": float(self.ep_rew[ids].mean())}
            self._reset(ids)
        obs, cobs = self._obs()
        return obs, cobs, rew.astype(np.float32), done, info

    def close(self):
        pass


def _quiet(*a, **k):
    pass


# --- GAE ---------------------------------------------------------------------------
def test_gae_hand_computed():
    # T=3, N=1, gamma=lam=0.5; t=1'de bolum biter
    st = RolloutStorage(1, 3, 1, 1, 1)
    st.rewards[:, 0] = torch.tensor([1.0, 2.0, 3.0])
    st.values[:, 0] = torch.tensor([0.5, 1.0, 1.5])
    st.dones[:, 0] = torch.tensor([0.0, 1.0, 0.0])
    st.compute_returns(torch.tensor([2.0]), gamma=0.5, lam=0.5, normalize_advantages=False)
    # d2 = 3 + .5*2 - 1.5 = 2.5 ; d1 = 2 - 1 = 1 (done) ; d0 = 1 + .5*1 - .5 = 1 ; A0 = 1 + .25*1
    assert torch.allclose(st.advantages[:, 0], torch.tensor([1.25, 1.0, 2.5]))
    assert torch.allclose(st.returns[:, 0], torch.tensor([1.75, 2.0, 4.0]))


def test_gae_matches_reference():
    rng = np.random.default_rng(1)
    T, N, gamma, lam = 7, 5, 0.97, 0.9
    r = rng.normal(size=(T, N))
    v = rng.normal(size=(T, N))
    d = (rng.uniform(size=(T, N)) < 0.25).astype(np.float64)
    last = rng.normal(size=N)
    st = RolloutStorage(N, T, 1, 1, 1)
    st.rewards.copy_(torch.tensor(r))
    st.values.copy_(torch.tensor(v))
    st.dones.copy_(torch.tensor(d))
    st.compute_returns(torch.tensor(last, dtype=torch.float32), gamma, lam, normalize_advantages=False)
    # dogrudan toplam: A_t = sum_l (gamma*lam)^l delta_{t+l}, bolum sonunda kesilir
    ref = np.zeros((T, N))
    for n in range(N):
        for t in range(T):
            acc, k = 0.0, 1.0
            for u in range(t, T):
                nv = last[n] if u == T - 1 else v[u + 1, n]
                delta = r[u, n] + gamma * nv * (1.0 - d[u, n]) - v[u, n]
                acc += k * delta
                if d[u, n]:
                    break
                k *= gamma * lam
            ref[t, n] = acc
    assert np.allclose(st.advantages.numpy(), ref, atol=1e-5)
    assert np.allclose(st.returns.numpy(), ref + v, atol=1e-5)
    st.compute_returns(torch.tensor(last, dtype=torch.float32), gamma, lam, normalize_advantages=True)
    a = st.advantages.numpy()
    assert abs(a.mean()) < 1e-5 and abs(a.std(ddof=1) - 1.0) < 1e-4


def test_timeout_bootstrap():
    torch.manual_seed(0)
    T, N = 4, 3
    ac = ActorCritic(4, 6, 2, **SMALL)
    alg = PPO(ac, N, num_steps_per_env=T)
    cobs_seq, rew_seq, to_seq = [], [], []
    for t in range(T):
        obs, cobs = torch.randn(N, 4), torch.randn(N, 6)
        alg.act(obs, cobs)
        rew = torch.randn(N)
        to = torch.tensor([t == 2, False, t == 3])
        alg.process_env_step(rew, to.clone(), to)
        cobs_seq.append(cobs)
        rew_seq.append(rew)
        to_seq.append(to.float())
    last = torch.randn(N, 6)
    alg.compute_returns(last)
    with torch.no_grad():
        # act() normalizatoru her adimda gunceller; depolanan kritik gozlemi o anki istatistikle normalize
        v = ac.critic(alg.storage.critic_obs.reshape(T * N, -1)).reshape(T, N)
    expect = torch.stack(rew_seq) + alg.gamma * v * torch.stack(to_seq)
    assert torch.allclose(alg.storage.rewards, expect, atol=1e-6)
    assert torch.allclose(alg.storage.values, v, atol=1e-6)
    assert alg.storage.rewards[2, 0] != rew_seq[2][0]


def test_gae_termination_vs_timeout():
    # tam PPO yolu: env0 t=1'de duser (bootstrap yok), env1 t=1'de zaman asimi (r += gamma V(s_t)),
    # env2 devam eder (son degerden bootstrap). returns - values = normalize edilmemis avantaj.
    torch.manual_seed(0)
    T, N, gamma, lam = 4, 3, 0.9, 0.8
    ac = ActorCritic(4, 6, 2, **SMALL)
    alg = PPO(ac, N, num_steps_per_env=T, cfg={"gamma": gamma, "lam": lam})
    done, to = torch.zeros(T, N), torch.zeros(T, N)
    done[1, 0] = done[1, 1] = to[1, 1] = 1.0
    rew = torch.randn(T, N)
    for t in range(T):
        alg.act(torch.randn(N, 4), torch.randn(N, 6))
        alg.process_env_step(rew[t].numpy(), done[t].numpy().astype(bool), to[t].numpy().astype(bool))
    last = torch.randn(N, 6)
    alg.compute_returns(last)
    st = alg.storage
    v = st.values
    with torch.no_grad():
        lv = ac.value(last)
    A = st.returns - v
    k = gamma * lam
    assert torch.allclose(A[1, 0], rew[1, 0] - v[1, 0], atol=1e-5)
    assert torch.allclose(A[1, 1], rew[1, 1] + gamma * v[1, 1] - v[1, 1], atol=1e-5)
    assert torch.allclose(A[0, 0], rew[0, 0] + gamma * v[1, 0] - v[0, 0] + k * (rew[1, 0] - v[1, 0]), atol=1e-5)
    d3 = rew[3, 2] + gamma * lv[2] - v[3, 2]
    d2 = rew[2, 2] + gamma * v[3, 2] - v[2, 2]
    assert torch.allclose(A[3, 2], d3, atol=1e-5) and torch.allclose(A[2, 2], d2 + k * d3, atol=1e-5)
    adv = st.advantages
    assert abs(adv.mean().item()) < 1e-5 and abs(adv.std().item() - 1.0) < 1e-4


def test_distribution_math():
    torch.manual_seed(0)
    mu, ls, a = torch.randn(50, 10), 0.3 * torch.randn(10), torch.randn(50, 10)
    d = Normal(mu, ls.exp().expand_as(mu))
    assert torch.allclose(gaussian_log_prob(a, mu, ls.expand_as(mu)), d.log_prob(a).sum(-1), atol=1e-5)
    ent = (0.5 + 0.5 * math.log(2.0 * math.pi) + ls.expand_as(mu)).sum(-1)
    assert torch.allclose(ent, d.entropy().sum(-1), atol=1e-5)
    # PPO.update'teki KL(eski||yeni) (rsl_rl, log icinde +1e-5)
    mu2, ls2 = mu + 0.1 * torch.randn_like(mu), ls + 0.05 * torch.randn(10)
    s, s2 = ls.exp().expand_as(mu), ls2.exp().expand_as(mu)
    kl = torch.sum(torch.log(s2 / s + 1e-5) + (s ** 2 + (mu - mu2) ** 2) / (2.0 * s2 ** 2) - 0.5, dim=-1)
    ref = kl_divergence(Normal(mu, s), Normal(mu2, s2)).sum(-1)
    assert torch.allclose(kl, ref, atol=5e-4)


def test_update_consistency_and_losses():
    # depolanan log-olasilik/degerler guncellemede yeniden hesaplananlarla ayni; kayiplar elle hesaplananla ayni
    torch.manual_seed(1)
    T, N = 3, 4
    ac = ActorCritic(6, 9, 3, [16], [16])
    cfg = {"num_mini_batches": 1, "num_learning_epochs": 1, "entropy_coef": 0.0, "schedule": "fixed"}
    alg = PPO(ac, N, num_steps_per_env=T, cfg=cfg)
    for t in range(T):
        alg.act(3.0 * torch.randn(N, 6) + 1.0, 2.0 * torch.randn(N, 9) - 1.0)
        alg.process_env_step(5.0 * torch.randn(N), torch.zeros(N), torch.zeros(N))
    alg.compute_returns(torch.randn(N, 9))
    st = alg.storage
    B = T * N
    obs, cobs, act = st.obs.reshape(B, -1), st.critic_obs.reshape(B, -1), st.actions.reshape(B, -1)
    oldv, ret, adv, olp = (x.reshape(B) for x in (st.values, st.returns, st.advantages, st.log_probs))
    with torch.no_grad():
        mu = ac.actor(obs)
        lp = gaussian_log_prob(act, mu, ac.log_std.expand_as(mu))
        v = ac.critic(cobs).squeeze(-1)
    assert torch.allclose(lp, olp, atol=1e-5) and torch.allclose(v, oldv, atol=1e-6)
    # toplamadan sonra agi boz: oranlar ve deger farklari kirpma araligi disina tassin
    with torch.no_grad():
        ac.actor[-1].weight.add_(0.5 * torch.randn_like(ac.actor[-1].weight))
        ac.critic[-1].weight.add_(0.3 * torch.randn_like(ac.critic[-1].weight))
        ac.critic[-1].bias.add_(0.5)
        mu = ac.actor(obs)
        lp = gaussian_log_prob(act, mu, ac.log_std.expand_as(mu))
        v = ac.critic(cobs).squeeze(-1)
    ratio = torch.exp(lp - olp)
    clipped = ((ratio - 1.0).abs() > 0.2).float().mean().item()
    assert 0.0 < clipped < 1.0 and ((v - oldv).abs() > 0.2).any() and ((v - oldv).abs() < 0.2).any()
    sur = torch.max(-adv * ratio, -adv * ratio.clamp(0.8, 1.2)).mean()
    vc = oldv + (v - oldv).clamp(-0.2, 0.2)
    vl = torch.max((v - ret) ** 2, (vc - ret) ** 2).mean()
    out = alg.update()
    assert abs(out["surrogate_loss"] - sur.item()) < 1e-5 and abs(out["value_loss"] - vl.item()) < 1e-4
    assert abs(out["clip_frac"] - clipped) < 1e-6 and out["lr_mean"] == out["lr"]
    # kirpmasiz deger kaybi farkli olmali (kirpma gercekten etkili)
    assert abs(((v - ret) ** 2).mean().item() - vl.item()) > 1e-3


def test_empirical_normalization():
    rng = np.random.default_rng(0)
    x = rng.normal(3.0, 2.0, size=(1000, 5)) * np.array([1, 10, 0.1, 1, 1])
    n = EmpiricalNormalization(5)
    for chunk in np.array_split(x, 7):
        n.update(torch.tensor(chunk, dtype=torch.float32))
    assert np.allclose(n.mean.numpy(), x.mean(0), rtol=1e-4, atol=1e-4)
    assert np.allclose(n.std.numpy(), x.std(0), rtol=1e-3)
    assert int(n.count.item()) == 1000


# --- ONNX --------------------------------------------------------------------------
def test_onnx_export_matches_torch():
    import onnx
    import onnxruntime as ort
    torch.manual_seed(0)
    ac = ActorCritic(C.NUM_OBS, C.NUM_CRITIC_OBS, C.NUM_ACTIONS)
    ac.actor_obs_normalizer.update(torch.randn(4000, C.NUM_OBS) * torch.rand(C.NUM_OBS) * 4 + torch.randn(C.NUM_OBS))
    with torch.no_grad():
        for p in ac.actor.parameters():
            p.add_(0.05 * torch.randn_like(p))
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "policy.onnx")
        meta = policy_meta(1234)
        err = export_policy(ac, path, meta=meta)
        assert err < 1e-5 and check_onnx(ac, path, n=33, seed=5) < 1e-5
        m = onnx.load(path)
        assert [i.name for i in m.graph.input] == ["obs"] and [o.name for o in m.graph.output] == ["action"]
        dims = [d_.dim_param or d_.dim_value for d_ in m.graph.input[0].type.tensor_type.shape.dim]
        assert dims == ["batch", C.NUM_OBS]
        sess = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
        for b in (1, 7, 1000):
            x = torch.randn(b, C.NUM_OBS) * 5
            with torch.no_grad():
                ref = ac.act_inference(x).numpy()
            out = sess.run(None, {"obs": x.numpy()})[0]
            assert out.shape == (b, C.NUM_ACTIONS) and out.dtype == np.float32
            assert np.abs(out - ref).max() < 1e-5
        with open(meta_path_for(path)) as f:
            mj = json.load(f)
        assert set(mj) == {"obs_dim", "action_dim", "ctrl_dt", "action_scale", "action_clip", "default_leg_q",
                           "leg_joints", "obs_joints", "default_obs_q", "gait_freq", "obs_scales", "backend",
                           "steps"}
        assert mj["obs_dim"] == 49 and mj["action_dim"] == 10 and mj["steps"] == 1234
        assert len(mj["default_leg_q"]) == 10 and len(mj["default_obs_q"]) == 14
        # normalizasyon kapali model
        ac2 = ActorCritic(C.NUM_OBS, C.NUM_CRITIC_OBS, C.NUM_ACTIONS, obs_normalization=False, activation="tanh")
        assert export_policy(ac2, os.path.join(d, "p2.onnx")) < 1e-5
        # buyuk cikisli ag: sinir goreli (atol * max|aksiyon|), sahte hata vermemeli
        ac3 = copy.deepcopy(ac)
        with torch.no_grad():
            ac3.actor[-1].weight.mul_(300.0)
        export_policy(ac3, os.path.join(d, "p3.onnx"))
        # normalizator disa aktarma aninda donar: sonradan guncellemek ONNX'i degistirmez
        x = (torch.randn(5, C.NUM_OBS) * 3).numpy()
        before = sess.run(None, {"obs": x})[0]
        ac.actor_obs_normalizer.update(torch.randn(500, C.NUM_OBS) * 10 + 5)
        after = ort.InferenceSession(path, providers=["CPUExecutionProvider"]).run(None, {"obs": x})[0]
        assert np.array_equal(before, after)


def test_export_validates_before_replace():
    # dogrulama basarisizsa onceki policy.onnx ve meta oldugu gibi kalir, gecici dosya kalmaz
    torch.manual_seed(0)
    ac = ActorCritic(C.NUM_OBS, C.NUM_CRITIC_OBS, C.NUM_ACTIONS, **SMALL)
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "policy.onnx")
        export_policy(ac, path, meta=policy_meta(1))
        with open(path, "rb") as f:
            old = f.read()
        with torch.no_grad():
            for p in ac.actor.parameters():
                p.add_(1.0)
        orig = export_onnx.check_onnx

        def bad_check(*a, **k):
            raise RuntimeError("test")
        export_onnx.check_onnx = bad_check
        try:
            export_policy(ac, path, meta=policy_meta(2))
            raise AssertionError("hata bekleniyordu")
        except RuntimeError as e:
            assert str(e) == "test"
        finally:
            export_onnx.check_onnx = orig
        with open(path, "rb") as f:
            assert f.read() == old
        with open(meta_path_for(path)) as f:
            assert json.load(f)["steps"] == 1
        assert sorted(os.listdir(d)) == ["policy.onnx", "policy_meta.json"]
        assert export_policy(ac, path, meta=policy_meta(3)) is not None
        with open(path, "rb") as f:
            assert f.read() != old


def test_cli_cpu_defaults():
    n = effective_cpus()
    assert n >= 1
    a = parse_args(["--out", "x"])
    assert a.workers == n and a.update_threads == max(1, min(16, n // 2))
    a = parse_args(["--out", "x", "--workers", "3", "--update-threads", "5"])
    assert a.workers == 3 and a.update_threads == 5


# --- ogrenme -----------------------------------------------------------------------
def _evaluate(ac, n=256, seed=123):
    env = PointMassVecEnv(n, seed=seed)
    obs, _ = env.reset()
    dists = []
    for t in range(PointMassVecEnv.MAX - 1):
        with torch.no_grad():
            a = ac.act_inference(torch.from_numpy(obs)).numpy()
        obs, _, _, _, _ = env.step(a)
        if t >= PointMassVecEnv.MAX - 11:
            dists.append(np.linalg.norm(env.target - env.pos, axis=1).mean())
    return float(np.mean(dists))


def test_ppo_learns_point_mass():
    set_torch_threads(2)
    with tempfile.TemporaryDirectory() as d:
        env = PointMassVecEnv(64, seed=0)
        runner = Runner(env, d, cfg=SMALL, seed=0, save_interval=10, threads=1, update_threads=2,
                        print_fn=_quiet)
        before = _evaluate(runner.alg.ac)
        t0 = time.time()
        runner.learn(max_steps=30 * runner.batch_size)
        dt = time.time() - t0
        after = _evaluate(runner.alg.ac)
        print("point mass: son 10 adim ort. mesafe %.3f -> %.3f, %.1f s" % (before, after, dt))
        assert dt < 120.0
        assert before > 0.5 and after < 0.12, (before, after)

        with open(os.path.join(d, "log.jsonl")) as f:
            log = [json.loads(line) for line in f]
        assert [r["iter"] for r in log] == list(range(1, 31))
        assert log[-1]["steps"] == 30 * runner.batch_size
        for k in ("fps", "wall_time", "reward_per_step", "ep_len", "ep_return", "kl", "lr", "action_std",
                  "value_loss", "surrogate_loss", "entropy", "episode"):
            assert k in log[-1], k
        assert "rew_dist" in log[-1]["episode"] and abs(log[-1]["episode"]["ep_len"] - 40) < 1e-6
        assert log[-1]["reward_per_step"] > log[0]["reward_per_step"] + 0.03
        for f_ in ("model_10.pt", "model_20.pt", "model_30.pt", "latest.pt", "policy.onnx"):
            assert os.path.exists(os.path.join(d, f_)), f_

        # devam: ayni agirliklar, normalizator ve optimizer; sayaclar surer
        ac2, ckpt = load_actor_critic(os.path.join(d, "latest.pt"))
        assert ckpt["iter"] == 30 and "optimizer" in ckpt
        assert torch.equal(ac2.actor_obs_normalizer.mean, runner.alg.ac.actor_obs_normalizer.mean)
        r2 = Runner(PointMassVecEnv(64, seed=1), d, cfg=SMALL, seed=1, save_interval=0,
                    resume=os.path.join(d, "latest.pt"), print_fn=_quiet)
        assert r2.iter == 30 and r2.steps == runner.steps and r2.alg.lr == runner.alg.lr
        for p, q in zip(r2.alg.ac.parameters(), runner.alg.ac.parameters()):
            assert torch.equal(p, q)
        b1, b2 = dict(runner.alg.ac.named_buffers()), dict(r2.alg.ac.named_buffers())
        assert set(b1) == set(b2) and all(torch.equal(b1[k], b2[k]) for k in b1)
        assert b2["actor_obs_normalizer.count"].item() == runner.steps
        o1, o2 = runner.alg.optimizer.state_dict(), r2.alg.optimizer.state_dict()
        assert o2["param_groups"][0]["lr"] == runner.alg.lr and set(o1["state"]) == set(o2["state"])
        for k in o1["state"]:
            for n in ("step", "exp_avg", "exp_avg_sq"):
                assert torch.equal(o1["state"][k][n], o2["state"][k][n])
        assert torch.equal(torch.get_rng_state(), ckpt["torch_rng"])
        r2.learn(max_steps=33 * runner.batch_size)
        assert r2.iter == 33 and os.path.exists(os.path.join(d, "model_33.pt"))
        with open(os.path.join(d, "log.jsonl")) as f:
            assert len(f.readlines()) == 33
        assert _evaluate(r2.alg.ac) < 0.15


def test_stop_request_saves_checkpoint():
    class StopAfter(PointMassVecEnv):
        runner, k = None, 0

        def step(self, actions):
            self.k += 1
            if self.k == 30:
                self.runner.request_stop()
            return PointMassVecEnv.step(self, actions)

    with tempfile.TemporaryDirectory() as d:
        env = StopAfter(16)
        runner = Runner(env, d, cfg=SMALL, save_interval=0, print_fn=_quiet)
        env.runner = runner
        runner.learn(max_steps=1e9)
        assert runner.iter == 1                                # 24 adimlik 1. iterasyon bitti, 2.si kesildi
        assert runner.alg.storage.step == 0
        assert os.path.exists(os.path.join(d, "model_1.pt")) and os.path.exists(os.path.join(d, "policy.onnx"))


def test_real_env_smoke():
    """Gercek CPU ortami varsa: 1 iterasyon egitim (16 ortam, surecsiz), ONNX, sonra 1 s oynatma."""
    try:
        from rl.env_cpu import VecEnv
    except ImportError as e:
        print("atlandi: rl.env_cpu yok (%s)" % e)
        return
    from rl import play
    from rl.robot_model import RobotIndex, build_model
    idx = RobotIndex(build_model())
    with tempfile.TemporaryDirectory() as d:
        env = VecEnv(16, num_workers=0, seed=0)
        try:
            r = Runner(env, d, seed=0, save_interval=0, print_fn=_quiet,
                       meta_fn=lambda steps: policy_meta(steps, "cpu_torch", idx))
            r.learn(max_steps=1)
        finally:
            env.close()
        assert r.iter == 1
        with open(os.path.join(d, "policy_meta.json")) as f:
            assert json.load(f)["steps"] == 16 * C.PPO["num_steps_per_env"]
        out = play.run(play.parse_args(["--policy", os.path.join(d, "policy.onnx"), "--seconds", "1"]))
        assert len(out["segments"]) == 1 and all(math.isfinite(v) for v in out["segments"][0]["achieved"])
        # oynatma, egitim modelinin qpos'unu gorsel modele kopyalar: ayni eklem/govde duzeni sart
        import mujoco
        from rl.env_cpu import physics_model
        pm, vm = physics_model(), build_model(visual=True)
        names = lambda m, o, n: [mujoco.mj_id2name(m, o, i) for i in range(n)]
        assert pm.nq == vm.nq and np.array_equal(pm.jnt_qposadr, vm.jnt_qposadr)
        assert names(pm, mujoco.mjtObj.mjOBJ_JOINT, pm.njnt) == names(vm, mujoco.mjtObj.mjOBJ_JOINT, vm.njnt)
        assert names(pm, mujoco.mjtObj.mjOBJ_BODY, pm.nbody) == names(vm, mujoco.mjtObj.mjOBJ_BODY, vm.nbody)
        # kisa goruntuleme: tek boyutlu mp4 (yuv420p cift boyut ister) + gif, demo programi, .pt politika
        if os.environ.get("MUJOCO_GL") != "egl":
            print("goruntuleme atlandi: MUJOCO_GL=%s" % os.environ.get("MUJOCO_GL"))
            return
        import imageio.v2 as iio
        mp4, gif = os.path.join(d, "a.mp4"), os.path.join(d, "a.gif")
        out = play.run(play.parse_args(["--policy", os.path.join(d, "latest.pt"), "--demo", "--seconds", "2.2",
                                        "--video", mp4, "--gif", gif, "--width", "161", "--height", "121",
                                        "--no-shadows"]))
        assert [s["name"] for s in out["segments"]] == ["stand", "walk forward"]
        frames = iio.mimread(mp4)
        assert len(frames) == int(round(2.2 * play.FPS)) + 1 and frames[0].shape[:2] == (122, 162)
        assert float(np.std(frames[-1])) > 5.0           # bos/siyah olmayan kare
        assert os.path.getsize(gif) > 0


# --- main / getup gorevleri ----------------------------------------------------------
def test_task_export_meta():
    """getup: 48 -> 14 ONNX, getup.onnx -> getup_meta.json (task, 14 aksiyon eklemi, eklem basina olcek);
    main: 53 -> 10, duzen ve filtre sabitleri; export_onnx CLI gorevi checkpoint'tan alir, uyusmazlik reddedilir."""
    import onnx
    from rl import tasks as TK
    from rl.train import task_ppo_cfg
    for task, (no, na) in (("getup", (48, 14)), ("main", (53, 10))):
        spec = TK.get_task(task)
        torch.manual_seed(0)
        ac = ActorCritic(spec.num_obs, spec.num_critic_obs, spec.num_actions, **SMALL)
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "%s.onnx" % task)
            assert export_policy(ac, path, meta=policy_meta(5, "cpu_torch", task=task)) < 1e-5
            m = onnx.load(path)
            dims = lambda v: [x.dim_param or x.dim_value for x in v.type.tensor_type.shape.dim]
            assert dims(m.graph.input[0]) == ["batch", no] and dims(m.graph.output[0]) == ["batch", na]
            with open(meta_path_for(path)) as f:
                mj = json.load(f)
            assert os.path.basename(meta_path_for(path)) == "%s_meta.json" % task
            assert mj["task"] == task and mj["obs_dim"] == no and mj["action_dim"] == na and mj["version"] >= 2
            assert [tuple(x) for x in mj["obs_layout"]] == [tuple(x) for x in spec.obs_layout]
            assert len(mj["action_joints"]) == na and len(mj["action_scale"]) == na and len(mj["default_action_q"]) == na
            if task == "getup":
                assert mj["action_mode"] == "home_clip" and mj["action_clip"] == 1.0 and max(mj["action_scale"]) > 1.0
                assert mj["head_control"] == "policy" and mj["command_names"] == []
            else:
                assert mj["action_mode"] == "ref" and "posture_filter" in mj and "posture_ref" in mj
                assert mj["command_names"] == ["vx", "vy", "wz", "h", "sit"]
            # CLI: checkpoint'taki gorev
            cfg = task_ppo_cfg(task)
            alg = PPO(ac, 2, cfg=dict(cfg, **SMALL))
            ck = alg.state_dict()
            ck.update(iter=1, steps=99, backend="cpu_torch", task=task, run_info={"task": task})
            cp = os.path.join(d, "latest.pt")
            torch.save(ck, cp)
            export_onnx.main(["--checkpoint", cp])
            with open(os.path.join(d, "policy_meta.json")) as f:
                mj = json.load(f)
            assert mj["task"] == task and mj["steps"] == 99
            other = "main" if task == "getup" else "getup"
            try:
                export_onnx.main(["--checkpoint", cp, "--task", other])
                raise AssertionError("task/arch mismatch accepted")
            except SystemExit:
                pass
    # walk meta: export_onnx.policy_meta == rl.policy_meta.build_meta (bugunku alanlar)
    from rl import policy_meta as PM
    assert policy_meta(3) == PM.build_meta(3, "cpu_torch") and "task" not in policy_meta(3)


def test_task_ppo_cfg_and_cli():
    from rl import tasks as TK
    from rl.train import task_ppo_cfg
    assert task_ppo_cfg("walk") is None
    g = task_ppo_cfg("getup")
    assert g == {"gamma": 0.99, "actor_hidden": [512, 256, 128], "critic_hidden": [512, 256, 128],
                 "init_noise_std": TK.get_task("getup").init_noise_std}
    assert task_ppo_cfg("main") == {"gamma": TK.get_task("main").gamma}
    assert parse_args(["--out", "x"]).task == "walk" and parse_args(["--out", "x", "--task", "getup"]).task == "getup"
    a = parse_args(["--out", "x"])
    assert not hasattr(a, "init_noise_std") and not hasattr(a, "entropy_coef")
    a = parse_args(["--out", "x", "--init-noise-std", "0.5", "--entropy-coef", "0.01"])
    assert a.init_noise_std == 0.5 and a.entropy_coef == 0.01
    from rl.train import PPO_ARG_KEYS
    assert all(not hasattr(parse_args(["--out", "x"]), k) for k in PPO_ARG_KEYS)
    a = parse_args(["--out", "x", "--learning-rate", "3e-4", "--schedule", "fixed", "--desired-kl", "0.02",
                    "--gamma", "0.97", "--num-steps-per-env", "32"])
    assert (a.learning_rate, a.schedule, a.desired_kl, a.gamma, a.num_steps_per_env) == (3e-4, "fixed", 0.02, 0.97, 32)
    assert all(k in C.PPO for k in PPO_ARG_KEYS)


def test_real_env_task_smoke():
    """main / getup gercek CPU ortaminda 2 iterasyon egitim (16 ortam), meta gorevli; getup mufredat ilerlemesi
    egitici tarafindan ayarlanir; disa aktarilan politikalar play ile oynatilir."""
    from rl import play
    from rl import tasks as TK
    from rl.env_cpu import VecEnv
    from rl.robot_model import RobotIndex, build_model
    from rl.train import task_ppo_cfg
    paths = {}
    with tempfile.TemporaryDirectory() as d:
        for task in ("main", "getup"):
            spec = TK.get_task(task)
            idx = RobotIndex(build_model(variant=spec.model_variant), list(spec.action_joints))
            out = os.path.join(d, task)
            env = VecEnv(16, num_workers=0, seed=0, task=task, bank_n={"seated": 16, "fallen": 32, "midfall": 16})
            try:
                cfg = dict(task_ppo_cfg(task), **SMALL)
                r = Runner(env, out, cfg=cfg, seed=0, save_interval=0, print_fn=_quiet,
                           meta_fn=lambda steps, t=task, i=idx: policy_meta(steps, "cpu_torch", i, t))
                r.learn(max_steps=2 * r.batch_size)
                if task == "getup":
                    assert abs(env.progress - 0.5) < 1e-9 and env._core.weights["head_impact"] == 0.0
            finally:
                env.close()
            assert r.iter == 2 and r.alg.gamma == spec.gamma
            with open(os.path.join(out, "policy_meta.json")) as f:
                mj = json.load(f)
            assert mj["task"] == task and mj["obs_dim"] == spec.num_obs and mj["action_dim"] == spec.num_actions
            ck = torch.load(os.path.join(out, "latest.pt"), map_location="cpu", weights_only=False)
            assert ck["task"] == task
            paths[task] = os.path.join(out, "policy.onnx")
        o = play.run(play.parse_args(["--policy", paths["main"], "--demo", "sit", "--seconds", "1"]))
        assert o["task"] == "main" and len(o["segments"][0]["command"]) == 5
        o = play.run(play.parse_args(["--policy", paths["main"], "--getup-policy", paths["getup"], "--demo", "getup",
                                      "--seconds", "1"]))
        assert o["getup_policy"] == paths["getup"]


def _run_all():
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for name, fn in tests:
        t0 = time.time()
        try:
            fn()
            print("PASS %-40s %.1fs" % (name, time.time() - t0), flush=True)
        except Exception:
            failed += 1
            import traceback
            traceback.print_exc()
            print("FAIL %s" % name, flush=True)
    print("%d/%d gecti" % (len(tests) - failed, len(tests)))
    return failed


if __name__ == "__main__":
    sys.exit(1 if _run_all() else 0)
