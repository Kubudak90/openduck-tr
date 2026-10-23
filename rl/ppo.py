# -*- coding: utf-8 -*-
"""
CPU PPO (PyTorch), rsl_rl tarzinda.

- ActorCritic: asimetrik (aktor NUM_OBS, kritik NUM_CRITIC_OBS girdi), durumdan bagimsiz
  ogrenilen log-std, aktor ve kritik girdileri icin calisan (empirik) gozlem normalizasyonu.
- RolloutStorage: (T, N) tamponlar, GAE, mini-batch uretici.
- PPO: kirpilmis vekil (surrogate) + kirpilmis deger kaybi, entropi bonusu, gradyan kirpma,
  KL'ye gore uyarlanan ogrenme orani. Zaman asiminda bootstrap: r += gamma * V(s) * time_out.

Depolamada normalize edilmis gozlemler tutulur (toplama anindaki istatistiklerle), boylece
guncelleme sirasindaki log-olasiliklar toplama anindakilerle tutarli kalir.
"""
import math

import torch
import torch.nn as nn

from . import config as C

ACTIVATIONS = {
    "elu": nn.ELU, "relu": nn.ReLU, "tanh": nn.Tanh, "selu": nn.SELU,
    "sigmoid": nn.Sigmoid, "leaky_relu": nn.LeakyReLU,
}


def set_torch_threads(n):
    torch.set_num_threads(max(1, int(n)))
    try:
        torch.set_num_interop_threads(1)
    except RuntimeError:
        pass      # paralel is basladiktan sonra degistirilemez


def mlp(in_dim, hidden, out_dim, activation):
    layers, d = [], in_dim
    for h in hidden:
        layers += [nn.Linear(d, h), ACTIVATIONS[activation]()]
        d = h
    layers.append(nn.Linear(d, out_dim))
    return nn.Sequential(*layers)


class EmpiricalNormalization(nn.Module):
    """y = (x - mean) / (std + eps); mean/var tum gorulen orneklerden (paralel Welford)."""

    def __init__(self, dim, eps=1e-2, until=None):
        super().__init__()
        self.eps = eps
        self.until = until
        self.register_buffer("mean", torch.zeros(dim))
        self.register_buffer("var", torch.ones(dim))
        self.register_buffer("std", torch.ones(dim))
        self.register_buffer("count", torch.zeros((), dtype=torch.float64))

    def forward(self, x):
        return (x - self.mean) / (self.std + self.eps)

    @torch.no_grad()
    def update(self, x):
        if self.until is not None and self.count.item() >= self.until:
            return
        n = x.shape[0]
        if n == 0:
            return
        total = self.count.item() + n
        rate = n / total
        var_x, mean_x = torch.var_mean(x, dim=0, unbiased=False)
        delta = mean_x - self.mean
        self.mean += rate * delta
        self.var += rate * (var_x - self.var + delta * (mean_x - self.mean))
        self.std.copy_(torch.sqrt(self.var))
        self.count.fill_(total)


class ActorCritic(nn.Module):
    def __init__(self, num_obs, num_critic_obs, num_actions, actor_hidden=None, critic_hidden=None,
                 activation=None, init_noise_std=None, obs_normalization=None):
        super().__init__()
        P = C.PPO
        actor_hidden = list(P["actor_hidden"] if actor_hidden is None else actor_hidden)
        critic_hidden = list(P["critic_hidden"] if critic_hidden is None else critic_hidden)
        activation = P["activation"] if activation is None else activation
        init_noise_std = P["init_noise_std"] if init_noise_std is None else init_noise_std
        obs_normalization = P["obs_normalization"] if obs_normalization is None else obs_normalization
        self.arch = dict(num_obs=num_obs, num_critic_obs=num_critic_obs, num_actions=num_actions,
                         actor_hidden=actor_hidden, critic_hidden=critic_hidden, activation=activation,
                         init_noise_std=float(init_noise_std), obs_normalization=bool(obs_normalization))
        self.actor = mlp(num_obs, actor_hidden, num_actions, activation)
        self.critic = mlp(num_critic_obs, critic_hidden, 1, activation)
        self.log_std = nn.Parameter(torch.full((num_actions,), math.log(init_noise_std)))
        self.obs_normalization = bool(obs_normalization)
        self.actor_obs_normalizer = EmpiricalNormalization(num_obs)
        self.critic_obs_normalizer = EmpiricalNormalization(num_critic_obs)

    @classmethod
    def from_arch(cls, arch):
        return cls(**arch)

    @property
    def std(self):
        return torch.exp(self.log_std)

    def norm_obs(self, obs):
        return self.actor_obs_normalizer(obs) if self.obs_normalization else obs

    def norm_critic_obs(self, critic_obs):
        return self.critic_obs_normalizer(critic_obs) if self.obs_normalization else critic_obs

    def update_normalizers(self, obs, critic_obs):
        if self.obs_normalization:
            self.actor_obs_normalizer.update(obs)
            self.critic_obs_normalizer.update(critic_obs)

    def act_inference(self, obs):
        """Ham gozlem -> deterministik (ortalama) aksiyon."""
        return self.actor(self.norm_obs(obs))

    def forward(self, obs):
        return self.act_inference(obs)

    def value(self, critic_obs):
        """Ham kritik gozlemi -> V(s), (N,)."""
        return self.critic(self.norm_critic_obs(critic_obs)).squeeze(-1)


def gaussian_log_prob(actions, mu, log_std):
    return (-0.5 * ((actions - mu) / torch.exp(log_std)) ** 2 - log_std
            - 0.5 * math.log(2.0 * math.pi)).sum(-1)


class RolloutStorage(object):
    def __init__(self, num_envs, num_steps, num_obs, num_critic_obs, num_actions, device="cpu"):
        T, N = num_steps, num_envs
        z = lambda *s: torch.zeros(*s, device=device)
        self.num_envs, self.num_steps = N, T
        self.obs = z(T, N, num_obs)                  # normalize edilmis
        self.critic_obs = z(T, N, num_critic_obs)    # normalize edilmis
        self.actions = z(T, N, num_actions)
        self.rewards = z(T, N)
        self.dones = z(T, N)
        self.time_outs = z(T, N)
        self.values = z(T, N)
        self.log_probs = z(T, N)
        self.mu = z(T, N, num_actions)
        self.sigma = z(T, N, num_actions)
        self.returns = z(T, N)
        self.advantages = z(T, N)
        self.step = 0

    def add(self, obs, critic_obs, actions, rewards, dones, log_probs, mu, sigma, time_outs=None, values=None):
        """values verilmezse PPO.compute_returns() hepsini tek seferde hesaplar."""
        if self.step >= self.num_steps:
            raise RuntimeError("RolloutStorage dolu; clear() cagrilmali")
        t = self.step
        self.obs[t].copy_(obs)
        self.critic_obs[t].copy_(critic_obs)
        self.actions[t].copy_(actions)
        self.rewards[t].copy_(rewards)
        self.dones[t].copy_(dones)
        if time_outs is None:
            self.time_outs[t].zero_()
        else:
            self.time_outs[t].copy_(time_outs)
        if values is not None:
            self.values[t].copy_(values)
        self.log_probs[t].copy_(log_probs)
        self.mu[t].copy_(mu)
        self.sigma[t].copy_(sigma)
        self.step += 1

    def clear(self):
        self.step = 0

    def compute_returns(self, last_values, gamma, lam, normalize_advantages=True):
        adv = torch.zeros_like(last_values)
        for t in reversed(range(self.num_steps)):
            next_values = last_values if t == self.num_steps - 1 else self.values[t + 1]
            not_done = 1.0 - self.dones[t]
            delta = self.rewards[t] + not_done * gamma * next_values - self.values[t]
            adv = delta + not_done * gamma * lam * adv
            self.returns[t] = adv + self.values[t]
        self.advantages = self.returns - self.values
        if normalize_advantages:
            self.advantages = (self.advantages - self.advantages.mean()) / (self.advantages.std() + 1e-8)

    def mini_batch_generator(self, num_mini_batches, num_epochs, generator=None):
        B = self.num_envs * self.num_steps
        mb = B // num_mini_batches
        flat = lambda x: x.reshape(B, *x.shape[2:])
        data = [flat(x) for x in (self.obs, self.critic_obs, self.actions, self.values, self.advantages,
                                  self.returns, self.log_probs, self.mu, self.sigma)]
        for _ in range(num_epochs):
            perm = torch.randperm(B, generator=generator, device=self.obs.device)
            for i in range(num_mini_batches):
                idx = perm[i * mb:(i + 1) * mb]
                yield [x[idx] for x in data]


class PPO(object):
    def __init__(self, actor_critic, num_envs, num_steps_per_env=None, cfg=None, device="cpu"):
        P = dict(C.PPO)
        P.update(cfg or {})
        if num_steps_per_env is not None:
            P["num_steps_per_env"] = num_steps_per_env
        self.cfg = P
        self.ac = actor_critic.to(device)
        self.device = device
        self.num_envs = num_envs
        self.num_steps = P["num_steps_per_env"]
        self.gamma, self.lam = P["gamma"], P["lam"]
        self.clip_param = P["clip_param"]
        self.lr = P["learning_rate"]
        self.schedule = P["schedule"]
        self.desired_kl = P["desired_kl"]
        self.optimizer = torch.optim.Adam(self.ac.parameters(), lr=self.lr)
        a = self.ac.arch
        self.storage = RolloutStorage(num_envs, self.num_steps, a["num_obs"], a["num_critic_obs"],
                                      a["num_actions"], device)
        self._t = None    # act() ile process_env_step() arasindaki gecici gecis

    def set_lr(self, lr):
        self.lr = lr
        for g in self.optimizer.param_groups:
            g["lr"] = lr

    @torch.no_grad()
    def act(self, obs, critic_obs, update_normalizer=True):
        """Ham gozlemler (N,*) -> ornek aksiyonlar (N,A). Gecisi process_env_step() tamamlar.
        Kritik burada calistirilmaz: V(s_t) toplama bitince compute_returns()'te toplu hesaplanir
        (ag toplama boyunca degismedigi icin sonuc aynidir, adim basina sure kisalir)."""
        if update_normalizer:
            self.ac.update_normalizers(obs, critic_obs)
        nobs = self.ac.norm_obs(obs)
        ncobs = self.ac.norm_critic_obs(critic_obs)
        mu = self.ac.actor(nobs)
        log_std = self.ac.log_std.expand_as(mu)
        actions = mu + torch.exp(log_std) * torch.randn_like(mu)
        self._t = dict(obs=nobs, critic_obs=ncobs, actions=actions,
                       log_probs=gaussian_log_prob(actions, mu, log_std), mu=mu, sigma=torch.exp(log_std))
        return actions

    @torch.no_grad()
    def process_env_step(self, rewards, dones, time_outs=None):
        t = self._t
        f = lambda x: None if x is None else torch.as_tensor(x).to(self.device, torch.float32)
        self.storage.add(t["obs"], t["critic_obs"], t["actions"], f(rewards), f(dones), t["log_probs"],
                         t["mu"], t["sigma"], time_outs=f(time_outs))
        self._t = None

    @torch.no_grad()
    def compute_returns(self, last_critic_obs):
        """V(s_t) toplu hesap, zaman asimi bootstrap'i (r += gamma V(s_t) time_out), sonra GAE."""
        st = self.storage
        T, N = st.num_steps, st.num_envs
        st.values.copy_(self.ac.critic(st.critic_obs.reshape(T * N, -1)).reshape(T, N))
        st.rewards += self.gamma * st.values * st.time_outs
        last_values = self.ac.value(last_critic_obs)
        st.compute_returns(last_values, self.gamma, self.lam)

    def update(self):
        P = self.cfg
        stats = dict(value_loss=0.0, surrogate_loss=0.0, entropy=0.0, kl=0.0, clip_frac=0.0, lr_mean=0.0)
        n = 0
        for (obs, cobs, actions, old_values, adv, returns, old_log_probs, old_mu, old_sigma) in \
                self.storage.mini_batch_generator(P["num_mini_batches"], P["num_learning_epochs"]):
            mu = self.ac.actor(obs)
            log_std = self.ac.log_std.expand_as(mu)
            sigma = torch.exp(log_std)
            log_probs = gaussian_log_prob(actions, mu, log_std)
            values = self.ac.critic(cobs).squeeze(-1)
            entropy = (0.5 + 0.5 * math.log(2.0 * math.pi) + log_std).sum(-1)

            with torch.no_grad():
                kl = torch.sum(torch.log(sigma / old_sigma + 1e-5)
                               + (old_sigma ** 2 + (old_mu - mu) ** 2) / (2.0 * sigma ** 2) - 0.5, dim=-1)
                kl_mean = kl.mean().item()
            if self.desired_kl is not None and self.schedule == "adaptive":
                if kl_mean > 2.0 * self.desired_kl:
                    self.set_lr(max(1e-5, self.lr / 1.5))
                elif 0.0 < kl_mean < 0.5 * self.desired_kl:
                    self.set_lr(min(1e-2, self.lr * 1.5))

            ratio = torch.exp(log_probs - old_log_probs)
            surr = -adv * ratio
            surr_clipped = -adv * torch.clamp(ratio, 1.0 - self.clip_param, 1.0 + self.clip_param)
            surrogate_loss = torch.max(surr, surr_clipped).mean()

            values_clipped = old_values + (values - old_values).clamp(-self.clip_param, self.clip_param)
            value_loss = torch.max((values - returns) ** 2, (values_clipped - returns) ** 2).mean()

            loss = surrogate_loss + P["value_loss_coef"] * value_loss - P["entropy_coef"] * entropy.mean()
            self.optimizer.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(self.ac.parameters(), P["max_grad_norm"])
            self.optimizer.step()

            stats["value_loss"] += value_loss.item()
            stats["surrogate_loss"] += surrogate_loss.item()
            stats["entropy"] += entropy.mean().item()
            stats["kl"] += kl_mean
            stats["clip_frac"] += ((ratio - 1.0).abs() > self.clip_param).float().mean().item()
            stats["lr_mean"] += self.lr
            n += 1
        self.storage.clear()
        # lr: guncelleme sonundaki deger (bir sonrakinin baslangici); lr_mean: adimlarda kullanilanlarin
        # ortalamasi. Uyarlama mini-batch basina oldugu icin ikisi cok farkli olabilir.
        out = {k: v / max(n, 1) for k, v in stats.items()}
        out["lr"] = self.lr
        out["action_std"] = self.ac.std.mean().item()
        return out

    def state_dict(self):
        return {"model": self.ac.state_dict(), "arch": self.ac.arch, "optimizer": self.optimizer.state_dict(),
                "lr": self.lr, "ppo_cfg": self.cfg}

    def load_state_dict(self, sd, load_optimizer=True):
        self.ac.load_state_dict(sd["model"])
        if load_optimizer and "optimizer" in sd:
            self.optimizer.load_state_dict(sd["optimizer"])
        if "lr" in sd:
            self.set_lr(sd["lr"])
