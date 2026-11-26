# 論文筆記：motion imitation

## [DeepMimic](https://arxiv.org/abs/1804.02717)

## [AMP](https://arxiv.org/abs/2104.02180)

- 貢獻：用 discriminator 學出 style reward，取代 DeepMimic 手調的 tracking reward。
- 缺點：無法適應多個動作或互相矛盾的動作。

## [BeyondMimic](https://arxiv.org/abs/2508.08241)

- 用了 diffusion policy，以及 imitation learning 的部分。
- 可以生成由動作 A 到動作 B 的平滑過渡動作。

## [SuperMimic](https://arxiv.org/abs/2610.28480)

- 解決的問題：PPO training 效率太差。
- 三個階段：
  1. 從 simulator 收集資料。
  2. 用這些資料訓練一個簡單的 world model。
  3. 用 world model 去優化 policy。
- 所以這篇是在試著建立可微分的 simulator（dynamics）。
