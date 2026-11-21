# References

Literature on observation and reward design for LiDAR navigation with deep RL, safety layers (control barrier
functions) in RL, and RL reporting practice. Found with Consensus on 2026-11-21 (titles, authors, years and
venues as that service gave them; links are Consensus pages, DOIs only where a PDF shows one). Papers marked
**READ** were read in full; the others were read at abstract level only, so a claim about them is about the
abstract.

## Read in full

- **Olayemi, K. B., Van, M., McLoone, S., McIlvanna, S., Sun, Y., Close, J., Nguyen, N. M.** The Impact of LiDAR
  Configuration on Goal-Based Navigation within a Deep Reinforcement Learning Framework. *Sensors* 23, 9732
  (2023). doi:10.3390/s23249732. [Consensus](https://consensus.app/papers/details/w:d923cd65-ae33-434d-8838-87497710e8b7/?utm_source=claude_desktop)
  - TD3 on a Husky. 10, 20 or 30 beams, field of view from sensor width over minimum safe distance; no-return
    reads max range (10 m), no validity flag. State = beams, goal distance, goal bearing, previous v and w.
    Reward = collision (to -100) + goal (+100) + -d/d0 + heading. gamma 0.99999, lr 5e-4, batch 200. Success,
    collision and timeout rates per environment over 100 runs.
- **Wang, X.** Ensuring Safety of Learning-Based Motion Planners Using Control Barrier Functions. *IEEE RA-L* 7(2),
  4773-4780 (2022). doi:10.1109/LRA.2022.3152313. [Consensus](https://consensus.app/papers/details/w:8b2f98d6-d136-44b0-84be-13b22715e12b/?utm_source=claude_desktop)
  - Post-hoc CBF shield: minimise ||u - u_RL||^2 subject to A u <= b, with a relaxation when the QP is
    infeasible (gamma = 3 inside the safe set, 1/dt outside). The agent does not observe the correction; it
    sees it in the reward (-50 |du|/range per axis). An agent trained with the layer degrades when the layer is
    removed unless that correction penalty was in the reward.
- **Yang, L., Werner, B., de Sa, M., Ames, A. D.** CBF-RL: Safety Filtering Reinforcement Learning in Training
  with Control Barrier Functions. arXiv:2510.14959v6 (ICRA 2026). [Consensus](https://consensus.app/papers/details/w:34afa5a8-83c4-47ad-b628-a1c168296b8e/?utm_source=claude_desktop)
  - Closed-form single-constraint CBF filter in training plus a reward term on the correction. Ablation over
    {nominal, reward only, filter only, dual} x {with, without the filter at deployment}: filter-only 98.8% with
    the filter, 38.7% without. Discrete-time bound h(q_k) >= (1 - dt alpha)^k h(q_0) - mu/(dt alpha).
- **Zhai, Y., Miao, Y.** 3D LiDAR Driven Reinforcement Learning With Safety Intervention for Mapless Navigation.
  *IEEE RA-L* 11(11), 12504-12511 (2026). doi:10.1109/LRA.2026.3728311. [Consensus](https://consensus.app/papers/details/w:a3ca4079-4f38-4a65-93bc-5cb52eeadb66/?utm_source=claude_desktop)
  - 16x360 depth image, forward sector sampled 4x denser, near-range reweighting (adaptive reciprocal factor).
    SAC, gamma 0.9, buffer 3000. A Gaussian-CBF controller applies the minimal-deviation action; its correction
    rate (SCR) is reported as a metric, not observed. 200 trials per scenario, mean and standard deviation.

## Abstract level

LiDAR observation and reward design: Botteghi et al. 2020 [On Reward Shaping for Mobile Robot Navigation](https://consensus.app/papers/details/w:4c6d7c66-6b78-420f-a463-f919de243294/?utm_source=claude_desktop);
Nakhleh et al. 2023 [SACPlanner](https://consensus.app/papers/details/w:8c4aea6e-4f9a-4f12-8c60-c50f37edd498/?utm_source=claude_desktop);
Yilmaz et al. 2026 [Comparative Study of DRL Algorithms for Map-Free Robot Navigation](https://consensus.app/papers/details/w:95066b1a-bd3f-454f-8661-e99579b192a7/?utm_source=claude_desktop);
Mohamed et al. 2026 [Wheeled-Robot Navigation in Harsh Environments Using DRL, Systematic Review](https://consensus.app/papers/details/w:d3aae6ef-56cc-4117-b192-c89152b57d4d/?utm_source=claude_desktop);
Miranda et al. 2023 [Generalization in DRL for Robotic Navigation by Reward Shaping](https://consensus.app/papers/details/w:e4fb57b7-925c-4aac-868e-2d088b837cdf/?utm_source=claude_desktop);
Jeng et al. 2023 [Survival Penalty Function](https://consensus.app/papers/details/w:486598b9-dcc6-4890-b35c-f6334d12d464/?utm_source=claude_desktop);
Yin et al. 2023 [A Mapless Local Path Planning Approach Using DRL](https://consensus.app/papers/details/w:133bdd6b-0508-43bb-a4cf-befa1d8a9a89/?utm_source=claude_desktop).

Safety layers: Cheng et al. 2019 [End-to-End Safe RL through Barrier Functions](https://consensus.app/papers/details/w:4d1f6f19-750b-4643-a257-4e9ff0426d19/?utm_source=claude_desktop);
Xiao et al. 2023 [BarrierNet](https://consensus.app/papers/details/w:a9a98655-253e-4f91-8b49-519b08ec1369/?utm_source=claude_desktop);
Xie et al. 2025 [Certificated Actor-Critic](https://consensus.app/papers/details/w:a2366cb3-9984-4e7b-8189-06f0917b3b49/?utm_source=claude_desktop);
Guerrier et al. 2025 [Guided by Guardrails](https://consensus.app/papers/details/w:9c085229-688b-4bca-a545-2d43381cec93/?utm_source=claude_desktop);
Park et al. 2026 [DODGER](https://consensus.app/papers/details/w:ff768661-4f42-4929-bc27-70c2102e294c/?utm_source=claude_desktop);
Lou et al. 2026 [USV Collision Avoidance via DRL, Safety Shield](https://consensus.app/papers/details/w:83cb5615-ea40-446b-8519-876c937210d8/?utm_source=claude_desktop);
Brunke et al. 2022 [Safe Learning in Robotics](https://consensus.app/papers/details/w:94c240ce-3eaa-44df-ab08-cd9019a351df/?utm_source=claude_desktop).

Reporting and reproducibility: Agarwal et al. 2021 [Deep RL at the Edge of the Statistical Precipice](https://consensus.app/papers/details/w:42faa08b-7735-4328-b73c-babc7b288c18/?utm_source=claude_desktop);
Colas et al. 2018 [How Many Random Seeds?](https://consensus.app/papers/details/w:a07ca724-a12d-419f-8709-da7c76feae5f/?utm_source=claude_desktop);
Eimer et al. 2023 [Hyperparameters in RL and How To Tune Them](https://consensus.app/papers/details/w:fb0fd319-4587-452b-971a-dc85988d38de/?utm_source=claude_desktop);
Islam et al. 2017 [Reproducibility of Benchmarked Deep RL Tasks for Continuous Control](https://consensus.app/papers/details/w:09bb9397-e861-4b19-b53b-367bb603b555/?utm_source=claude_desktop);
Patterson et al. 2023 [Empirical Design in Reinforcement Learning](https://consensus.app/papers/details/w:f02b30ff-33e5-4b5d-9de9-01cb532b721f/?utm_source=claude_desktop);
Lynnerup et al. 2019 [A Survey on Reproducibility by Evaluating DRL Algorithms on Real-World Robots](https://consensus.app/papers/details/w:b50325c6-f145-4ad7-aa5f-488b35be678e/?utm_source=claude_desktop).

Training environments (random layouts, curricula, domain randomisation, evaluation outside the training
environments; found 2026-11-21, abstract level): Wang et al. 2025 [Enhancing DRL-based Robot Navigation Generalization through Scenario Augmentation](https://consensus.app/papers/details/w:db1067de-3f62-4343-a847-6f31a7408331/?utm_source=claude_desktop) (limited training scenarios named as the primary cause of poor behaviour in unseen environments);
Morad et al. 2021 [Embodied Visual Navigation With Automatic Curriculum Learning in Real Environments](https://consensus.app/papers/details/w:3837e6fa-a9ee-43f2-bf5c-b88c423f3ed1/?utm_source=claude_desktop);
Xu et al. 2023 [Benchmarking Reinforcement Learning Techniques for Autonomous Navigation](https://consensus.app/papers/details/w:0f29abaf-2b55-47b9-b280-413e59239423/?utm_source=claude_desktop);
Sigal et al. 2023 [Improving Generalization in RL Training Regimes for Social Robot Navigation](https://consensus.app/papers/details/w:f5c847c7-3658-410b-ae0a-0173a1cd965e/?utm_source=claude_desktop) (many works never evaluate outside their training environments);
He et al. 2023 [Generalization Ability of DRL-based Navigation Based on Curriculum Learning](https://consensus.app/papers/details/w:26302b06-142c-4da0-ba78-57def847973a/?utm_source=claude_desktop);
Wang et al. 2023 [Curriculum RL From Avoiding Collisions to Navigating Among Movable Obstacles in Diverse Environments](https://consensus.app/papers/details/w:18dca7ae-8ea4-4bc9-bc27-6b7e568ce403/?utm_source=claude_desktop);
Wang et al. 2025 [Environment as Policy: Learning to Race in Unseen Tracks](https://consensus.app/papers/details/w:0e92b69d-af8e-40dc-b596-d764374022a1/?utm_source=claude_desktop) (training directly on diverse layouts can overburden the agent);
Chiang et al. 2019 [Learning Navigation Behaviors End-to-End With AutoRL](https://consensus.app/papers/details/w:8971ff65-16a9-4d25-9259-8ec9250c00e0/?utm_source=claude_desktop);
Ajani et al. 2023 [Evaluating Domain Randomization in DRL Locomotion Tasks](https://consensus.app/papers/details/w:1aa9f6e5-7e52-4d0d-a40a-ee103041813f/?utm_source=claude_desktop);
Sen et al. 2024 [Domain Randomization for Learning to Navigate in Human Environments](https://consensus.app/papers/details/w:a057ae45-9d5c-4b5c-a67e-9b552f828fcd/?utm_source=claude_desktop).

Not verified: DOIs and page numbers for the abstract-level papers (the search service returned none).
