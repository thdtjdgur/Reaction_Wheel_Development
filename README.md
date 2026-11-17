# 리액션휠 기반 위성 자세제어

<p align="center">
  <img src="https://img.shields.io/badge/MODEL-MATLAB%20%2F%20SIMULINK-25344F?style=for-the-badge&labelColor=555555" alt="MATLAB / Simulink" />
  <img src="https://img.shields.io/badge/CONTROL-MODEL--BASED%20PD-0C8D7B?style=for-the-badge&labelColor=555555" alt="Model-based PD control" />
  <img src="https://img.shields.io/badge/STATUS-SIMULATION%20VALIDATION-1F5FDB?style=for-the-badge&labelColor=555555" alt="Simulation validation" />
</p>

**모델 기반 자세제어기 설계 및 MATLAB/Simulink 응답 비교**

3축 리액션휠을 사용하는 위성의 회전 동역학과 쿼터니언 운동학을 모델링하고, 목표 감쇠비와 고유진동수로 PD 이득을 설계하는 프로젝트이다. 현재는 **X축 5° 목표 자세 기동**에서 이상적 토크 전달과 휠 속도·마찰 모델을 적용한 응답을 비교한다.

| 구분 | 진행 내용 |
|:--|:--|
| 모델 구현 | 위성 회전 동역학, 쿼터니언 운동학, PD 자세제어기, 휠 속도·마찰 모델 |
| 기존 시뮬레이션 기록 | X축 5° 목표 자세, 이상적 구동기와 마찰 포함 구동기의 응답 비교 |
| 이론 정리 | Spin-to-Spin 자세명령 생성과 유한 저크 프로파일 |
| 후속 개발 | 휠 각운동량 피드백, 모터 전류·속도 제어 루프, STM32 구현 및 실물 시험 |

이 문서는 [논문 정리 노트](자료/논문정리.pdf)와 [현재 Simulink 모델](Simulink/RW_3Axis_Control.slx)을 기준으로 작성하였다. 아래 성능 수치는 기존 시뮬레이션 실행 기록이며, 실물 시험 결과는 아직 없다.

## 1. 시스템 구성

```text
목표 자세 q_cmd, 목표 각속도 ω_cmd, 목표 각가속도 α_cmd
                       │
                       ▼
               자세·각속도 오차 계산 ◀──────────── q, ω_sc
                       │                            ▲
                       ▼                            │
                  PD 자세제어기                     │
                       │ τ_cmd                      │
                       ▼                            │
                Wheel_Control_Model ── τ_actual ──> 위성 모델
                       ▲       │                    │
                       │       └─ 휠 각가속도 → 적분 │
                       └──────── 휠 속도, ω_sc ──────┘
```

이상적 구동기 시험에서는 $\boldsymbol\tau_{actual}=\boldsymbol\tau_{cmd}$로 설정한다. 마찰 포함 시험에서는 휠 속도를 적분해 상태로 유지하고, 휠과 본체의 상대 각속도로 마찰 토크를 계산한다. **기준선에도 유효 관성 $J_{sc}-J_a$가 반영되어 있으며**, 두 번째 시험에서 휠 속도 상태와 마찰에 의한 토크 차이를 추가한다.

## 2. 위성 동역학과 자세 표현

### 회전 동역학과 토크 부호

휠 축이 위성 본체의 X·Y·Z축에 각각 정렬된 경우를 다룬다. 본체 좌표계에서 전체 각운동량과 회전 동역학은 다음과 같다.

```math
\boldsymbol H=J_{sc}\boldsymbol\omega+J_a\boldsymbol\nu,
\qquad
J_{sc}\dot{\boldsymbol\omega}
=\boldsymbol H\times\boldsymbol\omega-J_a\boldsymbol\mu
+\boldsymbol\tau_{dist}
```

| 기호 | 의미 |
|:--|:--|
| $J_{sc}$ | 휠을 포함한 위성 전체의 관성행렬 |
| $J_a$ | 본체 축에 정렬된 세 휠의 축방향 관성행렬 |
| $\boldsymbol\omega$ | 위성 본체의 각속도 |
| $\boldsymbol\nu$, $\boldsymbol\mu=\dot{\boldsymbol\nu}$ | 본체에 대한 휠의 상대 각속도 및 상대 각가속도 |
| $\boldsymbol T_m$, $\boldsymbol T_w$ | 휠의 모터 토크 및 마찰을 반영한 휠 순토크 |
| $\boldsymbol\tau_{dist}$ | 외부에서 작용하는 외란 토크 |

휠의 절대 각가속도에 대응하는 순토크를 $\boldsymbol T_w=J_a(\dot{\boldsymbol\omega}+\boldsymbol\mu)$로 놓고 $J_{eff}=J_{sc}-J_a$를 정의하면,

```math
\boxed{
J_{eff}\dot{\boldsymbol\omega}
=\boldsymbol H\times\boldsymbol\omega
-\boldsymbol T_w+\boldsymbol\tau_{dist}
}
```

가 된다. 위성 본체에 적용하는 반작용 토크는 $\boldsymbol\tau_{actual}=-\boldsymbol T_w$이다. 이상적 토크 전달에서는 $\boldsymbol T_w=\boldsymbol T_m=-\boldsymbol\tau_{cmd}$이며, 마찰 포함 모델의 토크 관계는 5절에서 설명한다.

현재 `Spacecraft_Model`은 다음 식을 구현한다.

```math
\dot{\boldsymbol\omega}
=J_{eff}^{-1}\left[
(J_{sc}\boldsymbol\omega+\boldsymbol h_{RW})\times\boldsymbol\omega
+\boldsymbol\tau_{actual}\right],
\qquad \boldsymbol h_{RW}=J_a\boldsymbol\nu
```

**현재 시험에서는 `hRW` 입력을 항상 0으로 고정한다.** 따라서 $J_{sc}\boldsymbol\omega$에 의한 본체의 축간 결합은 계산하지만, 휠 회전에 따른 $J_a\boldsymbol\nu$의 자이로 결합은 반영하지 않는다. 외란 토크 입력도 현재 시험에 추가하지 않았다.

### 쿼터니언 자세와 오차

자세 상태와 목표 자세는 **벡터 성분 우선, 스칼라 성분 마지막** 순서인 $q=[q_x,q_y,q_z,q_4]^T$를 사용한다.

```math
q(\theta,\hat{\boldsymbol e})=
\begin{bmatrix}
\hat{\boldsymbol e}\sin(\theta/2)\\
\cos(\theta/2)
\end{bmatrix},
\qquad
\dot q=\frac12\Omega(\boldsymbol\omega)q
```

단위 쿼터니언의 역원을 이용한 자세 오차는

```math
q_e=q\otimes q_{cmd}^{-1}
```

로 정의하고, 제어에는 벡터 성분 $q_{e,v}\in\mathbb R^3$를 사용한다. 현재 Z축 자세가 0°, 목표가 +1°이면

```math
q_{e,v}=\begin{bmatrix}0\\0\\-\sin(0.5^\circ)\end{bmatrix}
\approx\begin{bmatrix}0\\0\\-0.008727\end{bmatrix}
```

이므로, 비례 제어 항은 목표 방향으로 회전시키는 토크를 생성한다.

모델의 **Aerospace Blockset `Quaternion Multiplication` 블록은 스칼라 우선 형식**을 사용한다. 따라서 블록 입력 직전에 `[4 1 2 3]`으로 성분을 재배열하고, 곱셈 결과에서는 `[2 3 4]`를 선택해 오차 벡터를 추출한다. 자세 상태의 성분 순서와 라이브러리 블록의 입출력 순서를 구분해야 한다. [블록 공식 문서](https://www.mathworks.com/help/aeroblks/quaternionmultiplication.html)

## 3. 모델 기반 PD 제어기 설계

### 토크 명령

```math
\boxed{
\boldsymbol\tau_{cmd}
=-K_Pq_{e,v}
-K_D(\boldsymbol\omega-\boldsymbol\omega_{cmd})
+J_{eff}\boldsymbol\alpha_{cmd}
}
```

$\boldsymbol\tau_{cmd}$는 위성 본체에 적용하려는 토크 명령이며, 모터 토크 명령은 반대 부호이다. 마지막 항은 목표 각가속도에 대한 관성 피드포워드이다. 아래 이득 유도는 **목표 부근의 소각도, 같은 좌표계로 표현한 각속도 오차, 외란 및 자이로 결합을 무시하는 근사**에 기반한다.

### 특성방정식으로부터 이득 도출

작은 자세 오차 벡터를 $\boldsymbol e$로 두면,

```math
q_{e,v}\approx\frac{\boldsymbol e}{2},\qquad
\dot{\boldsymbol e}\approx\boldsymbol\omega-\boldsymbol\omega_{cmd},\qquad
\ddot{\boldsymbol e}\approx\dot{\boldsymbol\omega}-\boldsymbol\alpha_{cmd}
```

이고, 선형화한 오차 동역학은

```math
J_{eff}\ddot{\boldsymbol e}
+K_D\dot{\boldsymbol e}
+\frac12K_P\boldsymbol e=0
```

가 된다. $\boldsymbol e(t)=\boldsymbol v e^{st}$, $\boldsymbol v\ne0$를 대입하면

```math
\det\left(J_{eff}s^2+K_Ds+\frac12K_P\right)=0
```

를 얻는다. $J_{eff}$가 가역일 때, 원하는 감쇠비 $\zeta$와 고유진동수 $\omega_n$에 대해

```math
\boxed{K_P=2\omega_n^2J_{eff}},
\qquad
\boxed{K_D=2\zeta\omega_nJ_{eff}}
```

로 설정하면 선형 오차 동역학이

```math
\ddot{\boldsymbol e}+2\zeta\omega_n\dot{\boldsymbol e}
+\omega_n^2\boldsymbol e=0
```

형태가 된다. $K_P$와 $K_D$는 **3×3 행렬**이며, 관성행렬의 비대각 성분도 이득에 반영된다. 이 선형 설계 결과를 일반적인 큰각 기동이나 회전 목표 추종에 적용하려면 좌표계 변환과 비선형 결합 항을 추가로 검토해야 한다.

## 4. 자세명령 생성: Spin-to-Spin (이론 정리)

고기동 자세명령은 $q_{cmd}(t)$, $\boldsymbol\omega_{cmd}(t)$, $\boldsymbol\alpha_{cmd}(t)$가 서로 일관된 궤적을 이루도록 생성한다. 고정 회전축에서는 $\boldsymbol\omega_{cmd}=\hat{\boldsymbol e}\dot\theta$와 $\boldsymbol\alpha_{cmd}=\hat{\boldsymbol e}\ddot\theta$로 연결된다.

| 구간 | 동작 |
|:--|:--|
| Phase 1 — Spin-to-Rest | 초기 회전 감속 및 정지 |
| Phase 2 — Rest-to-Rest | 정지 상태에서 자세 변경 후 정지 |
| Phase 3 — Rest-to-Spin | 정지 상태에서 목표 각속도까지 가속 |
| Phase 4 — Stabilization | 최종 목표 각속도를 유지하며 회전 |

참고 논문의 Phase 3은 **저크 $j=\dot a$를 제한하는 프로파일**을 사용한다.

- **Jerk Bang-Bang:** 저크가 양·음으로 전환되어 각가속도가 삼각형을 이룬다.
- **Jerk Bang-Off-Bang:** 각가속도 상한에 도달하면 저크를 0으로 유지하는 구간을 추가해 각가속도가 사다리꼴을 이룬다. 이 구간의 각속도는 계속 증가한다.

각가속도 프로파일의 면적은 각속도 변화량 $\Delta\omega=\int a(t)\,dt$이다. 따라서 Phase 3에서는 가속도를 0으로 낮추며 목표 각속도에 도달하고, Phase 4에서 해당 각속도를 유지한다.

논문 정리 노트의 Phase 4 예시는 종료 시각 $t_6=10\,\mathrm{s}$, 안정화 시간 $3\,\mathrm{s}$, 최종 각속도 크기 $2^\circ/\mathrm{s}$, 고정 회전축 $\hat{\boldsymbol e}=[0.6,0.8,0]^T$를 사용한다. $7\le t\le10\,\mathrm{s}$에서

```math
\boldsymbol\omega_{cmd}
=\begin{bmatrix}1.2\\1.6\\0\end{bmatrix}\;[^\circ/\mathrm{s}],
\qquad
\boldsymbol\alpha_{cmd}=\boldsymbol0,
\qquad
\theta_{rel}(t)=2(t-10)\;[^\circ]
```

이다. $\theta_{rel}$은 최종 자세를 기준으로 한 상대 회전각으로, 7초에 −6°, 10초에 0°가 된다. **목표 각속도가 0이 아니므로 안정화 구간에도 목표 자세는 시간에 따라 변한다.** Simulink 입력에는 각도·각속도·각가속도를 각각 rad, rad/s, rad/s²로 변환한다.

이 절은 자세명령 생성 원리의 정리이며, 아래 계단입력 시험에는 아직 적용하지 않았다.

## 5. 시뮬레이션 조건과 결과

### 공통 설정: X축 5° 목표 자세

현재 모델의 Model Workspace에 저장된 관성행렬은 다음과 같다. 단위는 kg·m²이다.

```math
J_{sc}=\begin{bmatrix}
0.200&0.010&-0.005\\
0.010&0.250&0.008\\
-0.005&0.008&0.300
\end{bmatrix},
\qquad
J_a=\mathrm{diag}(0.001,0.001,0.001)
```

| 항목 | 설정 |
|:--|:--|
| 목표 자세 $q_{cmd}$ | `[sin(pi/72); 0; 0; cos(pi/72)]` — X축 +5° |
| 초기 자세 $q(0)$ | `[0; 0; 0; 1]` |
| 초기 본체 각속도 / 초기 휠 절대 각속도 | 모두 `[0; 0; 0]` rad/s |
| 목표 각속도 / 목표 각가속도 | 각각 `[0; 0; 0]` rad/s, `[0; 0; 0]` rad/s² |
| 감쇠비 / 고유진동수 | **$\zeta=1$, $\omega_n=1\,\mathrm{rad/s}$** |
| 유효 관성 $J_{eff}$ | $J_{sc}-J_a$; X축 대각 성분은 **0.199 kg·m²** |
| PD 이득 | $K_P=K_D=2J_{eff}$ — 수치상 동일하며 단위와 역할은 다름 |
| X축 대각 이득 | $K_{P,xx}=0.398$, $K_{D,xx}=0.398$ |
| 시뮬레이션 시간 | **10 s** |
| 적분 설정 | Variable-step `ode45`, RelTol `1e-8`, Max step `0.001 s` |
| 성능 지표 계측 주기 | **0.001 s** |
| 휠 각운동량 입력 | `hRW = [0; 0; 0]`으로 고정 |

X축 목표 기동에서도 관성행렬의 비대각 성분에 따라 Y·Z축 제어 토크가 생성될 수 있다. 아래 그래프는 X축 목표 기동에 대한 대표 응답이다.

### 시험 A. 이상적 토크 전달

마찰, 구동 지연, 토크 포화를 제외하고 $\boldsymbol\tau_{actual}=\boldsymbol\tau_{cmd}$로 놓는다.

![이상적 PD 제어의 해석적 참조 응답](assets/ideal_pd_response.svg)

*그림: $\zeta=1$, $\omega_n=1\,\mathrm{rad/s}$로 선형화한 단일축 2차계의 해석적 참조 곡선.*

![이상적 토크 전달 시험의 성능 지표](assets/ideal_pd_metrics.svg)

초기 자세 0°, 목표 자세 5°인 임계감쇠 2차계의 이론 응답은

```math
\theta(t)=5\left[1-(1+t)e^{-t}\right]\;[^\circ]
```

이다. 이론 상승시간은 약 **3.3579 s**, 정착시간은 약 **5.8339 s**, 10초 시점 오차는 약 **0.002497°**이며, 아래의 기존 시뮬레이션 기록과 잘 일치한다.

### 시험 B. 휠 속도·마찰 모델 적용

논문 정리 노트의 점성·쿨롱마찰식은

```math
\boldsymbol T_f
=\boldsymbol b\odot\boldsymbol\omega_{w,rel}
+\boldsymbol c\odot\mathrm{sgn}(\boldsymbol\omega_{w,rel})
```

이다. **현재 저장된 `Wheel_Control_Model`은 쿨롱마찰 항을 `tanh`로 평활화하여 구현한다.** 각 축별 근사식은 다음과 같다.

```math
\begin{aligned}
\boldsymbol\omega_{w,rel}
&=\boldsymbol\omega_{wheel,abs}-\boldsymbol\omega_{sc}\\
\boldsymbol T_f
&=\boldsymbol b\odot\boldsymbol\omega_{w,rel}
+\boldsymbol c\odot\tanh\!\left(\frac{\boldsymbol\omega_{w,rel}}{\omega_{\epsilon}}\right)\\
\boldsymbol T_m&=-\boldsymbol\tau_{cmd}\\
\boldsymbol T_w&=\boldsymbol T_m-\boldsymbol T_f\\
\boldsymbol\tau_{actual}&=-\boldsymbol T_w
=\boldsymbol\tau_{cmd}+\boldsymbol T_f\\
\dot{\boldsymbol\omega}_{wheel,abs}&=J_a^{-1}\boldsymbol T_w
\end{aligned}
```

여기서 $\odot$와 `tanh`는 성분별 연산이다. $\boldsymbol T_f$는 상대속도에 따라 부호가 정해지는 마찰 항이며, **휠에 작용하는 마찰 토크는 $-\boldsymbol T_f$**이다. 위성 토크 식의 덧셈 부호도 이 반작용 관계에서 나온다.

| 항목 | 현재 저장 모델의 설정 |
|:--|:--|
| 점성마찰계수 $\boldsymbol b$ | `[1e-5; 1e-5; 1e-5]` N·m·s/rad |
| 쿨롱마찰계수 $\boldsymbol c$ | `[1e-4; 1e-4; 1e-4]` N·m |
| 평활화 속도 $\omega_{\epsilon}$ | **0.1 rad/s** |
| 휠 각가속도 계산 | `Awheel = Ja \ Tw` |
| 휠 속도 상태 | `Awheel → Integrator → Wwheel` |

위 마찰계수와 평활화 속도는 시험용 설정값이다. 현재 모델은 휠 축별 회전과 점성·평활화 쿨롱마찰을 근사하며, 정지마찰에 의한 고착, 모터 전류·속도 폐루프, 구동 지연 및 토크·속도 제한은 포함하지 않는다. 휠 속도를 계산해도 `hRW`에는 연결하지 않은 상태이다.

![휠 속도·마찰 모델을 적용한 기존 시뮬레이션 응답](assets/wheel_friction_response.svg)

*그림: X축 5° 목표 자세 기동에서 휠 속도·마찰 모델을 적용한 토크, 위성 각속도 및 회전각 응답.*

![휠 속도·마찰 모델 시험의 성능 지표](assets/wheel_friction_metrics.svg)

### 기존 시험 결과 비교

현재 계측 블록은 쿼터니언의 스칼라 성분으로부터 $\theta=2\arccos(q_4)$를 계산하고, 이를 도 단위로 변환해 목표값 5°와 비교한다. 따라서 표의 오차는 **회전각 크기의 차이 $5^\circ-\theta(t)$**이다. 일반적인 3차원 자세오차 평가는 오차 쿼터니언 $q_e$로 별도 수행해야 한다.

| 성능 지표 | 이상적 토크 전달 | 휠 속도·마찰 적용 | 변화 |
|:--|--:|--:|--:|
| 상승시간 $T_r$ (10–90%) | 3.359 s | **3.446 s** | +0.087 s |
| 정착시간 $T_s$ (목표값의 ±2%) | 5.835 s | **6.276 s** | +0.441 s |
| 최대 오버슈트 | 0% | **0%** | 0%p |
| 10초 시점 회전각 크기 오차 | 0.002498° | **0.01896°** | +0.016462° |

- 정착시간은 10초 관측 구간에서 목표 회전각의 ±2% 범위에 들어온 뒤 유지되는 시점을 뜻한다.
- 10초 시점 오차는 유한 시각의 잔여 오차이다. 정상상태 오차를 판정하려면 더 긴 시간의 응답을 확인해야 한다.
- 감쇠비와 고유진동수의 **설계값**은 두 시험 모두 $\zeta=1$, $\omega_n=1\,\mathrm{rad/s}$이다. 마찰이 포함된 비선형 응답 전체를 동일한 2차계로 가정하지 않는다.
- 계측 블록의 `zeta`, `wn` 출력이 `-1`인 것은 **추정식을 적용하지 않았다는 표시**이다. 해당 코드는 양의 오버슈트가 있을 때만 역산하므로, 오버슈트 0%인 두 시험에서 `-1`을 출력한다.

마찰계수 $\boldsymbol b=\boldsymbol0$, $\boldsymbol c=\boldsymbol0$이면 휠 모델의 출력이 $\boldsymbol\tau_{actual}=\boldsymbol\tau_{cmd}$로 환원된다. 기존 시험에서는 이 조건에서 기준선과 같은 응답을 확인했다. 마찰 적용 시 상승·정착시간과 10초 시점 회전각 오차가 증가했다.

**확인 범위:** 이 결과는 현재 단순화한 모델에서 제어기와 마찰 항의 영향을 비교한 것이다. 휠 각운동량을 포함한 전체 3축 모델, 실측 마찰계수 및 하드웨어 자세제어 성능은 후속 검증 대상이다.

## 6. 모델 실행

**필요 도구:** MATLAB, Simulink, Aerospace Blockset. 저장 모델은 MATLAB R2026b에서 작성되었으며, 다른 버전에서의 호환성은 별도 확인이 필요하다.

저장소 루트에서 MATLAB 명령 창에 다음을 실행한다.

```matlab
mdl = 'RW_3Axis_Control';
open_system(fullfile('Simulink', [mdl '.slx']));
sim(mdl);
```

- `Jsc`, `Ja`, `zeta`, `wn`은 모델의 **Model Workspace**에 저장되어 있다.
- 기본 실행은 마찰 포함 조건이다. 이상적 토크 전달을 비교하려면 `Wheel_Control_Model`의 마찰계수 `b`, `c`를 모두 0으로 설정해 다시 실행한다.
- Dashboard와 Scope 또는 Simulation Data Inspector에서 응답을 확인한다. 성능 지표 출력 순서는 `[Tr; Ts; OS; error_deg; zeta; wn]`이다.

## 7. 후속 개발

- **자세명령 생성 구현:** Spin-to-Spin의 자세·각속도·각가속도 명령을 일관되게 생성하고 추종 시험 수행
- **휠 각운동량 반영:** 상대 휠 속도에서 `hRW = Ja * nu`를 계산해 위성 동역학에 연결하고 결합 응답 검증
- **구동기 모델 고도화:** 실측 마찰계수 식별, 정지마찰, 전류·속도 제어 루프 및 토크·속도 제한 반영
- **검증 범위 확대:** 다축·큰각 기동, 회전 목표 추종, 외란 및 쿼터니언 기반 자세오차 평가
- **STM32 및 실물 시험:** 펌웨어 구현 후 시뮬레이션 예측과 계측 응답 비교

## 8. 참고 자료

- [논문 정리 노트](자료/논문정리.pdf): 동역학·토크 부호, PD 이득 유도, Spin-to-Spin 예제, 쿼터니언 운동학 및 휠 마찰 모델 정리
- [리액션휠 기반 고기동 위성 자세제어 기법 연구](자료/리액션휠%20기반%20고기동%20위성%20자세제어%20기법%20연구.pdf): 프로젝트 참고 자료
- 목성훈·방효충·김희섭, [고기동 인공위성의 해석적 자세명령생성 기법 연구](자료/%5B19%5D%20고기동%20인공위성의%20해석적%20자세명령생성%20기법%20연구.pdf), 한국항공우주학회지 46(8), 639–651, 2018. [DOI](https://doi.org/10.5139/JKSAS.2018.46.8.639)
- Valdemir Carrara and Hélio K. Kuga, [Torque and Speed Control Loops of a Reaction Wheel](자료/%5B18%5D%20TORQUE%20AND%20SPEED%20CONTROL%20LOOPS%20OF%20A%20REACTION%20WHEEL.pdf), 11th International Conference on Vibration Problems, 2013. [한국어 번역본](자료/%5B18번역본%5D%20TORQUE%20AND%20SPEED%20CONTROL%20LOOPS%20OF%20A%20REACTION%20WHEEL.pdf)
