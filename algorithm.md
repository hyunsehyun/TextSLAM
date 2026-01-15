NavOCR-SLAM 논문 작성용 알고리즘 설명
1. 시스템 개요
NavOCR-SLAM은 OCR 기반 텍스트 간판을 랜드마크로 활용하는 시각적 SLAM 시스템입니다.

1.1 시스템 아키텍처

┌─────────────────────────────────────────────────────────────┐
│                      Input Layer                            │
├─────────────────────────────────────────────────────────────┤
│  • OCR Detection (NavOCR)  → 텍스트 + 바운딩박스 + 신뢰도   │
│  • Depth Image (RealSense) → 깊이 정보                      │
│  • Camera Info             → 카메라 내부 파라미터           │
│  • Odometry (rtabmap)      → 카메라 위치/자세               │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                   Processing Layer                          │
├─────────────────────────────────────────────────────────────┤
│  1. 3D 위치 추정 (깊이 + 카메라 모델)                        │
│  2. 데이터 연관 (마할라노비스 거리 + 텍스트 유사도)          │
│  3. 랜드마크 업데이트 (Welford 온라인 알고리즘)              │
│  4. 랜드마크 병합 (중복 제거)                                │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                      Output Layer                           │
├─────────────────────────────────────────────────────────────┤
│  • 랜드마크 맵 (3D 위치 + 텍스트)                           │
│  • RViz 시각화 마커                                         │
│  • Reprojection Error 평가                                  │
└─────────────────────────────────────────────────────────────┘
1.2 데이터 구조
Landmark 구조:

μ ∈ ℝ³: 3D 평균 위치 (월드 좌표계)
Σ ∈ ℝ³ˣ³: 공분산 행렬
N: 관측 횟수
T: 대표 텍스트 (가중 투표로 결정)
c: 텍스트 신뢰도 (0~1)
2. 3D 위치 추정 (Depth Projection)
2.1 핀홀 카메라 모델
바운딩박스 중심 (u, v)와 깊이 z를 이용해 카메라 좌표계에서 3D 점을 계산:

$$\mathbf{P}_{camera} = \begin{bmatrix} \frac{(u - c_x) \cdot z}{f_x} \ \frac{(v - c_y) \cdot z}{f_y} \ z \end{bmatrix}$$

여기서:

$(f_x, f_y)$: 초점 거리 (픽셀 단위)
$(c_x, c_y)$: 주점 (principal point)
$z$: 깊이 값 (미터)
2.2 월드 좌표 변환
TF를 통해 카메라 → 월드 좌표계 변환:

$$\mathbf{P}{world} = \mathbf{R} \cdot \mathbf{P}{camera} + \mathbf{t}$$

여기서:

$\mathbf{R}$: 회전 행렬 (3×3)
$\mathbf{t}$: 이동 벡터 (3×1)
3. 데이터 연관 (Data Association)
새로운 관측을 기존 랜드마크와 연관시키는 핵심 알고리즘입니다.

3.1 마할라노비스 거리 (Mahalanobis Distance)
공분산을 고려한 통계적 거리 측정:

$$d^2_{mahal} = (\mathbf{p} - \boldsymbol{\mu})^T \boldsymbol{\Sigma}^{-1} (\mathbf{p} - \boldsymbol{\mu})$$

여기서:

$\mathbf{p}$: 새로운 관측 위치
$\boldsymbol{\mu}$: 랜드마크 평균 위치
$\boldsymbol{\Sigma}$: 랜드마크 공분산 행렬
χ² 게이트 (99% 신뢰도):

$$d^2_{mahal} < \chi^2_{3, 0.99} = 13.5$$

3.2 텍스트 유사도 (Text Similarity)
Levenshtein Distance (편집 거리)
두 문자열 간 최소 편집 연산 수 (삽입, 삭제, 치환):

$$L(i,j) = \begin{cases}
j & \text{if } i = 0 \
i & \text{if } j = 0 \
\min \begin{cases}
L(i-1, j) + 1 & \text{(삭제)} \
L(i, j-1) + 1 & \text{(삽입)} \
L(i-1, j-1) + \delta & \text{(치환)}
\end{cases} & \text{otherwise}
\end{cases}$$

여기서 $\delta = 0$ if $s_1[i] = s_2[j]$, else $\delta = 1$

정규화된 유사도:

$$sim_{lev} = 1.0 - \frac{L(s_1, s_2)}{\max(|s_1|, |s_2|)}$$

추가 보너스
부분 문자열 보너스 (OCR이 간판 일부만 인식한 경우):
$$sim_{substr} = 0.25 \times \frac{|s_{short}|}{|s_{long}|}$$

접두사 보너스 (첫 3-4글자 일치):
$$sim_{prefix} = 0.10$$

최종 텍스트 유사도:
$$sim_{text} = \min(1.0, sim_{lev} + sim_{substr} + sim_{prefix})$$

3.3 복합 스코어 (Combined Score)
기하학적 거리와 텍스트 유사도를 결합:

$$score = w_{geo} \cdot s_{geo} + w_{text} \cdot s_{text}$$

여기서:

$s_{geo} = 1.0 - \frac{d^2_{mahal}}{\chi^2_{threshold}}$ (기하학적 스코어)
$s_{text} = sim_{text}$ (텍스트 스코어)
$w_{geo} = 0.9$, $w_{text} = 0.1$ (가중치)
연관 결정:

$score > 0.55$: 기존 랜드마크에 연관 → 업데이트
$score \leq 0.55$: 새로운 랜드마크 생성
4. 랜드마크 업데이트 (Welford's Online Algorithm)
4.1 온라인 평균 업데이트
새로운 관측이 들어올 때마다 평균을 점진적으로 업데이트:

$$\boldsymbol{\mu}{new} = \boldsymbol{\mu}{old} + \frac{\mathbf{p}{new} - \boldsymbol{\mu}{old}}{N}$$

여기서 $N$은 현재까지의 총 관측 수

장점:

O(1) 메모리 (모든 관측 저장 불필요)
수치적 안정성 (직접 평균 계산보다 오차 적음)
실시간 처리 가능
4.2 초기 공분산 설정
$$\boldsymbol{\Sigma}_0 = \begin{bmatrix}
\sigma^2 & 0 & 0 \
0 & \sigma^2 & 0 \
0 & 0 & 9\sigma^2
\end{bmatrix}$$

여기서 $\sigma = 0.3m$ (센서 노이즈)

Z축(깊이)은 3배 더 큰 불확실성 반영

5. 대표 텍스트 선정 (Weighted Voting)
OCR 노이즈에 강건한 텍스트 결정 방식:

5.1 가중 투표
각 텍스트별 OCR 신뢰도 합산:

$$score(T) = \sum_{i: text_i = T} conf_i$$

5.2 대표 텍스트 및 신뢰도
$$T_{rep} = \arg\max_{T} score(T)$$

$$confidence = \frac{score(T_{rep})}{\sum_{T} score(T)}$$

Sliding Window: 최근 50개 관측만 유지 (시간에 따른 적응)

6. 랜드마크 병합 (Duplicate Removal)
동일 간판이 여러 랜드마크로 생성되는 것을 방지:

6.1 병합 조건
두 조건 모두 만족 시 병합:

유클리드 거리: $|\boldsymbol{\mu}_1 - \boldsymbol{\mu}_2| < 1.0m$
텍스트 유사도: $sim_{text} > 0.3$
6.2 병합 시 위치 업데이트
관측 수 가중 평균:

$$\boldsymbol{\mu}_{merged} = \frac{N_1 \cdot \boldsymbol{\mu}_1 + N_2 \cdot \boldsymbol{\mu}_2}{N_1 + N_2}$$

$$N_{merged} = N_1 + N_2$$

7. 재투영 오차 (Reprojection Error)
7.1 재투영 공식
3D 랜드마크를 이미지 평면으로 재투영:

$$\mathbf{P}{camera} = \mathbf{R}^T \cdot (\mathbf{P}{world} - \mathbf{t}_{camera})$$

$$u_{proj} = f_x \cdot \frac{P_{camera,x}}{P_{camera,z}} + c_x$$

$$v_{proj} = f_y \cdot \frac{P_{camera,y}}{P_{camera,z}} + c_y$$

7.2 오차 계산
$$e = \sqrt{(u_{proj} - u_{obs})^2 + (v_{proj} - v_{obs})^2}$$

단위: 픽셀

필터링: $e > 50$ 픽셀이면 잘못된 연관으로 간주하여 제외

8. 주요 파라미터
파라미터	값	설명
χ² threshold	13.5	마할라노비스 거리 임계값 (99% 신뢰도)
Text similarity threshold	0.35	최소 텍스트 유사도
Acceptance threshold	0.55	데이터 연관 최소 스코어
Sensor noise σ	0.3m	깊이 센서 표준편차
Merge radius	10.0m	병합 탐색 반경
Min observations	3	출력 최소 관측 수
Text history size	50	텍스트 투표용 윈도우 크기
Geo:Text weight	90:10	복합 스코어 가중치
