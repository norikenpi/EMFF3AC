# EMFF3
EMFF simulation

-branch-
evaluate_formation0929
詳細は20230929のスライドを確認

実行手順
フォーメーションの固有値を求める方法
1. param = setSimulationParameters()でパラメータを設定
2. evaluateFormation/evaluate_formation_test.mを実行

実際にフォーメーションの収束特性を調べる方法（3衛星）
1. simulateTimeStepAC.mの56行目と72行目を調整して、ネットワークを変更し、シミュレーションを実行する。初期のずれはInitialize/getSatellitePosition3.mの3行目のシードで変更できる。
2. startSimulation.mを実行
3. evaluateFormation/plotFormationEvaluation.mを実行することでグラフを標示することができる。
4. evaluateFormation/plotExponential.mを実行することで指数関数の収束性がわかる。
5. コメントアウトした時の方が収束性が良ければ問題なし。


for gradute study

Danil's implementation of the control method

Simulation of electromagnetic force controlling the relative positions of multiple satellites in formation flight



setSimulationParameters

<Control>

<Initialize>

<Magneticforce>

<Other>

<Show>

<U2m>

<Update>