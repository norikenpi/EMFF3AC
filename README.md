# EMFF3
EMFF simulation

for gradute study

Danil's implementation of the control method

Simulation of electromagnetic force controlling the relative positions of multiple satellites in formation flight

実行方法
基本的にパラメータはsetSimulationParameters.mに全て入ってるから，そこで変更する．

図3　8基の衛星で構成される衛星群の目標形態
startSimulation.mでシミュレーションをしたあとに，show/showLastFrame.mでシミュレーションの最終フレームを図示．
凡例などを適宜削除する．

図4　ペア決め手法と展開に成功した回数の関係
図5　ペア決め手法と展開にかかった時間の関係
startManySimulation.mで各手法それぞれ100回ずつシミュレーションをする．
result_○○,result_○○01がそれぞれの手法に対して出力される．
result_○○は各シミュレーションで展開にかかった時間が記録されている（発散した場合はマイナス表示になってる）
result_○○01は展開に成功したら１が格納されていて，失敗したら０が格納されている配列．
show/plotConvergePlot.mで展開時間の散布図
show/hakohigehakohige.mで棒グラフと箱ひげ図を図示する．


