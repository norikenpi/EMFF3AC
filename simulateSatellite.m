% 衛星振動をシミュレーションするメイン関数。初期状態設定、シミュレーションパラメータ設定、シミュレーション実行、結果プロット、アニメーション作成の手順を実行します。
function simulateSatellite()
    % シミュレーションパラメータを設定
    param = setSimulationParameters();
    
    %N個の衛星の初期状態を設定
    satellites = setInitialSatelliteStates(param);

    % 衛星の位置、力、および磁気モーメントの履歴を格納するためのセル配列を初期化する
    histories = makeHistoriesMemory(param);

    % シミュレーションを実行
    histories = runSimulation(satellites, param, histories);

    %最終結果を表示
    showLastFrame(histories, param, satellites);
    assignin('base', 'satellites', satellites);
    assignin('base', 'param', param);
    assignin('base', 'histories', histories);
  
    % アニメーションを作成
    %createAnimation(histories, param, satellites);
end