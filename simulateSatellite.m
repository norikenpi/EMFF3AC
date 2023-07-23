

% 衛星振動をシミュレーションするメイン関数。初期状態設定、シミュレーションパラメータ設定、シミュレーション実行、結果プロット、アニメーション作成の手順を実行します。
function simulateSatellite(param)
    

    %パスを通す
    addPath(param);

    %N個の衛星の初期状態を設定
    satellites = setInitialSatelliteStates(param);

    satellites{5}.velocity = satellites{5}.velocity * 0.9; 
    satellites = adjustSatelliteState(satellites, param);
    satellites = adjustSatelliteDesiredState(satellites, param);

    % 衛星の位置、力、および磁気モーメントの履歴を格納するためのセル配列を初期化する
    histories = makeHistoriesMemory(param);


    %
    %assignin('base', 'satellites', satellites);
    %assignin('base', 'param', param);
    %assignin('base', 'histories', histories);


    % シミュレーションを実行
    [satellites, histories, param] = runSimulation(satellites, param, histories);
    assignin('base', 'satellites', satellites);
    assignin('base', 'param', param);
    assignin('base', 'histories', histories);

    
    %showLastFrame(histories, param, satellites);
     %最終結果を表示
        
  
    % アニメーションを作成
    %createAnimation(histories, param, satellites);
end