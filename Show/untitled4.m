%untitled3,4,5はシミュレーションたいむステップを変えたときにどれくらい誤差が出るかをチェックするもの。
%vector = 1:100;  % 要素が100個のベクトル

% 抽出する要素数
targetSize = 56770;

% 均等に要素を抽出
indices = round(linspace(1, numel(position01(:,1,1)), targetSize));
position01_56770 = (position01(indices,:,1));

disp(size(position005(indices,:,1),1))