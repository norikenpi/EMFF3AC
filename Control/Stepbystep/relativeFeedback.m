function [u, satellites, histories] = relativeFeedback(i, pair_satellite_idx, satellites, param, histories)

    
    %Hill方程式によって作られるオイラー近似離散状態方程式の係数
    A = [0, 0, 0, 1, 0, 0;
         0, 0, 0, 0, 1, 0;
         0, 0, 0, 0, 0, 1;
         0, 0, 0, 0, 0, 2*param.n;
         0, -param.n^2, 0, 0, 0, 0;
         0, 0, 3*param.n^2, -2*param.n, 0, 0];
    
    if not(param.hill_on)
        A = [0, 0, 0, 1, 0, 0;
             0, 0, 0, 0, 1, 0;
             0, 0, 0, 0, 0, 1;
             0, 0, 0, 0, 0, 0;
             0, 0, 0, 0, 0, 0;
             0, 0, 0, 0, 0, 0];
    end

    m = satellites{i}.mass;
    
    %Hill方程式によって作られるオイラー近似離散状態方程式の係数
    B = [0, 0, 0;
         0, 0, 0;
         0, 0, 0;
         1/m, 0, 0;
         0, 1/m, 0;
         0, 0, 1/m];
    
    B_sharp = (B.'*B)\B.';
    
    %オイラー近似を利用した差分方程式の係数
    A_d = eye(6) + param.dt*A;
    
    %オイラー近似を利用した差分方程式の係数
    %B_d = param.dt*B;
    
    Q = param.Q;
    R = param.R;
    
    %リッカチ方程式を解いて最適ゲインを求める
    [~,K,~] = icare(A,B,Q,R,[],[],[]);
    
    %現在の相対位置速度と目標の相対位置速度
    relative_position = satellites{pair_satellite_idx}.position - satellites{i}.position;
    relative_velocity = satellites{pair_satellite_idx}.velocity - satellites{i}.velocity;

    %ボロノイ図を用いて目標位置を計算
    %[satellites, c, v] = calcDesirePostion(satellites, param);


    %histories.c(int32(time/param.dt)+1, :, idx) = c;
    %histories.v(int32(time/param.dt)+1, :, idx) = v;

    relative_position_d = satellites{pair_satellite_idx}.position_d - satellites{i}.position_d;
    relative_velocity_d = satellites{pair_satellite_idx}.velocity_d - satellites{i}.velocity_d;
    relative_accelaration_d = [0;0;0];
    
    x = [relative_position; relative_velocity];
    xd = [relative_position_d; relative_velocity_d];
    vd = [relative_velocity_d; relative_accelaration_d];
    
    %フィードバック系を0に収束させるために入力変位を設定
    u_d = B_sharp*(vd - A_d*xd);
    
    %目標値と現在地のずれ
    x_tilda = x - xd;


    adjust_mat = [[10000,0,0,0,0,0];
                  [0,10000,0,0,0,0];
                  [0,0,10000,0,0,0];
                  [0,0,0,10000,0,0];
                  [0,0,0,0,10000,0];
                  [0,0,0,0,0,10000];]/100000000;

    K = K * adjust_mat; 
    
    %x_tildaを0にするためのフィードバック
    u_tilda = -K*x_tilda;
    
    %元の状態方程式の入力(N) 自分から見た相手の衛星の制御力とは逆の制御力を自分に加える
    u = -(u_tilda + u_d);

end

function [satellites, c, v] = calcDesirePostion(satellites, param)
    Px = zeros(param.N, 1);
    Py = zeros(param.N, 1);
    for i = 1:param.N
        Px(i) = satellites{i}.position(1);
        Py(i) = satellites{i}.position(2);
    end
    %LIMIT=5;
    LIMIT2=sqrt(2)*0.6;
    %LIMIT=0;
    
    % 0から2πまでの値をl個の間隔で生成。これは、円周上の点を表すための角度。l-1個
    l=5;
    theta = linspace(pi/4,9*pi/4,l); 
    theta(5)=[];
    
    %voronoi領域 LIMIT2を半径とする円周上の点を表すためのXとYの座標
    X=(LIMIT2*cos(theta)).';
    Y=(LIMIT2*sin(theta)).';
    XX=[X, Y];
    [Px, Py, c, v] = Yuta_lloydsAlgorithm2(Px,Py, XX);
    for i = 1:param.N
        satellites{i}.position_d(1) = Px(i);
        satellites{i}.position_d(2) = Py(i);
    end
end

function [Px, Py, c, v] = Yuta_lloydsAlgorithm2(Px,Py, crs)
    
    close all
    format compact
    
    % initialize random generator in repeatable fashion
    sd = 20;
    rng(sd)
    %入力があった場合,crsは領域Qを示す
    % 与えられた境界から最大・最小のx,yを取得
    xrange = max(crs(:,1));
    yrange = max(crs(:,2));
    [v,c]=VoronoiBounded(Px,Py, crs);

    % 各セルの重心を計算
    for i = 1:numel(c) %calculate the centroid of each cell
        [cx,cy] = PolyCentroid(v(c{i},1),v(c{i},2));
        cx = min(xrange,max(cx));
        cy = min(yrange,max(cy));

        % 重心が境界内にある場合、点を更新
        if ~isnan(cx) && inpolygon(cx,cy,crs(:,1),crs(:,2))
            Px(i) = cx;  %don't update if goal is outside the polygon
            Py(i) = cy;
        end
    end

end



function [Px, Py] = Yuta_lloydsAlgorithm(Px,Py, crs, numIterations, showPlot)
    % 与えられた領域内で点の集合を均等に分散させるための反復的な方法
    % すべての点に対してVoronoi図を計算します。
    % Voronoi図の各セルの重心を計算します。
    % 各点(agent)をそのVoronoiセルの重心に移動します。

    % Px と Py: 移動させる点のxとyの座標。
    % crs: 点が存在する境界（多角形）を表す頂点のリスト。
    % numIterations: アルゴリズムを実行する反復回数。
    % showPlot: 真偽値。true の場合、結果をグラフとして表示します。
    
    % LLOYDSALGORITHM runs Lloyd's algorithm on the particles at xy positions 
    % (Px,Py) within the boundary polygon crs for numIterations iterations
    % showPlot = true will display the results graphically.  
    % 
    % Lloyd's algorithm starts with an initial distribution of samples or
    % points and consists of repeatedly executing one relaxation step:
    %   1.  The Voronoi diagram of all the points is computed.
    %   2.  Each cell of the Voronoi diagram is integrated and the centroid is computed.
    %   3.  Each point is then moved to the centroid of its Voronoi cell.
    %
    % Inspired by http://www.mathworks.com/matlabcentral/fileexchange/34428-voronoilimit
    % Requires the Polybool function of the mapping toolbox to run.
    %
    % Run with no input to see example.  To initialize a square with 50 robots 
    % in left middle, run:
    %lloydsAlgorithm(0.01*rand(50,1),zeros(50,1)+1/2, [0,0;0,1;1,1;1,0], 200, true)
    %
    % Made by: Aaron Becker, atbecker@uh.edu

    % LLOYDSALGORITHMは、境界多角形crs内の位置 (Px,Py) にある粒子に対してLloydのアルゴリズムを実行します。反復回数はnumIterationsです。
    % showPlot = true の場合、結果はグラフとして表示されます。

    % Lloydのアルゴリズムは、サンプルや点の初期分布から開始し、以下のリラクゼーションステップを繰り返し実行します：

    % 全ての点のVoronoi図が計算されます。
    % Voronoi図の各セルが統合され、重心が計算されます。
    % 各点は、そのVoronoiセルの重心に移動されます。
    % このアルゴリズムは、こちらに触発されて作成されました。
    % この実行には、マッピングツールボックスのPolybool関数が必要です。

    % 入力なしで実行すると、例が表示されます。左中央に50台のロボットを持つ正方形を初期化するには、次のように実行します：
    % lloydsAlgorithm(0.01*rand(50,1), zeros(50,1)+1/2, [0,0;0,1;1,1;1,0], 200, true)

    % 制作者: Aaron Becker, atbecker@uh.edu
    close all
    format compact
    
    % initialize random generator in repeatable fashion
    sd = 20;
    rng(sd)
    
    %nargin:関数の入力の数
    if nargin < 1   % demo mode
     %{
        showPlot = true;
        numIterations  = 200;
        xrange = 10;  %region size
        yrange = 5;
        n = 50; %number of robots  (changing the number of robots is interesting)
    
    % Generate and Place  n stationary robots
        Px = 0.01*mod(1:n,ceil(sqrt(n)))'*xrange; %start the robots in a small grid
        Py = 0.01*floor((1:n)/sqrt(n))'*yrange;
        
    %     Px = 0.1*rand(n,1)*xrange; % place n  robots randomly
    %     Py = 0.1*rand(n,1)*yrange;
        
        crs = [ 0, 0;    
            0, yrange;
            1/3*xrange, yrange;  % a world with a narrow passage
            1/3*xrange, 1/4*yrange;
            2/3*xrange, 1/4*yrange;
            2/3*xrange, yrange;
            xrange, yrange;
            xrange, 0];
        
        for i = 1:numel(Px)  
            while ~inpolygon(Px(i),Py(i),crs(:,1),crs(:,2))% ensure robots are inside the boundary
                Px(i) = rand(1,1)*xrange; 
                Py(i) = rand(1,1)*yrange;
            end
        end
        %}
    else
        %入力があった場合,crsは領域Qを示す
        % 与えられた境界から最大・最小のx,yを取得
        xrange = max(crs(:,1));
        yrange = max(crs(:,2));
        xrange_min = min(crs(:,1));
        yrange_min = min(crs(:,2));
        n = numel(Px); %number of robots  
    end
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%% VISUALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % データを視覚的に表示する場合の処理
    if showPlot
        verCellHandle = zeros(n,1);
        cellColors = cool(n);
        % 各ロボットの初期位置をプロット
        for i = 1:numel(Px) % color according to
            verCellHandle(i)  = patch(Px(i),Py(i),cellColors(i,:)); % use color i  -- no robot assigned yet
            hold on
        end
        pathHandle = zeros(n,1);    
        %numHandle = zeros(n,1);    
        for i = 1:numel(Px) % color according to
            pathHandle(i)  = plot(Px(i),Py(i),'-','color',cellColors(i,:)*.8);
        %    numHandle(i) = text(Px(i),Py(i),num2str(i));
        end

        % 目標位置と現在位置を表示
        goalHandle = plot(Px,Py,'+','linewidth',2);
        currHandle = plot(Px,Py,'o','linewidth',2);

        % タイトルを設定
        titleHandle = title(['o = Robots, + = Goals, Iteration ', num2str(0)]);
    end
    %%%%%%%%%%%%%%%%%%%%%%%% END VISUALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Iteratively Apply LLYOD's Algorithm
    %numIterations=ステップ数
    % Lloydのアルゴリズムを繰り返し適用
    for counter = 1:numIterations

        %[v,c]=VoronoiLimit(Px,Py, crs, false);
        % Voronoi図を計算
        [v,c]=VoronoiBounded(Px,Py, crs);
        
        if showPlot

            % 現在の位置を更新
            set(currHandle,'XData',Px,'YData',Py);%plot current position

            % 各ロボットの経路を更新
            for i = 1:numel(Px) % color according to
                xD = [get(pathHandle(i),'XData'),Px(i)];
                yD = [get(pathHandle(i),'YData'),Py(i)];
                set(pathHandle(i),'XData',xD,'YData',yD);%plot path position
         %      set(numHandle(i),'Position',[ Px(i),Py(i)]);
            end
        end
        
        % 各セルの重心を計算
        for i = 1:numel(c) %calculate the centroid of each cell
            [cx,cy] = PolyCentroid(v(c{i},1),v(c{i},2));
            cx = min(xrange,max(cx));
            cy = min(yrange,max(cy));

            % 重心が境界内にある場合、点を更新
            if ~isnan(cx) && inpolygon(cx,cy,crs(:,1),crs(:,2))
                Px(i) = cx;  %don't update if goal is outside the polygon
                Py(i) = cy;
            end
        end
        
        if showPlot
            % Voronoiセルを更新
            for i = 1:numel(c) % update Voronoi cells
                set(verCellHandle(i), 'XData',v(c{i},1),'YData',v(c{i},2));
            end
            %assignin('base', 'verCellHandle',verCellHandle());
    
            % タイトルと目標位置を更新
            set(titleHandle,'string',['o = Robots, + = Goals, Iteration ', num2str(counter,'%3d')]);
            set(goalHandle,'XData',Px,'YData',Py);%plot goal position
            
            % グラフの表示設定
            axis equal
            axis([xrange_min,xrange,yrange_min,yrange]);

            % グラフをリフレッシュ
            drawnow
    %         if mod(counter,50) ==0
    %             pause
    %             %pause(0.1)
    %         end
        end
    end
end

function [Cx,Cy] = PolyCentroid(X,Y)
% 重心を計算
% 多角形の頂点の関係を使用して、その重心を計算するための数学的な公式を実装

% POLYCENTROID returns the coordinates for the centroid of polygon with vertices X,Y
% The centroid of a non-self-intersecting closed polygon defined by n vertices (x0,y0), (x1,y1), ..., (xn?1,yn?1) is the point (Cx, Cy), where
% In these formulas, the vertices are assumed to be numbered in order of their occurrence along the polygon's perimeter, and the vertex ( xn, yn ) is assumed to be the same as ( x0, y0 ). Note that if the points are numbered in clockwise order the area A, computed as above, will have a negative sign; but the centroid coordinates will be correct even in this case.http://en.wikipedia.org/wiki/Centroid
% A = polyarea(X,Y)
    
    Xa = [X(2:end);X(1)];
    Ya = [Y(2:end);Y(1)];
    %各ボロノイ領域の面積
    A = 1/2*sum(X.*Ya-Xa.*Y); %signed area of the polygon
    %各ボロノイ領域の重心
    Cx = (1/(6*A)*sum((X + Xa).*(X.*Ya-Xa.*Y)));
    Cy = (1/(6*A)*sum((Y + Ya).*(X.*Ya-Xa.*Y)));
end

function [V,C]=VoronoiBounded(x,y, crs)
    % 与えられた境界内(crs)でのVoronoi図を計算する関数
    %この関数は、与えられた境界（通常は多角形）内でのVoronoi図を計算。
    % Voronoi図は、2次元平面上の点の集合から生成され、各点のまわりにセルが形成されるのが特徴。
    % 各セル内の任意の位置は、そのセルに関連付けられた特定の点に他の点よりも近いという特性を持つ。

    % 与えられた境界内でのVoronoi図を計算する関数

    % x, y: 入力として与えられる点の座標
    % crs: 定義された境界(多角形)の頂点

    % 出力:
    % V: Voronoi頂点の座標
    % C: 各点に対応するVoronoiセルのインデックス
    
    % VORONOIBOUNDED computes the Voronoi cells about the points (x,y) inside
    % the bounding box (a polygon) crs.  If crs is not supplied, an
    % axis-aligned box containing (x,y) is used.
    
    % データの範囲を計算
    bnd=[min(x) max(x) min(y) max(y)]; %data bounds

    % crsが提供されていない場合、xとyを含む軸に整列したボックスを使用する
    if nargin < 3
        crs=double([bnd(1) bnd(4);bnd(2) bnd(4);bnd(2) bnd(3);bnd(1) bnd(3);bnd(1) bnd(4)]);
    end
    
    % 領域のサイズを計算
    rgx = max(crs(:,1))-min(crs(:,1));
    rgy = max(crs(:,2))-min(crs(:,2));

    % agent群は1辺rgの正方形に収まる
    rg = max(rgx,rgy);

    % 領域の中心を計算(crsに外接する長方形)
    midx = (max(crs(:,1))+min(crs(:,1)))/2;
    midy = (max(crs(:,2))+min(crs(:,2)))/2;
    
    % 4つの追加のエッジを追加（境界外でVoronoiを計算するため）
    % 元のagent座標配列xを大きく囲う新しい4つの頂点。なぜか菱形。
    xA = [x; midx + [0;0;-5*rg;+5*rg]];
    yA = [y; midy + [-5*rg;+5*rg;0;0]];
    
    % Voronoi図を計算
    [vi,ci]=voronoin([xA,yA]);
    
    % 最後の4つのセルを削除（上記で追加した4つの点に対応）
    C = ci(1:end-4);
    V = vi;
    
    
    % use Polybool to crop the cells
    %Polybool for restriction of polygons to domain.
    % 各Voronoiセルを境界に制限
    % Voronoi図の各セルに対してループを実行
    for ij=1:length(C)

            % thanks to http://www.mathworks.com/matlabcentral/fileexchange/34428-voronoilimit
            % first convert the contour coordinate to clockwise order:
            % 輪郭の座標を時計回りの順序に変換
            [X2, Y2] = poly2cw(V(C{ij},1),V(C{ij},2));

            % polybool関数を使用して、セルの輪郭と境界多角形との交差点を計算(セルの輪郭はかなり大きくとっているから)
            [xb, yb] = polybool('intersection',crs(:,1),crs(:,2),X2,Y2);

            %交差点の座標（xb, yb）が既存のVoronoi頂点（V）と一致するかどうかを確認
            ix=nan(1,length(xb));
            for il=1:length(xb)
                
                if any(V(:,1)==xb(il)) && any(V(:,2)==yb(il))
                    %交差点の座標（xb, yb）と既存のVoronoi頂点（V）が一致する頂点が見つかれば、その頂点のインデックスを保存
                    %交差点のx座標がVoronoi頂点リストVのどのx座標と一致するか
                    ix1=find(V(:,1)==xb(il));
                    ix2=find(V(:,2)==yb(il));

                    %ix1とix2のインデックスが一致するかどうかを確認。
                    % 一致する場合、そのインデックス（頂点）が交差点の座標と一致することが確認される
                    for ib=1:length(ix1)
                        if any(ix1(ib)==ix2)
                            ix(il)=ix1(ib);
                        end
                    end

                    if isnan(ix(il))==1
                        %ix(il)がNaN（未定義）の場合、交差点の座標と一致する既存のVoronoi頂点が見つからなかったということになる。

                        lv=length(V);
                        V(lv+1,1)=xb(il);
                        V(lv+1,2)=yb(il);
                        ix(il)=lv+1;
                    end
                else
                    % 一致するものが見つからない場合、新しい頂点としてVoronoi頂点リストVに追加
                    % 新しい頂点として交差点の座標をVoronoi頂点リストVに追加
                    lv=length(V);
                    V(lv+1,1)=xb(il);
                    V(lv+1,2)=yb(il);
                    ix(il)=lv+1;
                end
            end

            % セルのインデックスを更新
            C{ij}=ix;
       
    end
    %}
end

