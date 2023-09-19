function createAnimation_z(histories, param, satellites)
    fps = 31;
    pause = param.pause;
    filename_option = "z";

    %データを入れるフォルダを作る。
    mkdir(param.path_data)
    filename_movie = strcat(param.path_data, sprintf('/%s_movie_%s.avi', param.date, filename_option));
    filename_var = strcat(param.path_data, sprintf('/%s_var.mat', param.date));

    
    
    %matファイルに全てのワークスペース変数の保存
    %save(filename_var);

    writerObj = VideoWriter(filename_movie);
    writerObj.Quality = 100;
    writerObj.FrameRate = fps;
    open(writerObj);

    figcolor = [1 1 1];
    fig = figure('color', figcolor);

    m = satellites{1}.mass;
    a = satellites{1}.radius;

    colors = jet(size(histories.position_histories, 3)); % N個の衛星に対して異なる色を設定

    for i = 1:round(pause/(fps*param.dt)):abs(param.finished_time)/param.dt
    
        disp(i)
        %h_traj = cell(1, length(histories.position_histories));
        %h_force = cell(1, length(histories.position_histories));
        disp(i)
        [h4, h5] = plotSatelliteTrajectory(histories, param, i, colors(1, :), sprintf('Satellite %d', 1), 1);
        
        for j = 2:size(histories.position_histories, 3)
            [h_force, h_mag] = plotSatelliteTrajectory(histories, param, i, colors(j, :), sprintf('Satellite %d', j), j);
        end

        %plotVolonoi(squeeze(histories.position_histories(i, 1, :)), squeeze(histories.position_histories(i, 2, :)), param)

        %x方向からの視点
        view(0, 90)
        dim = [0.65 0.5 0.3 0.3];
        path_parent = append('質量 ', string(m), 'kg ','衛星半径', string(a), 'm');
        annotation('textbox',dim,'String',path_parent,'FitBoxToText','on')

        
        axis([-param.axis_norm,param.axis_norm,-param.axis_norm,param.axis_norm,-param.axis_norm,param.axis_norm])
        axis square 
        grid on
        xlabel('X(m)（進行方向）');
        ylabel('Y(m)（面外方向）');
        zlabel('Z(m)（地心方向）');
        set(gca,'YDir','reverse')
        set(gca,'ZDir','reverse')
        path_parent = append('(', string(pause), ' times speed)');
        title(append('Satellite Relative Motion with Trajectory ',path_parent))
        %legend([h1, h4, h5], 'Satellite trajectory', 'Satellite force', 'magnetic moment');
        %legend(h_pair, 'Pair satellite');
       
        %グラフ画像に日付を入れる。
        xPosition = xlim;
        yPosition = ylim; % x軸の範囲を取得
        zPosition = zlim; % y軸の範囲を取得

        yPosition2 = ylim/2;

        %text(xPosition(2)*1.5, yPosition(1)/5, zPosition(1)*2, string(param.date), 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
        
        %時間を表示
        %text(xPosition(2)*1.5, yPosition(1)/3, zPosition(1)*2, sprintf("time %s", string(i*param.dt)), 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
        drawnow;

        frame = getframe(fig);
        writeVideo(writerObj, frame);
        hold off
    end
    close(writerObj);


    %パラメータをテキストファイル化
    filename_param = strcat(param.path_data, '/param.txt');
    outputStructToTextFile(param, filename_param)
end


function plotVolonoi(x, y, param)
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
    cellColors = cool(param.N);
    [v,c] = VoronoiBounded(x,y, XX);
    for i = 1:param.N % update Voronoi cells
        patch(v(c{i},1), v(c{i},2), ones(1, length(c{i})), cellColors(i,:)*.8);
    end

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
