% =========================================================
%  GraphOptimizer.m  —  Tuana
%  Görev : Poz grafı üzerindeki birikimli hataları düzelt.
%
%  Yöntem:
%      Gauss-Newton least squares optimizasyonu.
%      Her kenar bir kısıt oluşturur: konum_j - konum_i = ölçüm_ij
%      Tüm kısıtları aynı anda minimize ederek optimal konumları bul.
%
%  Kullanım:
%      optimizer = GraphOptimizer(pg)
%      traj_opt  = optimizer.optimize()
%
%  Girdi:
%      pg  — PoseGraph objesi (Zeynep'in modülü)
%             pg.nodes : [N x 2] konum listesi [x, y]
%             pg.edges : struct array
%                .from      : kaynak düğüm indeksi
%                .to        : hedef düğüm indeksi
%                .transform : [dx, dy] ölçümü
%                .weight    : güven skoru [0,1]
%
%  Çıktı:
%      traj_opt — [N x 2] optimize edilmiş konum listesi
% =========================================================

classdef GraphOptimizer

    properties
        pg          % PoseGraph objesi
        maxIter     % maksimum iterasyon sayısı
        tolerance   % yakınsama toleransı
        lambda      % Levenberg-Marquardt damping faktörü
    end

    methods

        function obj = GraphOptimizer(pg)
            obj.pg        = pg;
            obj.maxIter   = 50;
            obj.tolerance = 1e-6;
            obj.lambda    = 1e-4;
        end

        function traj_opt = optimize(obj)
        % -------------------------------------------------------
        %  Ana optimizasyon fonksiyonu
        %  Gauss-Newton ile poz grafını optimize eder.
        % -------------------------------------------------------

            nodes = obj.pg.nodes;   % [N x 2]
            edges = obj.pg.edges;   % struct array

            N = size(nodes, 1);

            if N < 2
                fprintf('  [GraphOptimizer] Yeterli dugum yok, optimizasyon atlandi.\n');
                traj_opt = nodes;
                return;
            end

            if isempty(edges)
                fprintf('  [GraphOptimizer] Kenar yok, optimizasyon atlandi.\n');
                traj_opt = nodes;
                return;
            end

            fprintf('  [GraphOptimizer] %d dugum, %d kenar ile optimizasyon basliyor...\n', ...
                N, numel(edges));

            % Konum vektörü: [x1 y1 x2 y2 ... xN yN]' (2N x 1)
            x = reshape(nodes', [], 1);

            prevCost = inf;

            for iter = 1:obj.maxIter

                % Jacobian (H) ve artık vektörü (b) oluştur
                [H, b, cost] = obj.buildSystem(x, edges, N);

                % İlk düğümü sabitle (referans noktası)
                % 1. ve 2. satır/sütunları I ile değiştir → x1,y1 değişmez
                H(1,1) = H(1,1) + 1e10;
                H(2,2) = H(2,2) + 1e10;

                % Levenberg-Marquardt damping
                H = H + obj.lambda * speye(2*N);

                % Gauss-Newton adımı: H * dx = -b
                dx = -H \ b;

                % Konumları güncelle
                x = x + dx;

                % Yakınsama kontrolü
                if abs(prevCost - cost) < obj.tolerance
                    fprintf('  [GraphOptimizer] Yakinsadi: iter=%d  maliyet=%.6f\n', iter, cost);
                    break;
                end
                prevCost = cost;

                if mod(iter, 10) == 0
                    fprintf('  [GraphOptimizer] iter=%d  maliyet=%.6f\n', iter, cost);
                end
            end

            % Sonucu [N x 2] formatına çevir
            traj_opt = reshape(x, 2, [])';

            fprintf('  [GraphOptimizer] Tamamlandi.\n');
            fprintf('  Ham traje X araligi:  %.3f → %.3f\n', ...
                min(nodes(:,1)), max(nodes(:,1)));
            fprintf('  Opt traje X araligi:  %.3f → %.3f\n', ...
                min(traj_opt(:,1)), max(traj_opt(:,1)));
        end

    end

    methods (Access = private)

        function [H, b, cost] = buildSystem(obj, x, edges, N)
        % -------------------------------------------------------
        %  Gauss-Newton sistem matrisini oluştur.
        %
        %  Her kenar (i→j) için kısıt:
        %      r = (x_j - x_i) - ölçüm_ij
        %
        %  Maliyet: sum_edges( w * ||r||^2 )
        %  H += w * J' * J
        %  b += w * J' * r
        % -------------------------------------------------------

            H    = sparse(2*N, 2*N);
            b    = zeros(2*N, 1);
            cost = 0;

            for e = 1:numel(edges)

                i   = edges(e).from;
                j   = edges(e).to;
                mij = edges(e).transform(:);   % [dx; dy] ölçümü
                w   = edges(e).weight;         % güven skoru

                % Konum indeksleri
                xi = x(2*i-1 : 2*i);   % [xi; yi]
                xj = x(2*j-1 : 2*j);   % [xj; yj]

                % Artık (residual): gerçek fark - ölçüm
                r = (xj - xi) - mij;   % [2 x 1]

                % Maliyet
                cost = cost + w * (r' * r);

                % Jacobian blokları
                % dr/d(xi) = -I,  dr/d(xj) = +I
                ii = 2*i-1 : 2*i;
                jj = 2*j-1 : 2*j;

                H(ii, ii) = H(ii, ii) + w * eye(2);
                H(jj, jj) = H(jj, jj) + w * eye(2);
                H(ii, jj) = H(ii, jj) - w * eye(2);
                H(jj, ii) = H(jj, ii) - w * eye(2);

                b(ii) = b(ii) + w * (-r);
                b(jj) = b(jj) + w * ( r);
            end

            cost = cost / numel(edges);
        end

    end

end