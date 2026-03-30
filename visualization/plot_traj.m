function plot_traj(poses, optimizedNodes, frameIdx, options)
% plot_traj.m — Sema'nın visualization modülü
% Trajectory'yi ve graph optimizasyonu sonucunu çizer.
%
% GİRİŞ:
%   poses          — [Nx2] ham pose matrisi (her satır = [x, y])
%   optimizedNodes — [Mx2] optimize edilmiş node pozisyonları
%                    (henüz yoksa [] boş geçilebilir)
%   frameIdx       — anlık frame numarası
%   options        — struct (opsiyonel):
%                      .title    — grafik başlığı
%                      .saveDir  — kayıt dizini (boşsa kaydetmez)
%
% KULLANIM ÖRNEKLERİ (Frame.m içinden):
%   plot_traj(poses, [], frameIdx);
%   plot_traj(poses, optimizedNodes, frameIdx);
%   plot_traj(poses, optimizedNodes, frameIdx, opts);

% ── Varsayılan seçenekler ─────────────────────────────────────
if nargin < 4
    options = struct();
end
if ~isfield(options, 'title'),   options.title   = 'Thermal SLAM Trajectory'; end
if ~isfield(options, 'saveDir'), options.saveDir = '';                         end

% ── Figür hazırlığı ───────────────────────────────────────────
figure(100);   % sabit numara — her çağrıda aynı figürü güncelle
clf;           % temizle — eski çizimi sil

% ── Kaç alt grafik açılacak? ──────────────────────────────────
hasOptimized = ~isempty(optimizedNodes);

if hasOptimized
    subplot(1, 2, 1);   % sol: ham trajectory
end

% ── Alt grafik 1: Ham trajectory ──────────────────────────────
plot(poses(:,1), poses(:,2), 'b-', 'LineWidth', 1.2);
hold on;

% Başlangıç noktası — yeşil daire
plot(poses(1,1), poses(1,2), 'go', ...
     'MarkerSize', 10, 'MarkerFaceColor', 'g');

% Şu anki konum — kırmızı kare
plot(poses(end,1), poses(end,2), 'rs', ...
     'MarkerSize', 10, 'MarkerFaceColor', 'r');

xlabel('X (piksel)');
ylabel('Y (piksel)');
title(sprintf('Ham Trajectory — Frame %d', frameIdx));
legend('Yol', 'Başlangıç', 'Şu an', 'Location', 'best');
grid on;
axis equal;

% ── Alt grafik 2: Optimize edilmiş (varsa) ────────────────────
if hasOptimized
    subplot(1, 2, 2);

    % Ham yol — soluk mavi kesikli
    plot(poses(:,1), poses(:,2), '--', ...
         'LineWidth', 0.8, 'Color', [0.6 0.6 1.0]);
    hold on;

    % Optimize yol — belirgin kırmızı
    plot(optimizedNodes(:,1), optimizedNodes(:,2), 'r-', ...
         'LineWidth', 2.0);

    % Keyframe noktaları — kırmızı dolu daireler
    scatter(optimizedNodes(:,1), optimizedNodes(:,2), ...
            30, 'r', 'filled');

    xlabel('X (piksel)');
    ylabel('Y (piksel)');
    title('Optimizasyon: Önce vs Sonra');
    legend('Ham (önce)', 'Optimize (sonra)', 'Keyframeler', ...
           'Location', 'best');
    grid on;
    axis equal;
end

% ── Genel başlık ──────────────────────────────────────────────
sgtitle(options.title, 'FontWeight', 'bold');
drawnow;   % ekranı hemen güncelle (canlı güncelleme için şart)

% ── İstege bağlı kayıt ────────────────────────────────────────
if ~isempty(options.saveDir)
    if ~isfolder(options.saveDir)
        mkdir(options.saveDir);
    end
    savePath = fullfile(options.saveDir, ...
                        sprintf('traj_%05d.png', frameIdx));
    saveas(gcf, savePath);
    fprintf('Kaydedildi: %s\n', savePath);
end

end