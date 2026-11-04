function pos_ff_table()
% pos_ff_table.m — 由 pos_trim(θ) 数据生成 POS 位置前馈表（24 格 × 15°）
% 关键：每段先减自身 median(pos_trim)——常数部分（trim 误差等）留给积分器，
% 表里只放【θ 相关分量】（dz LUT 残差 + 摩擦位置项）。
% 输出：matlab/data/_ff_table.txt（24 值，duty，已平滑+限幅）

D = dir(fullfile(fileparts(mfilename('fullpath')), 'data', 'pos_*.csv'));
NB = 24;                                  % 15°/格
s = zeros(NB, 1); c = zeros(NB, 1);
fid = fopen(fullfile(fileparts(mfilename('fullpath')), 'data', '_ff_table.txt'), 'w');
assert(fid >= 0); cleanup = onCleanup(@()fclose(fid));
P = @(varargin) fprintf(fid, varargin{:});
out = @(varargin) {fprintf(1, varargin{:}); P(varargin{:})};

for i = 1:numel(D)
    T = readtable(fullfile(D(i).folder, D(i).name));
    m = abs(T.theta_e) < 3000 & abs(T.alpha_c01) < 500 & abs(T.pos_trim) < 590;
    if nnz(m) < 2000, continue; end
    pt = T.pos_trim(m);
    pt = pt - median(pt);                 % ★去常数
    th = mod(T.theta_c01(m) / 100, 360);
    idx = min(NB, floor(th / (360 / NB)) + 1);
    for k = 1:NB
        v = pt(idx == k);
        s(k) = s(k) + sum(v); c(k) = c(k) + numel(v);
    end
end
mu = s ./ max(c, 1);
% 3 格环形平滑
muS = conv([mu(end-1:end); mu; mu(1:2)], [1 1 1] / 3, 'valid');
muS = max(-300, min(300, round(muS / 5) * 5));    % 限幅 ±300、5 duty 量化
out('%6s %8s %9s\n', 'bin(°)', 'n', 'FF_duty');
for k = 1:NB
    out('%3d-%3d %8d %9d\n', (k-1)*15, k*15, c(k), muS(k));
end
out('\nRTL 片段：\nlocalparam reg signed [31:0] POS_FF_LUT [0:23] = %c\n', char(123));
fprintf(fid, '    %5d, %5d, %5d, %5d, %5d, %5d,\n    %5d, %5d, %5d, %5d, %5d, %5d,\n', ...
    muS(1:12)); 
fprintf(fid, '    %5d, %5d, %5d, %5d, %5d, %5d,\n    %5d, %5d, %5d, %5d, %5d, %5d\n%c\n', ...
    muS(13:24), char(125));
disp('POS_FF_TABLE_DONE');
end
