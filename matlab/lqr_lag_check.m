function rho = lqr_lag_check(kq)
% Linear sampled loop: 4ms difference, EMA/4, alpha d4 register,
% sensor Unit Delay and one control-period actuator delay.
% RTL arithmetic pipelines take 20ns clocks, NOT additional 1ms periods.
[A,B,P]=j280_linear_model();
if nargin==0
    g=fileread(fullfile(fileparts(mfilename('fullpath')),'..','rtl','j280_gains.vh'));
    tok=regexp(g,'`define G_K\d \((-?)16''sd(\d+)\)','tokens');
    assert(numel(tok)==4,'Expected four generated Q3.13 gains');
    kq=cellfun(@(v)str2double([v{1} v{2}]),tok);
end
E=expm([A B;zeros(1,5)]*P.Ts_ctrl); Ad=E(1:4,1:4); Bd=E(1:4,5);
% z = [physical x(4), sampled alpha/arm histories(5+5), d4, two EMA, applied u]
F=zeros(18);
for j=1:18
    z=zeros(18,1); z(j)=1;
    x=z(1:4); ah=z(5:9); th=z(10:14);
    av=z(16)+(25*z(15)-z(16))/4;
    tv=z(17)+(25*(th(1)-th(5))-z(17))/4;
    u=(kq/2^13)*[ah(1);av;th(1);tv]*P.Vbus/P.duty_full;
    F(:,j)=[Ad*x+Bd*z(18);x(2)*18000/pi;ah(1:4);x(1)*18000/pi;th(1:4);ah(1)-ah(5);av;tv;u];
end
rho=max(abs(eig(F)));
assert(rho<1,'Unstable sampled loop: spectral radius %.8f',rho);
fprintf('Sampled loop spectral radius %.8f (<1): PASS\n',rho);
end
