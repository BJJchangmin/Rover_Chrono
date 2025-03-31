function DP = DPmodel(W, T, th_1, s, r_s, param)

 % DPmodel : 주어진 입력 값에 대해 Drawbar Pull (DP)를 계산하는 함수.
    %
    % 입력:
    %   W     : 수직 하중 (scalar)
    %   T     : 토크 (scalar)
    %   th_1  : 입구 각도 (radian, scalar)
    %   s     : 슬립비 (scalar)
    %   r_s   : shearing radius (scalar)
    %   param : 파라미터 벡터, [c1; c2]
    %
    % 출력:
    %   DP    : 계산된 Drawbar Pull 값
    %

c1 = param(1);
c2 = param(2);

th_m = (c1 +c2.*s).*th_1;

A = (cos(th_m)-1)./th_m + (cos(th_m)-cos(th_1))./(th_m-th_1);

B = sin(th_m)./th_m + (sin(th_m)-sin(th_1))./(th_m-th_1);

C = th_1./2;

DP = (A.^2 + B.^2).*T./(r_s.*A.*C) - B.*W./A;

end