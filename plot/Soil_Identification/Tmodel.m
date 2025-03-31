function T = Tmodel(W, th_1, s, r_s, b, th_1_n, th_m, param)

A = (cos(th_m)-1)./th_m + (cos(th_m)-cos(th_1))./(th_m-th_1);

B = sin(th_m)./th_m + (sin(th_m)-sin(th_1))./(th_m-th_1);

C = th_1./2;

r= 0.25;

c = param(1);
phi = param(2);
k = param(3);

D_exp = -r_s.*( (th_1_n-th_m) - (1-s).*(sin(th_1_n)-sin(th_m)))./k;

D = (1-exp(D_exp));

up_left = r_s.*r_s.*C.*(b*c+W*tan(phi)./(r.*A));
down_left = r_s.*B*tan(phi)./(r.*A);

T = (up_left.*D)./(1+down_left.*D);

end