function Fn = Wmodel( T, th_1, s, r, r_s, b, th_m, param)

ks = param(1);
n0 = param(2);
n1 = param(3);

A = (cos(th_m)-1)./th_m + (cos(th_m)-cos(th_1))./(th_m-th_1);

B = sin(th_m)./th_m + (sin(th_m)-sin(th_1))./(th_m-th_1);

C = th_1./2;

term1 = r*b.*A;

term2 = ks*r.^(n0+n1.*s);

term3 = (cos(th_m)-cos(th_1)).^(n0+n1.*s);

term4 = (B.*T)./(r_s.*C);

Fn = term1.*term2.*term3+term4;



end

