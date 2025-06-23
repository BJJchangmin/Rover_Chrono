function y = LPF(f,data,time,init)

tau =1/(2*pi*f);
S = tf('s');
lpf = 1/(tau*S+1);

y = lsim(lpf,data,time,init);

end