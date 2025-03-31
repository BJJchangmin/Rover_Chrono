function th_slip_model = th_s_model(s,z)
z = 0.01;
r = 0.25;
h = 0.01;
s_j1 = 0.15;
s_j2 = 0.5;

R_j = zeros(size(s));  % R_j를 s와 같은 크기의 벡터로 초기화

s = abs(s);

% 논리적 인덱싱을 사용하여 벡터별 조건 처리
idx1 = (0 <= s) & (s < s_j1);
idx2 = (s_j1 <= s) & (s <= s_j2);
idx3 = (s_j2 < s);

% 각 조건에 해당하는 R_j 값 할당
R_j(idx1) = r+h;
R_j(idx2) = r+h.*(s_j2 - s(idx2))./(s_j2-s_j1);
R_j(idx3) = r;

% 나머지 값들은 100으로 설정
R_j(~(idx1 | idx2 | idx3)) = r;

if R_j == 0
    R_j = r;
end

value = (r - z)./R_j;

th_slip_model = acos(value);

end