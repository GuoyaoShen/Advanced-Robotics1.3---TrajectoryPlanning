A = [3;4;5;6;2;4;9];
B = [2;4.16;5.13;6.12;2.15;4.15;9.17];
error = 0.2;
D=0;
if (B > A-error & B < A+error)
   D = 1; 
end
disp(D);

% C = find(A < B_max & A > B_min)

% C = (A < B) & (A > B_min);
% C = (B_min < A < B);
% C = find(A < B & A > B_min);
% ind = find(C);
% disp(C);
% disp(ind);
% A(find(A < B & A > B_min))