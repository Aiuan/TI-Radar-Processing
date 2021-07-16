function p = psolution(lambda,f,Jf,D)
%求p的最小二乘解
%
A=[Jf;sqrt(lambda)*D];
B=-[f;zeros(length(diag(D)),1)];
p = lsqminnorm(A,B);
end

