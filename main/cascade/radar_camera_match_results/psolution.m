function p = psolution(lambda,f,Jf,D)
%��p����С���˽�
%
A=[Jf;sqrt(lambda)*D];
B=-[f;zeros(length(diag(D)),1)];
p = lsqminnorm(A,B);
end

