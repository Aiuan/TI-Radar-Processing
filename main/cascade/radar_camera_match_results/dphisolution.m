function dphi = dphisolution(lambda,f,Jf,D)
%Çódphi/dlambda
%
plambda=psolution(lambda,f,Jf,D);
qlambda=D*plambda;
[Q,R,pai]=qr(Jf);
Dlambda=sqrt(lambda)*pai'*D*pai;
[Q,R,E]=qr([R;Dlambda]);
Rlambda=R(1:length(diag(D)),:);
dphi=-sqrt(qlambda'*qlambda)*((inv(Rlambda))'*pai'*D'*qlambda/sqrt(qlambda'*qlambda))'*((inv(Rlambda))'*pai'*D'*qlambda/sqrt(qlambda'*qlambda));
end

