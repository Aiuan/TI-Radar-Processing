% ��/input/test1_param.m�ļ��л�ȡ�״����
function par = getPara(paraFile, paraName)
    run(paraFile);
    par = eval(paraName);
end