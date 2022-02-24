function f_total = pffGoalie(dball,dshotpath,dshotpathDef,dgoalAtt,dgoalDef,dbehindball,in7,in8)
%PFFGOALIE
%    F_TOTAL = PFFGOALIE(DBALL,DSHOTPATH,DSHOTPATHDEF,DGOALATT,DGOALDEF,DBEHINDBALL,IN7,IN8)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    27-Oct-2016 15:14:54

dside1 = in7(:,1);
dside2 = in7(:,2);
dside3 = in7(:,3);
dside4 = in7(:,4);
t2 = dball-1.0./1.0e2;
t3 = dgoalDef-1.0./2.0;
t4 = dshotpathDef-1.0./1.0e2;
t5 = 1.0./dside1-1.0e1;
t6 = 1.0./dside2-1.0e1;
t7 = 1.0./dside3-1.0e1;
t8 = 1.0./dside4-1.0e1;
f_total = t2.^2.*(heaviside(dball-1.0).*(1.0./2.0)-heaviside(dball).*(1.0./2.0)).*-5.0-t3.^2.*(heaviside(dgoalDef-9.0).*(1.0./2.0)-heaviside(dgoalDef).*(1.0./2.0)).*4.0-t5.^2.*(heaviside(dside1-1.0./1.0e1).*(1.0./2.0)-heaviside(dside1).*(1.0./2.0))-t6.^2.*(heaviside(dside2-1.0./1.0e1).*(1.0./2.0)-heaviside(dside2).*(1.0./2.0))-t7.^2.*(heaviside(dside3-1.0./1.0e1).*(1.0./2.0)-heaviside(dside3).*(1.0./2.0))-t8.^2.*(heaviside(dside4-1.0./1.0e1).*(1.0./2.0)-heaviside(dside4).*(1.0./2.0))-t4.^2.*(heaviside(dshotpathDef-1.0e1).*(1.0./2.0)-heaviside(dshotpathDef).*(1.0./2.0)).*4.0;