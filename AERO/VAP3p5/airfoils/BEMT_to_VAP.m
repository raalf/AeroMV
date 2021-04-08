load('Matrice_Airfoil_BEMT.mat')

Re_range = unique(Matrice_Airfoil.Re);
airfoil_name = 'Matrice_Airfoil';
for i = 1:length(Re_range)
    count(i) = sum(Re_range(i) == Matrice_Airfoil.Re);
end

pol = nan(max(count),9,length(Re_range));

for i = 1:length(Re_range)
    idx = Re_range(i) == Matrice_Airfoil.Re;
    eff = (Matrice_Airfoil.CL(idx).^3)./(Matrice_Airfoil.CD(idx).^2);
    pol(1:(sum(idx)),:,i) = [Matrice_Airfoil.alpha(idx),Matrice_Airfoil.CL(idx),Matrice_Airfoil.CD(idx),Matrice_Airfoil.CDp(idx),Matrice_Airfoil.CM(idx),Matrice_Airfoil.Top_Xtr(idx),Matrice_Airfoil.Bot_Xtr(idx),Matrice_Airfoil.Re(idx),eff];
end

save(airfoil_name)