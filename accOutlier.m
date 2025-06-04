function flag = accOutlier(acc, g0, tol)
    
    acc_mag = vecnorm(acc, 2, 1);

    if abs(acc_mag-g0) > tol
        flag = 1;
    else
        flag = 0;
    end

end